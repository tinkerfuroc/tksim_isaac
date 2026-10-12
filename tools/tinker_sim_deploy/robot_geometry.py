"""Robot geometry read from the URDF itself.

base_link is the xArm mount on the chassis, not the floor. How far above the
floor it rests follows from the wheels: the lowest point of every
``*_wheel`` link's collision, which must agree. Both the base_footprint TF
frame (runtime.with_base_footprint) and the AMCL map's scan band
(arena_map.livox_scan_height) use this one derivation, so they cannot
disagree.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET


def wheel_floor_z(root: ET.Element) -> float | None:
    """z of the floor in base_link (negative), or None without wheels.

    Raises ValueError if the wheels disagree by more than 1 mm, or a wheel
    hangs off a rotated mount.
    """
    # Top-level elements only: <ros2_control> nests its own <joint> tags.
    parent_joint = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
    bottoms: dict[str, float] = {}
    for link in root.findall("link"):
        name = link.get("name", "")
        if not name.endswith("_wheel"):
            continue
        for collision in link.findall("collision"):
            bottom = _wheel_collision_bottom(collision)
            if bottom is None:
                continue
            frame, z = name, bottom
            while frame != "base_link":
                joint = parent_joint.get(frame)
                if joint is None:
                    break
                xyz, rpy = origin(joint)
                if any(abs(angle) > 1e-6 for angle in rpy):
                    raise ValueError(f"{joint.get('name')}: rotated wheel mount is not supported")
                z += xyz[2]
                frame = joint.find("parent").get("link")
            if frame == "base_link":
                bottoms[name] = min(z, bottoms.get(name, z))
    if not bottoms:
        return None
    if max(bottoms.values()) - min(bottoms.values()) > 0.001:
        raise ValueError(f"wheel bottoms disagree on the floor height: {bottoms}")
    return min(bottoms.values())


def base_rest_height(urdf: str | bytes) -> float:
    """Height of base_link above the floor; 0.0 for a URDF without wheels."""
    floor = wheel_floor_z(ET.fromstring(urdf))
    return 0.0 if floor is None else -floor


def origin(element: ET.Element) -> tuple[list[float], list[float]]:
    found = element.find("origin")
    if found is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    xyz = [float(v) for v in found.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in found.get("rpy", "0 0 0").split()]
    return xyz, rpy


def _wheel_collision_bottom(collision: ET.Element) -> float | None:
    """Lowest z of a wheel's cylinder/sphere collision in its link frame."""
    geometry = collision.find("geometry")
    if geometry is None:
        return None
    xyz, rpy = origin(collision)
    sphere = geometry.find("sphere")
    if sphere is not None:
        return xyz[2] - float(sphere.get("radius"))
    cylinder = geometry.find("cylinder")
    if cylinder is None:
        return None
    radius, length = float(cylinder.get("radius")), float(cylinder.get("length"))
    roll, pitch = abs(rpy[0]), abs(rpy[1])
    if abs(roll - 1.5708) < 0.01 or abs(pitch - 1.5708) < 0.01:  # axle horizontal
        return xyz[2] - radius
    if roll < 0.01 and pitch < 0.01:  # axle vertical
        return xyz[2] - length / 2.0
    return None
