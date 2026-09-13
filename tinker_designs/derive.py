"""Derive the robot profile (kinematics, footprint, mass, reach) from a URDF + roles."""
from __future__ import annotations

import itertools
import math
import xml.etree.ElementTree as ET

from .model import Joint, Link, Origin, Vec3, apply, compose, compose_rotation, fixed_subtree, joints, links, parent_joint, rotate_axis_angle
from .schema import Design


class DeriveError(ValueError):
    """The profile cannot be derived from this URDF + design.yaml."""


def convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    unique = sorted(set(points))
    if len(unique) <= 2:
        return unique

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper: list[tuple[float, float]] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _frame_in_base(by_child: dict[str, Joint], link: str, base: str, angles: dict[str, float] | None = None) -> Origin:
    """Pose of `link` in `base` with the given joint angles (zero for unlisted joints)."""
    chain: list[Joint] = []
    current = link
    while current != base:
        joint = by_child.get(current)
        if joint is None:
            raise DeriveError(f"link {link!r} is not connected to {base!r}")
        chain.append(joint)
        current = joint.parent
    pose = Origin()
    for joint in reversed(chain):
        pose = compose(pose, joint.origin)
        angle = (angles or {}).get(joint.name, 0.0)
        if angle:
            pose = compose_rotation(pose, rotate_axis_angle(joint.axis, angle))
    return pose


def _radius(link: Link) -> float:
    for _, geometry in link.collisions:
        if geometry.kind in ("cylinder", "sphere") and geometry.radius:
            return geometry.radius
    raise DeriveError(f"wheel link {link.name!r} has no cylinder/sphere collision")


def _primitive_corners(origin: Origin, geometry) -> list[Vec3]:
    if geometry.kind == "box" and geometry.size:
        hx, hy, hz = (component / 2 for component in geometry.size)
        return [apply(origin, (sx * hx, sy * hy, sz * hz)) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    if geometry.kind == "cylinder" and geometry.radius is not None and geometry.length is not None:
        corners = []
        for angle in (k * math.pi / 8 for k in range(16)):
            for sz in (-1, 1):
                corners.append(apply(origin, (geometry.radius * math.cos(angle), geometry.radius * math.sin(angle), sz * geometry.length / 2)))
        return corners
    if geometry.kind == "sphere" and geometry.radius is not None:
        r = geometry.radius
        return [apply(origin, (sx * r, sy * r, 0.0)) for sx in (-1, 1) for sy in (-1, 1)]
    return []


def _footprint(root: ET.Element, design: Design, link_index: dict[str, Link], by_child: dict[str, Joint]) -> tuple[list[list[float]], str]:
    if design.footprint is not None:
        return [[x, y] for x, y in design.footprint], "override"
    chassis_link = link_index.get(design.base_frame)
    if chassis_link is None or not any(
        geometry.kind in ("box", "cylinder", "sphere") for _, geometry in chassis_link.collisions
    ):
        raise DeriveError(
            f"chassis link {design.base_frame!r} has no primitive collision geometry to derive a footprint from; "
            "add `footprint:` to design.yaml"
        )
    points: list[tuple[float, float]] = []
    chassis = fixed_subtree(root, design.base_frame)
    for name in chassis:
        pose = _frame_in_base(by_child, name, design.base_frame)
        for origin, geometry in link_index[name].collisions:
            for corner in _primitive_corners(compose(pose, origin), geometry):
                points.append((corner[0], corner[1]))
    joint_index = joints(root)
    for name in design.wheels.driven + design.wheels.caster_wheel:
        child = joint_index[name].child
        pose = _frame_in_base(by_child, child, design.base_frame)
        for origin, geometry in link_index[child].collisions:
            for corner in _primitive_corners(compose(pose, origin), geometry):
                points.append((corner[0], corner[1]))
    hull = convex_hull(points)
    if len(hull) < 3:
        raise DeriveError("derived footprint is degenerate; add `footprint:` to design.yaml")
    return [[round(x, 6), round(y, 6)] for x, y in hull], "derived"


def _mass_and_cog(link_index: dict[str, Link], by_child: dict[str, Joint], base: str, angles: dict[str, float]) -> tuple[float, Vec3]:
    total = 0.0
    weighted = [0.0, 0.0, 0.0]
    for link in link_index.values():
        if link.inertial is None or link.inertial.mass <= 0:
            continue
        if link.name != base and link.name not in by_child:
            continue  # e.g. the `world` link
        pose = _frame_in_base(by_child, link.name, base, angles)
        centre = apply(pose, link.inertial.origin.xyz)
        total += link.inertial.mass
        for axis in range(3):
            weighted[axis] += link.inertial.mass * centre[axis]
    if total <= 0:
        raise DeriveError("robot has no mass")
    return total, (weighted[0] / total, weighted[1] / total, weighted[2] / total)


def _reach(arm, joint_index: dict[str, Joint], by_child: dict[str, Joint], base: str) -> tuple[float, dict[str, float]]:
    mount = _frame_in_base(by_child, arm.mount, base).xyz
    last_link = joint_index[arm.joints[-1]].child
    corners = []
    for name in arm.joints:
        joint = joint_index[name]
        lower = joint.lower if joint.lower is not None else -math.pi
        upper = joint.upper if joint.upper is not None else math.pi
        corners.append((lower, 0.0, upper))
    best, best_angles = 0.0, {}
    for combination in itertools.product(*corners):
        angles = dict(zip(arm.joints, combination))
        tip = _frame_in_base(by_child, last_link, base, angles).xyz
        distance = math.dist(tip, mount)
        if distance > best:
            best, best_angles = distance, angles
    return best, best_angles


def derive_profile(root: ET.Element, design: Design) -> dict:
    link_index = links(root)
    joint_index = joints(root)
    by_child = parent_joint(root)
    left, right = (joint_index[name] for name in design.wheels.driven)
    radii = {_radius(link_index[joint.child]) for joint in (left, right)}
    if len(radii) != 1:
        raise DeriveError(f"driven wheels have different radii: {sorted(radii)}")
    centres = [_frame_in_base(by_child, joint.child, design.base_frame).xyz for joint in (left, right)]
    track = abs(centres[0][1] - centres[1][1])
    footprint, footprint_source = _footprint(root, design, link_index, by_child)
    mass, cog = _mass_and_cog(link_index, by_child, design.base_frame, {})
    arms = []
    extended_angles: dict[str, float] = {}
    for arm in design.arms:
        reach, angles = _reach(arm, joint_index, by_child, design.base_frame)
        extended_angles.update(angles)
        arms.append({
            "name": arm.name, "mount": arm.mount, "joints": list(arm.joints),
            "gripper": None if arm.gripper is None else {"drive": arm.gripper.drive, "mimics": list(arm.gripper.mimics)},
            "drive": {"stiffness": arm.stiffness, "damping": arm.damping},
            "reach_m": round(reach, 6), "estimated": arm.estimated,
        })
    _, cog_extended = _mass_and_cog(link_index, by_child, design.base_frame, extended_angles)
    return {
        "robot": design.name,
        "kinematics": design.kinematics,
        "base_frame": design.base_frame,
        "wheels": {
            "driven": list(design.wheels.driven),
            "caster_swivel": list(design.wheels.caster_swivel),
            "caster_wheel": list(design.wheels.caster_wheel),
            "radius_m": round(radii.pop(), 6),
            "track_m": round(track, 6),
        },
        "arms": arms,
        "pan_tilt": {"joints": list(design.pan_tilt)} if design.pan_tilt else None,
        "sensors": [{"type": sensor.type, "frame": sensor.frame} for sensor in design.sensors],
        "footprint": footprint,
        "footprint_source": footprint_source,
        "mass_kg": round(mass, 6),
        "cog_base_link": [round(component, 6) for component in cog],
        "cog_arms_extended": [round(component, 6) for component in cog_extended],
    }
