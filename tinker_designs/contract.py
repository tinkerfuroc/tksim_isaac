# tinker_designs/contract.py
"""Spec §5: what a candidate URDF must satisfy given its design.yaml roles."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from .model import Inertial, Joint, Link, apply, fixed_subtree, joints, links, movable_joints, parent_joint, root_links
from .schema import Design

GROUND_TOLERANCE_M = 0.001


class ContractError(ValueError):
    def __init__(self, violations: list[str]) -> None:
        super().__init__("contract violations:\n  " + "\n  ".join(violations))
        self.violations = violations


def _axis_is(joint: Joint, axis_index: int) -> bool:
    norm = math.sqrt(sum(c * c for c in joint.axis)) or 1.0
    return abs(joint.axis[axis_index] / norm) > 0.99


def _positive_definite(inertial: Inertial) -> bool:
    a, b, c = inertial.ixx, inertial.iyy, inertial.izz
    d, e, f = inertial.ixy, inertial.ixz, inertial.iyz
    minor2 = a * b - d * d
    det = a * (b * c - f * f) - d * (d * c - f * e) + e * (d * f - b * e)
    return a > 0 and minor2 > 0 and det > 0


def _link_origin_in_base(root_map: dict[str, Joint], link: str, base: str, angle_zero: bool = True) -> tuple[float, float, float] | None:
    """Position of `link`'s frame origin in `base`'s frame at zero joint angles."""
    point = (0.0, 0.0, 0.0)
    current = link
    guard = 0
    while current != base:
        joint = root_map.get(current)
        if joint is None or guard > 256:
            return None
        point = apply(joint.origin, point)
        current = joint.parent
        guard += 1
    return point


def _wheel_radius(link: Link) -> float | None:
    for _, geometry in link.collisions:
        if geometry.kind == "cylinder" and geometry.radius:
            return geometry.radius
        if geometry.kind == "sphere" and geometry.radius:
            return geometry.radius
    return None


def check_contract(root: ET.Element, design: Design) -> list[str]:
    violations: list[str] = []
    link_index = links(root)
    joint_index = joints(root)
    by_child = parent_joint(root)

    # Root: base_frame, or world -> base_frame via a zero fixed joint.
    roots = root_links(root)
    if roots == ["world"]:
        world_joint = by_child.get(design.base_frame)
        if world_joint is None or world_joint.parent != "world" or world_joint.type != "fixed" or any(abs(v) > 1e-9 for v in world_joint.origin.xyz + world_joint.origin.rpy):
            violations.append(f"world must connect to {design.base_frame} by a zero fixed joint")
    elif roots != [design.base_frame]:
        violations.append(f"URDF root must be {design.base_frame!r}, found {roots}")
    if design.base_frame not in link_index:
        violations.append(f"base_frame {design.base_frame!r} is not a link")
        return violations

    # Every movable joint claimed exactly once; every claim exists.
    claims = design.claimed_joints()
    for name in movable_joints(root):
        if name not in claims:
            violations.append(f"movable joint {name!r} is unclaimed by design.yaml")
    unusable = False
    for name, role in claims.items():
        if name not in joint_index:
            violations.append(f"{role} joint {name!r} is not in the URDF")
            unusable = True
        elif joint_index[name].type == "fixed":
            violations.append(f"{role} joint {name!r} is fixed")
            unusable = True
    if unusable:
        return violations

    fixed_from_base = fixed_subtree(root, design.base_frame)

    # Wheels.
    contact_heights: list[tuple[str, float]] = []
    for name in design.wheels.driven:
        joint = joint_index[name]
        if joint.type != "continuous":
            violations.append(f"driven wheel {name!r} must be continuous")
        if not _axis_is(joint, 1):
            violations.append(f"driven wheel {name!r} axis must be ±Y")
        if joint.parent not in fixed_from_base:
            violations.append(f"driven wheel {name!r} must hang off the fixed chassis subtree")
        radius = _wheel_radius(link_index[joint.child])
        centre = _link_origin_in_base(by_child, joint.child, design.base_frame)
        if radius is None or centre is None:
            violations.append(f"driven wheel {name!r} needs a cylinder/sphere collision on {joint.child!r}")
        else:
            contact_heights.append((name, centre[2] - radius))
    for swivel_name, wheel_name in zip(design.wheels.caster_swivel, design.wheels.caster_wheel):
        swivel = joint_index[swivel_name]
        wheel = joint_index[wheel_name]
        if swivel.type != "continuous" or not _axis_is(swivel, 2):
            violations.append(f"caster swivel {swivel_name!r} must be continuous about ±Z")
        if swivel.parent not in fixed_from_base:
            violations.append(f"caster swivel {swivel_name!r} must hang off the fixed chassis subtree")
        if wheel.type != "continuous" or not _axis_is(wheel, 1):
            violations.append(f"caster wheel {wheel_name!r} must be continuous about ±Y")
        if wheel.parent != swivel.child:
            violations.append(f"caster wheel {wheel_name!r} must be the child of swivel {swivel_name!r}")
        radius = _wheel_radius(link_index[wheel.child])
        centre = _link_origin_in_base(by_child, wheel.child, design.base_frame)
        if radius is None or centre is None:
            violations.append(f"caster wheel {wheel_name!r} needs a cylinder/sphere collision on {wheel.child!r}")
        else:
            contact_heights.append((wheel_name, centre[2] - radius))
    if contact_heights:
        lowest = min(height for _, height in contact_heights)
        for name, height in contact_heights:
            if abs(height - lowest) > GROUND_TOLERANCE_M:
                violations.append(f"wheel {name!r} ground contact z={height:.4f} differs from {lowest:.4f}; wheels must share a ground plane")

    # Arms.
    for arm in design.arms:
        if arm.mount not in link_index:
            violations.append(f"arm {arm.name!r} mount {arm.mount!r} is not a link")
            continue
        if arm.mount not in fixed_from_base:
            violations.append(f"arm {arm.name!r} mount {arm.mount!r} must be fixed to the chassis (fixed joints only from {design.base_frame})")
        expected_parent = arm.mount
        for name in arm.joints:
            joint = joint_index[name]
            if joint.parent != expected_parent:
                violations.append(f"arm {arm.name!r} joints must form a serial chain from {arm.mount!r}; {name!r} hangs off {joint.parent!r}, expected {expected_parent!r}")
                break
            expected_parent = joint.child
        last_link = joint_index[arm.joints[-1]].child
        if arm.gripper is not None:
            drive = joint_index[arm.gripper.drive]
            chain_link = drive.parent
            guard = 0
            while chain_link not in (last_link, design.base_frame) and chain_link in by_child and guard < 64:
                chain_link = by_child[chain_link].parent
                guard += 1
            if chain_link != last_link:
                violations.append(f"arm {arm.name!r} gripper drive {arm.gripper.drive!r} must descend from {last_link!r}")
            for mimic_name in arm.gripper.mimics:
                if joint_index[mimic_name].mimic != arm.gripper.drive:
                    violations.append(f"gripper mimic {mimic_name!r} must carry <mimic joint={arm.gripper.drive!r}>")

    # Sensors.
    for sensor in design.sensors:
        if sensor.frame not in link_index:
            violations.append(f"sensor {sensor.type!r} frame {sensor.frame!r} is not a link")

    # Inertia sanity.
    for link in link_index.values():
        if link.inertial is None:
            continue
        if link.inertial.mass <= 0:
            violations.append(f"link {link.name!r} mass must be > 0")
        elif not _positive_definite(link.inertial):
            violations.append(f"link {link.name!r} inertia tensor is not positive definite")
    return violations


def require_contract(root: ET.Element, design: Design) -> None:
    violations = check_contract(root, design)
    if violations:
        raise ContractError(violations)
