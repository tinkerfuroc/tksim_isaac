"""Draft a design.yaml from naming/topology conventions (design_import.py --init)."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

from .model import Joint, fixed_subtree, joints, links, root_links


def _axis_is(joint: Joint, index: int) -> bool:
    norm = math.sqrt(sum(c * c for c in joint.axis)) or 1.0
    return abs(joint.axis[index] / norm) > 0.99


def _arm_name(mount: str) -> str:
    if mount in ("link_base", "arm_base", "base"):
        return "arm"
    for suffix in ("_arm_base_link", "_arm_base", "_base_link", "_base"):
        if mount.endswith(suffix) and len(mount) > len(suffix):
            return mount[: -len(suffix)]
    return mount


def draft_design(root: ET.Element, name: str) -> dict:
    link_index = links(root)
    joint_index = joints(root)
    roots = root_links(root)
    base_frame = roots[0] if roots else "base_link"
    if base_frame == "world":
        base_frame = next((joint.child for joint in joint_index.values() if joint.parent == "world"), "base_link")
    chassis = fixed_subtree(root, base_frame)
    by_parent: dict[str, list[Joint]] = {}
    for joint in joint_index.values():
        by_parent.setdefault(joint.parent, []).append(joint)

    driven = sorted(
        (j.name for j in joint_index.values() if j.type == "continuous" and _axis_is(j, 1) and j.parent in chassis),
        key=lambda n: (0 if "front" in n else 1, n),
    )[:2]
    swivels = sorted(j.name for j in joint_index.values() if j.type == "continuous" and _axis_is(j, 2) and j.parent in chassis)
    caster_wheels = []
    for swivel_name in swivels:
        child = joint_index[swivel_name].child
        wheel = next((j.name for j in by_parent.get(child, []) if j.type == "continuous" and _axis_is(j, 1)), None)
        caster_wheels.append(wheel or "")

    arms = []
    for mount in sorted(chassis):
        if mount == base_frame:
            continue
        revolute_children = [j for j in by_parent.get(mount, []) if j.type == "revolute" and j.mimic is None]
        if len(revolute_children) != 1:
            continue
        chain = [revolute_children[0]]
        while True:
            nxt = [j for j in by_parent.get(chain[-1].child, []) if j.type == "revolute" and j.mimic is None]
            if len(nxt) == 1:
                chain.append(nxt[0])
                continue
            if not nxt:
                # No direct revolute continuation: look for a gripper reachable through
                # fixed pass-throughs only (never an unrelated actuated sibling branch).
                subtree = fixed_subtree(root, chain[-1].child)
                candidates = [
                    j
                    for j in joint_index.values()
                    if j.type == "revolute"
                    and j.mimic is None
                    and j.parent in subtree
                    and (
                        any(m.mimic == j.name for m in joint_index.values())
                        or any(token in j.name for token in ("grip", "finger", "hand"))
                    )
                ]
                if len(candidates) == 1:
                    chain.append(candidates[0])
            break
        if len(chain) < 3:
            continue
        # Split the gripper off: the last revolute whose child has a mimic sibling, or the last joint if ≥4 and named like a gripper.
        gripper = None
        arm_joints = chain
        drive_candidates = [j for j in chain[1:] if any(m.mimic == j.name for m in joint_index.values())]
        if drive_candidates:
            drive = drive_candidates[0]
            arm_joints = chain[: chain.index(drive)]
            gripper = {"drive": drive.name, "mimics": sorted(m.name for m in joint_index.values() if m.mimic == drive.name)}
        elif any(token in chain[-1].name for token in ("grip", "finger", "hand")):
            gripper = {"drive": chain[-1].name, "mimics": []}
            arm_joints = chain[:-1]
        arms.append({
            "name": _arm_name(mount), "mount": mount, "joints": [j.name for j in arm_joints],
            "gripper": gripper, "drive": {"stiffness": 400.0, "damping": 40.0},
        })

    pan_tilt = {"joints": ["pan_joint", "tilt_joint"]} if {"pan_joint", "tilt_joint"} <= set(joint_index) else None
    sensors = []
    for link_name in sorted(link_index):
        if "livox" in link_name:
            sensors.append({"type": "livox_mid360", "frame": link_name})
        elif link_name == "head_camera_link":
            sensors.append({"type": "head_camera", "frame": link_name})
    return {
        "name": name,
        "kinematics": "diff_drive",
        "base_frame": base_frame,
        "wheels": {"driven": driven, "caster_swivel": swivels, "caster_wheel": caster_wheels},
        "arms": arms,
        "pan_tilt": pan_tilt,
        "sensors": sensors,
    }
