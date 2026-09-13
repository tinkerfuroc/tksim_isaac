"""A two-arm, diff-drive primitive robot used by every tinker_designs test."""
from __future__ import annotations

import xml.etree.ElementTree as ET


def _link(root: ET.Element, name: str, mass: float, geometry: ET.Element | None = None,
          origin: tuple[str, str] = ("0 0 0", "0 0 0"), inertia: tuple[str, ...] = ("0.01",) * 3) -> None:
    link = ET.SubElement(root, "link", {"name": name})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": str(mass)})
    ET.SubElement(inertial, "inertia", {"ixx": inertia[0], "iyy": inertia[1], "izz": inertia[2], "ixy": "0", "ixz": "0", "iyz": "0"})
    if geometry is not None:
        for tag in ("visual", "collision"):
            element = ET.SubElement(link, tag)
            ET.SubElement(element, "origin", {"xyz": origin[0], "rpy": origin[1]})
            ET.SubElement(element, "geometry").append(ET.fromstring(ET.tostring(geometry)))


def _joint(root: ET.Element, name: str, type_: str, parent: str, child: str, xyz: str,
           axis: str | None = None, limits: tuple[str, str] | None = None, mimic: str | None = None) -> None:
    joint = ET.SubElement(root, "joint", {"name": name, "type": type_})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": "0 0 0"})
    if axis is not None:
        ET.SubElement(joint, "axis", {"xyz": axis})
    if type_ == "continuous":
        ET.SubElement(joint, "limit", {"effort": "10", "velocity": "20"})
    if limits is not None:
        ET.SubElement(joint, "limit", {"lower": limits[0], "upper": limits[1], "effort": "10", "velocity": "2"})
    if mimic is not None:
        ET.SubElement(joint, "mimic", {"joint": mimic, "multiplier": "1", "offset": "0"})


def two_arm_urdf() -> bytes:
    root = ET.Element("robot", {"name": "two_arm_fixture"})
    box = ET.Element("box", {"size": "0.5 0.4 0.2"})
    front = ET.Element("cylinder", {"radius": "0.06", "length": "0.05"})
    rear = ET.Element("cylinder", {"radius": "0.04", "length": "0.03"})
    _link(root, "base_link", 20.0, box)
    for side, y in (("left", "0.2"), ("right", "-0.2")):
        _link(root, f"front_{side}_wheel", 1.0, front, ("0 0 0", "1.5708 0 0"))
        _joint(root, f"front_{side}_wheel_joint", "continuous", "base_link", f"front_{side}_wheel", f"0 {y} -0.05", "0 1 0")
        _link(root, f"rear_{side}_swivel", 0.1)
        _joint(root, f"rear_{side}_swivel_joint", "continuous", "base_link", f"rear_{side}_swivel", f"-0.3 {y} -0.07", "0 0 1")
        _link(root, f"rear_{side}_wheel", 1.0, rear, ("0 0 0", "1.5708 0 0"))
        _joint(root, f"rear_{side}_wheel_joint", "continuous", f"rear_{side}_swivel", f"rear_{side}_wheel", "-0.02 0 0", "0 1 0")
    for prefix, y in (("l", "0.15"), ("r", "-0.15")):
        base = f"{'left' if prefix == 'l' else 'right'}_arm_base"
        _link(root, base, 0.5)
        _joint(root, f"{base}_joint", "fixed", "base_link", base, f"0.1 {y} 0.1")
        _link(root, f"{prefix}_link1", 1.0)
        _joint(root, f"{prefix}_j1", "revolute", base, f"{prefix}_link1", "0 0 0.05", "0 0 1", ("-3.14", "3.14"))
        _link(root, f"{prefix}_link2", 1.0)
        _joint(root, f"{prefix}_j2", "revolute", f"{prefix}_link1", f"{prefix}_link2", "0 0 0.1", "0 1 0", ("-1.57", "1.57"))
        _link(root, f"{prefix}_link3", 1.0)
        _joint(root, f"{prefix}_j3", "revolute", f"{prefix}_link2", f"{prefix}_link3", "0.3 0 0", "0 1 0", ("-1.57", "1.57"))
    _link(root, "l_gripper", 0.1)
    _joint(root, "l_grip", "revolute", "l_link3", "l_gripper", "0.3 0 0", "0 0 1", ("0", "0.8"))
    _link(root, "l_finger_link", 0.1)
    _joint(root, "l_finger", "revolute", "l_gripper", "l_finger_link", "0.02 0 0", "0 0 1", ("0", "0.8"), mimic="l_grip")
    _link(root, "pan_link", 0.2)
    _joint(root, "pan_joint", "revolute", "base_link", "pan_link", "0 0 0.3", "0 0 1", ("-1.5", "1.5"))
    _link(root, "tilt_link", 0.2)
    _joint(root, "tilt_joint", "revolute", "pan_link", "tilt_link", "0 0 0.05", "0 1 0", ("-0.5", "0.5"))
    _link(root, "livox_frame", 0.25, ET.Element("box", {"size": "0.06 0.1 0.06"}))
    _joint(root, "livox_joint", "fixed", "base_link", "livox_frame", "0.2 0 0.15")
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def two_arm_design() -> dict:
    return {
        "name": "two_arm_fixture",
        "kinematics": "diff_drive",
        "base_frame": "base_link",
        "wheels": {
            "driven": ["front_left_wheel_joint", "front_right_wheel_joint"],
            "caster_swivel": ["rear_left_swivel_joint", "rear_right_swivel_joint"],
            "caster_wheel": ["rear_left_wheel_joint", "rear_right_wheel_joint"],
        },
        "arms": [
            {"name": "left", "mount": "left_arm_base", "joints": ["l_j1", "l_j2", "l_j3"],
             "gripper": {"drive": "l_grip", "mimics": ["l_finger"]}, "drive": {"stiffness": 400.0, "damping": 40.0}},
            {"name": "right", "mount": "right_arm_base", "joints": ["r_j1", "r_j2", "r_j3"],
             "gripper": None, "drive": {"stiffness": 400.0, "damping": 40.0}},
        ],
        "pan_tilt": {"joints": ["pan_joint", "tilt_joint"]},
        "sensors": [{"type": "livox_mid360", "frame": "livox_frame"}],
    }
