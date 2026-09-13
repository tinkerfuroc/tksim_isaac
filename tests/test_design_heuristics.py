from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from design_fixtures import two_arm_design, two_arm_urdf
from tinker_designs.contract import check_contract
from tinker_designs.heuristics import _arm_name, draft_design
from tinker_designs.model import parse_urdf
from tinker_designs.schema import design_from_mapping


class HeuristicsTest(unittest.TestCase):
    def test_draft_reproduces_fixture_roles(self) -> None:
        root = parse_urdf(two_arm_urdf())
        draft = draft_design(root, "two_arm_fixture")
        expected = two_arm_design()
        self.assertEqual(draft["wheels"], expected["wheels"])
        self.assertEqual([arm["name"] for arm in draft["arms"]], ["left", "right"])
        self.assertEqual(draft["arms"][0]["joints"], ["l_j1", "l_j2", "l_j3"])
        self.assertEqual(draft["arms"][0]["gripper"], {"drive": "l_grip", "mimics": ["l_finger"]})
        self.assertIsNone(draft["arms"][1]["gripper"])
        self.assertEqual(draft["pan_tilt"], {"joints": ["pan_joint", "tilt_joint"]})
        self.assertEqual(draft["sensors"], [{"type": "livox_mid360", "frame": "livox_frame"}])
        design = design_from_mapping(draft, source="robot.urdf")
        self.assertEqual(check_contract(root, design), [])

    def test_draft_leaves_unknown_joints_unclaimed(self) -> None:
        root = parse_urdf(two_arm_urdf())
        import xml.etree.ElementTree as ET
        link = ET.SubElement(root, "link", {"name": "lift"})
        ET.SubElement(ET.SubElement(link, "inertial"), "mass", {"value": "1"})
        joint = ET.SubElement(root, "joint", {"name": "lift_joint", "type": "prismatic"})
        ET.SubElement(joint, "parent", {"link": "base_link"})
        ET.SubElement(joint, "child", {"link": "lift"})
        ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
        ET.SubElement(joint, "limit", {"lower": "0", "upper": "0.3", "effort": "1", "velocity": "1"})
        draft = draft_design(root, "two_arm_fixture")
        design = design_from_mapping(draft, source="robot.urdf")
        violations = check_contract(root, design)
        self.assertTrue(any("lift_joint" in item and "unclaimed" in item for item in violations), violations)


    def test_draft_follows_fixed_passthrough_to_gripper(self) -> None:
        import xml.etree.ElementTree as ET

        def link(root: ET.Element, name: str, mass: float | None = None, geometry: ET.Element | None = None) -> None:
            element = ET.SubElement(root, "link", {"name": name})
            if mass is not None:
                inertial = ET.SubElement(element, "inertial")
                ET.SubElement(inertial, "mass", {"value": str(mass)})
                ET.SubElement(inertial, "inertia", {"ixx": "0.01", "iyy": "0.01", "izz": "0.01", "ixy": "0", "ixz": "0", "iyz": "0"})
            if geometry is not None:
                collision = ET.SubElement(element, "collision")
                ET.SubElement(collision, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
                ET.SubElement(collision, "geometry").append(ET.fromstring(ET.tostring(geometry)))

        def joint(root: ET.Element, name: str, type_: str, parent: str, child: str, xyz: str,
                  axis: str | None = None, limits: tuple[str, str] | None = None, mimic: str | None = None) -> None:
            element = ET.SubElement(root, "joint", {"name": name, "type": type_})
            ET.SubElement(element, "parent", {"link": parent})
            ET.SubElement(element, "child", {"link": child})
            ET.SubElement(element, "origin", {"xyz": xyz, "rpy": "0 0 0"})
            if axis is not None:
                ET.SubElement(element, "axis", {"xyz": axis})
            if type_ == "continuous":
                ET.SubElement(element, "limit", {"effort": "10", "velocity": "20"})
            if limits is not None:
                ET.SubElement(element, "limit", {"lower": limits[0], "upper": limits[1], "effort": "10", "velocity": "2"})
            if mimic is not None:
                ET.SubElement(element, "mimic", {"joint": mimic, "multiplier": "1", "offset": "0"})

        root = ET.Element("robot", {"name": "tinker2_fixture"})
        link(root, "base_link", 20.0, ET.Element("box", {"size": "0.5 0.4 0.2"}))
        link(root, "link_base", 1.0)
        joint(root, "base_to_arm_joint", "fixed", "base_link", "link_base", "0.1 0 0.1")
        link(root, "link1", 1.0)
        joint(root, "joint1", "revolute", "link_base", "link1", "0 0 0.05", "0 0 1", ("-1", "1"))
        link(root, "link2", 1.0)
        joint(root, "joint2", "revolute", "link1", "link2", "0 0 0.1", "0 0 1", ("-1", "1"))
        link(root, "link3", 1.0)
        joint(root, "joint3", "revolute", "link2", "link3", "0 0 0.1", "0 0 1", ("-1", "1"))
        link(root, "link_eef", 0.1)
        joint(root, "joint_eef", "fixed", "link3", "link_eef", "0.05 0 0")
        link(root, "gripper_base", 0.1)
        joint(root, "gripper_fix", "fixed", "link_eef", "gripper_base", "0.02 0 0")
        link(root, "outer", 0.1)
        joint(root, "drive_joint", "revolute", "gripper_base", "outer", "0.02 0 0", "0 0 1", ("0", "0.8"))
        link(root, "finger", 0.05)
        joint(root, "finger_joint", "revolute", "outer", "finger", "0.02 0 0", "0 0 1", ("0", "0.8"), mimic="drive_joint")
        cylinder = ET.Element("cylinder", {"radius": "0.05", "length": "0.05"})
        link(root, "front_left_wheel", 1.0, cylinder)
        joint(root, "front_left_wheel_joint", "continuous", "base_link", "front_left_wheel", "0 0.2 -0.05", "0 1 0")
        link(root, "front_right_wheel", 1.0, cylinder)
        joint(root, "front_right_wheel_joint", "continuous", "base_link", "front_right_wheel", "0 -0.2 -0.05", "0 1 0")

        draft = draft_design(root, "tinker2_fixture")
        self.assertEqual(len(draft["arms"]), 1)
        arm = draft["arms"][0]
        self.assertEqual(arm["name"], "arm")
        self.assertEqual(arm["mount"], "link_base")
        self.assertEqual(arm["joints"], ["joint1", "joint2", "joint3"])
        self.assertEqual(arm["gripper"], {"drive": "drive_joint", "mimics": ["finger_joint"]})
        design = design_from_mapping(draft, source="robot.urdf")
        self.assertEqual(check_contract(root, design), [])

    def test_draft_ignores_actuated_sibling_branch(self) -> None:
        import xml.etree.ElementTree as ET

        def link(root: ET.Element, name: str, mass: float | None = None, geometry: ET.Element | None = None) -> None:
            element = ET.SubElement(root, "link", {"name": name})
            if mass is not None:
                inertial = ET.SubElement(element, "inertial")
                ET.SubElement(inertial, "mass", {"value": str(mass)})
                ET.SubElement(inertial, "inertia", {"ixx": "0.01", "iyy": "0.01", "izz": "0.01", "ixy": "0", "ixz": "0", "iyz": "0"})
            if geometry is not None:
                collision = ET.SubElement(element, "collision")
                ET.SubElement(collision, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
                ET.SubElement(collision, "geometry").append(ET.fromstring(ET.tostring(geometry)))

        def joint(root: ET.Element, name: str, type_: str, parent: str, child: str, xyz: str,
                  axis: str | None = None, limits: tuple[str, str] | None = None) -> None:
            element = ET.SubElement(root, "joint", {"name": name, "type": type_})
            ET.SubElement(element, "parent", {"link": parent})
            ET.SubElement(element, "child", {"link": child})
            ET.SubElement(element, "origin", {"xyz": xyz, "rpy": "0 0 0"})
            if axis is not None:
                ET.SubElement(element, "axis", {"xyz": axis})
            if type_ == "continuous":
                ET.SubElement(element, "limit", {"effort": "10", "velocity": "20"})
            if limits is not None:
                ET.SubElement(element, "limit", {"lower": limits[0], "upper": limits[1], "effort": "10", "velocity": "2"})

        root = ET.Element("robot", {"name": "sibling_branch_fixture"})
        link(root, "base_link", 20.0, ET.Element("box", {"size": "0.5 0.4 0.2"}))
        link(root, "link_base", 1.0)
        joint(root, "base_to_arm_joint", "fixed", "base_link", "link_base", "0.1 0 0.1")
        link(root, "link1", 1.0)
        joint(root, "joint1", "revolute", "link_base", "link1", "0 0 0.05", "0 0 1", ("-1", "1"))
        link(root, "link2", 1.0)
        joint(root, "joint2", "revolute", "link1", "link2", "0 0 0.1", "0 0 1", ("-1", "1"))
        link(root, "link3", 1.0)
        joint(root, "joint3", "revolute", "link2", "link3", "0 0 0.1", "0 0 1", ("-1", "1"))
        link(root, "link_eef", 0.1)
        joint(root, "joint_eef", "fixed", "link3", "link_eef", "0.05 0 0")
        link(root, "tool_tip", 0.05)
        joint(root, "tip_joint", "fixed", "link_eef", "tool_tip", "0.02 0 0")
        link(root, "wrist_cam_mount", 0.05)
        joint(root, "wrist_cam_joint", "fixed", "link_eef", "wrist_cam_mount", "0 0.02 0")
        link(root, "wrist_cam_link", 0.05)
        joint(root, "wrist_cam_tilt_joint", "revolute", "wrist_cam_mount", "wrist_cam_link", "0.01 0 0", "0 1 0", ("-0.5", "0.5"))
        cylinder = ET.Element("cylinder", {"radius": "0.05", "length": "0.05"})
        link(root, "front_left_wheel", 1.0, cylinder)
        joint(root, "front_left_wheel_joint", "continuous", "base_link", "front_left_wheel", "0 0.2 -0.05", "0 1 0")
        link(root, "front_right_wheel", 1.0, cylinder)
        joint(root, "front_right_wheel_joint", "continuous", "base_link", "front_right_wheel", "0 -0.2 -0.05", "0 1 0")

        draft = draft_design(root, "sibling_branch_fixture")
        self.assertEqual(len(draft["arms"]), 1)
        arm = draft["arms"][0]
        self.assertEqual(arm["joints"], ["joint1", "joint2", "joint3"])
        self.assertIsNone(arm["gripper"])
        design = design_from_mapping(draft, source="robot.urdf")
        violations = check_contract(root, design)
        self.assertTrue(any("wrist_cam_tilt_joint" in item and "unclaimed" in item for item in violations), violations)

    def test_caster_swivel_without_a_wheel_is_omitted_not_emitted_as_empty_string(self) -> None:
        root = parse_urdf(two_arm_urdf())
        for element in root.findall("joint"):
            if element.get("name") == "rear_left_wheel_joint":
                element.set("type", "fixed")
        draft = draft_design(root, "two_arm_fixture")
        self.assertNotIn("", draft["wheels"]["caster_wheel"])
        self.assertEqual(draft["wheels"]["caster_swivel"], ["rear_right_swivel_joint"])
        self.assertEqual(draft["wheels"]["caster_wheel"], ["rear_right_wheel_joint"])
        self.assertEqual(draft["caster_wheel_todo"], ["rear_left_swivel_joint"])

    def test_arm_name_strips_suffixes(self) -> None:
        self.assertEqual(_arm_name("link_base"), "arm")
        self.assertEqual(_arm_name("arm_base"), "arm")
        self.assertEqual(_arm_name("left_arm_base"), "left")
        self.assertEqual(_arm_name("right_arm_base_link"), "right")
        self.assertEqual(_arm_name("ur5_base"), "ur5")
        self.assertEqual(_arm_name("tool"), "tool")


if __name__ == "__main__":
    unittest.main()
