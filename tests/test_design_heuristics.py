from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from design_fixtures import two_arm_design, two_arm_urdf
from tinker_designs.contract import check_contract
from tinker_designs.heuristics import draft_design
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


if __name__ == "__main__":
    unittest.main()
