# tests/test_design_contract.py
from __future__ import annotations

import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from design_fixtures import two_arm_design, two_arm_urdf
from tinker_designs.contract import ContractError, check_contract, check_footprint, require_contract
from tinker_designs.derive import derive_profile
from tinker_designs.model import parse_urdf
from tinker_designs.schema import design_from_mapping


def _design(**overrides):
    raw = two_arm_design()
    raw.update(overrides)
    return design_from_mapping(raw, source="robot.urdf")


def _set_attr(root: ET.Element, joint: str, tag: str, attribute: str, value: str) -> None:
    for element in root.findall("joint"):
        if element.get("name") == joint:
            element.find(tag).set(attribute, value)


class ContractTest(unittest.TestCase):
    def test_fixture_passes(self) -> None:
        self.assertEqual(check_contract(parse_urdf(two_arm_urdf()), _design()), [])

    def test_unclaimed_movable_joint_is_a_violation(self) -> None:
        raw = two_arm_design()
        raw["pan_tilt"] = None
        design = design_from_mapping(raw, source="robot.urdf")
        violations = check_contract(parse_urdf(two_arm_urdf()), design)
        self.assertTrue(any("pan_joint" in item and "unclaimed" in item for item in violations), violations)
        self.assertTrue(any("tilt_joint" in item for item in violations))

    def test_claimed_joint_missing_from_urdf(self) -> None:
        raw = two_arm_design()
        raw["arms"][1]["joints"].append("r_j4")
        violations = check_contract(parse_urdf(two_arm_urdf()), design_from_mapping(raw, source="robot.urdf"))
        self.assertTrue(any("r_j4" in item and "not in the URDF" in item for item in violations), violations)

    def test_driven_wheel_axis_must_be_y(self) -> None:
        root = parse_urdf(two_arm_urdf())
        _set_attr(root, "front_left_wheel_joint", "axis", "xyz", "1 0 0")
        violations = check_contract(root, _design())
        self.assertTrue(any("front_left_wheel_joint" in item and "axis" in item for item in violations), violations)

    def test_arm_mount_must_be_fixed_to_base(self) -> None:
        root = parse_urdf(two_arm_urdf())
        for element in root.findall("joint"):
            if element.get("name") == "left_arm_base_joint":
                element.set("type", "revolute")
                ET.SubElement(element, "axis", {"xyz": "0 0 1"})
                ET.SubElement(element, "limit", {"lower": "-1", "upper": "1", "effort": "1", "velocity": "1"})
        violations = check_contract(root, _design())
        self.assertTrue(any("left_arm_base" in item and "fixed" in item for item in violations), violations)

    def test_arm_chain_must_be_serial_from_mount(self) -> None:
        raw = two_arm_design()
        raw["arms"][0]["joints"] = ["l_j1", "l_j3", "l_j2"]
        violations = check_contract(parse_urdf(two_arm_urdf()), design_from_mapping(raw, source="robot.urdf"))
        self.assertTrue(any("left" in item and "serial" in item for item in violations), violations)

    def test_mimic_must_point_at_drive(self) -> None:
        root = parse_urdf(two_arm_urdf())
        for element in root.findall("joint"):
            if element.get("name") == "l_finger":
                element.find("mimic").set("joint", "l_j1")
        violations = check_contract(root, _design())
        self.assertTrue(any("l_finger" in item and "mimic" in item for item in violations), violations)

    def test_wheels_must_share_a_ground_plane(self) -> None:
        root = parse_urdf(two_arm_urdf())
        _set_attr(root, "rear_left_swivel_joint", "origin", "xyz", "-0.3 0.2 -0.09")
        violations = check_contract(root, _design())
        self.assertTrue(any("ground" in item for item in violations), violations)

    def test_inertia_must_be_positive_definite(self) -> None:
        root = parse_urdf(two_arm_urdf())
        for link in root.findall("link"):
            if link.get("name") == "l_link2":
                link.find("inertial/inertia").set("ixx", "-0.01")
        violations = check_contract(root, _design())
        self.assertTrue(any("l_link2" in item and "inertia" in item for item in violations), violations)

    def test_zero_mass_is_a_violation(self) -> None:
        root = parse_urdf(two_arm_urdf())
        for link in root.findall("link"):
            if link.get("name") == "livox_frame":
                link.find("inertial/mass").set("value", "0")
        violations = check_contract(root, _design())
        self.assertTrue(any("livox_frame" in item and "mass" in item for item in violations), violations)

    def test_world_root_with_zero_fixed_joint_is_accepted(self) -> None:
        root = parse_urdf(two_arm_urdf())
        root.insert(0, ET.Element("link", {"name": "world"}))
        joint = ET.SubElement(root, "joint", {"name": "world_joint", "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": "world"})
        ET.SubElement(joint, "child", {"link": "base_link"})
        ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        self.assertEqual(check_contract(root, _design()), [])

    def test_footprint_fixture_design_has_no_violation(self) -> None:
        profile = derive_profile(parse_urdf(two_arm_urdf()), _design())
        self.assertEqual(check_footprint(profile), [])

    def test_footprint_bow_tie_polygon_is_a_violation(self) -> None:
        profile = {"footprint": [[0.0, 0.0], [1.0, 1.0], [1.0, 0.0], [0.0, 1.0]], "cog_base_link": [0.5, 0.5, 0.1]}
        violations = check_footprint(profile)
        self.assertTrue(any("simple" in item for item in violations), violations)

    def test_footprint_square_not_containing_cog_is_a_violation(self) -> None:
        profile = {"footprint": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], "cog_base_link": [5.0, 5.0, 0.1]}
        violations = check_footprint(profile)
        self.assertTrue(any("CoG" in item for item in violations), violations)

    def test_require_contract_raises_with_all_violations(self) -> None:
        raw = two_arm_design()
        raw["pan_tilt"] = None
        with self.assertRaises(ContractError) as caught:
            require_contract(parse_urdf(two_arm_urdf()), design_from_mapping(raw, source="robot.urdf"))
        self.assertEqual(len(caught.exception.violations), 2)


if __name__ == "__main__":
    unittest.main()
