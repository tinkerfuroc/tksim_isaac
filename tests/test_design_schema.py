# tests/test_design_schema.py
from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tinker_designs.schema import Design, DesignError, design_from_mapping, load_design

MINIMAL = {
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
         "gripper": {"drive": "l_grip", "mimics": ["l_finger"]},
         "drive": {"stiffness": 400.0, "damping": 40.0}},
        {"name": "right", "mount": "right_arm_base", "joints": ["r_j1", "r_j2", "r_j3"],
         "gripper": None, "drive": {"stiffness": 400.0, "damping": 40.0}},
    ],
    "pan_tilt": {"joints": ["pan_joint", "tilt_joint"]},
    "sensors": [{"type": "livox_mid360", "frame": "livox_frame"}],
}


class DesignSchemaTest(unittest.TestCase):
    def test_minimal_mapping_loads(self) -> None:
        design = design_from_mapping(MINIMAL, source="robot.urdf")
        self.assertIsInstance(design, Design)
        self.assertEqual(design.name, "two_arm_fixture")
        self.assertEqual(design.wheels.driven, ("front_left_wheel_joint", "front_right_wheel_joint"))
        self.assertEqual(design.arms[0].gripper.drive, "l_grip")
        self.assertIsNone(design.arms[1].gripper)
        self.assertEqual(design.arms[0].stiffness, 400.0)
        self.assertFalse(design.arms[0].estimated)
        self.assertEqual(design.pan_tilt, ("pan_joint", "tilt_joint"))
        self.assertIsNone(design.footprint)

    def test_kinematics_other_than_diff_drive_is_rejected(self) -> None:
        raw = dict(MINIMAL, kinematics="mecanum")
        with self.assertRaisesRegex(DesignError, "kinematics"):
            design_from_mapping(raw, source="robot.urdf")

    def test_duplicate_joint_claims_are_rejected(self) -> None:
        raw = dict(MINIMAL)
        raw["pan_tilt"] = {"joints": ["pan_joint", "l_j1"]}
        with self.assertRaisesRegex(DesignError, "l_j1"):
            design_from_mapping(raw, source="robot.urdf")

    def test_diff_drive_requires_exactly_two_driven_wheels(self) -> None:
        raw = dict(MINIMAL, wheels=dict(MINIMAL["wheels"], driven=["front_left_wheel_joint"]))
        with self.assertRaisesRegex(DesignError, "driven"):
            design_from_mapping(raw, source="robot.urdf")

    def test_footprint_override_is_parsed(self) -> None:
        raw = dict(MINIMAL, footprint=[[0.15, 0.25], [0.15, -0.25], [-0.35, -0.25], [-0.35, 0.25]])
        design = design_from_mapping(raw, source="robot.urdf")
        self.assertEqual(design.footprint[2], (-0.35, -0.25))

    def test_load_design_reads_yaml_and_finds_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            design_dir = Path(temporary) / "two_arm_fixture"
            design_dir.mkdir()
            (design_dir / "design.yaml").write_text(yaml.safe_dump(MINIMAL), encoding="utf-8")
            (design_dir / "robot.urdf").write_text("<robot name='x'/>", encoding="utf-8")
            design = load_design(design_dir)
            self.assertEqual(design.source, "robot.urdf")

    def test_load_design_requires_exactly_one_source(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            design_dir = Path(temporary) / "d"
            design_dir.mkdir()
            (design_dir / "design.yaml").write_text(yaml.safe_dump(MINIMAL), encoding="utf-8")
            with self.assertRaisesRegex(DesignError, "robot.urdf"):
                load_design(design_dir)
            (design_dir / "robot.urdf").write_text("<robot/>", encoding="utf-8")
            (design_dir / "robot.urdf.xacro").write_text("<robot/>", encoding="utf-8")
            with self.assertRaisesRegex(DesignError, "exactly one"):
                load_design(design_dir)

    def test_malformed_sensors_is_rejected(self) -> None:
        raw = dict(MINIMAL, sensors=5)
        with self.assertRaisesRegex(DesignError, "sensors"):
            design_from_mapping(raw, source="robot.urdf")

    def test_falsy_non_list_sensors_and_arms_are_rejected_not_masked_to_empty(self) -> None:
        for key, value in (("sensors", 0), ("sensors", False), ("arms", 0), ("arms", False)):
            with self.subTest(key=key, value=value):
                raw = dict(MINIMAL, **{key: value})
                with self.assertRaisesRegex(DesignError, key):
                    design_from_mapping(raw, source="robot.urdf")

    def test_malformed_pan_tilt_is_rejected(self) -> None:
        raw = dict(MINIMAL, pan_tilt="oops")
        with self.assertRaisesRegex(DesignError, "pan_tilt"):
            design_from_mapping(raw, source="robot.urdf")

    def test_missing_base_frame_is_rejected(self) -> None:
        raw = {key: value for key, value in MINIMAL.items() if key != "base_frame"}
        with self.assertRaisesRegex(DesignError, "base_frame"):
            design_from_mapping(raw, source="robot.urdf")

    def test_name_must_match_directory(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            design_dir = Path(temporary) / "other"
            design_dir.mkdir()
            (design_dir / "design.yaml").write_text(yaml.safe_dump(MINIMAL), encoding="utf-8")
            (design_dir / "robot.urdf").write_text("<robot/>", encoding="utf-8")
            with self.assertRaisesRegex(DesignError, "directory"):
                load_design(design_dir)


if __name__ == "__main__":
    unittest.main()
