from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from design_fixtures import two_arm_urdf
from tinker_designs.model import (
    Origin, apply, compose, fixed_subtree, joints, links, movable_joints, parse_urdf, root_links, rotate_axis_angle,
)


class ModelTest(unittest.TestCase):
    def setUp(self) -> None:
        self.root = parse_urdf(two_arm_urdf())

    def test_links_and_joints_are_indexed_by_name(self) -> None:
        link_index = links(self.root)
        joint_index = joints(self.root)
        self.assertEqual(link_index["base_link"].inertial.mass, 20.0)
        self.assertEqual(link_index["front_left_wheel"].collisions[0][1].kind, "cylinder")
        self.assertEqual(link_index["front_left_wheel"].collisions[0][1].radius, 0.06)
        self.assertEqual(link_index["rear_left_swivel"].collisions, ())
        wheel = joint_index["front_left_wheel_joint"]
        self.assertEqual((wheel.type, wheel.parent, wheel.child), ("continuous", "base_link", "front_left_wheel"))
        self.assertEqual(wheel.origin.xyz, (0.0, 0.2, -0.05))
        self.assertEqual(wheel.axis, (0.0, 1.0, 0.0))
        self.assertEqual(joint_index["l_finger"].mimic, "l_grip")
        self.assertEqual(joint_index["l_j2"].lower, -1.57)
        self.assertIsNone(wheel.lower)

    def test_roots_and_movable_joints(self) -> None:
        self.assertEqual(root_links(self.root), ["base_link"])
        movable = movable_joints(self.root)
        self.assertIn("front_left_wheel_joint", movable)
        self.assertIn("l_finger", movable)
        self.assertNotIn("livox_joint", movable)
        self.assertEqual(len(movable), 6 + 6 + 2 + 2)  # wheels/casters, arm joints, gripper+mimic, pan/tilt

    def test_fixed_subtree_stops_at_movable_joints(self) -> None:
        subtree = fixed_subtree(self.root, "base_link")
        self.assertEqual(subtree, {"base_link", "left_arm_base", "right_arm_base", "livox_frame"})

    def test_transforms(self) -> None:
        origin = Origin((1.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2))
        x, y, z = apply(origin, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 1.0)
        self.assertAlmostEqual(z, 0.0)
        composed = compose(origin, Origin((1.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
        self.assertAlmostEqual(composed.xyz[0], 1.0)
        self.assertAlmostEqual(composed.xyz[1], 1.0)
        rotation = rotate_axis_angle((0.0, 1.0, 0.0), -math.pi / 2)
        vx, vy, vz = (sum(rotation[row][col] * (0.3, 0.0, 0.0)[col] for col in range(3)) for row in range(3))
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(vz, 0.3)


if __name__ == "__main__":
    unittest.main()
