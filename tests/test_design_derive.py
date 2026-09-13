from __future__ import annotations

import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from design_fixtures import two_arm_design, two_arm_urdf
from tinker_designs.derive import DeriveError, convex_hull, derive_profile
from tinker_designs.model import parse_urdf
from tinker_designs.schema import design_from_mapping


class DeriveTest(unittest.TestCase):
    def setUp(self) -> None:
        self.root = parse_urdf(two_arm_urdf())
        self.design = design_from_mapping(two_arm_design(), source="robot.urdf")

    def test_wheel_geometry(self) -> None:
        profile = derive_profile(self.root, self.design)
        self.assertAlmostEqual(profile["wheels"]["radius_m"], 0.06)
        self.assertAlmostEqual(profile["wheels"]["track_m"], 0.4)
        self.assertEqual(profile["wheels"]["driven"], ["front_left_wheel_joint", "front_right_wheel_joint"])

    def test_mass_and_cog(self) -> None:
        profile = derive_profile(self.root, self.design)
        self.assertAlmostEqual(profile["mass_kg"], 32.05)
        x, y, z = profile["cog_base_link"]
        self.assertAlmostEqual(y, 0.0, places=2)  # left gripper (0.2 kg) breaks symmetry by ~1 mm
        self.assertGreater(x, -0.05)
        self.assertLess(x, 0.15)
        self.assertGreater(z, 0.0)

    def test_footprint_is_hull_of_primitives_and_wheels(self) -> None:
        profile = derive_profile(self.root, self.design)
        self.assertEqual(profile["footprint_source"], "derived")
        xs = [x for x, _ in profile["footprint"]]
        ys = [y for _, y in profile["footprint"]]
        self.assertAlmostEqual(max(xs), 0.25)            # chassis box front
        self.assertAlmostEqual(min(xs), -0.36)           # rear caster wheel: -0.3 - 0.02 - 0.04
        self.assertAlmostEqual(max(ys), 0.225)           # front wheel: 0.2 + 0.05/2
        self.assertAlmostEqual(min(ys), -0.225)

    def test_footprint_override_wins_when_present(self) -> None:
        raw = two_arm_design()
        raw["footprint"] = [[0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1]]
        profile = derive_profile(self.root, design_from_mapping(raw, source="robot.urdf"))
        self.assertEqual(profile["footprint_source"], "override")
        self.assertEqual(profile["footprint"], [[0.1, 0.1], [0.1, -0.1], [-0.1, -0.1], [-0.1, 0.1]])

    def test_mesh_chassis_without_override_is_an_error(self) -> None:
        for link in self.root.findall("link"):
            if link.get("name") == "base_link":
                for collision in link.findall("collision"):
                    geometry = collision.find("geometry")
                    geometry.remove(geometry.find("box"))
                    ET.SubElement(geometry, "mesh", {"filename": "package://x/body.stl"})
        with self.assertRaisesRegex(DeriveError, "footprint"):
            derive_profile(self.root, self.design)

    def test_arm_reach_and_extended_cog(self) -> None:
        profile = derive_profile(self.root, self.design)
        left = profile["arms"][0]
        self.assertEqual(left["name"], "left")
        self.assertAlmostEqual(left["reach_m"], 0.45, places=3)   # j2 = -1.57 points link2 (0.3 m) straight up
        self.assertEqual(left["gripper"], {"drive": "l_grip", "mimics": ["l_finger"]})
        self.assertIsNone(profile["arms"][1]["gripper"])
        self.assertGreater(profile["cog_arms_extended"][2], profile["cog_base_link"][2])

    def test_convex_hull_is_ccw_and_drops_interior(self) -> None:
        hull = convex_hull([(0, 0), (1, 0), (1, 1), (0, 1), (0.5, 0.5), (0, 0)])
        self.assertEqual(hull, [(0, 0), (1, 0), (1, 1), (0, 1)])


if __name__ == "__main__":
    unittest.main()
