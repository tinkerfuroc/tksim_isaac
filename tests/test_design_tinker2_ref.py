# tests/test_design_tinker2_ref.py
from __future__ import annotations

import hashlib
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from design_import import run_import
from tinker_designs.clean import canonical_bytes
from tinker_designs.model import parse_urdf

DESIGN = ROOT / "designs" / "tinker2_ref"

# sha256sum designs/tinker2_ref/robot.urdf, copied verbatim from the tinker2 artifact
# artifacts/robot/tinker2/347aef74…/robot.urdf (2026-09-13). Pins that the checked-in
# reference URDF has not drifted from the artifact it was copied from.
TINKER2_ARTIFACT_URDF_SHA256 = "e94dc5f1ca625f2a8621bc9f154e4a89f9a92451efae7bb5076f96c0820f40b3"


class Tinker2RefParityTest(unittest.TestCase):
    def test_reference_urdf_is_canonical_and_pipeline_preserves_it(self) -> None:
        source = (DESIGN / "robot.urdf").read_bytes()
        self.assertEqual(hashlib.sha256(source).hexdigest(), TINKER2_ARTIFACT_URDF_SHA256, "designs/tinker2_ref/robot.urdf has drifted from the tinker2 artifact it was copied from")
        self.assertEqual(canonical_bytes(parse_urdf(source)), source, "reference URDF must already be in canonical form")
        with tempfile.TemporaryDirectory() as temporary:
            result = run_import(DESIGN, ROOT, None, no_import=True, packages={}, work_dir=Path(temporary))
        self.assertEqual(result.canonical_urdf, source)
        self.assertIn(b"package://", result.canonical_urdf)

    def test_derived_profile_matches_workspace_constants(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            result = run_import(DESIGN, ROOT, None, no_import=True, packages={}, work_dir=Path(temporary))
        profile = result.profile
        self.assertAlmostEqual(profile["wheels"]["radius_m"], 0.0525)
        self.assertAlmostEqual(profile["wheels"]["track_m"], 0.25)
        self.assertEqual(profile["wheels"]["driven"], ["front_left_wheel_joint", "front_right_wheel_joint"])
        self.assertEqual(profile["footprint"], [[0.15, 0.25], [0.15, -0.25], [-0.35, -0.25], [-0.35, 0.25]])
        self.assertEqual(profile["footprint_source"], "override")
        self.assertEqual(profile["arms"][0]["joints"], [f"joint{i}" for i in range(1, 8)])
        self.assertEqual(profile["arms"][0]["gripper"]["drive"], "drive_joint")
        self.assertGreater(profile["mass_kg"], 60.0)
        self.assertGreater(profile["arms"][0]["reach_m"], 0.6)


if __name__ == "__main__":
    unittest.main()
