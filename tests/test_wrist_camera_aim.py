"""The wrist camera's opt-in aim correction must point the render axis at the tool.

The robot description mounts the wrist camera with its optical axis exactly
90 deg off the tool approach axis (measured 2026-08-31 from the artifact's
robot.urdf FK: at joint zeros the gripper points straight down while the
camera looks level; at the table-scan pose the TCP aims -48 deg while the
camera looks +42 deg up -- ceiling frames, so every live grasp fell back to
the referee). The ``tool-forward`` preset rotates the camera +90 deg about
the optical frame's own +X, mapping the render axis onto -Y_optical, which
equals the TCP forward at the scan pose to three decimals. As with the
head correction: opt-in, sim-only, deliberate parity break.
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.head_camera_aim import (  # noqa: E402
    TOOL_FORWARD_CORRECTION_WXYZ,
    WRIST_CAMERA_NAME,
    apply_head_aim_correction,
    resolve_wrist_aim_correction,
)
from tinker_sim_isaac.camera_rig import load_camera_specs  # noqa: E402

CONTRACT = ROOT / "simulation/sensors/hardware-parity.json"


def _rotate(quaternion, vector):
    w, x, y, z = quaternion
    vx, vy, vz = vector
    # q * (0, v) * q^-1 expanded.
    tx, ty, tz = (
        2.0 * (y * vz - z * vy),
        2.0 * (z * vx - x * vz),
        2.0 * (x * vy - y * vx),
    )
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class ResolveTest(unittest.TestCase):
    def test_unset_is_off(self):
        self.assertIsNone(resolve_wrist_aim_correction(None))
        self.assertIsNone(resolve_wrist_aim_correction(""))
        self.assertIsNone(resolve_wrist_aim_correction("   "))

    def test_preset_resolves_to_the_solved_correction(self):
        self.assertEqual(
            resolve_wrist_aim_correction("tool-forward"), TOOL_FORWARD_CORRECTION_WXYZ
        )
        self.assertEqual(
            resolve_wrist_aim_correction(" Tool-Forward "), TOOL_FORWARD_CORRECTION_WXYZ
        )

    def test_garbage_raises(self):
        with self.assertRaises(ValueError):
            resolve_wrist_aim_correction("toolforward")
        with self.assertRaises(ValueError):
            resolve_wrist_aim_correction("1,2,3")


class CorrectionGeometryTest(unittest.TestCase):
    def test_maps_optical_forward_onto_minus_y(self):
        # +90 deg about +X takes the optical +Z (the pre-correction render
        # axis) to -Y_optical -- which the URDF FK shows is the TCP forward.
        rotated = _rotate(TOOL_FORWARD_CORRECTION_WXYZ, (0.0, 0.0, 1.0))
        for got, want in zip(rotated, (0.0, -1.0, 0.0)):
            self.assertAlmostEqual(got, want, places=12)

    def test_preserves_image_right(self):
        # A rotation about +X leaves image-right (+X) alone: no roll change.
        rotated = _rotate(TOOL_FORWARD_CORRECTION_WXYZ, (1.0, 0.0, 0.0))
        for got, want in zip(rotated, (1.0, 0.0, 0.0)):
            self.assertAlmostEqual(got, want, places=12)

    def test_is_unit_length(self):
        self.assertAlmostEqual(
            math.fsum(v * v for v in TOOL_FORWARD_CORRECTION_WXYZ), 1.0, places=12
        )


class ApplyTest(unittest.TestCase):
    def test_only_the_wrist_spec_changes_and_gets_no_dolly(self):
        specs = load_camera_specs(CONTRACT)
        corrected = apply_head_aim_correction(
            specs, TOOL_FORWARD_CORRECTION_WXYZ, camera_name=WRIST_CAMERA_NAME
        )
        by_name = {spec.name: spec for spec in corrected}
        original = {spec.name: spec for spec in specs}
        for name, spec in by_name.items():
            if name == WRIST_CAMERA_NAME:
                self.assertNotEqual(
                    spec.mount_rotation_wxyz, original[name].mount_rotation_wxyz
                )
                # The head preset's housing-clearance dolly is specific to
                # the head geometry; the wrist correction must not inherit it.
                self.assertEqual(spec.view_axis_forward_offset_m, 0.0)
            else:
                self.assertEqual(
                    spec.mount_rotation_wxyz, original[name].mount_rotation_wxyz
                )


if __name__ == "__main__":
    unittest.main()
