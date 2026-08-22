"""Opt-in sim-only correction for the head camera's aim.

**This is a workaround, not a fix, and it breaks hardware parity by design.**
It is off unless ``TINKER_SIM_HEAD_CAMERA_AIM`` is set.

The robot description aims the head camera above the horizon everywhere it
can reach. Measured from ``tinker_full.full.urdf`` (and identically from
``tinker_real.urdf``), the optical axis sits at::

    pan 0,   tilt 0            +47.5 deg elevation
    pan 0,   tilt -30 (limit)  +17.5 deg
    pan 180, tilt -30 (best)   +13.6 deg

``tilt_joint`` spans -30..+90 deg, so nothing reaches level. With a 58.7 deg
vertical FOV the frame at tilt~0 covers +18..+77 deg -- wall and sky, over a
standing person's head. GPSR run13 reached the kitchen table and scanned
3930 times at a person it could not see; forced to pan=pi, tilt=-0.5236 the
unmodified detector returned ``cls='person' conf=0.93`` immediately.

The simulator reproduces the model faithfully -- rendered aim, measured from
the horizon, matched the URDF's forward kinematics to 2.6 deg -- so the real
fix belongs in the robot description. Until that lands, this correction lets
GPSR be exercised end to end.
"""
from __future__ import annotations

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.head_camera_aim import (  # noqa: E402
    HEAD_AIM_ENV,
    LEVEL_FORWARD_CORRECTION_WXYZ,
    apply_head_aim_correction,
    resolve_head_aim_correction,
)
from tinker_sim_isaac.camera_rig import load_camera_specs  # noqa: E402

CONTRACT = ROOT / "simulation/sensors/hardware-parity.json"


class ResolveTest(unittest.TestCase):
    def test_unset_is_off(self):
        """Parity is the default; the override never applies by accident."""
        self.assertIsNone(resolve_head_aim_correction(None))
        self.assertIsNone(resolve_head_aim_correction(""))
        self.assertIsNone(resolve_head_aim_correction("   "))

    def test_preset_resolves_to_the_measured_correction(self):
        self.assertEqual(
            resolve_head_aim_correction("level-forward"), LEVEL_FORWARD_CORRECTION_WXYZ
        )

    def test_preset_is_case_and_space_insensitive(self):
        self.assertEqual(
            resolve_head_aim_correction("  Level-Forward "), LEVEL_FORWARD_CORRECTION_WXYZ
        )

    def test_explicit_quaternion_is_accepted_and_normalised(self):
        got = resolve_head_aim_correction("0, 0, 0, 2")
        self.assertEqual(len(got), 4)
        self.assertAlmostEqual(math.fsum(v * v for v in got), 1.0, places=12)
        self.assertAlmostEqual(abs(got[3]), 1.0, places=12)

    def test_a_degenerate_quaternion_is_refused(self):
        with self.assertRaises(ValueError):
            resolve_head_aim_correction("0,0,0,0")

    def test_nonsense_is_refused_rather_than_ignored(self):
        """Silently ignoring a typo would look like the override working."""
        for value in ("levelforward", "1,2,3", "a,b,c,d", "1,2,3,4,5"):
            with self.assertRaises(ValueError, msg=value):
                resolve_head_aim_correction(value)

    def test_the_preset_is_a_unit_quaternion(self):
        self.assertAlmostEqual(
            math.fsum(v * v for v in LEVEL_FORWARD_CORRECTION_WXYZ), 1.0, places=12
        )

    def test_env_name_is_stable(self):
        self.assertEqual(HEAD_AIM_ENV, "TINKER_SIM_HEAD_CAMERA_AIM")


class ApplyTest(unittest.TestCase):
    def setUp(self):
        self.specs = load_camera_specs(CONTRACT)
        self.head = next(s for s in self.specs if s.name == "head_camera")

    def test_only_the_head_camera_is_touched(self):
        out = apply_head_aim_correction(self.specs, LEVEL_FORWARD_CORRECTION_WXYZ)
        self.assertEqual(len(out), len(self.specs))
        for before, after in zip(self.specs, out):
            if after.name == "head_camera":
                self.assertNotEqual(after.mount_rotation_wxyz, before.mount_rotation_wxyz)
            else:
                self.assertEqual(after, before)

    def test_identity_correction_changes_nothing(self):
        out = apply_head_aim_correction(self.specs, (1.0, 0.0, 0.0, 0.0))
        head = next(s for s in out if s.name == "head_camera")
        for got, want in zip(head.mount_rotation_wxyz, self.head.mount_rotation_wxyz):
            self.assertAlmostEqual(got, want, places=12)

    def test_result_stays_a_unit_quaternion(self):
        out = apply_head_aim_correction(self.specs, LEVEL_FORWARD_CORRECTION_WXYZ)
        head = next(s for s in out if s.name == "head_camera")
        self.assertAlmostEqual(
            math.fsum(v * v for v in head.mount_rotation_wxyz), 1.0, places=12
        )

    def test_correction_composes_on_the_left_of_the_mount_rotation(self):
        """camera_world = R_mount_prim . C . R_spec, so the spec becomes C.R_spec.

        Applied on the wrong side the camera would be corrected in its own
        optical frame instead of the mount's, and the aim would be wrong for
        every pan/tilt except the one it was solved at.
        """
        out = apply_head_aim_correction(self.specs, LEVEL_FORWARD_CORRECTION_WXYZ)
        head = next(s for s in out if s.name == "head_camera")
        # (0,1,0,0) is 180 deg about X; C . that, computed independently here.
        cw, cx, cy, cz = LEVEL_FORWARD_CORRECTION_WXYZ
        sw, sx, sy, sz = self.head.mount_rotation_wxyz
        want = (
            cw * sw - cx * sx - cy * sy - cz * sz,
            cw * sx + cx * sw + cy * sz - cz * sy,
            cw * sy - cx * sz + cy * sw + cz * sx,
            cw * sz + cx * sy - cy * sx + cz * sw,
        )
        for got, expected in zip(head.mount_rotation_wxyz, want):
            # apply_head_aim_correction re-normalises the product.
            self.assertAlmostEqual(got, expected, places=10)

    def test_missing_head_camera_is_reported(self):
        others = tuple(s for s in self.specs if s.name != "head_camera")
        with self.assertRaises(ValueError):
            apply_head_aim_correction(others, LEVEL_FORWARD_CORRECTION_WXYZ)

    def test_hardware_parity_contract_is_left_alone(self):
        """The correction lives in code, never in the parity spec file."""
        text = CONTRACT.read_text(encoding="utf-8")
        self.assertIn('"mount_rotation_wxyz"', text)
        self.assertNotIn("level-forward", text)
        self.assertNotIn(HEAD_AIM_ENV, text)


class AimGeometryTest(unittest.TestCase):
    """The preset must actually put the optical axis level and forward.

    Reproduces the URDF chain's rotation at pan=0, tilt=0 as a constant
    (measured once, checked here) rather than re-parsing tk25_ws, which may
    not exist in every checkout.
    """

    # head_camera_color_optical_frame orientation in base_link at pan=0,tilt=0.
    FRAME_R = (
        (-0.023544373, -0.736897005, -0.675594898),
        (+0.999679197, -0.023664844, -0.009026514),
        (-0.009336236, -0.675590689, +0.737217780),
    )

    @staticmethod
    def _quat_to_matrix(q):
        w, x, y, z = q
        return (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        )

    @staticmethod
    def _matmul(a, b):
        return tuple(
            tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
            for i in range(3)
        )

    def _view_direction(self, mount_rotation):
        cam = self._matmul(self.FRAME_R, self._quat_to_matrix(mount_rotation))
        # A USD camera looks along its own -Z.
        return tuple(-cam[i][2] for i in range(3))

    def test_uncorrected_aim_is_the_defect_we_measured(self):
        specs = load_camera_specs(CONTRACT)
        head = next(s for s in specs if s.name == "head_camera")
        view = self._view_direction(head.mount_rotation_wxyz)
        elevation = math.degrees(math.asin(max(-1.0, min(1.0, view[2]))))
        self.assertGreater(elevation, 40.0, "expected the known upward aim")

    def test_corrected_aim_is_level_and_forward(self):
        specs = apply_head_aim_correction(
            load_camera_specs(CONTRACT), LEVEL_FORWARD_CORRECTION_WXYZ
        )
        head = next(s for s in specs if s.name == "head_camera")
        view = self._view_direction(head.mount_rotation_wxyz)
        elevation = math.degrees(math.asin(max(-1.0, min(1.0, view[2]))))
        azimuth = math.degrees(math.atan2(view[1], view[0]))
        self.assertAlmostEqual(elevation, 0.0, delta=0.5)
        self.assertAlmostEqual(azimuth, 0.0, delta=0.5)


if __name__ == "__main__":
    unittest.main()
