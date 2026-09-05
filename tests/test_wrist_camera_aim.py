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
    CAM_STAND_CORRECTION_WXYZ,
    CAM_STAND_MOUNT_OFFSET_XYZ,
    LEVEL_FORWARD_CORRECTION_WXYZ,
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
    def test_tilts_render_axis_60_deg_toward_the_tool(self):
        # The tool axis sits at -Y_optical (URDF FK, 90 deg from the render
        # axis). The preset stops 30 deg short of it: a perfectly
        # tool-aligned view stares into the co-axial gripper (measured:
        # median depth 76 mm, all hand), so the sweep-picked 60 deg keeps
        # the gripper at the frame's bottom edge with the scene in view.
        rotated = _rotate(TOOL_FORWARD_CORRECTION_WXYZ, (0.0, 0.0, 1.0))
        angle_from_original = math.degrees(math.acos(rotated[2]))
        self.assertAlmostEqual(angle_from_original, 60.0, places=9)
        tool_axis = (0.0, -1.0, 0.0)
        cos_to_tool = sum(a * b for a, b in zip(rotated, tool_axis))
        self.assertAlmostEqual(math.degrees(math.acos(cos_to_tool)), 30.0, places=9)

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


# --- cam-stand preset -------------------------------------------------------
#
# The band across the top of every wrist frame (grasp bench, 2026-09-04: 9.4%
# of the image, depth 50-60 mm, deprojects onto xarm_gripper_base_link) is
# the gripper housing's far wall just past the 0.05 m near clip. The
# tool-forward preset rotates the camera IN PLACE, and the place is wrong:
# the sim description attaches Intel's sensor_d435 macro to link_eef with a
# placeholder identity origin, so the optical origin sits on the housing
# surface. The real robot mounts the D435 on xArm's cam-stand bracket
# (tinker_real.urdf / xarm realsense_d435i.urdf.xacro, factory-nominal
# extrinsics). ``cam-stand`` renders from exactly that pose.

_ART_CAMERA_LINK_XYZ = (0.0106, 0.0175, 0.0125)  # Intel macro, bottom screw -> camera_link
_VENDOR_CAMERA_LINK_XYZ = (0.06746, -0.0175, 0.0237)  # xArm cam-stand, link_eef -> camera_link
_VENDOR_CAMERA_LINK_RPY = (math.pi, -math.pi / 2, 0.0)
_COLOR_XYZ = (0.0, 0.015, 0.0)  # camera_link -> color frame (both descriptions)
_OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)  # REP-103 optical convention


def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = ((1, 0, 0), (0, cr, -sr), (0, sr, cr))
    ry = ((cp, 0, sp), (0, 1, 0), (-sp, 0, cp))
    rz = ((cy, -sy, 0), (sy, cy, 0), (0, 0, 1))
    return _mm(rz, _mm(ry, rx))


def _mm(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _mv(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def _transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def _pose(xyz, rpy):
    return (_rpy_matrix(*rpy), tuple(xyz))


def _compose(a, b):
    """(R, t) of a . b for column-vector poses."""
    ra, ta = a
    rb, tb = b
    return _mm(ra, rb), tuple(x + y for x, y in zip(ta, _mv(ra, tb)))


def _invert(a):
    r, t = a
    rt = _transpose(r)
    return rt, tuple(-x for x in _mv(rt, t))


def _quat_matrix(q):
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _artifact_optical_in_eef():
    chain = _pose((0, 0, 0), (0, 0, 0))
    for xyz, rpy in (
        (_ART_CAMERA_LINK_XYZ, (0, 0, 0)),
        (_COLOR_XYZ, (0, 0, 0)),
        ((0, 0, 0), _OPTICAL_RPY),
    ):
        chain = _compose(chain, _pose(xyz, rpy))
    return chain


def _vendor_optical_in_eef():
    chain = _pose(_VENDOR_CAMERA_LINK_XYZ, _VENDOR_CAMERA_LINK_RPY)
    for xyz, rpy in ((_COLOR_XYZ, (0, 0, 0)), ((0, 0, 0), _OPTICAL_RPY)):
        chain = _compose(chain, _pose(xyz, rpy))
    return chain


class CamStandResolveTest(unittest.TestCase):
    def test_preset_resolves_to_the_vendor_correction(self):
        self.assertEqual(
            resolve_wrist_aim_correction("cam-stand"), CAM_STAND_CORRECTION_WXYZ
        )
        self.assertEqual(
            resolve_wrist_aim_correction(" Cam-Stand "), CAM_STAND_CORRECTION_WXYZ
        )

    def test_tool_forward_survives_as_the_a_b_baseline(self):
        self.assertEqual(
            resolve_wrist_aim_correction("tool-forward"), TOOL_FORWARD_CORRECTION_WXYZ
        )

    def test_the_two_presets_differ(self):
        self.assertNotEqual(CAM_STAND_CORRECTION_WXYZ, TOOL_FORWARD_CORRECTION_WXYZ)

    def test_is_unit_length(self):
        self.assertAlmostEqual(
            math.fsum(v * v for v in CAM_STAND_CORRECTION_WXYZ), 1.0, places=12
        )


class CamStandGeometryTest(unittest.TestCase):
    """The constants must be exactly inv(T_artifact_optical) . T_vendor_optical.

    Both chains are re-derived here from their URDF <origin> values (the
    artifact's placeholder Intel-macro chain and xArm's cam-stand chain) so
    the preset cannot drift from the geometry it claims to reproduce.
    """

    def setUp(self):
        self.correction = _compose(
            _invert(_artifact_optical_in_eef()), _vendor_optical_in_eef()
        )

    def test_translation_is_the_bracket_offset_in_the_optical_frame(self):
        _, t = self.correction
        for got, want in zip(CAM_STAND_MOUNT_OFFSET_XYZ, t):
            self.assertAlmostEqual(got, want, places=6)

    def test_rotation_matches_the_vendor_chain(self):
        r, _ = self.correction
        q = _quat_matrix(CAM_STAND_CORRECTION_WXYZ)
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(q[i][j], r[i][j], places=9)

    def test_corrected_origin_lands_on_the_vendor_bracket(self):
        r_art, t_art = _artifact_optical_in_eef()
        origin_in_eef = tuple(
            a + b for a, b in zip(t_art, _mv(r_art, CAM_STAND_MOUNT_OFFSET_XYZ))
        )
        # xArm cam-stand: camera_link (0.06746, -0.0175, 0.0237), color frame
        # a further 0.015 along camera_link's Y, which the bracket's
        # rpy (pi, -pi/2, 0) maps onto -Y_eef.
        for got, want in zip(origin_in_eef, (0.06746, -0.0325, 0.0237)):
            self.assertAlmostEqual(got, want, places=6)

    def test_view_axis_is_the_tool_axis_and_image_up_is_outward(self):
        r_art, _ = _artifact_optical_in_eef()
        q = _quat_matrix(CAM_STAND_CORRECTION_WXYZ)
        view_in_eef = _mv(r_art, _mv(q, (0.0, 0.0, 1.0)))
        for got, want in zip(view_in_eef, (0.0, 0.0, 1.0)):
            self.assertAlmostEqual(got, want, places=9)
        # Image-up (-Y optical) points radially outward, +X_eef: the gripper
        # hangs off the bottom edge of the frame, as on the real camera.
        up_in_eef = _mv(r_art, _mv(q, (0.0, -1.0, 0.0)))
        for got, want in zip(up_in_eef, (1.0, 0.0, 0.0)):
            self.assertAlmostEqual(got, want, places=9)


class CamStandApplyTest(unittest.TestCase):
    def setUp(self):
        self.specs = load_camera_specs(CONTRACT)

    def _wrist(self, corrected):
        return next(spec for spec in corrected if spec.name == WRIST_CAMERA_NAME)

    def test_cam_stand_moves_the_wrist_origin_and_only_the_wrist(self):
        corrected = apply_head_aim_correction(
            self.specs, CAM_STAND_CORRECTION_WXYZ, camera_name=WRIST_CAMERA_NAME
        )
        wrist = self._wrist(corrected)
        self.assertEqual(wrist.mount_frame_offset_xyz, CAM_STAND_MOUNT_OFFSET_XYZ)
        self.assertEqual(wrist.view_axis_forward_offset_m, 0.0)
        for spec in corrected:
            if spec.name != WRIST_CAMERA_NAME:
                self.assertEqual(spec.mount_frame_offset_xyz, (0.0, 0.0, 0.0))

    def test_cam_stand_rotation_composes_on_the_left_of_the_mount_rotation(self):
        corrected = apply_head_aim_correction(
            self.specs, CAM_STAND_CORRECTION_WXYZ, camera_name=WRIST_CAMERA_NAME
        )
        wrist = self._wrist(corrected)
        base = next(s for s in self.specs if s.name == WRIST_CAMERA_NAME)
        got = _quat_matrix(wrist.mount_rotation_wxyz)
        want = _mm(
            _quat_matrix(CAM_STAND_CORRECTION_WXYZ), _quat_matrix(base.mount_rotation_wxyz)
        )
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(got[i][j], want[i][j], places=9)

    def test_tool_forward_keeps_the_origin_in_place(self):
        corrected = apply_head_aim_correction(
            self.specs, TOOL_FORWARD_CORRECTION_WXYZ, camera_name=WRIST_CAMERA_NAME
        )
        self.assertEqual(self._wrist(corrected).mount_frame_offset_xyz, (0.0, 0.0, 0.0))

    def test_the_head_preset_gets_no_mount_offset(self):
        corrected = apply_head_aim_correction(self.specs, LEVEL_FORWARD_CORRECTION_WXYZ)
        head = next(spec for spec in corrected if spec.name == "head_camera")
        self.assertEqual(head.mount_frame_offset_xyz, (0.0, 0.0, 0.0))
        self.assertNotEqual(head.view_axis_forward_offset_m, 0.0)


if __name__ == "__main__":
    unittest.main()
