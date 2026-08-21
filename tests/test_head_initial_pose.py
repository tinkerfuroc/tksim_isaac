"""The simulated head must start at a pose that cancels its mount pitch.

`camera_mount_joint` fixes the head camera to `tilt_link` with
rpy="0.0406528 -0.79457 3.0833" -- a **-45.53 deg pitch** that exists on the
real robot too. The pan-tilt mechanism is what compensates for it: `tilt_joint`
turns about +Y over [-30, +90] deg, so a tilt of +45.53 deg brings the optical
axis back to level.

On hardware the pan-tilt controller drives that pose itself -- it takes
`initial_pan_deg` / `initial_tilt_deg` (tk26_vision pan_tilt/config/pan_tilt.yaml)
and `follow_head` homes to `home_tilt_deg: 30.0`. In simulation
`pan_tilt_facade` stands in for that controller but had no startup pose at all,
so the head sat at ~0 and the camera stared 45 deg down into the robot's own
deck.

Measured 2026-08-20 (domain 71, gpsr-rcw2026), with the head uncommanded:
`door_detection_srv` returned `is_open=0` forever at a centre depth of ~1.03 m
while the lidar map showed the 1.5 m ahead of the spawn as free; a ~180 deg base
rotation changed only 15% of the head-camera pixels, because the geometry in
frame was the robot. Publishing a tilt command took the valid depth fraction
from 0.39 to 0.88.

Parameter names mirror the hardware controller's on purpose: the simulated
facade is the stand-in for that node, so it should be configured the same way.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.head_pose import (  # noqa: E402
    HEAD_CAMERA_MOUNT_PITCH_RAD,
    TILT_JOINT_LOWER_RAD,
    TILT_JOINT_UPPER_RAD,
    level_tilt_rad,
    resolve_initial_head_pose,
)

URDF = ROOT / "integration/model-bundle-r2/simulator_full_urdf/source-tinker-full.urdf"


class HeadInitialPoseTest(unittest.TestCase):
    def test_mount_pitch_matches_the_urdf(self):
        """The constant must not drift from the model it compensates."""
        text = URDF.read_text(encoding="utf-8")
        start = text.index('<joint name="camera_mount_joint"')
        block = text[start : start + 400]
        rpy = block.split('rpy="')[1].split('"')[0].split()
        self.assertAlmostEqual(
            float(rpy[1]), HEAD_CAMERA_MOUNT_PITCH_RAD, places=6,
            msg="camera_mount_joint pitch changed; update the constant",
        )

    def test_level_tilt_cancels_the_mount_pitch(self):
        self.assertAlmostEqual(
            level_tilt_rad() + HEAD_CAMERA_MOUNT_PITCH_RAD, 0.0, places=12
        )

    def test_level_tilt_is_about_forty_five_degrees(self):
        self.assertAlmostEqual(math.degrees(level_tilt_rad()), 45.526, places=2)

    def test_level_tilt_is_reachable(self):
        self.assertGreater(level_tilt_rad(), TILT_JOINT_LOWER_RAD)
        self.assertLess(level_tilt_rad(), TILT_JOINT_UPPER_RAD)

    def test_default_pose_is_level_and_forward(self):
        pan, tilt = resolve_initial_head_pose(None, None)
        self.assertAlmostEqual(pan, 0.0, places=12)
        self.assertAlmostEqual(tilt, level_tilt_rad(), places=12)

    def test_explicit_degrees_are_honoured(self):
        """Hardware's follow_head home (30 deg) must be expressible."""
        pan, tilt = resolve_initial_head_pose(0.0, 30.0)
        self.assertAlmostEqual(math.degrees(tilt), 30.0, places=9)
        self.assertAlmostEqual(pan, 0.0, places=12)

    def test_tilt_outside_the_joint_limit_is_refused(self):
        """Silently clamping would leave the camera pointing somewhere else."""
        for bad_deg in (95.0, -45.0):
            with self.assertRaises(ValueError, msg=f"accepted {bad_deg}"):
                resolve_initial_head_pose(0.0, bad_deg)

    def test_non_finite_is_refused(self):
        for bad in (float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                resolve_initial_head_pose(0.0, bad)
            with self.assertRaises(ValueError):
                resolve_initial_head_pose(bad, 0.0)

    def test_facade_publishes_an_initial_pose(self):
        source = (
            ROOT / "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/pan_tilt_facade.py"
        ).read_text(encoding="utf-8")
        self.assertIn("resolve_initial_head_pose", source)
        self.assertIn("initial_tilt_deg", source)
        self.assertIn("initial_pan_deg", source)


if __name__ == "__main__":
    unittest.main()
