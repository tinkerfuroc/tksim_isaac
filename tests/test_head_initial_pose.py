"""The simulated head must start at the hardware's startup pose.

The hardware pan-tilt controller drives its own startup pose from
``initial_pan_deg`` / ``initial_tilt_deg`` (tk26_vision
``pan_tilt/config/pan_tilt.yaml``), both **0.0**. At tilt 0 the head camera
looks approximately level -- the -45.5 deg pitch in ``camera_mount_joint`` is
accounted for downstream of the joint, not something the tilt joint cancels.

In simulation ``pan_tilt_facade`` stands in for that controller and had no
startup pose at all, so nothing held the head. These tests pin the default to
hardware's 0/0 and keep the joint-limit guard.

An earlier version of this module defaulted the tilt to +45.5 deg on the theory
that it had to cancel the mount pitch. That aimed the camera at the sky: the
head camera returned a blank frame with 0.000 valid depth at every tilt from
0.00 up, while the lidar reported arena walls at 2.4-3.6 m. Hence the explicit
test below that the default is level.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.head_pose import (  # noqa: E402
    DEFAULT_PAN_DEG,
    DEFAULT_TILT_DEG,
    TILT_JOINT_LOWER_RAD,
    TILT_JOINT_UPPER_RAD,
    resolve_initial_head_pose,
)


class HeadInitialPoseTest(unittest.TestCase):
    def test_default_matches_the_hardware_startup_pose(self):
        """tk26_vision pan_tilt.yaml: initial_pan_deg 0.0, initial_tilt_deg 0.0."""
        self.assertEqual(DEFAULT_PAN_DEG, 0.0)
        self.assertEqual(DEFAULT_TILT_DEG, 0.0)

    def test_default_pose_is_level(self):
        """A non-zero default tilt aimed the camera at the sky; pin it to 0."""
        pan, tilt = resolve_initial_head_pose(None, None)
        self.assertEqual(pan, 0.0)
        self.assertEqual(tilt, 0.0)

    def test_explicit_degrees_are_honoured(self):
        """follow_head's home (30 deg) must remain expressible."""
        pan, tilt = resolve_initial_head_pose(0.0, 30.0)
        self.assertAlmostEqual(math.degrees(tilt), 30.0, places=9)
        self.assertAlmostEqual(pan, 0.0, places=12)

    def test_negative_tilt_within_range_is_honoured(self):
        _, tilt = resolve_initial_head_pose(None, -20.0)
        self.assertAlmostEqual(math.degrees(tilt), -20.0, places=9)

    def test_tilt_outside_the_joint_limit_is_refused(self):
        """Silently clamping would leave the camera somewhere else."""
        for bad_deg in (95.0, -45.0):
            with self.assertRaises(ValueError, msg=f"accepted {bad_deg}"):
                resolve_initial_head_pose(0.0, bad_deg)

    def test_joint_limits_match_the_urdf(self):
        self.assertAlmostEqual(math.degrees(TILT_JOINT_LOWER_RAD), -30.0, places=6)
        self.assertAlmostEqual(math.degrees(TILT_JOINT_UPPER_RAD), 90.0, places=6)

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
