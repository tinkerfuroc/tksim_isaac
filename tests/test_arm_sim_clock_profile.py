"""The sim-clock arm profile must disable path-tolerance aborts.

Isaac's physics is hard-pinned to CPU by design, and on this host (Xeon
E5-2620 v4 @ 2.10 GHz) the measured real-time factor is ~0.07: 8.5 physics
steps per wall second against a 120 Hz target, with the GPU idle at 4-5%.
`physics_hz` is a backend constructor default with no CLI knob, so the tick
rate cannot simply be raised.

A MoveIt trajectory is time-parameterised for real-speed execution. Replayed
against a clock advancing at 7% of real time, the simulated arm falls
arbitrarily far behind its desired path -- measured 0.0015 rad of joint1
travel across 0.8 s of sim time -- and
`joint_trajectory_controller` aborts mid-flight with
PATH_TOLERANCE_VIOLATED. The shipped profile already carries 1.0-2.0 rad
path tolerances (and goal_time 60.0) from earlier rounds of this same
tracking-lag family, and the arm still blows through them.

This profile therefore stops policing the *path* at all and keeps only the
endpoint contract: reach the goal, within a generous goal window, however
long the simulated clock takes to get there. In
joint_trajectory_controller a per-joint `trajectory` tolerance of 0.0 means
"do not enforce", which is the documented way to let a slow plant converge
without a mid-trajectory abort.
"""

from __future__ import annotations

import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "ros2_ws/src/tinker_sim_bridge/config"
SHIPPED = CONFIG / "controllers.yaml"
SIM_CLOCK = CONFIG / "controllers.sim-clock.yaml"
LAUNCH = ROOT / "ros2_ws/src/tinker_sim_bridge/launch/gpsr.launch.py"
SETUP_PY = ROOT / "ros2_ws/src/tinker_sim_bridge/setup.py"

ARM = "xarm7_traj_controller"
JOINTS = [f"joint{i}" for i in range(1, 8)]


def _load(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


class ArmSimClockProfileTest(unittest.TestCase):
    def setUp(self):
        self.assertTrue(SIM_CLOCK.is_file(), f"{SIM_CLOCK} does not exist")
        self.profile = _load(SIM_CLOCK)
        self.constraints = self.profile[ARM]["ros__parameters"]["constraints"]

    def test_path_tolerance_is_disabled_for_every_arm_joint(self):
        for joint in JOINTS:
            self.assertIn(joint, self.constraints, f"{joint} missing from constraints")
            self.assertEqual(
                self.constraints[joint]["trajectory"], 0.0,
                f"{joint} must have trajectory tolerance 0.0 (disabled); a "
                f"nonzero value re-enables the mid-flight abort this profile "
                f"exists to prevent",
            )

    def test_goal_contract_is_preserved(self):
        """Relaxing the path must not silently accept a wrong final pose."""
        for joint in JOINTS:
            goal = self.constraints[joint]["goal"]
            self.assertGreater(goal, 0.0, f"{joint} goal tolerance must stay enforced")
            self.assertLessEqual(
                goal, 0.5,
                f"{joint} goal tolerance must be no looser than the shipped profile",
            )
        self.assertGreaterEqual(self.constraints["goal_time"], 60.0)

    def test_uses_the_simulated_clock(self):
        self.assertTrue(self.profile["controller_manager"]["ros__parameters"]["use_sim_time"])
        self.assertTrue(self.profile[ARM]["ros__parameters"]["use_sim_time"])

    def test_matches_the_shipped_profile_apart_from_path_tolerances(self):
        """Only the per-joint `trajectory` values may differ."""
        shipped = _load(SHIPPED)
        self.assertEqual(
            self.profile[ARM]["ros__parameters"]["joints"],
            shipped[ARM]["ros__parameters"]["joints"],
        )
        self.assertEqual(
            self.profile[ARM]["ros__parameters"]["command_interfaces"],
            shipped[ARM]["ros__parameters"]["command_interfaces"],
        )
        self.assertEqual(
            self.profile["controller_manager"]["ros__parameters"]["update_rate"],
            shipped["controller_manager"]["ros__parameters"]["update_rate"],
        )

    def test_profile_is_installed(self):
        source = SETUP_PY.read_text(encoding="utf-8")
        self.assertIn(
            "config/controllers.sim-clock.yaml", source,
            "the profile must be listed in setup.py data_files or it will "
            "never reach share/tinker_sim_bridge/config",
        )

    def test_launch_exposes_the_profile_as_an_opt_in_argument(self):
        source = LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            "controllers_file", source,
            "gpsr.launch.py must expose a controllers_file argument so the "
            "sim-clock profile can be selected without editing the shipped one",
        )


if __name__ == "__main__":
    unittest.main()
