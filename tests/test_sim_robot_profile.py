"""The sim launches must select the tinker2_sim robot profile.

tk26_navigation's navigation_dwb_launch.py rewrites every costmap ``footprint``
key with ``RobotField('base.footprint')`` AFTER nav_params_overlay has written
its sim footprint, so the overlay alone cannot set it: under ROBOT_NAME=tinker2
both costmaps (and approach_planner, which follows the published footprint)
ran with the real robot's 0.95 m envelope.
"""

from __future__ import annotations

import ast
import os
import sys
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.nav_params_overlay import prior_map_costmap_overlay  # noqa: E402

LAUNCH_DIR = ROOT / "ros2_ws/src/tinker_sim_bridge/launch"
SIM_PROFILE = Path(os.environ.get("TINKER_WS", "/home/tinker/tk25_ws")) / (
    "src/tk25_basic/src/tinker_robot_config/robots/tinker2_sim/robot.yaml"
)


def _robot_name_settings(path: Path) -> list[object]:
    found = []
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if (
            isinstance(node, ast.Call)
            and getattr(node.func, "id", "") == "SetEnvironmentVariable"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == "ROBOT_NAME"
        ):
            found.append(node.args[1].value)
    return found


class SimRobotProfileTest(unittest.TestCase):
    def test_sim_launches_select_the_sim_profile(self):
        for name in ("gpsr", "navigation"):
            with self.subTest(launch=name):
                self.assertEqual(
                    _robot_name_settings(LAUNCH_DIR / f"{name}.launch.py"),
                    ["tinker2_sim"],
                )

    @unittest.skipUnless(SIM_PROFILE.is_file(), f"{SIM_PROFILE} not present")
    def test_profile_footprint_matches_the_overlay(self):
        costmap = {"ros__parameters": {"footprint": "[]"}}
        overlaid = prior_map_costmap_overlay(
            {
                "local_costmap": {"local_costmap": costmap},
                "global_costmap": {"global_costmap": {"ros__parameters": {"footprint": "[]"}}},
            }
        )
        profile = yaml.safe_load(SIM_PROFILE.read_text(encoding="utf-8"))
        expected = yaml.safe_load(profile["base"]["footprint"])
        for name in ("local_costmap", "global_costmap"):
            with self.subTest(costmap=name):
                self.assertEqual(
                    yaml.safe_load(overlaid[name][name]["ros__parameters"]["footprint"]),
                    expected,
                )


if __name__ == "__main__":
    unittest.main()
