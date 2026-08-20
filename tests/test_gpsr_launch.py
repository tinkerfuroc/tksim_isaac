from __future__ import annotations

import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "ros2_ws/src/tinker_sim_bridge/launch/gpsr.launch.py"


def _executables(tree: ast.AST) -> list[str]:
    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "Node":
            for kw in node.keywords:
                if kw.arg == "executable" and isinstance(kw.value, ast.Constant):
                    found.append(kw.value.value)
    return found


class GpsrLaunchTest(unittest.TestCase):
    def setUp(self):
        self.assertTrue(LAUNCH.is_file(), f"{LAUNCH} does not exist")
        self.tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
        self.executables = _executables(self.tree)

    def test_singleton_nodes_are_not_duplicated(self):
        for name in ("command_gateway", "safety_supervisor", "contract_guard",
                     "robot_state_publisher"):
            self.assertEqual(
                self.executables.count(name), 1,
                f"{name} must appear exactly once; found {self.executables.count(name)}",
            )

    def test_manipulation_side_is_present(self):
        for name in ("ros2_control_node", "xarm_facade", "gripper_facade",
                     "pan_tilt_facade", "audio_fixtures"):
            self.assertIn(name, self.executables)

    def test_navigation_side_is_present(self):
        for name in ("base_facade", "initial_pose", "pointcloud_to_laserscan_node"):
            self.assertIn(name, self.executables)

    def test_supervisor_manages_controllers_for_the_arm(self):
        source = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("required_sources", source)
        self.assertNotIn('"manage_controllers": False', source,
                         "the composite owns the arm, so controllers must be managed")


if __name__ == "__main__":
    unittest.main()
