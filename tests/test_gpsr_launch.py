from __future__ import annotations

import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "ros2_ws/src/tinker_sim_bridge/launch/gpsr.launch.py"
SETUP_PY = ROOT / "ros2_ws/src/tinker_sim_bridge/setup.py"


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

    def test_map_defaults_to_the_scenario_arena_map(self):
        """A blank map_yaml must resolve the scenario's arena map, not the
        robot artifact's hardware map (they share no occupied cell)."""
        source = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("scenario_arena_id(", source)
        self.assertIn("resolve_arena_map_yaml(", source)

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


class GpsrLaunchInstallTest(unittest.TestCase):
    """Regression guard: gpsr.launch.py must be registered for installation.

    Task 3 added the composite launch source but omitted it from setup.py's
    data_files, so a colcon build would never install it and
    `ros2 launch tinker_sim_bridge gpsr.launch.py` would fail with
    "file not found" even though the source file existed and parsed fine.
    """

    def test_gpsr_launch_is_registered_in_setup_py_data_files(self):
        self.assertTrue(SETUP_PY.is_file(), f"{SETUP_PY} does not exist")
        source = SETUP_PY.read_text(encoding="utf-8")
        self.assertIn(
            "launch/gpsr.launch.py", source,
            "gpsr.launch.py must be listed in setup.py's data_files so it "
            "gets installed under share/tinker_sim_bridge/launch",
        )


class GpsrLaunchWorldFrameTest(unittest.TestCase):
    """The composite must make `world` reachable for the manipulation stack.

    `pick_and_place.cpp::transform_chain_ready()` rejects every joint-move goal
    unless BOTH `world -> base_link` and `base_link -> link_tcp` resolve. The
    robot URDF supplies a fixed `world -> base_link` joint, but this composite
    also runs navigation, which publishes a dynamic `odom -> base_link`. A frame
    cannot have two parents, so tf2 keeps `odom -> base_link` and drops the
    `world` edge entirely — `world` becomes unreachable and manipulation is dead
    on arrival with "tf chain lookup unavailable".

    Attaching `world` above the tree root (`map` has no parent, while `odom` is
    already owned by AMCL) restores the chain without creating a second parent
    for any frame. Measured live on 2026-08-20: with this edge published,
    can_transform(base_link <- world) flips 0 -> 1 and pick_and_place accepts
    goals immediately.
    """

    def setUp(self):
        self.assertTrue(LAUNCH.is_file(), f"{LAUNCH} does not exist")
        self.source = LAUNCH.read_text(encoding="utf-8")
        self.tree = ast.parse(self.source)

    def test_publishes_world_frame_for_manipulation_tf_chain(self):
        self.assertIn(
            "world", self.source,
            "the composite must publish a `world` edge; without it "
            "pick_and_place rejects every joint-move goal",
        )
        static_tf_names = [
            kw.value.value
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "Node"
            for kw in node.keywords
            if kw.arg == "name" and isinstance(kw.value, ast.Constant)
        ]
        self.assertIn(
            "world_static_tf", static_tf_names,
            "expected a static_transform_publisher named world_static_tf",
        )

    def test_world_is_parented_above_the_map_root_not_base_link(self):
        """`world -> map`, never `world -> base_link` (that double-parents)."""
        self.assertIn('"--child-frame-id", "map"', self.source)
        self.assertNotIn(
            '"--frame-id", "world", "--child-frame-id", "base_link"', self.source,
            "world must not parent base_link — navigation already owns "
            "odom -> base_link and tf2 would drop one of the two edges",
        )


if __name__ == "__main__":
    unittest.main()
