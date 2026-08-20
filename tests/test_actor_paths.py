from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.actor_path import path_length, path_pose_at
from tinker_sim_core.scenario import ScenarioDefinition


class PathPoseTest(unittest.TestCase):
    PATH = [[0.0, 0.0], [2.0, 0.0], [2.0, 1.0]]

    def test_length(self):
        self.assertAlmostEqual(path_length(self.PATH), 3.0)

    def test_pose_on_first_segment(self):
        x, y, yaw = path_pose_at(self.PATH, 1.0)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(yaw, 0.0)

    def test_pose_on_second_segment(self):
        x, y, yaw = path_pose_at(self.PATH, 2.5)
        self.assertAlmostEqual(x, 2.0)
        self.assertAlmostEqual(y, 0.5)
        self.assertAlmostEqual(yaw, math.pi / 2)

    def test_pose_clamps_to_end(self):
        x, y, yaw = path_pose_at(self.PATH, 99.0)
        self.assertAlmostEqual(x, 2.0)
        self.assertAlmostEqual(y, 1.0)

    def test_rejects_short_or_nonfinite_paths(self):
        with self.assertRaises(ValueError):
            path_pose_at([[0.0, 0.0]], 0.0)
        with self.assertRaises(ValueError):
            path_pose_at([[0.0, 0.0], [float("nan"), 1.0]], 0.0)


def _scenario_raw(events, actors):
    return {
        "schema_version": 2, "id": "x", "world": {"mode": "current"},
        "robot": {"id": "tinker2", "initial_pose": [0, 0, 0]},
        "actors": actors, "objects": [], "regions": [], "events": events,
        "dialogue": [], "postconditions": [{"name": "n", "path": "p", "operator": "equals", "value": True}],
    }


class EventValidationTest(unittest.TestCase):
    def _load(self, raw):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "x.json"
            p.write_text(json.dumps(raw))
            return ScenarioDefinition.load(p)

    def test_actor_path_start_requires_known_actor(self):
        raw = _scenario_raw(
            [{"at_sim_time": 0.0, "type": "actor_path_start", "actor": "ghost"}], []
        )
        with self.assertRaisesRegex(ValueError, "unknown actor"):
            self._load(raw)

    def test_actor_path_start_requires_valid_path(self):
        actor = {"id": "a", "asset_uri": "x.usda", "path": [[0.0, 0.0]]}
        raw = _scenario_raw(
            [{"at_sim_time": 0.0, "type": "actor_path_start", "actor": "a"}], [actor]
        )
        with self.assertRaisesRegex(ValueError, "path"):
            self._load(raw)

    def test_valid_actor_path_event_loads(self):
        actor = {"id": "a", "asset_uri": "x.usda", "path": [[0.0, 0.0], [1.0, 0.0]]}
        raw = _scenario_raw(
            [{"at_sim_time": 0.0, "type": "actor_path_start", "actor": "a"}], [actor]
        )
        self._load(raw)  # no raise


_ARENA_ARTIFACT = ROOT / "artifacts/arena/rcw2026/d2b559b43207c8d54ae2609f638dca1cc36ee8b7adc7e4d94aee86e7fb56729c"


@unittest.skipUnless(
    (_ARENA_ARTIFACT / "map.pgm").exists(),
    "rcw2026 arena artifact not present in this checkout (artifacts/ is gitignored)",
)
class ArenaScenarioPoseTest(unittest.TestCase):
    ARTIFACT = _ARENA_ARTIFACT

    def test_person_scenario_poses_are_free_on_derived_map(self):
        from tinker_sim_core.occupancy import OccupancyMap
        occupancy = OccupancyMap.from_pgm(
            self.ARTIFACT / "map.pgm", resolution=0.05, origin_x=-5.05, origin_y=-6.0
        )
        raw = json.loads(
            (ROOT / "simulation/scenarios/find-and-approach-person-rcw2026.json").read_text()
        )
        rx, ry, _ = raw["robot"]["initial_pose"]
        self.assertTrue(occupancy.free_with_clearance(rx, ry, 0.35))
        actor = raw["actors"][0]
        for x, y in actor["path"]:
            self.assertTrue(occupancy.free_with_clearance(x, y, 0.4))


class ActorPathDriverNodeAttributeTest(unittest.TestCase):
    """Guard the rclpy attribute-shadowing class of bug found live on 2026-08-20.

    ``rclpy.node.Node.__init__`` assigns instance attributes such as
    ``self._clock = ROSClock()``. A subclass method with a colliding name is
    shadowed by that attribute, so passing ``self._clock`` to
    ``create_subscription`` silently registers the clock object as the callback
    and the first message raises ``TypeError: 'ROSClock' object is not
    callable``. This is AST-only so it runs under system python without ROS.
    """

    DRIVER = ROOT / "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/actor_path_driver.py"
    # Instance attributes rclpy.node.Node sets on itself; a subclass must not
    # reuse these names for its own methods.
    RCLPY_NODE_ATTRIBUTES = frozenset(
        {"_clock", "_parameters", "_context", "_handle", "_executor", "_logger"}
    )

    def setUp(self):
        import ast

        self.ast = ast
        self.assertTrue(self.DRIVER.is_file(), f"{self.DRIVER} does not exist")
        self.tree = ast.parse(self.DRIVER.read_text(encoding="utf-8"))

    def test_no_method_shadows_an_rclpy_node_attribute(self):
        for node in self.ast.walk(self.tree):
            if not isinstance(node, self.ast.FunctionDef):
                continue
            self.assertNotIn(
                node.name,
                self.RCLPY_NODE_ATTRIBUTES,
                f"method {node.name!r} collides with an rclpy.node.Node instance "
                f"attribute and will be shadowed by it",
            )

    def test_subscription_callbacks_resolve_to_defined_methods(self):
        defined = {
            node.name
            for node in self.ast.walk(self.tree)
            if isinstance(node, self.ast.FunctionDef)
        }
        found_any = False
        for node in self.ast.walk(self.tree):
            if not isinstance(node, self.ast.Call):
                continue
            if getattr(node.func, "attr", "") != "create_subscription":
                continue
            self.assertGreaterEqual(len(node.args), 3, "unexpected create_subscription arity")
            callback = node.args[2]
            if not isinstance(callback, self.ast.Attribute):
                continue
            found_any = True
            self.assertIn(
                callback.attr,
                defined,
                f"subscription callback self.{callback.attr} is not a method defined "
                f"in this module; it will resolve to an inherited attribute",
            )
            self.assertNotIn(
                callback.attr,
                self.RCLPY_NODE_ATTRIBUTES,
                f"subscription callback self.{callback.attr} names an rclpy.node.Node "
                f"instance attribute, not this class's method",
            )
        self.assertTrue(found_any, "expected at least one create_subscription callback")


if __name__ == "__main__":
    unittest.main()
