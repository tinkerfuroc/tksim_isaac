"""``navigation.launch.py`` must run the prior-map costmap profile too.

``nav_params_overlay`` exists because ``tk26_navigation``'s
``nav2_dwb_params.yaml`` configures the GLOBAL costmap for SLAM without a prior
map: no ``static_layer``, and a rolling 10 x 10 m window on ``base_link``. Any
launch that instead runs ``map_server`` + AMCL needs the documented rollback,
or the planner rejects distant goals without searching.

``gpsr.launch.py`` applies it. ``navigation.launch.py`` did not -- it runs
exactly the same prior-map mode (it starts ``map_server`` on the arena map and
AMCL) but handed Nav2 the raw upstream file. Measured live, robot at map
(-0.169, -0.061), goal at a GPSR person-standing spot 3.4 m away and verified
free against the published OccupancyGrid::

    global_costmap: Using plugin "obstacle_layer"
    global_costmap: Using plugin "inflation_layer"        <- no static_layer
    planner_server: The goal sent to the planner is off the global costmap.
        Planning will always fail to this goal.

The goal was accepted, the robot burned 108 s in recovery behaviours, and
``navigate_to_pose`` aborted. That is the failure the overlay's own docstring
predicts verbatim, on the launch path a navigation battery actually uses.

These tests read the launch source rather than executing it: importing a launch
file pulls in ``launch``/``launch_ros``, which the repo's clean pytest env does
not carry.
"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "ros2_ws/src/tinker_sim_bridge/launch/navigation.launch.py"
GPSR_LAUNCH = ROOT / "ros2_ws/src/tinker_sim_bridge/launch/gpsr.launch.py"
UPSTREAM = "nav2_dwb_params.yaml"


class NavigationLaunchCostmapProfileTest(unittest.TestCase):
    def setUp(self) -> None:
        self.source = LAUNCH.read_text(encoding="utf-8")
        self.tree = ast.parse(self.source)

    def _imported_names(self) -> set[str]:
        names: set[str] = set()
        for node in ast.walk(self.tree):
            if isinstance(node, ast.ImportFrom):
                for alias in node.names:
                    names.add(alias.asname or alias.name)
        return names

    def test_overlay_is_imported(self) -> None:
        names = self._imported_names()
        self.assertIn(
            "write_prior_map_params",
            names,
            "navigation.launch.py runs map_server + AMCL, so it must apply the "
            "prior-map costmap overlay like gpsr.launch.py does",
        )

    def test_overlay_is_actually_called(self) -> None:
        called = {
            node.func.id
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }
        self.assertIn(
            "write_prior_map_params",
            called,
            "importing the overlay is not enough; it must be applied",
        )

    @staticmethod
    def _literal_mentions(tree: ast.Module) -> int:
        """How often the upstream filename appears in a real string literal.

        Counted through the AST, not the raw text: both launches also NAME the
        file in an explanatory comment, and a comment is not a params_file.
        """
        return sum(
            1
            for node in ast.walk(tree)
            if isinstance(node, ast.Constant)
            and isinstance(node.value, str)
            and UPSTREAM in node.value
        )

    def test_upstream_params_are_not_handed_to_nav2_directly(self) -> None:
        """The raw file may be referenced once, as the overlay's source.

        Every extra reference is a params_file that bypasses the overlay,
        which is exactly how the global costmap ended up with no static_layer.
        """
        mentions = self._literal_mentions(self.tree)
        self.assertEqual(
            mentions,
            1,
            f"{UPSTREAM} is used in {mentions} string literals in "
            "navigation.launch.py; it should be used once, as the overlay's "
            "input. Passing it straight to Nav2 gives the global costmap the "
            "SLAM profile (rolling window, no static_layer) and distant goals "
            "fail with 'off the global costmap'",
        )

    def test_it_matches_the_launch_that_already_gets_this_right(self) -> None:
        """gpsr.launch.py is the reference; both run prior-map mode."""
        gpsr_source = GPSR_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            "write_prior_map_params", gpsr_source, "reference launch changed"
        )
        self.assertEqual(
            self._literal_mentions(ast.parse(gpsr_source)),
            1,
            "reference launch should use the upstream file in exactly one "
            "string literal, as the overlay's input",
        )


if __name__ == "__main__":
    unittest.main()
