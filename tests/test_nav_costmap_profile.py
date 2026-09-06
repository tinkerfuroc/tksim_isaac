"""Nav2's global costmap must match the localization mode the launch chose.

``tk26_navigation``'s ``nav2_dwb_params.yaml`` configures the **global**
costmap for SLAM-without-a-prior-map: no ``static_layer``, and a rolling
10 x 10 m window pinned to ``base_link``.  The file says so in its own
comment, and offers the two-line rollback for prior-map use.

``gpsr.launch.py`` does not run SLAM.  It starts ``map_server`` on the arena
map and localizes with AMCL, so the rolling window is the wrong profile: any
goal further than half the window from the robot falls off the costmap and the
planner refuses it outright.  Observed in GPSR run11, with the robot at
(-2.02, -2.07) and the kitchen table at (3.29, -2.35), 5.3 m away::

    worldToMap failed: mx,my: 205,95, size_x,size_y: 200,200
    The goal sent to the planner is off the global costmap.
        Planning will always fail to this goal.
    Planning algorithm GridBased failed to generate a valid path to (3.29, -2.35)

The overlay below applies the rollback the upstream file documents, without
editing the upstream file: the simulation adapts to the hardware
configuration, never the other way around.
"""

from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.nav_params_overlay import (  # noqa: E402
    prior_map_costmap_overlay,
    write_prior_map_params,
)

SLAM_PROFILE = {
    "global_costmap": {
        "global_costmap": {
            "ros__parameters": {
                "global_frame": "map",
                "rolling_window": True,
                "width": 10,
                "height": 10,
                "resolution": 0.05,
                "track_unknown_space": False,
                "plugins": ["obstacle_layer", "inflation_layer"],
                "obstacle_layer": {"plugin": "nav2_costmap_2d::ObstacleLayer"},
                "inflation_layer": {"plugin": "nav2_costmap_2d::InflationLayer"},
            }
        }
    },
    "local_costmap": {
        "local_costmap": {
            "ros__parameters": {
                "global_frame": "odom",
                "rolling_window": True,
                "width": 5,
                "height": 5,
                "plugins": ["voxel_layer", "inflation_layer"],
                "voxel_layer": {
                    "plugin": "nav2_costmap_2d::VoxelLayer",
                    "observation_sources": "scan",
                    "scan": {"topic": "/scan", "data_type": "LaserScan"},
                },
                "inflation_layer": {"plugin": "nav2_costmap_2d::InflationLayer"},
            }
        }
    },
    "controller_server": {"ros__parameters": {"controller_frequency": 20.0}},
}


def _global(params):
    return params["global_costmap"]["global_costmap"]["ros__parameters"]


class PriorMapOverlayTest(unittest.TestCase):
    def setUp(self):
        self.source = copy.deepcopy(SLAM_PROFILE)
        self.result = prior_map_costmap_overlay(self.source)

    def test_global_costmap_stops_rolling(self):
        """A rolling window is what put the kitchen table off the map."""
        self.assertFalse(_global(self.result)["rolling_window"])

    def test_static_layer_runs_first(self):
        """StaticLayer sizes the master grid to the arena map."""
        self.assertEqual(
            _global(self.result)["plugins"],
            ["static_layer", "inflation_layer"],
        )

    def test_global_costmap_drops_the_obstacle_layer(self):
        """The sim lidar raycasts the same PGM the static layer loads.

        The obstacle layer can add nothing to the global map; projected with
        the AMCL/EKF estimate instead of the true pose it only draws
        displaced wall copies whose inscribed rings disconnected the planner
        for single ticks (run 20260906T041154: 42 refusal streaks, median
        0 s, 67 % during plain driving).
        """
        section = _global(self.result)
        self.assertNotIn("obstacle_layer", section)
        self.assertNotIn("obstacle_layer", section["plugins"])
        self.assertEqual(section["plugins"][-1], "inflation_layer")
        self.assertEqual(
            section["inflation_layer"]["plugin"],
            "nav2_costmap_2d::InflationLayer",
        )

    def test_static_layer_is_declared(self):
        """A plugin named in ``plugins`` needs its own type block."""
        static = _global(self.result)["static_layer"]
        self.assertEqual(static["plugin"], "nav2_costmap_2d::StaticLayer")
        self.assertTrue(static["map_subscribe_transient_local"])

    def test_unknown_space_is_tracked(self):
        """Prior-map mode distinguishes unknown cells from free ones."""
        self.assertTrue(_global(self.result)["track_unknown_space"])

    def test_local_costmap_is_untouched(self):
        """The odom-frame rolling window is correct and stays as it is.

        In particular its ``/scan`` source survives: the local costmap is
        the controller's only reactive layer, and it lives in ``odom`` so
        AMCL corrections never offset its marks (unlike the global map).
        """
        self.assertEqual(
            self.result["local_costmap"], SLAM_PROFILE["local_costmap"]
        )

    def test_unrelated_sections_survive(self):
        # controller_server keeps its upstream keys; the overlay's only
        # unconditional addition is the real odometry topic (the default
        # ``odom`` has no publisher on this robot).
        cs = self.result["controller_server"]["ros__parameters"]
        self.assertEqual(
            cs["controller_frequency"],
            SLAM_PROFILE["controller_server"]["ros__parameters"][
                "controller_frequency"
            ],
        )
        self.assertEqual(cs["odom_topic"], "/tracer/odom")

    def test_input_is_not_mutated(self):
        """The caller's parsed upstream params stay as they were read."""
        self.assertEqual(self.source, SLAM_PROFILE)


class WritePriorMapParamsTest(unittest.TestCase):
    def test_writes_overlaid_copy_and_leaves_source_alone(self):
        import tempfile

        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            source = tmp / "upstream.yaml"
            source.write_text(yaml.safe_dump(SLAM_PROFILE), encoding="utf-8")
            before = source.read_text(encoding="utf-8")

            written = write_prior_map_params(source, tmp / "out.yaml")

            self.assertEqual(written, tmp / "out.yaml")
            self.assertEqual(source.read_text(encoding="utf-8"), before)
            produced = yaml.safe_load(written.read_text(encoding="utf-8"))
            self.assertFalse(_global(produced)["rolling_window"])
            self.assertIn("static_layer", _global(produced)["plugins"])


class UpstreamParamsTest(unittest.TestCase):
    """The overlay has to work on the real file, not just the fixture."""

    UPSTREAM = Path(
        "/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup"
        "/params/nav2_dwb_params.yaml"
    )

    def setUp(self):
        if not self.UPSTREAM.exists():
            self.skipTest(f"tk25_ws checkout not present: {self.UPSTREAM}")
        self.params = yaml.safe_load(self.UPSTREAM.read_text(encoding="utf-8"))

    def test_overlay_puts_the_real_file_in_prior_map_mode(self):
        result = prior_map_costmap_overlay(self.params)
        section = _global(result)
        self.assertFalse(section["rolling_window"])
        self.assertTrue(section["track_unknown_space"])
        self.assertEqual(section["plugins"], ["static_layer", "inflation_layer"])
        # The global map is static-only in simulation (see
        # nav_params_overlay.py); the upstream obstacle block is dropped
        # from the copy rather than left as an unreferenced parameter set.
        self.assertNotIn("obstacle_layer", section)
        # The local costmap's live scan source is carried through untouched.
        local = result["local_costmap"]["local_costmap"]["ros__parameters"]
        upstream_local = self.params["local_costmap"]["local_costmap"]["ros__parameters"]
        self.assertEqual(local["voxel_layer"], upstream_local["voxel_layer"])
        self.assertIn("voxel_layer", local["plugins"])

    def test_inflation_forms_a_doorway_cost_bowl(self):
        # Doorway fix round 3 (Nav2 tuning guide): inflation must exceed
        # the inscribed radius by enough that a 0.95 m doorway becomes a
        # centering cost bowl; below it, plans legally hug the posts.
        result = prior_map_costmap_overlay(self.params)
        for costmap in ("local_costmap", "global_costmap"):
            radius = (
                result[costmap][costmap]["ros__parameters"]
                ["inflation_layer"]["inflation_radius"]
            )
            self.assertEqual(radius, 0.45)

    def test_progress_checker_tolerates_door_pivots(self):
        # SimpleProgressChecker counts XY translation only; the upstream
        # 0.5 m / 10 s window aborted every in-place door pivot.
        result = prior_map_costmap_overlay(self.params)
        checker = result["controller_server"]["ros__parameters"]["progress_checker"]
        self.assertEqual(checker["required_movement_radius"], 0.15)
        self.assertEqual(checker["movement_time_allowance"], 30.0)

    def test_follow_path_is_rotation_shim_over_rpp(self):
        # Doorway round 4: DWB failed three tuned rounds at the same
        # 0.95 m doorway turn; the Nav2 tuning guide recommends RPP for
        # doorway turns, fronted by the rotation shim so the door-entry
        # pivot is explicit rather than something the sampler must find.
        result = prior_map_costmap_overlay(self.params)
        fp = result["controller_server"]["ros__parameters"]["FollowPath"]
        # BARE RPP — no RotationShim: the Humble shim ramps its rotation
        # command from MEASURED odom velocity, which deadlocks against the
        # sim base's ~0.1 rad/s angular deadband (commanded 0.084 forever,
        # v_x never above 0.000). RPP runs bare.
        self.assertIn("RegulatedPurePursuitController", fp["plugin"])
        self.assertNotIn("primary_controller", fp)
        self.assertEqual(fp["rotate_to_heading_angular_vel"], 0.6)
        self.assertTrue(fp["use_cost_regulated_linear_velocity_scaling"])
        self.assertTrue(fp["use_rotate_to_heading"])
        self.assertNotIn("critics", fp)

    def test_controller_reads_the_robots_real_odometry(self):
        # Upstream leaves controller_server's odom_topic at the default
        # ``odom`` — a topic nothing publishes on this robot (odometry is
        # /tracer/odom). Measured speed then reads 0.0 forever: RPP's
        # rotate ramp is clamped to a single accel step (0.084 rad/s) and
        # the velocity-scaled lookahead never leaves its minimum.
        result = prior_map_costmap_overlay(self.params)
        cs = result["controller_server"]["ros__parameters"]
        self.assertEqual(cs["odom_topic"], "/tracer/odom")

    def test_min_lookahead_clears_the_goal_tolerance(self):
        # Humble RPP's shouldRotateToGoalHeading compares the CARROT
        # distance against the goal checker's xy tolerance. At standstill
        # the carrot sits at min_lookahead_dist, so min_lookahead below
        # the tolerance means "at goal" from the first tick: linear stays
        # 0.0 and the robot spins in place until the progress checker
        # aborts (measured: carrot pinned at 0.25 m vs tolerance 0.3,
        # v_x 0.000 for the whole 900 s battery run).
        result = prior_map_costmap_overlay(self.params)
        cs = result["controller_server"]["ros__parameters"]
        fp = cs["FollowPath"]
        xy_tol = cs["general_goal_checker"]["xy_goal_tolerance"]
        self.assertGreater(fp["min_lookahead_dist"], xy_tol)

    def test_footprint_is_sim_parity_not_hardware_envelope(self):
        # The upstream 0.95 m footprint models the REAL robot's rear
        # overhang; the sim body spans x [-0.38, +0.13] (robot.urdf base
        # mesh). With the hardware envelope a 90-degree turn into a
        # 0.95 m doorway has no collision-free DWB trajectory.
        result = prior_map_costmap_overlay(self.params)
        for costmap in ("local_costmap", "global_costmap"):
            fp = result[costmap][costmap]["ros__parameters"]["footprint"]
            self.assertIn("-0.45", fp)
            self.assertNotIn("-0.7", fp)

if __name__ == "__main__":
    unittest.main()
