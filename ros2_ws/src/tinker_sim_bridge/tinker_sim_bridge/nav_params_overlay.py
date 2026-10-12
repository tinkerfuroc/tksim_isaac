"""Match Nav2's global costmap to the localization mode the launch chose.

``tk26_navigation``'s ``nav2_dwb_params.yaml`` configures the global costmap
for SLAM without a prior map: no ``static_layer``, and a rolling 10 x 10 m
window centred on ``base_link``.  Its own comment records why, and names the
two lines to restore for prior-map use.

``gpsr.launch.py`` runs the prior-map mode -- ``map_server`` on the arena map
plus AMCL -- so the rolling window is the wrong profile for it.  A goal beyond
half the window simply is not on the costmap, and the planner rejects it
without searching::

    The goal sent to the planner is off the global costmap.
        Planning will always fail to this goal.

This module applies the documented rollback to a *copy* of the upstream file at
launch time.  The upstream file is hardware's, and is left untouched.
"""

from __future__ import annotations

import copy
import os
import tempfile
from pathlib import Path
from typing import Any, Mapping

import yaml

PRIOR_MAP_PLUGINS = ["static_layer", "obstacle_layer", "inflation_layer"]

STATIC_LAYER = {
    "plugin": "nav2_costmap_2d::StaticLayer",
    # map_server latches /map; a late-joining costmap needs the durable copy.
    "map_subscribe_transient_local": True,
}


def prior_map_costmap_overlay(params: Mapping[str, Any]) -> dict:
    """Return a copy of *params* with the global costmap in prior-map mode.

    The local costmap keeps its rolling window: it lives in ``odom`` and is
    meant to follow the robot.  Only the ``map``-frame global costmap has to
    span the arena.
    """
    result = copy.deepcopy(dict(params))
    node = result.get("global_costmap", {}).get("global_costmap", {})
    section = node.get("ros__parameters")
    if section is None:
        return result

    # StaticLayer resizes the master grid to the incoming map only when the
    # costmap is not rolling; left rolling, the arena map would be clipped to
    # the window and distant goals would still be off the grid.
    section["rolling_window"] = False
    section["track_unknown_space"] = True
    section["plugins"] = list(PRIOR_MAP_PLUGINS)
    section["static_layer"] = dict(STATIC_LAYER)

    # Goal tolerances: the upstream 0.10 m / 0.10 rad are hardware-precision
    # values. The sim controller's documented steady-state offsets
    # (0.014-0.072 rad per joint; base tracking in the same family) park the
    # robot ~0.13 m from the goal, so goals never complete — follow_path
    # aborts and the goto retries forever. Relax on the sim copy only; the
    # upstream file stays hardware's. 0.2 m / 0.25 rad (was 0.3 / 0.4): every
    # 2026-10-10/11 GPSR battery and Bug A check ran with these values; at
    # 0.3 the robot parks 0.3 m short and the side_table_02 departure pivot
    # still throws collision-ahead + a patience abort (2026-10-12 re-check).
    controller = (
        result.get("controller_server", {})
        .get("ros__parameters", {})
        .get("general_goal_checker")
    )
    if isinstance(controller, dict):
        controller["xy_goal_tolerance"] = 0.2
        controller["yaw_goal_tolerance"] = 0.25
        # Bug A pivot loop (2026-10-12): Humble RPP pivots to the goal yaw
        # only while the carrot is inside the checker's xy tolerance, with no
        # hysteresis, and controller_server resets SimpleGoalChecker's latch
        # on every replan (~1 Hz). Parked at the boundary, 1-3 cm of AMCL
        # jitter during the pivot flipped RPP between the goal-yaw and
        # path-heading pivots (at xy 0.3: estimate 0.282-0.314 m, 381 deg,
        # 5 reversals). The sticky checker keeps the latch for the goal
        # across replans and reports xy + release_margin to RPP while
        # latched; past that radius xy is required again. xy +
        # release_margin must stay under FollowPath's min_lookahead_dist,
        # or RPP's carrot-distance test reads "at goal" from any distance.
        # At xy 0.2 the stock checker did not loop in the same repros (the
        # robot coasts ~0.16-0.19 m in before pivoting), so this guards the
        # stop-on-the-boundary case rather than fixing a measured 0.2 loop.
        controller["plugin"] = "tinker_sim_nav_plugins::StickyGoalChecker"
        controller["release_margin"] = 0.04

    # Planner tolerance: with a relaxed goal tolerance the robot parks close
    # to furniture, sometimes inside the inflation ring. At the upstream
    # GridBased tolerance of 0.1 the global planner then refuses every
    # subsequent goal ("failed to generate a valid path", observed 152x on a
    # return-to-start leg) and the run burns its whole budget retrying.
    # 0.35 stays above the 0.2 m goal tolerance without the 0.6 m slop that
    # let the robot park half a metre from the intended pose.
    planner = (
        result.get("planner_server", {})
        .get("ros__parameters", {})
        .get("GridBased")
    )
    if isinstance(planner, dict):
        planner["tolerance"] = 0.35

    # AMCL odometry noise: the upstream alphas (0.2-0.3) model the real
    # tracer's wheel odometry. The sim base's odometry tracks Isaac ground
    # truth to 0.4% in rotation and millimetres in translation, so those
    # alphas only scatter the particles -- ~2.6 deg of heading spread per
    # filter update -- and the last update before the robot stops leaves
    # that scatter frozen in the estimate (AMCL does not update at rest).
    # Live vs ground truth, 2026-10-10, surface-only map, 4-stop route x 5
    # headings: alphas 0.2-0.3 -> mean 0.094 m, max 0.180 m; 0.1 -> 0.062 m;
    # 0.05 -> 0.057 m, max 0.109 m, yaw 1.6 -> 0.8 deg. (A 2026-09-11 trial
    # of 0.05 on the solid map was worse, 0.52 m vs 0.18 m, as particles
    # could not recover from scan/map mismatch; the surface-only map
    # removed that mismatch.)
    amcl = result.get("amcl", {}).get("ros__parameters")
    if isinstance(amcl, dict):
        for alpha in ("alpha1", "alpha2", "alpha3", "alpha4"):
            amcl[alpha] = 0.05
        # update_min_a stays at the upstream 0.1 rad. 3.0 (2026-10-11) held
        # the estimate through goal pivots to stop the Bug A loop, but
        # AMCL then cannot refine at a stop: route mean 0.055-0.076 m vs
        # 0.042-0.049 m at 0.1 (2026-10-12, fixed 10 Hz scan). The loop is
        # handled by the sticky goal checker instead (see
        # general_goal_checker above); with it, 0.1 gave a route mean of
        # 0.043 m, side_table_02 goals at 214-218 deg with no loop, and
        # 8/8 clean 120 deg boundary pivots.
        amcl["update_min_a"] = 0.1
        # AMCL localizes on a surface-only copy of the arena map, served by
        # its own map_server on this topic (see amcl_map.py); the costmaps
        # keep the solid map on /map.
        amcl["map_topic"] = "amcl_map"

    # Inflation, split by costmap (2026-08-28 doorway-wedge debug):
    #
    # GLOBAL 0.22: at 0.30/0.35 m (inscribed radius 0.26) a robot parked
    # 0.3 m from furniture starts INSIDE the lethal ring, and the global
    # planner refuses every path from there — the run wedges at its first
    # close approach (observed 152x). 0.22 keeps the parked poses
    # plannable.
    #
    # LOCAL 0.30: the upstream file's own annotation warns inflation must
    # stay ABOVE the asymmetric footprint's inscribed radius (0.26).
    # Dropping the local costmap to 0.22 removed the near-wall cost
    # gradient entirely (a cell 0.25 m from a wall — body overlapping —
    # scored 0), and DWB drove the robot's flank into the 0.95 m doorway
    # posts: live probe showed the base pressed against the wall at
    # (0.25, 2.87), v≈0, follow_path "Failed to make progress" aborting
    # every ~10 sim-s for the whole run.
    # 2026-08-28 round 3 (web-research-backed, Nav2 tuning guide): with
    # inflation below the door half-width there is no centering "cost
    # bowl" and plans run door-post-adjacent; the robot stalled on door
    # approaches even in open space. 0.45 on BOTH costmaps makes the
    # 0.95 m doorway a single-channel bowl that centers the path. (If
    # parked-near-furniture planner refusals return, lower ONLY the
    # global radius — that was the original reason for 0.22.)
    for costmap, radius in (("local_costmap", 0.45), ("global_costmap", 0.45)):
        node = result.get(costmap, {}).get(costmap, {}).get("ros__parameters", {})
        inflation = node.get("inflation_layer")
        if isinstance(inflation, dict) and "inflation_radius" in inflation:
            inflation["inflation_radius"] = radius

    # Footprint, sim-parity (2026-08-28 doorway-wedge debug, fix 2): the
    # upstream footprint [[0.25,0.25],[0.25,-0.25],[-0.7,-0.25],[-0.7,0.25]]
    # models the REAL robot's 0.95 m envelope (rear overhang). The sim
    # robot's collision model (artifacts robot.urdf base_link mesh, scale
    # 0.001, yaw -90deg, origin -0.38/0.20/0.03) spans x [-0.38, +0.13],
    # y [-0.205, +0.20] — about 0.51 x 0.41 m. With the 0.95 m footprint a
    # 90-degree turn INTO a 0.95 m doorway has no collision-free DWB
    # trajectory (validated: robot stalled at the north door mouth at
    # (-0.5, 3.66) with 32 progress-failures after the ObstacleFootprint
    # fix), while the actual sim body passes easily. Overlay the sim body
    # plus 3 cm (0.57 x 0.47): rear -0.41, front +0.16, half-width 0.235
    # (wheels sit inside |y| 0.157); circumscribed radius 0.473 m.
    # Upstream file stays hardware's. (The first sim envelope carried 7 cm,
    # radius 0.525: next to furniture it blocked in-place pivots the body
    # clears -- at the side_table_02 departure pose 22 of 72 headings hit
    # lethal and RPP's 1 s pivot projection deadlocked until a patience
    # abort, 2026-10-11. With 3 cm: 10 of 72, and that pivot clears.)
    # NOTE: navigation_dwb_launch.py then overwrites every footprint key from
    # the robot profile (RobotField('base.footprint')), so the value that
    # actually reaches the costmaps is tinker_robot_config's tinker2_sim
    # profile -- keep the two equal (tests/test_sim_robot_profile.py).
    sim_footprint = "[ [0.16, 0.235], [0.16, -0.235], [-0.41, -0.235], [-0.41, 0.235] ]"
    for costmap in ("local_costmap", "global_costmap"):
        node = result.get(costmap, {}).get(costmap, {}).get("ros__parameters", {})
        if "footprint" in node:
            node["footprint"] = sim_footprint

    # DWB obstacle critic: upstream scores trajectories with BaseObstacle
    # (CENTER-POINT cost, scale 0.02 — effectively decorative). A 0.5 m
    # wide, 0.95 m long footprint clears a 0.95 m door only when the WHOLE
    # polygon is checked; with the point critic DWB happily selected
    # door-clipping trajectories and the base physically caught the door
    # frame (same live probe as above). ObstacleFootprint vetoes any
    # candidate whose polygon touches lethal cost, independent of scale.
    # Controller (2026-08-28, doorway round 4): DWB failed three
    # progressively-tuned rounds at the same 0.95 m doorway turn
    # (ObstacleFootprint veto; sim-parity footprint; cost bowl + align
    # de-tune + pivot-tolerant progress checker — aborts fell 32 -> 10
    # but the door never cleared). The Nav2 tuning guide's own words:
    # RPP "makes better turns into doorways, whereas DWB can come close
    # to scraping the wall". Replace FollowPath with RotationShim
    # (pivots in place toward the path heading — exactly the door-entry
    # maneuver DWB could not sample) wrapping RegulatedPurePursuit
    # (exact path tracking with obstacle-proximity slowdown). The
    # upstream DWB config stays untouched in the hardware file.
    controller_ros = result.get("controller_server", {}).get(
        "ros__parameters", {}
    )
    # NO RotationShim (2026-08-28, measured): the sim base has an angular
    # DEADBAND — commanded |w|~0.08 rad/s produces ~zero actual rotation
    # (wheel-speed differential ~2 cm/s is eaten by PhysX friction). The
    # Humble shim ramps its rotation command from MEASURED odom velocity,
    # so it deadlocks: command 0.084, base does not move, measured stays
    # 0, command stays 0.084 forever (probe: 1810 cmd_vel msgs, mean and
    # max v_x exactly 0.000, w -0.084 constant, yaw drift ~0). RPP runs
    # bare.
    #
    # odom_topic (2026-08-28, measured): RPP — like the shim — ramps its
    # rotate command and scales its lookahead from MEASURED speed, which
    # controller_server reads from its ``odom_topic`` parameter. Upstream
    # leaves it at the default ``odom`` — a topic with ZERO publishers on
    # this robot (odometry is ``/tracer/odom``). Measured speed is then
    # permanently 0.0: the rotate ramp is clamped to one accel step
    # (2.1/25 Hz = 0.084 rad/s) forever, and the velocity-scaled
    # lookahead is pinned at min_lookahead_dist. Point it at the real
    # odometry.
    controller_ros["odom_topic"] = "/tracer/odom"
    if "FollowPath" in controller_ros:
        controller_ros["FollowPath"] = {
            "plugin":
                "nav2_regulated_pure_pursuit_controller"
                "::RegulatedPurePursuitController",
            "max_angular_accel": 2.1,
            # RPP: modest speed, velocity-scaled lookahead, cost-based
            # slowdown near the doorposts, pivot on large heading error.
            "desired_linear_vel": 0.35,
            "lookahead_dist": 0.5,
            # Humble RPP's shouldRotateToGoalHeading compares the CARROT
            # distance — not the true goal distance — against the goal
            # checker's xy tolerance (0.2 here). At standstill the
            # velocity-scaled lookahead clamps to min_lookahead_dist, so
            # a min below 0.2 makes RPP believe it is AT the goal from
            # the very first tick: linear stays 0.0 and it rotates to
            # the goal heading in place until the progress checker
            # aborts (measured at xy 0.3: carrot pinned at 0.25 m, v never
            # above 0.000 for 900 s). Keep min_lookahead_dist strictly above
            # the xy goal tolerance.
            "min_lookahead_dist": 0.25,
            "max_lookahead_dist": 0.7,
            "use_velocity_scaled_lookahead_dist": True,
            "lookahead_time": 1.5,
            "transform_tolerance": 0.1,
            "use_regulated_linear_velocity_scaling": True,
            "use_cost_regulated_linear_velocity_scaling": True,
            "regulated_linear_scaling_min_radius": 0.6,
            "regulated_linear_scaling_min_speed": 0.1,
            "use_rotate_to_heading": True,
            "rotate_to_heading_min_angle": 0.785,
            # Well above the sim base's measured angular deadband
            # (~0.1 rad/s); RPP ramps toward this from the last COMMANDED
            # velocity, so the pivot actually executes.
            "rotate_to_heading_angular_vel": 0.6,
            "max_angular_vel": 0.6,
            # Must exceed velocity_smoother's deadband_velocity (0.08):
            # near the goal RPP scales linear down to this floor, and a
            # command under the deadband is zeroed, so the base parks just
            # outside the tolerance and spins while AMCL's heading-
            # dependent estimate drags it back and forth across the
            # boundary (2026-10-10 live A/B: 0.05 -> median 74 s and
            # ~2000 deg per goal; 0.10 -> 16 s, ~480 deg).
            "min_approach_linear_velocity": 0.10,
            "approach_velocity_scaling_dist": 0.6,
            "max_robot_pose_search_dist": 10.0,
            "allow_reversing": False,
        }

    # SimpleProgressChecker counts XY translation only — an in-place
    # pivot into a doorway is "zero progress", and the upstream 0.5 m /
    # 10 s window matched our abort cadence exactly (aborts every ~10
    # sim-seconds at the door mouth). Loosen so slow door maneuvers and
    # pivots are not treated as being stuck.
    controller_params = result.get("controller_server", {}).get(
        "ros__parameters", {}
    )
    checker = controller_params.get("progress_checker")
    if isinstance(checker, dict):
        checker["required_movement_radius"] = 0.15
        checker["movement_time_allowance"] = 30.0
    return result


def write_prior_map_params(source: Path, destination: Path) -> Path:
    """Read *source*, apply the overlay, write it to *destination*."""
    params = yaml.safe_load(Path(source).read_text(encoding="utf-8"))
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        yaml.safe_dump(prior_map_costmap_overlay(params), sort_keys=True),
        encoding="utf-8",
    )
    return destination


def default_destination(source: Path) -> Path:
    """A per-user, per-domain scratch path for the generated params.

    The simulation host is shared, and several ROS domains run side by side.  A
    fixed ``/tmp`` name would have one session overwrite -- or fail to
    overwrite, on a foreign uid -- another session's params.
    """
    domain = os.environ.get("ROS_DOMAIN_ID", "0")
    name = f"{Path(source).stem}.prior_map.uid{os.getuid()}.domain{domain}.yaml"
    return Path(tempfile.gettempdir()) / name
