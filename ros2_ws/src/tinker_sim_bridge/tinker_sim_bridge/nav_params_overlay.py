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
    # upstream file stays hardware's.
    controller = (
        result.get("controller_server", {})
        .get("ros__parameters", {})
        .get("general_goal_checker")
    )
    if isinstance(controller, dict):
        controller["xy_goal_tolerance"] = 0.3
        controller["yaw_goal_tolerance"] = 0.4

    # Planner tolerance: with the relaxed 0.3 m goal tolerance the robot
    # parks close to furniture, sometimes inside the inflation ring. At the
    # upstream GridBased tolerance of 0.1 the global planner then refuses
    # every subsequent goal ("failed to generate a valid path", observed 152x
    # on a return-to-start leg) and the run burns its whole budget retrying.
    # 0.6 lets the planner accept a nearby reachable cell instead.
    planner = (
        result.get("planner_server", {})
        .get("ros__parameters", {})
        .get("GridBased")
    )
    if isinstance(planner, dict):
        planner["tolerance"] = 0.6

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
    # fix), while the actual sim body passes easily. Overlay a sim-true
    # envelope with margin (0.65 x 0.54): rear -0.45, front +0.20, half-
    # width 0.27 (covers wheels). Upstream file stays hardware's.
    sim_footprint = "[ [0.20, 0.27], [0.20, -0.27], [-0.45, -0.27], [-0.45, 0.27] ]"
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
    follow_path = (
        result.get("controller_server", {})
        .get("ros__parameters", {})
        .get("FollowPath")
    )
    if isinstance(follow_path, dict):
        critics = follow_path.get("critics")
        if isinstance(critics, list) and "BaseObstacle" in critics:
            follow_path["critics"] = [
                "ObstacleFootprint" if c == "BaseObstacle" else c
                for c in critics
            ]
            follow_path.pop("BaseObstacle.scale", None)
            follow_path["ObstacleFootprint.scale"] = 0.02
        # PathAlign/GoalAlign at 32/24 are documented (nav2 #938) to steer
        # "dangerously close to obstacles ... rounding corners"; the
        # forward-point fix (nav2 #1747) recommends a short lookahead.
        follow_path["PathAlign.scale"] = 12.0
        follow_path["GoalAlign.scale"] = 10.0
        follow_path["PathAlign.forward_point_distance"] = 0.1
        follow_path["GoalAlign.forward_point_distance"] = 0.1

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
