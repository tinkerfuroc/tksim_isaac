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
