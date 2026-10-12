"""Shared helpers for the GPSR navigation debugging scripts in this folder.

Paths default to the locations used on the sim host; every script also takes
them as arguments.
"""

from __future__ import annotations

import csv
import glob
import math
import os
import re

BENCH_DIR = "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench"
STACK_LOGS = "/home/tinker/tinker-sim/6.0.1/gpsr_stack_logs"
TRACE_DIR = "/tmp/gpsr-nav-traces"
SPAWN = (-1.93, -2.00)
COMMAND_POINT = (-2.00, -2.00)

# "[controller_server-22] [WARN] [1791027951.2] [controller_server]: message"
ROS_LINE = re.compile(r"^\[([a-z_]+)-\d+\] \[(\w+)\] \[([0-9.]+)\] \[[^\]]+\]: (.*)")
BEGIN_NAV = re.compile(
    r"Begin navigating from current location \((-?[\d.]+), (-?[\d.]+)\) to \((-?[\d.]+), (-?[\d.]+)\)")


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def load_trace(path: str) -> list[tuple[float, float, float, float, float]]:
    """Return [(t, x, y, yaw, odom_yaw)] of base_link; x, y, yaw in the map frame.

    Measure rotation with odom_yaw: map-frame yaw jumps whenever AMCL
    corrects map->odom, which reads as spurious turning.

    The CSV holds raw map->odom and odom->base_link rows (see
    extract_tf_pose.py). base_link rows before the first map->odom are
    dropped: they are in the odom frame only and would read as a jump.
    """
    mo = None
    out = []
    with open(path) as fh:
        for r in csv.DictReader(fh):
            x, y, yaw = float(r["x"]), float(r["y"]), float(r["yaw"])
            if r["child"] == "odom":
                mo = (x, y, yaw)
                continue
            if r["child"] != "base_link" or mo is None:
                continue
            c, s = math.cos(mo[2]), math.sin(mo[2])
            out.append((float(r["t"]), mo[0] + c * x - s * y, mo[1] + s * x + c * y, wrap(mo[2] + yaw), yaw))
    return out


def read_ros_log(path: str, nodes: set[str] | None = None,
                 t0: float | None = None, t1: float | None = None):
    """Yield (epoch, node, level, message) from a launch log."""
    if not os.path.exists(path):
        return
    with open(path, errors="replace") as fh:
        for line in fh:
            m = ROS_LINE.match(line)
            if not m:
                continue
            node, level, t, msg = m.group(1), m.group(2), float(m.group(3)), m.group(4)
            if nodes and node not in nodes:
                continue
            if (t0 is not None and t < t0) or (t1 is not None and t > t1):
                continue
            yield t, node, level, msg


def session_logs(session_dir: str) -> list[str]:
    """Nav2 (02-bridge) and approach_planner (05-nav_extras) logs of a session."""
    return sorted(glob.glob(os.path.join(session_dir, "02-*.log")) +
                  glob.glob(os.path.join(session_dir, "05-*.log")))


def session_for_batch(batch: str, bench_dir: str = BENCH_DIR) -> str | None:
    """Stack-log session timestamp for a bench batch, from gpsr-stack-up.log."""
    path = os.path.join(bench_dir, batch, "gpsr-stack-up.log")
    if not os.path.exists(path):
        return None
    m = re.search(r"stack up\. logs: gpsr_stack_logs/(\S+)", open(path, errors="replace").read())
    return m.group(1) if m else None


def batch_for_session(session: str, bench_dir: str = BENCH_DIR) -> str | None:
    for path in glob.glob(os.path.join(bench_dir, "*", "gpsr-stack-up.log")):
        if f"gpsr_stack_logs/{session}" in open(path, errors="replace").read():
            return os.path.basename(os.path.dirname(path))
    return None
