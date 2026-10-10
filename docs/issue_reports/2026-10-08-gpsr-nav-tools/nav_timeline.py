#!/usr/bin/env python3
"""Condensed Nav2 + approach_planner timeline for one session and time window.

    python3 nav_timeline.py <session> <epoch_start> <epoch_end> [--all-nodes]

<session> is a stack-log dir or its timestamp (e.g. 20261003T062109).
Consecutive repeats of the same node+message prefix are collapsed and
"Passing new path" / tick-rate spam is dropped. Times print relative to
<epoch_start>. Example (issue 2a):
    python3 nav_timeline.py 20261003T062109 1791025586 1791025655
"""

import argparse
import os

from navtrace import STACK_LOGS, read_ros_log, session_logs

NAV_NODES = {"bt_navigator", "controller_server", "planner_server", "behavior_server",
             "planner_node", "amcl", "initial_pose", "velocity_smoother", "safety_supervisor"}
NOISE = ("Passing new path", "tick rate", "approach check: corridor memory")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("t0", type=float)
    ap.add_argument("t1", type=float)
    ap.add_argument("--all-nodes", action="store_true")
    args = ap.parse_args()
    sdir = args.session if os.path.isdir(args.session) else os.path.join(STACK_LOGS, args.session)

    events = []
    for log in session_logs(sdir):
        nodes = None if args.all_nodes else NAV_NODES
        events += list(read_ros_log(log, nodes, args.t0, args.t1))
    events.sort()
    prev = None
    for t, node, level, msg in events:
        if any(n in msg for n in NOISE):
            continue
        key = (node, msg[:30])
        if key == prev:
            continue
        prev = key
        print(f"{t - args.t0:7.1f} {node:17} {level:5} {msg[:220]}")


if __name__ == "__main__":
    main()
