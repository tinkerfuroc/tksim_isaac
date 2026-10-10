#!/usr/bin/env python3
"""Tally Nav2 and approach_planner warnings/errors across sessions.

    python3 nav_warn_tally.py [--sessions '202610*'] [--node approach_planner] [--top 60] [--info]

Numbers in messages are replaced by N so repeats group together. Columns:
occurrences, sessions affected, node, level, message. --node planner_node
selects approach_planner (it runs as planner_node in 05-nav_extras-0.log).
"""

import argparse
import collections
import glob
import os
import re

from navtrace import STACK_LOGS, read_ros_log, session_logs

NODES = {"controller_server", "planner_server", "behavior_server", "bt_navigator",
         "velocity_smoother", "amcl", "initial_pose", "planner_node"}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sessions", default="202610*")
    ap.add_argument("--logs", default=STACK_LOGS)
    ap.add_argument("--node", help="only this node (e.g. planner_node, controller_server)")
    ap.add_argument("--info", action="store_true", help="include INFO lines")
    ap.add_argument("--top", type=int, default=60)
    args = ap.parse_args()

    count, sessions = collections.Counter(), collections.defaultdict(set)
    for sdir in sorted(glob.glob(os.path.join(args.logs, args.sessions))):
        for log in session_logs(sdir):
            for _t, node, level, msg in read_ros_log(log, {args.node} if args.node else NODES):
                if level == "INFO" and not args.info:
                    continue
                key = (node, level, re.sub(r"-?\d+(\.\d+)?", "N", msg)[:200])
                count[key] += 1
                sessions[key].add(os.path.basename(sdir))
    for key, n in count.most_common(args.top):
        print(f"{n:6} {len(sessions[key]):4}  {key[0]:17} {key[1]:5} {key[2]}")


if __name__ == "__main__":
    main()
