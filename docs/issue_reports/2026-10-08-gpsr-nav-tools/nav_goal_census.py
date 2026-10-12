#!/usr/bin/env python3
"""Census of every Nav2 goal to one point, across all stack-log sessions (issue 1a/1c).

    python3 nav_goal_census.py [--goal -2,-2] [--sessions '202610*'] [--worst 8]

Uses only 02-bridge.log, so it covers every session without rosbags. Goals
are bucketed by start distance; per bucket: count, median/max duration,
goals over 30 s, goals with RPP "collision ahead", "Failed to make progress",
a recovery spin, and outcomes. Baseline 2026-10-01..07, goal (-2,-2):
  <0.1 m: 146 goals, median 0.1 s;  0.1-0.5 m: 57 goals, median 64 s, 38 over 30 s.
"""

import argparse
import collections
import glob
import math
import os

from navtrace import BEGIN_NAV, STACK_LOGS, read_ros_log

BUCKETS = [("<0.1", 0.0, 0.1), ("0.1-0.5", 0.1, 0.5), ("0.5-2", 0.5, 2.0), (">=2", 2.0, 1e9)]


def goals_in_session(sdir):
    cur = None
    for t, node, _lvl, msg in read_ros_log(os.path.join(sdir, "02-bridge.log"),
                                            {"bt_navigator", "controller_server", "behavior_server"}):
        m = BEGIN_NAV.search(msg)
        if m:
            if cur:
                cur.update(end=t, outcome="preempted")
                yield cur
            sx, sy, gx, gy = map(float, m.groups())
            cur = dict(session=os.path.basename(sdir), t0=t, d=math.hypot(sx - gx, sy - gy),
                       goal=(gx, gy), coll=0, fail=0, spin=0)
            continue
        if not cur:
            continue
        if "collision ahead" in msg:
            cur["coll"] += 1
        elif "Failed to make progress" in msg:
            cur["fail"] += 1
        elif msg == "Running spin":
            cur["spin"] += 1
        elif node == "bt_navigator" and msg in ("Goal succeeded", "Goal failed", "Goal canceled"):
            cur.update(end=t, outcome=msg.split()[1])
            yield cur
            cur = None


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--goal", default="-2,-2")
    ap.add_argument("--sessions", default="202610*", help="glob under the stack-log dir")
    ap.add_argument("--logs", default=STACK_LOGS)
    ap.add_argument("--worst", type=int, default=8)
    args = ap.parse_args()
    gx, gy = map(float, args.goal.split(","))

    rows = [g for s in sorted(glob.glob(os.path.join(args.logs, args.sessions)))
            for g in goals_in_session(s) if math.hypot(g["goal"][0] - gx, g["goal"][1] - gy) < 0.01]
    print(f"goals to ({gx},{gy}): {len(rows)} across {len({r['session'] for r in rows})} sessions")
    for label, lo, hi in BUCKETS:
        b = [r for r in rows if lo <= r["d"] < hi]
        if not b:
            continue
        dur = sorted(r["end"] - r["t0"] for r in b)
        print(f"{label:8} n={len(b):3} median={dur[len(dur) // 2]:6.1f}s max={dur[-1]:6.1f}s "
              f">30s={sum(x > 30 for x in dur):3} collision_ahead={sum(r['coll'] > 0 for r in b):3} "
              f"progress_fail={sum(r['fail'] > 0 for r in b):3} spin_recovery={sum(r['spin'] > 0 for r in b):3} "
              f"outcomes={dict(collections.Counter(r['outcome'] for r in b))}")
    near = sorted((r for r in rows if 0.1 <= r["d"] < 0.5), key=lambda r: r["t0"] - r["end"])
    if near and args.worst:
        print(f"\nslowest 0.1-0.5 m starts:")
        for r in near[:args.worst]:
            print(f"  {r['session']} t0={r['t0']:.0f} d={r['d']:.2f} dur={r['end'] - r['t0']:.0f}s "
                  f"collision_ahead={r['coll']} progress_fail={r['fail']} spin={r['spin']} {r['outcome']}")


if __name__ == "__main__":
    main()
