#!/usr/bin/env python3
"""List the Nav2 goals sent during one traced run, with the motion each caused.

    python3 trace_goals.py <trace.csv> [--session STACK_LOG_DIR] [--max 10]

The session defaults to the one recorded for the trace's batch
(<batch>__<entry>.csv -> bench/<batch>/gpsr-stack-up.log). For each
"Begin navigating" line inside the trace's time window: start -> goal,
duration, RPP "collision ahead" count, rotation in degrees, outcome.
"""

import argparse
import math
import os

from navtrace import BEGIN_NAV, STACK_LOGS, load_trace, read_ros_log, session_for_batch, wrap


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--session")
    ap.add_argument("--max", type=int, default=10)
    args = ap.parse_args()

    session = args.session
    if not session:
        batch = os.path.basename(args.csv).split("__")[0]
        ts = session_for_batch(batch)
        if not ts:
            raise SystemExit(f"no stack session recorded for batch {batch}; pass --session")
        session = os.path.join(STACK_LOGS, ts)
    tr = load_trace(args.csv)
    t0, t1 = tr[0][0], tr[-1][0]

    goals = []
    for t, node, _lvl, msg in read_ros_log(os.path.join(session, "02-bridge.log"),
                                            {"bt_navigator", "controller_server"}, t0 - 5, t1 + 5):
        m = BEGIN_NAV.search(msg)
        if m:
            goals.append(dict(t=t, sg=tuple(map(float, m.groups())), coll=0, end=None, outcome="(open)"))
        elif goals and goals[-1]["end"] is None:
            if "collision ahead" in msg:
                goals[-1]["coll"] += 1
            elif node == "bt_navigator" and msg in ("Goal succeeded", "Goal failed", "Goal canceled"):
                goals[-1]["end"], goals[-1]["outcome"] = t, msg

    def rotation(a, b):
        pts = [p for p in tr if a <= p[0] <= b]
        return math.degrees(sum(abs(wrap(q[4] - p[4])) for p, q in zip(pts, pts[1:])))

    print(f"{os.path.basename(args.csv)}  window {t1 - t0:.0f}s  session {session}")
    for g in goals[:args.max]:
        end = g["end"] or t1
        sx, sy, gx, gy = g["sg"]
        print(f"  +{g['t'] - t0:7.1f}s ({sx:.2f},{sy:.2f}) -> ({gx:.2f},{gy:.2f}) "
              f"d={math.hypot(gx - sx, gy - sy):.2f} dur={end - g['t']:6.1f}s "
              f"collision_ahead={g['coll']:4d} rot={rotation(g['t'], end):6.0f}deg  {g['outcome']}")


if __name__ == "__main__":
    main()
