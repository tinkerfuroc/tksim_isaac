#!/usr/bin/env python3
"""Print a map-frame pose trace from a CSV made by extract_tf_pose.py.

    python3 pose_trace.py <trace.csv> [--step 1.0] [--goal -2,-2]

Columns: seconds since first pose, epoch, x, y, yaw (deg), distance from the
first pose (d0), distance to --goal, cumulative |yaw change| (deg).
"""

import argparse
import math

from navtrace import COMMAND_POINT, load_trace, wrap


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--step", type=float, default=1.0, help="print interval, seconds (0 = every sample)")
    ap.add_argument("--goal", default=f"{COMMAND_POINT[0]},{COMMAND_POINT[1]}")
    args = ap.parse_args()
    gx, gy = map(float, args.goal.split(","))

    tr = load_trace(args.csv)
    if not tr:
        raise SystemExit("no map-frame poses in trace")
    t0, x0, y0, _, prev = tr[0]
    cum, last = 0.0, None
    for t, x, y, yaw, oyaw in tr:
        cum += abs(wrap(oyaw - prev))
        prev = oyaw
        if last is None or t - last >= args.step:
            last = t
            print(f"{t - t0:7.1f}s epoch={t:.1f} x={x:6.2f} y={y:6.2f} yaw={math.degrees(yaw):7.1f} "
                  f"d0={math.hypot(x - x0, y - y0):5.2f} d_goal={math.hypot(x - gx, y - gy):5.2f} "
                  f"cum_rot={math.degrees(cum):6.0f}")


if __name__ == "__main__":
    main()
