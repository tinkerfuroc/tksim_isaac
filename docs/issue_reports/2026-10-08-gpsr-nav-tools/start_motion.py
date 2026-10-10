#!/usr/bin/env python3
"""Summarise how each trial leaves its start pose (issue 1a / 3a).

    python3 start_motion.py [TRACE_DIR or CSV ...]     (default /tmp/gpsr-nav-traces)

Per trace: start pose, distance from spawn, seconds until first motion,
dwell and cumulative rotation before first getting 0.5 m from the start,
yaw-direction reversals in that window, and spin-in-place windows anywhere
(>300 deg of rotation within 20 s while the pose stays inside 0.4 m).
Clean starts from spawn: 3-13 s dwell, 24-93 deg, 0 reversals.
"""

import glob
import math
import os
import sys

from navtrace import SPAWN, TRACE_DIR, load_trace, wrap


def spin_windows(tr, window=20.0, min_deg=300.0, max_extent=0.4):
    ds, last = [], -1e9
    for p in tr:                      # 1 Hz is enough and keeps this fast
        if p[0] - last >= 1.0:
            ds.append(p)
            last = p[0]
    cum = [0.0]
    for i in range(1, len(ds)):
        cum.append(cum[-1] + abs(wrap(ds[i][4] - ds[i - 1][4])))
    found, j = [], 0
    for i in range(len(ds)):
        j = max(j, i)
        while j + 1 < len(ds) and ds[j + 1][0] - ds[i][0] <= window:
            j += 1
        if math.degrees(cum[j] - cum[i]) <= min_deg:
            continue
        xs = [p[1] for p in ds[i:j + 1]]
        ys = [p[2] for p in ds[i:j + 1]]
        if math.hypot(max(xs) - min(xs), max(ys) - min(ys)) < max_extent:
            span = (ds[i][0], ds[j][0], math.degrees(cum[j] - cum[i]), ds[i][1], ds[i][2])
            if found and span[0] <= found[-1][1] + 1:
                f = found[-1]
                found[-1] = (f[0], max(f[1], span[1]), max(f[2], span[2]), f[3], f[4])
            else:
                found.append(span)
    return found


def summarise(path):
    tr = load_trace(path)
    if not tr:
        return None
    t0, x0, y0, _, prev = tr[0]
    cum, t_move, t_leave, rev, last_sign = 0.0, None, None, 0, 0
    for t, x, y, _yaw, oyaw in tr:
        d = wrap(oyaw - prev)
        prev = oyaw
        if t_move is None and (abs(d) > 1e-3 or math.hypot(x - x0, y - y0) > 0.03):
            t_move = t
        if math.hypot(x - x0, y - y0) > 0.5:
            t_leave = t
            break
        cum += abs(d)
        if abs(d) > 0.005:
            sign = 1 if d > 0 else -1
            if last_sign and sign != last_sign:
                rev += 1
            last_sign = sign
    return dict(
        name=os.path.basename(path)[:-4], t0=t0, start=(x0, y0),
        d_spawn=math.hypot(x0 - SPAWN[0], y0 - SPAWN[1]),
        t_move=(t_move - t0) if t_move else float("nan"),
        dwell=(t_leave - t_move) if t_leave and t_move else float("nan"),
        rot=math.degrees(cum), rev=rev, spins=spin_windows(tr))


def main() -> None:
    paths = []
    for arg in sys.argv[1:] or [TRACE_DIR]:
        paths += sorted(glob.glob(os.path.join(arg, "*.csv"))) if os.path.isdir(arg) else [arg]
    print("trace | start (x,y) | d_spawn | t_move s | dwell<0.5m s | rot deg | reversals | spin windows")
    for p in paths:
        if os.path.getsize(p) < 100:
            continue
        s = summarise(p)
        if not s:
            continue
        spins = "; ".join(f"{b - a:.0f}s {r:.0f}deg @({x:.1f},{y:.1f}) +{a - s['t0']:.0f}s"
                          for a, b, r, x, y in s["spins"])
        print(f"{s['name']} | ({s['start'][0]:.2f},{s['start'][1]:.2f}) | {s['d_spawn']:.2f} | "
              f"{s['t_move']:.1f} | {s['dwell']:.1f} | {s['rot']:.0f} | {s['rev']} | {spins}")


if __name__ == "__main__":
    main()
