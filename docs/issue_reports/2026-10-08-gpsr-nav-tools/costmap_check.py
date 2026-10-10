#!/usr/bin/env python3
"""Static-map costmap check of robot poses for the hardware vs sim footprint (issues 1b/1c/2b).

    python3 costmap_check.py --pose=3.33,-2.25,-125 --pose=-0.70,-4.28 [--reach=-4.29,0.63:-3.69,1.0]
                             [--map .../map.yaml] [--inflation 0.45] [--scaling 8]

Use the --opt=value form: values starting with "-" are otherwise read as flags.

Rebuilds an approximate Nav2 costmap from the arena map (lethal 254,
inscribed 253 per footprint, exponential inflation) and, per pose, prints the
centre-cell cost, distance to the nearest map obstacle, footprint cost at the
given yaw, and how many of 72 in-place headings (5 deg steps) hit lethal (254)
or inscribed (>=253) cost. --reach START:GOAL flood-fills from START over
cells < 253 and counts free cells within 0.35 m of GOAL (the planner tolerance).

Approximation: only the static map (furniture is rasterised into it as
bounding boxes); lidar-only obstacles such as persons and spawned objects are
not modelled. Local costmap uses cost_scaling 8, global 5 (params file).
Needs numpy, scipy, Pillow, PyYAML.
"""

import argparse
import glob
import math
import os
from collections import deque

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt

HW = [(0.25, 0.25), (0.25, -0.25), (-0.7, -0.25), (-0.7, 0.25)]          # tinker_robot_config _common
SIM = [(0.20, 0.27), (0.20, -0.27), (-0.45, -0.27), (-0.45, 0.27)]       # nav_params_overlay.py
DEFAULT_MAP = sorted(glob.glob("/home/tinker/tinker-sim/6.0.1/artifacts/arena/rcw2026/*/map.yaml"))


class Grid:
    def __init__(self, map_yaml):
        meta = yaml.safe_load(open(map_yaml))
        img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), meta["image"])))
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        self.occ = np.flipud(img < 128)            # row 0 = bottom edge (y = origin y)
        self.h, self.w = self.occ.shape
        self.dist = distance_transform_edt(~self.occ) * self.res

    def cell(self, x, y):
        return int((x - self.ox) / self.res), int((y - self.oy) / self.res)


def inscribed(fp):
    best = 1e9
    for i, a in enumerate(fp):
        a, b = np.array(a), np.array(fp[(i + 1) % len(fp)])
        d = b - a
        t = np.clip(-a.dot(d) / d.dot(d), 0, 1)
        best = min(best, float(np.linalg.norm(a + t * d)))
    return best


def costmap(g, fp, scaling, inflation):
    ins = inscribed(fp)
    c = np.where(g.dist <= ins, 253,
                 np.where(g.dist <= inflation, (252 * np.exp(-scaling * (g.dist - ins))).astype(int), 0))
    c[g.occ] = 254
    return c


def footprint_cost(g, c, fp, x, y, yaw):
    cs, sn = math.cos(yaw), math.sin(yaw)
    pts = [(x + cs * a - sn * b, y + sn * a + cs * b) for a, b in fp]
    worst = 0
    for i, p in enumerate(pts):
        q = pts[(i + 1) % len(pts)]
        n = int(max(abs(q[0] - p[0]), abs(q[1] - p[1])) / (g.res / 2)) + 2
        for k in range(n + 1):
            ix, iy = g.cell(p[0] + k / n * (q[0] - p[0]), p[1] + k / n * (q[1] - p[1]))
            if not (0 <= ix < g.w and 0 <= iy < g.h):
                return 254
            worst = max(worst, int(c[iy, ix]))
    return worst


def reach(g, c, start, goal, tol=0.35):
    sx, sy = g.cell(*start)
    seen = np.zeros(c.shape, bool)
    seen[sy, sx] = True
    q, hits = deque([(sx, sy)]), 0
    while q:
        x, y = q.popleft()
        if c[y, x] < 253 and math.hypot(g.ox + (x + .5) * g.res - goal[0], g.oy + (y + .5) * g.res - goal[1]) <= tol:
            hits += 1
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                nx, ny = x + dx, y + dy
                if (dx or dy) and 0 <= nx < g.w and 0 <= ny < g.h and not seen[ny, nx] and c[ny, nx] < 253:
                    seen[ny, nx] = True
                    q.append((nx, ny))
    return hits, int(seen.sum())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", action="append", default=[], help="x,y[,yaw_deg]")
    ap.add_argument("--reach", action="append", default=[], help="sx,sy:gx,gy")
    ap.add_argument("--map", default=DEFAULT_MAP[0] if DEFAULT_MAP else None)
    ap.add_argument("--inflation", type=float, default=0.45)
    ap.add_argument("--scaling", type=float, default=8.0)
    args = ap.parse_args()
    if not args.map:
        raise SystemExit("pass --map")
    g = Grid(args.map)
    maps = {name: (fp, costmap(g, fp, args.scaling, args.inflation)) for name, fp in (("HW", HW), ("SIM", SIM))}
    print(f"map {args.map}\ninscribed radius HW {inscribed(HW):.2f} m, SIM {inscribed(SIM):.2f} m; "
          f"inflation {args.inflation}, scaling {args.scaling}")

    for spec in args.pose:
        v = [float(s) for s in spec.split(",")]
        x, y = v[0], v[1]
        ix, iy = g.cell(x, y)
        line = f"({x:.2f},{y:.2f}) nearest obstacle {g.dist[iy, ix]:.2f} m"
        for name, (fp, c) in maps.items():
            heads = [footprint_cost(g, c, fp, x, y, math.radians(a)) for a in range(0, 360, 5)]
            at_yaw = f" at_yaw={footprint_cost(g, c, fp, x, y, math.radians(v[2]))}" if len(v) > 2 else ""
            line += (f" | {name}: centre={c[iy, ix]}{at_yaw} lethal_headings={sum(h >= 254 for h in heads)}/72 "
                     f"inscribed_headings={sum(h >= 253 for h in heads)}/72")
        print(line)

    for spec in args.reach:
        s, t = spec.split(":")
        start, goal = tuple(map(float, s.split(","))), tuple(map(float, t.split(",")))
        res = {name: reach(g, c, start, goal) for name, (_fp, c) in maps.items()}
        print(f"reach {start} -> {goal}: " +
              ", ".join(f"{n}: {h} goal cells / {r} reachable" for n, (h, r) in res.items()))


if __name__ == "__main__":
    main()
