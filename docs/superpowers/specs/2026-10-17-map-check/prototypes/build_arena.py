#!/usr/bin/env python3
"""Extrude every occupied cell of a Nav2 occupancy map into a 0.6 m box arena.

Occupied cells are merged into maximal row-run rectangles (same column span
over consecutive rows) so the stage holds a few hundred Cubes instead of one
per cell. Output follows the arena-artifact layout the backend loads
(``payload/arena.usd`` + colocated ``payload/map.yaml``).
"""
import argparse
import json
import shutil
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

HERE = Path(__file__).resolve().parent
DEFAULT_MAP = Path("/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/maps/"
                   "robocup_home_2026_measured_5cm.yaml")


def occupied_mask(map_yaml):
    meta = yaml.safe_load(map_yaml.read_text())
    img = np.array(Image.open(map_yaml.parent / meta["image"]).convert("L"), dtype=float)
    p = img / 255.0 if meta.get("negate", 0) else (255.0 - img) / 255.0
    return p > meta["occupied_thresh"], meta


def fill_enclosed(mask, res, max_area_m2):
    """Fill free regions fully enclosed by occupied cells (furniture drawn as
    outlines). Regions larger than ``max_area_m2`` -- a room sealed off by
    the map -- are refused rather than turned into a solid block."""
    from scipy import ndimage

    holes = ndimage.binary_fill_holes(mask) & ~mask
    labels, n = ndimage.label(holes)
    areas = ndimage.sum(holes, labels, range(1, n + 1)) * res * res
    too_big = [round(a, 2) for a in areas if a > max_area_m2]
    if too_big:
        raise SystemExit(f"enclosed regions larger than {max_area_m2} m2: {too_big}")
    return mask | holes, n


def merge_rects(mask):
    """(row0, col0, nrows, ncols) rectangles covering ``mask`` exactly."""
    open_runs = {}  # (c0, c1) -> start row
    rects = []
    for r in range(mask.shape[0] + 1):
        runs = set()
        if r < mask.shape[0]:
            row = np.concatenate(([False], mask[r], [False])).astype(np.int8)
            d = np.diff(row)
            runs = set(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))
        for run in list(open_runs):
            if run not in runs:
                r0 = open_runs.pop(run)
                rects.append((r0, run[0], r - r0, run[1] - run[0]))
        for run in runs:
            open_runs.setdefault(run, r)
    return rects


def cube(name, center, scale, color):
    return (f' def Cube "{name}" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {{\n'
            f'  double size = 1\n'
            f'  bool physics:collisionEnabled = true\n'
            f'  double3 xformOp:translate = ({center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f})\n'
            f'  double3 xformOp:scale = ({scale[0]:.4f}, {scale[1]:.4f}, {scale[2]:.4f})\n'
            f'  uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n'
            f'  color3f[] primvars:displayColor = [({color[0]}, {color[1]}, {color[2]})]\n'
            f' }}\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", type=Path, default=DEFAULT_MAP)
    ap.add_argument("--height", type=float, default=0.6)
    ap.add_argument("--out", type=Path, default=HERE / "payload")
    ap.add_argument("--no-fill", action="store_true",
                    help="keep enclosed regions (furniture outlines) hollow")
    ap.add_argument("--max-fill-area", type=float, default=4.0,
                    help="largest enclosed region (m2) that may be filled")
    args = ap.parse_args()

    mask, meta = occupied_mask(args.map)
    res = meta["resolution"]
    occupied = int(mask.sum())
    filled_regions = 0
    if not args.no_fill:
        mask, filled_regions = fill_enclosed(mask, res, args.max_fill_area)
    ox, oy = meta["origin"][:2]
    h_rows = mask.shape[0]
    rects = merge_rects(mask)
    assert sum(n * m for _, _, n, m in rects) == int(mask.sum())

    body = []
    for i, (r0, c0, nr, nc) in enumerate(sorted(rects)):
        # image row 0 is the map's top (max y)
        cx = ox + (c0 + nc / 2) * res
        cy = oy + (h_rows - r0 - nr / 2) * res
        body.append(cube(f"occ_{i:04d}", (cx, cy, args.height / 2),
                         (nc * res, nr * res, args.height), (0.45, 0.55, 0.62)))

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "arena.usd").write_text(
        '#usda 1.0\n(defaultPrim = "Arena"\n metersPerUnit = 1\n upAxis = "Z")\n'
        'def Xform "Arena" {\n' + "".join(body) + "}\n")
    shutil.copy2(args.map.parent / meta["image"], args.out / meta["image"])
    shutil.copy2(args.map, args.out / "map.yaml")
    (args.out / "manifest.json").write_text(json.dumps({
        "id": HERE.name,
        "source": f"Occupied cells of {args.map.name} extruded to {args.height} m"
                  + ("" if args.no_fill else ", enclosed furniture outlines filled"),
        "purpose": "Navigation simulation evaluation",
        "occupied_cells": occupied,
        "filled_regions": filled_regions,
        "solid_cells": int(mask.sum()),
        "boxes": len(rects),
        "height_m": args.height,
    }) + "\n")
    (HERE / "current.json").write_text(json.dumps(
        {"manifest": f"artifacts/arena/{HERE.name}/payload/manifest.json"}) + "\n")
    print(f"{occupied} occupied cells + {filled_regions} filled regions = {int(mask.sum())} "
          f"solid cells -> {len(rects)} boxes, height {args.height} m")


if __name__ == "__main__":
    main()
