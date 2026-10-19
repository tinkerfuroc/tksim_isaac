"""Arena geometry: occupied cells and filled outlines as boxes, USDA text, web grid."""
import base64
from dataclasses import dataclass
from typing import Literal

import numpy as np

from map_check.occupancy import OccupancyMap

# mockup tokens --wall #7f9cc0, --shape #d2a865 as RGB floats
WALL_COLOR = (0.498, 0.612, 0.753)
SHAPE_COLOR = (0.824, 0.659, 0.396)


@dataclass(frozen=True)
class ShapeSetting:
    fill: bool
    height: float


@dataclass(frozen=True)
class BuildSettings:
    wall_height: float
    shapes: dict
    erased: frozenset  # flat cell indices r*W+c, occupied cells only


@dataclass(frozen=True)
class Box:
    kind: Literal["wall", "shape"]
    shape_id: int | None
    x0: float
    x1: float
    y0: float
    y1: float
    z0: float
    z1: float


def default_settings(m, outlines, wall_height=0.6, shape_height=0.6, fill_area=4.0):
    shapes = {o.id: ShapeSetting(fill=o.area_m2 <= fill_area, height=shape_height)
              for o in outlines}
    return BuildSettings(wall_height=wall_height, shapes=shapes, erased=frozenset())


def merge_rects(mask):
    """(row0, col0, nrows, ncols) rectangles covering ``mask`` exactly.

    Row runs (maximal horizontal spans of True) are merged while the identical
    column span repeats on consecutive rows. The result is sorted ascending by
    the tuple (row0, col0, nrows, ncols), so output order is deterministic and
    a port can reproduce it. Elements are plain Python ints.
    """
    open_runs = {}  # (c0, c1) -> start row
    rects = []
    for r in range(mask.shape[0] + 1):
        runs = set()
        if r < mask.shape[0]:
            row = np.concatenate(([False], mask[r], [False])).astype(np.int8)
            d = np.diff(row)
            runs = set(zip(np.flatnonzero(d == 1).tolist(), np.flatnonzero(d == -1).tolist()))
        for run in list(open_runs):
            if run not in runs:
                r0 = open_runs.pop(run)
                rects.append((r0, run[0], r - r0, run[1] - run[0]))
        for run in runs:
            open_runs.setdefault(run, r)
    return sorted(rects)


def _boxes(m, mask, kind, shape_id, height):
    h = mask.shape[0]
    res = m.resolution
    ox, oy = m.origin
    out = []
    for r0, c0, nr, nc in merge_rects(mask):
        out.append(Box(kind, shape_id, ox + c0 * res, ox + (c0 + nc) * res,
                       oy + (h - r0 - nr) * res, oy + (h - r0) * res, 0.0, height))
    return out


def build_boxes(m: OccupancyMap, labels, settings: BuildSettings):
    walls = m.occupied.copy()
    if settings.erased:
        flat = walls.reshape(-1)
        idx = np.fromiter(settings.erased, dtype=np.int64)
        flat[idx] = False
    boxes = _boxes(m, walls, "wall", None, settings.wall_height)
    for k in sorted(settings.shapes):
        s = settings.shapes[k]
        if s.fill:
            boxes += _boxes(m, labels == k, "shape", k, s.height)
    return boxes


def _cube(name, b, color):
    cx, cy, cz = (b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2, (b.z0 + b.z1) / 2
    return (f' def Cube "{name}" (prepend apiSchemas = ["PhysicsCollisionAPI"]) {{\n'
            f'  double size = 1\n'
            f'  bool physics:collisionEnabled = true\n'
            f'  double3 xformOp:translate = ({cx:.4f}, {cy:.4f}, {cz:.4f})\n'
            f'  double3 xformOp:scale = ({b.x1 - b.x0:.4f}, {b.y1 - b.y0:.4f}, {b.z1 - b.z0:.4f})\n'
            f'  uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n'
            f'  color3f[] primvars:displayColor = [({color[0]}, {color[1]}, {color[2]})]\n'
            f' }}\n')


def arena_usda(boxes) -> str:
    counts = {}
    body = []
    for b in boxes:
        key = "wall" if b.kind == "wall" else f"shape{b.shape_id}"
        i = counts.get(key, 0)
        counts[key] = i + 1
        body.append(_cube(f"{key}_{i:03d}", b, WALL_COLOR if b.kind == "wall" else SHAPE_COLOR))
    return ('#usda 1.0\n(defaultPrim = "Arena"\n metersPerUnit = 1\n upAxis = "Z")\n'
            'def Xform "Arena" {\n' + "".join(body) + "}\n")


def web_grid(m: OccupancyMap, labels) -> dict:
    if labels.size and int(labels.max()) > 254:
        raise ValueError("outline ids above 254 cannot be encoded in the web grid")
    grid = labels.astype(np.uint8)
    grid[m.occupied] = 255
    h, w = m.occupied.shape
    return {"W": w, "H": h, "res": m.resolution, "ox": m.origin[0], "oy": m.origin[1],
            "grid": base64.b64encode(grid.tobytes()).decode("ascii")}
