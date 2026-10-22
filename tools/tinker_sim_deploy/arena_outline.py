"""Hull-vs-map outline check for the RoboCup arena import.

Each furniture instance's convex-hull colliders, seen from above inside the
2-D scan's height band, must match the declared boxes the nav map is
rasterized from: no bigger and no smaller, within ``TOLERANCE_MM``. Both
shapes are rasterized on one 1 mm grid and compared by Euclidean distance
transform. numpy/scipy/trimesh/Pillow are imported lazily so that
``arena_import`` can import the instance-name helper under plain Python.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Mapping, Sequence

from .arena_map import instance_footprints
from .arena_world import ArenaLayout, BoxCollider, FurniturePlacement

if TYPE_CHECKING:
    import numpy as np

    from .arena_coacd import ConvexPiece

TOLERANCE_MM = 5.0
RESOLUTION_M = 0.001
_MARGIN_M = 0.010


def _furniture_instance_suffixes(furniture: Sequence[FurniturePlacement]) -> dict[int, str]:
    """Map each placement's ``id()`` to the disambiguating suffix
    ``arena_convert.compose_arena`` would give its furniture prim: computed
    the same way (iterate placements in order, first-seen bare, later
    instances of the same model_id get an incrementing ``_NN`` suffix) so
    ``placement.json`` surface_ids always name the same instance as the
    corresponding ``arena.usd`` prim. ``id()`` is safe here because every
    placement is a distinct object built once during ``parse_world``.
    """
    counts: dict[str, int] = {}
    suffixes: dict[int, str] = {}
    for placement in furniture:
        counts[placement.model_id] = counts.get(placement.model_id, 0) + 1
        count = counts[placement.model_id]
        suffixes[id(placement)] = "" if count == 1 else f"_{count:02d}"
    return suffixes


def band_polygons(pieces: Sequence[ConvexPiece], item: FurniturePlacement,
                  band: tuple[float, float]) -> list[np.ndarray]:
    """Each piece placed in the world, clipped to the band, as its 2-D convex
    hull outline ((K, 2), counter-clockwise). Pieces that leave fewer than 3
    non-collinear points inside the band are dropped."""
    import numpy as np
    import trimesh
    from scipy.spatial import ConvexHull, QhullError

    low, high = band
    cos_y, sin_y = math.cos(item.yaw), math.sin(item.yaw)
    rot = np.array([[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]])
    polygons = []
    for piece in pieces:
        world = np.asarray(piece.vertices, dtype=np.float64) @ rot.T + np.asarray(item.position)
        verts, faces = world, np.asarray(piece.faces)
        # slice_faces_plane (not Trimesh.slice_plane): the latter imports shapely.
        for origin_z, normal_z in ((low, 1.0), (high, -1.0)):
            if len(faces) == 0:
                break
            verts, faces = trimesh.intersections.slice_faces_plane(
                verts, faces, [0.0, 0.0, normal_z], [0.0, 0.0, origin_z])[:2]
        if len(faces) == 0 or len(verts) < 3:
            continue
        xy = np.asarray(verts)[:, :2]
        try:
            hull = ConvexHull(xy)
        except QhullError:
            continue
        polygons.append(xy[hull.vertices])
    return polygons


@dataclass(frozen=True)
class OutlineDiff:
    bigger_mm: float     # max distance of a hull-only cell from the map shape
    smaller_mm: float    # max distance of a map-only cell from the hull shape
    origin_xy: tuple[float, float]
    hull_mask: np.ndarray   # bool, rows = +y up, 1 mm cells
    map_mask: np.ndarray


def _rect_corners(rect: tuple[float, float, float, float, float]) -> np.ndarray:
    import numpy as np

    cx, cy, hx, hy, yaw = rect
    c, s = math.cos(yaw), math.sin(yaw)
    local = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
    return local @ np.array([[c, s], [-s, c]]) + np.array([cx, cy])


def _fill_convex(mask: np.ndarray, poly: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> None:
    """OR into ``mask`` the cells whose centre passes every edge of the
    counter-clockwise convex polygon ``poly``."""
    import numpy as np

    inside = np.ones(mask.shape, dtype=bool)
    for i in range(len(poly)):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % len(poly)]
        inside &= (x1 - x0) * (ys[:, None] - y0) - (y1 - y0) * (xs[None, :] - x0) >= 0.0
    mask |= inside


def _max_distance(only: np.ndarray, other: np.ndarray, resolution: float) -> float:
    """Largest distance (mm) from a cell of ``only & ~other`` to ``other``."""
    from scipy.ndimage import distance_transform_edt

    cells = only & ~other
    if not cells.any():
        return 0.0
    if not other.any():
        return math.inf  # one shape missing entirely: always a failure
    dist = distance_transform_edt(~other)
    return float(dist[cells].max()) * 1000.0 * resolution


def compare_outlines(hull_polygons: Sequence[np.ndarray],
                     map_rects: Sequence[tuple[float, float, float, float, float]],
                     resolution: float = RESOLUTION_M) -> OutlineDiff:
    import numpy as np

    rect_polys = [_rect_corners(rect) for rect in map_rects]
    points = [np.asarray(p, dtype=np.float64) for p in list(hull_polygons) + rect_polys]
    if points:
        allp = np.concatenate(points)
        lo, hi = allp.min(axis=0) - _MARGIN_M, allp.max(axis=0) + _MARGIN_M
    else:
        lo = hi = np.zeros(2)
    ox, oy = math.floor(lo[0] / resolution) * resolution, math.floor(lo[1] / resolution) * resolution
    width = int(math.ceil((hi[0] - ox) / resolution)) + 1
    height = int(math.ceil((hi[1] - oy) / resolution)) + 1
    xs = ox + (np.arange(width) + 0.5) * resolution
    ys = oy + (np.arange(height) + 0.5) * resolution
    hull_mask = np.zeros((height, width), dtype=bool)
    map_mask = np.zeros((height, width), dtype=bool)
    for poly in hull_polygons:
        _fill_convex(hull_mask, np.asarray(poly, dtype=np.float64), xs, ys)
    for poly in rect_polys:
        _fill_convex(map_mask, poly, xs, ys)
    return OutlineDiff(
        bigger_mm=_max_distance(hull_mask, map_mask, resolution),
        smaller_mm=_max_distance(map_mask, hull_mask, resolution),
        origin_xy=(float(ox), float(oy)),
        hull_mask=hull_mask,
        map_mask=map_mask,
    )


@dataclass(frozen=True)
class InstanceOutline:
    instance: str        # model_id + compose_arena suffix, e.g. "rcw26_shelf_02"
    model_id: str
    bigger_mm: float
    smaller_mm: float
    passed: bool         # both <= TOLERANCE_MM
    diff: OutlineDiff


def check_arena(layout: ArenaLayout,
                map_colliders: Mapping[str, tuple[BoxCollider, ...]],
                pieces: Mapping[str, Sequence[ConvexPiece]],
                band: tuple[float, float]) -> list[InstanceOutline]:
    suffixes = _furniture_instance_suffixes(layout.furniture)
    results = []
    for item in layout.furniture:
        rects = instance_footprints(item, map_colliders.get(item.model_id, ()), band)
        polygons = band_polygons(pieces.get(item.model_id, ()), item, band)
        diff = compare_outlines(polygons, rects)
        results.append(InstanceOutline(
            instance=item.model_id + suffixes[id(item)],
            model_id=item.model_id,
            bigger_mm=diff.bigger_mm,
            smaller_mm=diff.smaller_mm,
            passed=diff.bigger_mm <= TOLERANCE_MM and diff.smaller_mm <= TOLERANCE_MM,
            diff=diff,
        ))
    return results


_GREY, _RED, _BLUE, _WHITE = (200, 200, 200), (220, 40, 40), (40, 80, 220), (255, 255, 255)
_MAX_PX = 1000
_HEADER_PX = 16


def _title_distances(result: InstanceOutline) -> str:
    if math.isinf(result.smaller_mm):
        return "no hull in band"
    if math.isinf(result.bigger_mm):
        return "no map footprint"
    return f"bigger {result.bigger_mm:.1f} mm  smaller {result.smaller_mm:.1f} mm"


def _json_mm(value: float) -> float | None:
    return None if math.isinf(value) else round(value, 1)


def _write_png(result: InstanceOutline, path: Path) -> None:
    import numpy as np
    from PIL import Image, ImageDraw

    hull, mapped = result.diff.hull_mask, result.diff.map_mask
    rgb = np.empty(hull.shape + (3,), dtype=np.uint8)
    rgb[:] = _WHITE
    rgb[hull & mapped] = _GREY
    rgb[hull & ~mapped] = _RED
    rgb[mapped & ~hull] = _BLUE
    image = Image.fromarray(rgb[::-1])  # rows were +y up
    longest = max(image.size)
    if longest > _MAX_PX:
        scale = _MAX_PX / longest
        image = image.resize((max(1, round(image.width * scale)),
                              max(1, round(image.height * scale))), Image.NEAREST)
    canvas = Image.new("RGB", (max(image.width, 360), image.height + _HEADER_PX), _WHITE)
    canvas.paste(image, (0, _HEADER_PX))
    ImageDraw.Draw(canvas).text(
        (2, 2),
        f"{result.instance}  {_title_distances(result)}  {'PASS' if result.passed else 'FAIL'}",
        fill=(0, 0, 0))
    canvas.save(path)


def write_outline_report(results: Sequence[InstanceOutline], out_dir: Path) -> Path:
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for result in results:
        _write_png(result, out_dir / f"{result.instance}.png")
    summary = {
        "tolerance_mm": TOLERANCE_MM,
        "passed": all(r.passed for r in results),
        "instances": [
            {"instance": r.instance, "model_id": r.model_id,
             "bigger_mm": _json_mm(r.bigger_mm), "smaller_mm": _json_mm(r.smaller_mm),
             "passed": r.passed}
            for r in results
        ],
    }
    path = out_dir / "summary.json"
    path.write_text(json.dumps(summary, indent=2) + "\n")
    return path
