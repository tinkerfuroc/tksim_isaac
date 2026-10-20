"""A/B sanity run of the shape-outline-height rule on real maps. Reads only; writes nothing.

For every map in the Map Studio maps folder, with default settings, it checks that each cell outside
the filled shapes' rings has the same built height under the old rule (every occupied cell at the
wall height, filled interiors at their shape height) and the new one. It also checks the vectorised
classifier against the prototypes' per-cell loops (the strip walk, then the corner extension of rule
step 3b), and prints free-edge and strip counts, the cells step 3b moves from free edge to strip,
and timings.
A second pass sets every filled shape to 0.45 m and then 1.50 m (both unlike the 0.60 m wall) and
checks the outside-ring invariant alone, printing how many ring cells changed height.

Usage (from the repo root):
  env -u PYTHONPATH -u AMENT_PREFIX_PATH uv run --project tools/map_check \
      python tools/map_check/scripts/outline_ab.py [MAPS_DIR]
"""
import sys
import time
from pathlib import Path

import numpy as np
from scipy import ndimage

from map_check.arena_build import BuildSettings, ShapeSetting, build_boxes, default_settings
from map_check.occupancy import MapFileError, closed_outlines, load_map_dir
from map_check.rings import classify_rings

MAPS = Path("/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/maps")
MEASURED = "robocup_home_2026_measured_5cm"
MEASURED_STRIPS = {5: 16, 6: 40, 7: 20, 8: 16, 14: 9}  # spec, "What the prototype showed"
CORNER_PROTO = (556, 36)  # spec, rule step 3b: cells corner-proto.py moved, over that many maps


def built_heights(m, boxes):
    h, w = m.occupied.shape
    out = np.full((h, w), np.nan)
    for b in boxes:
        c0 = round((b.x0 - m.origin[0]) / m.resolution)
        c1 = round((b.x1 - m.origin[0]) / m.resolution)
        r0 = round(h - (b.y1 - m.origin[1]) / m.resolution)
        r1 = round(h - (b.y0 - m.origin[1]) / m.resolution)
        assert np.isnan(out[r0:r1, c0:c1]).all(), "boxes overlap"
        out[r0:r1, c0:c1] = b.z1
    return out


def old_heights(m, labels, s):
    out = np.full(m.occupied.shape, np.nan)
    out[m.occupied] = s.wall_height
    for k, st in s.shapes.items():
        if st.fill:
            out[labels == k] = st.height
    return out


def reference(occ, labels, filled):
    """outline-proto.py's per-cell strip walk, verbatim in logic: (ring, strip by step 3)."""
    ring = np.zeros_like(occ)
    for k in filled:
        ring |= ndimage.binary_dilation(labels == k, structure=np.ones((3, 3), bool)) & occ
    wall = occ & ~ring
    h, w = occ.shape

    def runs(r, c):
        def run(dr, dc):
            n, rr, cc = 0, r + dr, c + dc
            while 0 <= rr < h and 0 <= cc < w and wall[rr, cc]:
                n += 1
                rr += dr
                cc += dc
            return n
        return 1 + run(0, -1) + run(0, 1), 1 + run(-1, 0) + run(1, 0)

    strip = np.zeros_like(occ)
    for r, c in zip(*np.nonzero(ring)):
        for dr, dc, horiz in ((0, -1, True), (0, 1, True), (-1, 0, False), (1, 0, False)):
            rr, cc = r + dr, c + dc
            while 0 <= rr < h and 0 <= cc < w and ring[rr, cc]:
                rr += dr
                cc += dc
            if 0 <= rr < h and 0 <= cc < w and wall[rr, cc]:
                hr, vr = runs(rr, cc)
                if (horiz and hr > vr) or (not horiz and vr > hr):
                    strip[r, c] = True
                    break
    return ring, strip


def ring_runs(ring, horiz):
    """Maximal straight runs of ring cells, each a list of (r, c)."""
    h, w = ring.shape
    out = []
    for a in range(h if horiz else w):
        cur = []
        for b in range(w if horiz else h):
            r, c = (a, b) if horiz else (b, a)
            if ring[r, c]:
                cur.append((r, c))
            elif cur:
                out.append(cur)
                cur = []
        if cur:
            out.append(cur)
    return out


def corner_extension(occ, ring, strip):
    """corner-proto.py's extend() (rule step 3b), verbatim in logic."""
    h, w = occ.shape

    def o(r, c):
        return 0 <= r < h and 0 <= c < w and occ[r, c]

    new = strip.copy()
    for horiz in (True, False):
        for run in ring_runs(ring, horiz):
            if all(strip[p] for p in run):
                continue
            for r, c in (run[0], run[-1]):
                if not strip[r, c]:
                    continue
                a, b = ((r - 1, c), (r + 1, c)) if horiz else ((r, c - 1), (r, c + 1))
                if not (o(*a) and o(*b)):
                    for p in run:
                        new[p] = True
                    break
    return new


def main(maps_dir: Path) -> int:
    paths = sorted(p for p in maps_dir.glob("*.yaml") if not p.name.endswith(".poses.yaml"))
    print(f"{'map':40} {'size':>9} {'filled':>6} {'free':>6} {'strip':>5} {'3b':>4} "
          f"{'classify ms':>11} {'build ms':>8}  strip cells by shape")
    worst = (0.0, "")
    checked = 0
    moved_total = moved_maps = 0
    for p in paths:
        name = p.name[:-len(".yaml")]
        try:
            m = load_map_dir(p)
        except (MapFileError, OSError) as e:
            print(f"{name:40} skipped: {e}")
            continue
        labels, outlines = closed_outlines(m)
        s = default_settings(m, outlines)
        filled = {k: st.height for k, st in s.shapes.items() if st.fill}
        t = time.perf_counter()
        rings = classify_rings(m.occupied, labels, filled)
        t_cls = time.perf_counter() - t
        t = time.perf_counter()
        boxes = build_boxes(m, labels, s)
        t_build = time.perf_counter() - t
        ring = rings.owner > 0
        free = ring & ~rings.strip
        new, old = built_heights(m, boxes), old_heights(m, labels, s)
        assert np.array_equal(new[~ring], old[~ring], equal_nan=True), f"{name}: outside rings differ"
        # defaults put every shape at the wall height, so the whole map is unchanged (case K)
        assert np.array_equal(new, old, equal_nan=True), f"{name}: defaults changed a ring height"
        ref_ring, ref_strip3 = reference(m.occupied, labels, filled)
        ref_strip = corner_extension(m.occupied, ref_ring, ref_strip3)
        assert (ref_ring == ring).all() and (ref_strip == rings.strip).all(), f"{name}: != prototype"
        moved = int((ref_strip & ~ref_strip3).sum())  # free edge -> strip by step 3b
        moved_total += moved
        moved_maps += bool(moved)
        by_shape = {k: int((rings.strip & (rings.owner == k)).sum()) for k in sorted(filled)}
        by_shape = {k: n for k, n in by_shape.items() if n}
        if name == MEASURED:
            assert by_shape == MEASURED_STRIPS, f"{name}: strip counts {by_shape}"
            assert moved == 0, f"{name}: step 3b moved {moved} cells"
        h, w = m.occupied.shape
        print(f"{name:40} {w:>4}x{h:<4} {len(filled):>6} {int(free.sum()):>6} "
              f"{int(rings.strip.sum()):>5} {moved:>4} {t_cls * 1e3:>11.1f} {t_build * 1e3:>8.1f}  "
              f"{by_shape or ''}")
        worst = max(worst, (t_cls, name))
        checked += 1
    print(f"\n{checked} maps checked, all assertions passed; "
          f"slowest classify {worst[0] * 1e3:.1f} ms ({worst[1]})")
    print(f"step 3b moved {moved_total} cells from free edge to strip over {moved_maps} maps "
          f"(corner-proto.py: {CORNER_PROTO[0]} over {CORNER_PROTO[1]})")
    second_pass(paths)
    return 0


def second_pass(paths, heights=(0.45, 1.5)):
    """Every filled shape at a height unlike the wall's: only cells inside rings may change."""
    print(f"\nsecond pass: filled shapes at {', '.join(f'{h:.2f}' for h in heights)} m; "
          "outside-ring invariant only")
    print(f"{'map':40} " + " ".join(f"{f'ring cells changed @{h:.2f}':>24}" for h in heights))
    checked = 0
    for p in paths:
        name = p.name[:-len(".yaml")]
        try:
            m = load_map_dir(p)
        except (MapFileError, OSError):
            continue
        labels, outlines = closed_outlines(m)
        base = default_settings(m, outlines)
        cols = []
        for h in heights:
            s = BuildSettings(wall_height=base.wall_height, erased=base.erased,
                              shapes={k: ShapeSetting(fill=st.fill, height=h)
                                      for k, st in base.shapes.items()})
            filled = {k: st.height for k, st in s.shapes.items() if st.fill}
            ring = classify_rings(m.occupied, labels, filled).owner > 0
            new, old = built_heights(m, build_boxes(m, labels, s)), old_heights(m, labels, s)
            assert np.array_equal(new[~ring], old[~ring], equal_nan=True), \
                f"{name} @ {h}: outside rings differ"
            cols.append(f"{int((new[ring] != old[ring]).sum()):>6} of {int(ring.sum()):<6}")
        print(f"{name:40} " + " ".join(f"{c:>24}" for c in cols))
        checked += 1
    print(f"\n{checked} maps checked in the second pass, all assertions passed")


if __name__ == "__main__":
    sys.exit(main(Path(sys.argv[1]) if len(sys.argv) > 1 else MAPS))
