"""Filled shapes' outline rings (spec 2026-10-20-shape-outline-height-design.md)."""
import json
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from map_check.arena_build import (OUTLINE_RULE, Box, BuildSettings, ShapeSetting, arena_usda,
                                   build_boxes, default_settings)
from map_check.arena_store import ArenaStore
from map_check.config import Settings
from map_check.occupancy import OccupancyMap, closed_outlines, load_map_dir, parse_map
from map_check.rings import Rings, classify_rings

GOLDEN = Path(__file__).parent / "fixtures" / "outline_golden.json"
CASES = json.loads(GOLDEN.read_text())
MEASURED = Path("reports/measured-nav-20261015/map/robocup_home_2026_measured_5cm.yaml")


def case_inputs(case, fill=None):
    """(map, labels, settings) for a fixture case; fill overrides every shape's fill flag."""
    cells = np.array(case["cells"], dtype=np.int32)
    m = OccupancyMap(occupied=cells == 255, resolution=1.0, origin=(0.0, 0.0), meta={},
                     image_name="", yaml_bytes=b"", image_bytes=b"")
    labels = np.where(cells == 255, 0, cells).astype(np.int32)
    shapes = {int(k): ShapeSetting(fill=s["fill"] if fill is None else fill, height=s["height"])
              for k, s in case["shapes"].items()}
    return m, labels, BuildSettings(wall_height=case["wall_height"], shapes=shapes,
                                    erased=frozenset(case["erased"]))


def erased_occ(m, settings):
    occ = m.occupied.copy()
    occ.reshape(-1)[sorted(settings.erased)] = False
    return occ


def raster(boxes, m):
    """Per-cell built height (None = no box) and kind ("wall", "shape:<id>", "")."""
    h, w = m.occupied.shape
    heights = [[None] * w for _ in range(h)]
    kinds = [[""] * w for _ in range(h)]
    for b in boxes:
        c0 = round((b.x0 - m.origin[0]) / m.resolution)
        c1 = round((b.x1 - m.origin[0]) / m.resolution)
        r0 = round(h - (b.y1 - m.origin[1]) / m.resolution)
        r1 = round(h - (b.y0 - m.origin[1]) / m.resolution)
        assert b.z0 == 0.0
        for r in range(r0, r1):
            for c in range(c0, c1):
                assert heights[r][c] is None, f"cell {(r, c)} covered twice"
                heights[r][c] = b.z1
                kinds[r][c] = "wall" if b.kind == "wall" else f"shape:{b.shape_id}"
    return heights, kinds


def ids(cases):
    return [c["name"] for c in cases]


@pytest.mark.parametrize("case", CASES, ids=ids(CASES))
def test_fixture_is_consistent(case):
    """Transcription check: heights/kinds expand the hand sketch; ids match closed_outlines."""
    legend = case["legend"]
    assert case["heights"] == [[legend[ch][0] for ch in row] for row in case["sketch"]]
    assert case["kinds"] == [[legend[ch][1] for ch in row] for row in case["sketch"]]
    m, labels, _ = case_inputs(case)
    got, outlines = closed_outlines(m)
    assert (got == labels).all()
    assert sorted(str(o.id) for o in outlines) == sorted(case["shapes"])
    h, w = m.occupied.shape
    assert h <= 10 and w <= 16


@pytest.mark.parametrize("case", CASES, ids=ids(CASES))
def test_golden_cases(case):
    m, labels, s = case_inputs(case)
    heights, kinds = raster(build_boxes(m, labels, s), m)
    assert kinds == case["kinds"]
    assert heights == case["heights"]


@pytest.mark.parametrize("case", CASES, ids=ids(CASES))
def test_classify_matches_sketch(case):
    """Strip cells are the sketch's S/R; free edges are occupied cells built as a shape."""
    m, labels, s = case_inputs(case)
    occ = erased_occ(m, s)
    heights = {k: v.height for k, v in s.shapes.items() if v.fill}
    rings = classify_rings(occ, labels, heights)
    assert isinstance(rings, Rings)
    assert rings.owner.dtype == np.int32 and rings.strip.dtype == bool
    sketch = np.array([list(r) for r in case["sketch"]])
    kinds = np.array(case["kinds"])
    assert (rings.strip == np.isin(sketch, ["S", "R"])).all()
    free = (rings.owner > 0) & ~rings.strip
    want_free = occ & np.char.startswith(kinds, "shape:")
    assert (free == want_free).all()
    for r, c in zip(*np.nonzero(free)):
        assert kinds[r, c] == f"shape:{rings.owner[r, c]}"
    assert not (rings.strip & ~(rings.owner > 0)).any()


@pytest.mark.parametrize("case", CASES, ids=ids(CASES))
def test_wall_line_never_notched(case):
    """Every original wall cell that is not a free edge keeps a box at >= the wall height."""
    m, labels, s = case_inputs(case)
    occ = erased_occ(m, s)
    heights = {k: v.height for k, v in s.shapes.items() if v.fill}
    rings = classify_rings(occ, labels, heights)
    free = (rings.owner > 0) & ~rings.strip
    built, kinds = raster(build_boxes(m, labels, s), m)
    for r, c in zip(*np.nonzero(occ & ~free)):
        assert built[r][c] is not None and built[r][c] >= s.wall_height
        assert kinds[r][c] == "wall"


@pytest.mark.parametrize("case", CASES, ids=ids(CASES))
def test_unfill_restores_wall(case):
    """With every shape unfilled, the build is today's: occupied cells are wall, nothing else."""
    m, labels, s = case_inputs(case, fill=False)
    occ = erased_occ(m, s)
    built, kinds = raster(build_boxes(m, labels, s), m)
    h, w = occ.shape
    assert built == [[s.wall_height if occ[r, c] else None for c in range(w)] for r in range(h)]
    assert kinds == [["wall" if occ[r, c] else "" for c in range(w)] for r in range(h)]


def test_unfill_one_shape_returns_its_ring_to_wall():
    """C with its fill unticked builds exactly H."""
    by_name = {c["name"]: c for c in CASES}
    m, labels, s = case_inputs(by_name["C_wall_1cell"])
    s = replace(s, shapes={1: replace(s.shapes[1], fill=False)})
    heights, kinds = raster(build_boxes(m, labels, s), m)
    assert heights == by_name["H_unfilled"]["heights"]
    assert kinds == by_name["H_unfilled"]["kinds"]


def test_box_group_order():
    """Walls at wall height, raised strips by height, then shapes by id; each group in rect order."""
    case = next(c for c in CASES if c["name"] == "D2_pair_on_wall")
    m, labels, s = case_inputs(case)
    s = replace(s, shapes={**s.shapes, 3: ShapeSetting(fill=True, height=0.9)})  # no interior
    boxes = build_boxes(m, labels, s)

    def group(b):
        if b.kind == "shape":
            return (2, b.shape_id)
        return (0, 0) if b.z1 == s.wall_height else (1, b.z1)

    keys = [group(b) for b in boxes]
    assert keys == sorted(keys)
    assert [k for i, k in enumerate(keys) if k not in keys[:i]] == [(0, 0), (1, 1.5), (2, 1), (2, 2)]
    for k in set(keys):
        rects = [(-b.y1, b.x0, b.y1 - b.y0, b.x1 - b.x0) for b in boxes if group(b) == k]
        assert rects == sorted(rects)


def _reference(occ, labels, heights):
    """The prototype's per-cell loop (outline-proto.py), plus the spec's ownership rule."""
    from scipy import ndimage
    ring = np.zeros_like(occ)
    owner = np.zeros(occ.shape, np.int32)
    for k in sorted(heights, key=lambda k: (-heights[k], k)):
        rk = ndimage.binary_dilation(labels == k, structure=np.ones((3, 3), bool)) & occ
        owner[rk & ~ring] = k
        ring |= rk
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
    return owner, strip


def test_matches_reference_loop_on_random_grids():
    rng = np.random.default_rng(20261020)
    checked = 0
    for _ in range(300):
        h, w = rng.integers(4, 24, size=2)
        occ = rng.random((h, w)) < rng.uniform(0.3, 0.75)
        m = OccupancyMap(occupied=occ, resolution=1.0, origin=(0.0, 0.0), meta={},
                         image_name="", yaml_bytes=b"", image_bytes=b"")
        labels, outlines = closed_outlines(m)
        heights = {o.id: float(rng.choice([0.3, 0.6, 0.9])) for o in outlines if rng.random() < 0.7}
        work = occ & (rng.random((h, w)) > 0.1)  # some cells erased
        got = classify_rings(work, labels, heights)
        owner, strip = _reference(work, labels, heights)
        assert (got.owner == owner).all() and (got.strip == strip).all()
        checked += bool(heights)
    assert checked > 100


@pytest.fixture
def measured(real_root):
    path = real_root / MEASURED
    if not path.is_file():
        pytest.skip(f"measured map absent: {path}")
    m = load_map_dir(path)
    labels, outlines = closed_outlines(m)
    return m, labels, outlines


def test_measured_map_defaults_unchanged(measured):
    """Case K: with defaults (16 shapes, all filled at 0.60) every cell's height is as before."""
    m, labels, outlines = measured
    s = default_settings(m, outlines)
    assert len(s.shapes) == 16 and all(v.fill and v.height == 0.6 for v in s.shapes.values())
    boxes = build_boxes(m, labels, s)
    h, w = m.occupied.shape
    got = np.full((h, w), np.nan)
    for b in boxes:
        c0, c1 = round((b.x0 - m.origin[0]) / m.resolution), round((b.x1 - m.origin[0]) / m.resolution)
        r0 = round(h - (b.y1 - m.origin[1]) / m.resolution)
        r1 = round(h - (b.y0 - m.origin[1]) / m.resolution)
        assert np.isnan(got[r0:r1, c0:c1]).all()
        got[r0:r1, c0:c1] = b.z1
    old = np.full((h, w), np.nan)
    old[m.occupied | (labels > 0)] = 0.6  # the previous rule: walls and filled interiors
    assert np.array_equal(got, old, equal_nan=True)


def test_measured_map_strip_counts(measured):
    m, labels, outlines = measured
    heights = {o.id: 0.6 for o in outlines}
    rings = classify_rings(m.occupied, labels, heights)
    counts = {k: int((rings.strip & (rings.owner == k)).sum()) for k in heights}
    assert {k: n for k, n in counts.items() if n} == {5: 16, 6: 40, 7: 20, 8: 16, 14: 9}
    for k in (3, 4, 9, 10, 11, 13):  # free-standing: whole ring is free edge
        assert (rings.owner == k).any() and counts[k] == 0


def _best_of_3(fn):
    best = float("inf")
    for _ in range(3):
        t = time.perf_counter()
        fn()
        best = min(best, time.perf_counter() - t)
    return best


def test_measured_map_speed(measured):
    """Loose limits (measured ~11 ms / ~21 ms): the CPU is shared with Isaac runs."""
    m, labels, outlines = measured
    heights = {o.id: 0.6 for o in outlines}
    s = default_settings(m, outlines)
    assert _best_of_3(lambda: classify_rings(m.occupied, labels, heights)) < 2.0
    assert _best_of_3(lambda: build_boxes(m, labels, s)) < 3.0


def _png():
    import io

    from PIL import Image
    img = Image.new("L", (8, 8), 254)
    for i in range(8):
        img.putpixel((i, 0), 0)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


def _store(tmp_path):
    s = replace(Settings(), maps_dir=tmp_path / "maps", repo_root=tmp_path,
                arena_root=tmp_path / "artifacts" / "arena")
    s.maps_dir.mkdir()
    return ArenaStore(s)


def _small_map():
    yaml_bytes = (b"image: m.png\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
                  b"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return parse_map(yaml_bytes, {"m.png": _png()})


def test_save_writes_outline_rule(tmp_path):
    store = _store(tmp_path)
    used = BuildSettings(wall_height=0.6, shapes={1: ShapeSetting(True, 0.5)}, erased=frozenset())
    v = store.save_arena("ring", used, [Box("wall", None, 0, 1, 0, 1, 0, 0.6)], _small_map())
    assert OUTLINE_RULE == "shape-height-v1"
    assert v.manifest["settings"]["outline_rule"] == OUTLINE_RULE
    man, _, _ = store.load_current("ring")
    assert man["settings"]["outline_rule"] == OUTLINE_RULE


def test_load_current_without_outline_rule(tmp_path):
    """An arena saved before the rule (no settings.outline_rule) still loads and renders."""
    store = _store(tmp_path)
    vdir = store.settings.arena_root / "old" / "20261019T000000Z-deadbeef"
    vdir.mkdir(parents=True)
    raw = [{"kind": "wall", "shape_id": None, "x0": 0.0, "x1": 1.0, "y0": 0.0, "y1": 0.05,
            "z0": 0.0, "z1": 0.6},
           {"kind": "shape", "shape_id": 1, "x0": 0.1, "x1": 0.3, "y0": 0.1, "y1": 0.3,
            "z0": 0.0, "z1": 0.45}]
    (vdir / "boxes.json").write_text(json.dumps(raw))
    (vdir / "manifest.json").write_text(json.dumps({
        "kind": "arena", "built_by": "map_check", "id": "old",
        "source_map": {"name": "old", "sha256": "0" * 64},
        "settings": {"wall_height": 0.6, "shapes": {"1": {"fill": True, "height": 0.45}},
                     "erased": []},
        "boxes": 2, "created_utc": "2026-10-19T00:00:00Z"}))
    (vdir.parent / "current.json").write_text(json.dumps(
        {"manifest": f"artifacts/arena/old/{vdir.name}/manifest.json", "schema_version": 1}))
    assert store.arenas() == ["old"]
    man, boxes, got_dir = store.load_current("old")
    assert got_dir == vdir and "outline_rule" not in man["settings"]
    assert boxes == [Box(**r) for r in raw]
    usda = arena_usda(boxes)
    assert usda.count('def Cube "') == 2 and '"shape1_000"' in usda
