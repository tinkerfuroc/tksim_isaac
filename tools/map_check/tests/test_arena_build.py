import base64
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from map_check.arena_build import (arena_usda, build_boxes, default_settings, merge_rects,
                                   web_grid)
from map_check.occupancy import closed_outlines, load_map_dir

MAP = Path("/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/maps/"
           "robocup_home_2026_measured_5cm.yaml")
GOLDEN = Path(__file__).parent / "fixtures" / "rects_golden.json"


@pytest.fixture(scope="module")
def real():
    if not MAP.is_file():
        pytest.skip("measured map absent")
    m = load_map_dir(MAP)
    labels, outlines = closed_outlines(m)
    return m, labels, outlines


def cover(boxes, m):
    h, w = m.occupied.shape
    cnt = np.zeros((h, w), int)
    for b in boxes:
        c0 = round((b.x0 - m.origin[0]) / m.resolution)
        c1 = round((b.x1 - m.origin[0]) / m.resolution)
        r0 = round(h - (b.y1 - m.origin[1]) / m.resolution)
        r1 = round(h - (b.y0 - m.origin[1]) / m.resolution)
        cnt[r0:r1, c0:c1] += 1
    return cnt


def test_merge_rects_exact_cover():
    rng = np.random.default_rng(1)
    for _ in range(50):
        mask = rng.random((30, 40)) < 0.3
        cnt = np.zeros(mask.shape, int)
        for r0, c0, nr, nc in merge_rects(mask):
            cnt[r0:r0 + nr, c0:c0 + nc] += 1
        assert (cnt == mask.astype(int)).all()


def test_build_boxes_partition(real):
    m, labels, outlines = real
    s = default_settings(m, outlines)
    boxes = build_boxes(m, labels, s)
    walls = [b for b in boxes if b.kind == "wall"]
    assert (cover(walls, m) == m.occupied.astype(int)).all()
    assert all(b.z0 == 0 and b.z1 == s.wall_height for b in walls)
    for k, st in s.shapes.items():
        sb = [b for b in boxes if b.shape_id == k]
        if st.fill:
            assert (cover(sb, m) == (labels == k).astype(int)).all()
            assert all(b.z1 == st.height for b in sb)
        else:
            assert not sb


def test_unfilled_shape_absent(real):
    m, labels, outlines = real
    s = default_settings(m, outlines)
    k = outlines[0].id
    s = replace(s, shapes={**s.shapes, k: replace(s.shapes[k], fill=False)})
    assert not [b for b in build_boxes(m, labels, s) if b.shape_id == k]


def test_erased_excluded(real):
    m, labels, outlines = real
    r, c = map(int, np.argwhere(m.occupied)[0])
    s = replace(default_settings(m, outlines), erased=frozenset({r * m.occupied.shape[1] + c}))
    walls = [b for b in build_boxes(m, labels, s) if b.kind == "wall"]
    want = m.occupied.astype(int)
    want[r, c] = 0
    assert (cover(walls, m) == want).all()


def test_usda_loads(real, tmp_path):
    from pxr import Usd, UsdGeom
    m, labels, outlines = real
    boxes = build_boxes(m, labels, default_settings(m, outlines))
    p = tmp_path / "arena.usda"
    p.write_text(arena_usda(boxes))
    stage = Usd.Stage.Open(str(p))
    assert stage.GetMetadata("upAxis") == "Z"
    assert len([x for x in stage.Traverse() if x.IsA(UsdGeom.Cube)]) == len(boxes)
    rng = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(
        stage.GetDefaultPrim()).ComputeAlignedRange()
    lo, hi = rng.GetMin(), rng.GetMax()
    got = [lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]]
    want = [min(b.x0 for b in boxes), max(b.x1 for b in boxes), min(b.y0 for b in boxes),
            max(b.y1 for b in boxes), 0.0, max(b.z1 for b in boxes)]
    assert got == pytest.approx(want, abs=1e-3)


def test_web_grid(real):
    m, labels, outlines = real
    g = web_grid(m, labels)
    grid = np.frombuffer(base64.b64decode(g["grid"]), np.uint8).reshape(g["H"], g["W"])
    assert (grid == 255).sum() == m.occupied.sum()
    sel = (labels > 0) & ~m.occupied
    assert sel.any() and (grid[sel] == labels[sel]).all()
    assert set(g) == {"W", "H", "res", "ox", "oy", "grid"}


def test_web_grid_too_many_outlines(real):
    m, labels, _ = real
    bad = labels.copy()
    bad[0, 0] = 255
    with pytest.raises(ValueError):
        web_grid(m, bad)


def test_write_golden():
    """Task 13's vitest reads fixtures/rects_golden.json (rects of seeded masks)."""
    rng = np.random.default_rng(2026)
    cases = []
    for _ in range(5):
        mask = rng.random((12, 16)) < 0.45
        cases.append({"mask": mask.astype(int).tolist(),
                      "rects": [list(r) for r in merge_rects(mask)]})
    GOLDEN.parent.mkdir(exist_ok=True)
    GOLDEN.write_text(json.dumps(cases))
    back = json.loads(GOLDEN.read_text())
    assert back == cases
    for c in back:
        assert [list(r) for r in merge_rects(np.array(c["mask"], bool))] == c["rects"]
