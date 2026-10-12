"""Surface-aware placement (fix4 run 3: objects walked off side_table_02).

Rule under test: every spawned object's footprint stays inside the spot's
measured ``surface_bbox_xy`` minus a 4 cm margin, at least PLACEMENT_GAP_M
(8 cm: room for the 0.14 m palm -- fix5's grasps were rejected "palm->env,
knuckle->env" at the old 2 cm) from every footprint already on the surface (base-scenario objects, earlier spawns), and
an object that cannot fit is refused loudly (`not_attempted` with a reason)
instead of being placed off the edge.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.gpsr_scene import (  # noqa: E402
    DEFAULT_FOOTPRINT_XY,
    NAME_TO_YCB_DIR,
    OBJECT_FOOTPRINT_XY,
    PLACEMENT_EDGE_MARGIN_M,
    PLACEMENT_GAP_M,
    ScenePlan,
    SpawnItem,
    SurfaceLayout,
    object_footprint_xy,
    plan_scene,
)
from tools.gpsr_spawn import apply_plan, emit_scenario  # noqa: E402

PLACEMENTS = json.loads((ROOT / "simulation" / "scenarios" / "rcw2026-placements.json").read_text())
BASE_SCENARIO = json.loads((ROOT / "simulation" / "scenarios" / "gpsr-rcw2026-bench.json").read_text())
CONSTANTS = Path("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json")
YCB = "artifacts/objects/ycb/x/{key}/object.usd"


def _rect(xy, fp):
    return (xy[0] - fp[0] / 2, xy[1] - fp[1] / 2, xy[0] + fp[0] / 2, xy[1] + fp[1] / 2)


def _inside(xy, fp, spot_info, margin=PLACEMENT_EDGE_MARGIN_M):
    (bx0, by0), (bx1, by1) = spot_info["surface_bbox_xy"]
    x0, y0, x1, y1 = _rect(xy, fp)
    eps = 1e-9
    return bx0 + margin - eps <= x0 and x1 <= bx1 - margin + eps and by0 + margin - eps <= y0 and y1 <= by1 - margin + eps


def _separated(a_xy, a_fp, b_xy, b_fp, gap=PLACEMENT_GAP_M):
    ax0, ay0, ax1, ay1 = _rect(a_xy, a_fp)
    bx0, by0, bx1, by1 = _rect(b_xy, b_fp)
    return ax1 + gap <= bx0 + 1e-9 or bx1 + gap <= ax0 + 1e-9 or ay1 + gap <= by0 + 1e-9 or by1 + gap <= ay0 + 1e-9


def _item(id_, key, spot, xyz=(0.0, 0.0, 0.0)):
    return SpawnItem(id=id_, kind="object", name=key, asset_uri=YCB.format(key=key), room="r", spot=spot,
                     xyz=xyz, quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))


# --------------------------------------------------------------- placements

def test_every_spot_has_a_measured_bbox_containing_its_surface_point():
    for spot, info in PLACEMENTS["spots"].items():
        (bx0, by0), (bx1, by1) = info["surface_bbox_xy"]
        sx, sy, _ = info["surface_xyz"]
        assert bx0 < sx < bx1 and by0 < sy < by1, spot
        hx, hy = info["half_extent_xy"]
        assert hx == pytest.approx((bx1 - bx0) / 2, abs=1e-3) and hy == pytest.approx((by1 - by0) / 2, abs=1e-3), spot


def test_footprint_table_covers_every_object_and_default_is_10cm():
    assert set(NAME_TO_YCB_DIR.values()) <= set(OBJECT_FOOTPRINT_XY)
    assert object_footprint_xy("ycb_010_tomato_soup_can") == (0.068, 0.068)
    assert object_footprint_xy("") == DEFAULT_FOOTPRINT_XY == (0.10, 0.10)


# ------------------------------------------------------------ SurfaceLayout

@pytest.mark.parametrize("spot", sorted(PLACEMENTS["spots"]))
@pytest.mark.parametrize("n", [1, 2, 3, 4])
def test_n_default_objects_fit_inside_every_spot(spot, n):
    info = PLACEMENTS["spots"][spot]
    layout = SurfaceLayout(spot, info)
    placed = layout.place_all([DEFAULT_FOOTPRINT_XY] * n)
    assert all(p is not None for p in placed), (spot, n, placed)
    for xy in placed:
        assert _inside(xy, DEFAULT_FOOTPRINT_XY, info), (spot, xy)
    for i in range(n):
        for j in range(i + 1, n):
            assert _separated(placed[i], DEFAULT_FOOTPRINT_XY, placed[j], DEFAULT_FOOTPRINT_XY), (spot, placed)
    # the block is centred on the surface point
    cx = sum(p[0] for p in placed) / n
    cy = sum(p[1] for p in placed) / n
    assert cx == pytest.approx(info["surface_xyz"][0], abs=0.06) and cy == pytest.approx(info["surface_xyz"][1], abs=0.06)


@pytest.mark.parametrize("spot", sorted(PLACEMENTS["spots"]))
def test_real_ycb_footprints_fit_inside_every_spot(spot):
    info = PLACEMENTS["spots"][spot]
    layout = SurfaceLayout(spot, info)
    keys = ["ycb_010_tomato_soup_can", "ycb_002_sugar_box", "ycb_005_spam", "ycb_006_mustard_bottle"]
    xys = layout.place_all([OBJECT_FOOTPRINT_XY[k] for k in keys])
    placed = [(xy, OBJECT_FOOTPRINT_XY[k]) for xy, k in zip(xys, keys)]
    assert all(xy is not None for xy, _ in placed), (spot, placed)
    for xy, fp in placed:
        assert _inside(xy, fp, info)
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            assert _separated(placed[i][0], placed[i][1], placed[j][0], placed[j][1])


def test_gap_leaves_room_for_the_gripper_palm():
    # 0.14 m palm centred on a ~0.10 m object overhangs ~0.02 m per side;
    # the gap must clear that with margin (fix5: "rejected: palm->env").
    # fix6 run 1: 0.08 gap + 0.04 margin refused 2 of 3 kitchen items on the
    # 0.38 m side table; 0.06 / 0.02 still clears the palm overhang.
    assert PLACEMENT_GAP_M >= 0.06
    assert PLACEMENT_EDGE_MARGIN_M == pytest.approx(0.02)


def test_two_real_objects_on_a_side_table_share_a_row_at_palm_pitch():
    info = PLACEMENTS["spots"]["side_table_02"]
    layout = SurfaceLayout("side_table_02", info)
    keys = ["ycb_010_tomato_soup_can", "ycb_006_mustard_bottle"]
    fps = [OBJECT_FOOTPRINT_XY[k] for k in keys]
    placed = layout.place_all(fps)
    assert all(p is not None for p in placed)
    assert len({round(p[1], 3) for p in placed}) == 1, "two small objects share one row"
    pitch = placed[1][0] - placed[0][0]
    assert pitch == pytest.approx(fps[0][0] / 2 + fps[1][0] / 2 + PLACEMENT_GAP_M, abs=1e-6)
    assert abs((placed[0][0] + placed[1][0]) / 2 - info["surface_xyz"][0]) < 0.03
    for xy, fp in zip(placed, fps):
        assert _inside(xy, fp, info)


def test_three_real_objects_on_a_side_table_all_fit():
    # 0.068 + 0.049 + 0.097 + 2 x 0.06 = 0.334 m <= 0.34 m usable (0.02 margin).
    info = PLACEMENTS["spots"]["side_table_02"]
    keys = ["ycb_010_tomato_soup_can", "ycb_002_sugar_box", "ycb_006_mustard_bottle"]
    fps = [OBJECT_FOOTPRINT_XY[k] for k in keys]
    placed = SurfaceLayout("side_table_02", info).place_all(fps)
    assert all(p is not None for p in placed)
    for i in range(3):
        assert _inside(placed[i], fps[i], info)
        for j in range(i + 1, 3):
            assert _separated(placed[i], fps[i], placed[j], fps[j])


def test_three_default_objects_on_a_side_table_form_an_l_and_four_a_2x2_block():
    info = PLACEMENTS["spots"]["side_table_02"]
    fp = DEFAULT_FOOTPRINT_XY
    # 3 x 0.10 m + 2 gaps = 0.46 m > 0.30 m usable: a single row cannot fit, so an L (2 + 1)
    three = SurfaceLayout("side_table_02", info).place_all([fp] * 3)
    assert all(p is not None for p in three)
    assert len({round(p[1], 3) for p in three}) == 2 and len({round(p[0], 3) for p in three}) == 2
    four = SurfaceLayout("side_table_02", info).place_all([fp] * 4)
    assert all(p is not None for p in four)
    assert len({round(p[1], 3) for p in four}) == 2 and len({round(p[0], 3) for p in four}) == 2
    xs = sorted({round(p[0], 3) for p in four})
    assert xs[1] - xs[0] == pytest.approx(0.16, abs=1e-6)  # 0.10 footprint + 0.06 gap
    for xy in four:
        assert _inside(xy, fp, info)
    # 2 x 0.10 + 0.08 = 0.28 m per axis: a fifth default object is refused.
    assert SurfaceLayout("side_table_02", info).place_all([fp] * 5)[-1] is None


def test_fix4_case_sugar_box_next_to_the_banana_keeps_the_palm_gap():
    # The base banana sits at side_table_02's centre; with the 0.02 margin a
    # sugar box still fits beside it, a full palm gap away and on the table.
    info = PLACEMENTS["spots"]["side_table_02"]
    layout = SurfaceLayout("side_table_02", info)
    banana_xy, banana_fp = (0.43, 1.755), OBJECT_FOOTPRINT_XY["ycb_011_banana"]
    layout.occupy(banana_xy, banana_fp)
    fp = OBJECT_FOOTPRINT_XY["ycb_002_sugar_box"]
    xy = layout.place(fp)
    assert xy is not None and _inside(xy, fp, info)
    assert _separated(xy, fp, banana_xy, banana_fp)


def test_object_beside_an_off_centre_neighbour_keeps_the_palm_gap():
    info = PLACEMENTS["spots"]["side_table_02"]
    layout = SurfaceLayout("side_table_02", info)
    (bx0, by0), _ = info["surface_bbox_xy"]
    corner_fp = OBJECT_FOOTPRINT_XY["ycb_010_tomato_soup_can"]
    corner = (bx0 + PLACEMENT_EDGE_MARGIN_M + corner_fp[0] / 2, by0 + PLACEMENT_EDGE_MARGIN_M + corner_fp[1] / 2)
    layout.occupy(corner, corner_fp)
    fp = OBJECT_FOOTPRINT_XY["ycb_002_sugar_box"]
    xy = layout.place(fp)
    assert xy is not None and _inside(xy, fp, info)
    assert _separated(xy, fp, corner, corner_fp)


def test_off_edge_refusal_when_the_surface_is_full():
    info = PLACEMENTS["spots"]["side_table_02"]
    layout = SurfaceLayout("side_table_02", info)
    layout.occupy((0.43, 1.755), OBJECT_FOOTPRINT_XY["ycb_011_banana"])
    bowl = OBJECT_FOOTPRINT_XY["ycb_024_bowl"]
    results = [layout.place(bowl) for _ in range(6)]
    assert None in results, "a 0.38 m table with a banana cannot take six bowls"
    for xy in results:
        if xy is not None:
            assert _inside(xy, bowl, info)
    assert "usable" in layout.describe() and "already on it" in layout.describe()


def test_footprint_larger_than_the_surface_is_refused_immediately():
    layout = SurfaceLayout("side_table_02", PLACEMENTS["spots"]["side_table_02"])
    assert layout.place((0.40, 0.10)) is None


def test_legacy_spot_without_bbox_falls_back_to_the_old_grid():
    layout = SurfaceLayout("x", {"surface_xyz": [1.0, 2.0, 0.5], "grid_dx": 0.18, "grid_dy": 0.15})
    got = layout.place_all([DEFAULT_FOOTPRINT_XY] * 4)
    assert got == [(pytest.approx(1.0), pytest.approx(2.0)), (pytest.approx(1.18), pytest.approx(2.0)),
                   (pytest.approx(1.36), pytest.approx(2.0)), (pytest.approx(1.0), pytest.approx(2.15))]
    assert "legacy grid" in layout.describe()


# ------------------------------------------------------------- apply_plan

class _Client:
    def __init__(self):
        self.spawned = []

    def spawn(self, item):
        self.spawned.append(item)
        return f"/World/Scenario/{item.id}"


def test_apply_plan_next_to_the_centred_base_banana_places_what_fits_and_refuses_the_rest():
    # With a palm-sized gap the banana leaves room for ONE small item beside it;
    # the second is refused with a reason, never crammed against the banana.
    plan = ScenePlan(items=(_item("cmd_sugar_box_0", "ycb_002_sugar_box", "side_table_02"),
                            _item("cmd_soup_0", "ycb_010_tomato_soup_can", "side_table_02")), notes=())
    client = _Client()
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    info = PLACEMENTS["spots"]["side_table_02"]
    banana = ((0.43, 1.755), OBJECT_FOOTPRINT_XY["ycb_011_banana"])
    for entity in manifest["entities"]:
        fp = object_footprint_xy(entity["asset_key"])
        xy = (entity["xyz"][0], entity["xyz"][1])
        assert _inside(xy, fp, info) and _separated(xy, fp, *banana)
    for refused in manifest["not_attempted"]:
        assert refused["error"].startswith("not attempted: no room on 'side_table_02'")
    assert len(manifest["entities"]) + len(manifest["not_attempted"]) == 2


def test_apply_plan_packs_around_the_base_objects_and_never_off_the_table():
    plan = ScenePlan(items=(_item("cmd_sugar_box_0", "ycb_002_sugar_box", "kitchen_table"),
                            _item("cmd_spam_0", "ycb_005_spam", "kitchen_table")), notes=())
    client = _Client()
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    assert manifest["not_attempted"] == [] and manifest["skipped"] == []
    info = PLACEMENTS["spots"]["kitchen_table"]
    base = [((o["pose"]["xyz"][0], o["pose"]["xyz"][1]), object_footprint_xy(o["asset_uri"].split("/")[-2]))
            for o in BASE_SCENARIO["objects"] if o["id"] in ("soup", "mug")]
    for entity, item in zip(manifest["entities"], client.spawned):
        fp = object_footprint_xy(entity["asset_key"])
        xy = (entity["xyz"][0], entity["xyz"][1])
        assert tuple(item.xyz) == tuple(entity["xyz"])
        assert entity["xyz"][2] == pytest.approx(0.734)
        assert _inside(xy, fp, info), entity
        for bxy, bfp in base:
            assert _separated(xy, fp, bxy, bfp), entity
    a, b = manifest["entities"]
    assert _separated((a["xyz"][0], a["xyz"][1]), object_footprint_xy(a["asset_key"]),
                      (b["xyz"][0], b["xyz"][1]), object_footprint_xy(b["asset_key"]))


def test_apply_plan_refuses_into_not_attempted_when_the_table_is_full():
    plan = ScenePlan(items=tuple(_item(f"cmd_bowl_{i}", "ycb_024_bowl", "side_table_02") for i in range(6)), notes=())
    client = _Client()
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    assert manifest["not_attempted"], "six bowls cannot fit next to the banana"
    for refused in manifest["not_attempted"]:
        assert refused["ok"] is False and refused["where"] == "side_table_02"
        assert refused["error"].startswith("not attempted: no room on 'side_table_02'")
        assert "ycb_024_bowl" in refused["error"]
    refused_ids = {r["id"] for r in manifest["not_attempted"]}
    assert refused_ids.isdisjoint(e["id"] for e in manifest["entities"])
    assert len(manifest["entities"]) + len(manifest["not_attempted"]) == 6
    info = PLACEMENTS["spots"]["side_table_02"]
    for entity in manifest["entities"]:
        assert _inside((entity["xyz"][0], entity["xyz"][1]), OBJECT_FOOTPRINT_XY["ycb_024_bowl"], info)


def test_apply_plan_respects_previous_manifest_positions():
    previous = {"entities": [
        {"id": "cmd_spam_0", "asset_key": "ycb_005_spam", "where": "side_table",
         "entity_name": "/World/Scenario/cmd_spam_0", "ok": True, "xyz": [-0.447, -5.045, 0.612]}], "skipped": []}
    plan = ScenePlan(items=(_item("cmd_soup_0", "ycb_010_tomato_soup_can", "side_table"),), notes=())
    manifest = apply_plan(plan, _Client(), base_scenario=BASE_SCENARIO, placements=PLACEMENTS,
                          previous_manifest=previous)
    fresh = [e for e in manifest["entities"] if e["id"] == "cmd_soup_0"][0]
    assert _separated((fresh["xyz"][0], fresh["xyz"][1]), OBJECT_FOOTPRINT_XY["ycb_010_tomato_soup_can"],
                      (-0.447, -5.045), OBJECT_FOOTPRINT_XY["ycb_005_spam"])


# ---------------------------------------------------------- emit_scenario

def test_emit_scenario_keeps_merged_objects_inside_and_clear_of_base_objects(capsys):
    plan = ScenePlan(items=tuple(_item(f"cmd_spam_{i}", "ycb_005_spam", "kitchen_table") for i in range(3)), notes=())
    merged = emit_scenario([plan], BASE_SCENARIO, PLACEMENTS)
    base_ids = {b["id"] for b in BASE_SCENARIO["objects"]}
    info = PLACEMENTS["spots"]["kitchen_table"]
    base_on_table = [((o["pose"]["xyz"][0], o["pose"]["xyz"][1]), object_footprint_xy(o["asset_uri"].split("/")[-2]))
                     for o in BASE_SCENARIO["objects"] if o["id"] in ("soup", "mug")]
    new = [o for o in merged["objects"] if o["id"] not in base_ids]
    assert len(new) == 3
    for o in new:
        xy = (o["pose"]["xyz"][0], o["pose"]["xyz"][1])
        assert _inside(xy, OBJECT_FOOTPRINT_XY["ycb_005_spam"], info)
        for bxy, bfp in base_on_table:
            assert _separated(xy, OBJECT_FOOTPRINT_XY["ycb_005_spam"], bxy, bfp)
    assert "no room" not in capsys.readouterr().err


def test_emit_scenario_omits_and_warns_when_nothing_fits(capsys):
    plan = ScenePlan(items=tuple(_item(f"cmd_bowl_{i}", "ycb_024_bowl", "side_table_02") for i in range(6)), notes=())
    merged = emit_scenario([plan], BASE_SCENARIO, PLACEMENTS)
    base_ids = {b["id"] for b in BASE_SCENARIO["objects"]}
    new = [o for o in merged["objects"] if o["id"] not in base_ids]
    assert len(new) < 6
    err = capsys.readouterr().err
    assert "no room on 'side_table_02'" in err and "omitted" in err


# ------------------------------------------------------------- plan_scene

@pytest.mark.skipif(not CONSTANTS.exists(), reason="decision constants not present")
def test_plan_scene_counting_command_stays_on_the_side_table():
    knowledge = json.loads(CONSTANTS.read_text())
    plan = plan_scene("tell me how many spam there are on the side_table_02", knowledge, PLACEMENTS, seed=1)
    assert len(plan.items) == 3
    info = PLACEMENTS["spots"]["side_table_02"]
    for it in plan.items:
        assert _inside((it.xyz[0], it.xyz[1]), OBJECT_FOOTPRINT_XY["ycb_005_spam"], info), it
    assert len({(it.xyz[0], it.xyz[1]) for it in plan.items}) == 3
