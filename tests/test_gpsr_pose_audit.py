"""Pure-geometry tests for tools/gpsr_pose_audit.py (no pxr needed).

A synthetic room: a 0.9 x 0.3 shelf with its back on a wall, a trash bin beside
it, and poses that (a) face it head-on, (b) look along it from past a corner,
(c) sit inside the wall. The verdict/recommendation rules the audit report relies
on are pinned here.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
_TOOLS = str(ROOT / "tools")
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)

import gpsr_pose_audit as audit  # noqa: E402
from gpsr_pose_audit import Box  # noqa: E402


def _q(yaw_deg):
    h = math.radians(yaw_deg) / 2
    return {"x": 0.0, "y": 0.0, "z": math.sin(h), "w": math.cos(h)}


def _pose(x, y, yaw):
    return {"point": {"x": x, "y": y, "z": 0.0}, "orientation": _q(yaw)}


SHELF = Box("rcw26_shelf", (-4.1, 0.15, 0.0), (-3.25, 0.45, 2.0))
BIN = Box("rcw26_trashbin", (-3.12, 0.2, 0.0), (-2.8, 0.45, 0.4))
WALL_S = Box("wall_s", (-4.4, -0.075, 0.0), (-2.0, 0.075, 1.25))
WALL_W = Box("wall_w", (-4.5, -1.0, 0.0), (-4.35, 3.0, 1.25))
FURNITURE = {"rcw26_shelf": SHELF, "rcw26_trashbin": BIN}
WALLS = {"wall_s": WALL_S, "wall_w": WALL_W}
PLACEMENTS = {"spots": {"shelf": {"surface_xyz": [-3.687, 0.309, 1.07]}}}


def test_box_distance_and_containment():
    assert SHELF.distance_xy(-3.7, 0.3) == 0.0
    assert SHELF.distance_xy(-3.7, 1.0) == pytest.approx(0.55)
    assert SHELF.distance_xy(-3.0, 0.75) == pytest.approx(math.hypot(0.25, 0.30))
    assert SHELF.contains_xy(-3.7, 0.3) and not SHELF.contains_xy(-3.0, 0.75)


def test_yaw_and_ray():
    assert audit.yaw_deg_from_quaternion(0, 0, -0.9238795, 0.3826834) == pytest.approx(-135.0, abs=1e-4)
    assert audit._ray_hits_box(SHELF, -3.687, 1.0, 0.0, -1.0)
    assert not audit._ray_hits_box(SHELF, -3.687, 1.0, 0.0, 1.0)


def test_head_on_pose_is_ok_and_recommendation_is_centred_at_standoff():
    r = audit.audit_pose("shelf", _pose(-3.675, 1.0, -90.0), FURNITURE, WALLS, PLACEMENTS,
                         standoff=0.55, clearance=0.30)
    assert r["verdict"] == "ok", r["reasons"]
    assert r["approached_face"] == "north" and r["axis_face"] == "north"
    assert r["approach_off_axis_deg"] == pytest.approx(0.0, abs=1e-6)
    assert r["angle_to_centre_deg"] == pytest.approx(0.0, abs=1e-6)
    assert r["distance_to_nearest_face_m"] == pytest.approx(0.55)
    assert r["inside_obstacle"] is None
    assert r["surface"]["inside_furniture_bbox_xy"] is True
    rec = r["recommended"]
    assert rec["face"] == "north" and rec["note"] == "centred"
    assert (rec["x"], rec["y"], rec["yaw_deg"]) == (pytest.approx(-3.675), pytest.approx(1.0), -90.0)


def test_diagonal_side_view_past_corner_is_wrong_and_moves_to_free_front_face():
    # The real 'shelf' defect: 3 cm past the east edge, looking along the shelf at -135 deg.
    r = audit.audit_pose("shelf", _pose(-3.22, 0.77, -135.0), FURNITURE, WALLS, PLACEMENTS,
                         standoff=0.55, clearance=0.30)
    assert r["approached_face"] == "north-east"
    assert r["approach_off_axis_deg"] == pytest.approx(45.0, abs=1e-6)
    assert "north" in r["free_faces"]
    assert r["verdict"] == "wrong"
    assert any("off the" in s and "face normal" in s for s in r["reasons"])
    assert r["recommended"]["face"] == "north"
    assert r["recommended"]["yaw_deg"] == -90.0
    assert r["nearest_obstacle"]["name"] == "rcw26_trashbin"


def test_pose_inside_wall_is_wrong():
    r = audit.audit_pose("shelf", _pose(-4.4, 0.3, 0.0), FURNITURE, WALLS, PLACEMENTS,
                         standoff=0.55, clearance=0.30)
    assert r["verdict"] == "wrong"
    assert r["inside_obstacle"] == "wall_w"


def test_boxed_in_table_gets_corner_approach():
    table = Box("rcw26_kitchen_table", (2.15, -3.49, 0.0), (3.02, -2.63, 0.72))
    chairs = {
        f"rcw26_chair_{i}": Box(f"rcw26_chair_{i}", lo, hi, ) for i, (lo, hi) in enumerate([
            ((2.36, -2.63, 0.0), (2.88, -2.01, 0.9)),   # north
            ((2.31, -4.01, 0.0), (2.83, -3.39, 0.9)),   # south
            ((2.92, -3.30, 0.0), (3.54, -2.78, 0.9)),   # east
            ((1.59, -3.33, 0.0), (2.21, -2.81, 0.9)),   # west
        ])
    }
    furniture = {"rcw26_kitchen_table": table, **chairs}
    r = audit.audit_pose("kitchen_table", _pose(3.294, -2.351, -135.0), furniture, {}, {},
                         standoff=0.55, clearance=0.30)
    assert r["free_faces"] == []
    assert r["verdict"] == "ok"
    assert any("boxed in" in s for s in r["reasons"])
    assert r["recommended"]["face"] == "north-east"
    assert r["recommended"]["yaw_deg"] == -135.0


def test_command_point_without_furniture_is_free_space_check_only():
    r = audit.audit_pose("command_point", _pose(-2.0, -2.0, 0.0), FURNITURE, WALLS, {},
                         standoff=0.55, clearance=0.30)
    assert r["furniture_prim"] is None
    assert r["verdict"] == "ok"
    assert r["recommended"]["note"] == "keep"


def test_run_audit_and_table_render():
    constants = {"possible_poses": {"shelf": _pose(-3.687, 1.0, -90.0),
                                    "command_point": _pose(-2.0, -2.0, 0.0)}}
    report = audit.run_audit(constants, PLACEMENTS, FURNITURE, WALLS)
    assert report["furniture"]["shelf"]["half_extent_xy"] == [pytest.approx(0.425), pytest.approx(0.15)]
    table = audit.format_table(report)
    assert "shelf" in table and "command_point" in table and "ok" in table
