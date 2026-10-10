"""Offline free-space checker for every GPSR person placement.

fix5 (2026-09-30) run 002: persons.bedroom stood INSIDE side_table_02's bbox,
was knocked over at spawn and never appeared in the head camera. Every pose
tools/gpsr_scene.plan_scene can spawn a person at -- the upright room pose for
each allowlisted asset (standing / sitting / gesture) and `lying_pose` on the
floor or on a lie-able surface -- is checked here against the measured
furniture and wall bboxes in simulation/scenarios/rcw2026-pose-audit.json.
Pure geometry, no Isaac.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.gpsr_scene import (  # noqa: E402
    LYING_BODY_LENGTH_M,
    LYING_QUATERNION_XYZW,
    PERSON_ASSET_FALLBACKS,
    _quat_rotate,
    lying_pose,
)

PLACEMENTS = json.loads((ROOT / "simulation/scenarios/rcw2026-placements.json").read_text())
AUDIT = json.loads((ROOT / "simulation/scenarios/rcw2026-pose-audit.json").read_text())
PERSON_BOXES = json.loads((ROOT / "config/person-import.json").read_text())["collision_box_m"]
CONSTANTS_PATH = Path(
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json"
)

STANDING_CLEARANCE_M = 0.30   # person_standing box to any furniture/wall bbox
ANY_ASSET_CLEARANCE_M = 0.15  # every allowlisted asset box (sitting/gesture are larger)
LYING_CLEARANCE_M = 0.30      # whole lying body on the floor
# Room quadrants of the rcw2026 arena (interior faces of the outer walls and
# the x=0 / y=0 partition walls, from the pose-audit wall bboxes).
ROOMS = {
    "kitchen": (0.07, -5.35, 4.35, -0.07),
    "living_room": (-4.35, -5.35, -0.07, -0.07),
    "bedroom": (0.07, 0.07, 4.35, 4.85),
    "laundry_room": (-4.35, 0.07, -0.07, 4.85),
}
OBSTACLES = {
    name: (tuple(box["min"][:2]), tuple(box["max"][:2]), float(box["max"][2]))
    for group in ("furniture", "walls")
    for name, box in AUDIT[group].items()
}

# Facts pinned from constants.rcw2026.json so this test runs even without the
# decision checkout; test_spot_poses_match_the_decision_constants re-reads it.
SPOT_POSES = {
    "kitchen_table": ((3.294, -2.351), -135.0),
    "sofa": ((-2.184, -3.837), -90.0),
    "side_table_02": ((1.08, 1.755), 180.0),
    "laundry_desk": ((-2.988, 3.677), 90.0),
}
ROOM_SPOT = {"kitchen": "kitchen_table", "living_room": "sofa",
             "bedroom": "side_table_02", "laundry_room": "laundry_desk"}


# ------------------------------------------------------------------ geometry

def _yaw_of(quaternion_xyzw) -> float:
    x, y, z, w = quaternion_xyzw
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _rect(xy, yaw, lo, hi):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(xy[0] + x * c - y * s, xy[1] + x * s + y * c)
            for x, y in ((lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1]))]


def _box(lo, hi):
    return [(lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1])]


def _segment_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _overlap(p, q):
    for poly in (p, q):
        for i in range(len(poly)):
            a, b = poly[i], poly[(i + 1) % len(poly)]
            nx, ny = b[1] - a[1], a[0] - b[0]
            pa = [nx * x + ny * y for x, y in p]
            qa = [nx * x + ny * y for x, y in q]
            if max(pa) < min(qa) or max(qa) < min(pa):
                return False
    return True


def _poly_distance(p, q):
    if _overlap(p, q):
        return 0.0
    edges = lambda poly: [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]  # noqa: E731
    return min(min(_segment_distance(v, a, b) for v in p for a, b in edges(q)),
               min(_segment_distance(v, a, b) for v in q for a, b in edges(p)))


def clearance(poly):
    """(metres, obstacle) to the nearest furniture/wall bbox."""
    return min((_poly_distance(poly, _box(lo, hi)), name) for name, (lo, hi, _h) in OBSTACLES.items())


def _segment_hits(a, b, lo, hi, n=400):
    return any(
        lo[0] <= a[0] + (b[0] - a[0]) * i / n <= hi[0] and lo[1] <= a[1] + (b[1] - a[1]) * i / n <= hi[1]
        for i in range(n + 1)
    )


def line_of_sight_blockers(spot_xy, target_xy, min_height):
    return [name for name, (lo, hi, h) in OBSTACLES.items()
            if h >= min_height and _segment_hits(spot_xy, target_xy, lo, hi)]


def _in_room(room, poly):
    x0, y0, x1, y1 = ROOMS[room]
    return all(x0 - 1e-6 <= x <= x1 + 1e-6 and y0 - 1e-6 <= y <= y1 + 1e-6 for x, y in poly)


def _bearing_deg(spot, xy):
    (sx, sy), heading = SPOT_POSES[spot]
    angle = math.degrees(math.atan2(xy[1] - sy, xy[0] - sx)) - heading
    return (angle + 180.0) % 360.0 - 180.0


def _allowlisted_assets():
    keys = {key for chain in PERSON_ASSET_FALLBACKS.values() for key in chain}
    return sorted(key for key in keys if key in PERSON_BOXES)


def _lying_footprint(xyz, quaternion_xyzw):
    """World xy corners of the lying body (collision box rotated by the full quat)."""
    box = PERSON_BOXES["person_standing"]
    corners = []
    for x in (box["min"][0], box["max"][0]):
        for y in (box["min"][1], box["max"][1]):
            for z in (0.0, LYING_BODY_LENGTH_M):
                v = _quat_rotate(tuple(quaternion_xyzw), (x, y, z))
                corners.append((xyz[0] + v[0], xyz[1] + v[1]))
    # Convex hull of the projected corners (monotone chain).
    pts = sorted(set((round(a, 9), round(b, 9)) for a, b in corners))
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])  # noqa: E731
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


ROOM_NAMES = sorted(PLACEMENTS["persons"])


# --------------------------------------------------------------------- tests

def test_checker_flags_the_fix5_bedroom_pose_inside_side_table_02():
    poly = _rect((0.28, 1.755), 0.0, *[tuple(PERSON_BOXES["person_standing"][k][:2]) for k in ("min", "max")])
    distance, name = clearance(poly)
    assert distance == 0.0 and name == "side_table_02"


@pytest.mark.parametrize("room", ROOM_NAMES)
def test_upright_person_is_clear_of_every_furniture_and_wall_bbox(room):
    pose = PLACEMENTS["persons"][room]
    xy, yaw = tuple(pose["xyz"][:2]), _yaw_of(pose["quaternion_xyzw"])
    assert pose["xyz"][2] == 0.0
    standing = _rect(xy, yaw, *[tuple(PERSON_BOXES["person_standing"][k][:2]) for k in ("min", "max")])
    distance, name = clearance(standing)
    assert distance >= STANDING_CLEARANCE_M, f"{room}: person_standing {distance:.3f} m from {name}"
    assert _in_room(room, standing), room
    # sitting / gesture assets reuse the same pose -- all must at least clear.
    for asset in _allowlisted_assets():
        box = PERSON_BOXES[asset]
        poly = _rect(xy, yaw, tuple(box["min"][:2]), tuple(box["max"][:2]))
        distance, name = clearance(poly)
        assert distance >= ANY_ASSET_CLEARANCE_M, f"{room}/{asset}: {distance:.3f} m from {name}"
        assert _in_room(room, poly), f"{room}/{asset}"


@pytest.mark.parametrize("room", ROOM_NAMES)
def test_upright_person_faces_and_is_visible_from_the_room_search_spot(room):
    pose = PLACEMENTS["persons"][room]
    spot = ROOM_SPOT[room]
    (sx, sy), _heading = SPOT_POSES[spot]
    x, y = pose["xyz"][:2]
    distance = math.hypot(x - sx, y - sy)
    assert 1.2 <= distance <= 2.5, f"{room}: {distance:.2f} m from {spot}"
    # pan +/-45 deg + half the ~90 deg head-camera FOV
    assert abs(_bearing_deg(spot, (x, y))) <= 90.0, room
    assert line_of_sight_blockers((sx, sy), (x, y), 0.8) == [], room
    # person_standing faces its own local -y (tools/gpsr_scene.py lying comment)
    yaw = _yaw_of(pose["quaternion_xyzw"])
    face = (math.sin(yaw), -math.cos(yaw))
    to_spot = ((sx - x) / distance, (sy - y) / distance)
    assert face[0] * to_spot[0] + face[1] * to_spot[1] > math.cos(math.radians(5)), room


@pytest.mark.parametrize("room", ROOM_NAMES)
def test_lying_person_on_the_floor_is_clear_and_in_view(room):
    pose = PLACEMENTS["persons"][room]
    spot = ROOM_SPOT[room]
    xyz, quat, note = lying_pose(pose, spot=spot, placements=PLACEMENTS)
    if "floor" not in note:
        pytest.skip(f"{room}: lies on '{spot}' (see the surface test)")
    assert "lying" in pose, f"{room}: floor-lying needs a precomputed free-floor `lying` pose"
    footprint = _lying_footprint(xyz, quat)
    distance, name = clearance(footprint)
    assert distance >= LYING_CLEARANCE_M, f"{room}: lying body {distance:.3f} m from {name}"
    assert _in_room(room, footprint), room
    centre = (sum(p[0] for p in footprint) / len(footprint), sum(p[1] for p in footprint) / len(footprint))
    (sx, sy), _ = SPOT_POSES[spot]
    assert 1.2 <= math.hypot(centre[0] - sx, centre[1] - sy) <= 2.5, room
    assert line_of_sight_blockers((sx, sy), centre, 0.0) == [], room
    # laundry_room's only free floor is 92 deg off the laundry_desk heading
    # (see _persons_comment); everything else is inside +/-90.
    assert abs(_bearing_deg(spot, centre)) <= 95.0, room


def test_lying_on_the_sofa_stays_on_the_sofa():
    pose = PLACEMENTS["persons"]["living_room"]
    xyz, quat, note = lying_pose(pose, spot="sofa", placements=PLACEMENTS)
    assert "sofa" in note
    (x0, y0), (x1, y1) = PLACEMENTS["spots"]["sofa"]["surface_bbox_xy"]
    for x, y in _lying_footprint(xyz, quat):
        assert x0 - 1e-6 <= x <= x1 + 1e-6 and y0 - 1e-6 <= y <= y1 + 1e-6, (x, y)
    # face up
    assert _quat_rotate(quat, (0.0, -1.0, 0.0)) == pytest.approx((0.0, 0.0, 1.0), abs=1e-6)
    assert LYING_QUATERNION_XYZW  # imported contract


@pytest.mark.skipif(not CONSTANTS_PATH.is_file(), reason="decision constants not present")
def test_spot_poses_match_the_decision_constants():
    constants = json.loads(CONSTANTS_PATH.read_text())
    for room, spots in constants["search_spots"].items():
        if room.startswith("_"):
            continue
        assert ROOM_SPOT[room] == spots[0], room
    for spot, ((x, y), heading) in SPOT_POSES.items():
        pose = constants["possible_poses"][spot]
        o = pose["orientation"]
        assert (pose["point"]["x"], pose["point"]["y"]) == pytest.approx((x, y)), spot
        yaw = math.degrees(2.0 * math.atan2(o["z"], o["w"]))
        assert ((yaw - heading + 180.0) % 360.0 - 180.0) == pytest.approx(0.0, abs=0.1), spot


def test_bench_scenario_actors_are_clear_too():
    for name in ("gpsr-rcw2026.json", "gpsr-rcw2026-bench.json"):
        scenario = json.loads((ROOT / "simulation/scenarios" / name).read_text())
        for actor in scenario.get("actors", []):
            xy = tuple(actor["pose"]["xyz"][:2])
            yaw = _yaw_of(actor["pose"]["quaternion_xyzw"])
            poly = _rect(xy, yaw, *[tuple(PERSON_BOXES["person_standing"][k][:2]) for k in ("min", "max")])
            distance, obstacle = clearance(poly)
            assert distance >= STANDING_CLEARANCE_M, f"{name}/{actor['id']}: {distance:.3f} m from {obstacle}"


def test_operator_stands_clear_in_front_of_the_command_point():
    pose = PLACEMENTS["operator"]
    xy, yaw = tuple(pose["xyz"][:2]), _yaw_of(pose["quaternion_xyzw"])
    for asset in ["person_standing"] + _allowlisted_assets():
        box = PERSON_BOXES[asset]
        poly = _rect(xy, yaw, tuple(box["min"][:2]), tuple(box["max"][:2]))
        distance, name = clearance(poly)
        floor = STANDING_CLEARANCE_M if asset == "person_standing" else ANY_ASSET_CLEARANCE_M
        assert distance >= floor, f"operator/{asset}: {distance:.3f} m from {name}"
    command_point = json.loads(CONSTANTS_PATH.read_text())["possible_poses"]["command_point"]["point"] \
        if CONSTANTS_PATH.is_file() else {"x": -2.0, "y": -2.0}
    # ~1 m in front of the robot at the command point, facing back at it.
    assert math.hypot(xy[0] - command_point["x"], xy[1] - command_point["y"]) == pytest.approx(1.0, abs=0.2)
    assert abs(math.degrees(yaw)) == pytest.approx(180.0, abs=1.0)
