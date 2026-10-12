#!/usr/bin/env python3
"""Audit the decision-side GPSR navigation poses against the arena USD geometry.

The decision repo's ``constants.rcw2026.json`` ``possible_poses`` are navigation
goals (x, y, quaternion yaw) the robot drives to in order to stand in front of a
piece of furniture. This tool reads the built rcw2026 arena USD (``pxr`` -- run
with the Isaac venv: ``.venv/bin/python tools/gpsr_pose_audit.py ...``), computes
the world-space bounding box of every furniture prim and wall, and for each pose
reports:

* distance from the pose to the nearest face of its furniture's XY bbox
  (and which face / corner it is approaching);
* angle error between the pose's forward vector (yaw) and the direction to the
  furniture centre;
* how far off the approach axis the pose is (angle between forward and the
  approached face's inward normal) and the lateral offset from the face's
  centreline;
* whether the pose lies inside any wall / furniture bbox, and the nearest
  obstacle with its clearance;
* the placements ``surface_xyz`` for the spot and whether it lies within the
  furniture bbox;
* a RECOMMENDED pose: ``--standoff`` (default 0.55 m) from the free front face,
  centred on it, facing it, with ``--clearance`` (default 0.30 m, the robot's
  half-width plus margin) to every other obstacle; when no face is free (the
  kitchen table is boxed in by four chairs) a diagonal corner approach is used.

Geometry helpers are pure Python (no pxr) so they can be unit-tested; only
``read_arena`` touches USD. Prints a table and writes ``--out`` JSON (also
consumable by ``tools/gpsr_run_truth_probe.py --extents-json`` for real
``half_extent_xy`` values).
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

FURNITURE_SCOPE = "/World/Arena/Furniture"
WALLS_SCOPE = "/World/Arena/Walls"
PRIM_PREFIX = "rcw26_"
# Robot footprint (tools/tinker_sim_deploy/workspace.py): 0.15 m ahead of the base
# origin, 0.35 m behind, 0.25 m to each side.
ROBOT_HALF_WIDTH = 0.25
ROBOT_FRONT = 0.15
DEFAULT_STANDOFF = 0.55
DEFAULT_CLEARANCE = 0.30
MIN_OBSTACLE_HEIGHT = 0.05  # ignore the floor / rugs when testing "inside an obstacle"

FACES = {  # name -> outward normal (unit)
    "north": (0.0, 1.0),
    "south": (0.0, -1.0),
    "east": (1.0, 0.0),
    "west": (-1.0, 0.0),
}


@dataclass(frozen=True)
class Box:
    name: str
    min: tuple[float, float, float]
    max: tuple[float, float, float]

    @property
    def centre(self) -> tuple[float, float, float]:
        return tuple((a + b) / 2.0 for a, b in zip(self.min, self.max))

    @property
    def half_extent_xy(self) -> tuple[float, float]:
        return ((self.max[0] - self.min[0]) / 2.0, (self.max[1] - self.min[1]) / 2.0)

    @property
    def height(self) -> float:
        return self.max[2] - self.min[2]

    def contains_xy(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (self.min[0] - margin <= x <= self.max[0] + margin
                and self.min[1] - margin <= y <= self.max[1] + margin)

    def distance_xy(self, x: float, y: float) -> float:
        """Euclidean distance from (x, y) to the box's XY rectangle (0 inside)."""
        dx = max(self.min[0] - x, 0.0, x - self.max[0])
        dy = max(self.min[1] - y, 0.0, y - self.max[1])
        return math.hypot(dx, dy)

    def face_centre(self, face: str) -> tuple[float, float]:
        cx, cy, _ = self.centre
        return {
            "north": (cx, self.max[1]), "south": (cx, self.min[1]),
            "east": (self.max[0], cy), "west": (self.min[0], cy),
        }[face]

    def face_half_length(self, face: str) -> float:
        hx, hy = self.half_extent_xy
        return hx if face in ("north", "south") else hy

    def corners(self) -> dict[str, tuple[float, float]]:
        return {
            "north-east": (self.max[0], self.max[1]), "north-west": (self.min[0], self.max[1]),
            "south-east": (self.max[0], self.min[1]), "south-west": (self.min[0], self.min[1]),
        }

    def as_dict(self) -> dict[str, Any]:
        return {"min": [round(v, 4) for v in self.min], "max": [round(v, 4) for v in self.max],
                "centre": [round(v, 4) for v in self.centre],
                "half_extent_xy": [round(v, 4) for v in self.half_extent_xy]}


def yaw_deg_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.degrees(math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def angle_between_deg(ax: float, ay: float, bx: float, by: float) -> float:
    return abs(wrap_deg(math.degrees(math.atan2(by, bx) - math.atan2(ay, ax))))


# --------------------------------------------------------------------------- USD

def read_arena(arena_usd: Path) -> tuple[dict[str, Box], dict[str, Box]]:
    """World-space XY/Z bboxes of every furniture prim and wall in the arena USD.

    The sim references the arena at ``/World/Arena`` with an identity transform
    (``backend.py`` ``add_reference_to_stage``), so stage-space == world-space.
    """
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(arena_usd))
    if stage is None:
        raise FileNotFoundError(arena_usd)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                              useExtentsHint=False)

    def _boxes(scope: str) -> dict[str, Box]:
        root = stage.GetPrimAtPath(scope)
        if not root:
            raise ValueError(f"{scope} not found in {arena_usd}")
        out: dict[str, Box] = {}
        for prim in root.GetChildren():
            r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if r.IsEmpty():
                continue
            mn, mx = r.GetMin(), r.GetMax()
            out[prim.GetName()] = Box(prim.GetName(), (mn[0], mn[1], mn[2]), (mx[0], mx[1], mx[2]))
        return out

    return _boxes(FURNITURE_SCOPE), _boxes(WALLS_SCOPE)


# ------------------------------------------------------------------------ audit

def _obstacles(furniture: Mapping[str, Box], walls: Mapping[str, Box],
               exclude: Optional[str] = None) -> list[Box]:
    out = [b for b in walls.values()]
    out += [b for n, b in furniture.items() if n != exclude and b.height >= MIN_OBSTACLE_HEIGHT]
    return out


def _nearest(obstacles: Iterable[Box], x: float, y: float) -> tuple[Optional[str], float]:
    best: tuple[Optional[str], float] = (None, math.inf)
    for b in obstacles:
        d = b.distance_xy(x, y)
        if d < best[1]:
            best = (b.name, d)
    return best


def _inside(obstacles: Iterable[Box], x: float, y: float) -> Optional[str]:
    for b in obstacles:
        if b.contains_xy(x, y):
            return b.name
    return None


def _clear(obstacles: Iterable[Box], x: float, y: float, clearance: float) -> bool:
    return all(b.distance_xy(x, y) >= clearance for b in obstacles)


def approached_face(box: Box, x: float, y: float) -> str:
    """Which face (or corner, e.g. 'north-east') of the box the point is nearest to."""
    outside_x = x > box.max[0] or x < box.min[0]
    outside_y = y > box.max[1] or y < box.min[1]
    ns = "north" if y > box.centre[1] else "south"
    ew = "east" if x > box.centre[0] else "west"
    if outside_x and outside_y:
        return f"{ns}-{ew}"
    if outside_y:
        return ns
    if outside_x:
        return ew
    # Inside the footprint: nearest face by distance.
    d = {"north": box.max[1] - y, "south": y - box.min[1], "east": box.max[0] - x, "west": x - box.min[0]}
    return min(d, key=d.get)


def candidate_faces(box: Box, x: float, y: float) -> list[str]:
    """Faces whose outward half-plane contains (x, y): one for a head-on approach,
    two past a corner, all four when inside the footprint."""
    faces = []
    if y > box.max[1]:
        faces.append("north")
    if y < box.min[1]:
        faces.append("south")
    if x > box.max[0]:
        faces.append("east")
    if x < box.min[0]:
        faces.append("west")
    return faces or list(FACES)


def best_face(box: Box, x: float, y: float, fx: float, fy: float) -> tuple[str, float]:
    """The approachable face whose inward normal best matches the forward vector
    (tie-break: nearer face centre) and the off-axis angle to it."""
    best: Optional[tuple[str, float]] = None
    for face in candidate_faces(box, x, y):
        nx, ny = FACES[face]
        off = angle_between_deg(fx, fy, -nx, -ny)
        cx, cy = box.face_centre(face)
        key = (round(off, 6), math.hypot(cx - x, cy - y))
        if best is None or key < best[1]:
            best = ((face, off), key)
    assert best is not None
    return best[0]


def free_face_candidates(box: Box, obstacles: Sequence[Box], *, standoff: float,
                         clearance: float) -> dict[str, tuple[float, float, float, float]]:
    """For each face, the free standing candidate with the smallest slide along
    the face (``(x, y, yaw, shift)``), trying the centre first then 5 cm steps."""
    out: dict[str, tuple[float, float, float, float]] = {}
    for face in FACES:
        half = box.face_half_length(face)
        shifts = [0.0]
        s = 0.05
        while s <= half + 1e-9:
            shifts += [s, -s]
            s += 0.05
        for shift in shifts:
            x, y, yaw = _candidate_on_face(box, face, standoff, shift)
            if _standing_clear(obstacles, x, y, yaw, clearance):
                out[face] = (x, y, yaw, shift)
                break
    return out


def _standing_clear(obstacles: Sequence[Box], x: float, y: float, yaw: float, clearance: float) -> bool:
    """Base origin ``clearance`` from every obstacle and the robot's rear (0.35 m
    behind) not inside one."""
    bx = x - math.cos(math.radians(yaw)) * 0.35
    by = y - math.sin(math.radians(yaw)) * 0.35
    return _clear(obstacles, x, y, clearance) and _inside(obstacles, bx, by) is None


def _candidate_on_face(box: Box, face: str, standoff: float, shift: float) -> tuple[float, float, float]:
    nx, ny = FACES[face]
    fx, fy = box.face_centre(face)
    tx, ty = -ny, nx  # tangent along the face
    x = fx + nx * standoff + tx * shift
    y = fy + ny * standoff + ty * shift
    yaw = math.degrees(math.atan2(-ny, -nx))  # face the furniture (inward normal)
    return x, y, yaw


def recommend(box: Box, obstacles: Sequence[Box], pose_xy: tuple[float, float], *,
              standoff: float, clearance: float) -> dict[str, Any]:
    """Free-space standing pose ``standoff`` from a free face, centred, facing it.

    Candidates on each face are tried at the centre first, then sliding along the
    face in 5 cm steps; the free candidate nearest the CURRENT pose is chosen so a
    fix moves the goal as little as possible. If no face is free the four corners
    are tried with a diagonal approach.
    """
    px, py = pose_xy
    best: Optional[dict[str, Any]] = None

    def consider(x: float, y: float, yaw: float, face: str, note: str) -> None:
        nonlocal best
        if not _standing_clear(obstacles, x, y, yaw, clearance):
            return
        move = math.hypot(x - px, y - py)
        cand = {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(wrap_deg(yaw), 1),
                "face": face, "note": note, "move_from_current_m": round(move, 3)}
        if best is None or move < best["move_from_current_m"] - 1e-9:
            best = cand

    for face, (x, y, yaw, shift) in free_face_candidates(
            box, obstacles, standoff=standoff, clearance=clearance).items():
        note = "centred" if abs(shift) < 1e-9 else f"shifted {shift:+.2f} m along face"
        consider(x, y, yaw, face, note)
    if best is not None:
        return best
    for corner, (cx, cy) in box.corners().items():
        dx = 1.0 if "east" in corner else -1.0
        dy = 1.0 if "north" in corner else -1.0
        n = 1.0 / math.sqrt(2.0)
        x, y = cx + dx * n * standoff, cy + dy * n * standoff
        yaw = math.degrees(math.atan2(-dy, -dx))
        consider(x, y, yaw, corner, "no free face; diagonal corner approach")
    return best or {"x": None, "y": None, "yaw_deg": None, "face": None,
                    "note": "no free standing spot found", "move_from_current_m": None}


def audit_pose(name: str, pose: Mapping[str, Any], furniture: Mapping[str, Box],
               walls: Mapping[str, Box], placements: Mapping[str, Any], *,
               standoff: float, clearance: float) -> dict[str, Any]:
    x = float(pose["point"]["x"])
    y = float(pose["point"]["y"])
    o = pose["orientation"]
    yaw = yaw_deg_from_quaternion(float(o["x"]), float(o["y"]), float(o["z"]), float(o["w"]))
    fx, fy = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    prim = f"{PRIM_PREFIX}{name}"
    box = furniture.get(prim)
    obstacles = _obstacles(furniture, walls, exclude=prim if box else None)
    inside = _inside(obstacles, x, y)
    nearest_name, nearest_d = _nearest(obstacles, x, y)
    spot = (placements.get("spots") or {}).get(name)

    rec: dict[str, Any] = {
        "pose": {"x": x, "y": y, "yaw_deg": round(yaw, 2)},
        "furniture_prim": f"{FURNITURE_SCOPE}/{prim}" if box else None,
        "inside_obstacle": inside,
        "nearest_obstacle": {"name": nearest_name, "distance_m": round(nearest_d, 3)},
    }
    reasons: list[str] = []
    verdict = "ok"
    if inside:
        reasons.append(f"pose is inside {inside}")
        verdict = "wrong"
    elif nearest_d < clearance:
        reasons.append(f"only {nearest_d:.2f} m from {nearest_name} (< {clearance:.2f} m robot clearance)")

    if box is None:
        # No furniture (command_point): free-space check only.
        rec.update({
            "verdict": verdict,
            "reasons": reasons or ["no furniture for this pose; free-space check only"],
            "recommended": {"x": round(x, 3), "y": round(y, 3), "yaw_deg": round(yaw, 1),
                            "face": None, "note": "keep", "move_from_current_m": 0.0}
            if verdict == "ok" else recommend_free(x, y, yaw, obstacles, clearance),
        })
        return rec

    cx, cy, _ = box.centre
    to_c = (cx - x, cy - y)
    angle_to_centre = angle_between_deg(fx, fy, *to_c)
    face = approached_face(box, x, y)
    dist_face = box.distance_xy(x, y)
    # Off-axis is always measured against a real face normal (never a corner
    # diagonal): a pose 3 cm past a corner that looks along the furniture's
    # length is a side view, not a frontal one.
    axis_face, off_axis = best_face(box, x, y, fx, fy)
    nx, ny = FACES[axis_face]
    fcx, fcy = box.face_centre(axis_face)
    lateral = abs((x - fcx) * -ny + (y - fcy) * nx)  # along the face tangent
    forward_hits = _ray_hits_box(box, x, y, fx, fy)

    free_faces = list(free_face_candidates(box, obstacles, standoff=standoff, clearance=clearance))
    surface = None
    if spot:
        sx, sy, sz = (float(v) for v in spot["surface_xyz"])
        surface = {"surface_xyz": [sx, sy, sz],
                   "inside_furniture_bbox_xy": box.contains_xy(sx, sy, margin=0.01),
                   "z_vs_bbox_top_m": round(sz - box.max[2], 3),
                   "distance_from_pose_m": round(math.hypot(sx - x, sy - y), 3)}

    if not forward_hits:
        reasons.append("forward ray misses the furniture bbox")
        verdict = "wrong"
    if angle_to_centre > 25.0:
        reasons.append(f"facing {angle_to_centre:.0f} deg away from the furniture centre")
        verdict = "wrong"
    if dist_face > 1.0:
        reasons.append(f"{dist_face:.2f} m from the nearest face (> 1.0 m)")
        verdict = "wrong"
    if dist_face < 0.2:
        reasons.append(f"{dist_face:.2f} m from the nearest face (< 0.2 m, robot front is {ROBOT_FRONT} m)")
        verdict = "wrong"
    if off_axis > 30.0 and free_faces:
        reasons.append(f"approach is {off_axis:.0f} deg off the {axis_face} face normal "
                       f"(diagonal/side view) although the {', '.join(free_faces)} face(s) are free")
        verdict = "wrong"
    elif "-" in face and not free_faces:
        reasons.append(f"corner approach ({face}); no face is free (boxed in)")
    if "-" not in face and lateral > box.face_half_length(axis_face):
        reasons.append(f"laterally {lateral:.2f} m off the {axis_face} face centreline (past the face edge)")
    elif "-" not in face and lateral > 0.15:
        reasons.append(f"laterally {lateral:.2f} m off the {axis_face} face centreline")
    if verdict == "ok" and dist_face < 0.4:
        reasons.append(f"close: {dist_face:.2f} m from the face")

    rec.update({
        "furniture_bbox": box.as_dict(),
        "approached_face": face,
        "axis_face": axis_face,
        "distance_to_nearest_face_m": round(dist_face, 3),
        "angle_to_centre_deg": round(angle_to_centre, 1),
        "approach_off_axis_deg": round(off_axis, 1),
        "lateral_offset_m": round(lateral, 3),
        "forward_ray_hits_furniture": forward_hits,
        "free_faces": free_faces,
        "surface": surface,
        "verdict": verdict,
        "reasons": reasons,
        "recommended": recommend(box, obstacles, (x, y), standoff=standoff, clearance=clearance),
    })
    return rec


def recommend_free(x: float, y: float, yaw: float, obstacles: Sequence[Box], clearance: float) -> dict[str, Any]:
    """Nearest clear point on a 5 cm grid around (x, y), keeping the yaw."""
    best = None
    r = 0.05
    while r <= 1.0 and best is None:
        for k in range(int(2 * math.pi * r / 0.05) + 1):
            a = 2 * math.pi * k / max(1, int(2 * math.pi * r / 0.05) + 1)
            cx, cy = x + r * math.cos(a), y + r * math.sin(a)
            if _clear(obstacles, cx, cy, clearance):
                best = {"x": round(cx, 3), "y": round(cy, 3), "yaw_deg": round(yaw, 1), "face": None,
                        "note": "moved to nearest clear point", "move_from_current_m": round(r, 3)}
                break
        r += 0.05
    return best or {"x": None, "y": None, "yaw_deg": None, "face": None,
                    "note": "no clear point within 1 m", "move_from_current_m": None}


def _ray_hits_box(box: Box, x: float, y: float, dx: float, dy: float, max_t: float = 3.0) -> bool:
    """Slab test: does the XY ray from (x, y) along (dx, dy) hit the box within max_t?"""
    tmin, tmax = 0.0, max_t
    for p, d, lo, hi in ((x, dx, box.min[0], box.max[0]), (y, dy, box.min[1], box.max[1])):
        if abs(d) < 1e-12:
            if p < lo or p > hi:
                return False
            continue
        t1, t2 = (lo - p) / d, (hi - p) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return False
    return True


def run_audit(constants: Mapping[str, Any], placements: Mapping[str, Any],
              furniture: Mapping[str, Box], walls: Mapping[str, Box], *,
              standoff: float = DEFAULT_STANDOFF, clearance: float = DEFAULT_CLEARANCE,
              arena_usd: Optional[str] = None) -> dict[str, Any]:
    poses = constants["possible_poses"]
    return {
        "schema_version": 1,
        "arena_usd": arena_usd,
        "standoff_m": standoff,
        "clearance_m": clearance,
        "furniture": {n[len(PRIM_PREFIX):] if n.startswith(PRIM_PREFIX) else n: b.as_dict()
                      for n, b in furniture.items()},
        "walls": {n: b.as_dict() for n, b in walls.items()},
        "poses": {name: audit_pose(name, pose, furniture, walls, placements,
                                   standoff=standoff, clearance=clearance)
                  for name, pose in poses.items()},
    }


def format_table(report: Mapping[str, Any]) -> str:
    cols = ("location", "pose (x, y, yaw)", "face", "dist", "ang", "off-ax", "nearest obstacle", "verdict",
            "recommended (x, y, yaw)")
    rows = []
    for name, r in report["poses"].items():
        p = r["pose"]
        rc = r["recommended"]
        near = r["nearest_obstacle"]
        rows.append((
            name,
            f"({p['x']:.3f}, {p['y']:.3f}, {p['yaw_deg']:.0f})",
            r.get("approached_face") or "-",
            f"{r['distance_to_nearest_face_m']:.2f}" if "distance_to_nearest_face_m" in r else "-",
            f"{r['angle_to_centre_deg']:.0f}" if "angle_to_centre_deg" in r else "-",
            f"{r['approach_off_axis_deg']:.0f}" if "approach_off_axis_deg" in r else "-",
            f"{near['name']} {near['distance_m']:.2f}" if near["name"] else "-",
            r["verdict"] + (" (inside)" if r["inside_obstacle"] else ""),
            (f"({rc['x']:.3f}, {rc['y']:.3f}, {rc['yaw_deg']:.0f}) {rc['face'] or ''} {rc['note']}"
             if rc["x"] is not None else rc["note"]),
        ))
    widths = [max(len(str(c)) for c in col) for col in zip(cols, *rows)]
    fmt = "  ".join("{:<" + str(w) + "}" for w in widths)
    lines = [fmt.format(*cols), fmt.format(*("-" * w for w in widths))]
    lines += [fmt.format(*row) for row in rows]
    for name, r in report["poses"].items():
        if r["reasons"]:
            lines.append(f"  {name}: " + "; ".join(r["reasons"]))
    return "\n".join(lines)


def _default_arena(root: Path) -> Optional[Path]:
    manifest = root / "artifacts" / "asset-manifest.json"
    if not manifest.exists():
        return None
    data = json.loads(manifest.read_text(encoding="utf-8"))
    for entry in data.get("generated_arena_usds", []):
        if "rcw2026" in entry.get("path", ""):
            return root / entry["path"]
    return None


def main(argv: Optional[list[str]] = None) -> int:
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--constants", required=True, help="decision constants.rcw2026.json")
    parser.add_argument("--placements", default=str(root / "simulation" / "scenarios" / "rcw2026-placements.json"))
    parser.add_argument("--arena-usd", default=None,
                        help="arena.usd (default: rcw2026 entry of artifacts/asset-manifest.json)")
    parser.add_argument("--out", default=None, help="write the JSON report here")
    parser.add_argument("--standoff", type=float, default=DEFAULT_STANDOFF)
    parser.add_argument("--clearance", type=float, default=DEFAULT_CLEARANCE)
    args = parser.parse_args(argv)

    arena = Path(args.arena_usd) if args.arena_usd else _default_arena(root)
    if arena is None or not arena.exists():
        parser.error(f"arena usd not found: {arena}")
    constants = json.loads(Path(args.constants).read_text(encoding="utf-8"))
    placements = json.loads(Path(args.placements).read_text(encoding="utf-8"))
    furniture, walls = read_arena(arena)
    try:  # record a repo-relative path so the report can be versioned
        arena_label = str(arena.resolve().relative_to(root))
    except ValueError:
        arena_label = str(arena)
    report = run_audit(constants, placements, furniture, walls, standoff=args.standoff,
                       clearance=args.clearance, arena_usd=arena_label)
    print(format_table(report))
    if args.out:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
