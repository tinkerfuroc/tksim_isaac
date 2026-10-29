#!/usr/bin/env python3
"""Live check that a candidate arena's placement surfaces hold an object.

Drops one test object 5 cm above each surface in `placement.json`, lets the
physics settle, and verdicts each surface: the object must still be on it
(not flung off, not sunk through, not floating) and no longer moving.

The geometry (`spawn_poses`, `judge_rest`) is pure; `rclpy` is imported only
inside `main`, which needs a running sim with the truth evaluator up.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path
from typing import Mapping, Sequence

REPO_ROOT = Path(__file__).resolve().parent.parent

DROP_M = 0.05
SANK_TOL_M = 0.01
FLOAT_TOL_M = 0.15
SETTLE_TOL_M = 0.005
SAMPLE_GAP_S = 1.0


def spawn_poses(
    placement: Mapping[str, object], drop_m: float = DROP_M
) -> list[tuple[str, tuple[float, float, float]]]:
    """One (surface_id, xyz) per surface, `drop_m` above its centre."""
    poses = []
    for surface in placement["surfaces"]:  # type: ignore[index]
        x, y, z = (float(v) for v in surface["center_xyz"])
        poses.append((str(surface["surface_id"]), (x, y, z + drop_m)))
    return poses


def judge_rest(
    surface: Mapping[str, object],
    first_xyz: Sequence[float],
    last_xyz: Sequence[float],
    settle_tol_m: float = SETTLE_TOL_M,
) -> tuple[bool, str]:
    """(passed, reason) for an object sampled at `first_xyz` then `last_xyz`."""
    cx, cy, surface_z = (float(v) for v in surface["center_xyz"])  # type: ignore[union-attr]
    size_x, size_y = (float(v) for v in surface["size_xy"])  # type: ignore[union-attr]
    margin = float(surface.get("edge_margin", 0.0))  # type: ignore[union-attr]
    yaw = float(surface.get("yaw", 0.0))  # type: ignore[union-attr]
    dx, dy = last_xyz[0] - cx, last_xyz[1] - cy
    # Into the surface frame (rotate by -yaw).
    lx = math.cos(yaw) * dx + math.sin(yaw) * dy
    ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
    if abs(lx) > size_x / 2 - margin or abs(ly) > size_y / 2 - margin:
        return False, "fell off"
    if last_xyz[2] < surface_z - SANK_TOL_M:
        return False, "sank"
    if last_xyz[2] > surface_z + FLOAT_TOL_M:
        return False, "floating"
    if math.dist(first_xyz, last_xyz) > settle_tol_m:
        return False, "still moving"
    return True, "rests"


class WaitTimeout(RuntimeError):
    """A wall-clock bound expired while waiting on the sim."""


def wait_sim(read_clock, spin, monotonic, seconds: float, wall_timeout_s: float | None = None) -> None:
    """Spin until `seconds` of sim time pass, counted from the first /clock message.

    `read_clock()` returns the latest sim time or None before any message;
    `spin()` processes callbacks once; `monotonic()` is the wall clock.
    Raises WaitTimeout if the wall deadline (default max(30, 5 * seconds))
    expires first, e.g. because /clock stalled.
    """
    if wall_timeout_s is None:
        wall_timeout_s = max(30.0, 5.0 * seconds)
    deadline = monotonic() + wall_timeout_s
    start = None
    while True:
        now = read_clock()
        if now is not None:
            if start is None:
                start = now
            if now - start >= seconds:
                return
        if monotonic() >= deadline:
            raise WaitTimeout(
                f"/clock did not advance {seconds:.1f}s of sim time within "
                f"{wall_timeout_s:.0f}s wall (sim stalled or no /clock publisher)"
            )
        spin()


def wait_fresh(get_stamps, baseline, ids, spin, monotonic, wall_timeout_s: float = 10.0) -> set:
    """Spin until every id in `ids` has a stamp differing from `baseline[id]`.

    Returns the ids that did get a new message (all of them unless the wall
    timeout expired first).
    """
    deadline = monotonic() + wall_timeout_s
    while True:
        stamps = get_stamps()
        fresh = {i for i in ids if i in stamps and stamps[i] != baseline.get(i)}
        if len(fresh) == len(set(ids)) or monotonic() >= deadline:
            return fresh
        spin()


def _asset_path(asset: str) -> str:
    pointer = json.loads((REPO_ROOT / "artifacts/objects/ycb/current.json").read_text())
    manifest = REPO_ROOT / pointer["manifest"]
    path = manifest.parent / asset / "object.usd"
    if not path.is_file():
        raise FileNotFoundError(f"object asset not found: {path}")
    return str(path)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--placement", type=Path, required=True)
    parser.add_argument("--asset", default="ycb_010_tomato_soup_can")
    parser.add_argument("--settle-s", type=float, default=8.0)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)

    placement = json.loads(args.placement.read_text())
    surfaces = {str(s["surface_id"]): s for s in placement["surfaces"]}
    poses = spawn_poses(placement)
    asset_uri = _asset_path(args.asset)

    sys.path.insert(0, str(REPO_ROOT))
    from tools.gpsr_scene import SpawnItem
    from tools.gpsr_spawn import _make_ros_service_client

    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from tinker_sim_interfaces.msg import ObjectTruth

    # _make_ros_service_client waits for services_ready and for both services.
    client = _make_ros_service_client()
    node = rclpy.create_node("placement_settle_check")
    clock: dict[str, float | None] = {"t": None}
    latest: dict[str, tuple[tuple[float, float, float], tuple[int, int]]] = {}

    def _on_clock(msg):
        clock["t"] = msg.clock.sec + msg.clock.nanosec * 1e-9

    def _on_truth(msg):
        p = msg.pose.position
        latest[msg.object_id] = ((p.x, p.y, p.z), (msg.stamp.sec, msg.stamp.nanosec))

    node.create_subscription(Clock, "/clock", _on_clock, 10)
    node.create_subscription(
        ObjectTruth, "/sim/truth/object_state", _on_truth,
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
    )

    def _spin() -> None:
        rclpy.spin_once(node, timeout_sec=0.1)

    def _wait_sim(seconds: float) -> None:
        wait_sim(lambda: clock["t"], _spin, time.monotonic, seconds)

    spawned: list[tuple[str, str]] = []  # (surface_id, entity)
    verdicts = []
    try:
        for n, (surface_id, xyz) in enumerate(poses):
            item = SpawnItem(
                id=f"settle_{n}", kind="object", name=args.asset,
                asset_uri=asset_uri, room="", spot="", xyz=xyz,
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            )
            spawned.append((surface_id, client.spawn(item)))
        _wait_sim(args.settle_s)
        first = dict(latest)
        _wait_sim(SAMPLE_GAP_S)
        ids = [entity.rstrip("/").rsplit("/", 1)[-1] for _, entity in spawned]
        baseline = {i: first[i][1] for i in ids if i in first}
        fresh = wait_fresh(lambda: {k: v[1] for k, v in latest.items()},
                           baseline, ids, _spin, time.monotonic)
        last = dict(latest)
        for (surface_id, _), object_id in zip(spawned, ids):
            if object_id not in first or object_id not in fresh:
                verdicts.append({"surface_id": surface_id, "object_id": object_id,
                                 "passed": False, "reason": "no fresh truth sample"})
                continue
            ok, reason = judge_rest(surfaces[surface_id], first[object_id][0], last[object_id][0])
            verdicts.append({
                "surface_id": surface_id, "object_id": object_id,
                "passed": ok, "reason": reason,
                "first_xyz": list(first[object_id][0]), "last_xyz": list(last[object_id][0]),
            })
    finally:
        for _, entity in spawned:
            try:
                client.delete(entity)
            except Exception as exc:  # best effort cleanup
                print(f"[placement_settle_check] delete {entity} failed: {exc}", file=sys.stderr)
        try:
            node.destroy_node()
            client.shutdown()
        except Exception:
            pass

    all_ok = bool(verdicts) and len(verdicts) == len(poses) and all(v["passed"] for v in verdicts)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(
        {"placement": str(args.placement), "asset": args.asset,
         "settle_s": args.settle_s, "passed": all_ok, "surfaces": verdicts},
        indent=2) + "\n")
    for v in verdicts:
        print(f"{'PASS' if v['passed'] else 'FAIL'} {v['surface_id']}: {v['reason']}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
