#!/usr/bin/env python3
"""Spawn/clear a GPSR command's scene in the sim, or pre-generate a merged
scenario file for a whole battery.

`plan`/`emit-scenario` are pure Python (no ROS import at all). `apply`/
`clear` talk to the running sim's `/spawn_entity` and `/delete_entity`
simulation_interfaces services -- the only place this module imports
rclpy is `_make_ros_service_client`, called lazily so every other
subcommand (and every test) never needs ROS on the path.

Exit codes: 0 ok, 2 service unavailable/timeout (a run's `apply`/`clear`
failing this way is treated the same as a reset failure by the bench),
3 bad plan/scenario/placements input.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Optional, Protocol, Sequence

from tools.gpsr_scene import (
    REPO_ROOT,
    ScenePlan,
    SpawnItem,
    plan_scene,
    scene_plan_from_json,
    scene_plan_to_json,
)

DEFAULT_CONSTANTS = Path(
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json"
)
DEFAULT_PLACEMENTS = REPO_ROOT / "simulation" / "scenarios" / "rcw2026-placements.json"
DEFAULT_BASE_SCENARIO = REPO_ROOT / "simulation" / "scenarios" / "gpsr-rcw2026-bench.json"


class ServiceClient(Protocol):
    def spawn(self, item: SpawnItem) -> str:
        """Spawn `item`; return its entity name, or raise on failure."""
        ...

    def delete(self, entity: str) -> bool:
        """Delete `entity`; return True on success (NOT_FOUND counts as
        success -- the entity is already gone)."""
        ...


def _asset_key(asset_uri: str) -> str:
    """The distinguishing directory name of an asset_uri, e.g.
    ".../ycb_010_tomato_soup_can/object.usd" -> "ycb_010_tomato_soup_can",
    ".../person_standing/person.usd" -> "person_standing".
    """
    parts = Path(asset_uri).parts
    return parts[-2] if len(parts) >= 2 else asset_uri


def _nearest_spot_or_room(xyz, placements: dict) -> Optional[str]:
    """The placement spot (or person room) whose known xy is closest to
    `xyz`. There is no fixed tolerance: a hand-authored base scenario's
    pose need not exactly equal the placement table's grid-formula pose
    (e.g. the bench scenario's "mug" sits ~0.25m from kitchen_table's
    surface_xyz, not on the (dx, dy) grid `plan_scene` itself would use)
    -- what matters is that it is unambiguously nearer to that spot/room
    than to any other, and every spot/room in the (small, sparse) table
    is metres apart from every other, so nearest-neighbour is unambiguous
    in practice. Returns None only if the table has no spots or persons
    at all.
    """
    best_key: Optional[str] = None
    best_dist = math.inf
    for spot, info in placements.get("spots", {}).items():
        sx, sy, _sz = info["surface_xyz"]
        dist = math.hypot(xyz[0] - sx, xyz[1] - sy)
        if dist < best_dist:
            best_dist = dist
            best_key = spot
    for room, info in placements.get("persons", {}).items():
        px, py, _pz = info["xyz"]
        dist = math.hypot(xyz[0] - px, xyz[1] - py)
        if dist < best_dist:
            best_dist = dist
            best_key = room
    return best_key


def base_scenario_keys(base_scenario: dict, placements: dict) -> set[tuple[str, str]]:
    """(asset_key, spot-or-room) already present in the base scenario,
    derived by matching each spawned pose against the placement table
    (not hardcoded, so it stays correct if the base scenario changes).
    """
    keys: set[tuple[str, str]] = set()
    for record in (*base_scenario.get("actors", []), *base_scenario.get("objects", [])):
        asset = _asset_key(record.get("asset_uri", ""))
        xyz = record.get("pose", {}).get("xyz", [0.0, 0.0, 0.0])
        where = _nearest_spot_or_room(xyz, placements)
        if where is not None:
            keys.add((asset, where))
    return keys


def apply_plan(
    plan: ScenePlan,
    client: "ServiceClient",
    *,
    base_scenario: dict,
    placements: dict,
    previous_manifest: Optional[dict] = None,
) -> dict:
    """Spawn every item in `plan` via `client`, skipping items whose
    (asset, spot-or-room) already exists in the base scenario or a
    previous manifest. Never raises for a per-item spawn failure -- it is
    recorded in the returned manifest's entities with `"ok": False`.
    """
    seen_keys = set(base_scenario_keys(base_scenario, placements))
    entities: list[dict] = []
    if previous_manifest:
        for e in previous_manifest.get("entities", []):
            seen_keys.add((e["asset_key"], e["where"]))
            entities.append(e)

    skipped: list[dict] = []
    for item in plan.items:
        where = item.spot if item.spot else item.room
        asset_key = _asset_key(item.asset_uri)
        key = (asset_key, where)
        if key in seen_keys:
            skipped.append({"id": item.id, "reason": "already present", "where": where})
            continue
        entity_name = f"/World/Scenario/{item.id}"
        try:
            actual = client.spawn(item)
        except Exception as exc:  # noqa: BLE001 - never crash a bench run
            entities.append(
                {"id": item.id, "asset_key": asset_key, "where": where,
                 "entity_name": entity_name, "ok": False, "error": repr(exc)}
            )
            continue
        seen_keys.add(key)
        entities.append(
            {"id": item.id, "asset_key": asset_key, "where": where,
             "entity_name": actual, "ok": True}
        )
    return {"entities": entities, "skipped": skipped}


def clear_manifest(manifest: dict, client: "ServiceClient") -> dict:
    """Delete every successfully-spawned entity in `manifest` via
    `client`. Tolerates NOT_FOUND (client.delete returning True). Never
    raises for a per-item delete failure.
    """
    results: list[dict] = []
    for e in manifest.get("entities", []):
        if not e.get("ok"):
            results.append(e)
            continue
        try:
            deleted = client.delete(e["entity_name"])
        except Exception as exc:  # noqa: BLE001 - never crash a bench run
            results.append({**e, "cleared": False, "clear_error": repr(exc)})
            continue
        results.append({**e, "cleared": bool(deleted)})
    return {"entities": results, "skipped": manifest.get("skipped", [])}


def emit_scenario(plans: Sequence[ScenePlan], base_scenario: dict, placements: dict) -> dict:
    """Merge every plan's items into a COPY of base_scenario (base_scenario
    itself is never mutated). Dedupe by (asset, spot-or-room) against the
    base scenario; when two plans want the same (asset, spot-or-room),
    the plan that proposed MORE instances of it wins outright (its whole
    item group is used, the smaller plan's group is dropped).
    """
    merged = json.loads(json.dumps(base_scenario))
    existing = base_scenario_keys(base_scenario, placements)

    best: dict[tuple[str, str], list[SpawnItem]] = {}
    for plan in plans:
        per_plan: dict[tuple[str, str], list[SpawnItem]] = {}
        for item in plan.items:
            where = item.spot if item.spot else item.room
            key = (_asset_key(item.asset_uri), where)
            per_plan.setdefault(key, []).append(item)
        for key, group in per_plan.items():
            if key in existing:
                continue
            if key not in best or len(group) > len(best[key]):
                best[key] = group

    for (_asset_key_value, where), group in best.items():
        target_list = merged["actors"] if group[0].kind == "person" else merged["objects"]
        for item in group:
            target_list.append(
                {
                    # Suffixed by `where` so two different (asset, where)
                    # groups across different plans can never collide on
                    # id even if their SpawnItem ids happened to match.
                    "id": f"{item.id}__{where}",
                    "asset_uri": item.asset_uri,
                    "pose": {"xyz": list(item.xyz), "quaternion_xyzw": list(item.quaternion_xyzw)},
                }
            )
    return merged


def _resolve_asset_uri(asset_uri: str) -> str:
    if "://" in asset_uri:
        return asset_uri
    path = Path(asset_uri)
    if not path.is_absolute():
        path = REPO_ROOT / path
    return str(path.resolve())


def _make_ros_service_client() -> "ServiceClient":
    """The only place this module imports rclpy -- constructed lazily so
    `plan`/`emit-scenario` (and every test) never need ROS on the path.
    Raises if `/spawn_entity` never becomes available.
    """
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from simulation_interfaces.msg import Result
    from simulation_interfaces.srv import DeleteEntity, SpawnEntity

    rclpy.init(args=None)
    node = Node("gpsr_spawn_client")
    spawn_client = node.create_client(SpawnEntity, "/spawn_entity")
    delete_client = node.create_client(DeleteEntity, "/delete_entity")
    if not spawn_client.wait_for_service(timeout_sec=10.0):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("/spawn_entity service unavailable after 10s")

    class _RclpyServiceClient:
        def spawn(self, item: SpawnItem) -> str:
            request = SpawnEntity.Request()
            request.name = f"/World/Scenario/{item.id}"
            request.allow_renaming = False
            request.uri = _resolve_asset_uri(item.asset_uri)
            request.entity_namespace = "Scenario"
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = item.xyz
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = item.quaternion_xyzw
            request.initial_pose = pose
            future = spawn_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
            if future.result() is None:
                raise RuntimeError(f"spawn_entity timed out for {item.id}")
            response = future.result()
            if response.result.result != Result.RESULT_OK:
                raise RuntimeError(
                    f"spawn_entity failed for {item.id}: {response.result.error_message}"
                )
            return str(response.entity_name)

        def delete(self, entity: str) -> bool:
            request = DeleteEntity.Request()
            request.entity = entity
            future = delete_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
            if future.result() is None:
                raise RuntimeError(f"delete_entity timed out for {entity}")
            response = future.result()
            if response.result.result == Result.RESULT_NOT_FOUND:
                return True
            return response.result.result == Result.RESULT_OK

        def shutdown(self) -> None:
            node.destroy_node()
            rclpy.shutdown()

    return _RclpyServiceClient()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Spawn/clear a GPSR command's scene in the sim.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    plan_p = sub.add_parser("plan", help="plan a scene from a command (no ROS)")
    plan_p.add_argument("--command", required=True)
    plan_p.add_argument("--seed", type=int, required=True)
    plan_p.add_argument("--out", required=True)
    plan_p.add_argument("--constants", default=str(DEFAULT_CONSTANTS))
    plan_p.add_argument("--placements", default=str(DEFAULT_PLACEMENTS))

    apply_p = sub.add_parser("apply", help="spawn a plan's items via /spawn_entity")
    apply_p.add_argument("--plan", required=True)
    apply_p.add_argument("--manifest", required=True)
    apply_p.add_argument("--base-scenario", default=str(DEFAULT_BASE_SCENARIO))
    apply_p.add_argument("--placements", default=str(DEFAULT_PLACEMENTS))

    clear_p = sub.add_parser("clear", help="delete a manifest's spawned entities via /delete_entity")
    clear_p.add_argument("--manifest", required=True)

    emit_p = sub.add_parser("emit-scenario", help="merge plans into a scenario file (no ROS)")
    emit_p.add_argument("--plans", nargs="+", required=True)
    emit_p.add_argument("--base", default=str(DEFAULT_BASE_SCENARIO))
    emit_p.add_argument("--placements", default=str(DEFAULT_PLACEMENTS))
    emit_p.add_argument("--out", required=True)

    return parser


def _load_json_or_none(path: str) -> Optional[dict]:
    p = Path(path)
    if not p.is_file():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None


def _cmd_plan(args: argparse.Namespace) -> int:
    try:
        knowledge = json.loads(Path(args.constants).read_text(encoding="utf-8"))
        placements = json.loads(Path(args.placements).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"gpsr_spawn: bad plan inputs: {exc}", file=sys.stderr)
        return 3
    plan = plan_scene(args.command, knowledge, placements, seed=args.seed)
    data = scene_plan_to_json(plan, command_text=args.command, seed=args.seed)
    Path(args.out).write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    return 0


def _cmd_apply(args: argparse.Namespace) -> int:
    try:
        plan_data = json.loads(Path(args.plan).read_text(encoding="utf-8"))
        base_scenario = json.loads(Path(args.base_scenario).read_text(encoding="utf-8"))
        placements = json.loads(Path(args.placements).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"gpsr_spawn: bad plan inputs: {exc}", file=sys.stderr)
        return 3
    plan = scene_plan_from_json(plan_data)
    previous_manifest = _load_json_or_none(args.manifest)
    try:
        client = _make_ros_service_client()
    except Exception as exc:  # noqa: BLE001
        print(f"gpsr_spawn: service client unavailable: {exc}", file=sys.stderr)
        return 2
    try:
        manifest = apply_plan(plan, client, base_scenario=base_scenario, placements=placements,
                              previous_manifest=previous_manifest)
    finally:
        client.shutdown()
    Path(args.manifest).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    if any(not e.get("ok", True) for e in manifest["entities"]):
        return 2
    return 0


def _cmd_clear(args: argparse.Namespace) -> int:
    manifest = _load_json_or_none(args.manifest)
    if manifest is None:
        print(f"gpsr_spawn: bad manifest: {args.manifest}", file=sys.stderr)
        return 3
    try:
        client = _make_ros_service_client()
    except Exception as exc:  # noqa: BLE001
        print(f"gpsr_spawn: service client unavailable: {exc}", file=sys.stderr)
        return 2
    try:
        updated = clear_manifest(manifest, client)
    finally:
        client.shutdown()
    Path(args.manifest).write_text(json.dumps(updated, indent=2) + "\n", encoding="utf-8")
    if any(not e.get("cleared", True) for e in updated["entities"]):
        return 2
    return 0


def _cmd_emit_scenario(args: argparse.Namespace) -> int:
    try:
        plans = [scene_plan_from_json(json.loads(Path(p).read_text(encoding="utf-8")))
                for p in args.plans]
        base_scenario = json.loads(Path(args.base).read_text(encoding="utf-8"))
        placements = json.loads(Path(args.placements).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"gpsr_spawn: bad plan inputs: {exc}", file=sys.stderr)
        return 3
    merged = emit_scenario(plans, base_scenario, placements)
    Path(args.out).write_text(json.dumps(merged, indent=2) + "\n", encoding="utf-8")
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    if args.cmd == "plan":
        return _cmd_plan(args)
    if args.cmd == "apply":
        return _cmd_apply(args)
    if args.cmd == "clear":
        return _cmd_clear(args)
    if args.cmd == "emit-scenario":
        return _cmd_emit_scenario(args)
    return 3  # pragma: no cover - argparse `required=True` prevents this


if __name__ == "__main__":
    sys.exit(main())
