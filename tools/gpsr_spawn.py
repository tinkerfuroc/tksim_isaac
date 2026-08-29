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
import dataclasses
import json
import math
import os
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Callable, Optional, Protocol, Sequence

# The bench invokes this file as a standalone script from another repo
# (`python3 /abs/path/tools/gpsr_spawn.py ...`), with no PYTHONPATH help --
# `from tools.gpsr_scene import ...` below would otherwise raise
# `ModuleNotFoundError: No module named 'tools'` in that mode (it only
# resolves under pytest, which runs from the repo root with the repo root
# already on sys.path). Guarded so this is a no-op -- and does not disturb
# import order/caching -- when the repo root is already on sys.path, e.g.
# under pytest.
_REPO_ROOT_FOR_IMPORT = str(Path(__file__).resolve().parent.parent)
if _REPO_ROOT_FOR_IMPORT not in sys.path:
    sys.path.insert(0, _REPO_ROOT_FOR_IMPORT)

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


class ServiceUnavailable(RuntimeError):
    """Raised by the real ROS service client (see `_make_ros_service_client`)
    when a simulation_interfaces service is missing at construction time, or
    when a spawn/delete call's `spin_until_future_complete` times out.
    `apply_plan` treats this specially: unlike an ordinary per-item spawn
    failure (the item itself is bad), a `ServiceUnavailable` means the sim
    is presumed down for the rest of the run, so the plan's remaining items
    are not attempted at all rather than tried and failed one by one.
    """


class ServiceClient(Protocol):
    def spawn(self, item: SpawnItem) -> str:
        """Spawn `item`; return its entity name, or raise on failure.
        Raise `ServiceUnavailable` specifically for an infra-level failure
        (missing service, call timeout); any other exception is treated as
        an ordinary per-item failure."""
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


def _base_scenario_counts(base_scenario: dict, placements: dict) -> tuple[Counter, Counter]:
    """(key_counts, spot_counts).

    `key_counts[(asset_key, where)]` is how many base-scenario records
    (actors + objects) resolve to that exact key -- the count-aware form
    of `base_scenario_keys`, used to dedupe a plan against the base
    scenario by count, not just presence (a plan wanting 3 of an asset
    that's already got 1 in the base scenario should still spawn 2).

    `spot_counts[spot]` is how many base-scenario OBJECT records (any
    asset -- persons/rooms are excluded, they are never grid-sloted) sit
    at that placement `spot`. Used to grid-slot newly-spawned items so
    they never land on the same xyz as a pre-existing base object,
    regardless of whether that object is the same asset or not.
    """
    key_counts: Counter = Counter()
    spot_counts: Counter = Counter()
    spots = placements.get("spots", {})
    for record in base_scenario.get("actors", []):
        asset = _asset_key(record.get("asset_uri", ""))
        xyz = record.get("pose", {}).get("xyz", [0.0, 0.0, 0.0])
        where = _nearest_spot_or_room(xyz, placements)
        if where is not None:
            key_counts[(asset, where)] += 1
    for record in base_scenario.get("objects", []):
        asset = _asset_key(record.get("asset_uri", ""))
        xyz = record.get("pose", {}).get("xyz", [0.0, 0.0, 0.0])
        where = _nearest_spot_or_room(xyz, placements)
        if where is not None:
            key_counts[(asset, where)] += 1
            if where in spots:
                spot_counts[where] += 1
    return key_counts, spot_counts


def base_scenario_keys(base_scenario: dict, placements: dict) -> set[tuple[str, str]]:
    """(asset_key, spot-or-room) already present in the base scenario,
    derived by matching each spawned pose against the placement table
    (not hardcoded, so it stays correct if the base scenario changes).
    """
    key_counts, _spot_counts = _base_scenario_counts(base_scenario, placements)
    return set(key_counts)


def _grid_slot_xyz(where: str, k: int, placements: dict) -> Optional[tuple[float, float, float]]:
    """The kth grid slot's xyz on placement spot `where` (`plan_scene`'s
    own (i % 3, i // 3) * (grid_dx, grid_dy) formula, so re-sloted items
    land on the same grid a fresh plan would have used). None if `where`
    is not a known placement spot (e.g. it's a person room, or unknown --
    callers keep the item's original xyz in that case).
    """
    spot_info = placements.get("spots", {}).get(where)
    if spot_info is None:
        return None
    surface_xyz = spot_info["surface_xyz"]
    grid_dx = spot_info["grid_dx"]
    grid_dy = spot_info["grid_dy"]
    return (
        surface_xyz[0] + (k % 3) * grid_dx,
        surface_xyz[1] + (k // 3) * grid_dy,
        surface_xyz[2],
    )


def apply_plan(
    plan: ScenePlan,
    client: "ServiceClient",
    *,
    base_scenario: dict,
    placements: dict,
    previous_manifest: Optional[dict] = None,
    on_progress: Optional[Callable[[dict], None]] = None,
) -> dict:
    """Spawn every item in `plan` via `client`, count-aware-deduping against
    the base scenario and a previous manifest's *successful* entries: for
    each (asset, spot-or-room) key, `existing = (# base-scenario records
    with that key) + (# ok:True previous-manifest entities with that
    key)`; of the plan's items sharing that key (in plan order), the
    first `existing` are skipped as "already present" and the rest are
    spawned -- so e.g. a 3-instance counting command against a base
    scenario/previous manifest that already has 1 of that asset at that
    spot spawns the other 2, rather than treating the whole key as either
    fully present or fully absent. A previously FAILED entry (`"ok":
    False`) does not count toward `existing` -- it is retried, and its
    stale manifest record (matched by (asset_key, where), not item id,
    since a retried item may be replanned under a different id) is
    replaced by the fresh attempt's result.

    Every item actually spawned (not skipped) is re-sloted onto a grid
    cell of its placement spot before being handed to `client.spawn`: slot
    index `k = (# base-scenario objects, any asset, at that spot) + (# ok
    previous-manifest entities, any asset, at that spot) + (# items
    already spawned at that spot earlier in *this* apply_plan call)`,
    `xyz = surface_xyz + ((k % 3) * grid_dx, (k // 3) * grid_dy, 0)`. This
    overrides the plan's own xyz (the planner's default slot, which does
    not know about the base scenario or other items sharing its spot) so
    a spawned item never overlaps a pre-existing base object or another
    item spawned earlier in the same call. Persons (room keys, not
    placement spots) are never re-sloted. The re-sloted xyz is what's
    recorded in the manifest entity and what actually gets spawned.

    Never raises for an ordinary per-item spawn failure -- it is recorded
    in the returned manifest's entities with `"ok": False` and the loop
    continues to the next item. A `ServiceUnavailable` failure is
    different: it means the sim itself is down, not just this one item,
    so the loop stops immediately and every remaining item is recorded in
    `"not_attempted"` instead of being attempted.

    If `on_progress` is given, it is called with the manifest-so-far after
    every item (skipped, attempted, or not-attempted) -- e.g. so the CLI
    can rewrite the manifest file incrementally, leaving a usable on-disk
    manifest even if the run is killed mid-plan.
    """
    base_key_counts, base_spot_counts = _base_scenario_counts(base_scenario, placements)
    entities: list[dict] = []
    prev_ok_key_counts: Counter = Counter()
    prev_ok_spot_counts: Counter = Counter()
    prev_failed_by_key: dict[tuple, list] = defaultdict(list)
    if previous_manifest:
        for e in previous_manifest.get("entities", []):
            key = (e.get("asset_key"), e.get("where"))
            if e.get("ok"):
                prev_ok_key_counts[key] += 1
                if e.get("where") in placements.get("spots", {}):
                    prev_ok_spot_counts[e.get("where")] += 1
            else:
                prev_failed_by_key[key].append(e)
            entities.append(e)

    existing_key_counts: Counter = Counter(base_key_counts)
    existing_key_counts.update(prev_ok_key_counts)
    spot_counts_so_far: Counter = Counter(base_spot_counts)
    spot_counts_so_far.update(prev_ok_spot_counts)

    def _replace_stale(key: tuple[str, str]) -> None:
        # Pop (at most) one stale FAILED previous-manifest record for this
        # key, matched by identity (not equality -- two distinct failed
        # records could otherwise compare equal), and drop it from
        # `entities` in favour of the fresh attempt about to be appended.
        # OK previous records are never touched here -- they're already
        # counted in `existing_key_counts` and were never candidates for
        # a retry in the first place.
        queue = prev_failed_by_key.get(key)
        if not queue:
            return
        stale = queue.pop(0)
        for i, existing_entity in enumerate(entities):
            if existing_entity is stale:
                del entities[i]
                break

    plan_key_seen: Counter = Counter()
    skipped: list[dict] = []
    not_attempted: list[dict] = []

    def _report() -> None:
        if on_progress:
            on_progress({"entities": list(entities), "skipped": list(skipped),
                        "not_attempted": list(not_attempted)})

    service_down = False
    for item in plan.items:
        where = item.spot if item.spot else item.room
        asset_key = _asset_key(item.asset_uri)
        key = (asset_key, where)
        idx_in_key = plan_key_seen[key]
        plan_key_seen[key] += 1

        if service_down:
            not_attempted.append(
                {"id": item.id, "ok": False, "error": "not attempted: service unavailable"}
            )
            _report()
            continue

        if idx_in_key < existing_key_counts.get(key, 0):
            skipped.append({"id": item.id, "reason": "already present", "where": where})
            _report()
            continue

        spawn_item = item
        if item.spot and item.spot in placements.get("spots", {}):
            k = spot_counts_so_far[where]
            spot_counts_so_far[where] += 1
            new_xyz = _grid_slot_xyz(where, k, placements)
            if new_xyz is not None:
                spawn_item = dataclasses.replace(item, xyz=new_xyz)

        entity_name = f"/World/Scenario/{item.id}"
        pose_fields = {"xyz": list(spawn_item.xyz), "quaternion_xyzw": list(spawn_item.quaternion_xyzw)}
        try:
            actual = client.spawn(spawn_item)
        except ServiceUnavailable as exc:
            service_down = True
            _replace_stale(key)
            entities.append(
                {"id": item.id, "asset_key": asset_key, "where": where,
                 "entity_name": entity_name, "ok": False, "error": repr(exc), **pose_fields}
            )
            _report()
            continue
        except Exception as exc:  # noqa: BLE001 - never crash a bench run
            _replace_stale(key)
            entities.append(
                {"id": item.id, "asset_key": asset_key, "where": where,
                 "entity_name": entity_name, "ok": False, "error": repr(exc), **pose_fields}
            )
            _report()
            continue
        _replace_stale(key)
        entities.append(
            {"id": item.id, "asset_key": asset_key, "where": where,
             "entity_name": actual, "ok": True, **pose_fields}
        )
        _report()
    return {"entities": entities, "skipped": skipped, "not_attempted": not_attempted}


def clear_manifest(
    manifest: dict,
    client: "ServiceClient",
    *,
    on_progress: Optional[Callable[[dict], None]] = None,
) -> dict:
    """Delete every successfully-spawned entity in `manifest` via
    `client`. Tolerates NOT_FOUND (client.delete returning True). Never
    raises for a per-item delete failure. If `on_progress` is given, it is
    called with the manifest-so-far after every entity.

    Every top-level key of `manifest` other than `"entities"` (e.g.
    `"skipped"`, or a partial `apply`'s `"not_attempted"`) is passed
    through unchanged into both the return value and every `on_progress`
    snapshot -- `clear_manifest` only owns `"entities"`, so it must not
    silently drop fields it doesn't recognise.
    """
    results: list[dict] = []
    passthrough = {k: v for k, v in manifest.items() if k != "entities"}
    passthrough.setdefault("skipped", [])

    def _snapshot() -> dict:
        return {**passthrough, "entities": list(results)}

    def _report() -> None:
        if on_progress:
            on_progress(_snapshot())

    for e in manifest.get("entities", []):
        if not e.get("ok"):
            results.append(e)
            _report()
            continue
        # A re-run over a manifest whose earlier clear failed must not keep the old error
        # text next to a fresh "cleared": true.
        base = {k: v for k, v in e.items() if k != "clear_error"}
        try:
            deleted = _delete_with_one_retry(client, e["entity_name"])
        except Exception as exc:  # noqa: BLE001 - never crash a bench run
            results.append({**base, "cleared": False, "clear_error": repr(exc)})
            _report()
            continue
        results.append({**base, "cleared": bool(deleted)})
        _report()
    return _snapshot()


def _delete_with_one_retry(client: ServiceClient, entity: str) -> bool:
    """`client.delete(entity)`, retried once if the service was unavailable.

    Live 2026-08-29: the first delete_entity issued right after the orchestrator's
    teardown timed out once (sim busy), and the same call succeeded seconds later.
    Item-level failures (any other exception) are not retried.
    """
    try:
        return client.delete(entity)
    except ServiceUnavailable:
        return client.delete(entity)


def emit_scenario(plans: Sequence[ScenePlan], base_scenario: dict, placements: dict) -> dict:
    """Merge every plan's items into a COPY of base_scenario (base_scenario
    itself is never mutated). Count-aware dedupe against the base scenario,
    same rule as `apply_plan`: for each (asset, spot-or-room) key, the
    merge target is `max(plan count, existing base-scenario count)` -- so
    only the plan count's excess over what's already in the base scenario
    is actually appended (a plan wanting 3 of an asset the base scenario
    already has 1 of adds 2, not 0 and not 3). When two plans want the
    same key, the plan that proposed MORE instances of it wins outright
    (its whole item group is used, the smaller plan's group is dropped) --
    unchanged from before.

    Every appended item is re-sloted the same way `apply_plan` re-slots a
    spawn: grid slot `k = (# base-scenario objects, any asset, at that
    spot) + (# items already merged into that spot earlier in *this*
    emit_scenario call)`, so a merged item never lands on the same xyz as
    a pre-existing base object (of any asset) or another item merged
    earlier in the same call. Persons (room keys) are never re-sloted.
    """
    merged = json.loads(json.dumps(base_scenario))
    existing_key_counts, base_spot_counts = _base_scenario_counts(base_scenario, placements)

    best: dict[tuple[str, str], list[SpawnItem]] = {}
    for plan in plans:
        per_plan: dict[tuple[str, str], list[SpawnItem]] = {}
        for item in plan.items:
            where = item.spot if item.spot else item.room
            key = (_asset_key(item.asset_uri), where)
            per_plan.setdefault(key, []).append(item)
        for key, group in per_plan.items():
            if key not in best or len(group) > len(best[key]):
                best[key] = group

    spot_counts_so_far: Counter = Counter(base_spot_counts)
    for key, group in best.items():
        _asset_key_value, where = key
        existing_count = existing_key_counts.get(key, 0)
        to_add = group[existing_count:]
        if not to_add:
            continue
        target_list = merged["actors"] if group[0].kind == "person" else merged["objects"]
        for item in to_add:
            spawn_item = item
            if item.spot and item.spot in placements.get("spots", {}):
                k = spot_counts_so_far[where]
                spot_counts_so_far[where] += 1
                new_xyz = _grid_slot_xyz(where, k, placements)
                if new_xyz is not None:
                    spawn_item = dataclasses.replace(item, xyz=new_xyz)
            target_list.append(
                {
                    # Suffixed by `where` so two different (asset, where)
                    # groups across different plans can never collide on
                    # id even if their SpawnItem ids happened to match.
                    "id": f"{spawn_item.id}__{where}",
                    "asset_uri": spawn_item.asset_uri,
                    "pose": {"xyz": list(spawn_item.xyz),
                             "quaternion_xyzw": list(spawn_item.quaternion_xyzw)},
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
    Waits for BOTH `/spawn_entity` and `/delete_entity` (10s each) before
    returning, raising `ServiceUnavailable` naming whichever service never
    came up -- a client this module hands to `apply_plan`/`clear_manifest`
    should never be able to hit a service that was never actually there.
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
    for service_name, ros_client in (
        ("/spawn_entity", spawn_client),
        ("/delete_entity", delete_client),
    ):
        if not ros_client.wait_for_service(timeout_sec=10.0):
            node.destroy_node()
            rclpy.shutdown()
            raise ServiceUnavailable(f"{service_name} service unavailable after 10s")

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
                raise ServiceUnavailable(f"spawn_entity timed out for {item.id}")
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
                raise ServiceUnavailable(f"delete_entity timed out for {entity}")
            response = future.result()
            if response.result.result == Result.RESULT_NOT_FOUND:
                return True
            return response.result.result == Result.RESULT_OK

        def shutdown(self) -> None:
            node.destroy_node()
            rclpy.shutdown()

    return _RclpyServiceClient()


def _manifest_writer(path: str) -> Callable[[dict], None]:
    """An `on_progress` callback that rewrites `path` after every item: it
    writes to `<path>.tmp` then atomically `os.replace`s it over the real
    file, so a run killed mid-`apply`/`clear` still leaves a valid,
    complete-as-of-its-last-item manifest on disk -- never a half-written
    or corrupt one.
    """
    def _write(manifest: dict) -> None:
        tmp_path = f"{path}.tmp"
        Path(tmp_path).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        os.replace(tmp_path, path)

    return _write


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
                              previous_manifest=previous_manifest,
                              on_progress=_manifest_writer(args.manifest))
    finally:
        client.shutdown()
    Path(args.manifest).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    if manifest.get("not_attempted") or any(not e.get("ok", True) for e in manifest["entities"]):
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
        updated = clear_manifest(manifest, client, on_progress=_manifest_writer(args.manifest))
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
