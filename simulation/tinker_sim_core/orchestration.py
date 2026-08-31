from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from .scenario import ScenarioDefinition


@dataclass(frozen=True)
class ScenarioOperation:
    kind: str
    payload: Mapping[str, Any]


def standard_operations(
    root: Path,
    scenario: ScenarioDefinition,
    seed: int,
    *,
    spawn_while_playing: bool = False,
) -> tuple[ScenarioOperation, ...]:
    """Compile a scenario into standard simulation_interfaces operations.

    ``spawn_while_playing`` skips the boot-time ``reset_spawned`` and the
    ``SPAWN_READY`` stop, spawning every entity onto the still-playing
    first-run timeline. Measured 2026-08-31 (in-process overlap probe): a
    rigid body spawned mid-play on a never-stopped timeline collides fully
    with the robot's articulation links (261 N depenetration on
    left_finger), while after any timeline STOP -> PLAY cycle, newly
    spawned bodies pair only with static geometry and pass straight
    through the gripper -- which made every spawned object ungraspable
    (the live-manip referee-fallback root). The stop bracket is what puts
    the whole session into that poisoned regime, so opting out of it is
    the fix; it stays opt-in until the A/B baselines that assume the old
    boot sequence are re-cut. Requires a scenario with no world uri (a
    world load belongs on a stopped timeline).
    """
    if seed < 0 or seed > 2**64 - 1:
        raise ValueError("seed must fit uint64")
    operations: list[ScenarioOperation] = []
    world_uri = scenario.world.get("uri")
    if spawn_while_playing and world_uri:
        raise ValueError(
            "spawn_while_playing cannot load a world uri: world loads "
            "require a stopped timeline"
        )
    if world_uri:
        operations.append(
            ScenarioOperation("load_world", {"uri": _uri(root, world_uri)})
        )
    elif not spawn_while_playing:
        operations.append(ScenarioOperation("reset_spawned", {"scope": 4}))
    if not spawn_while_playing:
        # ResetSimulation may restart the timeline; reassert STOPPED before spawn.
        operations.append(
            ScenarioOperation(
                "set_simulation_state",
                {"state": 0, "boundary": "SPAWN_READY"},
            )
        )
    for record in (*scenario.actors, *scenario.objects):
        logical_id = record.get("id")
        if not isinstance(logical_id, str) or not logical_id:
            raise ValueError("every spawned entity requires a logical id")
        if "/" in logical_id or "\\" in logical_id:
            raise ValueError(f"entity id is not a stable path component: {logical_id!r}")
        asset_uri = record.get("asset_uri")
        if not isinstance(asset_uri, str) or not asset_uri:
            raise ValueError(f"entity {record.get('id')!r} requires asset_uri")
        pose = record.get("pose", {})
        xyz = pose.get("xyz", [0.0, 0.0, 0.0])
        xyzw = pose.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        if len(xyz) != 3 or len(xyzw) != 4:
            raise ValueError(f"entity {record['id']!r} has invalid pose")
        operations.append(
            ScenarioOperation(
                "spawn_entity",
                {
                    "logical_id": logical_id,
                    "name": f"/World/Scenario/{logical_id}",
                    "entity_namespace": "Scenario",
                    "prim_path": f"/World/Scenario/{logical_id}",
                    "uri": _uri(root, asset_uri),
                    "frame_id": str(pose.get("frame_id", "world")),
                    "xyz": tuple(float(value) for value in xyz),
                    "quaternion_xyzw": tuple(float(value) for value in xyzw),
                },
            )
        )
    final_operation: dict[str, Any] = {
        "state": 1,
        "boundary": "PHYSICS_READY",
        "seed": seed,
    }
    if scenario.declaration is not None:
        final_operation["scenario"] = {
            "id": scenario.scenario_id,
            "seed": seed,
            "declaration": dict(scenario.declaration),
        }
    if scenario.planning_scene is not None:
        final_operation["planning_scene"] = dict(scenario.planning_scene)
    if scenario.integrated is not None:
        final_operation["integrated"] = dict(scenario.integrated)
    operations.append(
        ScenarioOperation("set_simulation_state", final_operation)
    )
    return tuple(operations)


def _uri(root: Path, value: str) -> str:
    path = Path(value)
    if "://" in value:
        return value
    if not path.is_absolute():
        path = root / path
    resolved = path.resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"scenario resource not found: {resolved}")
    return str(resolved)
