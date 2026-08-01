from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

_FIXTURE_PREFIX = "sim_fixture/"
_TARGET_HANDOFF = "pick_and_place/object_mesh"
_HEX64 = re.compile(r"^(?!0{64}$)[0-9a-f]{64}$")
_PRIMITIVE_TYPES = {"box", "cylinder", "sphere"}


@dataclass(frozen=True)
class ScenarioDefinition:
    schema_version: int
    scenario_id: str
    world: Mapping[str, Any]
    robot: Mapping[str, Any]
    actors: tuple[Mapping[str, Any], ...]
    objects: tuple[Mapping[str, Any], ...]
    regions: tuple[Mapping[str, Any], ...]
    events: tuple[Mapping[str, Any], ...]
    dialogue: tuple[Mapping[str, Any], ...]
    postconditions: tuple[Mapping[str, Any], ...]
    planning_scene: Mapping[str, Any] | None = None

    @classmethod
    def load(cls, path: Path) -> "ScenarioDefinition":
        raw = json.loads(path.read_text(encoding="utf-8"))
        if raw.get("schema_version") != 2:
            raise ValueError(f"{path}: scenario schema_version must be 2")
        scenario_id = raw.get("id")
        if not isinstance(scenario_id, str) or not scenario_id:
            raise ValueError(f"{path}: scenario id must be a non-empty string")
        world = _mapping(raw, "world", path)
        robot = _mapping(raw, "robot", path)
        planning_scene = raw.get("planning_scene")
        if planning_scene is not None:
            _validate_planning_scene(planning_scene, path)
        result = cls(
            schema_version=2,
            scenario_id=scenario_id,
            world=world,
            robot=robot,
            actors=_records(raw, "actors", path),
            objects=_records(raw, "objects", path),
            regions=_records(raw, "regions", path),
            events=_records(raw, "events", path),
            dialogue=_records(raw, "dialogue", path),
            postconditions=_records(raw, "postconditions", path, required=True),
            planning_scene=planning_scene,
        )
        result._validate_identifiers(path)
        return result

    def _validate_identifiers(self, path: Path) -> None:
        for group_name, records in (
            ("actors", self.actors),
            ("objects", self.objects),
            ("regions", self.regions),
        ):
            identifiers = [record.get("id") for record in records]
            if any(not isinstance(value, str) or not value for value in identifiers):
                raise ValueError(f"{path}: every {group_name} record requires an id")
            if len(identifiers) != len(set(identifiers)):
                raise ValueError(f"{path}: duplicate {group_name} id")
        for event in self.events:
            if "at_sim_time" not in event and "trigger" not in event:
                raise ValueError(f"{path}: event requires at_sim_time or trigger")
        for item in self.dialogue:
            for key in ("endpoint", "actor", "outcome"):
                if key not in item:
                    raise ValueError(f"{path}: dialogue entry requires {key}")


def load_named_scenario(root: Path, name: str) -> ScenarioDefinition:
    if not name or "/" in name or "\\" in name or name in {".", ".."}:
        raise ValueError(f"unsafe scenario name: {name!r}")
    path = root / "simulation" / "scenarios" / f"{name}.json"
    if not path.is_file():
        raise FileNotFoundError(f"scenario not found: {name}")
    scenario = ScenarioDefinition.load(path)
    if scenario.scenario_id != name:
        raise ValueError(f"{path}: id does not match filename")
    return scenario


def _canonical_json(value: Mapping[str, Any]) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")


def _planning_scene_digest(planning_scene: Mapping[str, Any]) -> str:
    payload = {key: value for key, value in planning_scene.items() if key != "revision_digest"}
    return hashlib.sha256(_canonical_json(payload)).hexdigest()


def _finite_numbers(value: Any, length: int, label: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{label} must contain {length} numbers")
    result: list[float] = []
    for item in value:
        if isinstance(item, bool):
            raise ValueError(f"{label} entries must be finite numbers")
        try:
            number = float(item)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{label} entries must be numeric") from exc
        if not math.isfinite(number):
            raise ValueError(f"{label} entries must be finite")
        result.append(number)
    return result


def _validate_fixture_pose(record: Mapping[str, Any], path: Path, fixture_id: str) -> None:
    pose = record.get("pose")
    if not isinstance(pose, dict):
        raise ValueError(f"{path}: fixture {fixture_id} requires a pose object")
    _finite_numbers(pose.get("xyz"), 3, f"{fixture_id}.pose.xyz")
    _finite_numbers(pose.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0]), 4, f"{fixture_id}.pose.quaternion_xyzw")


def _validate_fixture_shape(record: Mapping[str, Any], path: Path, fixture_id: str) -> None:
    has_primitive = "primitive" in record
    has_mesh = "mesh" in record
    if has_primitive == has_mesh:
        raise ValueError(f"{path}: fixture {fixture_id} must declare exactly one of primitive or mesh")
    if has_primitive:
        primitive = record["primitive"]
        if not isinstance(primitive, dict):
            raise ValueError(f"{path}: fixture {fixture_id} primitive must be an object")
        primitive_type = primitive.get("type")
        if primitive_type not in _PRIMITIVE_TYPES:
            raise ValueError(f"{path}: fixture {fixture_id} has unsupported primitive type")
        dimensions = _finite_numbers(primitive.get("dimensions"), {"box": 3, "cylinder": 2, "sphere": 1}[primitive_type], f"{fixture_id}.primitive.dimensions")
        if any(value <= 0 for value in dimensions):
            raise ValueError(f"{path}: fixture {fixture_id} primitive dimensions must be positive")
    else:
        mesh = record["mesh"]
        if not isinstance(mesh, dict):
            raise ValueError(f"{path}: fixture {fixture_id} mesh must be an object")
        uri = mesh.get("uri")
        if not isinstance(uri, str) or not uri or ".." in uri:
            raise ValueError(f"{path}: fixture {fixture_id} mesh uri must be a non-empty path without traversal")
        digest = mesh.get("sha256")
        if not isinstance(digest, str) or not _HEX64.fullmatch(digest):
            raise ValueError(f"{path}: fixture {fixture_id} mesh sha256 must be a lowercase 64-hex digest")
        if "scale" in mesh:
            scale = _finite_numbers(mesh.get("scale"), 3, f"{fixture_id}.mesh.scale")
            if any(value <= 0 for value in scale):
                raise ValueError(f"{path}: fixture {fixture_id} mesh scale must be positive")


def _validate_planning_scene(planning_scene: Mapping[str, Any], path: Path) -> None:
    if not isinstance(planning_scene, dict):
        raise ValueError(f"{path}: planning_scene must be an object")
    revision = planning_scene.get("revision")
    if not isinstance(revision, str) or not revision:
        raise ValueError(f"{path}: planning_scene.revision must be a non-empty string")
    if planning_scene.get("frame_id") != "base_link":
        raise ValueError(f"{path}: planning_scene.frame_id must be base_link")
    declared_digest = planning_scene.get("revision_digest")
    if not isinstance(declared_digest, str) or not _HEX64.fullmatch(declared_digest):
        raise ValueError(f"{path}: planning_scene.revision_digest must be a lowercase 64-hex digest")
    if declared_digest != _planning_scene_digest(planning_scene):
        raise ValueError(f"{path}: planning_scene.revision_digest does not match the canonical digest")
    target_source_id = planning_scene.get("target_source_id")
    if not isinstance(target_source_id, str) or not target_source_id:
        raise ValueError(f"{path}: planning_scene.target_source_id must be a non-empty string")
    if planning_scene.get("target_handoff") != _TARGET_HANDOFF:
        raise ValueError(f"{path}: planning_scene.target_handoff must be {_TARGET_HANDOFF!r}")

    objects = planning_scene.get("objects")
    if not isinstance(objects, list) or not objects:
        raise ValueError(f"{path}: planning_scene.objects must be a non-empty array")
    diagnostics = planning_scene.get("diagnostic_objects", [])
    if not isinstance(diagnostics, list):
        raise ValueError(f"{path}: planning_scene.diagnostic_objects must be an array")

    fixture_ids: list[str] = []
    for record in [*objects, *diagnostics]:
        if not isinstance(record, dict):
            raise ValueError(f"{path}: every planning_scene fixture must be an object")
        fixture_id = record.get("id")
        if not isinstance(fixture_id, str) or not fixture_id.startswith(_FIXTURE_PREFIX):
            raise ValueError(f"{path}: fixture id must be prefixed {_FIXTURE_PREFIX!r}")
        fixture_ids.append(fixture_id)
        _validate_fixture_pose(record, path, fixture_id)
        _validate_fixture_shape(record, path, fixture_id)
    if len(fixture_ids) != len(set(fixture_ids)):
        raise ValueError(f"{path}: sim_fixture ids must be unique")
    if target_source_id not in fixture_ids:
        raise ValueError(f"{path}: planning_scene.target_source_id must name a declared fixture id")

    for record in diagnostics:
        if "enter_collision_bodies" not in record:
            raise ValueError(
                f"{path}: diagnostic {record.get('id')} must declare enter_collision_bodies"
            )
        if record.get("enter_collision_bodies") not in (True, False):
            raise ValueError(f"{path}: diagnostic {record.get('id')} enter_collision_bodies must be a boolean")


def _mapping(raw: Mapping[str, Any], key: str, path: Path) -> Mapping[str, Any]:
    value = raw.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"{path}: {key} must be an object")
    return value


def _records(
    raw: Mapping[str, Any], key: str, path: Path, *, required: bool = False
) -> tuple[Mapping[str, Any], ...]:
    value = raw.get(key, [])
    if not isinstance(value, list) or (required and not value):
        suffix = " and non-empty" if required else ""
        raise ValueError(f"{path}: {key} must be an array{suffix}")
    if any(not isinstance(record, dict) for record in value):
        raise ValueError(f"{path}: every {key} entry must be an object")
    return tuple(value)
