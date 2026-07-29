from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping


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

