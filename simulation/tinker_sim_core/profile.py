from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping


BEHAVIOR_PROFILES = (
    "physics-only",
    "sensor-rich",
    "streaming",
    "navigation-parity",
    "manipulation-core",
    "manipulation-sensor",
    "manipulation-cumotion",
    "vision-head",
    "task-rich",
    "vla-eval",
)


@dataclass(frozen=True)
class SimulationProfile:
    name: str
    physics_device: str
    rendering: bool
    modules: tuple[str, ...]
    raw: Mapping[str, Any]

    @classmethod
    def load(cls, root: Path, name: str) -> "SimulationProfile":
        path = root / "simulation" / "profiles" / f"{name}.json"
        raw = json.loads(path.read_text(encoding="utf-8"))
        physics_device = raw.get("physics_device")
        if name in BEHAVIOR_PROFILES and physics_device != "cpu":
            raise ValueError(
                f"behavior profile {name} must use CPU physics, got {physics_device!r}"
            )
        modules = raw.get("modules", [])
        if not isinstance(modules, list) or any(not isinstance(item, str) for item in modules):
            raise ValueError(f"{path}: modules must be a string array")
        return cls(
            name=name,
            physics_device=str(physics_device),
            rendering=bool(raw.get("render", False)),
            modules=tuple(modules),
            raw=raw,
        )
