from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@dataclass(frozen=True)
class Config:
    root: Path
    raw: dict[str, Any]

    @classmethod
    def load(cls, root: Path | None = None) -> "Config":
        resolved_root = (root or PROJECT_ROOT).resolve()
        with (resolved_root / "deployment.json").open(encoding="utf-8") as stream:
            return cls(root=resolved_root, raw=json.load(stream))

    @property
    def platform(self) -> dict[str, Any]:
        return self.raw["platform"]

    @property
    def runtime(self) -> dict[str, str]:
        return self.raw["runtime"]

    @property
    def ros(self) -> dict[str, Any]:
        return self.raw["ros"]

    def path(self, name: str) -> Path:
        return (self.root / self.raw["paths"][name]).resolve()

    @property
    def lab_commit(self) -> str:
        return self.runtime["isaac_lab_commit"]

    @property
    def uv_minor(self) -> tuple[int, int]:
        major, minor, *_ = self.runtime["uv"].split(".")
        return int(major), int(minor)

    def dds_profile(self, name: str) -> Path | None:
        profiles = self.ros["dds_profiles"]
        if name not in profiles:
            raise RuntimeError(f"unknown DDS profile: {name}")
        value = profiles[name]
        return None if value is None else (self.root / value).resolve()
