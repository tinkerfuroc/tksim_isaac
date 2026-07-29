from __future__ import annotations

import json
from pathlib import Path

from .config import Config, sha256_file


def verify_assets(config: Config) -> dict[str, object]:
    manifest_path = config.path("artifacts") / "asset-manifest.json"
    if not manifest_path.is_file():
        raise RuntimeError(
            "artifacts/asset-manifest.json is required; populate it from the example"
        )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    groups = ("generated_robot_usds", "warmed_isaac_assets")
    for group in groups:
        entries = manifest.get(group)
        if not isinstance(entries, list) or not entries:
            raise RuntimeError(f"asset manifest group must be non-empty: {group}")
        for entry in entries:
            relative = Path(entry["path"])
            if relative.is_absolute() or ".." in relative.parts:
                raise RuntimeError(f"unsafe asset path: {relative}")
            path = config.root / relative
            if not path.is_file():
                raise RuntimeError(f"asset is missing: {relative}")
            actual = sha256_file(path)
            if actual != entry["sha256"]:
                raise RuntimeError(f"asset checksum mismatch: {relative}")
    return manifest
