from __future__ import annotations

import json
from pathlib import Path

from .config import Config, sha256_file


def _asset_path(root: Path, value: object) -> Path:
    if not isinstance(value, str) or not value or "\\" in value:
        raise RuntimeError("asset path must be a relative POSIX path")
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        raise RuntimeError(f"unsafe asset path: {value}")
    candidate = (root / relative).resolve(strict=False)
    try:
        candidate.relative_to(root.resolve())
    except ValueError as error:
        raise RuntimeError(f"asset path escapes project root: {value}") from error
    return root / relative


def verify_assets(config: Config) -> dict[str, object]:
    root = config.root.resolve()
    manifest_path = config.path("artifacts") / "asset-manifest.json"
    if manifest_path.is_symlink() or not manifest_path.is_file():
        raise RuntimeError("artifacts/asset-manifest.json is required; populate it from the example")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    groups = ("generated_robot_usds", "warmed_isaac_assets")
    for group in groups:
        entries = manifest.get(group)
        if not isinstance(entries, list) or not entries:
            raise RuntimeError(f"asset manifest group must be non-empty: {group}")
        for entry in entries:
            if not isinstance(entry, dict) or set(entry) != {"path", "sha256"}:
                raise RuntimeError(f"asset manifest entry is invalid: {group}")
            path = _asset_path(root, entry["path"])
            if path.is_symlink() or not path.is_file():
                raise RuntimeError(f"asset is missing or symlinked: {entry['path']}")
            actual = sha256_file(path)
            if actual != entry["sha256"]:
                raise RuntimeError(f"asset checksum mismatch: {entry['path']}")
    return manifest
