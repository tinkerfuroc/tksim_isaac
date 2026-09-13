"""Source lock for a design: every file under designs/<name>/ plus resolved upstream inputs."""
from __future__ import annotations

import hashlib
from collections.abc import Sequence
from pathlib import Path

from tinker_sim_deploy.workspace import UnsafePathError, _normalized_source_lock


def _record(path: Path, label: str) -> dict[str, object]:
    if path.is_symlink():
        raise UnsafePathError(f"design source is a symlink: {path}")
    data = path.read_bytes()
    return {"path": label, "size": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def _label(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return str(path.resolve())


def design_records(design_dir: Path, repo_root: Path, extra_files: Sequence[Path]) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    for path in sorted(Path(design_dir).rglob("*")):
        if path.is_symlink():
            raise UnsafePathError(f"design source is a symlink: {path}")
        if path.is_file():
            records.append(_record(path, _label(path, repo_root)))
    for path in extra_files:
        records.append(_record(Path(path), _label(Path(path), repo_root)))
    records.sort(key=lambda item: str(item["path"]))
    return records


def design_source_lock(design_dir: Path, repo_root: Path, extra_files: Sequence[Path]) -> bytes:
    return _normalized_source_lock(design_records(design_dir, repo_root, extra_files), robot=Path(design_dir).name)
