"""Source lock for a design: every file under designs/<name>/ plus resolved upstream inputs.

`_label` records an upstream file outside `repo_root` (e.g. a `package://`-resolved mesh) by
its absolute path -- these design source-locks are record-only provenance, never validated by
`tinker_sim_deploy.workspace._validate_lock_records` (which rejects absolute paths; it only
validates the tinker2 export's source lock)."""
from __future__ import annotations

import hashlib
from collections.abc import Sequence
from pathlib import Path

from tinker_sim_deploy.workspace import UnsafePathError, normalized_source_lock


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
    by_label: dict[str, dict[str, object]] = {}

    def _add(path: Path, label: str) -> None:
        record = _record(path, label)
        existing = by_label.get(label)
        if existing is not None:
            if existing["sha256"] != record["sha256"]:
                raise UnsafePathError(f"design source path collision with mismatched content: {label}")
            return
        by_label[label] = record

    for path in sorted(Path(design_dir).rglob("*")):
        if path.is_symlink():
            raise UnsafePathError(f"design source is a symlink: {path}")
        if path.is_file():
            _add(path, _label(path, repo_root))
    for path in extra_files:
        _add(Path(path), _label(Path(path), repo_root))
    return sorted(by_label.values(), key=lambda item: str(item["path"]))


def design_source_lock(design_dir: Path, repo_root: Path, extra_files: Sequence[Path]) -> bytes:
    return normalized_source_lock(design_records(design_dir, repo_root, extra_files), robot=Path(design_dir).name)
