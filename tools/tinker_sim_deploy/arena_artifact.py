"""Content-addressed publication of arena/objects asset artifacts.

This is the trust anchor for the arena and objects importer CLIs: every
byte an importer publishes is identified purely by the sha256 of its
content (never a timestamp or a counter), so re-running an importer against
unchanged inputs is a no-op and any downstream consumer can independently
verify what it is looking at.

The staging/lock/pointer choreography mirrors the Tinker 2 robot artifact
exporter in ``workspace.py`` (see ``export_tinker2`` /
``_export_tinker2_locked``):

1. Acquire an exclusive, per-``(kind, asset_id)`` publication lock so
   concurrent publishers cannot interleave.
2. Recover (delete) any staging directory left behind by a prior crash.
3. If the content-addressed destination directory does not already exist,
   stage the full payload plus ``source-lock.json`` and ``manifest.json``
   under a fresh ``.artifact-stage-*`` directory (the prefix
   ``_recover_staging`` recognizes), fsync every file via
   ``_atomic_write``, then atomically rename the staging directory into
   its final, immutable, content-addressed home.
4. Only after that immutable directory exists do we atomically replace
   ``current.json`` to point at it.

A crash between steps 3 and 4 can leave an *orphan* artifact directory
(fully written, internally self-consistent, just not yet referenced by the
pointer) but can never leave a half-written directory or a pointer that
references missing content -- the pointer only ever names a directory that
was already durably committed to disk before the pointer write began.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from .workspace import (
    _atomic_write,
    _fsync_directory,
    _publication_lock,
    _recover_staging,
)

ALGORITHM_VERSION = "arena-artifact-1"

_RESERVED_PAYLOAD_NAMES = {"manifest.json", "source-lock.json", "current.json"}


class AssetArtifactError(RuntimeError):
    """Raised when an asset artifact cannot be published or validated safely."""


@dataclass(frozen=True)
class AssetPublication:
    artifact_dir: Path
    identity: str
    created: bool


def _canonical(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _validate_segment(value: str, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise AssetArtifactError(f"{label} must be a non-empty string: {value!r}")
    if value in {".", ".."} or "/" in value or "\\" in value or "\x00" in value:
        raise AssetArtifactError(f"{label} is not a safe path segment: {value!r}")
    return value


def _validate_payload_name(name: object) -> str:
    if not isinstance(name, str) or not name:
        raise AssetArtifactError(f"invalid payload path: {name!r}")
    if name in _RESERVED_PAYLOAD_NAMES:
        raise AssetArtifactError(f"reserved payload path: {name!r}")
    if name.startswith("/") or "\\" in name or "\x00" in name:
        raise AssetArtifactError(f"invalid payload path: {name!r}")
    path = Path(name)
    if path.is_absolute():
        raise AssetArtifactError(f"invalid payload path: {name!r}")
    parts = path.parts
    if not parts or ".." in parts or any(part in {"", "."} for part in parts):
        raise AssetArtifactError(f"invalid payload path: {name!r}")
    return name


def publish_asset_artifact(
    repo_root: Path,
    *,
    kind: str,
    asset_id: str,
    payload: Mapping[str, bytes],
    source_lock: Mapping[str, object],
) -> AssetPublication:
    repo_root = Path(repo_root)
    _validate_segment(kind, "kind")
    _validate_segment(asset_id, "asset_id")
    if not payload:
        raise AssetArtifactError("payload must not be empty")
    for name, data in payload.items():
        _validate_payload_name(name)
        if not isinstance(data, (bytes, bytearray)):
            raise AssetArtifactError(f"payload for {name!r} must be bytes, got {type(data).__name__}")

    artifact_root = repo_root / "artifacts" / kind / asset_id
    artifact_root.mkdir(parents=True, exist_ok=True)

    lock_bytes = _canonical(source_lock)
    payload_hashes = {name: hashlib.sha256(bytes(data)).hexdigest() for name, data in payload.items()}
    identity = hashlib.sha256(
        _canonical(
            {
                "algorithm": ALGORITHM_VERSION,
                "kind": kind,
                "id": asset_id,
                "payload": payload_hashes,
                "source_lock_sha256": hashlib.sha256(lock_bytes).hexdigest(),
            }
        )
    ).hexdigest()
    manifest_bytes = _canonical(
        {
            "schema_version": 1,
            "kind": kind,
            "id": asset_id,
            "identity": identity,
            "algorithm": ALGORITHM_VERSION,
            "payload": payload_hashes,
        }
    )
    pointer_bytes = _canonical(
        {
            "schema_version": 1,
            "manifest": f"artifacts/{kind}/{asset_id}/{identity}/manifest.json",
        }
    )

    with _publication_lock(artifact_root):
        _recover_staging(artifact_root)
        final_dir = artifact_root / identity
        created = not final_dir.is_dir()
        if created:
            stage = Path(tempfile.mkdtemp(prefix=".artifact-stage-", dir=str(artifact_root)))
            try:
                for name, data in payload.items():
                    target = stage / name
                    target.parent.mkdir(parents=True, exist_ok=True)
                    _atomic_write(target, bytes(data))
                _atomic_write(stage / "source-lock.json", lock_bytes)
                _atomic_write(stage / "manifest.json", manifest_bytes)
                os.rename(stage, final_dir)
            except BaseException:
                shutil.rmtree(stage, ignore_errors=True)
                raise
            _fsync_directory(artifact_root)
        _atomic_write(artifact_root / "current.json", pointer_bytes)

    return AssetPublication(artifact_dir=final_dir, identity=identity, created=created)


def verify_asset_artifact(artifact_dir: Path) -> list[str]:
    """Independently re-derive identity from on-disk bytes and report every mismatch.

    Returns an empty list when the artifact directory is fully self-consistent:
    every payload file's bytes hash to what ``manifest.json`` recorded, the
    recomputed identity (from the manifest's own declared payload hashes plus
    the on-disk ``source-lock.json`` bytes) matches ``manifest.json``'s
    ``identity`` field, and the directory name equals that identity.
    """
    artifact_dir = Path(artifact_dir)
    errors: list[str] = []

    manifest_path = artifact_dir / "manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text())
    except FileNotFoundError:
        return [f"missing manifest.json in {artifact_dir}"]
    except json.JSONDecodeError as error:
        return [f"manifest.json is not valid JSON: {error}"]

    if not isinstance(manifest, dict):
        return ["manifest.json does not contain a JSON object"]

    payload_hashes = manifest.get("payload")
    if not isinstance(payload_hashes, dict):
        return ["manifest.json is missing a 'payload' object"]

    lock_path = artifact_dir / "source-lock.json"
    try:
        lock_bytes = lock_path.read_bytes()
    except FileNotFoundError:
        errors.append(f"missing source-lock.json in {artifact_dir}")
        lock_bytes = b""

    for name, expected_hash in payload_hashes.items():
        target = artifact_dir / name
        try:
            data = target.read_bytes()
        except FileNotFoundError:
            errors.append(f"missing payload file: {name}")
            continue
        actual_hash = hashlib.sha256(data).hexdigest()
        if actual_hash != expected_hash:
            errors.append(
                f"payload hash mismatch for {name}: manifest says {expected_hash}, on-disk is {actual_hash}"
            )

    recomputed_identity = hashlib.sha256(
        _canonical(
            {
                "algorithm": manifest.get("algorithm"),
                "kind": manifest.get("kind"),
                "id": manifest.get("id"),
                "payload": payload_hashes,
                "source_lock_sha256": hashlib.sha256(lock_bytes).hexdigest(),
            }
        )
    ).hexdigest()

    manifest_identity = manifest.get("identity")
    if recomputed_identity != manifest_identity:
        errors.append(
            f"identity mismatch: manifest.json claims {manifest_identity!r}, "
            f"recomputed {recomputed_identity!r}"
        )

    if artifact_dir.name != recomputed_identity:
        errors.append(
            f"directory name {artifact_dir.name!r} does not match recomputed identity {recomputed_identity!r}"
        )

    return errors


def attribution_markdown(sections: Sequence[tuple[str, str]]) -> bytes:
    """Deterministic Markdown concatenation of (title, body) attribution sections."""
    return "\n".join(f"## {title}\n\n{body.rstrip()}\n" for title, body in sections).encode("utf-8")
