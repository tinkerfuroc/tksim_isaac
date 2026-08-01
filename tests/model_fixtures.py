"""Shared artifact-tree fixtures for the Task 3 model overlay tests.

These helpers build the legacy (unversioned pointer + schema-2 manifest) and
schema-4 publication shapes exercised by ``test_model_bundle.py``,
``test_model_preflight.py``, ``test_model_limits.py``, and
``test_current_artifact.py``.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

from tinker_sim_bridge.model_contract import sha256_file


def write_legacy_current(root: Path, artifact_id: str, urdf_bytes: bytes) -> Path:
    """Write a minimal valid legacy artifact tree under *root*.

    Returns the absolute selected ``robot.urdf`` path.
    """
    artifact_dir = root / "artifacts" / "robot" / "tinker2" / artifact_id
    artifact_dir.mkdir(parents=True, exist_ok=True)
    urdf_path = artifact_dir / "robot.urdf"
    urdf_path.write_bytes(urdf_bytes)
    digest = hashlib.sha256(urdf_bytes).hexdigest()
    manifest = {
        "schema_version": 2,
        "robot": "tinker2",
        "artifact_id": artifact_id,
        "canonicalization": {
            "algorithm": "tinker2-urdf-canonical-v1",
            "output_sha256": digest,
            "source_path": "src/tk26_sim/_generated/tinker_full.full.urdf",
            "source_sha256": "0" * 64,
        },
        "files": [
            {
                "path": "artifacts/robot/tinker2/{}/robot.urdf".format(artifact_id),
                "sha256": digest,
            }
        ],
        "source_lock": "artifacts/provenance/tinker2-source-lock.json",
    }
    (artifact_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8")
    current = {
        "artifact_id": artifact_id,
        "manifest": "artifacts/robot/tinker2/{}/manifest.json".format(artifact_id),
    }
    (root / "artifacts" / "robot" / "tinker2" / "current.json").write_text(
        json.dumps(current, indent=2, sort_keys=True), encoding="utf-8"
    )
    return urdf_path
