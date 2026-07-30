from __future__ import annotations

import hashlib
import json
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from .workspace import (
    ARM_JOINTS,
    ARTIFACT_FILES,
    CANONICALIZER_ALGORITHM,
    PUBLICATION_SCHEMA,
    UnsafePathError,
    _contained,
    _load_json_bytes,
    _path_parts_are_safe,
    _safe_dir,
    _safe_file_bytes,
    _safe_relative,
    _validate_lock_records,
    artifact_identity,
    validate_canonical_urdf,
)


@dataclass(frozen=True)
class ArtifactResolution:
    project_root: Path
    artifact_dir: Path
    current: dict[str, object]
    manifest: dict[str, object]
    source_lock: dict[str, object]
    robot_urdf: bytes


def _hash(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _artifact_root(project_root: Path) -> Path:
    root = _safe_dir(project_root, "project root")
    artifacts = _safe_dir(root / "artifacts", "artifacts root")
    robot_root = _safe_dir(artifacts / "robot", "robot artifact root")
    return _safe_dir(robot_root / "tinker2", "Tinker 2 artifact root")


def _validate_manifest(root: Path, artifact_dir: Path, current: dict[str, object], manifest_bytes: bytes, manifest: dict[str, object], lock_bytes: bytes, lock: dict[str, object]) -> bytes:
    if manifest.get("schema_version") != PUBLICATION_SCHEMA:
        raise RuntimeError("artifact manifest schema is unsupported")
    artifact_id = manifest.get("artifact_id")
    if not isinstance(artifact_id, str) or artifact_dir.name != artifact_id:
        raise RuntimeError("artifact_id does not match manifest and directory")
    if current.get("manifest_sha256") != _hash(manifest_bytes):
        raise RuntimeError("current manifest hash mismatch")
    if current.get("source_lock_sha256") != _hash(lock_bytes):
        raise RuntimeError("current source-lock hash mismatch")
    source_lock_path = manifest.get("source_lock")
    if not isinstance(source_lock_path, str):
        raise RuntimeError("manifest source_lock is missing")
    expected_lock = f"artifacts/robot/tinker2/{artifact_id}/source-lock.json"
    if source_lock_path != expected_lock:
        raise RuntimeError("manifest source_lock is not the immutable per-artifact lock")
    if current.get("source_lock") != source_lock_path:
        raise RuntimeError("current and manifest source locks differ")
    _validate_lock_records(lock)
    if lock.get("schema_version") != 2 or "source_identity" not in lock:
        raise RuntimeError("immutable source lock schema is unsupported")
    source_identity = lock.get("source_identity")
    if not isinstance(source_identity, dict) or not isinstance(source_identity.get("repository"), str) or not isinstance(source_identity.get("revision"), str):
        raise RuntimeError("source lock identity is not portable")
    records = manifest.get("files")
    expected_paths = {f"artifacts/robot/tinker2/{artifact_id}/{name}" for name in ARTIFACT_FILES}
    actual_paths = [item.get("path") for item in records] if isinstance(records, list) and all(isinstance(item, dict) for item in records) else []
    if not isinstance(records, list) or len(records) != len(ARTIFACT_FILES) or len(set(actual_paths)) != len(actual_paths) or set(actual_paths) != expected_paths:
        raise RuntimeError("manifest payload set is incomplete, duplicated, or contains foreign files")
    payload_hashes: dict[str, str] = {}
    for record in records:
        if not isinstance(record, dict):
            raise RuntimeError("manifest contains an invalid file record")
        path_value = _safe_relative(record.get("path"), "manifest file path")
        path = _contained(root, path_value, "manifest file path")
        try:
            path.relative_to(artifact_dir)
        except ValueError as error:
            raise RuntimeError("manifest file escapes its artifact directory") from error
        if path.name not in ARTIFACT_FILES or not isinstance(record.get("sha256"), str):
            raise RuntimeError("manifest file record is invalid")
        data = _safe_file_bytes(path, f"artifact payload {path.name}")
        if _hash(data) != record["sha256"]:
            raise RuntimeError(f"artifact payload hash mismatch: {path}")
        payload_hashes[path.name] = record["sha256"]
    canonicalization = manifest.get("canonicalization")
    provenance = manifest.get("provenance")
    if not isinstance(canonicalization, dict) or canonicalization.get("algorithm") != CANONICALIZER_ALGORITHM:
        raise RuntimeError("artifact canonicalizer provenance is invalid")
    if canonicalization.get("output_sha256") != payload_hashes["robot.urdf"]:
        raise RuntimeError("canonical URDF provenance hash mismatch")
    if not isinstance(provenance, dict) or provenance.get("source_lock_sha256") != _hash(lock_bytes):
        raise RuntimeError("artifact source-lock provenance mismatch")
    expected_identity = artifact_identity(payload_hashes, _safe_file_bytes(artifact_dir / "robot.urdf", "canonical URDF"), lock_bytes, CANONICALIZER_ALGORITHM)
    if expected_identity != artifact_id:
        raise RuntimeError("artifact identity does not bind its immutable contents")
    urdf = _safe_file_bytes(artifact_dir / "robot.urdf", "canonical URDF")
    validate_canonical_urdf(urdf)
    if current.get("robot_urdf_sha256") != payload_hashes["robot.urdf"] or current.get("robot_usd_sha256") != payload_hashes["robot.usd"]:
        raise RuntimeError("current payload hashes do not match manifest")
    return urdf


def resolve_current_artifact(project_root: Path) -> ArtifactResolution:
    root = _safe_dir(project_root, "project root")
    artifact_root = _artifact_root(root)
    current_path = artifact_root / "current.json"
    current_bytes, current = _load_json_bytes(current_path, "current artifact pointer")
    if current.get("schema_version") != PUBLICATION_SCHEMA:
        raise RuntimeError("current artifact pointer schema is unsupported")
    artifact_id = current.get("artifact_id")
    artifact_dir_value = current.get("artifact_dir")
    manifest_value = current.get("manifest")
    if not isinstance(artifact_id, str) or len(artifact_id) != 64 or any(c not in "0123456789abcdef" for c in artifact_id):
        raise RuntimeError("current artifact_id is not a full SHA-256 identity")
    if not isinstance(artifact_dir_value, str) or artifact_dir_value != f"artifacts/robot/tinker2/{artifact_id}":
        raise RuntimeError("current artifact directory identity is invalid")
    artifact_dir = _contained(root, artifact_dir_value, "current artifact directory")
    _safe_dir(artifact_dir, "selected artifact directory")
    if not isinstance(manifest_value, str) or manifest_value != f"{artifact_dir_value}/manifest.json":
        raise RuntimeError("current manifest path is not inside the selected artifact")
    manifest_path = _contained(root, manifest_value, "current manifest")
    manifest_bytes, manifest = _load_json_bytes(manifest_path, "artifact manifest")
    source_lock_value = current.get("source_lock")
    if not isinstance(source_lock_value, str) or source_lock_value != f"{artifact_dir_value}/source-lock.json":
        raise RuntimeError("current source lock path is invalid")
    source_lock_path = _contained(root, source_lock_value, "current source lock")
    source_lock_bytes, source_lock = _load_json_bytes(source_lock_path, "immutable artifact source lock")
    urdf = _validate_manifest(root, artifact_dir, current, manifest_bytes, manifest, source_lock_bytes, source_lock)
    return ArtifactResolution(root, artifact_dir, current, manifest, source_lock, urdf)


def topic_control_description(urdf: str | bytes) -> str:
    raw = urdf.decode("utf-8") if isinstance(urdf, bytes) else urdf
    root = ET.fromstring(raw)
    for existing in list(root.findall("ros2_control")):
        root.remove(existing)
    control = ET.SubElement(root, "ros2_control", name="TinkerTopicSystem", type="system")
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "topic_based_ros2_control/TopicBasedSystem"
    for name, value in (("joint_commands_topic", "/sim/controller/ros2_control_commands"), ("joint_states_topic", "/isaac_joint_states"), ("trigger_joint_command_threshold", "-1")):
        ET.SubElement(hardware, "param", name=name).text = value
    for name in ARM_JOINTS:
        joint = ET.SubElement(control, "joint", name=name)
        ET.SubElement(joint, "command_interface", name="position")
        ET.SubElement(joint, "command_interface", name="velocity")
        ET.SubElement(joint, "state_interface", name="position")
        ET.SubElement(joint, "state_interface", name="velocity")
        ET.SubElement(joint, "state_interface", name="effort")
    return ET.tostring(root, encoding="unicode")
