from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from .workspace import (
    ARM_JOINTS,
    ARTIFACT_FILES,
    CANONICALIZER_ALGORITHM,
    PUBLICATION_SCHEMA,
    SOURCE_PATHS,
    ArtifactExportError,
    UnsafePathError,
    _contained,
    _load_json_bytes,
    _safe_dir,
    _safe_file_bytes,
    _safe_relative,
    _source_identity,
    _validate_source_lock,
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


def _record_by_path(records: list[dict[str, object]], path: str) -> dict[str, object]:
    matches = [record for record in records if record.get("path") == path]
    if len(matches) != 1:
        raise RuntimeError(f"source lock is missing exactly one record for {path}")
    return matches[0]


def _validate_manifest(
    root: Path,
    artifact_dir: Path,
    current: dict[str, object],
    manifest_bytes: bytes,
    manifest: dict[str, object],
    lock_bytes: bytes,
    lock: dict[str, object],
) -> bytes:
    if manifest.get("schema_version") != PUBLICATION_SCHEMA or manifest.get("robot") != "tinker2":
        raise RuntimeError("artifact manifest schema or robot is unsupported")
    artifact_id = manifest.get("artifact_id")
    if not isinstance(artifact_id, str) or artifact_dir.name != artifact_id:
        raise RuntimeError("artifact_id does not match manifest and directory")
    if current.get("manifest_sha256") != _hash(manifest_bytes):
        raise RuntimeError("current manifest hash mismatch")
    if current.get("source_lock_sha256") != _hash(lock_bytes):
        raise RuntimeError("current source-lock hash mismatch")
    source_lock_path = manifest.get("source_lock")
    expected_lock = f"artifacts/robot/tinker2/{artifact_id}/source-lock.json"
    if source_lock_path != expected_lock or current.get("source_lock") != source_lock_path:
        raise RuntimeError("current and manifest source locks differ")

    records = _validate_source_lock(lock)
    lock_identity = lock["source_identity"]
    if lock_identity != _source_identity(records):
        raise RuntimeError("source lock identity is not derived from its records")
    manifest_files = manifest.get("files")
    expected_paths = [f"artifacts/robot/tinker2/{artifact_id}/{name}" for name in ARTIFACT_FILES]
    if not isinstance(manifest_files, list) or len(manifest_files) != len(ARTIFACT_FILES):
        raise RuntimeError("manifest payload set is incomplete")
    payload_hashes: dict[str, str] = {}
    for index, record in enumerate(manifest_files):
        if not isinstance(record, dict) or set(record) != {"path", "sha256"}:
            raise RuntimeError("manifest file record is invalid")
        path_value = _safe_relative(record.get("path"), "manifest file path")
        if path_value != expected_paths[index]:
            raise RuntimeError("manifest payload ordering or identity is invalid")
        path = _contained(root, path_value, "manifest file path")
        try:
            path.relative_to(artifact_dir)
        except ValueError as error:
            raise RuntimeError("manifest file escapes its artifact directory") from error
        data = _safe_file_bytes(path, f"artifact payload {path.name}")
        if not isinstance(record.get("sha256"), str) or _hash(data) != record["sha256"]:
            raise RuntimeError(f"artifact payload hash mismatch: {path}")
        payload_hashes[path.name] = record["sha256"]

    canonicalization = manifest.get("canonicalization")
    provenance = manifest.get("provenance")
    if not isinstance(canonicalization, dict) or set(canonicalization) != {
        "algorithm", "source_path", "source_sha256", "source_lock_record", "output_sha256"
    }:
        raise RuntimeError("artifact canonicalization provenance is incomplete")
    if canonicalization["algorithm"] != CANONICALIZER_ALGORITHM:
        raise RuntimeError("artifact canonicalizer provenance is invalid")
    urdf_source_path = canonicalization["source_path"]
    if urdf_source_path != SOURCE_PATHS["robot.urdf"]:
        raise RuntimeError("canonical URDF source path is invalid")
    urdf_record = _record_by_path(records, urdf_source_path)
    if canonicalization["source_lock_record"] != urdf_record or canonicalization["source_sha256"] != urdf_record["sha256"]:
        raise RuntimeError("canonical URDF source provenance does not match the immutable lock")
    if canonicalization["output_sha256"] != payload_hashes["robot.urdf"]:
        raise RuntimeError("canonical URDF provenance hash mismatch")

    if not isinstance(provenance, dict) or set(provenance) != {
        "source_lock_sha256", "source_identity", "source_files", "usd_source_path", "usd_source_sha256"
    }:
        raise RuntimeError("artifact source provenance is incomplete")
    if provenance["source_lock_sha256"] != _hash(lock_bytes):
        raise RuntimeError("artifact source-lock provenance mismatch")
    if provenance["source_identity"] != lock_identity or provenance["source_files"] != records:
        raise RuntimeError("artifact source provenance does not exactly match the immutable lock")
    if provenance["usd_source_path"] != SOURCE_PATHS["robot.usd"]:
        raise RuntimeError("USD source provenance path is invalid")
    usd_record = _record_by_path(records, SOURCE_PATHS["robot.usd"])
    if provenance["usd_source_sha256"] != usd_record["sha256"] or payload_hashes["robot.usd"] != usd_record["sha256"]:
        raise RuntimeError("USD source provenance does not match the immutable lock and payload")

    urdf = _safe_file_bytes(artifact_dir / "robot.urdf", "canonical URDF")
    expected_identity = artifact_identity(payload_hashes, urdf, lock_bytes, CANONICALIZER_ALGORITHM)
    if expected_identity != artifact_id:
        raise RuntimeError("artifact identity does not bind its immutable contents")
    validate_canonical_urdf(urdf)
    if current.get("robot_urdf_sha256") != payload_hashes["robot.urdf"] or current.get("robot_usd_sha256") != payload_hashes["robot.usd"]:
        raise RuntimeError("current payload hashes do not match manifest")
    return urdf


def _resolve_schema4(root: Path, artifact_root: Path, current: dict[str, object]) -> ArtifactResolution:
    if current.get("robot") != "tinker2":
        raise RuntimeError("current artifact pointer schema is unsupported")
    artifact_id = current.get("artifact_id")
    artifact_dir_value = current.get("artifact_dir")
    manifest_value = current.get("manifest")
    if not isinstance(artifact_id, str) or len(artifact_id) != 64 or any(c not in "0123456789abcdef" for c in artifact_id):
        raise RuntimeError("current artifact_id is not a full SHA-256 identity")
    if artifact_dir_value != f"artifacts/robot/tinker2/{artifact_id}":
        raise RuntimeError("current artifact directory identity is invalid")
    artifact_dir = _contained(root, artifact_dir_value, "current artifact directory")
    _safe_dir(artifact_dir, "selected artifact directory")
    if manifest_value != f"{artifact_dir_value}/manifest.json":
        raise RuntimeError("current manifest path is not inside the selected artifact")
    manifest_path = _contained(root, manifest_value, "current manifest")
    manifest_bytes, manifest = _load_json_bytes(manifest_path, "artifact manifest")
    source_lock_value = current.get("source_lock")
    if source_lock_value != f"{artifact_dir_value}/source-lock.json":
        raise RuntimeError("current source lock path is invalid")
    source_lock_path = _contained(root, source_lock_value, "current source lock")
    source_lock_bytes, source_lock = _load_json_bytes(source_lock_path, "immutable artifact source lock")
    urdf = _validate_manifest(root, artifact_dir, current, manifest_bytes, manifest, source_lock_bytes, source_lock)
    return ArtifactResolution(root, artifact_dir, current, manifest, source_lock, urdf)


def _resolve_legacy(root: Path, artifact_root: Path, current: dict[str, object]) -> ArtifactResolution:
    """Resolve the legacy unversioned ``current.json`` + schema-2 manifest.

    The legacy publication pipeline wrote an unversioned pointer
    ``{"artifact_id": "<16-hex>", "manifest": "artifacts/robot/tinker2/<id>/manifest.json"}``
    with a manifest whose ``schema_version`` is not the publication schema.
    This compatibility path validates every invariant the legacy pointer and
    manifest actually expose: exact Tinker-2/artifact binding, safe contained
    paths, manifest agreement, and the selected canonical ``robot.urdf``.
    """
    artifact_id = current.get("artifact_id")
    if not isinstance(artifact_id, str) or len(artifact_id) < 16 or any(c not in "0123456789abcdef" for c in artifact_id):
        raise RuntimeError("legacy current artifact_id is not a valid content-addressed identity")
    artifact_dir_value = f"artifacts/robot/tinker2/{artifact_id}"
    manifest_value = current.get("manifest")
    expected_manifest = f"{artifact_dir_value}/manifest.json"
    if manifest_value != expected_manifest:
        raise RuntimeError("legacy current manifest path is not inside the selected artifact")
    artifact_dir = _contained(root, artifact_dir_value, "legacy artifact directory")
    _safe_dir(artifact_dir, "selected legacy artifact directory")
    manifest_path = _contained(root, manifest_value, "legacy manifest")
    _, manifest = _load_json_bytes(manifest_path, "legacy artifact manifest")
    if manifest.get("robot") != "tinker2" or manifest.get("artifact_id") != artifact_id:
        raise RuntimeError("legacy manifest does not bind the selected Tinker 2 artifact")
    manifest_schema = manifest.get("schema_version")
    if manifest_schema not in (None, 2):
        raise RuntimeError(
            "legacy manifest schema_version {} is not a deployed legacy shape".format(manifest_schema)
        )
    canonicalization = manifest.get("canonicalization")
    if not isinstance(canonicalization, dict) or not isinstance(canonicalization.get("output_sha256"), str):
        raise RuntimeError("legacy manifest canonicalization provenance is incomplete")
    urdf_path = _contained(root, f"{artifact_dir_value}/robot.urdf", "legacy canonical URDF")
    urdf = _safe_file_bytes(urdf_path, "legacy canonical URDF")
    if canonicalization["output_sha256"] != _hash(urdf):
        raise RuntimeError("legacy canonical URDF hash does not match manifest provenance")
    files = manifest.get("files")
    if isinstance(files, list):
        for record in files:
            if isinstance(record, dict) and str(record.get("path", "")).endswith("/robot.urdf"):
                if not isinstance(record.get("sha256"), str) or record["sha256"] != _hash(urdf):
                    raise RuntimeError("legacy manifest payload hash does not match canonical URDF")
                break
    source_lock_value = manifest.get("source_lock")
    if source_lock_value is not None:
        if not isinstance(source_lock_value, str):
            raise RuntimeError("legacy manifest source lock path is invalid")
        _safe_relative(source_lock_value, "legacy source lock path")
    try:
        ET.fromstring(urdf)
    except ET.ParseError as error:
        raise RuntimeError("legacy canonical URDF is not well-formed XML") from error
    source_lock: dict[str, object] = {}
    if isinstance(source_lock_value, str):
        try:
            lock_path = _contained(root, source_lock_value, "legacy source lock")
            _, source_lock = _load_json_bytes(lock_path, "legacy source lock")
        except (ArtifactExportError, UnsafePathError, OSError, json.JSONDecodeError):
            # The legacy provenance lock is not a required deployment invariant;
            # its absence/decay must not fail the pointer resolution.
            source_lock = {}
    return ArtifactResolution(root, artifact_dir, current, manifest, source_lock, urdf)


def resolve_current_artifact(project_root: Path) -> ArtifactResolution:
    """Resolve the selected canonical Tinker 2 artifact through ``current.json``.

    One authoritative resolver for runtime selection, model-bundle resolution,
    and preflight identity.  It explicitly dispatches the currently deployed
    legacy (unversioned pointer + schema-2 manifest) shape and the schema-4
    publication shape; any other shape is rejected.
    """
    root = _safe_dir(project_root, "project root")
    artifact_root = _artifact_root(root)
    current_path = artifact_root / "current.json"
    _, current = _load_json_bytes(current_path, "current artifact pointer")
    schema = current.get("schema_version")
    if schema == PUBLICATION_SCHEMA:
        return _resolve_schema4(root, artifact_root, current)
    if schema is None:
        return _resolve_legacy(root, artifact_root, current)
    raise RuntimeError("current artifact pointer schema is unsupported")


def scenario_arena_id(scenario_data: object) -> str | None:
    """Return the arena id a scenario's ``world`` block names, if any."""
    if not isinstance(scenario_data, dict):
        return None
    world = scenario_data.get("world")
    if not isinstance(world, dict):
        return None
    arena = world.get("arena")
    if not isinstance(arena, str) or not arena.strip():
        return None
    return arena.strip()


def resolve_arena_map_yaml(project_root: Path, arena_id: str) -> Path:
    """Resolve ``artifacts/arena/<arena_id>/<current>/map.yaml``.

    Mirrors ``validation/run_sim.py``'s ``resolve_arena_artifact``: the
    ``current.json`` pointer names the selected manifest, and the map is
    colocated with it.  This is the map the simulator raycasts its synthetic
    lidar against, so it is the only map AMCL can localize on when the
    simulator runs with ``--arena``; the robot artifact's own ``map.yaml`` is
    the hardware arena and shares no occupied cell with it.
    """
    if not arena_id or "/" in arena_id or "\\" in arena_id or arena_id in {".", ".."}:
        raise RuntimeError(f"unsafe arena id: {arena_id!r}")
    pointer = Path(project_root) / "artifacts" / "arena" / arena_id / "current.json"
    if not pointer.is_file():
        raise RuntimeError(f"arena artifact pointer does not exist: {pointer}")
    current = json.loads(pointer.read_text(encoding="utf-8"))
    manifest_value = current.get("manifest") if isinstance(current, dict) else None
    if not isinstance(manifest_value, str) or not manifest_value:
        raise RuntimeError(f"arena artifact pointer has no manifest: {pointer}")
    manifest = Path(manifest_value)
    if not manifest.is_absolute():
        manifest = Path(project_root) / manifest
    map_yaml = manifest.parent / "map.yaml"
    if not map_yaml.is_file():
        raise RuntimeError(f"arena artifact missing map.yaml: {map_yaml}")
    return map_yaml


def topic_control_description(urdf: str | bytes) -> str:
    raw = urdf.decode("utf-8") if isinstance(urdf, bytes) else urdf
    root = ET.fromstring(raw)
    for existing in list(root.findall("ros2_control")):
        root.remove(existing)
    # Strip the URDF's world fixture (<link name="world"/> plus the fixed
    # world -> base_link joint). It exists for offline model tooling, but
    # fed to robot_state_publisher it publishes a STATIC identity
    # world->base_link TF — giving base_link a SECOND parent besides the
    # real odom->base_link. With gpsr.launch's static world->map, TF
    # lookups that resolve through the static chain pin the robot at the
    # map origin: the costmap obstacle layer then inserts every lidar
    # scan as if the robot stood at (0,0,0), fabricating phantom walls
    # and erasing real ones (2026-08-28 doorway-wedge root cause — four
    # controller/costmap tuning rounds were no-ops against it). Runtime
    # consumers that want a world anchor still get one via
    # world->map->odom->base_link.
    for joint in list(root.findall("joint")):
        parent = joint.find("parent")
        if joint.get("name") == "world_joint" or (
            parent is not None and parent.get("link") == "world"
        ):
            root.remove(joint)
    for link in list(root.findall("link")):
        if link.get("name") == "world":
            root.remove(link)
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
    drive = ET.SubElement(control, "joint", name="drive_joint")
    ET.SubElement(drive, "state_interface", name="position")
    ET.SubElement(drive, "state_interface", name="velocity")
    ET.SubElement(drive, "state_interface", name="effort")
    return ET.tostring(root, encoding="unicode")
