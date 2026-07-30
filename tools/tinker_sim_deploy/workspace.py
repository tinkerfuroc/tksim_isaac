from __future__ import annotations

import fcntl
import hashlib
import json
import math
import os
import shutil
import stat
import tempfile
import xml.etree.ElementTree as ET
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Iterable, Iterator

from .config import sha256_file

SOURCE_GLOBS = (
    "src/tk25_basic/src/tinker_robot_config/robots/_common/**",
    "src/tk25_basic/src/tinker_robot_config/robots/tinker2/**",
    "src/tk25_basic/src/tinker_urdf/**",
    "src/tk26_navigation/src/navigation_bringup/launch/**",
    "src/tk26_navigation/src/navigation_bringup/params/ekf.yaml",
    "src/tk26_navigation/src/navigation_bringup/params/nav2_dwb_params.yaml",
    "src/tk26_navigation/src/navigation_bringup/maps/0701_robocup_arena3.*",
    "src/tk26_sim/_generated/tinker_full.full.urdf",
    "src/tk26_sim/_generated/tinker_full.usd",
    "src/tk26_sim/reference/tinker_gazebo_snapshot/docs/**",
)
SOURCE_PATHS = {
    "robot.usd": "src/tk26_sim/_generated/tinker_full.usd",
    "robot.urdf": "src/tk26_sim/_generated/tinker_full.full.urdf",
    "map.yaml": "src/tk26_navigation/src/navigation_bringup/maps/0701_robocup_arena3.yaml",
    "map.pgm": "src/tk26_navigation/src/navigation_bringup/maps/0701_robocup_arena3.pgm",
    "robot-profile.yaml": "src/tk25_basic/src/tinker_robot_config/robots/tinker2/robot.yaml",
}
SOURCE_CONTRACT_VERSION = 1
CANONICALIZER_ALGORITHM = "tinker2-urdf-canonical-v3"
PUBLICATION_SCHEMA = 4
SOURCE_LOCK_SCHEMA = 3
ARTIFACT_FILES = ("robot.urdf", "robot.usd", "map.yaml", "map.pgm", "robot-profile.yaml")
ARM_JOINTS = tuple(f"joint{index}" for index in range(1, 8))
_ZERO_ORIGIN = (0.0, 0.0, 0.0)
_ARM_MOUNT_ORIGIN = (-0.03, 0.0, 0.527)


class ArtifactExportError(RuntimeError):
    """Base class for actionable artifact producer failures."""


class CanonicalizationError(ArtifactExportError):
    """The source URDF cannot produce an unambiguous canonical model."""


class ArtifactPublicationError(ArtifactExportError):
    """A content-addressed artifact could not be published safely."""


class UnsafePathError(ArtifactExportError):
    """A supplied or recorded path is not safe for deployment use."""


def _path_parts_are_safe(path: Path, label: str) -> None:
    if not path.is_absolute():
        raise UnsafePathError(f"{label} must be absolute: {path}")
    if ".." in path.parts:
        raise UnsafePathError(f"{label} contains path traversal: {path}")
    current = Path(path.anchor)
    for part in path.parts[1:]:
        current /= part
        try:
            mode = os.lstat(current).st_mode
        except FileNotFoundError:
            continue
        if stat.S_ISLNK(mode):
            raise UnsafePathError(f"{label} contains symlink component: {current}")


def _safe_dir(path: Path, label: str, *, create: bool = False) -> Path:
    path = Path(path)
    if not path.is_absolute():
        path = path.absolute()
    _path_parts_are_safe(path, label)
    if create:
        path.mkdir(parents=True, exist_ok=True)
    if not path.is_dir() or path.is_symlink():
        raise UnsafePathError(f"{label} is not a regular directory: {path}")
    return path


def _safe_file_bytes(path: Path, label: str) -> bytes:
    path = Path(path)
    _path_parts_are_safe(path, label)
    try:
        descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0))
    except OSError as error:
        raise UnsafePathError(f"cannot safely open {label}: {path}: {error}") from error
    try:
        mode = os.fstat(descriptor).st_mode
        if not stat.S_ISREG(mode):
            raise UnsafePathError(f"{label} is not a regular file: {path}")
        chunks: list[bytes] = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _atomic_write(path: Path, data: bytes) -> None:
    path = Path(path)
    _path_parts_are_safe(path, "atomic output")
    _safe_dir(path.parent, "atomic output parent", create=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    temporary_path = Path(temporary)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
        _fsync_directory(path.parent)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise


def _safe_relative(value: object, label: str) -> str:
    if not isinstance(value, str) or not value or "\\" in value:
        raise UnsafePathError(f"{label} must be a normalized relative POSIX path")
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or path.as_posix() != value:
        raise UnsafePathError(f"{label} is unsafe: {value!r}")
    return value


def _contained(root: Path, relative: str, label: str) -> Path:
    _safe_relative(relative, label)
    candidate = root / Path(*PurePosixPath(relative).parts)
    _path_parts_are_safe(candidate, label)
    resolved_root = root.resolve()
    resolved_candidate = candidate.resolve(strict=False)
    try:
        resolved_candidate.relative_to(resolved_root)
    except ValueError as error:
        raise UnsafePathError(f"{label} escapes {root}: {relative!r}") from error
    return candidate


def _files(workspace: Path) -> Iterable[Path]:
    workspace = _safe_dir(workspace, "workspace")
    seen: set[Path] = set()
    for pattern in SOURCE_GLOBS:
        glob_pattern = pattern + "/*" if pattern.endswith("/**") else pattern
        for path in workspace.glob(glob_pattern):
            if path.is_symlink():
                raise UnsafePathError(f"source glob matched symlink: {path}")
            if path.is_file():
                _path_parts_are_safe(path, "source file")
                if path not in seen:
                    seen.add(path)
                    yield path


def _source_identity(records: list[dict[str, object]]) -> dict[str, str]:
    contract = {
        "version": SOURCE_CONTRACT_VERSION,
        "robot": "tinker2",
        "source_globs": list(SOURCE_GLOBS),
        "source_paths": SOURCE_PATHS,
    }
    canonical = {"contract": contract, "files": records}
    encoded = (json.dumps(canonical, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
    return {"algorithm": "sha256", "value": hashlib.sha256(encoded).hexdigest()}


def _snapshot_workspace(workspace: Path) -> tuple[list[dict[str, object]], dict[str, bytes]]:
    workspace = _safe_dir(workspace, "workspace")
    records: list[dict[str, object]] = []
    consumed: dict[str, bytes] = {}
    for path in sorted(_files(workspace)):
        relative = path.relative_to(workspace).as_posix()
        data = _safe_file_bytes(path, f"source file {relative}")
        consumed[relative] = data
        records.append({"path": relative, "size": len(data), "sha256": hashlib.sha256(data).hexdigest()})
    if not records:
        raise ArtifactExportError(f"no Tinker source files found under {workspace}")
    return records, consumed


def _validate_lock_records(raw: object) -> list[dict[str, object]]:
    if not isinstance(raw, dict):
        raise ArtifactExportError("source lock must be a JSON object")
    records_raw = raw.get("files")
    if not isinstance(records_raw, list) or not records_raw:
        raise ArtifactExportError("source lock files must be a non-empty list")
    records: list[dict[str, object]] = []
    seen: set[str] = set()
    for item in records_raw:
        if not isinstance(item, dict):
            raise ArtifactExportError("source lock contains a non-object record")
        if set(item) != {"path", "size", "sha256"}:
            raise ArtifactExportError("source lock record has unexpected fields")
        relative = _safe_relative(item.get("path"), "source lock path")
        if relative in seen:
            raise ArtifactExportError(f"duplicate source lock path: {relative}")
        seen.add(relative)
        digest = item.get("sha256")
        size = item.get("size")
        if not isinstance(digest, str) or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
            raise ArtifactExportError(f"invalid source lock digest for {relative}")
        if not isinstance(size, int) or size < 0:
            raise ArtifactExportError(f"invalid source lock size for {relative}")
        records.append({"path": relative, "size": size, "sha256": digest})
    paths = [str(item["path"]) for item in records]
    if paths != sorted(paths):
        raise ArtifactExportError("source lock records must be in canonical path order")
    return records


def _normalized_source_lock(records: list[dict[str, object]]) -> bytes:
    payload = {
        "schema_version": SOURCE_LOCK_SCHEMA,
        "robot": "tinker2",
        "source_identity": _source_identity(records),
        "files": records,
    }
    return (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _validate_source_lock(raw: object, expected_records: list[dict[str, object]] | None = None) -> list[dict[str, object]]:
    if not isinstance(raw, dict) or set(raw) != {"schema_version", "robot", "source_identity", "files"}:
        raise ArtifactExportError("source lock schema is unsupported")
    if raw["schema_version"] != SOURCE_LOCK_SCHEMA or raw["robot"] != "tinker2":
        raise ArtifactExportError("source lock schema or robot is unsupported")
    identity = raw["source_identity"]
    if not isinstance(identity, dict) or set(identity) != {"algorithm", "value"} or identity.get("algorithm") != "sha256":
        raise ArtifactExportError("source lock identity is unsupported")
    records = _validate_lock_records(raw)
    if identity.get("value") != _source_identity(records)["value"]:
        raise ArtifactExportError("source lock identity does not match its records")
    if expected_records is not None and records != expected_records:
        raise ArtifactExportError("source lock records do not exactly match the read-once source snapshot")
    return records


def capture_workspace_lock(workspace: Path, output: Path) -> dict[str, object]:
    workspace = _safe_dir(workspace, "workspace")
    output = Path(output)
    _path_parts_are_safe(output, "source lock output")
    records, _ = _snapshot_workspace(workspace)
    payload = json.loads(_normalized_source_lock(records))
    _atomic_write(output, _normalized_source_lock(records))
    return payload


def _load_json_bytes(path: Path, label: str) -> tuple[bytes, dict[str, object]]:
    data = _safe_file_bytes(path, label)
    try:
        raw = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ArtifactExportError(f"{label} is not valid UTF-8 JSON") from error
    if not isinstance(raw, dict):
        raise ArtifactExportError(f"{label} must contain a JSON object")
    return data, raw


def verify_workspace_lock(workspace: Path, lock_path: Path) -> list[str]:
    records, _ = _snapshot_workspace(workspace)
    _, raw = _load_json_bytes(Path(lock_path), "source lock")
    locked = _validate_source_lock(raw)
    current_by_path = {str(item["path"]): item for item in records}
    locked_by_path = {str(item["path"]): item for item in locked}
    mismatches: list[str] = []
    for path in sorted(set(current_by_path) - set(locked_by_path)):
        mismatches.append(f"added:{path}")
    for path in sorted(set(locked_by_path) - set(current_by_path)):
        mismatches.append(f"removed:{path}")
    for path in sorted(set(current_by_path) & set(locked_by_path)):
        if current_by_path[path] != locked_by_path[path]:
            mismatches.append(f"changed:{path}")
    return mismatches


def _parse_triplet(value: str | None, label: str) -> tuple[float, float, float]:
    if value is None:
        raise CanonicalizationError(f"{label} is missing an origin")
    try:
        numbers = tuple(float(item) for item in value.split())
    except ValueError as error:
        raise CanonicalizationError(f"{label} has a nonnumeric origin: {value!r}") from error
    if len(numbers) != 3 or not all(math.isfinite(item) for item in numbers):
        raise CanonicalizationError(f"{label} has an invalid origin: {value!r}")
    return numbers  # type: ignore[return-value]


def _origin_matches(joint: ET.Element, expected_xyz: tuple[float, float, float], expected_rpy: tuple[float, float, float]) -> bool:
    origin = joint.find("origin")
    if origin is None:
        return False
    try:
        xyz = _parse_triplet(origin.get("xyz"), f"joint {joint.get('name')!r}")
        rpy = _parse_triplet(origin.get("rpy"), f"joint {joint.get('name')!r}")
    except CanonicalizationError:
        return False
    return all(math.isclose(actual, wanted, rel_tol=0.0, abs_tol=1e-12) for actual, wanted in (*zip(xyz, expected_xyz), *zip(rpy, expected_rpy)))


def _set_origin(joint: ET.Element, xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> None:
    origin = joint.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        joint.insert(0, origin)
    origin.set("xyz", "-0.03 0 0.527" if xyz == _ARM_MOUNT_ORIGIN else "0 0 0")
    origin.set("rpy", "0 0 0")


def _parse_urdf(data: bytes) -> ET.Element:
    try:
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        root = ET.fromstring(data, parser=parser)
    except ET.ParseError as error:
        raise CanonicalizationError(f"malformed XML in source URDF: {error}") from error
    if root.tag != "robot":
        raise CanonicalizationError(f"source URDF root must be <robot>, found <{root.tag}>")
    return root


def _validate_graph(root: ET.Element) -> None:
    links = root.findall("link")
    link_names = [link.get("name") for link in links]
    if any(not name for name in link_names):
        raise CanonicalizationError("source graph contains a link without a name")
    duplicates = sorted({name for name in link_names if link_names.count(name) > 1})
    if duplicates:
        raise CanonicalizationError(f"duplicate link definitions: {', '.join(duplicates)}")
    known_links = set(link_names)
    joints = root.findall("joint")
    joint_names = [joint.get("name") for joint in joints]
    if any(not name for name in joint_names):
        raise CanonicalizationError("source graph contains a joint without a name")
    duplicates = sorted({name for name in joint_names if joint_names.count(name) > 1})
    if duplicates:
        raise CanonicalizationError(f"duplicate joint definitions: {', '.join(duplicates)}")
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or not parent.get("link"):
            raise CanonicalizationError(f"joint {joint.get('name')!r} is missing a parent link")
        if child is None or not child.get("link"):
            raise CanonicalizationError(f"joint {joint.get('name')!r} is missing a child link")
        if parent.get("link") not in known_links:
            raise CanonicalizationError(f"joint {joint.get('name')!r} references missing parent link {parent.get('link')!r}")
        if child.get("link") not in known_links:
            raise CanonicalizationError(f"joint {joint.get('name')!r} references missing child link {child.get('link')!r}")
        if parent.get("link") == child.get("link"):
            raise CanonicalizationError(f"joint {joint.get('name')!r} connects a link to itself")


def _finite_float(value: str | None, label: str) -> float:
    try:
        number = float(value) if value is not None else float("nan")
    except ValueError as error:
        raise CanonicalizationError(f"{label} is not numeric") from error
    if not math.isfinite(number):
        raise CanonicalizationError(f"{label} is not finite")
    return number


def _validate_physical_arm(root: ET.Element) -> None:
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    for name in ARM_JOINTS:
        joint = joints.get(name)
        if joint is None:
            raise CanonicalizationError(f"missing physical arm joint: {name}")
        if joint.get("type") not in {"revolute", "continuous", "prismatic"}:
            raise CanonicalizationError(f"physical arm joint {name} is not actuated")
        axis = joint.find("axis")
        if axis is None:
            raise CanonicalizationError(f"physical arm joint {name} is missing an axis")
        axis_values = (axis.get("xyz") or "").split()
        if len(axis_values) != 3:
            raise CanonicalizationError(f"physical arm joint {name} axis must contain three components")
        components = [_finite_float(value, f"physical arm joint {name} axis") for value in axis_values]
        if sum(component * component for component in components) <= 0.0:
            raise CanonicalizationError(f"physical arm joint {name} has a zero axis")
        limit = joint.find("limit")
        if limit is None:
            raise CanonicalizationError(f"physical arm joint {name} is missing limits")
        for key in ("effort", "velocity"):
            if _finite_float(limit.get(key), f"physical arm joint {name} limit {key}") <= 0.0:
                raise CanonicalizationError(f"physical arm joint {name} has an invalid {key} limit")
        if joint.get("type") != "continuous":
            lower = _finite_float(limit.get("lower"), f"physical arm joint {name} lower limit")
            upper = _finite_float(limit.get("upper"), f"physical arm joint {name} upper limit")
            if lower >= upper:
                raise CanonicalizationError(f"physical arm joint {name} has inverted limits")


def _validate_physical_drive(root: ET.Element) -> None:
    joint = _special_joint(root, "drive_joint")
    if joint is None or joint.get("type") not in {"revolute", "continuous", "prismatic"}:
        raise CanonicalizationError("physical drive_joint must be a non-fixed actuated joint")
    axis = joint.find("axis")
    if axis is None:
        raise CanonicalizationError("physical drive_joint must have an axis")
    values = (axis.get("xyz") or "").split()
    if len(values) != 3 or sum(_finite_float(value, "drive_joint axis") ** 2 for value in values) <= 0.0:
        raise CanonicalizationError("physical drive_joint must have a nonzero axis")


def _special_joint(root: ET.Element, name: str) -> ET.Element | None:
    matches = [joint for joint in root.findall("joint") if joint.get("name") == name]
    if len(matches) > 1:
        raise CanonicalizationError(f"duplicate {name} definitions")
    return matches[0] if matches else None


def _validate_ros2_control(root: ET.Element, *, require_drive: bool) -> None:
    controls = root.findall("ros2_control")
    if len(controls) != 1:
        raise CanonicalizationError(f"expected exactly one ros2_control block, found {len(controls)}")
    control = controls[0]
    joints = control.findall("joint")
    names = [joint.get("name") for joint in joints]
    if any(not name for name in names) or len(set(names)) != len(names):
        raise CanonicalizationError("ros2_control contains duplicate or unnamed joint definitions")
    expected = set(ARM_JOINTS) | ({"drive_joint"} if require_drive else set())
    if set(names) != expected:
        raise CanonicalizationError(f"ros2_control joints must be exactly {sorted(expected)}")
    for joint in joints:
        name = joint.get("name")
        if name not in expected:
            raise CanonicalizationError(f"unexpected ros2_control joint: {name}")
        commands = [item.get("name") for item in joint.findall("command_interface")]
        states = [item.get("name") for item in joint.findall("state_interface")]
        if name in ARM_JOINTS:
            if commands != ["position", "velocity"] or states != ["position", "velocity"]:
                raise CanonicalizationError(f"arm joint {name} has an invalid ros2_control interface contract")
        elif commands or states != ["position", "velocity", "effort"]:
            raise CanonicalizationError("drive_joint must be state-only with position, velocity, effort states")
    for parameter in control.findall("hardware/param"):
        name = (parameter.get("name") or "").lower()
        value = (parameter.text or "").strip().lower()
        if name in {"add_gripper", "add_bio_gripper"} and value not in {"false", "0", "no"}:
            raise CanonicalizationError(f"gripper command provider metadata is not allowed: {name}")
        if "gripper" in name and name not in {"default_gripper_baud", "add_gripper", "add_bio_gripper"}:
            raise CanonicalizationError(f"gripper command provider metadata is not allowed: {name}")
    for joint in joints:
        if "gripper" in (joint.get("name") or "").lower() or "finger" in (joint.get("name") or "").lower():
            raise CanonicalizationError("ros2_control contains gripper command provider metadata")


def _ensure_mount_topology(root: ET.Element) -> None:
    worlds = [link for link in root.findall("link") if link.get("name") == "world"]
    if len(worlds) > 1:
        raise CanonicalizationError("duplicate world link definitions")
    if not any(link.get("name") == "base_link" for link in root.findall("link")):
        raise CanonicalizationError("source graph is missing required base_link")
    if not any(link.get("name") == "link_base" for link in root.findall("link")):
        raise CanonicalizationError("source graph is missing required link_base")
    world_joint = _special_joint(root, "world_joint")
    base_to_arm = _special_joint(root, "base_to_arm_joint")
    legacy_mount = None
    if world_joint is not None:
        parent = world_joint.find("parent")
        child = world_joint.find("child")
        if world_joint.get("type") == "fixed" and parent is not None and parent.get("link") == "world" and child is not None and child.get("link") == "base_link" and _origin_matches(world_joint, _ZERO_ORIGIN, _ZERO_ORIGIN):
            pass
        elif world_joint.get("type") == "fixed" and parent is not None and parent.get("link") == "base_link" and child is not None and child.get("link") == "link_base" and _origin_matches(world_joint, _ARM_MOUNT_ORIGIN, _ZERO_ORIGIN):
            legacy_mount = world_joint
        else:
            raise CanonicalizationError("world_joint has an unsupported parent, child, type, or origin")
    if base_to_arm is not None and not (base_to_arm.get("type") == "fixed" and base_to_arm.find("parent") is not None and base_to_arm.find("parent").get("link") == "base_link" and base_to_arm.find("child") is not None and base_to_arm.find("child").get("link") == "link_base" and _origin_matches(base_to_arm, _ARM_MOUNT_ORIGIN, _ZERO_ORIGIN)):
        raise CanonicalizationError("base_to_arm_joint does not match the required arm mount")
    if legacy_mount is not None and base_to_arm is not None:
        raise CanonicalizationError("duplicate base_to_arm_joint mount definitions")
    if legacy_mount is not None:
        legacy_mount.set("name", "base_to_arm_joint")
        _set_origin(legacy_mount, _ARM_MOUNT_ORIGIN, _ZERO_ORIGIN)
    elif base_to_arm is not None:
        _set_origin(base_to_arm, _ARM_MOUNT_ORIGIN, _ZERO_ORIGIN)
    else:
        raise CanonicalizationError("source graph has no unambiguous base_link to link_base arm mount")
    if not worlds:
        world = ET.Element("link", {"name": "world"})
        base_index = next(index for index, child in enumerate(root) if child.tag == "link" and child.get("name") == "base_link")
        root.insert(base_index, world)
    if world_joint is None or legacy_mount is world_joint:
        new_world_joint = ET.Element("joint", {"name": "world_joint", "type": "fixed"})
        ET.SubElement(new_world_joint, "parent", {"link": "world"})
        ET.SubElement(new_world_joint, "child", {"link": "base_link"})
        ET.SubElement(new_world_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        base_index = next(index for index, child in enumerate(root) if child.tag == "link" and child.get("name") == "base_link")
        root.insert(base_index, new_world_joint)
    else:
        _set_origin(world_joint, _ZERO_ORIGIN, _ZERO_ORIGIN)


def _ensure_drive_control(root: ET.Element) -> None:
    control = root.findall("ros2_control")[0] if len(root.findall("ros2_control")) == 1 else None
    existing = [] if control is None else [joint for joint in control.findall("joint") if joint.get("name") == "drive_joint"]
    if len(existing) > 1:
        raise CanonicalizationError("duplicate drive control definitions")
    if existing:
        _validate_ros2_control(root, require_drive=True)
        return
    _validate_ros2_control(root, require_drive=False)
    drive = ET.SubElement(control, "joint", {"name": "drive_joint"})
    for name in ("position", "velocity", "effort"):
        ET.SubElement(drive, "state_interface", {"name": name})


def _validate_canonical_root(root: ET.Element) -> None:
    _validate_graph(root)
    _validate_physical_arm(root)
    _validate_physical_drive(root)
    if len([link for link in root.findall("link") if link.get("name") == "world"]) != 1:
        raise CanonicalizationError("canonical URDF must contain exactly one world link")
    world_joints = [joint for joint in root.findall("joint") if joint.get("name") == "world_joint"]
    if len(world_joints) != 1:
        raise CanonicalizationError("canonical URDF must contain exactly one world_joint")
    world_joint = world_joints[0]
    if world_joint.get("type") != "fixed" or world_joint.find("parent").get("link") != "world" or world_joint.find("child").get("link") != "base_link" or not _origin_matches(world_joint, _ZERO_ORIGIN, _ZERO_ORIGIN):
        raise CanonicalizationError("world_joint must be a zero-transform world to base_link fixed joint")
    mounts = [joint for joint in root.findall("joint") if joint.get("name") == "base_to_arm_joint"]
    if len(mounts) != 1:
        raise CanonicalizationError("canonical URDF must contain exactly one base_to_arm_joint")
    mount = mounts[0]
    if mount.get("type") != "fixed" or mount.find("parent").get("link") != "base_link" or mount.find("child").get("link") != "link_base" or not _origin_matches(mount, _ARM_MOUNT_ORIGIN, _ZERO_ORIGIN):
        raise CanonicalizationError("base_to_arm_joint must preserve the exact arm mount transform")
    if len([joint for joint in root.findall("joint") if joint.get("name") == "drive_joint"]) != 1:
        raise CanonicalizationError("canonical URDF must contain exactly one physical drive_joint")
    _validate_ros2_control(root, require_drive=True)
    links = {link.get("name") for link in root.findall("link")}
    incoming = {name: 0 for name in links}
    children: dict[str, list[str]] = {name: [] for name in links}
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        incoming[child] += 1
        children[parent].append(child)
    if incoming.get("world") != 0 or any(incoming[name] != 1 for name in links if name != "world"):
        raise CanonicalizationError("canonical URDF graph must have world as its only root")
    seen: set[str] = set()
    stack = ["world"]
    while stack:
        name = stack.pop()
        if name in seen:
            raise CanonicalizationError(f"canonical URDF graph contains a cycle at {name!r}")
        seen.add(name)
        stack.extend(children[name])
    if seen != links:
        raise CanonicalizationError("canonical URDF graph contains disconnected links")


def canonicalize_urdf(data: bytes) -> bytes:
    root = _parse_urdf(data)
    for control in root.findall("ros2_control"):
        for parameter in control.findall("hardware/param"):
            if (parameter.get("name") or "").lower() in {"add_gripper", "add_bio_gripper"}:
                parameter.text = "False"
    _validate_graph(root)
    _ensure_mount_topology(root)
    _ensure_drive_control(root)
    _validate_canonical_root(root)
    xml = ET.tostring(root, encoding="unicode")
    canonical = ET.canonicalize(xml_data=xml, with_comments=False, strip_text=False)
    return (canonical.rstrip("\n") + "\n").encode("utf-8")


def validate_canonical_urdf(data: bytes) -> None:
    _validate_canonical_root(_parse_urdf(data))


def artifact_identity(payload_hashes: dict[str, str], canonical_urdf: bytes, source_lock_bytes: bytes, canonicalizer_version: str) -> str:
    lines = [
        f"publication-schema:{PUBLICATION_SCHEMA}",
        f"canonicalizer:{canonicalizer_version}",
        f"source-lock-sha256:{hashlib.sha256(source_lock_bytes).hexdigest()}",
        f"canonical-robot.urdf-sha256:{hashlib.sha256(canonical_urdf).hexdigest()}",
    ]
    lines.extend(f"payload:{name}:{digest}" for name, digest in sorted(payload_hashes.items()))
    return hashlib.sha256(("\n".join(lines) + "\n").encode("utf-8")).hexdigest()


def _same_directory(left: Path, right: Path) -> bool:
    def regular_entries(root: Path) -> list[Path] | None:
        entries: list[Path] = []
        for candidate in root.rglob("*"):
            if candidate.is_symlink() or not candidate.is_file():
                return None
            entries.append(candidate.relative_to(root))
        return sorted(entries)

    left_files = regular_entries(left)
    right_files = regular_entries(right)
    if left_files is None or right_files is None or left_files != right_files:
        return False
    return all((left / relative).read_bytes() == (right / relative).read_bytes() for relative in left_files)


def _source_paths(workspace: Path) -> dict[str, str]:
    return dict(SOURCE_PATHS)


@dataclass(frozen=True)
class ExportResult:
    artifact_dir: Path
    manifest: dict[str, object]


@contextmanager
def _publication_lock(artifact_root: Path) -> Iterator[None]:
    lock_path = artifact_root / ".artifact-export.lock"
    descriptor = os.open(lock_path, os.O_RDWR | os.O_CREAT, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise ArtifactPublicationError("another artifact exporter is active") from error
        yield
    finally:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)


def _recover_staging(artifact_root: Path) -> None:
    if not artifact_root.exists():
        return
    for path in artifact_root.iterdir():
        if path.name.startswith(".artifact-stage-"):
            if path.is_symlink() or not path.is_dir():
                raise ArtifactPublicationError(f"unsafe interrupted artifact staging path: {path}")
            shutil.rmtree(path)
    _fsync_directory(artifact_root)


def export_tinker2(workspace: Path, artifacts: Path, lock_path: Path) -> ExportResult:
    artifacts = _safe_dir(artifacts, "artifacts root", create=True)
    artifact_root = artifacts / "robot" / "tinker2"
    _safe_dir(artifact_root, "artifact root", create=True)
    with _publication_lock(artifact_root):
        return _export_tinker2_locked(workspace, artifacts, lock_path)


def _export_tinker2_locked(workspace: Path, artifacts: Path, lock_path: Path) -> ExportResult:
    workspace = _safe_dir(workspace, "workspace")
    artifacts = _safe_dir(artifacts, "artifacts root", create=True)
    lock_path = Path(lock_path)
    _path_parts_are_safe(lock_path, "source lock")
    _path_parts_are_safe(lock_path.parent, "source lock parent")
    try:
        lock_path.relative_to(artifacts / "provenance")
    except ValueError as error:
        raise UnsafePathError("source lock must be under artifacts/provenance") from error

    current_records, consumed = _snapshot_workspace(workspace)
    if lock_path.exists():
        _, lock_input = _load_json_bytes(lock_path, "source lock")
        _validate_source_lock(lock_input, current_records)

    source_lock_bytes = _normalized_source_lock(current_records)
    source_paths = _source_paths(workspace)
    missing = [relative for relative in source_paths.values() if relative not in consumed]
    if missing:
        raise ArtifactExportError("required Tinker 2 exports are missing from validated source snapshot: " + ", ".join(missing))
    source_data = {name: consumed[relative] for name, relative in source_paths.items()}
    canonical_urdf = canonicalize_urdf(source_data["robot.urdf"])
    file_bytes: dict[str, bytes] = {"robot.urdf": canonical_urdf}
    for name in ARTIFACT_FILES:
        if name == "robot.urdf":
            continue
        data = source_data[name]
        if name == "map.yaml":
            lines = data.decode("utf-8").splitlines()
            data = ("\n".join("image: map.pgm" if line.strip().startswith("image:") else line for line in lines) + "\n").encode("utf-8")
        file_bytes[name] = data
    payload_hashes = {name: hashlib.sha256(data).hexdigest() for name, data in file_bytes.items()}
    digest = artifact_identity(payload_hashes, canonical_urdf, source_lock_bytes, CANONICALIZER_ALGORITHM)
    artifact_root = artifacts / "robot" / "tinker2"
    _safe_dir(artifact_root, "artifact root", create=True)
    _recover_staging(artifact_root)
    destination = artifact_root / digest
    if destination.exists() and (destination.is_symlink() or not destination.is_dir()):
        raise ArtifactPublicationError(f"content-addressed artifact path is unsafe: {destination}")

    manifest: dict[str, object] = {
        "schema_version": PUBLICATION_SCHEMA,
        "robot": "tinker2",
        "artifact_id": digest,
        "source_lock": f"artifacts/robot/tinker2/{digest}/source-lock.json",
        "qualification": "blocked_calibration_missing",
        "files": [{"path": f"artifacts/robot/tinker2/{digest}/{name}", "sha256": payload_hashes[name]} for name in ARTIFACT_FILES],
        "canonicalization": {
            "algorithm": CANONICALIZER_ALGORITHM,
            "source_path": source_paths["robot.urdf"],
            "source_sha256": hashlib.sha256(source_data["robot.urdf"]).hexdigest(),
            "source_lock_record": next(record for record in current_records if record["path"] == source_paths["robot.urdf"]),
            "output_sha256": payload_hashes["robot.urdf"],
        },
        "provenance": {
            "source_lock_sha256": hashlib.sha256(source_lock_bytes).hexdigest(),
            "source_identity": json.loads(source_lock_bytes)["source_identity"],
            "source_files": current_records,
            "usd_source_path": source_paths["robot.usd"],
            "usd_source_sha256": hashlib.sha256(source_data["robot.usd"]).hexdigest(),
        },
        "kinematics": {
            "front_left_joint": "front_left_wheel_joint", "front_right_joint": "front_right_wheel_joint",
            "wheel_radius_m": 0.0525, "wheel_track_m": 0.25,
            "footprint": [[0.15, 0.25], [0.15, -0.25], [-0.35, -0.25], [-0.35, 0.25]],
        },
    }
    source_lock_snapshot = source_lock_bytes
    manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")
    current_payload = {
        "schema_version": PUBLICATION_SCHEMA,
        "robot": "tinker2",
        "artifact_id": digest,
        "artifact_dir": f"artifacts/robot/tinker2/{digest}",
        "manifest": f"artifacts/robot/tinker2/{digest}/manifest.json",
        "manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
        "source_lock": f"artifacts/robot/tinker2/{digest}/source-lock.json",
        "source_lock_sha256": hashlib.sha256(source_lock_snapshot).hexdigest(),
        "robot_urdf_sha256": payload_hashes["robot.urdf"],
        "robot_usd_sha256": payload_hashes["robot.usd"],
    }
    current_bytes = (json.dumps(current_payload, indent=2, sort_keys=True) + "\n").encode("utf-8")

    stage = Path(tempfile.mkdtemp(prefix=".artifact-stage-", dir=str(artifact_root)))
    try:
        for name, data in file_bytes.items():
            target = stage / name
            target.write_bytes(data)
            with target.open("rb") as stream:
                stream.flush()
                os.fsync(stream.fileno())
        (stage / "source-lock.json").write_bytes(source_lock_snapshot)
        with (stage / "source-lock.json").open("rb") as stream:
            stream.flush()
            os.fsync(stream.fileno())
        (stage / "manifest.json").write_bytes(manifest_bytes)
        with (stage / "manifest.json").open("rb") as stream:
            stream.flush()
            os.fsync(stream.fileno())
        _fsync_directory(stage)
        if destination.exists():
            if not _same_directory(destination, stage):
                raise ArtifactPublicationError(f"content-addressed artifact already exists with different bytes: {destination}")
            shutil.rmtree(stage)
        else:
            try:
                os.rename(stage, destination)
            except FileExistsError:
                if not _same_directory(destination, stage):
                    raise ArtifactPublicationError(f"concurrent artifact collision at {destination}")
                shutil.rmtree(stage)
            _fsync_directory(artifact_root)
    except BaseException:
        if stage.exists():
            shutil.rmtree(stage, ignore_errors=True)
        raise

    current = artifact_root / "current.json"
    _atomic_write(current, current_bytes)
    return ExportResult(destination, manifest)
