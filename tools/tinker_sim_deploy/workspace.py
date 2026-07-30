from __future__ import annotations

import hashlib
import json
import math
import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

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

CANONICALIZER_ALGORITHM = "tinker2-urdf-canonical-v1"
_ZERO_ORIGIN = (0.0, 0.0, 0.0)
_ARM_MOUNT_ORIGIN = (-0.03, 0.0, 0.527)


class ArtifactExportError(RuntimeError):
    """Base class for actionable artifact producer failures."""


class CanonicalizationError(ArtifactExportError):
    """The source URDF cannot produce an unambiguous canonical model."""


class ArtifactPublicationError(ArtifactExportError):
    """A content-addressed artifact could not be published safely."""


def _files(workspace: Path) -> Iterable[Path]:
    seen: set[Path] = set()
    for pattern in SOURCE_GLOBS:
        for path in workspace.glob(pattern):
            if path.is_file() and path not in seen:
                seen.add(path)
                yield path


def _git_head(path: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", "-C", str(path), "rev-parse", "HEAD"], text=True,
            stderr=subprocess.DEVNULL, timeout=10,
        ).strip()
    except (OSError, subprocess.SubprocessError):
        return None


def capture_workspace_lock(workspace: Path, output: Path) -> dict[str, object]:
    workspace = workspace.resolve()
    records = [
        {"path": path.relative_to(workspace).as_posix(), "size": path.stat().st_size, "sha256": sha256_file(path)}
        for path in sorted(_files(workspace))
    ]
    if not records:
        raise RuntimeError(f"no Tinker source files found under {workspace}")
    manifest: dict[str, object] = {
        "schema_version": 1, "robot": "tinker2",
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "workspace": str(workspace), "workspace_git_head": _git_head(workspace),
        "tk26_sim_git_head": _git_head(workspace / "src" / "tk26_sim"), "files": records,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def verify_workspace_lock(workspace: Path, lock_path: Path) -> list[str]:
    raw = json.loads(lock_path.read_text(encoding="utf-8"))
    mismatches: list[str] = []
    for record in raw.get("files", []):
        path = workspace / str(record["path"])
        if not path.is_file():
            mismatches.append(f"missing:{record['path']}")
        elif sha256_file(path) != record["sha256"]:
            mismatches.append(f"sha256:{record['path']}")
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
    xyz_text = "-0.03 0 0.527" if xyz == _ARM_MOUNT_ORIGIN else "0 0 0"
    origin.set("xyz", xyz_text)
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
    duplicate_links = sorted({name for name in link_names if link_names.count(name) > 1})
    if duplicate_links:
        raise CanonicalizationError(f"duplicate link definitions: {', '.join(duplicate_links)}")
    known_links = set(link_names)

    joints = root.findall("joint")
    joint_names = [joint.get("name") for joint in joints]
    if any(not name for name in joint_names):
        raise CanonicalizationError("source graph contains a joint without a name")
    duplicate_joints = sorted({name for name in joint_names if joint_names.count(name) > 1})
    if duplicate_joints:
        raise CanonicalizationError(f"duplicate joint definitions: {', '.join(duplicate_joints)}")
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


def _special_joint(root: ET.Element, name: str) -> ET.Element | None:
    matches = [joint for joint in root.findall("joint") if joint.get("name") == name]
    if len(matches) > 1:
        raise CanonicalizationError(f"duplicate {name} definitions")
    return matches[0] if matches else None


def _ensure_mount_topology(root: ET.Element) -> None:
    world_links = [link for link in root.findall("link") if link.get("name") == "world"]
    if len(world_links) > 1:
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
        if (
            world_joint.get("type") == "fixed"
            and parent is not None and parent.get("link") == "world"
            and child is not None and child.get("link") == "base_link"
            and _origin_matches(world_joint, _ZERO_ORIGIN, _ZERO_ORIGIN)
        ):
            pass
        elif (
            world_joint.get("type") == "fixed"
            and parent is not None and parent.get("link") == "base_link"
            and child is not None and child.get("link") == "link_base"
            and _origin_matches(world_joint, _ARM_MOUNT_ORIGIN, _ZERO_ORIGIN)
        ):
            legacy_mount = world_joint
        else:
            raise CanonicalizationError("world_joint has an unsupported parent, child, type, or origin")

    if base_to_arm is not None and not (
        base_to_arm.get("type") == "fixed"
        and base_to_arm.find("parent") is not None
        and base_to_arm.find("parent").get("link") == "base_link"
        and base_to_arm.find("child") is not None
        and base_to_arm.find("child").get("link") == "link_base"
        and _origin_matches(base_to_arm, _ARM_MOUNT_ORIGIN, _ZERO_ORIGIN)
    ):
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

    if not world_links:
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
    physical_drive = [joint for joint in root.findall("joint") if joint.get("name") == "drive_joint"]
    if len(physical_drive) != 1:
        raise CanonicalizationError(f"expected exactly one physical drive_joint, found {len(physical_drive)}")
    drive_controls = [
        joint
        for control in root.findall("ros2_control")
        for joint in control.findall("joint")
        if joint.get("name") == "drive_joint"
    ]
    if len(drive_controls) > 1:
        raise CanonicalizationError("duplicate drive control definitions")
    if drive_controls:
        drive = drive_controls[0]
        for child in list(drive):
            drive.remove(child)
    else:
        controls = root.findall("ros2_control")
        if not controls:
            raise CanonicalizationError("source URDF has no ros2_control block to preserve arm entries")
        drive = ET.SubElement(controls[0], "joint", {"name": "drive_joint"})
    drive.set("name", "drive_joint")
    for name in ("position", "velocity", "effort"):
        ET.SubElement(drive, "state_interface", {"name": name})


def _validate_canonical_root(root: ET.Element) -> None:
    _validate_graph(root)
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
    controls = [
        joint
        for control in root.findall("ros2_control")
        for joint in control.findall("joint")
        if joint.get("name") == "drive_joint"
    ]
    if len(controls) != 1:
        raise CanonicalizationError("canonical URDF must contain exactly one drive_joint control entry")
    if controls[0].findall("command_interface"):
        raise CanonicalizationError("drive_joint control entry must be state-only")
    if [child.get("name") for child in controls[0].findall("state_interface")] != ["position", "velocity", "effort"]:
        raise CanonicalizationError("drive_joint state interfaces must be position, velocity, effort in order")

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
    _validate_graph(root)
    _ensure_mount_topology(root)
    _ensure_drive_control(root)
    _validate_canonical_root(root)
    xml = ET.tostring(root, encoding="unicode")
    canonical = ET.canonicalize(xml_data=xml, with_comments=False, strip_text=True)
    return (canonical.rstrip("\n") + "\n").encode("utf-8")


def validate_canonical_urdf(data: bytes) -> None:
    root = _parse_urdf(data)
    _validate_canonical_root(root)


def artifact_identity(source_hashes: dict[str, str], canonical_urdf: bytes, canonicalizer_version: str) -> str:
    lines = [
        "artifact-identity-schema:2",
        f"canonicalizer:{canonicalizer_version}",
        f"canonical-robot.urdf:{hashlib.sha256(canonical_urdf).hexdigest()}",
    ]
    lines.extend(f"source:{name}:{digest}" for name, digest in sorted(source_hashes.items()))
    return hashlib.sha256(("\n".join(lines) + "\n").encode("utf-8")).hexdigest()[:16]


def _atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        Path(temporary).unlink(missing_ok=True)
        raise


def _same_directory(left: Path, right: Path) -> bool:
    left_files = sorted(path.relative_to(left) for path in left.rglob("*") if path.is_file())
    right_files = sorted(path.relative_to(right) for path in right.rglob("*") if path.is_file())
    if left_files != right_files:
        return False
    return all((left / relative).read_bytes() == (right / relative).read_bytes() for relative in left_files)


def _relative_to(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return str(path)


@dataclass(frozen=True)
class ExportResult:
    artifact_dir: Path
    manifest: dict[str, object]


def export_tinker2(workspace: Path, artifacts: Path, lock_path: Path) -> ExportResult:
    workspace = workspace.resolve()
    artifacts = artifacts.resolve()
    if not lock_path.is_file():
        capture_workspace_lock(workspace, lock_path)
    mismatches = verify_workspace_lock(workspace, lock_path)
    if mismatches:
        raise RuntimeError("workspace differs from source lock: " + ", ".join(mismatches[:8]))
    sources = {
        "robot.usd": workspace / "src/tk26_sim/_generated/tinker_full.usd",
        "robot.urdf": workspace / "src/tk26_sim/_generated/tinker_full.full.urdf",
        "map.yaml": workspace / "src/tk26_navigation/src/navigation_bringup/maps/0701_robocup_arena3.yaml",
        "map.pgm": workspace / "src/tk26_navigation/src/navigation_bringup/maps/0701_robocup_arena3.pgm",
        "robot-profile.yaml": workspace / "src/tk25_basic/src/tinker_robot_config/robots/tinker2/robot.yaml",
    }
    missing = [str(path) for path in sources.values() if not path.is_file()]
    if missing:
        raise RuntimeError("required Tinker 2 exports are missing: " + ", ".join(missing))

    source_hashes = {name: sha256_file(path) for name, path in sources.items()}
    canonical_urdf = canonicalize_urdf(sources["robot.urdf"].read_bytes())
    digest = artifact_identity(source_hashes, canonical_urdf, CANONICALIZER_ALGORITHM)
    destination = artifacts / "robot" / "tinker2" / digest
    destination.parent.mkdir(parents=True, exist_ok=True)

    file_bytes: dict[str, bytes] = {"robot.urdf": canonical_urdf}
    for name, source in sources.items():
        if name == "robot.urdf":
            continue
        if name == "map.yaml":
            lines = source.read_text(encoding="utf-8").splitlines()
            file_bytes[name] = ("\n".join("image: map.pgm" if line.strip().startswith("image:") else line for line in lines) + "\n").encode("utf-8")
        else:
            file_bytes[name] = source.read_bytes()

    with tempfile.TemporaryDirectory(prefix=f".{digest}.", dir=str(destination.parent)) as temporary:
        staged = Path(temporary) / digest
        staged.mkdir()
        for name, data in file_bytes.items():
            (staged / name).write_bytes(data)
        file_records = [
            {"path": (destination / name).relative_to(artifacts.parent).as_posix(), "sha256": hashlib.sha256(data).hexdigest()}
            for name, data in file_bytes.items()
        ]
        source_lock_ref = _relative_to(lock_path, artifacts.parent)
        manifest: dict[str, object] = {
            "schema_version": 2,
            "robot": "tinker2",
            "artifact_id": digest,
            "source_lock": source_lock_ref,
            "qualification": "blocked_calibration_missing",
            "files": file_records,
            "canonicalization": {
                "algorithm": CANONICALIZER_ALGORITHM,
                "source_path": _relative_to(sources["robot.urdf"], workspace),
                "source_sha256": source_hashes["robot.urdf"],
                "output_sha256": hashlib.sha256(canonical_urdf).hexdigest(),
            },
            "provenance": {
                "source_files": source_hashes,
                "usd_source_sha256": source_hashes["robot.usd"],
            },
            "kinematics": {
                "front_left_joint": "front_left_wheel_joint", "front_right_joint": "front_right_wheel_joint",
                "wheel_radius_m": 0.0525, "wheel_track_m": 0.25,
                "footprint": [[0.15, 0.25], [0.15, -0.25], [-0.35, -0.25], [-0.35, 0.25]],
            },
        }
        (staged / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if destination.exists():
            if not _same_directory(destination, staged):
                raise ArtifactPublicationError(f"content-addressed artifact already exists with different bytes: {destination}")
        else:
            os.replace(staged, destination)

    lock = json.loads(lock_path.read_text(encoding="utf-8"))
    lock["artifact_export"] = {
        "artifact_id": digest,
        "canonicalizer": CANONICALIZER_ALGORITHM,
        "source_urdf_sha256": source_hashes["robot.urdf"],
        "output_sha256": hashlib.sha256(canonical_urdf).hexdigest(),
        "usd_sha256": source_hashes["robot.usd"],
    }
    _atomic_write(lock_path, (json.dumps(lock, indent=2, sort_keys=True) + "\n").encode("utf-8"))

    current = artifacts / "robot" / "tinker2" / "current.json"
    manifest_pointer = (destination / "manifest.json").relative_to(artifacts.parent).as_posix()
    _atomic_write(current, (json.dumps({"artifact_id": digest, "manifest": manifest_pointer}, indent=2) + "\n").encode("utf-8"))
    return ExportResult(destination, manifest)
