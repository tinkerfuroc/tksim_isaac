"""Real producer of the canonical Tinker manipulation model-bundle manifest.

This module implements the schema defined by the production
``xarm_moveit_config`` model-bundle consumer.  It is ROS-free at import time
and runs under both simulator CPython 3.12 and system Humble CPython 3.10.

The producer accepts exactly the five artifact paths plus the normalization
inputs, parses the narrow manipulation subgraph, computes exact byte hashes and
the structural fingerprint, and writes the complete manifest atomically to the
output directory.
"""
from __future__ import annotations

import argparse
import json
import os
import tempfile
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from .model_contract import (
    GROUPS,
    ORDERED_JOINTS,
    PRODUCER,
    SCHEMA_VERSION,
    ModelContractError,
    canonical_contract,
    canonical_json,
    contract_fingerprint,
    sha256_file,
    validate_bundle_manifest,
)

_ARTIFACT_ARGUMENTS = (
    ("simulator_full_urdf", "simulator full URDF"),
    ("planning_urdf", "planning URDF"),
    ("planning_srdf", "planning SRDF"),
    ("joint_limits", "joint limits YAML"),
    ("kinematics", "kinematics YAML"),
)


def _read_yaml_mapping(path: Path, kind: str) -> Mapping[str, object]:
    try:
        value = yaml.safe_load(path.read_bytes())
    except (OSError, yaml.YAMLError) as exc:
        raise ModelContractError(
            "invalid_{}".format(kind), "unable to parse {}: {}".format(path, exc), field=str(path)
        ) from exc
    if not isinstance(value, dict):
        raise ModelContractError(
            "invalid_{}".format(kind), "{} must contain a mapping".format(path), field=str(path)
        )
    return value


def _require_regular_absolute(path: Path, label: str) -> Path:
    path = Path(path)
    if not path.is_absolute():
        raise ModelContractError("artifact_path", "{} must be an absolute path".format(label), field=str(path))
    if not path.is_file() or path.is_symlink():
        raise ModelContractError("artifact_path", "{} is not an existing regular file".format(label), field=str(path))
    return path


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def write_manifest(manifest: Mapping[str, object], output: Path | str) -> Path:
    """Atomically publish *manifest* to *output* (temp file + rename + fsync)."""
    output = Path(output)
    if not output.is_absolute():
        raise ModelContractError("output_path", "output must be an absolute path", field=str(output))
    data = canonical_json(manifest)
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".{}.".format(output.name), dir=str(output.parent))
    temporary_path = Path(temporary)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, output)
        _fsync_directory(output.parent)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise
    return output


def build_manifest(
    *,
    simulator_full_urdf: Path | str,
    planning_urdf: Path | str,
    planning_srdf: Path | str,
    joint_limits: Path | str,
    kinematics: Path | str,
    prefix: str,
    mount: Mapping[str, object],
) -> dict[str, object]:
    """Build a complete, validated model-bundle manifest from the supplied paths."""
    paths = {
        "simulator_full_urdf": _require_regular_absolute(Path(simulator_full_urdf), "simulator_full_urdf"),
        "planning_urdf": _require_regular_absolute(Path(planning_urdf), "planning_urdf"),
        "planning_srdf": _require_regular_absolute(Path(planning_srdf), "planning_srdf"),
        "joint_limits": _require_regular_absolute(Path(joint_limits), "joint_limits"),
        "kinematics": _require_regular_absolute(Path(kinematics), "kinematics"),
    }
    artifacts = {
        name: {"path": str(path.resolve()), "sha256": sha256_file(path)}
        for name, path in paths.items()
    }
    sim_xml = paths["simulator_full_urdf"].read_text(encoding="utf-8")
    plan_xml = paths["planning_urdf"].read_text(encoding="utf-8")
    srdf_xml = paths["planning_srdf"].read_text(encoding="utf-8")
    limits_root = _read_yaml_mapping(paths["joint_limits"], "limits")
    if not isinstance(limits_root, dict) or set(limits_root) != {"joint_limits"} or not isinstance(limits_root["joint_limits"], dict):
        raise ModelContractError(
            "invalid_limits", "joint_limits YAML must contain exactly a joint_limits mapping", field=str(paths["joint_limits"])
        )
    kinematics_root = _read_yaml_mapping(paths["kinematics"], "kinematics")
    if not isinstance(kinematics_root, dict):
        raise ModelContractError(
            "invalid_kinematics", "kinematics YAML must contain a mapping", field=str(paths["kinematics"])
        )
    contract = canonical_contract(
        sim_xml,
        plan_xml,
        srdf_xml,
        limits_root["joint_limits"],
        kinematics_root,
        prefix=prefix,
        mount=mount,
    )
    normalization = {
        "prefix": prefix,
        "mount": mount,
        "groups": GROUPS,
        "ordered_joints": list(ORDERED_JOINTS),
        "selected_links": list(contract["selected_links"]),
    }
    manifest: dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "producer": PRODUCER,
        "artifacts": artifacts,
        "normalization": normalization,
        "contract": contract,
        "structural_fingerprint": contract_fingerprint(contract),
    }
    validate_bundle_manifest(manifest)
    return manifest


def resolve_simulator_full_urdf(project_root: Path | str) -> Path:
    """Resolve the selected canonical Tinker 2 ``robot.urdf`` through current.json.

    Follows the content-addressed selector without pinning any specific
    artifact hash: reads ``artifacts/robot/tinker2/current.json`` and returns
    the selected generation's ``robot.urdf`` absolute path.
    """
    root = Path(project_root)
    if not root.is_absolute():
        root = root.absolute()
    artifact_root = root / "artifacts" / "robot" / "tinker2"
    current_path = artifact_root / "current.json"
    try:
        current = json.loads(current_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ModelContractError(
            "artifact_current", "cannot read current artifact selector {}".format(current_path), field=str(current_path)
        ) from exc
    if not isinstance(current, dict):
        raise ModelContractError("artifact_current", "current.json must contain a JSON object", field=str(current_path))
    artifact_id = current.get("artifact_id")
    if (
        not isinstance(artifact_id, str)
        or len(artifact_id) < 16
        or any(char not in "0123456789abcdef" for char in artifact_id)
    ):
        raise ModelContractError(
            "artifact_current",
            "current.json artifact_id is not a valid content-addressed identity",
            field=str(current_path),
        )
    selected = root / "artifacts" / "robot" / "tinker2" / artifact_id / "robot.urdf"
    if not selected.is_absolute() or not selected.is_file() or selected.is_symlink():
        raise ModelContractError(
            "artifact_current",
            "selected artifact robot.urdf is not an existing regular file: {}".format(selected),
            field=str(selected),
        )
    return selected


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="model_bundle",
        description="Produce the canonical Tinker manipulation model-bundle manifest.",
    )
    parser.add_argument("--simulator-full-urdf", required=True, metavar="PATH", help="canonical simulator full URDF")
    parser.add_argument("--planning-urdf", required=True, metavar="PATH", help="production planning URDF")
    parser.add_argument("--planning-srdf", required=True, metavar="PATH", help="production planning SRDF")
    parser.add_argument("--joint-limits", required=True, metavar="PATH", help="joint limits YAML")
    parser.add_argument("--kinematics", required=True, metavar="PATH", help="kinematics YAML")
    parser.add_argument("--prefix", default="", metavar="PREFIX", help="link/joint name prefix to strip")
    parser.add_argument("--mount-parent", default="world", metavar="WORLD", help="fixed mount parent link")
    parser.add_argument("--mount-child", default="base_link", metavar="BASE", help="fixed mount child link")
    parser.add_argument("--output", required=True, metavar="PATH", help="absolute output manifest path")
    args = parser.parse_args(argv)

    mount = {
        "parent": args.mount_parent,
        "child": args.mount_child,
        "xyz": [0.0, 0.0, 0.0],
        "rpy": [0.0, 0.0, 0.0],
    }
    manifest = build_manifest(
        simulator_full_urdf=Path(args.simulator_full_urdf),
        planning_urdf=Path(args.planning_urdf),
        planning_srdf=Path(args.planning_srdf),
        joint_limits=Path(args.joint_limits),
        kinematics=Path(args.kinematics),
        prefix=args.prefix,
        mount=mount,
    )
    output = write_manifest(manifest, Path(args.output))
    print("wrote model bundle manifest: {}".format(output))
    print("structural_fingerprint: {}".format(manifest["structural_fingerprint"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
