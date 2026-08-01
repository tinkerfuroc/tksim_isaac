"""Committed deterministic arm+gripper joint-limit synthesis.

The production arm limits file (``xarm_moveit_config/config/xarm7/``) defines
``joint1``..``joint7`` but not ``drive_joint``, while the canonical model-bundle
schema requires all eight selected joints.  This module deterministically
synthesizes the canonical eight-joint ``joint_limits`` artifact from the two
committed source YAML files and writes it atomically, so the merged artifact is
itself the path+bytes hashed into the manifest and is reproducible by later
qualification tooling.

The module is ROS-free at import time and runs under both simulator CPython
3.12 and system Humble CPython 3.10.
"""
from __future__ import annotations

import argparse
import os
import tempfile
from pathlib import Path
from typing import Sequence

import yaml

from .model_contract import ARM_JOINTS, ORDERED_JOINTS, ModelContractError

_MAX_SOURCE_BYTES = 16 * 1024 * 1024


def _read_limits(path: Path, label: str) -> dict[str, object]:
    path = Path(path)
    if not path.is_absolute() or not path.is_file() or path.is_symlink():
        raise ModelContractError("artifact_path", "{} must be an existing absolute regular file".format(label), field=str(path))
    if path.stat().st_size > _MAX_SOURCE_BYTES:
        raise ModelContractError("artifact_path", "{} exceeds the {} MiB source bound".format(label, _MAX_SOURCE_BYTES // (1024 * 1024)), field=str(path))
    try:
        value = yaml.safe_load(path.read_bytes())
    except (OSError, yaml.YAMLError) as exc:
        raise ModelContractError("invalid_limits", "unable to parse {}: {}".format(path, exc), field=str(path)) from exc
    if not isinstance(value, dict) or set(value) != {"joint_limits"} or not isinstance(value["joint_limits"], dict):
        raise ModelContractError("invalid_limits", "{} must contain exactly a joint_limits mapping".format(path), field=str(path))
    return dict(value["joint_limits"])


def synthesize_joint_limits(arm_limits_path, gripper_limits_path) -> dict[str, object]:
    """Merge arm + gripper joint-limits mappings into the canonical eight-joint set.

    Returns the full ``{"joint_limits": {...}}`` root mapping with exactly the
    eight ``ORDERED_JOINTS`` in canonical order and deterministic values taken
    from the supplied source files.
    """
    arm = _read_limits(Path(arm_limits_path), "arm joint limits")
    gripper = _read_limits(Path(gripper_limits_path), "gripper joint limits")
    merged: dict[str, object] = {}
    for name in ORDERED_JOINTS:
        if name in ARM_JOINTS:
            source = arm
            label = "arm joint limits"
        else:
            source = gripper
            label = "gripper joint limits"
        entry = source.get(name)
        if not isinstance(entry, dict):
            raise ModelContractError(
                "invalid_limits", "{} is missing from {}".format(name, label), field="joint_limits." + name
            )
        merged[name] = dict(entry)
    return {"joint_limits": merged}


def write_synthesized(arm_limits_path, gripper_limits_path, output_path) -> Path:
    """Synthesize the canonical joint-limits YAML and write it atomically."""
    output = Path(output_path)
    if not output.is_absolute():
        raise ModelContractError("output_path", "output must be an absolute path", field=str(output))
    data = yaml.safe_dump(
        synthesize_joint_limits(arm_limits_path, gripper_limits_path),
        sort_keys=True,
        default_flow_style=False,
    ).encode("utf-8")
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".{}.".format(output.name), dir=str(output.parent))
    temporary_path = Path(temporary)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, output)
        directory = os.open(output.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise
    return output


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="model_limits",
        description="Synthesize the canonical eight-joint joint-limits artifact from the arm and gripper source YAML files.",
    )
    parser.add_argument("--arm-joint-limits", required=True, metavar="PATH", help="xarm7/joint_limits.yaml (joint1..joint7)")
    parser.add_argument("--gripper-joint-limits", required=True, metavar="PATH", help="xarm_gripper/joint_limits.yaml (drive_joint)")
    parser.add_argument("--output", required=True, metavar="PATH", help="absolute synthesized joint-limits output path")
    args = parser.parse_args(argv)
    output = write_synthesized(args.arm_joint_limits, args.gripper_joint_limits, args.output)
    print("wrote synthesized joint limits: {}".format(output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
