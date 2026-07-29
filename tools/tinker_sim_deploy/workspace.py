from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
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


@dataclass(frozen=True)
class ExportResult:
    artifact_dir: Path
    manifest: dict[str, object]


def export_tinker2(workspace: Path, artifacts: Path, lock_path: Path) -> ExportResult:
    workspace = workspace.resolve()
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
    identity = "".join(f"{name}:{sha256_file(path)}\n" for name, path in sorted(sources.items()))
    digest = hashlib.sha256(identity.encode()).hexdigest()[:16]
    destination = artifacts / "robot" / "tinker2" / digest
    destination.mkdir(parents=True, exist_ok=True)
    file_records = []
    for name, source in sources.items():
        target = destination / name
        if name == "map.yaml":
            lines = source.read_text(encoding="utf-8").splitlines()
            normalized = ["image: map.pgm" if line.strip().startswith("image:") else line for line in lines]
            target.write_text("\n".join(normalized) + "\n", encoding="utf-8")
        elif not target.exists() or sha256_file(target) != sha256_file(source):
            shutil.copy2(source, target)
        file_records.append({"path": target.relative_to(artifacts.parent).as_posix(), "sha256": sha256_file(target)})
    manifest: dict[str, object] = {
        "schema_version": 1, "robot": "tinker2", "artifact_id": digest,
        "source_lock": lock_path.relative_to(artifacts.parent).as_posix(),
        "qualification": "blocked_calibration_missing", "files": file_records,
        "kinematics": {
            "front_left_joint": "front_left_wheel_joint", "front_right_joint": "front_right_wheel_joint",
            "wheel_radius_m": 0.0525, "wheel_track_m": 0.25,
            "footprint": [[0.15, 0.25], [0.15, -0.25], [-0.35, -0.25], [-0.35, 0.25]],
        },
    }
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    current = artifacts / "robot" / "tinker2" / "current.json"
    manifest_pointer = (destination / "manifest.json").relative_to(artifacts.parent).as_posix()
    current.write_text(
        json.dumps({"artifact_id": digest, "manifest": manifest_pointer}, indent=2) + "\n",
        encoding="utf-8",
    )
    return ExportResult(destination, manifest)
