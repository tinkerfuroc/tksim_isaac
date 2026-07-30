from __future__ import annotations

import gzip
import hashlib
import json
import os
import shutil
import stat
import tarfile
import tempfile
from pathlib import Path, PurePosixPath
from typing import Iterable

from .assets import verify_assets
from .config import Config, sha256_file
from .runtime import resolve_current_artifact
from .workspace import _safe_relative


PROJECT_ENTRIES = (
    ".python-version",
    "config",
    "contracts",
    "deployment.env.example",
    "deployment.json",
    "docs",
    "pyproject.toml",
    "README.md",
    "release-manifest.json",
    "ros2_ws",
    "scripts",
    "simulation",
    "tools",
    "uv.lock",
    "validation",
)


def _copy_entry(source: Path, destination: Path) -> None:
    if source.is_dir():
        shutil.copytree(
            source,
            destination,
            symlinks=True,
            ignore=shutil.ignore_patterns("__pycache__", "*.pyc", ".pytest_cache"),
        )
        for source_link in source.rglob("*"):
            if not source_link.is_symlink():
                continue
            target = Path(os.readlink(source_link))
            if not target.is_absolute():
                continue
            resolved = target.resolve()
            try:
                relative_target = resolved.relative_to(source.resolve())
            except ValueError as error:
                raise RuntimeError(
                    f"bundle input contains an external absolute symlink: {source_link} -> {target}"
                ) from error
            destination_link = destination / source_link.relative_to(source)
            destination_target = destination / relative_target
            destination_link.unlink()
            destination_link.symlink_to(
                os.path.relpath(destination_target, start=destination_link.parent)
            )
    elif source.is_file():
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)


def _payload_entries(root: Path) -> Iterable[Path]:
    for path in sorted(root.rglob("*")):
        if (path.is_file() or path.is_symlink()) and path.name != "checksums.json":
            yield path


def _checksums(root: Path) -> dict[str, dict[str, str]]:
    entries: dict[str, dict[str, str]] = {}
    for path in _payload_entries(root):
        relative = path.relative_to(root).as_posix()
        if path.is_symlink():
            entries[relative] = {"type": "symlink", "target": os.readlink(path)}
        else:
            entries[relative] = {"type": "file", "sha256": sha256_file(path)}
    return entries


def _tar_filter(info: tarfile.TarInfo) -> tarfile.TarInfo:
    info.uid = 0
    info.gid = 0
    info.uname = ""
    info.gname = ""
    info.mtime = 0
    return info


def _write_reproducible_tar_gz(source: Path, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as raw:
        with gzip.GzipFile(fileobj=raw, mode="wb", filename="", mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode="w") as archive:
                for path in sorted(source.rglob("*")):
                    archive.add(
                        path,
                        arcname=path.relative_to(source).as_posix(),
                        recursive=False,
                        filter=_tar_filter,
                    )


def create(config: Config, output: Path, uv_executable: Path) -> Path:
    # Whole-robot bundles are not usable without one verified immutable robot generation.
    resolve_current_artifact(config.root)
    verify_assets(config)
    required = [
        config.root / "uv.lock",
        config.path("uv_cache"),
        config.path("uv_python"),
        config.path("isaac_lab"),
        config.path("isaacsim_ros_workspaces"),
        config.path("ros_deb_cache"),
        config.path("ros_vendor"),
    ]
    missing = [str(path) for path in required if not path.exists()]
    if missing:
        raise RuntimeError("offline bundle inputs are missing: " + ", ".join(missing))
    with tempfile.TemporaryDirectory(prefix="tinker-sim-bundle-") as temporary:
        stage = Path(temporary)
        for entry in PROJECT_ENTRIES:
            _copy_entry(config.root / entry, stage / entry)
        _copy_entry(config.path("uv_cache"), stage / config.raw["paths"]["uv_cache"])
        _copy_entry(config.path("uv_python"), stage / config.raw["paths"]["uv_python"])
        _copy_entry(config.path("isaac_cache"), stage / config.raw["paths"]["isaac_cache"])
        _copy_entry(config.path("isaac_lab"), stage / config.raw["paths"]["isaac_lab"])
        _copy_entry(
            config.path("isaacsim_ros_workspaces"),
            stage / config.raw["paths"]["isaacsim_ros_workspaces"],
        )
        _copy_entry(
            config.path("ros_deb_cache"), stage / config.raw["paths"]["ros_deb_cache"]
        )
        _copy_entry(config.path("ros_vendor"), stage / config.raw["paths"]["ros_vendor"])
        _copy_entry(config.path("artifacts"), stage / config.raw["paths"]["artifacts"])
        bin_dir = stage / "bin"
        bin_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(uv_executable, bin_dir / "uv")
        (bin_dir / "uv").chmod((bin_dir / "uv").stat().st_mode | stat.S_IXUSR)
        manifest = {
            "schema_version": 1,
            "isaac_lab_commit": config.lab_commit,
            "uv_sha256": sha256_file(bin_dir / "uv"),
            "files": _checksums(stage),
        }
        (stage / "checksums.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        _write_reproducible_tar_gz(stage, output)
    return output


def _validate_member(member: tarfile.TarInfo) -> None:
    path = PurePosixPath(member.name)
    if path.is_absolute() or ".." in path.parts:
        raise RuntimeError(f"unsafe path in offline bundle: {member.name}")
    if member.issym() or member.islnk():
        link = PurePosixPath(member.linkname)
        if link.is_absolute():
            raise RuntimeError(f"unsafe link in offline bundle: {member.name}")
        depth = 0
        for part in (path.parent / link).parts:
            if part in ("", "."):
                continue
            if part == "..":
                depth -= 1
            else:
                depth += 1
            if depth < 0:
                raise RuntimeError(f"unsafe link in offline bundle: {member.name}")


def restore(bundle: Path, destination: Path, *, profile: str = "whole_robot") -> Path:
    if profile not in {"whole_robot", "physics_only"}:
        raise RuntimeError(f"unknown bundle restore profile: {profile}")
    if destination.exists():
        destination_stat = os.lstat(destination)
        if stat.S_ISLNK(destination_stat.st_mode) or not stat.S_ISDIR(destination_stat.st_mode):
            raise RuntimeError(f"restore destination must be a real directory: {destination}")
        if any(destination.iterdir()):
            raise RuntimeError(f"restore destination must be empty: {destination}")
    else:
        destination.mkdir(parents=True, exist_ok=False)
    with tarfile.open(bundle, "r:gz") as archive:
        members = archive.getmembers()
        for member in members:
            _validate_member(member)
        archive.extractall(destination, members=members)
    manifest_path = destination / "checksums.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or not isinstance(manifest.get("files"), dict):
        raise RuntimeError("offline bundle checksums manifest is invalid")
    failures = []
    for relative, expected in manifest["files"].items():
        safe_relative = _safe_relative(relative, "offline bundle checksum path")
        path = destination / Path(*PurePosixPath(safe_relative).parts)
        try:
            path.parent.resolve().relative_to(destination.resolve())
        except ValueError as error:
            raise RuntimeError(f"offline bundle checksum path escapes destination: {relative}") from error
        if not isinstance(expected, dict) or expected.get("type") not in {"file", "symlink"}:
            raise RuntimeError(f"offline bundle checksum record is invalid: {relative}")
        if expected["type"] == "symlink":
            valid = path.is_symlink() and os.readlink(path) == expected["target"]
        else:
            valid = (
                path.is_file()
                and not path.is_symlink()
                and sha256_file(path) == expected["sha256"]
            )
        if not valid:
            failures.append(relative)
    if failures:
        raise RuntimeError("offline bundle checksum failure: " + ", ".join(failures[:20]))
    if profile == "whole_robot":
        resolve_current_artifact(destination)
    return destination
