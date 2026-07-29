from __future__ import annotations

import json
import shutil
from pathlib import Path

from .config import Config, sha256_file
from .process import run


def _expect_hash(path: Path, expected: str, name: str) -> None:
    if not path.is_file():
        raise RuntimeError(f"{name} is missing: {path}")
    actual = sha256_file(path)
    if actual != expected:
        raise RuntimeError(f"{name} hash mismatch: expected {expected}, found {actual}")


def verify(config: Config, *, require_python: bool) -> dict[str, object]:
    manifest_path = config.root / "release-manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    _expect_hash(
        config.root / "uv.lock",
        manifest["environment"]["lock_sha256"],
        "uv.lock",
    )
    _expect_hash(
        config.root / "pyproject.toml",
        manifest["environment"]["pyproject_sha256"],
        "pyproject.toml",
    )
    uv = shutil.which("uv")
    if uv is None:
        raise RuntimeError("uv is not installed")
    _expect_hash(Path(uv).resolve(), manifest["generated_with"]["uv_sha256"], "uv executable")
    version = run(["uv", "--version"], check=True).stdout.strip().split()[-1]
    if version != manifest["generated_with"]["uv"]:
        raise RuntimeError(
            f"uv version mismatch: expected {manifest['generated_with']['uv']}, found {version}"
        )

    lab = config.path("isaac_lab")
    if not (lab / ".git").is_dir():
        raise RuntimeError(f"Isaac Lab checkout is missing: {lab}")
    commit = run(["git", "-C", str(lab), "rev-parse", "HEAD"], check=True).stdout.strip()
    tree = run(["git", "-C", str(lab), "rev-parse", "HEAD^{tree}"], check=True).stdout.strip()
    dirty = run(["git", "-C", str(lab), "status", "--porcelain"], check=True).stdout.strip()
    if commit != manifest["isaac_lab"]["commit"]:
        raise RuntimeError(f"Isaac Lab commit mismatch: {commit}")
    if tree != manifest["isaac_lab"]["git_tree"] or dirty:
        raise RuntimeError("Isaac Lab checkout is modified or its Git tree hash does not match")

    python_candidates = sorted(
        config.path("uv_python").glob(
            f"cpython-{manifest['python']['version']}-*/bin/python3.12"
        )
    )
    if require_python and len(python_candidates) != 1:
        raise RuntimeError("exactly one pinned managed CPython 3.12 executable is required")
    if python_candidates:
        _expect_hash(
            python_candidates[0],
            manifest["python"]["executable_sha256"],
            "managed Python executable",
        )
    return manifest
