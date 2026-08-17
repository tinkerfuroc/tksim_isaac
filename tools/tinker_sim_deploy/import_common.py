"""Shared, stdlib-only git/pin/source-record helpers for the RoboCup asset
importer CLIs (``tools/arena_import.py``, ``tools/ycb_import.py``).

Extracted (Task 10 review fix round, Finding 3) because both importers had
byte-identical copies of this exact boilerplate: pin verification and
``git init``+``fetch``+``checkout FETCH_HEAD`` cloning (the same pattern
works for any reachable commit, not just branch tips -- see ``clone_pin``),
plus the ``{path, size, sha256}`` source-lock record shape both importers'
source locks are built from. No Kit/pxr import anywhere in this module (it
is pure stdlib -- ``subprocess``, ``hashlib``, ``pathlib``), so it stays
importable under plain system Python exactly like the rest of each
importer's own orchestration layer.
"""
from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path
from typing import Sequence

from .arena_artifact import AssetArtifactError


def _run_git(args: Sequence[str], *, cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(cwd), *args], capture_output=True, text=True, check=False
    )


def _git_head(checkout: Path) -> str:
    result = _run_git(["rev-parse", "HEAD"], cwd=checkout)
    if result.returncode != 0:
        raise AssetArtifactError(f"could not read checkout HEAD: {result.stderr.strip()}")
    return result.stdout.strip()


def _verify_pin(checkout: Path, expected_commit: str) -> None:
    actual = _git_head(checkout)
    if actual != expected_commit:
        raise AssetArtifactError(
            f"checkout HEAD {actual!r} does not match pinned commit {expected_commit!r}"
        )
    # Matching HEAD alone does not prove the working tree still holds exactly
    # the pinned bytes -- an uncommitted local edit (tampered or merely
    # dirty) is invisible to a HEAD check but would silently feed different
    # content into the importer than the source lock claims to record. Fail
    # closed instead of trusting an unclean tree.
    status = _run_git(["status", "--porcelain"], cwd=checkout)
    if status.returncode != 0:
        raise AssetArtifactError(f"could not read checkout status: {status.stderr.strip()}")
    if status.stdout.strip():
        raise AssetArtifactError(
            f"checkout has uncommitted changes despite matching pinned commit "
            f"{expected_commit!r}; refusing to import from a dirty working tree"
        )


def clone_pin(repository: str, commit: str, checkout: Path) -> None:
    """``git init`` + ``fetch`` + ``checkout FETCH_HEAD`` -- works for any
    reachable commit, not just branch tips. Verifies HEAD before returning.
    """
    checkout.mkdir(parents=True, exist_ok=True)
    for args in (["init", "-q"], ["fetch", "-q", repository, commit], ["checkout", "-q", "FETCH_HEAD"]):
        result = _run_git(args, cwd=checkout)
        if result.returncode != 0:
            raise AssetArtifactError(f"git {' '.join(args)} failed: {result.stderr.strip()}")
    _verify_pin(checkout, commit)


def _source_record(path: str, data: bytes) -> dict[str, object]:
    return {"path": path, "size": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def _read_upstream(checkout: Path, relative: str) -> bytes:
    return (checkout / relative).read_bytes()
