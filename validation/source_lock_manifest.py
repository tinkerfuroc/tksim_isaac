#!/usr/bin/env python3
"""Offline source-lock manifest observer (integrated qualification Gate B).

This module implements the canonical, non-self-referential source-lock
observer.  It validates three separate immutable authorizations, each recorded
by an already completed lock-only phase:

* ``simulator_overlay`` -- the existing simulator
  ``integration/source-locks.json`` (schema-transition lock, path history two);
* ``production`` -- the existing production
  ``integration/source-locks.json`` (lock-only commit, path history one);
* ``qualification_tooling`` -- the future simulator
  ``integration/integrated-qualification-source-lock.json`` (lock-only commit,
  path history one, created only in the post-Task-10 lock-only phase).

No policy contains a ``policy_commit`` field and none is ever read, written, or
required.  The resolved containing/transition commit is discovered from real
Git history and emitted only in observer output.  ``authorization.commit=null``
means the resolved containing/transition commit itself is the pre-attempt
authorization; ``null`` is valid and is not unauthenticated.

Observation runs exact argv under ``LC_ALL=C`` (never shell strings):

* ``git status --porcelain=v1 -z --untracked-files=all``
* ``git diff --binary --no-ext-diff``
* ``git diff --cached --binary --no-ext-diff`` (must be empty)

Raw status/diff bytes are compared to the stored base64 and SHA-256.  A
canonical path-sorted untracked manifest is derived from ``??`` records with
regular mode ``100644``/``100755`` or symlink ``120000``, size and file-content
SHA-256, without following symlinks and with traversal/directory/device
rejection.  ``clean`` requires exact empty status/diff/index/untracked;
``authorized_dirty`` requires the exact stored status/diff/manifest and an
empty index.

The observer never writes a policy.  Output is canonical finite JSON, fsynced
and atomically replaced.  Commit identity/history/blob checks remain
load-bearing; mtime alone never authorizes.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import re
import stat
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

SCHEMA_VERSION = 1
HEX40 = re.compile(r"^[0-9a-f]{40}$")
HEX64 = re.compile(r"^(?!0{64}$)[0-9a-f]{64}$")
BASE64_ENCODING = "base64"

ROLE_SIMULATOR_OVERLAY = "simulator_overlay"
ROLE_PRODUCTION = "production"
ROLE_QUALIFICATION_TOOLING = "qualification_tooling"
ROLES = (
    ROLE_SIMULATOR_OVERLAY,
    ROLE_PRODUCTION,
    ROLE_QUALIFICATION_TOOLING,
)

# The policy ``repository`` field records the Git repository identity, not the
# config role key.  Both simulator roles (historical overlay and the future
# qualification-tooling lock) belong to the simulator repository; ``production``
# belongs to the production repository.
ROLE_REPOSITORY = {
    ROLE_SIMULATOR_OVERLAY: "simulator",
    ROLE_PRODUCTION: "production",
    ROLE_QUALIFICATION_TOOLING: "simulator",
}

# Closed schemas (fix round 1 / F3.4): unknown top-level policy keys and unknown
# authorization keys are rejected fail-closed.  The allowlist is the union of
# the two current real policies' ancillary fields.
ALLOWED_POLICY_KEYS = frozenset(
    {
        "authorization",
        "capture_commands",
        "diff_bytes",
        "diff_sha256",
        "implementation_head",
        "implementation_tree_policy",
        "isaacsim_ros_workspaces",
        "mode",
        "policy_commit_resolution",
        "policy_path",
        "repository",
        "root",
        "schema_version",
        "status_bytes",
        "status_sha256",
        "tinker_cumotion",
        "tinker_isaac_ros_common",
        "untracked_manifest",
        "untracked_manifest_sha256",
        "workspace_policy",
    }
)
ALLOWED_AUTHORIZATION_KEYS = frozenset({"commit", "phase", "report_path"})

# A lock-only commit may change only the policy path plus the review-clean
# acceptance doc (both historical locks ``ab8cf7e`` and ``1e248262`` changed
# exactly ``docs/acceptance.md`` alongside the policy).  Anything else is a
# bundled source/config payload and fails closed.
LOCK_ALLOWED_EXTRA_PATH = "docs/acceptance.md"

AUTHORIZATION_FIELDS = ("repository", "implementation_head", "policy_commit_resolution")

STATUS_PASS = "verified-pass"
STATUS_FAIL = "verified-fail"
STATUS_INVALID = "evidence-invalid"


class SourceLockError(Exception):
    """An observer-side contract violation (schema, history, evidence)."""


def _git_bytes(root: Path, *args: str) -> bytes:
    """Run a git command with ``LC_ALL=C`` and capture raw stdout bytes."""
    proc = subprocess.run(
        ["git", "-C", str(root), *args],
        capture_output=True,
        env={**os.environ, "LC_ALL": "C"},
        check=False,
    )
    if proc.returncode != 0:
        raise SourceLockError(
            "git {} failed in {}: {}".format(
                " ".join(args), root, proc.stderr.decode("utf-8", errors="replace").strip()
            )
        )
    return proc.stdout


def _git_status(root: Path) -> bytes:
    return _git_bytes(
        root, "status", "--porcelain=v1", "-z", "--untracked-files=all"
    )


def _git_diff(root: Path) -> bytes:
    return _git_bytes(root, "diff", "--binary", "--no-ext-diff")


def _git_diff_cached(root: Path) -> bytes:
    return _git_bytes(root, "diff", "--cached", "--binary", "--no-ext-diff")


def _git_head(root: Path) -> str:
    return _git_bytes(root, "rev-parse", "HEAD").decode("ascii").strip()


def _git_commit_time(root: Path, commit: str) -> int | None:
    raw = _git_bytes(root, "show", "-s", "--format=%ct", commit).decode("ascii").strip()
    try:
        value = int(raw)
    except (TypeError, ValueError):
        return None
    return value if value > 0 else None


def _git_is_ancestor(root: Path, ancestor: str, descendant: str) -> bool:
    proc = subprocess.run(
        ["git", "-C", str(root), "merge-base", "--is-ancestor", ancestor, descendant],
        capture_output=True,
        env={**os.environ, "LC_ALL": "C"},
        check=False,
    )
    return proc.returncode == 0


def _git_blob(root: Path, commit: str, rel_path: str) -> bytes:
    return _git_bytes(root, "show", "{}:{}".format(commit, rel_path))


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _canonical_json_bytes(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _sha256_json(value: object) -> str:
    return hashlib.sha256(_canonical_json_bytes(value)).hexdigest()


def _read_policy_json(policy_path: Path) -> dict[str, Any] | None:
    if not policy_path.is_file():
        return None
    try:
        raw = json.loads(policy_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise SourceLockError("policy is not finite JSON: {} ({})".format(policy_path, error))
    if not isinstance(raw, dict):
        raise SourceLockError("policy is not a JSON object: {}".format(policy_path))
    return raw


def _decode_bytes_record(record: object, *, field: str, policy_path: Path) -> bytes:
    if not isinstance(record, dict) or record.get("encoding") != BASE64_ENCODING:
        raise SourceLockError(
            "policy {} field {!r} is not a base64 bytes record".format(policy_path, field)
        )
    data = record.get("data")
    if not isinstance(data, str):
        raise SourceLockError(
            "policy {} field {!r}.data is not a string".format(policy_path, field)
        )
    try:
        return base64.b64decode(data, validate=True)
    except (ValueError, TypeError) as error:
        raise SourceLockError(
            "policy {} field {!r}.data is not valid base64: {}".format(policy_path, field, error)
        )


def _validate_policy_schema(
    policy: Mapping[str, Any],
    *,
    role: str,
    root: Path,
    policy_path: Path,
    rel_path: str,
) -> list[str]:
    reasons: list[str] = []
    if policy.get("schema_version") != 1:
        reasons.append("schema_version must be 1")
    expected_repository = ROLE_REPOSITORY.get(role)
    if str(policy.get("repository")) != expected_repository:
        reasons.append(
            "repository mismatch: {!r} != expected {!r}".format(
                policy.get("repository"), expected_repository
            )
        )
    policy_root = Path(str(policy.get("root", ""))).resolve()
    if policy_root != root.resolve():
        reasons.append("root mismatch: {!r} != {!r}".format(str(policy.get("root")), str(root.resolve())))
    policy_path_norm = _normalize_posix(policy.get("policy_path"))
    if policy_path_norm != rel_path:
        reasons.append(
            "policy_path mismatch: {!r} != {!r}".format(policy.get("policy_path"), rel_path)
        )
    implementation_head = policy.get("implementation_head")
    if not isinstance(implementation_head, str) or not HEX40.fullmatch(implementation_head):
        reasons.append("implementation_head must match 40-hex")
    if "policy_commit" in policy:
        reasons.append(
            "in-file policy_commit is a schema extra and is rejected fail-closed; "
            "the containing commit is resolved from Git history, never recorded"
        )
    mode = policy.get("mode")
    if mode not in {"clean", "authorized_dirty"}:
        reasons.append("mode must be clean or authorized_dirty")
    for field in ("status_sha256", "diff_sha256", "untracked_manifest_sha256"):
        digest = policy.get(field)
        if not isinstance(digest, str) or not HEX64.fullmatch(digest):
            reasons.append("{} must match 64-hex and not be all-zero".format(field))
    for field in ("status_bytes", "diff_bytes"):
        if field in policy:
            try:
                _decode_bytes_record(policy[field], field=field, policy_path=policy_path)
            except SourceLockError as error:
                reasons.append(str(error))
    # Closed schema (F3.4): reject unknown top-level keys.
    unknown = set(policy.keys()) - ALLOWED_POLICY_KEYS
    if unknown:
        reasons.append(
            "unknown top-level policy keys rejected fail-closed: {}".format(
                ", ".join(sorted(unknown))
            )
        )
    authorization = policy.get("authorization")
    if not isinstance(authorization, dict):
        reasons.append("authorization must be an object")
    else:
        unknown_auth = set(authorization.keys()) - ALLOWED_AUTHORIZATION_KEYS
        if unknown_auth:
            reasons.append(
                "unknown authorization keys rejected fail-closed: {}".format(
                    ", ".join(sorted(unknown_auth))
                )
            )
        commit = authorization.get("commit")
        if commit is not None and not (isinstance(commit, str) and HEX40.fullmatch(commit)):
            reasons.append("authorization.commit must be 40-hex or null")
        if not isinstance(authorization.get("report_path"), str) or not authorization["report_path"]:
            reasons.append("authorization.report_path must be a non-empty string")
    return reasons


def _path_history(root: Path, rel_path: str) -> list[str]:
    raw = _git_bytes(root, "log", "--follow", "--format=%H", "--", rel_path).decode("ascii")
    return [line for line in raw.splitlines() if line]


def _blob_has_authorization_fields(root: Path, commit: str, rel_path: str) -> bool:
    try:
        blob = _git_blob(root, commit, rel_path)
    except SourceLockError:
        return False
    try:
        parsed = json.loads(blob.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
        return False
    if not isinstance(parsed, dict):
        return False
    return all(key in parsed for key in AUTHORIZATION_FIELDS)


def _resolve_policy_commit(
    role: str,
    root: Path,
    rel_path: str,
    policy: Mapping[str, Any],
) -> tuple[str | None, list[str]]:
    """Resolve the containing/transition commit from Git history only.

    Returns ``(commit, reasons)``.  A failed resolution returns ``(None, reasons)``.
    Never reads a ``policy_commit`` field and never infers a commit from
    ``HEAD``/``HEAD^``.
    """
    reasons: list[str] = []
    history = _path_history(root, rel_path)
    if role in (ROLE_PRODUCTION, ROLE_QUALIFICATION_TOOLING):
        if len(history) != 1:
            reasons.append(
                "path history count must be exactly 1 (got {}); later touch, "
                "rewrite-then-revert, or deletion/recreation fails closed".format(len(history))
            )
            return None, reasons
        commit = history[0]
        return commit, reasons
    if role == ROLE_SIMULATOR_OVERLAY:
        if len(history) != 2:
            reasons.append(
                "overlay path history count must be exactly 2 (got {}); "
                "the raw count is never a false count of one".format(len(history))
            )
            return None, reasons
        transitions: list[str] = []
        for commit in history:
            if not _blob_has_authorization_fields(root, commit, rel_path):
                continue
            try:
                parent_blob = _git_blob(root, "{}^".format(commit), rel_path)
            except SourceLockError:
                # The baseline commit has no parent; it cannot be the transition.
                continue
            try:
                parent = json.loads(parent_blob.decode("utf-8"))
            except (ValueError, UnicodeDecodeError):
                parent = None
            parent_has = (
                isinstance(parent, dict)
                and all(key in parent for key in AUTHORIZATION_FIELDS)
            )
            if not parent_has:
                transitions.append(commit)
        if not transitions:
            reasons.append("no unique schema-transition commit found in overlay path history")
            return None, reasons
        if len(transitions) > 1:
            reasons.append("ambiguous schema transition ({} candidates)".format(len(transitions)))
            return None, reasons
        return transitions[0], reasons
    reasons.append("unknown role {}".format(role))
    return None, reasons


def _untracked_manifest(
    status_bytes: bytes, root: Path
) -> tuple[list[dict[str, Any]], str]:
    """Build the canonical path-sorted untracked manifest from ``??`` records.

    Regular files use mode ``100644``/``100755``; symlinks use ``120000`` and
    hash the raw target bytes without following the link.  Directories, devices
    and paths escaping the repository are rejected.
    """
    entries: list[dict[str, Any]] = []
    root_resolved = root.resolve()
    records = status_bytes.split(b"\x00")
    for record in records:
        if not record.startswith(b"?? "):
            continue
        rel = record[3:].decode("utf-8", errors="surrogateescape")
        if not rel:
            raise SourceLockError("empty untracked path record in status bytes")
        full = root / rel
        try:
            full.resolve().relative_to(root_resolved)
        except (ValueError, OSError) as error:
            raise SourceLockError(
                "untracked path escapes the repository: {!r} ({})".format(rel, error)
            )
        try:
            st = os.lstat(full)
        except OSError as error:
            raise SourceLockError("cannot stat untracked path {!r}: {}".format(rel, error))
        mode_kind = stat.S_IFMT(st.st_mode)
        if stat.S_ISREG(mode_kind):
            data = full.read_bytes()
            entries.append(
                {
                    "kind": "regular",
                    "mode": "100755" if (st.st_mode & 0o111) else "100644",
                    "path": rel,
                    "sha256": hashlib.sha256(data).hexdigest(),
                    "size": len(data),
                }
            )
        elif stat.S_ISLNK(mode_kind):
            target = os.readlink(full).encode("utf-8", errors="surrogateescape")
            entries.append(
                {
                    "kind": "symlink",
                    "mode": "120000",
                    "path": rel,
                    "sha256": hashlib.sha256(target).hexdigest(),
                    "size": len(target),
                }
            )
        else:
            raise SourceLockError(
                "untracked path is not a regular file or symlink (directory/device "
                "rejected): {!r}".format(rel)
            )
    entries.sort(key=lambda entry: entry["path"])
    manifest_sha = _sha256_json(entries)
    return entries, manifest_sha


def _normalize_posix(value: object) -> str:
    """Return a canonical repository-relative POSIX path string."""
    if not isinstance(value, str):
        return ""
    raw = Path(value)
    if raw.is_absolute():
        # The policy records repository-relative paths; strip any accidental
        # repository prefix so absolute/relative CLI args compare equal.
        return str(raw)
    return raw.as_posix()


def _coerce_attempt_start(attempt_started_at: datetime | None) -> datetime | None:
    if attempt_started_at is None:
        return None
    if attempt_started_at.tzinfo is None:
        return None
    try:
        _ = attempt_started_at.timestamp()
    except (OverflowError, OSError, ValueError):
        return None
    return attempt_started_at


def _resolved_commit_scope(
    root: Path, resolved: str, rel_path: str
) -> tuple[bool, list[str]]:
    """Verify the resolved lock-only commit changes only ``{policy_path,
    docs/acceptance.md}`` (policy required).  A lock commit carrying
    source/config/test payload fails closed (F3.1)."""
    raw = _git_bytes(root, "show", "--name-only", "--format=", resolved)
    changed = {
        line.strip() for line in raw.decode("utf-8", errors="replace").splitlines() if line.strip()
    }
    allowed = {rel_path, LOCK_ALLOWED_EXTRA_PATH}
    extra = sorted(changed - allowed)
    if rel_path not in changed:
        extra.append("<policy path {} not changed>".format(rel_path))
    return (not extra, extra)


def _observe_repository(
    *,
    role: str,
    root: Path,
    policy_arg: str | Path,
    attempt_started_at: datetime | None,
) -> dict[str, Any]:
    root_path = Path(root).resolve()
    policy_arg_path = Path(policy_arg)
    policy_path = (
        policy_arg_path if policy_arg_path.is_absolute() else root_path / policy_arg_path
    )
    record: dict[str, Any] = {
        "repository": role,
        "root": str(root_path),
        "policy_path": None,
        "head": None,
        "implementation_head": None,
        "resolved_policy_commit": None,
        "mode": None,
        "status": STATUS_PASS,
        "policy_file_missing": False,
        "checks": {},
        "reasons": [],
    }

    # Relative path recorded inside the policy (always repository-relative,
    # canonical POSIX form).
    if policy_arg_path.is_absolute():
        try:
            rel_path = policy_path.resolve().relative_to(root_path)
        except ValueError:
            record["reasons"].append("policy path is outside the repository root")
            record["status"] = STATUS_FAIL
            return record
        rel_path = rel_path.as_posix()
    else:
        rel_path = policy_arg_path.as_posix()
    record["policy_path"] = rel_path

    policy = _read_policy_json(policy_path)
    if policy is None:
        record["policy_file_missing"] = True
        record["status"] = STATUS_INVALID
        record["reasons"].append("authorization policy file is absent")
        record["checks"]["policy_file_present"] = False
        return record

    # ---- schema -----------------------------------------------------------
    schema_reasons = _validate_policy_schema(
        policy, role=role, root=root_path, policy_path=policy_path, rel_path=rel_path
    )
    record["checks"]["policy_file_present"] = True
    record["checks"]["schema"] = not schema_reasons
    record["implementation_head"] = policy.get("implementation_head")
    record["mode"] = policy.get("mode")
    if schema_reasons:
        record["status"] = STATUS_FAIL
        record["reasons"].extend(schema_reasons)
        return record

    # ---- resolution (never from HEAD/HEAD^, never from policy_commit) -------
    resolved, resolution_reasons = _resolve_policy_commit(
        role, root_path, rel_path, policy
    )
    if resolved is None:
        record["status"] = STATUS_FAIL
        record["reasons"].extend(resolution_reasons)
        record["checks"]["history"] = False
        return record

    head = _git_head(root_path)
    record["head"] = head
    record["resolved_policy_commit"] = resolved

    first_parent = _git_bytes(root_path, "rev-parse", "{}^".format(resolved)).decode("ascii").strip()
    parent_matches = first_parent == policy.get("implementation_head")
    ancestor_of_head = _git_is_ancestor(root_path, resolved, head)

    try:
        blob_resolved = _git_blob(root_path, resolved, rel_path)
        blob_head = _git_blob(root_path, head, rel_path)
    except SourceLockError as error:
        record["status"] = STATUS_FAIL
        record["reasons"].append("cannot read policy blob: {}".format(error))
        return record
    working_bytes = policy_path.read_bytes()
    blob_resolved_equals_working = blob_resolved == working_bytes
    blob_head_equals_working = blob_head == working_bytes
    blob_resolved_equals_head = blob_resolved == blob_head

    history = _path_history(root_path, rel_path)
    history_ok = False
    if role in (ROLE_PRODUCTION, ROLE_QUALIFICATION_TOOLING):
        history_ok = len(history) == 1 and history[0] == resolved
    elif role == ROLE_SIMULATOR_OVERLAY:
        history_ok = len(history) == 2
    record["checks"]["history"] = history_ok
    record["checks"]["first_parent_matches"] = parent_matches
    record["checks"]["resolved_ancestor_of_head"] = ancestor_of_head
    record["checks"]["blob_resolved_equals_head"] = blob_resolved_equals_head
    record["checks"]["blob_resolved_equals_working"] = blob_resolved_equals_working
    record["checks"]["blob_head_equals_working"] = blob_head_equals_working

    # F3.1 lock-only scope: the resolved commit may change only the policy path
    # plus docs/acceptance.md (policy required).
    scope_ok, scope_extra = _resolved_commit_scope(root_path, resolved, rel_path)
    record["checks"]["lock_commit_scope"] = scope_ok
    record["checks"]["lock_commit_scope_extra"] = scope_extra

    # F3.2 qualification current HEAD: for qualification_tooling only the
    # checked-out HEAD must exactly equal the resolved qualification lock commit.
    head_matches_resolved = head == resolved
    record["checks"]["qualification_head_matches_resolved"] = head_matches_resolved
    qualification_head_ok = (
        role != ROLE_QUALIFICATION_TOOLING or head_matches_resolved
    )

    resolution_ok = bool(
        history_ok
        and parent_matches
        and ancestor_of_head
        and blob_resolved_equals_head
        and blob_resolved_equals_working
        and blob_head_equals_working
        and scope_ok
        and qualification_head_ok
    )
    if not resolution_ok:
        record["status"] = STATUS_FAIL
        record["reasons"].append("policy resolution checks failed")
        if not parent_matches:
            record["reasons"].append(
                "first parent {} != implementation_head {}".format(
                    first_parent, policy.get("implementation_head")
                )
            )
        if not ancestor_of_head:
            record["reasons"].append("resolved policy commit is not an ancestor of HEAD")
        if not blob_resolved_equals_head or not blob_resolved_equals_working or not blob_head_equals_working:
            record["reasons"].append(
                "blob identity disagreement across resolved commit / HEAD / working file"
            )
        if not scope_ok:
            record["reasons"].append(
                "resolved lock commit changes non-lock paths: {}".format(
                    ", ".join(scope_extra)
                )
            )
        if not qualification_head_ok:
            record["reasons"].append(
                "qualification_tooling checked-out HEAD must exactly equal the "
                "resolved qualification lock commit"
            )
        return record

    # ---- observation ---------------------------------------------------------
    status_bytes = _git_status(root_path)
    diff_bytes = _git_diff(root_path)
    index_bytes = _git_diff_cached(root_path)
    observed_status_sha = _sha256_bytes(status_bytes)
    observed_diff_sha = _sha256_bytes(diff_bytes)
    observed_index_sha = _sha256_bytes(index_bytes)

    untracked_entries: list[dict[str, Any]] = []
    observed_untracked_sha = ""
    try:
        untracked_entries, observed_untracked_sha = _untracked_manifest(status_bytes, root_path)
    except SourceLockError as error:
        record["status"] = STATUS_INVALID
        record["reasons"].append("untracked manifest rejected: {}".format(error))
        record["checks"]["evidence"] = False
        return record

    expected_status_sha = str(policy.get("status_sha256"))
    expected_diff_sha = str(policy.get("diff_sha256"))
    expected_untracked_sha = str(policy.get("untracked_manifest_sha256"))

    status_match = observed_status_sha == expected_status_sha
    diff_match = observed_diff_sha == expected_diff_sha
    index_empty = index_bytes == b""
    untracked_match = observed_untracked_sha == expected_untracked_sha

    mode = str(policy.get("mode"))
    if mode == "clean":
        evidence_ok = bool(
            status_match and diff_match and untracked_match and index_empty
            and status_bytes == b"" and diff_bytes == b"" and untracked_entries == []
        )
    else:
        evidence_ok = bool(
            status_match and diff_match and untracked_match and index_empty
        )

    # Recompute the stored untracked manifest from the policy to compare entries
    # exactly when dirty.
    stored_manifest = policy.get("untracked_manifest")
    manifest_match = True
    if mode == "authorized_dirty" and isinstance(stored_manifest, list):
        manifest_match = stored_manifest == untracked_entries

    record["status_match"] = status_match
    record["diff_match"] = diff_match
    record["index_match"] = index_empty
    record["untracked_match"] = untracked_match and manifest_match
    record["observed_clean"] = status_bytes == b"" and diff_bytes == b"" and untracked_entries == []
    record["expected_status_sha256"] = expected_status_sha
    record["observed_status_sha256"] = observed_status_sha
    record["expected_diff_sha256"] = expected_diff_sha
    record["observed_diff_sha256"] = observed_diff_sha
    record["expected_untracked_manifest_sha256"] = expected_untracked_sha
    record["observed_untracked_manifest_sha256"] = observed_untracked_sha
    record["untracked_manifest"] = untracked_entries
    record["checks"]["evidence"] = evidence_ok
    record["checks"]["manifest_entries_match"] = manifest_match

    if not evidence_ok:
        record["status"] = STATUS_FAIL
        if not status_match:
            record["reasons"].append("raw status bytes differ from the recorded authorization")
        if not diff_match:
            record["reasons"].append("raw diff bytes differ from the recorded authorization")
        if not index_empty:
            record["reasons"].append("staged index is not empty")
        if not untracked_match or not manifest_match:
            record["reasons"].append("untracked manifest differs from the recorded authorization")
        return record

    # ---- attempt freshness ----------------------------------------------------
    if attempt_started_at is None:
        record["status"] = STATUS_INVALID
        record["reasons"].append("attempt start time is missing or ambiguous")
        return record

    attempt_epoch = attempt_started_at.timestamp()
    commit_time = _git_commit_time(root_path, resolved)
    try:
        policy_mtime = os.stat(policy_path).st_mtime
    except OSError:
        policy_mtime = None

    commit_predates = commit_time is not None and commit_time < attempt_epoch
    file_predates = policy_mtime is not None and policy_mtime < attempt_epoch
    record["checks"]["commit_predates_attempt"] = commit_predates
    record["checks"]["policy_file_predates_attempt"] = file_predates
    record["commit_time"] = commit_time
    record["policy_mtime"] = policy_mtime

    if commit_time is None:
        record["status"] = STATUS_INVALID
        record["reasons"].append("resolved policy commit time is missing/non-finite")
        return record
    if not commit_predates:
        record["status"] = STATUS_INVALID
        record["reasons"].append("resolved policy commit does not predate the attempt")
        return record
    if not file_predates:
        record["status"] = STATUS_INVALID
        record["reasons"].append("working policy file does not predate the attempt")
        return record

    # F3.3 authorization report: normalize within the repository, require a
    # regular existing file that predates the attempt.  A non-null
    # authorization.commit must be a real 40-hex commit, an ancestor of the
    # resolved lock commit, and predate the attempt.
    authorization = policy.get("authorization")
    report_ok = True
    if isinstance(authorization, dict):
        report_path_raw = authorization.get("report_path")
        if isinstance(report_path_raw, str) and report_path_raw:
            report_rel = Path(report_path_raw)
            report_full = (
                report_rel if report_rel.is_absolute() else root_path / report_rel
            )
            try:
                report_st = os.stat(report_full)
            except OSError:
                report_st = None
            record["checks"]["authorization_report_present"] = report_st is not None
            record["checks"]["authorization_report_regular"] = bool(
                report_st is not None and stat.S_ISREG(report_st.st_mode)
            )
            record["checks"]["authorization_report_predates_attempt"] = bool(
                report_st is not None and report_st.st_mtime < attempt_epoch
            )
            if report_st is None:
                record["reasons"].append(
                    "authorization report does not exist: {!r}".format(report_path_raw)
                )
                report_ok = False
            elif not stat.S_ISREG(report_st.st_mode):
                record["reasons"].append(
                    "authorization report is not a regular file: {!r}".format(report_path_raw)
                )
                report_ok = False
            elif report_st.st_mtime >= attempt_epoch:
                record["reasons"].append(
                    "authorization report does not predate the attempt: {!r}".format(report_path_raw)
                )
                report_ok = False
        commit = authorization.get("commit")
        if commit is not None:
            if not isinstance(commit, str) or not HEX40.fullmatch(commit):
                record["reasons"].append(
                    "authorization.commit must be 40-hex or null"
                )
                report_ok = False
            else:
                is_ancestor = _git_is_ancestor(root_path, commit, resolved)
                auth_commit_time = _git_commit_time(root_path, commit)
                auth_commit_predates = (
                    auth_commit_time is not None and auth_commit_time < attempt_epoch
                )
                record["checks"]["authorization_commit_ancestor_of_resolved"] = is_ancestor
                record["checks"]["authorization_commit_predates_attempt"] = auth_commit_predates
                if not is_ancestor:
                    record["reasons"].append(
                        "authorization.commit is not an ancestor of the resolved lock commit"
                    )
                    report_ok = False
                if not auth_commit_predates:
                    record["reasons"].append(
                        "authorization.commit does not predate the attempt"
                    )
                    report_ok = False
    record["checks"]["authorization_report"] = report_ok
    if not report_ok:
        record["status"] = STATUS_FAIL
        return record

    record["status"] = STATUS_PASS
    return record


def _aggregate_status(records: Mapping[str, Mapping[str, Any]]) -> str:
    statuses = [records[role]["status"] for role in ROLES if role in records]
    if STATUS_INVALID in statuses:
        return STATUS_INVALID
    if STATUS_FAIL in statuses:
        return STATUS_FAIL
    return STATUS_PASS


def capture_manifest(
    *,
    simulator_root: str | Path,
    production_root: str | Path,
    simulator_policy: str | Path,
    production_policy: str | Path,
    qualification_policy: str | Path,
    attempt_started_at: datetime,
    output: str | Path,
) -> dict[str, Any]:
    """Capture the three-repository source-lock manifest.

    Returns the canonical observer dict.  The manifest is also written
    atomically to *output* (fsync + ``os.replace``).  The policy files are never
    created or modified.
    """
    attempt = _coerce_attempt_start(attempt_started_at)
    if attempt is None:
        raise SourceLockError(
            "attempt_started_at must be a finite, timezone-aware datetime"
        )

    records = {
        ROLE_SIMULATOR_OVERLAY: _observe_repository(
            role=ROLE_SIMULATOR_OVERLAY,
            root=Path(simulator_root),
            policy_arg=simulator_policy,
            attempt_started_at=attempt,
        ),
        ROLE_PRODUCTION: _observe_repository(
            role=ROLE_PRODUCTION,
            root=Path(production_root),
            policy_arg=production_policy,
            attempt_started_at=attempt,
        ),
        ROLE_QUALIFICATION_TOOLING: _observe_repository(
            role=ROLE_QUALIFICATION_TOOLING,
            root=Path(simulator_root),
            policy_arg=qualification_policy,
            attempt_started_at=attempt,
        ),
    }

    status = _aggregate_status(records)
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "attempt_started_at": attempt.isoformat(),
        "repositories": sorted(ROLES),
    }
    manifest.update(records)

    # The output manifest must postdate the attempt start (freshness).  F3.7:
    # the artifact is written atomically, its real filesystem mtime is observed
    # from the written file, and only a stable boolean/status is persisted --
    # no stale embedded exact ``output_mtime`` timestamp is claimed.
    output_path = Path(output)
    _atomic_write_fsync_json(output_path, manifest)
    try:
        output_mtime = os.stat(output_path).st_mtime
    except OSError:
        output_mtime = None
    output_postdates = output_mtime is not None and output_mtime > attempt.timestamp()
    manifest["output_predates_attempt"] = not output_postdates
    if status == STATUS_PASS and not output_postdates:
        manifest["status"] = STATUS_INVALID
        manifest["reasons"] = ["output manifest does not postdate the attempt start"]
    # Rewrite with the stable boolean/status; no timestamp is embedded.
    _atomic_write_fsync_json(output_path, manifest)
    return manifest


def _atomic_write_fsync_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".{}.".format(path.name), suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, sort_keys=True, separators=(",", ":"), allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        # fsync the containing directory so the rename is durable.
        dir_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def _read_attempt_start_file(path: str | Path) -> datetime:
    """Read an atomically created ``attempt-start.json`` into an aware datetime.

    Prefers a finite ``started_at`` ISO field; falls back to file mtime.
    """
    attempt_path = Path(path)
    if not attempt_path.is_file():
        raise SourceLockError("attempt-start file is absent: {}".format(attempt_path))
    try:
        raw = json.loads(attempt_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise SourceLockError(
            "attempt-start file is not finite JSON: {} ({})".format(attempt_path, error)
        )
    started = raw.get("started_at") if isinstance(raw, dict) else None
    # F3.6: a finite ISO ``started_at`` is required; the weak file-mtime fallback
    # is removed.  A naive timestamp is normalized to UTC (still finite/aware).
    if not isinstance(started, str):
        raise SourceLockError(
            "attempt-start file must contain a finite ISO started_at string"
        )
    try:
        parsed = datetime.fromisoformat(started)
    except ValueError as error:
        raise SourceLockError("attempt-start started_at is not ISO: {}".format(error))
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    try:
        _ = parsed.timestamp()
    except (OverflowError, OSError, ValueError) as error:
        raise SourceLockError("attempt-start started_at is not finite: {}".format(error))
    return parsed


def capture_source_lock_manifest(argv: Sequence[str] | None = None) -> int:
    """CLI-facing alias of :func:`capture_manifest`."""
    parser = argparse.ArgumentParser(
        description="Capture the three-repository source-lock manifest for Gate B."
    )
    parser.add_argument("--simulator-root", required=True)
    parser.add_argument("--production-root", required=True)
    parser.add_argument("--simulator-policy", required=True)
    parser.add_argument("--production-policy", required=True)
    parser.add_argument("--qualification-policy", required=True)
    parser.add_argument("--attempt-start-file", required=True)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args(argv)

    attempt_started_at = _read_attempt_start_file(arguments.attempt_start_file)
    try:
        manifest = capture_manifest(
            simulator_root=arguments.simulator_root,
            production_root=arguments.production_root,
            simulator_policy=arguments.simulator_policy,
            production_policy=arguments.production_policy,
            qualification_policy=arguments.qualification_policy,
            attempt_started_at=attempt_started_at,
            output=arguments.output,
        )
    except SourceLockError as error:
        print("source-lock manifest capture failed: {}".format(error), file=sys.stderr)
        return 2
    print(json.dumps({"status": manifest["status"]}, sort_keys=True))
    return 0 if manifest["status"] == STATUS_PASS else 1


if __name__ == "__main__":
    sys.exit(capture_source_lock_manifest())
