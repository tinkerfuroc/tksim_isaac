"""Integrated qualification reproducibility evidence index (ROS-free, Python 3.12).

Gate F closes provenance, rosbag, process/GPU teardown, and visual evidence.
This module builds a deterministic ``evidence-index.json`` from the real
preserved artifact bytes and metadata of an integrated qualification attempt
suite, and validates it (``validate_gate_f``).  Task 10 wires Gate F into the
orchestrator; this module is the offline artifact contract it consumes.

Determinism contract
--------------------
- Canonical JSON is ``json.dumps(..., sort_keys=True, separators=(",", ":"),
  ensure_ascii=False)``; ``canonical_sha256`` hashes that exact projection.
- Every file's SHA-256 is the lowercase 64-hex digest of its exact bytes.
- ``files`` is sorted by canonical relative path; repeated builds over
  unchanged bytes are byte-identical.
- ``evidence-index.json`` excludes only itself from its checksum list; after
  contact sheets and the qualification summary are generated, rebuilding the
  index may include those files while still excluding only itself.

Fail-closed contract
--------------------
- Missing commit identity, rosbag metadata/QoS/counts, planning-scene journal,
  required artifacts, and unbound captures make ``validate_gate_f`` return
  ``verified-fail`` with explicit reasons.
- Every visual capture is bound to exact scenario/attempt/execution-request plus
  ``(frame_index, timestamp)`` metadata from ``visual-capture-requests.jsonl``.
  PlanningScene/action/screenshot evidence is diagnostic only and is never
  physical pass authority.
- Path traversal, symlink escape, duplicate canonical paths/events,
  output-as-input, and files changing during hashing are rejected.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

INDEX_NAME = "evidence-index.json"
SUMMARY_NAME = "qualification-summary.json"

REQUIRED_POSITIVE_EVENTS = (
    "readiness",
    "approach",
    "bilateral-contact",
    "attached-transport",
    "place-target",
    "released-settled",
    "terminal",
)
CANCEL_EVENTS = (
    "cancel-execution-start",
    "cancel-trigger",
    "cancel-velocity-compliant",
    "cancel-terminal",
)
SAFETY_EVENTS = (
    "safety-execution-start",
    "safety-trigger",
    "safety-velocity-compliant",
    "safety-post-clear",
)
_SOURCE_IDENTITY_FILES = {
    "source/simulator-commit.json",
    "source/production-commit.json",
}

_CAPTURE_REQUEST_NAME = "visual-capture-requests.jsonl"


def canonical_sha256(value: Any) -> str:
    """Lowercase 64-hex SHA-256 over the canonical JSON projection."""
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _read_bytes(path: Path) -> bytes:
    return path.read_bytes()


def _hash_file_stable(path: Path) -> str:
    """Hash exact file bytes, rejecting files that change during hashing."""
    before = path.stat()
    data = _read_bytes(path)
    after = path.stat()
    if before.st_size != after.st_size or before.st_mtime_ns != after.st_mtime_ns:
        raise ValueError(f"file changed during hashing: {path}")
    return hashlib.sha256(data).hexdigest()


def _atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def _parse_json(path: Path) -> tuple[Any, str | None]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value, None
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return None, str(error)


def _canonical_relative_paths(suite_dir: Path) -> list[str]:
    """All regular files under ``suite_dir`` as sorted canonical relative paths.

    Rejects symlink escape (a symlink whose resolved target leaves the suite)
    and any path traversal that escapes the suite root.
    """
    suite_resolved = suite_dir.resolve()
    relative_paths: list[str] = []
    for path in sorted(suite_resolved.rglob("*")):
        if path.is_symlink():
            try:
                path.resolve().relative_to(suite_resolved)
            except ValueError:
                raise ValueError(f"symlink escape or path traversal: {path}")
        if not path.is_file():
            continue
        try:
            rel = path.relative_to(suite_resolved).as_posix()
        except ValueError:
            raise ValueError(f"path traversal escaping suite root: {path}")
        relative_paths.append(rel)
    return relative_paths


def _category(rel_path: str) -> str:
    if rel_path.startswith("config/"):
        return "config"
    if rel_path.startswith("scenario/"):
        return "scenario"
    if rel_path == "overlay-contract.json":
        return "overlay-contract"
    if rel_path == "model-fingerprint.json":
        return "model-fingerprint"
    if rel_path in _SOURCE_IDENTITY_FILES:
        return "source-identity"
    if rel_path == "source/source-locks.json":
        return "source-lock"
    if rel_path == "source/dependency-locks.json":
        return "dependency-lock"
    if rel_path.startswith("runtime/"):
        return "runtime"
    if rel_path.startswith("planning-scene/"):
        return "planning-scene-journal"
    if rel_path.startswith("moveit/"):
        return "moveit"
    if rel_path.startswith("physics/"):
        return "physics"
    if rel_path.startswith("verdict/"):
        return "verdict"
    if rel_path.startswith("rosbag/"):
        return "rosbag"
    if rel_path.startswith("captures/"):
        return "capture"
    if rel_path == _CAPTURE_REQUEST_NAME:
        return "capture-request-journal"
    if rel_path.startswith("contact-sheet-integrated-") and rel_path.endswith(".png"):
        return "contact-sheet"
    if rel_path == SUMMARY_NAME:
        return "qualification-summary"
    return "other"


def _json_identity(rel_path: str, value: Any) -> dict[str, Any]:
    """Extract reproducible identity metadata from a parsed JSON artifact."""
    if not isinstance(value, Mapping):
        return {}
    identity: dict[str, Any] = {}
    if rel_path in _SOURCE_IDENTITY_FILES:
        if isinstance(value.get("repository"), str):
            identity["repository"] = value["repository"]
        if isinstance(value.get("commit"), str) and value["commit"]:
            identity["commit"] = value["commit"]
    elif rel_path == "overlay-contract.json":
        for key in ("repository", "implementation_head"):
            if isinstance(value.get(key), str):
                identity[key] = value[key]
    elif rel_path == "model-fingerprint.json":
        for key in ("robot", "sha256"):
            if isinstance(value.get(key), str):
                identity[key] = value[key]
    elif rel_path == "runtime/ros.json":
        for key in ("domain_id", "rmw_implementation", "dds_profile"):
            if value.get(key) is not None:
                identity[key] = value[key]
    elif rel_path == "rosbag/rosbag-metadata.json":
        topics = value.get("topics")
        identity["message_count"] = value.get("message_count")
        identity["topic_count"] = len(topics) if isinstance(topics, Mapping) else None
        identity["duration_s"] = value.get("duration_s")
    elif rel_path == "verdict/gate-verdict.json":
        for key in ("status", "scenario_id"):
            if isinstance(value.get(key), str):
                identity[key] = value[key]
    elif rel_path == "source/source-locks.json":
        for key in ("repository", "implementation_head", "policy_commit", "policy_path", "mode"):
            if isinstance(value.get(key), str):
                identity[key] = value[key]
    elif rel_path == "source/dependency-locks.json":
        dependencies = value.get("dependencies")
        if isinstance(dependencies, Mapping):
            identity["dependency_count"] = len(dependencies)
    elif rel_path == "runtime/command.json":
        if isinstance(value.get("argv"), list):
            identity["argv_count"] = len(value["argv"])
    return identity


def _load_capture_bindings(suite_dir: Path, diagnostics: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    """Parse ``visual-capture-requests.jsonl`` into path -> binding metadata.

    Rejects duplicate canonical paths and duplicate event bindings.  A binding
    that references a nonexistent capture is reported as a diagnostic (the
    index still builds; ``validate_gate_f`` fails closed on the missing event).
    """
    request_path = suite_dir / _CAPTURE_REQUEST_NAME
    bindings: dict[str, dict[str, Any]] = {}
    event_owners: dict[str, str] = {}
    if not request_path.is_file():
        return bindings
    lines = request_path.read_text(encoding="utf-8").splitlines()
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            diagnostics.append({"code": "blank-capture-request-record", "line": line_number})
            continue
        try:
            record = json.loads(line)
        except (ValueError, json.JSONDecodeError) as error:
            diagnostics.append({"code": "corrupt-capture-request-record", "line": line_number, "detail": str(error)})
            continue
        if not isinstance(record, Mapping):
            diagnostics.append({"code": "invalid-capture-request-record", "line": line_number})
            continue
        raw_path = str(record.get("path", "")).strip().replace("\\", "/")
        if not raw_path:
            diagnostics.append({"code": "capture-request-missing-path", "line": line_number})
            continue
        if raw_path in bindings:
            raise ValueError(f"duplicate capture request binding for path: {raw_path}")
        event = record.get("event")
        if not isinstance(event, str) or not event:
            diagnostics.append({"code": "capture-request-missing-event", "path": raw_path})
            event = None
        if event is not None:
            if event in event_owners:
                raise ValueError(f"duplicate capture event binding: {event}")
            event_owners[event] = raw_path
        bindings[raw_path] = {
            "event": event,
            "scenario": record.get("scenario"),
            "attempt": record.get("attempt"),
            "execution_request": record.get("execution_request"),
            "frame_index": record.get("frame_index"),
            "timestamp": record.get("timestamp"),
            "camera": record.get("camera"),
        }
    return bindings


def _scenario_kinds(suite_dir: Path, files: list[str]) -> list[str]:
    kinds: set[str] = set()
    for rel_path in files:
        if not rel_path.startswith("scenario/") or not rel_path.endswith(".json"):
            continue
        value, _ = _parse_json(suite_dir / rel_path)
        if not isinstance(value, Mapping):
            continue
        integrated = value.get("integrated")
        if not isinstance(integrated, Mapping):
            continue
        kind = integrated.get("kind")
        kinds.add(kind if isinstance(kind, str) and kind else "positive")
    return sorted(kinds)


def build_evidence_index(suite_dir: Path, output: Path) -> dict[str, Any]:
    """Build a deterministic evidence index from real artifact bytes.

    ``output`` is excluded from the ``files`` list (the index excludes only
    itself).  If ``output`` collides with an existing non-index source artifact,
    this is rejected as output-as-input.
    """
    suite_resolved = Path(suite_dir).resolve()
    output_resolved = Path(output).resolve()
    try:
        rel_output = output_resolved.relative_to(suite_resolved).as_posix()
    except ValueError:
        rel_output = None
    if rel_output is not None and rel_output != INDEX_NAME and output_resolved.exists():
        raise ValueError(f"output-as-input: refusing to overwrite source artifact {rel_output}")

    diagnostics: list[dict[str, Any]] = []
    files = _canonical_relative_paths(suite_resolved)
    bindings = _load_capture_bindings(suite_resolved, diagnostics)

    entries: list[dict[str, Any]] = []
    bound_paths: set[str] = set()
    for rel_path in files:
        if rel_path == rel_output:
            continue
        path = suite_resolved / rel_path
        try:
            stat = path.stat()
        except (OSError, ValueError) as error:
            diagnostics.append({"code": "unreadable-file", "path": rel_path, "detail": str(error)})
            continue
        digest = _hash_file_stable(path)
        entry: dict[str, Any] = {
            "path": rel_path,
            "sha256": digest,
            "size": stat.st_size,
            "mode": oct(stat.st_mode & 0o777),
            "category": _category(rel_path),
        }
        parsed, parse_error = (None, None)
        if rel_path.endswith(".json"):
            parsed, parse_error = _parse_json(path)
        if parse_error is not None:
            diagnostics.append({"code": "unparseable-json", "path": rel_path, "detail": parse_error})
        else:
            identity = _json_identity(rel_path, parsed)
            if identity:
                entry["identity"] = identity
        if entry["category"] == "capture":
            binding = bindings.get(rel_path)
            if binding is None:
                entry.update({"event": None, "bound": False})
            else:
                bound_paths.add(rel_path)
                entry.update(
                    {
                        "event": binding["event"],
                        "scenario": binding["scenario"],
                        "attempt": binding["attempt"],
                        "execution_request": binding["execution_request"],
                        "frame_index": binding["frame_index"],
                        "timestamp": binding["timestamp"],
                        "camera": binding["camera"],
                        "bound": binding["event"] is not None,
                    }
                )
        entries.append(entry)
    for rel_path in sorted(set(bindings) - bound_paths):
        diagnostics.append({"code": "capture-request-without-source", "path": rel_path})

    index: dict[str, Any] = {
        "schema_version": 1,
        "kind": "integrated-evidence-index",
        "checksum_algorithm": "sha256",
        "suite_dir": str(suite_resolved),
        "scenario_kinds": _scenario_kinds(suite_resolved, files),
        "files": entries,
        "diagnostics": diagnostics,
    }
    checksum_payload = {key: value for key, value in index.items() if key != "index_checksum"}
    index["index_checksum"] = canonical_sha256(checksum_payload)
    _atomic_json(output_resolved, index)
    return index


def _entry_by_path(index: Mapping[str, Any]) -> dict[str, Mapping[str, Any]]:
    files = index.get("files")
    if not isinstance(files, list):
        return {}
    by_path: dict[str, Mapping[str, Any]] = {}
    for entry in files:
        if isinstance(entry, Mapping) and isinstance(entry.get("path"), str):
            if entry["path"] in by_path:
                raise ValueError(f"duplicate canonical path in index: {entry['path']}")
            by_path[entry["path"]] = entry
    return by_path


def _require(by_path: dict[str, Mapping[str, Any]], suffix: str, reason: str, reasons: list[str]) -> bool:
    present = any(path.endswith(suffix) for path in by_path)
    if not present:
        reasons.append(reason)
    return present


def _capture_entries(index: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    files = index.get("files")
    if not isinstance(files, list):
        return []
    return [entry for entry in files if isinstance(entry, Mapping) and entry.get("category") == "capture"]


def _required_event_sets(scenario_kinds: Sequence[str]) -> dict[str, tuple[str, ...]]:
    required: dict[str, tuple[str, ...]] = {"positive": REQUIRED_POSITIVE_EVENTS}
    kinds = set(scenario_kinds)
    if "cancel" in kinds:
        required["cancel"] = CANCEL_EVENTS
    if "safety" in kinds:
        required["safety"] = SAFETY_EVENTS
    return required


def validate_gate_f(index: Mapping[str, Any], *, output: Path | None = None) -> dict[str, Any]:
    """Evaluate the complete index; never fabricates a verdict.

    Returns ``verified-pass`` only when every required artifact category, both
    repositories' commit identities, dependency/source locks, runtime/ROS
    evidence, planning-scene journal, raw/evaluator/drain evidence, rosbag
    metadata/QoS/counts, verdict/cleanup/GPU/process reports, both integrated
    contact sheets, and every required bound capture event are present.
    """
    reasons: list[str] = []
    by_path = _entry_by_path(index)
    scenario_kinds = index.get("scenario_kinds")
    if not isinstance(scenario_kinds, list):
        scenario_kinds = []

    if index.get("checksum_algorithm") != "sha256":
        reasons.append("unsupported checksum algorithm: expected sha256")

    if INDEX_NAME in by_path:
        reasons.append("index self-inclusion detected")

    # Repository / source identities.
    source_identities = [entry for entry in by_path.values() if entry.get("category") == "source-identity"]
    sim_commit = next((entry.get("identity", {}).get("commit") for entry in source_identities if entry.get("identity", {}).get("repository") == "simulator"), None)
    prod_commit = next((entry.get("identity", {}).get("commit") for entry in source_identities if entry.get("identity", {}).get("repository") == "production"), None)
    if not sim_commit:
        reasons.append("missing simulator commit identity (absent or unreadable)")
    if not prod_commit:
        reasons.append("missing production commit identity (absent or unreadable)")
    _require(by_path, "source/source-locks.json", "missing source lock manifest", reasons)
    _require(by_path, "source/dependency-locks.json", "missing dependency locks", reasons)

    # Runtime / config / scenario / model.
    _require(by_path, "runtime/command.json", "missing runtime command evidence", reasons)
    _require(by_path, "runtime/environment.json", "missing runtime environment evidence", reasons)
    _require(by_path, "runtime/ros.json", "missing ROS domain/RMW/DDS runtime evidence", reasons)
    _require(by_path, "overlay-contract.json", "missing overlay contract", reasons)
    _require(by_path, "model-fingerprint.json", "missing model fingerprint", reasons)
    _require(by_path, "config/integrated-ompl.json", "missing configuration", reasons)
    if not any(path.startswith("scenario/") for path in by_path):
        reasons.append("missing scenario declaration")

    # MoveIt / journal / physics.
    _require(by_path, "moveit/moveit-plans.jsonl", "missing moveit plans", reasons)
    _require(by_path, "moveit/controller-results.jsonl", "missing controller results", reasons)
    _require(by_path, "planning-scene/planning-scene.jsonl", "missing planning scene journal", reasons)
    _require(by_path, "physics/physics_truth.jsonl", "missing raw physics truth", reasons)
    _require(by_path, "physics/evaluator.jsonl", "missing evaluator drain", reasons)
    _require(by_path, "physics/drain.jsonl", "missing drain evidence", reasons)

    # Verdict / cleanup / GPU / process.
    _require(by_path, "verdict/gate-verdict.json", "missing gate verdict", reasons)
    _require(by_path, "verdict/cleanup-report.json", "missing cleanup report", reasons)
    _require(by_path, "verdict/gpu-report.json", "missing gpu report", reasons)
    _require(by_path, "verdict/process-report.json", "missing process report", reasons)

    # Rosbag metadata/QoS/counts.
    _require(by_path, "rosbag/rosbag-metadata.json", "missing rosbag metadata (QoS/counts)", reasons)

    # Contact sheets (both roles).
    _require(by_path, "contact-sheet-integrated-agent.png", "missing contact sheet contact-sheet-integrated-agent.png", reasons)
    _require(by_path, "contact-sheet-integrated-user.png", "missing contact sheet contact-sheet-integrated-user.png", reasons)

    # Required visual events with exact binding metadata.
    required = _required_event_sets(scenario_kinds)
    captures = _capture_entries(index)
    present_events = {entry.get("event") for entry in captures if entry.get("bound")}
    for group in required.values():
        for event in group:
            if event not in present_events:
                reasons.append(f"missing required visual event: {event}")
    for entry in captures:
        if not entry.get("bound"):
            reasons.append(f"unbound capture: {entry.get('path')}")

    status = "verified-pass" if not reasons else "verified-fail"
    verdict: dict[str, Any] = {
        "schema_version": 1,
        "kind": "integrated-qualification-summary",
        "status": status,
        "reasons": reasons,
        "checksum_algorithm": index.get("checksum_algorithm"),
        "index_checksum": index.get("index_checksum"),
        "required_events": {group: list(events) for group, events in required.items()},
        "event_coverage": {
            "bound": sorted(event for event in present_events if event is not None),
            "missing": [reason for reason in reasons if reason.startswith("missing required visual event")],
        },
    }
    if output is not None:
        _atomic_json(Path(output), verdict)
    return verdict


def build_qualification_summary(
    suite_dir: Path,
    *,
    index_output: Path | None = None,
    summary_output: Path | None = None,
) -> dict[str, Any]:
    """Build the evidence index and write the qualification summary."""
    suite_resolved = Path(suite_dir).resolve()
    index_path = Path(index_output) if index_output is not None else suite_resolved / INDEX_NAME
    summary_path = Path(summary_output) if summary_output is not None else suite_resolved / SUMMARY_NAME
    index = build_evidence_index(suite_dir=suite_resolved, output=index_path)
    return validate_gate_f(index, output=summary_path)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite-dir", required=True, type=Path)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--validate", action="store_true", help="run validate_gate_f and print the verdict")
    parser.add_argument("--summary", type=Path, default=None, help="write qualification-summary.json")
    args = parser.parse_args(argv)
    suite_dir = Path(args.suite_dir).resolve()
    output = Path(args.output) if args.output else suite_dir / INDEX_NAME
    index = build_evidence_index(suite_dir=suite_dir, output=output)
    if args.summary:
        validate_gate_f(index, output=Path(args.summary))
    if args.validate:
        verdict = validate_gate_f(index)
        print(json.dumps(verdict, indent=2, sort_keys=True, ensure_ascii=False))
        return 0 if verdict["status"] == "verified-pass" else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
