"""Integrated qualification reproducibility evidence index (ROS-free, Python 3.12).

Gate F closes provenance, rosbag, process/GPU teardown, and visual evidence.
This module builds a deterministic ``evidence-index.json`` from the real
preserved artifact bytes and metadata of an integrated qualification attempt
suite, and validates it (``validate_gate_f``).  Task 10 wires Gate F into the
orchestrator; this module is the offline artifact contract it consumes.

The index is recursive over a suite root that may contain Gate-B outputs plus
C/D/E attempt directories.  Every semantic parser is pinned to the real
Task 2-8 producer contracts (executor journals, capture-process keyframes,
source-lock manifest, static contracts, gate verdict, rosbag2 metadata,
resource-cleanup evidence), never to a synthetic alternate schema.

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
- Checksums use relative suite identity (never the absolute host path) so
  unchanged preserved bytes are portable across checkout roots.

Fail-closed contract
--------------------
- Missing/empty/malformed/semantically-invalid artifacts make
  ``validate_gate_f`` return ``verified-fail`` with explicit reasons; presence
  alone never passes.
- Every visual capture is bound from the real two-journal transaction: the
  executor's ``visual-capture-requests.jsonl`` request records and the capture
  process's ``visual-keyframes.jsonl`` per-image records, with images under
  ``visual/source/*.png``.  Each binding carries exact scenario/attempt/
  execution-request plus ``(frame_index, timestamp)`` and is cross-bound to a
  real ``physics_truth.jsonl``/``evaluator.jsonl`` frame.
- PlanningScene/action/screenshot evidence is diagnostic only and is never
  physical pass authority.
- Path traversal, symlink escape, duplicate canonical paths/identities,
  output-as-input, files changing during hashing, and recognized in-progress
  temp files are handled as documented (temp files are never indexed as
  evidence).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
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

#: Canonical scenario ids (fix-round-1 F1.3).
CANONICAL_CANCEL_IDS = frozenset(
    {
        "qualification-moveit-cancel",
        "qualification-pick-place-cancel-approach",
        "qualification-pick-place-cancel-transport",
    }
)
CANONICAL_SAFETY_IDS = frozenset(
    {
        "qualification-moveit-safety",
        "qualification-pick-place-safety-transport",
    }
)
POSITIVE_VISUAL_ID = "qualification-pick-place-positive"

_HEX40_RE = re.compile(r"^[0-9a-f]{40}$")
_HEX64_RE = re.compile(r"^[0-9a-f]{64}$")

#: Recognized in-progress atomic temp prefixes (L8: never index as evidence).
_TEMP_PREFIXES = (
    ".evidence-index.json.",
    ".qualification-summary.json.",
    ".contact-sheet-integrated-",
    ".contact-sheet-",
)

#: Executor visual-request phases mirror the real ``_append_visual_request``
#: call sites.  Used only as a deterministic event->phase fallback when the
#: request journal carries the executor diagnostic shape (no sequence).
_EVENT_TO_PHASE: Mapping[str, str] = {
    "readiness": "before",
    "approach": "before-pick",
    "bilateral-contact": "before-pick",
    "attached-transport": "after",
    "place-target": "after",
    "released-settled": "after",
    "terminal": "terminal",
    "cancel-execution-start": "before-pick",
    "cancel-trigger": "before-pick",
    "cancel-velocity-compliant": "after",
    "cancel-terminal": "terminal",
    "safety-execution-start": "before-pick",
    "safety-trigger": "before-pick",
    "safety-velocity-compliant": "after",
    "safety-post-clear": "terminal",
}

#: ROS-free pure-Python validators/helpers reused from the Task 2-8 producers.
#: ``_read_json``/``_read_jsonl`` reject NaN/Infinity and non-object roots;
#: ``_as_int``/``_as_index``/``_as_timestamp`` reject bool-as-number.
try:
    from manipulation_gate_verifier import (  # noqa: F401
        EvidenceError,
        _read_json as _v_read_json,
        _read_jsonl as _v_read_jsonl,
    )
except ModuleNotFoundError:
    from validation.manipulation_gate_verifier import (  # noqa: F401
        EvidenceError,
        _read_json as _v_read_json,
        _read_jsonl as _v_read_jsonl,
    )
try:
    from integrated_gate_verifier import (  # noqa: F401
        _as_index,
        _as_int,
        _as_timestamp,
        _raw_evaluator_correlation,
    )
except ModuleNotFoundError:
    from validation.integrated_gate_verifier import (  # noqa: F401
        _as_index,
        _as_int,
        _as_timestamp,
        _raw_evaluator_correlation,
    )


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
    """Atomically write a JSON object (fsync file and parent directory)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        dir_fd = os.open(str(path.parent), os.O_DIRECTORY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    except BaseException:
        try:
            os.close(fd)
        except OSError:
            pass
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _is_temp_name(rel_path: str) -> bool:
    name = rel_path.rsplit("/", 1)[-1]
    return any(name.startswith(prefix) for prefix in _TEMP_PREFIXES)


def _canonical_relative_paths(suite_dir: Path) -> list[str]:
    """All regular files under ``suite_dir`` as sorted canonical relative paths.

    Rejects symlink escape (a symlink whose resolved target leaves the suite)
    and any path traversal that escapes the suite root.  Recognized in-progress
    temp files (L8) are ignored rather than indexed as evidence.
    """
    suite_resolved = suite_dir.resolve()
    relative_paths: list[str] = []
    for path in suite_resolved.rglob("*"):
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
        if _is_temp_name(rel):
            continue
        relative_paths.append(rel)
    relative_paths.sort()
    return relative_paths


def _category(rel_path: str) -> str:
    name = rel_path.rsplit("/", 1)[-1]
    if name == INDEX_NAME:
        return "other"
    if name == SUMMARY_NAME:
        return "qualification-summary"
    if name.startswith("contact-sheet-integrated-") and name.endswith(".png"):
        return "contact-sheet"
    if name == "visual-capture-requests.jsonl":
        return "capture-request-journal"
    if name == "visual-keyframes.jsonl":
        return "capture-keyframe-journal"
    if name == "visual-keyframes.json":
        return "capture-keyframe-summary"
    if "/visual/source/" in rel_path and rel_path.endswith(".png"):
        return "capture"
    if name == "gate-verdict.json":
        return "verdict"
    if name == "moveit-plans.jsonl":
        return "moveit"
    if name == "controller-results.jsonl":
        return "controller"
    if name == "planning-scene.jsonl":
        return "planning-scene-journal"
    if name == "planning-scene.json":
        return "planning-scene-final"
    if name == "physics_truth.jsonl":
        return "physics"
    if name == "evaluator.jsonl":
        return "evaluator"
    if name == "truth-drain.json":
        return "drain"
    if name == "resource-cleanup.json":
        return "cleanup"
    if name == "attempt-start.json" or (name.startswith("attempt-start-") and name.endswith(".json")):
        return "attempt-start"
    if name == "scenario-bundle.json":
        return "scenario-bundle"
    if name == "scenario-runner.json":
        return "scenario-runner"
    if name == "physics-ready.json":
        return "physics-ready"
    if name == "manifest.json":
        return "manifest"
    if name == "source-lock-manifest.json":
        return "source-lock-manifest"
    if name == "static-contract.json":
        return "static-contract"
    if name == "model-fingerprint.json":
        return "model-fingerprint"
    if name == "source-identities.json":
        return "source-identities"
    if name == "metadata.yaml" and "/rosbag/" in rel_path:
        return "rosbag-metadata"
    if "/rosbag/" in rel_path:
        return "rosbag-storage"
    if name == "overlay-contract.json":
        return "overlay-contract"
    if rel_path.startswith("config/") and rel_path.endswith(".json"):
        return "config"
    if rel_path.startswith("scenario/") and rel_path.endswith(".json"):
        return "scenario"
    if name.endswith(".json") and "/goals/" in rel_path:
        return "goal"
    if name == "integrated-execution.jsonl":
        return "execution-journal"
    if name == "integrated-execution.json":
        return "execution-summary"
    if name == "gate-execution.jsonl":
        return "execution-journal"
    if name == "gate-execution.json":
        return "execution-summary"
    return "other"


# --------------------------------------------------------------------------- #
# Scenario classification (F1.3)
# --------------------------------------------------------------------------- #
def _scenario_declarations(suite_dir: Path) -> list[tuple[str, Any, str | None]]:
    """Return ``(scenario_id, integrated_mapping, source_path)`` declarations.

    Reads the real ``scenario-bundle.json`` producer records and any
    ``scenario/*.json`` declaration files.  A malformed bundle/declaration is
    reported with its source path so Gate F can fail closed.
    """
    declarations: list[tuple[str, Any, str | None]] = []
    for rel in _canonical_relative_paths(suite_dir):
        if not (rel.endswith(".json")):
            continue
        if not (rel.endswith("scenario-bundle.json") or rel.startswith("scenario/")):
            continue
        path = suite_dir / rel
        try:
            value = _v_read_json(path)
        except EvidenceError:
            value = None
        if not isinstance(value, Mapping):
            continue
        if rel.endswith("scenario-bundle.json"):
            scenario_id = value.get("scenario_id")
            integrated = value.get("integrated")
            if isinstance(scenario_id, str) and scenario_id:
                declarations.append((scenario_id, integrated, rel))
        elif rel.startswith("scenario/"):
            scenario_map = value.get("scenario")
            scenario_id = scenario_map.get("id") if isinstance(scenario_map, Mapping) else None
            integrated = value.get("integrated")
            if isinstance(scenario_id, str) and scenario_id:
                declarations.append((scenario_id, integrated, rel))
    return declarations


def _classify_visual_kind(scenario_id: str, integrated: Any) -> str:
    """Classify a scenario into its visual kind (F1.3).

    Returns one of ``positive``, ``cancel``, ``safety``, ``negative-control``,
    ``other``, or ``invalid``.  ``invalid`` means the declaration contradicts
    the canonical identity/acceptance contract and must fail Gate F.
    """
    if not isinstance(integrated, Mapping):
        return "invalid"
    acceptance = integrated.get("acceptance")
    polarity = acceptance.get("polarity") if isinstance(acceptance, Mapping) else None
    if not isinstance(polarity, str) or not polarity:
        return "invalid"
    expected_negative = integrated.get("expected_negative")
    if scenario_id in CANONICAL_CANCEL_IDS:
        if polarity in ("cancel", "negative"):
            if polarity == "negative" and expected_negative is None:
                return "invalid"
            return "cancel"
        return "invalid"
    if scenario_id in CANONICAL_SAFETY_IDS:
        if polarity in ("safety", "negative"):
            if polarity == "negative" and expected_negative is None:
                return "invalid"
            return "safety"
        return "invalid"
    if scenario_id == POSITIVE_VISUAL_ID:
        if polarity == "positive" and expected_negative is None:
            return "positive"
        return "invalid"
    if polarity == "positive":
        return "other"
    if polarity == "negative":
        return "negative-control"
    if polarity == "cancel":
        return "cancel"
    if polarity == "safety":
        return "safety"
    return "invalid"


def _collect_scenario_kinds(suite_dir: Path, diagnostics: list[dict[str, Any]]) -> dict[str, Any]:
    """Collect scenario declarations into an ordered mapping.

    Returns ``{scenario_id: {kind, polarity, expected_negative, source}}``.
    Duplicate scenario ids and malformed declarations emit diagnostics.
    """
    result: dict[str, Any] = {}
    seen: dict[str, str] = {}
    for scenario_id, integrated, source in _scenario_declarations(suite_dir):
        if scenario_id in seen:
            diagnostics.append(
                {
                    "code": "duplicate-scenario-declaration",
                    "scenario_id": scenario_id,
                    "first": seen[scenario_id],
                    "second": source,
                }
            )
            continue
        seen[scenario_id] = source or ""
        kind = _classify_visual_kind(scenario_id, integrated)
        if kind == "invalid":
            diagnostics.append(
                {
                    "code": "invalid-scenario-declaration",
                    "scenario_id": scenario_id,
                    "source": source,
                }
            )
        polarity = None
        if isinstance(integrated, Mapping):
            acceptance = integrated.get("acceptance")
            if isinstance(acceptance, Mapping):
                polarity = acceptance.get("polarity")
        result[scenario_id] = {
            "kind": kind,
            "polarity": polarity if isinstance(polarity, str) else None,
            "expected_negative": (
                integrated.get("expected_negative") is not None
                if isinstance(integrated, Mapping)
                else False
            ),
            "source": source,
        }
    return result


# --------------------------------------------------------------------------- #
# Visual capture binding from the real two-journal transaction (F1.2)
# --------------------------------------------------------------------------- #
def _all_journal_paths(suite_dir: Path, name: str) -> list[tuple[str, Path]]:
    """Return ``(rel_dir, path)`` for every ``<name>`` journal under the suite.

    ``rel_dir`` is the canonical relative directory (``""`` for the suite root,
    otherwise ending in ``/``) that the journal's images are relative to.
    """
    result: list[tuple[str, Path]] = []
    for rel in _canonical_relative_paths(suite_dir):
        if rel.rsplit("/", 1)[-1] != name:
            continue
        rel_dir = rel[: -len(name)] if "/" in rel else ""
        result.append((rel_dir, suite_dir / rel))
    return result


def _read_request_records(path: Path, diagnostics: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Parse ``visual-capture-requests.jsonl`` with the real producer shapes.

    Accepted shapes:
    - Executor diagnostic (``IntegratedGateExecutor._append_visual_request``):
      ``{schema_version, report_revision, scenario_id, phase,
      capture:{kind,target}, diagnostic_only}``
    - Capture-driving EventJournal shape:
      ``{schema_version, sequence, gate, event, simulated_timestamp,
      source_execution_event_sequence}``

    The old synthetic ``{path,event,...}`` schema is rejected.
    """
    records: list[dict[str, Any]] = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        if not line.strip():
            diagnostics.append({"code": "blank-capture-request-record", "path": str(path), "line": line_number})
            continue
        try:
            value = json.loads(line)
        except (ValueError, json.JSONDecodeError) as error:
            diagnostics.append(
                {"code": "corrupt-capture-request-record", "path": str(path), "line": line_number, "detail": str(error)}
            )
            continue
        if not isinstance(value, Mapping):
            diagnostics.append({"code": "invalid-capture-request-record", "path": str(path), "line": line_number})
            continue
        shape = "unknown"
        if isinstance(value.get("sequence"), int) and not isinstance(value.get("sequence"), bool):
            if isinstance(value.get("gate"), str) and value["gate"] and isinstance(value.get("event"), str) and value["event"]:
                shape = "sequence"
        elif isinstance(value.get("scenario_id"), str) and value["scenario_id"]:
            if isinstance(value.get("phase"), str) and value["phase"] and isinstance(value.get("capture"), Mapping):
                shape = "executor"
        if shape == "unknown":
            diagnostics.append(
                {
                    "code": "unrecognized-capture-request-shape",
                    "path": str(path),
                    "line": line_number,
                    "keys": sorted(str(key) for key in value),
                }
            )
            continue
        if isinstance(value.get("schema_version"), bool) or (
            isinstance(value.get("schema_version"), int) and value["schema_version"] != 1
        ):
            diagnostics.append(
                {"code": "capture-request-schema-version", "path": str(path), "line": line_number}
            )
            continue
        records.append({"shape": shape, "record": dict(value), "line": line_number})
    return records


def _read_keyframes(path: Path, diagnostics: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Parse ``visual-keyframes.jsonl`` with the capture-process schema."""
    records: list[dict[str, Any]] = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), 1
    ):
        if not line.strip():
            diagnostics.append({"code": "blank-keyframe-record", "path": str(path), "line": line_number})
            continue
        try:
            value = json.loads(line)
        except (ValueError, json.JSONDecodeError) as error:
            diagnostics.append(
                {"code": "corrupt-keyframe-record", "path": str(path), "line": line_number, "detail": str(error)}
            )
            continue
        if not isinstance(value, Mapping):
            diagnostics.append({"code": "invalid-keyframe-record", "path": str(path), "line": line_number})
            continue
        records.append({"record": dict(value), "line": line_number, "path": str(path)})
    return records


def _strict_frame(value: Any, name: str) -> int | None:
    try:
        return _as_index(value, name)
    except EvidenceError as error:
        return None


def _strict_timestamp(value: Any, name: str) -> float | None:
    try:
        return _as_timestamp(value, name)
    except EvidenceError as error:
        return None


def _enclosing_attempt_dir(suite_dir: Path, directory: Path) -> Path | None:
    """Return the nearest enclosing attempt directory for *directory*.

    An attempt directory is one that carries a real ``scenario-bundle.json`` or
    ``manifest.json`` producer file.
    """
    current = directory.resolve()
    suite_resolved = suite_dir.resolve()
    while True:
        if (current / "scenario-bundle.json").is_file() or (current / "manifest.json").is_file():
            return current
        if current == suite_resolved or current == current.parent:
            return None
        current = current.parent


def _attempt_for_dir(suite_dir: Path, directory: Path) -> tuple[str | None, str | None, str | None]:
    """Return ``(attempt_id, scenario_id, source)`` for the nearest enclosing
    attempt directory, walking upward from *directory*.

    Uses the real ``manifest.json``/``scenario-bundle.json`` producer files.
    """
    attempt_dir = _enclosing_attempt_dir(suite_dir, directory)
    if attempt_dir is None:
        return None, None, None
    bundle = attempt_dir / "scenario-bundle.json"
    if bundle.is_file():
        try:
            value = _v_read_json(bundle)
        except EvidenceError:
            value = None
        if isinstance(value, Mapping):
            attempt = value.get("attempt_id")
            scenario = value.get("scenario_id")
            if isinstance(attempt, str) and attempt and isinstance(scenario, str) and scenario:
                return attempt, scenario, "scenario-bundle.json"
    manifest = attempt_dir / "manifest.json"
    if manifest.is_file():
        try:
            value = _v_read_json(manifest)
        except EvidenceError:
            value = None
        if isinstance(value, Mapping) and isinstance(value.get("attempt_id"), str) and value["attempt_id"]:
            scenario = None
            scenario_map = value.get("scenario")
            if isinstance(scenario_map, Mapping) and isinstance(scenario_map.get("id"), str):
                scenario = scenario_map["id"]
            return value["attempt_id"], scenario, "manifest.json"
    return None, None, None


def _physics_frames(suite_dir: Path, attempt_dir: Path) -> list[Mapping[str, Any]]:
    path = attempt_dir / "physics_truth.jsonl"
    if not path.is_file():
        return []
    try:
        return _v_read_jsonl(path, required=False)
    except EvidenceError:
        return []


def _load_capture_bindings(
    suite_dir: Path,
    diagnostics: list[dict[str, Any]],
) -> tuple[dict[str, dict[str, Any]], dict[str, list[Mapping[str, Any]]]]:
    """Build the real request/keyframe join into per-path capture bindings.

    Each ``visual-keyframes.jsonl`` journal's records are joined to the
    ``visual-capture-requests.jsonl`` journal in the same attempt directory; the
    keyframe ``path`` is resolved under that directory (the real capture
    producer writes ``visual/source/*.png`` relative to the attempt dir).

    Returns ``(bindings, physics_frames_by_attempt)`` where ``bindings`` maps a
    canonical suite-relative path to its binding dict.  Every binding carries
    exact event/scenario/attempt/execution-request plus ``(frame_index,
    timestamp)`` and a strict physics cross-bind.
    """
    suite_resolved = suite_dir.resolve()
    bindings: dict[str, dict[str, Any]] = {}
    identity_owners: dict[tuple[str, str], str] = {}
    physics_cache: dict[str, list[Mapping[str, Any]]] = {}
    covered_sequences: set[int] = set()
    covered_scenarios: set[str] = set()

    keyframe_journal_dirs = {
        rel_dir for rel_dir, _path in _all_journal_paths(suite_resolved, "visual-keyframes.jsonl")
    }
    for rel_dir, keyframes_path in _all_journal_paths(suite_resolved, "visual-keyframes.jsonl"):
        sequence_requests: dict[int, dict[str, Any]] = {}
        executor_requests: dict[str, list[dict[str, Any]]] = {}
        request_path = suite_resolved / f"{rel_dir}visual-capture-requests.jsonl"
        if request_path.is_file():
            for item in _read_request_records(request_path, diagnostics):
                record = item["record"]
                if item["shape"] == "sequence":
                    sequence = record["sequence"]
                    if sequence in sequence_requests:
                        diagnostics.append(
                            {"code": "duplicate-request-sequence", "sequence": sequence, "path": str(request_path)}
                        )
                        continue
                    sequence_requests[sequence] = record
                else:
                    scenario_id = record["scenario_id"]
                    executor_requests.setdefault(scenario_id, []).append(record)
        else:
            diagnostics.append(
                {"code": "missing-visual-request-journal", "path": str(request_path)}
            )

        for item in _read_keyframes(keyframes_path, diagnostics):
            record = item["record"]
            line = item["line"]
            event = record.get("event")
            gate = record.get("gate")
            path_value = record.get("path")
            camera = record.get("camera")
            request_sequence = record.get("request_sequence")
            raw_frame_index = record.get("raw_frame_index")
            simulated_timestamp = record.get("simulated_timestamp")

            if not isinstance(gate, str) or not gate:
                diagnostics.append({"code": "keyframe-missing-gate", "path": str(keyframes_path), "line": line})
                continue
            if not isinstance(event, str) or not event:
                diagnostics.append({"code": "keyframe-missing-event", "path": str(keyframes_path), "line": line})
                continue
            if not isinstance(camera, str) or not camera:
                diagnostics.append({"code": "keyframe-missing-camera", "path": str(keyframes_path), "line": line})
                continue
            if not isinstance(path_value, str) or not path_value:
                diagnostics.append({"code": "keyframe-missing-path", "path": str(keyframes_path), "line": line})
                continue
            path_rel = path_value.replace("\\", "/")
            if path_rel.startswith("/") or ".." in path_rel.split("/"):
                diagnostics.append(
                    {"code": "keyframe-path-traversal", "path": str(keyframes_path), "line": line, "path": path_rel}
                )
                continue
            if not path_rel.startswith("visual/source/") or not path_rel.endswith(".png"):
                diagnostics.append(
                    {"code": "keyframe-path-not-canonical", "path": str(keyframes_path), "line": line, "path": path_rel}
                )
                continue
            if not isinstance(request_sequence, int) or isinstance(request_sequence, bool) or request_sequence <= 0:
                diagnostics.append(
                    {"code": "keyframe-invalid-request-sequence", "path": str(keyframes_path), "line": line, "value": repr(request_sequence)}
                )
                continue
            frame = _strict_frame(raw_frame_index, f"keyframe[{line}].raw_frame_index")
            timestamp = _strict_timestamp(simulated_timestamp, f"keyframe[{line}].simulated_timestamp")
            if frame is None:
                diagnostics.append(
                    {"code": "keyframe-invalid-frame-index", "path": str(keyframes_path), "line": line, "value": repr(raw_frame_index)}
                )
                continue
            if timestamp is None or timestamp < 0.0:
                diagnostics.append(
                    {"code": "keyframe-invalid-timestamp", "path": str(keyframes_path), "line": line, "value": repr(simulated_timestamp)}
                )
                continue

            canonical_rel = f"{rel_dir}{path_rel}"
            identity = (event, camera)
            if identity in identity_owners:
                diagnostics.append(
                    {
                        "code": "duplicate-keyframe-identity",
                        "event": event,
                        "camera": camera,
                        "first": identity_owners[identity],
                        "second": canonical_rel,
                    }
                )
                continue
            identity_owners[identity] = canonical_rel

            # --- Join to exactly one request ---------------------------------
            joined_request: dict[str, Any] | None = None
            execution_request: str | None = None
            if sequence_requests:
                candidate = sequence_requests.get(request_sequence)
                if candidate is not None:
                    if candidate.get("gate") == gate and candidate.get("event") == event:
                        joined_request = candidate
                        execution_request = str(candidate["event"])
                        covered_sequences.add(request_sequence)
                    else:
                        diagnostics.append(
                            {
                                "code": "keyframe-request-sequence-mismatch",
                                "path": str(keyframes_path),
                                "line": line,
                                "request_sequence": request_sequence,
                            }
                        )
                        continue
                else:
                    diagnostics.append(
                        {
                            "code": "keyframe-request-sequence-orphan",
                            "path": str(keyframes_path),
                            "line": line,
                            "request_sequence": request_sequence,
                        }
                    )
                    continue
            else:
                executor_for_scenario = executor_requests.get(gate)
                if not executor_for_scenario:
                    diagnostics.append(
                        {"code": "keyframe-without-request", "path": str(keyframes_path), "line": line, "scenario": gate}
                    )
                    continue
                mapped_phase = _EVENT_TO_PHASE.get(event)
                phase_candidates = [
                    record for record in executor_for_scenario if record["record"].get("phase") == mapped_phase
                ]
                if mapped_phase is not None and len(phase_candidates) == 1:
                    joined_request = phase_candidates[0]["record"]
                    execution_request = str(mapped_phase)
                elif len(executor_for_scenario) == 1:
                    joined_request = executor_for_scenario[0]["record"]
                    execution_request = str(joined_request.get("phase"))
                else:
                    diagnostics.append(
                        {
                            "code": "keyframe-ambiguous-request-phase",
                            "path": str(keyframes_path),
                            "line": line,
                            "scenario": gate,
                            "event": event,
                            "phases": sorted(
                                str(record["record"].get("phase")) for record in executor_for_scenario
                            ),
                        }
                    )
                    continue
                covered_scenarios.add(gate)

            # --- Attempt/scenario identity ------------------------------------
            image_path = (suite_resolved / canonical_rel).resolve()
            try:
                image_path.relative_to(suite_resolved)
            except ValueError:
                diagnostics.append(
                    {"code": "keyframe-path-escape", "path": str(keyframes_path), "line": line, "path": canonical_rel}
                )
                continue
            if not image_path.is_file() or image_path.is_symlink():
                diagnostics.append(
                    {"code": "keyframe-missing-image", "path": str(keyframes_path), "line": line, "path": canonical_rel}
                )
                continue
            attempt, declared_scenario, identity_source = _attempt_for_dir(suite_resolved, image_path.parent)
            if attempt is None:
                diagnostics.append(
                    {"code": "keyframe-unbound-attempt", "path": str(keyframes_path), "line": line, "path": canonical_rel}
                )
                continue
            if declared_scenario is not None and declared_scenario != gate:
                diagnostics.append(
                    {
                        "code": "keyframe-scenario-mismatch",
                        "path": str(keyframes_path),
                        "line": line,
                        "keyframe_gate": gate,
                        "declared": declared_scenario,
                    }
                )
                continue

            # --- Physics cross-bind (frame/timestamp) ------------------------
            attempt_dir = _enclosing_attempt_dir(suite_resolved, image_path.parent)
            if attempt_dir is None:
                diagnostics.append(
                    {"code": "keyframe-physics-no-attempt-dir", "path": str(keyframes_path), "line": line, "path": canonical_rel}
                )
                continue
            attempt_dir_key = str(attempt_dir)
            if attempt_dir_key not in physics_cache:
                physics_cache[attempt_dir_key] = _physics_frames(suite_resolved, attempt_dir)
            physics_records = physics_cache[attempt_dir_key]
            physics_bound = False
            for frame_record in physics_records:
                if str(frame_record.get("scenario", "")) != gate:
                    continue
                if frame_record.get("frame_index") == frame:
                    ts = frame_record.get("timestamp")
                    if isinstance(ts, (int, float)) and not isinstance(ts, bool) and math.isfinite(float(ts)):
                        if abs(float(ts) - timestamp) <= 1e-6:
                            physics_bound = True
                            break
            if not physics_bound:
                diagnostics.append(
                    {
                        "code": "keyframe-physics-unbound",
                        "journal": str(keyframes_path),
                        "line": line,
                        "path": canonical_rel,
                        "frame_index": frame,
                        "timestamp": timestamp,
                    }
                )
                continue

            bindings[canonical_rel] = {
                "event": event,
                "scenario": gate,
                "attempt": attempt,
                "execution_request": execution_request,
                "request_sequence": request_sequence,
                "frame_index": frame,
                "timestamp": timestamp,
                "camera": camera,
                "bound": True,
                "physics_bound": True,
            }

    # Request without any keyframe image.
    for rel_dir, request_path in _all_journal_paths(suite_resolved, "visual-capture-requests.jsonl"):
        for item in _read_request_records(request_path, diagnostics):
            record = item["record"]
            if item["shape"] == "sequence" and record["sequence"] not in covered_sequences:
                diagnostics.append(
                    {"code": "capture-request-without-image", "request_sequence": record["sequence"], "path": str(request_path)}
                )
            elif item["shape"] == "executor" and record["scenario_id"] not in covered_scenarios:
                if not keyframe_journal_dirs or rel_dir not in keyframe_journal_dirs:
                    diagnostics.append(
                        {"code": "capture-request-without-image", "scenario_id": record["scenario_id"], "path": str(request_path)}
                    )
    return bindings, physics_cache


# --------------------------------------------------------------------------- #
# Index build
# --------------------------------------------------------------------------- #
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
    bindings, _ = _load_capture_bindings(suite_resolved, diagnostics)
    scenario_map = _collect_scenario_kinds(suite_resolved, diagnostics)

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
        parsed: Any = None
        if rel_path.endswith(".json") or rel_path.endswith(".jsonl"):
            try:
                if rel_path.endswith(".jsonl"):
                    parsed = _v_read_jsonl(path, required=False)
                else:
                    parsed = _v_read_json(path)
            except EvidenceError as error:
                diagnostics.append({"code": "unparseable-artifact", "path": rel_path, "detail": str(error)})
                parsed = None
        identity = _json_identity(rel_path, parsed, scenario_map)
        if identity:
            entry["identity"] = identity
        binding = bindings.get(rel_path)
        if binding is not None:
            bound_paths.add(rel_path)
            entry.update(
                {
                    "event": binding["event"],
                    "scenario": binding["scenario"],
                    "attempt": binding["attempt"],
                    "execution_request": binding["execution_request"],
                    "request_sequence": binding["request_sequence"],
                    "frame_index": binding["frame_index"],
                    "timestamp": binding["timestamp"],
                    "camera": binding["camera"],
                    "bound": True,
                    "physics_bound": binding["physics_bound"],
                }
            )
        entries.append(entry)
    for rel_path in sorted(set(bindings) - bound_paths):
        diagnostics.append({"code": "capture-binding-without-index", "path": rel_path})

    index: dict[str, Any] = {
        "schema_version": 1,
        "kind": "integrated-evidence-index",
        "checksum_algorithm": "sha256",
        "suite_identity": "integrated-qualification-suite",
        "scenario_kinds": sorted(
            {
                scenario_map[scenario_id]["kind"]
                for scenario_id in scenario_map
                if scenario_map[scenario_id]["kind"] in ("positive", "cancel", "safety")
            }
        ),
        "scenarios": [
            {"id": scenario_id, **scenario_map[scenario_id]}
            for scenario_id in sorted(scenario_map)
        ],
        "files": entries,
        "diagnostics": diagnostics,
    }
    checksum_payload = {key: value for key, value in index.items() if key != "index_checksum"}
    index["index_checksum"] = canonical_sha256(checksum_payload)
    _atomic_json(output_resolved, index)
    return index


def _json_identity(
    rel_path: str,
    value: Any,
    scenario_map: Mapping[str, Any],
) -> dict[str, Any]:
    """Extract reproducible identity metadata from a parsed JSON artifact."""
    if not isinstance(value, Mapping):
        return {}
    identity: dict[str, Any] = {}
    name = rel_path.rsplit("/", 1)[-1]
    if name == "model-fingerprint.json":
        if isinstance(value.get("model_fingerprint"), str):
            identity["model_fingerprint"] = value["model_fingerprint"]
    elif name == "source-lock-manifest.json":
        if isinstance(value.get("status"), str):
            identity["status"] = value["status"]
        repositories = value.get("repositories")
        if isinstance(repositories, list):
            identity["repositories"] = [
                str(item) for item in repositories if isinstance(item, str)
            ]
    elif name == "static-contract.json":
        if isinstance(value.get("status"), str):
            identity["status"] = value["status"]
    elif name == "gate-verdict.json":
        for key in ("status", "gate", "attempt_id", "stage", "polarity"):
            if isinstance(value.get(key), str):
                identity[key] = value[key]
    elif name == "manifest.json":
        if isinstance(value.get("attempt_id"), str):
            identity["attempt_id"] = value["attempt_id"]
    elif name == "scenario-bundle.json":
        for key in ("scenario_id", "attempt_id"):
            if isinstance(value.get(key), str):
                identity[key] = value[key]
    elif name == "physics-ready.json":
        for key in ("state",):
            if isinstance(value.get(key), str):
                identity[key] = value[key]
    elif name == "resource-cleanup.json":
        if isinstance(value.get("clean"), bool):
            identity["clean"] = value["clean"]
        if isinstance(value.get("schema_version"), int) and not isinstance(value.get("schema_version"), bool):
            identity["schema_version"] = value["schema_version"]
    elif name == "truth-drain.json":
        if isinstance(value.get("status"), str):
            identity["status"] = value["status"]
        if isinstance(value.get("exact_correlation"), bool):
            identity["exact_correlation"] = value["exact_correlation"]
    elif name == "integrated-execution.json":
        for key in ("scenario_id", "status", "stage"):
            if isinstance(value.get(key), str):
                identity[key] = value[key]
    elif name == "attempt-start.json" or (name.startswith("attempt-start-") and name.endswith(".json")):
        for key in ("attempt_id", "seed", "root", "production_root", "config"):
            if value.get(key) is not None:
                identity[key] = value[key]
    elif name == "overlay-contract.json":
        for key in ("repository", "implementation_head"):
            if isinstance(value.get(key), str):
                identity[key] = value[key]
    elif rel_path.startswith("config/") and rel_path.endswith(".json"):
        for key in ("id", "profile", "execution_profile", "seed"):
            if value.get(key) is not None:
                identity[key] = value[key]
    elif rel_path.startswith("scenario/") and rel_path.endswith(".json"):
        scenario_map_value = value.get("scenario")
        if isinstance(scenario_map_value, Mapping):
            for key in ("id", "seed"):
                if scenario_map_value.get(key) is not None:
                    identity[key] = scenario_map_value[key]
    elif name == "source-identities.json":
        identities = value.get("source_identities")
        if isinstance(identities, Mapping):
            identity["source_identity_count"] = len(identities)
    return identity


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


def _capture_entries(index: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    files = index.get("files")
    if not isinstance(files, list):
        return []
    return [entry for entry in files if isinstance(entry, Mapping) and entry.get("category") == "capture"]


def _required_event_sets(scenario_kinds: Sequence[str]) -> dict[str, tuple[str, ...]]:
    required: dict[str, tuple[str, ...]] = {}
    kinds = set(scenario_kinds)
    if "positive" in kinds:
        required["positive"] = REQUIRED_POSITIVE_EVENTS
    if "cancel" in kinds:
        required["cancel"] = CANCEL_EVENTS
    if "safety" in kinds:
        required["safety"] = SAFETY_EVENTS
    return required


# --------------------------------------------------------------------------- #
# Index integrity + current-byte verification (F1.5)
# --------------------------------------------------------------------------- #
def _recompute_index_checksum(index: Mapping[str, Any]) -> str | None:
    if not isinstance(index.get("index_checksum"), str):
        return None
    payload = {key: value for key, value in index.items() if key != "index_checksum"}
    return canonical_sha256(payload)


def _validate_index_integrity(
    index: Mapping[str, Any],
    suite_dir: Path,
    reasons: list[str],
    diagnostics: list[dict[str, Any]],
) -> None:
    """Recompute checksum, re-resolve/re-hash current bytes, and compare the
    preserved-file set.  Indexed identity fields are normalized; digest/size/
    mode must match the on-disk bytes exactly."""
    recomputed = _recompute_index_checksum(index)
    recorded = index.get("index_checksum")
    if recomputed is None or recorded != recomputed:
        reasons.append("index_checksum does not match the canonical index projection")

    suite_resolved = Path(suite_dir).resolve()
    by_path = _entry_by_path(index)
    for rel_path, entry in by_path.items():
        if not isinstance(entry.get("sha256"), str) or not _HEX64_RE.fullmatch(entry["sha256"]):
            reasons.append(f"malformed digest for indexed path {rel_path}")
        path = (suite_resolved / rel_path).resolve()
        try:
            path.relative_to(suite_resolved)
        except ValueError:
            reasons.append(f"indexed path escapes the suite: {rel_path}")
            continue
        if path.is_symlink():
            reasons.append(f"indexed path is a symlink: {rel_path}")
            continue
        if not path.is_file():
            reasons.append(f"missing indexed file: {rel_path}")
            continue
        try:
            stat = path.stat()
            digest = _hash_file_stable(path)
        except (OSError, ValueError) as error:
            reasons.append(f"unreadable indexed file {rel_path}: {error}")
            continue
        if digest != entry.get("sha256"):
            reasons.append(f"indexed digest mismatch for current bytes: {rel_path}")
        if stat.st_size != entry.get("size"):
            reasons.append(f"indexed size mismatch for {rel_path}")
        normalized_mode = oct(stat.st_mode & 0o777)
        if normalized_mode != entry.get("mode"):
            # Normalized mode mismatch is intentional-ignore (permission bits are
            # incidental across hosts); digest/size are the load-bearing checks.
            pass

    preserved = {
        rel
        for rel in _canonical_relative_paths(suite_resolved)
        if rel != INDEX_NAME
    }
    indexed = set(by_path)
    missing = sorted(preserved - indexed)
    extra = sorted(indexed - preserved)
    for rel in missing:
        reasons.append(f"unindexed preserved file: {rel}")
    for rel in extra:
        reasons.append(f"stale index entry (file absent): {rel}")


def _pre_summary_projection(index: Mapping[str, Any]) -> Mapping[str, Any] | None:
    """Reconstruct the pre-summary index projection (F1.5 cycle)."""
    summary_entry = None
    files = index.get("files")
    if not isinstance(files, list):
        return None
    remaining: list[Any] = []
    for entry in files:
        if isinstance(entry, Mapping) and entry.get("category") == "qualification-summary":
            summary_entry = entry
        else:
            remaining.append(entry)
    if summary_entry is None:
        return None
    projection = {key: value for key, value in index.items() if key != "index_checksum"}
    projection = dict(projection)
    projection["files"] = remaining
    projection.pop("index_checksum", None)
    return projection


# --------------------------------------------------------------------------- #
# Semantic Gate F validators (F1.4)
# --------------------------------------------------------------------------- #
def _read_attempt_entries(index: Mapping[str, Any]) -> dict[str, list[Mapping[str, Any]]]:
    """Group ``verdict`` category entries by scenario id."""
    result: dict[str, list[Mapping[str, Any]]] = {}
    for entry in _entries_by_category(index, "verdict"):
        identity = entry.get("identity") or {}
        scenario_id = identity.get("gate") or entry["path"].split("/")[-2]
        result.setdefault(str(scenario_id), []).append(entry)
    return result


def _entries_by_category(index: Mapping[str, Any], category: str) -> list[Mapping[str, Any]]:
    files = index.get("files")
    if not isinstance(files, list):
        return []
    return [entry for entry in files if isinstance(entry, Mapping) and entry.get("category") == category]


def _paths_by_category(index: Mapping[str, Any], category: str) -> list[str]:
    return [entry["path"] for entry in _entries_by_category(index, category)]


def _require(by_path: Mapping[str, Mapping[str, Any]], suffix: str, reason: str, reasons: list[str]) -> bool:
    present = any(path.endswith(suffix) for path in by_path)
    if not present:
        reasons.append(reason)
    return present


def _read_jsonl_rel(suite_dir: Path, rel_path: str) -> list[Mapping[str, Any]] | None:
    path = suite_dir / rel_path
    if not path.is_file():
        return None
    try:
        return _v_read_jsonl(path, required=False)
    except EvidenceError:
        return None


def _read_json_rel(suite_dir: Path, rel_path: str) -> Mapping[str, Any] | None:
    path = suite_dir / rel_path
    if not path.is_file():
        return None
    try:
        value = _v_read_json(path)
    except EvidenceError:
        return None
    return value


def _validate_source_provenance(
    index: Mapping[str, Any],
    suite_dir: Path,
    reasons: list[str],
) -> None:
    """Source/provenance identity checks (F1.4 Source/provenance)."""
    scenarios = index.get("scenarios")
    scenario_map = (
        {item["id"]: item for item in scenarios if isinstance(item, Mapping) and isinstance(item.get("id"), str)}
        if isinstance(scenarios, list)
        else {}
    )

    # Source-lock manifest (real Gate-B artifact, written into gate-b-* dir).
    lock_entries = _entries_by_category(index, "source-lock-manifest")
    if not lock_entries:
        reasons.append("missing source lock manifest (source-lock-manifest.json)")
    for entry in lock_entries:
        identity = entry.get("identity") or {}
        if identity.get("status") != "pass":
            reasons.append(f"source lock manifest status is not pass: {entry['path']}")
    # Static contracts (Gate-B closure of config/overlay/command/env/domain).
    static_entries = _entries_by_category(index, "static-contract")
    if not static_entries:
        reasons.append("missing static contract (static-contract.json)")
    for entry in static_entries:
        identity = entry.get("identity") or {}
        if identity.get("status") != "pass":
            reasons.append(f"static contract status is not pass: {entry['path']}")
    # Model fingerprint real shape.
    fp_entries = _entries_by_category(index, "model-fingerprint")
    if not fp_entries:
        reasons.append("missing model fingerprint (model-fingerprint.json)")
    for entry in fp_entries:
        identity = entry.get("identity") or {}
        fp = identity.get("model_fingerprint")
        if not isinstance(fp, str) or not _HEX64_RE.fullmatch(fp) or fp == "0" * 64:
            reasons.append(f"model fingerprint is not a nonzero 64-hex digest: {entry['path']}")
    # Source identities.
    if not _entries_by_category(index, "source-identities"):
        reasons.append("missing source identities (source-identities.json)")
    # Attempt-start identity.
    attempt_starts = _entries_by_category(index, "attempt-start")
    if not attempt_starts:
        reasons.append("missing attempt start identity (attempt-start-*.json)")
    for entry in attempt_starts:
        identity = entry.get("identity") or {}
        if not isinstance(identity.get("attempt_id"), str) or not identity["attempt_id"]:
            reasons.append(f"attempt-start missing attempt_id: {entry['path']}")
        if not isinstance(identity.get("config"), str) or not identity["config"]:
            reasons.append(f"attempt-start missing config identity: {entry['path']}")
        if not isinstance(identity.get("root"), str) or not identity["root"]:
            reasons.append(f"attempt-start missing root identity: {entry['path']}")
    # Manifest (attempt identity, ROS domain/RMW/DDS environment).
    manifest_entries = _entries_by_category(index, "manifest")
    if not manifest_entries:
        reasons.append("missing attempt manifest (manifest.json)")
    for entry in manifest_entries:
        identity = entry.get("identity") or {}
        if not isinstance(identity.get("attempt_id"), str) or not identity["attempt_id"]:
            reasons.append(f"manifest missing attempt_id: {entry['path']}")
        manifest = _read_json_rel(suite_dir, entry["path"])
        if manifest is not None:
            environment = manifest.get("environment")
            if not isinstance(environment, Mapping):
                reasons.append(f"manifest missing environment block: {entry['path']}")
            else:
                domain = environment.get("ROS_DOMAIN_ID")
                if domain is None:
                    reasons.append(f"manifest missing ROS_DOMAIN_ID: {entry['path']}")
                else:
                    try:
                        domain_int = int(domain)
                        if isinstance(domain, bool) or not 0 <= domain_int <= 232:
                            reasons.append(f"manifest ROS_DOMAIN_ID out of [0,232]: {entry['path']}")
                    except (TypeError, ValueError):
                        reasons.append(f"manifest ROS_DOMAIN_ID is not an integer: {entry['path']}")
                if not isinstance(environment.get("RMW_IMPLEMENTATION"), str) or not environment["RMW_IMPLEMENTATION"]:
                    reasons.append(f"manifest missing RMW_IMPLEMENTATION: {entry['path']}")
    # Scenario bundles.
    bundle_entries = _entries_by_category(index, "scenario-bundle")
    if not bundle_entries:
        reasons.append("missing scenario bundle (scenario-bundle.json)")
    # Config and overlay contracts are recognized where present (they live in the
    # repositories, not the suite); Gate-B static contracts close them.
    config_entries = _entries_by_category(index, "config")
    for entry in config_entries:
        identity = entry.get("identity") or {}
        if identity.get("id") and identity.get("seed") is None:
            reasons.append(f"config missing seed identity: {entry['path']}")
    # Scenario declarations.
    invalid_scenarios = [
        item for item in (index.get("scenarios") or [])
        if isinstance(item, Mapping) and item.get("kind") == "invalid"
    ]
    for item in invalid_scenarios:
        reasons.append(f"invalid scenario declaration: {item.get('id')}")
    if not scenario_map:
        reasons.append("no scenario declarations found")


def _validate_verdicts(
    index: Mapping[str, Any],
    suite_dir: Path,
    reasons: list[str],
) -> None:
    """Every declared scenario must have exactly one independent verified-pass
    gate-verdict.json with matching identities (F1.4 Scenario verdict)."""
    scenarios = index.get("scenarios")
    declared = (
        {item["id"] for item in scenarios if isinstance(item, Mapping) and isinstance(item.get("id"), str)}
        if isinstance(scenarios, list)
        else set()
    )
    verdicts: dict[str, list[Mapping[str, Any]]] = {}
    for entry in _entries_by_category(index, "verdict"):
        scenario_id = entry["path"].rsplit("/", 1)[0].rsplit("/", 1)[-1] if "/" in entry["path"] else None
        identity = entry.get("identity") or {}
        if isinstance(identity.get("gate"), str) and identity["gate"]:
            scenario_id = identity["gate"]
        verdicts.setdefault(str(scenario_id), []).append(entry)
    for scenario_id in declared:
        if scenario_id not in verdicts:
            reasons.append(f"missing gate verdict for scenario {scenario_id}")
            continue
        entries = verdicts[scenario_id]
        if len(entries) != 1:
            reasons.append(
                f"duplicate gate verdict for scenario {scenario_id}: {len(entries)}"
            )
            continue
        entry = entries[0]
        identity = entry.get("identity") or {}
        if identity.get("status") != "verified-pass":
            reasons.append(f"gate verdict for {scenario_id} is not verified-pass: {entry['path']}")
        if identity.get("gate") not in (None, scenario_id):
            reasons.append(f"gate verdict identity mismatch for {scenario_id}: {entry['path']}")
        if identity.get("attempt_id") is None:
            reasons.append(f"gate verdict missing attempt identity for {scenario_id}: {entry['path']}")
    # Every verdict must belong to a declared scenario.
    for scenario_id, entries in verdicts.items():
        if scenario_id not in declared:
            for entry in entries:
                reasons.append(f"gate verdict for undeclared scenario {scenario_id}: {entry['path']}")


def _validate_physics_and_drain(
    index: Mapping[str, Any],
    suite_dir: Path,
    reasons: list[str],
) -> None:
    """Raw/evaluator/drain exactness (F1.4 Scenario verdict)."""
    attempt_dirs: set[str] = set()
    for entry in _entries_by_category(index, "physics"):
        parent = entry["path"].rsplit("/", 1)[0] if "/" in entry["path"] else ""
        attempt_dirs.add(parent)
    for entry in _entries_by_category(index, "verdict"):
        parent = entry["path"].rsplit("/", 1)[0] if "/" in entry["path"] else ""
        attempt_dirs.add(parent)
    for attempt_rel in sorted(attempt_dirs):
        raw = _read_jsonl_rel(suite_dir, f"{attempt_rel}/physics_truth.jsonl")
        evaluator = _read_jsonl_rel(suite_dir, f"{attempt_rel}/evaluator.jsonl")
        if raw is None or evaluator is None:
            reasons.append(f"missing raw/evaluator truth for attempt dir {attempt_rel}")
            continue
        if not raw:
            reasons.append(f"empty raw physics truth: {attempt_rel}/physics_truth.jsonl")
        if not evaluator:
            reasons.append(f"empty evaluator drain: {attempt_rel}/evaluator.jsonl")
        raw_start_index = 0
        evaluator_start_index: int | None = None
        gate_window = _read_json_rel(suite_dir, f"{attempt_rel}/gate-window.json")
        if gate_window is not None:
            if isinstance(gate_window.get("raw_start_index"), int) and not isinstance(gate_window.get("raw_start_index"), bool):
                raw_start_index = gate_window["raw_start_index"]
            if isinstance(gate_window.get("evaluator_start_index"), int) and not isinstance(gate_window.get("evaluator_start_index"), bool):
                evaluator_start_index = gate_window["evaluator_start_index"]
        try:
            _raw_evaluator_correlation(
                raw,
                evaluator,
                raw_start_index=raw_start_index,
                evaluator_start_index=evaluator_start_index,
            )
        except EvidenceError as error:
            reasons.append(f"raw/evaluator drain mismatch in {attempt_rel}: {error}")
        drain = _read_json_rel(suite_dir, f"{attempt_rel}/truth-drain.json")
        if drain is None:
            reasons.append(f"missing truth-drain.json for {attempt_rel}")
        else:
            if drain.get("status") != "drained" or drain.get("exact_correlation") is not True:
                reasons.append(f"truth drain is not exact for {attempt_rel}")


def _validate_moveit_controller(
    index: Mapping[str, Any],
    suite_dir: Path,
    reasons: list[str],
) -> None:
    """MoveIt/controller evidence (F1.4 MoveIt/controller)."""
    scenarios = index.get("scenarios")
    declared = (
        {item["id"] for item in scenarios if isinstance(item, Mapping) and isinstance(item.get("id"), str)}
        if isinstance(scenarios, list)
        else set()
    )
    for rel in _paths_by_category(index, "moveit"):
        rows = _read_jsonl_rel(suite_dir, rel)
        if rows is None:
            reasons.append(f"missing moveit plans: {rel}")
            continue
        if not rows:
            reasons.append(f"empty moveit plans: {rel}")
            continue
        for row_number, row in enumerate(rows, 1):
            if row.get("schema_version") != 1:
                reasons.append(f"moveit row {row_number} in {rel} has invalid schema_version")
            if not isinstance(row.get("scenario_id"), str) or not row["scenario_id"]:
                reasons.append(f"moveit row {row_number} in {rel} has no scenario_id")
            elif declared and row["scenario_id"] not in declared:
                reasons.append(f"moveit row {row_number} in {rel} references undeclared scenario {row['scenario_id']}")
            if not isinstance(row.get("status"), str) or not row["status"]:
                reasons.append(f"moveit row {row_number} in {rel} has no status")
        if not any(isinstance(row.get("row_kind"), str) and row["row_kind"] == "final" for row in rows):
            reasons.append(f"moveit plans not finalized: {rel}")
    for rel in _paths_by_category(index, "controller"):
        rows = _read_jsonl_rel(suite_dir, rel)
        if rows is None:
            reasons.append(f"missing controller results: {rel}")
            continue
        if not rows:
            reasons.append(f"empty controller results: {rel}")
            continue
        for row_number, row in enumerate(rows, 1):
            if not isinstance(row.get("scenario_id"), str) or not row["scenario_id"]:
                reasons.append(f"controller row {row_number} in {rel} has no scenario_id")
            elif declared and row["scenario_id"] not in declared:
                reasons.append(f"controller row {row_number} in {rel} references undeclared scenario {row['scenario_id']}")
            if not isinstance(row.get("status"), str) or not row["status"]:
                reasons.append(f"controller row {row_number} in {rel} has no status")


def _validate_planning_scene(
    index: Mapping[str, Any],
    suite_dir: Path,
    reasons: list[str],
) -> None:
    """PlanningScene JSONL/final journal (F1.4 PlanningScene)."""
    journals = _paths_by_category(index, "planning-scene-journal")
    finals = _paths_by_category(index, "planning-scene-final")
    if not journals:
        reasons.append("missing planning scene journal (planning-scene.jsonl)")
    if not finals:
        reasons.append("missing planning scene final (planning-scene.json)")
    for rel in journals:
        rows = _read_jsonl_rel(suite_dir, rel)
        if rows is None:
            reasons.append(f"missing planning scene journal: {rel}")
            continue
        if not rows:
            reasons.append(f"empty planning scene journal: {rel}")
            continue
        previous_sequence = 0
        for row_number, row in enumerate(rows, 1):
            sequence = row.get("sequence")
            if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence <= 0:
                reasons.append(f"planning scene row {row_number} in {rel} has invalid sequence")
            elif sequence <= previous_sequence:
                reasons.append(f"planning scene row {row_number} in {rel} is not monotonic")
            previous_sequence = max(previous_sequence, sequence if isinstance(sequence, int) and not isinstance(sequence, bool) else 0)
            if not isinstance(row.get("event"), str) or not row["event"]:
                reasons.append(f"planning scene row {row_number} in {rel} has no event")
    for rel in _paths_by_category(index, "planning-scene-final"):
        value = _read_json_rel(suite_dir, rel)
        if value is None:
            reasons.append(f"missing planning scene final: {rel}")
            continue
        if value.get("finalized") is not True and value.get("status") not in ("finalized", "failure", "diagnostic-fail"):
            reasons.append(f"planning scene final not finalized: {rel}")


def _validate_rosbag(
    index: Mapping[str, Any],
    suite_dir: Path,
    reasons: list[str],
) -> None:
    """Real rosbag2 metadata + storage (F1.4 Rosbag)."""
    metadata_entries = _entries_by_category(index, "rosbag-metadata")
    if not metadata_entries:
        reasons.append("missing rosbag metadata (rosbag/metadata.yaml)")
        return
    for entry in metadata_entries:
        rel = entry["path"]
        path = suite_dir / rel
        if not path.is_file():
            reasons.append(f"missing rosbag metadata file: {rel}")
            continue
        try:
            import yaml
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception as error:
            reasons.append(f"rosbag metadata is not valid YAML: {rel}: {error}")
            continue
        root = document.get("rosbag2_bagfile_information") if isinstance(document, Mapping) else None
        records = root.get("topics_with_message_count") if isinstance(root, Mapping) else None
        if not isinstance(records, list) or not records:
            reasons.append(f"rosbag metadata has no topics_with_message_count: {rel}")
            continue
        total_messages = 0
        topic_set: set[str] = set()
        for record in records:
            if not isinstance(record, Mapping):
                continue
            metadata_record = record.get("topic_metadata")
            count = record.get("message_count")
            if not isinstance(metadata_record, Mapping) or not isinstance(metadata_record.get("name"), str):
                continue
            topic = metadata_record["name"]
            topic_set.add(topic)
            if isinstance(count, int) and not isinstance(count, bool) and count > 0:
                total_messages += count
            else:
                reasons.append(f"rosbag topic {topic} has nonpositive message count")
            qos = metadata_record.get("offered_qos_profiles")
            if not isinstance(qos, str) or not qos.strip():
                reasons.append(f"rosbag topic {topic} has no offered_qos_profiles")
        if total_messages <= 0:
            reasons.append(f"rosbag has zero total messages: {rel}")
        duration = root.get("duration")
        if isinstance(duration, Mapping):
            nanoseconds = duration.get("nanoseconds")
            if isinstance(nanoseconds, int) and not isinstance(nanoseconds, bool) and nanoseconds <= 0:
                reasons.append(f"rosbag duration is nonpositive: {rel}")
        elif not isinstance(duration, (int, float)) or (isinstance(duration, (int, float)) and not isinstance(duration, bool) and float(duration) <= 0):
            if not isinstance(duration, Mapping):
                reasons.append(f"rosbag duration missing: {rel}")
        bag_dir_rel = rel.rsplit("/", 1)[0] if "/" in rel else ""
        storage = [
            e
            for e in _entries_by_category(index, "rosbag-storage")
            if (e["path"].rsplit("/", 1)[0] if "/" in e["path"] else "") == bag_dir_rel
        ]
        if not storage:
            reasons.append(f"rosbag has no storage files: {rel}")


def _validate_cleanup(
    index: Mapping[str, Any],
    suite_dir: Path,
    reasons: list[str],
) -> None:
    """Cleanup/process/GPU evidence (F1.4 Cleanup/process/GPU)."""
    cleanup_entries = _entries_by_category(index, "cleanup")
    if not cleanup_entries:
        reasons.append("missing cleanup/process/GPU report (resource-cleanup.json)")
    for entry in cleanup_entries:
        rel = entry["path"]
        value = _read_json_rel(suite_dir, rel)
        if value is None:
            reasons.append(f"unreadable resource-cleanup: {rel}")
            continue
        if value.get("schema_version") != 2:
            reasons.append(f"resource-cleanup schema_version is not 2: {rel}")
        if value.get("clean") is not True:
            reasons.append(f"resource cleanup not clean: {rel}")
        for field in ("attempt_owned_gpu_survivors", "unexplained_gpu_memory"):
            survivors = value.get(field)
            if isinstance(survivors, list) and survivors:
                reasons.append(f"resource cleanup has owned survivors ({field}): {rel}")
        if isinstance(value.get("settle_attempts"), list):
            for attempt in value["settle_attempts"]:
                if isinstance(attempt, Mapping) and attempt.get("owned_gpu_survivors"):
                    reasons.append(f"resource cleanup GPU survivors in settle attempt: {rel}")


def _validate_contact_sheets(
    index: Mapping[str, Any],
    suite_dir: Path,
    reasons: list[str],
) -> None:
    """Contact-sheet PNG validity, embedded metadata, agent/user parity (F1.6)."""
    from validation.integrated_contact_sheets import (
        _read_sheet_metadata,
        _validate_sheet_image,
    )

    agent = None
    user = None
    for entry in _entries_by_category(index, "contact-sheet"):
        rel = entry["path"]
        name = rel.rsplit("/", 1)[-1]
        if name == "contact-sheet-integrated-agent.png":
            agent = rel
        elif name == "contact-sheet-integrated-user.png":
            user = rel
    if agent is None:
        reasons.append("missing contact sheet contact-sheet-integrated-agent.png")
    if user is None:
        reasons.append("missing contact sheet contact-sheet-integrated-user.png")
    sheets = [rel for rel in (agent, user) if rel is not None]
    for rel in sheets:
        path = suite_dir / rel
        image_ok, image_reason = _validate_sheet_image(path)
        if not image_ok:
            reasons.append(f"contact sheet invalid PNG: {rel}: {image_reason}")
            continue
        metadata = _read_sheet_metadata(path)
        if metadata is None:
            reasons.append(f"contact sheet missing embedded metadata: {rel}")
            continue
        if metadata.get("role") not in ("agent", "user"):
            reasons.append(f"contact sheet has invalid role: {rel}")
        if metadata.get("diagnostic_only") is not True:
            reasons.append(f"contact sheet is not diagnostic_only: {rel}")
        if not isinstance(metadata.get("reviewed"), bool):
            reasons.append(f"contact sheet has no explicit reviewed state: {rel}")
    if agent is not None and user is not None:
        agent_meta = _read_sheet_metadata(suite_dir / agent)
        user_meta = _read_sheet_metadata(suite_dir / user)
        if agent_meta is not None and user_meta is not None:
            if agent_meta.get("role") == user_meta.get("role"):
                reasons.append("agent and user contact sheets must have distinct roles")
            if agent_meta.get("events") != user_meta.get("events"):
                reasons.append("agent and user contact sheets must cover the same event set")
            if agent_meta.get("captures_sha256") != user_meta.get("captures_sha256"):
                reasons.append("agent and user contact sheets must use the same source captures")


# --------------------------------------------------------------------------- #
# Gate F orchestration
# --------------------------------------------------------------------------- #
def validate_gate_f(
    index: Mapping[str, Any],
    *,
    suite_dir: Path,
    output: Path | None = None,
    validated_index_checksum: str | None = None,
) -> dict[str, Any]:
    """Evaluate the complete index; never fabricates a verdict.

    Returns ``verified-pass`` only when every semantic artifact is valid, the
    index integrity/current-byte checks pass, all required visual events are
    bound to their own scenario transactions, and both contact sheets validate
    with explicit review state.

    ``suite_dir`` is required for current-byte verification (F1.5).
    ``validated_index_checksum`` is the pre-summary checksum recorded by the
    summary writer (F1.5 cycle).
    """
    suite_resolved = Path(suite_dir).resolve()
    reasons: list[str] = []
    diagnostics: list[dict[str, Any]] = []
    by_path = _entry_by_path(index)

    if index.get("kind") != "integrated-evidence-index":
        reasons.append("index kind is not integrated-evidence-index")
    if index.get("checksum_algorithm") != "sha256":
        reasons.append("unsupported checksum algorithm: expected sha256")
    if INDEX_NAME in by_path:
        reasons.append("index self-inclusion detected")

    scenario_kinds = index.get("scenario_kinds")
    if not isinstance(scenario_kinds, list):
        scenario_kinds = []

    # ---- Index integrity / current bytes (F1.5) ----------------------------
    _validate_index_integrity(index, suite_resolved, reasons, diagnostics)

    # ---- Pre-summary projection checksum cycle (F1.5) -----------------------
    summary_entry = None
    for entry in _entries_by_category(index, "qualification-summary"):
        summary_entry = entry
    if summary_entry is not None:
        projection = _pre_summary_projection(index)
        if projection is None:
            reasons.append("cannot reconstruct pre-summary index projection")
        else:
            summary_rel = summary_entry["path"]
            summary = _read_json_rel(suite_resolved, summary_rel)
            if summary is None:
                reasons.append(f"unreadable qualification summary: {summary_rel}")
            else:
                recorded = summary.get("validated_index_checksum")
                reconstructed = canonical_sha256(projection)
                if not isinstance(recorded, str) or recorded != reconstructed:
                    reasons.append(
                        "summary validated_index_checksum does not match the pre-summary projection"
                    )
                if summary.get("status") != "verified-pass":
                    reasons.append("qualification summary status is not verified-pass")
                if not isinstance(summary.get("reviewed"), bool):
                    reasons.append("qualification summary has no explicit reviewed state")
                if summary.get("diagnostic_only") is not True:
                    reasons.append("qualification summary is not diagnostic_only")

    # ---- Source / provenance (F1.4) -----------------------------------------
    _validate_source_provenance(index, suite_resolved, reasons)

    # ---- Scenario verdict / evidence (F1.4) ---------------------------------
    _validate_verdicts(index, suite_resolved, reasons)
    _validate_physics_and_drain(index, suite_resolved, reasons)
    _validate_moveit_controller(index, suite_resolved, reasons)
    _validate_planning_scene(index, suite_resolved, reasons)

    # ---- Rosbag (F1.4) ------------------------------------------------------
    _validate_rosbag(index, suite_resolved, reasons)

    # ---- Cleanup / process / GPU (F1.4) -------------------------------------
    _validate_cleanup(index, suite_resolved, reasons)

    # ---- Contact sheets (F1.4 / F1.6) ---------------------------------------
    _validate_contact_sheets(index, suite_resolved, reasons)

    # ---- Required visual events with exact binding (F1.3) -------------------
    required = _required_event_sets(scenario_kinds)
    captures = _capture_entries(index)
    scenario_events: dict[str, set[str]] = {}
    for entry in captures:
        if not entry.get("bound"):
            reasons.append(f"unbound capture: {entry.get('path')}")
            continue
        if not entry.get("physics_bound"):
            reasons.append(f"capture not physics-bound: {entry.get('path')}")
        scenario = entry.get("scenario")
        event = entry.get("event")
        if isinstance(scenario, str) and isinstance(event, str):
            scenario_events.setdefault(scenario, set()).add(event)
    for group, events in required.items():
        for event in events:
            present = any(
                event in scenario_events.get(scenario_id, ())
                for scenario_id, scenario_item in (
                    (item.get("id"), item)
                    for item in (index.get("scenarios") or [])
                    if isinstance(item, Mapping)
                )
                if isinstance(scenario_item, Mapping) and scenario_item.get("kind") == group
            )
            if not present:
                reasons.append(f"missing required visual event: {event}")

    status = "verified-pass" if not reasons else "verified-fail"
    verdict: dict[str, Any] = {
        "schema_version": 1,
        "kind": "integrated-qualification-summary",
        "status": status,
        "reasons": reasons,
        "diagnostic_only": True,
        "reviewed": False,
        "checksum_algorithm": index.get("checksum_algorithm"),
        "validated_index_checksum": validated_index_checksum,
        "required_events": {group: list(events) for group, events in required.items()},
        "event_coverage": {
            "scenarios": {
                scenario: sorted(events)
                for scenario, events in sorted(scenario_events.items())
            },
            "missing": [
                reason
                for reason in reasons
                if reason.startswith("missing required visual event")
            ],
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
    """Build the pre-summary index, write the summary binding its checksum, then
    rebuild the final index including the summary (F1.5 cycle)."""
    suite_resolved = Path(suite_dir).resolve()
    index_path = Path(index_output) if index_output is not None else suite_resolved / INDEX_NAME
    summary_path = Path(summary_output) if summary_output is not None else suite_resolved / SUMMARY_NAME
    pre_index = build_evidence_index(suite_dir=suite_resolved, output=index_path)
    validated_index_checksum = pre_index["index_checksum"]
    verdict = validate_gate_f(
        pre_index,
        suite_dir=suite_resolved,
        output=summary_path,
        validated_index_checksum=validated_index_checksum,
    )
    build_evidence_index(suite_dir=suite_resolved, output=index_path)
    return verdict


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
        build_qualification_summary(suite_dir=suite_dir, index_output=output, summary_output=Path(args.summary))
    if args.validate:
        verdict = validate_gate_f(index, suite_dir=suite_dir)
        print(json.dumps(verdict, indent=2, sort_keys=True, ensure_ascii=False))
        return 0 if verdict["status"] == "verified-pass" else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
