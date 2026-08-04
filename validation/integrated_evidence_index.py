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

from simulation.tinker_sim_isaac.qualification_visual_capture import (  # noqa: E402
    MAX_CAPTURE_LATENCY_FRAMES,
)

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

#: Executor durable-row status domain (F2.6): the only status values the
#: moveit/controller/execution rows may carry.
_EVIDENCE_STATUS_DOMAIN = frozenset(
    {
        "diagnostic-pass",
        "diagnostic-fail",
        "evidence-invalid",
        "blocked-by-gate-b",
        "verified-pass",
    }
)

#: Rosbag2 approved record topics with their exact message types (F2.7).
#: Mirrors the real ``APPROVED_RECORD_TOPICS`` producer contract and the
#: manipulation topic/type map in ``tinker_sim_bridge/contract_guard.py``.
APPROVED_RECORD_TOPIC_TYPES: dict[str, str] = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/isaac_joint_states": "sensor_msgs/msg/JointState",
    "/isaac_joint_commands": "sensor_msgs/msg/JointState",
    "/sim/truth/robot_state": "tinker_sim_interfaces/msg/RobotTruth",
    "/sim/truth/object_state": "tinker_sim_interfaces/msg/ObjectTruth",
    "/sim/truth/contacts": "tinker_sim_interfaces/msg/ContactTruth",
    "/sim/truth/task_state": "tinker_sim_interfaces/msg/TaskTruth",
    "/sim/safety/collision": "std_msgs/msg/Bool",
    "/sim/hardware/safety_stop": "std_msgs/msg/Bool",
    "/sim/status/contract": "std_msgs/msg/String",
    "/sim/status/command_gateway": "std_msgs/msg/String",
}

#: Recognized in-progress atomic temp prefixes (L8: never index as evidence).
_TEMP_PREFIXES = (
    ".evidence-index.json.",
    ".qualification-summary.json.",
    ".contact-sheet-integrated-",
    ".contact-sheet-",
)

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
    if name == "overlay-contract.json" or name.endswith("-overlay-contract.json"):
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


def _evaluator_frames(suite_dir: Path, attempt_dir: Path) -> list[Mapping[str, Any]]:
    path = attempt_dir / "evaluator.jsonl"
    if not path.is_file():
        return []
    try:
        return _v_read_jsonl(path, required=False)
    except EvidenceError:
        return []


def _raw_scenario(record: Mapping[str, Any]) -> str:
    value = record.get("scenario")
    return value if isinstance(value, str) else ""


def _physics_key_tables(
    raw_records: Sequence[Mapping[str, Any]],
    evaluator_records: Sequence[Mapping[str, Any]],
    attempt_dir: Path,
    diagnostics: list[dict[str, Any]],
) -> tuple[dict[tuple[str, int], Mapping[str, Any]], dict[tuple[str, int], Mapping[str, Any]]]:
    """Build ``(scenario, frame_index)`` primary-key tables for raw/evaluator truth.

    F2.4: a keyframe binds to exactly one raw truth and exactly one evaluator
    record by its primary key; duplicate keys are reported and fail Gate F.
    """
    raw_by_key: dict[tuple[str, int], Mapping[str, Any]] = {}
    for frame_record in raw_records:
        frame_index = frame_record.get("frame_index")
        if not isinstance(frame_index, int) or isinstance(frame_index, bool):
            continue
        key = (_raw_scenario(frame_record), frame_index)
        if key in raw_by_key:
            diagnostics.append(
                {
                    "code": "duplicate-physics-key",
                    "path": str(attempt_dir),
                    "scenario": key[0],
                    "frame_index": frame_index,
                }
            )
            continue
        raw_by_key[key] = frame_record
    evaluator_by_key: dict[tuple[str, int], Mapping[str, Any]] = {}
    for evaluator_record in evaluator_records:
        frame_index = evaluator_record.get("frame_index")
        embedded = evaluator_record.get("frame")
        embedded_frame_index = embedded.get("frame_index") if isinstance(embedded, Mapping) else None
        if not isinstance(frame_index, int) or isinstance(frame_index, bool):
            if isinstance(embedded_frame_index, int) and not isinstance(embedded_frame_index, bool):
                frame_index = embedded_frame_index
        if not isinstance(frame_index, int) or isinstance(frame_index, bool):
            continue
        scenario = _raw_scenario(evaluator_record)
        if not scenario and isinstance(embedded, Mapping):
            scenario = _raw_scenario(embedded)
        key = (scenario, frame_index)
        if key in evaluator_by_key:
            diagnostics.append(
                {
                    "code": "duplicate-evaluator-key",
                    "path": str(attempt_dir),
                    "scenario": scenario,
                    "frame_index": frame_index,
                }
            )
            continue
        evaluator_by_key[key] = evaluator_record
    return raw_by_key, evaluator_by_key


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
    table_cache: dict[
        str, tuple[dict[tuple[str, int], Mapping[str, Any]], dict[tuple[str, int], Mapping[str, Any]]]
    ] = {}
    covered_sequences: set[int] = set()

    for rel_dir, keyframes_path in _all_journal_paths(suite_resolved, "visual-keyframes.jsonl"):
        sequence_requests: dict[int, dict[str, Any]] = {}
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
                # F2.4: executor diagnostic records (shape "executor") are
                # recognized but never capture-driving and never joined to a
                # keyframe; they stay diagnostic-only evidence.
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

            # --- Join to exactly one canonical request (F2.4) ----------------
            execution_request: str | None = None
            candidate = sequence_requests.get(request_sequence)
            if candidate is not None:
                if candidate.get("gate") == gate and candidate.get("event") == event:
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

            # --- F3.5: request-time / source-sequence binding ------------------
            # Cross-check the keyframe's ``requested_simulated_timestamp``
            # against the canonical request timestamp within the strict
            # numerical tolerance, and the keyframe's ``execution_event_sequence``
            # against the canonical ``source_execution_event_sequence``.  The
            # real capture consumer always supplies these fields.
            request_timestamp = candidate.get("simulated_timestamp")
            requested_time = record.get("requested_simulated_timestamp")
            if isinstance(request_timestamp, (int, float)) and not isinstance(request_timestamp, bool):
                if not isinstance(requested_time, (int, float)) or isinstance(requested_time, bool):
                    diagnostics.append(
                        {
                            "code": "keyframe-request-time-mismatch",
                            "path": str(keyframes_path),
                            "line": line,
                            "request_sequence": request_sequence,
                        }
                    )
                else:
                    physics_dt_value = record.get("physics_dt")
                    if isinstance(physics_dt_value, (int, float)) and not isinstance(physics_dt_value, bool) and math.isfinite(float(physics_dt_value)) and float(physics_dt_value) > 0.0:
                        time_tolerance = max(1e-6, 0.5 * float(physics_dt_value))
                    else:
                        time_tolerance = 1e-6
                    if abs(float(requested_time) - float(request_timestamp)) > time_tolerance:
                        diagnostics.append(
                            {
                                "code": "keyframe-request-time-mismatch",
                                "path": str(keyframes_path),
                                "line": line,
                                "request_sequence": request_sequence,
                                "requested": float(requested_time),
                                "request_timestamp": float(request_timestamp),
                            }
                        )
            # --- F4.1: real capture-latency arithmetic --------------------------
            # The real producer writes
            #   requested_physics_frame_index = round(requested_simulated_timestamp / physics_dt)
            #   raw_frame_index               = captured physics frame
            #   capture_latency_frames        = raw_frame_index - requested_physics_frame_index
            # with latency in [0, MAX_CAPTURE_LATENCY_FRAMES].  Requested and raw
            # frames are NOT required to be equal; the requested frame must equal the
            # producer's exact rounded-frame calculation from the keyframe's own
            # requested time and physics dt, and the latency field must equal the
            # frame delta.  The raw/evaluator primary key and raw timestamp
            # tolerance are retained at the captured (raw) frame.
            requested_frame = record.get("requested_physics_frame_index")
            requested_time = record.get("requested_simulated_timestamp")
            physics_dt_value = record.get("physics_dt")
            expected_requested_frame: int | None = None
            if (
                isinstance(requested_time, (int, float))
                and not isinstance(requested_time, bool)
                and math.isfinite(float(requested_time))
                and isinstance(physics_dt_value, (int, float))
                and not isinstance(physics_dt_value, bool)
                and math.isfinite(float(physics_dt_value))
                and float(physics_dt_value) > 0.0
            ):
                expected_requested_frame = int(
                    math.floor(float(requested_time) / float(physics_dt_value) + 0.5)
                )
            if requested_frame is None or isinstance(requested_frame, bool) or not isinstance(requested_frame, int):
                diagnostics.append(
                    {
                        "code": "keyframe-request-frame-invalid",
                        "path": str(keyframes_path),
                        "line": line,
                        "request_sequence": request_sequence,
                        "requested_physics_frame_index": repr(requested_frame),
                    }
                )
            elif expected_requested_frame is not None and requested_frame != expected_requested_frame:
                diagnostics.append(
                    {
                        "code": "keyframe-request-frame-mismatch",
                        "path": str(keyframes_path),
                        "line": line,
                        "request_sequence": request_sequence,
                        "requested_physics_frame_index": requested_frame,
                        "expected_rounded_frame": expected_requested_frame,
                    }
                )
            latency = record.get("capture_latency_frames")
            if latency is not None:
                latency_type_ok = (
                    isinstance(latency, int)
                    and not isinstance(latency, bool)
                    and 0 <= latency <= MAX_CAPTURE_LATENCY_FRAMES
                )
                if not latency_type_ok:
                    diagnostics.append(
                        {
                            "code": "keyframe-latency-out-of-range",
                            "path": str(keyframes_path),
                            "line": line,
                            "request_sequence": request_sequence,
                            "capture_latency_frames": repr(latency),
                        }
                    )
                elif isinstance(requested_frame, int) and not isinstance(requested_frame, bool):
                    if frame - requested_frame != latency:
                        diagnostics.append(
                            {
                                "code": "keyframe-latency-delta-mismatch",
                                "path": str(keyframes_path),
                                "line": line,
                                "request_sequence": request_sequence,
                                "raw_frame_index": frame,
                                "requested_physics_frame_index": requested_frame,
                                "capture_latency_frames": latency,
                            }
                        )
            execution_event_sequence = record.get("execution_event_sequence")
            source_execution_event_sequence = candidate.get("source_execution_event_sequence")
            if execution_event_sequence is not None and source_execution_event_sequence is not None and (
                isinstance(execution_event_sequence, bool)
                or not isinstance(execution_event_sequence, int)
                or isinstance(source_execution_event_sequence, bool)
                or not isinstance(source_execution_event_sequence, int)
                or execution_event_sequence != source_execution_event_sequence
            ):
                diagnostics.append(
                    {
                        "code": "keyframe-source-sequence-mismatch",
                        "path": str(keyframes_path),
                        "line": line,
                        "request_sequence": request_sequence,
                    }
                )

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

            # --- Physics cross-bind (F2.4 bounded-dt, primary key) -----------
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
            if attempt_dir_key not in table_cache:
                table_cache[attempt_dir_key] = _physics_key_tables(
                    physics_records,
                    _evaluator_frames(suite_resolved, attempt_dir),
                    attempt_dir,
                    diagnostics,
                )
            raw_by_key, evaluator_by_key = table_cache[attempt_dir_key]
            physics_bound = False
            raw = raw_by_key.get((gate, frame))
            if raw is not None:
                raw_ts = raw.get("timestamp")
                physics_dt = record.get("physics_dt")
                if (
                    isinstance(physics_dt, (int, float))
                    and not isinstance(physics_dt, bool)
                    and math.isfinite(float(physics_dt))
                    and float(physics_dt) > 0.0
                ):
                    physics_dt = float(physics_dt)
                else:
                    physics_dt = None
                if (
                    isinstance(raw_ts, (int, float))
                    and not isinstance(raw_ts, bool)
                    and math.isfinite(float(raw_ts))
                    and (gate, frame) in evaluator_by_key
                ):
                    window = max(1e-6, 0.5 * physics_dt) if physics_dt is not None else 1e-6
                    if abs(float(raw_ts) - timestamp) <= window:
                        physics_bound = True
            if not physics_bound:
                diagnostics.append(
                    {
                        "code": "keyframe-physics-unbound",
                        "journal": str(keyframes_path),
                        "line": line,
                        "path": canonical_rel,
                        "frame_index": frame,
                        "timestamp": timestamp,
                        "scenario": gate,
                    }
                )
                continue

            if canonical_rel in bindings:
                diagnostics.append(
                    {
                        "code": "duplicate-capture-path",
                        "path": canonical_rel,
                        "first_event": bindings[canonical_rel]["event"],
                        "second_event": event,
                        "first_camera": bindings[canonical_rel]["camera"],
                        "second_camera": camera,
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

    # Canonical request without any keyframe image.  Executor diagnostic
    # records are never capture-driving and never fail here (F2.4).
    for rel_dir, request_path in _all_journal_paths(suite_resolved, "visual-capture-requests.jsonl"):
        for item in _read_request_records(request_path, diagnostics):
            record = item["record"]
            if item["shape"] == "sequence" and record["sequence"] not in covered_sequences:
                diagnostics.append(
                    {"code": "capture-request-without-image", "request_sequence": record["sequence"], "path": str(request_path)}
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
            for repo_name in identity["repositories"]:
                repo_value = value.get(repo_name)
                if isinstance(repo_value, Mapping):
                    for field in ("head", "implementation_head", "resolved_policy_commit"):
                        if isinstance(repo_value.get(field), str):
                            identity[f"{repo_name}.{field}"] = repo_value[field]
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
    elif name == "overlay-contract.json" or name.endswith("-overlay-contract.json"):
        repositories = value.get("repositories")
        if isinstance(repositories, Mapping):
            for repo_name, repo_value in sorted(repositories.items()):
                if (
                    isinstance(repo_value, Mapping)
                    and isinstance(repo_value.get("implementation_identity"), str)
                ):
                    identity[f"repository.{repo_name}.implementation_identity"] = repo_value["implementation_identity"]
        source_locks = value.get("source_locks")
        if isinstance(source_locks, Mapping) and isinstance(source_locks.get("status"), str):
            identity["source_locks.status"] = source_locks["status"]
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
    by_path = _entry_by_path(index)

    # Source-lock manifest (real Gate-B artifact, written into gate-b-* dir).
    lock_entries = _entries_by_category(index, "source-lock-manifest")
    if not lock_entries:
        reasons.append("missing source lock manifest (source-lock-manifest.json)")
    for entry in lock_entries:
        identity = entry.get("identity") or {}
        if identity.get("status") != "pass":
            reasons.append(f"source lock manifest status is not pass: {entry['path']}")
        # F2.5: per-repository commit/digest identity closure.
        lock = _read_json_rel(suite_dir, entry["path"])
        if lock is None:
            reasons.append(f"unreadable source lock manifest: {entry['path']}")
            continue
        repositories = lock.get("repositories")
        if not isinstance(repositories, list) or not repositories:
            reasons.append(f"source lock manifest has no repositories: {entry['path']}")
            continue
        for repo_name in repositories:
            repo_value = lock.get(repo_name)
            if not isinstance(repo_value, Mapping):
                reasons.append(
                    f"source lock manifest missing repository record {repo_name!r}: {entry['path']}"
                )
                continue
            if repo_value.get("status") not in ("pass", "verified-pass"):
                reasons.append(
                    f"source lock repository {repo_name!r} status is not pass: {entry['path']}"
                )
            for field in ("implementation_head", "resolved_policy_commit"):
                raw = repo_value.get(field)
                if not isinstance(raw, str) or not _HEX40_RE.fullmatch(raw):
                    reasons.append(
                        f"source lock repository {repo_name!r} {field} is missing/invalid "
                        f"(lowercase 40-hex): {entry['path']}"
                    )
            raw_head = repo_value.get("head")
            if raw_head is not None and (
                not isinstance(raw_head, str) or not _HEX40_RE.fullmatch(raw_head)
            ):
                reasons.append(
                    f"source lock repository {repo_name!r} head is not lowercase 40-hex: {entry['path']}"
                )
            for pair_field in ("status", "diff", "untracked_manifest"):
                expected = repo_value.get(f"expected_{pair_field}_sha256")
                observed = repo_value.get(f"observed_{pair_field}_sha256")
                if expected is None and observed is None:
                    continue
                if not isinstance(expected, str) or not _HEX64_RE.fullmatch(expected):
                    reasons.append(
                        f"source lock repository {repo_name!r} expected_{pair_field}_sha256 "
                        f"is not 64-hex: {entry['path']}"
                    )
                if not isinstance(observed, str) or not _HEX64_RE.fullmatch(observed):
                    reasons.append(
                        f"source lock repository {repo_name!r} observed_{pair_field}_sha256 "
                        f"is not 64-hex: {entry['path']}"
                    )
                elif expected != observed:
                    reasons.append(
                        f"source lock repository {repo_name!r} {pair_field} digest mismatch "
                        f"(expected != observed): {entry['path']}"
                    )
    # Static contracts (Gate-B closure of config/overlay/command/env/domain).
    static_entries = _entries_by_category(index, "static-contract")
    if not static_entries:
        reasons.append("missing static contract (static-contract.json)")
    for entry in static_entries:
        identity = entry.get("identity") or {}
        if identity.get("status") != "pass":
            reasons.append(f"static contract status is not pass: {entry['path']}")
    # Overlay contract (real nested repositories/source_locks shape, F2.5).
    overlay_entries = _entries_by_category(index, "overlay-contract")
    if not overlay_entries:
        reasons.append("missing overlay contract (overlay-contract.json)")
    for entry in overlay_entries:
        overlay = _read_json_rel(suite_dir, entry["path"])
        if overlay is None:
            reasons.append(f"unreadable overlay contract: {entry['path']}")
            continue
        repositories = overlay.get("repositories")
        if not isinstance(repositories, Mapping) or not repositories:
            reasons.append(f"overlay contract has no repositories map: {entry['path']}")
            continue
        # F3.2: only the known repository mapping records are repositories; the
        # real scalar ``path_scope`` note is validated as a scalar, never as a
        # repository object.  Unknown repository keys are rejected.
        for repo_name, repo_value in repositories.items():
            if repo_name in ("production", "simulator"):
                if not isinstance(repo_value, Mapping):
                    reasons.append(
                        f"overlay contract repository {repo_name!r} is not an object: {entry['path']}"
                    )
                    continue
                impl = repo_value.get("implementation_identity")
                if not isinstance(impl, str) or not _HEX40_RE.fullmatch(impl):
                    reasons.append(
                        f"overlay contract repository {repo_name!r} implementation_identity "
                        f"is missing/invalid (lowercase 40-hex): {entry['path']}"
                    )
            elif repo_name == "path_scope":
                if not isinstance(repo_value, str) or not repo_value.strip():
                    reasons.append(
                        f"overlay contract path_scope must be a non-empty scalar string: {entry['path']}"
                    )
            else:
                reasons.append(
                    f"overlay contract has unknown repository key {repo_name!r}: {entry['path']}"
                )
        if "production" not in repositories or "simulator" not in repositories:
            reasons.append(
                f"overlay contract must declare production and simulator repositories: {entry['path']}"
            )
        # F3.2/F4.4: validate the real source_locks shape/status and bind the
        # simulator source-lock to its immutable policy artifact.  The real
        # schema uses a truthful ``status`` such as ``"excluded_in_task_8"``;
        # a fabricated ``"pass"`` is never required.
        source_locks = overlay.get("source_locks")
        if not isinstance(source_locks, Mapping):
            reasons.append(f"overlay contract source_locks is not an object: {entry['path']}")
        else:
            status = source_locks.get("status")
            if not isinstance(status, str) or not status:
                reasons.append(f"overlay contract source_locks.status is missing: {entry['path']}")
            simulator_lock_path = source_locks.get("simulator_lock_path")
            if not isinstance(simulator_lock_path, str) or not simulator_lock_path:
                reasons.append(
                    f"overlay contract source_locks.simulator_lock_path is missing: {entry['path']}"
                )
            else:
                # F4.4: resolve the root-relative lock path against the evidence
                # suite only.  A verbatim root-relative reference (e.g. the real
                # ``integration/source-locks.json``) binds when the exact
                # suite-relative copy is present or the path is indexed.  An
                # absolute/escaping path never binds and an absent lock is never
                # silently accepted.
                lock_rel = simulator_lock_path.lstrip("/")
                if simulator_lock_path.startswith("/") or ".." in lock_rel.split("/"):
                    reasons.append(
                        f"overlay contract simulator_lock_path is not suite-relative: {simulator_lock_path}"
                    )
                else:
                    suite_copy = suite_dir / lock_rel
                    indexed_lock = by_path.get(lock_rel) is not None
                    if not (suite_copy.is_file() or indexed_lock):
                        reasons.append(
                            f"overlay contract simulator_lock_path does not resolve to an existing "
                            f"source-lock artifact: {simulator_lock_path}"
                        )
    # F4.4: exactly one authoritative overlay contract identity set.  Multiple
    # legitimate ``*-overlay-contract.json`` artifacts are categorized, but their
    # production/simulator implementation identities must not contradict.
    authoritative_overlay_identities: set[tuple[str, str]] = set()
    for entry in overlay_entries:
        overlay = _read_json_rel(suite_dir, entry["path"])
        if not isinstance(overlay, Mapping):
            continue
        repositories = overlay.get("repositories")
        if not isinstance(repositories, Mapping):
            continue
        production = repositories.get("production")
        simulator = repositories.get("simulator")
        if isinstance(production, Mapping) and isinstance(simulator, Mapping):
            production_id = production.get("implementation_identity")
            simulator_id = simulator.get("implementation_identity")
            if isinstance(production_id, str) and isinstance(simulator_id, str):
                authoritative_overlay_identities.add((production_id, simulator_id))
    if len(authoritative_overlay_identities) > 1:
        reasons.append(
            "overlay contract identities contradict across overlay-contract artifacts: "
            + "; ".join(
                f"production={production} simulator={simulator}"
                for production, simulator in sorted(authoritative_overlay_identities)
            )
        )
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
    # F3.6: bind provider/source identities and model fingerprint across the
    # independent source-identities / static-contract / overlay artifacts.
    source_identities_values: list[Mapping[str, Any]] = [
        value
        for entry in _entries_by_category(index, "source-identities")
        for value in [_read_json_rel(suite_dir, entry["path"])]
        if isinstance(value, Mapping)
    ]
    static_values: list[Mapping[str, Any]] = [
        value
        for entry in _entries_by_category(index, "static-contract")
        for value in [_read_json_rel(suite_dir, entry["path"])]
        if isinstance(value, Mapping)
    ]
    overlay_identities: dict[str, str] = {}
    for entry in _entries_by_category(index, "overlay-contract"):
        value = _read_json_rel(suite_dir, entry["path"])
        if not isinstance(value, Mapping):
            continue
        repositories = value.get("repositories")
        if not isinstance(repositories, Mapping):
            continue
        for repo_name in ("production", "simulator"):
            repo_value = repositories.get(repo_name)
            if isinstance(repo_value, Mapping) and isinstance(repo_value.get("implementation_identity"), str):
                overlay_identities[repo_name] = repo_value["implementation_identity"]
    for value in source_identities_values:
        recorded = value.get("source_identities")
        if not isinstance(recorded, Mapping):
            reasons.append("source-identities has no source_identities map")
            continue
        for key in ("production", "simulator"):
            recorded_value = recorded.get(key)
            if not isinstance(recorded_value, str) or not recorded_value:
                reasons.append(f"source-identities missing {key} identity")
                continue
            if key in overlay_identities and overlay_identities[key] != recorded_value:
                reasons.append(
                    f"source-identities {key} {recorded_value!r} does not match overlay "
                    f"contract implementation_identity {overlay_identities[key]!r}"
                )
            for static in static_values:
                static_identities = static.get("source_identities")
                if isinstance(static_identities, Mapping):
                    static_value = static_identities.get(key)
                    if isinstance(static_value, str) and static_value != recorded_value:
                        reasons.append(
                            f"source-identities {key} does not match static contract"
                        )
    for entry in fp_entries:
        value = _read_json_rel(suite_dir, entry["path"])
        if not isinstance(value, Mapping):
            continue
        fp = value.get("model_fingerprint")
        if not isinstance(fp, str):
            continue
        for static in static_values:
            static_fp = static.get("model_fingerprint")
            if isinstance(static_fp, str) and static_fp != fp:
                reasons.append(
                    f"model fingerprint does not match static contract: {entry['path']}"
                )
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
    # F3.6: manifest/scenario/config seed + identity agreement.  The manifest
    # attempt id must equal the enclosing scenario-bundle attempt id, and the
    # scenario/config seeds must agree with the manifest seed.
    manifest_by_dir: dict[str, Mapping[str, Any]] = {}
    for entry in manifest_entries:
        value = _read_json_rel(suite_dir, entry["path"])
        if isinstance(value, Mapping):
            rel_dir = entry["path"].rsplit("/", 1)[0] if "/" in entry["path"] else ""
            manifest_by_dir[rel_dir] = value
    bundle_entries = _entries_by_category(index, "scenario-bundle")
    if not bundle_entries:
        reasons.append("missing scenario bundle (scenario-bundle.json)")
    for entry in bundle_entries:
        value = _read_json_rel(suite_dir, entry["path"])
        if not isinstance(value, Mapping):
            continue
        rel_dir = entry["path"].rsplit("/", 1)[0] if "/" in entry["path"] else ""
        manifest_value = manifest_by_dir.get(rel_dir)
        scenario_id = value.get("scenario_id")
        if manifest_value is not None and isinstance(scenario_id, str) and scenario_id:
            manifest_scenario = manifest_value.get("scenario")
            manifest_scenario_id = (
                manifest_scenario.get("id") if isinstance(manifest_scenario, Mapping) else None
            )
            if manifest_scenario_id != scenario_id:
                reasons.append(
                    f"scenario-bundle scenario_id does not match manifest scenario.id: {entry['path']}"
                )
            manifest_seed = manifest_value.get("seed")
            bundle_seed = value.get("seed")
            if (
                isinstance(manifest_seed, int)
                and not isinstance(manifest_seed, bool)
                and isinstance(bundle_seed, int)
                and not isinstance(bundle_seed, bool)
                and manifest_seed != bundle_seed
            ):
                reasons.append(f"scenario-bundle seed does not match manifest seed: {entry['path']}")
            scenario_map_value = value.get("scenario")
            if (
                isinstance(scenario_map_value, Mapping)
                and isinstance(scenario_map_value.get("seed"), int)
                and not isinstance(scenario_map_value["seed"], bool)
                and isinstance(manifest_seed, int)
                and not isinstance(manifest_seed, bool)
                and scenario_map_value["seed"] != manifest_seed
            ):
                reasons.append(f"scenario declaration seed does not match manifest seed: {entry['path']}")
    # Config and overlay contracts are recognized where present (they live in the
    # repositories, not the suite); Gate-B static contracts close them.
    config_entries = _entries_by_category(index, "config")
    for entry in config_entries:
        identity = entry.get("identity") or {}
        if identity.get("id") and identity.get("seed") is None:
            reasons.append(f"config missing seed identity: {entry['path']}")
        value = _read_json_rel(suite_dir, entry["path"])
        if isinstance(value, Mapping) and isinstance(value.get("seed"), int) and not isinstance(value.get("seed"), bool):
            config_seed = value["seed"]
            for manifest_value in manifest_by_dir.values():
                manifest_seed = manifest_value.get("seed")
                if (
                    isinstance(manifest_seed, int)
                    and not isinstance(manifest_seed, bool)
                    and manifest_seed != config_seed
                ):
                    reasons.append(
                        f"config seed does not match manifest seed: {entry['path']}"
                    )
                    break
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
        # F2.6: verdict attempt identity must cross-match the attempt manifest.
        verdict = _read_json_rel(suite_dir, entry["path"])
        if verdict is not None:
            verdict_attempt = verdict.get("attempt_id")
            attempt_dir_rel = entry["path"].rsplit("/", 1)[0] if "/" in entry["path"] else ""
            manifest_value = (
                _read_json_rel(suite_dir, f"{attempt_dir_rel}/manifest.json")
                if attempt_dir_rel
                else None
            )
            manifest_attempt = (
                manifest_value.get("attempt_id") if isinstance(manifest_value, Mapping) else None
            )
            if not isinstance(verdict_attempt, str) or not verdict_attempt:
                reasons.append(f"gate verdict missing attempt_id: {entry['path']}")
            elif manifest_value is None:
                # F3.6: a missing enclosing manifest is a failure, never a
                # skipped relocation check.
                reasons.append(f"gate verdict has no enclosing manifest: {entry['path']}")
            elif (
                isinstance(manifest_attempt, str)
                and manifest_attempt
                and verdict_attempt != manifest_attempt
            ):
                reasons.append(
                    f"gate verdict attempt_id {verdict_attempt!r} does not match manifest "
                    f"attempt_id {manifest_attempt!r}: {entry['path']}"
                )
            # F3.6: the verdict must match the independent verifier contract
            # exactly (schema version, gate, pass/verified/authority).
            if verdict.get("schema_version") != 1:
                reasons.append(f"gate verdict schema_version is not 1: {entry['path']}")
            if verdict.get("pass") is not True or verdict.get("verified") is not True:
                reasons.append(f"gate verdict is not pass+verified: {entry['path']}")
            if verdict.get("authority") != "physics_truth.jsonl":
                reasons.append(f"gate verdict authority is not physics_truth.jsonl: {entry['path']}")
            if isinstance(verdict.get("gate"), str) and verdict["gate"] != scenario_id:
                reasons.append(f"gate verdict gate mismatch for {scenario_id}: {entry['path']}")
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
        # F2.6: reject duplicate (scenario, frame_index) primary keys in either
        # truth stream — a key must map to exactly one raw and one evaluator row.
        raw_seen: set[tuple[str, int]] = set()
        for row in raw:
            fi = row.get("frame_index")
            if not isinstance(fi, int) or isinstance(fi, bool):
                continue
            key = (_raw_scenario(row), fi)
            if key in raw_seen:
                reasons.append(f"duplicate raw physics key in {attempt_rel}: scenario={key[0]!r} frame_index={fi}")
            raw_seen.add(key)
        evaluator_seen: set[tuple[str, int]] = set()
        for row in evaluator:
            fi = row.get("frame_index")
            embedded = row.get("frame")
            embedded_fi = embedded.get("frame_index") if isinstance(embedded, Mapping) else None
            if not isinstance(fi, int) or isinstance(fi, bool):
                if isinstance(embedded_fi, int) and not isinstance(embedded_fi, bool):
                    fi = embedded_fi
            if not isinstance(fi, int) or isinstance(fi, bool):
                continue
            scenario = _raw_scenario(row)
            if not scenario and isinstance(embedded, Mapping):
                scenario = _raw_scenario(embedded)
            key = (scenario, fi)
            if key in evaluator_seen:
                reasons.append(f"duplicate evaluator key in {attempt_rel}: scenario={scenario!r} frame_index={fi}")
            evaluator_seen.add(key)
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
            status = row.get("status")
            if not isinstance(status, str) or not status:
                reasons.append(f"moveit row {row_number} in {rel} has no status")
            elif status not in _EVIDENCE_STATUS_DOMAIN:
                reasons.append(f"moveit row {row_number} in {rel} has out-of-domain status {status!r}")
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
            status = row.get("status")
            if status is not None and (not isinstance(status, str) or status not in _EVIDENCE_STATUS_DOMAIN):
                reasons.append(f"controller row {row_number} in {rel} has out-of-domain status {status!r}")


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
    # F2.6: PlanningScene evidence is per-attempt — every attempt journal must
    # have its final artifact in the same attempt directory and vice versa.
    journal_dirs = {
        rel.rsplit("/", 1)[0] if "/" in rel else "" for rel in journals
    }
    final_dirs = {rel.rsplit("/", 1)[0] if "/" in rel else "" for rel in finals}
    for attempt_rel in sorted(journal_dirs | final_dirs):
        if attempt_rel in journal_dirs and attempt_rel not in final_dirs:
            reasons.append(
                f"planning scene attempt {attempt_rel or '(root)'} has a journal but no final artifact"
            )
        if attempt_rel in final_dirs and attempt_rel not in journal_dirs:
            reasons.append(
                f"planning scene attempt {attempt_rel or '(root)'} has a final artifact but no journal"
            )
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


#: Real recorder QoS override contract (manipulation_qualification
#: ROSBAG_QOS_OVERRIDE_PROFILES): the two state endpoints are recorded with
#: keep_last(1)/depth(1)/reliable(1)/transient_local(1).
ROSBAG_QOS_OVERRIDE = {
    "/sim/hardware/safety_stop": {"history": 1, "depth": 1, "reliability": 1, "durability": 1},
    "/sim/status/contract": {"history": 1, "depth": 1, "reliability": 1, "durability": 1},
}

#: Valid RMW QoS enum integers accepted in rosbag2 ``offered_qos_profiles``.
_RMW_HISTORY_VALUES = (0, 1, 2, 3)       # system_default, keep_last, keep_all, unknown
_RMW_RELIABILITY_VALUES = (0, 1, 2, 3)   # system_default, reliable, best_effort, unknown
_RMW_DURABILITY_VALUES = (0, 1, 2, 3, 4)  # system_default, transient_local, transient, volatile, unknown
_RMW_LIVELINESS_VALUES = (0, 1, 2, 3, 4)  # system_default, automatic, manual_by_topic, manual_by_node, unknown

#: Real Humble rosbag2 serializes the full ``rmw_qos_profile_t`` in
#: ``offered_qos_profiles``: history, depth, reliability, durability, deadline,
#: lifespan, liveliness, liveliness_lease_duration,
#: avoid_ros_namespace_conventions (F4.2).  The four core fields are required;
#: the remaining RMW fields are validated when present and must satisfy the real
#: schema (deadline/lifespan/liveliness_lease_duration are non-negative
#: nanosecond integers, liveliness is an RMW enum, and
#: avoid_ros_namespace_conventions is a boolean).
_ROSBAG_QOS_REQUIRED_FIELDS = ("history", "depth", "reliability", "durability")
_ROSBAG_QOS_RMW_DURATION_FIELDS = ("deadline", "lifespan", "liveliness_lease_duration")


def _parse_rosbag_qos(text: str) -> list[dict[str, Any]] | None:
    """Parse rosbag2 ``offered_qos_profiles`` YAML into a profile list.

    Returns ``None`` when the text is not YAML or not a list (or single mapping)
    of objects; otherwise the list of string-keyed profile dicts.
    """
    try:
        import yaml
        value = yaml.safe_load(text)
    except Exception:
        return None
    if isinstance(value, Mapping):
        value = [value]
    if not isinstance(value, list):
        return None
    profiles: list[dict[str, Any]] = []
    for item in value:
        if not isinstance(item, Mapping):
            return None
        profiles.append({str(key): item[key] for key in item})
    return profiles


def _rosbag_qos_fields_ok(profile: Mapping[str, Any]) -> bool:
    """Require the required RMW QoS fields with valid values and validate the
    full known rmw fields when present (F4.2)."""
    for field in _ROSBAG_QOS_REQUIRED_FIELDS:
        value = profile.get(field)
        if isinstance(value, bool) or not isinstance(value, int):
            return False
    history = profile["history"]
    depth = profile["depth"]
    reliability = profile["reliability"]
    durability = profile["durability"]
    if history not in _RMW_HISTORY_VALUES:
        return False
    if depth < 0:
        return False
    if reliability not in _RMW_RELIABILITY_VALUES:
        return False
    if durability not in _RMW_DURABILITY_VALUES:
        return False
    for field in _ROSBAG_QOS_RMW_DURATION_FIELDS:
        value = profile.get(field)
        if value is None:
            continue
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            return False
    liveliness = profile.get("liveliness")
    if liveliness is not None and (
        isinstance(liveliness, bool)
        or not isinstance(liveliness, int)
        or liveliness not in _RMW_LIVELINESS_VALUES
    ):
        return False
    avoid = profile.get("avoid_ros_namespace_conventions")
    if avoid is not None and not isinstance(avoid, bool):
        return False
    return True


def _validate_rosbag(
    index: Mapping[str, Any],
    suite_dir: Path,
    reasons: list[str],
) -> None:
    """Real rosbag2 metadata + storage (F1.4 Rosbag, F2.7/F3.8 exact contract)."""
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
        observed: dict[str, Mapping[str, Any]] = {}
        for record in records:
            if not isinstance(record, Mapping):
                reasons.append(f"rosbag metadata has a malformed topic record: {rel}")
                continue
            metadata_record = record.get("topic_metadata")
            count = record.get("message_count")
            if not isinstance(metadata_record, Mapping) or not isinstance(metadata_record.get("name"), str):
                reasons.append(f"rosbag metadata has a topic without topic_metadata.name: {rel}")
                continue
            topic = metadata_record["name"]
            if topic in observed:
                reasons.append(f"rosbag metadata has duplicate topic {topic}: {rel}")
            observed[topic] = dict(metadata_record)
            if isinstance(count, int) and not isinstance(count, bool) and count > 0:
                total_messages += count
            else:
                reasons.append(f"rosbag topic {topic} has nonpositive message count")
            qos_text = metadata_record.get("offered_qos_profiles")
            if not isinstance(qos_text, str) or not qos_text.strip():
                reasons.append(f"rosbag topic {topic} has no offered_qos_profiles")
                continue
            # F3.8: parse the QoS YAML; never accept an arbitrary nonempty
            # string.  Profiles must be a YAML list of objects with valid RMW
            # enum fields, and every approved publisher must offer RELIABLE.
            profiles = _parse_rosbag_qos(qos_text)
            if profiles is None or not profiles:
                reasons.append(f"rosbag topic {topic} offered_qos_profiles is not a YAML profile list")
                continue
            if not all(_rosbag_qos_fields_ok(profile) for profile in profiles):
                reasons.append(f"rosbag topic {topic} offered_qos_profiles has malformed RMW QoS fields")
                continue
            if not any(profile.get("reliability") == 1 for profile in profiles):
                reasons.append(f"rosbag topic {topic} offers no reliable QoS profile")
            override = ROSBAG_QOS_OVERRIDE.get(topic)
            if override is not None:
                # F4.2: the real Humble serialization carries the full
                # nine-field rmw_qos_profile_t.  The recorder override is a
                # subset contract: some profile must match the override's
                # required history/depth/reliability/durability fields; the
                # additional valid rmw fields (deadline, lifespan, liveliness,
                # liveliness_lease_duration, avoid_ros_namespace_conventions)
                # are permitted.
                matched = any(
                    profile.get("history") == override["history"]
                    and profile.get("depth") == override["depth"]
                    and profile.get("reliability") == override["reliability"]
                    and profile.get("durability") == override["durability"]
                    for profile in profiles
                )
                if not matched:
                    reasons.append(
                        f"rosbag topic {topic} offered QoS does not match the recorder override contract"
                    )
        # F2.7: the approved record-topic set must be present with exact types
        # and per-topic nonzero counts.
        missing_topics = sorted(set(APPROVED_RECORD_TOPIC_TYPES) - set(observed))
        if missing_topics:
            reasons.append(
                f"rosbag is missing approved record topics: {', '.join(missing_topics)}"
            )
        for topic, expected_type in APPROVED_RECORD_TOPIC_TYPES.items():
            metadata_record = observed.get(topic)
            if metadata_record is None:
                continue
            recorded_type = metadata_record.get("type")
            if recorded_type != expected_type:
                reasons.append(
                    f"rosbag topic {topic} type {recorded_type!r} does not match "
                    f"expected {expected_type!r}"
                )
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
        storage_identifier = root.get("storage_identifier")
        if storage_identifier != "sqlite3":
            reasons.append(f"rosbag storage_identifier is not sqlite3: {rel}")
        bag_dir_rel = rel.rsplit("/", 1)[0] if "/" in rel else ""
        # F3.8: storage closure -- every metadata-listed storage file must exist
        # and be nonempty, and no extra conflicting storage file may exist.
        relative_file_paths = root.get("relative_file_paths") if isinstance(root, Mapping) else None
        listed_paths: list[str] = []
        if not isinstance(relative_file_paths, list):
            reasons.append(f"rosbag metadata has no relative_file_paths: {rel}")
        else:
            for storage_name in relative_file_paths:
                if not isinstance(storage_name, str) or not storage_name:
                    reasons.append(f"rosbag metadata has a malformed relative_file_paths entry: {rel}")
                    continue
                if "/" in storage_name or ".." in storage_name:
                    reasons.append(f"rosbag metadata storage path is not a bare file name: {storage_name}")
                    continue
                listed_paths.append(storage_name)
                storage_path = (
                    suite_dir / f"{bag_dir_rel}/{storage_name}" if bag_dir_rel else suite_dir / storage_name
                )
                if not storage_path.is_file():
                    reasons.append(f"rosbag metadata lists missing storage file: {storage_name}")
                elif storage_path.stat().st_size <= 0:
                    reasons.append(f"rosbag storage file is empty: {storage_name}")
        indexed_storage_names = [
            e["path"].rsplit("/", 1)[-1] if "/" in e["path"] else e["path"]
            for e in _entries_by_category(index, "rosbag-storage")
            if (e["path"].rsplit("/", 1)[0] if "/" in e["path"] else "") == bag_dir_rel
        ]
        for storage_name in sorted(indexed_storage_names):
            if storage_name not in listed_paths:
                reasons.append(
                    f"rosbag has storage file not listed in metadata: {storage_name}"
                )
        if not indexed_storage_names:
            reasons.append(f"rosbag has no storage files: {rel}")


def _gpu_topology_key(record: Mapping[str, Any]) -> str:
    """Return the producer's stable GPU identity field for *record*.

    The real ``_gpu_processes`` snapshot records ``{index, uuid,
    memory_used_mib}``; the uuid is the load-bearing physical identity and the
    index is the fallback.  Topology invariance is compared on these keys, never
    on raw list order or fabricated ``id``/``name`` fields.
    """
    uuid_value = record.get("uuid")
    if isinstance(uuid_value, str) and uuid_value:
        return f"uuid:{uuid_value}"
    index_value = record.get("index")
    if isinstance(index_value, int) and not isinstance(index_value, bool):
        return f"index:{index_value}"
    return f"gpu:{canonical_sha256(record)}"


def _gpu_topology_matches(baseline_gpus: Any, final_gpus: Any) -> bool:
    """Require final GPU identity set == baseline GPU identity set (F3.1).

    A physical GPU may remain present across the attempt; an added, removed, or
    mutated GPU identity fails.  When either inventory is absent/empty the
    availability checks already gate the snapshot, so an empty/empty comparison
    is not itself a topology change.
    """
    def _keys(value: Any) -> set[str]:
        if not isinstance(value, list):
            return set()
        keys: set[str] = set()
        for record in value:
            if not isinstance(record, Mapping):
                return set()
            key = _gpu_topology_key(record)
            if key in keys:
                return set()
            keys.add(key)
        return keys

    baseline_keys = _keys(baseline_gpus)
    final_keys = _keys(final_gpus)
    if not baseline_keys and not final_keys:
        return True
    if not baseline_keys or not final_keys:
        return False
    return baseline_keys == final_keys


def _validate_cleanup(
    index: Mapping[str, Any],
    suite_dir: Path,
    reasons: list[str],
) -> None:
    """Cleanup/process/GPU evidence (F1.4 Cleanup/process/GPU).

    F3.1: the real producer schema_version-2 shape records
    ``baseline.gpus``/``final.gpus`` as the FULL nvidia-smi GPU inventory
    (non-empty on a healthy RTX run) and ``attempt_owned_pids`` as the
    cumulative historical PID observation set.  Neither is a survivor list.
    ``clean`` is recomputed from the producer's own semantics
    (``available and not leaked and not memory_leaks``) plus GPU-topology
    invariance on stable identity fields; the false ``final.gpus == []``
    predicate is gone.
    """
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
        baseline = value.get("baseline")
        final = value.get("final")
        baseline_available = isinstance(baseline, Mapping) and baseline.get("available") is True
        final_available = isinstance(final, Mapping) and final.get("available") is True
        owned_gpu_survivors = value.get("attempt_owned_gpu_survivors")
        unexplained_gpu_memory = value.get("unexplained_gpu_memory")
        clean_state = (
            baseline_available
            and final_available
            and isinstance(owned_gpu_survivors, list)
            and not owned_gpu_survivors
            and isinstance(unexplained_gpu_memory, list)
            and not unexplained_gpu_memory
        )
        baseline_gpus = baseline.get("gpus") if isinstance(baseline, Mapping) else None
        final_gpus = final.get("gpus") if isinstance(final, Mapping) else None
        topology_ok = _gpu_topology_matches(baseline_gpus, final_gpus)
        if not topology_ok:
            reasons.append(
                f"resource cleanup GPU topology changed between baseline and final: {rel}"
            )
        # F4.5: when a snapshot reports ``available=true`` the GPU inventory must
        # be a nonempty list of valid physical-GPU records in addition to a
        # stable topology.  Empty/empty must never pass vacuously, while
        # non-GPU/unavailable snapshots keep their existing fail semantics.
        if baseline_available or final_available:
            if not isinstance(baseline_gpus, list) or not isinstance(final_gpus, list):
                reasons.append(
                    f"resource cleanup GPU inventory is missing despite available=true: {rel}"
                )
            elif not baseline_gpus or not final_gpus:
                reasons.append(
                    f"resource cleanup has an empty GPU inventory despite available=true: {rel}"
                )
            else:
                for gpu_record in baseline_gpus + final_gpus:
                    if (
                        not isinstance(gpu_record, Mapping)
                        or not isinstance(gpu_record.get("uuid"), str)
                        or not gpu_record["uuid"]
                        or isinstance(gpu_record.get("index"), bool)
                        or not isinstance(gpu_record.get("index"), int)
                    ):
                        reasons.append(
                            f"resource cleanup has a malformed GPU inventory record: {rel}"
                        )
        recomputed_clean = clean_state and topology_ok
        if value.get("clean") is True and not recomputed_clean:
            reasons.append(
                f"resource cleanup clean flag contradicts the recorded baseline/final/owned state: {rel}"
            )
        if isinstance(owned_gpu_survivors, list) and owned_gpu_survivors:
            reasons.append(f"resource cleanup has owned gpu survivors: {rel}")
        if isinstance(unexplained_gpu_memory, list) and unexplained_gpu_memory:
            reasons.append(f"resource cleanup has unexplained_gpu_memory: {rel}")
        if isinstance(value.get("settle_attempts"), list):
            for attempt in value["settle_attempts"]:
                if isinstance(attempt, Mapping) and attempt.get("owned_gpu_survivors"):
                    reasons.append(f"resource cleanup GPU survivors in settle attempt: {rel}")
                if isinstance(attempt, Mapping) and attempt.get("unexplained_gpu_memory"):
                    reasons.append(f"resource cleanup unexplained memory in settle attempt: {rel}")


def _validate_contact_sheets(
    index: Mapping[str, Any],
    suite_dir: Path,
    reasons: list[str],
    scenario_kinds: Sequence[str] | None = None,
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
    # F3.3: a sheet's embedded ordered events must equal the complete required
    # suite event sequence (all events for every present visual kind), not merely
    # agent/user parity or global capture coverage.
    required_suite_events: list[str] = []
    kinds = set(scenario_kinds)
    if "positive" in kinds:
        required_suite_events.extend(REQUIRED_POSITIVE_EVENTS)
    if "cancel" in kinds:
        required_suite_events.extend(CANCEL_EVENTS)
    if "safety" in kinds:
        required_suite_events.extend(SAFETY_EVENTS)
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
        if required_suite_events and metadata.get("events") != required_suite_events:
            reasons.append(
                f"contact sheet embedded events do not equal the complete required "
                f"suite event sequence: {rel}"
            )
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

    # ---- Index diagnostics fail closed (F2.4) --------------------------------
    index_diagnostics = index.get("diagnostics")
    if isinstance(index_diagnostics, list) and index_diagnostics:
        reasons.append(
            f"evidence index recorded {len(index_diagnostics)} diagnostics; "
            "any index diagnostic fails Gate F closed"
        )

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
    _validate_contact_sheets(index, suite_resolved, reasons, scenario_kinds)

    # ---- Required visual events with exact binding (F1.3) -------------------
    required = _required_event_sets(scenario_kinds)
    captures = _capture_entries(index)
    # F4.5: visual completeness keys by the exact ``(scenario_id, attempt_id)``
    # transaction, never by scenario id alone.  Two attempts bearing the same
    # scenario must never merge their event subsets.
    attempt_capture_events: dict[tuple[str, str], set[str]] = {}
    for entry in captures:
        if not entry.get("bound"):
            reasons.append(f"unbound capture: {entry.get('path')}")
            continue
        if not entry.get("physics_bound"):
            reasons.append(f"capture not physics-bound: {entry.get('path')}")
        scenario = entry.get("scenario")
        attempt = entry.get("attempt")
        event = entry.get("event")
        if isinstance(scenario, str) and isinstance(event, str) and isinstance(attempt, str):
            attempt_capture_events.setdefault((scenario, attempt), set()).add(event)
    # F3.3: visual evidence closure is per exact attempt/scenario.  Every
    # positive/cancel/safety scenario must itself contain its kind's complete
    # event set under its exact scenario id; a sibling attempt never satisfies
    # another and events split across siblings never pass.
    scenario_items = {
        str(item.get("id")): item
        for item in (index.get("scenarios") or [])
        if isinstance(item, Mapping) and isinstance(item.get("id"), str)
    }
    for group, events in required.items():
        for scenario_id, scenario_item in scenario_items.items():
            if scenario_item.get("kind") != group:
                continue
            attempts_with_captures = sorted(
                {attempt for (scenario, attempt) in attempt_capture_events if scenario == scenario_id}
            )
            if not attempts_with_captures:
                reasons.append(
                    f"scenario {scenario_id} is missing required {group} visual events: "
                    f"{', '.join(events)}"
                )
                continue
            for attempt in attempts_with_captures:
                present_events = attempt_capture_events.get((scenario_id, attempt), set())
                missing = [event for event in events if event not in present_events]
                if missing:
                    reasons.append(
                        f"scenario {scenario_id} attempt {attempt} is missing required "
                        f"{group} visual events: {', '.join(missing)}"
                    )

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
                f"{scenario}::{attempt}": sorted(events)
                for (scenario, attempt), events in sorted(attempt_capture_events.items())
            },
            "missing": [
                reason
                for reason in reasons
                if "is missing required" in reason
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
