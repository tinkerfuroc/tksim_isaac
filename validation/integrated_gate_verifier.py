#!/usr/bin/env python3
"""Independent integrated raw-physics gate verifier (ROS-free, Python 3.12).

Task 7 of the integrated OMPL manipulation qualification.  This module is a
standalone, offline verifier that recomputes every physical postcondition from
``physics_truth.jsonl`` raw physics truth and the planning-scene journal.  The
integrated executor's ``integrated-execution.jsonl``/``.json``,
``controller-results.jsonl``, ``moveit-plans.jsonl``, and ``goals/*.json`` rows
are treated as diagnostic endpoint/claim evidence only — never as physical
proof.  It writes one atomically replaced ``gate-verdict.json``.

The module is deliberately ROS-free: it imports no ``rclpy``, no generated ROS
messages, and no geometry packages.  It reuses the committed pure-Python
helpers from ``manipulation_gate_verifier`` (raw truth parsing, window
selection, joint metrics, quaternion math) and the committed executor tables
for the exact 17-scenario contract.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

# --------------------------------------------------------------------------- #
# Compatibility imports (works when validation/ is on sys.path or the repo
# root is the import root).  Everything imported here is ROS-free pure Python.
# --------------------------------------------------------------------------- #
try:
    from manipulation_gate_verifier import (  # noqa: F401
        EvidenceError,
        _arm_joint_data,
        _as_bool,
        _bilateral,
        _contact_force,
        _contacts,
        _distance,
        _finite,
        _first,
        _gripper_efforts,
        _gripper_position,
        _joint_data,
        _joint_metrics,
        _max_delta,
        _norm_body,
        _object,
        _object_pose,
        _objects,
        _parse_truth,
        _pose_position,
        _pose_quaternion,
        _q_inverse,
        _q_mul,
        _q_rotate,
        _raw_gate_window,
        _read_json,
        _read_jsonl,
        _robot,
        _safety,
        _target_vector,
        _vector,
        _walk,
    )
except ModuleNotFoundError:
    from validation.manipulation_gate_verifier import (  # noqa: F401
        EvidenceError,
        _arm_joint_data,
        _as_bool,
        _bilateral,
        _contact_force,
        _contacts,
        _distance,
        _finite,
        _first,
        _gripper_efforts,
        _gripper_position,
        _joint_data,
        _joint_metrics,
        _max_delta,
        _norm_body,
        _object,
        _object_pose,
        _objects,
        _parse_truth,
        _pose_position,
        _pose_quaternion,
        _q_inverse,
        _q_mul,
        _q_rotate,
        _raw_gate_window,
        _read_json,
        _read_jsonl,
        _robot,
        _safety,
        _target_vector,
        _vector,
        _walk,
    )

try:
    from integrated_gate_executor import (  # noqa: F401
        D_FORBIDDEN_EVENTS,
        EXECUTE_STATUS_CANCELED,
        EXECUTE_STATUS_SUCCEEDED,
        GATE_C_FORBIDDEN_EVENTS,
        GATE_C_REQUIRED_EVENT_ORDER,
        GRIPPER_CLOSE_FIRST_EVENT_ORDER,
        PICK_PLACE_RESULT_SUCCESS,
        POSE_APPROACH_QUATERNION_XYZW,
        POSE_APPROACH_Z_OFFSET,
        Q_OUTBOUND,
        REQUIRED_ACTIONS,
        REQUIRED_SERVICES,
        STAGE_C_SCENARIOS,
        STAGE_D_EXPECTED_PHYSICAL,
        STAGE_D_EXPECTED_POLARITY,
        STAGE_D_KIND,
        STAGE_D_REQUIRED_EVENT_ORDER,
        STAGE_D_SCENARIOS,
        STAGE_E_EXPECTED_NEGATIVE,
        STAGE_E_EXPECTED_PHYSICAL,
        STAGE_E_EXPECTED_POLARITY,
        STAGE_E_FORBIDDEN_EVENTS,
        STAGE_E_KIND,
        STAGE_E_REQUIRED_EVENT_ORDER,
        STAGE_E_SCENARIOS,
        TARGET_OBJECT_ID,
        _REQUIRED_ENDPOINT_SOURCES,
    )
except ModuleNotFoundError:
    from validation.integrated_gate_executor import (  # noqa: F401
        D_FORBIDDEN_EVENTS,
        EXECUTE_STATUS_CANCELED,
        EXECUTE_STATUS_SUCCEEDED,
        GATE_C_FORBIDDEN_EVENTS,
        GATE_C_REQUIRED_EVENT_ORDER,
        GRIPPER_CLOSE_FIRST_EVENT_ORDER,
        PICK_PLACE_RESULT_SUCCESS,
        POSE_APPROACH_QUATERNION_XYZW,
        POSE_APPROACH_Z_OFFSET,
        Q_OUTBOUND,
        REQUIRED_ACTIONS,
        REQUIRED_SERVICES,
        STAGE_C_SCENARIOS,
        STAGE_D_EXPECTED_PHYSICAL,
        STAGE_D_EXPECTED_POLARITY,
        STAGE_D_KIND,
        STAGE_D_REQUIRED_EVENT_ORDER,
        STAGE_D_SCENARIOS,
        STAGE_E_EXPECTED_NEGATIVE,
        STAGE_E_EXPECTED_PHYSICAL,
        STAGE_E_EXPECTED_POLARITY,
        STAGE_E_FORBIDDEN_EVENTS,
        STAGE_E_KIND,
        STAGE_E_REQUIRED_EVENT_ORDER,
        STAGE_E_SCENARIOS,
        TARGET_OBJECT_ID,
        _REQUIRED_ENDPOINT_SOURCES,
    )

try:
    from planning_scene_journal import (  # noqa: F401
        CANONICAL_LINK_TCP,
        CANONICAL_TARGET_HANDOFF,
        CANONICAL_TOUCH_LINKS,
        POSITIVE_ORDER,
    )
except ModuleNotFoundError:
    from validation.planning_scene_journal import (  # noqa: F401
        CANONICAL_LINK_TCP,
        CANONICAL_TARGET_HANDOFF,
        CANONICAL_TOUCH_LINKS,
        POSITIVE_ORDER,
    )

SCHEMA_VERSION = 1
VERDICT_FILENAME = "gate-verdict.json"
PHYSICS_HZ_DEFAULT = 120.0

#: Terminal success strings written by the executor / action clients.
_SUCCESS_TERMINALS = frozenset(
    {"succeeded", "success", "successful", "complete", "completed", "done"}
)
#: Definite non-success terminal strings.
_NON_SUCCESS_TERMINALS = frozenset(
    {"canceled", "cancelled", "aborted", "failed", "rejected", "timeout", "timed_out"}
)

#: Endpoint fields the executor/controller/moveit artifacts actually persist.
_ENDPOINT_KEYS = frozenset(
    {
        "endpoint",
        "action_endpoint",
        "action_server",
        "action_name",
        "action",
        "server",
        "controller_endpoint",
    }
)
#: Paired provider fields persisted by the committed artifact schemas.
_SOURCE_KEYS = frozenset({"source_node", "provider_node", "source", "source_provider"})

#: Allowed endpoint set is exactly the executor's required action/service keys.
ALLOWED_ENDPOINTS = frozenset(REQUIRED_ACTIONS) | frozenset(REQUIRED_SERVICES)

#: Table 1 per-scenario terminal anchors (gate_end event names).
_D_GATE_END_EVENT = {
    "execute-joint": "execution-terminal",
    "execute-pose": "execution-terminal",
    "retreat": "retreat-terminal",
    "gripper": None,  # final (second) gripper-*-terminal in observed order
    "cancel": "quiescent",
    "safety": "quiescent",
}
_E_GATE_END_EVENT = {
    "positive": "released-settled",
    "blocked-approach": "pick-terminal",
    "unreachable-grasp": "pick-terminal",
    "malformed-back": "teardown",
    "cancel-approach": "quiescent",
    "cancel-transport": "quiescent",
    "safety-transport": "quiescent",
    "occupied-place": "quiescent",
}

#: Table 2 scenario-owned observation subwindows (event keys, both inclusive).
_OBSERVATION_SUBWINDOW = {
    "cancel": ("cancel-requested", "quiescent"),
    "safety": ("effective-stop", "quiescent"),
    "positive": ("scene-detach", "released-settled"),
    "cancel-approach": ("cancel-requested", "quiescent"),
    "cancel-transport": ("cancel-requested", "quiescent"),
    "safety-transport": ("operator-clear", "quiescent"),
    "occupied-place": ("place-goal-accepted", "quiescent"),
}

#: Task target is the handoff object (planning_scene_journal.CANONICAL_TARGET_HANDOFF).
TASK_TARGET_ID = CANONICAL_TARGET_HANDOFF

_ARM_CONTACT_BODIES = tuple(f"link{index}" for index in range(1, 8))
_GRASP_CONTACT_BODIES = ("left_finger", "right_finger", "link_tcp")
_ROBOT_CONTACT_BODIES = frozenset(_ARM_CONTACT_BODIES) | frozenset(_GRASP_CONTACT_BODIES)

#: Raw backend-emitted object id for the task target.  The backend
#: ``_expected_scenario_objects`` uses ``record["id"]`` verbatim and every E
#: scenario declares the bare ``qualification_cube`` id (``sim_fixture/`` is the
#: planning-scene declaration domain, not the raw physics domain).  Only the
#: bare id is accepted as raw measured target identity (F2.5).
_OBJECT_ID_CANDIDATES = ("qualification_cube",)

#: Committed semantic provenance values that are not endpoint provider metadata
#: (F1.3/F2.2).  ``env_cloud_evidence.source == "observed-environment-cloud"``
#: is the executor's real environment-cloud provenance; it must never be
#: treated as a forbidden provider token, but any other source value carrying a
#: forbidden token is rejected.
_SEMANTIC_SOURCE_PROVENANCE = frozenset({"observed-environment-cloud"})

#: Provider/goal selection fields whose values must never carry a forbidden
#: token (F1.6/F2.2).  ``goal_kind`` is persisted by the committed executor in
#: moveit-plans rows and summaries.  Deliberately narrow: semantic free-text
#: fields such as environment-cloud provenance stay out of the scan.
_PROVIDER_KEYS = frozenset(
    {
        "pipeline_id",
        "provider",
        "execution_profile",
        "planner_id",
        "planner_name",
        "goal_kind",
    }
)


def _as_int(value: Any, name: str) -> int:
    """Validate a non-boolean integer scalar (F1.5)."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise EvidenceError(f"{name} must be an integer, not {type(value).__name__}")
    return value


def _as_index(value: Any, name: str) -> int:
    """Validate a non-boolean non-negative integer index (F1.5)."""
    result = _as_int(value, name)
    if result < 0:
        raise EvidenceError(f"{name} must be non-negative")
    return result


def _as_timestamp(value: Any, name: str) -> float:
    """Validate a finite numeric timestamp scalar (F1.5)."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvidenceError(f"{name} must be numeric")
    timestamp = float(value)
    if not math.isfinite(timestamp):
        raise EvidenceError(f"{name} must be finite")
    return timestamp


def _object_pose_target(frame: Mapping[str, Any]) -> tuple[list[float], list[float], list[float]] | None:
    """Return the task-target object pose (raw backend id ``qualification_cube``).

    Mirrors ``manipulation_gate_verifier._object_pose``.  ``sim_fixture/...``
    belongs to the planning-scene diagnostic domain and is never a raw object id
    (F2.5).  Returns None when the target is absent.
    """
    for object_id in _OBJECT_ID_CANDIDATES:
        obj = _object(frame, object_id)
        if obj is None:
            continue
        pose = obj.get("pose")
        position = _pose_position(pose, "object.pose")
        orientation = _pose_quaternion(pose, "object.pose")
        twist = obj.get("twist", {})
        if not isinstance(twist, Mapping):
            raise EvidenceError("object.twist must be an object")
        velocity = _vector(
            _first(twist, ("linear", "linear_velocity")) or [0, 0, 0],
            "object.twist.linear",
            3,
        )
        return position, orientation, velocity
    return None


def _canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _mk_check(
    name: str,
    passed: bool,
    *,
    metrics: Mapping[str, Any] | None = None,
    reasons: Sequence[str] = (),
    frames: Sequence[int] = (),
) -> dict[str, Any]:
    return {
        "name": name,
        "passed": bool(passed),
        "metrics": dict(metrics or {}),
        "reasons": list(reasons),
        "source_frame_indices": sorted({int(index) for index in frames}),
    }


def _atomic_write_json(path: Path, value: Mapping[str, Any]) -> None:
    """Atomically write a finite JSON object via temp file + os.replace."""
    rendered = json.dumps(value, sort_keys=True, indent=2) + "\n"
    directory = path.parent
    directory.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(directory))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(rendered)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, path)
        dir_fd = os.open(str(directory), os.O_DIRECTORY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    except BaseException:
        try:
            os.unlink(temp_name)
        except OSError:
            pass
        raise


# --------------------------------------------------------------------------- #
# Public window API
# --------------------------------------------------------------------------- #
def select_integrated_gate_window(
    records: Sequence[Mapping[str, Any]],
    attempt: Path,
    gate: str,
    *,
    attempt_id: str,
    manifest_present: bool,
    gate_start: float,
    gate_end: float,
    physics_hz: float,
) -> tuple[list[Mapping[str, Any]], Path | None]:
    """Select the authoritative raw-physics gate window.

    Wraps ``manipulation_gate_verifier._raw_gate_window`` and narrows it:
    retain only the nearest pre-start frame, retain every exact in-gate frame,
    never admit a post-terminal frame, and reject duplicate frame indices.
    """
    if not math.isfinite(float(physics_hz)) or float(physics_hz) <= 0:
        raise EvidenceError("physics_hz must be finite and positive")
    if not math.isfinite(float(gate_start)) or not math.isfinite(float(gate_end)):
        raise EvidenceError("gate boundaries must be finite")
    if float(gate_end) < float(gate_start):
        raise EvidenceError("gate_end precedes gate_start")
    selected, window_path = _raw_gate_window(
        records,
        attempt,
        gate,
        attempt_id=attempt_id,
        manifest_present=manifest_present,
        gate_start=gate_start,
        gate_end=gate_end,
        physics_hz=physics_hz,
    )
    pre_start = [record for record in selected if _record_timestamp(record) < gate_start]
    in_gate = [
        record
        for record in selected
        if gate_start <= _record_timestamp(record) <= gate_end
    ]
    if not pre_start:
        raise EvidenceError("integrated gate window requires one pre-start frame")
    nearest_pre_start = max(pre_start, key=_record_timestamp)
    result = [nearest_pre_start, *in_gate]
    if len({int(record["frame_index"]) for record in result}) != len(result):
        raise EvidenceError("integrated gate window contains duplicate frame indices")
    return result, window_path


def _record_timestamp(record: Mapping[str, Any]) -> float:
    value = record.get("timestamp")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvidenceError("physics truth record requires a numeric timestamp")
    timestamp = float(value)
    if not math.isfinite(timestamp):
        raise EvidenceError("physics truth timestamp must be finite")
    return timestamp


# --------------------------------------------------------------------------- #
# Physics / boundary resolution
# --------------------------------------------------------------------------- #
def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _resolve_physics_hz(config: Mapping[str, Any]) -> float:
    """Resolve ``physics.hz`` through the integrated config's ``core_config``."""
    core_config = config.get("core_config")
    if not isinstance(core_config, str) or not core_config:
        raise EvidenceError("integrated config requires a string core_config path")
    core_path = Path(core_config)
    if not core_path.is_absolute():
        core_path = _repo_root() / core_path
    core = _read_json(core_path)
    physics = core.get("physics")
    if not isinstance(physics, Mapping):
        raise EvidenceError("core config has no physics object")
    hz = _finite(physics.get("hz"), "core_config.physics.hz")
    if hz <= 0:
        raise EvidenceError("core_config.physics.hz must be positive")
    return hz


def _journal_event_keys(
    journal_records: Sequence[Mapping[str, Any]],
) -> dict[str, tuple[int, float]]:
    """First occurrence of each journal event -> (frame_index, timestamp)."""
    keys: dict[str, tuple[int, float]] = {}
    for record in journal_records:
        event = str(record.get("event", ""))
        if event and event not in keys:
            keys[event] = (
                _as_index(record["frame_index"], f"journal.{event}.frame_index"),
                _as_timestamp(record["timestamp"], f"journal.{event}.timestamp"),
            )
    return keys


def _derive_boundaries(
    journal_records: Sequence[Mapping[str, Any]],
    stage: str,
    kind: str,
) -> tuple[float, float]:
    """Derive sim-time gate boundaries from the planning-scene journal keys.

    ``gate_start`` is always the ``fixture-ready`` timestamp.  ``gate_end`` is
    the per-scenario terminal anchor (Table 1): ``teardown`` only for Gate C and
    malformed-back; otherwise the named terminal event (never teardown).
    """
    keys = _journal_event_keys(journal_records)
    if "fixture-ready" not in keys:
        raise EvidenceError("planning-scene journal has no fixture-ready record")
    gate_start = keys["fixture-ready"][1]
    if stage == "C":
        anchor_event = "teardown"
    elif stage == "D":
        anchor_event = _D_GATE_END_EVENT.get(kind)
        if anchor_event is None and kind == "gripper":
            gripper_terminals = [
                str(record["event"])
                for record in journal_records
                if str(record["event"]).startswith("gripper-")
                and str(record["event"]).endswith("-terminal")
            ]
            if not gripper_terminals:
                raise EvidenceError(
                    "gripper journal has no gripper-*-terminal anchor"
                )
            anchor_event = gripper_terminals[-1]
    elif stage == "E":
        anchor_event = _E_GATE_END_EVENT.get(kind)
    else:
        raise EvidenceError(f"unknown scenario stage {stage!r}")
    if anchor_event is None or anchor_event not in keys:
        raise EvidenceError(
            f"planning-scene journal has no terminal anchor {anchor_event!r}"
        )
    gate_end = keys[anchor_event][1]
    if gate_end < gate_start:
        raise EvidenceError("terminal anchor timestamp precedes fixture-ready")
    return gate_start, gate_end


def _subwindow(
    ctx: Mapping[str, Any],
    start_event: str,
    end_event: str,
) -> list[Mapping[str, Any]]:
    """Select raw window records in ``[start_event, end_event]`` journal keys."""
    keys = ctx["event_keys"]
    if start_event not in keys or end_event not in keys:
        return []
    start_ts = keys[start_event][1]
    end_ts = keys[end_event][1]
    return [
        record
        for record in ctx["window_records"]
        if start_ts <= _record_timestamp(record) <= end_ts
    ]


# --------------------------------------------------------------------------- #
# Common evidence validation
# --------------------------------------------------------------------------- #
def _raw_evaluator_correlation(
    raw_records: Sequence[Mapping[str, Any]],
    evaluator_records: Sequence[Mapping[str, Any]],
    *,
    raw_start_index: int,
    evaluator_start_index: int | None,
) -> None:
    """Exact per-index canonical equality with a distinct drain-mismatch code."""
    raw_tail = raw_records[raw_start_index:]
    ev_start = int(evaluator_start_index) if evaluator_start_index is not None else 0
    ev_tail = evaluator_records[ev_start:]
    if len(ev_tail) != len(raw_tail):
        raise EvidenceError(
            "raw/evaluator drain mismatch: "
            f"raw={len(raw_tail)} evaluator={len(ev_tail)}"
        )
    for index, (raw, evaluated) in enumerate(zip(raw_tail, ev_tail), 1):
        frame = evaluated.get("frame")
        if not isinstance(frame, Mapping):
            raise EvidenceError(
                f"evaluator record {index} has no embedded raw frame"
            )
        if _canonical_json(frame) != _canonical_json(raw):
            raise EvidenceError(
                f"evaluator record {index} does not exactly match raw truth"
            )


def _integrated_endpoint_validator(
    records: Sequence[Mapping[str, Any]],
    *,
    forbidden_endpoints: Sequence[str],
    forbidden_tokens: Sequence[str],
) -> list[str]:
    """Endpoint/source allowlist validator for integrated diagnostic artifacts.

    Never calls ``manipulation_gate_verifier._is_external``: ``diagnostic_only``
    rows are the expected executor output.  Endpoints present in the artifacts
    must be in ``REQUIRED_ACTIONS ∪ REQUIRED_SERVICES`` and not forbidden.
    Paired source metadata, when actually persisted in the **same** mapping as
    its endpoint, must match the committed endpoint-source mapping.  Unrelated
    semantic ``source`` fields (e.g. ``env_cloud_evidence.source ==
    "observed-environment-cloud"``) are not endpoint provider metadata and are
    never flagged (F1.3).  Provider/goal selection fields are scanned for
    forbidden tokens and a persisted ``pipeline_id`` must be ``"ompl"`` (F1.6).
    Returns a list of reason strings (empty = valid).
    """
    reasons: list[str] = []
    forbidden = {str(value) for value in forbidden_endpoints}
    forbidden = {value for value in forbidden if value}
    tokens = [str(value) for value in forbidden_tokens if str(value)]

    def _tainted(value: Any) -> bool:
        lowered = str(value).lower()
        return any(token.lower() in lowered for token in tokens)

    for record in records:
        for mapping in _walk(record):
            endpoint_values: list[str] = []
            source_values: list[str] = []
            provider_values: list[str] = []
            pipeline_values: list[str] = []
            for key, value in mapping.items():
                normalized = str(key).lower()
                if normalized in _ENDPOINT_KEYS and value is not None:
                    endpoint_values.append(str(value))
                if normalized in _SOURCE_KEYS and value is not None and str(value):
                    source_values.append(str(value))
                if normalized in _PROVIDER_KEYS and value is not None and str(value):
                    provider_values.append(str(value))
                if normalized == "pipeline_id" and value is not None and str(value):
                    pipeline_values.append(str(value))
            for endpoint in endpoint_values:
                if not endpoint:
                    continue
                if endpoint not in ALLOWED_ENDPOINTS:
                    reasons.append(f"endpoint {endpoint!r} is not in the allowlist")
                if endpoint in forbidden:
                    reasons.append(f"endpoint {endpoint!r} is forbidden")
                if _tainted(endpoint):
                    reasons.append(
                        f"endpoint {endpoint!r} contains a forbidden token"
                    )
            # Provider/goal field taint (F1.6/F2.2).
            for provider in provider_values:
                if _tainted(provider):
                    reasons.append(
                        f"provider field {provider!r} contains a forbidden token"
                    )
            # Source/provenance values are scanned for forbidden tokens too,
            # EXCEPT the committed semantic provenance values that are not
            # endpoint provider metadata (F2.2).  A source value carrying a
            # forbidden token fails because it carries the token, not because it
            # is absent from the endpoint-provider node allowlist.
            for source in source_values:
                if source in _SEMANTIC_SOURCE_PROVENANCE:
                    continue
                if _tainted(source):
                    reasons.append(
                        f"source field {source!r} contains a forbidden token"
                    )
            # A persisted pipeline_id must be the integrated ompl planner.
            # F2.2: exact lowercase ``"ompl"`` is canonical identity strictness,
            # deliberately not normalized; case variants are evidence-invalid.
            for pipeline in pipeline_values:
                if pipeline != "ompl":
                    reasons.append(
                        f"pipeline_id {pipeline!r} is not the integrated ompl planner"
                    )
            # Paired source ownership is validated ONLY when the endpoint and
            # its source coexist in the same endpoint-evidence mapping (F1.3).
            if endpoint_values and source_values:
                for endpoint in endpoint_values:
                    expected = _REQUIRED_ENDPOINT_SOURCES.get(endpoint)
                    if expected is not None:
                        for source in source_values:
                            if source != expected:
                                reasons.append(
                                    f"endpoint {endpoint!r} source {source!r} "
                                    f"does not match expected {expected!r}"
                                )
    return reasons


def _validate_scene_journal(
    journal_records: Sequence[Mapping[str, Any]],
    ctx: Mapping[str, Any],
) -> None:
    """Validate planning-scene journal event order, ownership, attachment, keys.

    Raises ``EvidenceError`` on any violation.  Never uses the journal as
    physical evidence; only event order, ownership, phase-aware attachment,
    and ``(frame_index, timestamp)`` correlation to raw truth.
    """
    stage = ctx["stage"]
    kind = ctx["kind"]
    integrated = ctx["integrated"]
    target = TASK_TARGET_ID
    events = [str(record.get("event", "")) for record in journal_records]
    if not events:
        raise EvidenceError("planning-scene journal contains no records")

    # --- 1. Exact required event order -------------------------------------
    if stage == "C":
        required = GATE_C_REQUIRED_EVENT_ORDER
        forbidden = GATE_C_FORBIDDEN_EVENTS
    elif stage == "D":
        if kind == "gripper" and tuple(events) == GRIPPER_CLOSE_FIRST_EVENT_ORDER:
            required = GRIPPER_CLOSE_FIRST_EVENT_ORDER
        else:
            required = STAGE_D_REQUIRED_EVENT_ORDER.get(kind, GATE_C_REQUIRED_EVENT_ORDER)
        forbidden = D_FORBIDDEN_EVENTS
    else:
        required = STAGE_E_REQUIRED_EVENT_ORDER.get(kind, POSITIVE_ORDER)
        forbidden = STAGE_E_FORBIDDEN_EVENTS.get(kind, ())
    if tuple(events) != tuple(required):
        raise EvidenceError(
            "planning-scene journal event order violated: "
            f"{events} must equal {list(required)} exactly"
        )
    leaked = [event for event in events if event in set(forbidden)]
    if leaked:
        raise EvidenceError(
            f"planning-scene journal leaked forbidden events: {leaked}"
        )

    # --- 2. Ownership --------------------------------------------------------
    expected_owned = set(integrated.get("expected_scene", {}).get("owned_ids", []))
    for record in journal_records:
        if set(record.get("owned_ids", [])) != expected_owned:
            raise EvidenceError(
                f"journal {record.get('event')!r} owned_ids differ from "
                "integrated.expected_scene.owned_ids"
            )

    # --- 3. Phase-aware attachment ------------------------------------------
    scene_attach_indices = [
        index for index, event in enumerate(events) if event == "scene-attach"
    ]
    scene_detach_indices = [
        index for index, event in enumerate(events) if event == "scene-detach"
    ]
    has_attach = bool(scene_attach_indices)
    if scene_attach_indices:
        attach_index = scene_attach_indices[0]
    else:
        attach_index = None
    if scene_detach_indices:
        detach_index = scene_detach_indices[0]
    else:
        detach_index = None

    for index, record in enumerate(journal_records):
        attached = target in set(record.get("attached_ids", []))
        if attach_index is None:
            # No legal attach for this scenario class.
            if attached:
                raise EvidenceError(
                    f"journal {record.get('event')!r} attaches the target where "
                    "the scenario order forbids scene-attach"
                )
            continue
        if index < attach_index:
            if attached:
                raise EvidenceError(
                    f"journal {record.get('event')!r} attaches the target before "
                    "the scene-attach record"
                )
        elif index == attach_index:
            if not attached:
                raise EvidenceError(
                    "scene-attach record does not attach the task target"
                )
            attached_links = record.get("attached_links", {})
            touch_links = record.get("touch_links", {})
            if attached_links.get(target) != CANONICAL_LINK_TCP:
                raise EvidenceError(
                    "scene-attach attaches the target to a non-canonical link"
                )
            if tuple(touch_links.get(target, ())) != tuple(CANONICAL_TOUCH_LINKS):
                raise EvidenceError(
                    "scene-attach target touch links differ from canonical touch links"
                )
        elif detach_index is not None and index >= detach_index:
            # At and after scene-detach the task target is absent (world-state
            # transition back).  The scene-detach record itself is detached:
            # the executor snapshots the current scene only after the target has
            # left the attached set (F1.2).
            if attached:
                raise EvidenceError(
                    f"journal {record.get('event')!r} carries the target at or "
                    "after scene-detach"
                )
        else:
            # Retained phase: attach_index < index < detach_index (or through
            # the anchor when the scenario order forbids detach).  Target stays
            # attached.
            if not attached:
                raise EvidenceError(
                    f"journal {record.get('event')!r} dropped the target during "
                    "the retained phase"
                )

    # --- 4. (frame_index, timestamp) correlation to raw truth ----------------
    raw_by_frame = ctx["raw_by_frame"]
    tolerance = 1.0 / ctx["physics_hz"]
    for record in journal_records:
        frame_index = _as_index(record["frame_index"], "journal.frame_index")
        timestamp = _as_timestamp(record["timestamp"], "journal.timestamp")
        raw = raw_by_frame.get(frame_index)
        if raw is None:
            raise EvidenceError(
                f"journal {record.get('event')!r} frame_index {frame_index} "
                "does not reference a raw physics truth frame"
            )
        raw_timestamp = _record_timestamp(raw)
        if abs(raw_timestamp - timestamp) > tolerance + 1e-12:
            raise EvidenceError(
                f"journal {record.get('event')!r} timestamp {timestamp} does not "
                f"match raw frame {frame_index} timestamp {raw_timestamp}"
            )

    # --- 5. Task-7 rule: bilateral contact strictly before scene-attach ------
    if attach_index is not None:
        attach_key = (
            _as_index(journal_records[attach_index]["frame_index"], "scene-attach.frame_index"),
            _as_timestamp(journal_records[attach_index]["timestamp"], "scene-attach.timestamp"),
        )
        earliest_bilateral: int | None = None
        for parsed in ctx["all_parsed"]:
            if _bilateral_strict(parsed["raw"], ctx["thresholds"]["contact_force_n"]) == (
                True,
                True,
            ):
                earliest_bilateral = int(parsed["frame_index"])
                break
        if earliest_bilateral is not None and earliest_bilateral >= attach_key[0]:
            raise EvidenceError(
                "bilateral physical contact is not strictly before the "
                "scene-attach journal join key"
            )


def _bilateral_strict(frame: Mapping[str, Any], threshold: float) -> tuple[bool, bool]:
    """Bilateral finger/cube contact with strict force `>` threshold."""
    left = right = False
    for contact in _contacts(frame):
        if _contact_force(contact) <= threshold:
            continue
        bodies = {
            _norm_body(contact.get("body_a", "")),
            _norm_body(contact.get("body_b", "")),
        }
        if "qualification_cube" not in bodies:
            continue
        if bodies & {"left_finger", "left_finger_link"}:
            left = True
        if bodies & {"right_finger", "right_finger_link"}:
            right = True
    return left, right


def _finger_cube_contacts(frame: Mapping[str, Any], threshold: float) -> list[Mapping[str, Any]]:
    """Contacts between either finger and the cube above the strict threshold."""
    result: list[Mapping[str, Any]] = []
    for contact in _contacts(frame):
        if _contact_force(contact) <= threshold:
            continue
        bodies = {
            _norm_body(contact.get("body_a", "")),
            _norm_body(contact.get("body_b", "")),
        }
        if "qualification_cube" not in bodies:
            continue
        if bodies & {"left_finger", "left_finger_link", "right_finger", "right_finger_link"}:
            result.append(contact)
    return result


def _arm_obstacle_contacts(
    frame: Mapping[str, Any],
    threshold: float,
    obstacle_bodies: frozenset[str],
) -> list[Mapping[str, Any]]:
    """Contacts between a PhysX-monitored arm body and an obstacle body."""
    result: list[Mapping[str, Any]] = []
    for contact in _contacts(frame):
        if _contact_force(contact) <= threshold:
            continue
        bodies = {
            _norm_body(contact.get("body_a", "")),
            _norm_body(contact.get("body_b", "")),
        }
        arm = bodies & _ROBOT_CONTACT_BODIES
        obstacle = bodies & obstacle_bodies
        if arm and obstacle:
            result.append(contact)
    return result


def _obstacle_body_set(scenario_bundle: Mapping[str, Any]) -> frozenset[str]:
    declaration = scenario_bundle.get("scenario", {}).get("declaration", {})
    ids: set[str] = set()
    for collection in ("actors", "objects"):
        for record in declaration.get(collection, []):
            if isinstance(record, Mapping):
                ids.add(str(record.get("id", "")))
    ids.discard("qualification_cube")
    ids.discard("")
    return frozenset(ids)


def _planning_scene_final(attempt_dir: Path) -> None:
    final_path = attempt_dir / "planning-scene.json"
    if not final_path.is_file():
        return
    final = _read_json(final_path)
    if str(final.get("status")) != "diagnostic-pass":
        raise EvidenceError(
            f"planning-scene.json status is {final.get('status')!r}, expected diagnostic-pass"
        )
    if str(final.get("authority")) != "physics_truth":
        raise EvidenceError(
            f"planning-scene.json authority is {final.get('authority')!r}, expected physics_truth"
        )


def _execution_gateway_errors(
    execution_rows: Sequence[Mapping[str, Any]],
) -> list[str]:
    errors: list[str] = []
    for row in execution_rows:
        reason = row.get("reason_code")
        if reason is None:
            continue
        rendered = str(reason).lower()
        if rendered.startswith("gateway") or rendered.startswith("command-"):
            errors.append(f"execution reason_code {reason!r} is a gateway/command class")
    return errors


# --------------------------------------------------------------------------- #
# Stage-specific physical checks
# --------------------------------------------------------------------------- #
_GOAL_STATUS_NON_SUCCESS = frozenset({EXECUTE_STATUS_CANCELED, 6})


def _terminal_success(summary: Mapping[str, Any]) -> bool:
    """Resolve the executor terminal across all present terminal domains.

    Each present domain is parsed with exact types and exact enum semantics.
    When multiple domains are present they must agree on the success/non-success
    polarity; a contradictory pair raises ``EvidenceError`` (-> evidence-invalid),
    never a permissive selection.  ``diagnostic-pass`` is an artifact status and
    is never terminal proof.  ``terminal_status``, ``execute_result_status``
    (action GoalStatus) and ``task_result_status`` (Pick/Place Result) are kept
    as separate domains.
    """
    verdicts: list[bool] = []

    terminal = summary.get("terminal_status")
    if terminal is not None:
        rendered = str(terminal).strip().lower()
        if rendered in _SUCCESS_TERMINALS:
            verdicts.append(True)
        elif rendered in _NON_SUCCESS_TERMINALS:
            verdicts.append(False)
        else:
            raise EvidenceError(
                f"terminal_status {terminal!r} is not a known terminal"
            )

    result_status = summary.get("execute_result_status")
    if result_status is not None:
        status = _as_int(result_status, "execute_result_status")
        if status == EXECUTE_STATUS_SUCCEEDED:
            verdicts.append(True)
        elif status in _GOAL_STATUS_NON_SUCCESS:
            verdicts.append(False)
        else:
            raise EvidenceError(
                f"execute_result_status {status} is not a terminal GoalStatus"
            )

    task_result = summary.get("task_result_status")
    if task_result is not None:
        status = _as_int(task_result, "task_result_status")
        verdicts.append(status == PICK_PLACE_RESULT_SUCCESS)

    if not verdicts:
        raise EvidenceError("execution summary has no terminal evidence")
    first = verdicts[0]
    if any(value != first for value in verdicts[1:]):
        raise EvidenceError(
            f"conflicting terminal domains: {verdicts}"
        )
    return first


def _terminal_non_success(summary: Mapping[str, Any]) -> bool:
    return _terminal_success(summary) is False


def _expected_target_pose(ctx: Mapping[str, Any]) -> list[float] | None:
    """Derive the D execute-pose expected TCP target from the scenario bundle."""
    declaration = ctx.get("planning_scene_declaration") or {}
    target_source_id = declaration.get("target_source_id")
    objects = declaration.get("objects", [])
    if not isinstance(objects, list):
        return None
    for record in objects:
        if isinstance(record, Mapping) and record.get("id") == target_source_id:
            pose = record.get("pose")
            if isinstance(pose, Mapping):
                xyz = pose.get("xyz")
                if isinstance(xyz, Sequence) and not isinstance(xyz, (str, bytes)) and len(xyz) == 3:
                    return [_finite(value, "target pose xyz") for value in xyz]
    return None


def _pose_error_deg(actual: Sequence[float], target: Sequence[float]) -> float:
    dot = abs(sum(a * b for a, b in zip(actual, target)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def _world_to_base_link(
    tcp_pose: Any, base_pose: Any
) -> tuple[list[float], list[float]]:
    """Transform a WORLD-frame TCP pose into the base_link frame.

    The sim reports ``tcp_pose`` and ``base_pose`` in the WORLD frame (backend.py
    ``body_pos_w`` / ``root_pos_w``); the commanded execute-pose target is in
    base_link.  The free-floating sim base can yaw under arm reaction torque, so
    the world-frame TCP must be transformed into base_link before comparing (live
    rerun-11: 72.6 deg base yaw -> false 1.13 m / 71.5 deg error; base-frame error
    is 6.2 mm / 3.7 deg).  Pure Python quaternion math, ROS-free.

    Returns ``(xyz, quaternion_xyzw)`` in base_link.  When ``base_pose`` is
    absent the TCP is treated as already base_link (backward compatible).
    """
    tcp_xyz = _pose_position(tcp_pose, "tcp_pose")
    tcp_quat = _pose_quaternion(tcp_pose, "tcp_pose")
    if base_pose is None:
        return tcp_xyz, tcp_quat
    base_xyz = _pose_position(base_pose, "base_pose")
    base_quat = _pose_quaternion(base_pose, "base_pose")
    # body -> world is q_base; world -> body is q_base^-1 (conjugate).
    rel = [tcp_xyz[0] - base_xyz[0], tcp_xyz[1] - base_xyz[1], tcp_xyz[2] - base_xyz[2]]
    local_xyz = _q_rotate(_q_inverse(base_quat), rel)
    local_quat = _q_mul(_q_inverse(base_quat), tcp_quat)
    return local_xyz, local_quat


# --- Gate C --------------------------------------------------------------- #
def _gate_c_checks(ctx: Mapping[str, Any]) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    kind = ctx["kind"]
    thresholds = ctx["thresholds"]
    tolerance = thresholds["numeric_tolerance"]
    window_parsed = ctx["window_parsed"]
    summary = ctx["execution_summary"]
    moveit_plans = ctx["moveit_plans"]

    # 1. Plan-only OMPL diagnostic result.
    plan_rows = [row for row in moveit_plans if row.get("row_kind") == "lifecycle"]
    if not plan_rows:
        raise EvidenceError("moveit-plans.jsonl contains no plan-result rows")
    planner_status = plan_rows[-1].get("planner_status")
    nonempty_plan = plan_rows[-1].get("nonempty_plan")
    error_code = plan_rows[-1].get("error_code")
    summary_result = summary.get("result", {})
    if isinstance(summary_result, Mapping):
        if summary_result.get("nonempty_plan") is not None:
            nonempty_plan = summary_result["nonempty_plan"]
        if summary_result.get("error_code") is not None:
            error_code = summary_result["error_code"]
    expected_success = str(planner_status).strip().lower() == "diagnostic-pass"
    expected_nonempty = _as_bool(nonempty_plan) is True
    passed = expected_success and expected_nonempty
    metrics = {
        "planner_status": planner_status,
        "nonempty_plan": nonempty_plan,
        "error_code": error_code,
    }
    reasons = []
    if not expected_success:
        reasons.append("planner_status is not success")
    if not expected_nonempty:
        reasons.append("nonempty_plan is not true")
    checks.append(_mk_check("plan_only_ompl_result", passed, metrics=metrics, reasons=reasons))

    # 2. No target-command delta.
    # Plan-only command targets are zero-filled by the sim publisher except for
    # a one-frame ``command_targets`` population race that transiently fills the
    # seven arm entries with the real current positions (0 -> real -> 0).  A
    # frame whose seven arm values are all within the zero threshold carries no
    # command and does not contribute to the delta; an isolated non-zero frame
    # is that transient race and is ignored.  A SUSTAINED run of genuinely
    # commanded targets still fails exactly as before: the entry step from the
    # preceding zero-filled frame into a run of length >= 2 is a real delta, and
    # intra-run motion is a real delta.
    target_deltas: list[list[float]] = []
    used_frames: list[int] = []
    for parsed in window_parsed:
        target = _target_vector(parsed["raw"])
        if target is None:
            continue
        target_deltas.append(target)
        used_frames.append(int(parsed["frame_index"]))
    zero_threshold = float(
        thresholds.get("plan_only_target_zero_threshold", tolerance)
    )
    max_target_delta = _plan_only_target_delta(target_deltas, zero_threshold)
    checks.append(
        _mk_check(
            "no_target_command_delta",
            max_target_delta <= tolerance,
            metrics={"max_target_delta_rad": max_target_delta},
            reasons=() if max_target_delta <= tolerance else ("command_targets moved during plan-only gate",),
            frames=used_frames,
        )
    )

    # 3. No physical motion.
    max_joint_speed = 0.0
    max_tcp_delta = 0.0
    first_tcp: list[float] | None = None
    for parsed in window_parsed:
        _, _, velocities, _ = _arm_joint_data(parsed["raw"])
        max_joint_speed = max(max_joint_speed, max(abs(value) for value in velocities))
        tcp = _pose_position(_robot(parsed["raw"]).get("tcp_pose"), "tcp_pose")
        if first_tcp is None:
            first_tcp = tcp
        else:
            max_tcp_delta = max(max_tcp_delta, _distance(tcp, first_tcp))
    speed_tol = float(
        thresholds.get("plan_only_max_joint_speed_rad_s", thresholds["numeric_tolerance"])
    )
    disp_tol = float(
        thresholds.get("plan_only_max_tcp_displacement_m", thresholds["numeric_tolerance"])
    )
    motion_passed = max_joint_speed <= speed_tol and max_tcp_delta <= disp_tol
    checks.append(
        _mk_check(
            "no_physical_motion",
            motion_passed,
            metrics={"max_joint_speed_rad_s": max_joint_speed, "max_tcp_displacement_m": max_tcp_delta},
            reasons=() if motion_passed else ("plan-only gate observed physical motion",),
        )
    )

    # 4. No execution / no contact / no safety.
    no_execution = all(
        _as_bool(row.get("execute_trajectory_goal_sent")) is not True
        and _as_bool(row.get("controller_goal_sent")) is not True
        for row in ctx["execution_rows"]
    )
    contact_force_threshold = thresholds["contact_force_n"]
    any_contact = any(
        _finger_cube_contacts(parsed["raw"], contact_force_threshold)
        or _arm_obstacle_contacts(
            parsed["raw"], contact_force_threshold, ctx["obstacle_bodies"]
        )
        for parsed in window_parsed
    )
    any_safety = any(_safety(parsed["raw"]) for parsed in window_parsed)
    execution_passed = no_execution and not any_contact and not any_safety
    checks.append(
        _mk_check(
            "no_execution_no_contact_no_safety",
            execution_passed,
            metrics={"no_execution": no_execution, "any_contact": any_contact, "any_safety": any_safety},
            reasons=() if execution_passed else ("plan-only gate observed execution/contact/safety",),
        )
    )
    return checks


# --- Gate D --------------------------------------------------------------- #
def _gate_d_checks(ctx: Mapping[str, Any]) -> list[dict[str, Any]]:
    kind = ctx["kind"]
    thresholds = ctx["thresholds"]
    window_parsed = ctx["window_parsed"]
    summary = ctx["execution_summary"]
    if kind in ("execute-joint", "execute-pose", "retreat", "gripper"):
        terminal_ok = _terminal_success(summary)
        terminal_passed = terminal_ok is True
        terminal_check = _mk_check(
            "terminal_success",
            terminal_passed,
            metrics={"terminal_status": summary.get("terminal_status"),
                     "execute_result_status": summary.get("execute_result_status")},
            reasons=() if terminal_passed else ("executor terminal is not success",),
        )
    if kind == "execute-joint":
        expected = list(Q_OUTBOUND)
        metrics, used = _joint_metrics(window_parsed, expected)
        passed = (
            metrics["final_max_error_rad"] <= thresholds["joint_final_error_rad"]
            and metrics["rms_error_rad"] <= thresholds["joint_rms_error_rad"]
        )
        return [
            _mk_check(
                "joint_execution_tracks",
                passed,
                metrics={
                    "joint_final_error_rad": metrics["final_max_error_rad"],
                    "joint_rms_error_rad": metrics["rms_error_rad"],
                },
                reasons=() if passed else ("measured joint tracking exceeded thresholds",),
                frames=used,
            ),
            terminal_check,
        ]
    if kind == "execute-pose":
        expected_xyz = _expected_target_pose(ctx)
        if expected_xyz is None:
            raise EvidenceError(
                "execute-pose requires a planning-scene target pose for target_source_id"
            )
        # F2 (round 2): the executor plans/executes to the DECLARED target raised
        # by POSE_APPROACH_Z_OFFSET (the generated pose-goal target hovers above
        # the declared target box, never at its center), with the fixed z-down
        # orientation.  The verifier must compare the reached TCP to the APPROACH
        # pose, not the declared pose.
        expected_xyz = [
            expected_xyz[0],
            expected_xyz[1],
            expected_xyz[2] + POSE_APPROACH_Z_OFFSET,
        ]
        # R13: the sim reports tcp_pose and base_pose in the WORLD frame
        # (backend.py body_pos_w / root_pos_w), but the commanded execute-pose
        # target is in base_link.  The free-floating sim base can yaw under arm
        # reaction torque, so the world-frame TCP must be transformed into
        # base_link before comparing (live rerun-11: 72.6 deg base yaw -> false
        # 1.13 m / 71.5 deg error; the base-frame error is 6.2 mm / 3.7 deg).
        # When base_pose is absent the TCP is treated as already base_link.
        robot = _robot(window_parsed[-1]["raw"])
        final_tcp, final_quat = _world_to_base_link(robot.get("tcp_pose"), robot.get("base_pose"))
        position_error = _distance(final_tcp, expected_xyz)
        # G1 (round 3): the executor's generated pose-goal target carries the
        # fixed approach orientation (``POSE_APPROACH_QUATERNION_XYZW``), not the
        # declared yaw-only target-box quaternion.  Comparing the reached TCP
        # against the declared quaternion measured a ~63 deg false orientation
        # error.  The verifier must compare against the approach orientation the
        # executor actually plans to.
        target_quat = list(POSE_APPROACH_QUATERNION_XYZW)
        orientation_error = (
            _pose_error_deg(final_quat, target_quat) if target_quat is not None else 0.0
        )
        passed = (
            position_error <= thresholds["tcp_position_error_m"]
            and orientation_error <= thresholds["tcp_orientation_error_deg"]
        )
        return [
            _mk_check(
                "pose_execution_reaches_tcp",
                passed,
                metrics={
                    "tcp_position_error_m": position_error,
                    "tcp_orientation_error_deg": orientation_error,
                },
                reasons=() if passed else ("measured TCP tracking exceeded thresholds",),
                frames=[int(window_parsed[-1]["frame_index"])],
            ),
            terminal_check,
        ]
    if kind == "retreat":
        start_tcp = _pose_position(_robot(window_parsed[0]["raw"]).get("tcp_pose"), "tcp_pose")
        final_tcp = _pose_position(_robot(window_parsed[-1]["raw"]).get("tcp_pose"), "tcp_pose")
        displacement_z = final_tcp[2] - start_tcp[2]
        total_displacement = _distance(final_tcp, start_tcp)
        distance_passed = (
            displacement_z >= 0.10 and total_displacement >= 0.10
        )
        contact_threshold = thresholds["contact_force_n"]
        collision_contacts: list[int] = []
        for parsed in window_parsed:
            if _arm_obstacle_contacts(
                parsed["raw"], contact_threshold, ctx["obstacle_bodies"]
            ):
                collision_contacts.append(int(parsed["frame_index"]))
        collision_passed = not collision_contacts
        passed = distance_passed and collision_passed
        return [
            _mk_check(
                "cartesian_retreat_collision_aware",
                passed,
                metrics={"tcp_displacement_z_m": displacement_z,
                         "tcp_displacement_m": total_displacement},
                reasons=() if passed else ("retreat distance or obstacle contact failed",),
            ),
            terminal_check,
        ]
    if kind == "gripper":
        travel = _gripper_travel(window_parsed)
        efforts = _gripper_efforts(window_parsed, ctx["execution_rows"])
        min_travel = _finite(ctx["core_thresholds"].get("free_gripper_min_travel_rad", 0.75), "free_gripper_min_travel_rad")
        max_effort = _finite(ctx["core_thresholds"].get("gripper_max_effort_n", 20.0), "gripper_max_effort_n")
        travel_passed = travel >= min_travel
        effort_passed = max((abs(value) for value in efforts), default=0.0) <= max_effort + thresholds["numeric_tolerance"]
        passed = travel_passed and effort_passed
        return [
            _mk_check(
                "gripper_travel_predicates",
                passed,
                metrics={"drive_joint_travel_rad": travel,
                         "max_abs_effort_n": max((abs(value) for value in efforts), default=0.0)},
                reasons=() if passed else ("gripper travel or effort predicate failed",),
            ),
            terminal_check,
        ]
    if kind == "cancel":
        return _gate_d_cancel_checks(ctx)
    if kind == "safety":
        return _gate_d_safety_checks(ctx)
    raise EvidenceError(f"unknown Gate D kind {kind!r}")


def _gripper_travel(window_parsed: Sequence[Mapping[str, Any]]) -> float:
    positions: list[float] = []
    for parsed in window_parsed:
        value = _gripper_position(parsed["raw"])
        if value is not None:
            positions.append(value)
    if not positions:
        raise EvidenceError("gripper window has no drive_joint position samples")
    return max(positions) - min(positions)


def _max_arm_speed(records: Sequence[Mapping[str, Any]]) -> float:
    maximum = 0.0
    for parsed in records:
        _, _, velocities, _ = _arm_joint_data(parsed["raw"])
        maximum = max(maximum, max(abs(value) for value in velocities))
    return maximum


def _plan_only_target_delta(
    targets: Sequence[Sequence[float]],
    zero_threshold: float,
) -> float:
    """Max arm command-target delta for a plan-only gate.

    Plan-only publishes zero-filled ``command_targets`` (seven arm values all
    ``~0``); a one-frame ``command_targets`` population race transiently fills
    them with the real current arm positions before returning to zero.  Such a
    zero-filled frame carries no command and contributes no delta, and an
    isolated non-zero frame (a run of length one) is that race and is ignored.
    A sustained run of genuinely commanded targets still counts exactly as the
    strict check did: the entry step from the last preceding zero-filled frame
    into a run of length >= 2 is a real delta, as is every intra-run delta.
    Fewer than two non-zero frames yields a static (passing) target.
    """
    maximum = 0.0
    run: list[Sequence[float]] = []
    pending_entry = 0.0
    preceding_zero: Sequence[float] | None = None
    for target in targets:
        nonzero = any(abs(value) >= zero_threshold for value in target)
        if nonzero:
            if not run:
                pending_entry = (
                    _max_delta(preceding_zero, target)
                    if preceding_zero is not None
                    else 0.0
                )
                preceding_zero = None
            run.append(target)
        else:
            if run:
                if len(run) >= 2:
                    maximum = max(maximum, pending_entry)
                    for first, second in zip(run, run[1:]):
                        maximum = max(maximum, _max_delta(first, second))
                run = []
                pending_entry = 0.0
            preceding_zero = target
    if run and len(run) >= 2:
        maximum = max(maximum, pending_entry)
        for first, second in zip(run, run[1:]):
            maximum = max(maximum, _max_delta(first, second))
    return maximum


def _max_target_delta(records: Sequence[Mapping[str, Any]]) -> float:
    vectors: list[list[float]] = []
    for parsed in records:
        target = _target_vector(parsed["raw"])
        if target is not None:
            vectors.append(target)
    maximum = 0.0
    for first, second in zip(vectors, vectors[1:]):
        maximum = max(maximum, _max_delta(first, second))
    return maximum


def _terminal_quiescence_tail(
    ctx: Mapping[str, Any],
    *,
    end_event: str = "quiescent",
    tail_size: int = 2,
) -> list[Mapping[str, Any]]:
    """Bounded terminal-quiescence tail ending at/including the ``end_event`` key.

    F2.1: terminal quiescence is proven from this bounded tail, never from
    max-over-the-whole-window (which would reject the arm's real deceleration
    ramp after ``cancel-requested`` / ``operator-clear``).  Requires at least
    ``tail_size`` consecutive fresh raw physics frames ending at/including the
    ``end_event`` join key; missing tail samples are evidence-invalid, not an
    assumed pass.  The exact frame order/timestamps come from the authoritative
    window records (already bounded at the Table-1 terminal anchor, so
    post-terminal drain is structurally excluded).
    """
    keys = ctx["event_keys"]
    if end_event not in keys:
        raise EvidenceError(
            f"journal has no {end_event} key for the terminal-quiescence tail"
        )
    end_ts = keys[end_event][1]
    candidates = [
        record
        for record in ctx["window_records"]
        if _record_timestamp(record) <= end_ts
    ]
    if len(candidates) < tail_size:
        raise EvidenceError(
            f"terminal-quiescence tail needs {tail_size} raw frames ending at "
            f"{end_event}; got {len(candidates)}"
        )
    tail = candidates[-tail_size:]
    for first, second in zip(tail, tail[1:]):
        if int(second["frame_index"]) != int(first["frame_index"]) + 1:
            raise EvidenceError(
                "terminal-quiescence tail is not consecutive raw frames"
            )
    return tail


def _terminal_quiescence(
    ctx: Mapping[str, Any],
    *,
    end_event: str = "quiescent",
) -> tuple[list[Mapping[str, Any]], float, float]:
    """Resolve the terminal-quiescence tail into (parsed, max_speed, target_delta)."""
    tail = _terminal_quiescence_tail(ctx, end_event=end_event)
    tail_parsed = [
        ctx["parsed_by_frame"][_as_index(r["frame_index"], "tail.frame_index")]
        for r in tail
    ]
    return tail_parsed, _max_arm_speed(tail_parsed), _max_target_delta(tail_parsed)


def _gate_d_cancel_checks(ctx: Mapping[str, Any]) -> list[dict[str, Any]]:
    thresholds = ctx["thresholds"]
    tolerance = thresholds["numeric_tolerance"]
    summary = ctx["execution_summary"]
    # F1.4: parse terminal domains with exact semantics; a contradictory pair
    # raises EvidenceError instead of resolving permissively.
    _terminal_success(summary)
    canceled = (
        str(summary.get("terminal_status", "")).strip().lower()
        in {"canceled", "cancelled"}
    ) or summary.get("execute_result_status") == EXECUTE_STATUS_CANCELED
    cancel_check = _mk_check(
        "execute_goal_canceled",
        canceled,
        metrics={"execute_result_status": summary.get("execute_result_status"),
                 "terminal_status": summary.get("terminal_status")},
        reasons=() if canceled else ("execute goal was not canceled",),
    )
    # F1.7/F2.1: every scenario-owned temporal predicate reads only its exact
    # observation subwindow [cancel-requested, quiescent]; motion after
    # quiescent (post-terminal drain before teardown) is never admitted.
    # Terminal quiescence is proven from a bounded tail ending at the quiescent
    # join key (at least two consecutive settled frames).  The arm's real
    # deceleration ramp inside the observation window is allowed — only the tail
    # must be at rest.  A NEW command target/goal anywhere in the subwindow
    # (even if the velocity later settles) fails closed.
    sub = _subwindow(ctx, "cancel-requested", "quiescent")
    sub_parsed = [ctx["parsed_by_frame"][_as_index(r["frame_index"], "subwindow.frame_index")] for r in sub]
    sub_target = _max_target_delta(sub_parsed)
    tail_parsed, tail_speed, tail_target = _terminal_quiescence(ctx, end_event="quiescent")
    quiescent_passed = (
        tail_speed <= thresholds["safety_stop_velocity_rad_s"]
        and sub_target <= tolerance
    )
    quiescent_check = _mk_check(
        "quiescent_after_cancel",
        quiescent_passed,
        metrics={
            "tail_joint_speed_rad_s": tail_speed,
            "subwindow_target_delta_rad": sub_target,
            "tail_frame_count": len(tail_parsed),
        },
        reasons=() if quiescent_passed else (
            "arm not quiescent at the cancel tail or a new command target appeared",
        ),
        frames=[int(p["frame_index"]) for p in tail_parsed],
    )
    keys = ctx["event_keys"]
    cancel_key = keys.get("cancel-requested", (0, 0))
    later_stage = any(
        event in ("execution-terminal", "execution-start")
        for event in keys
        if keys[event][0] > cancel_key[0]
    )
    # F2.1 item 4: no_later_stage forbids later journal stages AND requires the
    # same terminal-quiescence tail (rest + no new command target in the
    # subwindow); it never requires <= numeric_tolerance velocity over the
    # entire braking interval.
    no_later_passed = (not later_stage) and quiescent_passed
    no_later_check = _mk_check(
        "no_later_stage",
        no_later_passed,
        metrics={
            "later_stage": later_stage,
            "tail_joint_speed_rad_s": tail_speed,
            "subwindow_target_delta_rad": sub_target,
        },
        reasons=() if no_later_passed else (
            "cancel was followed by a later stage or the cancel tail is not quiescent",
        ),
    )
    return [cancel_check, quiescent_check, no_later_check]


def _gate_d_safety_checks(ctx: Mapping[str, Any]) -> list[dict[str, Any]]:
    thresholds = ctx["thresholds"]
    tolerance = thresholds["numeric_tolerance"]
    # S2: the sim's commanded joint target drifts ~0.003 rad even while the arm
    # is physically frozen — the same phantom-motion floor the round-4 velocity
    # threshold addressed.  The target-frozen and no_auto_resume target-delta
    # checks must therefore bound target delta by the position-creep bound
    # (``safety_target_delta_rad``, defaulting to ``safety_position_creep_rad``),
    # never ``numeric_tolerance`` (1e-06), which models exact numeric equality
    # and false-fails a genuinely-frozen arm.  ``numeric_tolerance`` remains the
    # authority for every other numeric-equality check.
    target_delta_limit = float(
        thresholds.get(
            "safety_target_delta_rad",
            thresholds.get("safety_position_creep_rad", 0.005),
        )
    )
    keys = ctx["event_keys"]
    summary = ctx["execution_summary"]
    effective_key = keys.get("effective-stop")
    quiescent_key = keys.get("quiescent")
    if effective_key is None or quiescent_key is None:
        raise EvidenceError("safety journal missing effective-stop/quiescent keys")

    # Safety effective stop within safety_stop_frames.
    stop_deadline_frame = int(effective_key[0]) + int(thresholds["safety_stop_frames"])
    stop_frame: int | None = None
    for parsed in ctx["all_parsed"]:
        if int(parsed["frame_index"]) < int(effective_key[0]):
            continue
        if int(parsed["frame_index"]) > stop_deadline_frame:
            break
        if _safety(parsed["raw"]):
            stop_frame = int(parsed["frame_index"])
            break
    stop_passed = stop_frame is not None
    stop_check = _mk_check(
        "safety_effective_stop",
        stop_passed,
        metrics={"safety_stop_frame": stop_frame, "effective_stop_frame": int(effective_key[0])},
        reasons=() if stop_passed else ("no safety stop within safety_stop_frames",),
        frames=[] if stop_frame is None else [stop_frame],
    )

    # Target frozen + position creep bounded through quiescent.
    frozen_records = [
        parsed
        for parsed in ctx["all_parsed"]
        if int(parsed["frame_index"]) >= int(effective_key[0])
        and int(parsed["frame_index"]) <= int(quiescent_key[0])
    ]
    if not frozen_records:
        raise EvidenceError("safety observation window has no raw frames")
    max_target = _max_target_delta(frozen_records)
    stop_position = _arm_joint_data(frozen_records[0]["raw"])[1]
    creep = 0.0
    for parsed in frozen_records[1:]:
        _, positions, _, _ = _arm_joint_data(parsed["raw"])
        creep = max(creep, _max_delta(positions, stop_position))
    frozen_passed = (
        max_target <= target_delta_limit and creep <= thresholds["safety_position_creep_rad"]
    )
    frozen_check = _mk_check(
        "target_frozen",
        frozen_passed,
        metrics={"max_target_delta_rad": max_target, "position_creep_rad": creep},
        reasons=() if frozen_passed else ("target or position not frozen after effective stop",),
        frames=[int(p["frame_index"]) for p in frozen_records],
    )

    # No auto-resume: no target change after operator-clear without a new goal.
    # F1.7: the check reads only the [operator-clear, quiescent] subwindow;
    # post-quiescent drain is never admitted.
    auto_resume = False
    clear_key = keys.get("operator-clear")
    if clear_key is not None and quiescent_key is not None:
        frozen_target = _arm_joint_data(frozen_records[0]["raw"])[1]
        for parsed in ctx["all_parsed"]:
            if parsed["timestamp"] < clear_key[1]:
                continue
            if parsed["timestamp"] > quiescent_key[1]:
                break
            target = _target_vector(parsed["raw"])
            if target is not None:
                if _max_delta(target, frozen_target) > target_delta_limit:
                    auto_resume = True
                    break
    no_resume_check = _mk_check(
        "no_auto_resume",
        not auto_resume,
        metrics={"auto_resume": auto_resume},
        reasons=() if not auto_resume else ("arm auto-resumed after operator-clear",),
    )

    # F2.3: the safety terminal must be a consistent non-success across every
    # present real status domain (GoalStatus + task Result kept separate).
    # ``_terminal_non_success`` raises EvidenceError on a contradictory pair
    # (-> evidence-invalid); a terminal claiming success fails the safety check.
    safety_terminal_ok = _terminal_non_success(summary)
    terminal_check = _mk_check(
        "safety_terminal_non_success",
        safety_terminal_ok,
        metrics={"terminal_status": summary.get("terminal_status"),
                 "execute_result_status": summary.get("execute_result_status")},
        reasons=() if safety_terminal_ok else ("safety terminal was success",),
    )
    return [stop_check, frozen_check, no_resume_check, terminal_check]


# --- Gate E positive ------------------------------------------------------ #
def _gate_e_positive_checks(ctx: Mapping[str, Any]) -> list[dict[str, Any]]:
    thresholds = ctx["thresholds"]
    tolerance = thresholds["numeric_tolerance"]
    contact_threshold = thresholds["contact_force_n"]
    window_parsed = ctx["window_parsed"]
    initial = ctx["initial"]
    region = ctx["region_center"]

    # 1. bilateral_contact (strict > threshold).
    bilateral_frames: list[int] = []
    for parsed in window_parsed:
        if _bilateral_strict(parsed["raw"], contact_threshold) == (True, True):
            bilateral_frames.append(int(parsed["frame_index"]))
    bilateral_passed = bool(bilateral_frames)
    bilateral_check = _mk_check(
        "bilateral_contact",
        bilateral_passed,
        metrics={"bilateral_frame_count": len(bilateral_frames)},
        reasons=() if bilateral_passed else ("no bilateral finger/cube contact above threshold",),
        frames=bilateral_frames,
    )

    # 2. lift from the pre-start frame's cube pose.
    lift_samples: list[tuple[int, float]] = []
    for parsed in window_parsed:
        pose =  _object_pose_target(parsed["raw"])
        if pose is None:
            continue
        lift_samples.append((int(parsed["frame_index"]), pose[0][2]))
    max_z = max((value for _, value in lift_samples), default=initial[2])
    lift = max_z - initial[2]
    pre_start_velocity = ctx["pre_start_velocity"]
    lift_passed = lift >= thresholds["object_lift_m"] and _distance(pre_start_velocity, [0.0, 0.0, 0.0]) <= 1.0e-3
    lift_check = _mk_check(
        "lift",
        lift_passed,
        metrics={"lift_m": lift, "initial_z_m": initial[2], "max_object_z_m": max_z},
        reasons=() if lift_passed else ("object lift below threshold or pre-start not at rest",),
        frames=[int(p["frame_index"]) for p in window_parsed],
    )

    # 3. transport combined with release-in-region (m3 direction guard):
    #    the object must move >= object_translation_m AND its final horizontal
    #    position must be strictly closer to the region center than the initial
    #    pose (a 0.20 m move away from the region cannot satisfy transport).
    translations: list[float] = []
    final_cube: list[float] = initial
    for parsed in window_parsed:
        pose =  _object_pose_target(parsed["raw"])
        if pose is None:
            continue
        translations.append(_distance(pose[0], initial))
        final_cube = pose[0]
    max_translation = max(translations, default=0.0)
    region_xy = region[:2]
    initial_radial = _distance(initial[:2], region_xy)
    final_radial = _distance(final_cube[:2], region_xy)
    direction_ok = final_radial < initial_radial - tolerance
    transport_passed = (
        max_translation >= thresholds["object_translation_m"]
        and direction_ok
    )
    reasons = []
    if max_translation < thresholds["object_translation_m"]:
        reasons.append("object translation below threshold")
    if not direction_ok:
        reasons.append("object path does not head toward the place region")
    transport_check = _mk_check(
        "transport",
        transport_passed,
        metrics={"object_translation_m": max_translation,
                 "initial_xyz": initial,
                 "final_radial_m": final_radial,
                 "initial_radial_m": initial_radial,
                 "toward_region": direction_ok},
        reasons=reasons,
    )

    # 4. bounded_tcp_object_drift during the transported (bilateral) phase.
    drift_check = _bounded_drift_check(ctx, bilateral_frames, contact_threshold)

    # 5. release_in_place_region in [scene-detach, released-settled].
    release_sub = _subwindow(ctx, "scene-detach", "released-settled")
    release_parsed = [ctx["parsed_by_frame"][int(r["frame_index"])] for r in release_sub]
    release_contact_present = False
    region_z = region[2]
    region_xy = region[:2]
    final_radial = _distance(final_cube[:2], region_xy)
    final_z_error = abs(final_cube[2] - region_z)
    for parsed in release_parsed:
        if _finger_cube_contacts(parsed["raw"], contact_threshold):
            release_contact_present = True
            break
    release_passed = (
        not release_contact_present
        and final_radial <= thresholds["placement_region_radius_m"]
        and final_z_error <= thresholds["placement_region_z_tolerance_m"]
        and bool(release_parsed)
    )
    release_check = _mk_check(
        "release_in_place_region",
        release_passed,
        metrics={
            "final_radial_m": final_radial,
            "final_z_error_m": final_z_error,
            "release_contact_present": release_contact_present,
        },
        reasons=() if release_passed else ("release did not occur in the place region",),
        frames=[int(p["frame_index"]) for p in release_parsed],
    )

    # 6. settled_speed in the release subwindow.
    max_speed = 0.0
    for parsed in release_parsed:
        pose =  _object_pose_target(parsed["raw"])
        if pose is None:
            continue
        max_speed = max(max_speed, _distance(pose[2], [0.0, 0.0, 0.0]))
    settled_passed = bool(release_parsed) and max_speed <= thresholds["settled_speed_m_s"]
    settled_check = _mk_check(
        "settled_speed",
        settled_passed,
        metrics={"max_object_speed_m_s": max_speed},
        reasons=() if settled_passed else ("object not settled after release",),
        frames=[int(p["frame_index"]) for p in release_parsed],
    )

    # 7. no_arm_obstacle_contact (M2: obstacle set excludes the grasped target).
    obstacle_contacts: list[int] = []
    for parsed in window_parsed:
        if _arm_obstacle_contacts(parsed["raw"], contact_threshold, ctx["obstacle_bodies"]):
            obstacle_contacts.append(int(parsed["frame_index"]))
    obstacle_passed = not obstacle_contacts
    obstacle_check = _mk_check(
        "no_arm_obstacle_contact",
        obstacle_passed,
        metrics={"arm_obstacle_contact_frames": obstacle_contacts},
        reasons=() if obstacle_passed else ("arm made obstacle contact during E positive",),
        frames=obstacle_contacts,
    )

    # 8. no safety stop.
    any_safety = any(_safety(parsed["raw"]) for parsed in window_parsed)
    safety_check = _mk_check(
        "no_safety_stop",
        not any_safety,
        metrics={"safety_stop_frames": sum(1 for p in window_parsed if _safety(p["raw"]))},
        reasons=() if not any_safety else ("safety stop observed during E positive",),
    )
    return [
        bilateral_check,
        lift_check,
        transport_check,
        drift_check,
        release_check,
        settled_check,
        obstacle_check,
        safety_check,
    ]


def _bounded_drift_check(
    ctx: Mapping[str, Any],
    bilateral_frames: Sequence[int],
    contact_threshold: float,
) -> dict[str, Any]:
    """TCP-relative object drift during the bilaterally-held phase."""
    thresholds = ctx["thresholds"]
    bilateral_indices = set(bilateral_frames)
    reference: tuple[list[float], list[float]] | None = None
    max_drift_m = 0.0
    max_drift_deg = 0.0
    used: list[int] = []
    for parsed in ctx["window_parsed"]:
        frame_index = int(parsed["frame_index"])
        if frame_index not in bilateral_indices:
            continue
        pose =  _object_pose_target(parsed["raw"])
        if pose is None:
            continue
        position, orientation, _ = pose
        tcp_pose = _robot(parsed["raw"]).get("tcp_pose")
        tcp_position = _pose_position(tcp_pose, "tcp_pose")
        tcp_orientation = _pose_quaternion(tcp_pose, "tcp_pose")
        if reference is None:
            reference = (
                _q_rotate(_q_inverse(tcp_orientation), [a - b for a, b in zip(position, tcp_position)]),
                _q_mul(_q_inverse(tcp_orientation), orientation),
            )
            used.append(frame_index)
            continue
        relative_position = _q_rotate(_q_inverse(tcp_orientation), [a - b for a, b in zip(position, tcp_position)])
        relative_orientation = _q_mul(_q_inverse(tcp_orientation), orientation)
        dot = abs(sum(a * b for a, b in zip(relative_orientation, reference[1])))
        max_drift_m = max(max_drift_m, _distance(relative_position, reference[0]))
        max_drift_deg = max(max_drift_deg, math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot)))))
        used.append(frame_index)
    passed = (
        bool(used)
        and max_drift_m <= thresholds["object_drift_m"]
        and max_drift_deg <= thresholds["object_drift_deg"]
    )
    return _mk_check(
        "bounded_tcp_object_drift",
        passed,
        metrics={"max_drift_m": max_drift_m, "max_drift_deg": max_drift_deg},
        reasons=() if passed else ("TCP-relative object drift exceeded thresholds",),
        frames=used,
    )


# --- Gate E negative ------------------------------------------------------ #
def _gate_e_negative_checks(ctx: Mapping[str, Any]) -> list[dict[str, Any]]:
    scenario_id = ctx["scenario_id"]
    expected_negative = ctx["integrated"].get("expected_negative")
    if not isinstance(expected_negative, Mapping):
        raise EvidenceError(
            f"{scenario_id} is an E negative but has no integrated.expected_negative"
        )
    checks: list[dict[str, Any]] = []
    for token in expected_negative.get("required", []):
        checks.append(_required_negative_predicate(str(token), ctx))
    for token in expected_negative.get("forbidden", []):
        checks.append(_forbidden_negative_predicate(str(token), ctx))
    return checks


def _required_negative_predicate(token: str, ctx: Mapping[str, Any]) -> dict[str, Any]:
    thresholds = ctx["thresholds"]
    contact_threshold = thresholds["contact_force_n"]
    tolerance = thresholds["numeric_tolerance"]
    window_parsed = ctx["window_parsed"]
    keys = ctx["event_keys"]
    initial = ctx["initial"]
    summary = ctx["execution_summary"]

    if token == "pick_terminal_non_success":
        status = summary.get("task_result_status")
        if status is None:
            status = summary.get("execute_result_status")
        if status is None:
            return _mk_check(token, False, metrics={"task_result_status": None},
                             reasons=("no pick terminal status recorded",))
        code = _as_int(status, "pick_terminal_non_success.status")
        passed = code != PICK_PLACE_RESULT_SUCCESS
        return _mk_check(token, passed, metrics={"task_result_status": status},
                         reasons=() if passed else ("pick terminal was success",))
    if token == "contact_absent":
        frames = [
            int(p["frame_index"])
            for p in window_parsed
            if _finger_cube_contacts(p["raw"], contact_threshold)
            or _arm_obstacle_contacts(p["raw"], contact_threshold, ctx["obstacle_bodies"])
        ]
        passed = not frames
        return _mk_check(token, passed, metrics={"contact_frames": frames},
                         reasons=() if passed else ("unexpected contact in window",), frames=frames)
    if token == "scene_attach_absent":
        attached = any(
            TASK_TARGET_ID in set(record.get("attached_ids", []))
            for record in ctx["journal_records"]
        )
        passed = not attached
        return _mk_check(token, passed, metrics={"attached": attached},
                         reasons=() if passed else ("scene-attach observed",))
    if token == "lift_m_lt:0.02":
        max_z = initial[2]
        for p in window_parsed:
            pose =  _object_pose_target(p["raw"])
            if pose is not None:
                max_z = max(max_z, pose[0][2])
        lift = max_z - initial[2]
        passed = lift < 0.02
        return _mk_check(token, passed, metrics={"lift_m": lift},
                         reasons=() if passed else ("object lifted more than 0.02 m",))
    if token == "approach_tcp_delta_lt:0.02":
        first_tcp = _pose_position(_robot(window_parsed[0]["raw"]).get("tcp_pose"), "tcp_pose")
        max_delta = 0.0
        for p in window_parsed:
            tcp = _pose_position(_robot(p["raw"]).get("tcp_pose"), "tcp_pose")
            max_delta = max(max_delta, _distance(tcp, first_tcp))
        passed = max_delta < 0.02
        return _mk_check(token, passed, metrics={"max_tcp_delta_m": max_delta},
                         reasons=() if passed else ("approach moved TCP more than 0.02 m",))
    if token == "goal_rejected_pre_send":
        pick_sent = _as_bool(summary.get("pick_goal_sent")) is True
        reasons = []
        if pick_sent:
            reasons.append("pick goal was sent")
        raw_motion = any(
            _distance(_pose_position(_robot(p["raw"]).get("tcp_pose"), "tcp_pose"),
                      _pose_position(_robot(window_parsed[0]["raw"]).get("tcp_pose"), "tcp_pose"))
            > tolerance
            for p in window_parsed
        )
        contact = any(
            _finger_cube_contacts(p["raw"], contact_threshold)
            or _arm_obstacle_contacts(p["raw"], contact_threshold, ctx["obstacle_bodies"])
            for p in window_parsed
        )
        if raw_motion:
            reasons.append("raw TCP motion observed")
        if contact:
            reasons.append("raw contact observed")
        passed = not reasons
        return _mk_check(token, passed, metrics={"pick_goal_sent": pick_sent},
                         reasons=reasons)
    if token == "no_planning_scene_mutation":
        events = [str(record.get("event", "")) for record in ctx["journal_records"]]
        passed = tuple(events) == ("fixture-ready", "teardown")
        return _mk_check(token, passed, metrics={"events": events},
                         reasons=() if passed else ("planning-scene mutated",))
    if token == "cancel_trigger_after_approach_start":
        approach = keys.get("approach-start")
        cancel = keys.get("cancel-requested")
        passed = approach is not None and cancel is not None and cancel[0] > approach[0]
        return _mk_check(token, passed, metrics={"approach_start": approach, "cancel": cancel},
                         reasons=() if passed else ("cancel did not follow approach-start",))
    if token == "release_absent":
        detach = any(
            str(record.get("event", "")) == "scene-detach"
            for record in ctx["journal_records"]
        )
        passed = not detach
        return _mk_check(token, passed, metrics={"scene_detach": detach},
                         reasons=() if passed else ("release/detach observed",))
    if token == "cancel_trigger_after_lift":
        lift = keys.get("lift-complete")
        cancel = keys.get("cancel-requested")
        passed = lift is not None and cancel is not None and cancel[0] > lift[0]
        return _mk_check(token, passed, metrics={"lift_complete": lift, "cancel": cancel},
                         reasons=() if passed else ("cancel did not follow lift-complete",))
    if token == "contact_present_before_cancel":
        cancel_key = keys.get("cancel-requested")
        if cancel_key is None:
            return _mk_check(token, False, metrics={}, reasons=("no cancel-requested key",))
        found = False
        for p in ctx["all_parsed"]:
            if p["timestamp"] > cancel_key[1]:
                break
            if _bilateral_strict(p["raw"], contact_threshold) == (True, True):
                found = True
                break
        return _mk_check(token, found, metrics={"contact_before_cancel": found},
                         reasons=() if found else ("no bilateral contact before cancel",))
    if token == "scene_attached_before_cancel":
        attach_key = keys.get("scene-attach")
        cancel_key = keys.get("cancel-requested")
        passed = attach_key is not None and cancel_key is not None and attach_key[0] < cancel_key[0]
        return _mk_check(token, passed, metrics={"scene_attach": attach_key, "cancel": cancel_key},
                         reasons=() if passed else ("scene-attach did not precede cancel",))
    if token == "no_post_cancel_stage":
        # F1.7/F2.1: the check reads only the [cancel-requested, quiescent]
        # subwindow.  Journal events after cancel-requested are scanned for
        # later-stage leaks; terminal quiescence is proven from the bounded tail
        # ending at quiescent, so the arm's real deceleration ramp inside the
        # window is allowed.  A NEW command target/goal in the subwindow fails
        # closed even if the velocity later settles.  Motion after quiescent
        # (post-terminal drain) is never admitted.
        cancel_key = keys.get("cancel-requested")
        forbidden_after = ("place-goal-accepted", "scene-detach", "released-settled")
        later = [
            event
            for event in forbidden_after
            if event in keys and cancel_key is not None and keys[event][0] > cancel_key[0]
        ]
        sub = _subwindow(ctx, "cancel-requested", "quiescent")
        sub_parsed = [ctx["parsed_by_frame"][_as_index(r["frame_index"], "subwindow.frame_index")] for r in sub]
        sub_target = _max_target_delta(sub_parsed)
        tail_parsed, tail_speed, tail_target = _terminal_quiescence(ctx, end_event="quiescent")
        tail_ok = (
            tail_speed <= thresholds["safety_stop_velocity_rad_s"]
            and sub_target <= tolerance
        )
        passed = not later and tail_ok
        return _mk_check(token, passed, metrics={"later_events": later,
                                                 "tail_joint_speed_rad_s": tail_speed,
                                                 "subwindow_target_delta_rad": sub_target},
                         reasons=() if passed else ("post-cancel stage or non-quiescent cancel tail observed",))
    if token == "safety_observed_during_transport":
        transport_key = keys.get("transport")
        clear_key = keys.get("operator-clear")
        found = False
        for p in ctx["all_parsed"]:
            ts = p["timestamp"]
            if transport_key is not None and ts < transport_key[1]:
                continue
            if clear_key is not None and ts > clear_key[1]:
                break
            if _safety(p["raw"]):
                found = True
                break
        return _mk_check(token, found, metrics={"safety_during_transport": found},
                         reasons=() if found else ("no safety stop during transport",))
    if token == "controller_terminal_non_success":
        passed = _terminal_non_success(summary)
        return _mk_check(token, passed, metrics={"terminal_status": summary.get("terminal_status")},
                         reasons=() if passed else ("controller terminal was success",))
    if token == "velocity_below_stop_limit":
        effective_key = keys.get("effective-stop")
        if effective_key is None:
            return _mk_check(token, False, metrics={}, reasons=("no effective-stop key",))
        deadline = int(effective_key[0]) + int(thresholds["safety_stop_frames"])
        max_speed = 0.0
        for p in ctx["all_parsed"]:
            if int(p["frame_index"]) < int(effective_key[0]):
                continue
            if int(p["frame_index"]) > deadline:
                break
            _, _, velocities, _ = _arm_joint_data(p["raw"])
            max_speed = max(max_speed, max(abs(value) for value in velocities))
        passed = max_speed <= thresholds["safety_stop_velocity_rad_s"]
        return _mk_check(token, passed, metrics={"max_joint_speed_rad_s": max_speed},
                         reasons=() if passed else ("velocity above stop limit",))
    if token == "no_post_clear_resume":
        # F1.7/F2.1 item 5: the check reads only the [operator-clear, quiescent]
        # subwindow.  Motion between clear and quiescence is only a candidate
        # resume when it is a NEW target/goal — the pre-existing command's
        # deceleration is not a resume.  Require no target change and terminal
        # quiescence at the tail; motion after quiescent is never admitted.
        sub = _subwindow(ctx, "operator-clear", "quiescent")
        if not sub:
            return _mk_check(token, False, metrics={}, reasons=("no operator-clear/quiescent keys",))
        sub_parsed = [ctx["parsed_by_frame"][_as_index(r["frame_index"], "subwindow.frame_index")] for r in sub]
        target_change = _max_target_delta(sub_parsed)
        tail_parsed, tail_speed, tail_target = _terminal_quiescence(ctx, end_event="quiescent")
        tail_ok = (
            tail_speed <= thresholds["safety_stop_velocity_rad_s"]
            and tail_target <= tolerance
        )
        passed = target_change <= tolerance and tail_ok
        return _mk_check(token, passed, metrics={"post_clear_target_delta_rad": target_change,
                                                 "tail_joint_speed_rad_s": tail_speed,
                                                 "tail_target_delta_rad": tail_target},
                         reasons=() if passed else ("new target or non-quiescent tail after operator-clear",))
    if token == "pick_physical_retained":
        sub = _subwindow(ctx, "place-goal-accepted", "quiescent")
        sub_parsed = [ctx["parsed_by_frame"][int(r["frame_index"])] for r in sub]
        held = any(
            _bilateral_strict(p["raw"], contact_threshold) == (True, True)
            for p in sub_parsed
        )
        passed = bool(sub_parsed) and held
        return _mk_check(token, passed, metrics={"held_after_place_failure": held},
                         reasons=() if passed else ("object not retained after place failure",),
                         frames=[int(p["frame_index"]) for p in sub_parsed])
    if token == "place_terminal_non_success":
        status = summary.get("task_result_status")
        if status is None:
            status = summary.get("execute_result_status")
        if status is None:
            return _mk_check(token, False, metrics={"task_result_status": None},
                             reasons=("no place terminal status recorded",))
        code = _as_int(status, "place_terminal_non_success.status")
        passed = code != PICK_PLACE_RESULT_SUCCESS
        return _mk_check(token, passed, metrics={"task_result_status": status},
                         reasons=() if passed else ("place terminal was success",))
    if token == "scene_attached_after_place_failure":
        place_key = keys.get("place-goal-accepted")
        quiescent_key = keys.get("quiescent")
        if place_key is None:
            return _mk_check(token, False, metrics={},
                             reasons=("no place-goal-accepted key",))
        if quiescent_key is None:
            return _mk_check(token, False, metrics={},
                             reasons=("no quiescent key",))
        # F1.8: the target must remain attached in the journal records after the
        # place failure through quiescent — not merely that a scene-attach ever
        # occurred.  Records before the place-failure terminal are ignored.
        attached_after = [
            record.get("event")
            for record in ctx["journal_records"]
            if _as_index(record["frame_index"], "journal.frame_index") >= place_key[0]
            and _as_index(record["frame_index"], "journal.frame_index") <= quiescent_key[0]
            and TASK_TARGET_ID in set(record.get("attached_ids", []))
        ]
        passed = bool(attached_after)
        return _mk_check(
            token,
            passed,
            metrics={"attached_after_place_failure": attached_after},
            reasons=() if passed else ("target not attached through quiescent after place failure",),
        )
    raise EvidenceError(f"unknown required negative predicate {token!r}")


def _forbidden_negative_predicate(token: str, ctx: Mapping[str, Any]) -> dict[str, Any]:
    thresholds = ctx["thresholds"]
    contact_threshold = thresholds["contact_force_n"]
    tolerance = thresholds["numeric_tolerance"]
    window_parsed = ctx["window_parsed"]
    keys = ctx["event_keys"]
    initial = ctx["initial"]

    if token in ("gripper_close", "gripper_open"):
        travel = _gripper_travel(window_parsed)
        passed = travel <= tolerance
        return _mk_check(token, passed, metrics={"drive_joint_travel_rad": travel},
                         reasons=() if passed else (f"forbidden {token} observed",))
    if token in ("scene_attach", "scene_detach"):
        present = any(
            str(record.get("event", "")) == token
            for record in ctx["journal_records"]
        )
        return _mk_check(token, not present, metrics={"event_present": present},
                         reasons=() if not present else (f"forbidden {token} observed",))
    if token in ("lift", "lift_complete", "release"):
        max_z = initial[2]
        for p in window_parsed:
            pose =  _object_pose_target(p["raw"])
            if pose is not None:
                max_z = max(max_z, pose[0][2])
        lift = max_z - initial[2]
        passed = lift < 0.02
        return _mk_check(token, passed, metrics={"lift_m": lift},
                         reasons=() if passed else (f"forbidden {token} observed",))
    if token == "contact":
        frames = [
            int(p["frame_index"])
            for p in window_parsed
            if _finger_cube_contacts(p["raw"], contact_threshold)
            or _arm_obstacle_contacts(p["raw"], contact_threshold, ctx["obstacle_bodies"])
        ]
        return _mk_check(token, not frames, metrics={"contact_frames": frames},
                         reasons=() if not frames else ("forbidden contact observed",), frames=frames)
    if token in ("place_goal_sent", "pick_goal_sent", "move_group_goal_sent", "new_goal_after_clear"):
        summary = ctx["execution_summary"]
        value = _as_bool(summary.get(token)) is True
        return _mk_check(token, not value, metrics={"observed": value},
                         reasons=() if not value else (f"forbidden {token} observed",))
    if token == "post_clear_resume":
        # F1.7/F2.1 item 5: reads only the [operator-clear, quiescent] subwindow.
        # A candidate resume is a NEW target/goal between clear and quiescence,
        # not the pre-existing command's deceleration.  Also require terminal
        # quiescence at the tail.
        sub = _subwindow(ctx, "operator-clear", "quiescent")
        target_change = 0.0
        if sub:
            sub_parsed = [ctx["parsed_by_frame"][_as_index(r["frame_index"], "subwindow.frame_index")] for r in sub]
            target_change = _max_target_delta(sub_parsed)
        tail_parsed, tail_speed, tail_target = _terminal_quiescence(ctx, end_event="quiescent")
        tail_ok = (
            tail_speed <= thresholds["safety_stop_velocity_rad_s"]
            and tail_target <= tolerance
        )
        resume = target_change > tolerance
        passed = not resume and tail_ok
        return _mk_check(token, passed, metrics={"post_clear_target_delta_rad": target_change,
                                                 "tail_joint_speed_rad_s": tail_speed},
                         reasons=() if passed else ("forbidden post-clear resume (new target) observed",))
    if token == "target_region_settled":
        region = ctx["region_center"]
        final_radial = 1.0e9
        for p in window_parsed:
            pose =  _object_pose_target(p["raw"])
            if pose is not None:
                final_radial = _distance(pose[0][:2], region[:2])
        passed = final_radial > thresholds["placement_region_radius_m"]
        return _mk_check(token, passed, metrics={"final_radial_m": final_radial},
                         reasons=() if passed else ("object settled at target region",))
    raise EvidenceError(f"unknown forbidden negative predicate {token!r}")


# --------------------------------------------------------------------------- #
# Orchestration
# --------------------------------------------------------------------------- #
def _base_verdict(
    attempt_id: str,
    scenario_id: str,
    stage: str,
    polarity: str,
) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "attempt_id": attempt_id,
        "gate": scenario_id,
        "stage": stage,
        "polarity": polarity,
        "status": "evidence-invalid",
        "pass": False,
        "verified": False,
        "authority": "physics_truth.jsonl",
        "action_results_diagnostic_only": True,
        "checks": [],
        "metrics": {},
        "errors": [],
        "execution_sources": ["integrated-execution.jsonl", "integrated-execution.json"],
    }


def _gate_b_status(
    attempt_dir: Path,
    attempt_id: str,
    scenario_id: str,
    stage: str,
    polarity: str,
) -> dict[str, Any] | None:
    marker = attempt_dir / "gate-b-status.json"
    if not marker.is_file():
        return None
    base = _base_verdict(attempt_id, scenario_id, stage, polarity)
    try:
        value = _read_json(marker)
    except EvidenceError as error:
        base["errors"].append(f"gate-b-status.json invalid: {error}")
        return base
    try:
        schema_version = _as_int(
            value.get("schema_version", 0), "gate-b-status.json.schema_version"
        )
    except EvidenceError as error:
        base["errors"].append(str(error))
        return base
    if schema_version != 1:
        base["errors"].append("gate-b-status.json schema_version must be 1")
        return base
    if str(value.get("status")) == "blocked":
        base["status"] = "blocked-by-gate-b"
        base["errors"] = ["Gate B blocked this attempt"]
        return base
    base["errors"].append(
        f"gate-b-status.json status {value.get('status')!r} is not 'blocked'"
    )
    return base


def _threshold_map(config: Mapping[str, Any]) -> dict[str, float]:
    values = config.get("thresholds", {})
    result: dict[str, float] = {}
    if isinstance(values, Mapping):
        for key, value in values.items():
            result[str(key)] = _finite(value, f"thresholds.{key}")
    return result


def _invalid_bundle_verdict(
    attempt_dir: Path,
    *,
    attempt_id: str,
    gate: str,
    error: Any,
) -> dict[str, Any]:
    """Atomically write and return an evidence-invalid verdict for a bundle/shape failure.

    F2.4: malformed/missing bundle structures, bool/string/list/null seed,
    missing integrated mapping, and malformed report identities must return and
    write ``evidence-invalid`` (never traceback).  A stable fallback gate/attempt
    identity is used when the scenario identity itself cannot be trusted.
    """
    verdict = _base_verdict(attempt_id, gate, "", "")
    verdict["errors"].append(str(error))
    _atomic_write_json(attempt_dir / VERDICT_FILENAME, verdict)
    return verdict


def verify_integrated_attempt(
    *,
    scenario: Mapping[str, Any],
    attempt_dir: Path,
    config: Mapping[str, Any],
) -> dict[str, Any]:
    """Verify one integrated attempt from raw physics truth.

    ``scenario`` is the complete ``load_test_scenario()`` bundle
    (``{scenario, planning_scene, planning_scene_declaration, integrated,
    report_identities}``).  ``config`` is the committed integrated config
    (``physics.hz`` resolved through its ``core_config``).  Atomically writes
    ``gate-verdict.json`` and returns the verdict mapping.
    """
    attempt_dir = Path(attempt_dir)
    # F2.4: resolve a stable fallback identity BEFORE trusting any scenario
    # field.  Malformed bundle structures fail closed below, never traceback.
    fallback_attempt = attempt_dir.name
    fallback_gate = attempt_dir.name
    try:
        scenario_map = scenario.get("scenario") if isinstance(scenario, Mapping) else None
        if not isinstance(scenario_map, Mapping):
            raise EvidenceError("scenario bundle has no scenario.scenario mapping")
        scenario_id = str(scenario_map.get("id", ""))
        if not scenario_id:
            raise EvidenceError("scenario.scenario.id is missing or empty")
        seed = _as_int(scenario_map.get("seed", None), "scenario.scenario.seed")
        integrated = scenario.get("integrated")
        if not isinstance(integrated, Mapping):
            raise EvidenceError("scenario bundle has no integrated mapping")
    except EvidenceError as error:
        return _invalid_bundle_verdict(
            attempt_dir,
            attempt_id=fallback_attempt,
            gate=fallback_gate,
            error=error,
        )
    stage = str(integrated.get("stage", ""))
    polarity_raw = integrated.get("acceptance", {})
    polarity = str(polarity_raw.get("polarity", "")) if isinstance(polarity_raw, Mapping) else ""

    # F2.4: report_identities, when present, must be consistent (diagnostic
    # only).  A malformed identity (bad seed type, mismatch) fails closed with a
    # durable evidence-invalid verdict, never a traceback.
    try:
        report_identities = scenario.get("report_identities")
        if isinstance(report_identities, Mapping):
            if str(report_identities.get("scenario_id", "")) != scenario_id:
                raise EvidenceError("report_identities.scenario_id does not match scenario id")
            identity_seed = _as_int(
                report_identities.get("seed", None), "report_identities.seed"
            )
            if identity_seed != seed:
                raise EvidenceError("report_identities.seed does not match scenario seed")
    except EvidenceError as error:
        return _invalid_bundle_verdict(
            attempt_dir,
            attempt_id=attempt_dir.name,
            gate=scenario_id,
            error=error,
        )

    manifest_path = attempt_dir / "manifest.json"
    manifest_present = manifest_path.is_file()
    if manifest_present:
        manifest = _read_json(manifest_path)
        attempt_id = str(manifest.get("attempt_id") or attempt_dir.name)
    else:
        attempt_id = attempt_dir.name

    blocked = _gate_b_status(attempt_dir, attempt_id, scenario_id, stage, polarity)
    if blocked is not None:
        _atomic_write_json(attempt_dir / VERDICT_FILENAME, blocked)
        return blocked

    verdict = _base_verdict(attempt_id, scenario_id, stage, polarity)
    try:
        physics_hz = _resolve_physics_hz(config)
        thresholds = _threshold_map(config)
        core_config_path = Path(config["core_config"])
        if not core_config_path.is_absolute():
            core_config_path = _repo_root() / core_config_path
        core_config = _read_json(core_config_path)
        core_thresholds = _threshold_map(core_config)

        raw_records = _read_jsonl(attempt_dir / "physics_truth.jsonl", required=True)
        evaluator_records = _read_jsonl(attempt_dir / "evaluator.jsonl", required=True)
        execution_rows = _read_jsonl(attempt_dir / "integrated-execution.jsonl", required=True)
        execution_summary = _read_json(attempt_dir / "integrated-execution.json")
        moveit_plans = _read_jsonl(attempt_dir / "moveit-plans.jsonl", required=True)
        controller_rows = _read_jsonl(attempt_dir / "controller-results.jsonl", required=True)
        journal_records = _read_jsonl(attempt_dir / "planning-scene.jsonl", required=True)

        # gate-window.json slicing indices (full production schema, M4).
        # F1.5: raw_start_index/evaluator_start_index are validated as
        # non-boolean non-negative integers; malformed values fail closed.
        gate_window_path = attempt_dir / "gate-window.json"
        raw_start_index = 0
        evaluator_start_index: int | None = None
        if gate_window_path.is_file():
            gate_window = _read_json(gate_window_path)
            if str(gate_window.get("gate", "")) != scenario_id:
                raise EvidenceError("gate-window.json gate does not match scenario id")
            if str(gate_window.get("attempt_id", "")) != attempt_id:
                raise EvidenceError("gate-window.json attempt_id does not match")
            if "raw_start_index" in gate_window:
                raw_start_index = _as_index(
                    gate_window["raw_start_index"], "gate-window.raw_start_index"
                )
            if "evaluator_start_index" in gate_window:
                evaluator_start_index = _as_index(
                    gate_window["evaluator_start_index"], "gate-window.evaluator_start_index"
                )

        # Raw/evaluator exact correlation (distinct drain-mismatch reason code).
        _raw_evaluator_correlation(
            raw_records,
            evaluator_records,
            raw_start_index=raw_start_index,
            evaluator_start_index=evaluator_start_index,
        )

        # Scenario/seed match on every raw frame.  F1.5: every raw seed is a
        # non-boolean integer, validated exactly, never coerced.
        for index, raw in enumerate(raw_records):
            if str(raw.get("scenario", "")) != scenario_id:
                raise EvidenceError(f"physics_truth[{index}] scenario does not match the attempt")
            raw_seed = _as_int(raw.get("seed", None), f"physics_truth[{index}].seed")
            if raw_seed != seed:
                raise EvidenceError(f"physics_truth[{index}] seed does not match the attempt")

        # Expected objects are not measured truth: every measured object must be
        # declared in expected_objects and vice versa (per-frame).
        for index, raw in enumerate(raw_records):
            expected_objects = raw.get("expected_objects")
            if not isinstance(expected_objects, Mapping):
                raise EvidenceError(
                    f"physics_truth[{index}] expected_objects must be a mapping"
                )
            measured_ids = {
                str(obj.get("id", obj.get("object_id", "")))
                for obj in _objects(raw)
            }
            if measured_ids != set(expected_objects.keys()):
                raise EvidenceError(
                    f"physics_truth[{index}] measured objects do not match "
                    "expected_objects (expected objects are not measured truth)"
                )

        # Derive sim-time boundaries from the journal (B1, Table 1).
        kind = _scenario_kind(scenario_id, stage, integrated)
        gate_start, gate_end = _derive_boundaries(journal_records, stage, kind)

        # Authoritative window selected BEFORE strict contiguity parsing so a
        # pre-window warmup epoch (frame-index reset before fixture-ready) is
        # never misread as a gap inside the selected attempt window.
        window, window_path = select_integrated_gate_window(
            raw_records,
            attempt_dir,
            scenario_id,
            attempt_id=attempt_id,
            manifest_present=manifest_present,
            gate_start=gate_start,
            gate_end=gate_end,
            physics_hz=physics_hz,
        )
        if window_path is None:
            raise EvidenceError("attempt has no gate-window.json window")

        # Parse truth (finite, contiguous, monotonic, gateway-error scan) over
        # the authoritative window only.  A gap strictly inside the selected
        # window still fails contiguity.
        parsed = _parse_truth(
            [dict(record) for record in window], physics_hz=physics_hz
        )
        parsed_by_frame = {int(item["frame_index"]): item for item in parsed}
        window_parsed = [parsed_by_frame[int(r["frame_index"])] for r in window]

        # Revision match from the journal fixture_revision.
        declaration = scenario.get("planning_scene_declaration")
        if not isinstance(declaration, Mapping):
            raise EvidenceError("scenario bundle has no planning_scene_declaration")
        revision = str(declaration.get("revision", ""))
        for record in journal_records:
            if str(record.get("fixture_revision", "")) != revision:
                raise EvidenceError("journal fixture_revision does not match the scenario revision")

        # Endpoint allowlist (B3, C4).
        forbidden_endpoints = integrated.get("forbidden_endpoints", [])
        forbidden_tokens = (
            config.get("execution_contract", {}).get("forbidden_tokens", [])
            if isinstance(config.get("execution_contract"), Mapping)
            else []
        )
        endpoint_records = list(execution_rows) + [execution_summary] + list(controller_rows) + list(moveit_plans)
        goals_dir = attempt_dir / "goals"
        if goals_dir.is_dir():
            for goal_path in sorted(goals_dir.glob("*.json")):
                endpoint_records.append(_read_json(goal_path))
        endpoint_reasons = _integrated_endpoint_validator(
            endpoint_records,
            forbidden_endpoints=forbidden_endpoints,
            forbidden_tokens=forbidden_tokens,
        )
        if endpoint_reasons:
            raise EvidenceError("; ".join(endpoint_reasons))

        # Execution gateway/command reason_code check (4.8).
        execution_errors = _execution_gateway_errors(execution_rows)
        if execution_errors:
            raise EvidenceError("; ".join(execution_errors))

        # Planning-scene.json final (4.9), optional-by-stage.
        _planning_scene_final(attempt_dir)

        # Journal validation (event order, ownership, phase-aware attachment,
        # key correlation, Task-7 bilateral-before-attach rule).
        obstacle_bodies = _obstacle_body_set(scenario)
        ctx: dict[str, Any] = {
            "scenario_id": scenario_id,
            "stage": stage,
            "kind": kind,
            "polarity": polarity,
            "integrated": integrated,
            "planning_scene_declaration": declaration,
            "window_parsed": window_parsed,
            "window_records": window,
            "all_parsed": parsed,
            "journal_records": journal_records,
            "event_keys": _journal_event_keys(journal_records),
            "execution_rows": execution_rows,
            "execution_summary": execution_summary,
            "controller_rows": controller_rows,
            "moveit_plans": moveit_plans,
            "thresholds": thresholds,
            "core_thresholds": core_thresholds,
            "physics_hz": physics_hz,
            "config": config,
            "parsed_by_frame": parsed_by_frame,
            "raw_by_frame": {int(raw["frame_index"]): raw for raw in raw_records},
            "obstacle_bodies": obstacle_bodies,
        }
        _validate_scene_journal(journal_records, ctx)

        # Pre-start initial cube pose and velocity (F1.1).  The qualification
        # cube exists only for Stage E scenarios; C/D raw truth carries
        # ``objects: []`` / ``object: None`` and never needs the cube.  Only E
        # predicates (positive and negative) consume ``initial``.
        pre_start = window_parsed[0]
        initial_pose = _object_pose_target(pre_start["raw"])
        if stage == "E":
            if initial_pose is None:
                raise EvidenceError("pre-start frame has no qualification_cube object")
            ctx["initial"] = initial_pose[0]
            ctx["pre_start_velocity"] = initial_pose[2]
        else:
            # C/D: no cube is expected; E-only predicates are never dispatched.
            ctx["initial"] = [0.0, 0.0, 0.0]
            ctx["pre_start_velocity"] = [0.0, 0.0, 0.0]
        geometry = config.get("geometry_contract", {})
        region = geometry.get("place_region_center_xyz", [0.85, 0.0, 0.64])
        if not isinstance(region, Sequence) or isinstance(region, (str, bytes)) or len(region) != 3:
            raise EvidenceError("geometry_contract.place_region_center_xyz must be a 3-vector")
        ctx["region_center"] = [_finite(value, "region_center") for value in region]

        # Scenario-JSON == executor-table equivalence (fail closed).
        _assert_scenario_contract_equivalence(ctx)

        # Stage dispatch.
        if stage == "C":
            checks = _gate_c_checks(ctx)
        elif stage == "D":
            checks = _gate_d_checks(ctx)
        elif stage == "E":
            if polarity == "positive":
                checks = _gate_e_positive_checks(ctx)
            else:
                checks = _gate_e_negative_checks(ctx)
        else:
            raise EvidenceError(f"unknown stage {stage!r}")

        verdict["checks"] = checks
        verdict["metrics"] = {
            str(check["name"]): dict(check.get("metrics") or {}) for check in checks
        }
        failed = [check for check in checks if not check["passed"]]
        if failed:
            verdict["status"] = "verified-fail"
            verdict["pass"] = False
            verdict["verified"] = True
            verdict["errors"] = [
                f"{check['name']}: {'; '.join(check.get('reasons') or [])}"
                for check in failed
            ]
        else:
            verdict["status"] = "verified-pass"
            verdict["pass"] = True
            verdict["verified"] = True
            verdict["errors"] = []
    except EvidenceError as error:
        verdict["status"] = "evidence-invalid"
        verdict["pass"] = False
        verdict["verified"] = False
        verdict["errors"].append(str(error))

    _atomic_write_json(attempt_dir / VERDICT_FILENAME, verdict)
    return verdict


def _scenario_kind(scenario_id: str, stage: str, integrated: Mapping[str, Any]) -> str:
    if stage == "C":
        kind = "plan-joint"
        if scenario_id == "qualification-moveit-plan-pose":
            kind = "plan-pose"
        return kind
    if stage == "D":
        return STAGE_D_KIND.get(scenario_id, "execute-joint")
    if stage == "E":
        return STAGE_E_KIND.get(scenario_id, "positive")
    raise EvidenceError(f"unknown stage {stage!r} for {scenario_id}")


def _assert_scenario_contract_equivalence(ctx: Mapping[str, Any]) -> None:
    """Assert scenario-JSON integrated contract == the committed executor tables."""
    scenario_id = ctx["scenario_id"]
    stage = ctx["stage"]
    integrated = ctx["integrated"]
    if stage == "C":
        if scenario_id not in STAGE_C_SCENARIOS:
            raise EvidenceError(f"{scenario_id} is not a declared Stage C scenario")
        return
    if stage == "D":
        if scenario_id not in STAGE_D_SCENARIOS:
            raise EvidenceError(f"{scenario_id} is not a declared Stage D scenario")
        expected_polarity = STAGE_D_EXPECTED_POLARITY.get(scenario_id)
        expected_physical = list(STAGE_D_EXPECTED_PHYSICAL.get(scenario_id, ()))
        actual_polarity = str(integrated.get("acceptance", {}).get("polarity", ""))
        actual_physical = list(integrated.get("expected_physical", ()) or [])
        if actual_polarity != expected_polarity:
            raise EvidenceError(f"{scenario_id} polarity does not match the executor table")
        if actual_physical != expected_physical:
            raise EvidenceError(f"{scenario_id} expected_physical does not match the executor table")
        return
    if stage == "E":
        if scenario_id not in STAGE_E_SCENARIOS:
            raise EvidenceError(f"{scenario_id} is not a declared Stage E scenario")
        expected_polarity = STAGE_E_EXPECTED_POLARITY.get(scenario_id)
        expected_physical = list(STAGE_E_EXPECTED_PHYSICAL.get(scenario_id, ()))
        actual_polarity = str(integrated.get("acceptance", {}).get("polarity", ""))
        actual_physical = list(integrated.get("expected_physical", ()) or [])
        if actual_polarity != expected_polarity:
            raise EvidenceError(f"{scenario_id} polarity does not match the executor table")
        if actual_physical != expected_physical:
            raise EvidenceError(f"{scenario_id} expected_physical does not match the executor table")
        expected_negative = STAGE_E_EXPECTED_NEGATIVE.get(scenario_id)
        actual_negative = integrated.get("expected_negative")
        if expected_negative is not None:
            if not isinstance(actual_negative, Mapping):
                raise EvidenceError(f"{scenario_id} expected_negative does not match the executor table")
            if (
                tuple(actual_negative.get("required", ()) or ())
                != tuple(expected_negative.get("required", ()) or ())
                or tuple(actual_negative.get("forbidden", ()) or ())
                != tuple(expected_negative.get("forbidden", ()) or ())
            ):
                raise EvidenceError(f"{scenario_id} expected_negative does not match the executor table")
        elif actual_negative is not None:
            raise EvidenceError(f"{scenario_id} expected_negative should be None per the executor table")
        return
    raise EvidenceError(f"unknown stage {stage!r}")


def _expected_target_quaternion(ctx: Mapping[str, Any]) -> list[float] | None:
    declaration = ctx.get("planning_scene_declaration") or {}
    target_source_id = declaration.get("target_source_id")
    objects = declaration.get("objects", [])
    if not isinstance(objects, list):
        return None
    for record in objects:
        if isinstance(record, Mapping) and record.get("id") == target_source_id:
            pose = record.get("pose")
            if isinstance(pose, Mapping):
                quat = pose.get("quaternion_xyzw")
                if isinstance(quat, Sequence) and not isinstance(quat, (str, bytes)) and len(quat) == 4:
                    return [_finite(value, "target pose quaternion") for value in quat]
    return None


def _scenario_bundle_from_declaration(
    raw: Mapping[str, Any],
    *,
    expected_id: str | None = None,
) -> dict[str, Any]:
    """Normalize a raw scenario declaration into the canonical bundle shape.

    ``expected_id`` is the scenario id derived from the CLI argument (the
    filename basename for an explicit path, or the bare id).  When provided it
    is compared against the declaration's own ``id`` and a mismatch fails closed
    (F1.9).  The bundle mirrors ``tests/qualification_test_helpers.load_test_scenario``
    so both the fixture path and the production CLI feed
    ``verify_integrated_attempt`` the same shape.
    """
    scenario_id = str(raw.get("id", ""))
    if not scenario_id:
        raise EvidenceError("scenario declaration has no id")
    if expected_id is not None and scenario_id != expected_id:
        raise EvidenceError(
            f"scenario id {scenario_id!r} does not match {expected_id!r}"
        )
    if raw.get("schema_version") != 2:
        raise EvidenceError(f"{scenario_id}: scenario schema_version must be 2")
    seed = _as_int(raw.get("seed", None), f"{scenario_id}.seed")
    declaration = {
        str(key): value for key, value in raw.items() if key not in {"id", "seed"}
    }
    planning_scene_declaration = raw.get("planning_scene")
    integrated = raw.get("integrated")
    if not isinstance(planning_scene_declaration, Mapping):
        raise EvidenceError(f"{scenario_id}: scenario has no planning_scene object")
    if not isinstance(integrated, Mapping):
        raise EvidenceError(f"{scenario_id}: scenario has no integrated object")
    public_planning_scene = {
        "revision": planning_scene_declaration.get("revision"),
        "frame_id": planning_scene_declaration.get("frame_id"),
        "target_source_id": planning_scene_declaration.get("target_source_id"),
        "target_handoff": planning_scene_declaration.get("target_handoff"),
        "objects": planning_scene_declaration.get("objects", []),
        "revision_digest": planning_scene_declaration.get("revision_digest"),
    }
    return {
        "scenario": {"id": scenario_id, "seed": seed, "declaration": declaration},
        "planning_scene": public_planning_scene,
        "planning_scene_declaration": planning_scene_declaration,
        "integrated": integrated,
        "report_identities": None,
    }


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def _main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True, help="scenario id or path to scenario JSON")
    parser.add_argument("--attempt-dir", required=True, help="attempt directory")
    parser.add_argument("--config", required=True, help="integrated config path")
    args = parser.parse_args(list(argv) if argv is not None else None)

    scenario_arg = args.scenario
    attempt_dir = Path(args.attempt_dir)
    config_path = Path(args.config)
    # F1.9/F2.4: scenario/config resolution is part of the fail-closed boundary.
    # A mismatched filename/id (or any EvidenceError during bundle/config
    # normalization) yields a durable evidence-invalid ``gate-verdict.json`` and
    # exit code 2, never a traceback.  A stable fallback gate is used when the
    # scenario identity itself cannot be trusted.
    fallback_gate = attempt_dir.name
    expected_id: str | None = None
    try:
        raw_path = Path(scenario_arg)
        if raw_path.is_file():
            raw = _read_json(raw_path)
            expected_id = raw_path.stem
        else:
            scenario_path = _repo_root() / "simulation" / "scenarios" / f"{scenario_arg}.json"
            raw = _read_json(scenario_path)
            expected_id = scenario_arg
        scenario = _scenario_bundle_from_declaration(raw, expected_id=expected_id)
        config = _read_json(config_path)
    except EvidenceError as error:
        verdict = _base_verdict(attempt_dir.name, expected_id or fallback_gate, "", "")
        verdict["errors"].append(str(error))
        _atomic_write_json(attempt_dir / VERDICT_FILENAME, verdict)
        print(json.dumps(verdict, sort_keys=True, indent=2))
        return 2
    verdict = verify_integrated_attempt(
        scenario=scenario,
        attempt_dir=attempt_dir,
        config=config,
    )
    print(json.dumps(verdict, sort_keys=True, indent=2))
    exit_codes = {
        "verified-pass": 0,
        "verified-fail": 1,
        "evidence-invalid": 2,
        "blocked-by-gate-b": 3,
    }
    return exit_codes.get(str(verdict.get("status")), 2)


if __name__ == "__main__":
    sys.exit(_main())
