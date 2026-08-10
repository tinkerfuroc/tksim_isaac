#!/usr/bin/env python3
"""Offline, physics-truth based manipulation gate verification.

This module intentionally has no ROS dependency.  It consumes the append-only
artifacts of one qualification attempt and writes one atomically replaced
``gate-verdict.json``.  Action results are used only to establish that the
requested endpoint was reached; physical postconditions are recomputed from
raw truth.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

SCHEMA_VERSION = 1
# Float guard for the one-frame pre-start skew window in ``_raw_gate_window``.
# ``gate_start`` and frame timestamps are ``k/physics_hz`` values whose exact
# difference can land a hair outside ``gate_start - 1/physics_hz`` purely from
# floating-point rounding (observed 1e-12 for 1303/120 vs 1304/120).  1e-9 is
# far below any real tick (1/physics_hz ~= 8.3e-3), so it cannot admit a frame
# two ticks back; it only preserves the genuinely-adjacent pre-start frame.
PHYSICS_TOLERANCE_EPS = 1e-9
TRAJECTORY_ACTION = "/xarm7_traj_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/xarm_gripper/gripper_action"
SAFETY_TOPIC = "/sim/safety/operator"
TERMINAL_EXECUTION_EVENTS = {
    "terminal",
    "open-terminal",
    "stable-hold",
    "action_result",
    "gate_finished",
    "executor_finished",
    "executor_complete",
    "completed",
}
GATES = {
    "free-space-fjt",
    "safety-stop",
    "free-gripper",
    "obstructed-gripper",
    "arm-collision",
    "retention",
}
DEFAULT_SCENARIOS: dict[str, tuple[str, ...]] = {
    "free-space-fjt": ("qualification-free-space",),
    "safety-stop": ("qualification-safety-stop", "qualification-free-space"),
    "free-gripper": ("qualification-free-gripper",),
    "obstructed-gripper": ("qualification-obstructed-gripper",),
    "arm-collision": ("qualification-arm-collision",),
    "retention": ("qualification-retention",),
}
DEFAULT_THRESHOLDS: dict[str, float] = {
    "fjt_final_max_error_rad": 0.01,
    "fjt_rms_error_rad": 0.05,
    "safety_stop_velocity_rad_s": 0.02,
    "safety_stop_frames": 5,
    "safety_stop_position_creep_rad": 0.005,
    "retention_lift_m": 0.10,
    "retention_translation_m": 0.20,
    "retention_hold_s": 1.0,
    "retention_drift_m": 0.02,
    "retention_drift_deg": 5.0,
    "stable_speed_m_s": 0.02,
    "free_gripper_min_travel_rad": 0.75,
    "obstructed_gripper_min_gap_rad": 0.02,
    "contact_force_n": 1.0,
    "gripper_target_rad": 0.83,
    "gripper_effort_limit_n": 20.0,
    "numeric_tolerance": 1.0e-6,
}


class EvidenceError(Exception):
    """A malformed or incomplete evidence artifact."""


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise EvidenceError(f"{name} must be numeric, not boolean")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise EvidenceError(f"{name} must be numeric") from error
    if not math.isfinite(result):
        raise EvidenceError(f"{name} must be finite")
    return result


def _validate_finite(value: Any, path: str = "evidence") -> None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            _validate_finite(child, f"{path}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            _validate_finite(child, f"{path}[{index}]")
    elif isinstance(value, float):
        if not math.isfinite(value):
            raise EvidenceError(f"{path} must be finite")


def _parse_constant(value: str) -> Any:
    raise EvidenceError(f"non-finite JSON constant {value}")


def _read_json(path: Path) -> Mapping[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"), parse_constant=_parse_constant)
    except EvidenceError:
        raise
    except (OSError, ValueError, json.JSONDecodeError) as error:
        raise EvidenceError(f"{path.name}: invalid JSON ({error})") from error
    if not isinstance(value, Mapping):
        raise EvidenceError(f"{path.name}: root must be a JSON object")
    _validate_finite(value, path.name)
    return value


def _read_jsonl(path: Path, *, required: bool) -> list[Mapping[str, Any]]:
    if not path.is_file():
        if required:
            raise EvidenceError(f"missing {path.name}")
        return []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise EvidenceError(f"cannot read {path.name}: {error}") from error
    records: list[Mapping[str, Any]] = []
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line, parse_constant=_parse_constant)
        except EvidenceError as error:
            raise EvidenceError(f"{path.name}:{line_number}: {error}") from error
        except (ValueError, json.JSONDecodeError) as error:
            raise EvidenceError(f"{path.name}:{line_number}: invalid JSON ({error})") from error
        if not isinstance(value, Mapping):
            raise EvidenceError(f"{path.name}:{line_number}: record must be a JSON object")
        _validate_finite(value, f"{path.name}:{line_number}")
        records.append(value)
    if required and not records:
        raise EvidenceError(f"{path.name}: contains no records")
    return records


def _as_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return bool(value)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "yes", "1", "accepted", "success", "succeeded", "stalled", "active"}:
            return True
        if normalized in {"false", "no", "0", "failed", "aborted", "canceled", "cancelled", "rejected", "inactive"}:
            return False
    return None


def _walk(value: Any) -> Iterable[Mapping[str, Any]]:
    if isinstance(value, Mapping):
        yield value
        for child in value.values():
            yield from _walk(child)
    elif isinstance(value, list):
        for child in value:
            yield from _walk(child)


def _string(value: Any) -> str:
    return value.strip().lower() if isinstance(value, str) else ""


def _norm_body(value: Any) -> str:
    rendered = str(value).strip().lower().replace("\\", "/")
    return rendered.rsplit("/", 1)[-1]


def _vector(value: Any, name: str, size: int | None = None) -> list[float]:
    if isinstance(value, Mapping):
        keys = ("x", "y", "z", "w")[:size or 3]
        value = [value.get(key) for key in keys]
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise EvidenceError(f"{name} must be an array")
    if size is not None and len(value) != size:
        raise EvidenceError(f"{name} must contain {size} values")
    return [_finite(item, f"{name}[{index}]") for index, item in enumerate(value)]


def _first(mapping: Mapping[str, Any], names: Sequence[str]) -> Any:
    for name in names:
        if name in mapping:
            return mapping[name]
    return None


def _timestamp(value: Mapping[str, Any], name: str) -> float:
    raw = _first(value, ("timestamp", "sim_timestamp", "simulation_time", "sim_time", "stamp"))
    if raw is None:
        raise EvidenceError(f"{name}: missing simulated timestamp")
    if isinstance(raw, Mapping):
        return _finite(raw.get("sec", 0), f"{name}.stamp.sec") + _finite(raw.get("nanosec", 0), f"{name}.stamp.nanosec") * 1e-9
    return _finite(raw, f"{name}.timestamp")


def _pose_position(pose: Any, name: str) -> list[float]:
    if not isinstance(pose, Mapping):
        raise EvidenceError(f"{name} must be an object")
    raw = _first(pose, ("xyz", "position", "translation"))
    return _vector(raw, f"{name}.position", 3)


def _pose_quaternion(pose: Any, name: str) -> list[float]:
    if not isinstance(pose, Mapping):
        raise EvidenceError(f"{name} must be an object")
    raw = _first(pose, ("quaternion_xyzw", "orientation", "quaternion"))
    if raw is None:
        return [0.0, 0.0, 0.0, 1.0]
    result = _vector(raw, f"{name}.orientation", 4)
    norm = math.sqrt(sum(item * item for item in result))
    if norm <= 1e-12:
        raise EvidenceError(f"{name}.orientation must not be zero")
    return [item / norm for item in result]


def _robot(frame: Mapping[str, Any]) -> Mapping[str, Any]:
    value = frame.get("robot")
    if not isinstance(value, Mapping):
        raise EvidenceError("truth frame requires robot object")
    return value


def _joint_data(frame: Mapping[str, Any]) -> tuple[list[str], list[float], list[float], list[float]]:
    robot = _robot(frame)
    names = robot.get("joint_names")
    positions = robot.get("joint_positions")
    velocities = robot.get("joint_velocities")
    efforts = robot.get("joint_efforts")
    if not isinstance(names, Sequence) or isinstance(names, (str, bytes)):
        raise EvidenceError("robot.joint_names must be an array")
    if not isinstance(positions, Sequence) or isinstance(positions, (str, bytes)):
        raise EvidenceError("robot.joint_positions must be an array")
    if not isinstance(velocities, Sequence) or isinstance(velocities, (str, bytes)):
        raise EvidenceError("robot.joint_velocities must be an array")
    if len(names) != len(positions) or len(names) != len(velocities):
        raise EvidenceError("robot joint arrays have inconsistent lengths")
    if efforts is None:
        effort_values: list[float] = []
    elif isinstance(efforts, Sequence) and not isinstance(efforts, (str, bytes)):
        effort_values = [_finite(item, "robot.joint_efforts") for item in efforts]
    else:
        raise EvidenceError("robot.joint_efforts must be an array")
    if effort_values and len(effort_values) != len(names):
        raise EvidenceError("robot joint effort array has inconsistent length")
    return (
        [str(name) for name in names],
        [_finite(item, "robot.joint_positions") for item in positions],
        [_finite(item, "robot.joint_velocities") for item in velocities],
        effort_values,
    )


def _arm_joint_data(
    frame: Mapping[str, Any],
) -> tuple[list[str], list[float], list[float], list[float]]:
    names, positions, velocities, efforts = _joint_data(frame)
    required = [f"joint{index}" for index in range(1, 8)]
    if all(name in names for name in required):
        indices = [names.index(name) for name in required]
        return (
            required,
            [positions[index] for index in indices],
            [velocities[index] for index in indices],
            [efforts[index] for index in indices] if efforts else [],
        )
    if len(names) == 7:
        return names, positions, velocities, efforts
    raise EvidenceError("raw truth does not contain the complete joint1-joint7 arm")


def _safety(frame: Mapping[str, Any]) -> bool:
    robot = frame.get("robot")
    values: list[Any] = []
    if "safety_stop" in frame:
        values.append(frame["safety_stop"])
    if isinstance(robot, Mapping) and "safety_stop" in robot:
        values.append(robot["safety_stop"])
    if not values:
        raise EvidenceError("physics truth frame missing safety_stop field")
    for value in values:
        parsed = _as_bool(value)
        if parsed is True:
            return True
        if parsed is None:
            raise EvidenceError("physics truth safety_stop field is not boolean-like")
    return False


def _objects(frame: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    raw = frame.get("objects")
    if raw is None and frame.get("object") is not None:
        raw = [frame.get("object")]
    if raw is None:
        return []
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)):
        raise EvidenceError("objects must be an array")
    result: list[Mapping[str, Any]] = []
    for index, value in enumerate(raw):
        if not isinstance(value, Mapping):
            raise EvidenceError(f"objects[{index}] must be an object")
        result.append(value)
    return result


def _object(frame: Mapping[str, Any], object_id: str = "qualification_cube") -> Mapping[str, Any] | None:
    for value in _objects(frame):
        if str(value.get("id", value.get("object_id", ""))) == object_id:
            return value
    return None


def _contacts(frame: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    if "contacts" in frame:
        raw = frame["contacts"]
    elif "contact_pairs" in frame:
        raw = frame["contact_pairs"]
    else:
        raise EvidenceError("physics truth frame missing contacts field")
    if raw is None:
        raise EvidenceError("contacts must be an array")
    if raw == [] and "contact_pairs" in frame:
        raw = frame["contact_pairs"]
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)):
        raise EvidenceError("contacts must be an array")
    result: list[Mapping[str, Any]] = []
    for index, value in enumerate(raw):
        if not isinstance(value, Mapping):
            raise EvidenceError(f"contacts[{index}] must be an object")
        force = _first(value, ("normal_force", "force", "normal_force_n"))
        if force is None:
            raise EvidenceError(f"contacts[{index}] missing normal force")
        _finite(force, f"contacts[{index}].normal_force")
        result.append(value)
    return result


def _gateway_errors(frame: Mapping[str, Any]) -> list[str]:
    errors: list[str] = []
    candidates: list[tuple[str, Any]] = []
    for key in ("command_gateway", "gateway", "command_gateway_status", "gateway_status"):
        if key in frame:
            candidates.append((key, frame[key]))
    for key in ("command_gateway_error", "gateway_error", "command_rejected", "command_rejection"):
        if key in frame:
            candidates.append((key, frame[key]))
    for prefix, value in candidates:
        for mapping in _walk(value):
            for key, child in mapping.items():
                normalized = str(key).lower()
                if normalized in {"status", "state"} and _string(child) in {"error", "errored", "rejected", "fault", "faulted"}:
                    errors.append(f"{prefix}.{key}={child!r}")
                    continue
                if not any(token in normalized for token in ("error", "reject", "fault")):
                    continue
                if child not in (None, False, 0, "", [], {}):
                    errors.append(f"{prefix}.{key}={child!r}")
        if not isinstance(value, Mapping) and value not in (None, False, 0, "", [], {}):
            errors.append(f"{prefix}={value!r}")
    return errors


def _parse_truth(
    records: Sequence[Mapping[str, Any]],
    *,
    physics_hz: float | None = None,
    allow_sparse_timestamps: bool = False,
) -> list[dict[str, Any]]:
    parsed: list[dict[str, Any]] = []
    previous: float | None = None
    previous_frame_index: int | None = None
    scenario: str | None = None
    task: str | None = None
    for index, raw in enumerate(records):
        frame_index = raw.get("frame_index")
        if isinstance(frame_index, bool) or not isinstance(frame_index, int) or frame_index < 0:
            raise EvidenceError(f"physics_truth[{index}] requires a non-negative integer frame_index")
        if previous_frame_index is not None and frame_index != previous_frame_index + 1:
            raise EvidenceError(f"physics_truth[{index}] frame_index is not contiguous")
        timestamp = _timestamp(raw, f"physics_truth[{index}]")
        if previous is not None and timestamp <= previous:
            raise EvidenceError(f"physics_truth[{index}] timestamp is not strictly increasing")
        if (
            previous is not None
            and physics_hz is not None
            and not allow_sparse_timestamps
            and timestamp - previous > (1.5 / physics_hz) + 1.0e-9
        ):
            raise EvidenceError(
                f"physics_truth[{index}] timestamp gap exceeds the configured physics frame interval"
            )
        previous = timestamp
        previous_frame_index = frame_index
        current_scenario = str(raw.get("scenario", ""))
        if not current_scenario:
            raise EvidenceError(f"physics_truth[{index}] missing scenario")
        current_task = str(raw.get("task", ""))
        if scenario is not None and current_scenario != scenario:
            raise EvidenceError("physics truth scenario changed during attempt")
        if task is not None and current_task != task:
            raise EvidenceError("physics truth task changed during attempt")
        scenario = current_scenario
        task = current_task
        _joint_data(raw)
        robot = _robot(raw)
        tcp = robot.get("tcp_pose", robot.get("tcp", robot.get("base_pose")))
        if tcp is None:
            raise EvidenceError(f"physics_truth[{index}] missing robot.tcp_pose")
        _pose_position(tcp, f"physics_truth[{index}].robot.tcp_pose")
        _pose_quaternion(tcp, f"physics_truth[{index}].robot.tcp_pose")
        for obj in _objects(raw):
            _pose_position(obj.get("pose"), f"physics_truth[{index}].object.pose")
            twist = obj.get("twist", {})
            if not isinstance(twist, Mapping):
                raise EvidenceError(f"physics_truth[{index}].object.twist must be an object")
            _vector(_first(twist, ("linear", "linear_velocity")) or [0, 0, 0], "object.twist.linear", 3)
        _contacts(raw)
        gateway_errors = _gateway_errors(raw)
        if _safety(raw):
            expected_stop_errors = (
                "command ignored while safety stop is active",
                "command stream expired",
            )
            gateway_errors = [
                error
                for error in gateway_errors
                if not any(expected in error for expected in expected_stop_errors)
            ]
        for error in gateway_errors:
            raise EvidenceError(f"physics_truth[{index}] command gateway error: {error}")
        for key in ("reset", "teleport", "time_reset", "simulation_reset"):
            if _as_bool(raw.get(key)) is True:
                raise EvidenceError(f"physics_truth[{index}] unexplained {key}")
        parsed.append({"index": index, "frame_index": frame_index, "timestamp": timestamp, "raw": raw})
    if not parsed:
        raise EvidenceError("physics truth contains no frames")
    return parsed


def _scenario_options(config: Mapping[str, Any], gate: str) -> tuple[str, ...]:
    for key in ("scenarios", "gate_scenarios", "scenario_by_gate"):
        value = config.get(key)
        if isinstance(value, Mapping) and gate in value:
            selected = value[gate]
            if isinstance(selected, str):
                return (selected,)
            if isinstance(selected, Sequence) and not isinstance(selected, (str, bytes)):
                return tuple(str(item) for item in selected)
    return DEFAULT_SCENARIOS[gate]


def _thresholds(config: Mapping[str, Any]) -> dict[str, float]:
    result = dict(DEFAULT_THRESHOLDS)
    values = config.get("thresholds", {})
    if isinstance(values, Mapping):
        for key, value in values.items():
            result[str(key)] = _finite(value, f"thresholds.{key}")
    return result


def _load_execution(attempt: Path) -> tuple[list[Mapping[str, Any]], list[Path]]:
    records: list[Mapping[str, Any]] = []
    paths: list[Path] = []
    jsonl = attempt / "gate-execution.jsonl"
    if jsonl.is_file():
        records.extend(_read_jsonl(jsonl, required=True))
        paths.append(jsonl)
    summary_names = ("gate-execution.json", "gate-result.json", "gate_results.json", "result.json")
    for name in summary_names:
        path = attempt / name
        if path.is_file():
            records.append(_read_json(path))
            paths.append(path)
            break
    if not records:
        raise EvidenceError("missing gate execution JSONL/final summary")
    return records, paths


def _execution_window(
    records: Sequence[Mapping[str, Any]],
) -> tuple[float | None, float | None]:
    timestamps: list[tuple[str, float]] = []
    gate_starts: list[float] = []
    for record in records:
        raw = _first(
            record,
            (
                "simulated_timestamp",
                "sim_timestamp",
                "simulation_time",
                "sim_time",
            ),
        )
        event = str(record.get("event", ""))
        if raw is None:
            if event == "gate_started":
                raise EvidenceError("gate execution gate_started is missing a simulated timestamp")
            if event in TERMINAL_EXECUTION_EVENTS:
                raise EvidenceError("gate execution terminal boundary is missing a simulated timestamp")
            continue
        value = _finite(raw, "execution simulated timestamp")
        timestamps.append((event, value))
        if event == "gate_started":
            gate_starts.append(value)
    if len(gate_starts) != 1:
        raise EvidenceError(
            "gate execution requires exactly one gate_started simulated timestamp"
        )
    if not timestamps:
        raise EvidenceError("gate execution is missing a terminal simulated timestamp")
    for (_, first), (_, second) in zip(timestamps, timestamps[1:]):
        if second < first:
            raise EvidenceError(
                "gate execution simulated timestamps regressed during the gate"
            )
    gate_start = gate_starts[0]
    gate_end = timestamps[-1][1]
    if gate_end < gate_start:
        raise EvidenceError("gate execution terminal timestamp precedes gate_started")
    if timestamps[-1][0] not in TERMINAL_EXECUTION_EVENTS:
        raise EvidenceError(
            "gate execution final timestamp is not a terminal executor boundary"
        )
    return (
        gate_start,
        gate_end,
    )


def _raw_gate_window(
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
    path = attempt / "gate-window.json"
    if not path.is_file():
        if manifest_present:
            raise EvidenceError("production manifest requires gate-window.json")
        window_path = None
    else:
        window = _read_json(path)
        if "gate" in window and str(window["gate"]) != gate:
            raise EvidenceError("gate-window.json gate identity does not match the requested gate")
        if str(window.get("attempt_id", "")) != attempt_id:
            raise EvidenceError("gate-window.json attempt identity does not match the requested attempt")
        raw_start_index = window.get("raw_start_index")
        if isinstance(raw_start_index, bool) or not isinstance(raw_start_index, int) or raw_start_index < 0:
            raise EvidenceError("gate-window.json requires a non-negative integer raw_start_index")
        if raw_start_index >= len(records):
            raise EvidenceError("gate-window.json raw_start_index is beyond physics truth")
        # The executor's raw_start_index is the count of physics-truth records AT
        # the gate boundary (the first in-gate frame), so slicing [N:] would drop
        # the pre-start frame the integrated verifier requires.  Keep the frame
        # immediately before the boundary; the tolerance filter below removes
        # anything further back, so the shared non-integrated path is unaffected
        # and raw_start_index=0 stays a no-op.
        slice_start = max(0, raw_start_index - 1)
        # The journal's fixture-ready can land ON the boundary frame itself:
        # raw_start_index counts that frame, so the frame at raw_start_index-1
        # has timestamp == gate_start rather than strictly before it.  Back the
        # slice up one more frame so the integrated verifier always retains a
        # genuinely-before frame (timestamp < gate_start).  Monotonic physics
        # timestamps make a single back-up sufficient; the tolerance filter
        # below keeps the extra frame only when it sits within the one-frame
        # sampling skew it was designed to permit.
        if (
            slice_start > 0
            and _timestamp(records[slice_start], "physics truth gate window") >= gate_start
        ):
            slice_start -= 1
        records = records[slice_start:]
        window_path = path
    if not records:
        raise EvidenceError("gate execution raw_start_index leaves no physics truth records")
    tolerance = 1.0 / physics_hz
    # The terminal boundary is authoritative: permit only a one-frame
    # pre-start sampling skew, never post-terminal truth that could satisfy a
    # safety, collision, or retention predicate after the executor completed.
    selected: list[Mapping[str, Any]] = []
    has_exact_overlap = False
    for record in records:
        timestamp = _timestamp(record, "physics truth gate window")
        if gate_start <= timestamp <= gate_end:
            has_exact_overlap = True
        # Epsilon only on the lower bound: it preserves the one-frame pre-start
        # skew frame whose timestamp is marginally below the bound from float
        # rounding.  The upper (terminal) bound stays authoritative -- never
        # admit post-terminal truth that could satisfy a predicate after the
        # executor completed.
        if gate_start - tolerance - PHYSICS_TOLERANCE_EPS <= timestamp <= gate_end:
            selected.append(record)
    if not has_exact_overlap:
        raise EvidenceError("physics truth does not overlap the executor gate window")
    if not selected:
        raise EvidenceError("physics truth gate window selected no records")
    return selected, window_path


def _action_mappings(records: Sequence[Mapping[str, Any]]) -> list[Mapping[str, Any]]:
    result: list[Mapping[str, Any]] = []
    for record in records:
        for mapping in _walk(record):
            keys = {str(key).lower() for key in mapping}
            if keys.intersection({"accepted", "success", "succeeded", "stalled", "reached_goal", "endpoint", "action_endpoint", "action", "action_name", "action_server", "canceled", "cancelled", "aborted"}):
                result.append(mapping)
    return result


def _is_external(records: Sequence[Mapping[str, Any]]) -> bool:
    tokens = ("external", "diagnostic", "executed-unverified", "gate-command", "unverified")
    for record in records:
        for mapping in _walk(record):
            for key, value in mapping.items():
                if str(key).lower() in {"mode", "execution_mode", "source", "kind", "status", "verification", "verified"}:
                    rendered = str(value).lower()
                    if any(token in rendered for token in tokens):
                        return True
                    if str(key).lower() == "verified" and value is False:
                        return True
    return False


def _endpoint(mapping: Mapping[str, Any]) -> str:
    value = _first(mapping, ("endpoint", "action_endpoint", "action_server", "action_name", "action", "server"))
    return str(value) if value is not None else ""


def _status_success(mapping: Mapping[str, Any]) -> bool | None:
    if _as_bool(mapping.get("canceled")) is True or _as_bool(
        mapping.get("cancelled")
    ) is True or _as_bool(mapping.get("aborted")) is True:
        return False
    for key in ("success", "succeeded", "goal_success", "action_success"):
        if key in mapping:
            return _as_bool(mapping[key])
    status = _string(mapping.get("status"))
    if status in {"success", "succeeded", "succeed", "completed", "complete"}:
        return True
    if status in {"failed", "aborted", "cancelled", "canceled", "rejected", "timeout", "timed_out"}:
        return False
    return None


def _latest_value(mappings: Sequence[Mapping[str, Any]], names: Sequence[str]) -> Any:
    value: Any = None
    for mapping in mappings:
        candidate = _first(mapping, names)
        if candidate is not None:
            value = candidate
    return value


def _action_summary(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    mappings = _action_mappings(records)
    endpoints = [_endpoint(mapping) for mapping in mappings if _endpoint(mapping)]
    accepted = any(_as_bool(mapping.get("accepted")) is True for mapping in mappings)
    successes = [_status_success(mapping) for mapping in mappings]
    explicit_success = [value for value in successes if value is not None]
    stalled = _latest_value(mappings, ("stalled", "is_stalled"))
    reached = _latest_value(mappings, ("reached_goal", "reached", "goal_reached"))
    return {
        "mappings": mappings,
        "endpoints": endpoints,
        "accepted": accepted,
        "success": explicit_success[-1] if explicit_success else None,
        "success_values": explicit_success,
        "stalled": _as_bool(stalled),
        "reached_goal": _as_bool(reached),
    }


def _phase_summary(
    records: Sequence[Mapping[str, Any]],
    phase: str,
    *,
    endpoint: str | None = None,
) -> dict[str, Any]:
    selected: list[Mapping[str, Any]] = []
    for mapping in _action_mappings(records):
        if _string(mapping.get("phase")) != phase:
            continue
        if endpoint is not None and _endpoint(mapping) != endpoint:
            continue
        if _string(mapping.get("phase")) == phase:
            selected.append(mapping)
    summary = _action_summary(selected)
    summary["selected"] = selected
    return summary


def _number_values(records: Sequence[Mapping[str, Any]], names: Sequence[str]) -> list[float]:
    values: list[float] = []
    wanted = {name.lower() for name in names}
    for record in records:
        for mapping in _walk(record):
            for key, value in mapping.items():
                if str(key).lower() in wanted and isinstance(value, (int, float)) and not isinstance(value, bool):
                    values.append(_finite(value, f"execution.{key}"))
    return values


def _target_vector(frame: Mapping[str, Any]) -> list[float] | None:
    command_targets = frame.get("command_targets")
    if isinstance(command_targets, Mapping):
        names = command_targets.get("joint_names")
        positions = command_targets.get("joint_positions")
        if (
            isinstance(names, Sequence)
            and not isinstance(names, (str, bytes))
            and isinstance(positions, Sequence)
            and not isinstance(positions, (str, bytes))
        ):
            rendered_names = [str(name) for name in names]
            arm_names = [f"joint{index}" for index in range(1, 8)]
            if all(name in rendered_names for name in arm_names):
                return [
                    _finite(
                        positions[rendered_names.index(name)],
                        f"command_targets.{name}",
                    )
                    for name in arm_names
                ]
    for container in (frame, frame.get("robot", {})):
        if not isinstance(container, Mapping):
            continue
        raw = _first(container, ("commanded_positions", "command_positions", "target_positions", "joint_targets", "active_targets"))
        if raw is not None:
            return _vector(raw, "command target")
        gateway = container.get("command_gateway")
        if isinstance(gateway, Mapping):
            raw = _first(gateway, ("commanded_positions", "target_positions", "active_targets"))
            if raw is not None:
                return _vector(raw, "command gateway target")
    return None


def _required_target_vectors(
    frames: Sequence[Mapping[str, Any]],
    start: int,
    end: int,
    *,
    name: str,
) -> list[list[float]]:
    vectors: list[list[float]] = []
    for item in frames[start : end + 1]:
        target = _target_vector(item["raw"])
        if target is None:
            raise EvidenceError(f"{name} frame {item['index']} is missing command_targets")
        if len(target) != 7:
            raise EvidenceError(f"{name} command_targets must contain seven arm joints")
        vectors.append(target)
    if not vectors:
        raise EvidenceError(f"{name} has no command_targets samples")
    return vectors


def _expected_vector(records: Sequence[Mapping[str, Any]]) -> list[float] | None:
    names = ("expected_final_positions", "final_positions", "target_positions", "commanded_positions", "joint_targets")
    for record in reversed(records):
        for mapping in _walk(record):
            raw = _first(mapping, names)
            if isinstance(raw, Sequence) and not isinstance(raw, (str, bytes)):
                return _vector(raw, "expected final positions")
    return None


def _check(name: str, passed: bool, *, metrics: Mapping[str, Any] | None = None, reasons: Iterable[str] = (), frames: Iterable[int] = ()) -> dict[str, Any]:
    return {
        "name": name,
        "passed": bool(passed),
        "metrics": dict(metrics or {}),
        "reasons": list(reasons),
        "source_frame_indices": sorted({int(index) for index in frames}),
    }


def _joint_metrics(frames: Sequence[Mapping[str, Any]], expected: list[float] | None) -> tuple[dict[str, Any], list[int]]:
    if expected is None:
        raise EvidenceError("FJT evidence missing expected final/command target positions")
    errors: list[float] = []
    used: list[int] = []
    target_frames = 0
    for item in frames:
        _, positions, _, _ = _arm_joint_data(item["raw"])
        target = _target_vector(item["raw"])
        if target is None:
            target = expected if len(expected) == len(positions) else None
        if target is None or len(target) != len(positions):
            continue
        target_frames += 1
        errors.extend(abs(actual - desired) for actual, desired in zip(positions, target))
        used.append(item["index"])
    if not used:
        raise EvidenceError("FJT evidence has no compatible target/joint samples")
    _, final_positions, _, _ = _arm_joint_data(frames[-1]["raw"])
    if len(final_positions) != len(expected):
        raise EvidenceError("expected final target length differs from robot joints")
    final_errors = [abs(actual - desired) for actual, desired in zip(final_positions, expected)]
    return {
        "final_max_error_rad": max(final_errors),
        "rms_error_rad": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "target_frames": target_frames,
        "final_errors_rad": final_errors,
    }, used


def _contact_force(contact: Mapping[str, Any]) -> float:
    return _finite(_first(contact, ("normal_force", "force", "normal_force_n")), "contact force")


def _qualification_scenario_contact(contact: Mapping[str, Any]) -> bool:
    bodies = [
        str(contact.get("body_a", "")).lower().replace("\\", "/"),
        str(contact.get("body_b", "")).lower().replace("\\", "/"),
    ]
    return any(
        "/world/scenario/" in body
        or _norm_body(body).startswith("qualification_")
        for body in bodies
    )


def _bilateral(frame: Mapping[str, Any], minimum: float) -> tuple[bool, bool, list[int]]:
    left = right = False
    for contact in _contacts(frame):
        if _contact_force(contact) < minimum:
            continue
        bodies = {_norm_body(contact.get("body_a", "")), _norm_body(contact.get("body_b", ""))}
        if "qualification_cube" not in bodies:
            continue
        left = left or bool(bodies.intersection({"left_finger", "left_finger_link"}))
        right = right or bool(
            bodies.intersection({"right_finger", "right_finger_link"})
        )
    return left, right, []


def _gripper_position(frame: Mapping[str, Any]) -> float | None:
    robot = frame.get("robot")
    if not isinstance(robot, Mapping):
        return None
    for key in ("drive_joint", "gripper_position", "gripper_drive_joint", "gripper_joint_position"):
        if key in robot:
            return _finite(robot[key], f"robot.{key}")
    gripper = robot.get("gripper", frame.get("gripper"))
    if isinstance(gripper, Mapping):
        raw = _first(gripper, ("drive_joint", "position", "joint_position"))
        if raw is not None:
            return _finite(raw, "gripper.position")
    names, positions, _, _ = _joint_data(frame)
    for index, name in enumerate(names):
        if name in {"drive_joint", "gripper_joint"} or "gripper" in name.lower():
            return positions[index]
    return None


def _gripper_efforts(frames: Sequence[Mapping[str, Any]], execution: Sequence[Mapping[str, Any]]) -> list[float]:
    values: list[float] = []
    for item in frames:
        raw = item["raw"]
        names, _, _, measured_efforts = _joint_data(raw)
        if measured_efforts and "drive_joint" in names:
            values.append(measured_efforts[names.index("drive_joint")])
        for container in (raw, raw.get("robot"), raw.get("gripper")):
            if not isinstance(container, Mapping):
                continue
            for key in ("gripper_effort", "drive_effort", "effort", "applied_effort", "max_effort"):
                if key in container and isinstance(container[key], (int, float)) and not isinstance(container[key], bool):
                    values.append(_finite(container[key], key))
    final_limits = frames[-1]["raw"].get("actuator_limits") if frames else None
    if isinstance(final_limits, Mapping) and "drive_joint" in final_limits:
        values.append(
            _finite(final_limits["drive_joint"], "actuator_limits.drive_joint")
        )
    values.extend(_number_values(execution, ("max_effort", "gripper_effort", "drive_effort", "applied_effort")))
    return values


def _object_pose(frame: Mapping[str, Any]) -> tuple[list[float], list[float], list[float]] | None:
    obj = _object(frame)
    if obj is None:
        return None
    pose = obj.get("pose")
    position = _pose_position(pose, "object.pose")
    orientation = _pose_quaternion(pose, "object.pose")
    twist = obj.get("twist", {})
    if not isinstance(twist, Mapping):
        raise EvidenceError("object.twist must be an object")
    velocity = _vector(_first(twist, ("linear", "linear_velocity")) or [0, 0, 0], "object.twist.linear", 3)
    return position, orientation, velocity


def _q_inverse(q: Sequence[float]) -> list[float]:
    return [-q[0], -q[1], -q[2], q[3]]


def _q_mul(a: Sequence[float], b: Sequence[float]) -> list[float]:
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return [w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2]


def _q_rotate(q: Sequence[float], vector: Sequence[float]) -> list[float]:
    return _q_mul(_q_mul(q, [*vector, 0.0]), _q_inverse(q))[:3]


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((first - second) ** 2 for first, second in zip(a, b)))


def _max_delta(a: Sequence[float], b: Sequence[float]) -> float:
    if len(a) != len(b):
        raise EvidenceError("target vectors have inconsistent lengths")
    return max((abs(first - second) for first, second in zip(a, b)), default=0.0)


def _retention_metrics(frames: Sequence[Mapping[str, Any]], threshold: Mapping[str, float]) -> tuple[dict[str, Any], list[int]]:
    parsed: list[dict[str, Any]] = []
    initial: list[float] | None = None
    reference: tuple[list[float], list[float]] | None = None
    for item in frames:
        pose = _object_pose(item["raw"])
        if pose is None:
            continue
        position, orientation, velocity = pose
        if initial is None:
            initial = position
        left, right, _ = _bilateral(item["raw"], threshold["contact_force_n"])
        tcp_pose = _robot(item["raw"]).get("tcp_pose")
        tcp_position = _pose_position(tcp_pose, "robot.tcp_pose")
        tcp_orientation = _pose_quaternion(tcp_pose, "robot.tcp_pose")
        if left and right and reference is None:
            reference = (_q_rotate(_q_inverse(tcp_orientation), [a-b for a, b in zip(position, tcp_position)]), _q_mul(_q_inverse(tcp_orientation), orientation))
        if reference is None:
            drift_m = drift_deg = 0.0
        else:
            relative_position = _q_rotate(_q_inverse(tcp_orientation), [a-b for a, b in zip(position, tcp_position)])
            relative_orientation = _q_mul(_q_inverse(tcp_orientation), orientation)
            dot = abs(sum(a*b for a, b in zip(relative_orientation, reference[1])))
            drift_m = _distance(relative_position, reference[0])
            drift_deg = math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))
        speed = _distance(velocity, [0.0, 0.0, 0.0])
        lift = position[2] - initial[2]
        translation = _distance(position, initial)
        stable = bool(left and right and lift >= threshold["retention_lift_m"] and translation >= threshold["retention_translation_m"] and drift_m <= threshold["retention_drift_m"] and drift_deg <= threshold["retention_drift_deg"] and speed <= threshold["stable_speed_m_s"] and not _safety(item["raw"]))
        parsed.append({"item": item, "position": position, "lift": lift, "translation": translation, "drift_m": drift_m, "drift_deg": drift_deg, "speed": speed, "left": left, "right": right, "stable": stable})
    if not parsed or initial is None:
        raise EvidenceError("retention requires qualification_cube object samples")
    stable_runs: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    for entry in parsed:
        if entry["stable"]:
            current.append(entry)
        elif current:
            stable_runs.append(current)
            current = []
    if current:
        stable_runs.append(current)
    best = max(stable_runs, key=lambda run: run[-1]["item"]["timestamp"] - run[0]["item"]["timestamp"], default=[])
    hold = (best[-1]["item"]["timestamp"] - best[0]["item"]["timestamp"]) if best else 0.0
    frames_used = [entry["item"]["index"] for entry in best]
    return {
        "lift_m": max(entry["lift"] for entry in parsed),
        "translation_m": max(entry["translation"] for entry in parsed),
        "max_drift_m": max(entry["drift_m"] for entry in parsed),
        "max_drift_deg": max(entry["drift_deg"] for entry in parsed),
        "max_object_speed_m_s": max(
            (entry["speed"] for entry in best),
            default=max(entry["speed"] for entry in parsed),
        ),
        "stable_hold_s": hold,
        "stable_frame_count": len(best),
        "bilateral_frames": sum(1 for entry in parsed if entry["left"] and entry["right"]),
    }, frames_used


def _safety_operator_events(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    events: list[tuple[int, str, bool, float | None]] = []
    for order, mapping in enumerate(records):
        event = str(mapping.get("event", ""))
        if event not in {"safety_asserted", "safety_cleared"}:
            continue
        if _endpoint(mapping) != SAFETY_TOPIC:
            continue
        result = mapping.get("result")
        active = result.get("active") if isinstance(result, Mapping) else mapping.get("active")
        parsed = _as_bool(active)
        if parsed is None:
            continue
        raw_timestamp = _first(mapping, ("simulated_timestamp", "sim_timestamp", "simulation_time", "sim_time"))
        timestamp = _finite(raw_timestamp, "safety journal timestamp") if raw_timestamp is not None else None
        events.append((order, event, parsed, timestamp))
    asserted = next((event for event in events if event[1] == "safety_asserted" and event[2] is True), None)
    cleared = next((event for event in events if event[1] == "safety_cleared" and event[2] is False), None)
    ordered = bool(asserted and cleared and asserted[0] < cleared[0])
    if ordered and asserted[3] is not None and cleared[3] is not None:
        ordered = asserted[3] <= cleared[3]
    return {
        "ordered": ordered,
        "asserted": asserted is not None,
        "cleared": cleared is not None,
        "events": [{"event": event, "active": active, "timestamp": timestamp} for _, event, active, timestamp in events],
    }


def _verify_gate(gate: str, frames: Sequence[Mapping[str, Any]], execution: Sequence[Mapping[str, Any]], thresholds: Mapping[str, float]) -> list[dict[str, Any]]:
    summary = _action_summary(execution)
    checks: list[dict[str, Any]] = []
    endpoints = summary["endpoints"]
    endpoint_contracts = {
        "free-space-fjt": ({TRAJECTORY_ACTION}, {TRAJECTORY_ACTION}),
        "safety-stop": ({TRAJECTORY_ACTION, SAFETY_TOPIC}, {TRAJECTORY_ACTION, SAFETY_TOPIC}),
        "free-gripper": ({GRIPPER_ACTION}, {GRIPPER_ACTION}),
        "obstructed-gripper": ({TRAJECTORY_ACTION, GRIPPER_ACTION}, {TRAJECTORY_ACTION, GRIPPER_ACTION}),
        "arm-collision": ({TRAJECTORY_ACTION}, {TRAJECTORY_ACTION}),
        "retention": ({TRAJECTORY_ACTION, GRIPPER_ACTION}, {TRAJECTORY_ACTION, GRIPPER_ACTION}),
    }
    allowed_endpoints, required_endpoints = endpoint_contracts[gate]
    endpoint_set = set(endpoints)
    endpoint_ok = endpoint_set.issubset(allowed_endpoints) and required_endpoints.issubset(endpoint_set)
    checks.append(_check("action_endpoint", endpoint_ok, metrics={"endpoints": endpoints}, reasons=() if endpoint_ok else ("action endpoint does not match the gate contract",)))
    if gate == "free-space-fjt":
        expected = _expected_vector(execution)
        metrics, used = _joint_metrics(frames, expected)
        checks.extend([
            _check("action_accepted_success", summary["accepted"] and summary["success"] is True, metrics={"accepted": summary["accepted"], "success": summary["success"]}, reasons=() if summary["accepted"] and summary["success"] is True else ("trajectory action was not explicitly accepted and successful",)),
            _check("fjt_error", metrics["final_max_error_rad"] <= thresholds["fjt_final_max_error_rad"] and metrics["rms_error_rad"] <= thresholds["fjt_rms_error_rad"], metrics=metrics, reasons=() if metrics["final_max_error_rad"] <= thresholds["fjt_final_max_error_rad"] and metrics["rms_error_rad"] <= thresholds["fjt_rms_error_rad"] else ("joint tracking error exceeds threshold",), frames=used),
        ])
        contacts = [
            item["index"]
            for item in frames
            if any(
                _contact_force(contact) > 0
                and _qualification_scenario_contact(contact)
                for contact in _contacts(item["raw"])
            )
        ]
        safety = [item["index"] for item in frames if _safety(item["raw"])]
        checks.append(_check("no_safety_contact_gateway_error", not contacts and not safety, metrics={"contact_frames": contacts, "safety_frames": safety}, reasons=() if not contacts and not safety else ("unexpected contact or safety stop in free-space gate",), frames=contacts + safety))
    elif gate == "safety-stop":
        if summary["success"] is None:
            raise EvidenceError("safety gate missing terminal action result")
        safety_events = _safety_operator_events(execution)
        checks.append(_check("safety_operator_journal", safety_events["ordered"], metrics=safety_events, reasons=() if safety_events["ordered"] else ("missing ordered safety_asserted/safety_cleared executor events",)))
        stop_indices = [item["index"] for item in frames if _safety(item["raw"])]
        if not stop_indices:
            checks.append(_check("effective_safety_stop", False, reasons=("no effective safety stop in raw truth",)))
            raise EvidenceError("safety gate requires safety-stop samples")
        stop_position = stop_indices[0]
        first = next(index for index, item in enumerate(frames) if item["index"] == stop_position)
        compliant = None
        velocities: list[float] = []
        for index in range(first, len(frames)):
            _, _, velocity, _ = _arm_joint_data(frames[index]["raw"])
            speed = max(abs(value) for value in velocity)
            velocities.append(speed)
            if speed <= thresholds["safety_stop_velocity_rad_s"]:
                compliant = index
                break
        checks.append(_check("action_accepted_old_goal_non_success", summary["accepted"] and summary["success"] is False, metrics={"accepted": summary["accepted"], "success": summary["success"]}, reasons=() if summary["accepted"] and summary["success"] is False else ("old trajectory did not explicitly terminate non-successfully",)))
        checks.append(_check("effective_safety_stop", compliant is not None and compliant - first <= int(thresholds["safety_stop_frames"]), metrics={"stop_frame": stop_position, "first_compliant_frame": frames[compliant]["index"] if compliant is not None else None, "max_velocity_before_compliance": max(velocities) if velocities else None}, reasons=() if compliant is not None and compliant - first <= int(thresholds["safety_stop_frames"]) else ("velocity did not become compliant within the stop window",), frames=stop_indices))
        _, initial_positions, _, _ = _arm_joint_data(frames[first]["raw"])
        clear = next((index for index in range(first + 1, len(frames)) if not _safety(frames[index]["raw"])), None)
        if clear is None:
            raise EvidenceError("safety gate missing safety clear samples")
        stopped_end = next((index for index in range(clear, first - 1, -1) if frames[index]["timestamp"] - frames[first]["timestamp"] >= 0.5), None)
        post_end = next((index for index in range(clear, len(frames)) if frames[index]["timestamp"] - frames[clear]["timestamp"] >= 1.0), None)
        if stopped_end is None or post_end is None:
            raise EvidenceError("safety gate lacks 0.5s stopped and 1s post-clear samples")
        positions = []
        target_vectors = _required_target_vectors(frames, first, post_end, name="safety stop window")
        for item in frames[first:post_end + 1]:
            _, current, _, _ = _arm_joint_data(item["raw"])
            positions.extend(abs(a-b) for a, b in zip(current, initial_positions))
        target_motion = max((_max_delta(a, b) for a, b in zip(target_vectors, target_vectors[1:])), default=0.0)
        checks.append(_check("safety_position_creep_and_target_freeze", max(positions, default=0.0) <= thresholds["safety_stop_position_creep_rad"] and target_motion <= thresholds["safety_stop_position_creep_rad"], metrics={"max_position_creep_rad": max(positions, default=0.0), "target_motion": target_motion, "stopped_end_frame": frames[stopped_end]["index"], "post_clear_end_frame": frames[post_end]["index"]}, reasons=() if max(positions, default=0.0) <= thresholds["safety_stop_position_creep_rad"] and target_motion <= thresholds["safety_stop_position_creep_rad"] else ("position or active target moved after safety stop",), frames=[item["index"] for item in frames[first:post_end + 1]]))
    elif gate in {"free-gripper", "obstructed-gripper"}:
        close = _phase_summary(execution, "close", endpoint=GRIPPER_ACTION)
        open_ = _phase_summary(execution, "open", endpoint=GRIPPER_ACTION)
        if gate == "free-gripper":
            checks.append(_check("close_open_actions", close["accepted"] and close["success"] is True and open_["accepted"] and open_["success"] is True and close["reached_goal"] is True and open_["reached_goal"] is True and close["stalled"] is False and open_["stalled"] is False, metrics={"close": {key: close[key] for key in ("accepted", "success", "reached_goal", "stalled")}, "open": {key: open_[key] for key in ("accepted", "success", "reached_goal", "stalled")}}, reasons=() if close["accepted"] and close["success"] is True and open_["accepted"] and open_["success"] is True and close["reached_goal"] is True and open_["reached_goal"] is True and close["stalled"] is False and open_["stalled"] is False else ("close/open action endpoint did not meet the free-gripper result contract",)))
        else:
            grasp = _phase_summary(execution, "grasp", endpoint=TRAJECTORY_ACTION)
            checks.append(_check("obstructed_action_result", close["accepted"] and close["success"] is True and close["stalled"] is True and close["reached_goal"] is False, metrics={"close": {key: close[key] for key in ("accepted", "success", "reached_goal", "stalled")}}, reasons=() if close["accepted"] and close["success"] is True and close["stalled"] is True and close["reached_goal"] is False else ("obstructed close did not explicitly succeed with stalled/not-reached result",)))
            checks.append(_check("obstructed_grasp_action", grasp["accepted"] and grasp["success"] is True, metrics={"grasp": {key: grasp[key] for key in ("accepted", "success")}}, reasons=() if grasp["accepted"] and grasp["success"] is True else ("obstructed grasp trajectory phase was not accepted and successful",)))
        positions = [value for item in frames if (value := _gripper_position(item["raw"])) is not None]
        if not positions:
            raise EvidenceError("gripper gate missing drive_joint position samples")
        efforts = _gripper_efforts(frames, execution)
        if not efforts:
            raise EvidenceError("gripper gate missing effort evidence")
        contacts = []
        arm_collisions = []
        for item in frames:
            for contact in _contacts(item["raw"]):
                bodies = {_norm_body(contact.get("body_a", "")), _norm_body(contact.get("body_b", ""))}
                if _contact_force(contact) > 0:
                    if "qualification_cube" in bodies and bodies.intersection({"left_finger", "right_finger", "left_finger_link", "right_finger_link"}):
                        contacts.append(item["index"])
                    if any(f"link{number}" in bodies for number in range(1, 8)) and "qualification_arm_obstacle" in bodies:
                        arm_collisions.append(item["index"])
        if gate == "free-gripper":
            checks.append(_check("free_gripper_travel_effort", max(positions) - min(positions) >= thresholds["free_gripper_min_travel_rad"] and max(abs(value) for value in efforts) <= thresholds["gripper_effort_limit_n"] + thresholds["numeric_tolerance"], metrics={"travel_rad": max(positions)-min(positions), "max_abs_effort_n": max(abs(value) for value in efforts)}, reasons=() if max(positions)-min(positions) >= thresholds["free_gripper_min_travel_rad"] and max(abs(value) for value in efforts) <= thresholds["gripper_effort_limit_n"] + thresholds["numeric_tolerance"] else ("gripper travel or effort limit failed",)))
            checks.append(_check("no_contact_safety_gateway_error", not contacts and not arm_collisions and not any(_safety(item["raw"]) for item in frames), metrics={"contact_frames": contacts, "collision_frames": arm_collisions}, reasons=() if not contacts and not arm_collisions and not any(_safety(item["raw"]) for item in frames) else ("unexpected contact, collision, or safety stop",), frames=contacts + arm_collisions))
        else:
            if any(_object(item["raw"]) is None for item in frames):
                raise EvidenceError("obstructed gripper gate requires qualification_cube state in every frame")
            target = thresholds["gripper_target_rad"]
            gap = target - max(positions)
            left_right = [item["index"] for item in frames if _bilateral(item["raw"], thresholds["contact_force_n"])[0] and _bilateral(item["raw"], thresholds["contact_force_n"])[1]]
            checks.append(_check("obstructed_gap_contact_effort", gap >= thresholds["obstructed_gripper_min_gap_rad"] and bool(left_right) and max(abs(value) for value in efforts) <= thresholds["gripper_effort_limit_n"] + thresholds["numeric_tolerance"], metrics={"target_gap_rad": gap, "bilateral_contact_frames": left_right, "max_abs_effort_n": max(abs(value) for value in efforts)}, reasons=() if gap >= thresholds["obstructed_gripper_min_gap_rad"] and bool(left_right) and max(abs(value) for value in efforts) <= thresholds["gripper_effort_limit_n"] + thresholds["numeric_tolerance"] else ("obstructed gripper gap, exact bilateral contact, or effort limit failed",), frames=left_right + arm_collisions))
            checks.append(_check("no_arm_collision_safety_gateway_error", not arm_collisions and not any(_safety(item["raw"]) for item in frames), metrics={"collision_frames": arm_collisions}, reasons=() if not arm_collisions and not any(_safety(item["raw"]) for item in frames) else ("arm collision or safety stop occurred",), frames=arm_collisions))
    elif gate == "arm-collision":
        collision_frames = []
        for item in frames:
            for contact in _contacts(item["raw"]):
                bodies = {_norm_body(contact.get("body_a", "")), _norm_body(contact.get("body_b", ""))}
                arm = any(f"link{number}" in bodies for number in range(1, 8))
                if arm and "qualification_arm_obstacle" in bodies and _contact_force(contact) >= thresholds["contact_force_n"]:
                    collision_frames.append(item["index"])
        stop = next((item["index"] for item in frames if _safety(item["raw"])), None)
        if stop is None:
            raise EvidenceError("collision gate missing effective safety assertion")
        stop_index = next(index for index, item in enumerate(frames) if item["index"] == stop)
        compliant = None
        for index in range(stop_index, len(frames)):
            _, _, velocity, _ = _arm_joint_data(frames[index]["raw"])
            if max(abs(value) for value in velocity) <= thresholds["safety_stop_velocity_rad_s"]:
                compliant = index
                break
        target_vectors = _required_target_vectors(frames, stop_index, len(frames) - 1, name="collision stop window")
        target_motion = max((_max_delta(a, b) for a, b in zip(target_vectors, target_vectors[1:])), default=0.0)
        _, stop_positions, _, _ = _arm_joint_data(frames[stop_index]["raw"])
        post_stop_creep = []
        for item in frames[stop_index:]:
            _, positions, _, _ = _arm_joint_data(item["raw"])
            post_stop_creep.extend(abs(actual - baseline) for actual, baseline in zip(positions, stop_positions))
        checks.extend([
            _check("exact_arm_obstacle_contact", bool(collision_frames), metrics={"contact_frames": collision_frames}, reasons=() if collision_frames else ("missing exact link1-link7 to qualification_arm_obstacle contact",), frames=collision_frames),
            _check("safety_asserted_and_action_non_success", stop is not None and summary["success"] is False, metrics={"safety_frame": stop, "action_success": summary["success"]}, reasons=() if stop is not None and summary["success"] is False else ("collision did not assert safety or terminate the action non-successfully",), frames=[stop] if stop is not None else ()),
            _check("collision_stop_and_no_pass_through", compliant is not None and compliant-stop_index <= int(thresholds["safety_stop_frames"]) and target_motion <= thresholds["safety_stop_position_creep_rad"] and max(post_stop_creep, default=0.0) <= thresholds["safety_stop_position_creep_rad"], metrics={"first_compliant_frame": frames[compliant]["index"] if compliant is not None else None, "target_motion": target_motion, "post_stop_creep_rad": max(post_stop_creep, default=0.0)}, reasons=() if compliant is not None and compliant-stop_index <= int(thresholds["safety_stop_frames"]) and target_motion <= thresholds["safety_stop_position_creep_rad"] and max(post_stop_creep, default=0.0) <= thresholds["safety_stop_position_creep_rad"] else ("collision stopping or target freeze/pass-through check failed",), frames=[item["index"] for item in frames[stop_index:]]),
        ])
    elif gate == "retention":
        if any(_object(item["raw"]) is None for item in frames):
            raise EvidenceError("retention gate requires qualification_cube state in every frame")
        grasp = _phase_summary(execution, "grasp", endpoint=TRAJECTORY_ACTION)
        close = _phase_summary(execution, "close", endpoint=GRIPPER_ACTION)
        lift = _phase_summary(execution, "lift", endpoint=TRAJECTORY_ACTION)
        checks.extend([
            _check("retention_grasp_action", grasp["accepted"] and grasp["success"] is True, metrics={"grasp": {key: grasp[key] for key in ("accepted", "success")}}, reasons=() if grasp["accepted"] and grasp["success"] is True else ("retention grasp trajectory phase was not accepted and successful",)),
            _check(
                "retention_close_action",
                close["accepted"]
                and close["success"] is True
                and close["stalled"] is True
                and close["reached_goal"] is False,
                metrics={
                    "close": {
                        key: close[key]
                        for key in ("accepted", "success", "reached_goal", "stalled")
                    }
                },
                reasons=()
                if (
                    close["accepted"]
                    and close["success"] is True
                    and close["stalled"] is True
                    and close["reached_goal"] is False
                )
                else (
                    "retention close phase did not explicitly succeed with "
                    "stalled/not-reached result",
                ),
            ),
            _check("retention_lift_action", lift["accepted"] and lift["success"] is True, metrics={"lift": {key: lift[key] for key in ("accepted", "success")}}, reasons=() if lift["accepted"] and lift["success"] is True else ("retention lift trajectory phase was not accepted and successful",)),
        ])
        metrics, used = _retention_metrics(frames, thresholds)
        checks.append(_check("retention_physics_truth", metrics["lift_m"] >= thresholds["retention_lift_m"] and metrics["translation_m"] >= thresholds["retention_translation_m"] and metrics["stable_hold_s"] >= thresholds["retention_hold_s"] and metrics["max_drift_m"] <= thresholds["retention_drift_m"] and metrics["max_drift_deg"] <= thresholds["retention_drift_deg"] and metrics["max_object_speed_m_s"] <= thresholds["stable_speed_m_s"], metrics=metrics, reasons=() if metrics["lift_m"] >= thresholds["retention_lift_m"] and metrics["translation_m"] >= thresholds["retention_translation_m"] and metrics["stable_hold_s"] >= thresholds["retention_hold_s"] and metrics["max_drift_m"] <= thresholds["retention_drift_m"] and metrics["max_drift_deg"] <= thresholds["retention_drift_deg"] and metrics["max_object_speed_m_s"] <= thresholds["stable_speed_m_s"] else ("retention physical postcondition failed",), frames=used))
        checks.append(_check("retention_no_safety_gateway_error", not any(_safety(item["raw"]) for item in frames), metrics={}, reasons=() if not any(_safety(item["raw"]) for item in frames) else ("safety stop occurred during retention",)))
    return checks


def _atomic_write(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def verify(gate: str, attempt_dir: Path, config_path: Path) -> dict[str, Any]:
    manifest_path = attempt_dir / "manifest.json"
    attempt_id = attempt_dir.name
    manifest_error: EvidenceError | None = None
    if manifest_path.is_file():
        try:
            manifest = _read_json(manifest_path)
            attempt_id = str(manifest.get("attempt_id", attempt_id))
        except EvidenceError as error:
            manifest_error = error
    base: dict[str, Any] = {"schema_version": SCHEMA_VERSION, "attempt_id": attempt_id, "gate": gate, "status": "evidence-invalid", "pass": False, "verified": False, "checks": [], "metrics": {}, "errors": []}
    try:
        if manifest_error is not None:
            raise manifest_error
        if gate not in GATES:
            raise EvidenceError(f"unknown gate {gate!r}")
        if not attempt_dir.is_dir():
            raise EvidenceError(f"missing attempt directory {attempt_dir}")
        config = _read_json(config_path)
        thresholds = _thresholds(config)
        physics = config.get("physics", {})
        physics_hz = None
        if isinstance(physics, Mapping) and physics.get("hz") is not None:
            physics_hz = _finite(physics["hz"], "physics.hz")
            if physics_hz <= 0:
                raise EvidenceError("physics.hz must be positive")
        if physics_hz is None:
            raise EvidenceError("physics.hz is required to bound the gate evidence window")
        allow_sparse_timestamps = config.get("test_only_allow_sparse_frames") is True
        if allow_sparse_timestamps and manifest_path.is_file():
            raise EvidenceError("test_only_allow_sparse_frames is forbidden for production manifests")
        execution, execution_paths = _load_execution(attempt_dir)
        gate_start, gate_end = _execution_window(execution)
        if gate_start is None or gate_end is None:
            raise EvidenceError("gate execution has an invalid simulated-time window")
        raw_records = _read_jsonl(attempt_dir / "physics_truth.jsonl", required=True)
        gate_records, _window_path = _raw_gate_window(
            raw_records,
            attempt_dir,
            gate,
            attempt_id=attempt_id,
            manifest_present=manifest_path.is_file(),
            gate_start=gate_start,
            gate_end=gate_end,
            physics_hz=physics_hz if physics_hz is not None else 0.0,
        )
        frames = _parse_truth(
            gate_records,
            physics_hz=physics_hz,
            allow_sparse_timestamps=allow_sparse_timestamps,
        )
        allowed_scenarios = _scenario_options(config, gate)
        actual_scenarios = {str(item["raw"].get("scenario", "")) for item in frames}
        if not actual_scenarios.intersection(set(allowed_scenarios)) or not actual_scenarios.issubset(set(allowed_scenarios)):
            raise EvidenceError(f"unexpected scenario identity: {sorted(actual_scenarios)}; expected {list(allowed_scenarios)}")
        for record_index, record in enumerate(execution):
            for mapping in _walk(record):
                if "gate" in mapping and str(mapping["gate"]) != gate:
                    raise EvidenceError(f"execution[{record_index}] gate identity does not match {gate}")
                if "scenario" in mapping and str(mapping["scenario"]) not in allowed_scenarios:
                    raise EvidenceError(f"execution[{record_index}] scenario identity is unexpected")
                for key in ("reset", "teleport", "time_reset", "simulation_reset"):
                    if _as_bool(mapping.get(key)) is True:
                        raise EvidenceError(f"execution[{record_index}] unexplained {key}")
        execution_gateway_errors: list[str] = []
        for index, record in enumerate(execution):
            execution_gateway_errors.extend(
                f"execution[{index}] {error}" for error in _gateway_errors(record)
            )
        if execution_gateway_errors:
            raise EvidenceError("command gateway error: " + "; ".join(execution_gateway_errors))
        if _is_external(execution):
            base["errors"] = ["external or diagnostic execution can never establish verified-pass"]
            base["status"] = "verified-fail"
            base["verified"] = True
            base["checks"] = [_check("execution_source", False, reasons=(base["errors"][0],))]
            return base
        summary = _action_summary(execution)
        if not summary["endpoints"]:
            raise EvidenceError("gate execution has no action endpoint")
        minimum_frames = 2
        if len(frames) < minimum_frames:
            raise EvidenceError(f"{gate} requires at least {minimum_frames} raw truth frames")
        evaluator_path = attempt_dir / "evaluator.jsonl"
        if evaluator_path.is_file():
            evaluator = _read_jsonl(evaluator_path, required=True)
            if len(evaluator) < len(raw_records):
                raise EvidenceError(
                    "evaluator contains fewer records than raw physics truth"
                )
        checks = _verify_gate(gate, frames, execution, thresholds)
        base["checks"] = checks
        base["metrics"] = {check["name"]: check["metrics"] for check in checks}
        base["status"] = "verified-pass" if all(check["passed"] for check in checks) else "verified-fail"
        base["pass"] = base["status"] == "verified-pass"
        base["verified"] = True
        base["execution_sources"] = [path.name for path in execution_paths]
    except EvidenceError as error:
        base["errors"] = [str(error)]
        base["status"] = "evidence-invalid"
        base["pass"] = False
        base["verified"] = False
    return base


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gate", required=True, choices=sorted(GATES))
    parser.add_argument("--attempt-dir", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    args = parser.parse_args(argv)
    verdict = verify(args.gate, args.attempt_dir, args.config)
    output = args.attempt_dir / "gate-verdict.json"
    _atomic_write(output, verdict)
    print(json.dumps(verdict, indent=2, sort_keys=True))
    return {"verified-pass": 0, "verified-fail": 1, "evidence-invalid": 2}[verdict["status"]]


if __name__ == "__main__":
    sys.exit(main())
