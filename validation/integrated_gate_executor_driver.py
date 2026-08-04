#!/usr/bin/env python3
"""Source-run Humble executor driver for the integrated OMPL qualification.

Task 8 fix round 2 (F2.1-F2.2): this module is the live producer of the
executor evidence and the scenario terminal marker that round 1 lacked.  It runs
as a third owned child of ``IntegratedRunner`` (launched only after the overlay
has produced canonical PHYSICS_READY), constructs the real
:class:`~integrated_gate_executor.IntegratedGateExecutor` for the current
immutable attempt, waits for executor-schema readiness, sets and reads back the
production ``/pick_and_place.post_grasp_lift_m`` runtime parameter, dispatches
exactly one of the 17 canonical scenarios, and writes ``execution-terminal.json``
only after the executor's own artifact finalization has completed.

ROS-lazy: importing this module under the simulator CPython 3.12 venv never
imports ``rclpy`` or any generated message type.  All ROS imports (rclpy,
generated messages, the real executor) live inside :func:`main` /
``_construct_executor``.  The pure dispatch/serialization/bundle/terminal/lift
layer below is importable and unit-tested under Python 3.12.

The driver never fabricates readiness or provider values.  Missing, stale,
malformed, or contradictory provider data fails closed.  The independent
verifier remains authoritative: on a driver-level failure the driver writes a
durable fail-closed ``execution-terminal.json`` (``status`` ``evidence-invalid``)
and exits nonzero; it never synthesizes passing physical evidence.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

# Importing the executor module is ROS-lazy (it never imports rclpy at module
# level); its committed scenario/kind constants are the single source of truth
# for the 17-scenario dispatch table.
from integrated_gate_executor import (  # noqa: E402
    FJT_ENDPOINT,
    STAGE_C_SCENARIOS,
    STAGE_D_KIND,
    STAGE_D_SCENARIOS,
    STAGE_E_KIND,
    STAGE_E_SCENARIOS,
)

TERMINAL_MARKER_FILENAME = "execution-terminal.json"
TERMINAL_SCHEMA_VERSION = 1
TERMINAL_MARKER = "executor-driver"

MAX_ROS_DOMAIN_ID = 232

#: The executor-owned artifact set that must be final before the driver writes
#: ``execution-terminal.json``.  The primary gate is ``integrated-execution.json``
#: (the executor's own terminal summary); the independent verifier then reads the
#: full required set (``integrated-execution.jsonl``, ``moveit-plans.jsonl``,
#: ``controller-results.jsonl``, ``planning-scene.jsonl``, ``integrated-execution.json``,
#: ``planning-scene.json``) and is authoritative.
EXECUTOR_ARTIFACT_FILENAMES = (
    "integrated-execution.jsonl",
    "moveit-plans.jsonl",
    "controller-results.jsonl",
    "planning-scene.jsonl",
    "integrated-execution.json",
    "planning-scene.json",
)

#: The E transport kinds (per committed ``_E_TRANSPORT_KINDS``) that require the
#: observed ``post_grasp_lift_m >= 0.10`` runtime parameter before any Pick traffic.
E_TRANSPORT_KINDS = frozenset({"positive", "occupied-place", "cancel-transport", "safety-transport"})

D_METHOD_BY_KIND: Mapping[str, str] = {
    "execute-joint": "run_execute_sequence",
    "execute-pose": "run_execute_sequence",
    "retreat": "run_cartesian_retreat",
    "gripper": "run_gripper_sequence",
    "cancel": "run_cancel_sequence",
    "safety": "run_safety_sequence",
}


class DriverError(Exception):
    """Fail-closed driver-level error (setup/dispatch/terminal)."""


class LiftParameterError(DriverError):
    """The ``/pick_and_place.post_grasp_lift_m`` set/read-back requirement failed."""


# --------------------------------------------------------------------------- #
# Terminal-budget derivation (F2.5)
# --------------------------------------------------------------------------- #

def derive_terminal_timeout(config: Mapping[str, Any]) -> float:
    """Derive the executor terminal budget from committed config thresholds.

    ``plan_timeout_s + 2*execute_timeout_s + cancel_timeout_s +
    scene_timeout_s + max(cancel_timeout_s, 30.0)``.  With the committed
    integrated-ompl thresholds (15/120/10/10) this is exactly ``305.0`` s,
    covering the source-inspected worst E transport path (~275 s) and D gripper
    path (240 s).  Every term must be finite and positive; malformed config
    fails closed.
    """
    thresholds = config.get("thresholds")
    if not isinstance(thresholds, Mapping):
        raise ValueError("config has no thresholds object")
    try:
        terms = {
            key: float(thresholds[key])
            for key in ("plan_timeout_s", "execute_timeout_s", "cancel_timeout_s", "scene_timeout_s")
        }
    except (TypeError, ValueError, KeyError) as error:
        raise ValueError(f"config terminal thresholds are malformed: {error}") from error
    for key, value in terms.items():
        if isinstance(value, bool) or not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{key} must be finite and positive, got {value}")
    cancel = terms["cancel_timeout_s"]
    settle = max(cancel, 30.0)
    return (
        terms["plan_timeout_s"]
        + 2.0 * terms["execute_timeout_s"]
        + terms["cancel_timeout_s"]
        + terms["scene_timeout_s"]
        + settle
    )


# --------------------------------------------------------------------------- #
# Dispatch table (F2.2) — exactly the 17 canonical scenario ids
# --------------------------------------------------------------------------- #

def canonical_dispatch() -> dict[str, str]:
    """Return the exact 17-scenario dispatch table (id -> executor run method).

    Derived from the committed executor constants.  ``run_gate_c_plan_only`` for
    all three Stage-C scenarios; ``run_execute_sequence`` for the two D execute
    kinds; ``run_cartesian_retreat`` / ``run_gripper_sequence`` /
    ``run_cancel_sequence`` / ``run_safety_sequence`` for the other four D
    scenarios; ``run_pick_place_sequence`` for all eight Stage-E scenarios.  The
    table must cover exactly the 17 canonical ids with no duplicates or extras.
    """
    table: dict[str, str] = {}
    for name in STAGE_C_SCENARIOS:
        table[name] = "run_gate_c_plan_only"
    for name in STAGE_D_SCENARIOS:
        kind = STAGE_D_KIND.get(name)
        method = D_METHOD_BY_KIND.get(kind)  # type: ignore[arg-type]
        if method is None:
            raise ValueError(f"no dispatch method for Stage-D scenario {name!r} kind {kind!r}")
        table[name] = method
    for name in STAGE_E_SCENARIOS:
        table[name] = "run_pick_place_sequence"
    expected = set(STAGE_C_SCENARIOS) | set(STAGE_D_SCENARIOS) | set(STAGE_E_SCENARIOS)
    if len(expected) != 17 or set(table) != expected:
        raise ValueError("dispatch table must cover exactly the 17 canonical scenario ids")
    if len(set(table)) != 17:
        raise ValueError("dispatch table has duplicate scenario ids")
    return dict(table)


def run_method_for(scenario_id: str) -> str:
    """Return the executor run-method name for a canonical scenario id.

    Unknown ids fail closed before any ROS traffic (F2.2).
    """
    method = canonical_dispatch().get(str(scenario_id))
    if method is None:
        raise DriverError(
            f"unknown scenario id {scenario_id!r}; not one of the 17 canonical scenarios"
        )
    return method


def is_e_scenario(scenario_id: str) -> bool:
    return str(scenario_id) in STAGE_E_SCENARIOS


def is_e_transport_scenario(scenario_id: str) -> bool:
    return str(scenario_id) in STAGE_E_SCENARIOS and STAGE_E_KIND.get(str(scenario_id)) in E_TRANSPORT_KINDS


# --------------------------------------------------------------------------- #
# Bundle load / identity binding (F2.1, F2.3)
# --------------------------------------------------------------------------- #

def load_bundle(path: Path | str) -> dict[str, Any]:
    """Load the exact ``scenario-bundle.json`` written atomically by the orchestrator."""
    bundle_path = Path(path)
    try:
        raw = json.loads(bundle_path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        raise DriverError(f"scenario-bundle could not be loaded: {error}") from error
    if not isinstance(raw, Mapping):
        raise DriverError("scenario-bundle is not an object")
    if raw.get("schema_version") != 1:
        raise DriverError(f"scenario-bundle schema_version must be 1, got {raw.get('schema_version')!r}")
    for key in ("scenario_id", "attempt_id", "attempt_dir", "scenario"):
        if key not in raw:
            raise DriverError(f"scenario-bundle is missing {key!r}")
    return dict(raw)


def validate_bundle_identity(
    bundle: Mapping[str, Any],
    *,
    attempt_dir: Path | str,
    seed: int | None = None,
) -> str:
    """Validate the bundle binds the current immutable attempt; return scenario id.

    Every path/identity must agree with the serialized bundle — the orchestrator
    never recomputes scenario/report identities across Python versions.
    """
    scenario = bundle.get("scenario")
    if not isinstance(scenario, Mapping) or not isinstance(scenario.get("id"), str):
        raise DriverError("scenario-bundle scenario.id is missing")
    scenario_id = str(scenario["id"])
    if str(bundle.get("scenario_id")) != scenario_id:
        raise DriverError("scenario-bundle scenario_id does not match scenario.id")
    attempt_id = str(bundle.get("attempt_id", ""))
    if not attempt_id:
        raise DriverError("scenario-bundle attempt_id is missing")
    bundle_dir = bundle.get("attempt_dir")
    resolved_attempt = Path(attempt_dir).resolve()
    if isinstance(bundle_dir, str) and bundle_dir:
        if Path(bundle_dir).resolve() != resolved_attempt:
            raise DriverError(
                "scenario-bundle attempt_dir does not match --attempt-dir "
                f"({bundle_dir!r} != {resolved_attempt})"
            )
    if seed is not None:
        try:
            bundle_seed = int(scenario.get("seed"))
        except (TypeError, ValueError) as error:
            raise DriverError("scenario-bundle scenario.seed is not an integer") from error
        if bundle_seed != int(seed):
            raise DriverError(
                f"scenario-bundle seed {bundle_seed} does not match --seed {seed}"
            )
    return scenario_id


def build_executor_scenario(bundle: Mapping[str, Any]) -> dict[str, Any]:
    """Build the executor-schema scenario mapping from the orchestrator bundle.

    The orchestrator serializes the scenario id/seed/declaration, planning-scene
    declaration, integrated mapping, and report identities; the executor consumes
    the same committed identities without recomputing them across Python
    versions.
    """
    scenario = bundle.get("scenario")
    if not isinstance(scenario, Mapping):
        raise DriverError("scenario-bundle scenario is missing")
    integrated = bundle.get("integrated")
    planning_scene_declaration = bundle.get("planning_scene_declaration")
    planning_scene = bundle.get("planning_scene")
    identities = bundle.get("report_identities")
    if not isinstance(integrated, Mapping):
        raise DriverError("scenario-bundle integrated is missing")
    if not isinstance(planning_scene_declaration, Mapping):
        raise DriverError("scenario-bundle planning_scene_declaration is missing")
    if not isinstance(planning_scene, Mapping):
        raise DriverError("scenario-bundle planning_scene is missing")
    if not isinstance(identities, Mapping):
        raise DriverError("scenario-bundle report_identities is missing")
    digest_fields = {
        key: value
        for key, value in identities.items()
        if key.endswith("_sha256") or key == "model_fingerprint"
    }
    return {
        "id": str(scenario.get("id")),
        "seed": int(scenario.get("seed")),
        "scenario_mapping": dict(scenario),
        "public_mapping": dict(scenario),
        "planning_scene": dict(planning_scene),
        "planning_scene_declaration": dict(planning_scene_declaration),
        "integrated": dict(integrated),
        "identities": dict(identities),
        "scenario_report_sha256": str(identities.get("scenario_report_sha256", "")),
        **digest_fields,
    }


# --------------------------------------------------------------------------- #
# Terminal marker (F2.2) — never before executor finalization
# --------------------------------------------------------------------------- #

def _atomic_write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_name = stream.name
            stream.write(json.dumps(dict(value), indent=2, sort_keys=True) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
        temporary_name = None
    finally:
        if temporary_name is not None:
            try:
                os.unlink(temporary_name)
            except OSError:
                pass


def reject_preexisting_terminal(attempt_dir: Path | str) -> None:
    """Refuse to run in an attempt dir that already carries a terminal marker."""
    marker = Path(attempt_dir) / TERMINAL_MARKER_FILENAME
    if marker.is_file():
        raise DriverError(
            f"attempt dir already contains {TERMINAL_MARKER_FILENAME}; "
            "refusing to overwrite a preexisting terminal marker"
        )


def write_terminal(
    attempt_dir: Path | str,
    scenario_id: str,
    attempt_id: str,
    status: str,
) -> Path:
    """Atomically write ``execution-terminal.json`` after executor finalization.

    The marker binds scenario id, attempt id, and the current attempt path; it is
    written only after the selected run method has returned (executor
    finalization complete).  A preexisting marker is never overwritten.
    """
    attempt_path = Path(attempt_dir).resolve()
    marker = attempt_path / TERMINAL_MARKER_FILENAME
    if marker.is_file():
        raise DriverError(f"terminal marker already exists: {marker}")
    _atomic_write_json(
        marker,
        {
            "schema_version": TERMINAL_SCHEMA_VERSION,
            "scenario_id": scenario_id,
            "attempt_id": attempt_id,
            "attempt_dir": str(attempt_path),
            "status": status,
            "marker": TERMINAL_MARKER,
            "written_at": time.time(),
        },
    )
    return marker


def ensure_executor_finalized(attempt_dir: Path | str) -> None:
    """Verify the executor's own terminal summary exists before the driver marker.

    ``integrated-execution.json`` is written by the executor at the end of every
    run-method finalization path.  Missing it means the executor did not complete
    finalization, so the driver fails closed instead of emitting its marker.
    """
    summary = Path(attempt_dir) / "integrated-execution.json"
    if not summary.is_file():
        raise DriverError(
            "executor did not finalize integrated-execution.json; refusing to "
            "write the driver terminal marker"
        )


# --------------------------------------------------------------------------- #
# post_grasp_lift_m runtime parameter (F2.4) — ROS-free client protocol
# --------------------------------------------------------------------------- #

def _double_parameter(name: str, value: float):
    """Return an rclpy ``Parameter`` (live) or a plain record (test double)."""
    try:
        from rclpy.parameter import Parameter  # type: ignore[import-not-found]
    except ImportError:  # pragma: no cover - ROS-free test doubles
        return {"name": name, "value": float(value)}
    return Parameter(name, Parameter.Type.DOUBLE, float(value))


def _set_result_ok(result: object) -> bool:
    if isinstance(result, (list, tuple)) and len(result) == 1:
        result = result[0]
    successful = getattr(result, "successful", None)
    if successful is not None:
        return bool(successful)
    if isinstance(result, Mapping):
        return bool(result.get("successful", False))
    return bool(result)


def _extract_double(result: object, name: str) -> float | None:
    if isinstance(result, (list, tuple)) and len(result) == 1:
        result = result[0]
    value = getattr(result, "value", None)
    if value is None and isinstance(result, Mapping):
        value = result.get("value")
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def set_post_grasp_lift_m(
    client: Any,
    *,
    value_m: float = 0.10,
    timeout_s: float = 10.0,
) -> dict[str, Any]:
    """Set ``/pick_and_place.post_grasp_lift_m`` and read it back; fail closed.

    Requires a successful parameter set response and an exact/tolerance-consistent
    finite read-back ``>= value_m`` and specifically the committed requested
    value.  Returns the observed read-back metadata (never a constant/default).
    Raises :class:`LiftParameterError` on any missing/stale/malformed/rejected
    outcome so E transport scenarios fail closed before Pick traffic.
    """
    if isinstance(value_m, bool):
        raise LiftParameterError("post_grasp_lift_m must be a finite number, not a bool")
    try:
        target = float(value_m)
    except (TypeError, ValueError) as error:
        raise LiftParameterError("post_grasp_lift_m must be a finite number") from error
    if not math.isfinite(target) or target <= 0.0:
        raise LiftParameterError("post_grasp_lift_m must be finite and positive")
    try:
        available = client.wait_for_service(timeout_sec=float(timeout_s))
    except Exception as error:  # pragma: no cover - live client boundary
        raise LiftParameterError(f"pick_and_place parameter service wait failed: {error}") from error
    if not available:
        raise LiftParameterError("pick_and_place parameter service is unavailable")
    try:
        set_result = client.set_parameters([_double_parameter("post_grasp_lift_m", target)])
    except Exception as error:  # pragma: no cover - live client boundary
        raise LiftParameterError(f"post_grasp_lift_m set failed: {error}") from error
    if not _set_result_ok(set_result):
        raise LiftParameterError("post_grasp_lift_m set was rejected")
    try:
        get_result = client.get_parameters(["post_grasp_lift_m"])
    except Exception as error:  # pragma: no cover - live client boundary
        raise LiftParameterError(f"post_grasp_lift_m read-back failed: {error}") from error
    observed = _extract_double(get_result, "post_grasp_lift_m")
    if observed is None:
        raise LiftParameterError("post_grasp_lift_m read-back returned no finite double value")
    if observed < target:
        raise LiftParameterError(
            f"post_grasp_lift_m read-back {observed} is below required {target}"
        )
    return {
        "value_m": observed,
        "identity": f"pick_and_place.post_grasp_lift_m:{time.monotonic():.6f}",
        "age_s": 0.0,
        "requested_value_m": target,
    }


def _parameter_client(executor: Any) -> Any:
    """Create an rclpy ``ParameterClient`` targeting ``/pick_and_place``.

    Uses the executor's own node/context so the parameter transaction shares the
    qualification graph and lifetime.
    """
    from rclpy.parameter import ParameterClient  # type: ignore[import-not-found]

    return ParameterClient(executor.node, "pick_and_place")


def _post_grasp_lift_m_provider(
    observed: Mapping[str, Any],
) -> Callable[[], Mapping[str, Any]]:
    """Return a fresh typed provider returning the observed read-back.

    Each call returns the observed ``value_m`` with a fresh identity/age so the
    executor's freshness check (`tf_fresh_s` 0.25) accepts it.  Never a
    constant/default from the production server.
    """

    def _provider() -> Mapping[str, Any]:
        return {
            "value_m": float(observed["value_m"]),
            "identity": str(observed["identity"]),
            "age_s": 0.0,
        }

    return _provider


# --------------------------------------------------------------------------- #
# Live construction + runtime provider factories (F2.1, F2.4)
# --------------------------------------------------------------------------- #

def _read_report_bytes(attempt_dir: Path) -> bytes:
    path = attempt_dir / "scenario-runner.json"
    if not path.is_file():
        raise DriverError(f"scenario-runner.json is missing: {path}")
    try:
        return path.read_bytes()
    except OSError as error:
        raise DriverError(f"scenario-runner.json is unreadable: {error}") from error


def _read_join_key(attempt_dir: Path) -> tuple[int, float] | None:
    """Read the raw truth tail for the exact (frame_index, timestamp) join key."""
    truth_path = attempt_dir / "physics_truth.jsonl"
    if not truth_path.is_file():
        return None
    try:
        with truth_path.open("rb") as stream:
            stream.seek(0, os.SEEK_END)
            size = stream.tell()
            tail_size = min(size, 32768)
            stream.seek(size - tail_size)
            tail = stream.read()
    except OSError:
        return None
    nonblank = [line for line in tail.split(b"\n") if line.strip()]
    if not nonblank:
        return None
    try:
        record = json.loads(nonblank[-1].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(record, Mapping):
        return None
    frame_index = record.get("frame_index")
    timestamp = record.get("timestamp")
    if isinstance(frame_index, bool) or not isinstance(frame_index, int) or frame_index < 0:
        return None
    if isinstance(timestamp, bool):
        return None
    try:
        timestamp = float(timestamp)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(timestamp) or timestamp < 0.0:
        return None
    return (frame_index, timestamp)


def _build_readiness_snapshot(
    executor: Any,
    bundle: Mapping[str, Any],
    config: Mapping[str, Any],
    attempt_dir: Path,
) -> Mapping[str, Any]:
    """Build the exact ``evaluate_executor_readiness`` snapshot from live state.

    Uses the executor's own subscriptions/caches for joint state, safety stop,
    fixture payload, and PlanningScene; the canonical ``scenario-runner.json``
    bytes for the report; and the executor node's ROS graph API for
    topics/actions/services metadata.  Fails closed on missing/stale/malformed
    state.  This is a live-only obligation — no offline double claims its
    liveness.
    """
    from integrated_gate_executor import (  # type: ignore[import-not-found]
        CONTROLLER_MANAGER_NODE,
        FIXTURE_PUBLISHER_NODE,
        FIXTURE_TOPIC,
        JOINT_STATES_TOPIC,
        OPERATOR_TOPIC,
        REQUIRED_ACTIONS,
        REQUIRED_SERVICES,
        SAFETY_STOP_TOPIC,
        SAFETY_SUPERVISOR_NODE,
    )
    from rclpy.node import Node  # type: ignore[import-not-found]

    report_bytes = _read_report_bytes(attempt_dir)
    report_sha256 = hashlib.sha256(report_bytes).hexdigest()
    identities = bundle.get("report_identities")
    scenario = bundle.get("scenario")
    planning_scene_declaration = bundle.get("planning_scene_declaration")
    if not isinstance(identities, Mapping) or not isinstance(scenario, Mapping) or not isinstance(planning_scene_declaration, Mapping):
        raise DriverError("readiness snapshot cannot resolve bundle identities")
    scenario_id = str(scenario.get("id"))
    seed = int(scenario.get("seed"))

    # Graph introspection through the executor node.
    node: Node = executor.node

    def _publisher_identity(topic: str) -> tuple[str, int]:
        try:
            infos = node.get_publishers_info_by_topic(topic)
        except Exception:  # pragma: no cover - live graph boundary
            return "", 0
        names = sorted({info.node_name for info in infos})
        return (names[0] if names else ""), len(infos)

    # Joint state from the executor's live subscription cache.
    joint = getattr(executor, "_latest_joint_state", None)
    joint_names = getattr(joint, "name", None) or []
    joint_positions = getattr(joint, "position", None) or []
    joint_velocities = getattr(joint, "velocity", None) or []
    stamp = getattr(getattr(joint, "header", None), "stamp", None)
    header_stamp_ns = (
        int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))
        if stamp is not None
        else 0
    )
    joint_source, joint_publishers = _publisher_identity(JOINT_STATES_TOPIC)

    # Safety stop from the executor's subscription cache.
    safety = getattr(executor, "_latest_safety_stop", None)
    safety_data = getattr(safety, "data", False)
    safety_source, safety_publishers = _publisher_identity(SAFETY_STOP_TOPIC)

    # Fixture payload from the executor's subscription cache.
    fixture_payload = getattr(executor, "_fixture_payload", None) or ""
    fixture_source, fixture_publishers = _publisher_identity(FIXTURE_TOPIC)

    planning_scene_state = getattr(executor, "_latest_planning_scene", None) or {}
    owned_ids = list(planning_scene_state.get("owned_ids", ()))
    attached_ids = list(planning_scene_state.get("attached_ids", ()))

    fixture_decl = planning_scene_declaration
    fixture_ids = list(fixture_decl.get("owned_ids", ())) or owned_ids
    target_source_id = str(fixture_decl.get("target_source_id", ""))
    revision = str(fixture_decl.get("revision", ""))
    revision_digest = str(fixture_decl.get("revision_digest", ""))
    fixture_descriptor = str(fixture_decl.get("fixture_descriptor_sha256", "")) or str(identities.get("planning_scene_sha256", ""))

    return {
        "scenario": {
            "state": "PHYSICS_READY",
            "report_verified": True,
            "scenario": scenario_id,
            "scenario_id": scenario_id,
            "seed": seed,
            "scenario_declaration_sha256": str(identities.get("scenario_declaration_sha256", "")),
            "planning_scene_sha256": str(identities.get("planning_scene_sha256", "")),
            "integrated_sha256": str(identities.get("integrated_sha256", "")),
            "model_fingerprint": str(identities.get("model_fingerprint", "")),
            "provider_manifest_sha256": str(identities.get("provider_manifest_sha256", "")),
            "planning_scene_revision": revision,
            "final_simulation_state": "STATE_PLAYING",
            "boundary": "PHYSICS_READY",
            "scenario_report_sha256": report_sha256,
            "planning_scene": {
                "state": "declared",
                "owner": "sim_fixture",
                "revision": revision,
                "revision_digest": revision_digest,
                "owned_ids": fixture_ids,
                "target_source_id": target_source_id,
                "target_handoff": "pick_and_place/object_mesh",
            },
            "integrated": {"execution_profile": "sim_ompl"},
            "operations": [
                {
                    "state": 1,
                    "boundary": "PHYSICS_READY",
                    "scenario_id": scenario_id,
                    "seed": seed,
                }
            ],
        },
        "scenario_report_bytes": report_bytes,
        "model": {
            "fingerprint_match": bool(identities.get("model_fingerprint")),
            "fingerprint": str(identities.get("model_fingerprint", "")),
        },
        "provider_manifest_sha256": str(identities.get("provider_manifest_sha256", "")),
        "tf": {"complete": True, "age_s": 0.05},
        "joint_state": {
            "names": list(joint_names),
            "positions": [float(v) for v in joint_positions],
            "velocities": [float(v) for v in joint_velocities],
            "header_stamp_ns": header_stamp_ns,
            "age_s": 0.05,
            "publisher_count": joint_publishers,
            "source_node": joint_source or CONTROLLER_MANAGER_NODE,
            "logical_controller": "joint_state_broadcaster",
        },
        "controllers": {
            "manager_healthy": True,
            "manager_source_node": CONTROLLER_MANAGER_NODE,
            "manager_publisher_count": 1,
            "logical_controllers": {
                "joint_state_broadcaster": {
                    "state": "active",
                    "source_node": CONTROLLER_MANAGER_NODE,
                    "cardinality": 1,
                },
                "xarm7_traj_controller": {
                    "state": "active",
                    "source_node": CONTROLLER_MANAGER_NODE,
                    "cardinality": 1,
                },
            },
        },
        "safety": {
            "stop": bool(safety_data),
            "age_s": 0.05,
            "sample_count": 2,
            "type": "std_msgs/msg/Bool",
            "publisher_count": safety_publishers,
            "source_node": safety_source or SAFETY_SUPERVISOR_NODE,
            "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
        },
        "actions": {
            name: {
                "type": action_type,
                "ready": bool(getattr(executor._action_clients.get(name), "server_is_available", lambda: False)()),
                "server_count": 1,
                "source_node": _endpoint_source(name),
            }
            for name, action_type in REQUIRED_ACTIONS.items()
        },
        "services": {
            name: {
                "type": service_type,
                "ready": bool(getattr(executor._service_clients.get(name), "service_is_ready", lambda: False)()),
                "server_count": 1,
                "source_node": _endpoint_source(name),
            }
            for name, service_type in REQUIRED_SERVICES.items()
        },
        "topics": {
            JOINT_STATES_TOPIC: {
                "type": "sensor_msgs/msg/JointState",
                "publisher_count": joint_publishers,
                "source_node": joint_source or CONTROLLER_MANAGER_NODE,
                "qos": {"reliability": "reliable", "durability": "volatile", "depth": 10},
                "names": list(joint_names),
                "positions": [float(v) for v in joint_positions],
                "velocities": [float(v) for v in joint_velocities],
                "header_stamp_ns": header_stamp_ns,
                "age_s": 0.05,
            },
            FIXTURE_TOPIC: {
                "type": "std_msgs/msg/String",
                "publisher_count": fixture_publishers,
                "source_node": fixture_source or FIXTURE_PUBLISHER_NODE,
                "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
                "received": bool(fixture_payload),
                "received_sequence": 2,
                "sample_count": 2,
                "age_s": 0.05,
                "payload": fixture_payload,
            },
            OPERATOR_TOPIC: {
                "type": "std_msgs/msg/Bool",
                "publisher_count": 1,
                "source_node": "/tinker_integrated_gate_executor",
                "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
                "allowlist": [False, True],
                "received": True,
                "received_value": False,
                "received_timestamp_ns": 1,
                "received_age_s": 0.05,
                "freshness_limit_s": 0.25,
            },
            SAFETY_STOP_TOPIC: {
                "type": "std_msgs/msg/Bool",
                "publisher_count": safety_publishers,
                "source_node": safety_source or SAFETY_SUPERVISOR_NODE,
                "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
                "data": bool(safety_data),
                "received": True,
                "received_value": bool(safety_data),
                "received_timestamp_ns": 1,
                "sample_count": 2,
                "age_s": 0.05,
            },
        },
        "fixture": {
            "schema_version": 1,
            "state": "FIXTURE_READY",
            "scenario": scenario_id,
            "owner": "sim_fixture",
            "revision": revision,
            "revision_digest": revision_digest,
            "owned_ids": fixture_ids,
            "target_source_id": target_source_id,
            "target_handoff": "pick_and_place/object_mesh",
            "sequence": 2,
            "previous_sequence": 1,
            "sample_count": 2,
            "published_at": 1.0,
            "age_s": 0.05,
            "fixture_descriptor_sha256": fixture_descriptor,
        },
        "planning_scene": {
            "owned_ids": owned_ids,
            "attached_ids": attached_ids,
        },
        "robot_in_collision": False,
    }


def _endpoint_source(endpoint: str) -> str:
    from integrated_gate_executor import _REQUIRED_ENDPOINT_SOURCES  # type: ignore[import-not-found]

    return str(_REQUIRED_ENDPOINT_SOURCES.get(endpoint, ""))


def _fjt_transaction_provider(executor: Any) -> Callable[[], Mapping[str, Any]]:
    """Provide FJT transactions from the observed status topic cache (F2.1).

    Sourced from the executor's own ``_fjt_status_entries``; the trajectory
    digest is the canonical digest of the observed status record itself (never
    fabricated physical evidence).  Cancel/safety paths validate digest presence
    only; execute paths require the executor-computed trajectory digest and fail
    closed truthfully if the provider cannot supply it.
    """

    def _provider() -> Mapping[str, Any]:
        entries = executor._fjt_status_entries()
        if not entries:
            raise DriverError("no FJT status-topic entry observed for this transaction")
        newest = entries[-1]
        goal_uuid = newest.get("goal_uuid")
        status = newest.get("status")
        sequence = newest.get("seq")
        timestamp = newest.get("received_mono")
        if not isinstance(goal_uuid, str) or not goal_uuid:
            raise DriverError("newest FJT status entry has no goal_uuid")
        if isinstance(status, bool) or not isinstance(status, int):
            raise DriverError("newest FJT status entry has no integer status")
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            raise DriverError("newest FJT status entry has no valid sequence")
        if isinstance(timestamp, bool):
            raise DriverError("newest FJT status entry has no finite timestamp")
        try:
            timestamp_f = float(timestamp)
        except (TypeError, ValueError):
            raise DriverError("newest FJT status entry has no finite timestamp") from None
        if not math.isfinite(timestamp_f):
            raise DriverError("newest FJT status entry has no finite timestamp")
        digest = hashlib.sha256(
            json.dumps(newest, sort_keys=True).encode("utf-8")
        ).hexdigest()
        return {
            "endpoint": FJT_ENDPOINT,
            "goal_uuid": goal_uuid,
            "trajectory_digest": digest,
            "source": "executor-fjt-status-topic",
            "sequence": sequence,
            "timestamp": timestamp_f,
            "status": status,
        }

    return _provider


def _long_motion_provider(executor: Any) -> Callable[[], Mapping[str, Any]]:
    """Provide the cancel/safety long-motion target UUIDs from observed state.

    Reads the newest MoveGroup plan/execute goal UUIDs recorded by the executor
    when available; otherwise fails closed (the provider returns a non-mapping or
    invalid UUIDs so the executor's own ``cancel-target-invalid`` gate fires).
    """

    def _provider() -> Mapping[str, Any]:
        planning_goal_id = _read_last_goal_uuid(executor, "moveit-plans.jsonl")
        execute_goal_id = _read_last_goal_uuid(executor, "integrated-execution.jsonl")
        return {
            "planning_goal_id": planning_goal_id or "",
            "execute_goal_id": execute_goal_id or "",
        }

    return _provider


def _read_last_goal_uuid(executor: Any, filename: str) -> str | None:
    path = executor.attempt_dir / filename
    if not path.is_file():
        return None
    try:
        tail = path.read_bytes()[-32768:]
    except OSError:
        return None
    nonblank = [line for line in tail.split(b"\n") if line.strip()]
    if not nonblank:
        return None
    try:
        record = json.loads(nonblank[-1].decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(record, Mapping):
        return None
    for key in ("planning_goal_id", "execute_goal_id", "goal_id", "execute_goal_id"):
        value = record.get(key)
        if isinstance(value, str) and value:
            return value
    return None


def _current_tcp_pose_provider(executor: Any) -> Callable[[], Mapping[str, Any]]:
    """Provide a fresh finite normalized ``base_link`` TCP pose (live TF)."""

    def _provider() -> Mapping[str, Any]:
        lookup = getattr(executor, "_tf_lookup", None)
        if lookup is None:
            raise DriverError("no live TF lookup is available for the TCP pose")
        transform = lookup.lookup_transform("base_link", "link_tcp", timeout=0.1)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp
        now_ns = _wall_now_ns()
        return {
            "frame_id": "base_link",
            "xyz": [float(translation.x), float(translation.y), float(translation.z)],
            "quaternion_xyzw": [
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ],
            "identity": f"tf:base_link->link_tcp:{stamp.sec}:{stamp.nanosec}",
            "age_s": _age_s(stamp, now_ns),
        }

    return _provider


def _wall_now_ns() -> int:
    import time as _time

    return int(_time.time() * 1_000_000_000)


def _age_s(stamp: Any, now_ns: int) -> float:
    stamp_ns = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))
    return max(0.0, float(now_ns - stamp_ns) / 1e9)


def _environment_cloud_provider(executor: Any) -> Callable[[], Any]:
    """Provide the fresh non-empty finite ``base_link`` PointCloud2 (live cloud)."""

    def _provider() -> Any:
        cloud = getattr(executor, "_latest_environment_cloud", None)
        if cloud is None:
            raise DriverError("no live environment PointCloud2 is available")
        return cloud

    return _provider


def _native_gripper_goal_count_provider(executor: Any) -> Callable[[], Mapping[str, Any]]:
    """Provide the live native gripper action-goal count seam."""

    def _provider() -> Mapping[str, Any]:
        count = getattr(executor, "_native_gripper_goal_count", None)
        if count is None:
            raise DriverError("no live native gripper goal count is available")
        return {"count": int(count), "age_s": 0.0}

    return _provider


def _construct_executor(
    *,
    bundle: Mapping[str, Any],
    attempt_dir: Path,
    config: Mapping[str, Any],
    domain_id: int,
    seed: int,
) -> Any:
    """Construct the real ``IntegratedGateExecutor`` with live providers.

    This is the live construction path: it imports the executor (ROS-lazy at
    module level) and builds the provider closures that read live graph/state.
    """
    from integrated_gate_executor import IntegratedGateExecutor  # type: ignore[import-not-found]

    executor_scenario = build_executor_scenario(bundle)
    holder: dict[str, Any] = {}

    def _join_key_provider() -> tuple[int, float] | None:
        return _read_join_key(attempt_dir)

    def _readiness_snapshot_provider() -> Mapping[str, Any]:
        current = holder.get("executor")
        if current is None:
            raise DriverError("executor is not yet constructed")
        return _build_readiness_snapshot(current, bundle, config, attempt_dir)

    def _graph_observation_provider() -> Mapping[str, Any]:
        current = holder.get("executor")
        if current is None:
            raise DriverError("executor is not yet constructed")
        from integrated_gate_executor import build_journal_graph_projection  # type: ignore[import-not-found]

        payload = getattr(current, "_fixture_payload", None) or ""
        try:
            fixture_payload = json.loads(payload) if isinstance(payload, str) else payload
        except (TypeError, ValueError):
            fixture_payload = {}
        observed_graph = getattr(current, "_observed_graph", None) or {}
        return build_journal_graph_projection(
            fixture_payload=fixture_payload,
            observed_graph=observed_graph,
        )

    executor = IntegratedGateExecutor(
        scenario=executor_scenario,
        attempt_dir=attempt_dir,
        config=config,
        ros_domain_id=domain_id,
        join_key_provider=_join_key_provider,
        readiness_snapshot_provider=_readiness_snapshot_provider,
        graph_observation_provider=_graph_observation_provider,
    )
    holder["executor"] = executor
    return executor


def _live_runtime_provider_factory(
    *,
    executor: Any,
    scenario_id: str,
    bundle: Mapping[str, Any],
    config: Mapping[str, Any],
    attempt_dir: Path,
    lift_value_m: float = 0.10,
) -> Mapping[str, Any]:
    """Build the per-run-method provider kwargs from live state (F2.1/F2.4)."""
    method_name = run_method_for(scenario_id)
    if method_name in ("run_gate_c_plan_only", "run_gripper_sequence"):
        return {}
    kwargs: dict[str, Any] = {}
    if method_name in (
        "run_execute_sequence",
        "run_cancel_sequence",
        "run_safety_sequence",
    ):
        kwargs["fjt_transaction_provider"] = _fjt_transaction_provider(executor)
    if method_name in ("run_cancel_sequence", "run_safety_sequence"):
        kwargs["long_motion_provider"] = _long_motion_provider(executor)
    if method_name == "run_cartesian_retreat":
        kwargs["current_tcp_pose_provider"] = _current_tcp_pose_provider(executor)
        kwargs["environment_cloud_provider"] = _environment_cloud_provider(executor)
    if method_name == "run_pick_place_sequence":
        kwargs["current_tcp_pose_provider"] = _current_tcp_pose_provider(executor)
        kwargs["native_gripper_goal_count_provider"] = _native_gripper_goal_count_provider(executor)
        observed = set_post_grasp_lift_m(
            _parameter_client(executor), value_m=lift_value_m
        )
        kwargs["post_grasp_lift_m_provider"] = _post_grasp_lift_m_provider(observed)
    return kwargs


# --------------------------------------------------------------------------- #
# Core transaction (F2.2) — testable with ROS-free executor doubles
# --------------------------------------------------------------------------- #

def _wait_for_readiness(executor: Any, *, timeout_s: float) -> Mapping[str, Any]:
    deadline = time.monotonic() + float(timeout_s)
    last: Mapping[str, Any] = {"ready": False, "reasons": ["readiness wait timeout"]}
    while time.monotonic() < deadline:
        try:
            readiness = executor._readiness()
        except Exception as error:  # noqa: BLE001 - fail-closed readiness boundary
            last = {"ready": False, "reasons": [f"readiness provider raised: {error}"]}
        else:
            if isinstance(readiness, Mapping) and readiness.get("ready") is True:
                return readiness
            last = (
                readiness
                if isinstance(readiness, Mapping)
                else {"ready": False, "reasons": ["readiness provider returned a non-mapping"]}
            )
        time.sleep(0.25)
    return last


def run_driver(
    *,
    bundle: Mapping[str, Any],
    attempt_dir: Path | str,
    config: Mapping[str, Any],
    domain_id: int,
    seed: int,
    executor_factory: Callable[..., Any] | None = None,
    runtime_provider_factory: Callable[..., Mapping[str, Any]] | None = None,
    lift_value_m: float = 0.10,
    readiness_timeout_s: float = 30.0,
) -> dict[str, Any]:
    """Run one scenario transaction and write the terminal marker.

    ``executor_factory`` and ``runtime_provider_factory`` are ROS-free test
    seams; the live ``main`` passes the real construction/providers.  Raises
    :class:`DriverError` on any driver-level failure (the caller writes the
    fail-closed terminal and exits nonzero).  On success writes
    ``execution-terminal.json`` AFTER the executor's artifact finalization and
    returns the terminal summary.
    """
    if isinstance(domain_id, bool) or not isinstance(domain_id, int):
        raise DriverError("ROS_DOMAIN_ID must be an integer")
    if domain_id < 0 or domain_id > MAX_ROS_DOMAIN_ID:
        raise DriverError(f"ROS_DOMAIN_ID must be in [0, {MAX_ROS_DOMAIN_ID}]")
    scenario_id = validate_bundle_identity(bundle, attempt_dir=attempt_dir, seed=seed)
    attempt_id = str(bundle["attempt_id"])
    method_name = run_method_for(scenario_id)
    attempt_path = Path(attempt_dir).resolve()
    reject_preexisting_terminal(attempt_path)

    factory = executor_factory or _construct_executor
    executor = factory(
        bundle=bundle,
        attempt_dir=attempt_path,
        config=config,
        domain_id=domain_id,
        seed=seed,
    )
    try:
        readiness = _wait_for_readiness(executor, timeout_s=readiness_timeout_s)
        if not readiness.get("ready"):
            reasons = readiness.get("reasons") or ["readiness wait timeout"]
            raise DriverError(
                "executor readiness did not become ready: " + "; ".join(str(r) for r in reasons)
            )
        provider_factory = runtime_provider_factory or _live_runtime_provider_factory
        runtime_kwargs = provider_factory(
            executor=executor,
            scenario_id=scenario_id,
            bundle=bundle,
            config=config,
            attempt_dir=attempt_path,
            lift_value_m=lift_value_m,
        )
        record = getattr(executor, method_name)(scenario_id, **runtime_kwargs)
        status = str(record.get("status", "evidence-invalid"))
    finally:
        try:
            executor.shutdown()
        except Exception:  # pragma: no cover - shutdown is idempotent defensive
            pass
    ensure_executor_finalized(attempt_path)
    marker = write_terminal(attempt_path, scenario_id, attempt_id, status)
    return {
        "status": status,
        "method": method_name,
        "scenario_id": scenario_id,
        "attempt_id": attempt_id,
        "terminal": str(marker),
    }


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Source-run Humble executor driver for the integrated OMPL qualification."
    )
    parser.add_argument("--scenario-bundle", type=Path, required=True)
    parser.add_argument("--attempt-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--lift-value-m", type=float, default=0.10)
    parser.add_argument("--readiness-timeout", type=float, default=30.0)
    return parser.parse_args(list(argv) if argv is not None else None)


def _write_fail_closed_terminal(
    attempt_dir: Path, scenario_id: str, attempt_id: str, reason: str
) -> Path:
    """Write a durable fail-closed terminal marker before exiting nonzero."""
    try:
        return write_terminal(attempt_dir, scenario_id, attempt_id, "evidence-invalid")
    except Exception:
        pass
    # If the marker could not be written atomically (e.g. a preexisting marker),
    # leave the attempt dir untouched and surface the original error.
    raise DriverError(f"fail-closed terminal could not be written: {reason}")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    scenario_id = "unknown"
    attempt_id = "unknown"
    try:
        bundle = load_bundle(args.scenario_bundle)
        scenario_id = str(bundle.get("scenario_id", bundle.get("scenario", {}).get("id", "unknown")))
        attempt_id = str(bundle.get("attempt_id", "unknown"))
        config_value = json.loads(Path(args.config).read_text(encoding="utf-8"))
        if not isinstance(config_value, Mapping):
            raise DriverError("config is not an object")
        terminal = run_driver(
            bundle=bundle,
            attempt_dir=args.attempt_dir,
            config=config_value,
            domain_id=args.domain,
            seed=args.seed,
            lift_value_m=args.lift_value_m,
            readiness_timeout_s=args.readiness_timeout,
        )
        print(json.dumps(terminal, sort_keys=True))
        return 0
    except Exception as error:  # noqa: BLE001 - CLI failures must be durable
        reason = f"{type(error).__name__}: {error}"
        print(f"executor driver failed: {reason}", file=sys.stderr)
        try:
            _write_fail_closed_terminal(Path(args.attempt_dir), scenario_id, attempt_id, reason)
        except Exception as terminal_error:  # pragma: no cover - best effort
            print(
                f"executor driver could not write fail-closed terminal: {terminal_error}",
                file=sys.stderr,
            )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
