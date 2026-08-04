#!/usr/bin/env python3
"""Source-run Humble executor driver for the integrated OMPL qualification.

Task 8 fix round 3 (F3.1-F3.12): this module is the live producer of the
executor evidence and the scenario terminal marker.  It runs as a third owned
child of ``IntegratedRunner`` (launched only after the overlay has produced
canonical PHYSICS_READY), constructs the real
:class:`~integrated_gate_executor.IntegratedGateExecutor` for the current
immutable attempt, drives a driver-owned observer node on the executor's
private rclpy context, waits for executor-schema readiness from live
observations (never constants), sets and reads back the production
``/pick_and_place.post_grasp_lift_m`` runtime parameter over real
``rcl_interfaces`` services, dispatches exactly one of the 17 canonical
scenarios, and writes ``execution-terminal.json`` only after the executor's own
artifact finalization has completed.

ROS-lazy: importing this module under the simulator CPython 3.12 venv never
imports ``rclpy`` or any generated message type.  All ROS imports (rclpy,
generated messages, the real executor, tf2, rcl_interfaces) live inside
:func:`main` / ``_construct_executor`` / the observer and recorder classes.
The pure dispatch/serialization/bundle/terminal/lift layer below is importable
and unit-tested under Python 3.12.

The driver never fabricates readiness or provider values.  Missing, stale,
malformed, or contradictory provider data fails closed.  The independent
verifier remains authoritative: on a driver-level failure the driver writes a
durable fail-closed ``execution-terminal.json`` (``status`` ``evidence-invalid``)
and exits nonzero; it never synthesizes passing physical evidence.

Option A+ environment cloud (fix-3 coordinator decision): for integrated
qualification only, ``validation/run_sim.py`` populates ``backend.occupancy``
from the committed scenario ``planning_scene.objects`` footprint geometry and
enables ``development_lidar``; the bridge launch owns a static
``base_link -> livox360`` transform.  The driver subscribes to the real
``/livox/lidar`` PointCloud2 (sensor-data QoS), transforms each non-empty cloud
with ``tf2_sensor_msgs.do_transform_cloud(lookup_transform("base_link", ...))``,
and returns the ``base_link`` cloud to ``run_cartesian_retreat``.  That cloud is
scenario-derived planning input, not independent raw-physics truth; the Task-7
raw/evaluator verifier remains the sole physical authority.
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
import uuid as _uuid
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
#: full required set and is authoritative.
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

#: Humble never reports publisher/consumer depth (always 0); reliability and
#: durability are observable.  The contract depth is asserted only after a
#: compatible live sample has been received on the topic (see ``_build_readiness_snapshot``).
#: This mirrors the production ``integrated_readiness`` depth handling.
_CONTRACT_DEPTH_BY_TOPIC: Mapping[str, int] = {
    "/joint_states": 10,
    "/sim/status/planning_scene_fixture": 1,
    "/sim/safety/operator": 1,
    "/sim/hardware/safety_stop": 1,
}

#: Wall-clock window (seconds) within which a live ``/livox/lidar`` cloud is
#: considered fresh.  The executor itself does not age-gate the cloud
#: (``_env_cloud_evidence`` is structural only), so the driver is the freshness
#: gate.  5.0 s comfortably covers the 10 Hz lidar plus a dropped frame.
ENV_CLOUD_MAX_AGE_S = 5.0

#: The neutral-ish long-motion joint target pre-sent for the cancel/safety
#: scenarios.  A real accepted ExecuteTrajectory goal handle is retained and
#: passed to the immutable run method; the UUIDs/digest come from the in-memory
#: action recorders.
_LONG_MOTION_JOINT_TARGET = (0.0, -0.3, 0.0, 0.6, 0.0, 0.3, 0.0)

#: rcl_interfaces ``ParameterType.PARAMETER_DOUBLE``.
_PARAMETER_DOUBLE = 3

#: Wall-clock bound for a single ``ListControllers`` query from the readiness
#: snapshot.  The controller manager responds on the shared spinner within
#: ~100 ms; a 1.0 s bound keeps a dead manager from stalling readiness.
_CONTROLLER_QUERY_TIMEOUT_S = 1.0

#: Wall-clock bound for each ``/pick_and_place`` parameter set/get transaction.
_PARAMETER_SERVICE_TIMEOUT_S = 5.0


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
    integrated-ompl thresholds (15/120/10/10) this is exactly ``305.0`` s.
    Every term must be finite and positive; malformed config fails closed.
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
    """Return the exact 17-scenario dispatch table (id -> executor run method)."""
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
    """Return the executor run-method name for a canonical scenario id."""
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
    """Validate the bundle binds the current immutable attempt; return scenario id."""
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
    """Build the executor-schema scenario mapping from the orchestrator bundle."""
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
    """Atomically write ``execution-terminal.json`` after executor finalization."""
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
    """Verify the executor's own terminal summary exists before the driver marker."""
    summary = Path(attempt_dir) / "integrated-execution.json"
    if not summary.is_file():
        raise DriverError(
            "executor did not finalize integrated-execution.json; refusing to "
            "write the driver terminal marker"
        )


# --------------------------------------------------------------------------- #
# post_grasp_lift_m runtime parameter (F3.7) — hermetic ROS-free client protocol
# --------------------------------------------------------------------------- #

def _double_parameter(name: str, value: float) -> dict[str, object]:
    """Return the pure double-parameter record (never an rclpy object).

    F3.7: the pure layer is permanently pure and always returns a plain dict so
    its behavior cannot depend on polluted ``sys.modules`` (fix2 Me-1).  The ROS
    ``rcl_interfaces`` conversion happens only inside the live service client.
    """
    return {"name": str(name), "value": float(value)}


def _set_result_ok(result: object) -> bool:
    """Duck-typed ``SetParameters.Response.results[0].successful`` reader.

    Accepts a single ``SetParametersResult``, a list/tuple of them, or a plain
    record (the ROS-free double).  Missing/rejected results fail closed.
    """
    if isinstance(result, (list, tuple)):
        if not result:
            return False
        result = result[0]
    successful = getattr(result, "successful", None)
    if successful is not None:
        return bool(successful)
    if isinstance(result, Mapping):
        return bool(result.get("successful", False))
    return bool(result)


def _extract_double(result: object, name: str) -> float | None:
    """Extract a finite DOUBLE from a ``GetParameters.Response`` (or double).

    F3.7: the live response exposes ``values: sequence<ParameterValue>``; a
    DOUBLE requires ``values[0].type == PARAMETER_DOUBLE`` and a finite
    ``double_value``.  Plain records (the ROS-free doubles) are also accepted.
    """
    if result is None:
        return None
    values = getattr(result, "values", None)
    if values is None and isinstance(result, Mapping):
        values = result.get("values")
    if isinstance(values, (list, tuple)) and len(values) == 1:
        value = values[0]
    else:
        value = result
    param_type = getattr(value, "type", None)
    if param_type is not None:
        try:
            if int(param_type) != _PARAMETER_DOUBLE:
                return None
        except (TypeError, ValueError):
            return None
        number = getattr(value, "double_value", None)
    elif isinstance(value, Mapping):
        number = value.get("double_value")
        if number is None:
            number = value.get("value")
    else:
        number = getattr(value, "double_value", None)
        if number is None:
            number = getattr(value, "value", None)
    if isinstance(number, bool):
        return None
    try:
        number = float(number)
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

    Requires a successful parameter set result and an exact/tolerance-consistent
    finite read-back ``>= value_m``.  Returns the observed read-back metadata.
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


def _post_grasp_lift_m_provider(
    observed: Mapping[str, Any],
) -> Callable[[], Mapping[str, Any]]:
    """Return a fresh typed provider returning the observed read-back.

    The ``age_s: 0.0`` is the age of the just-observed parameter transaction,
    not a fabricated constant: the value was set and read back immediately
    before dispatch.
    """

    def _provider() -> Mapping[str, Any]:
        return {
            "value_m": float(observed["value_m"]),
            "identity": str(observed["identity"]),
            "age_s": 0.0,
        }

    return _provider


def _call_service_with_spinner(
    executor: Any,
    client: Any,
    request: Any,
    *,
    timeout_s: float,
) -> Any | None:
    """Call a ServiceClient while driving the shared executor/observer spinner.

    F3.1/F3.7: observer and parameter clients live on the executor's private
    rclpy context, and each response is delivered to the owning node's wait set.
    ``Client.call`` and ``spin_until_future_complete(node, ...)`` spin only that
    one node, so the shared spinner's other nodes are never serviced and the
    call can hang indefinitely.  Driving ``executor._spin_once()`` (which
    services the executor AND observer nodes on the shared spinner) while
    polling the async future delivers the response reliably.  Returns the
    response object or ``None`` on unavailable/timeout/failure (fail closed).
    """
    try:
        if not client.service_is_ready():
            return None
        future = client.call_async(request)
    except Exception:  # noqa: BLE001 - fail-closed service boundary
        return None
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        if future.done():
            try:
                return future.result()
            except Exception:  # noqa: BLE001 - fail-closed service boundary
                return None
        try:
            executor._spin_once()
        except Exception:  # noqa: BLE001 - fail-closed service boundary
            pass
        time.sleep(0.01)
    return None


class _ParameterServiceClient:
    """Live ``rcl_interfaces`` set/get parameter client for ``/pick_and_place``.

    F3.7: Humble rclpy ships no remote ``ParameterClient``; the idiom is a
    ``ServiceClient`` to ``/pick_and_place/set_parameters`` and
    ``/get_parameters``.  Built on the executor so the parameter transaction
    shares the qualification graph, lifetime, and the shared spinner (responses
    are delivered by driving ``executor._spin_once()``).
    """

    def __init__(self, executor: Any) -> None:
        from rcl_interfaces.srv import GetParameters, SetParameters

        self._executor = executor
        self._node = executor.node
        self._set = self._node.create_client(SetParameters, "/pick_and_place/set_parameters")
        self._get = self._node.create_client(GetParameters, "/pick_and_place/get_parameters")

    def wait_for_service(self, timeout_sec: float) -> bool:
        try:
            set_ok = self._set.wait_for_service(timeout_sec=float(timeout_sec))
        except Exception:  # pragma: no cover - live client boundary
            set_ok = False
        try:
            get_ok = self._get.wait_for_service(timeout_sec=float(timeout_sec))
        except Exception:  # pragma: no cover - live client boundary
            get_ok = False
        return bool(set_ok and get_ok)

    def set_parameters(self, params: Sequence[Any]) -> Any:
        from rcl_interfaces.msg import Parameter as ParameterMsg
        from rcl_interfaces.msg import ParameterValue
        from rcl_interfaces.msg import ParameterType
        from rclpy.parameter import Parameter

        request = self._set.srv_type.Request()
        request.parameters = []
        for param in params:
            if isinstance(param, Mapping):
                msg = ParameterMsg()
                msg.name = str(param.get("name", ""))
                msg.value = ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(param.get("value", 0.0)),
                )
            else:
                msg = (
                    param.to_parameter_msg()
                    if hasattr(param, "to_parameter_msg")
                    else Parameter(param.name, param.type, param.value).to_parameter_msg()
                )
            request.parameters.append(msg)
        response = _call_service_with_spinner(
            self._executor, self._set, request, timeout_s=_PARAMETER_SERVICE_TIMEOUT_S
        )
        return getattr(response, "results", None)

    def get_parameters(self, names: Sequence[str]) -> Any:
        request = self._get.srv_type.Request()
        request.names = list(names)
        # Return the full ``GetParameters.Response`` (with its ``values``
        # sequence); ``_extract_double`` reads ``values[0]`` off the response.
        return _call_service_with_spinner(
            self._executor, self._get, request, timeout_s=_PARAMETER_SERVICE_TIMEOUT_S
        )


# --------------------------------------------------------------------------- #
# Live observer + recorder layer (F3.1, F3.4) — live-only
# --------------------------------------------------------------------------- #

def _endpoint_label(node_name: str, node_namespace: str | None = None) -> str:
    """Normalize a graph node identity to the canonical leading-slash form.

    Humble graph-cache node names omit the leading slash; this restores the
    canonical ``/name`` (or ``/ns/name``) identity the executor compares.
    """
    namespace = str(node_namespace or "")
    name = str(node_name or "")
    namespace = namespace.rstrip("/")
    if namespace:
        return f"{namespace}/{name}"
    return "/" + name


def _normalize_qos_value(value: Any) -> str:
    """Normalize a Humble QoS enum value to its short uppercase name."""
    text = str(value)
    if "." in text:
        text = text.rsplit(".", 1)[-1]
    return text.strip().upper()


def _qos_value_lower(value: Any) -> str:
    """Normalize a Humble QoS enum value to its short lowercase name."""
    return _normalize_qos_value(value).lower()


def _topic_qos_lower(qos_profile: Any) -> dict[str, object]:
    """Observed reliability/durability (lowercase) + contract depth.

    Humble ``PublishersInfo.qos_profile`` never reports depth (always 0); the
    driver observes reliability/durability and asserts the contract depth only
    after a compatible live sample has been received on the topic (checked by
    the caller), mirroring the production readiness depth handling.
    """
    return {
        "reliability": _qos_value_lower(getattr(qos_profile, "reliability", "")),
        "durability": _qos_value_lower(getattr(qos_profile, "durability", "")),
        "depth": int(getattr(qos_profile, "depth", 0)),
    }


class _LiveProviderObserver:
    """Driver-owned observer node in the executor's private rclpy context.

    F3.1: records latest message, wall-monotonic receipt time, sample count, and
    real endpoint metadata for every stream the readiness snapshot and runtime
    providers consume.  Hosts the TF buffer/listener, the controller-manager
    ``ListControllers`` client, and the ``/livox/lidar`` subscription.  The node
    is added to ``executor._spinner`` so one ``_spin_once()`` services both the
    executor and observer nodes.
    """

    def __init__(self, executor: Any, *, node_name: str = "tinker_integrated_gate_executor_observer") -> None:
        import tf2_ros
        from rclpy.duration import Duration
        from rclpy.node import Node as _Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from action_msgs.msg import GoalStatusArray
        from controller_manager_msgs.srv import ListControllers
        from sensor_msgs.msg import JointState, PointCloud2
        from std_msgs.msg import Bool, String
        from tf2_msgs.msg import TFMessage

        self._executor = executor
        self.context = executor.context
        self.node = _Node(
            node_name,
            namespace="/",
            cli_args=[],
            context=self.context,
            use_global_arguments=False,
        )
        self.install_mono = time.monotonic()

        # Per-stream trackers (message + receipt wall time + sample count).
        self.joint_received_mono: float | None = None
        self.joint_samples = 0
        self.safety_received_mono: float | None = None
        self.safety_samples = 0
        self.safety_value = False
        self.fixture_received_mono: float | None = None
        self.fixture_samples = 0
        self.fixture_payload = ""
        self.operator_received_mono: float | None = None
        self.operator_samples = 0
        self.operator_value = False
        self.collision_received_mono: float | None = None
        self.collision_samples = 0
        self.collision_value = False
        self.cloud_received_mono: float | None = None
        self.cloud_samples = 0
        self.latest_cloud: Any = None
        self.last_tf_received_mono: float | None = None
        self.gripper_status_received_mono: float | None = None

        def _on_joint(message: Any) -> None:
            self.joint_samples += 1
            self.joint_received_mono = time.monotonic()

        def _on_safety(message: Any) -> None:
            self.safety_samples += 1
            self.safety_received_mono = time.monotonic()
            self.safety_value = bool(getattr(message, "data", False))

        def _on_fixture(message: Any) -> None:
            self.fixture_samples += 1
            self.fixture_received_mono = time.monotonic()
            self.fixture_payload = str(getattr(message, "data", "") or "")

        def _on_operator(message: Any) -> None:
            self.operator_samples += 1
            self.operator_received_mono = time.monotonic()
            self.operator_value = bool(getattr(message, "data", False))

        def _on_collision(message: Any) -> None:
            self.collision_samples += 1
            self.collision_received_mono = time.monotonic()
            self.collision_value = bool(getattr(message, "data", False))

        def _on_cloud(message: Any) -> None:
            self.cloud_samples += 1
            self.cloud_received_mono = time.monotonic()
            self.latest_cloud = message

        def _on_tf(_message: Any) -> None:
            self.last_tf_received_mono = time.monotonic()

        def _on_gripper_status(_message: Any) -> None:
            self.gripper_status_received_mono = time.monotonic()

        joint_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        fixture_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        operator_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)

        self._subscriptions = [
            self.node.create_subscription(JointState, "/joint_states", _on_joint, joint_qos),
            self.node.create_subscription(Bool, "/sim/hardware/safety_stop", _on_safety, fixture_qos),
            self.node.create_subscription(String, "/sim/status/planning_scene_fixture", _on_fixture, fixture_qos),
            self.node.create_subscription(Bool, "/sim/safety/operator", _on_operator, operator_qos),
            self.node.create_subscription(Bool, "/sim/safety/collision", _on_collision, fixture_qos),
            self.node.create_subscription(PointCloud2, "/livox/lidar", _on_cloud, qos_profile_sensor_data),
            self.node.create_subscription(GoalStatusArray, "/xarm_gripper/gripper_action/_action/status", _on_gripper_status, fixture_qos),
            # Stock TF2 QoS: /tf is reliable/volatile (depth 100); /tf_static is
            # reliable/transient-local (depth 1) so late subscribers get the latched
            # static transforms.  Humble rclpy ships no qos_profile_tf constant.
            self.node.create_subscription(
                TFMessage, "/tf", _on_tf,
                QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE),
            ),
            self.node.create_subscription(
                TFMessage, "/tf_static", _on_tf,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
        ]

        # TF buffer/listener (spin_thread=False; driven by the shared spinner).
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10), node=self.node)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node, spin_thread=False)

        # Controller-manager ListControllers client (synchronous call in the
        # readiness snapshot).
        self._controllers_client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")

        # Share the executor spinner: one _spin_once() services both nodes.
        executor._spinner.add_node(self.node)

    def list_controllers(self) -> list[Any] | None:
        """Return the live ``ListControllers`` controller records or ``None``."""
        client = self._controllers_client
        request = client.srv_type.Request()
        response = _call_service_with_spinner(
            self._executor, client, request, timeout_s=_CONTROLLER_QUERY_TIMEOUT_S
        )
        if response is None:
            return None
        return list(getattr(response, "controller", ()))

    def destroy(self) -> None:
        """Remove the node from the spinner and destroy it (idempotent)."""
        spinner = getattr(self._executor, "_spinner", None)
        node = getattr(self, "node", None)
        if spinner is not None and node is not None:
            try:
                spinner.remove_node(node)
            except Exception:  # noqa: BLE001 - spinner may already be down
                pass
        listener = getattr(self, "tf_listener", None)
        if listener is not None:
            try:
                listener.unregister()
            except Exception:  # noqa: BLE001
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        self.node = None
        self.tf_listener = None


def _install_deterministic_serialize(executor: Any) -> None:
    """Make the executor's CDR digest computation deterministic (F3.4).

    Identical technique to ``_install_deterministic_serialize`` in the Task-4
    Humble suite: rclpy ``serialize_message`` writes uninitialized
    alignment-padding bytes, so the byte digest of a semantically identical
    message can differ between calls under memory churn.  This per-executor
    wrapper caches the first serialization of each message object and returns
    those exact bytes on later serializations of the same object, so the digest
    of the exact ExecuteTrajectory ``goal.trajectory`` captured by the execute
    recorder matches the executor's own ``executed_digest_after``.  The
    executor's ``self.ros`` dict is copied first so the shared module-level ROS
    import cache is never mutated.
    """
    executor.ros = dict(executor.ros)
    cache: dict[int, tuple[object, bytes]] = {}
    original = executor.ros["serialize_message"]

    def _deterministic_serialize(message: Any) -> bytes:
        key = id(message)
        cached = cache.get(key)
        if cached is not None:
            return cached[1]
        raw = original(message)
        cache[key] = (message, raw)
        return raw

    executor.ros["serialize_message"] = _deterministic_serialize


class _ActionClientRecorder:
    """Base wrapper preserving the ActionClient interface the executor uses.

    The executor re-fetches ``self._action_clients[...]`` per call and uses
    ``wait_for_server`` / ``send_goal_async`` / ``server_is_ready``, so a
    delegate wrapper is honored transparently.
    """

    def __init__(self, executor: Any, endpoint: str, real: Any) -> None:
        self._executor = executor
        self._real = real
        self.endpoint = endpoint

    def wait_for_server(self, timeout_sec: float | None = None) -> bool:
        return bool(self._real.wait_for_server(timeout_sec=float(timeout_sec)))

    def server_is_ready(self) -> bool:
        return bool(self._real.server_is_ready())

    def send_goal_async(self, goal: Any) -> Any:
        return self._real.send_goal_async(goal)


class _ExecuteTrajectoryRecorder(_ActionClientRecorder):
    """Wraps ``/execute_trajectory`` to capture the exact goal trajectory digest.

    F3.4: the FJT provider's ``trajectory_digest`` must be the SHA-256 of the
    exact ``ExecuteTrajectory.Goal.trajectory`` bytes, captured before
    delegation and equal to the executor's deterministic ``executed_digest_after``.
    """

    def __init__(self, executor: Any, real: Any) -> None:
        super().__init__(executor, "/execute_trajectory", real)
        self.last_trajectory_digest: str | None = None
        self.send_count = 0
        self.last_send_mono: float | None = None

    def send_goal_async(self, goal: Any) -> Any:
        self.send_count += 1
        self.last_send_mono = time.monotonic()
        trajectory = getattr(goal, "trajectory", None)
        if trajectory is not None:
            raw = self._executor.ros["serialize_message"](trajectory)
            self.last_trajectory_digest = hashlib.sha256(bytes(raw)).hexdigest()
        return self._real.send_goal_async(goal)


class _MoveActionRecorder(_ActionClientRecorder):
    """Wraps ``/move_action`` for planning transaction identity (long-motion setup)."""

    def __init__(self, executor: Any, real: Any) -> None:
        super().__init__(executor, "/move_action", real)
        self.send_count = 0
        self.last_send_mono: float | None = None

    def send_goal_async(self, goal: Any) -> Any:
        self.send_count += 1
        self.last_send_mono = time.monotonic()
        return self._real.send_goal_async(goal)


class _GripperRecorder(_ActionClientRecorder):
    """Wraps ``/xarm_gripper/gripper_action`` to count native gripper goals (F3.5)."""

    def __init__(self, executor: Any, real: Any) -> None:
        super().__init__(executor, "/xarm_gripper/gripper_action", real)
        self.goal_count = 0
        self.last_goal_mono: float | None = None
        self.install_mono = time.monotonic()

    def send_goal_async(self, goal: Any) -> Any:
        self.goal_count += 1
        self.last_goal_mono = time.monotonic()
        return self._real.send_goal_async(goal)


def _install_action_client_recorders(executor: Any) -> None:
    """Install the execute/move/gripper recorders into the executor's client map."""
    clients = executor._action_clients
    executor._execute_recorder = _ExecuteTrajectoryRecorder(executor, clients["/execute_trajectory"])
    executor._move_recorder = _MoveActionRecorder(executor, clients["/move_action"])
    executor._gripper_recorder = _GripperRecorder(executor, clients["/xarm_gripper/gripper_action"])
    clients["/execute_trajectory"] = executor._execute_recorder
    clients["/move_action"] = executor._move_recorder
    clients["/xarm_gripper/gripper_action"] = executor._gripper_recorder


# --------------------------------------------------------------------------- #
# File-derived helpers (readiness + runtime providers)
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


def _fresh_seconds(age: Any, limit: float) -> bool:
    try:
        age_f = float(age)
    except (TypeError, ValueError):
        return False
    return math.isfinite(age_f) and 0.0 <= age_f <= float(limit)


# --------------------------------------------------------------------------- #
# Readiness snapshot (F3.2) — every field observed, none fabricated
# --------------------------------------------------------------------------- #

def _parse_fixture_payload(payload: str) -> dict[str, Any] | None:
    try:
        parsed = json.loads(payload)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    return parsed if isinstance(parsed, Mapping) else None


def _publishers_for(node: Any, topic: str) -> tuple[list[str], list[Any]]:
    """Return (endpoint labels, info objects) for the publishers of a topic."""
    try:
        infos = list(node.get_publishers_info_by_topic(topic))
    except Exception:  # noqa: BLE001 - live graph boundary
        return [], []
    labels = [_endpoint_label(info.node_name, info.node_namespace) for info in infos]
    return labels, infos


def _subscribers_for(node: Any, topic: str) -> tuple[list[str], list[Any]]:
    try:
        infos = list(node.get_subscriptions_info_by_topic(topic))
    except Exception:  # noqa: BLE001 - live graph boundary
        return [], []
    labels = [_endpoint_label(info.node_name, info.node_namespace) for info in infos]
    return labels, infos


def _service_servers_and_clients(node: Any, service_name: str) -> tuple[list[str], list[str]]:
    """Return (server labels, client labels) hosting a named service."""
    servers: list[str] = []
    clients: list[str] = []
    try:
        pairs = list(node.get_node_names_and_namespaces())
    except Exception:  # noqa: BLE001 - live graph boundary
        return [], []
    for node_name, node_namespace in pairs:
        label = _endpoint_label(node_name, node_namespace)
        try:
            server_names = [n for n, _types in node.get_service_names_and_types_by_node(node_name, node_namespace)]
            if service_name in server_names:
                servers.append(label)
        except Exception:  # noqa: BLE001 - per-node boundary
            pass
        try:
            client_names = [n for n, _types in node.get_client_names_and_types_by_node(node_name, node_namespace)]
            if service_name in client_names:
                clients.append(label)
        except Exception:  # noqa: BLE001 - per-node boundary
            pass
    return servers, clients


def _action_servers_and_source(node: Any, action_name: str) -> tuple[int, str]:
    """Derive action server count + source from the send_goal service graph."""
    send_goal_service = f"{action_name}/_action/send_goal"
    servers, _clients = _service_servers_and_clients(node, send_goal_service)
    if not servers:
        return 0, ""
    return len(servers), servers[0]


def _service_servers_and_source(node: Any, service_name: str) -> tuple[int, str]:
    servers, _clients = _service_servers_and_clients(node, service_name)
    if not servers:
        return 0, ""
    return len(servers), servers[0]


def _build_readiness_snapshot(
    executor: Any,
    bundle: Mapping[str, Any],
    config: Mapping[str, Any],
    attempt_dir: Path,
) -> Mapping[str, Any]:
    """Build the exact ``evaluate_executor_readiness`` snapshot from live state.

    F3.2: every liveness field is read from the real caches/subscriptions/TF/
    service-graph — the observer's receipt times and sample counts, the
    executor's JointState/fixture/safety/PlanningScene caches, real
    ``get_publishers_info_by_topic`` / service discovery, real TF lookups, and a
    real ``publish_operator(False)`` baseline receipt.  No fixed ages, sample
    counts, sequences, timestamps, source fallbacks, or ``robot_in_collision``
    constant remain.  A missing observation either raises :class:`DriverError`
    or produces a snapshot value that ``evaluate_executor_readiness`` rejects.
    """
    from integrated_gate_executor import (  # noqa: F401 - live-only
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

    observer = getattr(executor, "_driver_observer", None)
    if observer is None or getattr(observer, "node", None) is None:
        raise DriverError("live observer is not available; readiness cannot be observed")
    node = executor.node
    thresholds = _as_thresholds(config)
    tf_fresh = float(thresholds.get("tf_fresh_s", 0.25))
    joint_fresh = float(thresholds.get("joint_state_fresh_s", 0.25))
    fixture_fresh = float(thresholds.get("fixture_fresh_s", 0.25))
    now = time.monotonic()

    report_bytes = _read_report_bytes(attempt_dir)
    report_sha256 = hashlib.sha256(report_bytes).hexdigest()
    identities = bundle.get("report_identities")
    scenario = bundle.get("scenario")
    planning_scene_declaration = bundle.get("planning_scene_declaration")
    if not isinstance(identities, Mapping) or not isinstance(scenario, Mapping) or not isinstance(planning_scene_declaration, Mapping):
        raise DriverError("readiness snapshot cannot resolve bundle identities")
    scenario_id = str(scenario.get("id"))
    seed = int(scenario.get("seed"))

    # ---- TF -------------------------------------------------------------
    tf_complete = False
    tf_age_s = float("inf")
    try:
        import tf2_ros
        observer.tf_buffer.lookup_transform("base_link", "link_tcp", _tf_zero_time())
        tf_complete = True
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        tf_complete = False
    if observer.last_tf_received_mono is not None:
        tf_age_s = now - observer.last_tf_received_mono

    # ---- Joint state ------------------------------------------------------
    joint = getattr(executor, "_latest_joint_state", None)
    joint_names = list(getattr(joint, "name", None) or [])
    joint_positions = list(getattr(joint, "position", None) or [])
    joint_velocities = list(getattr(joint, "velocity", None) or [])
    stamp = getattr(getattr(joint, "header", None), "stamp", None)
    header_stamp_ns = (
        int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))
        if stamp is not None
        else 0
    )
    joint_sources, joint_infos = _publishers_for(node, JOINT_STATES_TOPIC)
    joint_source = joint_sources[0] if joint_sources else ""
    joint_publishers = len(joint_infos)
    joint_age_s = (
        now - observer.joint_received_mono
        if observer.joint_received_mono is not None
        else float("inf")
    )
    joint_qos_observed = _topic_qos_lower(joint_infos[0].qos_profile) if joint_infos else {}

    # ---- Safety ----------------------------------------------------------
    safety = getattr(executor, "_latest_safety_stop", None)
    safety_data = bool(getattr(safety, "data", False))
    safety_sources, safety_infos = _publishers_for(node, SAFETY_STOP_TOPIC)
    safety_source = safety_sources[0] if safety_sources else ""
    safety_publishers = len(safety_infos)
    safety_qos_observed = _topic_qos_lower(safety_infos[0].qos_profile) if safety_infos else {}
    safety_age_s = (
        now - observer.safety_received_mono
        if observer.safety_received_mono is not None
        else float("inf")
    )

    # ---- Fixture payload ---------------------------------------------------
    fixture_payload = str(getattr(executor, "_fixture_payload", None) or "")
    fixture_sources, fixture_infos = _publishers_for(node, FIXTURE_TOPIC)
    fixture_source = fixture_sources[0] if fixture_sources else ""
    fixture_publishers = len(fixture_infos)
    fixture_qos_observed = _topic_qos_lower(fixture_infos[0].qos_profile) if fixture_infos else {}
    fixture_age_s = (
        now - observer.fixture_received_mono
        if observer.fixture_received_mono is not None
        else float("inf")
    )
    parsed_payload = _parse_fixture_payload(fixture_payload) if fixture_payload else None
    fixture_sequence = int(parsed_payload["sequence"]) if parsed_payload and isinstance(parsed_payload.get("sequence"), int) else 0
    fixture_owned = (
        list(parsed_payload["owned_ids"])
        if parsed_payload and isinstance(parsed_payload.get("owned_ids"), list)
        else []
    )
    fixture_published_at = (
        float(parsed_payload["published_at"])
        if parsed_payload and parsed_payload.get("published_at") is not None
        else 0.0
    )

    # ---- PlanningScene cache ------------------------------------------------
    planning_scene_state = getattr(executor, "_latest_planning_scene", None) or {}
    owned_ids = list(planning_scene_state.get("owned_ids", ()))
    attached_ids = list(planning_scene_state.get("attached_ids", ()))

    # ---- Controller manager --------------------------------------------------
    controller_records_raw = observer.list_controllers()
    controllers = _controllers_block(node, controller_records_raw)

    # ---- Operator (real publish_operator(False) baseline + observed receipt) --
    # F3.2: operator evidence is only accepted after a real ``publish_operator(False)``
    # baseline and a real observer subscription receipt.  Re-publish the baseline
    # immediately before building the block and spin the shared spinner so the
    # observer receipt is fresh (the operator topic carries no continuous stream).
    try:
        executor.publish_operator(False)
    except Exception:  # noqa: BLE001 - fail-closed operator baseline
        pass
    try:
        executor._spin_once()
    except Exception:  # noqa: BLE001 - fail-closed operator baseline
        pass
    # The operator receipt is stamped after the publish+spin; refresh the age
    # base so ``operator_age_s`` is non-negative.
    now = time.monotonic()
    operator_sources, operator_infos = _publishers_for(node, OPERATOR_TOPIC)
    operator_source = operator_sources[0] if operator_sources else ""
    operator_publishers = len(operator_infos)
    operator_qos_observed = _topic_qos_lower(operator_infos[0].qos_profile) if operator_infos else {}
    operator_received = observer.operator_samples >= 1
    operator_age_s = (
        now - observer.operator_received_mono
        if observer.operator_received_mono is not None
        else float("inf")
    )

    # ---- Collision -----------------------------------------------------------
    collision_sources, collision_infos = _publishers_for(node, "/sim/safety/collision")
    collision_publishers = len(collision_infos)
    collision_value = observer.collision_value if observer.collision_samples >= 1 else True
    collision_age_s = (
        now - observer.collision_received_mono
        if observer.collision_received_mono is not None
        else float("inf")
    )
    if not (
        observer.collision_samples >= 1
        and collision_value is False
        and _fresh_seconds(collision_age_s, 0.25)
        and collision_publishers == 1
    ):
        collision_value = True

    # ---- Actions / services ---------------------------------------------------
    actions: dict[str, Any] = {}
    for name, action_type in REQUIRED_ACTIONS.items():
        client = executor._action_clients.get(name)
        ready = False
        if client is not None:
            try:
                ready = bool(client.server_is_ready())
            except Exception:  # noqa: BLE001 - live client boundary
                ready = False
        server_count, source_node = _action_servers_and_source(node, name)
        actions[name] = {
            "type": action_type,
            "ready": ready,
            "server_count": server_count,
            "source_node": source_node,
        }
    services: dict[str, Any] = {}
    for name, service_type in REQUIRED_SERVICES.items():
        client = executor._service_clients.get(name)
        ready = False
        if client is not None:
            try:
                ready = bool(client.service_is_ready())
            except Exception:  # noqa: BLE001 - live client boundary
                ready = False
        server_count, source_node = _service_servers_and_source(node, name)
        services[name] = {
            "type": service_type,
            "ready": ready,
            "server_count": server_count,
            "source_node": source_node,
        }

    # ---- Topics ----------------------------------------------------------------
    topics = {
        JOINT_STATES_TOPIC: {
            "type": "sensor_msgs/msg/JointState",
            "publisher_count": joint_publishers,
            "source_node": joint_source,
            "qos": _topic_qos_with_depth(joint_qos_observed, _CONTRACT_DEPTH_BY_TOPIC[JOINT_STATES_TOPIC]),
            "names": joint_names,
            "positions": [float(v) for v in joint_positions],
            "velocities": [float(v) for v in joint_velocities],
            "header_stamp_ns": header_stamp_ns,
            "age_s": joint_age_s,
        },
        FIXTURE_TOPIC: {
            "type": "std_msgs/msg/String",
            "publisher_count": fixture_publishers,
            "source_node": fixture_source,
            "qos": _topic_qos_with_depth(fixture_qos_observed, _CONTRACT_DEPTH_BY_TOPIC[FIXTURE_TOPIC]),
            "received": bool(fixture_payload),
            "received_sequence": fixture_sequence,
            "sample_count": observer.fixture_samples,
            "age_s": fixture_age_s,
            "payload": fixture_payload,
        },
        OPERATOR_TOPIC: {
            "type": "std_msgs/msg/Bool",
            "publisher_count": operator_publishers,
            "source_node": operator_source,
            "qos": _topic_qos_with_depth(operator_qos_observed, _CONTRACT_DEPTH_BY_TOPIC[OPERATOR_TOPIC]),
            "allowlist": [False, True],
            "received": operator_received,
            "received_value": observer.operator_value,
            "received_timestamp_ns": int(time.time() * 1_000_000_000) if operator_received else 0,
            "received_age_s": operator_age_s,
            "freshness_limit_s": float(thresholds.get("operator_fresh_s", thresholds.get("fixture_fresh_s", 0.25))),
        },
        SAFETY_STOP_TOPIC: {
            "type": "std_msgs/msg/Bool",
            "publisher_count": safety_publishers,
            "source_node": safety_source,
            "qos": _topic_qos_with_depth(safety_qos_observed, _CONTRACT_DEPTH_BY_TOPIC[SAFETY_STOP_TOPIC]),
            "data": safety_data,
            "received": observer.safety_samples >= 1,
            "received_value": safety_data,
            "received_timestamp_ns": int(time.time() * 1_000_000_000) if observer.safety_samples >= 1 else 0,
            "sample_count": observer.safety_samples,
            "age_s": safety_age_s,
        },
    }

    fixture_decl = planning_scene_declaration
    revision = str(fixture_decl.get("revision", ""))
    revision_digest = str(fixture_decl.get("revision_digest", ""))
    target_source_id = str(fixture_decl.get("target_source_id", ""))
    fixture_descriptor = (
        str(fixture_decl.get("fixture_descriptor_sha256", ""))
        or str(identities.get("planning_scene_sha256", ""))
    )

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
                "owned_ids": fixture_owned,
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
        "tf": {"complete": tf_complete, "age_s": tf_age_s},
        "joint_state": {
            "names": joint_names,
            "positions": [float(v) for v in joint_positions],
            "velocities": [float(v) for v in joint_velocities],
            "header_stamp_ns": header_stamp_ns,
            "age_s": joint_age_s,
            "publisher_count": joint_publishers,
            "source_node": joint_source,
            "logical_controller": (
                "joint_state_broadcaster"
                if _logical_controller_active(controllers, "joint_state_broadcaster")
                else ""
            ),
        },
        "controllers": controllers,
        "safety": {
            "stop": safety_data,
            "age_s": safety_age_s,
            "sample_count": observer.safety_samples,
            "type": "std_msgs/msg/Bool",
            "publisher_count": safety_publishers,
            "source_node": safety_source,
            "qos": _topic_qos_with_depth(safety_qos_observed, _CONTRACT_DEPTH_BY_TOPIC[SAFETY_STOP_TOPIC]),
        },
        "actions": actions,
        "services": services,
        "topics": topics,
        "fixture": {
            "schema_version": 1,
            "state": parsed_payload.get("state", "") if parsed_payload else "",
            "scenario": parsed_payload.get("scenario", "") if parsed_payload else "",
            "owner": parsed_payload.get("owner", "") if parsed_payload else "",
            "revision": parsed_payload.get("revision", revision) if parsed_payload else revision,
            "revision_digest": parsed_payload.get("revision_digest", revision_digest) if parsed_payload else revision_digest,
            "owned_ids": fixture_owned,
            "target_source_id": parsed_payload.get("target_source_id", target_source_id) if parsed_payload else target_source_id,
            "target_handoff": parsed_payload.get("target_handoff", "pick_and_place/object_mesh") if parsed_payload else "pick_and_place/object_mesh",
            "sequence": fixture_sequence,
            "previous_sequence": fixture_sequence - 1,
            "sample_count": observer.fixture_samples,
            "published_at": fixture_published_at,
            "age_s": fixture_age_s,
            "fixture_descriptor_sha256": (
                parsed_payload.get("fixture_descriptor_sha256", fixture_descriptor)
                if parsed_payload
                else fixture_descriptor
            ),
        },
        "planning_scene": {
            "owned_ids": owned_ids,
            "attached_ids": attached_ids,
        },
        "robot_in_collision": collision_value,
    }


def _as_thresholds(config: Mapping[str, Any]) -> Mapping[str, Any]:
    thresholds = config.get("thresholds")
    return thresholds if isinstance(thresholds, Mapping) else {}


def _topic_qos_with_depth(observed: Mapping[str, object], contract_depth: int) -> dict[str, object]:
    """Lowercase reliability/durability (observed) + contract depth.

    The contract depth is asserted only when a compatible live sample has been
    received on the topic — the caller gates freshness; a stale/missing topic
    already fails readiness independently of this value.
    """
    return {
        "reliability": observed.get("reliability", ""),
        "durability": observed.get("durability", ""),
        "depth": int(contract_depth),
    }


def _logical_controller_active(controllers: Mapping[str, Any], name: str) -> bool:
    records = controllers.get("logical_controllers")
    if not isinstance(records, Mapping):
        return False
    record = records.get(name)
    return isinstance(record, Mapping) and record.get("state") == "active"


def _controllers_block(node: Any, records_raw: list[Any] | None) -> dict[str, Any]:
    """Build the ``controllers`` readiness block from live observations."""
    from integrated_gate_executor import CONTROLLER_MANAGER_NODE

    controller_nodes = 0
    try:
        for node_name, _namespace in node.get_node_names_and_namespaces():
            if _endpoint_label(node_name, _namespace) == CONTROLLER_MANAGER_NODE:
                controller_nodes += 1
    except Exception:  # noqa: BLE001 - live graph boundary
        controller_nodes = 0

    logical: dict[str, Any] = {}
    if records_raw is not None:
        for record in records_raw:
            name = str(getattr(record, "name", "") or "")
            state = str(getattr(record, "state", "") or "")
            if name in ("joint_state_broadcaster", "xarm7_traj_controller"):
                logical[name] = {
                    "state": state,
                    "source_node": CONTROLLER_MANAGER_NODE,
                    "cardinality": 1,
                }
    return {
        "manager_healthy": records_raw is not None,
        "manager_source_node": CONTROLLER_MANAGER_NODE,
        "manager_publisher_count": controller_nodes,
        "logical_controllers": logical,
    }


def _tf_zero_time() -> Any:
    from rclpy.time import Time

    return Time()


# --------------------------------------------------------------------------- #
# Journal graph observation (F3.6) — real introspection, no endpoint constants
# --------------------------------------------------------------------------- #

def _observed_topic_qos(infos: list[Any], expected_depth: int) -> dict[str, object]:
    """Observed reliability/durability + contract depth for a graph topic.

    Humble never reports depth (always 0); the journal contract depth is
    asserted from the committed constant, mirroring the production
    ``integrated_readiness`` handling.  Reliability/durability are observed
    from the first real endpoint; a QoS mismatch therefore fails closed.
    """
    if not infos:
        return {}
    profile = infos[0].qos_profile
    return {
        "reliability": _normalize_qos_value(getattr(profile, "reliability", "")),
        "durability": _normalize_qos_value(getattr(profile, "durability", "")),
        "depth": int(expected_depth),
    }


def _observe_journal_graph(executor: Any) -> dict[str, Any]:
    """Build the exact ``build_journal_graph_projection`` observed graph (F3.6).

    Introspects the executor node's real publishers/subscribers/services/clients
    for the three PlanningScene/fixture topics and the two planning-scene
    services.  The executor node is the journal recorder, so its own
    subscriptions/clients satisfy the recorder-subscriber/client requirements.
    Spins before observing so the rmw graph cache is populated.
    """
    from integrated_gate_executor import (  # noqa: F401 - live-only
        FIXTURE_TOPIC,
        JOURNAL_FIXTURE_TOPIC_QOS,
        JOURNAL_PLANNING_SCENE_TOPIC_QOS,
        JOURNAL_SERVICE_QOS,
        MONITORED_PLANNING_SCENE_TOPIC,
        OPERATOR_NODE,
        OPERATOR_NODE_NAMESPACE,
        PLANNING_SCENE_TOPIC,
    )

    node = executor.node
    # F3.6: spin before graph observation so the rmw graph cache is populated.
    # Remote-node service discovery (``get_service_names_and_types_by_node``) is
    # event-driven; a few spins with short delays settle it under load.
    for _ in range(10):
        try:
            executor._spin_once()
        except Exception:  # noqa: BLE001 - graph cache warm-up
            pass
        time.sleep(0.02)

    planner_qos = JOURNAL_PLANNING_SCENE_TOPIC_QOS
    fixture_qos = JOURNAL_FIXTURE_TOPIC_QOS
    service_qos = JOURNAL_SERVICE_QOS

    def _topic_entry(name: str, expected_type: str, expected_qos: Mapping[str, Any]) -> dict[str, Any]:
        pub_labels, pub_infos = _publishers_for(node, name)
        sub_labels, sub_infos = _subscribers_for(node, name)
        return {
            "type": expected_type,
            "requested_qos": _observed_topic_qos(sub_infos, int(expected_qos["depth"])),
            "offered_qos": _observed_topic_qos(pub_infos, int(expected_qos["depth"])),
            "publishers": [{"node": label, "node_namespace": ""} for label in pub_labels],
            "subscribers": [{"node": label, "node_namespace": ""} for label in sub_labels],
        }

    def _service_entry(name: str, expected_type: str) -> dict[str, Any]:
        servers, clients = _service_servers_and_clients(node, name)
        return {
            "type": expected_type,
            "requested_qos": dict(service_qos),
            "offered_qos": dict(service_qos),
            "servers": [{"node": label, "node_namespace": ""} for label in servers],
            "clients": [{"node": label, "node_namespace": ""} for label in clients],
        }

    return {
        "node_name": OPERATOR_NODE,
        "namespace": OPERATOR_NODE_NAMESPACE,
        "remap_table": {},
        "topics": {
            PLANNING_SCENE_TOPIC: _topic_entry(PLANNING_SCENE_TOPIC, "moveit_msgs/msg/PlanningScene", planner_qos),
            MONITORED_PLANNING_SCENE_TOPIC: _topic_entry(MONITORED_PLANNING_SCENE_TOPIC, "moveit_msgs/msg/PlanningScene", planner_qos),
            FIXTURE_TOPIC: _topic_entry(FIXTURE_TOPIC, "std_msgs/msg/String", fixture_qos),
        },
        "services": {
            "/get_planning_scene": _service_entry("/get_planning_scene", "moveit_msgs/srv/GetPlanningScene"),
            "/apply_planning_scene": _service_entry("/apply_planning_scene", "moveit_msgs/srv/ApplyPlanningScene"),
        },
    }


# --------------------------------------------------------------------------- #
# Runtime providers (F3.3, F3.4, F3.5)
# --------------------------------------------------------------------------- #

def _current_tcp_pose_provider(executor: Any) -> Callable[[], Mapping[str, Any]]:
    """Provide a fresh finite normalized ``base_link`` TCP pose from live TF."""

    def _provider() -> Mapping[str, Any]:
        observer = getattr(executor, "_driver_observer", None)
        if observer is None or getattr(observer, "tf_buffer", None) is None:
            raise DriverError("no live TF buffer is available for the TCP pose")
        try:
            import tf2_ros
            transform = observer.tf_buffer.lookup_transform(
                "base_link", "link_tcp", _tf_zero_time(), timeout=_tf_duration(0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            raise DriverError(f"no base_link -> link_tcp transform: {exc}") from exc
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp
        identity = f"tf:base_link->link_tcp:{int(getattr(stamp, 'sec', 0))}:{int(getattr(stamp, 'nanosec', 0))}"
        age_s = (
            time.monotonic() - observer.last_tf_received_mono
            if observer.last_tf_received_mono is not None
            else float("inf")
        )
        return {
            "frame_id": "base_link",
            "xyz": [float(translation.x), float(translation.y), float(translation.z)],
            "quaternion_xyzw": [
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ],
            "identity": identity,
            "age_s": age_s,
        }

    return _provider


def _tf_duration(seconds: float) -> Any:
    from rclpy.duration import Duration

    return Duration(seconds=float(seconds))


def _environment_cloud_provider(executor: Any) -> Callable[[], Any]:
    """Provide the fresh non-empty ``base_link`` PointCloud2 (Option A+).

    F3.3: the provider transforms the live ``/livox/lidar`` cloud (frame
    ``livox360``) into ``base_link`` with ``tf2_sensor_msgs.do_transform_cloud``.
    Rejects missing, stale, empty, malformed, wrong-frame, or untransformable
    clouds.  Never constructs provider success data from PlanningScene objects.
    """

    def _provider() -> Any:
        observer = getattr(executor, "_driver_observer", None)
        if observer is None or getattr(observer, "latest_cloud", None) is None:
            raise DriverError("no live environment PointCloud2 is available")
        cloud = observer.latest_cloud
        width = int(getattr(cloud, "width", 0) or 0)
        height = int(getattr(cloud, "height", 0) or 0)
        data = getattr(cloud, "data", None)
        data_len = len(bytes(data)) if data is not None else 0
        if width < 1 or height < 1 or data_len < 1:
            raise DriverError("environment PointCloud2 is empty")
        if observer.cloud_received_mono is None:
            raise DriverError("environment PointCloud2 has no receipt timestamp")
        if time.monotonic() - observer.cloud_received_mono > ENV_CLOUD_MAX_AGE_S:
            raise DriverError("environment PointCloud2 is stale")
        frame_id = str(getattr(getattr(cloud, "header", None), "frame_id", "") or "")
        if not frame_id:
            raise DriverError("environment PointCloud2 has no frame_id")
        try:
            import tf2_ros
            transform = observer.tf_buffer.lookup_transform(
                "base_link", frame_id, _tf_zero_time()
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            raise DriverError(f"no base_link transform for environment cloud: {exc}") from exc
        from tf2_sensor_msgs import do_transform_cloud

        return do_transform_cloud(cloud, transform)

    return _provider


def _native_gripper_goal_count_provider(executor: Any) -> Callable[[], Mapping[str, Any]]:
    """Provide the live native gripper action-goal count seam (F3.5)."""

    def _provider() -> Mapping[str, Any]:
        recorder = getattr(executor, "_gripper_recorder", None)
        observer = getattr(executor, "_driver_observer", None)
        if recorder is None:
            raise DriverError("no live gripper action-client recorder is available")
        baseline_mono = recorder.install_mono
        fresh_mono = max(
            [marker for marker in (recorder.last_goal_mono, observer.gripper_status_received_mono if observer is not None else None) if marker is not None] or [baseline_mono]
        )
        return {
            "count": int(recorder.goal_count),
            "age_s": max(0.0, time.monotonic() - fresh_mono),
        }

    return _provider


def _fjt_transaction_provider(executor: Any) -> Callable[[], Mapping[str, Any]]:
    """Provide FJT transactions from the observed status cache + execute recorder.

    F3.4: goal_uuid/status/sequence/timestamp come from the executor's newest
    fresh real ``_fjt_status_cache`` entry; the trajectory digest is the digest
    of the exact ExecuteTrajectory goal captured by the execute recorder.  No
    status-record hashing and no invented digest.
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
        recorder = getattr(executor, "_execute_recorder", None)
        digest = recorder.last_trajectory_digest if recorder is not None else None
        if not isinstance(digest, str) or not digest:
            raise DriverError("no executed-trajectory digest captured by the execute recorder")
        return {
            "endpoint": FJT_ENDPOINT,
            "goal_uuid": goal_uuid,
            "trajectory_digest": digest,
            "source": "executor-action-client-goal-introspection",
            "sequence": sequence,
            "timestamp": timestamp_f,
            "status": status,
        }

    return _provider


def _goal_id_hex(goal_handle: Any) -> str | None:
    """Normalize a real rclpy goal handle's UUID to lowercase 16-byte hex.

    F3.5: the immutable executor's ``_normalize_goal_uuid`` accepts bytes/str but
    not the real ``unique_identifier_msgs/msg/UUID`` message a live rclpy
    ``ClientGoalHandle.goal_id`` carries (its ``.uuid`` field is a numpy
    ``uint8[16]`` array).  The driver owns this live normalization for the
    plan/execute handles it retains.  Returns ``None`` on any malformed/absent
    UUID so split-path contracts fail closed.
    """
    raw = getattr(goal_handle, "goal_id", None)
    if raw is None or isinstance(raw, bool):
        return None
    try:
        if isinstance(raw, (bytes, bytearray)):
            return _uuid.UUID(bytes=bytes(raw)).hex
        if isinstance(raw, str):
            return _uuid.UUID(str(raw)).hex
        candidate = getattr(raw, "uuid", None)
        if candidate is not None:
            return _uuid.UUID(bytes=bytes(candidate)).hex
        candidate = getattr(raw, "bytes", None)
        if candidate is not None:
            return _uuid.UUID(bytes=bytes(candidate)).hex
    except (TypeError, ValueError):
        return None
    return None


def _long_motion_provider_from_presend(
    planning_goal_id: str,
    execute_goal_id: str,
) -> Callable[[], Mapping[str, Any]]:
    """Return a long-motion provider bound to the pre-sent transaction UUIDs."""

    def _provider() -> Mapping[str, Any]:
        return {
            "planning_goal_id": planning_goal_id,
            "execute_goal_id": execute_goal_id,
        }

    return _provider


def _send_execute_retaining_handle(
    executor: Any,
    scenario_id: str,
    goal: Any,
    *,
    server_timeout_s: float = 5.0,
    accept_timeout_s: float = 10.0,
) -> tuple[str, Any]:
    """Send one ExecuteTrajectory goal and retain the accepted goal handle.

    F3.5: the accepted handle is passed to ``run_cancel_sequence`` as the live
    ``execute_goal_handle`` (never an artifact-file-derived UUID).  The execute
    recorder captures the exact goal trajectory digest at send time.
    """
    client = executor._action_clients["/execute_trajectory"]
    if not client.wait_for_server(timeout_sec=server_timeout_s):
        raise DriverError("/execute_trajectory server was not available for pre-send")
    send_future = client.send_goal_async(goal)
    accept_deadline = time.monotonic() + accept_timeout_s
    while not send_future.done() and time.monotonic() < accept_deadline:
        executor._spin_once()
    if not send_future.done():
        try:
            send_future.cancel()
        except Exception:  # noqa: BLE001
            pass
        raise DriverError("pre-send execute goal acceptance timed out")
    goal_handle = send_future.result()
    if goal_handle is None or not getattr(goal_handle, "accepted", False):
        raise DriverError("pre-send execute goal was rejected")
    execute_goal_id = _goal_id_hex(goal_handle)
    if not (isinstance(execute_goal_id, str) and execute_goal_id):
        raise DriverError("pre-send execute goal produced no valid normalized UUID")
    return execute_goal_id, goal_handle


def _presend_long_motion(executor: Any, scenario_id: str) -> dict[str, Any]:
    """Pre-send the committed long motion for the cancel/safety scenarios.

    Uses the executor's own planning/execution helpers so the plan/execute
    UUIDs and digest come from the in-memory action recorders, never from
    artifact files (which are written only at finalization).
    """
    from integrated_gate_executor import (  # noqa: F401 - live-only
        build_execute_trajectory_goal,
        build_joint_move_group_goal,
        stage_d_dispatch,
    )

    spec = stage_d_dispatch(scenario_id, scenario=executor.scenario)
    plan_goal = build_joint_move_group_goal(_LONG_MOTION_JOINT_TARGET, plan_only=True)
    plan_record = executor._send_plan_only_retaining_handle(scenario_id, plan_goal, spec)
    # The immutable executor normalizes the planning UUID from bytes/str; for a
    # real rclpy handle the UUID message carries a numpy uint8 array, so fall
    # back to the driver-owned live normalization of the retained goal handle.
    planning_goal_id = plan_record.get("planning_goal_id")
    if not (isinstance(planning_goal_id, str) and planning_goal_id):
        plan_handle = plan_record.get("goal_handle")
        planning_goal_id = _goal_id_hex(plan_handle) if plan_handle is not None else None
    planned_trajectory = plan_record.get("planned_trajectory")
    if not (isinstance(planning_goal_id, str) and planning_goal_id):
        raise DriverError(
            "pre-send long-motion planning produced no valid planning UUID: "
            f"{plan_record.get('status', 'unknown')}"
        )
    if planned_trajectory is None:
        raise DriverError("pre-send long-motion planning produced no planned trajectory")
    execute_goal = build_execute_trajectory_goal(planned_trajectory)
    execute_goal_id, execute_goal_handle = _send_execute_retaining_handle(executor, scenario_id, execute_goal)
    return {
        "planning_goal_id": planning_goal_id,
        "execute_goal_id": execute_goal_id,
        "execute_goal_handle": execute_goal_handle,
    }


# --------------------------------------------------------------------------- #
# Live construction + runtime provider factory (F3.1, F3.5)
# --------------------------------------------------------------------------- #

def _build_journal_graph_projection(executor: Any) -> Mapping[str, Any]:
    from integrated_gate_executor import build_journal_graph_projection  # noqa: F401

    payload = getattr(executor, "_fixture_payload", None) or ""
    observed = _observe_journal_graph(executor)
    return build_journal_graph_projection(fixture_payload=str(payload), observed_graph=observed)


def _construct_executor(
    *,
    bundle: Mapping[str, Any],
    attempt_dir: Path,
    config: Mapping[str, Any],
    domain_id: int,
    seed: int,
) -> Any:
    """Construct the real ``IntegratedGateExecutor`` with live observers.

    Installs the deterministic-serialize seam and the action-client recorders
    immediately after construction (before any run method), creates the
    driver-owned observer node in the executor's private context, and publishes
    the operator-clear baseline that the observer then receives.
    """
    from integrated_gate_executor import IntegratedGateExecutor  # noqa: F401

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
        return _build_journal_graph_projection(current)

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
    observer = _LiveProviderObserver(executor)
    executor._driver_observer = observer
    _install_deterministic_serialize(executor)
    _install_action_client_recorders(executor)
    try:
        executor.publish_operator(False)
    except Exception:
        observer.destroy()
        try:
            executor.shutdown()
        except Exception:  # noqa: BLE001
            pass
        raise
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
    """Build the per-run-method provider kwargs from live state (F3.3-F3.5)."""
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
        presend = _presend_long_motion(executor, scenario_id)
        kwargs["long_motion_provider"] = _long_motion_provider_from_presend(
            presend["planning_goal_id"],
            presend["execute_goal_id"],
        )
        if method_name == "run_cancel_sequence":
            kwargs["planning_goal_id"] = presend["planning_goal_id"]
            kwargs["execute_goal_id"] = presend["execute_goal_id"]
            kwargs["execute_goal_handle"] = presend["execute_goal_handle"]
    if method_name == "run_cartesian_retreat":
        kwargs["current_tcp_pose_provider"] = _current_tcp_pose_provider(executor)
        kwargs["environment_cloud_provider"] = _environment_cloud_provider(executor)
    if method_name == "run_pick_place_sequence":
        kwargs["current_tcp_pose_provider"] = _current_tcp_pose_provider(executor)
        kwargs["native_gripper_goal_count_provider"] = _native_gripper_goal_count_provider(executor)
        observed = set_post_grasp_lift_m(
            _ParameterServiceClient(executor), value_m=lift_value_m
        )
        kwargs["post_grasp_lift_m_provider"] = _post_grasp_lift_m_provider(observed)
    return kwargs


# --------------------------------------------------------------------------- #
# Core transaction (F2.2, F3.2) — testable with ROS-free executor doubles
# --------------------------------------------------------------------------- #

def _wait_for_readiness(executor: Any, *, timeout_s: float) -> Mapping[str, Any]:
    """Spin the executor/observer spinner and poll readiness until ready.

    F3.2: ``executor._spin_once()`` services both the executor and the observer
    nodes on the shared spinner, so subscriptions (joint/fixture/safety/
    operator/collision/TF/lidar) fire and the real caches populate before each
    readiness evaluation.
    """
    deadline = time.monotonic() + float(timeout_s)
    last: Mapping[str, Any] = {"ready": False, "reasons": ["readiness wait timeout"]}
    while time.monotonic() < deadline:
        try:
            executor._spin_once()
        except Exception:  # noqa: BLE001 - fail-closed readiness boundary
            pass
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
        time.sleep(0.05)
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
    :class:`DriverError` on any driver-level failure.  On success writes
    ``execution-terminal.json`` AFTER the executor's artifact finalization and
    returns the terminal summary.  The driver-owned observer is removed and
    destroyed during teardown without masking the original result.
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
            observer = getattr(executor, "_driver_observer", None)
            if observer is not None:
                observer.destroy()
        except Exception:  # pragma: no cover - defensive teardown
            pass
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
