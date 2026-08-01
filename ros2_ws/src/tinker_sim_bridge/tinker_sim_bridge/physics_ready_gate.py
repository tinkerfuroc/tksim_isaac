"""Executable staged PHYSICS_READY gate (Task 6).

The gate is started by the integrated launch only after the one-shot
``scenario_runner`` process exits 0.  It synchronously opens the atomically
replaced ``scenario-runner.json``, computes ``scenario_report_sha256`` from the
final bytes, parses the canonical shared report schema, and validates every
expected identity (scenario id/seed/declaration digest, planning-scene
revision/digest/owned ids/target handoff, full integrated mapping and digest,
model fingerprint, provider-manifest path/digest) before creating the ready
state.  Only then does it atomically write ``physics-ready.json`` (the canonical
report mapping plus ``scenario_report_sha256``), publish a transient reliable
``std_msgs/msg/String`` status, and serve ``/sim/ready/physics`` as
``std_srvs/srv/Trigger``.  The gate remains alive so its source/cardinality/
freshness evidence stays observable; it is not a one-shot process.
"""
from __future__ import annotations

import json
import os
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .integrated_readiness import (
    FINAL_SIMULATION_STATE,
    PHYSICS_READY_BOUNDARY,
    build_canonical_report,
    canonical_json,
    parse_canonical_report,
    planning_scene_digest,
    planning_scene_mapping,
    report_identities,
    sha256_bytes,
    sha256_json,
    validate_report,
)

_STATUS_TOPIC = "/sim/status/physics_ready"
_READY_SERVICE = "/sim/ready/physics"
_STATE_SERVICE = "/get_simulation_state"

_STATE_PENDING = "PHYSICS_PENDING"
_STATE_READY = "PHYSICS_READY"
_STATE_FAILED = "PHYSICS_FAILED"

_STATE_PLAYING = 1


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def write_physics_ready(report: dict[str, object], scenario_report_sha256: str, path: Path) -> bytes:
    """Atomically write ``physics-ready.json`` (canonical report + digest)."""
    payload = {
        "schema_version": 1,
        "state": _STATE_READY,
        "scenario_report_sha256": scenario_report_sha256,
        "report": report,
    }
    data = canonical_json(payload)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=".{}.".format(path.name), dir=str(path.parent)
    )
    temporary_path = Path(temporary)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
        _fsync_directory(path.parent)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise
    return data


class PhysicsReadyGate(Node):
    """Persistent physics-ready gate serving ``/sim/ready/physics``."""

    def __init__(
        self,
        *,
        node_name: str | None = None,
        context=None,
        parameter_overrides=None,
    ) -> None:
        super().__init__(
            node_name or "tinker_sim_physics_ready_gate",
            context=context,
            parameter_overrides=parameter_overrides or [],
        )
        try:
            self._initialize()
        except Exception:
            try:
                self.destroy_node()
            except Exception:  # noqa: BLE001 - destroy must never mask the cause
                pass
            raise

    def _initialize(self) -> None:
        self.declare_parameter("report_path", "")
        self.declare_parameter("physics_ready_path", "")
        self.declare_parameter("check_period_s", 0.5)
        self.declare_parameter("timeout_s", 30.0)
        self.declare_parameter("scenario_id", "")
        self.declare_parameter("seed", -1)
        self.declare_parameter("scenario_declaration_sha256", "")
        self.declare_parameter("planning_scene_revision", "")
        self.declare_parameter("planning_scene_revision_digest", "")
        self.declare_parameter("planning_scene_owned_ids", "[]")
        self.declare_parameter("planning_scene_target_source_id", "")
        self.declare_parameter("planning_scene_target_handoff", "")
        self.declare_parameter("integrated_mapping", "{}")
        self.declare_parameter("public_integrated_mapping", "")
        self.declare_parameter("integrated_sha256", "")
        self.declare_parameter("runtime_contract_sha256", "")
        self.declare_parameter("model_fingerprint", "")
        self.declare_parameter("provider_manifest_path", "")
        self.declare_parameter("provider_manifest_sha256", "")
        self._report_path = str(self.get_parameter("report_path").value)
        if not self._report_path:
            raise ValueError("report_path parameter is required")
        self._physics_ready_path = str(self.get_parameter("physics_ready_path").value)
        if not self._physics_ready_path:
            raise ValueError("physics_ready_path parameter is required")
        period = float(self.get_parameter("check_period_s").value)
        timeout = float(self.get_parameter("timeout_s").value)
        if not (period > 0 and timeout > 0):
            raise ValueError("check_period_s and timeout_s must be positive")
        self._period = period
        self._timeout_s = timeout
        self._started = time.monotonic()
        self._expected = self._build_expected()
        self._state = _STATE_PENDING
        self._fail_reason: str | None = None
        self._scenario_report_sha256: str | None = None
        self._report: dict[str, object] | None = None
        self._report_reasons: list[str] = []
        self._sim_state_observed: object = None
        self._state_service_state: dict[str, object] = {}
        self._service_group = ReentrantCallbackGroup()

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(String, _STATUS_TOPIC, status_qos)
        self._ready_service = self.create_service(
            Trigger, _READY_SERVICE, self._on_ready
        )
        self.create_timer(period, self._tick)

    def _build_expected(self) -> dict[str, object]:
        raw_mapping = str(self.get_parameter("integrated_mapping").value)
        try:
            integrated = json.loads(raw_mapping)
        except json.JSONDecodeError as exc:
            raise ValueError("integrated_mapping parameter is not valid JSON") from exc
        if not isinstance(integrated, dict):
            raise ValueError("integrated_mapping parameter must be a JSON object")
        # The public report's ``integrated`` field is the production-canonical
        # one-key mapping; the full runtime contract is carried separately.
        public_raw = str(self.get_parameter("public_integrated_mapping").value)
        if public_raw and public_raw.strip():
            try:
                public_integrated = json.loads(public_raw)
            except json.JSONDecodeError as exc:
                raise ValueError("public_integrated_mapping parameter is not valid JSON") from exc
            if not isinstance(public_integrated, dict):
                raise ValueError("public_integrated_mapping parameter must be a JSON object")
        else:
            public_integrated = integrated
        return {
            "scenario_id": str(self.get_parameter("scenario_id").value),
            "seed": int(self.get_parameter("seed").value),
            "scenario_declaration_sha256": str(
                self.get_parameter("scenario_declaration_sha256").value
            ),
            "planning_scene_revision": str(
                self.get_parameter("planning_scene_revision").value
            ),
            "planning_scene_revision_digest": str(
                self.get_parameter("planning_scene_revision_digest").value
            ),
            "planning_scene_owned_ids": str(
                self.get_parameter("planning_scene_owned_ids").value
            ),
            "planning_scene_target_source_id": str(
                self.get_parameter("planning_scene_target_source_id").value
            ),
            "planning_scene_target_handoff": str(
                self.get_parameter("planning_scene_target_handoff").value
            ),
            "integrated_mapping": public_integrated,
            "public_integrated_mapping": public_integrated,
            "integrated_sha256": str(self.get_parameter("integrated_sha256").value),
            "runtime_contract_mapping": integrated,
            "runtime_contract_sha256": str(
                self.get_parameter("runtime_contract_sha256").value
            ),
            "model_fingerprint": str(self.get_parameter("model_fingerprint").value),
            "provider_manifest_path": str(
                self.get_parameter("provider_manifest_path").value
            ),
            "provider_manifest_sha256": str(
                self.get_parameter("provider_manifest_sha256").value
            ),
        }

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _current_status(self) -> dict[str, object]:
        report = self._report or {}
        return {
            "schema_version": 1,
            "state": self._state,
            "scenario": report.get("scenario", {}),
            "planning_scene": report.get("planning_scene", {}),
            "integrated": report.get("integrated", {}),
            "identities": report.get("identities", {}),
            "final_simulation_state": report.get("final_simulation_state"),
            "provider_manifest_path": self._expected.get("provider_manifest_path", ""),
            "provider_manifest_sha256": self._expected.get(
                "provider_manifest_sha256", ""
            ),
            "runtime_contract_sha256": self._expected.get(
                "runtime_contract_sha256", ""
            ),
            "scenario_report_sha256": self._scenario_report_sha256,
            "sim_state_observed": self._sim_state_observed,
            "published_at": float(self.get_clock().now().nanoseconds) / 1e9,
        }

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            self._current_status(), sort_keys=True, separators=(",", ":")
        )
        self._publisher.publish(message)

    def _on_ready(self, request: Trigger.Request, response: Trigger.Response):
        del request
        if self._state == _STATE_READY and self._report is not None:
            response.success = True
        else:
            response.success = False
        status = self._current_status()
        response.message = json.dumps(status, sort_keys=True, separators=(",", ":"))
        return response

    # ------------------------------------------------------------------
    # Simulation-state observation (supplemental evidence)
    # ------------------------------------------------------------------

    def _observe_sim_state(self) -> None:
        """Record the live simulation state as supplemental evidence only.

        The report's ``final_simulation_state`` is the authoritative evidence;
        the live state observation never blocks readiness.
        """
        try:
            from simulation_interfaces.msg import SimulationState
            from simulation_interfaces.srv import GetSimulationState
        except ImportError:
            self._sim_state_observed = "unavailable"
            return
        state = self._state_service_state
        if state.get("succeeded"):
            self._sim_state_observed = state.get("result")
            return
        client = state.get("client")
        if client is None:
            client = self.create_client(
                GetSimulationState, _STATE_SERVICE, callback_group=self._service_group
            )
            state["client"] = client
        if not client.service_is_ready():
            if state.get("started_at") is not None and time.monotonic() - state.get("started_at") > self._timeout_s:
                state["client"] = None
                state["started_at"] = None
                state["error"] = "get_simulation_state timed out"
            return
        if state.get("future") is None:
            state["future"] = client.call_async(client.srv_type.Request())
            state["started_at"] = time.monotonic()
            return
        future = state.get("future")
        if not future.done():
            return
        state["future"] = None
        try:
            response = future.result()
            sim_state = getattr(response, "state", None)
            value = int(getattr(sim_state, "state", -1)) if sim_state is not None else -1
        except Exception as exc:  # noqa: BLE001 - transient failures must recover
            state["error"] = str(exc)
            state["client"] = None
            return
        if value == _STATE_PLAYING:
            state["succeeded"] = True
            state["result"] = "STATE_PLAYING"
            self._sim_state_observed = "STATE_PLAYING"
        else:
            state["error"] = "simulation state is {}, expected STATE_PLAYING".format(value)

    # ------------------------------------------------------------------
    # Report parse + validation
    # ------------------------------------------------------------------

    def _parse_report(self) -> None:
        path = Path(self._report_path)
        if not path.is_file():
            self._fail_reason = "scenario report not found: {}".format(path)
            self._state = _STATE_FAILED
            return
        data = path.read_bytes()
        self._scenario_report_sha256 = sha256_bytes(data)
        try:
            report = parse_canonical_report(data)
        except Exception as exc:  # noqa: BLE001 - parse failures fail closed
            self._fail_reason = "scenario report parse failed: {}".format(exc)
            self._state = _STATE_FAILED
            return
        self._report = dict(report)
        validation = validate_report(report, self._expected)
        self._report_reasons = list(validation["reasons"])
        if not validation["ready"]:
            self._fail_reason = "; ".join(validation["reasons"])
            self._state = _STATE_FAILED
            return
        write_physics_ready(
            report, self._scenario_report_sha256, Path(self._physics_ready_path)
        )
        self._state = _STATE_READY
        self.get_logger().info(
            "physics ready: scenario={} report_sha256={}".format(
                report.get("scenario", {}).get("id"), self._scenario_report_sha256
            )
        )

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        self._observe_sim_state()
        if self._state == _STATE_PENDING:
            if time.monotonic() - self._started > self._timeout_s:
                self._fail_reason = self._fail_reason or "timed out waiting for physics ready"
                self._state = _STATE_FAILED
            else:
                self._parse_report()
        self._publish_status()
        if self._state == _STATE_FAILED and self._fail_reason:
            self.get_logger().error("physics ready gate failed: {}".format(self._fail_reason))


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = PhysicsReadyGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
