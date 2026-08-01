from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from simulation_interfaces.msg import Result, SimulationState
from simulation_interfaces.srv import (
    LoadWorld,
    ResetSimulation,
    SetSimulationState,
    SpawnEntity,
)

from tinker_sim_core.orchestration import standard_operations
from tinker_sim_core.scenario import load_named_scenario

from .fixture_planning_scene import fixture_owned_ids
from .integrated_readiness import (
    FINAL_SIMULATION_STATE,
    build_canonical_report,
    planning_scene_digest,
    planning_scene_mapping,
    serialize_report,
    sha256_bytes,
    validate_report,
)


def verify_expected_values(
    *,
    scenario_id: str,
    seed: int,
    declaration: dict[str, object],
    planning_scene: dict[str, object],
    integrated: dict[str, object],
    expected: dict[str, object],
) -> list[str]:
    """Verify the live scenario identities against the launch-supplied expected values.

    Returns a list of human-readable reasons; an empty list means every expected
    identity matches.  The launch computes the same digests over the same
    unchanged scenario bytes, so a mismatch is a hard fail-closed error.
    """
    reasons: list[str] = []
    report = build_canonical_report(
        scenario_id=scenario_id,
        seed=seed,
        declaration=declaration,
        planning_scene=planning_scene,
        integrated=integrated,
        operations=[{"operation": "set_simulation_state", "accepted": True, "state": 1, "boundary": "PHYSICS_READY"}],
        model_fingerprint=str(expected.get("model_fingerprint", "")),
        provider_manifest_sha256=str(expected.get("provider_manifest_sha256", "")),
    )
    validation = validate_report(report, expected)
    reasons.extend(validation["reasons"])
    actual_digest = planning_scene_digest(planning_scene)
    expected_digest = str(expected.get("planning_scene_revision_digest", ""))
    if actual_digest != expected_digest:
        reasons.append(
            "planning_scene digest {!r} != expected {!r}".format(
                actual_digest, expected_digest
            )
        )
    owned = fixture_owned_ids(planning_scene)
    expected_owned = tuple(
        str(item)
        for item in json.loads(str(expected.get("planning_scene_owned_ids", "[]")))
    )
    if owned != expected_owned:
        reasons.append(
            "planning_scene owned ids {!r} != expected {!r}".format(
                list(owned), list(expected_owned)
            )
        )
    return reasons


def write_report_atomic(report: dict[str, object], path: Path) -> bytes:
    """Atomically publish the canonical compact report bytes to *path*.

    Writes to a sibling temporary file, flushes/closes it, uses ``os.replace``,
    and returns the exact final report bytes for digest recording.
    """
    data = serialize_report(report)
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
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise
    return data


class _RetryableServiceError(RuntimeError):
    """A service call failed before the server could accept its result."""


class ScenarioRunner(Node):
    def __init__(
        self,
        *,
        timeout_s: float,
        reset_attempts: int = 3,
        reset_retry_delay_s: float = 0.5,
    ) -> None:
        super().__init__("tinker_sim_scenario_runner")
        if timeout_s <= 0:
            raise ValueError("timeout must be positive")
        if reset_attempts < 1:
            raise ValueError("reset attempts must be at least one")
        if reset_retry_delay_s < 0:
            raise ValueError("reset retry delay must not be negative")
        self.timeout_s = timeout_s
        self.reset_attempts = reset_attempts
        self.reset_retry_delay_s = reset_retry_delay_s
        self._load_world = self.create_client(LoadWorld, "/load_world")
        self._reset = self.create_client(ResetSimulation, "/reset_simulation")
        self._spawn = self.create_client(SpawnEntity, "/spawn_entity")
        self._state = self.create_client(
            SetSimulationState, "/set_simulation_state"
        )

    def call(self, client, request, *, timeout_s: float | None = None):
        timeout = self.timeout_s if timeout_s is None else timeout_s
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise _RetryableServiceError(
                    f"standard service unavailable: {client.srv_name}"
                )
            if client.wait_for_service(timeout_sec=min(0.25, remaining)):
                break
        future = client.call_async(request)
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                try:
                    client.remove_pending_request(future)
                except AttributeError:
                    future.cancel()
                raise _RetryableServiceError(
                    f"standard service timed out: {client.srv_name}"
                )
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))
        response = future.result()
        if response is None:
            raise _RetryableServiceError(f"standard service failed: {client.srv_name}")
        if response.result.result != Result.RESULT_OK:
            raise RuntimeError(
                f"{client.srv_name} rejected operation "
                f"({response.result.result}): {response.result.error_message}"
            )
        return response

    def _initial_reset(self, request):
        """Retry only the idempotent startup reset within a finite budget."""
        failures = []
        for attempt in range(1, self.reset_attempts + 1):
            try:
                return self.call(self._reset, request)
            except _RetryableServiceError as exc:
                failures.append(str(exc))
                if attempt == self.reset_attempts:
                    break
                self.get_logger().warning(
                    f"initial reset attempt {attempt}/{self.reset_attempts} "
                    f"did not complete: {exc}; retrying"
                )
                if self.reset_retry_delay_s:
                    time.sleep(self.reset_retry_delay_s)
        raise RuntimeError(
            f"initial reset failed after {self.reset_attempts} attempts: "
            + failures[-1]
        )

    def execute(self, operations) -> list[dict[str, object]]:
        results = []
        for index, operation in enumerate(operations):
            if operation.kind == "load_world":
                request = LoadWorld.Request()
                request.uri = str(operation.payload["uri"])
                request.fail_on_unsupported_element = True
                request.ignore_missing_or_unsupported_assets = False
                self.call(self._load_world, request)
            elif operation.kind == "reset_spawned":
                request = ResetSimulation.Request()
                request.scope = ResetSimulation.Request.SCOPE_SPAWNED
                if index == 0:
                    self._initial_reset(request)
                else:
                    self.call(self._reset, request)
            elif operation.kind == "spawn_entity":
                request = SpawnEntity.Request()
                request.name = str(operation.payload["name"])
                request.allow_renaming = False
                request.uri = str(operation.payload["uri"])
                request.entity_namespace = str(operation.payload["entity_namespace"])
                pose = PoseStamped()
                pose.header.frame_id = str(operation.payload["frame_id"])
                xyz = operation.payload["xyz"]
                xyzw = operation.payload["quaternion_xyzw"]
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = xyz
                (
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ) = xyzw
                request.initial_pose = pose
                response = self.call(self._spawn, request)
                actual_name = str(getattr(response, "entity_name", ""))
                expected_name = str(operation.payload["prim_path"])
                if actual_name and actual_name != expected_name:
                    raise RuntimeError(
                        f"spawned entity path changed from {expected_name} to {actual_name}"
                    )
            elif operation.kind == "set_simulation_state":
                request = SetSimulationState.Request()
                state = int(operation.payload["state"])
                allowed_states = {
                    SimulationState.STATE_STOPPED,
                    SimulationState.STATE_PLAYING,
                }
                if state not in allowed_states:
                    raise RuntimeError(f"unsupported scenario simulation state: {state}")
                request.state = SimulationState(state=state)
                self.call(self._state, request)
            else:
                raise RuntimeError(f"unsupported standard operation: {operation.kind}")
            result = {"operation": operation.kind, "accepted": True}
            if operation.kind == "set_simulation_state":
                result["state"] = int(operation.payload["state"])
                if "boundary" in operation.payload:
                    result["boundary"] = str(operation.payload["boundary"])
            if operation.kind == "spawn_entity":
                result.update(
                    {
                        "logical_id": str(operation.payload["logical_id"]),
                        "prim_path": str(operation.payload["prim_path"]),
                    }
                )
            results.append(result)
        return results


def _parse_json_argument(value: str | None, label: str) -> object:
    if value is None or not value.strip():
        return None
    try:
        return json.loads(value)
    except json.JSONDecodeError as exc:
        raise RuntimeError("{} is not valid JSON: {}".format(label, exc)) from exc


def _build_integrated_mapping(arguments) -> dict[str, object] | None:
    """Parse the expected full integrated mapping argument, if provided.

    ``None`` means the legacy non-overlay path (no canonical report is built);
    the overlay path always provides the mapping so its digest agrees everywhere.
    """
    raw = _parse_json_argument(
        arguments.expected_integrated_mapping, "expected_integrated_mapping"
    )
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise RuntimeError("expected_integrated_mapping must be a JSON object")
    return {str(key): value for key, value in raw.items()}


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--reset-attempts", type=int, default=3)
    parser.add_argument("--reset-retry-delay", type=float, default=0.5)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--expected-scenario-declaration-sha256")
    parser.add_argument("--expected-planning-scene-revision")
    parser.add_argument("--expected-planning-scene-revision-digest")
    parser.add_argument("--expected-planning-scene-owned-ids", default="[]")
    parser.add_argument("--expected-planning-scene-target-source-id")
    parser.add_argument("--expected-planning-scene-target-handoff")
    parser.add_argument("--expected-integrated-mapping")
    parser.add_argument("--expected-integrated-sha256")
    parser.add_argument("--expected-model-fingerprint")
    parser.add_argument("--provider-manifest", type=Path)
    parser.add_argument("--provider-manifest-sha256")
    arguments = parser.parse_args(
        rclpy.utilities.remove_ros_args(args=sys.argv if argv is None else argv)[1:]
    )
    root = arguments.root.resolve()
    scenario = load_named_scenario(root, arguments.scenario)
    scenario_path = root / "simulation" / "scenarios" / "{}.json".format(arguments.scenario)
    raw = json.loads(scenario_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise RuntimeError("scenario declaration must be a JSON object")
    declaration = {
        str(key): value for key, value in raw.items() if key not in {"id", "seed"}
    }
    integrated = _build_integrated_mapping(arguments)

    if integrated is not None:
        if scenario.planning_scene is None:
            raise RuntimeError(
                "canonical integrated report requires a planning_scene declaration"
            )
        planning_scene = dict(scenario.planning_scene)
        expected = {
            "scenario_id": arguments.scenario,
            "seed": arguments.seed,
            "scenario_declaration_sha256": arguments.expected_scenario_declaration_sha256 or "",
            "planning_scene_revision": arguments.expected_planning_scene_revision or "",
            "planning_scene_revision_digest": arguments.expected_planning_scene_revision_digest or "",
            "planning_scene_owned_ids": arguments.expected_planning_scene_owned_ids or "[]",
            "planning_scene_target_source_id": arguments.expected_planning_scene_target_source_id or "",
            "planning_scene_target_handoff": arguments.expected_planning_scene_target_handoff or "",
            "integrated_mapping": integrated,
            "integrated_sha256": arguments.expected_integrated_sha256 or "",
            "model_fingerprint": arguments.expected_model_fingerprint or "",
            "provider_manifest_path": str(arguments.provider_manifest or ""),
            "provider_manifest_sha256": arguments.provider_manifest_sha256 or "",
        }
        reasons = verify_expected_values(
            scenario_id=arguments.scenario,
            seed=arguments.seed,
            declaration=declaration,
            planning_scene=planning_scene,
            integrated=integrated,
            expected=expected,
        )
        if reasons:
            raise RuntimeError("scenario identity mismatch: {}".format("; ".join(reasons)))
        if arguments.provider_manifest is not None:
            provider_data = arguments.provider_manifest.read_bytes()
            actual_digest = sha256_bytes(provider_data)
            if actual_digest != arguments.provider_manifest_sha256:
                raise RuntimeError(
                    "provider manifest sha256 {} does not match bytes {}".format(
                        arguments.provider_manifest_sha256, actual_digest
                    )
                )

    operations = standard_operations(root, scenario, arguments.seed)
    rclpy.init(args=argv)
    node = ScenarioRunner(
        timeout_s=arguments.timeout,
        reset_attempts=arguments.reset_attempts,
        reset_retry_delay_s=arguments.reset_retry_delay,
    )
    try:
        results = node.execute(operations)
        if integrated is None:
            # Legacy non-overlay path: restore the byte/schema-compatible report
            # (top-level scenario string + seed + control_api/custom_control_
            # services + operations) and honor --report, exactly as before the
            # overlay landed.  The canonical compact report is produced only in
            # the integrated overlay mode.
            legacy_report = {
                "schema_version": 1,
                "scenario": scenario.scenario_id,
                "seed": arguments.seed,
                "control_api": "simulation_interfaces",
                "custom_control_services": False,
                "operations": results,
            }
            output = json.dumps(legacy_report, indent=2, sort_keys=True)
            if arguments.report is not None:
                write_report_atomic(legacy_report, arguments.report)
            print(output)
            return
        report = build_canonical_report(
            scenario_id=scenario.scenario_id,
            seed=arguments.seed,
            declaration=declaration,
            planning_scene=planning_scene,
            integrated=integrated,
            operations=results,
            model_fingerprint=arguments.expected_model_fingerprint or "",
            provider_manifest_sha256=arguments.provider_manifest_sha256 or "",
            final_simulation_state=FINAL_SIMULATION_STATE,
        )
        data = write_report_atomic(report, arguments.report) if arguments.report is not None else serialize_report(report)
        print(data.decode("utf-8"))
        if arguments.report is not None:
            print("scenario_report_sha256: {}".format(sha256_bytes(data)))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
