from __future__ import annotations

import argparse
import json
import sys
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


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--reset-attempts", type=int, default=3)
    parser.add_argument("--reset-retry-delay", type=float, default=0.5)
    parser.add_argument("--report", type=Path)
    arguments = parser.parse_args(
        rclpy.utilities.remove_ros_args(args=sys.argv if argv is None else argv)[1:]
    )
    root = arguments.root.resolve()
    scenario = load_named_scenario(root, arguments.scenario)
    operations = standard_operations(root, scenario, arguments.seed)
    rclpy.init(args=argv)
    node = ScenarioRunner(
        timeout_s=arguments.timeout,
        reset_attempts=arguments.reset_attempts,
        reset_retry_delay_s=arguments.reset_retry_delay,
    )
    try:
        results = node.execute(operations)
        report = {
            "schema_version": 1,
            "scenario": scenario.scenario_id,
            "seed": arguments.seed,
            "control_api": "simulation_interfaces",
            "custom_control_services": False,
            "operations": results,
        }
        output = json.dumps(report, indent=2, sort_keys=True)
        if arguments.report is not None:
            arguments.report.parent.mkdir(parents=True, exist_ok=True)
            arguments.report.write_text(output + "\n", encoding="utf-8")
        print(output)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
