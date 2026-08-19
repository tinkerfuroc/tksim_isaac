from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from simulation_interfaces.srv import SetEntityState

from tinker_sim_core.actor_path import path_length, path_pose_at
from tinker_sim_core.scenario import load_named_scenario


class ActorPathDriver(Node):
    """Drive scenario actors along their declared paths via /set_entity_state.

    ScenarioRunner is one-shot by contract (launch gates key off its exit),
    so path execution lives in this separate, also one-shot, node.
    """

    def __init__(self, timeout_s: float) -> None:
        super().__init__("tinker_sim_actor_path_driver")
        self._sim_time: float | None = None
        self.create_subscription(Clock, "/clock", self._clock, 10)
        self._client = self.create_client(SetEntityState, "/set_entity_state")
        if not self._client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError("/set_entity_state is not available")

    def _clock(self, message: Clock) -> None:
        self._sim_time = message.clock.sec + message.clock.nanosec * 1e-9

    def wait_sim_time(self, at_least: float, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._sim_time is not None and self._sim_time >= at_least:
                return True
        return False

    def place(self, prim_path: str, x: float, y: float, yaw: float, timeout_s: float) -> bool:
        request = SetEntityState.Request()
        request.entity = prim_path
        request.state.header.frame_id = "world"
        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.orientation.z = math.sin(yaw / 2.0)
        request.state.pose.orientation.w = math.cos(yaw / 2.0)
        future = self._client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done() and future.result() is not None


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--speed-mps", type=float, default=0.3)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    arguments = parser.parse_args(
        rclpy.utilities.remove_ros_args(args=sys.argv if argv is None else argv)[1:]
    )
    if arguments.speed_mps <= 0.0 or arguments.rate_hz <= 0.0:
        raise SystemExit("--speed-mps and --rate-hz must be positive")

    root = arguments.root.resolve()
    scenario = load_named_scenario(root, arguments.scenario)
    actor_records = {str(record["id"]): record for record in scenario.actors}
    walks = []
    for event in scenario.events:
        if event.get("type") != "actor_path_start":
            continue
        record = actor_records[str(event["actor"])]
        walks.append(
            (
                float(event.get("at_sim_time", 0.0)),
                f"/World/Scenario/{record['id']}",
                list(record["path"]),
            )
        )
    if not walks:
        print("no actor_path_start events; nothing to drive")
        return

    rclpy.init(args=argv)
    node = ActorPathDriver(timeout_s=arguments.timeout)
    try:
        for at_sim_time, prim_path, path in sorted(walks):
            if not node.wait_sim_time(at_sim_time, arguments.timeout):
                raise SystemExit(f"sim time {at_sim_time} not reached")
            total = path_length(path)
            start = time.monotonic()
            period = 1.0 / arguments.rate_hz
            while True:
                distance = min(total, (time.monotonic() - start) * arguments.speed_mps)
                x, y, yaw = path_pose_at(path, distance)
                if not node.place(prim_path, x, y, yaw, arguments.timeout):
                    raise SystemExit(f"set_entity_state failed for {prim_path}")
                if distance >= total:
                    break
                time.sleep(period)
            print(f"actor {prim_path} completed {total:.2f} m path")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
