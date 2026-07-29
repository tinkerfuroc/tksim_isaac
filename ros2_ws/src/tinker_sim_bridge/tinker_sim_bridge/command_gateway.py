from __future__ import annotations

import json
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from tinker_sim_core.command_mux import (
    CommandSource,
    JointCommandMux,
    command_from_sequences,
    encode_command_epoch,
    encode_command_frame,
    encode_snapshot_packet,
    new_command_session,
)


class CommandGateway(Node):
    """The only ROS publisher allowed to command the Isaac articulation."""

    SOURCE_TOPICS = {
        "base": "/sim/controller/base_commands",
        "ros2_control": "/sim/controller/ros2_control_commands",
        "gripper": "/sim/controller/gripper_commands",
        "pan_tilt": "/sim/controller/pan_tilt_commands",
    }

    def __init__(self) -> None:
        super().__init__("tinker_sim_command_gateway")
        self.declare_parameter(
            "base_joints",
            [
                "front_left_wheel_joint",
                "front_right_wheel_joint",
                "rear_left_wheel_joint",
                "rear_right_wheel_joint",
            ],
        )
        self.declare_parameter(
            "arm_joints", [f"joint{index}" for index in range(1, 8)]
        )
        self.declare_parameter("gripper_joints", ["drive_joint"])
        self.declare_parameter("pan_tilt_joints", ["pan_joint", "tilt_joint"])
        self.declare_parameter("base_timeout_s", 0.25)
        self.declare_parameter("controller_timeout_s", 0.5)
        self.declare_parameter("safety_timeout_s", 1.0)
        sources = {
            "base": CommandSource(
                frozenset(self.get_parameter("base_joints").value),
                float(self.get_parameter("base_timeout_s").value),
            ),
            "ros2_control": CommandSource(
                frozenset(self.get_parameter("arm_joints").value),
                float(self.get_parameter("controller_timeout_s").value),
            ),
            "gripper": CommandSource(
                frozenset(self.get_parameter("gripper_joints").value), 0.5
            ),
            "pan_tilt": CommandSource(
                frozenset(self.get_parameter("pan_tilt_joints").value), 0.5
            ),
        }
        self._mux = JointCommandMux(sources)
        self._rejected: dict[str, str] = {}
        # Discovery is not proof of an effective safety-clear state.
        self._safety_active = True
        self._safety_timeout_s = float(self.get_parameter("safety_timeout_s").value)
        if self._safety_timeout_s <= 0.0:
            raise ValueError("safety timeout must be positive")
        self._safety_last_sample_at: float | None = None
        self._mux.stop(True)
        # The gateway is the sole authority for command epochs.  The random
        # session prevents a restarted gateway from reusing delayed packets;
        # the generation changes on every effective safety transition.
        self._command_session_id = new_command_session()
        self._command_generation = 0
        self._command_epoch = encode_command_epoch(
            self._command_session_id, self._command_generation
        )
        self._snapshot_id = 0
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        safety_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._publisher = self.create_publisher(
            JointState, "/isaac_joint_commands", reliable
        )
        self._status = self.create_publisher(
            String, "/sim/status/command_gateway", reliable
        )
        for source, topic in self.SOURCE_TOPICS.items():
            self.create_subscription(
                JointState,
                topic,
                lambda message, source=source: self._accept(source, message),
                reliable,
            )
        self.create_subscription(
            JointState, "/isaac_joint_states", self._joint_state, reliable
        )
        self.create_subscription(
            Bool, "/sim/hardware/safety_stop", self._safety_stop, safety_qos
        )
        self.create_timer(
            1.0 / 150.0,
            self._publish,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _accept(self, source: str, message: JointState) -> None:
        if self._safety_active:
            self._rejected[source] = "blocked by safety stop"
            return
        self._enforce_safety_deadline()
        if self._safety_active:
            self._rejected[source] = "blocked by safety stop"
            return
        try:
            command = command_from_sequences(
                message.name, message.position, message.velocity, message.effort
            )
            self._mux.accept(source, command, time.monotonic())
            self._rejected.pop(source, None)
        except Exception as error:
            self._rejected[source] = str(error)
            self.get_logger().error(f"rejected {source} joint command: {error}")

    def _advance_command_epoch(self) -> None:
        """Advance the gateway-owned epoch without a peer-local counter."""
        if not hasattr(self, "_command_session_id"):
            # Keep lightweight object.__new__ test fixtures compatible with
            # the pre-session protocol; production instances always use the
            # session-aware branch.
            self._command_epoch += 1
            return
        self._command_generation += 1
        self._command_epoch = encode_command_epoch(
            self._command_session_id, self._command_generation
        )

    def _safety_stop(self, message: Bool) -> None:
        self._safety_last_sample_at = time.monotonic()
        active = bool(message.data)
        if active == self._safety_active:
            return
        self._safety_active = active
        if hasattr(self, "_command_session_id"):
            self._advance_command_epoch()
        else:
            self._command_epoch += 1
        self._mux.stop(active)
        # A clear boundary starts a fresh command baseline.  Stopped packets
        # published during discovery cannot consume the post-clear sequence.
        self._snapshot_id = 0

    def _enforce_safety_deadline(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else float(now)
        last = getattr(self, "_safety_last_sample_at", None)
        timeout = getattr(self, "_safety_timeout_s", 1.0)
        if last is not None and now - last < timeout:
            return
        if self._safety_active:
            return
        self._safety_active = True
        self._advance_command_epoch()
        self._mux.stop(True)
        self._snapshot_id = 0
        self._rejected["safety"] = "safety heartbeat expired"

    def _joint_state(self, message: JointState) -> None:
        if not message.position:
            return
        try:
            self._mux.observe_positions(message.name, message.position)
        except ValueError as error:
            self.get_logger().error(f"rejected observed joint state: {error}")

    def _publish(self) -> None:
        self._enforce_safety_deadline()
        commands = self._mux.compose(time.monotonic())
        snapshot_id = self._snapshot_id
        self._snapshot_id += 1
        packet_count = len(commands)
        for packet_index, command in enumerate(commands, start=1):
            frame_id = encode_command_frame(
                self._command_epoch,
                encode_snapshot_packet(snapshot_id, packet_count, packet_index),
            )
            message = JointState()
            message.header.frame_id = frame_id
            message.name = list(command.names)
            message.position = list(command.positions)
            message.velocity = list(command.velocities)
            message.effort = list(command.efforts)
            self._publisher.publish(message)
        status = String()
        status.data = json.dumps(
            {
                "sole_isaac_command_publisher": True,
                "safety_stop": self._mux.safety_stop,
                "command_epoch": self._command_epoch,
                "snapshot_id": snapshot_id,
                "packet_count": packet_count,
                "owners": self._mux.owners,
                "rejected": self._rejected,
                "packets": len(commands),
            },
            sort_keys=True,
        )
        self._status.publish(status)


def main() -> None:
    rclpy.init()
    node = CommandGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
