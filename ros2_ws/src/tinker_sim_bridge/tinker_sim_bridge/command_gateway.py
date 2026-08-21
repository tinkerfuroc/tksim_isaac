from __future__ import annotations

import json
import math
import time
from typing import Sequence

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from tinker_sim_core.command_mux import (
    CommandSource,
    JointCommand,
    JointCommandMux,
    encode_command_epoch,
    encode_command_frame,
    encode_snapshot_packet,
    new_command_session,
)


class CommandGateway(Node):
    """The only ROS publisher allowed to command the Isaac articulation."""

    KEEPALIVE_PERIOD_S = 1.0 / 20.0
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
        # The 150 Hz tick still evaluates deadlines and composes the snapshot,
        # but an unchanged snapshot is only re-sent at the keepalive cadence.
        # Every packet the simulator receives costs it main-loop time (GIL
        # hand-off plus a PhysX target write), and resending four identical
        # packets 150 times a second was measured to cut its real-time factor
        # from ~0.8 to ~0.23.  The simulator's command-stream watchdog is 0.5 s,
        # so a 20 Hz keepalive keeps a 10x margin.
        self._last_published_commands: tuple | None = None
        self._last_publish_at = -math.inf
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
            command = self._owned_command(
                source,
                message.name,
                message.position,
                message.velocity,
                message.effort,
            )
            self._mux.accept(source, command, time.monotonic())
            self._rejected.pop(source, None)
        except Exception as error:
            self._rejected[source] = str(error)
            self.get_logger().error(f"rejected {source} joint command: {error}")

    def _owned_command(
        self,
        source: str,
        names: Sequence[str],
        positions: Sequence[float],
        velocities: Sequence[float],
        efforts: Sequence[float],
    ) -> JointCommand:
        """Project a source message onto the joints that source is allowed to own.

        The vendored topic_based_ros2_control publishes all eight joints in
        ``names`` (the seven arm joints plus a state-only ``drive_joint``) but
        carries values only for the seven arm joints, so a raw JointCommand is
        rejected before mux ownership filtering can drop ``drive_joint``.
        Normalize before validation: unowned names are removed together with
        their values, and the live value-shorter-than-names shape is accepted
        only when the owned names form a contiguous prefix so the value-to-name
        mapping is unambiguous.  Any other layout is malformed and rejected.
        """
        owned = self._mux.sources[source].joints
        owned_names = [name for name in names if name in owned]
        if not owned_names:
            raise ValueError(f"{source} message contains no owned joints")
        owned_name_count = len(owned_names)
        name_count = len(names)

        def project(values: Sequence[float]) -> tuple[float, ...]:
            if not values:
                # Empty channels are absent, matching JointCommand defaults.
                return ()
            value_count = len(values)
            if value_count == name_count:
                # Canonical parallel arrays: value[i] belongs to name[i], so
                # drop each unowned name together with its value.
                return tuple(
                    float(value) for name, value in zip(names, values) if name in owned
                )
            if (
                value_count == owned_name_count
                and owned_names == list(names[:owned_name_count])
            ):
                # Live shape: values are emitted only for the owned joints in
                # name order; the prefix guard keeps this unambiguous.
                return tuple(float(value) for value in values)
            raise ValueError(
                f"{source} {value_count} values do not align with "
                f"{name_count} names / {owned_name_count} owned joints"
            )

        return JointCommand(
            tuple(owned_names),
            project(positions),
            project(velocities),
            project(efforts),
        )

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
        # A new epoch must reach the simulator on the next tick even if the
        # composed packets are unchanged.
        self._last_published_commands = None

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
        # A new epoch must reach the simulator on the next tick even if the
        # composed packets are unchanged.
        self._last_published_commands = None
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
        now = time.monotonic()
        commands = tuple(self._mux.compose(now))
        if (
            commands == getattr(self, "_last_published_commands", None)
            and now - getattr(self, "_last_publish_at", -math.inf)
            < self.KEEPALIVE_PERIOD_S
        ):
            return
        self._last_published_commands = commands
        self._last_publish_at = now
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
