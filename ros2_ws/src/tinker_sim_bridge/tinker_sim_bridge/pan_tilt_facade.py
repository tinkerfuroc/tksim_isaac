from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tinker_vision_msgs_26.msg import PanTiltCommand, PanTiltState

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from tinker_sim_bridge.head_pose import resolve_initial_head_pose
from tinker_sim_bridge.head_tf import head_transforms


class PanTiltFacade(Node):
    def __init__(self) -> None:
        super().__init__("tinker_sim_pan_tilt_facade")
        self._pan = 0.0
        self._tilt = 0.0
        self._commands = self.create_publisher(
            JointState, "/sim/controller/pan_tilt_commands", 20
        )
        self._tf = TransformBroadcaster(self)
        self._states = self.create_publisher(
            PanTiltState, "/pan_tilt_controller/state", 20
        )
        self.create_subscription(
            PanTiltCommand, "/pan_tilt_controller/cmd", self._command, 20
        )
        self.create_subscription(
            JointState, "/isaac_joint_states", self._joint_state, 20
        )
        # The hardware pan-tilt controller drives its own startup pose
        # (initial_pan_deg / initial_tilt_deg in tk26_vision's pan_tilt.yaml,
        # both 0.0 -- at tilt 0 the head camera looks about level). This facade
        # stands in for that controller, so it takes the same knobs and the
        # same default. Holding the pose matters because nothing else in a
        # simulation bring-up commands the head.
        self.declare_parameter("initial_pan_deg", float("nan"))
        self.declare_parameter("initial_tilt_deg", float("nan"))
        self._initial_pan, self._initial_tilt = resolve_initial_head_pose(
            self._optional_degrees("initial_pan_deg"),
            self._optional_degrees("initial_tilt_deg"),
        )
        # Re-assert until the mechanism reports it arrived: a single publish
        # would be lost to a timeline reset, and the facade has no other way to
        # know the head actually moved.
        self._initial_pose_reached = False
        self.create_timer(0.5, self._drive_initial_pose)

    def _optional_degrees(self, name: str) -> float | None:
        """NaN is this node's "unset" -- rclpy has no optional double."""
        value = float(self.get_parameter(name).value)
        return None if math.isnan(value) else value

    def _drive_initial_pose(self) -> None:
        if self._initial_pose_reached:
            return
        if (
            abs(self._pan - self._initial_pan) < 0.02
            and abs(self._tilt - self._initial_tilt) < 0.02
        ):
            self._initial_pose_reached = True
            self.get_logger().info(
                f"head at initial pose: pan={self._initial_pan:.4f} rad "
                f"tilt={self._initial_tilt:.4f} rad"
            )
            return
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = ["pan_joint", "tilt_joint"]
        command.position = [self._initial_pan, self._initial_tilt]
        self._commands.publish(command)

    def _command(self, message: PanTiltCommand) -> None:
        if message.mode == PanTiltCommand.RELATIVE:
            pan = self._pan + float(message.pan_rad)
            tilt = self._tilt + float(message.tilt_rad)
        else:
            pan = float(message.pan_rad)
            tilt = float(message.tilt_rad)
        command = JointState()
        command.header = message.header
        command.name = ["pan_joint", "tilt_joint"]
        command.position = [pan, tilt]
        self._commands.publish(command)

    def _joint_state(self, message: JointState) -> None:
        try:
            pan_index = message.name.index("pan_joint")
            tilt_index = message.name.index("tilt_joint")
            self._pan = float(message.position[pan_index])
            self._tilt = float(message.position[tilt_index])
        except (ValueError, IndexError):
            return
        state = PanTiltState()
        state.header = message.header
        state.pan_rad = self._pan
        state.tilt_rad = self._tilt
        state.connected = True
        state.feedback_ok = True
        self._states.publish(state)
        self._broadcast_head_tf(message.header)

    def _broadcast_head_tf(self, header) -> None:
        """Publish the two transforms robot_state_publisher cannot.

        The head joints are not ros2_control joints, so they never appear in
        /joint_states and RSP never emits base_link -> pan_link or
        pan_link -> tilt_link. Everything below them is fixed and therefore
        does reach /tf_static, which leaves the whole head camera subtree
        floating unconnected from base_link -- and any detection asked for in
        map is silently dropped. Adding a second /joint_states publisher is
        not an option here (pick_and_place accepts exactly one), so the
        facade, which already owns this state, closes the chain itself.
        """
        for transform in head_transforms(self._pan, self._tilt):
            message = TransformStamped()
            message.header.stamp = header.stamp
            message.header.frame_id = transform.parent
            message.child_frame_id = transform.child
            message.transform.translation.x = transform.xyz[0]
            message.transform.translation.y = transform.xyz[1]
            message.transform.translation.z = transform.xyz[2]
            (
                message.transform.rotation.x,
                message.transform.rotation.y,
                message.transform.rotation.z,
                message.transform.rotation.w,
            ) = transform.quaternion_xyzw
            self._tf.sendTransform(message)


def main() -> None:
    rclpy.init()
    node = PanTiltFacade()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
