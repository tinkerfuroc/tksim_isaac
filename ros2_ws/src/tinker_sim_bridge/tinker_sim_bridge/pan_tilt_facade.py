from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tinker_vision_msgs_26.msg import PanTiltCommand, PanTiltState


class PanTiltFacade(Node):
    def __init__(self) -> None:
        super().__init__("tinker_sim_pan_tilt_facade")
        self._pan = 0.0
        self._tilt = 0.0
        self._commands = self.create_publisher(
            JointState, "/sim/controller/pan_tilt_commands", 20
        )
        self._states = self.create_publisher(
            PanTiltState, "/pan_tilt_controller/state", 20
        )
        self.create_subscription(
            PanTiltCommand, "/pan_tilt_controller/cmd", self._command, 20
        )
        self.create_subscription(
            JointState, "/isaac_joint_states", self._joint_state, 20
        )

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
