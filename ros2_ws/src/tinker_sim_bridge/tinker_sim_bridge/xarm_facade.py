from __future__ import annotations

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import Call, SetInt16, SetInt16ById


class XArmFacade(Node):
    """Minimum driver-compatible state and safety services for MoveIt tasks."""

    JOINTS = tuple(f"joint{index}" for index in range(1, 8))
    SAFETY_HEARTBEAT_PERIOD_S = 0.25

    def __init__(self) -> None:
        super().__init__("tinker_sim_xarm_facade")
        self._enabled = True
        self._mode = 0
        self._state = 2
        self._robot = self.create_publisher(RobotMsg, "/xarm/robot_states", 20)
        self._joints = self.create_publisher(JointState, "/xarm/joint_states", 50)
        safety_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._safety = self.create_publisher(Bool, "/sim/safety/xarm", safety_qos)
        self.create_subscription(
            JointState, "/isaac_joint_states", self._on_joints, 50
        )
        self.create_service(SetInt16ById, "/xarm/motion_enable", self._motion_enable)
        self.create_service(SetInt16, "/xarm/set_mode", self._set_mode)
        self.create_service(SetInt16, "/xarm/set_state", self._set_state)
        self.create_service(Call, "/xarm/clear_err", self._clear)
        self.create_service(Call, "/xarm/clear_warn", self._clear)
        self.create_timer(
            self.SAFETY_HEARTBEAT_PERIOD_S,
            self._publish_safety,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self._publish_safety()

    @staticmethod
    def _ok(response, message: str = "simulated") -> object:
        response.ret = 0
        response.message = message
        return response

    def _motion_enable(self, request, response):
        self._enabled = bool(request.data)
        self._publish_safety()
        return self._ok(response)

    def _set_mode(self, request, response):
        if request.data not in (0, 1):
            response.ret = -1
            response.message = "simulation supports POSITION and SERVOJ modes only"
            return response
        self._mode = int(request.data)
        return self._ok(response)

    def _set_state(self, request, response):
        self._state = int(request.data)
        self._publish_safety()
        return self._ok(response)

    def _clear(self, _request, response):
        return self._ok(response)

    def _publish_safety(self) -> None:
        message = Bool()
        message.data = not self._enabled or self._state == 4
        self._safety.publish(message)

    def _on_joints(self, message: JointState) -> None:
        indices = []
        try:
            indices = [message.name.index(name) for name in self.JOINTS]
        except ValueError:
            return
        arm = JointState()
        arm.header = message.header
        arm.name = list(self.JOINTS)
        if message.position:
            arm.position = [message.position[index] for index in indices]
        if message.velocity:
            arm.velocity = [message.velocity[index] for index in indices]
        if message.effort:
            arm.effort = [message.effort[index] for index in indices]
        self._joints.publish(arm)
        state = RobotMsg()
        state.header = message.header
        state.state = self._state
        state.mode = self._mode
        state.cmdnum = 0
        state.mt_brake = 127
        state.mt_able = 127 if self._enabled else 0
        state.err = 0
        state.warn = 0
        state.angle = [float(value) for value in arm.position]
        self._robot.publish(state)


def main() -> None:
    rclpy.init()
    node = XArmFacade()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
