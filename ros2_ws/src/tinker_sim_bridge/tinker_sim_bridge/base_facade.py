from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from tinker_sim_core.base import BaseParityModel, Twist2D
from tinker_sim_core.calibration import BaseCalibration


def _seconds(stamp: object) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class BaseFacade(Node):
    """Hardware-like Tracer boundary; never consumes simulator world pose."""

    def __init__(self) -> None:
        super().__init__("tinker_base_facade")
        self.declare_parameter("calibration", "")
        self.declare_parameter(
            "left_joints", ["front_left_wheel_joint", "rear_left_wheel_joint"]
        )
        self.declare_parameter(
            "right_joints", ["front_right_wheel_joint", "rear_right_wheel_joint"]
        )
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("battery_voltage", 29.0)
        raw_path = str(self.get_parameter("calibration").value)
        calibration = BaseCalibration.load(Path(raw_path) if raw_path else None)
        self._model = BaseParityModel(calibration)
        self._left_joints = tuple(self.get_parameter("left_joints").value)
        self._right_joints = tuple(self.get_parameter("right_joints").value)
        self._left = self._left_joints[0]
        self._right = self._right_joints[0]
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._last_stamp = -1.0
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self._command_pub = self.create_publisher(
            JointState, "/sim/controller/base_commands", reliable
        )
        self._odom_pub = self.create_publisher(Odometry, "/tracer/odom", reliable)
        self._status_pub = self.create_publisher(String, "/sim/status/base", reliable)
        self.create_subscription(Twist, "/cmd_vel", self._on_command, reliable)
        self.create_subscription(
            JointState, "/isaac_joint_states", self._on_joints, qos_profile_sensor_data
        )
        # The watchdog must keep running while /clock is paused.
        self.create_timer(
            1.0 / calibration.odom_rate_hz,
            self._publish_command,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self._tracer_pub = None
        try:
            from tracer_msgs.msg import TracerStatus
            self._tracer_type = TracerStatus
            self._tracer_pub = self.create_publisher(TracerStatus, "/tracer_status", reliable)
        except ImportError:
            self.get_logger().warning("tracer_msgs unavailable; /tracer_status disabled")
        self.get_logger().warning(
            f"base calibration={calibration.profile_id} status={calibration.status.value}; "
            "qualification is blocked" if calibration.qualification_error() else "base calibration qualified"
        )

    def _on_command(self, message: Twist) -> None:
        self._model.accept_command(Twist2D(message.linear.x, message.angular.z), time.monotonic())

    def _publish_command(self) -> None:
        command = self._model.wheel_command(time.monotonic())
        message = JointState()
        message.name = list(self._left_joints + self._right_joints)
        message.velocity = [
            *([command.left_rad_s] * len(self._left_joints)),
            *([command.right_rad_s] * len(self._right_joints)),
        ]
        self._command_pub.publish(message)
        status = String()
        status.data = json.dumps(
            {
                "calibration_status": self._model.calibration.status.value,
                "profile_id": self._model.calibration.profile_id,
                "watchdog_stop": command.watchdog_stop,
                "safety_stop": self._model.safety_stop,
                "truth_odometry": False,
            },
            sort_keys=True,
        )
        self._status_pub.publish(status)

    def _on_joints(self, message: JointState) -> None:
        try:
            left_index = message.name.index(self._left)
            right_index = message.name.index(self._right)
            left = float(message.velocity[left_index])
            right = float(message.velocity[right_index])
        except (ValueError, IndexError):
            return
        stamp = _seconds(message.header.stamp)
        if stamp <= self._last_stamp:
            return
        self._last_stamp = stamp
        estimate = self._model.observe_wheels(left, right, stamp)
        odom = Odometry()
        odom.header = message.header
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = estimate.x
        odom.pose.pose.position.y = estimate.y
        odom.pose.pose.orientation.z = math.sin(estimate.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(estimate.yaw / 2.0)
        odom.twist.twist.linear.x = estimate.linear_x
        odom.twist.twist.angular.z = estimate.angular_z
        calibration = self._model.calibration
        odom.pose.covariance[0] = calibration.pose_variance_xy
        odom.pose.covariance[7] = calibration.pose_variance_xy
        odom.pose.covariance[35] = calibration.pose_variance_yaw
        odom.twist.covariance[0] = calibration.twist_variance_linear
        odom.twist.covariance[35] = calibration.twist_variance_angular
        self._odom_pub.publish(odom)
        if self._tracer_pub is not None:
            status = self._tracer_type()
            status.header = message.header
            status.linear_velocity = estimate.linear_x
            status.angular_velocity = estimate.angular_z
            status.control_mode = 1
            status.error_code = 1 if self._model.safety_stop else 0
            status.battery_voltage = float(self.get_parameter("battery_voltage").value)
            rpm_scale = 60.0 / (2.0 * math.pi)
            status.actuator_states[0].rpm = int(right * rpm_scale)
            status.actuator_states[1].rpm = int(left * rpm_scale)
            self._tracer_pub.publish(status)


def main() -> None:
    rclpy.init()
    node = BaseFacade()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
