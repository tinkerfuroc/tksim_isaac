from __future__ import annotations

import json
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class InitialPose(Node):
    """Seed AMCL from the scenario spawn and confirm localization consumed it."""

    def __init__(self) -> None:
        super().__init__("tinker_sim_initial_pose")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("timeout_s", 30.0)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)
        self._status = self.create_publisher(String, "/sim/status/localization_seed", qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._confirmed, 10)
        self._started = time.monotonic()
        self._done = False
        self.create_timer(0.5, self._tick)

    def _tick(self) -> None:
        if self._done:
            return
        import math

        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = float(self.get_parameter("x").value)
        message.pose.pose.position.y = float(self.get_parameter("y").value)
        yaw = float(self.get_parameter("yaw").value)
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.068
        self._publisher.publish(message)
        if time.monotonic() - self._started > float(self.get_parameter("timeout_s").value):
            self._publish_status("fail", "AMCL did not confirm the scenario spawn pose")

    def _confirmed(self, _message: PoseWithCovarianceStamped) -> None:
        if not self._done:
            self._done = True
            self._publish_status("pass", "AMCL localization initialized")

    def _publish_status(self, state: str, detail: str) -> None:
        message = String()
        message.data = json.dumps({"state": state, "detail": detail}, sort_keys=True)
        self._status.publish(message)
        if state == "fail":
            self.get_logger().error(detail)
        else:
            self.get_logger().info(detail)


def main() -> None:
    rclpy.init()
    node = InitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
