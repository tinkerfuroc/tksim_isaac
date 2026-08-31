#!/usr/bin/env python3
"""Broadcast pan/tilt joint TFs from a JointState side-topic.

Why this exists: the GPSR stack needs the head-camera TF subtree connected
(pan/tilt joint transforms), but pick_and_place's live-manipulation readiness
contract requires /joint_states to have EXACTLY ONE publisher (the ros2_control
joint_state_broadcaster) — it rejects every arm goal otherwise ("controller
manager observation refresh failed", observed 2026-08-31 on the first live-manip
run). So the pan_tilt state publisher is remapped to publish its offset-corrected
JointState on a side topic, and this node turns those messages into the same
TF robot_state_publisher would have produced: for each named joint found in the
URDF (from /robot_description), publish parent->child = joint origin * R(axis, q).

Deliberately generic over joint names: every revolute/continuous URDF joint whose
name appears in the incoming JointState gets a transform; unknown names warn once.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quat_from_axis_angle(axis: tuple[float, float, float], angle: float) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(a * a for a in axis))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2) / norm
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def _quat_mul(a, b) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


class JointSpec:
    __slots__ = ("parent", "child", "xyz", "origin_quat", "axis")

    def __init__(self, parent: str, child: str, xyz, rpy, axis):
        self.parent = parent
        self.child = child
        self.xyz = xyz
        self.origin_quat = _quat_from_rpy(*rpy)
        self.axis = axis

    def transform(self, position: float) -> tuple[tuple, tuple]:
        # robot_state_publisher convention: T = origin * R(axis, q). Rotation
        # about the joint axis leaves the translation (origin xyz) unchanged.
        return self.xyz, _quat_mul(self.origin_quat, _quat_from_axis_angle(self.axis, position))


def parse_urdf_joints(urdf_xml: str) -> dict[str, JointSpec]:
    """Extract every revolute/continuous joint's parent, child, origin, axis."""
    joints: dict[str, JointSpec] = {}
    root = ET.fromstring(urdf_xml)
    for joint in root.iter("joint"):
        if joint.get("type") not in ("revolute", "continuous"):
            continue
        name = joint.get("name")
        parent = joint.find("parent")
        child = joint.find("child")
        if name is None or parent is None or child is None:
            continue
        origin = joint.find("origin")
        xyz = tuple(float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split())
        rpy = tuple(float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split())
        axis_el = joint.find("axis")
        axis = tuple(float(v) for v in (axis_el.get("xyz", "1 0 0") if axis_el is not None else "1 0 0").split())
        joints[name] = JointSpec(parent.get("link"), child.get("link"), xyz, rpy, axis)
    return joints


class PanTiltTfPublisher(Node):
    def __init__(self):
        super().__init__("pan_tilt_tf_publisher")
        self.declare_parameter("joint_state_topic", "/pan_tilt/joint_states")
        self._joints: dict[str, JointSpec] | None = None
        self._warned_unknown: set[str] = set()
        self._broadcaster = TransformBroadcaster(self)
        # robot_state_publisher publishes /robot_description transient-local.
        self.create_subscription(
            String,
            "/robot_description",
            self._handle_description,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_state_topic").value,
            self._handle_joint_state,
            10,
        )

    def _handle_description(self, msg: String):
        try:
            self._joints = parse_urdf_joints(msg.data)
        except ET.ParseError as exc:
            self.get_logger().error(f"robot_description parse failed: {exc}")
            return
        self.get_logger().info(
            f"URDF loaded: {len(self._joints)} revolute/continuous joints available"
        )

    def _handle_joint_state(self, msg: JointState):
        if self._joints is None:
            return
        transforms = []
        for name, position in zip(msg.name, msg.position):
            spec = self._joints.get(name)
            if spec is None:
                if name not in self._warned_unknown:
                    self._warned_unknown.add(name)
                    self.get_logger().warning(f"joint '{name}' not in URDF; ignoring")
                continue
            xyz, quat = spec.transform(position)
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = spec.parent
            t.child_frame_id = spec.child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            (t.transform.rotation.x, t.transform.rotation.y,
             t.transform.rotation.z, t.transform.rotation.w) = quat
            transforms.append(t)
        if transforms:
            self._broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
