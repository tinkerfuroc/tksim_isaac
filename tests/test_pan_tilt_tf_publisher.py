"""Unit tests for the URDF parsing + transform math in pan_tilt_tf_publisher.

Only the pure parts are tested (parse_urdf_joints, JointSpec.transform,
quaternion helpers); the rclpy node needs a live ROS graph and is exercised by
the stack itself. The math must match robot_state_publisher's convention:
child transform = joint origin * R(axis, position).
"""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from pan_tilt_tf_publisher import (  # noqa: E402
    JointSpec,
    _quat_from_axis_angle,
    _quat_from_rpy,
    _quat_mul,
    parse_urdf_joints,
)

URDF = """
<robot name="tinker">
  <link name="head_base"/><link name="pan_link"/><link name="tilt_link"/><link name="foot"/>
  <joint name="pan_joint" type="revolute">
    <parent link="head_base"/><child link="pan_link"/>
    <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 -1"/>
  </joint>
  <joint name="tilt_joint" type="revolute">
    <parent link="pan_link"/><child link="tilt_link"/>
    <origin xyz="0 0 0.05" rpy="0 0.1 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  <joint name="fixed_foot" type="fixed">
    <parent link="head_base"/><child link="foot"/>
  </joint>
</robot>
"""


def test_parse_extracts_revolute_joints_only():
    joints = parse_urdf_joints(URDF)
    assert set(joints) == {"pan_joint", "tilt_joint"}
    pan = joints["pan_joint"]
    assert pan.parent == "head_base"
    assert pan.child == "pan_link"
    assert pan.xyz == (0.1, 0.0, 0.2)
    assert pan.axis == (0.0, 0.0, -1.0)


def test_zero_position_reproduces_origin():
    joints = parse_urdf_joints(URDF)
    xyz, quat = joints["tilt_joint"].transform(0.0)
    assert xyz == (0.0, 0.0, 0.05)
    expected = _quat_from_rpy(0.0, 0.1, 0.0)
    assert quat == pytest.approx(expected)


def test_pan_negative_z_axis_matches_negative_yaw():
    # URDF pan axis "0 0 -1": a +0.5 rad joint value is a -0.5 rad yaw.
    joints = parse_urdf_joints(URDF)
    _, quat = joints["pan_joint"].transform(0.5)
    assert quat == pytest.approx(_quat_from_rpy(0.0, 0.0, -0.5))


def test_tilt_composes_origin_pitch_with_joint_pitch():
    # Origin pitch 0.1 and axis "0 1 0" with q=0.2 compose to pitch 0.3.
    joints = parse_urdf_joints(URDF)
    _, quat = joints["tilt_joint"].transform(0.2)
    assert quat == pytest.approx(_quat_from_rpy(0.0, 0.3, 0.0))


def test_axis_angle_normalizes_axis():
    q_unit = _quat_from_axis_angle((0.0, 0.0, 1.0), 0.7)
    q_scaled = _quat_from_axis_angle((0.0, 0.0, 10.0), 0.7)
    assert q_scaled == pytest.approx(q_unit)


def test_quat_mul_identity():
    q = _quat_from_rpy(0.3, -0.2, 1.1)
    ident = (0.0, 0.0, 0.0, 1.0)
    assert _quat_mul(q, ident) == pytest.approx(q)
    assert _quat_mul(ident, q) == pytest.approx(q)


def test_offset_origin_translation_unchanged_by_rotation():
    spec = JointSpec("a", "b", (1.0, 2.0, 3.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    xyz, _ = spec.transform(1.234)
    assert xyz == (1.0, 2.0, 3.0)
