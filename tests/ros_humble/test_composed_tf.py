"""Task 6: real Humble composed-TF readiness evidence test.

``robot_state_publisher`` publishes one transform per joint (dynamic joints on
``/tf`` plus the fixed ``joint_tcp`` on ``/tf_static``); ``base_link ->
link_tcp`` is multi-hop and is never published as a single edge.  The
readiness node therefore composes the chain through ``tf2_ros.Buffer`` +
``TransformListener`` (which consume both ``/tf`` and ``/tf_static``).  These
tests prove the real node's ``_tf_evidence`` passes with a live multi-hop
chain and fails closed on missing and wrong-frame chains.

Requires the Humble ROS Python runtime; skipped under the simulator 3.12 venv.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="requires Humble ROS Python runtime")
pytest.importorskip("tf2_ros", reason="requires Humble tf2_ros")
pytest.importorskip("geometry_msgs", reason="requires Humble geometry_msgs")

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.integrated_readiness_node import IntegratedReadiness  # noqa: E402

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402


@pytest.fixture(scope="module")
def rclpy_context():
    if not rclpy.ok():
        rclpy.init(args=[])
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _make_transform(parent: str, child: str) -> TransformStamped:
    message = TransformStamped()
    message.header.stamp.sec = 1
    message.header.stamp.nanosec = 0
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = 0.1
    message.transform.translation.y = 0.0
    message.transform.translation.z = 0.2
    message.transform.rotation.w = 1.0
    return message


def _publish(tf_pub, static_pub, dynamic: list[tuple[str, str]], static: list[tuple[str, str]]) -> None:
    dynamic_msg = TFMessage()
    dynamic_msg.transforms = [_make_transform(p, c) for p, c in dynamic]
    tf_pub.publish(dynamic_msg)
    static_msg = TFMessage()
    static_msg.transforms = [_make_transform(p, c) for p, c in static]
    static_pub.publish(static_msg)


def _static_qos():
    # tf2_ros.TransformListener subscribes to /tf_static with TRANSIENT_LOCAL
    # durability (static transforms are latched); a VOLATILE publisher cannot
    # match that subscription, so the static edge would never reach the buffer.
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _spin(executor, count: int = 30) -> None:
    for _ in range(count):
        executor.spin_once(timeout_sec=0.05)


def _readiness_node() -> IntegratedReadiness:
    node = IntegratedReadiness(
        node_name="composed_tf_test_reader",
        create_status_publisher=False,
    )
    return node


def test_composed_multihop_chain_passes(rclpy_context) -> None:
    """Dynamic base_link->link_1->link_2 on /tf + fixed link_2->link_tcp on
    /tf_static composes into a base_link->link_tcp lookup."""
    tf_pub_node = Node("composed_tf_test_publisher")
    tf_pub = tf_pub_node.create_publisher(TFMessage, "/tf", 10)
    static_pub = tf_pub_node.create_publisher(TFMessage, "/tf_static", _static_qos())
    reader = _readiness_node()
    executor = SingleThreadedExecutor()
    executor.add_node(tf_pub_node)
    executor.add_node(reader)
    try:
        _spin(executor)
        _publish(
            tf_pub,
            static_pub,
            dynamic=[("base_link", "link_1"), ("link_1", "link_2")],
            static=[("link_2", "link_tcp")],
        )
        _spin(executor, count=60)
        evidence = reader._tf_evidence()
        assert evidence["ready"] is True, evidence["reasons"]
        assert evidence["parent"] == "base_link"
        assert evidence["child"] == "link_tcp"
        assert evidence["stamp_ns"] != 0
    finally:
        executor.remove_node(tf_pub_node)
        executor.remove_node(reader)
        executor.shutdown()
        reader.destroy_node()
        tf_pub_node.destroy_node()


def test_missing_chain_fails_closed(rclpy_context) -> None:
    """An empty TF tree yields a fail-closed composed lookup."""
    reader = _readiness_node()
    executor = SingleThreadedExecutor()
    executor.add_node(reader)
    try:
        _spin(executor)
        evidence = reader._tf_evidence()
        assert evidence["ready"] is False
        assert evidence["exists"] is False
        assert any("composed lookup failed" in reason for reason in evidence["reasons"])
    finally:
        executor.remove_node(reader)
        executor.shutdown()
        reader.destroy_node()


def test_wrong_frame_fails_closed(rclpy_context) -> None:
    """A chain that never connects link_tcp cannot compose base_link->link_tcp."""
    tf_pub_node = Node("composed_tf_test_wrong")
    tf_pub = tf_pub_node.create_publisher(TFMessage, "/tf", 10)
    static_pub = tf_pub_node.create_publisher(TFMessage, "/tf_static", _static_qos())
    reader = _readiness_node()
    executor = SingleThreadedExecutor()
    executor.add_node(tf_pub_node)
    executor.add_node(reader)
    try:
        _spin(executor)
        _publish(
            tf_pub,
            static_pub,
            dynamic=[("base_link", "link_1")],
            static=[("link_1", "other_tcp")],
        )
        _spin(executor, count=60)
        evidence = reader._tf_evidence()
        assert evidence["ready"] is False
        assert evidence["exists"] is False
        assert any("composed lookup failed" in reason for reason in evidence["reasons"])
    finally:
        executor.remove_node(tf_pub_node)
        executor.remove_node(reader)
        executor.shutdown()
        reader.destroy_node()
        tf_pub_node.destroy_node()
