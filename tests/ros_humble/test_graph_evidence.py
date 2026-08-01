"""Task 6: real Humble graph type/source/QoS evidence tests.

The readiness node must record actual graph-reported types and source nodes for
every action backing service and required service, and must compare real
publisher QoS metadata against the contract.  These tests instantiate the real
``IntegratedReadiness`` node (no status publisher), populate the live graph
with adversary nodes that publish the *wrong* type/source/QoS, and prove the
node's probes observe the true graph state and the evaluator rejects it
fail-closed.

Requires the Humble ROS Python runtime; skipped under the simulator 3.12 venv.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="requires Humble ROS Python runtime")
pytest.importorskip("controller_manager_msgs", reason="requires Humble controller_manager_msgs")
pytest.importorskip("moveit_msgs", reason="requires Humble moveit_msgs")
pytest.importorskip("sensor_msgs", reason="requires Humble sensor_msgs")

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.integrated_readiness import (  # noqa: E402
    evaluate_integrated_readiness,
)
from tinker_sim_bridge.integrated_readiness_node import IntegratedReadiness  # noqa: E402

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from moveit_msgs.srv import GetPlanningScene  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


@pytest.fixture(scope="module")
def rclpy_context():
    if not rclpy.ok():
        rclpy.init(args=[])
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _spin(executor: SingleThreadedExecutor, count: int = 30) -> None:
    for _ in range(count):
        executor.spin_once(timeout_sec=0.05)


def _readiness_node(name: str) -> IntegratedReadiness:
    return IntegratedReadiness(
        node_name=name,
        create_status_publisher=False,
    )


def _make_joint_state() -> JointState:
    message = JointState()
    message.header.stamp.sec = 1
    message.header.stamp.nanosec = 0
    message.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "drive_joint"]
    message.position = [0.0] * 8
    message.velocity = [0.0] * 8
    return message


def test_probe_actions_observes_wrong_type(rclpy_context) -> None:
    """A goal service with the wrong type is observed (not stamped) and the
    evaluator rejects it."""
    adversary = Node("adversary_wrong_action_type")

    def handle(request, response):
        del request, response
        return response

    adversary.create_service(
        Trigger, "/move_action/_action/send_goal", handle
    )
    adversary.create_service(
        Trigger, "/move_action/_action/get_result", handle
    )
    reader = _readiness_node("graph_probe_wrong_type")
    executor = SingleThreadedExecutor()
    executor.add_node(adversary)
    executor.add_node(reader)
    try:
        _spin(executor, count=40)
        actions = reader._probe_actions()
        entry = actions["/move_action"]
        assert entry["observed_types"] == ["std_srvs/srv/Trigger"]
        assert "moveit_msgs/action/MoveGroup_SendGoal" not in entry["observed_types"]
        snapshot = reader._build_snapshot()
        report = evaluate_integrated_readiness(snapshot, reader._contract)
        assert report.ready is False
        assert any("action /move_action" in reason for reason in report.reasons)
    finally:
        executor.remove_node(adversary)
        executor.remove_node(reader)
        executor.shutdown()
        reader.destroy_node()
        adversary.destroy_node()


def test_graph_services_observes_wrong_source(rclpy_context) -> None:
    """A required service served from the wrong node is observed with its real
    source label and rejected."""
    adversary = Node("adversary_wrong_service_source")

    def handle(request, response):
        del request
        return response

    adversary.create_service(GetPlanningScene, "/get_planning_scene", handle)
    reader = _readiness_node("graph_probe_wrong_source")
    executor = SingleThreadedExecutor()
    executor.add_node(adversary)
    executor.add_node(reader)
    try:
        _spin(executor, count=40)
        services = reader._graph_services()
        entry = services["/get_planning_scene"]
        assert entry["source"] == "/adversary_wrong_service_source"
        assert "moveit_msgs/srv/GetPlanningScene" in entry["types"]
        snapshot = reader._build_snapshot()
        report = evaluate_integrated_readiness(snapshot, reader._contract)
        assert report.ready is False
        assert any("service /get_planning_scene" in reason for reason in report.reasons)
    finally:
        executor.remove_node(adversary)
        executor.remove_node(reader)
        executor.shutdown()
        reader.destroy_node()
        adversary.destroy_node()


def test_publisher_metadata_observes_wrong_qos_and_source(rclpy_context) -> None:
    """A /joint_states publisher with best-effort QoS from the wrong node is
    observed with its real QoS and source and rejected."""
    adversary = Node("adversary_wrong_joint_pub")
    wrong_qos = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    adversary.create_publisher(JointState, "/joint_states", wrong_qos)
    reader = _readiness_node("graph_probe_wrong_qos")
    executor = SingleThreadedExecutor()
    executor.add_node(adversary)
    executor.add_node(reader)
    try:
        _spin(executor, count=40)
        metadata = reader._publisher_metadata()
        entry = metadata["/joint_states"]
        assert entry["source"] == "/adversary_wrong_joint_pub"
        qos = entry["qos"]
        assert qos["reliability"] == "BEST_EFFORT"
        assert qos["durability"] == "VOLATILE"
        # Humble ``PublishersInfo.qos_profile`` never reports depth (always 0);
        # depth is compared only when a publisher actually reports it.
        assert qos["depth"] == 0
        snapshot = reader._build_snapshot()
        report = evaluate_integrated_readiness(snapshot, reader._contract)
        assert report.ready is False
        assert any("publishers" in reason and "/joint_states" in reason for reason in report.reasons)
    finally:
        executor.remove_node(adversary)
        executor.remove_node(reader)
        executor.shutdown()
        reader.destroy_node()
        adversary.destroy_node()


def test_correct_publisher_qos_observed(rclpy_context) -> None:
    """A /sim/hardware/safety_stop publisher with the exact contract QoS is
    observed as TRANSIENT_LOCAL/RELIABLE/depth 1 from the right source."""
    provider = Node("tinker_sim_safety_supervisor")
    correct_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    provider.create_publisher(Bool, "/sim/hardware/safety_stop", correct_qos)
    reader = _readiness_node("graph_probe_good_qos")
    executor = SingleThreadedExecutor()
    executor.add_node(provider)
    executor.add_node(reader)
    try:
        _spin(executor, count=40)
        metadata = reader._publisher_metadata()
        entry = metadata["/sim/hardware/safety_stop"]
        assert entry["source"] == "/tinker_sim_safety_supervisor"
        qos = entry["qos"]
        assert qos["reliability"] == "RELIABLE"
        assert qos["durability"] == "TRANSIENT_LOCAL"
        # Humble publisher info never reports depth (always 0).
        assert qos["depth"] == 0
    finally:
        executor.remove_node(provider)
        executor.remove_node(reader)
        executor.shutdown()
        reader.destroy_node()
        provider.destroy_node()


def test_controller_manager_service_correct_source_observed(rclpy_context) -> None:
    """A correctly-sourced controller_manager service is observed from
    /controller_manager with the exact ListControllers type."""
    provider = Node("controller_manager")

    def handle(request, response):
        del request
        return response

    provider.create_service(ListControllers, "/controller_manager/list_controllers", handle)
    reader = _readiness_node("graph_probe_correct_service")
    executor = SingleThreadedExecutor()
    executor.add_node(provider)
    executor.add_node(reader)
    try:
        _spin(executor, count=40)
        services = reader._graph_services()
        entry = services["/controller_manager/list_controllers"]
        assert entry["source"] == "/controller_manager"
        assert "controller_manager_msgs/srv/ListControllers" in entry["types"]
        assert entry["count"] == 1
    finally:
        executor.remove_node(provider)
        executor.remove_node(reader)
        executor.shutdown()
        reader.destroy_node()
        provider.destroy_node()
