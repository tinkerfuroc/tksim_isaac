"""Task 7 (fix round 1): real Humble live-seam tests for the OMPL smoke.

Exercises actual generated ROS types and graph behavior under Humble CPython
3.10, closing the Task 7 I5 gap (the live Humble seam was previously untested):

- real ``OmplPlanSmokeClient._to_moveit_goal`` conversion for joint/pose
  (pipeline, plan_only, group, frame, link, constraints);
- readiness ``std_msgs/msg/String`` publisher metadata + canonical callback
  parsing (pass, malformed);
- ``/isaac_joint_commands`` publisher discovery/QoS and callback window;
- ``/move_action`` goal/result service type/source/cardinality probe,
  including a wrong-typed result service;
- result/status adjudication through a real ``MoveGroup`` action server
  (SUCCESS + nonempty trajectory for joint/pose; ABORTED + a real
  ``PLANNING_FAILED`` code for blocked), plus the bounded cancellation path.

The simulator (CPython 3.12) suite at ``tests/test_ompl_plan_smoke.py`` remains
ROS-free; these tests ``importorskip`` ROS and are skipped under 3.12.
"""
from __future__ import annotations

import json
import sys
import threading
import time
from argparse import Namespace
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="requires Humble ROS Python runtime")
pytest.importorskip("moveit_msgs", reason="requires Humble moveit_msgs")
pytest.importorskip("sensor_msgs", reason="requires Humble sensor_msgs")
pytest.importorskip("std_msgs", reason="requires Humble std_msgs")

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from ompl_goal_builders import (  # noqa: E402
    build_joint_goal,
    build_pose_goal,
)
from ompl_plan_smoke import (  # noqa: E402
    COMMAND_TOPIC,
    COMMAND_SOURCE,
    COMMAND_TYPE,
    MOVE_ACTION,
    MOVE_ACTION_SOURCE,
    MOVE_ACTION_TYPE,
    READINESS_SOURCE,
    READINESS_TOPIC,
    READINESS_TYPE,
    SUCCESS_ERROR_CODE,
    STATUS_ABORTED,
    STATUS_SUCCEEDED,
    OmplPlanSmokeClient,
    _action_name_from_type,
    _evaluate_commands,
    _evaluate_readiness,
    derive_goal_service_type,
    derive_result_service_type,
    parse_readiness_payload,
)

from rclpy.action import ActionClient  # noqa: E402
from rclpy.action.server import ActionServer  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from moveit_msgs.action import MoveGroup  # noqa: E402
from moveit_msgs.action._move_group import (  # noqa: E402
    MoveGroup_GetResult,
    MoveGroup_SendGoal,
)
from moveit_msgs.msg import MoveItErrorCodes  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

JOINT_NAMES = ["joint{}".format(i) for i in range(1, 8)]


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


def _spin_until(executor: SingleThreadedExecutor, predicate, timeout_s: float = 5.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def _args(timeout: float = 5.0) -> Namespace:
    return Namespace(
        mode="joint",
        timeout=timeout,
        readiness_timeout=5.0,
        readiness_max_age=1.0,
        post_result_tail=0.1,
        group_name="xarm7",
        planner_id="",
        allowed_planning_time=5.0,
        position_tolerance=0.02,
    )


def _make_client(node_name: str, args: Namespace | None = None) -> OmplPlanSmokeClient:
    node = Node(node_name)
    return OmplPlanSmokeClient(node, args or _args())


# ---------------------------------------------------------------------------
# 1. Real goal conversion
# ---------------------------------------------------------------------------


def test_to_moveit_goal_joint(rclpy_context) -> None:
    node = Node("to_moveit_goal_joint")
    client = OmplPlanSmokeClient(node, _args())
    try:
        goal = build_joint_goal()
        msg = client._to_moveit_goal(goal)
        assert isinstance(msg, MoveGroup.Goal)
        request = msg.request
        assert request.pipeline_id == "ompl"
        assert request.group_name == "xarm7"
        assert msg.planning_options.plan_only is True
        assert len(request.goal_constraints) == 1
        constraint = request.goal_constraints[0]
        names = [j.joint_name for j in constraint.joint_constraints]
        assert names == JOINT_NAMES
        positions = {j.joint_name: j.position for j in constraint.joint_constraints}
        assert positions["joint4"] == pytest.approx(0.2)
        assert positions["joint6"] == pytest.approx(0.3)
        assert all(
            j.tolerance_above == pytest.approx(0.02) for j in constraint.joint_constraints
        )
    finally:
        node.destroy_node()


def test_to_moveit_goal_pose(rclpy_context) -> None:
    node = Node("to_moveit_goal_pose")
    client = OmplPlanSmokeClient(node, _args())
    try:
        goal = build_pose_goal([0.45, 0.2, 0.99], [0.0, 0.0, 0.0, 1.0])
        msg = client._to_moveit_goal(goal)
        request = msg.request
        assert request.pipeline_id == "ompl"
        assert request.group_name == "xarm7"
        assert msg.planning_options.plan_only is True
        constraint = request.goal_constraints[0]
        assert len(constraint.position_constraints) == 1
        position_constraint = constraint.position_constraints[0]
        assert position_constraint.link_name == "link_tcp"
        assert position_constraint.header.frame_id == "base_link"
        region = position_constraint.constraint_region
        assert len(region.primitives) == 1
        assert len(region.primitive_poses) == 1
        pose = region.primitive_poses[0]
        assert pose.position.x == pytest.approx(0.45)
        assert pose.position.y == pytest.approx(0.2)
        assert pose.position.z == pytest.approx(0.99)
        assert pose.orientation.w == pytest.approx(1.0)
    finally:
        node.destroy_node()


# ---------------------------------------------------------------------------
# 2. Readiness publisher metadata + canonical callback parsing
# ---------------------------------------------------------------------------


def _canonical_pass_json(*, scenario_id: str = "qualification-moveit-plan-joint") -> str:
    payload = {
        "schema_version": 1,
        "state": "pass",
        "ready": True,
        "reasons": [],
        "published_at": time.monotonic(),
        "evidence": {
            "shared_report": {
                "ready": True,
                "identities": {
                    "scenario_id": scenario_id,
                    "seed": 7,
                    "scenario_declaration_sha256": "a" * 64,
                    "planning_scene_sha256": "b" * 64,
                    "integrated_sha256": "c" * 64,
                    "model_fingerprint": "fp",
                    "provider_manifest_sha256": "d" * 64,
                },
            },
            "fixture_status": {
                "ready": True,
                "status": {
                    "scenario": scenario_id,
                    "revision": "2026-08-01-moveit-qualification-joint",
                    "revision_digest": "e" * 64,
                    "sequence": 1,
                    "published_at": time.monotonic(),
                    "owned_ids": ["sim_fixture/pedestal", "sim_fixture/public_target"],
                    "target_source_id": "sim_fixture/public_target",
                    "target_handoff": "pick_and_place/object_mesh",
                },
            },
        },
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def _canonical_fail_json() -> str:
    payload = {
        "schema_version": 1,
        "state": "fail",
        "ready": False,
        "reasons": ["boom"],
        "published_at": time.monotonic(),
        "evidence": {"shared_report": {"ready": False, "identities": {}}, "fixture_status": {"status": None}},
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def _matching_expected() -> dict[str, object]:
    """Expected-identity dict matching _canonical_pass_json's dummy digests."""
    return {
        "scenario_id": "qualification-moveit-plan-joint",
        "seed": 7,
        "scenario_declaration_sha256": "a" * 64,
        "planning_scene_sha256": "b" * 64,
        "planning_scene_revision": "2026-08-01-moveit-qualification-joint",
        "planning_scene_revision_digest": "e" * 64,
        "planning_scene_owned_ids": ["sim_fixture/pedestal", "sim_fixture/public_target"],
        "planning_scene_target_source_id": "sim_fixture/public_target",
        "planning_scene_target_handoff": "pick_and_place/object_mesh",
    }


def test_readiness_publisher_metadata_observed(rclpy_context) -> None:
    provider = Node("integrated_readiness")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    provider.create_publisher(String, READINESS_TOPIC, qos)
    node = Node("readiness_pub_probe")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(provider)
    executor.add_node(node)
    try:
        _spin(executor, count=40)
        obs = client._probe_readiness_publisher(5.0)
        assert obs["count"] == 1
        assert obs["source"] == READINESS_SOURCE
        assert obs["type"] == READINESS_TYPE
        assert obs["qos"]["reliability"] == "RELIABLE"
        assert obs["qos"]["durability"] == "TRANSIENT_LOCAL"
        assert obs["settled"] is True
    finally:
        executor.remove_node(provider)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        provider.destroy_node()


def test_readiness_callback_parses_canonical_pass(rclpy_context) -> None:
    publisher = Node("integrated_readiness")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    pub = publisher.create_publisher(String, READINESS_TOPIC, qos)
    node = Node("readiness_callback_pass")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(node)
    try:
        message = String()
        message.data = _canonical_pass_json()
        for _ in range(5):
            pub.publish(message)
            executor.spin_once(timeout_sec=0.1)
        assert _spin_until(executor, lambda: client._readiness_valid)
        assert client._readiness_obs["state"] == "pass"
        assert client._readiness_obs["schema_version"] == 1
        parsed = parse_readiness_payload(message.data)
        assert parsed["valid"] is True
    finally:
        executor.remove_node(publisher)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        publisher.destroy_node()


def test_readiness_callback_rejects_malformed(rclpy_context) -> None:
    publisher = Node("integrated_readiness")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    pub = publisher.create_publisher(String, READINESS_TOPIC, qos)
    node = Node("readiness_callback_malformed")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(node)
    try:
        message = String()
        message.data = "not json {"
        for _ in range(5):
            pub.publish(message)
            executor.spin_once(timeout_sec=0.1)
        _spin(executor, count=10)
        assert client._readiness_valid is False
        assert client._readiness_counts["malformed"] >= 1
    finally:
        executor.remove_node(publisher)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        publisher.destroy_node()


# ---------------------------------------------------------------------------
# 3. /isaac_joint_commands publisher discovery/QoS + callback window
# ---------------------------------------------------------------------------


def test_command_publisher_discovery_and_qos(rclpy_context) -> None:
    gateway = Node("tinker_sim_command_gateway")
    # Match the verified gateway truth (depth=50); Humble still reports 0.
    qos = QoSProfile(
        depth=50,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    gateway.create_publisher(JointState, COMMAND_TOPIC, qos)
    node = Node("command_pub_probe")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(gateway)
    executor.add_node(node)
    try:
        _spin(executor, count=40)
        obs = client._probe_command_publisher(5.0)
        assert obs["count"] == 1
        assert obs["source"] == COMMAND_SOURCE
        assert obs["type"] == "sensor_msgs/msg/JointState"
        assert obs["qos"]["reliability"] == "RELIABLE"
        assert obs["qos"]["durability"] == "VOLATILE"
        assert obs["expected_depth"] == 50
        assert obs["settled"] is True
    finally:
        executor.remove_node(gateway)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        gateway.destroy_node()


def test_command_callback_counts_only_in_window(rclpy_context) -> None:
    gateway = Node("tinker_sim_command_gateway")
    pub = gateway.create_publisher(JointState, COMMAND_TOPIC, 10)
    node = Node("command_callback_probe")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(gateway)
    executor.add_node(node)
    try:
        message = JointState()
        message.name = JOINT_NAMES
        message.position = [0.0] * 7
        # Outside the window: sample must be ignored.
        for _ in range(3):
            pub.publish(message)
            executor.spin_once(timeout_sec=0.05)
        _spin(executor, count=5)
        assert client._command_samples == 0
        # Inside the window: sample must be counted.
        client._window_open = True
        pub.publish(message)
        assert _spin_until(executor, lambda: client._command_samples == 1)
        client._window_open = False
        assert client._command_samples == 1
        # Back outside: ignored again.
        pub.publish(message)
        _spin(executor, count=5)
        assert client._command_samples == 1
    finally:
        executor.remove_node(gateway)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        gateway.destroy_node()


# ---------------------------------------------------------------------------
# 4. /move_action goal/result service probe (incl. wrong result type)
# ---------------------------------------------------------------------------


def test_probe_move_action_correct_services(rclpy_context) -> None:
    provider = Node("move_group")
    _service_handler = lambda request, response: response  # noqa: E731
    provider.create_service(
        MoveGroup_SendGoal,
        "{}/_action/send_goal".format(MOVE_ACTION),
        _service_handler,
    )
    provider.create_service(
        MoveGroup_GetResult,
        "{}/_action/get_result".format(MOVE_ACTION),
        _service_handler,
    )
    node = Node("move_action_probe")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(provider)
    executor.add_node(node)
    try:
        _spin(executor, count=40)
        probe = client._probe_move_action()
        assert probe["count"] == 1
        assert probe["source"] == MOVE_ACTION_SOURCE
        assert probe["observed_types"] == [derive_goal_service_type(MOVE_ACTION_TYPE)]
        assert probe["kind"] == "MoveGroup"
        assert probe["result_service_types"] == [derive_result_service_type(MOVE_ACTION_TYPE)]
        assert probe["result_service_present"] is True
    finally:
        executor.remove_node(provider)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        provider.destroy_node()


def test_probe_move_action_rejects_wrong_result_type(rclpy_context) -> None:
    provider = Node("move_group")
    _service_handler = lambda request, response: response  # noqa: E731
    provider.create_service(
        MoveGroup_SendGoal,
        "{}/_action/send_goal".format(MOVE_ACTION),
        _service_handler,
    )
    # Wrong-typed get_result (Trigger instead of MoveGroup_GetResult).
    provider.create_service(
        Trigger,
        "{}/_action/get_result".format(MOVE_ACTION),
        _service_handler,
    )
    node = Node("move_action_wrong_result")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(provider)
    executor.add_node(node)
    try:
        _spin(executor, count=40)
        probe = client._probe_move_action()
        assert probe["result_service_types"] == ["std_srvs/srv/Trigger"]
        assert probe["result_service_types"] != [derive_result_service_type(MOVE_ACTION_TYPE)]
        # The evaluator must reject the wrong result-service type.
        from ompl_plan_smoke import _evaluate_endpoint

        reasons = _evaluate_endpoint(
            probe, {"action_type": MOVE_ACTION_TYPE, "source": MOVE_ACTION_SOURCE}
        )
        assert any("result-service" in reason for reason in reasons)
    finally:
        executor.remove_node(provider)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        provider.destroy_node()


def test_action_name_derivation_from_real_types() -> None:
    assert _action_name_from_type(derive_goal_service_type(MOVE_ACTION_TYPE)) == "MoveGroup"
    assert _action_name_from_type(derive_result_service_type(MOVE_ACTION_TYPE)) == "MoveGroup"


# ---------------------------------------------------------------------------
# 5. Result/status adjudication through a real MoveGroup action server
# ---------------------------------------------------------------------------


def _joint_success_result() -> MoveGroup.Result:
    result = MoveGroup.Result()
    result.error_code.val = MoveItErrorCodes.SUCCESS
    trajectory = result.planned_trajectory
    trajectory.joint_trajectory.joint_names = list(JOINT_NAMES)
    point = JointTrajectoryPoint()
    point.positions = [0.0] * 7
    point.velocities = [0.0] * 7
    point.time_from_start.sec = 1
    trajectory.joint_trajectory.points.append(point)
    return result


def _blocked_failure_result() -> MoveGroup.Result:
    result = MoveGroup.Result()
    result.error_code.val = MoveItErrorCodes.PLANNING_FAILED
    return result


def _spin_server(
    server_node: Node,
) -> tuple[SingleThreadedExecutor, threading.Thread]:
    executor = SingleThreadedExecutor()
    executor.add_node(server_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread


def _stop_server(executor: SingleThreadedExecutor, thread: threading.Thread) -> None:
    executor.shutdown()
    thread.join(timeout=2.0)


def test_run_action_joint_success(rclpy_context) -> None:
    server_node = Node("move_group")

    def execute(goal_handle):
        goal_handle.succeed()
        return _joint_success_result()

    server = ActionServer(server_node, MoveGroup, MOVE_ACTION, execute)
    client_node = Node("run_action_joint_client")
    client = OmplPlanSmokeClient(client_node, _args(timeout=5.0))
    server_executor, server_thread = _spin_server(server_node)
    try:
        time.sleep(0.5)  # let the server services be discovered
        action_client = ActionClient(client_node, MoveGroup, MOVE_ACTION)
        outcome = client._run_action(
            action_client, build_joint_goal(), time.monotonic() + 5.0
        )
        assert outcome["kind"] == "success"
        assert outcome["goal_accepted"] is True
        assert outcome["terminal_status"] == STATUS_SUCCEEDED
        assert outcome["error_code"] == SUCCESS_ERROR_CODE
        assert outcome["result_received"] is True
        assert outcome["trajectory_point_count"] == 1
        assert outcome["trajectory_joint_names"] == JOINT_NAMES
        assert outcome["planning_time"] == pytest.approx(0.0)
        action_client.destroy()
    finally:
        server.destroy()
        _stop_server(server_executor, server_thread)
        client_node.destroy_node()
        server_node.destroy_node()


def test_run_action_blocked_aborted_planning_failed(rclpy_context) -> None:
    server_node = Node("move_group")

    def execute(goal_handle):
        goal_handle.abort()
        return _blocked_failure_result()

    server = ActionServer(server_node, MoveGroup, MOVE_ACTION, execute)
    client_node = Node("run_action_blocked_client")
    client = OmplPlanSmokeClient(client_node, _args(timeout=5.0))
    server_executor, server_thread = _spin_server(server_node)
    try:
        time.sleep(0.5)
        action_client = ActionClient(client_node, MoveGroup, MOVE_ACTION)
        goal = build_pose_goal([0.35, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0])
        outcome = client._run_action(action_client, goal, time.monotonic() + 5.0)
        assert outcome["kind"] == "non_success"
        assert outcome["goal_accepted"] is True
        assert outcome["terminal_status"] == STATUS_ABORTED
        assert outcome["terminal_status_name"] == "STATUS_ABORTED"
        assert outcome["error_code"] == MoveItErrorCodes.PLANNING_FAILED
        assert outcome["result_received"] is True
        action_client.destroy()
    finally:
        server.destroy()
        _stop_server(server_executor, server_thread)
        client_node.destroy_node()
        server_node.destroy_node()


# ---------------------------------------------------------------------------
# 6. Bounded cancellation path (fail-closed)
# ---------------------------------------------------------------------------


def test_cancel_path_is_bounded_and_fail_closed(rclpy_context) -> None:
    """A slow server that never answers forces the client's result deadline; the
    client requests cancellation and returns a fail-closed timeout outcome."""
    release = threading.Event()

    def execute(goal_handle):
        release.wait(timeout=5.0)
        goal_handle.abort()
        return _blocked_failure_result()

    server_node = Node("move_group")
    server = ActionServer(server_node, MoveGroup, MOVE_ACTION, execute)
    client_node = Node("cancel_path_client")
    client = OmplPlanSmokeClient(client_node, _args(timeout=1.5))
    server_executor, server_thread = _spin_server(server_node)
    try:
        time.sleep(0.5)
        action_client = ActionClient(client_node, MoveGroup, MOVE_ACTION)
        outcome = client._run_action(
            action_client, build_joint_goal(), time.monotonic() + 1.0
        )
        assert outcome["kind"] == "timeout"
        assert outcome["goal_accepted"] is True
        assert outcome["cancel_requested"] is True
        assert outcome["terminal_status"] is None
        assert outcome["result_received"] is False
        # The whole path is bounded: the run must return well under the release
        # wait (the server never confirms), proving fail-closed boundedness.
        action_client.destroy()
    finally:
        release.set()
        server.destroy()
        _stop_server(server_executor, server_thread)
        client_node.destroy_node()
        server_node.destroy_node()


# ---------------------------------------------------------------------------
# 7. Adverse publisher/window paths (fix round 2)
# ---------------------------------------------------------------------------


def test_command_wrong_source_publisher_rejected(rclpy_context) -> None:
    """A command publisher from the wrong source must fail the real metadata
    checker/evaluator."""
    rogue = Node("rogue_gateway")
    rogue.create_publisher(JointState, COMMAND_TOPIC, 50)
    node = Node("cmd_wrong_source_probe")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(rogue)
    executor.add_node(node)
    try:
        _spin(executor, count=40)
        pub = client._read_publisher_metadata(COMMAND_TOPIC)
        assert pub["source"] == "/rogue_gateway"
        assert pub["type"] == COMMAND_TYPE
        obs = {
            "publisher": pub,
            "settled": True,
            "samples": 0,
            "window_start_s": 1.0,
            "window_end_s": 2.0,
            "publisher_graph_changed": False,
        }
        reasons = _evaluate_commands(obs)
        assert any("publisher source" in reason for reason in reasons)
    finally:
        executor.remove_node(rogue)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rogue.destroy_node()


def test_command_wrong_qos_publisher_rejected(rclpy_context) -> None:
    """A correct-source command publisher with BEST_EFFORT QoS must be
    rejected by the real metadata checker."""
    gateway = Node("tinker_sim_command_gateway")
    qos = QoSProfile(
        depth=50,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    gateway.create_publisher(JointState, COMMAND_TOPIC, qos)
    node = Node("cmd_wrong_qos_probe")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(gateway)
    executor.add_node(node)
    try:
        _spin(executor, count=40)
        pub = client._read_publisher_metadata(COMMAND_TOPIC)
        assert pub["source"] == COMMAND_SOURCE
        assert pub["qos"]["reliability"] == "BEST_EFFORT"
        obs = {
            "publisher": pub,
            "settled": True,
            "samples": 0,
            "window_start_s": 1.0,
            "window_end_s": 2.0,
            "publisher_graph_changed": False,
        }
        reasons = _evaluate_commands(obs)
        assert any("reliability" in reason for reason in reasons)
    finally:
        executor.remove_node(gateway)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        gateway.destroy_node()


def test_command_duplicate_publisher_rejected(rclpy_context) -> None:
    """Two publishers on /isaac_joint_commands must be rejected (cardinality)."""
    gateway = Node("tinker_sim_command_gateway")
    rogue = Node("rogue_gateway")
    gateway.create_publisher(JointState, COMMAND_TOPIC, 50)
    rogue.create_publisher(JointState, COMMAND_TOPIC, 50)
    node = Node("cmd_dup_probe")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(gateway)
    executor.add_node(rogue)
    executor.add_node(node)
    try:
        _spin(executor, count=50)
        pub = client._read_publisher_metadata(COMMAND_TOPIC)
        assert pub["count"] == 2
        obs = {
            "publisher": pub,
            "settled": True,
            "samples": 0,
            "window_start_s": 1.0,
            "window_end_s": 2.0,
            "publisher_graph_changed": False,
        }
        reasons = _evaluate_commands(obs)
        assert any("publisher count" in reason for reason in reasons)
    finally:
        executor.remove_node(gateway)
        executor.remove_node(rogue)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rogue.destroy_node()
        gateway.destroy_node()


def test_readiness_wrong_source_publisher_fails_settle(rclpy_context) -> None:
    """A readiness publisher from the wrong source never settles the live
    probe (source mismatch), so the gate refuses to open."""
    rogue = Node("rogue_readiness")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    rogue.create_publisher(String, READINESS_TOPIC, qos)
    node = Node("readiness_wrong_source_probe")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(rogue)
    executor.add_node(node)
    try:
        _spin(executor, count=40)
        obs = client._probe_readiness_publisher(2.0)
        assert obs["settled"] is False
    finally:
        executor.remove_node(rogue)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rogue.destroy_node()


def test_readiness_fail_mid_window_rejects(rclpy_context) -> None:
    """A canonical readiness 'fail' published mid-window must reject the
    assembled observation even if a later correct pass arrives."""
    publisher = Node("integrated_readiness")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    pub = publisher.create_publisher(String, READINESS_TOPIC, qos)
    node = Node("readiness_fail_window_client")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(node)
    try:
        client._readiness_expected = _matching_expected()
        pass_msg = String()
        pass_msg.data = _canonical_pass_json()
        for _ in range(5):
            pub.publish(pass_msg)
            executor.spin_once(timeout_sec=0.1)
        assert _spin_until(executor, lambda: client._readiness_valid)
        client._readiness_window_open = True
        client._readiness_window["start_s"] = time.monotonic()
        client._readiness_window_counts = {
            "pass": 0,
            "fail": 0,
            "malformed": 0,
            "identity_invalid": 0,
        }
        fail_msg = String()
        fail_msg.data = _canonical_fail_json()
        pub.publish(fail_msg)
        _spin(executor, count=5)
        # A later correct pass arrives but the window already observed fail.
        pub.publish(pass_msg)
        _spin(executor, count=5)
        client._readiness_window_open = False
        client._readiness_window["end_s"] = time.monotonic()
        obs = client._assemble_readiness_obs(_matching_expected(), True)
        assert obs["window_counts"]["fail"] >= 1
        reasons = _evaluate_readiness(obs, _matching_expected())
        assert any("fail" in reason for reason in reasons)
    finally:
        executor.remove_node(publisher)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        publisher.destroy_node()


def test_readiness_wrong_identity_mid_window_rejects(rclpy_context) -> None:
    """A wrong-identity pass published mid-window must be tracked and reject
    the assembled observation even if a correct pass follows."""
    publisher = Node("integrated_readiness")
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    pub = publisher.create_publisher(String, READINESS_TOPIC, qos)
    node = Node("readiness_identity_window_client")
    client = OmplPlanSmokeClient(node, _args())
    executor = SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(node)
    try:
        client._readiness_expected = _matching_expected()
        pass_msg = String()
        pass_msg.data = _canonical_pass_json()
        for _ in range(5):
            pub.publish(pass_msg)
            executor.spin_once(timeout_sec=0.1)
        assert _spin_until(executor, lambda: client._readiness_valid)
        client._readiness_window_open = True
        client._readiness_window["start_s"] = time.monotonic()
        client._readiness_window_counts = {
            "pass": 0,
            "fail": 0,
            "malformed": 0,
            "identity_invalid": 0,
        }
        wrong_msg = String()
        wrong_msg.data = _canonical_pass_json(
            scenario_id="qualification-moveit-plan-pose"
        )
        pub.publish(wrong_msg)
        _spin(executor, count=5)
        # A later correct pass arrives but the window already observed a
        # wrong-identity pass.
        pub.publish(pass_msg)
        _spin(executor, count=5)
        client._readiness_window_open = False
        client._readiness_window["end_s"] = time.monotonic()
        obs = client._assemble_readiness_obs(_matching_expected(), True)
        assert obs["window_counts"]["identity_invalid"] >= 1
        assert obs["any_identity_invalid_in_window"] is True
        reasons = _evaluate_readiness(obs, _matching_expected())
        assert any("identity" in reason for reason in reasons)
    finally:
        executor.remove_node(publisher)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        publisher.destroy_node()


def test_command_sample_in_window_rejects(rclpy_context) -> None:
    """A command published during the request/result/tail window is counted
    and the assembled observation rejects."""
    server_node = Node("move_group")

    def execute(goal_handle):
        goal_handle.succeed()
        return _joint_success_result()

    server = ActionServer(server_node, MoveGroup, MOVE_ACTION, execute)
    gateway = Node("tinker_sim_command_gateway")
    gw_pub = gateway.create_publisher(JointState, COMMAND_TOPIC, 50)
    client_node = Node("cmd_sample_window_client")
    client = OmplPlanSmokeClient(client_node, _args(timeout=5.0))
    server_executor, server_thread = _spin_server(server_node)
    gw_executor = SingleThreadedExecutor()
    gw_executor.add_node(gateway)
    gw_thread = threading.Thread(target=gw_executor.spin, daemon=True)
    gw_thread.start()
    stop_publish = threading.Event()

    def publish_commands() -> None:
        message = JointState()
        message.name = list(JOINT_NAMES)
        message.position = [0.0] * 7
        while not stop_publish.is_set():
            gw_pub.publish(message)
            time.sleep(0.01)

    gw_pub_thread = threading.Thread(target=publish_commands, daemon=True)
    gw_pub_thread.start()
    try:
        time.sleep(0.5)
        client._command_publisher = client._read_publisher_metadata(COMMAND_TOPIC)
        action_client = ActionClient(client_node, MoveGroup, MOVE_ACTION)
        outcome = client._execute_goal(build_joint_goal())
        assert outcome["kind"] == "success"
        assert client._command_samples > 0
        obs = client._assemble_command_obs()
        assert obs["samples"] > 0
        reasons = _evaluate_commands(obs)
        assert any("command sample" in reason for reason in reasons)
        action_client.destroy()
    finally:
        stop_publish.set()
        gw_pub_thread.join(timeout=2.0)
        server.destroy()
        _stop_server(server_executor, server_thread)
        gw_executor.shutdown()
        gw_thread.join(timeout=2.0)
        client_node.destroy_node()
        server_node.destroy_node()
        gateway.destroy_node()


def test_command_publisher_graph_change_during_window_rejects(rclpy_context) -> None:
    """A command publisher graph change during the window (a second gateway
    appearing mid-planning) must be detected and the assembled observation
    rejects."""
    second_node = {"node": None}

    def execute(goal_handle):
        rogue = Node("rogue_gateway_second")
        second_node["node"] = rogue
        rogue.create_publisher(JointState, COMMAND_TOPIC, 50)
        time.sleep(0.3)  # let it appear on the graph during the window
        goal_handle.succeed()
        return _joint_success_result()

    server_node = Node("move_group")
    server = ActionServer(server_node, MoveGroup, MOVE_ACTION, execute)
    gateway = Node("tinker_sim_command_gateway")
    gateway.create_publisher(JointState, COMMAND_TOPIC, 50)
    client_node = Node("cmd_graph_change_client")
    client = OmplPlanSmokeClient(client_node, _args(timeout=5.0))
    server_executor, server_thread = _spin_server(server_node)
    try:
        time.sleep(0.5)
        client._command_publisher = client._read_publisher_metadata(COMMAND_TOPIC)
        assert client._command_publisher["count"] == 1
        action_client = ActionClient(client_node, MoveGroup, MOVE_ACTION)
        outcome = client._execute_goal(build_joint_goal())
        assert outcome["kind"] == "success"
        # Wait for the second publisher to be discovered by spinning the client.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(client_node, timeout_sec=0.05)
            if client._read_publisher_metadata(COMMAND_TOPIC)["count"] == 2:
                break
        tail = client._read_publisher_metadata(COMMAND_TOPIC)
        changed = not OmplPlanSmokeClient._publisher_same(
            client._command_publisher, tail
        )
        assert changed is True
        obs = client._assemble_command_obs()
        obs["publisher_graph_changed"] = changed
        reasons = _evaluate_commands(obs)
        assert any("publisher graph changed" in reason for reason in reasons)
        action_client.destroy()
    finally:
        if second_node["node"] is not None:
            second_node["node"].destroy_node()
        server.destroy()
        _stop_server(server_executor, server_thread)
        client_node.destroy_node()
        server_node.destroy_node()
        gateway.destroy_node()
