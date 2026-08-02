"""Task 4 (Humble): real generated-message / geometry executor tests.

Runs under sourced ROS Humble Python 3.10 with the simulator paths prepended
onto the sourced ``PYTHONPATH``:

.. code-block:: bash

    source /opt/ros/humble/setup.zsh
    source /home/tinker/tk25_ws/install/setup.zsh
    export PYTHONPATH=/home/tinker/tinker-sim/6.0.1:/home/tinker/tinker-sim/6.0.1/validation:/home/tinker/tinker-sim/6.0.1/tests:/home/tinker/tinker-sim/6.0.1/simulation:/home/tinker/tinker-sim/6.0.1/ros2_ws/src/tinker_sim_bridge${PYTHONPATH:+:$PYTHONPATH}
    python3 -m pytest -q /home/tinker/tinker-sim/6.0.1/tests/test_integrated_gate_executor_ros.py

The suite FAILS (never skips) when the Humble ROS runtime is unavailable, so the
documented sourced-Humble command is the verified run.  Covers the real
``MoveGroup`` joint/pose goal fields, Pick/Place goal field construction and
their fail-closed validation, the deterministic ``PointCloud2`` cube geometry,
real-shape multi-operation report bytes, and the live ``IntegratedGateExecutor``
(private-context node construction, typed ``PlanningScene`` subscriptions,
scene normalization, stubbed plan-only flow, readiness gating, and bounded
cancellation).
"""
from __future__ import annotations

import hashlib
import json
import math
import re
import sys
import time
from pathlib import Path

import pytest


def _require_ros_runtime() -> None:
    """Fail (not skip) when the Humble ROS runtime is unavailable.

    The exact sourced-Humble command prepends the simulator paths onto the
    sourced ``$PYTHONPATH``, so ``rclpy`` and the generated message packages are
    importable.  A missing runtime is an environment error for this suite, not a
    skip.
    """
    missing: list[str] = []
    for name in (
        "rclpy",
        "moveit_msgs",
        "sensor_msgs",
        "sensor_msgs_py",
        "geometry_msgs",
        "tinker_arm_msgs",
    ):
        try:
            __import__(name)
        except ImportError as exc:  # pragma: no cover - depends on the runtime
            missing.append(f"{name}: {exc}")
    if missing:
        pytest.fail(
            "Task-4 Humble suite requires the ROS Humble Python runtime; run under "
            "sourced /opt/ros/humble with the simulator paths prepended onto "
            "PYTHONPATH (see module docstring). Missing: " + "; ".join(missing)
        )


_require_ros_runtime()

import rclpy  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "tests"))

from tinker_sim_bridge.fixture_planning_scene import fixture_owned_ids  # noqa: E402
from tinker_sim_bridge.integrated_readiness import (  # noqa: E402
    build_canonical_report,
    public_integrated_mapping,
)
from validation.integrated_gate_executor import (  # noqa: E402
    IntegratedGateExecutor,
    build_joint_move_group_goal,
    build_pick_goal,
    build_place_goal,
    build_pose_move_group_goal,
    deterministic_cube_cloud,
    evaluate_executor_readiness,
    validate_physics_ready_snapshot,
)

Q_OUTBOUND = (0.20, -0.20, 0.15, 0.30, -0.15, 0.20, 0.15)


def test_joint_move_group_goal_is_ompl_plan_only():
    goal = build_joint_move_group_goal(Q_OUTBOUND, plan_only=True)
    assert goal.request.group_name == "xarm7"
    assert goal.request.pipeline_id == "ompl"
    assert goal.request.num_planning_attempts == 3
    assert goal.request.allowed_planning_time == 3.0
    assert goal.planning_options.plan_only is True
    assert goal.planning_options.replan is False
    assert len(goal.request.goal_constraints) == 1
    joints = goal.request.goal_constraints[0].joint_constraints
    assert [joint.joint_name for joint in joints] == [f"joint{i}" for i in range(1, 8)]
    assert [joint.position for joint in joints] == list(Q_OUTBOUND)


def test_joint_move_group_goal_rejects_wrong_arity():
    from validation.integrated_gate_executor import build_joint_move_group_goal

    with pytest.raises(ValueError, match="7 finite"):
        build_joint_move_group_goal(Q_OUTBOUND[:6], plan_only=True)
    with pytest.raises(ValueError, match="7 finite"):
        build_joint_move_group_goal([float("nan")] * 7, plan_only=True)


def test_pose_move_group_goal_fields():
    from geometry_msgs.msg import PoseStamped

    target = PoseStamped()
    target.header.frame_id = "base_link"
    target.pose.position.x = 0.65
    target.pose.position.y = 0.0
    target.pose.position.z = 0.72
    target.pose.orientation.w = 1.0
    goal = build_pose_move_group_goal(target, plan_only=True)
    assert goal.request.group_name == "xarm7"
    assert goal.request.pipeline_id == "ompl"
    assert goal.planning_options.plan_only is True
    assert goal.planning_options.replan is False
    constraint = goal.request.goal_constraints[0]
    assert constraint.position_constraints[0].link_name == "link_tcp"
    assert constraint.orientation_constraints[0].link_name == "link_tcp"


def test_pose_builder_rejects_zero_quaternion():
    from geometry_msgs.msg import PoseStamped

    target = PoseStamped()
    target.header.frame_id = "base_link"
    target.pose.orientation.x = 0.0
    target.pose.orientation.y = 0.0
    target.pose.orientation.z = 0.0
    target.pose.orientation.w = 0.0
    with pytest.raises(ValueError, match="quaternion"):
        build_pose_move_group_goal(target, plan_only=True)


def test_pose_builder_rejects_wrong_frame():
    from geometry_msgs.msg import PoseStamped

    target = PoseStamped()
    target.header.frame_id = "world"
    target.pose.orientation.w = 1.0
    with pytest.raises(ValueError, match="base_link"):
        build_pose_move_group_goal(target, plan_only=True)


def test_pick_builder_rejects_six_back_positions():
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.orientation.w = 1.0
    cloud = deterministic_cube_cloud()
    with pytest.raises(ValueError, match="7 finite"):
        build_pick_goal(
            target_pose=pose,
            candidate_poses=[pose],
            env_points=cloud,
            object_points=cloud,
            back_positions=Q_OUTBOUND[:6],
            use_mesh=True,
            stay=False,
        )


def test_pick_builder_rejects_candidate_not_starting_with_target():
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.orientation.w = 1.0
    other = Pose()
    other.orientation.w = 1.0
    other.position.x = 0.1
    cloud = deterministic_cube_cloud()
    with pytest.raises(ValueError, match="candidate_poses"):
        build_pick_goal(
            target_pose=pose,
            candidate_poses=[other],
            env_points=cloud,
            object_points=cloud,
            back_positions=Q_OUTBOUND,
            use_mesh=True,
            stay=False,
        )


def test_pick_builder_constructs_real_goal_fields():
    from geometry_msgs.msg import Pose
    from tinker_arm_msgs.action import Pick

    pose = Pose()
    pose.position.x = 0.65
    pose.position.z = 0.72
    pose.orientation.w = 1.0
    cloud = deterministic_cube_cloud()
    goal = build_pick_goal(
        target_pose=pose,
        candidate_poses=[pose],
        env_points=cloud,
        object_points=cloud,
        back_positions=Q_OUTBOUND,
        use_mesh=True,
        stay=False,
        two_stage_plan=True,
    )
    assert isinstance(goal, Pick.Goal)
    assert goal.target_pose == pose
    assert goal.candidate_poses == [pose]
    assert goal.env_points == cloud
    assert goal.object_points == cloud
    assert list(goal.back_positions) == pytest.approx(list(Q_OUTBOUND))
    assert goal.two_stage_plan is True
    assert goal.use_mesh is True
    assert goal.stay is False


def test_place_builder_rejects_wrong_frame_and_back_positions():
    from geometry_msgs.msg import PointStamped, Pose

    target_point = PointStamped()
    target_point.header.frame_id = "world"
    orientation = Pose()
    orientation.orientation.w = 1.0
    cloud = deterministic_cube_cloud()
    with pytest.raises(ValueError, match="base_link"):
        build_place_goal(
            target_point=target_point,
            orientation=orientation,
            env_points=cloud,
            back_positions=Q_OUTBOUND,
        )
    target_point.header.frame_id = "base_link"
    with pytest.raises(ValueError, match="7 finite"):
        build_place_goal(
            target_point=target_point,
            orientation=orientation,
            env_points=cloud,
            back_positions=Q_OUTBOUND[:6],
        )


def test_place_builder_constructs_real_goal_fields():
    from geometry_msgs.msg import PointStamped, Pose
    from tinker_arm_msgs.action import Place

    target_point = PointStamped()
    target_point.header.frame_id = "base_link"
    target_point.point.x = 0.85
    target_point.point.z = 0.72
    orientation = Pose()
    orientation.orientation.w = 1.0
    cloud = deterministic_cube_cloud()
    goal = build_place_goal(
        target_point=target_point,
        orientation=orientation,
        env_points=cloud,
        back_positions=Q_OUTBOUND,
    )
    assert isinstance(goal, Place.Goal)
    assert goal.target_point == target_point
    assert goal.orientation == orientation
    assert goal.env_points == cloud
    assert list(goal.back_positions) == pytest.approx(list(Q_OUTBOUND))


def test_object_cloud_has_125_finite_points():
    from sensor_msgs_py import point_cloud2

    cloud = deterministic_cube_cloud(frame_id="base_link")
    points = list(point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=False))
    assert (cloud.height, cloud.width) == (1, 125)
    assert [(field.name, field.offset, field.datatype, field.count) for field in cloud.fields] == [
        ("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1)
    ]
    assert (cloud.is_bigendian, cloud.point_step, cloud.row_step, cloud.is_dense) == (
        False, 12, 1500, True
    )
    assert len(points) == 125
    assert all(math.isfinite(float(value)) for point in points for value in point)


def test_real_shape_report_bytes_pass_readiness_under_humble():
    from test_integrated_gate_executor import (
        _config,
        POSITIVE_REPORT_CONTRACT,
        readiness_scenario,
        ready_executor_snapshot,
    )

    contract = POSITIVE_REPORT_CONTRACT
    report = build_canonical_report(
        scenario_id=contract["scenario_mapping"]["id"],
        seed=contract["scenario_mapping"]["seed"],
        declaration=contract["scenario_mapping"]["declaration"],
        planning_scene=contract["planning_scene_declaration"],
        integrated=public_integrated_mapping(),
        operations=[
            {"operation": "reset_spawned", "accepted": True},
            {
                "operation": "spawn_entity",
                "accepted": True,
                "logical_id": "sim_fixture/pedestal",
                "prim_path": "/World/pedestal",
            },
            {
                "operation": "set_simulation_state",
                "accepted": True,
                "state": 1,
                "boundary": "PHYSICS_READY",
            },
        ],
        model_fingerprint=contract["identities"]["model_fingerprint"],
        provider_manifest_sha256=contract["identities"]["provider_manifest_sha256"],
    )
    report_bytes = json.dumps(
        report, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    snapshot = ready_executor_snapshot()
    snapshot["scenario_report_bytes"] = report_bytes
    snapshot["scenario"]["scenario_report_sha256"] = hashlib.sha256(report_bytes).hexdigest()
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is True
    assert result["reasons"] == []


def test_single_operation_fabricated_report_is_rejected():
    from test_integrated_gate_executor import (
        POSITIVE_REPORT_CONTRACT,
        readiness_scenario,
    )

    contract = POSITIVE_REPORT_CONTRACT
    # A fabricated single-operation report (no accepted reset/spawn
    # standard-operation record before PHYSICS_READY) is exactly the shape the
    # corrections forbid comparing against; the validator rejects it.
    report = build_canonical_report(
        scenario_id=contract["scenario_mapping"]["id"],
        seed=contract["scenario_mapping"]["seed"],
        declaration=contract["scenario_mapping"]["declaration"],
        planning_scene=contract["planning_scene_declaration"],
        integrated=public_integrated_mapping(),
        operations=[
            {
                "operation": "set_simulation_state",
                "accepted": True,
                "state": 1,
                "boundary": "PHYSICS_READY",
            }
        ],
        model_fingerprint=contract["identities"]["model_fingerprint"],
        provider_manifest_sha256=contract["identities"]["provider_manifest_sha256"],
    )
    report_bytes = json.dumps(
        report, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    snapshot = {
        "scenario": {"scenario_report_sha256": hashlib.sha256(report_bytes).hexdigest()},
        "scenario_report_bytes": report_bytes,
        "model": {"fingerprint": contract["identities"]["model_fingerprint"]},
        "provider_manifest_sha256": contract["identities"]["provider_manifest_sha256"],
    }
    with pytest.raises(ValueError, match="standard-operation records"):
        validate_physics_ready_snapshot(snapshot, readiness_scenario(contract))


# ---------------------------------------------------------------------------
# Fix round 1 additions: live executor construction, typed subscriptions,
# scene normalization, stubbed plan-only flow, readiness gating, cancellation.
# ---------------------------------------------------------------------------

def _test_config() -> dict[str, object]:
    return {
        "execution_profile": "sim_ompl",
        "thresholds": {
            "joint_state_fresh_s": 0.25,
            "tf_fresh_s": 0.25,
            "fixture_fresh_s": 0.25,
            "action_server_wait_s": 0.2,
            "goal_accept_timeout_s": 0.2,
            "plan_result_timeout_s": 0.2,
            "cancel_timeout_s": 0.2,
            "scene_acquire_timeout_s": 0.2,
        },
    }


def _join_key_provider():
    state = {"i": 0}

    def _provider():
        state["i"] += 1
        return (state["i"] * 10, float(state["i"]))

    return _provider


class _FakeFuture:
    def __init__(self, value=None, *, ready_at=0.0, exc=None):
        self._value = value
        self._ready_at = time.monotonic() + float(ready_at)
        self._cancelled = False
        self._exc = exc

    def done(self):
        return time.monotonic() >= self._ready_at

    def result(self):
        if not self.done():
            raise RuntimeError("future is not done")
        if self._exc is not None:
            raise self._exc
        return self._value

    def cancel(self):
        self._cancelled = True
        return False


class _FakeGoalHandle:
    def __init__(
        self,
        *,
        accepted=True,
        result=None,
        result_ready_at=0.0,
        cancel_ready_at=0.0,
        cancel_response="cancelled",
    ):
        self.accepted = accepted
        self.result = result
        self._result_ready_at = float(result_ready_at)
        self._cancel_ready_at = float(cancel_ready_at)
        self._cancel_response = cancel_response
        self.cancel_called = False

    def get_result_async(self):
        return _FakeFuture(self, ready_at=self._result_ready_at)

    def cancel_goal_async(self):
        self.cancel_called = True
        return _FakeFuture(self._cancel_response, ready_at=self._cancel_ready_at)


class _FakeMoveClient:
    def __init__(self, *, server_ready=True, goal_handle=None, send_ready_at=0.0, send_exc=None):
        self._server_ready = server_ready
        self._goal_handle = goal_handle
        self._send_ready_at = float(send_ready_at)
        self._send_exc = send_exc
        self.sent_goals = []

    def wait_for_server(self, timeout_sec=None):
        return self._server_ready

    def send_goal_async(self, goal):
        self.sent_goals.append(goal)
        return _FakeFuture(self._goal_handle, ready_at=self._send_ready_at, exc=self._send_exc)


def _success_result():
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    result = MoveGroup.Result()
    result.error_code = MoveItErrorCodes()
    result.error_code.val = 1  # MoveItErrorCodes.SUCCESS
    result.planned_trajectory = RobotTrajectory()
    result.planned_trajectory.joint_trajectory = JointTrajectory()
    point = JointTrajectoryPoint()
    point.positions = list(Q_OUTBOUND)
    result.planned_trajectory.joint_trajectory.points.append(point)
    return result


def _non_success_result():
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MoveItErrorCodes

    result = MoveGroup.Result()
    result.error_code = MoveItErrorCodes()
    result.error_code.val = 5  # NO_IK_SOLUTION (planner non-success)
    return result


def _empty_plan_success_result():
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory

    result = MoveGroup.Result()
    result.error_code = MoveItErrorCodes()
    result.error_code.val = 1
    result.planned_trajectory = RobotTrajectory()
    result.planned_trajectory.joint_trajectory = JointTrajectory()
    return result


def _request_error_result():
    """MoveItErrorCodes.FAILURE (99999): a generic request-level error."""
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MoveItErrorCodes

    result = MoveGroup.Result()
    result.error_code = MoveItErrorCodes()
    result.error_code.val = 99999
    return result


def _unknown_error_result():
    """An unknown/unsupported MoveItErrorCodes value."""
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MoveItErrorCodes

    result = MoveGroup.Result()
    result.error_code = MoveItErrorCodes()
    result.error_code.val = 12345
    return result


def _non_success_with_trajectory_result():
    """An allowlisted planning non-success code (NO_IK_SOLUTION=5) carrying a
    contradictory non-empty planned trajectory (F3.4)."""
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    result = MoveGroup.Result()
    result.error_code = MoveItErrorCodes()
    result.error_code.val = 5  # NO_IK_SOLUTION (planner non-success)
    result.planned_trajectory = RobotTrajectory()
    result.planned_trajectory.joint_trajectory = JointTrajectory()
    point = JointTrajectoryPoint()
    point.positions = list(Q_OUTBOUND)
    result.planned_trajectory.joint_trajectory.points.append(point)
    return result


def _mutated_invalid_graph():
    """A graph whose planning-scene topic type is corrupted so projection fails."""
    from test_integrated_gate_executor import _observed_graph_double

    graph = _observed_graph_double()
    graph["topics"]["/planning_scene"]["type"] = "std_msgs/msg/String"
    return graph


def _scene_with_ids(executor, contract, object_ids) -> None:
    """Feed a PlanningScene containing exactly *object_ids* through the callback.

    The objects carry IDs only (no geometry), so any scene whose owned IDs do not
    match the declared fixture contract is rejected on the ID projection before
    the F3.3 geometry projection is consulted.
    """
    from moveit_msgs.msg import AllowedCollisionMatrix, CollisionObject, PlanningScene, RobotState

    scene = PlanningScene()
    for object_id in object_ids:
        collision_object = CollisionObject()
        collision_object.id = object_id
        scene.world.collision_objects.append(collision_object)
    scene.allowed_collision_matrix = AllowedCollisionMatrix()
    scene.robot_state = RobotState()
    executor._make_scene_callback("/planning_scene")(scene)
    assert executor._latest_planning_scene is not None


class _MalformedScene:
    """A wrong-shaped PlanningScene input (no ``world``) that raises
    ``AttributeError`` at normalization time — F3.2 must contain it."""


def _declaration_records(declaration):
    """Ordered records the fixture adapter turns into collision bodies."""
    records = list(declaration.get("objects", []))
    for record in declaration.get("diagnostic_objects", []):
        if record.get("enter_collision_bodies") is True:
            records.append(record)
    return records


def _collision_object_from_record(record, frame_id, *, mutate=None):
    """Build a real CollisionObject from a declared fixture record."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive

    shape_types = {
        "box": SolidPrimitive.BOX,
        "sphere": SolidPrimitive.SPHERE,
        "cylinder": SolidPrimitive.CYLINDER,
    }
    obj = CollisionObject()
    obj.id = str(record["id"])
    obj.header.frame_id = frame_id
    if "primitive" in record:
        primitive = record["primitive"]
        sp = SolidPrimitive()
        sp.type = shape_types[primitive["type"]]
        sp.dimensions = [float(value) for value in primitive["dimensions"]]
        obj.primitives.append(sp)
    pose = record.get("pose", {})
    p = Pose()
    xyz = pose.get("xyz", [0.0, 0.0, 0.0])
    quat = pose.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
    p.position.x = float(xyz[0])
    p.position.y = float(xyz[1])
    p.position.z = float(xyz[2])
    p.orientation.x = float(quat[0])
    p.orientation.y = float(quat[1])
    p.orientation.z = float(quat[2])
    p.orientation.w = float(quat[3])
    obj.primitive_poses.append(p)
    if mutate is not None:
        mutate(obj)
    return obj


def _scene_from_declaration(executor, contract, *, mutate=None) -> None:
    """Feed a PlanningScene carrying the declared fixture geometry (optionally
    mutated per-object) through the callback."""
    from moveit_msgs.msg import AllowedCollisionMatrix, PlanningScene, RobotState

    declaration = contract["planning_scene_declaration"]
    frame_id = declaration["frame_id"]
    by_id = {str(record["id"]): record for record in _declaration_records(declaration)}
    scene = PlanningScene()
    for object_id in fixture_owned_ids(declaration):
        scene.world.collision_objects.append(
            _collision_object_from_record(by_id[str(object_id)], frame_id, mutate=mutate)
        )
    scene.allowed_collision_matrix = AllowedCollisionMatrix()
    scene.robot_state = RobotState()
    executor._make_scene_callback("/planning_scene")(scene)
    assert executor._latest_planning_scene is not None


def _synthetic_scene(executor, contract) -> None:
    from std_msgs.msg import String

    from test_integrated_gate_executor import _canonical_fixture_payload

    _scene_from_declaration(executor, contract)
    payload = String()
    payload.data = _canonical_fixture_payload(contract)
    executor._on_fixture_payload(payload)
    assert executor._fixture_payload is not None


def _make_executor(
    tmp_path, scenario_id="qualification-moveit-plan-joint", *, config=None, **kwargs
):
    from test_integrated_gate_executor import readiness_scenario, scenario_report_contract

    contract = scenario_report_contract(scenario_id)
    executor = IntegratedGateExecutor(
        scenario=readiness_scenario(contract),
        attempt_dir=Path(tmp_path),
        config=config if config is not None else _test_config(),
        ros_domain_id=47,
        **kwargs,
    )
    return executor, contract


def test_executor_constructs_isolated_node_and_clients(tmp_path):
    executor, _ = _make_executor(tmp_path)
    try:
        assert executor.node.get_name() == "tinker_integrated_gate_executor"
        assert executor.node.get_namespace() == "/"
        assert executor.node.get_fully_qualified_name() == "/tinker_integrated_gate_executor"
        assert set(executor._action_clients) == {
            "/move_action",
            "/execute_trajectory",
            "/xarm7_traj_controller/follow_joint_trajectory",
            "/xarm_gripper/gripper_action",
            "/pickup_action",
            "/place_action",
            "/cartesian_move_action",
            "/joint_move_action",
            "/fold_action",
        }
        assert len(executor._service_clients) == 11
        assert executor.ros["rclpy"].get_rmw_implementation_identifier() == "rmw_fastrtps_cpp"
    finally:
        executor.shutdown()


def test_executor_repeated_construct_shutdown_construct(tmp_path):
    executor, _ = _make_executor(tmp_path)
    executor.shutdown()
    executor2, _ = _make_executor(tmp_path, scenario_id="qualification-moveit-plan-pose")
    try:
        assert executor2.node.get_fully_qualified_name() == "/tinker_integrated_gate_executor"
        assert executor2.node.get_namespace() == "/"
    finally:
        executor2.shutdown()


def test_executor_subscriptions_use_real_planning_scene_type(tmp_path):
    executor, _ = _make_executor(tmp_path)
    try:
        by_topic = {sub.topic_name: sub for sub in executor.node.subscriptions}
        assert by_topic["/planning_scene"].msg_type.__name__ == "PlanningScene"
        assert by_topic["/monitored_planning_scene"].msg_type.__name__ == "PlanningScene"
        assert by_topic["/sim/status/planning_scene_fixture"].msg_type.__name__ == "String"
        assert by_topic["/joint_states"].msg_type.__name__ == "JointState"
        assert by_topic["/sim/hardware/safety_stop"].msg_type.__name__ == "Bool"
    finally:
        executor.shutdown()


def test_executor_planning_scene_subscription_qos_is_volatile_depth_100(tmp_path):
    """F2.3: the live subscriptions request the stock MoveIt2 Humble
    RELIABLE/VOLATILE/depth-100 contract; the fixture topic stays
    RELIABLE/TRANSIENT_LOCAL/depth 1."""
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

    executor, _ = _make_executor(tmp_path)
    try:
        by_topic = {sub.topic_name: sub for sub in executor.node.subscriptions}
        for name in ("/planning_scene", "/monitored_planning_scene"):
            profile = by_topic[name].qos_profile
            assert profile.depth == 100
            assert profile.reliability == ReliabilityPolicy.RELIABLE
            assert profile.durability == DurabilityPolicy.VOLATILE
        fixture_profile = by_topic["/sim/status/planning_scene_fixture"].qos_profile
        assert fixture_profile.depth == 1
        assert fixture_profile.reliability == ReliabilityPolicy.RELIABLE
        assert fixture_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
    finally:
        executor.shutdown()


def test_executor_normalizes_real_planning_scene(tmp_path):
    executor, contract = _make_executor(tmp_path)
    try:
        _synthetic_scene(executor, contract)
        scene = executor._latest_planning_scene
        declaration = contract["planning_scene_declaration"]
        expected_ids = list(fixture_owned_ids(declaration))
        assert scene["owned_ids"] == expected_ids
        assert scene["attached_ids"] == []
        assert scene["fixture_revision"] == declaration["revision"]
        assert scene["source"] == "/planning_scene"
        assert isinstance(scene["scene_sequence"], int) and scene["scene_sequence"] >= 1
        for field in ("scene_revision_digest", "acm_digest", "robot_state_digest"):
            assert re.fullmatch(r"[0-9a-f]{64}", scene[field]) is not None
    finally:
        executor.shutdown()


@pytest.mark.parametrize(
    ("scenario_id", "result_factory", "expected_status", "goal_has_joints"),
    [
        ("qualification-moveit-plan-joint", _success_result, "diagnostic-pass", True),
        ("qualification-moveit-plan-pose", _success_result, "diagnostic-pass", False),
        ("qualification-moveit-plan-blocked", _non_success_result, "diagnostic-pass", False),
    ],
)
def test_executor_run_gate_c_plan_only_stubbed_flow(
    tmp_path, scenario_id, result_factory, expected_status, goal_has_joints
):
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        scenario_id=scenario_id,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=result_factory())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)

        record = executor.run_gate_c_plan_only(scenario_id)
        assert record["status"] == expected_status
        assert record["diagnostic_only"] is True
        assert record["execute_trajectory_goal_sent"] is False
        assert record["isaac_joint_commands_published"] is False
        assert len(client.sent_goals) == 1
        goal = client.sent_goals[0]
        assert goal.request.group_name == "xarm7"
        assert goal.request.pipeline_id == "ompl"
        assert goal.request.num_planning_attempts == 3
        assert goal.request.allowed_planning_time == 3.0
        assert goal.planning_options.plan_only is True
        assert goal.planning_options.replan is False
        if goal_has_joints:
            constraints = goal.request.goal_constraints[0].joint_constraints
            assert [joint.joint_name for joint in constraints] == [f"joint{i}" for i in range(1, 8)]
            assert [joint.position for joint in constraints] == list(Q_OUTBOUND)
        else:
            assert goal.request.goal_constraints[0].position_constraints[0].link_name == "link_tcp"

        for name in (
            "integrated-execution.jsonl",
            "moveit-plans.jsonl",
            "controller-results.jsonl",
            "visual-capture-requests.jsonl",
            "planning-scene.jsonl",
        ):
            assert (tmp_path / name).exists(), name
        assert (tmp_path / "planning-scene.json").stat().st_size > 0
        assert (tmp_path / "integrated-execution.json").stat().st_size > 0
        goal_path = tmp_path / "goals" / f"{scenario_id}.json"
        assert goal_path.stat().st_size > 0
    finally:
        executor.shutdown()


def test_executor_joint_requires_nonempty_plan(tmp_path):
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_empty_plan_success_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "diagnostic-fail"
        assert record["nonempty_plan"] is False
    finally:
        executor.shutdown()


def test_executor_blocked_unexpected_success_is_failure(tmp_path):
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        scenario_id="qualification-moveit-plan-blocked",
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_success_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-blocked")
        assert record["status"] == "diagnostic-fail"
    finally:
        executor.shutdown()


def test_executor_readiness_gates_goal_before_send(tmp_path):
    from test_integrated_gate_executor import _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _ready_snapshot_for_contract(contract),
    )
    try:
        bad = _ready_snapshot_for_contract(contract)
        bad["robot_in_collision"] = True
        executor.readiness_snapshot_provider = lambda: bad
        client = _FakeMoveClient(
            server_ready=True,
            goal_handle=_FakeGoalHandle(accepted=True, result=_success_result()),
            send_ready_at=0.0,
        )
        executor._action_clients["/move_action"] = client
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "readiness-failed"
        assert client.sent_goals == []
    finally:
        executor.shutdown()


def test_executor_missing_providers_fail_closed(tmp_path):
    executor, _ = _make_executor(tmp_path)
    try:
        client = _FakeMoveClient(
            server_ready=True,
            goal_handle=_FakeGoalHandle(accepted=True, result=_success_result()),
            send_ready_at=0.0,
        )
        executor._action_clients["/move_action"] = client
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "no-join-key"
        assert client.sent_goals == []
    finally:
        executor.shutdown()


def test_executor_rejects_non_c_scenario(tmp_path):
    executor, _ = _make_executor(tmp_path, scenario_id="qualification-pick-place-positive")
    try:
        client = _FakeMoveClient(
            server_ready=True,
            goal_handle=_FakeGoalHandle(accepted=True, result=_success_result()),
            send_ready_at=0.0,
        )
        executor._action_clients["/move_action"] = client
        record = executor.run_gate_c_plan_only("qualification-pick-place-positive")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "scenario-rejected"
        assert client.sent_goals == []
    finally:
        executor.shutdown()


def test_executor_result_timeout_cancels_goal(tmp_path):
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(
            accepted=True,
            result=_success_result(),
            result_ready_at=1.0,
            cancel_ready_at=0.01,
            cancel_response="cancelled",
        )
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "result-timeout"
        assert record["planner_status"] == "timeout"
        assert record["cancel_response"] == "completed"
        assert handle.cancel_called is True
        assert record["execute_trajectory_goal_sent"] is False
    finally:
        executor.shutdown()


def test_executor_acceptance_uses_its_own_budget(tmp_path):
    """F1.7/m4: goal acceptance consuming most of its own deadline still leaves
    the result wait a fresh budget."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_success_result(), result_ready_at=0.01)
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.15)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "diagnostic-pass"
    finally:
        executor.shutdown()


def test_executor_rejected_goal_records_goal_rejected(tmp_path):
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=False, result=None)
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "goal-rejected"
        assert record["planner_status"] == "goal-rejected"
        assert record["execute_trajectory_goal_sent"] is False
    finally:
        executor.shutdown()


def test_executor_action_server_unavailable_fails_closed(tmp_path):
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        client = _FakeMoveClient(server_ready=False, goal_handle=None, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "action-server-unavailable"
        assert record["planner_status"] == "action-server-unavailable"
        assert client.sent_goals == []
    finally:
        executor.shutdown()


# --------------------------------------------------------------------------- #
# Fix round 2 (F2.1-F2.8): fail-dominant evidence, QoS, blocked allowlist,
# self-spin scene acquisition, fixture-scene match, visual chronology
# --------------------------------------------------------------------------- #


def test_executor_graph_unavailable_dominates_plan_pass(tmp_path):
    """F2.1 (BLOCKER): a successful plan is downgraded to evidence-invalid when
    graph evidence is unavailable; the raw planner pass is preserved separately
    and a failure planning-scene.json is always written."""
    from test_integrated_gate_executor import _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_success_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["planner_status"] == "diagnostic-pass"
        assert "invalid" in record["graph"]
        exec_json = json.loads((tmp_path / "integrated-execution.json").read_text(encoding="utf-8"))
        assert exec_json["status"] == "evidence-invalid"
        assert exec_json["planner_status"] == "diagnostic-pass"
        assert exec_json["execute_trajectory_goal_sent"] is False
        assert exec_json["isaac_joint_commands_published"] is False
        ps_json = json.loads((tmp_path / "planning-scene.json").read_text(encoding="utf-8"))
        assert ps_json["status"] == "evidence-invalid"
        assert "observed graph evidence is unavailable" in ps_json["reason"]
    finally:
        executor.shutdown()


def test_executor_graph_invalid_dominates_plan_pass(tmp_path):
    """F2.1: an invalid graph projection downgrades a plan pass to evidence-invalid."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_success_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        # Corrupt the observed graph so projection validation fails.
        executor.graph_observation_provider = lambda: _mutated_invalid_graph()
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["planner_status"] == "diagnostic-pass"
        assert "invalid" in record["graph"]
        assert (tmp_path / "planning-scene.json").stat().st_size > 0
    finally:
        executor.shutdown()


def test_executor_exceptional_send_future_fails_closed_with_complete_artifacts(tmp_path):
    """F2.2 (MAJOR): an exceptional send_goal_async future is converted to
    evidence-invalid with complete attempt artifacts and no pass claim."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        client = _FakeMoveClient(
            server_ready=True,
            goal_handle=None,
            send_ready_at=0.0,
            send_exc=RuntimeError("send failed hard"),
        )
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "goal-send-exception"
        assert record["planner_status"] == "goal-send-exception"
        for name in (
            "integrated-execution.jsonl",
            "moveit-plans.jsonl",
            "controller-results.jsonl",
            "visual-capture-requests.jsonl",
            "planning-scene.jsonl",
        ):
            assert (tmp_path / name).exists(), name
        assert (tmp_path / "integrated-execution.json").stat().st_size > 0
        assert (tmp_path / "planning-scene.json").stat().st_size > 0
        exec_json = json.loads((tmp_path / "integrated-execution.json").read_text(encoding="utf-8"))
        assert exec_json["status"] == "evidence-invalid"
        assert exec_json["execute_trajectory_goal_sent"] is False
        assert exec_json["isaac_joint_commands_published"] is False
        # The send future raised; no accepted goal handle existed, so teardown is
        # still recorded and the journal has fixture-ready + teardown.
        assert (tmp_path / "planning-scene.jsonl").stat().st_size > 0
    finally:
        executor.shutdown()


def test_executor_blocked_request_level_error_is_not_pass(tmp_path):
    """F2.4: a generic request-level MoveItErrorCode must not pass the blocked
    scenario; the classification is recorded."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        scenario_id="qualification-moveit-plan-blocked",
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_request_error_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-blocked")
        assert record["status"] == "diagnostic-fail"
        assert record["error_code_classification"] == "request-level-or-unknown"
        assert record["error_code"] == 99999
    finally:
        executor.shutdown()


def test_executor_blocked_unknown_code_is_not_pass(tmp_path):
    """F2.4: an unknown/unsupported MoveItErrorCode must not pass blocked."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        scenario_id="qualification-moveit-plan-blocked",
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_unknown_error_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-blocked")
        assert record["status"] == "diagnostic-fail"
        assert record["error_code_classification"] == "request-level-or-unknown"
    finally:
        executor.shutdown()


def test_executor_self_spin_acquires_scene(tmp_path):
    """F2.5: the scene becomes available only through the private spinner."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        # The scene is not yet cached; only the executor's own spin path seeds it.
        original_spin = executor._spin_once

        def seed_and_spin():
            _synthetic_scene(executor, contract)
            original_spin()

        executor._spin_once = seed_and_spin
        handle = _FakeGoalHandle(accepted=True, result=_success_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "diagnostic-pass"
        assert len(client.sent_goals) == 1
    finally:
        executor.shutdown()


def test_executor_scene_timeout_sends_zero_goals(tmp_path):
    """F2.5: if no scene arrives within the finite timeout, the attempt is
    evidence-invalid with zero goals sent."""
    from test_integrated_gate_executor import _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        client = _FakeMoveClient(
            server_ready=True,
            goal_handle=_FakeGoalHandle(accepted=True, result=_success_result()),
            send_ready_at=0.0,
        )
        executor._action_clients["/move_action"] = client
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "no-planning-scene"
        assert client.sent_goals == []
        assert not (tmp_path / "planning-scene.jsonl").exists() or (tmp_path / "planning-scene.jsonl").stat().st_size == 0
    finally:
        executor.shutdown()


@pytest.mark.parametrize(
    ("ids", "match"),
    [
        ([], "must equal"),
        (["sim_fixture/pedestal", "sim_fixture/qualification_cube"], "must equal"),
        (["sim_fixture/qualification_cube", "sim_fixture/pedestal", "sim_fixture/place_pedestal"], "must equal"),
        (["sim_fixture/pedestal", "sim_fixture/qualification_cube", "sim_fixture/place_pedestal", "sim_fixture/extra"], "must equal"),
        (["sim_fixture/pedestal", "sim_fixture/pedestal", "sim_fixture/qualification_cube", "sim_fixture/place_pedestal"], "must equal"),
    ],
)
def test_executor_fixture_scene_mismatch_rejected_before_goal(tmp_path, ids, match):
    """F2.6/F3.3: empty/missing/reordered/extra/duplicate fixture scenes are
    rejected before any goal send; the attempt is evidence-invalid."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        client = _FakeMoveClient(
            server_ready=True,
            goal_handle=_FakeGoalHandle(accepted=True, result=_success_result()),
            send_ready_at=0.0,
        )
        executor._action_clients["/move_action"] = client
        _scene_with_ids(executor, contract, list(ids))
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "fixture-scene-mismatch"
        assert match in record["reasons"][0]
        assert client.sent_goals == []
    finally:
        executor.shutdown()


def test_executor_fixture_scene_valid_matches(tmp_path):
    """F2.6: a full scene whose owned_ids match the declaration exactly passes."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_success_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "diagnostic-pass"
        assert len(client.sent_goals) == 1
    finally:
        executor.shutdown()


def test_executor_visual_before_precedes_goal_send(tmp_path):
    """F2.7: the `before` visual request is durably flushed before the goal send
    and `after` appears only in the post-transaction phase."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_success_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)

        phases_at_send: dict[str, list[str]] = {}
        original_send = client.send_goal_async

        def tracking_send(goal):
            path = tmp_path / "visual-capture-requests.jsonl"
            if path.exists():
                phases_at_send["phases"] = [
                    json.loads(line)["phase"]
                    for line in path.read_text(encoding="utf-8").splitlines()
                    if line
                ]
            else:
                phases_at_send["phases"] = []
            return original_send(goal)

        client.send_goal_async = tracking_send
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "diagnostic-pass"
        assert phases_at_send["phases"] == ["before"]

        lines = [
            json.loads(line)
            for line in (tmp_path / "visual-capture-requests.jsonl").read_text(encoding="utf-8").splitlines()
            if line
        ]
        assert [line["phase"] for line in lines] == ["before", "after"]
    finally:
        executor.shutdown()


def test_executor_acceptance_timeout_does_not_claim_server_cancel(tmp_path):
    """F2.8: an unaccepted/indeterminate send future is evidence-invalid and the
    evidence states that canceling the client future is not proof of server-side
    cancellation."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        client = _FakeMoveClient(
            server_ready=True,
            goal_handle=_FakeGoalHandle(accepted=True, result=_success_result()),
            send_ready_at=10.0,
        )
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "goal-accept-timeout"
        assert "not proof of server-side cancellation" in record["error"]
        assert record["planner_status"] == "goal-accept-timeout"
        assert (tmp_path / "integrated-execution.json").stat().st_size > 0
        exec_json = json.loads((tmp_path / "integrated-execution.json").read_text(encoding="utf-8"))
        assert exec_json["status"] == "evidence-invalid"
    finally:
        executor.shutdown()


# --------------------------------------------------------------------------- #
# Fix round 3 (F3.1): no persisted artifact may retain a pass after a late
# artifact failure
# --------------------------------------------------------------------------- #

def _jsonl_rows(path):
    if not Path(path).exists():
        return []
    return [
        json.loads(line)
        for line in Path(path).read_text(encoding="utf-8").splitlines()
        if line
    ]


def test_executor_artifact_write_failure_downgrades_planning_scene_pass(tmp_path):
    """F3.1: an artifact output failure after a valid graph + successful planner/
    journal processing downgrades every status-bearing artifact to
    evidence-invalid; the raw planner pass is preserved as planner_status only."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_success_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)

        # Force a failure at the early JSONL write position: the first
        # integrated-execution.jsonl row is durable with the provisional pass,
        # then the write of the remaining artifacts raises.
        def failing_write(scenario_id, spec, goal, record, readiness, graph_status):
            executor._append_jsonl(
                tmp_path / "integrated-execution.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": "integrated-manipulation-v1",
                    "scenario_id": scenario_id,
                    "event": "gate-c-plan-only",
                    "status": record.get("status"),
                    "planner_status": record.get("planner_status"),
                    "diagnostic_only": True,
                    "execute_trajectory_goal_sent": False,
                    "isaac_joint_commands_published": False,
                    "timestamp": 0.0,
                },
            )
            raise OSError("simulated jsonl write failure")

        executor._write_artifacts = failing_write
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "artifact-write-failed"
        assert record["planner_status"] == "diagnostic-pass"

        ps_json = json.loads((tmp_path / "planning-scene.json").read_text(encoding="utf-8"))
        assert ps_json["status"] == "evidence-invalid"
        exec_json = json.loads((tmp_path / "integrated-execution.json").read_text(encoding="utf-8"))
        assert exec_json["status"] == "evidence-invalid"
        assert exec_json["planner_status"] == "diagnostic-pass"
        rows = _jsonl_rows(tmp_path / "integrated-execution.jsonl")
        assert rows[-1]["status"] == "evidence-invalid"
        assert rows[-1]["row_kind"] == "final"
        assert not any(
            row.get("row_kind") == "final" and row["status"] == "diagnostic-pass"
            for row in rows
        )
        moveit_rows = _jsonl_rows(tmp_path / "moveit-plans.jsonl")
        assert moveit_rows and moveit_rows[-1]["status"] == "evidence-invalid"
        assert moveit_rows[-1]["row_kind"] == "final"
    finally:
        executor.shutdown()


def test_executor_late_final_summary_failure_downgrades_evidence(tmp_path):
    """F3.1: a failure at the late final-summary boundary (integrated-execution.json)
    after all JSONL rows are durable still leaves every status-bearing artifact
    evidence-invalid with planner_status preserved."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_success_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)

        calls = {"n": 0}
        original_write_json_atomic = executor._write_json_atomic

        def flaky_write_json_atomic(path, value):
            if Path(path).name == "integrated-execution.json" and calls["n"] == 0:
                calls["n"] += 1
                raise OSError("simulated final-summary write failure")
            calls["n"] += 1
            return original_write_json_atomic(path, value)

        executor._write_json_atomic = flaky_write_json_atomic
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "artifact-write-failed"
        assert record["planner_status"] == "diagnostic-pass"

        exec_json = json.loads((tmp_path / "integrated-execution.json").read_text(encoding="utf-8"))
        assert exec_json["status"] == "evidence-invalid"
        assert exec_json["planner_status"] == "diagnostic-pass"
        rows = _jsonl_rows(tmp_path / "integrated-execution.jsonl")
        assert rows[-1]["status"] == "evidence-invalid"
        assert rows[-1]["row_kind"] == "final"
        ps_json = json.loads((tmp_path / "planning-scene.json").read_text(encoding="utf-8"))
        assert ps_json["status"] == "evidence-invalid"
    finally:
        executor.shutdown()


def test_executor_journal_finalize_write_failure_downgrades_evidence(tmp_path):
    """F3.1: the successful final journal artifact is deferred until all other
    required artifacts are durable; if the planning-scene.json write itself fails
    after the other artifacts carried a provisional pass, everything is downgraded
    to evidence-invalid and planning-scene.json ends as a failure artifact."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_success_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)

        original_finalize = executor.journal.finalize

        def flaky_finalize(status, *, graph=None, json_path=None):
            if json_path is not None:
                raise OSError("simulated planning-scene.json write failure")
            return original_finalize(status, graph=graph, json_path=None)

        executor.journal.finalize = flaky_finalize
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "artifact-write-failed"
        assert record["planner_status"] == "diagnostic-pass"

        exec_json = json.loads((tmp_path / "integrated-execution.json").read_text(encoding="utf-8"))
        assert exec_json["status"] == "evidence-invalid"
        rows = _jsonl_rows(tmp_path / "integrated-execution.jsonl")
        assert rows[-1]["status"] == "evidence-invalid"
        assert rows[-1]["row_kind"] == "final"
        ps_json = json.loads((tmp_path / "planning-scene.json").read_text(encoding="utf-8"))
        assert ps_json["status"] == "evidence-invalid"
        assert "planning-scene.json write failure" in ps_json.get("reason", "")
    finally:
        executor.shutdown()


# --------------------------------------------------------------------------- #
# Fix round 3 (F3.2): valid PlanningScene data clears transient invalid state
# --------------------------------------------------------------------------- #

def test_executor_valid_then_invalid_scene_fails_closed(tmp_path):
    """F3.2: after a valid scene is cached, a newer invalid message makes the
    cached observation stale; acquisition fails closed and sends zero goals."""
    from test_integrated_gate_executor import _ready_snapshot_for_contract

    executor, contract = _make_executor(tmp_path, join_key_provider=_join_key_provider())
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        client = _FakeMoveClient(
            server_ready=True,
            goal_handle=_FakeGoalHandle(accepted=True, result=_success_result()),
            send_ready_at=0.0,
        )
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        assert executor._planning_scene_invalid is False
        executor._make_scene_callback("/planning_scene")(_MalformedScene())
        assert executor._planning_scene_invalid is True
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "planning-scene-invalid"
        assert client.sent_goals == []
    finally:
        executor.shutdown()


def test_executor_invalid_then_valid_scene_recovers_and_sends(tmp_path):
    """F3.2: a valid scene received after an invalid one clears the latch and the
    attempt can send a goal."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_success_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        executor._make_scene_callback("/planning_scene")(_MalformedScene())
        assert executor._planning_scene_invalid is True
        _synthetic_scene(executor, contract)
        assert executor._planning_scene_invalid is False
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-joint")
        assert record["status"] == "diagnostic-pass"
        assert len(client.sent_goals) == 1
    finally:
        executor.shutdown()


def test_executor_valid_invalid_valid_across_attempts_clears_latch(tmp_path):
    """F3.2: a transient invalid scene between two valid ones latches fail-closed
    for the intervening attempt, then a later valid scene clears the latch so the
    next attempt can acquire and proceed."""
    executor, contract = _make_executor(tmp_path)
    try:
        _synthetic_scene(executor, contract)
        assert executor._acquire_scene("qualification-moveit-plan-joint") is None
        executor._make_scene_callback("/planning_scene")(_MalformedScene())
        error = executor._acquire_scene("qualification-moveit-plan-joint")
        assert error["reason_code"] == "planning-scene-invalid"
        _synthetic_scene(executor, contract)
        assert executor._planning_scene_invalid is False
        assert executor._acquire_scene("qualification-moveit-plan-joint") is None
    finally:
        executor.shutdown()


def test_executor_wrong_shaped_scene_attribute_error_does_not_escape(tmp_path):
    """F3.2: a wrong-shaped message raising AttributeError is contained by the
    callback boundary; it latches invalid without escaping or erasing the last
    valid cached scene."""
    executor, contract = _make_executor(tmp_path)
    try:
        _synthetic_scene(executor, contract)
        previous = executor._latest_planning_scene
        executor._make_scene_callback("/planning_scene")(_MalformedScene())
        assert executor._planning_scene_invalid is True
        assert executor._latest_planning_scene is previous
    finally:
        executor.shutdown()


# --------------------------------------------------------------------------- #
# Fix round 3 (F3.3): fixture-ready binds exact declared geometry and pose
# --------------------------------------------------------------------------- #

def test_executor_fixture_scene_geometry_binds_declared_pose_and_shape(tmp_path):
    """F3.3: a full scene carrying the declared primitive geometry and poses is
    fixture-ready; the projection digest binds the exact declared geometry, not
    only the owned-ID set."""
    from validation.integrated_gate_executor import expected_fixture_geometry_digest

    executor, contract = _make_executor(tmp_path)
    try:
        declaration = contract["planning_scene_declaration"]
        _scene_from_declaration(executor, contract)
        scene = executor._latest_planning_scene
        assert scene["owned_ids"] == list(fixture_owned_ids(declaration))
        assert scene.get("fixture_geometry_digest") == expected_fixture_geometry_digest(declaration)
        assert executor._fixture_scene_error(scene) is None
    finally:
        executor.shutdown()


@pytest.mark.parametrize(
    ("mutation_name", "mutate"),
    [
        ("stale pose", lambda obj: setattr(obj.primitive_poses[0].position, "x", obj.primitive_poses[0].position.x + 0.05)),
        ("wrong dimensions", lambda obj: setattr(obj.primitives[0], "dimensions", [d + (0.1 if i == 2 else 0.0) for i, d in enumerate(obj.primitives[0].dimensions)])),
        ("wrong frame", lambda obj: setattr(obj.header, "frame_id", "world")),
    ],
)
def test_executor_fixture_scene_geometry_mutation_rejected(tmp_path, mutation_name, mutate):
    """F3.3: stale pose, wrong primitive dimensions, and wrong frame each break
    the fixture geometry projection and are rejected before fixture-ready."""
    executor, contract = _make_executor(tmp_path)
    try:
        _scene_from_declaration(executor, contract, mutate=mutate)
        scene = executor._latest_planning_scene
        error = executor._fixture_scene_error(scene)
        assert error is not None, mutation_name
        assert "geometry" in error, mutation_name
    finally:
        executor.shutdown()


def test_executor_fixture_scene_duplicate_id_rejected(tmp_path):
    """F3.3: a duplicate fixture-owned id in the received scene is rejected
    before fixture-ready (duplicate ids can never match the exact declared list)."""
    executor, contract = _make_executor(tmp_path)
    try:
        declaration = contract["planning_scene_declaration"]
        ids = list(fixture_owned_ids(declaration))
        ids.append(ids[0])
        _scene_with_ids(executor, contract, ids)
        scene = executor._latest_planning_scene
        error = executor._fixture_scene_error(scene)
        assert error is not None
        assert "owned_ids must equal" in error
    finally:
        executor.shutdown()


# --------------------------------------------------------------------------- #
# Fix round 3 (F3.4): blocked planning failure cannot pass with a contradictory
# non-empty trajectory
# --------------------------------------------------------------------------- #

def test_executor_blocked_allowlisted_code_with_nonempty_trajectory_is_failure(tmp_path):
    """F3.4: an allowlisted planning non-success code with a contradictory
    non-empty trajectory is diagnostic-fail with an explicit contradiction
    classification, never a blocked pass."""
    from test_integrated_gate_executor import _observed_graph_double, _ready_snapshot_for_contract

    executor, contract = _make_executor(
        tmp_path,
        scenario_id="qualification-moveit-plan-blocked",
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    try:
        executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
        handle = _FakeGoalHandle(accepted=True, result=_non_success_with_trajectory_result())
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gate_c_plan_only("qualification-moveit-plan-blocked")
        assert record["status"] == "diagnostic-fail"
        assert record["error_code_classification"] == "contradictory-nonempty-trajectory"
        assert record["error_code"] == 5
        assert record["nonempty_plan"] is True
        assert record["planner_status"] == "diagnostic-fail"
    finally:
        executor.shutdown()
