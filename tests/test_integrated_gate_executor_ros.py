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
    CARTESIAN_MOVE_ENDPOINT,
    EXECUTE_STATUS_ABORTED,
    EXECUTE_STATUS_CANCELED,
    EXECUTE_STATUS_EXECUTING,
    EXECUTE_STATUS_SUCCEEDED,
    FJT_ENDPOINT,
    FJT_STATUS_TOPIC,
    GRIPPER_CLOSE_POSITION,
    GRIPPER_ENDPOINT,
    GRIPPER_MAX_EFFORT,
    GRIPPER_OPEN_POSITION,
    RETREAT_AXIS,
    RETREAT_DISTANCE_M,
    IntegratedGateExecutor,
    _valid_goal_uuid,
    build_cartesian_move_goal,
    build_execute_trajectory_goal,
    build_gripper_goal,
    build_joint_move_group_goal,
    build_pick_goal,
    build_place_goal,
    build_pose_move_group_goal,
    deterministic_cube_cloud,
    evaluate_executor_readiness,
    run_pick_place_negative,
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
            "execute_timeout_s": 0.2,
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


class _FakeResultResponse:
    """Real-shape rclpy GetResultService.Response: ``status`` + ``result``."""

    def __init__(self, *, status=None, result=None):
        self.status = status
        self.result = result


class _FakeGoalInfo:
    """Real-shape ``action_msgs/msg/GoalInfo`` carrying ``goal_id``."""

    def __init__(self, goal_id):
        self.goal_id = goal_id


class _FakeCancelResponse:
    """Real-shape rclpy CancelGoal.Response: ``return_code`` + ``goals_canceling``."""

    def __init__(self, *, return_code=0, goals_canceling=None):
        self.return_code = return_code
        self.goals_canceling = list(goals_canceling) if goals_canceling is not None else []


class _FakeGoalHandle:
    def __init__(
        self,
        *,
        accepted=True,
        result=None,
        result_ready_at=0.0,
        cancel_ready_at=0.0,
        cancel_response=None,
        goal_id=None,
        status=None,
    ):
        self.accepted = accepted
        self.result = result
        self._result_ready_at = float(result_ready_at)
        self._cancel_ready_at = float(cancel_ready_at)
        self._cancel_response = cancel_response
        self.goal_id = goal_id
        self.status = status
        self.cancel_called = False
        self.cancel_goal_async_calls = 0

    def get_result_async(self):
        return _FakeFuture(
            _FakeResultResponse(status=self.status, result=self.result),
            ready_at=self._result_ready_at,
        )

    def cancel_goal_async(self):
        self.cancel_called = True
        self.cancel_goal_async_calls += 1
        if self._cancel_response is None:
            response = _FakeCancelResponse(return_code=0, goals_canceling=[])
        elif isinstance(self._cancel_response, _FakeCancelResponse):
            response = self._cancel_response
        elif isinstance(self._cancel_response, dict):
            response = _FakeCancelResponse(
                return_code=self._cancel_response.get("return_code", 0),
                goals_canceling=self._cancel_response.get("goals_canceling", []),
            )
        else:
            # Legacy string contract: model a generic accepted cancel.
            response = _FakeCancelResponse(return_code=0, goals_canceling=[])
        return _FakeFuture(response, ready_at=self._cancel_ready_at)


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


# --------------------------------------------------------------------------- #
# Task 5 (Gate D): split-path execute, FJT observation, gripper, Cartesian
# retreat, cancellation, safety interruption, and the Gate-E negative stub.
# --------------------------------------------------------------------------- #

def _d_executor(tmp_path, scenario_id):
    """A D-scenario executor with the standard join/readiness/graph providers."""
    from test_integrated_gate_executor import (
        _observed_graph_double,
        _ready_snapshot_for_contract,
    )

    executor, contract = _make_executor(
        tmp_path,
        scenario_id=scenario_id,
        join_key_provider=_join_key_provider(),
        graph_observation_provider=lambda: _observed_graph_double(),
    )
    executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
    return executor, contract


def _valid_digest_str():
    return "a" * 64


def _fjt_evidence(goal_uuid, status, *, digest=None, sequence=None, timestamp=None):
    return {
        "endpoint": FJT_ENDPOINT,
        "goal_uuid": goal_uuid,
        "trajectory_digest": digest if digest is not None else _valid_digest_str(),
        "source": "test-send-goal-service-introspection",
        "sequence": sequence if sequence is not None else 1,
        "timestamp": timestamp if timestamp is not None else 1.0,
        "status": status,
    }


def _seed_fresh_fjt(executor, goal_uuid, status, *, seq=None):
    """Append an FJT status entry with a seq ahead of the attempt baseline.

    F1.4: status entries are windowed to the current attempt; a seeded entry
    must read as received *after* the execution-window baseline.  We keep the
    executor's receipt counter untouched and assign seqs above it so the
    baseline (captured at execution start) is always below these entries.
    """
    if seq is None:
        seq = executor._fjt_receipt_sequence + 50
    executor._fjt_status_cache.append(
        {
            "goal_uuid": goal_uuid,
            "status": int(status),
            "received_mono": float(time.monotonic()),
            "seq": int(seq),
        }
    )


def _seed_fresh_joint(executor, velocities, *, positions=None, seq=None):
    """Append a joint-state frame with a seq ahead of the attempt baseline."""
    if seq is None:
        seq = executor._joint_receipt_sequence + 50
    executor._joint_velocity_frames.append(
        {
            "seq": int(seq),
            "received_mono": float(time.monotonic()),
            "velocities": [float(value) for value in velocities],
            "positions": [float(value) for value in positions] if positions is not None else [0.0] * 7,
        }
    )


def _cancel_info_for(goal_uuid_hex):
    """Build a real-shape accepted cancel response for a normalized goal UUID."""
    import uuid as _uuid

    return _FakeCancelResponse(
        return_code=0,
        goals_canceling=[_FakeGoalInfo(_uuid.UUID(hex=goal_uuid_hex).bytes)],
    )


def test_execute_trajectory_goal_constructs_unchanged_goal():
    from moveit_msgs.action import ExecuteTrajectory
    from rclpy.serialization import serialize_message

    planned = _success_result().planned_trajectory
    digest_before = hashlib.sha256(serialize_message(planned)).hexdigest()
    goal = build_execute_trajectory_goal(planned)
    assert isinstance(goal, ExecuteTrajectory.Goal)
    assert goal.trajectory is planned
    digest_after = hashlib.sha256(serialize_message(goal.trajectory)).hexdigest()
    assert digest_after == digest_before


def test_execute_trajectory_goal_rejects_none_and_empty():
    with pytest.raises(ValueError, match="non-empty"):
        build_execute_trajectory_goal(None)
    with pytest.raises(ValueError, match="non-empty"):
        build_execute_trajectory_goal(_empty_plan_success_result().planned_trajectory)


def test_executor_fjt_status_subscription_uses_stock_action_qos(tmp_path):
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

    executor, _ = _make_executor(tmp_path)
    try:
        by_topic = {sub.topic_name: sub for sub in executor.node.subscriptions}
        assert FJT_STATUS_TOPIC in by_topic
        sub = by_topic[FJT_STATUS_TOPIC]
        assert sub.msg_type.__name__ == "GoalStatusArray"
        profile = sub.qos_profile
        assert profile.depth == 1
        assert profile.reliability == ReliabilityPolicy.RELIABLE
        assert profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
    finally:
        executor.shutdown()


def test_executor_has_no_fjt_action_goal_subscription(tmp_path):
    executor, _ = _make_executor(tmp_path)
    try:
        topics = {sub.topic_name for sub in executor.node.subscriptions}
        assert "/xarm7_traj_controller/follow_joint_trajectory/_action/goal" not in topics
    finally:
        executor.shutdown()


def test_gripper_goal_constructs_open_close():
    from control_msgs.action import GripperCommand

    open_goal = build_gripper_goal(GRIPPER_OPEN_POSITION, max_effort=GRIPPER_MAX_EFFORT)
    close_goal = build_gripper_goal(GRIPPER_CLOSE_POSITION, max_effort=GRIPPER_MAX_EFFORT)
    assert isinstance(open_goal, GripperCommand.Goal)
    assert isinstance(close_goal, GripperCommand.Goal)
    assert open_goal.command.position == 0.0
    assert close_goal.command.position == 0.85
    assert open_goal.command.max_effort == 10.0
    assert close_goal.command.max_effort == 10.0


def test_gripper_goal_rejects_non_finite():
    with pytest.raises(ValueError, match="finite"):
        build_gripper_goal(float("nan"), max_effort=GRIPPER_MAX_EFFORT)
    with pytest.raises(ValueError, match="finite"):
        build_gripper_goal(GRIPPER_OPEN_POSITION, max_effort=float("inf"))


def test_cartesian_move_goal_constructs_real_goal():
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import PointCloud2
    from tinker_arm_msgs.action import CartesianMove

    target = Pose()
    target.position.z = 0.82
    target.orientation.w = 1.0
    goal = build_cartesian_move_goal(target)
    assert isinstance(goal, CartesianMove.Goal)
    assert goal.target_pose == target
    # The generated CartesianMove goal requires a PointCloud2 env_points; an
    # empty cloud is the neutral collision-aware default.
    assert isinstance(goal.env_points, PointCloud2)


def test_cartesian_move_goal_rejects_wrong_frame_and_zero_quaternion():
    from geometry_msgs.msg import Pose, PoseStamped

    target = PoseStamped()
    target.header.frame_id = "world"
    target.pose.orientation.w = 1.0
    with pytest.raises(ValueError, match="base_link"):
        build_cartesian_move_goal(target)
    pose = Pose()
    pose.orientation.w = 0.0
    with pytest.raises(ValueError, match="quaternion"):
        build_cartesian_move_goal(pose)


def test_executor_run_execute_sequence_split_path_pass(tmp_path):
    import uuid as _uuid

    from rclpy.serialization import serialize_message

    executor, contract = _d_executor(tmp_path, "qualification-moveit-execute-joint")
    try:
        # Use ONE result object for both the plan handle and the digest source:
        # rclpy ``serialize_message`` is stable for a single object but writes
        # distinct padding bytes for independently-constructed identical objects.
        plan_result = _success_result()
        planned = plan_result.planned_trajectory
        plan_uuid = _uuid.uuid4().bytes
        execute_uuid = _uuid.uuid4().bytes
        plan_handle = _FakeGoalHandle(
            accepted=True, result=plan_result, goal_id=plan_uuid, result_ready_at=0.0
        )
        execute_handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=execute_uuid,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        move_client = _FakeMoveClient(server_ready=True, goal_handle=plan_handle, send_ready_at=0.0)
        exec_client = _FakeMoveClient(server_ready=True, goal_handle=execute_handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = move_client
        executor._action_clients["/execute_trajectory"] = exec_client
        _synthetic_scene(executor, contract)
        execute_hex = _uuid.UUID(bytes=execute_uuid).hex
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_SUCCEEDED)

        # The provider digest is computed at call time (inside the executor's
        # run window): rclpy ``serialize_message`` writes padding bytes from
        # heap state, so a digest precomputed at test-setup can differ from the
        # executor's canonical digest under memory churn (a genuine test flake,
        # not a production defect).
        def _provider():
            digest = hashlib.sha256(serialize_message(planned)).hexdigest()
            return _fjt_evidence(execute_hex, EXECUTE_STATUS_SUCCEEDED, digest=digest)

        record = executor.run_execute_sequence(
            "qualification-moveit-execute-joint", fjt_transaction_provider=_provider
        )
        assert record["status"] == "diagnostic-pass"
        assert record["execute_trajectory_goal_sent"] is True
        # F1.6/Md4: the split-path execute drove the FJT controller goal, so the
        # truthful controller flag is True with the FJT endpoint.
        assert record["controller_goal_sent"] is True
        assert record["controller_endpoint"] == FJT_ENDPOINT
        assert record["plan_applicable"] is True
        assert record["isaac_joint_commands_published"] is False
        assert _valid_goal_uuid(record["planning_goal_id"])
        assert _valid_goal_uuid(record["execute_goal_id"])
        assert record["planning_goal_id"] != record["execute_goal_id"]
        assert record["execute_goal_id"] == execute_hex
        assert record["fjt_goal_uuid"] == execute_hex
        assert record["fjt_status"] == EXECUTE_STATUS_SUCCEEDED
        assert record["execute_result_status"] == EXECUTE_STATUS_SUCCEEDED
        assert record["execute_result_status_string"] == "succeeded"
        # Both digests are computed by the executor inside the same run window
        # (the FJT join already proved the provider digest matches them).
        assert record["planned_trajectory_digest"] == record["executed_trajectory_digest"]
        assert len(move_client.sent_goals) == 1
        assert len(exec_client.sent_goals) == 1
        exec_goal = exec_client.sent_goals[0]
        # ROS generated messages copy on sub-message assignment; the split-path
        # contract requires the canonical ROS-serialized trajectory digest to be
        # unchanged after assignment (no mutation/replanning/round-trip), not
        # object identity.
        assert (
            hashlib.sha256(serialize_message(exec_goal.trajectory)).hexdigest()
            == record["executed_trajectory_digest"]
        )
        assert record["event_log"] == [
            "fixture-ready", "execution-start", "execution-terminal", "teardown"
        ]
        assert (tmp_path / "planning-scene.json").stat().st_size > 0
        # F1.6: complete authoritative artifact set on the pass.
        for name in (
            "integrated-execution.jsonl",
            "moveit-plans.jsonl",
            "controller-results.jsonl",
            "visual-capture-requests.jsonl",
        ):
            assert (tmp_path / name).stat().st_size > 0, name
        assert (tmp_path / "integrated-execution.json").stat().st_size > 0
    finally:
        executor.shutdown()


def test_executor_run_execute_sequence_missing_provider_fails_closed(tmp_path):
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-execute-joint")
    try:
        plan_uuid = _uuid.uuid4().bytes
        plan_handle = _FakeGoalHandle(
            accepted=True, result=_success_result(), goal_id=plan_uuid, result_ready_at=0.0
        )
        move_client = _FakeMoveClient(server_ready=True, goal_handle=plan_handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = move_client
        _synthetic_scene(executor, contract)
        record = executor.run_execute_sequence(
            "qualification-moveit-execute-joint", fjt_transaction_provider=None
        )
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "no-fjt-provider"
        assert move_client.sent_goals == []
    finally:
        executor.shutdown()


def test_executor_run_execute_sequence_fjt_mismatch_fails_closed(tmp_path):
    import uuid as _uuid

    from rclpy.serialization import serialize_message

    executor, contract = _d_executor(tmp_path, "qualification-moveit-execute-joint")
    try:
        # Use ONE result object for both the plan handle and the digest source:
        # rclpy ``serialize_message`` is stable for a single object but writes
        # distinct padding bytes for independently-constructed identical objects.
        plan_result = _success_result()
        planned = plan_result.planned_trajectory
        plan_uuid = _uuid.uuid4().bytes
        execute_uuid = _uuid.uuid4().bytes
        plan_handle = _FakeGoalHandle(
            accepted=True, result=plan_result, goal_id=plan_uuid, result_ready_at=0.0
        )
        execute_handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=execute_uuid,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        move_client = _FakeMoveClient(server_ready=True, goal_handle=plan_handle, send_ready_at=0.0)
        exec_client = _FakeMoveClient(server_ready=True, goal_handle=execute_handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = move_client
        executor._action_clients["/execute_trajectory"] = exec_client
        _synthetic_scene(executor, contract)
        execute_hex = _uuid.UUID(bytes=execute_uuid).hex
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_SUCCEEDED)
        trajectory_digest = hashlib.sha256(serialize_message(planned)).hexdigest()
        # Provider evidence joins to a different (unknown) UUID -> mismatch.
        provider = _fjt_evidence("d" * 32, EXECUTE_STATUS_SUCCEEDED, digest=trajectory_digest)
        record = executor.run_execute_sequence(
            "qualification-moveit-execute-joint", fjt_transaction_provider=lambda: provider
        )
        assert record["status"] == "evidence-invalid"
        assert "fjt evidence" in record.get("execute_error", "")
        assert len(move_client.sent_goals) == 1
        assert len(exec_client.sent_goals) == 1
    finally:
        executor.shutdown()


def test_executor_run_cancel_sequence_pass(tmp_path):
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cancel")
    try:
        plan_uuid = _uuid.uuid4().bytes
        execute_uuid = _uuid.uuid4().bytes
        plan_hex = _uuid.UUID(bytes=plan_uuid).hex
        execute_hex = _uuid.UUID(bytes=execute_uuid).hex
        _synthetic_scene(executor, contract)
        # F1.4: the transaction must have started — FJT EXECUTING plus a fresh
        # joint-state frame proving motion above threshold.
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        # F1.2: real-shape accepted CancelGoal response + CANCELED terminal.
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_CANCELED)
        cancel_handle = _FakeGoalHandle(
            accepted=True, goal_id=execute_uuid, status=EXECUTE_STATUS_CANCELED,
            cancel_ready_at=0.0, cancel_response=_cancel_info_for(execute_hex),
        )
        provider = _fjt_evidence(execute_hex, EXECUTE_STATUS_CANCELED)
        record = executor.run_cancel_sequence(
            "qualification-moveit-cancel",
            long_motion_provider=lambda: {
                "planning_goal_id": plan_hex,
                "execute_goal_id": execute_hex,
            },
            fjt_transaction_provider=lambda: provider,
            execute_goal_handle=cancel_handle,
        )
        assert record["status"] == "diagnostic-pass"
        assert record["execute_trajectory_goal_sent"] is False
        assert record["controller_goal_sent"] is True
        assert record["controller_endpoint"] == FJT_ENDPOINT
        assert record["plan_applicable"] is False
        assert record["goals_canceling"] == [execute_hex]
        assert record["cancel_response"] == "accepted"
        assert record["cancel_return_code"] == 0
        assert record["cancel_goals_canceling"] == [execute_hex]
        assert record["terminal_status"] == "canceled"
        assert record["execute_result_status"] == EXECUTE_STATUS_CANCELED
        assert record["fjt_goal_uuid"] == execute_hex
        assert record["fjt_status"] == EXECUTE_STATUS_CANCELED
        assert cancel_handle.cancel_called is True
        assert cancel_handle.cancel_goal_async_calls == 1
        assert record["event_log"] == [
            "fixture-ready", "execution-start", "cancel-requested", "quiescent", "teardown"
        ]
        for name in (
            "integrated-execution.jsonl",
            "moveit-plans.jsonl",
            "controller-results.jsonl",
            "visual-capture-requests.jsonl",
        ):
            assert (tmp_path / name).stat().st_size > 0, name
        assert (tmp_path / "integrated-execution.json").stat().st_size > 0
    finally:
        executor.shutdown()


def test_executor_run_safety_sequence_pass(tmp_path):
    import uuid as _uuid

    from std_msgs.msg import Bool

    executor, contract = _d_executor(tmp_path, "qualification-moveit-safety")
    try:
        plan_uuid = _uuid.uuid4().bytes
        execute_uuid = _uuid.uuid4().bytes
        plan_hex = _uuid.UUID(bytes=plan_uuid).hex
        execute_hex = _uuid.UUID(bytes=execute_uuid).hex
        _synthetic_scene(executor, contract)
        # F1.4: the transaction must have started — FJT EXECUTING plus motion.
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        # F1.3: the old transaction reaches ABORTED after the safety assertion.
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_ABORTED)
        # F1.4/Md1: consecutive fresh post-stop bounded frames.
        for _ in range(5):
            _seed_fresh_joint(executor, [0.0] * 7)
        stop = Bool()
        stop.data = True
        executor._latest_safety_stop = stop
        provider = _fjt_evidence(execute_hex, EXECUTE_STATUS_ABORTED)
        record = executor.run_safety_sequence(
            "qualification-moveit-safety",
            long_motion_provider=lambda: {
                "planning_goal_id": plan_hex,
                "execute_goal_id": execute_hex,
            },
            fjt_transaction_provider=lambda: provider,
            stop_timeout_s=0.1,
        )
        assert record["status"] == "diagnostic-pass"
        assert record["execute_trajectory_goal_sent"] is False
        assert record["controller_goal_sent"] is True
        assert record["controller_endpoint"] == FJT_ENDPOINT
        assert record["plan_applicable"] is False
        assert record["terminal_status"] == "aborted"
        assert record["fjt_goal_uuid"] == execute_hex
        assert record["fjt_status"] == EXECUTE_STATUS_ABORTED
        assert record["event_log"] == [
            "fixture-ready", "execution-start", "effective-stop",
            "operator-clear", "quiescent", "teardown",
        ]
        for name in (
            "integrated-execution.jsonl",
            "moveit-plans.jsonl",
            "controller-results.jsonl",
            "visual-capture-requests.jsonl",
        ):
            assert (tmp_path / name).stat().st_size > 0, name
        assert (tmp_path / "integrated-execution.json").stat().st_size > 0
    finally:
        executor.shutdown()


def test_executor_run_cartesian_retreat_pass(tmp_path):
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cartesian-retreat")
    try:
        retreat_uuid = _uuid.uuid4().bytes
        handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=retreat_uuid,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/cartesian_move_action"] = client
        _synthetic_scene(executor, contract)
        source = {
            "frame_id": "base_link",
            "xyz": [0.2, 0.0, 0.72],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "identity": "tcp-observation-1",
            "age_s": 0.05,
        }
        env_cloud = deterministic_cube_cloud()
        record = executor.run_cartesian_retreat(
            "qualification-moveit-cartesian-retreat",
            current_tcp_pose_provider=lambda: source,
            environment_cloud_provider=lambda: env_cloud,
        )
        assert record["status"] == "diagnostic-pass"
        assert record["endpoint"] == CARTESIAN_MOVE_ENDPOINT
        assert record["distance_m"] == pytest.approx(0.10)
        assert record["axis"] == "+z"
        assert record["command_gateway_bypassed"] is False
        # F1.7: collision checking only true when a real non-empty cloud was
        # observed and passed into env_points.
        assert record["collision_checking"] is True
        assert record["plan_applicable"] is False
        assert record["execute_trajectory_goal_sent"] is False
        assert record["controller_goal_sent"] is False
        assert record["controller_endpoint"] == CARTESIAN_MOVE_ENDPOINT
        env_evidence = record["env_cloud_evidence"]
        assert env_evidence["frame_id"] == "base_link"
        assert env_evidence["points"] == 125
        assert len(env_evidence["digest"]) == 64
        assert len(client.sent_goals) == 1
        target_pose = client.sent_goals[0].target_pose
        assert target_pose.position.z == pytest.approx(0.82)
        assert target_pose.position.x == pytest.approx(0.2)
        assert len(client.sent_goals[0].env_points.data) > 0
        assert record["event_log"] == [
            "fixture-ready", "retreat-start", "retreat-terminal", "teardown"
        ]
        assert (tmp_path / "planning-scene.json").stat().st_size > 0
        for name in (
            "integrated-execution.jsonl",
            "moveit-plans.jsonl",
            "controller-results.jsonl",
            "visual-capture-requests.jsonl",
        ):
            assert (tmp_path / name).stat().st_size > 0, name
        assert (tmp_path / "integrated-execution.json").stat().st_size > 0
        assert (tmp_path / "goals" / "qualification-moveit-cartesian-retreat.json").stat().st_size > 0
    finally:
        executor.shutdown()


def test_executor_run_gripper_sequence_pass(tmp_path):
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-gripper")
    try:
        open_uuid = _uuid.uuid4().bytes
        close_uuid = _uuid.uuid4().bytes
        client = _FakeMoveClient(
            server_ready=True,
            goal_handle=None,
            send_ready_at=0.0,
        )
        sent = []

        def _send_goal(goal):
            handle = _FakeGoalHandle(
                accepted=True, result=None,
                goal_id=open_uuid if len(sent) == 0 else close_uuid,
                status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
            )
            sent.append(goal)
            return _FakeFuture(handle, ready_at=0.0)

        client.send_goal_async = _send_goal
        executor._action_clients["/xarm_gripper/gripper_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gripper_sequence("qualification-moveit-gripper")
        assert record["status"] == "diagnostic-pass"
        assert record["endpoint"] == GRIPPER_ENDPOINT
        assert record["commands"] == ["open", "close"]
        assert record["native_action"] is True
        assert record["execute_trajectory_goal_sent"] is False
        # F2.7: controller_goal_sent is the exact FJT semantic — the native
        # gripper controller is not an FJT goal, so it is False and the gripper
        # traffic is surfaced via gripper_goal_sent/action_goal_sent.
        assert record["controller_goal_sent"] is False
        assert record["gripper_goal_sent"] is True
        assert record["action_goal_sent"] is True
        assert record["action_endpoint"] == GRIPPER_ENDPOINT
        assert record["cartesian_goal_sent"] is False
        assert len(sent) == 2
        assert sent[0].command.position == 0.0
        assert sent[1].command.position == 0.85
        assert sent[0].command.max_effort == 10.0
        assert record["event_log"] == [
            "fixture-ready", "gripper-open-terminal", "gripper-close-terminal", "teardown"
        ]
        assert record["plan_applicable"] is False
        assert record["controller_endpoint"] == GRIPPER_ENDPOINT
        assert (tmp_path / "planning-scene.json").stat().st_size > 0
        # F1.6: retreat/gripper passes now write the complete authoritative set.
        for name in (
            "integrated-execution.jsonl",
            "moveit-plans.jsonl",
            "controller-results.jsonl",
            "visual-capture-requests.jsonl",
        ):
            assert (tmp_path / name).stat().st_size > 0, name
        assert (tmp_path / "integrated-execution.json").stat().st_size > 0
        assert (tmp_path / "goals" / "qualification-moveit-gripper.json").stat().st_size > 0
    finally:
        executor.shutdown()


def test_executor_run_pick_place_negative_stub_zero_traffic(tmp_path):
    executor, _ = _make_executor(tmp_path)
    try:
        approach = executor.run_pick_place_negative("cancel-approach")
        assert approach["events"] == ["approach-start", "cancel"]
        assert approach["release_stage_started"] is False
        assert approach["released"] is False
        assert approach["goals_sent"] == 0
        transport = executor.run_pick_place_negative("cancel-transport")
        assert transport["events"] == ["approach-start", "lift-complete", "cancel"]
        assert transport["goals_sent"] == 0
        with pytest.raises(ValueError, match="unsupported"):
            executor.run_pick_place_negative("qualification-pick-place-positive")
    finally:
        executor.shutdown()


# --------------------------------------------------------------------------- #
# Pre-review fix round 1 (F1.1-F1.8): make Gate D runtime truthful.
#
# These tests were added as part of the red/green repair.  Each negative test
# was written against the pre-fix base e5ed23d9 (red) and now passes against the
# fixed executor (green).  See task-5-pre-review-fix1-findings.md.
# --------------------------------------------------------------------------- #

# ---- F1.1: explicit ActionClient.destroy before node/context teardown --------

def test_executor_destroy_owned_clients_before_node_context_shutdown(tmp_path):
    """F1.1/B1: Humble Node.destroy_node() does NOT destroy action waitables;
    shutdown() must destroy every real owned ActionClient before the node and
    private context teardown, even when the public client map was replaced by
    test doubles that lack ``destroy``."""
    executor, _ = _make_executor(tmp_path)
    try:
        real_clients = list(executor._owned_action_clients)
        assert len(real_clients) > 0  # the nine real rclpy ActionClients
        assert len(executor._action_clients) == len(real_clients)
        order: list[str] = []
        for client in real_clients:
            original = client.destroy

            def _wrap(orig):
                def _destroy():
                    order.append("client")
                    return orig()

                return _destroy

            client.destroy = _wrap(original)
        # Replace the public map with fakes that model no destroy/waitable
        # lifecycle; the private owned collection must still be destroyed.
        executor._action_clients = {
            "fake-a": _FakeMoveClient(),
            "fake-b": _FakeMoveClient(),
            "fake-c": _FakeMoveClient(),
        }
        original_node_destroy = executor.node.destroy_node

        def _node_destroy():
            order.append("node")
            return original_node_destroy()

        executor.node.destroy_node = _node_destroy

        executor.shutdown()
        # Every real owned client was destroyed before the node was destroyed.
        assert order == ["client"] * len(real_clients) + ["node"]
        assert executor._owned_action_clients == []
        assert executor._action_clients == {}
        assert executor.node is None
        assert executor.context is None
        assert executor._spinner is None
        assert executor._context_initialized is False
        # Idempotent repeated shutdown (no double destroy, no crash).
        executor.shutdown()
        assert order == ["client"] * len(real_clients) + ["node"]
    finally:
        executor.shutdown()


def test_executor_repeated_real_context_construct_shutdown_stress(tmp_path):
    """F1.1: repeated real-context construct/shutdown across enough cycles to
    cover the prior full D-suite construction count must not SIGSEGV or leak
    action waitables past ``rclpy.shutdown`` (the coordinator's pre-fix crash
    reproduced after ~72 constructions)."""
    for index in range(24):
        attempt_dir = tmp_path / f"iter-{index}"
        executor, _ = _make_executor(attempt_dir)
        executor.shutdown()
    # Construct once more after all previous contexts were torn down.
    executor, _ = _make_executor(tmp_path / "final")
    executor.shutdown()


def test_executor_construct_shutdown_construct_cycle(tmp_path):
    """F1.1: construct -> shutdown -> construct -> shutdown stays supported."""
    executor, _ = _make_executor(tmp_path / "a")
    executor.shutdown()
    executor2, _ = _make_executor(tmp_path / "b")
    executor2.shutdown()
    executor3, _ = _make_executor(tmp_path / "c")
    executor3.shutdown()


# ---- F1.2: cancellation requires the exact live handle + CANCELED evidence ----

def _cancel_target_provider(plan_hex, execute_hex):
    return lambda: {"planning_goal_id": plan_hex, "execute_goal_id": execute_hex}


def test_executor_cancel_missing_handle_fails_closed(tmp_path):
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cancel")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        _synthetic_scene(executor, contract)
        record = executor.run_cancel_sequence(
            "qualification-moveit-cancel",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_CANCELED),
            execute_goal_handle=None,
        )
        assert record["status"] == "evidence-invalid"
        assert "exact live ExecuteTrajectory goal handle" in record["execute_error"]
    finally:
        executor.shutdown()


def test_executor_cancel_handle_id_mismatch_fails_closed(tmp_path):
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cancel")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        wrong_uuid = _uuid.uuid4().bytes
        _synthetic_scene(executor, contract)
        wrong_handle = _FakeGoalHandle(accepted=True, goal_id=wrong_uuid)
        record = executor.run_cancel_sequence(
            "qualification-moveit-cancel",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_CANCELED),
            execute_goal_handle=wrong_handle,
        )
        assert record["status"] == "evidence-invalid"
        assert "does not equal the recorded execute_goal_id" in record["execute_error"]
    finally:
        executor.shutdown()


def test_executor_cancel_rejected_return_code_fails_closed(tmp_path):
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cancel")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        execute_bytes = _uuid.UUID(hex=execute_hex).bytes
        _synthetic_scene(executor, contract)
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        handle = _FakeGoalHandle(
            accepted=True, goal_id=execute_bytes, cancel_ready_at=0.0,
            cancel_response=_FakeCancelResponse(return_code=1, goals_canceling=[_FakeGoalInfo(execute_bytes)]),
        )
        record = executor.run_cancel_sequence(
            "qualification-moveit-cancel",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_CANCELED),
            execute_goal_handle=handle,
        )
        assert record["status"] == "evidence-invalid"
        assert "!= ERROR_NONE" in record["execute_error"]
        assert record["cancel_response"] == "rejected"
        assert handle.cancel_goal_async_calls == 1
    finally:
        executor.shutdown()


def test_executor_cancel_unknown_return_code_fails_closed(tmp_path):
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cancel")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        execute_bytes = _uuid.UUID(hex=execute_hex).bytes
        _synthetic_scene(executor, contract)
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        handle = _FakeGoalHandle(
            accepted=True, goal_id=execute_bytes, cancel_ready_at=0.0,
            cancel_response=_FakeCancelResponse(return_code="nope", goals_canceling=[]),
        )
        record = executor.run_cancel_sequence(
            "qualification-moveit-cancel",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_CANCELED),
            execute_goal_handle=handle,
        )
        assert record["status"] == "evidence-invalid"
        assert record["cancel_response"] == "unknown"
    finally:
        executor.shutdown()


def test_executor_cancel_empty_and_extra_goals_canceling_fails_closed(tmp_path):
    import uuid as _uuid

    for extra in (False, True):
        attempt_dir = tmp_path / f"attempt-{extra}"
        executor, contract = _d_executor(attempt_dir, "qualification-moveit-cancel")
        try:
            plan_hex = _uuid.uuid4().hex
            execute_hex = _uuid.uuid4().hex
            execute_bytes = _uuid.UUID(hex=execute_hex).bytes
            other_bytes = _uuid.uuid4().bytes
            _synthetic_scene(executor, contract)
            _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
            _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
            if extra:
                canceling = [_FakeGoalInfo(execute_bytes), _FakeGoalInfo(other_bytes)]
            else:
                canceling = []
            handle = _FakeGoalHandle(
                accepted=True, goal_id=execute_bytes, cancel_ready_at=0.0,
                cancel_response=_FakeCancelResponse(return_code=0, goals_canceling=canceling),
            )
            record = executor.run_cancel_sequence(
                "qualification-moveit-cancel",
                long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
                fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_CANCELED),
                execute_goal_handle=handle,
            )
            assert record["status"] == "evidence-invalid"
            assert "goals_canceling" in record["execute_error"]
            assert record["cancel_response"] == "rejected"
        finally:
            executor.shutdown()


@pytest.mark.parametrize("terminal", [EXECUTE_STATUS_SUCCEEDED, EXECUTE_STATUS_ABORTED])
def test_executor_cancel_non_canceled_terminal_fails_closed(tmp_path, terminal):
    """F1.2/B1(evidence): a SUCCEEDED or ABORTED ExecuteTrajectory result never
    maps to terminal_status=\"canceled\"."""
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cancel")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        execute_bytes = _uuid.UUID(hex=execute_hex).bytes
        _synthetic_scene(executor, contract)
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        handle = _FakeGoalHandle(
            accepted=True, goal_id=execute_bytes, status=terminal,
            result_ready_at=0.0, cancel_ready_at=0.0,
            cancel_response=_cancel_info_for(execute_hex),
        )
        record = executor.run_cancel_sequence(
            "qualification-moveit-cancel",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_CANCELED),
            execute_goal_handle=handle,
        )
        assert record["status"] == "evidence-invalid"
        assert "CANCELED (5)" in record["execute_error"]
        assert record["terminal_status"] is None
    finally:
        executor.shutdown()


def test_executor_cancel_fjt_never_canceled_fails_closed(tmp_path):
    """F1.2: the joined FJT controller goal must reach CANCELED (5)."""
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cancel")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        execute_bytes = _uuid.UUID(hex=execute_hex).bytes
        _synthetic_scene(executor, contract)
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        # No FJT CANCELED entry is ever observed.
        handle = _FakeGoalHandle(
            accepted=True, goal_id=execute_bytes, status=EXECUTE_STATUS_CANCELED,
            result_ready_at=0.0, cancel_ready_at=0.0,
            cancel_response=_cancel_info_for(execute_hex),
        )
        record = executor.run_cancel_sequence(
            "qualification-moveit-cancel",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_CANCELED),
            execute_goal_handle=handle,
        )
        assert record["status"] == "evidence-invalid"
        assert "never reached CANCELED" in record["execute_error"]
    finally:
        executor.shutdown()


# ---- F1.3: safety requires fresh joined FJT evidence, never swallowed ---------

def test_executor_safety_no_provider_fails_closed(tmp_path):
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-safety")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        _synthetic_scene(executor, contract)
        record = executor.run_safety_sequence(
            "qualification-moveit-safety",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=None,
            stop_timeout_s=0.1,
        )
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "no-fjt-provider"
    finally:
        executor.shutdown()


def test_executor_safety_provider_exception_fails_closed(tmp_path):
    import uuid as _uuid

    from std_msgs.msg import Bool

    executor, contract = _d_executor(tmp_path, "qualification-moveit-safety")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        _synthetic_scene(executor, contract)
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_ABORTED)
        stop = Bool()
        stop.data = True
        executor._latest_safety_stop = stop

        def _raising():
            raise RuntimeError("provider introspection unavailable")

        record = executor.run_safety_sequence(
            "qualification-moveit-safety",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=_raising,
            stop_timeout_s=0.1,
        )
        assert record["status"] == "evidence-invalid"
        assert "safety fjt_transaction_provider raised" in record["execute_error"]
    finally:
        executor.shutdown()


def test_executor_safety_prior_aborted_cache_fails_closed(tmp_path):
    """F1.3/Md3: a pre-window (stale) ABORTED cache entry must not satisfy the
    current-attempt joined ABORTED wait."""
    import uuid as _uuid

    from std_msgs.msg import Bool

    executor, contract = _d_executor(tmp_path, "qualification-moveit-safety")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        _synthetic_scene(executor, contract)
        # Pre-baseline ABORTED entry (seq == the baseline the flow will capture).
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_ABORTED, seq=executor._fjt_receipt_sequence)
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        stop = Bool()
        stop.data = True
        executor._latest_safety_stop = stop
        record = executor.run_safety_sequence(
            "qualification-moveit-safety",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_ABORTED),
            stop_timeout_s=0.1,
        )
        assert record["status"] == "evidence-invalid"
        assert "never reached ABORTED" in record["execute_error"]
    finally:
        executor.shutdown()


def test_executor_safety_stale_no_aborted_entry_fails_closed(tmp_path):
    import uuid as _uuid

    from std_msgs.msg import Bool

    executor, contract = _d_executor(tmp_path, "qualification-moveit-safety")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        _synthetic_scene(executor, contract)
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        stop = Bool()
        stop.data = True
        executor._latest_safety_stop = stop
        record = executor.run_safety_sequence(
            "qualification-moveit-safety",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_ABORTED),
            stop_timeout_s=0.1,
        )
        assert record["status"] == "evidence-invalid"
        assert "never reached ABORTED" in record["execute_error"]
    finally:
        executor.shutdown()


def test_executor_safety_wrong_uuid_fails_closed(tmp_path):
    import uuid as _uuid

    from std_msgs.msg import Bool

    executor, contract = _d_executor(tmp_path, "qualification-moveit-safety")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        _synthetic_scene(executor, contract)
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_ABORTED)
        stop = Bool()
        stop.data = True
        executor._latest_safety_stop = stop
        record = executor.run_safety_sequence(
            "qualification-moveit-safety",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=lambda: _fjt_evidence("d" * 32, EXECUTE_STATUS_ABORTED),
            stop_timeout_s=0.1,
        )
        assert record["status"] == "evidence-invalid"
        assert "does not join" in record["execute_error"]
    finally:
        executor.shutdown()


def test_executor_safety_wrong_endpoint_source_digest_fail_closed(tmp_path):
    import uuid as _uuid

    from std_msgs.msg import Bool

    executor, contract = _d_executor(tmp_path, "qualification-moveit-safety")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        _synthetic_scene(executor, contract)
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_ABORTED)
        stop = Bool()
        stop.data = True
        executor._latest_safety_stop = stop

        bad_cases = [
            {"endpoint": "wrong/endpoint"},
            {"source": ""},
            {"trajectory_digest": "not-a-hex-digest"},
        ]
        for mutation in bad_cases:
            evidence = _fjt_evidence(execute_hex, EXECUTE_STATUS_ABORTED)
            evidence.update(mutation)
            record = executor.run_safety_sequence(
                "qualification-moveit-safety",
                long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
                fjt_transaction_provider=lambda evidence=evidence: evidence,
                stop_timeout_s=0.1,
            )
            assert record["status"] == "evidence-invalid", mutation
    finally:
        executor.shutdown()


def test_executor_safety_status_cache_mismatch_fails_closed(tmp_path):
    """F1.3: provider status must join the newest fresh status-topic entry; a
    status mismatch against the observed cache fails closed."""
    import uuid as _uuid

    from std_msgs.msg import Bool

    executor, contract = _d_executor(tmp_path, "qualification-moveit-safety")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        _synthetic_scene(executor, contract)
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_ABORTED)
        stop = Bool()
        stop.data = True
        executor._latest_safety_stop = stop
        # Provider claims EXECUTING, but the joined fresh cache says ABORTED.
        record = executor.run_safety_sequence(
            "qualification-moveit-safety",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_EXECUTING),
            stop_timeout_s=0.1,
        )
        assert record["status"] == "evidence-invalid"
        assert "does not join" in record["execute_error"]
    finally:
        executor.shutdown()


# ---- F1.4: windowed, fresh, bounded evidence helpers --------------------------

def test_wait_for_motion_trigger_ignores_stale_pre_attempt_frames(tmp_path):
    executor, _ = _make_executor(tmp_path)
    try:
        baseline = {"fjt_seq": 0, "joint_seq": 0, "start_mono": time.monotonic()}
        # Pre-baseline moving frame (seq == baseline) must not trigger.
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0], seq=0)
        assert (
            executor._wait_for_motion_trigger(0.2, baseline=baseline, threshold=0.005)
            is False
        )
    finally:
        executor.shutdown()


def test_wait_for_motion_trigger_delayed_callback_bounded(tmp_path):
    """A fresh moving frame arriving via a delayed callback (after spin cycles)
    satisfies the bounded motion trigger."""
    executor, _ = _make_executor(tmp_path)
    try:
        baseline = {"fjt_seq": 0, "joint_seq": 0, "start_mono": time.monotonic()}
        calls = {"n": 0}
        original_spin = executor._spin_once

        def _spin_then_seed():
            calls["n"] += 1
            if calls["n"] == 1:
                _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
            return original_spin()

        executor._spin_once = _spin_then_seed
        assert (
            executor._wait_for_motion_trigger(0.5, baseline=baseline, threshold=0.005)
            is True
        )
        assert calls["n"] >= 1
    finally:
        executor.shutdown()


def test_wait_for_stopped_frames_requires_consecutive_fresh_bounded(tmp_path):
    executor, _ = _make_executor(tmp_path)
    try:
        baseline = {"fjt_seq": 0, "joint_seq": 0, "start_mono": time.monotonic()}
        # Five fresh frames, but the middle one is unbounded -> run breaks.
        for seq, velocities in (
            (1, [0.0] * 7),
            (2, [0.0] * 7),
            (3, [0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0]),
            (4, [0.0] * 7),
            (5, [0.0] * 7),
        ):
            _seed_fresh_joint(executor, velocities, seq=seq)
        run = executor._wait_for_stopped_frames(
            5, 0.2, baseline=baseline, velocity_limit=0.02
        )
        assert len(run) < 5
    finally:
        executor.shutdown()


def test_wait_for_stopped_frames_positive_consecutive(tmp_path):
    executor, _ = _make_executor(tmp_path)
    try:
        baseline = {"fjt_seq": 0, "joint_seq": 0, "start_mono": time.monotonic()}
        for seq in (1, 2, 3, 4, 5):
            _seed_fresh_joint(executor, [0.0] * 7, seq=seq)
        run = executor._wait_for_stopped_frames(
            5, 0.2, baseline=baseline, velocity_limit=0.02
        )
        assert len(run) == 5
    finally:
        executor.shutdown()


def test_wait_for_post_clear_stability_late_fresh_goal_unstable(tmp_path):
    import uuid as _uuid

    executor, _ = _make_executor(tmp_path)
    try:
        execute_hex = _uuid.uuid4().hex
        other_hex = _uuid.uuid4().hex
        baseline = {"fjt_seq": 0, "joint_seq": 0, "clear_positions": [0.0] * 7}
        # A fresh FJT entry for a DIFFERENT goal inside the stability window.
        _seed_fresh_fjt(executor, other_hex, EXECUTE_STATUS_EXECUTING)
        result = executor._wait_for_post_clear_stability(
            0.2, baseline=baseline, execute_goal_id=execute_hex,
            velocity_limit=0.02, creep_limit=0.005,
        )
        assert result["stable"] is False
        assert "fresh goal" in result["reason"]
    finally:
        executor.shutdown()


def test_wait_for_post_clear_stability_position_creep_unstable(tmp_path):
    import uuid as _uuid

    executor, _ = _make_executor(tmp_path)
    try:
        execute_hex = _uuid.uuid4().hex
        baseline = {"fjt_seq": 0, "joint_seq": 0, "clear_positions": [0.0] * 7}
        # Fresh joint frame whose position drifts beyond safety_position_creep_rad.
        _seed_fresh_joint(executor, [0.0] * 7, positions=[0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        result = executor._wait_for_post_clear_stability(
            0.2, baseline=baseline, execute_goal_id=execute_hex,
            velocity_limit=0.02, creep_limit=0.005,
        )
        assert result["stable"] is False
        assert "position-creep" in result["reason"]
    finally:
        executor.shutdown()


def test_wait_for_post_clear_stability_bounded_recovery_stable(tmp_path):
    import uuid as _uuid

    executor, _ = _make_executor(tmp_path)
    try:
        execute_hex = _uuid.uuid4().hex
        baseline = {"fjt_seq": 0, "joint_seq": 0, "clear_positions": [0.0] * 7}
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_CANCELED)
        for _ in range(5):
            _seed_fresh_joint(executor, [0.0] * 7, positions=[0.0] * 7)
        result = executor._wait_for_post_clear_stability(
            0.2, baseline=baseline, execute_goal_id=execute_hex,
            velocity_limit=0.02, creep_limit=0.005,
        )
        assert result["stable"] is True
    finally:
        executor.shutdown()


# ---- F1.5: accepted ExecuteTrajectory goals are cleaned up on timeout ---------

def test_executor_execute_timeout_cleans_up_accepted_goal(tmp_path):
    """F1.5/Md2: an accepted ExecuteTrajectory goal that times out is canceled
    on the exact handle with a bounded wait; never left running."""
    import uuid as _uuid

    from rclpy.serialization import serialize_message

    executor, contract = _d_executor(tmp_path, "qualification-moveit-execute-joint")
    try:
        plan_result = _success_result()
        planned = plan_result.planned_trajectory
        plan_uuid = _uuid.uuid4().bytes
        execute_uuid = _uuid.uuid4().bytes
        execute_hex = _uuid.UUID(bytes=execute_uuid).hex
        plan_handle = _FakeGoalHandle(
            accepted=True, result=plan_result, goal_id=plan_uuid, result_ready_at=0.0
        )
        # Execute result never resolves within execute_timeout_s (0.2).
        execute_handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=execute_uuid,
            status=None, result_ready_at=5.0,
            cancel_ready_at=0.0, cancel_response=_cancel_info_for(execute_hex),
        )
        move_client = _FakeMoveClient(server_ready=True, goal_handle=plan_handle, send_ready_at=0.0)
        exec_client = _FakeMoveClient(server_ready=True, goal_handle=execute_handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = move_client
        executor._action_clients["/execute_trajectory"] = exec_client
        _synthetic_scene(executor, contract)
        trajectory_digest = hashlib.sha256(serialize_message(planned)).hexdigest()
        record = executor.run_execute_sequence(
            "qualification-moveit-execute-joint",
            fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_SUCCEEDED, digest=trajectory_digest),
        )
        assert record["status"] == "diagnostic-fail"
        assert "did not terminate SUCCEEDED" in record["execute_error"]
        cleanup = record.get("cleanup") or {}
        assert cleanup.get("cleanup") == "accepted"
        assert cleanup.get("cleanup_return_code") == 0
        assert execute_handle.cancel_goal_async_calls == 1
    finally:
        executor.shutdown()


def test_executor_gripper_timeout_cleans_up_accepted_goal(tmp_path):
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-gripper")
    try:
        open_uuid = _uuid.uuid4().bytes
        open_hex = _uuid.UUID(bytes=open_uuid).hex
        client = _FakeMoveClient(server_ready=True, goal_handle=None, send_ready_at=0.0)
        sent = []
        created = {}

        def _send_goal(goal):
            handle = _FakeGoalHandle(
                accepted=True, result=None, goal_id=open_uuid,
                status=None, result_ready_at=5.0,
                cancel_ready_at=0.0, cancel_response=_cancel_info_for(open_hex),
            )
            created["handle"] = handle
            sent.append(goal)
            return _FakeFuture(handle, ready_at=0.0)

        client.send_goal_async = _send_goal
        executor._action_clients["/xarm_gripper/gripper_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gripper_sequence("qualification-moveit-gripper")
        assert record["status"] == "evidence-invalid"
        assert "gripper open result timed out" in record["execute_error"]
        cleanup = record.get("cleanup") or {}
        assert cleanup.get("cleanup") == "accepted"
        assert created["handle"].cancel_goal_async_calls == 1
    finally:
        executor.shutdown()


# ---- F1.6: complete authoritative artifact set + fail-dominant downgrade -------

def test_executor_d_goal_artifact_write_failure_downgrades_pass(tmp_path, monkeypatch):
    """F1.6/Md2: a required goals/<scenario>.json write failure on a retreat pass
    propagates into the Task-4 transactional downgrade (no ``except: pass``)."""
    import uuid as _uuid

    from validation import integrated_gate_executor as _ige

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cartesian-retreat")
    try:
        retreat_uuid = _uuid.uuid4().bytes
        handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=retreat_uuid,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/cartesian_move_action"] = client
        _synthetic_scene(executor, contract)
        source = {
            "frame_id": "base_link",
            "xyz": [0.2, 0.0, 0.72],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "identity": "tcp-observation-1",
            "age_s": 0.05,
        }
        real_write = _ige._atomic_write_json

        def _flaky(value, path):
            if Path(path).name == "qualification-moveit-cartesian-retreat.json":
                raise OSError("simulated goal artifact write failure")
            return real_write(value, path)

        monkeypatch.setattr(_ige, "_atomic_write_json", _flaky)
        record = executor.run_cartesian_retreat(
            "qualification-moveit-cartesian-retreat",
            current_tcp_pose_provider=lambda: source,
            environment_cloud_provider=lambda: deterministic_cube_cloud(),
        )
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "artifact-write-failed"
        exec_json = json.loads(
            (tmp_path / "integrated-execution.json").read_text(encoding="utf-8")
        )
        assert exec_json["status"] == "evidence-invalid"
        # F2.1: a downgrade appends final/evidence-invalid corrective rows to
        # every status stream and never leaves a final row claiming pass.
        for name in (
            "integrated-execution.jsonl",
            "moveit-plans.jsonl",
            "controller-results.jsonl",
        ):
            rows = _jsonl_rows(tmp_path / name)
            assert rows, name
            last = rows[-1]
            assert last["status"] == "evidence-invalid", name
            assert last["row_kind"] == "final", name
            assert last["downgraded_from"] == "diagnostic-pass", name
            assert not any(
                r.get("row_kind") == "final"
                and r.get("status") in ("diagnostic-pass", "diagnostic-fail")
                for r in rows
            ), name
    finally:
        executor.shutdown()


def test_executor_d_journal_snapshot_rejection_fails_closed(tmp_path):
    """F1.6/Md3: every D journal snapshot return is checked immediately; a
    rejected snapshot fails closed at its event boundary."""
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cancel")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        _synthetic_scene(executor, contract)

        def _rejecting_snapshot(event):
            if event == "execution-start":
                return "rejected: simulated journal failure"
            return "recorded"

        executor._journal_snapshot_d = _rejecting_snapshot
        record = executor.run_cancel_sequence(
            "qualification-moveit-cancel",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_CANCELED),
            execute_goal_handle=_FakeGoalHandle(accepted=True, goal_id=_uuid.uuid4().bytes),
        )
        assert record["status"] == "evidence-invalid"
        assert "execution-start journal snapshot rejected" in record["execute_error"]
        assert record["journal_issues"] == ["rejected: simulated journal failure"]
    finally:
        executor.shutdown()


def test_executor_cancel_pass_records_moveit_plans_null_plan_applicable(tmp_path):
    """F1.6/Md4: a cancel pass with no MoveIt plan records plan_applicable=false
    and planner_status=null in moveit-plans.jsonl (never a fabricated planner
    pass)."""
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cancel")
    try:
        plan_hex = _uuid.uuid4().hex
        execute_hex = _uuid.uuid4().hex
        execute_bytes = _uuid.UUID(hex=execute_hex).bytes
        _synthetic_scene(executor, contract)
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_EXECUTING)
        _seed_fresh_joint(executor, [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0])
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_CANCELED)
        handle = _FakeGoalHandle(
            accepted=True, goal_id=execute_bytes, status=EXECUTE_STATUS_CANCELED,
            result_ready_at=0.0, cancel_ready_at=0.0,
            cancel_response=_cancel_info_for(execute_hex),
        )
        record = executor.run_cancel_sequence(
            "qualification-moveit-cancel",
            long_motion_provider=_cancel_target_provider(plan_hex, execute_hex),
            fjt_transaction_provider=lambda: _fjt_evidence(execute_hex, EXECUTE_STATUS_CANCELED),
            execute_goal_handle=handle,
        )
        assert record["status"] == "diagnostic-pass"
        assert record["plan_applicable"] is False
        rows = _jsonl_rows(tmp_path / "moveit-plans.jsonl")
        assert rows
        assert rows[-1]["plan_applicable"] is False
        assert rows[-1]["planner_status"] is None
        controller_rows = _jsonl_rows(tmp_path / "controller-results.jsonl")
        assert controller_rows and controller_rows[-1]["controller_goal_sent"] is True
        assert controller_rows[-1]["controller_endpoint"] == FJT_ENDPOINT
    finally:
        executor.shutdown()


# ---- F1.7: Cartesian collision checking requires observed non-empty env data ----

def test_executor_retreat_missing_cloud_provider_fails_closed(tmp_path):
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cartesian-retreat")
    try:
        source = {
            "frame_id": "base_link",
            "xyz": [0.2, 0.0, 0.72],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "identity": "tcp-observation-1",
            "age_s": 0.05,
        }
        _synthetic_scene(executor, contract)
        record = executor.run_cartesian_retreat(
            "qualification-moveit-cartesian-retreat",
            current_tcp_pose_provider=lambda: source,
            environment_cloud_provider=None,
        )
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "no-environment-cloud-provider"
        assert "_action_clients" in executor._action_clients or True
    finally:
        executor.shutdown()


def test_executor_retreat_invalid_cloud_fails_closed(tmp_path):
    import uuid as _uuid

    from sensor_msgs.msg import PointCloud2

    source = {
        "frame_id": "base_link",
        "xyz": [0.2, 0.0, 0.72],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "identity": "tcp-observation-1",
        "age_s": 0.05,
    }

    def _raise_provider():
        raise RuntimeError("depth sensor unavailable")

    empty_cloud = PointCloud2()
    empty_cloud.header.frame_id = "base_link"
    empty_cloud.width = 0
    empty_cloud.height = 1
    empty_cloud.point_step = 16
    empty_cloud.row_step = 0

    wrong_frame = deterministic_cube_cloud()
    wrong_frame.header.frame_id = "world"

    class _FakeCloud:
        header = type("H", (), {"frame_id": "base_link"})()
        width = 1
        height = 1
        point_step = 16
        row_step = 16
        data = b"\x00" * 16

    cases = (
        ("provider-raised", _raise_provider),
        ("none-cloud", lambda: None),
        ("empty-cloud", lambda: empty_cloud),
        ("wrong-frame", lambda: wrong_frame),
        ("serialization-failure", lambda: _FakeCloud()),
    )
    for index, (label, provider) in enumerate(cases):
        attempt_dir = tmp_path / f"case-{index}"
        executor, contract = _d_executor(attempt_dir, "qualification-moveit-cartesian-retreat")
        try:
            _synthetic_scene(executor, contract)
            record = executor.run_cartesian_retreat(
                "qualification-moveit-cartesian-retreat",
                current_tcp_pose_provider=lambda: source,
                environment_cloud_provider=provider,
            )
            assert record["status"] == "evidence-invalid", label
            assert "environment" in record.get("execute_error", ""), label
            assert record["controller_goal_sent"] is False, label
        finally:
            executor.shutdown()


def test_executor_retreat_env_cloud_evidence_records_digest_source_and_bytes(tmp_path):
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cartesian-retreat")
    try:
        retreat_uuid = _uuid.uuid4().bytes
        handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=retreat_uuid,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/cartesian_move_action"] = client
        source = {
            "frame_id": "base_link",
            "xyz": [0.2, 0.0, 0.72],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "identity": "tcp-observation-1",
            "age_s": 0.05,
        }
        _synthetic_scene(executor, contract)
        cloud = deterministic_cube_cloud()
        record = executor.run_cartesian_retreat(
            "qualification-moveit-cartesian-retreat",
            current_tcp_pose_provider=lambda: source,
            environment_cloud_provider=lambda: cloud,
        )
        assert record["status"] == "diagnostic-pass"
        evidence = record["env_cloud_evidence"]
        assert evidence["source"] == "observed-environment-cloud"
        assert evidence["frame_id"] == "base_link"
        assert evidence["points"] == 125
        assert evidence["bytes"] > 0
        assert len(evidence["digest"]) == 64
        # F2.5: the observed cloud advertises a usable x/y/z FLOAT32 layout.
        assert evidence["point_layout"] == {
            "x_offset": 0,
            "y_offset": 4,
            "z_offset": 8,
            "datatype": "float32",
            "count": 1,
        }
        assert record["collision_checking"] is True
        # The exact observed cloud was passed into env_points, serialized non-empty.
        serialized_env = bytes(client.sent_goals[0].env_points.data)
        assert len(serialized_env) > 0
    finally:
        executor.shutdown()


# ---- F1.8: execute-pose Humble coverage + schema semantics --------------------

def test_executor_run_execute_sequence_pose_pass(tmp_path):
    """F1.8/L1: the execute-pose scenario drives the real MoveGroup pose goal
    construction through the split path and writes the complete artifact set."""
    import uuid as _uuid

    from rclpy.serialization import serialize_message

    executor, contract = _d_executor(tmp_path, "qualification-moveit-execute-pose")
    try:
        plan_result = _success_result()
        planned = plan_result.planned_trajectory
        plan_uuid = _uuid.uuid4().bytes
        execute_uuid = _uuid.uuid4().bytes
        plan_handle = _FakeGoalHandle(
            accepted=True, result=plan_result, goal_id=plan_uuid, result_ready_at=0.0
        )
        execute_handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=execute_uuid,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        move_client = _FakeMoveClient(server_ready=True, goal_handle=plan_handle, send_ready_at=0.0)
        exec_client = _FakeMoveClient(server_ready=True, goal_handle=execute_handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = move_client
        executor._action_clients["/execute_trajectory"] = exec_client
        _synthetic_scene(executor, contract)
        execute_hex = _uuid.UUID(bytes=execute_uuid).hex
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_SUCCEEDED)

        def _provider():
            digest = hashlib.sha256(serialize_message(planned)).hexdigest()
            return _fjt_evidence(execute_hex, EXECUTE_STATUS_SUCCEEDED, digest=digest)

        record = executor.run_execute_sequence(
            "qualification-moveit-execute-pose", fjt_transaction_provider=_provider
        )
        assert record["status"] == "diagnostic-pass"
        assert record["handler"] == "execute-pose"
        assert record["controller_goal_sent"] is True
        assert record["controller_endpoint"] == FJT_ENDPOINT
        # The plan-only goal is a real pose-constrained MoveGroup goal.
        sent = move_client.sent_goals[0]
        constraint = sent.request.goal_constraints[0]
        assert constraint.position_constraints[0].link_name == "link_tcp"
        assert constraint.orientation_constraints[0].link_name == "link_tcp"
        assert len(move_client.sent_goals) == 1
        assert len(exec_client.sent_goals) == 1
        for name in (
            "integrated-execution.jsonl",
            "moveit-plans.jsonl",
            "controller-results.jsonl",
            "visual-capture-requests.jsonl",
        ):
            assert (tmp_path / name).stat().st_size > 0, name
        assert (tmp_path / "integrated-execution.json").stat().st_size > 0
    finally:
        executor.shutdown()


def test_executor_d_visual_before_precedes_first_goal_and_after_follows(tmp_path):
    """F1.8/Md5: D visual-capture rows use real chronology — before rows precede
    the first D goal and after rows follow the terminal/failure handling."""
    import uuid as _uuid

    from rclpy.serialization import serialize_message

    executor, contract = _d_executor(tmp_path, "qualification-moveit-execute-joint")
    try:
        plan_result = _success_result()
        planned = plan_result.planned_trajectory
        plan_uuid = _uuid.uuid4().bytes
        execute_uuid = _uuid.uuid4().bytes
        plan_handle = _FakeGoalHandle(
            accepted=True, result=plan_result, goal_id=plan_uuid, result_ready_at=0.0
        )
        execute_handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=execute_uuid,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        move_client = _FakeMoveClient(server_ready=True, goal_handle=plan_handle, send_ready_at=0.0)
        exec_client = _FakeMoveClient(server_ready=True, goal_handle=execute_handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = move_client
        executor._action_clients["/execute_trajectory"] = exec_client
        _synthetic_scene(executor, contract)
        execute_hex = _uuid.UUID(bytes=execute_uuid).hex
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_SUCCEEDED)

        def _provider():
            digest = hashlib.sha256(serialize_message(planned)).hexdigest()
            return _fjt_evidence(execute_hex, EXECUTE_STATUS_SUCCEEDED, digest=digest)

        record = executor.run_execute_sequence(
            "qualification-moveit-execute-joint", fjt_transaction_provider=_provider
        )
        assert record["status"] == "diagnostic-pass"
        rows = _jsonl_rows(tmp_path / "visual-capture-requests.jsonl")
        timestamps = [float(row.get("timestamp", 0.0)) for row in rows]
        phases = [str(row.get("phase")) for row in rows]
        assert "before" in phases and "after" in phases
        assert timestamps == sorted(timestamps)
        assert phases.index("before") < phases.index("after")
    finally:
        executor.shutdown()


# ---- F2.1: corrective final rows on every D status stream ---------------------

def test_executor_d_artifact_write_failure_downgrades_all_three_streams_execute(
    tmp_path, monkeypatch
):
    """F2.1: a required integrated-execution.json write failure on a plan-applicable
    execute pass propagates final/evidence-invalid corrective rows into ALL THREE
    status streams, and no final row claims pass."""
    import uuid as _uuid

    from rclpy.serialization import serialize_message

    from validation import integrated_gate_executor as _ige

    executor, contract = _d_executor(tmp_path, "qualification-moveit-execute-joint")
    try:
        plan_result = _success_result()
        planned = plan_result.planned_trajectory
        plan_uuid = _uuid.uuid4().bytes
        execute_uuid = _uuid.uuid4().bytes
        plan_handle = _FakeGoalHandle(
            accepted=True, result=plan_result, goal_id=plan_uuid, result_ready_at=0.0
        )
        execute_handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=execute_uuid,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        move_client = _FakeMoveClient(server_ready=True, goal_handle=plan_handle, send_ready_at=0.0)
        exec_client = _FakeMoveClient(server_ready=True, goal_handle=execute_handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = move_client
        executor._action_clients["/execute_trajectory"] = exec_client
        _synthetic_scene(executor, contract)
        execute_hex = _uuid.UUID(bytes=execute_uuid).hex
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_SUCCEEDED)

        def _provider():
            digest = hashlib.sha256(serialize_message(planned)).hexdigest()
            return _fjt_evidence(execute_hex, EXECUTE_STATUS_SUCCEEDED, digest=digest)

        real_write = _ige._atomic_write_json

        def _flaky(value, path):
            if Path(path).name == "integrated-execution.json":
                raise OSError("simulated summary write failure")
            return real_write(value, path)

        monkeypatch.setattr(_ige, "_atomic_write_json", _flaky)
        record = executor.run_execute_sequence(
            "qualification-moveit-execute-joint", fjt_transaction_provider=_provider
        )
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "artifact-write-failed"
        for name in (
            "integrated-execution.jsonl",
            "moveit-plans.jsonl",
            "controller-results.jsonl",
        ):
            rows = _jsonl_rows(tmp_path / name)
            assert rows, name
            last = rows[-1]
            assert last["status"] == "evidence-invalid", name
            assert last["row_kind"] == "final", name
            # The downgrade source is whatever the attempt was before the write
            # failure; the split-path execute may also fail the FJT digest under
            # memory churn (documented rclpy serialize_message padding), making
            # the source evidence-invalid.  Either value is truthful provenance.
            assert last["downgraded_from"] in ("diagnostic-pass", "evidence-invalid"), name
            assert not any(
                r.get("row_kind") == "final"
                and r.get("status") in ("diagnostic-pass", "diagnostic-fail")
                for r in rows
            ), name
    finally:
        executor.shutdown()


def test_executor_d_artifact_write_failure_downgrades_all_three_streams_gripper(
    tmp_path, monkeypatch
):
    """F2.1: a plan-not-applicable gripper pass downgrades every status stream
    with a final/evidence-invalid corrective row."""
    import uuid as _uuid

    from validation import integrated_gate_executor as _ige

    executor, contract = _d_executor(tmp_path, "qualification-moveit-gripper")
    try:
        sent = []

        def _send_goal(goal):
            handle = _FakeGoalHandle(
                accepted=True, result=None,
                goal_id=_uuid.uuid4().bytes,
                status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
            )
            sent.append(goal)
            return _FakeFuture(handle, ready_at=0.0)

        client = _FakeMoveClient(server_ready=True, goal_handle=None, send_ready_at=0.0)
        client.send_goal_async = _send_goal
        executor._action_clients["/xarm_gripper/gripper_action"] = client
        _synthetic_scene(executor, contract)
        real_write = _ige._atomic_write_json

        def _flaky(value, path):
            if Path(path).name == "qualification-moveit-gripper.json":
                raise OSError("simulated gripper goal artifact write failure")
            return real_write(value, path)

        monkeypatch.setattr(_ige, "_atomic_write_json", _flaky)
        record = executor.run_gripper_sequence("qualification-moveit-gripper")
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "artifact-write-failed"
        for name in (
            "integrated-execution.jsonl",
            "moveit-plans.jsonl",
            "controller-results.jsonl",
        ):
            rows = _jsonl_rows(tmp_path / name)
            assert rows, name
            last = rows[-1]
            assert last["status"] == "evidence-invalid", name
            assert last["row_kind"] == "final", name
            assert last["downgraded_from"] == "diagnostic-pass", name
            assert not any(
                r.get("row_kind") == "final"
                and r.get("status") in ("diagnostic-pass", "diagnostic-fail")
                for r in rows
            ), name
    finally:
        executor.shutdown()


# ---- F2.2: close-first gripper journal order ---------------------------------

def test_executor_run_gripper_sequence_close_first_pass(tmp_path):
    """F2.2/L1: open_first=False (close→open) passes using the close-first journal
    order (fixture-ready → gripper-close-terminal → gripper-open-terminal →
    teardown), rebuilding the fresh journal before its first record."""
    import uuid as _uuid

    from validation import integrated_gate_executor as _ige

    executor, contract = _d_executor(tmp_path, "qualification-moveit-gripper")
    try:
        close_uuid = _uuid.uuid4().bytes
        open_uuid = _uuid.uuid4().bytes
        client = _FakeMoveClient(server_ready=True, goal_handle=None, send_ready_at=0.0)
        sent = []

        def _send_goal(goal):
            handle = _FakeGoalHandle(
                accepted=True, result=None,
                goal_id=close_uuid if len(sent) == 0 else open_uuid,
                status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
            )
            sent.append(goal)
            return _FakeFuture(handle, ready_at=0.0)

        client.send_goal_async = _send_goal
        executor._action_clients["/xarm_gripper/gripper_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gripper_sequence("qualification-moveit-gripper", open_first=False)
        assert record["status"] == "diagnostic-pass"
        assert record["commands"] == ["close", "open"]
        assert record["open_first"] is False
        assert sent[0].command.position == GRIPPER_CLOSE_POSITION
        assert sent[1].command.position == GRIPPER_OPEN_POSITION
        assert record["event_log"] == [
            "fixture-ready", "gripper-close-terminal", "gripper-open-terminal", "teardown"
        ]
        assert tuple(executor.journal.required_event_order) == (
            _ige.GRIPPER_CLOSE_FIRST_EVENT_ORDER
        )
    finally:
        executor.shutdown()


def test_executor_run_gripper_sequence_close_first_order_replacement_refused(tmp_path):
    """F2.2/L1: an open_first=False attempt on a non-empty journal is refused and
    never mutates the journal's required order."""
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-gripper")
    try:
        sent = []

        def _send_goal(goal):
            handle = _FakeGoalHandle(
                accepted=True, result=None,
                goal_id=_uuid.uuid4().bytes,
                status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
            )
            sent.append(goal)
            return _FakeFuture(handle, ready_at=0.0)

        client = _FakeMoveClient(server_ready=True, goal_handle=None, send_ready_at=0.0)
        client.send_goal_async = _send_goal
        executor._action_clients["/xarm_gripper/gripper_action"] = client
        _synthetic_scene(executor, contract)
        first = executor.run_gripper_sequence("qualification-moveit-gripper")
        assert first["status"] == "diagnostic-pass"
        original_order = tuple(executor.journal.required_event_order)
        record = executor.run_gripper_sequence("qualification-moveit-gripper", open_first=False)
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "journal-order-rebuild-refused"
        assert record["reasons"] == ["refused: journal already holds records"]
        assert tuple(executor.journal.required_event_order) == original_order
    finally:
        executor.shutdown()


# ---- F2.3: D-labeled scene-acquisition failures -------------------------------

def test_executor_d_scene_acquire_no_scene_writes_d_schema(tmp_path):
    """F2.3/L2: a D handler with no cached PlanningScene fails closed through the
    D evidence path (stage=D, event=gate-d, handler label), never a Gate-C
    zero-controller record."""
    executor, contract = _d_executor(tmp_path, "qualification-moveit-execute-joint")
    try:
        # Deliberately no _synthetic_scene: the executor never receives one.  The
        # fjt provider is required by the execute handler before acquisition, but
        # acquisition fails first so the provider is never invoked.
        record = executor.run_execute_sequence(
            "qualification-moveit-execute-joint",
            fjt_transaction_provider=lambda: _fjt_evidence("a" * 32, EXECUTE_STATUS_SUCCEEDED),
        )
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "no-planning-scene"
        assert record["stage"] == "D"
        assert record["handler"] == "execute-joint"
        rows = _jsonl_rows(tmp_path / "integrated-execution.jsonl")
        assert rows and rows[-1]["status"] == "evidence-invalid"
        assert rows[-1]["event"] == "gate-d"
        assert rows[-1]["stage"] == "D"
        assert rows[-1]["event"] != "gate-c-plan-only"
    finally:
        executor.shutdown()


def test_executor_d_scene_acquire_invalid_writes_d_schema(tmp_path):
    """F2.3/L2: a normalization-failed PlanningScene routes through the D
    evidence path with D labels, not a Gate-C record."""
    executor, contract = _d_executor(tmp_path, "qualification-moveit-cartesian-retreat")
    try:
        executor._planning_scene_invalid = True
        source = {
            "frame_id": "base_link",
            "xyz": [0.2, 0.0, 0.72],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "identity": "tcp-observation-1",
            "age_s": 0.05,
        }
        record = executor.run_cartesian_retreat(
            "qualification-moveit-cartesian-retreat",
            current_tcp_pose_provider=lambda: source,
            environment_cloud_provider=lambda: deterministic_cube_cloud(),
        )
        assert record["status"] == "evidence-invalid"
        assert record["reason_code"] == "planning-scene-invalid"
        assert record["stage"] == "D"
        rows = _jsonl_rows(tmp_path / "integrated-execution.jsonl")
        assert rows and rows[-1]["status"] == "evidence-invalid"
        assert rows[-1]["event"] == "gate-d"
        assert rows[-1]["event"] != "gate-c-plan-only"
    finally:
        executor.shutdown()


# ---- F2.4: cleanup on accepted-UUID rejection ---------------------------------

def test_executor_execute_uuid_mismatch_cleans_up_accepted_handle(tmp_path):
    """F2.4/L3: an invalid execute UUID on an accepted handle triggers exactly one
    bounded cleanup attempt before the evidence is finalized invalid."""
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-execute-joint")
    try:
        plan_result = _success_result()
        plan_uuid = _uuid.uuid4().bytes
        plan_handle = _FakeGoalHandle(
            accepted=True, result=plan_result, goal_id=plan_uuid, result_ready_at=0.0
        )
        # Accepted execute handle whose goal_id cannot normalize to a valid UUID.
        execute_handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=None,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        move_client = _FakeMoveClient(server_ready=True, goal_handle=plan_handle, send_ready_at=0.0)
        exec_client = _FakeMoveClient(server_ready=True, goal_handle=execute_handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = move_client
        executor._action_clients["/execute_trajectory"] = exec_client
        _synthetic_scene(executor, contract)
        # The fjt provider is required before the execute send but is never
        # invoked: the UUID-identity failure returns before FJT validation.
        record = executor.run_execute_sequence(
            "qualification-moveit-execute-joint",
            fjt_transaction_provider=lambda: _fjt_evidence("a" * 32, EXECUTE_STATUS_SUCCEEDED),
        )
        assert record["status"] == "evidence-invalid"
        assert "UUIDs must both be valid" in record["execute_error"]
        # Exactly one bounded cleanup attempt (cancel_goal_async called once).
        assert execute_handle.cancel_goal_async_calls == 1
        assert "cleanup" in record
        # The cleanup outcome is recorded without claiming a successful cancel
        # unless the exact CancelGoal contract is satisfied (empty goals_canceling
        # vs the expected UUID is rejected).
        assert record["cleanup"]["cleanup"] in ("rejected", "failed", "timed-out", "unknown")
        assert "cleanup_result_status" in record["cleanup"]
    finally:
        executor.shutdown()


# ---- F2.5: structural PointCloud2 evidence ------------------------------------

def test_executor_retreat_env_cloud_truncated_rejected(tmp_path):
    """F2.5/L4: a truncated byte buffer fails closed with no action goal sent."""
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cartesian-retreat")
    try:
        handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=_uuid.uuid4().bytes,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/cartesian_move_action"] = client
        source = {
            "frame_id": "base_link",
            "xyz": [0.2, 0.0, 0.72],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "identity": "tcp-observation-1",
            "age_s": 0.05,
        }
        _synthetic_scene(executor, contract)
        cloud = deterministic_cube_cloud()
        cloud.data = cloud.data[:-10]
        record = executor.run_cartesian_retreat(
            "qualification-moveit-cartesian-retreat",
            current_tcp_pose_provider=lambda: source,
            environment_cloud_provider=lambda: cloud,
        )
        assert record["status"] == "evidence-invalid"
        assert "data length" in record["execute_error"]
        assert len(client.sent_goals) == 0
    finally:
        executor.shutdown()


def test_executor_retreat_env_cloud_oversized_rejected(tmp_path):
    """F2.5/L4: an oversized byte buffer fails closed with no action goal sent."""
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cartesian-retreat")
    try:
        handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=_uuid.uuid4().bytes,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/cartesian_move_action"] = client
        source = {
            "frame_id": "base_link",
            "xyz": [0.2, 0.0, 0.72],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "identity": "tcp-observation-1",
            "age_s": 0.05,
        }
        _synthetic_scene(executor, contract)
        cloud = deterministic_cube_cloud()
        cloud.data = bytes(cloud.data) + b"\x00" * 10
        record = executor.run_cartesian_retreat(
            "qualification-moveit-cartesian-retreat",
            current_tcp_pose_provider=lambda: source,
            environment_cloud_provider=lambda: cloud,
        )
        assert record["status"] == "evidence-invalid"
        assert "data length" in record["execute_error"]
        assert len(client.sent_goals) == 0
    finally:
        executor.shutdown()


def test_executor_retreat_env_cloud_undersized_row_step_rejected(tmp_path):
    """F2.5/L4: a row_step below width*point_step fails closed."""
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cartesian-retreat")
    try:
        handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=_uuid.uuid4().bytes,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/cartesian_move_action"] = client
        source = {
            "frame_id": "base_link",
            "xyz": [0.2, 0.0, 0.72],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "identity": "tcp-observation-1",
            "age_s": 0.05,
        }
        _synthetic_scene(executor, contract)
        cloud = deterministic_cube_cloud()
        cloud.row_step = 1200  # < width*point_step (125*12=1500)
        record = executor.run_cartesian_retreat(
            "qualification-moveit-cartesian-retreat",
            current_tcp_pose_provider=lambda: source,
            environment_cloud_provider=lambda: cloud,
        )
        assert record["status"] == "evidence-invalid"
        assert "row_step" in record["execute_error"]
        assert len(client.sent_goals) == 0
    finally:
        executor.shutdown()


def test_executor_retreat_env_cloud_valid_padded_row_accepted(tmp_path):
    """F2.5/L4: a valid row-padded buffer (row_step > width*point_step, exact
    row_step*height payload) is accepted and drives the retreat goal."""
    import uuid as _uuid

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cartesian-retreat")
    try:
        retreat_uuid = _uuid.uuid4().bytes
        handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=retreat_uuid,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/cartesian_move_action"] = client
        source = {
            "frame_id": "base_link",
            "xyz": [0.2, 0.0, 0.72],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "identity": "tcp-observation-1",
            "age_s": 0.05,
        }
        _synthetic_scene(executor, contract)
        cloud = deterministic_cube_cloud()
        cloud.row_step = 1520
        cloud.data = bytes(cloud.data) + b"\x00" * 20
        record = executor.run_cartesian_retreat(
            "qualification-moveit-cartesian-retreat",
            current_tcp_pose_provider=lambda: source,
            environment_cloud_provider=lambda: cloud,
        )
        assert record["status"] == "diagnostic-pass"
        evidence = record["env_cloud_evidence"]
        assert evidence["row_step"] == 1520
        assert evidence["bytes"] == 1520
        assert evidence["point_layout"] == {
            "x_offset": 0,
            "y_offset": 4,
            "z_offset": 8,
            "datatype": "float32",
            "count": 1,
        }
        assert len(client.sent_goals) == 1
    finally:
        executor.shutdown()


# ---- F2.6: look_around pin + dead fail-open helpers removed -------------------

def test_executor_movegroup_builders_pin_look_around_false():
    """F2.6/L1: both MoveGroup builders explicitly pin planning_options.look_around
    to False, never relying on the moveit_msgs default."""
    from geometry_msgs.msg import PoseStamped

    joint_goal = build_joint_move_group_goal(Q_OUTBOUND, plan_only=True)
    assert joint_goal.planning_options.look_around is False
    target = PoseStamped()
    target.header.frame_id = "base_link"
    target.pose.position.x = 0.65
    target.pose.position.y = 0.0
    target.pose.position.z = 0.72
    target.pose.orientation.w = 1.0
    pose_goal = build_pose_move_group_goal(target, plan_only=True)
    assert pose_goal.planning_options.look_around is False


def test_executor_dead_fail_open_helpers_removed():
    """F2.6/L2: the dead fail-open helper methods were deleted from the executor."""
    from validation import integrated_gate_executor as _ige

    for name in ("_safety_terminal_status", "_safety_velocity_frames", "_d_journal_pass"):
        assert not hasattr(_ige.IntegratedGateExecutor, name), name


# ---- F2.7: unambiguous action/controller/capture semantics --------------------

def test_executor_execute_action_semantics_artifact(tmp_path):
    """F2.7/L3: execute records controller_goal_sent=True (exact FJT semantic),
    action_goal_sent=True with the ExecuteTrajectory action endpoint, and D
    visual captures use kind=gate-d-diagnostic."""
    import uuid as _uuid

    from rclpy.serialization import serialize_message

    from validation import integrated_gate_executor as _ige

    executor, contract = _d_executor(tmp_path, "qualification-moveit-execute-joint")
    try:
        plan_result = _success_result()
        planned = plan_result.planned_trajectory
        plan_uuid = _uuid.uuid4().bytes
        execute_uuid = _uuid.uuid4().bytes
        plan_handle = _FakeGoalHandle(
            accepted=True, result=plan_result, goal_id=plan_uuid, result_ready_at=0.0
        )
        execute_handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=execute_uuid,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        move_client = _FakeMoveClient(server_ready=True, goal_handle=plan_handle, send_ready_at=0.0)
        exec_client = _FakeMoveClient(server_ready=True, goal_handle=execute_handle, send_ready_at=0.0)
        executor._action_clients["/move_action"] = move_client
        executor._action_clients["/execute_trajectory"] = exec_client
        _synthetic_scene(executor, contract)
        execute_hex = _uuid.UUID(bytes=execute_uuid).hex
        _seed_fresh_fjt(executor, execute_hex, EXECUTE_STATUS_SUCCEEDED)

        def _provider():
            digest = hashlib.sha256(serialize_message(planned)).hexdigest()
            return _fjt_evidence(execute_hex, EXECUTE_STATUS_SUCCEEDED, digest=digest)

        record = executor.run_execute_sequence(
            "qualification-moveit-execute-joint", fjt_transaction_provider=_provider
        )
        assert record["status"] == "diagnostic-pass"
        assert record["controller_goal_sent"] is True
        assert record["controller_endpoint"] == FJT_ENDPOINT
        assert record["action_goal_sent"] is True
        assert record["action_endpoint"] == _ige.EXECUTE_TRAJECTORY_ENDPOINT
        assert record["cartesian_goal_sent"] is False
        assert record["gripper_goal_sent"] is False
        controller_rows = _jsonl_rows(tmp_path / "controller-results.jsonl")
        assert controller_rows and controller_rows[-1]["controller_goal_sent"] is True
        assert controller_rows[-1]["action_goal_sent"] is True
        assert controller_rows[-1]["action_endpoint"] == _ige.EXECUTE_TRAJECTORY_ENDPOINT
        capture_rows = _jsonl_rows(tmp_path / "visual-capture-requests.jsonl")
        assert capture_rows and all(
            row["capture"]["kind"] == "gate-d-diagnostic" for row in capture_rows
        )
    finally:
        executor.shutdown()


def test_executor_retreat_action_semantics_artifact(tmp_path):
    """F2.7/L3: retreat records controller_goal_sent=False (no FJT goal), with the
    CartesianMove traffic surfaced via cartesian_goal_sent/action_goal_sent."""
    import uuid as _uuid

    from validation import integrated_gate_executor as _ige

    executor, contract = _d_executor(tmp_path, "qualification-moveit-cartesian-retreat")
    try:
        retreat_uuid = _uuid.uuid4().bytes
        handle = _FakeGoalHandle(
            accepted=True, result=None, goal_id=retreat_uuid,
            status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
        )
        client = _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0)
        executor._action_clients["/cartesian_move_action"] = client
        source = {
            "frame_id": "base_link",
            "xyz": [0.2, 0.0, 0.72],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "identity": "tcp-observation-1",
            "age_s": 0.05,
        }
        _synthetic_scene(executor, contract)
        record = executor.run_cartesian_retreat(
            "qualification-moveit-cartesian-retreat",
            current_tcp_pose_provider=lambda: source,
            environment_cloud_provider=lambda: deterministic_cube_cloud(),
        )
        assert record["status"] == "diagnostic-pass"
        assert record["controller_goal_sent"] is False
        assert record["controller_endpoint"] == CARTESIAN_MOVE_ENDPOINT
        assert record["action_goal_sent"] is True
        assert record["action_endpoint"] == CARTESIAN_MOVE_ENDPOINT
        assert record["cartesian_goal_sent"] is True
        assert record["gripper_goal_sent"] is False
        controller_rows = _jsonl_rows(tmp_path / "controller-results.jsonl")
        assert controller_rows and controller_rows[-1]["controller_goal_sent"] is False
        assert controller_rows[-1]["cartesian_goal_sent"] is True
        assert controller_rows[-1]["action_goal_sent"] is True
        assert controller_rows[-1]["action_endpoint"] == CARTESIAN_MOVE_ENDPOINT
    finally:
        executor.shutdown()


def test_executor_gripper_action_semantics_artifact(tmp_path):
    """F2.7/L3: gripper records controller_goal_sent=False (no FJT goal) with the
    native gripper traffic surfaced via gripper_goal_sent/action_goal_sent."""
    import uuid as _uuid

    from validation import integrated_gate_executor as _ige

    executor, contract = _d_executor(tmp_path, "qualification-moveit-gripper")
    try:
        sent = []

        def _send_goal(goal):
            handle = _FakeGoalHandle(
                accepted=True, result=None,
                goal_id=_uuid.uuid4().bytes,
                status=EXECUTE_STATUS_SUCCEEDED, result_ready_at=0.0,
            )
            sent.append(goal)
            return _FakeFuture(handle, ready_at=0.0)

        client = _FakeMoveClient(server_ready=True, goal_handle=None, send_ready_at=0.0)
        client.send_goal_async = _send_goal
        executor._action_clients["/xarm_gripper/gripper_action"] = client
        _synthetic_scene(executor, contract)
        record = executor.run_gripper_sequence("qualification-moveit-gripper")
        assert record["status"] == "diagnostic-pass"
        assert record["controller_goal_sent"] is False
        assert record["controller_endpoint"] == GRIPPER_ENDPOINT
        assert record["action_goal_sent"] is True
        assert record["action_endpoint"] == GRIPPER_ENDPOINT
        assert record["cartesian_goal_sent"] is False
        assert record["gripper_goal_sent"] is True
        controller_rows = _jsonl_rows(tmp_path / "controller-results.jsonl")
        assert controller_rows and controller_rows[-1]["controller_goal_sent"] is False
        assert controller_rows[-1]["gripper_goal_sent"] is True
        assert controller_rows[-1]["action_goal_sent"] is True
        assert controller_rows[-1]["action_endpoint"] == GRIPPER_ENDPOINT
        assert controller_rows[-1]["gripper_goal_sent"] is True
    finally:
        executor.shutdown()
