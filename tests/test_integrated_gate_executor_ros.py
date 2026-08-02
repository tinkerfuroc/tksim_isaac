"""Task 4 (Humble): real generated-message / geometry executor tests.

Runs under sourced ROS Humble Python 3.10 with the simulator
``validation``/``tests`` directories on ``PYTHONPATH``:

.. code-block:: bash

    source /opt/ros/humble/setup.zsh
    source /home/tinker/tk25_ws/install/setup.zsh
    PYTHONPATH=/home/tinker/tinker-sim/6.0.1/validation:/home/tinker/tinker-sim/6.0.1/tests \\
      python3 -m pytest -q /home/tinker/tinker-sim/6.0.1/tests/test_integrated_gate_executor_ros.py

Covers the real ``MoveGroup`` joint goal fields, Pick/Place goal field
construction and their fail-closed validation, the deterministic
``PointCloud2`` cube geometry, and real-shape multi-operation report bytes fed
through the readiness evaluator.
"""
from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="requires Humble ROS Python runtime")
pytest.importorskip("moveit_msgs", reason="requires Humble moveit_msgs")
pytest.importorskip("sensor_msgs", reason="requires Humble sensor_msgs")
pytest.importorskip("sensor_msgs_py", reason="requires Humble sensor_msgs_py")
pytest.importorskip("geometry_msgs", reason="requires Humble geometry_msgs")
pytest.importorskip("tinker_arm_msgs", reason="requires Humble tinker_arm_msgs")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "tests"))

from tinker_sim_bridge.integrated_readiness import (  # noqa: E402
    build_canonical_report,
    public_integrated_mapping,
)
from validation.integrated_gate_executor import (  # noqa: E402
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
