"""Task 7: deterministic OMPL plan-only smoke contract tests.

Runs under simulator CPython 3.12.  Exercises the ROS-free
``validation/ompl_goal_builders`` and ``validation/ompl_plan_smoke`` pure
seams: deterministic plain-data goal builders, the fail-closed smoke
evaluator, deterministic compact report serialization, atomic report writes,
mode/scenario consistency against the real qualification scenarios, and the
ROS-free import boundary.  The live Humble client behavior is verified
separately (bounded fail-closed invocation) because the integrated overlay is
not required to be running for these pure tests.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from ompl_goal_builders import (  # noqa: E402
    JOINT_TARGET_POSITIONS,
    POSE_APPROACH_Z_OFFSET,
    PIPELINE_ID,
    PLAN_ONLY,
    JointGoal,
    PoseGoal,
    build_joint_goal,
    build_pose_goal,
    goal_kind,
    goal_to_dict,
)
from ompl_plan_smoke import (  # noqa: E402
    COMMAND_TOPIC,
    MOVE_ACTION,
    MOVE_ACTION_KIND,
    MOVE_ACTION_SOURCE,
    MOVE_ACTION_TYPE,
    READINESS_TOPIC,
    READINESS_STATE_PASS,
    SUCCESS_ERROR_CODE,
    build_goal,
    build_report,
    canonical_json,
    derive_goal_service_type,
    evaluate_smoke,
    fail_closed_report,
    load_scenario,
    scenario_qualification_gate,
    serialize_report,
    sha256_json,
    write_report_atomic,
)

JOINT_SCENARIO = ROOT / "simulation/scenarios/qualification-moveit-plan-joint.json"
POSE_SCENARIO = ROOT / "simulation/scenarios/qualification-moveit-plan-pose.json"
BLOCKED_SCENARIO = ROOT / "simulation/scenarios/qualification-moveit-plan-blocked.json"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _endpoint() -> dict[str, object]:
    return {
        "action": MOVE_ACTION,
        "count": 1,
        "source": MOVE_ACTION_SOURCE,
        "sources": [MOVE_ACTION_SOURCE],
        "observed_types": ["moveit_msgs/action/MoveGroup_SendGoal"],
        "kind": MOVE_ACTION_KIND,
        "result_service_present": True,
        "result_service_types": ["moveit_msgs/action/MoveGroup_GetResult"],
    }


def _command_observations() -> dict[str, object]:
    return {
        "topic": COMMAND_TOPIC,
        "samples": 0,
        "window_start_s": 10.0,
        "window_end_s": 11.0,
    }


def _outcome(mode: str) -> dict[str, object]:
    if mode in ("joint", "pose"):
        return {
            "kind": "success",
            "error_code": SUCCESS_ERROR_CODE,
            "detail": "",
            "trajectory_point_count": 12,
            "nonempty": True,
            "planning_time": 0.5,
        }
    return {
        "kind": "non_success",
        "error_code": -1,
        "detail": "",
        "trajectory_point_count": 0,
        "nonempty": False,
        "planning_time": 0.1,
    }


def ready_snapshot(mode: str = "joint") -> dict[str, object]:
    """Return a complete observation snapshot that evaluates ready for *mode*."""
    return {
        "readiness": {"state": READINESS_STATE_PASS, "received_at_s": 100.0, "now_s": 100.2},
        "endpoint": _endpoint(),
        "command_observations": _command_observations(),
        "outcome": _outcome(mode),
    }


def expected_for(mode: str) -> dict[str, object]:
    return {
        "mode": mode,
        "action_type": MOVE_ACTION_TYPE,
        "source": MOVE_ACTION_SOURCE,
        "readiness_max_age_s": 1.0,
        "success_error_code": SUCCESS_ERROR_CODE,
        "trajectory_min_points": 1,
    }


# ---------------------------------------------------------------------------
# Goal builders
# ---------------------------------------------------------------------------


def test_build_joint_goal_good_defaults() -> None:
    goal = build_joint_goal()
    assert isinstance(goal, JointGoal)
    assert goal.group_name == "xarm7"
    assert goal.pipeline_id == "ompl"
    assert goal.plan_only is True
    assert set(goal.joint_positions) == set(JOINT_TARGET_POSITIONS)
    for value in goal.joint_positions.values():
        assert math.isfinite(value)


def test_build_joint_goal_explicit_positions() -> None:
    goal = build_joint_goal({"joint1": 0.1, "joint2": 0.2})
    assert goal.joint_positions == {"joint1": 0.1, "joint2": 0.2}
    assert goal.tolerances == {"joint1": 0.02, "joint2": 0.02}


def test_build_joint_goal_rejects_empty() -> None:
    with pytest.raises(ValueError):
        build_joint_goal({})


def test_build_joint_goal_rejects_nonfinite() -> None:
    with pytest.raises(ValueError):
        build_joint_goal({"joint1": float("nan")})
    with pytest.raises(ValueError):
        build_joint_goal({"joint1": float("inf")})


def test_build_joint_goal_rejects_nonpositive_tolerance() -> None:
    with pytest.raises(ValueError):
        build_joint_goal({"joint1": 0.0}, tolerances={"joint1": 0.0})


def test_build_pose_goal_good_defaults() -> None:
    goal = build_pose_goal([0.55, 0.0, 0.95])
    assert isinstance(goal, PoseGoal)
    assert goal.group_name == "xarm7"
    assert goal.link_name == "link_tcp"
    assert goal.frame_id == "base_link"
    assert goal.pipeline_id == "ompl"
    assert goal.plan_only is True
    assert goal.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert goal.use_orientation is False


def test_build_pose_goal_normalizes_quaternion() -> None:
    goal = build_pose_goal([0.55, 0.0, 0.95], [0.0, 0.0, 0.0, 2.0])
    norm = math.sqrt(sum(v * v for v in goal.orientation_xyzw))
    assert norm == pytest.approx(1.0)


def test_build_pose_goal_rejects_nonfinite_position() -> None:
    with pytest.raises(ValueError):
        build_pose_goal([float("nan"), 0.0, 0.95])


def test_build_pose_goal_rejects_bad_length() -> None:
    with pytest.raises(ValueError):
        build_pose_goal([0.55, 0.0])


def test_build_pose_goal_rejects_zero_quaternion() -> None:
    with pytest.raises(ValueError):
        build_pose_goal([0.55, 0.0, 0.95], [0.0, 0.0, 0.0, 0.0])


def test_goal_kind_and_to_dict() -> None:
    joint = build_joint_goal()
    pose = build_pose_goal([0.55, 0.0, 0.95])
    assert goal_kind(joint) == "joint"
    assert goal_kind(pose) == "pose"
    joint_dict = goal_to_dict(joint)
    assert joint_dict["kind"] == "joint"
    assert joint_dict["pipeline_id"] == "ompl"
    assert joint_dict["plan_only"] is True
    assert goal_to_dict(joint) == goal_to_dict(joint)


# ---------------------------------------------------------------------------
# Mode/scenario consistency
# ---------------------------------------------------------------------------


def test_scenario_gates_match_modes() -> None:
    assert scenario_qualification_gate(load_scenario(JOINT_SCENARIO)) == "moveit-plan-joint"
    assert scenario_qualification_gate(load_scenario(POSE_SCENARIO)) == "moveit-plan-pose"
    assert scenario_qualification_gate(load_scenario(BLOCKED_SCENARIO)) == "moveit-plan-blocked"


def test_build_goal_joint_from_scenario() -> None:
    goal = build_goal("joint", load_scenario(JOINT_SCENARIO))
    assert isinstance(goal, JointGoal)
    assert goal.pipeline_id == PIPELINE_ID
    assert goal.plan_only is PLAN_ONLY


def test_build_goal_pose_from_scenario_applies_approach_offset() -> None:
    scenario = load_scenario(POSE_SCENARIO)
    goal = build_goal("pose", scenario)
    assert isinstance(goal, PoseGoal)
    target = scenario_target_xyz(scenario)
    assert goal.position_xyz[0] == pytest.approx(target[0])
    assert goal.position_xyz[1] == pytest.approx(target[1])
    assert goal.position_xyz[2] == pytest.approx(target[2] + POSE_APPROACH_Z_OFFSET)


def test_build_goal_blocked_targets_blocker_interior() -> None:
    scenario = load_scenario(BLOCKED_SCENARIO)
    goal = build_goal("blocked", scenario)
    assert isinstance(goal, PoseGoal)
    blocker_xyz = scenario_blocker_xyz(scenario)
    assert goal.position_xyz == tuple(blocker_xyz)


def test_build_goal_joint_ignores_pose_only_overrides() -> None:
    goal = build_goal(
        "joint", load_scenario(JOINT_SCENARIO), position_tolerance=0.05
    )
    assert isinstance(goal, JointGoal)
    assert goal.pipeline_id == "ompl"


def test_build_goal_rejects_mode_scenario_mismatch() -> None:
    with pytest.raises(ValueError):
        build_goal("pose", load_scenario(JOINT_SCENARIO))
    with pytest.raises(ValueError):
        build_goal("joint", load_scenario(BLOCKED_SCENARIO))


def scenario_target_xyz(scenario: dict[str, object]) -> list[float]:
    objects = (scenario.get("planning_scene") or {}).get("objects") or []
    for obj in objects:
        if obj.get("class") == "target":
            return [float(v) for v in obj["pose"]["xyz"]]
    raise AssertionError("no target object")


def scenario_blocker_xyz(scenario: dict[str, object]) -> list[float]:
    objects = (scenario.get("planning_scene") or {}).get("objects") or []
    for obj in objects:
        if obj.get("class") == "blocker":
            return [float(v) for v in obj["pose"]["xyz"]]
    raise AssertionError("no blocker object")


# ---------------------------------------------------------------------------
# Endpoint type derivation
# ---------------------------------------------------------------------------


def test_derive_goal_service_type() -> None:
    assert derive_goal_service_type("moveit_msgs/action/MoveGroup") == (
        "moveit_msgs/action/MoveGroup_SendGoal"
    )
    assert derive_goal_service_type("no-slash") == ""


def test_action_name_from_type() -> None:
    from ompl_plan_smoke import _action_name_from_type

    assert _action_name_from_type("moveit_msgs/action/MoveGroup_SendGoal") == "MoveGroup"
    assert _action_name_from_type("moveit_msgs/action/MoveGroup_GetResult") == "MoveGroup"
    assert _action_name_from_type("moveit_msgs/action/ExecuteTrajectory_SendGoal") == "ExecuteTrajectory"
    assert _action_name_from_type("") == ""


# ---------------------------------------------------------------------------
# Evaluator: good paths
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("mode", ["joint", "pose"])
def test_evaluate_good_joint_and_pose(mode: str) -> None:
    ready, reasons = evaluate_smoke(ready_snapshot(mode), expected_for(mode))
    assert ready, reasons
    assert reasons == []


def test_evaluate_good_blocked() -> None:
    ready, reasons = evaluate_smoke(ready_snapshot("blocked"), expected_for("blocked"))
    assert ready, reasons
    assert reasons == []


# ---------------------------------------------------------------------------
# Evaluator: fail-closed paths
# ---------------------------------------------------------------------------


def test_evaluate_blocked_rejects_unexpected_success() -> None:
    snapshot = ready_snapshot("blocked")
    snapshot["outcome"] = _outcome("joint")
    ready, reasons = evaluate_smoke(snapshot, expected_for("blocked"))
    assert not ready
    assert any("non-success" in reason for reason in reasons)


def test_evaluate_rejects_empty_successful_plan() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["outcome"] = {
        "kind": "success",
        "error_code": SUCCESS_ERROR_CODE,
        "detail": "",
        "trajectory_point_count": 0,
        "nonempty": False,
        "planning_time": 0.4,
    }
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("empty trajectory" in reason for reason in reasons)


def test_evaluate_rejects_readiness_not_pass() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"] = {"state": "fail", "received_at_s": 100.0, "now_s": 100.2}
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("readiness state" in reason for reason in reasons)


def test_evaluate_rejects_readiness_stale() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"] = {"state": "pass", "received_at_s": 100.0, "now_s": 102.0}
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("stale" in reason for reason in reasons)


def test_evaluate_rejects_command_sample() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"] = {
        "topic": COMMAND_TOPIC,
        "samples": 1,
        "window_start_s": 10.0,
        "window_end_s": 11.0,
    }
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("command sample" in reason for reason in reasons)


def test_evaluate_rejects_wrong_endpoint_count() -> None:
    snapshot = ready_snapshot("joint")
    endpoint = _endpoint()
    endpoint["count"] = 2
    snapshot["endpoint"] = endpoint
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("server count" in reason for reason in reasons)


def test_evaluate_rejects_wrong_endpoint_type() -> None:
    snapshot = ready_snapshot("joint")
    endpoint = _endpoint()
    endpoint["observed_types"] = ["tinker_arm_msgs/action/Pick_SendGoal"]
    endpoint["kind"] = "Pick"
    snapshot["endpoint"] = endpoint
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("observed goal-service types" in reason for reason in reasons)


def test_evaluate_rejects_wrong_endpoint_kind() -> None:
    snapshot = ready_snapshot("joint")
    endpoint = _endpoint()
    endpoint["kind"] = "ExecuteTrajectory"
    snapshot["endpoint"] = endpoint
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("kind" in reason for reason in reasons)


def test_evaluate_rejects_wrong_endpoint_source() -> None:
    snapshot = ready_snapshot("joint")
    endpoint = _endpoint()
    endpoint["source"] = "/someone_else"
    snapshot["endpoint"] = endpoint
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("source" in reason for reason in reasons)


def test_evaluate_rejects_missing_result_service() -> None:
    snapshot = ready_snapshot("joint")
    endpoint = _endpoint()
    endpoint["result_service_present"] = False
    snapshot["endpoint"] = endpoint
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("result service" in reason for reason in reasons)


@pytest.mark.parametrize("mode", ["joint", "pose", "blocked"])
@pytest.mark.parametrize("bad_kind", ["timeout", "cancelled", "invalid"])
def test_evaluate_rejects_timeout_cancelled_invalid(mode: str, bad_kind: str) -> None:
    snapshot = ready_snapshot(mode)
    snapshot["outcome"] = {
        "kind": bad_kind,
        "error_code": None,
        "detail": "bounded path",
        "trajectory_point_count": 0,
        "nonempty": False,
    }
    ready, reasons = evaluate_smoke(snapshot, expected_for(mode))
    assert not ready
    if mode == "blocked":
        assert any("non-success" in reason for reason in reasons)
    else:
        assert any("successful plan" in reason for reason in reasons)


def test_evaluate_rejects_unknown_mode() -> None:
    ready, reasons = evaluate_smoke(ready_snapshot("joint"), {"mode": "bogus"})
    assert not ready
    assert any("mode" in reason for reason in reasons)


# ---------------------------------------------------------------------------
# Report serialization
# ---------------------------------------------------------------------------


def test_report_bytes_deterministic() -> None:
    snapshot = ready_snapshot("joint")
    meta = {
        "mode": "joint",
        "scenario": "qualification-moveit-plan-joint",
        "goal": {"kind": "joint", "pipeline_id": "ompl"},
        "pipeline_id": "ompl",
        "plan_only": True,
        "meta": {"python": "3.12.13", "domain_id": "25"},
    }
    report = build_report(snapshot, evaluate_smoke(snapshot, expected_for("joint")), meta)
    first = serialize_report(report)
    second = serialize_report(report)
    assert first == second
    assert sha256_json(report) == sha256_json(json.loads(first.decode("utf-8")))


def test_report_is_compact_canonical_json() -> None:
    snapshot = ready_snapshot("joint")
    meta = {
        "mode": "joint",
        "scenario": "qualification-moveit-plan-joint",
        "goal": {},
        "pipeline_id": "ompl",
        "plan_only": True,
        "meta": {},
    }
    report = build_report(snapshot, evaluate_smoke(snapshot, expected_for("joint")), meta)
    data = serialize_report(report)
    assert b' " ' not in data  # no spaces after separators
    assert b'"schema_version":1' in data
    parsed = json.loads(data.decode("utf-8"))
    assert parsed["evaluation"]["ready"] is True
    assert parsed["goal"] == {}
    assert parsed["readiness"]["state"] == READINESS_STATE_PASS
    assert parsed["command_observations"]["samples"] == 0
    assert parsed["endpoint"]["kind"] == MOVE_ACTION_KIND


def test_report_bytes_include_goal_and_mode() -> None:
    goal = build_joint_goal()
    snapshot = ready_snapshot("joint")
    meta = {
        "mode": "joint",
        "scenario": "qualification-moveit-plan-joint",
        "goal": goal_to_dict(goal),
        "pipeline_id": "ompl",
        "plan_only": True,
        "meta": {},
    }
    report = build_report(snapshot, evaluate_smoke(snapshot, expected_for("joint")), meta)
    assert report["goal"]["pipeline_id"] == "ompl"
    assert report["goal"]["plan_only"] is True
    assert report["goal"]["kind"] == "joint"


def test_fail_closed_report() -> None:
    report = fail_closed_report(
        mode="joint",
        scenario="qualification-moveit-plan-joint",
        blocker="integrated readiness did not reach pass within 5.0 s",
        meta={"python": "3.10.12"},
    )
    data = serialize_report(report)
    parsed = json.loads(data.decode("utf-8"))
    assert parsed["evaluation"]["ready"] is False
    assert any("blocked" in reason for reason in parsed["evaluation"]["reasons"])
    assert parsed["blocker"].startswith("integrated readiness")


def test_write_report_atomic(tmp_path) -> None:
    target = tmp_path / "nested" / "report.json"
    data = b'{"schema_version":1}'
    written = write_report_atomic(target, data)
    assert written == target
    assert target.read_bytes() == data
    assert not list(target.parent.glob("*.tmp"))


# ---------------------------------------------------------------------------
# ROS-free import boundary + readiness-contract consistency
# ---------------------------------------------------------------------------


def test_ros_free_imports() -> None:
    assert "rclpy" not in sys.modules
    assert "moveit_msgs" not in sys.modules
    import ompl_goal_builders  # noqa: F401
    import ompl_plan_smoke  # noqa: F401

    assert "rclpy" not in sys.modules
    assert "moveit_msgs" not in sys.modules
    assert "sensor_msgs" not in sys.modules
    assert "std_msgs" not in sys.modules


def test_move_action_spec_matches_readiness_contract() -> None:
    from tinker_sim_bridge.integrated_readiness import INTEGRATED_ACTIONS

    spec = INTEGRATED_ACTIONS[MOVE_ACTION]
    assert spec["type"] == MOVE_ACTION_TYPE
    assert spec["source"] == MOVE_ACTION_SOURCE
