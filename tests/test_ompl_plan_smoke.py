"""Task 7 (fix round 1): deterministic OMPL plan-only smoke contract tests.

Runs under simulator CPython 3.12.  Exercises the ROS-free
``validation/ompl_goal_builders`` and ``validation/ompl_plan_smoke`` pure
seams: deterministic plain-data goal builders, the fail-closed smoke
evaluator (readiness publisher/canonical payload/identity, command
publisher proof + zero-command window, exact endpoint + result-service type,
outcome adjudication), deterministic
compact report serialization, atomic report writes, mode/scenario consistency
against the real qualification scenarios, and the ROS-free import boundary.

The live Humble client behavior is verified separately: a bounded fail-closed
invocation plus the real-Humble suite at ``tests/ros_humble/test_ompl_plan_smoke.py``.
"""
from __future__ import annotations

import json
import math
import os
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
    COMMAND_TYPE,
    COMMAND_SOURCE,
    MOVE_ACTION,
    MOVE_ACTION_KIND,
    MOVE_ACTION_SOURCE,
    MOVE_ACTION_TYPE,
    READINESS_TOPIC,
    READINESS_TYPE,
    READINESS_SOURCE,
    READINESS_STATE_PASS,
    STATUS_ABORTED,
    STATUS_SUCCEEDED,
    SUCCESS_ERROR_CODE,
    build_goal,
    build_report,
    canonical_json,
    derive_goal_service_type,
    derive_result_service_type,
    evaluate_smoke,
    fail_closed_report,
    load_scenario,
    moveit_error_code_name,
    parse_readiness_payload,
    readiness_identity_reasons,
    scenario_expected_identities,
    scenario_qualification_gate,
    serialize_report,
    sha256_json,
    write_report_atomic,
)

JOINT_SCENARIO = ROOT / "simulation/scenarios/qualification-moveit-plan-joint.json"
POSE_SCENARIO = ROOT / "simulation/scenarios/qualification-moveit-plan-pose.json"

SCENARIOS = {
    "joint": JOINT_SCENARIO,
    "pose": POSE_SCENARIO,
}

GOAL_SERVICE_TYPE = "moveit_msgs/action/MoveGroup_SendGoal"
RESULT_SERVICE_TYPE = "moveit_msgs/action/MoveGroup_GetResult"


# ---------------------------------------------------------------------------
# Observation snapshot helpers (mirror the live client's report schema)
# ---------------------------------------------------------------------------


def _publisher(
    *,
    count: int = 1,
    source: str = COMMAND_SOURCE,
    type_: str = COMMAND_TYPE,
    reliability: str = "RELIABLE",
    durability: str = "VOLATILE",
    depth: int = 50,
    expected_depth: int | None = 50,
    settled: bool = True,
) -> dict[str, object]:
    return {
        "topic": COMMAND_TOPIC,
        "count": count,
        "source": source,
        "sources": [source] if count == 1 else sorted([source]),
        "types": [type_] if type_ else [],
        "type": type_,
        "qos": {
            "reliability": reliability,
            "durability": durability,
            "depth": depth,
        },
        "expected_depth": expected_depth,
        "settled": settled,
    }


def _readiness_publisher() -> dict[str, object]:
    pub = _publisher(
        source=READINESS_SOURCE,
        type_=READINESS_TYPE,
        durability="TRANSIENT_LOCAL",
        depth=1,
        expected_depth=1,
    )
    pub["topic"] = READINESS_TOPIC
    return pub


def _identity(
    mode: str, *, broken: tuple[str, ...] = ()
) -> dict[str, object]:
    """Real scenario-derived identity dict; *broken* flips ok=False fields."""
    scenario = load_scenario(SCENARIOS[mode])
    expected = scenario_expected_identities(scenario)
    observed = {
        "scenario_id": expected["scenario_id"],
        "seed": expected["seed"],
        "scenario_declaration_sha256": expected["scenario_declaration_sha256"],
        "planning_scene_sha256": expected["planning_scene_sha256"],
        "fixture_scenario": expected["scenario_id"],
        "fixture_revision": expected["planning_scene_revision"],
        "fixture_revision_digest": expected["planning_scene_revision_digest"],
        "fixture_owned_ids": list(expected["planning_scene_owned_ids"]),
        "fixture_target_source_id": expected["planning_scene_target_source_id"],
        "fixture_target_handoff": expected["planning_scene_target_handoff"],
        "integrated_sha256": None,
        "model_fingerprint": None,
        "provider_manifest_sha256": None,
    }
    return {
        field: {
            "ok": field not in broken,
            "observed": observed.get(field),
            "expected": expected.get(field),
        }
        for field in observed
    }


def _readiness(mode: str, *, gate_refreshed: bool = True) -> dict[str, object]:
    return {
        "valid": True,
        "state": READINESS_STATE_PASS,
        "ready": True,
        "schema_version": 1,
        "reasons": [],
        "published_at": 100.0,
        "received_at_s": 100.0,
        "now_s": 100.15,
        "producer_age_s": 0.15,
        "consumer_age_s": 0.15,
        "gate_refreshed": gate_refreshed,
        "gate_refreshed_at_s": 100.14,
        "publisher": _readiness_publisher(),
        "identity": _identity(mode),
        "counts": {"pass": 1, "fail": 0, "malformed": 0},
        "window_counts": {"pass": 1, "fail": 0, "malformed": 0, "identity_invalid": 0},
        "any_fail_in_window": False,
        "any_malformed_in_window": False,
        "any_identity_invalid_in_window": False,
        "publisher_graph_changed": False,
        "window": {"start_s": 100.1, "end_s": 100.15},
    }


def _endpoint() -> dict[str, object]:
    return {
        "action": MOVE_ACTION,
        "count": 1,
        "source": MOVE_ACTION_SOURCE,
        "sources": [MOVE_ACTION_SOURCE],
        "observed_types": [GOAL_SERVICE_TYPE],
        "kind": MOVE_ACTION_KIND,
        "result_service_present": True,
        "result_service_types": [RESULT_SERVICE_TYPE],
    }


def _command_observations() -> dict[str, object]:
    return {
        "topic": COMMAND_TOPIC,
        "samples": 0,
        "sample_events": [],
        "window_start_s": 100.1,
        "window_end_s": 101.0,
        "send_time_s": 100.5,
        "result_time_s": 100.9,
        "tail_time_s": 101.0,
        "tail_s": 0.25,
        "duration_s": 0.9,
        "publisher": _publisher(),
        "settled": True,
        "publisher_graph_changed": False,
    }


def _outcome(mode: str) -> dict[str, object]:
    return {
        "kind": "success",
        "detail": "",
        "goal_accepted": True,
        "terminal_status": STATUS_SUCCEEDED,
        "terminal_status_name": "STATUS_SUCCEEDED",
        "error_code": SUCCESS_ERROR_CODE,
        "error_code_name": "SUCCESS",
        "result_received": True,
        "trajectory_point_count": 12,
        "trajectory_joint_names": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
        "planning_time": 0.5,
        "cancel_requested": False,
        "cancel_accepted": False,
        "cancel_confirmed": False,
    }


def ready_snapshot(mode: str = "joint") -> dict[str, object]:
    """Return a complete observation snapshot that evaluates ready for *mode*."""
    return {
        "readiness": _readiness(mode),
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
        build_goal("joint", load_scenario(POSE_SCENARIO))


def scenario_target_xyz(scenario: dict[str, object]) -> list[float]:
    objects = (scenario.get("planning_scene") or {}).get("objects") or []
    for obj in objects:
        if obj.get("class") == "target":
            return [float(v) for v in obj["pose"]["xyz"]]
    raise AssertionError("no target object")


# ---------------------------------------------------------------------------
# Endpoint type derivation
# ---------------------------------------------------------------------------


def test_derive_goal_service_type() -> None:
    assert derive_goal_service_type("moveit_msgs/action/MoveGroup") == GOAL_SERVICE_TYPE
    assert derive_goal_service_type("no-slash") == ""


def test_derive_result_service_type() -> None:
    assert derive_result_service_type("moveit_msgs/action/MoveGroup") == RESULT_SERVICE_TYPE
    assert derive_result_service_type("no-slash") == ""


def test_action_name_from_type() -> None:
    from ompl_plan_smoke import _action_name_from_type

    assert _action_name_from_type(GOAL_SERVICE_TYPE) == "MoveGroup"
    assert _action_name_from_type(RESULT_SERVICE_TYPE) == "MoveGroup"
    assert _action_name_from_type("moveit_msgs/action/ExecuteTrajectory_SendGoal") == "ExecuteTrajectory"
    assert _action_name_from_type("") == ""


def test_derive_action_kind_from_contract() -> None:
    from ompl_plan_smoke import derive_action_kind

    assert derive_action_kind(MOVE_ACTION_TYPE) == MOVE_ACTION_KIND
    # The contract and the hand-stamped kind agree by construction.
    assert derive_action_kind("nav2_msgs/action/FollowWaypoints") == "FollowWaypoints"


# ---------------------------------------------------------------------------
# Readiness payload parsing (canonical schema)
# ---------------------------------------------------------------------------


def _payload(**overrides: object) -> dict[str, object]:
    payload = {
        "schema_version": 1,
        "state": "pass",
        "ready": True,
        "reasons": [],
        "published_at": 100.0,
        "evidence": {
            "shared_report": {
                "ready": True,
                "identities": {
                    "scenario_id": "qualification-moveit-plan-joint",
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
                    "schema_version": 1,
                    "state": "FIXTURE_READY",
                    "scenario": "qualification-moveit-plan-joint",
                    "owner": "fixture_planning_scene",
                    "revision": "2026-08-01-moveit-qualification-joint",
                    "revision_digest": "e" * 64,
                    "sequence": 3,
                    "published_at": 99.0,
                    "owned_ids": ["sim_fixture/pedestal", "sim_fixture/public_target"],
                    "target_source_id": "sim_fixture/public_target",
                    "target_handoff": "pick_and_place/object_mesh",
                },
            },
        },
    }
    payload.update(overrides)
    return payload


def test_parse_readiness_payload_canonical_pass() -> None:
    result = parse_readiness_payload(json.dumps(_payload()))
    assert result["valid"] is True
    assert result["reason"] == ""


def test_parse_readiness_payload_rejects_malformed_json() -> None:
    result = parse_readiness_payload("not json {{{")
    assert result["valid"] is False
    assert "malformed" in result["reason"]


def test_parse_readiness_payload_rejects_wrong_schema_version() -> None:
    result = parse_readiness_payload(json.dumps(_payload(schema_version=2)))
    assert result["valid"] is False
    assert "schema_version" in result["reason"]


def test_parse_readiness_payload_rejects_pass_with_ready_false() -> None:
    result = parse_readiness_payload(json.dumps(_payload(ready=False)))
    assert result["valid"] is False
    assert "ready" in result["reason"]


def test_parse_readiness_payload_rejects_nonempty_reasons_on_pass() -> None:
    result = parse_readiness_payload(json.dumps(_payload(reasons=["boom"])))
    assert result["valid"] is False
    assert "reasons" in result["reason"]


def test_parse_readiness_payload_rejects_nonfinite_published_at() -> None:
    result = parse_readiness_payload(json.dumps(_payload(published_at=float("nan"))))
    assert result["valid"] is False
    assert "published_at" in result["reason"]


def test_parse_readiness_payload_rejects_fail_state_with_ready_true() -> None:
    result = parse_readiness_payload(json.dumps(_payload(state="fail", ready=True)))
    assert result["valid"] is False
    assert "ready" in result["reason"]


# ---------------------------------------------------------------------------
# Identity agreement (fail-closed)
# ---------------------------------------------------------------------------


def _real_payload(mode: str) -> str:
    scenario = load_scenario(SCENARIOS[mode])
    expected = scenario_expected_identities(scenario)
    payload = _payload()
    payload["evidence"]["shared_report"]["identities"] = {
        "scenario_id": expected["scenario_id"],
        "seed": expected["seed"],
        "scenario_declaration_sha256": expected["scenario_declaration_sha256"],
        "planning_scene_sha256": expected["planning_scene_sha256"],
        "integrated_sha256": "i" * 64,
        "model_fingerprint": "fp",
        "provider_manifest_sha256": "p" * 64,
    }
    payload["evidence"]["fixture_status"]["status"] = {
        "scenario": expected["scenario_id"],
        "revision": expected["planning_scene_revision"],
        "revision_digest": expected["planning_scene_revision_digest"],
        "sequence": 3,
        "published_at": 99.0,
        "owned_ids": list(expected["planning_scene_owned_ids"]),
        "target_source_id": expected["planning_scene_target_source_id"],
        "target_handoff": expected["planning_scene_target_handoff"],
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def test_identity_agreement_ok() -> None:
    scenario = load_scenario(JOINT_SCENARIO)
    expected = scenario_expected_identities(scenario)
    parsed = parse_readiness_payload(_real_payload("joint"))
    assert parsed["valid"] is True
    reasons = readiness_identity_reasons(parsed["payload"], expected)
    assert reasons == []


def test_identity_agreement_wrong_scenario_id() -> None:
    scenario = load_scenario(JOINT_SCENARIO)
    expected = scenario_expected_identities(scenario)
    parsed = parse_readiness_payload(_real_payload("joint"))
    parsed["payload"]["evidence"]["shared_report"]["identities"]["scenario_id"] = "qualification-moveit-plan-pose"
    reasons = readiness_identity_reasons(parsed["payload"], expected)
    assert any("scenario_id" in reason for reason in reasons)


def test_identity_agreement_wrong_seed() -> None:
    scenario = load_scenario(JOINT_SCENARIO)
    expected = scenario_expected_identities(scenario)
    parsed = parse_readiness_payload(_real_payload("joint"))
    parsed["payload"]["evidence"]["shared_report"]["identities"]["seed"] = 99
    reasons = readiness_identity_reasons(parsed["payload"], expected)
    assert any("seed" in reason for reason in reasons)


def test_identity_agreement_wrong_scenario_digest() -> None:
    scenario = load_scenario(JOINT_SCENARIO)
    expected = scenario_expected_identities(scenario)
    parsed = parse_readiness_payload(_real_payload("joint"))
    parsed["payload"]["evidence"]["shared_report"]["identities"]["scenario_declaration_sha256"] = "f" * 64
    reasons = readiness_identity_reasons(parsed["payload"], expected)
    assert any("scenario_declaration_sha256" in reason for reason in reasons)


def test_identity_agreement_wrong_planning_scene_digest() -> None:
    scenario = load_scenario(JOINT_SCENARIO)
    expected = scenario_expected_identities(scenario)
    parsed = parse_readiness_payload(_real_payload("joint"))
    parsed["payload"]["evidence"]["shared_report"]["identities"]["planning_scene_sha256"] = "f" * 64
    reasons = readiness_identity_reasons(parsed["payload"], expected)
    assert any("planning_scene_sha256" in reason for reason in reasons)


def test_identity_agreement_wrong_fixture_revision() -> None:
    scenario = load_scenario(JOINT_SCENARIO)
    expected = scenario_expected_identities(scenario)
    parsed = parse_readiness_payload(_real_payload("joint"))
    parsed["payload"]["evidence"]["fixture_status"]["status"]["revision"] = "wrong-revision"
    reasons = readiness_identity_reasons(parsed["payload"], expected)
    assert any("fixture revision" in reason for reason in reasons)


def test_identity_agreement_wrong_fixture_revision_digest() -> None:
    scenario = load_scenario(JOINT_SCENARIO)
    expected = scenario_expected_identities(scenario)
    parsed = parse_readiness_payload(_real_payload("joint"))
    parsed["payload"]["evidence"]["fixture_status"]["status"]["revision_digest"] = "f" * 64
    reasons = readiness_identity_reasons(parsed["payload"], expected)
    assert any("revision_digest" in reason for reason in reasons)


def test_identity_agreement_wrong_owned_ids() -> None:
    scenario = load_scenario(JOINT_SCENARIO)
    expected = scenario_expected_identities(scenario)
    parsed = parse_readiness_payload(_real_payload("joint"))
    parsed["payload"]["evidence"]["fixture_status"]["status"]["owned_ids"] = ["sim_fixture/other"]
    reasons = readiness_identity_reasons(parsed["payload"], expected)
    assert any("owned_ids" in reason for reason in reasons)


def test_identity_agreement_wrong_target_source() -> None:
    scenario = load_scenario(JOINT_SCENARIO)
    expected = scenario_expected_identities(scenario)
    parsed = parse_readiness_payload(_real_payload("joint"))
    parsed["payload"]["evidence"]["fixture_status"]["status"]["target_source_id"] = "sim_fixture/other"
    reasons = readiness_identity_reasons(parsed["payload"], expected)
    assert any("target_source_id" in reason for reason in reasons)


def test_identity_agreement_wrong_target_handoff() -> None:
    scenario = load_scenario(JOINT_SCENARIO)
    expected = scenario_expected_identities(scenario)
    parsed = parse_readiness_payload(_real_payload("joint"))
    parsed["payload"]["evidence"]["fixture_status"]["status"]["target_handoff"] = "wrong/handoff"
    reasons = readiness_identity_reasons(parsed["payload"], expected)
    assert any("target_handoff" in reason for reason in reasons)


# ---------------------------------------------------------------------------
# Evaluator: good paths
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("mode", ["joint", "pose"])
def test_evaluate_good_joint_and_pose(mode: str) -> None:
    ready, reasons = evaluate_smoke(ready_snapshot(mode), expected_for(mode))
    assert ready, reasons
    assert reasons == []


# ---------------------------------------------------------------------------
# Evaluator: readiness mutations
# ---------------------------------------------------------------------------


def test_evaluate_rejects_readiness_not_valid() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["valid"] = False
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("valid canonical pass" in reason for reason in reasons)


def test_evaluate_rejects_readiness_not_pass() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["state"] = "fail"
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("readiness state" in reason for reason in reasons)


def test_evaluate_rejects_readiness_ready_flag_false() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["ready"] = False
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("ready flag" in reason for reason in reasons)


def test_evaluate_rejects_readiness_wrong_schema_version() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["schema_version"] = 2
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("schema_version" in reason for reason in reasons)


def test_evaluate_rejects_readiness_with_reasons() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["reasons"] = ["boom"]
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("reasons" in reason for reason in reasons)


def test_evaluate_rejects_readiness_stale_consumer() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["now_s"] = 102.0
    snapshot["readiness"]["consumer_age_s"] = 2.0
    snapshot["readiness"]["producer_age_s"] = 2.0
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("consumer age" in reason for reason in reasons)


def test_evaluate_rejects_readiness_stale_producer() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["now_s"] = 102.0
    snapshot["readiness"]["producer_age_s"] = 2.0
    snapshot["readiness"]["consumer_age_s"] = 0.1
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("producer age" in reason for reason in reasons)


def test_evaluate_rejects_readiness_not_gate_refreshed() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["gate_refreshed"] = False
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("gate" in reason for reason in reasons)


def test_evaluate_rejects_readiness_fail_during_window() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["any_fail_in_window"] = True
    snapshot["readiness"]["window_counts"] = {"pass": 1, "fail": 1, "malformed": 0}
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("fail" in reason for reason in reasons)


def test_evaluate_rejects_readiness_malformed_during_window() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["any_malformed_in_window"] = True
    snapshot["readiness"]["window_counts"] = {"pass": 1, "fail": 0, "malformed": 1}
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("malformed" in reason for reason in reasons)


def test_evaluate_rejects_readiness_publisher_graph_changed() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["publisher_graph_changed"] = True
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("publisher graph" in reason for reason in reasons)


# ---------------------------------------------------------------------------
# Evaluator: readiness publisher metadata mutations
# ---------------------------------------------------------------------------


def test_evaluate_rejects_readiness_publisher_wrong_type() -> None:
    snapshot = ready_snapshot("joint")
    pub = _readiness_publisher()
    pub["type"] = "std_msgs/msg/UInt8MultiArray"
    pub["types"] = ["std_msgs/msg/UInt8MultiArray"]
    snapshot["readiness"]["publisher"] = pub
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("publisher type" in reason for reason in reasons)


def test_evaluate_rejects_readiness_publisher_wrong_source() -> None:
    snapshot = ready_snapshot("joint")
    pub = _readiness_publisher()
    pub["source"] = "/rogue_node"
    pub["sources"] = ["/rogue_node"]
    snapshot["readiness"]["publisher"] = pub
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("publisher source" in reason for reason in reasons)


def test_evaluate_rejects_readiness_publisher_wrong_count() -> None:
    snapshot = ready_snapshot("joint")
    pub = _readiness_publisher()
    pub["count"] = 2
    snapshot["readiness"]["publisher"] = pub
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("publisher count" in reason for reason in reasons)


def test_evaluate_rejects_readiness_publisher_wrong_durability() -> None:
    snapshot = ready_snapshot("joint")
    pub = _readiness_publisher()
    pub["qos"]["durability"] = "VOLATILE"
    snapshot["readiness"]["publisher"] = pub
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("durability" in reason for reason in reasons)


def test_evaluate_rejects_readiness_publisher_wrong_reliability() -> None:
    snapshot = ready_snapshot("joint")
    pub = _readiness_publisher()
    pub["qos"]["reliability"] = "BEST_EFFORT"
    snapshot["readiness"]["publisher"] = pub
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("reliability" in reason for reason in reasons)


def test_evaluate_rejects_readiness_identity_broken_field() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["identity"] = _identity("joint", broken=("scenario_id",))
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("scenario_id" in reason for reason in reasons)


def test_evaluate_rejects_readiness_identity_broken_digest() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["identity"] = _identity("joint", broken=("planning_scene_sha256",))
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("planning_scene_sha256" in reason for reason in reasons)


def test_evaluate_rejects_readiness_identity_broken_owned_ids() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["identity"] = _identity("joint", broken=("fixture_owned_ids",))
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("owned_ids" in reason for reason in reasons)


def test_evaluate_rejects_readiness_identity_broken_handoff() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["identity"] = _identity("joint", broken=("fixture_target_handoff",))
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("target_handoff" in reason for reason in reasons)


def test_evaluate_rejects_readiness_identity_missing() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["identity"] = {}
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("identity" in reason for reason in reasons)


# ---------------------------------------------------------------------------
# Evaluator: command mutations
# ---------------------------------------------------------------------------


def test_evaluate_rejects_command_sample() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"]["samples"] = 1
    snapshot["command_observations"]["sample_events"] = [{"stamp_ns": 0, "received_at_s": 100.5}]
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("command sample" in reason for reason in reasons)


def test_evaluate_rejects_command_publisher_wrong_type() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"]["publisher"] = _publisher(type_="std_msgs/msg/String")
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("publisher type" in reason for reason in reasons)


def test_evaluate_rejects_command_publisher_wrong_source() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"]["publisher"] = _publisher(source="/rogue")
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("publisher source" in reason for reason in reasons)


def test_evaluate_rejects_command_publisher_duplicate() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"]["publisher"] = _publisher(count=2)
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("publisher count" in reason for reason in reasons)


def test_evaluate_rejects_command_publisher_absent() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"]["publisher"] = _publisher(count=0, source="", type_="", settled=False)
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("publisher count" in reason for reason in reasons)


def test_evaluate_rejects_command_publisher_wrong_durability() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"]["publisher"] = _publisher(durability="TRANSIENT_LOCAL")
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("durability" in reason for reason in reasons)


def test_evaluate_rejects_command_not_settled() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"]["settled"] = False
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("settle" in reason for reason in reasons)


def test_evaluate_rejects_command_empty_window() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"]["window_start_s"] = 11.0
    snapshot["command_observations"]["window_end_s"] = 11.0
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("window" in reason for reason in reasons)


def test_evaluate_rejects_command_publisher_graph_changed() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"]["publisher_graph_changed"] = True
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("publisher graph changed" in reason for reason in reasons)


def test_evaluate_accepts_command_publisher_graph_unchanged() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"]["publisher_graph_changed"] = False
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert ready, reasons


def test_evaluate_rejects_command_publisher_wrong_depth() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"]["publisher"] = _publisher(depth=10)
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("publisher depth" in reason for reason in reasons)


def test_evaluate_accepts_command_publisher_depth_50() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["command_observations"]["publisher"] = _publisher(depth=50)
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert ready, reasons


# ---------------------------------------------------------------------------
# Evaluator: readiness identity-invalid window mutations
# ---------------------------------------------------------------------------


def test_evaluate_rejects_readiness_identity_invalid_during_window() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["any_identity_invalid_in_window"] = True
    snapshot["readiness"]["window_counts"]["identity_invalid"] = 1
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("identity" in reason for reason in reasons)


def test_evaluate_accepts_no_identity_invalid_in_window() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["readiness"]["any_identity_invalid_in_window"] = False
    snapshot["readiness"]["window_counts"]["identity_invalid"] = 0
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert ready, reasons


# ---------------------------------------------------------------------------
# Evaluator: endpoint mutations
# ---------------------------------------------------------------------------


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


def test_evaluate_rejects_wrong_result_service_type() -> None:
    snapshot = ready_snapshot("joint")
    endpoint = _endpoint()
    endpoint["result_service_types"] = ["tinker_arm_msgs/action/Pick_GetResult"]
    endpoint["result_service_present"] = True
    snapshot["endpoint"] = endpoint
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("result-service" in reason for reason in reasons)


def test_evaluate_rejects_missing_result_service() -> None:
    snapshot = ready_snapshot("joint")
    endpoint = _endpoint()
    endpoint["result_service_present"] = False
    endpoint["result_service_types"] = []
    snapshot["endpoint"] = endpoint
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("result-service" in reason for reason in reasons)


# ---------------------------------------------------------------------------
# Live-client pre-send endpoint gate (ROS-free method, no rclpy needed)
# ---------------------------------------------------------------------------


def _client_without_rclpy() -> "object":
    """Construct an OmplPlanSmokeClient instance without running __init__
    (which would import rclpy).  _endpoint_ok only uses the endpoint dict and
    pure derive functions, so no ROS is required."""
    from ompl_plan_smoke import OmplPlanSmokeClient

    return object.__new__(OmplPlanSmokeClient)


def test_endpoint_ok_accepts_exact_result_type() -> None:
    client = _client_without_rclpy()
    assert client._endpoint_ok(_endpoint()) is True


def test_endpoint_ok_rejects_wrong_result_type() -> None:
    client = _client_without_rclpy()
    endpoint = _endpoint()
    endpoint["result_service_types"] = ["tinker_arm_msgs/action/Pick_GetResult"]
    assert client._endpoint_ok(endpoint) is False


def test_endpoint_ok_rejects_missing_result_service() -> None:
    client = _client_without_rclpy()
    endpoint = _endpoint()
    endpoint["result_service_present"] = False
    endpoint["result_service_types"] = []
    assert client._endpoint_ok(endpoint) is False


def test_endpoint_ok_rejects_wrong_kind() -> None:
    client = _client_without_rclpy()
    endpoint = _endpoint()
    endpoint["kind"] = "ExecuteTrajectory"
    assert client._endpoint_ok(endpoint) is False


# ---------------------------------------------------------------------------
# Evaluator: outcome adjudication — joint/pose success
# ---------------------------------------------------------------------------


def test_evaluate_rejects_success_without_accepted_goal() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["outcome"]["goal_accepted"] = False
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("accepted goal" in reason for reason in reasons)


def test_evaluate_rejects_success_wrong_terminal_status() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["outcome"]["terminal_status"] = STATUS_ABORTED
    snapshot["outcome"]["terminal_status_name"] = "STATUS_ABORTED"
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("STATUS_SUCCEEDED" in reason for reason in reasons)


def test_evaluate_rejects_success_without_result() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["outcome"]["result_received"] = False
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("typed result" in reason for reason in reasons)


def test_evaluate_rejects_success_wrong_error_code() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["outcome"]["error_code"] = -1
    snapshot["outcome"]["error_code_name"] = "PLANNING_FAILED"
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("error_code" in reason for reason in reasons)


def test_evaluate_rejects_empty_successful_plan() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["outcome"]["trajectory_point_count"] = 0
    snapshot["outcome"]["trajectory_joint_names"] = []
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("empty trajectory" in reason for reason in reasons)


def test_evaluate_rejects_successful_plan_without_joint_names() -> None:
    snapshot = ready_snapshot("joint")
    snapshot["outcome"]["trajectory_joint_names"] = []
    snapshot["outcome"]["trajectory_point_count"] = 5
    ready, reasons = evaluate_smoke(snapshot, expected_for("joint"))
    assert not ready
    assert any("joint names" in reason for reason in reasons)


# ---------------------------------------------------------------------------
# Evaluator: non-terminal kinds
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("mode", ["joint", "pose"])
@pytest.mark.parametrize("bad_kind", ["timeout", "cancelled", "invalid", "rejected"])
def test_evaluate_rejects_non_terminal_kinds_all_modes(mode: str, bad_kind: str) -> None:
    snapshot = ready_snapshot(mode)
    snapshot["outcome"] = {
        "kind": bad_kind,
        "detail": "bounded path",
        "goal_accepted": False,
        "result_received": False,
        "terminal_status": None,
        "terminal_status_name": "UNKNOWN_STATUS",
        "error_code": None,
        "error_code_name": "CODE_None",
        "trajectory_point_count": 0,
        "trajectory_joint_names": [],
        "planning_time": None,
        "cancel_requested": False,
        "cancel_accepted": False,
        "cancel_confirmed": False,
    }
    ready, reasons = evaluate_smoke(snapshot, expected_for(mode))
    assert not ready
    assert any("successful plan" in reason for reason in reasons)


def test_evaluate_rejects_unknown_mode() -> None:
    ready, reasons = evaluate_smoke(ready_snapshot("joint"), {"mode": "bogus"})
    assert not ready
    assert any("mode" in reason for reason in reasons)


# ---------------------------------------------------------------------------
# Error-code name derivation
# ---------------------------------------------------------------------------


def test_moveit_error_code_names() -> None:
    assert moveit_error_code_name(1) == "SUCCESS"
    assert moveit_error_code_name(-1) == "PLANNING_FAILED"
    assert moveit_error_code_name(-12) == "GOAL_IN_COLLISION"
    assert moveit_error_code_name(99999) == "FAILURE"
    assert moveit_error_code_name(-12345) == "CODE_-12345"


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
    assert parsed["outcome"]["terminal_status"] == STATUS_SUCCEEDED
    assert parsed["outcome"]["trajectory_joint_names"]


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
        meta={"python": "3.10.12", "domain_id": "25"},
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


def test_write_report_atomic_cleans_temp_on_failure(tmp_path) -> None:
    target = tmp_path / "report.json"
    data = b'{"schema_version":1}'
    # Make the parent read-only so the atomic rename fails after temp write.
    try:
        tmp_path.chmod(0o500)
        target.parent.mkdir(parents=True, exist_ok=True)
        with pytest.raises((OSError, PermissionError)):
            write_report_atomic(target, data)
    finally:
        tmp_path.chmod(0o700)
    assert not target.exists()
    assert not list(tmp_path.glob("*.tmp"))


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


def test_readiness_contract_identities_match_bridge_helpers() -> None:
    """The smoke's local identity computation agrees with the Task 6 helpers."""
    from tinker_sim_bridge.integrated_readiness import (
        planning_scene_mapping,
        scenario_mapping,
        sha256_json,
    )

    scenario = load_scenario(JOINT_SCENARIO)
    expected = scenario_expected_identities(scenario)
    scenario_id = expected["scenario_id"]
    seed = expected["seed"]
    declaration = {
        str(key): value for key, value in scenario.items() if key not in ("id", "seed")
    }
    assert expected["scenario_declaration_sha256"] == sha256_json(
        scenario_mapping(scenario_id, seed, declaration)
    )
    assert expected["planning_scene_sha256"] == sha256_json(
        planning_scene_mapping(scenario["planning_scene"])
    )


# ---------------------------------------------------------------------------
# CLI arg parsing (ROS-free)
# ---------------------------------------------------------------------------


def test_parse_args_rejects_bad_mode() -> None:
    from ompl_plan_smoke import parse_args

    with pytest.raises(SystemExit):
        parse_args(["--mode", "bogus"])


def test_parse_args_defaults() -> None:
    from ompl_plan_smoke import parse_args

    args = parse_args(["--mode", "pose"])
    assert args.mode == "pose"
    assert args.timeout == 30.0
    assert args.readiness_timeout == 60.0
    assert args.post_result_tail == 0.25
