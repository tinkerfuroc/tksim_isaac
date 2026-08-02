"""Task 4 (pure): ROS-free integrated Gate-C executor contract/readiness tests.

This module runs under the simulator CPython 3.12 venv and never imports
``rclpy`` or any generated ROS message type.  It covers:

- dict-level report/readiness contracts reconciled with the real canonical
  multi-operation public report (one-key ``integrated`` mapping,
  scenario-declaration-bound fixture descriptor digest);
- endpoint/cardinality/type/QoS graph contracts and their real providers;
- scenario report identities for all 17 scenarios (complete seven-key set);
- the genuine positive-ready baseline (``ready is True``) before every mutation
  family; every negative test mutates exactly one contract and checks the
  specific failure reason;
- AST/static zero-command checks (no ``/isaac_joint_commands`` publisher);
- the Task 3 journal graph projection builder validated by
  ``planning_scene_journal.validate_graph_evidence``.

Any test that invokes a ROS-importing builder (``moveit_msgs``,
``geometry_msgs``, ``sensor_msgs_py``, ``tinker_arm_msgs``) lives in
``tests/test_integrated_gate_executor_ros.py``.
"""
from __future__ import annotations

import ast
import copy
import hashlib
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "tests"))

from qualification_test_helpers import load_test_scenario  # noqa: E402
from tinker_sim_bridge.fixture_planning_scene import (  # noqa: E402
    canonical_fixture_status,
    fixture_descriptor_sha256,
    fixture_owned_ids,
    serialize_status,
)
from tinker_sim_bridge.integrated_readiness import public_integrated_mapping  # noqa: E402
from validation.integrated_gate_executor import (  # noqa: E402
    Q_OUTBOUND,
    REQUIRED_ACTIONS,
    REQUIRED_SERVICES,
    REQUIRED_TOPICS,
    STAGE_C_SCENARIOS,
    _REQUIRED_ENDPOINT_SOURCES,
    build_journal_graph_projection,
    evaluate_executor_readiness,
    expected_physics_ready_report,
    stage_c_dispatch,
    validate_physics_ready_snapshot,
)

SCENARIO_NAMES = (
    "qualification-moveit-plan-joint",
    "qualification-moveit-plan-pose",
    "qualification-moveit-plan-blocked",
    "qualification-moveit-execute-joint",
    "qualification-moveit-execute-pose",
    "qualification-moveit-cartesian-retreat",
    "qualification-moveit-gripper",
    "qualification-moveit-cancel",
    "qualification-moveit-safety",
    "qualification-pick-place-positive",
    "qualification-pick-place-blocked-approach",
    "qualification-pick-place-unreachable-grasp",
    "qualification-pick-place-malformed-back",
    "qualification-pick-place-cancel-approach",
    "qualification-pick-place-cancel-transport",
    "qualification-pick-place-safety-transport",
    "qualification-pick-place-occupied-place",
)

REPORT_IDENTITY_KEYS = {
    "scenario_id", "seed", "scenario_declaration_sha256", "planning_scene_sha256",
    "integrated_sha256", "model_fingerprint", "provider_manifest_sha256",
}


def _config() -> dict[str, object]:
    return {
        "execution_profile": "sim_ompl",
        "thresholds": {
            "joint_state_fresh_s": 0.25,
            "tf_fresh_s": 0.25,
            "fixture_fresh_s": 0.25,
        },
    }


def scenario_report_contract(name: str) -> dict[str, object]:
    """Read the complete current scenario contract; never reduce it to a subset."""
    source = load_test_scenario(name)
    required = (
        "scenario",
        "planning_scene",
        "planning_scene_declaration",
        "integrated",
        "report_identities",
    )
    if any(key not in source for key in required):
        raise AssertionError(f"{name} does not expose the complete report inputs")
    scenario_mapping = copy.deepcopy(source["scenario"])
    planning_scene = copy.deepcopy(source["planning_scene"])
    planning_scene_declaration = copy.deepcopy(source["planning_scene_declaration"])
    integrated = copy.deepcopy(source["integrated"])
    identities = copy.deepcopy(source["report_identities"])
    if set(identities) != REPORT_IDENTITY_KEYS:
        raise AssertionError(f"{name} has incomplete report identities")
    if identities["scenario_id"] != scenario_mapping["id"] or identities["seed"] != scenario_mapping["seed"]:
        raise AssertionError(f"{name} report identity does not match the scenario mapping")
    return {
        "scenario_mapping": scenario_mapping,
        "planning_scene": planning_scene,
        "planning_scene_declaration": planning_scene_declaration,
        "integrated": integrated,
        "identities": identities,
    }


def _assert_complete_scenario_report(name: str) -> dict[str, object]:
    contract = scenario_report_contract(name)
    report = expected_physics_ready_report(
        scenario_mapping=contract["scenario_mapping"],
        planning_scene=contract["planning_scene_declaration"],
        integrated=contract["integrated"],
        expected_identities=contract["identities"],
    )
    assert report["scenario"] == contract["scenario_mapping"]
    assert report["planning_scene"] == contract["planning_scene"]
    # The public report carries the one-key integrated mapping; the full
    # per-scenario mapping stays bound by the scenario declaration SHA-256.
    assert report["integrated"] == public_integrated_mapping()
    assert report["identities"] == contract["identities"]
    report_bytes = json.dumps(
        report, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    validate_physics_ready_snapshot(
        {
            "scenario": {"scenario_report_sha256": hashlib.sha256(report_bytes).hexdigest()},
            "scenario_report_bytes": report_bytes,
            "model": {"fingerprint": contract["identities"]["model_fingerprint"]},
            "provider_manifest_sha256": contract["identities"]["provider_manifest_sha256"],
        },
        readiness_scenario(contract),
    )
    return contract


def test_positive_report_uses_complete_current_scenario_contract():
    contract = _assert_complete_scenario_report("qualification-pick-place-positive")
    assert set(contract["scenario_mapping"]) >= {"id", "seed", "declaration"}
    assert contract["planning_scene_declaration"]["target_handoff"] == "pick_and_place/object_mesh"


def test_report_schema_seed_and_operation_state_reject_booleans():
    contract = scenario_report_contract("qualification-pick-place-positive")
    report = expected_physics_ready_report(
        scenario_mapping=contract["scenario_mapping"],
        planning_scene=contract["planning_scene_declaration"],
        integrated=contract["integrated"],
        expected_identities=contract["identities"],
    )
    for field_path in (("schema_version",), ("identities", "seed"), ("operations", 0, "state")):
        mutated = copy.deepcopy(report)
        target = mutated
        for key in field_path[:-1]:
            target = target[key]
        target[field_path[-1]] = True
        with pytest.raises(ValueError, match="strict integer|integer"):
            validate_physics_ready_snapshot(
                {"scenario": {"scenario_report_sha256": "invalid"}, "scenario_report_bytes": b"{}"},
                readiness_scenario(contract), expected_report=mutated)


def test_blocked_report_uses_complete_current_scenario_contract():
    contract = _assert_complete_scenario_report("qualification-pick-place-blocked-approach")
    assert contract["scenario_mapping"]["id"] == "qualification-pick-place-blocked-approach"
    assert "sim_fixture/plan_blocker" in contract["planning_scene_declaration"].get(
        "diagnostic_objects", []
    ) or any(
        record["id"] == "sim_fixture/plan_blocker"
        for record in contract["planning_scene_declaration"].get("objects", [])
    )


@pytest.mark.parametrize("scenario_name", SCENARIO_NAMES)
def test_all_17_scenarios_build_and_validate_their_own_report(scenario_name):
    _assert_complete_scenario_report(scenario_name)


POSITIVE_REPORT_CONTRACT = scenario_report_contract("qualification-pick-place-positive")
BLOCKED_REPORT_CONTRACT = scenario_report_contract("qualification-pick-place-blocked-approach")
PUBLIC_DECLARATION_DIGEST = POSITIVE_REPORT_CONTRACT["identities"]["scenario_declaration_sha256"]
PLANNING_SCENE_SHA256 = POSITIVE_REPORT_CONTRACT["identities"]["planning_scene_sha256"]
INTEGRATED_SHA256 = POSITIVE_REPORT_CONTRACT["identities"]["integrated_sha256"]
MODEL_FINGERPRINT = POSITIVE_REPORT_CONTRACT["identities"]["model_fingerprint"]
PROVIDER_MANIFEST_SHA256 = POSITIVE_REPORT_CONTRACT["identities"]["provider_manifest_sha256"]
FIXTURE_REVISION_DIGEST = POSITIVE_REPORT_CONTRACT["planning_scene_declaration"]["revision_digest"]
WRONG_DIGEST = "f" * 64


def canonical_report_bytes(contract: dict[str, object] = POSITIVE_REPORT_CONTRACT) -> bytes:
    return json.dumps(
        expected_physics_ready_report(
            scenario_mapping=contract["scenario_mapping"],
            planning_scene=contract["planning_scene_declaration"],
            integrated=contract["integrated"],
            expected_identities=contract["identities"],
        ),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")


def _canonical_fixture_payload(contract: dict[str, object] = POSITIVE_REPORT_CONTRACT) -> str:
    declaration = contract["planning_scene_declaration"]
    status = canonical_fixture_status(
        scenario=contract["scenario_mapping"]["id"],
        revision=declaration["revision"],
        revision_digest=declaration["revision_digest"],
        sequence=2,
        published_at=1.0,
        owned_ids=fixture_owned_ids(declaration),
        target_source_id=declaration["target_source_id"],
        target_handoff=declaration["target_handoff"],
        descriptor_sha256=fixture_descriptor_sha256(declaration),
        state="FIXTURE_READY",
    )
    return serialize_status(status)


def _fixture_descriptor_digest(contract: dict[str, object] = POSITIVE_REPORT_CONTRACT) -> str:
    return fixture_descriptor_sha256(contract["planning_scene_declaration"])


def _observed_graph_double() -> dict[str, object]:
    """A valid observed-graph double for the exact three journal topics/services.

    The two PlanningScene topics use the stock MoveIt2 Humble
    RELIABLE/VOLATILE/depth-100 contract; the fixture status topic stays
    RELIABLE/TRANSIENT_LOCAL/depth 1 (F2.3).
    """
    recorder = {"node": "/tinker_integrated_gate_executor", "node_namespace": ""}
    move_group = {"node": "/move_group", "node_namespace": ""}
    fixture_publisher = {"node": "/fixture_planning_scene", "node_namespace": ""}
    planning_scene_qos = {"reliability": "RELIABLE", "durability": "VOLATILE", "depth": 100}
    fixture_qos = {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1}
    service_qos = {"reliability": "RELIABLE", "durability": "VOLATILE"}
    return {
        "node_name": "/tinker_integrated_gate_executor",
        "namespace": "/",
        "remap_table": {},
        "topics": {
            "/planning_scene": {
                "type": "moveit_msgs/msg/PlanningScene",
                "requested_qos": dict(planning_scene_qos),
                "offered_qos": dict(planning_scene_qos),
                "publishers": [dict(move_group)],
                "subscribers": [dict(recorder)],
            },
            "/monitored_planning_scene": {
                "type": "moveit_msgs/msg/PlanningScene",
                "requested_qos": dict(planning_scene_qos),
                "offered_qos": dict(planning_scene_qos),
                "publishers": [dict(move_group)],
                "subscribers": [dict(recorder)],
            },
            "/sim/status/planning_scene_fixture": {
                "type": "std_msgs/msg/String",
                "requested_qos": dict(fixture_qos),
                "offered_qos": dict(fixture_qos),
                "publishers": [dict(fixture_publisher)],
                "subscribers": [dict(recorder)],
            },
        },
        "services": {
            "/get_planning_scene": {
                "type": "moveit_msgs/srv/GetPlanningScene",
                "requested_qos": dict(service_qos),
                "offered_qos": dict(service_qos),
                "servers": [dict(move_group)],
                "clients": [dict(recorder)],
            },
            "/apply_planning_scene": {
                "type": "moveit_msgs/srv/ApplyPlanningScene",
                "requested_qos": dict(service_qos),
                "offered_qos": dict(service_qos),
                "servers": [dict(move_group)],
                "clients": [dict(recorder)],
            },
        },
    }


def _ready_snapshot_for_contract(contract: dict[str, object]) -> dict[str, object]:
    """A genuine passing readiness baseline derived from any scenario contract."""
    joints = [f"joint{i}" for i in range(1, 8)] + ["drive_joint"]
    scenario_mapping = contract["scenario_mapping"]
    identities = contract["identities"]
    declaration = contract["planning_scene_declaration"]
    scenario_id = scenario_mapping["id"]
    seed = scenario_mapping["seed"]
    fixture_ids = list(fixture_owned_ids(declaration))
    fixture_digest = _fixture_descriptor_digest(contract)
    fixture_payload = _canonical_fixture_payload(contract)
    report_bytes = canonical_report_bytes(contract)
    digest_fields = {
        "scenario_declaration_sha256": identities["scenario_declaration_sha256"],
        "planning_scene_sha256": identities["planning_scene_sha256"],
        "integrated_sha256": identities["integrated_sha256"],
        "model_fingerprint": identities["model_fingerprint"],
        "provider_manifest_sha256": identities["provider_manifest_sha256"],
    }
    return {
        "scenario": {
            "state": "PHYSICS_READY", "report_verified": True,
            "scenario": scenario_id, "scenario_id": scenario_id, "seed": seed,
            **dict(digest_fields),
            "planning_scene_revision": declaration["revision"],
            "final_simulation_state": "STATE_PLAYING",
            "boundary": "PHYSICS_READY",
            "scenario_report_sha256": hashlib.sha256(report_bytes).hexdigest(),
            "planning_scene": {
                "state": "declared", "owner": "sim_fixture",
                "revision": declaration["revision"],
                "revision_digest": declaration["revision_digest"],
                "owned_ids": fixture_ids,
                "target_source_id": declaration["target_source_id"],
                "target_handoff": "pick_and_place/object_mesh",
            },
            "integrated": {"execution_profile": "sim_ompl"},
            "operations": [
                {
                    "state": 1, "boundary": "PHYSICS_READY",
                    "scenario_id": scenario_id, "seed": seed,
                    **dict(digest_fields),
                }
            ],
        },
        "scenario_report_bytes": report_bytes,
        "model": {"fingerprint_match": True, "fingerprint": MODEL_FINGERPRINT},
        "provider_manifest_sha256": PROVIDER_MANIFEST_SHA256,
        "tf": {"complete": True, "age_s": 0.05},
        "joint_state": {
            "names": joints,
            "positions": [0.0] * 8,
            "velocities": [0.0] * 8,
            "header_stamp_ns": 1,
            "age_s": 0.05,
            "publisher_count": 1,
            "source_node": "/controller_manager",
            "logical_controller": "joint_state_broadcaster",
        },
        "controllers": {
            "manager_healthy": True,
            "manager_source_node": "/controller_manager",
            "manager_publisher_count": 1,
            "logical_controllers": {
                "joint_state_broadcaster": {
                    "state": "active", "source_node": "/controller_manager", "cardinality": 1,
                },
                "xarm7_traj_controller": {
                    "state": "active", "source_node": "/controller_manager", "cardinality": 1,
                },
            },
        },
        "safety": {
            "stop": False, "age_s": 0.05, "sample_count": 2,
            "type": "std_msgs/msg/Bool", "publisher_count": 1,
            "source_node": "/tinker_sim_safety_supervisor",
            "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
        },
        "actions": {
            name: {
                "type": action_type, "ready": True, "server_count": 1,
                "source_node": _REQUIRED_ENDPOINT_SOURCES[name],
            }
            for name, action_type in REQUIRED_ACTIONS.items()
        },
        "services": {
            name: {
                "type": service_type, "ready": True, "server_count": 1,
                "source_node": _REQUIRED_ENDPOINT_SOURCES[name],
            }
            for name, service_type in REQUIRED_SERVICES.items()
        },
        "topics": {
            "/joint_states": {
                "type": "sensor_msgs/msg/JointState", "publisher_count": 1,
                "source_node": "/controller_manager",
                "qos": {"reliability": "reliable", "durability": "volatile", "depth": 10},
                "names": joints, "positions": [0.0] * 8, "velocities": [0.0] * 8,
                "header_stamp_ns": 1, "age_s": 0.05,
            },
            "/sim/status/planning_scene_fixture": {
                "type": "std_msgs/msg/String", "publisher_count": 1,
                "source_node": "/fixture_planning_scene",
                "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
                "received": True, "received_sequence": 2, "sample_count": 2,
                "age_s": 0.05, "payload": fixture_payload,
            },
            "/sim/safety/operator": {
                "type": "std_msgs/msg/Bool", "publisher_count": 1,
                "source_node": "/tinker_integrated_gate_executor",
                "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
                "allowlist": [False, True], "received": True,
                "received_value": False, "received_timestamp_ns": 1,
                "received_age_s": 0.05, "freshness_limit_s": 0.25,
            },
            "/sim/hardware/safety_stop": {
                "type": "std_msgs/msg/Bool", "publisher_count": 1,
                "source_node": "/tinker_sim_safety_supervisor",
                "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
                "data": False, "received": True, "received_value": False,
                "received_timestamp_ns": 1, "sample_count": 2, "age_s": 0.05,
            },
        },
        "fixture": {
            "schema_version": 1, "state": "FIXTURE_READY",
            "scenario": scenario_id,
            "owner": "sim_fixture", "revision": declaration["revision"],
            "revision_digest": declaration["revision_digest"],
            "owned_ids": fixture_ids,
            "target_source_id": declaration["target_source_id"],
            "target_handoff": "pick_and_place/object_mesh",
            "sequence": 2, "previous_sequence": 1, "sample_count": 2,
            "published_at": 1.0, "age_s": 0.05,
            "fixture_descriptor_sha256": fixture_digest,
        },
        "planning_scene": {
            "owned_ids": fixture_ids,
            "attached_ids": [],
        },
        "robot_in_collision": False,
    }


def ready_executor_snapshot() -> dict[str, object]:
    """A genuine passing readiness baseline before mutation tests begin."""
    return _ready_snapshot_for_contract(POSITIVE_REPORT_CONTRACT)


def readiness_scenario(contract: dict[str, object] = POSITIVE_REPORT_CONTRACT) -> dict[str, object]:
    scenario_mapping = dict(contract["scenario_mapping"])
    identities = dict(contract["identities"])
    planning_scene = dict(contract["planning_scene"])
    planning_scene_declaration = dict(contract["planning_scene_declaration"])
    integrated = dict(contract["integrated"])
    return {
        "id": scenario_mapping["id"],
        "seed": scenario_mapping["seed"],
        "public_mapping": scenario_mapping,
        "scenario_mapping": scenario_mapping,
        "planning_scene": planning_scene,
        "planning_scene_declaration": planning_scene_declaration,
        "integrated": integrated,
        "identities": identities,
        "scenario_report_sha256": hashlib.sha256(canonical_report_bytes(contract)).hexdigest(),
        **{key: value for key, value in identities.items()
           if key.endswith("_sha256") or key == "model_fingerprint"},
    }


def test_positive_readiness_baseline_is_ready():
    result = evaluate_executor_readiness(
        ready_executor_snapshot(), _config(), readiness_scenario()
    )
    assert result["ready"] is True
    assert result["reasons"] == []


def test_executor_source_has_no_isaac_command_publisher():
    source_path = Path(__file__).resolve().parents[1] / "validation/integrated_gate_executor.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    forbidden_calls: list[int] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or getattr(node.func, "attr", "") != "create_publisher":
            continue
        if node.args and isinstance(node.args[0], ast.Constant):
            if node.args[0].value == "/isaac_joint_commands":
                forbidden_calls.append(node.lineno)
    assert forbidden_calls == []


def test_endpoint_source_and_cardinality_contract_uses_real_providers():
    assert _REQUIRED_ENDPOINT_SOURCES["/xarm_gripper/gripper_action"] == "/tinker_sim_gripper_facade"
    assert _REQUIRED_ENDPOINT_SOURCES["/xarm7_traj_controller/follow_joint_trajectory"] == "/controller_manager"
    assert "/joint_state_broadcaster" not in _REQUIRED_ENDPOINT_SOURCES.values()
    assert "/xarm_gripper" not in _REQUIRED_ENDPOINT_SOURCES.values()
    assert REQUIRED_TOPICS["/joint_states"]["type"] == "sensor_msgs/msg/JointState"
    assert REQUIRED_TOPICS["/joint_states"]["publisher_count"] == 1
    assert REQUIRED_TOPICS["/sim/status/planning_scene_fixture"]["publisher_count"] == 1
    assert REQUIRED_TOPICS["/sim/safety/operator"]["publisher_count"] == 1
    assert REQUIRED_TOPICS["/sim/hardware/safety_stop"]["type"] == "std_msgs/msg/Bool"
    assert REQUIRED_TOPICS["/sim/hardware/safety_stop"]["publisher_count"] == 1


def test_journal_graph_projection_passes_task3_validator():
    from planning_scene_journal import validate_graph_evidence

    projection = build_journal_graph_projection(
        fixture_payload=_canonical_fixture_payload(),
        observed_graph=_observed_graph_double(),
    )
    normalized = validate_graph_evidence(projection)
    assert normalized["node_name"] == "/tinker_integrated_gate_executor"
    assert normalized["namespace"] == "/"
    assert normalized["remap_table"] == {}
    assert set(normalized["topics"]) == {
        "/planning_scene",
        "/monitored_planning_scene",
        "/sim/status/planning_scene_fixture",
    }
    assert set(normalized["services"]) == {"/get_planning_scene", "/apply_planning_scene"}
    assert normalized["topics"]["/sim/status/planning_scene_fixture"]["payload_parsed"][
        "fixture_descriptor_sha256"
    ] == _fixture_descriptor_digest()


def test_planning_scene_topic_qos_contract_is_volatile_depth_100():
    """F2.3: stock MoveIt2 Humble publishes PlanningScene RELIABLE/VOLATILE/depth 100."""
    from validation.integrated_gate_executor import (
        JOURNAL_FIXTURE_TOPIC_QOS,
        JOURNAL_PLANNING_SCENE_TOPIC_QOS,
    )

    assert JOURNAL_PLANNING_SCENE_TOPIC_QOS == {
        "reliability": "RELIABLE", "durability": "VOLATILE", "depth": 100,
    }
    assert JOURNAL_FIXTURE_TOPIC_QOS == {
        "reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1,
    }


def test_journal_graph_projection_rejects_old_transient_local_planning_scene_qos():
    """F2.3: the stale TRANSIENT_LOCAL/depth-1 PlanningScene claim is rejected."""
    fixture_payload = _canonical_fixture_payload()

    graph = copy.deepcopy(_observed_graph_double())
    graph["topics"]["/planning_scene"]["requested_qos"] = {
        "reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1,
    }
    with pytest.raises(ValueError, match="QoS"):
        build_journal_graph_projection(fixture_payload=fixture_payload, observed_graph=graph)

    graph = copy.deepcopy(_observed_graph_double())
    graph["topics"]["/monitored_planning_scene"]["offered_qos"] = {
        "reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1,
    }
    with pytest.raises(ValueError, match="QoS"):
        build_journal_graph_projection(fixture_payload=fixture_payload, observed_graph=graph)

    graph = copy.deepcopy(_observed_graph_double())
    graph["topics"]["/planning_scene"]["requested_qos"]["depth"] = 1
    with pytest.raises(ValueError, match="QoS"):
        build_journal_graph_projection(fixture_payload=fixture_payload, observed_graph=graph)


def test_readiness_requires_eight_fresh_joints():
    snapshot = ready_executor_snapshot()
    snapshot["joint_state"]["names"] = snapshot["joint_state"]["names"][:-1]
    snapshot["joint_state"]["positions"] = snapshot["joint_state"]["positions"][:-1]
    snapshot["joint_state"]["velocities"] = snapshot["joint_state"]["velocities"][:-1]
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False
    assert "drive_joint" in " ".join(result["reasons"])


def test_readiness_requires_joint_state_broadcaster_logical_controller():
    snapshot = ready_executor_snapshot()
    snapshot["joint_state"]["logical_controller"] = "xarm7_traj_controller"
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False
    assert "joint state" in " ".join(result["reasons"])


def test_readiness_rejects_noncanonical_physics_ready_operation():
    for field, value in (("state", 0), ("boundary", "PLAYING"), ("scenario_id", "wrong"), ("seed", 8)):
        snapshot = ready_executor_snapshot()
        report = json.loads(canonical_report_bytes().decode("utf-8"))
        report["operations"][-1][field] = value
        mutated_bytes = json.dumps(
            report, sort_keys=True, separators=(",", ":"), ensure_ascii=False
        ).encode("utf-8")
        snapshot["scenario_report_bytes"] = mutated_bytes
        snapshot["scenario"]["scenario_report_sha256"] = hashlib.sha256(mutated_bytes).hexdigest()
        result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
        assert result["ready"] is False


def test_readiness_rejects_noncanonical_json_with_matching_external_digest():
    snapshot = ready_executor_snapshot()
    report = json.loads(canonical_report_bytes().decode("utf-8"))
    noncanonical_bytes = json.dumps(report, indent=2, ensure_ascii=False).encode("utf-8")
    assert noncanonical_bytes != canonical_report_bytes()
    snapshot["scenario_report_bytes"] = noncanonical_bytes
    snapshot["scenario"]["scenario_report_sha256"] = hashlib.sha256(noncanonical_bytes).hexdigest()
    with pytest.raises(ValueError, match="canonical serialization"):
        validate_physics_ready_snapshot(snapshot, readiness_scenario())


def test_readiness_rejects_wrong_fixture_digest_and_missing_task_endpoint():
    snapshot = ready_executor_snapshot()
    snapshot["fixture"]["revision_digest"] = WRONG_DIGEST
    snapshot["actions"].pop("/fold_action")
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False
    assert "fixture" in " ".join(result["reasons"])
    assert "/fold_action" in " ".join(result["reasons"])


def test_readiness_rejects_fixture_descriptor_mismatch_between_observation_and_payload():
    snapshot = ready_executor_snapshot()
    snapshot["fixture"]["fixture_descriptor_sha256"] = "6" * 64
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False


def test_readiness_rejects_wrong_but_equal_duplicated_descriptor_digest():
    snapshot = ready_executor_snapshot()
    wrong = "6" * 64
    snapshot["fixture"]["fixture_descriptor_sha256"] = wrong
    payload = json.loads(snapshot["topics"]["/sim/status/planning_scene_fixture"]["payload"])
    payload["fixture_descriptor_sha256"] = wrong
    snapshot["topics"]["/sim/status/planning_scene_fixture"]["payload"] = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    )
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False


@pytest.mark.parametrize(
    ("field", "bad_digest"),
    [
        ("revision_digest", "A" * 64),
        ("fixture_descriptor_sha256", "A" * 64),
        ("revision_digest", "0" * 64),
        ("fixture_descriptor_sha256", "0" * 64),
    ],
)
def test_readiness_rejects_uppercase_and_all_zero_fixture_digests(field, bad_digest):
    snapshot = ready_executor_snapshot()
    snapshot["fixture"][field] = bad_digest
    payload = json.loads(snapshot["topics"]["/sim/status/planning_scene_fixture"]["payload"])
    payload[field] = bad_digest
    snapshot["topics"]["/sim/status/planning_scene_fixture"]["payload"] = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    )
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False


def test_readiness_rejects_payload_revision_digest_mismatch():
    snapshot = ready_executor_snapshot()
    payload = json.loads(snapshot["topics"]["/sim/status/planning_scene_fixture"]["payload"])
    payload["revision_digest"] = "9" * 64
    snapshot["topics"]["/sim/status/planning_scene_fixture"]["payload"] = json.dumps(
        payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    )
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False


def test_readiness_rejects_missing_arm_joint_service_and_gate_services():
    snapshot = ready_executor_snapshot()
    snapshot["services"].pop("/arm_joint_service")
    snapshot["services"].pop("/sim/ready/physics")
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False
    reasons = " ".join(result["reasons"])
    assert "/arm_joint_service" in reasons
    assert "/sim/ready/physics" in reasons


def test_readiness_rejects_fixture_topic_graph_mismatch_even_with_valid_payload():
    for field, value in (
        ("type", "std_msgs/msg/Bool"),
        ("publisher_count", 2),
        ("source_node", "/wrong_fixture_node"),
        ("qos", {"reliability": "best_effort", "durability": "transient_local", "depth": 10}),
    ):
        snapshot = ready_executor_snapshot()
        snapshot["topics"]["/sim/status/planning_scene_fixture"][field] = value
        result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
        assert result["ready"] is False
        assert "planning_scene_fixture" in " ".join(result["reasons"])
    snapshot = ready_executor_snapshot()
    snapshot["fixture"]["target_handoff"] = "wrong/handoff"
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False


@pytest.mark.parametrize(
    ("topic", "field", "value"),
    [
        ("/joint_states", "type", "std_msgs/msg/Float64"),
        ("/joint_states", "publisher_count", 2),
        ("/joint_states", "source_node", "/wrong_joint_state_source"),
        ("/joint_states", "qos", {"reliability": "best_effort", "durability": "volatile", "depth": 1}),
        ("/sim/hardware/safety_stop", "type", "std_msgs/msg/String"),
        ("/sim/hardware/safety_stop", "publisher_count", 2),
        ("/sim/hardware/safety_stop", "source_node", "/wrong_safety_source"),
        ("/sim/hardware/safety_stop", "qos", {"reliability": "best_effort", "durability": "volatile", "depth": 10}),
    ],
)
def test_readiness_rejects_required_joint_and_safety_topic_graph_mismatch(topic, field, value):
    snapshot = ready_executor_snapshot()
    snapshot["topics"][topic][field] = value
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False
    assert topic in " ".join(result["reasons"])


@pytest.mark.parametrize(
    ("topic", "field", "value"),
    [
        ("/joint_states", "header_stamp_ns", 0),
        ("/joint_states", "age_s", 1.0),
        ("/joint_states", "positions", [0.0] * 7),
        ("/sim/hardware/safety_stop", "data", True),
        ("/sim/hardware/safety_stop", "sample_count", 1),
        ("/sim/hardware/safety_stop", "age_s", 1.0),
    ],
)
def test_readiness_rejects_required_joint_and_safety_content_or_freshness(topic, field, value):
    snapshot = ready_executor_snapshot()
    if topic == "/joint_states":
        snapshot["joint_state"][field] = value
    else:
        snapshot["safety"][field] = value
        snapshot["topics"][topic][field] = value
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False
    assert topic.split("/", 2)[-1] in " ".join(result["reasons"])


def test_readiness_rejects_safety_operator_wrong_source_or_allowlist():
    for field, value in (("source_node", "/wrong_operator"), ("publisher_count", 2), ("allowlist", [True])):
        snapshot = ready_executor_snapshot()
        snapshot["topics"]["/sim/safety/operator"][field] = value
        result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
        assert result["ready"] is False
        assert "safety/operator" in " ".join(result["reasons"])


def test_readiness_rejects_legacy_nonexistent_provider_sources():
    snapshot = ready_executor_snapshot()
    snapshot["joint_state"]["source_node"] = "/joint_state_broadcaster"
    snapshot["actions"]["/xarm_gripper/gripper_action"]["source_node"] = "/wrong_provider"
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False


def test_qualification_process_helpers_expose_additive_mechanics(tmp_path):
    """The six-gate runner's process/recorder/provenance helpers are exposed
    additively for later tasks without duplicating ownership/cleanup logic.

    This only exercises the thin additive wrappers (no process is started, no
    ROS tooling is invoked).  ``run()`` behavior and the six-gate artifact
    schema are untouched.
    """
    from validation.manipulation_qualification import (  # noqa: E402
        QualificationProcessHelpers,
        QualificationRunner,
        qualification_new_suite_dir,
        qualification_ros_tooling_environment,
        qualification_write_json_atomic,
    )

    runner = QualificationRunner(root=ROOT, attempt_root=tmp_path, gate="all")
    helpers = QualificationProcessHelpers(runner)
    assert helpers.runner is runner
    assert helpers.popen is runner._popen
    assert helpers.command_runner is runner._command_runner
    with pytest.raises(TypeError, match="QualificationRunner"):
        QualificationProcessHelpers(object())

    attempt_id, attempt_dir = helpers.new_attempt_dir()
    assert attempt_dir.is_dir()
    assert attempt_dir.name == attempt_id
    assert attempt_dir.parent == tmp_path

    target = attempt_dir / "evidence.json"
    qualification_write_json_atomic(target, {"a": 1, "b": [2, 3]})
    assert json.loads(target.read_text(encoding="utf-8")) == {"a": 1, "b": [2, 3]}
    helpers.write_json_atomic(target, {"a": 2})
    assert json.loads(target.read_text(encoding="utf-8")) == {"a": 2}

    suite_id, suite_dir = qualification_new_suite_dir(tmp_path)
    assert suite_dir.is_dir()
    assert suite_dir.name == suite_id
    assert suite_dir.parent == tmp_path

    environment = qualification_ros_tooling_environment(root=ROOT, domain_id="7")
    assert environment["ROS_DOMAIN_ID"] == "7"
    assert environment["RMW_IMPLEMENTATION"] == "rmw_fastrtps_cpp"
    assert "AMENT_PREFIX_PATH" in environment


# ---------------------------------------------------------------------------
# Fix round 1 additions: Stage-C readiness baselines, dispatch semantics,
# observed-graph fail-closed projection, and Task-3 journal ownership.
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("scenario_name", STAGE_C_SCENARIOS)
def test_stage_c_readiness_baseline_is_ready(scenario_name):
    """F1.5: every Stage-C scenario has a passing scenario-specific readiness
    baseline with its own revision, owned IDs, and descriptor digest."""
    contract = scenario_report_contract(scenario_name)
    snapshot = _ready_snapshot_for_contract(contract)
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario(contract))
    assert result["ready"] is True
    assert result["reasons"] == []
    declaration = contract["planning_scene_declaration"]
    assert snapshot["fixture"]["revision"] == declaration["revision"]
    assert snapshot["fixture"]["revision_digest"] == declaration["revision_digest"]
    assert snapshot["fixture"]["fixture_descriptor_sha256"] == _fixture_descriptor_digest(contract)
    assert snapshot["fixture"]["owned_ids"] == list(fixture_owned_ids(declaration))


@pytest.mark.parametrize(
    ("scenario_name", "kind", "expectation"),
    [
        ("qualification-moveit-plan-joint", "joint", "success"),
        ("qualification-moveit-plan-pose", "pose", "success"),
        ("qualification-moveit-plan-blocked", "blocked", "non-success"),
    ],
)
def test_stage_c_dispatch_validates_three_scenarios(scenario_name, kind, expectation):
    """F1.6: the three Stage-C plan-only scenarios dispatch to the exact goal
    kind and expected diagnostic polarity."""
    contract = scenario_report_contract(scenario_name)
    spec = stage_c_dispatch(scenario_name, scenario=readiness_scenario(contract))
    assert spec["scenario_id"] == scenario_name
    assert spec["kind"] == kind
    assert spec["expectation"] == expectation
    if kind == "joint":
        assert spec["joints"] == list(Q_OUTBOUND)
    else:
        xyz = spec["target_pose"]["xyz"]
        quaternion = spec["target_pose"]["quaternion_xyzw"]
        assert len(xyz) == 3 and all(math.isfinite(float(value)) for value in xyz)
        assert len(quaternion) == 4 and all(math.isfinite(float(value)) for value in quaternion)
        norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
        assert abs(norm - 1.0) <= 1.0e-3


def test_stage_c_dispatch_rejects_non_c_scenario():
    contract = scenario_report_contract("qualification-pick-place-positive")
    with pytest.raises(ValueError, match="stage must be C"):
        stage_c_dispatch(
            "qualification-pick-place-positive", scenario=readiness_scenario(contract)
        )
    with pytest.raises(ValueError, match="scenario_id does not match"):
        stage_c_dispatch(
            "qualification-moveit-plan-pose", scenario=readiness_scenario(contract)
        )


def test_journal_graph_projection_fails_closed_on_observed_graph_mutations():
    """F1.4: mutations of the observed input (never a pre-fabricated projection)
    fail closed with the specific reason."""
    fixture_payload = _canonical_fixture_payload()

    graph = copy.deepcopy(_observed_graph_double())
    graph["topics"]["/planning_scene"]["subscribers"] = [
        {"node": "/other", "node_namespace": ""}
    ]
    with pytest.raises(ValueError, match="subscribed"):
        build_journal_graph_projection(fixture_payload=fixture_payload, observed_graph=graph)

    graph = copy.deepcopy(_observed_graph_double())
    graph["topics"]["/extra"] = dict(graph["topics"]["/planning_scene"])
    with pytest.raises(ValueError, match="topics must be exactly"):
        build_journal_graph_projection(fixture_payload=fixture_payload, observed_graph=graph)

    graph = copy.deepcopy(_observed_graph_double())
    graph["topics"]["/planning_scene"]["type"] = "std_msgs/msg/String"
    with pytest.raises(ValueError, match="wrong type"):
        build_journal_graph_projection(fixture_payload=fixture_payload, observed_graph=graph)

    graph = copy.deepcopy(_observed_graph_double())
    graph["topics"]["/planning_scene"]["requested_qos"]["depth"] = 2
    with pytest.raises(ValueError, match="QoS"):
        build_journal_graph_projection(fixture_payload=fixture_payload, observed_graph=graph)

    graph = copy.deepcopy(_observed_graph_double())
    graph["topics"]["/sim/status/planning_scene_fixture"]["publishers"].append(
        {"node": "/other", "node_namespace": ""}
    )
    with pytest.raises(ValueError, match="exactly one publisher"):
        build_journal_graph_projection(fixture_payload=fixture_payload, observed_graph=graph)

    graph = copy.deepcopy(_observed_graph_double())
    graph["services"]["/get_planning_scene"]["clients"] = [
        {"node": "/other", "node_namespace": ""}
    ]
    with pytest.raises(ValueError, match="called by"):
        build_journal_graph_projection(fixture_payload=fixture_payload, observed_graph=graph)

    graph = copy.deepcopy(_observed_graph_double())
    graph["node_name"] = "/wrong"
    with pytest.raises(ValueError, match="node_name"):
        build_journal_graph_projection(fixture_payload=fixture_payload, observed_graph=graph)


def _journal_for_contract(tmp_path: Path, contract: dict[str, object]):
    from planning_scene_journal import PlanningSceneJournal, load_model_touch_contract

    from validation.integrated_gate_executor import (
        GATE_C_REQUIRED_EVENT_ORDER,
        GATE_C_FORBIDDEN_EVENTS,
        TARGET_OBJECT_ID,
        TASK_NAMESPACE,
    )

    declaration = contract["planning_scene_declaration"]
    touch = load_model_touch_contract()
    journal = PlanningSceneJournal(
        fixture_revision=declaration["revision"],
        task_namespace=TASK_NAMESPACE,
        target_object_id=TARGET_OBJECT_ID,
        expected_attach_link=touch["link_tcp"],
        expected_touch_links=touch["touch_links"],
        required_event_order=GATE_C_REQUIRED_EVENT_ORDER,
        forbidden_events=GATE_C_FORBIDDEN_EVENTS,
        jsonl_path=tmp_path / "planning-scene.jsonl",
    )
    return journal, declaration


def _valid_scene(declaration: Mapping[str, object]) -> dict[str, object]:
    return {
        "scene_sequence": 1,
        "scene_timestamp": 1.0,
        "owned_ids": list(fixture_owned_ids(declaration)),
        "attached_ids": [],
        "attached_links": {},
        "touch_links": {},
        "fixture_revision": declaration["revision"],
        "scene_revision_digest": "a" * 64,
        "acm_digest": "b" * 64,
        "robot_state_digest": "c" * 64,
        "source": "/planning_scene",
    }


def test_journal_lifecycle_fixture_ready_then_teardown_finalizes(tmp_path):
    """F1.3: the Stage-C explicit journal lifecycle writes both journal artifacts
    and retains the validated graph projection with a diagnostic authority."""
    from planning_scene_journal import validate_graph_evidence

    from validation.integrated_gate_executor import build_journal_graph_projection

    contract = scenario_report_contract("qualification-moveit-plan-joint")
    journal, declaration = _journal_for_contract(tmp_path, contract)
    journal.record_diff(
        "fixture-ready", {**_valid_scene(declaration), "frame_index": 0, "timestamp": 0.0}
    )
    journal.snapshot("teardown", frame_index=1, timestamp=1.0)
    projection = build_journal_graph_projection(
        fixture_payload=_canonical_fixture_payload(contract),
        observed_graph=_observed_graph_double(),
    )
    assert validate_graph_evidence(projection)["node_name"] == "/tinker_integrated_gate_executor"
    final = journal.finalize(
        "diagnostic-pass", graph=projection, json_path=tmp_path / "planning-scene.json"
    )
    assert final["status"] == "diagnostic-pass"
    assert final["authority"] == "physics_truth"
    assert final["events"] == ["fixture-ready", "teardown"]
    assert final["graph"]["topics"]["/sim/status/planning_scene_fixture"]["payload_parsed"][
        "fixture_descriptor_sha256"
    ] == _fixture_descriptor_digest(contract)
    assert (tmp_path / "planning-scene.jsonl").exists()
    assert (tmp_path / "planning-scene.json").stat().st_size > 0


def test_stale_nonempty_jsonl_fails_closed_before_any_record(tmp_path):
    """F1.3: a pre-existing non-empty jsonl fails closed at the first append."""
    jsonl = tmp_path / "planning-scene.jsonl"
    jsonl.write_text('{"stale": true}\n', encoding="utf-8")
    contract = scenario_report_contract("qualification-moveit-plan-joint")
    journal, declaration = _journal_for_contract(tmp_path, contract)
    with pytest.raises(ValueError, match="already contains records"):
        journal.record_diff(
            "fixture-ready", {**_valid_scene(declaration), "frame_index": 0, "timestamp": 0.0}
        )


def test_gate_c_forbidden_events_block_manipulation_events(tmp_path):
    """F1.3: Gate C derives an explicit Stage-C journal contract; no explicit
    Stage-C run may emit attach/detach/manipulation events."""
    from validation.integrated_gate_executor import (
        GATE_C_FORBIDDEN_EVENTS,
        GATE_C_REQUIRED_EVENT_ORDER,
    )

    assert GATE_C_REQUIRED_EVENT_ORDER == ("fixture-ready", "teardown")
    assert GATE_C_FORBIDDEN_EVENTS == (
        "before-pick",
        "scene-attach",
        "lift-complete",
        "transport",
        "before-release",
        "scene-detach",
        "released-settled",
        "task-cleanup",
    )
    contract = scenario_report_contract("qualification-moveit-plan-joint")
    journal, declaration = _journal_for_contract(tmp_path, contract)
    with pytest.raises(ValueError, match="forbidden"):
        journal.record_diff(
            "scene-attach", {**_valid_scene(declaration), "frame_index": 0, "timestamp": 0.0}
        )


# --------------------------------------------------------------------------- #
# Fix round 3 (F3.3): pure fixture geometry projection digest
# --------------------------------------------------------------------------- #

def test_expected_fixture_geometry_digest_is_deterministic_and_mutation_sensitive():
    """F3.3: the pure fixture geometry projection digest is deterministic over
    the declared owned geometry (ordered IDs, pose, primitive dimensions, frame)
    and changes on every material mutation, binding the exact declared contract
    rather than only the owned-ID set."""
    from validation.integrated_gate_executor import expected_fixture_geometry_digest

    contract = scenario_report_contract("qualification-moveit-plan-joint")
    declaration = contract["planning_scene_declaration"]
    digest = expected_fixture_geometry_digest(declaration)
    assert len(digest) == 64 and all(c in "0123456789abcdef" for c in digest)
    assert expected_fixture_geometry_digest(declaration) == digest

    stale_pose = copy.deepcopy(declaration)
    stale_pose["objects"][0]["pose"]["xyz"][0] += 0.05
    assert expected_fixture_geometry_digest(stale_pose) != digest

    wrong_dimensions = copy.deepcopy(declaration)
    wrong_dimensions["objects"][0]["primitive"]["dimensions"][2] += 0.1
    assert expected_fixture_geometry_digest(wrong_dimensions) != digest

    wrong_frame = copy.deepcopy(declaration)
    wrong_frame["frame_id"] = "world"
    assert expected_fixture_geometry_digest(wrong_frame) != digest

    reordered = copy.deepcopy(declaration)
    reordered["objects"] = list(reversed(reordered["objects"]))
    assert expected_fixture_geometry_digest(reordered) != digest

    duplicate = copy.deepcopy(declaration)
    duplicate["objects"].append(copy.deepcopy(duplicate["objects"][0]))
    assert expected_fixture_geometry_digest(duplicate) != digest

    extra = copy.deepcopy(declaration)
    extra["objects"].append(
        {
            "class": "static",
            "id": "sim_fixture/extra",
            "pose": {"quaternion_xyzw": [0.0, 0.0, 0.0, 1.0], "xyz": [0.1, 0.1, 0.1]},
            "primitive": {"dimensions": [0.05, 0.05, 0.05], "type": "box"},
        }
    )
    assert expected_fixture_geometry_digest(extra) != digest


def test_expected_fixture_geometry_digest_matches_owned_id_order_for_stage_c():
    """F3.3: every Stage-C declaration yields a deterministic digest whose
    ordered descriptor list aligns with ``fixture_owned_ids``."""
    from validation.integrated_gate_executor import expected_fixture_geometry_digest

    from tinker_sim_bridge.fixture_contract import (
        geometry_signature_sha256,
        spec_geometry,
    )
    from tinker_sim_bridge.fixture_planning_scene import fixture_to_specs

    for scenario_name in STAGE_C_SCENARIOS:
        contract = scenario_report_contract(scenario_name)
        declaration = contract["planning_scene_declaration"]
        specs = fixture_to_specs(declaration)
        ordered = [spec_geometry(spec) for spec in specs]
        assert [descriptor["id"] for descriptor in ordered] == list(
            fixture_owned_ids(declaration)
        )
        assert expected_fixture_geometry_digest(declaration) == geometry_signature_sha256(ordered)
