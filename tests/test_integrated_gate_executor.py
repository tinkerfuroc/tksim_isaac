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

import array
import ast
import copy
import hashlib
import json
import math
import os
import sys
import types
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
            "source_node": "/joint_state_broadcaster",
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
                "source_node": "/joint_state_broadcaster",
                "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 10},
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


def test_readiness_accepts_reordered_joint_state_after_driver_canonicalization():
    """A /joint_states broadcast in live controller order is accepted once the
    driver canonicalizes names to joint1..joint7,drive_joint with reordered
    positions/velocities and the owning joint_state_broadcaster source."""
    live_order = ["joint2", "joint3", "joint5", "joint6", "joint1", "joint4", "joint7", "drive_joint"]
    live_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
    live_velocities = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08]
    canonical = [f"joint{index}" for index in range(1, 8)] + ["drive_joint"]
    snapshot = ready_executor_snapshot()
    snapshot["joint_state"]["names"] = canonical
    snapshot["joint_state"]["positions"] = [live_positions[live_order.index(name)] for name in canonical]
    snapshot["joint_state"]["velocities"] = [live_velocities[live_order.index(name)] for name in canonical]
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is True
    assert result["reasons"] == []


def test_readiness_accepts_reordered_scene_owned_ids():
    """RED: planning-scene owned_ids must be accepted in any order (set contract).

    Live MoveIt returns the planning-scene world objects alphabetically sorted
    (e.g. ``sim_fixture/pedestal, sim_fixture/plan_blocker,
    sim_fixture/public_target``) while the scenario declares them in a different
    order.  The readiness scene check must compare owned_ids as a SET (same IDs
    regardless of order) for the world-only contract; the fixture payload vs
    scenario declaration exact-order check stays strict and is untouched here.
    The negative case proves set-equality, not "superset": dropping one owned id
    still fails.
    """
    snapshot = ready_executor_snapshot()
    snapshot["planning_scene"]["owned_ids"] = list(
        reversed(snapshot["planning_scene"]["owned_ids"])
    )
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is True
    assert result["reasons"] == []

    # Set-equality, not "superset": a snapshot missing one owned id still fails.
    snapshot = ready_executor_snapshot()
    target = snapshot["fixture"]["target_source_id"]
    owned = list(snapshot["planning_scene"]["owned_ids"])
    dropped = next(object_id for object_id in owned if object_id != target)
    snapshot["planning_scene"]["owned_ids"] = [
        object_id for object_id in owned if object_id != dropped
    ]
    result = evaluate_executor_readiness(snapshot, _config(), readiness_scenario())
    assert result["ready"] is False


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
    snapshot["joint_state"]["source_node"] = "/wrong_joint_state_source"
    snapshot["topics"]["/joint_states"]["source_node"] = "/wrong_joint_state_source"
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
    ],
)
def test_stage_c_dispatch_validates_two_scenarios(scenario_name, kind, expectation):
    """F1.6: the two Stage-C plan-only scenarios dispatch to the exact goal
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


def test_stage_c_plan_only_classify_success_emits_expectation():
    """F2.4: the Stage-C plan-only classification must not raise and must carry
    the scenario's expectation for a successful MoveGroup result.

    Regression for the live ``evidence-invalid`` crash (``name 'expectation'
    is not defined``): ``_classify_plan_only_result`` referenced ``expectation``
    after the blocked-scenario removal deleted its binding.  A real OMPL
    success (``error_code.val == 1`` with a nonempty trajectory) must classify
    ``diagnostic-pass`` and report the ``success`` expectation.
    """
    from validation.integrated_gate_executor import IntegratedGateExecutor

    executor = object.__new__(IntegratedGateExecutor)
    executor.ros = {"serialize_message": lambda message: b"serialized"}

    result = types.SimpleNamespace(
        result=types.SimpleNamespace(
            error_code=types.SimpleNamespace(val=1),
            planned_trajectory=types.SimpleNamespace(
                joint_trajectory=types.SimpleNamespace(points=[object()])
            ),
        )
    )
    spec = stage_c_dispatch(
        "qualification-moveit-plan-joint",
        scenario=readiness_scenario(
            scenario_report_contract("qualification-moveit-plan-joint")
        ),
    )
    classified = executor._classify_plan_only_result(
        "qualification-moveit-plan-joint", result, spec
    )
    assert classified["status"] == "diagnostic-pass"
    assert classified["error_code_classification"] == "success"
    assert classified["nonempty_plan"] is True
    assert classified["expectation"] == "success"


# ---------------------------------------------------------------------------
# Task 65 live blockers: pose approach target.
#
# Mirrors validation/ompl_plan_smoke.py POSE_APPROACH_Z_OFFSET=0.10: the
# generated pose goal z is target z + 0.10 (approach hover), never the
# collision-box center; the orientation is the overhead z-down
# Rz45*Rx180 quaternion (approximately x=0.9238795, y=0.3826834, z=0, w=0,
# sign-equivalent accepted), not the z-up yaw-only declaration quaternion.
# ---------------------------------------------------------------------------

def _pose_contract(spec: Mapping[str, object]) -> tuple[list[float], list[float]]:
    pose = spec["target_pose"]
    xyz = [float(value) for value in pose["xyz"]]
    quat = [float(value) for value in pose["quaternion_xyzw"]]
    return xyz, quat


def _assert_overhead_z_down_quaternion(quat: list[float]) -> None:
    """Assert the overhead z-down Rz45*Rx180 quaternion (sign-equivalent)."""
    expected = [0.9238795, 0.3826834, 0.0, 0.0]
    norm = math.sqrt(sum(float(value) ** 2 for value in quat))
    assert abs(norm - 1.0) <= 1.0e-3, quat
    dot = abs(sum(a * b for a, b in zip(quat, expected)))
    assert dot > 0.999, f"orientation {quat} is not overhead z-down Rz45*Rx180"


def test_stage_c_pose_dispatch_approaches_above_target_not_collision_center():
    """RED: the Stage-C pose goal must hover POSE_APPROACH_Z_OFFSET above the
    target object z, not point at the collision-box center."""
    from validation.ompl_goal_builders import POSE_APPROACH_Z_OFFSET

    contract = scenario_report_contract("qualification-moveit-plan-pose")
    declaration = contract["planning_scene_declaration"]
    target = next(
        record for record in declaration["objects"]
        if record["id"] == declaration["target_source_id"]
    )
    target_xyz = [float(value) for value in target["pose"]["xyz"]]

    spec = stage_c_dispatch(
        "qualification-moveit-plan-pose", scenario=readiness_scenario(contract)
    )
    xyz, quat = _pose_contract(spec)

    assert POSE_APPROACH_Z_OFFSET == pytest.approx(0.10)
    assert xyz[0] == pytest.approx(target_xyz[0])
    assert xyz[1] == pytest.approx(target_xyz[1])
    # Approach hover, not collision-box center.
    assert xyz[2] == pytest.approx(target_xyz[2] + POSE_APPROACH_Z_OFFSET), xyz
    _assert_overhead_z_down_quaternion(quat)


def test_stage_d_execute_pose_dispatch_uses_approach_offset_and_overhead_orientation():
    """RED: the Gate-D execute-pose twin shares the pose approach target
    behavior (z + POSE_APPROACH_Z_OFFSET, overhead z-down orientation)."""
    from validation.ompl_goal_builders import POSE_APPROACH_Z_OFFSET

    contract = scenario_report_contract("qualification-moveit-execute-pose")
    declaration = contract["planning_scene_declaration"]
    target = next(
        record for record in declaration["objects"]
        if record["id"] == declaration["target_source_id"]
    )
    target_xyz = [float(value) for value in target["pose"]["xyz"]]

    from validation.integrated_gate_executor import stage_d_dispatch

    spec = stage_d_dispatch(
        "qualification-moveit-execute-pose", scenario=readiness_scenario(contract)
    )
    assert spec["kind"] == "execute-pose"
    xyz, quat = _pose_contract(spec)

    assert POSE_APPROACH_Z_OFFSET == pytest.approx(0.10)
    assert xyz[0] == pytest.approx(target_xyz[0])
    assert xyz[1] == pytest.approx(target_xyz[1])
    assert xyz[2] == pytest.approx(target_xyz[2] + POSE_APPROACH_Z_OFFSET), xyz
    _assert_overhead_z_down_quaternion(quat)


def test_qualification_pose_scenario_declares_overhead_z_down_orientation():
    """RED: the generated pose goal (smoke ``build_goal`` pose mode) carries the
    overhead z-down Rz45*Rx180 orientation and hovers z + POSE_APPROACH_Z_OFFSET.
    The fixture declaration itself stays yaw-only identity; only the generated
    goal carries the overhead orientation."""
    from validation.ompl_goal_builders import POSE_APPROACH_Z_OFFSET
    from validation.ompl_plan_smoke import build_goal, load_scenario

    scenario = load_scenario(
        ROOT / "simulation/scenarios/qualification-moveit-plan-pose.json"
    )
    declaration = scenario["planning_scene"]
    target = next(
        record for record in declaration["objects"]
        if record["id"] == declaration["target_source_id"]
    )
    declared_quat = [float(value) for value in target["pose"]["quaternion_xyzw"]]
    target_xyz = [float(value) for value in target["pose"]["xyz"]]
    # Fixture stays yaw-only identity: [0, 0, 0, 1].
    assert declared_quat[0] == pytest.approx(0.0)
    assert declared_quat[1] == pytest.approx(0.0)
    assert declared_quat[2] == pytest.approx(0.0)
    assert declared_quat[3] == pytest.approx(1.0)
    # The generated pose goal carries the overhead z-down orientation and the
    # approach z-offset, not the fixture's yaw-only declaration quaternion.
    goal = build_goal("pose", scenario)
    _assert_overhead_z_down_quaternion(list(goal.orientation_xyzw))
    assert goal.position_xyz[2] == pytest.approx(
        target_xyz[2] + POSE_APPROACH_Z_OFFSET
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


def test_journal_graph_projection_accepts_no_publisher_planning_scene_input():
    """RED: /planning_scene is a subscribed input in this stack.

    The real observed shape for /planning_scene carries no local publisher
    (``publishers=[]`` and ``offered_qos={}``); only the recorder subscribes and
    its ``requested_qos`` must stay exactly RELIABLE/VOLATILE/depth 100.
    No fake QoS may be borrowed for the publisher-less input.  The projection is
    fed through the downstream ``planning_scene_journal.validate_graph_evidence``
    so the honest input-only shape is actually accepted by the journal validator.
    The input-only exception covers both PlanningScene topics; a present-but-wrong
    offered QoS on /monitored_planning_scene must still fail closed downstream
    (see test_journal_graph_projection_accepts_monitored_planning_scene_no_publisher_input).
    """
    from planning_scene_journal import validate_graph_evidence

    from validation.integrated_gate_executor import (
        JOURNAL_PLANNING_SCENE_TOPIC_QOS,
    )

    fixture_payload = _canonical_fixture_payload()
    graph = copy.deepcopy(_observed_graph_double())
    # Real shape: no local publisher -> no offered QoS on /planning_scene.
    graph["topics"]["/planning_scene"]["publishers"] = []
    graph["topics"]["/planning_scene"]["offered_qos"] = {}

    projection = build_journal_graph_projection(
        fixture_payload=fixture_payload, observed_graph=graph
    )

    entry = projection["topics"]["/planning_scene"]
    assert entry["publishers"] == []
    assert entry["offered_qos"] == {}
    assert entry["requested_qos"] == dict(JOURNAL_PLANNING_SCENE_TOPIC_QOS)
    # /monitored_planning_scene keeps its observed /move_group publisher/QoS.
    monitored = projection["topics"]["/monitored_planning_scene"]
    assert monitored["publishers"] == [{"node": "/move_group", "node_namespace": ""}]
    assert monitored["offered_qos"] == dict(JOURNAL_PLANNING_SCENE_TOPIC_QOS)

    # The downstream journal validator must accept the honest input-only
    # /planning_scene projection and preserve the empty offered QoS / publishers.
    normalized = validate_graph_evidence(projection)
    entry = normalized["topics"]["/planning_scene"]
    assert entry["publishers"] == []
    assert entry["offered_qos"] == {}
    assert entry["requested_qos"] == dict(JOURNAL_PLANNING_SCENE_TOPIC_QOS)

    # Negative: the input-only exception now covers both PlanningScene topics.
    # A present-but-wrong offered QoS on /monitored_planning_scene must still be
    # rejected by the downstream journal validator.
    graph = copy.deepcopy(_observed_graph_double())
    projection = build_journal_graph_projection(
        fixture_payload=fixture_payload, observed_graph=graph
    )
    projection["topics"]["/monitored_planning_scene"]["offered_qos"] = {
        "reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1,
    }
    with pytest.raises(ValueError, match="QoS"):
        validate_graph_evidence(projection)


def test_journal_graph_projection_accepts_monitored_planning_scene_no_publisher_input():
    """RED: /monitored_planning_scene with no publisher and empty offered QoS.

    Live move_group in this production launch does NOT publish
    /monitored_planning_scene (build_tinker_ompl_config sets no monitored topic;
    provider-manifest publishers omit it), yet the executor still subscribes to
    it as a journal recorder.  The projection must accept the honest empty
    offered QoS with an empty publisher set (requested QoS stays strict
    RELIABLE/VOLATILE/depth 100), and the downstream journal validator must
    accept it too.  A present-but-wrong offered QoS on /monitored_planning_scene
    still fails closed.
    """
    from planning_scene_journal import validate_graph_evidence

    from validation.integrated_gate_executor import (
        JOURNAL_PLANNING_SCENE_TOPIC_QOS,
    )

    fixture_payload = _canonical_fixture_payload()
    graph = copy.deepcopy(_observed_graph_double())
    graph["topics"]["/monitored_planning_scene"]["publishers"] = []
    graph["topics"]["/monitored_planning_scene"]["offered_qos"] = {}

    projection = build_journal_graph_projection(
        fixture_payload=fixture_payload, observed_graph=graph
    )
    monitored = projection["topics"]["/monitored_planning_scene"]
    assert monitored["publishers"] == []
    assert monitored["offered_qos"] == {}
    assert monitored["requested_qos"] == dict(JOURNAL_PLANNING_SCENE_TOPIC_QOS)

    normalized = validate_graph_evidence(projection)
    monitored = normalized["topics"]["/monitored_planning_scene"]
    assert monitored["publishers"] == []
    assert monitored["offered_qos"] == {}
    assert monitored["requested_qos"] == dict(JOURNAL_PLANNING_SCENE_TOPIC_QOS)

    # A present-but-wrong offered QoS on /monitored_planning_scene fails closed.
    graph = copy.deepcopy(_observed_graph_double())
    projection = build_journal_graph_projection(
        fixture_payload=fixture_payload, observed_graph=graph
    )
    projection["topics"]["/monitored_planning_scene"]["offered_qos"] = {
        "reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL", "depth": 1,
    }
    with pytest.raises(ValueError, match="QoS"):
        validate_graph_evidence(projection)


def test_journal_graph_projection_accepts_non_move_group_publisher_planning_scene_input():
    """RED: /planning_scene with a real non-/move_group publisher and no offered QoS.

    Live MoveIt2 shows ``pick_and_place``'s internal ``pick_place_group_node``
    publishing /planning_scene (via PlanningSceneInterface/MoveGroupInterface);
    the driver's ``_select_endpoint_qos`` finds no /move_group publisher, so the
    observed ``offered_qos`` is ``{}``.  The projection builder must accept ANY
    nonempty publisher set with ``offered_qos=={}`` on /planning_scene (the
    requested QoS stays strict RELIABLE/VOLATILE/depth 100) and preserve both
    the publisher metadata and the empty offered QoS; the downstream journal
    validator must do the same.  A /planning_scene with a present-but-WRONG
    offered QoS still fails closed.
    """
    from planning_scene_journal import validate_graph_evidence

    from validation.integrated_gate_executor import JOURNAL_PLANNING_SCENE_TOPIC_QOS

    fixture_payload = _canonical_fixture_payload()
    graph = copy.deepcopy(_observed_graph_double())
    # Live pick_and_place publishes /planning_scene; no /move_group publisher
    # exists, so the driver observes no offered QoS.
    graph["topics"]["/planning_scene"]["publishers"] = [
        {"node": "/pick_place_group_node", "node_namespace": ""}
    ]
    graph["topics"]["/planning_scene"]["offered_qos"] = {}

    projection = build_journal_graph_projection(
        fixture_payload=fixture_payload, observed_graph=graph
    )
    entry = projection["topics"]["/planning_scene"]
    assert entry["publishers"] == [{"node": "/pick_place_group_node", "node_namespace": ""}]
    assert entry["offered_qos"] == {}
    assert entry["requested_qos"] == dict(JOURNAL_PLANNING_SCENE_TOPIC_QOS)

    # The downstream journal validator must also accept and preserve the shape.
    normalized = validate_graph_evidence(projection)
    entry = normalized["topics"]["/planning_scene"]
    assert entry["publishers"] == [{"node": "/pick_place_group_node", "node_namespace": ""}]
    assert entry["offered_qos"] == {}
    assert entry["requested_qos"] == dict(JOURNAL_PLANNING_SCENE_TOPIC_QOS)

    # Negative: a present-but-wrong offered QoS (TRANSIENT_LOCAL instead of
    # VOLATILE) is still rejected even with a nonempty publisher set.
    graph = copy.deepcopy(_observed_graph_double())
    graph["topics"]["/planning_scene"]["publishers"] = [
        {"node": "/pick_place_group_node", "node_namespace": ""}
    ]
    graph["topics"]["/planning_scene"]["offered_qos"] = {
        "reliability": "RELIABLE",
        "durability": "TRANSIENT_LOCAL",
        "depth": 100,
    }
    with pytest.raises(ValueError, match="QoS"):
        build_journal_graph_projection(
            fixture_payload=fixture_payload, observed_graph=graph
        )


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


# ---------------------------------------------------------------------------
# Task 5 (Gate D): fake executor transaction contract, Stage-D dispatch,
# split-path UUID/status helpers, retreat/gripper constants, D journal order,
# Gate-E negative stub.
# ---------------------------------------------------------------------------


class FakeIntegratedExecutor:
    """Task-5 test-local deterministic state-machine double.

    Models only the observable transaction contract; it never pretends to be a
    ROS client.
    """

    def run_cancel_sequence(
        self, *, planning_goal_id: str, execute_goal_id: str, timeout_s: float
    ) -> dict[str, object]:
        return {
            "planning_goal_id": planning_goal_id,
            "execute_goal_id": execute_goal_id,
            "goals_canceling": [execute_goal_id],
            "quiescent": True, "elapsed_s": min(timeout_s, 0.1),
            "terminal_status": "canceled",
            "cancel_endpoint": "/execute_trajectory",
            "move_group_plan_only": True,
            "events": ["plan-only", "execute-trajectory-start", "cancel-requested", "quiescent", "canceled"],
        }

    def run_safety_sequence(self, *, goal_id: str, stop_timeout_s: float) -> dict[str, object]:
        return {
            "goal_id": goal_id,
            "events": ["execution-start", "effective-stop", "operator-clear"],
            "effective_stop": True, "terminal_status": "aborted",
            "operator_clear": True, "sent_goal_ids": [goal_id], "resumed": False,
        }

    def run_cartesian_retreat(self, *, target_frame: str, distance_m: float) -> dict[str, object]:
        return {
            "endpoint": "/cartesian_move_action", "target_frame": target_frame,
            "distance_m": distance_m, "collision_checking": True,
            "command_gateway_bypassed": False,
        }

    def run_gripper_sequence(self, *, open_first: bool) -> dict[str, object]:
        return {
            "endpoint": "/xarm_gripper/gripper_action",
            "commands": ["open", "close"] if open_first else ["close", "open"],
            "native_action": True,
        }

    def run_pick_place_negative(self, scenario: str) -> dict[str, object]:
        if scenario == "cancel-approach":
            return {"events": ["approach-start", "cancel"], "release_stage_started": False,
                    "released": False}
        if scenario == "cancel-transport":
            return {"events": ["approach-start", "lift-complete", "cancel"],
                    "release_stage_started": False, "released": False}
        raise ValueError(f"unsupported test scenario: {scenario}")


@pytest.fixture
def fake_executor() -> FakeIntegratedExecutor:
    return FakeIntegratedExecutor()


def test_cancel_records_distinct_plan_and_execute_ids(fake_executor):
    trace = fake_executor.run_cancel_sequence(planning_goal_id="plan-42", execute_goal_id="exec-42", timeout_s=2.0)
    assert trace["planning_goal_id"] == "plan-42"
    assert trace["execute_goal_id"] == "exec-42"
    assert trace["planning_goal_id"] != trace["execute_goal_id"]
    assert trace["goals_canceling"] == ["exec-42"]
    assert trace["quiescent"] is True
    assert trace["elapsed_s"] <= 2.0


def test_cancel_sequence_cannot_emit_later_execution_stage(fake_executor):
    trace = fake_executor.run_cancel_sequence(planning_goal_id="plan-43", execute_goal_id="exec-43", timeout_s=2.0)
    assert trace["terminal_status"] == "canceled"
    assert trace["events"][-1] == "canceled"
    assert "release" not in trace["events"]
    assert "retreat" not in trace["events"]


def test_cancel_targets_execute_trajectory_goal_not_move_group_goal(fake_executor):
    trace = fake_executor.run_cancel_sequence(planning_goal_id="plan-46", execute_goal_id="exec-46", timeout_s=2.0)
    assert trace["cancel_endpoint"] == "/execute_trajectory"
    assert trace["execute_goal_id"] == "exec-46"
    assert trace["planning_goal_id"] != trace["execute_goal_id"]
    assert trace["goals_canceling"] == [trace["execute_goal_id"]]
    assert trace["move_group_plan_only"] is True


def test_safety_sequence_waits_effective_stop_before_clear(fake_executor):
    trace = fake_executor.run_safety_sequence(goal_id="goal-44", stop_timeout_s=2.0)
    assert trace["events"].index("effective-stop") < trace["events"].index("operator-clear")
    assert trace["effective_stop"] is True
    assert trace["terminal_status"] == "aborted"


def test_safety_clear_does_not_resend_old_goal(fake_executor):
    trace = fake_executor.run_safety_sequence(goal_id="goal-45", stop_timeout_s=2.0)
    assert trace["operator_clear"] is True
    assert trace["sent_goal_ids"] == ["goal-45"]
    assert trace["resumed"] is False


def test_cartesian_retreat_uses_production_action(fake_executor):
    trace = fake_executor.run_cartesian_retreat(target_frame="base_link", distance_m=0.10)
    assert trace["endpoint"] == "/cartesian_move_action"
    assert trace["collision_checking"] is True
    assert trace["command_gateway_bypassed"] is False


def test_gripper_sequence_uses_native_action(fake_executor):
    trace = fake_executor.run_gripper_sequence(open_first=True)
    assert trace["endpoint"] == "/xarm_gripper/gripper_action"
    assert trace["commands"] == ["open", "close"]
    assert trace["native_action"] is True


# --- Stage-D dispatch contract ---------------------------------------------

STAGE_D_SCENARIO_NAMES = (
    "qualification-moveit-execute-joint",
    "qualification-moveit-execute-pose",
    "qualification-moveit-cartesian-retreat",
    "qualification-moveit-gripper",
    "qualification-moveit-cancel",
    "qualification-moveit-safety",
)


@pytest.mark.parametrize(
    ("scenario_name", "kind", "polarity"),
    [
        ("qualification-moveit-execute-joint", "execute-joint", "positive"),
        ("qualification-moveit-execute-pose", "execute-pose", "positive"),
        ("qualification-moveit-cartesian-retreat", "retreat", "positive"),
        ("qualification-moveit-gripper", "gripper", "positive"),
        ("qualification-moveit-cancel", "cancel", "cancel"),
        ("qualification-moveit-safety", "safety", "safety"),
    ],
)
def test_stage_d_dispatch_validates_six_scenarios(scenario_name, kind, polarity):
    """D1: the six Stage-D scenarios dispatch to the exact handler kind and
    declared polarity with the exact expected_physical list."""
    from validation.integrated_gate_executor import (
        STAGE_D_EXPECTED_PHYSICAL,
        stage_d_dispatch,
    )

    contract = scenario_report_contract(scenario_name)
    spec = stage_d_dispatch(scenario_name, scenario=readiness_scenario(contract))
    assert spec["scenario_id"] == scenario_name
    assert spec["kind"] == kind
    assert spec["polarity"] == polarity
    assert spec["expected_physical"] == list(STAGE_D_EXPECTED_PHYSICAL[scenario_name])
    assert spec["forbidden_endpoints"] == ["/isaac_joint_commands"]
    if kind == "execute-joint":
        assert spec["joints"] == list(Q_OUTBOUND)
    elif kind == "execute-pose":
        xyz = spec["target_pose"]["xyz"]
        quaternion = spec["target_pose"]["quaternion_xyzw"]
        assert len(xyz) == 3 and all(math.isfinite(float(value)) for value in xyz)
        assert len(quaternion) == 4 and all(math.isfinite(float(value)) for value in quaternion)
        norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
        assert abs(norm - 1.0) <= 1.0e-3


def test_stage_d_dispatch_rejects_non_d_scenario():
    """D2: C/E-stage and unknown scenarios fail closed."""
    from validation.integrated_gate_executor import stage_d_dispatch

    contract = scenario_report_contract("qualification-moveit-plan-joint")
    with pytest.raises(ValueError, match="stage must be D"):
        stage_d_dispatch("qualification-moveit-plan-joint", scenario=readiness_scenario(contract))
    contract = scenario_report_contract("qualification-pick-place-positive")
    with pytest.raises(ValueError, match="stage must be D"):
        stage_d_dispatch("qualification-pick-place-positive", scenario=readiness_scenario(contract))
    with pytest.raises(ValueError, match="scenario_id does not match"):
        stage_d_dispatch("qualification-moveit-cancel", scenario=readiness_scenario(contract))


def test_stage_d_dispatch_rejects_mutated_polarity_and_expected_physical():
    """D3: mutating polarity or expected_physical fails closed."""
    from validation.integrated_gate_executor import stage_d_dispatch

    contract = scenario_report_contract("qualification-moveit-cancel")
    scenario = readiness_scenario(contract)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["acceptance"]["polarity"] = "positive"
    with pytest.raises(ValueError, match="polarity"):
        stage_d_dispatch("qualification-moveit-cancel", scenario=mutated)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["expected_physical"] = ["wrong"]
    with pytest.raises(ValueError, match="expected_physical"):
        stage_d_dispatch("qualification-moveit-cancel", scenario=mutated)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["forbidden_endpoints"] = ["/other"]
    with pytest.raises(ValueError, match="forbidden_endpoints"):
        stage_d_dispatch("qualification-moveit-cancel", scenario=mutated)


def test_stage_d_dispatch_rejects_mutated_execution_profile():
    from validation.integrated_gate_executor import stage_d_dispatch

    contract = scenario_report_contract("qualification-moveit-safety")
    scenario = readiness_scenario(contract)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["execution_profile"] = "other"
    with pytest.raises(ValueError, match="execution_profile"):
        stage_d_dispatch("qualification-moveit-safety", scenario=mutated)


# --- Stage-E dispatch contract (Task 6 / Gate E) ----------------------------

STAGE_E_SCENARIO_NAMES = (
    "qualification-pick-place-positive",
    "qualification-pick-place-blocked-approach",
    "qualification-pick-place-unreachable-grasp",
    "qualification-pick-place-malformed-back",
    "qualification-pick-place-cancel-approach",
    "qualification-pick-place-cancel-transport",
    "qualification-pick-place-safety-transport",
    "qualification-pick-place-occupied-place",
)

STAGE_E_KIND_EXPECTED = {
    "qualification-pick-place-positive": "positive",
    "qualification-pick-place-blocked-approach": "blocked-approach",
    "qualification-pick-place-unreachable-grasp": "unreachable-grasp",
    "qualification-pick-place-malformed-back": "malformed-back",
    "qualification-pick-place-cancel-approach": "cancel-approach",
    "qualification-pick-place-cancel-transport": "cancel-transport",
    "qualification-pick-place-safety-transport": "safety-transport",
    "qualification-pick-place-occupied-place": "occupied-place",
}

STAGE_E_TRIGGER_TIMEOUTS = {
    "qualification-pick-place-positive": None,
    "qualification-pick-place-blocked-approach": 10.0,
    "qualification-pick-place-unreachable-grasp": 10.0,
    "qualification-pick-place-malformed-back": 5.0,
    "qualification-pick-place-cancel-approach": 10.0,
    "qualification-pick-place-cancel-transport": 15.0,
    "qualification-pick-place-safety-transport": 15.0,
    "qualification-pick-place-occupied-place": 15.0,
}


@pytest.mark.parametrize("scenario_name", STAGE_E_SCENARIO_NAMES)
def test_stage_e_dispatch_validates_eight_scenarios(scenario_name):
    """E1: all eight Stage-E scenarios dispatch to the exact kind with the
    declared polarity, expected_physical, expected_negative, trigger timeout,
    and the fixed-target geometry."""
    from validation.integrated_gate_executor import (
        STAGE_E_EXPECTED_PHYSICAL,
        stage_e_dispatch,
    )

    contract = scenario_report_contract(scenario_name)
    spec = stage_e_dispatch(scenario_name, scenario=readiness_scenario(contract))
    kind = STAGE_E_KIND_EXPECTED[scenario_name]
    assert spec["scenario_id"] == scenario_name
    assert spec["kind"] == kind
    assert spec["polarity"] == ("positive" if kind == "positive" else "negative")
    assert spec["expected_physical"] == list(STAGE_E_EXPECTED_PHYSICAL[scenario_name])
    assert spec["forbidden_endpoints"] == ["/isaac_joint_commands"]
    assert spec["trigger_timeout_s"] == STAGE_E_TRIGGER_TIMEOUTS[scenario_name]
    assert spec["geometry"]["grasp_tcp_xyz"] == [0.65, 0.0, 0.72]
    assert spec["geometry"]["object_root_xyz"] == [0.65, 0.0, 0.60]
    assert spec["geometry"]["place_target_point"] == {
        "frame_id": "base_link", "xyz": [0.85, 0.0, 0.72],
    }
    assert spec["geometry"]["place_orientation_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    if kind != "positive":
        assert spec["expected_negative"]["required"]
        assert spec["expected_negative"]["forbidden"]
    if kind == "malformed-back":
        assert spec["back_positions"] == [0.2, -0.2, 0.15, 0.3, -0.15, 0.2]


def test_stage_e_dispatch_rejects_non_e_scenario():
    """E2: C/D-stage and unknown scenarios fail closed."""
    from validation.integrated_gate_executor import stage_e_dispatch

    contract = scenario_report_contract("qualification-moveit-execute-joint")
    with pytest.raises(ValueError, match="stage must be E"):
        stage_e_dispatch(
            "qualification-moveit-execute-joint", scenario=readiness_scenario(contract)
        )
    contract = scenario_report_contract("qualification-pick-place-positive")
    with pytest.raises(ValueError, match="scenario_id does not match"):
        stage_e_dispatch(
            "qualification-pick-place-cancel-transport", scenario=readiness_scenario(contract)
        )
    scenario = readiness_scenario(contract)
    scenario["id"] = "other"
    with pytest.raises(ValueError, match="not one of the Stage-E"):
        stage_e_dispatch("other", scenario=scenario)


def test_stage_e_dispatch_rejects_mutated_polarity_and_expected_negative():
    """E3: mutating polarity/expected_physical/expected_negative/forbidden fails closed."""
    from validation.integrated_gate_executor import stage_e_dispatch

    contract = scenario_report_contract("qualification-pick-place-cancel-transport")
    scenario = readiness_scenario(contract)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["acceptance"]["polarity"] = "positive"
    with pytest.raises(ValueError, match="polarity"):
        stage_e_dispatch("qualification-pick-place-cancel-transport", scenario=mutated)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["expected_physical"] = ["wrong"]
    with pytest.raises(ValueError, match="expected_physical"):
        stage_e_dispatch("qualification-pick-place-cancel-transport", scenario=mutated)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["expected_negative"]["required"] = ["wrong"]
    with pytest.raises(ValueError, match="expected_negative"):
        stage_e_dispatch("qualification-pick-place-cancel-transport", scenario=mutated)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["forbidden_endpoints"] = ["/other"]
    with pytest.raises(ValueError, match="forbidden_endpoints"):
        stage_e_dispatch("qualification-pick-place-cancel-transport", scenario=mutated)


def test_stage_e_dispatch_rejects_mutated_execution_profile():
    from validation.integrated_gate_executor import stage_e_dispatch

    contract = scenario_report_contract("qualification-pick-place-occupied-place")
    scenario = readiness_scenario(contract)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["execution_profile"] = "other"
    with pytest.raises(ValueError, match="execution_profile"):
        stage_e_dispatch("qualification-pick-place-occupied-place", scenario=mutated)


def test_stage_e_dispatch_rejects_malformed_back_and_trigger_timeout_mutation():
    from validation.integrated_gate_executor import stage_e_dispatch

    contract = scenario_report_contract("qualification-pick-place-malformed-back")
    scenario = readiness_scenario(contract)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["back_positions"] = [0.1] * 7
    with pytest.raises(ValueError, match="back_positions"):
        stage_e_dispatch("qualification-pick-place-malformed-back", scenario=mutated)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["trigger_timeout_s"] = 99.0
    with pytest.raises(ValueError, match="trigger_timeout_s"):
        stage_e_dispatch("qualification-pick-place-malformed-back", scenario=mutated)


def test_stage_e_dispatch_rejects_positive_trigger_timeout_mutation():
    from validation.integrated_gate_executor import stage_e_dispatch

    contract = scenario_report_contract("qualification-pick-place-positive")
    scenario = readiness_scenario(contract)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["trigger_timeout_s"] = 10.0
    with pytest.raises(ValueError, match="trigger_timeout_s"):
        stage_e_dispatch("qualification-pick-place-positive", scenario=mutated)


def test_f52_d_wait_timeout_defaults_are_10_and_malformed_overrides_fail_closed():
    """F5.2: bounded production-real D wait defaults and fail-closed overrides."""
    from validation.integrated_gate_executor import IntegratedGateExecutor

    def _bare_executor(config):
        executor = object.__new__(IntegratedGateExecutor)
        executor.config = config
        return executor

    # No overrides: the production defaults are exactly 10.0 seconds for both
    # the FJT wait and the motion-trigger wait.
    executor = _bare_executor({"thresholds": {}})
    assert executor._threshold_timeout("fjt_wait_timeout_s", 10.0) == 10.0
    assert executor._threshold_timeout("motion_trigger_timeout_s", 10.0) == 10.0

    # Explicit finite positive scenario overrides remain authoritative.
    executor = _bare_executor(
        {"thresholds": {"fjt_wait_timeout_s": 3.0, "motion_trigger_timeout_s": 4.5}}
    )
    assert executor._threshold_timeout("fjt_wait_timeout_s", 10.0) == 3.0
    assert executor._threshold_timeout("motion_trigger_timeout_s", 10.0) == 4.5

    # Malformed / non-finite / boolean / zero / negative overrides fail closed.
    malformed = (
        True, False, 0.0, -1.0, float("nan"), float("inf"), float("-inf"),
        "not-a-number", None, [3.0], {"s": 1},
    )
    for key in ("fjt_wait_timeout_s", "motion_trigger_timeout_s"):
        for bad in malformed:
            executor = _bare_executor({"thresholds": {key: bad}})
            with pytest.raises(ValueError, match="finite positive number"):
                executor._threshold_timeout(key, 10.0)


def test_join_key_retries_on_stale_but_valid_key():
    """F5.4: a valid-but-non-advancing join key waits bounded for the next
    advancing truth frame instead of emitting a spurious ``no-join-key``.

    Two journal snapshots landing inside one truth frame observe the same
    (frame_index, timestamp); the executor must wait for the next advancing
    frame (bounded), while a missing provider or a genuinely stalled/malformed
    stream still fails closed.
    """
    from validation.integrated_gate_executor import IntegratedGateExecutor

    calls = {"n": 0}
    stale_calls = {"n": 0}

    def _advancing_provider():
        calls["n"] += 1
        return (calls["n"] * 10, float(calls["n"]))

    # Fresh executor: first call succeeds immediately (no previous key), the
    # strict-increase guarantee is preserved for an always-advancing stream.
    executor = object.__new__(IntegratedGateExecutor)
    executor._last_join_key = None
    executor.join_key_provider = _advancing_provider
    assert executor._join_key() == (10, 1.0)
    assert executor._join_key() == (20, 2.0)
    assert calls["n"] == 2

    # A stale-but-valid key (same frame/timestamp as the previous capture) must
    # retry within a bounded window until the truth advances.
    def _stale_then_advance():
        stale_calls["n"] += 1
        if stale_calls["n"] <= 2:
            return (5, 1.0)  # stale vs the (5, 1.0) baseline and against itself
        return (6, 2.0)  # advancing

    executor = object.__new__(IntegratedGateExecutor)
    executor._last_join_key = (5, 1.0)
    executor.join_key_provider = _stale_then_advance
    key = executor._join_key()
    assert key == (6, 2.0), key
    assert stale_calls["n"] >= 3, stale_calls["n"]

    # A missing provider still fails closed immediately.
    executor = object.__new__(IntegratedGateExecutor)
    executor._last_join_key = None
    executor.join_key_provider = None
    assert executor._join_key() is None


def test_stage_e_dispatch_rejects_blocked_approach_goal_mutation():
    from validation.integrated_gate_executor import stage_e_dispatch

    contract = scenario_report_contract("qualification-pick-place-blocked-approach")
    scenario = readiness_scenario(contract)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["goal"]["target_tcp_xyz"] = [0.1, 0.0, 0.1]
    with pytest.raises(ValueError, match="target_tcp_xyz"):
        stage_e_dispatch("qualification-pick-place-blocked-approach", scenario=mutated)
    mutated = copy.deepcopy(scenario)
    mutated["integrated"]["goal"]["approach"] = "sideways"
    with pytest.raises(ValueError, match="approach"):
        stage_e_dispatch("qualification-pick-place-blocked-approach", scenario=mutated)


def test_e_stage_event_order_is_scenario_specific():
    """E4: E journals use the exact per-scenario event order (never POSITIVE_ORDER
    for negatives and never the Gate C/D order)."""
    from validation.integrated_gate_executor import (
        GATE_C_REQUIRED_EVENT_ORDER,
        _e_stage_event_order,
    )

    assert _e_stage_event_order({"integrated": {"stage": "C"}}) == GATE_C_REQUIRED_EVENT_ORDER
    assert _e_stage_event_order({"integrated": {"stage": "D"}}) == GATE_C_REQUIRED_EVENT_ORDER
    orders = {
        "qualification-pick-place-positive": (
            "fixture-ready", "before-pick", "scene-attach", "lift-complete",
            "transport", "before-release", "scene-detach", "released-settled",
            "teardown",
        ),
        "qualification-pick-place-blocked-approach": (
            "fixture-ready", "before-pick", "pick-terminal", "teardown",
        ),
        "qualification-pick-place-unreachable-grasp": (
            "fixture-ready", "before-pick", "pick-terminal", "teardown",
        ),
        "qualification-pick-place-malformed-back": ("fixture-ready", "teardown"),
        "qualification-pick-place-cancel-approach": (
            "fixture-ready", "before-pick", "approach-start", "cancel-requested",
            "quiescent", "teardown",
        ),
        "qualification-pick-place-cancel-transport": (
            "fixture-ready", "before-pick", "scene-attach", "lift-complete",
            "transport", "cancel-requested", "quiescent", "teardown",
        ),
        "qualification-pick-place-safety-transport": (
            "fixture-ready", "before-pick", "scene-attach", "lift-complete",
            "transport", "effective-stop", "operator-clear", "quiescent", "teardown",
        ),
        "qualification-pick-place-occupied-place": (
            "fixture-ready", "before-pick", "scene-attach", "lift-complete",
            "transport", "place-goal-accepted", "cancel-requested", "quiescent",
            "teardown",
        ),
    }
    for scenario_name, order in orders.items():
        assert (
            _e_stage_event_order({"id": scenario_name, "integrated": {"stage": "E"}})
            == order
        )


def test_e_forbidden_events_are_scenario_specific():
    from validation.integrated_gate_executor import _e_forbidden_events

    transport_block = (
        "before-release", "scene-detach", "released-settled", "task-cleanup",
    )
    assert _e_forbidden_events(
        {"id": "qualification-pick-place-positive", "integrated": {"stage": "E"}}
    ) == ()
    assert _e_forbidden_events(
        {"id": "qualification-pick-place-blocked-approach", "integrated": {"stage": "E"}}
    ) == (
        "scene-attach", "lift-complete", "transport", "before-release",
        "scene-detach", "released-settled", "task-cleanup",
    )
    assert _e_forbidden_events(
        {"id": "qualification-pick-place-malformed-back", "integrated": {"stage": "E"}}
    ) == (
        "before-pick", "scene-attach", "lift-complete", "transport",
        "before-release", "scene-detach", "released-settled", "task-cleanup",
    )
    assert _e_forbidden_events(
        {"id": "qualification-pick-place-cancel-approach", "integrated": {"stage": "E"}}
    ) == (
        "scene-attach", "lift-complete", "transport", "before-release",
        "scene-detach", "released-settled", "task-cleanup",
    )
    for name in (
        "qualification-pick-place-cancel-transport",
        "qualification-pick-place-safety-transport",
        "qualification-pick-place-occupied-place",
    ):
        assert _e_forbidden_events(
            {"id": name, "integrated": {"stage": "E"}}
        ) == transport_block


def test_e_positive_order_matches_journal_positive_order():
    from planning_scene_journal import POSITIVE_ORDER

    from validation.integrated_gate_executor import STAGE_E_REQUIRED_EVENT_ORDER

    assert STAGE_E_REQUIRED_EVENT_ORDER["positive"] == POSITIVE_ORDER


def _e_journal_for(tmp_path: Path, contract: dict[str, object], *, kind: str):
    from planning_scene_journal import (
        PlanningSceneJournal,
        load_model_touch_contract,
    )

    from validation.integrated_gate_executor import (
        STAGE_E_FORBIDDEN_EVENTS,
        STAGE_E_REQUIRED_EVENT_ORDER,
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
        required_event_order=STAGE_E_REQUIRED_EVENT_ORDER[kind],
        forbidden_events=STAGE_E_FORBIDDEN_EVENTS[kind],
        jsonl_path=tmp_path / "planning-scene.jsonl",
    )
    return journal, declaration


def _e_scene(
    declaration: Mapping[str, object],
    *,
    seq: int,
    world_ids: Sequence[str],
    attached_ids: Sequence[str] = (),
):
    from planning_scene_journal import load_model_touch_contract

    from validation.integrated_gate_executor import TARGET_OBJECT_ID

    touch = load_model_touch_contract()
    attached_links: dict[str, str] = {}
    touch_links: dict[str, list[str]] = {}
    for object_id in attached_ids:
        attached_links[object_id] = touch["link_tcp"]
        touch_links[object_id] = list(touch["touch_links"])
    return {
        "scene_sequence": int(seq),
        "scene_timestamp": float(seq),
        "frame_index": int(seq),
        "timestamp": float(seq),
        "owned_ids": [str(object_id) for object_id in world_ids],
        "attached_ids": [str(object_id) for object_id in attached_ids],
        "attached_links": attached_links,
        "touch_links": touch_links,
        "fixture_revision": declaration["revision"],
        "scene_revision_digest": "a" * 64,
        "acm_digest": "b" * 64,
        "robot_state_digest": "c" * 64,
        "source": "/planning_scene",
    }


def test_e_positive_journal_finalizes_exact_positive_order(tmp_path):
    """E5: the positive E journal finalizes only with the exact POSITIVE_ORDER."""
    from validation.integrated_gate_executor import STAGE_E_REQUIRED_EVENT_ORDER

    contract = scenario_report_contract("qualification-pick-place-positive")
    journal, declaration = _e_journal_for(tmp_path, contract, kind="positive")
    fixture = list(fixture_owned_ids(declaration))
    target = "pick_and_place/object_mesh"
    journal.record_diff(
        "fixture-ready", _e_scene(declaration, seq=1, world_ids=fixture + [target])
    )
    journal.record_diff(
        "before-pick", _e_scene(declaration, seq=2, world_ids=fixture + [target])
    )
    journal.record_diff(
        "scene-attach",
        _e_scene(declaration, seq=3, world_ids=fixture, attached_ids=[target]),
    )
    journal.snapshot("lift-complete", frame_index=4, timestamp=4.0)
    journal.snapshot("transport", frame_index=5, timestamp=5.0)
    journal.snapshot("before-release", frame_index=6, timestamp=6.0)
    journal.record_diff(
        "scene-detach", _e_scene(declaration, seq=7, world_ids=fixture + [target])
    )
    journal.snapshot("released-settled", frame_index=8, timestamp=8.0)
    journal.snapshot("teardown", frame_index=9, timestamp=9.0)
    final = journal.finalize("diagnostic-pass")
    assert final["status"] == "diagnostic-pass"
    assert final["events"] == list(STAGE_E_REQUIRED_EVENT_ORDER["positive"])
    assert final["authority"] == "physics_truth"


def test_e_blocked_approach_journal_forbids_scene_attach(tmp_path):
    """E6: a blocked-approach journal rejects a forbidden scene-attach transition."""
    from validation.integrated_gate_executor import STAGE_E_REQUIRED_EVENT_ORDER

    contract = scenario_report_contract("qualification-pick-place-blocked-approach")
    journal, declaration = _e_journal_for(tmp_path, contract, kind="blocked-approach")
    fixture = list(fixture_owned_ids(declaration))
    target = "pick_and_place/object_mesh"
    journal.record_diff(
        "fixture-ready", _e_scene(declaration, seq=1, world_ids=fixture + [target])
    )
    journal.record_diff(
        "before-pick", _e_scene(declaration, seq=2, world_ids=fixture + [target])
    )
    with pytest.raises(ValueError, match="forbidden"):
        journal.record_diff(
            "scene-attach",
            _e_scene(declaration, seq=3, world_ids=fixture, attached_ids=[target]),
        )
    journal.snapshot("pick-terminal", frame_index=3, timestamp=3.0)
    journal.snapshot("teardown", frame_index=4, timestamp=4.0)
    final = journal.finalize("diagnostic-pass")
    assert final["events"] == list(STAGE_E_REQUIRED_EVENT_ORDER["blocked-approach"])


def test_e_malformed_back_journal_is_fixture_ready_teardown(tmp_path):
    """E7: the malformed-back journal never records before-pick (forbidden)."""
    from validation.integrated_gate_executor import STAGE_E_REQUIRED_EVENT_ORDER

    contract = scenario_report_contract("qualification-pick-place-malformed-back")
    journal, declaration = _e_journal_for(tmp_path, contract, kind="malformed-back")
    fixture = list(fixture_owned_ids(declaration))
    journal.record_diff(
        "fixture-ready", _e_scene(declaration, seq=1, world_ids=fixture)
    )
    with pytest.raises(ValueError, match="forbidden"):
        journal.snapshot("before-pick", frame_index=2, timestamp=2.0)
    journal.snapshot("teardown", frame_index=2, timestamp=2.0)
    final = journal.finalize("diagnostic-pass")
    assert final["events"] == list(STAGE_E_REQUIRED_EVENT_ORDER["malformed-back"])


def test_occupied_place_has_distinct_fixture_owner():
    """E8: the occupied-place occupant is fixture-owned and distinct from the
    pick target source, matching the place region."""
    contract = scenario_report_contract("qualification-pick-place-occupied-place")
    target_id = contract["integrated"]["goal"]["target_object_id"]
    objects = contract["scenario_mapping"]["declaration"]["objects"]
    occupant = next(item for item in objects if item.get("role") == "occupied-place")
    assert target_id == "pick_and_place/object_mesh"
    assert occupant["owner"] == "sim_fixture"
    assert occupant["planning_scene_id"].startswith("sim_fixture/")
    assert occupant["planning_scene_id"] != contract["integrated"]["goal"]["target_source_id"]
    assert occupant["region"] == contract["integrated"]["goal"]["place_region"]


def test_e_status_domains_are_distinct():
    """E9: action-client GoalStatus and Pick/Place Result.status are two domains."""
    from validation.integrated_gate_executor import (
        PICK_PLACE_RESULT_NAMES,
        _execute_status_name,
        _pick_place_result_name,
    )

    assert _execute_status_name(4) == "succeeded"
    assert _execute_status_name(5) == "canceled"
    assert _execute_status_name(6) == "aborted"
    # tinker_arm_msgs Pick/Place Result.status: 0 success .. 9 internal_error.
    assert _pick_place_result_name(0) == "success"
    assert _pick_place_result_name(4) == "canceled"
    assert _pick_place_result_name(5) == "safety_stop"
    assert _pick_place_result_name(9) == "internal_error"
    assert PICK_PLACE_RESULT_NAMES[4] == "canceled"
    for bad in (10, -1, 3.5, True, "4", None):
        with pytest.raises(ValueError):
            _pick_place_result_name(bad)


def test_receipt_window_fjt_selection_is_sequence_bounded_not_causal():
    """E10: FJT correlation is a receipt-window (seq-bounded), never a UUID claim."""
    from validation.integrated_gate_executor import (
        _first_fjt_goal_after_acceptance,
        _next_fjt_goal,
    )

    base = {"fjt_seq": 10}
    entries = [
        {"goal_uuid": "a", "status": 2, "seq": 11},
        {"goal_uuid": "b", "status": 1, "seq": 12},
        {"goal_uuid": "c", "status": 2, "seq": 13},
        {"goal_uuid": "d", "status": 2, "seq": 14},
    ]
    first = _first_fjt_goal_after_acceptance(entries, base=base)
    assert first["goal_uuid"] == "a"
    assert first["status"] == 2
    assert first["seq"] == 11
    next_goal = _next_fjt_goal(entries, after_seq=first["seq"])
    assert next_goal["goal_uuid"] == "c"
    assert _first_fjt_goal_after_acceptance([], base=base) is None
    assert _next_fjt_goal(entries, after_seq=14) is None
    # The observed FJT goal_uuid is recorded as evidence and never asserted
    # equal to the Pick/Place internal ExecuteTrajectory UUID.
    assert first["goal_uuid"] != "pick-internal-uuid"


def test_tcp_speed_and_z_derive_from_two_newest_fresh_samples():
    """E11: tcp_z_m and tcp_speed_m_s come from the two newest fresh samples."""
    from validation.integrated_gate_executor import (
        _tcp_speed_from_samples,
        _tcp_z_from_samples,
    )

    samples = [
        {"received_mono": 100.0, "xyz": [0.5, 0.0, 0.70]},
        {"received_mono": 101.0, "xyz": [0.5, 0.0, 0.72]},
    ]
    assert _tcp_z_from_samples(samples) == pytest.approx(0.72)
    assert _tcp_speed_from_samples(samples) == pytest.approx(0.02)
    assert _tcp_z_from_samples(samples[:1]) == pytest.approx(0.70)
    assert _tcp_speed_from_samples(samples[:1]) is None
    assert _tcp_speed_from_samples([]) is None
    assert _tcp_z_from_samples([]) is None
    assert _tcp_speed_from_samples(
        [
            {"received_mono": 100.0, "xyz": [0.5, 0.0, 0.70]},
            {"received_mono": 100.0, "xyz": [0.5, 0.0, 0.72]},
        ]
    ) is None


def test_e_public_exports_import_star_succeeds():
    """F1.12: ``from validation.integrated_gate_executor import *`` must succeed;
    the stale module-level ``run_pick_place_*`` stub names are gone."""
    namespace: dict[str, object] = {}
    exec("from validation.integrated_gate_executor import *", namespace)
    assert "IntegratedGateExecutor" in namespace
    assert "stage_e_dispatch" in namespace
    assert "build_pick_goal" in namespace
    assert "Q_OUTBOUND" in namespace
    assert "run_pick_place_negative" not in namespace
    assert "run_pick_place_positive" not in namespace
    assert "run_pick_place_sequence" not in namespace


def test_e_fjt_receipt_window_boundaries():
    """F1.6: the 2.0 s receipt-time correlation window accepts inside, rejects
    just outside, and treats negative/missing/non-finite deltas as stale."""
    from validation.integrated_gate_executor import (
        E_FJT_CORRELATION_TIMEOUT_S,
        _fjt_receipt_delta_s,
        _fjt_within_receipt_window,
    )

    boundary = 1000.0
    window = float(E_FJT_CORRELATION_TIMEOUT_S)
    assert window == pytest.approx(2.0)
    exactly = {"received_mono": boundary + window}
    assert _fjt_receipt_delta_s(exactly, boundary) == pytest.approx(window)
    assert _fjt_within_receipt_window(exactly, boundary, window) is True
    just_inside = {"received_mono": boundary + window - 1e-6}
    assert _fjt_within_receipt_window(just_inside, boundary, window) is True
    just_outside = {"received_mono": boundary + window + 1e-6}
    assert _fjt_within_receipt_window(just_outside, boundary, window) is False
    stale = {"received_mono": boundary - 1.0}
    assert _fjt_receipt_delta_s(stale, boundary) is None
    assert _fjt_within_receipt_window(stale, boundary, window) is False
    assert _fjt_receipt_delta_s({}, boundary) is None
    assert _fjt_receipt_delta_s({"received_mono": float("nan")}, boundary) is None
    assert _fjt_receipt_delta_s({"received_mono": float("inf")}, boundary) is None
    assert _fjt_within_receipt_window({}, boundary, window) is False
    assert _fjt_within_receipt_window({"received_mono": 1.0}, boundary, "not-a-number") is False


def test_e_fixture_scene_error_keeps_gate_cd_strict():
    """F1.8: a stray ``pick_and_place/*`` world object fails shared fixture
    readiness for Gates C/D; only the exact task target is permitted through the
    explicit Stage-E ``allow_e_target`` path, and an arbitrary task-namespace
    object is still rejected there."""
    from validation.integrated_gate_executor import (
        TARGET_OBJECT_ID,
        IntegratedGateExecutor,
        expected_fixture_geometry_digest,
    )

    contract = scenario_report_contract("qualification-pick-place-positive")
    declaration = contract["planning_scene_declaration"]
    expected_ids = list(fixture_owned_ids(declaration))
    expected_digest = expected_fixture_geometry_digest(declaration)

    class _FixtureSceneStub:
        scenario = readiness_scenario(contract)

        def _expected_fixture_geometry_digest(self):
            return expected_digest

    stub = _FixtureSceneStub()

    def _scene(owned_ids):
        return {
            "owned_ids": list(owned_ids),
            "attached_ids": [],
            "fixture_geometry_digest": expected_digest,
        }

    assert IntegratedGateExecutor._fixture_scene_error(stub, _scene(expected_ids)) is None
    # C/D: any stray world object (task-namespace or otherwise) fails fixture
    # readiness with the exact ordered "must equal" message.
    for stray in ("pick_and_place/other", TARGET_OBJECT_ID, "sim_fixture/extra"):
        error = IntegratedGateExecutor._fixture_scene_error(
            stub, _scene(expected_ids + [stray])
        )
        assert error is not None and "must equal" in error, stray
    # E: the exact target is permitted only with allow_e_target=True.
    with_target = _scene(expected_ids + [TARGET_OBJECT_ID])
    assert (
        IntegratedGateExecutor._fixture_scene_error(stub, with_target, allow_e_target=True)
        is None
    )
    assert (
        IntegratedGateExecutor._fixture_scene_error(stub, with_target)
        is not None
    )
    # E: an arbitrary task-namespace object is still rejected even when allowed.
    stray_e = _scene(expected_ids + ["pick_and_place/other"])
    error_e = IntegratedGateExecutor._fixture_scene_error(stub, stray_e, allow_e_target=True)
    assert error_e is not None and "must equal" in error_e


# --- Stage-D pure helpers ---------------------------------------------------

def test_execute_status_name_maps_terminal_statuses():
    """D4: ExecuteTrajectory terminal statuses use action_msgs constants."""
    from validation.integrated_gate_executor import _execute_status_name

    assert _execute_status_name(4) == "succeeded"
    assert _execute_status_name(5) == "canceled"
    assert _execute_status_name(6) == "aborted"
    for bad in (0, 1, 3, 7, 999, True, "4", None):
        with pytest.raises(ValueError):
            _execute_status_name(bad)


def test_goal_uuid_normalization_and_validity():
    """D5: 16-byte UUID normalization and validity."""
    from validation.integrated_gate_executor import _normalize_goal_uuid, _valid_goal_uuid

    raw = bytes.fromhex("00112233445566778899aabbccddeeff")
    normalized = _normalize_goal_uuid(raw)
    assert normalized == "00112233445566778899aabbccddeeff"
    assert _valid_goal_uuid(normalized)
    assert _valid_goal_uuid("f" * 32)
    assert not _valid_goal_uuid("F" * 32)
    assert not _valid_goal_uuid("f" * 31)
    assert not _valid_goal_uuid(None)
    assert not _valid_goal_uuid(123)
    assert _normalize_goal_uuid("00112233-4455-6677-8899-aabbccddeeff") == "00112233445566778899aabbccddeeff"
    assert _normalize_goal_uuid(b"short") is None
    assert _normalize_goal_uuid(None) is None
    assert _normalize_goal_uuid(True) is None


def test_goal_uuid_normalization_accepts_real_uuid_containers():
    """F4.1: a real Humble ClientGoalHandle.goal_id is a UUID message whose
    ``.uuid`` is a numpy ``uint8[16]``; the normalizer must accept any strict
    16-byte iterable/buffer and reject malformed element ranges/lengths."""
    from array import array

    from validation.integrated_gate_executor import _normalize_goal_uuid

    raw_hex = "00112233445566778899aabbccddeeff"
    raw = bytes.fromhex(raw_hex)

    class _UUIDMsg:
        """Stand-in for ``unique_identifier_msgs/msg/UUID`` (``.uuid`` array)."""

        def __init__(self, value):
            self.uuid = value

    class _UUIDBytesMsg:
        """A container exposing ``.bytes`` instead of ``.uuid``."""

        def __init__(self, value):
            self.bytes = value

    import numpy as np

    containers = [
        np.array(list(raw), dtype=np.uint8),          # numpy uint8[16]
        np.frombuffer(raw, dtype=np.uint8),           # numpy read-only view
        array("B", raw),                              # array('B')
        list(raw),                                    # list of byte integers
        tuple(raw),                                   # tuple of byte integers
        memoryview(raw),                              # memoryview
        bytearray(raw),                               # bytearray
    ]
    for container in containers:
        assert _normalize_goal_uuid(container) == raw_hex, type(container)
        assert _normalize_goal_uuid(_UUIDMsg(container)) == raw_hex
    assert _normalize_goal_uuid(_UUIDBytesMsg(raw)) == raw_hex
    assert _normalize_goal_uuid(_UUIDMsg(np.array(list(raw), dtype=np.uint8))) == raw_hex

    # Malformed element ranges/types/lengths are rejected.
    assert _normalize_goal_uuid(_UUIDMsg(np.array([-1] + list(raw[1:]), dtype=np.int64))) is None
    assert _normalize_goal_uuid(_UUIDMsg(np.array([256] + list(raw[1:]), dtype=np.int64))) is None
    assert _normalize_goal_uuid(_UUIDMsg(list(raw) + [0])) is None  # 17 elements
    assert _normalize_goal_uuid(_UUIDMsg([str(v) for v in raw])) is None  # strings
    assert _normalize_goal_uuid(_UUIDMsg([0.0] * 16)) is None  # floats
    assert _normalize_goal_uuid(_UUIDMsg(b"short")) is None
    assert _normalize_goal_uuid(_UUIDMsg(True)) is None
    assert _normalize_goal_uuid(_UUIDMsg(None)) is None


def test_arm_velocity_within_limit_predicate():
    """D6: effective-stop bounded-velocity predicate."""
    from validation.integrated_gate_executor import _arm_velocity_within_limit

    bounded = [[0.0] * 7, [0.01, 0.02, 0.0, 0.01, 0.0, 0.0, 0.01], [0.02] * 7]
    assert _arm_velocity_within_limit(bounded, 0.02) is True
    assert _arm_velocity_within_limit(bounded, 0.05) is True
    over = [0.03] * 7
    assert _arm_velocity_within_limit([bounded[0], over], 0.02) is False
    assert _arm_velocity_within_limit([[0.0] * 6], 0.02) is False
    assert _arm_velocity_within_limit([[float("nan")] * 7], 0.02) is False
    assert _arm_velocity_within_limit(bounded, float("nan")) is False
    assert _arm_velocity_within_limit("not frames", 0.02) is False
    assert _arm_velocity_within_limit([], 0.02) is True


def test_derive_retreat_target_plus_z_preserves_orientation():
    """D7: +Z 0.10 m retreat from the observed TCP pose preserves orientation."""
    from validation.integrated_gate_executor import derive_retreat_target_pose

    source = {
        "frame_id": "base_link",
        "xyz": [0.65, 0.0, 0.72],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "identity": 7,
        "age_s": 0.05,
    }
    target = derive_retreat_target_pose(source, distance_m=0.10, axis="+z")
    assert target["frame_id"] == "base_link"
    assert target["xyz"] == [0.65, 0.0, 0.82]
    assert target["quaternion_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    for bad_axis in ("z", "+w", ""):
        with pytest.raises(ValueError, match="axis"):
            derive_retreat_target_pose(source, distance_m=0.10, axis=bad_axis)


def test_derive_retreat_target_fails_closed_on_bad_input():
    from validation.integrated_gate_executor import derive_retreat_target_pose

    for source in (
        {"frame_id": "world", "xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
        {"frame_id": "base_link", "xyz": [0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
        {"frame_id": "base_link", "xyz": [0.0, 0.0, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 0.0]},
    ):
        with pytest.raises(ValueError):
            derive_retreat_target_pose(source, distance_m=0.10, axis="+z")


def test_stage_e_dispatch_rejects_unknown_and_non_e_identities():
    """E12: unknown identities fail closed and the positive scenario is
    dispatchable (its kind is positive, so the negative runner's ValueError
    gate is reachable)."""
    from validation.integrated_gate_executor import stage_e_dispatch

    contract = scenario_report_contract("qualification-pick-place-positive")
    positive = stage_e_dispatch(
        "qualification-pick-place-positive",
        scenario=readiness_scenario(contract),
    )
    assert positive["kind"] == "positive"
    # An unknown id that does not match the scenario mapping fails closed.
    cancel_contract = scenario_report_contract("qualification-pick-place-cancel-approach")
    with pytest.raises(ValueError, match="scenario_id does not match"):
        stage_e_dispatch("other", scenario=readiness_scenario(cancel_contract))
    # An id that matches the mapping but is not a committed E scenario fails
    # closed after identity/stage/profile checks.
    for bad in ("cancel", "other"):
        scenario = readiness_scenario(cancel_contract)
        scenario["id"] = bad
        with pytest.raises(ValueError, match="not one of the Stage-E"):
            stage_e_dispatch(bad, scenario=scenario)
    # An empty scenario id fails closed as a nonempty-string requirement.
    with pytest.raises(ValueError, match="nonempty string"):
        stage_e_dispatch("", scenario=readiness_scenario(cancel_contract))


def test_d_stage_event_order_is_scenario_specific():
    """D9: D journals use scenario-specific event orders; Gate C stays exact."""
    from validation.integrated_gate_executor import (
        GATE_C_REQUIRED_EVENT_ORDER,
        STAGE_D_REQUIRED_EVENT_ORDER,
        _d_stage_event_order,
    )

    assert GATE_C_REQUIRED_EVENT_ORDER == ("fixture-ready", "teardown")
    assert _d_stage_event_order({"integrated": {"stage": "C"}}) == GATE_C_REQUIRED_EVENT_ORDER
    joint = _d_stage_event_order(
        {"id": "qualification-moveit-execute-joint", "integrated": {"stage": "D"}}
    )
    assert joint == ("fixture-ready", "execution-start", "execution-terminal", "teardown")
    cancel = _d_stage_event_order(
        {"id": "qualification-moveit-cancel", "integrated": {"stage": "D"}}
    )
    assert cancel == ("fixture-ready", "execution-start", "cancel-requested", "quiescent", "teardown")
    safety = _d_stage_event_order(
        {"id": "qualification-moveit-safety", "integrated": {"stage": "D"}}
    )
    assert safety == ("fixture-ready", "execution-start", "effective-stop", "operator-clear", "quiescent", "teardown")
    assert STAGE_D_REQUIRED_EVENT_ORDER["retreat"] == ("fixture-ready", "retreat-start", "retreat-terminal", "teardown")
    assert STAGE_D_REQUIRED_EVENT_ORDER["gripper"] == ("fixture-ready", "gripper-open-terminal", "gripper-close-terminal", "teardown")


def test_d_journal_rejects_gate_c_only_order(tmp_path):
    """D10: a D journal rejects a Gate-C-only sequence and vice versa."""
    from validation.integrated_gate_executor import (
        D_FORBIDDEN_EVENTS,
        GATE_C_REQUIRED_EVENT_ORDER,
        STAGE_D_REQUIRED_EVENT_ORDER,
        _d_stage_event_order,
    )

    from tinker_sim_bridge.fixture_planning_scene import fixture_owned_ids

    contract = scenario_report_contract("qualification-moveit-execute-joint")
    declaration = contract["planning_scene_declaration"]
    scene = {
        **_valid_scene(declaration),
        "frame_index": 0,
        "timestamp": 0.0,
    }

    def _make_journal(required_order, name):
        from planning_scene_journal import PlanningSceneJournal, load_model_touch_contract

        touch = load_model_touch_contract()
        return PlanningSceneJournal(
            fixture_revision=declaration["revision"],
            task_namespace="pick_and_place/",
            target_object_id="pick_and_place/object_mesh",
            expected_attach_link=touch["link_tcp"],
            expected_touch_links=touch["touch_links"],
            required_event_order=tuple(required_order),
            forbidden_events=D_FORBIDDEN_EVENTS,
            jsonl_path=tmp_path / f"{name}-planning-scene.jsonl",
        )

    d_order = STAGE_D_REQUIRED_EVENT_ORDER["execute-joint"]
    # A D journal recording a Gate-C-only sequence must fail finalization.
    d_journal = _make_journal(d_order, "d")
    d_journal.record_diff("fixture-ready", scene)
    d_journal.snapshot("teardown", frame_index=1, timestamp=1.0)
    with pytest.raises(ValueError, match="required event order"):
        d_journal.finalize("diagnostic-pass")

    # A Gate-C journal recording a D sequence must fail finalization.
    c_journal = _make_journal(GATE_C_REQUIRED_EVENT_ORDER, "c")
    c_journal.record_diff("fixture-ready", {**_valid_scene(declaration), "frame_index": 0, "timestamp": 0.0})
    c_journal.snapshot("execution-start", frame_index=1, timestamp=1.0)
    with pytest.raises(ValueError, match="required event order"):
        c_journal.finalize("diagnostic-pass")


def test_d_forbidden_events_match_gate_c_manipulation_events():
    """D11: the eight forbidden manipulation events apply unchanged to D."""
    from validation.integrated_gate_executor import D_FORBIDDEN_EVENTS, GATE_C_FORBIDDEN_EVENTS

    assert D_FORBIDDEN_EVENTS == GATE_C_FORBIDDEN_EVENTS
    assert D_FORBIDDEN_EVENTS == (
        "before-pick",
        "scene-attach",
        "lift-complete",
        "transport",
        "before-release",
        "scene-detach",
        "released-settled",
        "task-cleanup",
    )


def test_journal_graph_projection_unchanged_with_fjt_status_subscription():
    """D12: the Task-3 graph projection shape is unchanged when the executor
    additionally subscribes to the FJT status topic (the status subscription
    stays outside the exact three-topic/two-service journal projection)."""
    from validation.integrated_gate_executor import (
        FJT_STATUS_TOPIC,
        build_journal_graph_projection,
    )

    projection = build_journal_graph_projection(
        fixture_payload=_canonical_fixture_payload(),
        observed_graph=_observed_graph_double(),
    )
    assert set(projection["topics"]) == {
        "/planning_scene",
        "/monitored_planning_scene",
        "/sim/status/planning_scene_fixture",
    }
    assert set(projection["services"]) == {"/get_planning_scene", "/apply_planning_scene"}
    assert FJT_STATUS_TOPIC not in projection["topics"]


def test_stage_d_expected_physical_lists_match_scenario_declarations():
    """D13: every D scenario's declared expected_physical equals the exact
    configured list carried by the dispatch spec."""
    from validation.integrated_gate_executor import STAGE_D_EXPECTED_PHYSICAL, stage_d_dispatch

    for scenario_name in STAGE_D_SCENARIO_NAMES:
        contract = scenario_report_contract(scenario_name)
        declared = contract["integrated"].get("expected_physical")
        spec = stage_d_dispatch(scenario_name, scenario=readiness_scenario(contract))
        assert list(declared) == spec["expected_physical"]
        assert list(declared) == list(STAGE_D_EXPECTED_PHYSICAL[scenario_name])


def test_post_grasp_lift_m_observation_enforces_010_threshold():
    """F2.1: the injected ``post_grasp_lift_m`` runtime-parameter observation
    accepts a finite value >= ``object_lift_m`` (0.10 m) and rejects the
    production default 0.08, bool, non-finite, missing-identity, and stale
    samples with a stable reason string (never a 15 s transport timeout)."""
    from validation.integrated_gate_executor import _post_grasp_lift_m_observation

    ok = _post_grasp_lift_m_observation(
        {"value_m": 0.10, "identity": "post-grasp-lift-m", "age_s": 0.0},
        object_lift_m=0.10,
        fresh_limit_s=0.25,
    )
    assert not isinstance(ok, str)
    value_m, meta = ok
    assert value_m == pytest.approx(0.10)
    assert meta["identity"] == "post-grasp-lift-m"
    assert meta["value_m"] == pytest.approx(0.10)
    assert meta["object_lift_m"] == pytest.approx(0.10)
    assert meta["received_mono"] > 0.0
    # A value above the threshold is equally accepted (0.10 + headroom).
    high = _post_grasp_lift_m_observation(
        {"value_m": 0.12, "identity": "post-grasp-lift-m", "age_s": 0.0},
        object_lift_m=0.10,
        fresh_limit_s=0.25,
    )
    assert not isinstance(high, str)

    assert _post_grasp_lift_m_observation(
        {"value_m": 0.08, "identity": "x", "age_s": 0.0},
        object_lift_m=0.10,
        fresh_limit_s=0.25,
    ).startswith("below-object-lift")
    # bool is non-finite in this contract (a raw float parameter is required).
    assert _post_grasp_lift_m_observation(
        {"value_m": True, "identity": "x", "age_s": 0.0},
        object_lift_m=0.10,
        fresh_limit_s=0.25,
    ) == "non-finite"
    assert _post_grasp_lift_m_observation(
        {"value_m": float("nan"), "identity": "x", "age_s": 0.0},
        object_lift_m=0.10,
        fresh_limit_s=0.25,
    ) == "non-finite"
    assert _post_grasp_lift_m_observation(
        {"value_m": "x", "identity": "x", "age_s": 0.0},
        object_lift_m=0.10,
        fresh_limit_s=0.25,
    ) == "non-finite"
    # Missing identity/receipt metadata is rejected (fresh identity required).
    assert _post_grasp_lift_m_observation(
        {"value_m": 0.10, "age_s": 0.0},
        object_lift_m=0.10,
        fresh_limit_s=0.25,
    ) == "missing"
    assert _post_grasp_lift_m_observation(
        "not-a-sample",
        object_lift_m=0.10,
        fresh_limit_s=0.25,
    ) == "missing"
    # Stale samples are rejected.
    assert _post_grasp_lift_m_observation(
        {"value_m": 0.10, "identity": "x", "age_s": 99.0},
        object_lift_m=0.10,
        fresh_limit_s=0.25,
    ) == "stale"


def test_post_grasp_lift_m_transport_kind_set_is_exact():
    """F2.1: exactly the four E transport scenarios require the observed
    ``post_grasp_lift_m`` runtime parameter (positive, occupied-place,
    cancel-transport, safety-transport); cancel-approach and the other
    negatives never do."""
    from validation.integrated_gate_executor import (
        STAGE_E_KIND,
        STAGE_E_SCENARIOS,
        _E_TRANSPORT_KINDS,
    )

    transport_kinds = {STAGE_E_KIND[scenario] for scenario in STAGE_E_SCENARIOS}
    assert _E_TRANSPORT_KINDS == frozenset(
        {"positive", "occupied-place", "cancel-transport", "safety-transport"}
    )
    assert transport_kinds == {
        "positive",
        "blocked-approach",
        "unreachable-grasp",
        "malformed-back",
        "cancel-approach",
        "cancel-transport",
        "safety-transport",
        "occupied-place",
    }


# ---------------------------------------------------------------------------
# F3.9: fail-dominant visual-event producer routing.
#
# A required canonical visual event that cannot be durably recorded (duplicate,
# no-join-key, invalid timestamp, rejected append, or any other non-"recorded"
# producer status) must make the current diagnostic attempt ``evidence-invalid``
# through the fail-dominant finalization path — never a silent pass with missing
# visual evidence, and never a fabricated capture.  These tests drive the real
# executor flows (E positive / E cancel / E safety and D cancel / D safety) to
# the exact first required visual event and inject a producer failure.
# ---------------------------------------------------------------------------


class _JournalStub:
    """Minimal producer-facing journal double exposing only ``record_count``."""

    def __init__(self, record_count: int) -> None:
        self.record_count = int(record_count)


class _StubSpinner:
    """Stand-in for ``ExecutorSingleThreadedExecutor`` (never spun in these tests)."""

    def spin_once(self, timeout_sec: object = 0.05) -> None:
        return None


def _flow_executor(
    tmp_path: Path,
    scenario_name: str,
    *,
    required_event_order: Sequence[str],
    forbidden_events: Sequence[str],
):
    """Build a real executor harness able to reach the first required visual event.

    Constructs the real ``IntegratedGateExecutor`` via ``object.__new__`` with
    every per-attempt attribute the D/E flows touch before the first
    ``_append_visual_event`` call: a real PlanningScene journal, an advancing
    join-key provider, a genuine passing readiness snapshot derived from the
    scenario contract, a valid fixture scene carrying the exact declared
    geometry projection digest, and the receipt/latch seams the reset and
    preamble paths clear.  No ROS, no Isaac, no camera.
    """
    from planning_scene_journal import PlanningSceneJournal, load_model_touch_contract

    from validation.integrated_gate_executor import (
        TARGET_OBJECT_ID,
        TASK_NAMESPACE,
        expected_fixture_geometry_digest,
    )

    contract = scenario_report_contract(scenario_name)
    declaration = contract["planning_scene_declaration"]
    touch = load_model_touch_contract()
    journal = PlanningSceneJournal(
        fixture_revision=declaration["revision"],
        task_namespace=TASK_NAMESPACE,
        target_object_id=TARGET_OBJECT_ID,
        expected_attach_link=touch["link_tcp"],
        expected_touch_links=touch["touch_links"],
        required_event_order=tuple(required_event_order),
        forbidden_events=tuple(forbidden_events),
        jsonl_path=tmp_path / "planning-scene.jsonl",
    )
    from validation.integrated_gate_executor import IntegratedGateExecutor

    executor = object.__new__(IntegratedGateExecutor)
    executor.config = _config()
    executor.scenario = readiness_scenario(contract)
    executor.attempt_dir = tmp_path
    executor.journal = journal
    counter = {"n": 0}

    def _advancing_join():
        counter["n"] += 1
        return (counter["n"], float(counter["n"]))

    executor.join_key_provider = _advancing_join
    executor._last_join_key = None
    executor.readiness_snapshot_provider = lambda: _ready_snapshot_for_contract(contract)
    executor.graph_observation_provider = None
    executor._latest_planning_scene = {
        **_valid_scene(declaration),
        "frame_index": 0,
        "timestamp": 0.0,
        "fixture_geometry_digest": expected_fixture_geometry_digest(declaration),
    }
    executor._planning_scene_invalid = False
    executor._scene_invalid_sequence = None
    executor._spinner = _StubSpinner()
    executor._tcp_pose_samples = []
    executor._e_goal_state = {
        "pick_sent": False,
        "pick_goal_id": None,
        "place_sent": False,
        "place_goal_id": None,
    }
    executor._e_active_goal_handle = None
    executor._e_native_gripper_count_provider = None
    executor._e_native_gripper_count_baseline = None
    executor._e_post_grasp_lift_m_provider = None
    executor._e_post_grasp_lift_m_observed = None
    executor._e_lift_latch_mono = None
    executor._e_observed_fjt_trigger = None
    executor._joint_velocity_frames = []
    executor._fjt_status_cache = []
    executor._fjt_receipt_sequence = 0
    executor._joint_receipt_sequence = 0
    executor._emitted_visual_events = set()
    executor._visual_event_sequence = 0
    executor._visual_event_failures = []
    executor._reset_visual_event_state()
    return executor


def _tcp_provider() -> dict[str, object]:
    return {"xyz": [0.2, 0.0, 0.5], "identity": "tcp", "age_s": 0.05}


def _gripper_count_provider() -> dict[str, object]:
    return {"count": 0, "age_s": 0.05}


def _lift_m_provider() -> dict[str, object]:
    return {"identity": "pick_and_place", "value_m": 0.10, "age_s": 0.05}


def _fail_visual_append(executor, event: str, status: str):
    real_append = executor._append_visual_event

    def _failing_append(call_event, scenario_id):
        if call_event == event:
            return status
        return real_append(call_event, scenario_id)

    return _failing_append


def test_f39_append_visual_event_producer_contract(tmp_path):
    """F3.9/F3.4: the canonical producer emits exactly the six-key sequence shape."""
    from validation.integrated_gate_executor import IntegratedGateExecutor

    executor = object.__new__(IntegratedGateExecutor)
    executor.attempt_dir = tmp_path
    executor.journal = _JournalStub(record_count=7)
    executor._emitted_visual_events = set()
    executor._visual_event_sequence = 0
    executor._visual_event_failures = []
    executor._reset_visual_event_state()
    executor._last_join_key = (3, 2.5)

    status = executor._append_visual_event(
        "cancel-execution-start", "qualification-moveit-cancel"
    )
    assert status == "recorded"
    lines = (tmp_path / "visual-capture-requests.jsonl").read_text(
        encoding="utf-8"
    ).strip().splitlines()
    assert len(lines) == 1
    record = json.loads(lines[0])
    assert record == {
        "schema_version": 1,
        "sequence": 1,
        "gate": "qualification-moveit-cancel",
        "event": "cancel-execution-start",
        "simulated_timestamp": 2.5,
        "source_execution_event_sequence": 7,
    }


def test_f39_append_visual_event_monotonic_sequence_and_truth_timestamps(tmp_path):
    """F3.9: sequences advance monotonically and timestamps come from the durable
    join key (never fabricated), with the exact source sequence bound to the
    journal record count at emission time."""
    from validation.integrated_gate_executor import IntegratedGateExecutor

    def _new_executor(record_count, subdir):
        executor = object.__new__(IntegratedGateExecutor)
        executor.attempt_dir = tmp_path / subdir
        executor.journal = _JournalStub(record_count=record_count)
        executor._emitted_visual_events = set()
        executor._visual_event_sequence = 0
        executor._visual_event_failures = []
        executor._reset_visual_event_state()
        return executor

    executor = _new_executor(record_count=2, subdir="monotonic")
    executor._last_join_key = (1, 1.0)
    assert executor._append_visual_event("safety-execution-start", "g") == "recorded"
    executor._last_join_key = (2, 2.0)
    assert executor._append_visual_event("safety-trigger", "g") == "recorded"
    executor.journal = _JournalStub(record_count=5)
    executor._last_join_key = (2, 2.0)
    assert executor._append_visual_event("safety-post-clear", "g") == "recorded"

    records = [
        json.loads(line)
        for line in (tmp_path / "monotonic" / "visual-capture-requests.jsonl")
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    ]
    assert [record["sequence"] for record in records] == [1, 2, 3]
    assert [record["simulated_timestamp"] for record in records] == [1.0, 2.0, 2.0]
    assert [record["event"] for record in records] == [
        "safety-execution-start",
        "safety-trigger",
        "safety-post-clear",
    ]
    assert [record["source_execution_event_sequence"] for record in records] == [2, 2, 5]


def test_f39_append_visual_event_duplicate_no_join_key_invalid_timestamp(tmp_path):
    """F3.9: the producer fails closed on duplicate, missing join key, and
    invalid timestamps, recording each rejection in the per-attempt failure list
    without writing any capture request."""
    from validation.integrated_gate_executor import IntegratedGateExecutor

    def _new_executor(subdir):
        executor = object.__new__(IntegratedGateExecutor)
        executor.attempt_dir = tmp_path / subdir
        executor.journal = _JournalStub(record_count=0)
        executor._emitted_visual_events = set()
        executor._visual_event_sequence = 0
        executor._visual_event_failures = []
        executor._reset_visual_event_state()
        return executor

    # Duplicate: the same required event may only be emitted once per attempt.
    executor = _new_executor("duplicate")
    executor._last_join_key = (1, 1.0)
    assert executor._append_visual_event("cancel-trigger", "g") == "recorded"
    assert executor._append_visual_event("cancel-trigger", "g") == "duplicate"
    assert executor._visual_event_failures == [("cancel-trigger", "duplicate")]
    assert len(
        (tmp_path / "duplicate" / "visual-capture-requests.jsonl")
        .read_text().strip().splitlines()
    ) == 1

    # No join key: a required event must never emit before its durable checkpoint.
    executor = _new_executor("nojoin")
    assert executor._append_visual_event("cancel-execution-start", "g") == "no-join-key"
    assert executor._visual_event_failures == [("cancel-execution-start", "no-join-key")]
    assert not (tmp_path / "nojoin" / "visual-capture-requests.jsonl").exists()

    # Invalid timestamp: garbage / non-finite / negative truth tails fail closed.
    for index, bad in enumerate(
        [(0, "not-a-number"), (0, float("nan")), (0, float("-inf")), (0, -1.0)]
    ):
        executor = _new_executor(f"invalid-{index}")
        executor._last_join_key = bad
        assert executor._append_visual_event("safety-trigger", "g") == "invalid-timestamp"
        assert executor._visual_event_failures == [("safety-trigger", "invalid-timestamp")]
        assert not (tmp_path / f"invalid-{index}" / "visual-capture-requests.jsonl").exists()


def test_f39_append_visual_event_rejected_append_records_failure(tmp_path):
    """F3.9: an append I/O failure surfaces as a fail-closed ``rejected:`` status
    and is recorded in the per-attempt failure list (no capture is fabricated)."""
    from validation.integrated_gate_executor import IntegratedGateExecutor

    blocker = tmp_path / "visual-capture-requests.jsonl"
    blocker.mkdir()  # a directory cannot be appended as a JSONL stream
    executor = object.__new__(IntegratedGateExecutor)
    executor.attempt_dir = tmp_path
    executor.journal = _JournalStub(record_count=0)
    executor._emitted_visual_events = set()
    executor._visual_event_sequence = 0
    executor._visual_event_failures = []
    executor._reset_visual_event_state()
    executor._last_join_key = (1, 1.0)

    status = executor._append_visual_event("terminal", "g")
    assert status.startswith("rejected: ")
    assert executor._visual_event_failures == [("terminal", status)]


@pytest.mark.parametrize(
    "failure_status",
    ["duplicate", "no-join-key", "invalid-timestamp", "rejected: disk-full"],
)
def test_f39_e_cancel_visual_event_rejection_is_fail_dominant(
    tmp_path, monkeypatch, failure_status
):
    """F3.9: every non-``recorded`` producer status for the E cancel first
    required visual event makes the attempt ``evidence-invalid``."""
    from validation.integrated_gate_executor import (
        STAGE_E_FORBIDDEN_EVENTS,
        STAGE_E_REQUIRED_EVENT_ORDER,
    )

    executor = _flow_executor(
        tmp_path,
        "qualification-pick-place-cancel-approach",
        required_event_order=STAGE_E_REQUIRED_EVENT_ORDER["cancel-approach"],
        forbidden_events=STAGE_E_FORBIDDEN_EVENTS["cancel-approach"],
    )
    monkeypatch.setattr(
        executor,
        "_append_visual_event",
        _fail_visual_append(executor, "cancel-execution-start", failure_status),
    )
    result = executor.run_pick_place_negative(
        "qualification-pick-place-cancel-approach",
        current_tcp_pose_provider=_tcp_provider,
        native_gripper_goal_count_provider=_gripper_count_provider,
    )
    assert result["status"] == "evidence-invalid"
    assert "cancel-execution-start" in result["reason_code"]
    assert failure_status in result["reason_code"]
    # The durable failure artifact is written, never a pass.
    assert (tmp_path / "planning-scene.json").exists()
    assert (tmp_path / "integrated-execution.jsonl").exists()


@pytest.mark.parametrize(
    "failure_status",
    ["duplicate", "no-join-key", "invalid-timestamp", "rejected: disk-full"],
)
def test_f39_e_positive_visual_event_rejection_is_fail_dominant(
    tmp_path, monkeypatch, failure_status
):
    """F3.9: a rejected ``readiness`` visual event fails the E positive attempt
    to ``evidence-invalid`` before any Pick/Place goal is sent."""
    from validation.integrated_gate_executor import (
        STAGE_E_FORBIDDEN_EVENTS,
        STAGE_E_REQUIRED_EVENT_ORDER,
    )

    executor = _flow_executor(
        tmp_path,
        "qualification-pick-place-positive",
        required_event_order=STAGE_E_REQUIRED_EVENT_ORDER["positive"],
        forbidden_events=STAGE_E_FORBIDDEN_EVENTS["positive"],
    )
    monkeypatch.setattr(
        executor,
        "_append_visual_event",
        _fail_visual_append(executor, "readiness", failure_status),
    )
    result = executor.run_pick_place_positive(
        current_tcp_pose_provider=_tcp_provider,
        post_grasp_lift_m_provider=_lift_m_provider,
    )
    assert result["status"] == "evidence-invalid"
    assert "readiness" in result["reason_code"]
    assert failure_status in result["reason_code"]
    assert result["pick_goal_sent"] is False
    assert result["place_goal_sent"] is False


def test_f39_e_safety_visual_event_rejection_is_fail_dominant(tmp_path, monkeypatch):
    """F3.9: a rejected ``safety-execution-start`` visual event fails the E
    safety-transport attempt to ``evidence-invalid`` before any Pick goal."""
    from validation.integrated_gate_executor import (
        STAGE_E_FORBIDDEN_EVENTS,
        STAGE_E_REQUIRED_EVENT_ORDER,
    )

    executor = _flow_executor(
        tmp_path,
        "qualification-pick-place-safety-transport",
        required_event_order=STAGE_E_REQUIRED_EVENT_ORDER["safety-transport"],
        forbidden_events=STAGE_E_FORBIDDEN_EVENTS["safety-transport"],
    )
    monkeypatch.setattr(
        executor,
        "_append_visual_event",
        _fail_visual_append(executor, "safety-execution-start", "duplicate"),
    )
    result = executor.run_pick_place_negative(
        "qualification-pick-place-safety-transport",
        current_tcp_pose_provider=_tcp_provider,
        post_grasp_lift_m_provider=_lift_m_provider,
    )
    assert result["status"] == "evidence-invalid"
    assert "safety-execution-start" in result["reason_code"]
    assert "duplicate" in result["reason_code"]
    assert result["pick_goal_sent"] is False


def test_f39_d_cancel_visual_event_rejection_is_fail_dominant(tmp_path, monkeypatch):
    """F3.9: a rejected ``cancel-execution-start`` visual event fails the D
    cancel attempt to ``evidence-invalid`` with the exact execute error."""
    from validation.integrated_gate_executor import (
        D_FORBIDDEN_EVENTS,
        STAGE_D_REQUIRED_EVENT_ORDER,
    )

    executor = _flow_executor(
        tmp_path,
        "qualification-moveit-cancel",
        required_event_order=STAGE_D_REQUIRED_EVENT_ORDER["cancel"],
        forbidden_events=D_FORBIDDEN_EVENTS,
    )
    monkeypatch.setattr(
        executor,
        "_append_visual_event",
        _fail_visual_append(executor, "cancel-execution-start", "invalid-timestamp"),
    )
    result = executor.run_cancel_sequence(
        "qualification-moveit-cancel",
        planning_goal_id="a" * 32,
        execute_goal_id="b" * 32,
        fjt_goal_id="c" * 32,
        fjt_transaction_provider=lambda: {},
    )
    assert result["status"] == "evidence-invalid"
    assert "cancel-execution-start" in result["execute_error"]
    assert "invalid-timestamp" in result["execute_error"]


def test_f39_d_safety_visual_event_rejection_is_fail_dominant(tmp_path, monkeypatch):
    """F3.9: a rejected ``safety-execution-start`` visual event fails the D
    safety attempt to ``evidence-invalid`` with the exact execute error."""
    from validation.integrated_gate_executor import (
        D_FORBIDDEN_EVENTS,
        STAGE_D_REQUIRED_EVENT_ORDER,
    )

    executor = _flow_executor(
        tmp_path,
        "qualification-moveit-safety",
        required_event_order=STAGE_D_REQUIRED_EVENT_ORDER["safety"],
        forbidden_events=D_FORBIDDEN_EVENTS,
    )
    monkeypatch.setattr(
        executor,
        "_append_visual_event",
        _fail_visual_append(executor, "safety-execution-start", "no-join-key"),
    )
    result = executor.run_safety_sequence(
        "qualification-moveit-safety",
        long_motion_provider=lambda: {
            "planning_goal_id": "a" * 32,
            "execute_goal_id": "b" * 32,
        },
        fjt_goal_id="c" * 32,
        fjt_transaction_provider=lambda: {},
    )
    assert result["status"] == "evidence-invalid"
    assert "safety-execution-start" in result["execute_error"]
    assert "no-join-key" in result["execute_error"]


def test_f39_visual_event_failures_reset_per_attempt(tmp_path):
    """F3.9: the per-attempt visual-event failure list resets between attempts so
    a reused executor never carries a previous attempt's rejections forward."""
    from validation.integrated_gate_executor import IntegratedGateExecutor

    executor = object.__new__(IntegratedGateExecutor)
    executor.attempt_dir = tmp_path
    executor.journal = _JournalStub(record_count=0)
    executor._last_join_key = None
    executor._emitted_visual_events = set()
    executor._visual_event_sequence = 0
    executor._visual_event_failures = []
    executor._reset_visual_event_state()
    executor._last_join_key = (1, 1.0)
    assert executor._append_visual_event("a", "g") == "recorded"
    assert executor._append_visual_event("a", "g") == "duplicate"
    assert executor._visual_event_failures == [("a", "duplicate")]

    executor._reset_visual_event_state()
    assert executor._visual_event_failures == []
    assert executor._emitted_visual_events == set()
    assert executor._visual_event_sequence == 0


# ---------------------------------------------------------------------------
# F4: Gate-D retreat/gripper success paths must record terminal evidence.
#
# The verifier's ``_terminal_success`` raises "execution summary has no terminal
# evidence" when ``terminal_status``/``execute_result_status``/
# ``task_result_status`` are all absent.  The gripper/retreat success paths
# finalize with ``final_status="diagnostic-pass"`` but historically passed NO
# terminal evidence, so a physically-successful gripper/retreat was rejected by
# the verifier.  These tests pin the success-path ``_finalize_d_attempt`` call
# to carry ``terminal_status="success"``.
# ---------------------------------------------------------------------------


def _f4_fake_gripper_goal(position, *, max_effort):
    """ROS-free stand-in for ``build_gripper_goal`` (control_msgs import)."""
    return types.SimpleNamespace(
        command=types.SimpleNamespace(
            position=float(position), max_effort=float(max_effort)
        )
    )


class _F4SucceedingActionClient:
    """Deterministic action client whose goals are accepted and succeed at once."""

    class _Result:
        status = 4  # EXECUTE_STATUS_SUCCEEDED

    class _ResultFuture:
        def done(self):
            return True

        def result(self):
            return _F4SucceedingActionClient._Result()

    class _GoalHandle:
        accepted = True

        def get_result_async(self):
            return _F4SucceedingActionClient._ResultFuture()

    class _SendFuture:
        def done(self):
            return True

        def result(self):
            return _F4SucceedingActionClient._GoalHandle()

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal):
        return self._SendFuture()


def _f4_dummy_cloud():
    """A structurally valid ``base_link`` PointCloud2-shaped object (duck-typed)."""
    return types.SimpleNamespace(
        header=types.SimpleNamespace(frame_id="base_link"),
        width=1,
        height=1,
        data=b"\x00" * 8,
        point_step=8,
        row_step=8,
        fields=None,
    )


def _f4_capture_finalize(executor, captured):
    """Replace ``_finalize_d_attempt`` with a spy recording the finalization call."""

    def _spy_finalize(*args, **kwargs):
        captured["final_status"] = args[4] if len(args) > 4 else kwargs.get("final_status")
        captured["terminal_status"] = kwargs.get("terminal_status")
        captured["kind"] = args[1].get("kind") if len(args) > 1 else None
        return {
            "status": "diagnostic-pass",
            "terminal_status": kwargs.get("terminal_status"),
        }

    executor._finalize_d_attempt = _spy_finalize
    return executor


def test_f4_gripper_success_path_records_terminal_evidence(tmp_path, monkeypatch):
    """RED (F4): the gripper success ``_finalize_d_attempt`` call must carry
    ``terminal_status="success"`` so the verifier accepts the terminal."""
    import validation.integrated_gate_executor as exec_mod

    from validation.integrated_gate_executor import (
        D_FORBIDDEN_EVENTS,
        STAGE_D_REQUIRED_EVENT_ORDER,
    )

    monkeypatch.setattr(exec_mod, "build_gripper_goal", _f4_fake_gripper_goal)
    executor = _flow_executor(
        tmp_path,
        "qualification-moveit-gripper",
        required_event_order=STAGE_D_REQUIRED_EVENT_ORDER["gripper"],
        forbidden_events=D_FORBIDDEN_EVENTS,
    )
    client = _F4SucceedingActionClient()
    executor._action_clients = {
        "/xarm_gripper/gripper_action": client,
        "/cartesian_move_action": client,
    }
    # The real journal snapshot path is exercised elsewhere; here we only need
    # to reach the success finalization with the terminal-evidence wiring.
    executor._journal_snapshot_d = lambda event: "recorded"
    captured: dict[str, object] = {}
    _f4_capture_finalize(executor, captured)

    record = executor.run_gripper_sequence("qualification-moveit-gripper")

    assert captured["final_status"] == "diagnostic-pass"
    assert captured["kind"] == "gripper"
    assert captured["terminal_status"] == "success"
    assert record["terminal_status"] == "success"


def test_f4_retreat_success_path_records_terminal_evidence(tmp_path, monkeypatch):
    """RED (F4): the retreat success ``_finalize_d_attempt`` call must carry
    ``terminal_status="success"`` so the verifier accepts the terminal."""
    import validation.integrated_gate_executor as exec_mod

    from validation.integrated_gate_executor import (
        D_FORBIDDEN_EVENTS,
        STAGE_D_REQUIRED_EVENT_ORDER,
    )

    def _fake_cartesian_goal(target_pose, *, env_points=None):
        return types.SimpleNamespace(target_pose=target_pose, env_points=env_points)

    monkeypatch.setattr(exec_mod, "build_cartesian_move_goal", _fake_cartesian_goal)
    executor = _flow_executor(
        tmp_path,
        "qualification-moveit-cartesian-retreat",
        required_event_order=STAGE_D_REQUIRED_EVENT_ORDER["retreat"],
        forbidden_events=D_FORBIDDEN_EVENTS,
    )
    client = _F4SucceedingActionClient()
    executor._action_clients = {
        "/xarm_gripper/gripper_action": client,
        "/cartesian_move_action": client,
    }
    executor._journal_snapshot_d = lambda event: "recorded"
    executor.ros = {"serialize_message": lambda msg: b"env-cloud"}
    captured: dict[str, object] = {}
    _f4_capture_finalize(executor, captured)

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
        environment_cloud_provider=_f4_dummy_cloud,
    )

    assert captured["final_status"] == "diagnostic-pass"
    assert captured["kind"] == "retreat"
    assert captured["terminal_status"] == "success"
    assert record["terminal_status"] == "success"


def test_f4_safety_cancel_presend_after_operator_clear(tmp_path, monkeypatch):
    """RED (S2): ``_cancel_presend_after_clear`` must cancel the retained
    ExecuteTrajectory goal on the exact handle after operator-clear so the FJT
    controller stops streaming a commanded target into the frozen arm.  An
    absent handle is a no-op and a rejected cancel is recorded, never raised."""
    from validation.integrated_gate_executor import IntegratedGateExecutor

    executor = object.__new__(IntegratedGateExecutor)
    executor.attempt_dir = tmp_path
    executor._thresholds = lambda: {"cancel_timeout_s": 2.0}
    handle = types.SimpleNamespace(goal_id=bytes.fromhex("d" * 32))
    calls: dict[str, object] = {}

    def _fake_normalize(goal_handle):
        return "d" * 32

    def _fake_cancel(goal_handle, *, expected_goal_uuid, timeout_s):
        calls["handle"] = goal_handle
        calls["uuid"] = expected_goal_uuid
        calls["timeout_s"] = timeout_s
        return {
            "response": "accepted",
            "return_code": 0,
            "goals_canceling": [expected_goal_uuid],
            "error": None,
        }

    monkeypatch.setattr(executor, "_normalize_goal_id", _fake_normalize)
    monkeypatch.setattr(executor, "_cancel_execute_goal", _fake_cancel)

    outcome = executor._cancel_presend_after_clear(handle)
    assert outcome["response"] == "accepted"
    assert calls["handle"] is handle
    assert calls["uuid"] == "d" * 32
    assert calls["timeout_s"] == 2.0
    assert executor._safety_post_clear_cancel["response"] == "accepted"


def test_f4_safety_cancel_presend_none_handle_is_noop(tmp_path, monkeypatch):
    """A missing presend handle must be a no-op (returns None, records nothing)."""
    from validation.integrated_gate_executor import IntegratedGateExecutor

    executor = object.__new__(IntegratedGateExecutor)
    executor.attempt_dir = tmp_path
    executor._thresholds = lambda: {"cancel_timeout_s": 2.0}

    def _boom(*args, **kwargs):
        raise AssertionError("cancel must not be called with a None handle")

    monkeypatch.setattr(executor, "_cancel_execute_goal", _boom)
    assert executor._cancel_presend_after_clear(None) is None


def test_f4_safety_cancel_presend_rejected_never_raises(tmp_path, monkeypatch):
    """A rejected post-clear cancel (goal already terminal after FJT ABORTED)
    is recorded diagnostically and never raised or failed."""
    from validation.integrated_gate_executor import IntegratedGateExecutor

    executor = object.__new__(IntegratedGateExecutor)
    executor.attempt_dir = tmp_path
    executor._thresholds = lambda: {"cancel_timeout_s": 2.0}
    handle = types.SimpleNamespace(goal_id=bytes.fromhex("e" * 32))
    monkeypatch.setattr(executor, "_normalize_goal_id", lambda h: "e" * 32)
    monkeypatch.setattr(
        executor,
        "_cancel_execute_goal",
        lambda h, *, expected_goal_uuid, timeout_s: {
            "response": "rejected",
            "return_code": 3,
            "goals_canceling": [],
            "error": "goal already terminated",
        },
    )
    outcome = executor._cancel_presend_after_clear(handle)
    assert outcome["response"] == "rejected"
    assert executor._safety_post_clear_cancel["error"] == "goal already terminated"


# ---------------------------------------------------------------------------
# F3 — cartesian-retreat bounded retry on pick_and_place goal rejection.
#
# pick_and_place rejects the retreat goal when its INTERNAL runtime observation
# is not yet ready (``refresh_live_observations`` fails — "live runtime
# observation is not ready"), even though the executor's own readiness passed.
# The rejection is transient (pick_and_place becomes ready within seconds), so
# the executor must retry the send with a bounded budget instead of failing
# closed on the first rejection.
# ---------------------------------------------------------------------------

class _F3RejectedGoalHandle:
    accepted = False


class _F3AcceptedGoalHandle:
    accepted = True

    class _Result:
        status = 4  # EXECUTE_STATUS_SUCCEEDED

    class _ResultFuture:
        def done(self):
            return True

        def result(self):
            return _F3AcceptedGoalHandle._Result()

    def get_result_async(self):
        return self._ResultFuture()


class _F3RejectThenAcceptClient:
    """Rejects the first ``reject_count`` sends, then accepts."""

    def __init__(self, reject_count: int = 1):
        self.reject_count = int(reject_count)
        self.sends = 0

    class _SendFuture:
        def __init__(self, handle):
            self._handle = handle

        def done(self):
            return True

        def result(self):
            return self._handle

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal):
        self.sends += 1
        if self.sends <= self.reject_count:
            return self._SendFuture(_F3RejectedGoalHandle())
        return self._SendFuture(_F3AcceptedGoalHandle())


def test_f3_cartesian_retreat_retries_transient_rejection(tmp_path, monkeypatch):
    """RED (F3): a transient pick_and_place goal rejection (internal runtime
    observation not yet ready) must be retried with a bounded budget so the
    retreat still reaches the success finalization."""
    import validation.integrated_gate_executor as exec_mod

    from validation.integrated_gate_executor import (
        D_FORBIDDEN_EVENTS,
        STAGE_D_REQUIRED_EVENT_ORDER,
    )

    def _fake_cartesian_goal(target_pose, *, env_points=None):
        return types.SimpleNamespace(target_pose=target_pose, env_points=env_points)

    monkeypatch.setattr(exec_mod, "build_cartesian_move_goal", _fake_cartesian_goal)
    executor = _flow_executor(
        tmp_path,
        "qualification-moveit-cartesian-retreat",
        required_event_order=STAGE_D_REQUIRED_EVENT_ORDER["retreat"],
        forbidden_events=D_FORBIDDEN_EVENTS,
    )
    executor.config = {
        **executor.config,
        "thresholds": {
            **executor.config["thresholds"],
            "cartesian_reject_retry_budget_s": 0.5,
        },
    }
    client = _F3RejectThenAcceptClient(reject_count=1)
    executor._action_clients = {
        "/xarm_gripper/gripper_action": client,
        "/cartesian_move_action": client,
    }
    executor._journal_snapshot_d = lambda event: "recorded"
    executor.ros = {"serialize_message": lambda msg: b"env-cloud"}
    captured: dict[str, object] = {}
    _f4_capture_finalize(executor, captured)

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
        environment_cloud_provider=_f4_dummy_cloud,
    )

    assert captured["final_status"] == "diagnostic-pass"
    assert client.sends >= 2, (
        "a transient rejection must be retried (observed sends "
        f"={client.sends}, expected >= 2)"
    )
    assert record["status"] == "diagnostic-pass"


def test_f3_cartesian_retreat_fails_closed_when_rejection_persists(tmp_path, monkeypatch):
    """F3: when the rejection persists beyond the bounded retry budget the
    retreat fails closed with the rejection reason (never retries forever)."""
    import validation.integrated_gate_executor as exec_mod

    from validation.integrated_gate_executor import (
        D_FORBIDDEN_EVENTS,
        STAGE_D_REQUIRED_EVENT_ORDER,
    )

    def _fake_cartesian_goal(target_pose, *, env_points=None):
        return types.SimpleNamespace(target_pose=target_pose, env_points=env_points)

    monkeypatch.setattr(exec_mod, "build_cartesian_move_goal", _fake_cartesian_goal)
    executor = _flow_executor(
        tmp_path,
        "qualification-moveit-cartesian-retreat",
        required_event_order=STAGE_D_REQUIRED_EVENT_ORDER["retreat"],
        forbidden_events=D_FORBIDDEN_EVENTS,
    )
    executor.config = {
        **executor.config,
        "thresholds": {
            **executor.config["thresholds"],
            "cartesian_reject_retry_budget_s": 0.1,
        },
    }
    # Reject every send (never becomes ready).
    client = _F3RejectThenAcceptClient(reject_count=10 ** 9)
    executor._action_clients = {
        "/xarm_gripper/gripper_action": client,
        "/cartesian_move_action": client,
    }
    executor._journal_snapshot_d = lambda event: "recorded"
    executor.ros = {"serialize_message": lambda msg: b"env-cloud"}
    # Keep the real _finalize_d_attempt so the record carries the rejection error.
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
        environment_cloud_provider=_f4_dummy_cloud,
    )
    assert record["status"] == "evidence-invalid"
    assert "cartesian goal was rejected" in record["execute_error"]


def test_f3_cartesian_rejection_reason_persists_to_execution_json(tmp_path, monkeypatch):
    """RED (R2): the durable ``integrated-execution.json`` must record the
    ``execute_error`` rejection reason.  rerun-6 showed ``execute_error: null``
    on a 20 s cartesian-rejection attempt even though the retry loop set it —
    ``_write_d_artifacts`` dropped the field from the JSON write.  Without the
    reason in the durable artifact the operator/verifier cannot distinguish a
    server-unavailable failure from a persistent pick_and_place rejection."""
    import json as _json

    import validation.integrated_gate_executor as exec_mod

    from validation.integrated_gate_executor import (
        D_FORBIDDEN_EVENTS,
        STAGE_D_REQUIRED_EVENT_ORDER,
    )

    def _fake_cartesian_goal(target_pose, *, env_points=None):
        return types.SimpleNamespace(target_pose=target_pose, env_points=env_points)

    monkeypatch.setattr(exec_mod, "build_cartesian_move_goal", _fake_cartesian_goal)
    executor = _flow_executor(
        tmp_path,
        "qualification-moveit-cartesian-retreat",
        required_event_order=STAGE_D_REQUIRED_EVENT_ORDER["retreat"],
        forbidden_events=D_FORBIDDEN_EVENTS,
    )
    executor.config = {
        **executor.config,
        "thresholds": {
            **executor.config["thresholds"],
            "cartesian_reject_retry_budget_s": 0.1,
        },
    }
    # Reject every send (never becomes ready) — the real pick_and_place contract
    # rejection (joint state topic contract invalid) persists for the full budget.
    client = _F3RejectThenAcceptClient(reject_count=10 ** 9)
    executor._action_clients = {
        "/xarm_gripper/gripper_action": client,
        "/cartesian_move_action": client,
    }
    executor._journal_snapshot_d = lambda event: "recorded"
    executor.ros = {"serialize_message": lambda msg: b"env-cloud"}
    source = {
        "frame_id": "base_link",
        "xyz": [0.2, 0.0, 0.72],
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "identity": "tcp-observation-1",
        "age_s": 0.05,
    }
    executor.run_cartesian_retreat(
        "qualification-moveit-cartesian-retreat",
        current_tcp_pose_provider=lambda: source,
        environment_cloud_provider=_f4_dummy_cloud,
    )
    execution_path = tmp_path / "integrated-execution.json"
    assert execution_path.exists(), "integrated-execution.json was not written"
    payload = _json.loads(execution_path.read_text(encoding="utf-8"))
    assert payload.get("execute_error"), (
        "integrated-execution.json must record the execute_error rejection "
        f"reason (got {payload.get('execute_error')!r})"
    )
    assert "cartesian goal was rejected" in payload["execute_error"]


# ---------------------------------------------------------------------------
# F1 — production-equivalent execution slowdown for execute-joint/execute-pose.
#
# Production pick_and_place applies ``apply_execution_slowdown(k=2.0)`` to every
# planned trajectory AFTER planning, BEFORE execution (grasp_node.hpp): time
# scales by k, velocities by 1/k, accelerations by 1/k^2.  The Stage-D executor
# did NOT, so its trajectories executed twice as fast as production and the FJT
# controller was still settling at the execution-terminal frame — the verifier
# measured ``joint_final_error_rad`` 0.0165 > 0.01.  These tests pin the exact
# production scaling so the qualification validates the PRODUCTION integration.
# ---------------------------------------------------------------------------


def _f1_point(t_sec: float, velocities, accelerations):
    from builtin_interfaces.msg import Duration

    whole = int(t_sec)
    point = types.SimpleNamespace(
        velocities=list(velocities),
        accelerations=list(accelerations),
        time_from_start=Duration(),
    )
    point.time_from_start.sec = whole
    point.time_from_start.nanosec = int(round((t_sec - whole) * 1e9))
    return point


def _f1_trajectory(points):
    return types.SimpleNamespace(
        joint_trajectory=types.SimpleNamespace(points=list(points))
    )


def test_f1_apply_execution_slowdown_matches_production_scaling():
    """RED (F1): the executor must expose an ``apply_execution_slowdown`` helper
    matching production: time_from_start *= k, velocities /= k, accelerations /=
    k^2 for every trajectory point."""
    from validation.integrated_gate_executor import apply_execution_slowdown

    pts = [
        _f1_point(1.0, [0.4, 0.2], [0.8, 0.4]),
        _f1_point(2.5, [0.0, 0.0], [0.0, 0.0]),
    ]
    apply_execution_slowdown(_f1_trajectory(pts), k=2.0)

    assert pts[0].time_from_start.sec == 2
    assert pts[0].time_from_start.nanosec == 0
    assert pts[1].time_from_start.sec == 5
    assert pts[1].time_from_start.nanosec == 0
    assert pts[0].velocities == pytest.approx([0.2, 0.1])
    assert pts[0].accelerations == pytest.approx([0.2, 0.1])
    assert pts[1].velocities == pytest.approx([0.0, 0.0])
    assert pts[1].accelerations == pytest.approx([0.0, 0.0])


def test_f1_apply_execution_slowdown_noop_below_or_at_unity():
    """A slowdown factor <= 1.0 is a no-op (production contract)."""
    from validation.integrated_gate_executor import apply_execution_slowdown

    pts = [_f1_point(1.0, [0.4], [0.8])]
    apply_execution_slowdown(_f1_trajectory(pts), k=1.0)
    assert pts[0].time_from_start.sec == 1
    assert pts[0].velocities == pytest.approx([0.4])
    assert pts[0].accelerations == pytest.approx([0.8])


def test_p2_execute_pose_uses_pose_specific_slowdown_k():
    """RED (P2): the Cartesian/TCP execute-pose path needs a higher execution
    slowdown than execute-joint — the FJT controller aborts
    PATH_TOLERANCE_VIOLATED on the pose path (live rerun-5 joint2 position error
    -1.017 > 1.0 tolerance) while execute-joint passes at k=2.0.  The executor
    must select a pose-specific slowdown factor (3.0) for execute-pose while
    keeping the production k=2.0 for execute-joint."""
    from validation.integrated_gate_executor import execution_slowdown_k

    assert execution_slowdown_k({"kind": "execute-pose"}) == 3.0
    assert execution_slowdown_k({"kind": "execute-joint"}) == 2.0
    # Non-pose D kinds keep the production k=2.0.
    for kind in ("execute-joint", "safety", "cancel", "retreat", "gripper"):
        assert execution_slowdown_k({"kind": kind}) == 2.0, kind


def _p_point(joint_positions, t_sec: float = 1.0):
    """Build a trajectory point with a 7-joint positions array (no velocities)."""
    point = types.SimpleNamespace(
        positions=[float(value) for value in joint_positions],
        velocities=[],
        accelerations=[],
    )
    point.time_from_start = types.SimpleNamespace(sec=int(t_sec), nanosec=0)
    return point


def _p_point_array(joint_positions, t_sec: float = 1.0):
    """Build a trajectory point exactly like ``_p_point`` but with the positions
    stored in an ``array.array`` (the real rclpy ``JointTrajectoryPoint.positions``
    shape), to prove the type-guarded unwrap path handles live messages."""
    point = types.SimpleNamespace(
        positions=array.array("d", [float(value) for value in joint_positions]),
        velocities=[],
        accelerations=[],
    )
    point.time_from_start = types.SimpleNamespace(sec=int(t_sec), nanosec=0)
    return point


def test_p_unwrap_joint_trajectory_removes_full_turn_wrap():
    """RED (P): the execute-pose OMPL plan wrapped joint1 through -4.478 rad
    (a full extra turn) instead of the equivalent +1.805 rad short path, so the
    arm swung the long way around and the FJT controller aborted
    PATH_TOLERANCE_VIOLATED (state tolerance, joint2 error -1.044 rad).  The
    executor must unwrap every joint so each step stays within (-pi, pi]."""
    from validation.integrated_gate_executor import unwrap_joint_trajectory

    # Live execute-pose shape: joint1 0 -> -4.478 (wrapped), other joints small.
    pts = [
        _p_point([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], t_sec=0.0),
        _p_point([-4.478, 0.7, 0.0, 0.9, 0.0, 0.5, 0.0], t_sec=1.0),
    ]
    unwrap_joint_trajectory(_f1_trajectory(pts))
    j1 = [point.positions[0] for point in pts]
    # joint1 must take the short path: 0 -> +1.805 (=-4.478+2*pi), not -4.478.
    assert abs(j1[-1] - 1.805) <= 0.001, f"joint1 not unwrapped to short path: {j1}"
    assert abs(j1[0] - 0.0) <= 1e-9, "start position must stay at the current state"
    # Every step within (-pi, pi].
    for point in pts:
        for value in point.positions:
            assert -math.pi <= value <= math.pi, f"step not within (-pi, pi]: {value}"


def test_p_unwrap_joint_trajectory_handles_multi_point_and_other_joints():
    """A multi-waypoint path that wraps mid-way is unwrapped only after the jump,
    and joints already within (-pi, pi] are left untouched."""
    from validation.integrated_gate_executor import unwrap_joint_trajectory

    pts = [
        _p_point([0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0], t_sec=0.0),
        _p_point([0.5, 0.31, 0.0, 0.0, 0.0, 0.0, 0.0], t_sec=1.0),
        _p_point([-4.478, 0.32, 0.0, 0.0, 0.0, 0.0, 0.0], t_sec=2.0),
        _p_point([-4.2, 0.33, 0.0, 0.0, 0.0, 0.0, 0.0], t_sec=3.0),
    ]
    unwrap_joint_trajectory(_f1_trajectory(pts))
    j1 = [point.positions[0] for point in pts]
    assert abs(j1[0] - 0.0) <= 1e-9
    assert abs(j1[1] - 0.5) <= 1e-9
    # After the wrap jump, subsequent points are shifted by +2*pi.
    assert abs(j1[2] - (-4.478 + 2 * math.pi)) <= 1e-9, f"j1[2]={j1[2]}"
    assert abs(j1[3] - (-4.2 + 2 * math.pi)) <= 1e-9, f"j1[3]={j1[3]}"
    # Non-wrapped joint (joint2) untouched.
    j2 = [point.positions[1] for point in pts]
    assert j2 == pytest.approx([0.3, 0.31, 0.32, 0.33])


def test_p_unwrap_joint_trajectory_removes_full_turn_wrap_array():
    """RED (P): real rclpy ``JointTrajectoryPoint.positions`` is an
    ``array.array``, not a ``list``/``tuple``, so the ``isinstance`` guards in
    ``unwrap_joint_trajectory`` skip every point and the full-turn wrap survives.
    With the type-guard fix the array shape must unwrap exactly like the list
    shape (joint1 0 -> -4.478 must take the short path 0 -> +1.805)."""
    from validation.integrated_gate_executor import unwrap_joint_trajectory

    # Live execute-pose shape but stored in array.array (real message layout).
    pts = [
        _p_point_array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], t_sec=0.0),
        _p_point_array([-4.478, 0.7, 0.0, 0.9, 0.0, 0.5, 0.0], t_sec=1.0),
    ]
    unwrap_joint_trajectory(_f1_trajectory(pts))
    j1 = [point.positions[0] for point in pts]
    assert abs(j1[-1] - 1.805) <= 0.001, f"joint1 not unwrapped to short path: {j1}"
    assert abs(j1[0] - 0.0) <= 1e-9, "start position must stay at the current state"
    for point in pts:
        for value in point.positions:
            assert -math.pi <= value <= math.pi, f"step not within (-pi, pi]: {value}"


def test_canonical_goal_endpoint_returns_canonical_when_wrapped():
    """RED (P3): a pose-goal OMPL plan can sample a WRAPPED IK goal — a
    continuous joint offset by a full 2*pi turn (joint5 -> -6.283 instead of 0,
    joint7 -> +5.519 instead of -0.764).  Per-step unwrap cannot shorten a
    smooth full-turn wind, so ``canonical_goal_endpoint`` must canonicalize the
    endpoint relative to the start into (-pi, pi] for each continuous joint
    (joint1/3/5/7) and return a re-plan goal; a short endpoint returns None."""
    from validation.integrated_gate_executor import canonical_goal_endpoint

    # joint5 winds smoothly 0 -> -6.283 (a full -2*pi turn), joint7 winds
    # 0 -> +5.519 (a +2*pi turn short of -0.764); both stored array.array.
    pts = [
        _p_point_array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], t_sec=0.0),
        _p_point_array([0.0, 0.0, 0.0, 0.0, -0.5, 0.0, 0.5], t_sec=1.0),
        _p_point_array([0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0], t_sec=2.0),
        _p_point_array([0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 2.0], t_sec=3.0),
        _p_point_array([0.0, 0.0, 0.0, 0.0, -4.0, 0.0, 4.0], t_sec=4.0),
        _p_point_array([0.0, 0.0, 0.0, 0.0, -6.283, 0.0, 5.519], t_sec=5.0),
    ]
    canonical = canonical_goal_endpoint(_f1_trajectory(pts))
    assert canonical is not None, "wrapped endpoint must canonicalize to a re-plan goal"
    assert len(canonical) == 7
    # joint5: -6.283 is within (-pi, pi] of start 0 after dropping one turn -> ~0.
    assert abs(canonical[4] - 0.0) <= 0.001, f"joint5 not canonicalized: {canonical[4]}"
    # joint7: +5.519 minus one turn = -0.764.
    assert abs(canonical[6] - (5.519 - 2 * math.pi)) <= 0.001, \
        f"joint7 not canonicalized: {canonical[6]}"


def test_canonical_goal_endpoint_none_when_short():
    """A trajectory whose endpoint is already within (-pi, pi] of the start on
    every continuous joint is short — no re-plan goal is needed (None)."""
    from validation.integrated_gate_executor import canonical_goal_endpoint

    pts = [
        _p_point_array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], t_sec=0.0),
        _p_point_array([0.5, 0.3, 0.2, 0.1, 1.5, -0.3, -0.764], t_sec=1.0),
    ]
    assert canonical_goal_endpoint(_f1_trajectory(pts)) is None


def test_r1_plan_invalid_postprocessing_signature():
    """RED (R1): the Gate-D execute sequence retries the plan-only goal ONLY on
    the OMPL invalid-postprocessing signature — a ``diagnostic-fail`` plan that
    still returned a NON-EMPTY ``planned_trajectory``.  rerun-7 execute-pose
    failed exactly this way: the parallel planner found a 46-state solution, the
    pipeline's dense ``isPathValid`` rejected the simplified path at waypoint 11
    (link6<->base_link) with ``INVALID_MOTION_PLAN``, yet the trajectory digest
    was populated.  A genuinely-blocked goal has an EMPTY trajectory and must
    never be retried."""
    from types import SimpleNamespace

    from validation.integrated_gate_executor import _plan_invalid_postprocessing

    with_points = SimpleNamespace(joint_trajectory=SimpleNamespace(points=[object()]))
    empty = SimpleNamespace(joint_trajectory=SimpleNamespace(points=[]))
    assert _plan_invalid_postprocessing("diagnostic-fail", with_points) is True
    # Success and blocked signatures never retry.
    assert _plan_invalid_postprocessing("diagnostic-pass", with_points) is False
    assert _plan_invalid_postprocessing("diagnostic-fail", empty) is False
    assert _plan_invalid_postprocessing("diagnostic-fail", None) is False
    assert _plan_invalid_postprocessing("evidence-invalid", with_points) is False


# ---------------------------------------------------------------------------
# F3.4: qualification visual capture consumer (ROS-free, fake app/backend).
#
# The consumer services the executor-written ``visual-capture-requests.jsonl``
# journal, captures each canonical request once to both cameras, co-tenants the
# executor diagnostic records without treating them as capture requests, and
# never re-captures a sequence — even across a consumer process restart (the
# durable ``visual-keyframes.jsonl`` journal seeds the handled set).
# ---------------------------------------------------------------------------


class _FakeRenderBackend:
    """Deterministic physics-truth double for the Isaac backend seam."""

    def __init__(
        self,
        *,
        dt: float = 0.01,
        physics_frame_index: int = 102,
        simulation_time: float = 1.02,
    ) -> None:
        self.dt = dt
        self.physics_frame_index = physics_frame_index
        self.simulation_time = simulation_time

    def render_frame(self) -> None:
        return None


class _FakeSensor:
    def __init__(self, array: object) -> None:
        self.array = array

    def get_data(self, annotator: str):
        if annotator == "rgb":
            return self.array, {"width": 960, "height": 540, "frames": 1}
        return None, {}

    def close(self) -> None:
        return None


def _rgb_array():
    import numpy as np

    return (
        np.arange(540 * 960 * 3, dtype=np.int32).reshape(540, 960, 3) % 256
    ).astype(np.uint8)


def _make_capture(monkeypatch, tmp_path, backend, *, gate: str = "g"):
    from simulation.tinker_sim_isaac.qualification_visual_capture import (
        QualificationVisualCapture,
    )

    sensors = {
        "overview": _FakeSensor(_rgb_array()),
        "manipulation_closeup": _FakeSensor(_rgb_array()),
    }

    def _fake_initialize(self):
        self._sensors = dict(sensors)

    monkeypatch.setattr(QualificationVisualCapture, "_initialize_cameras", _fake_initialize)
    return QualificationVisualCapture(
        app=object(),
        backend=backend,
        attempt_dir=tmp_path,
        gate=gate,
        event_pump=None,
    )


def _write_visual_request(tmp_path, record: Mapping[str, object]) -> None:
    path = tmp_path / "visual-capture-requests.jsonl"
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, sort_keys=True) + "\n")


def _canonical_visual_request(**overrides: object) -> dict[str, object]:
    request: dict[str, object] = {
        "schema_version": 1,
        "sequence": 1,
        "gate": "g",
        "event": "cancel-execution-start",
        "simulated_timestamp": 1.0,
        "source_execution_event_sequence": 7,
    }
    request.update(overrides)
    return request


def test_f34_consumer_captures_both_cameras_from_canonical_request(tmp_path, monkeypatch):
    """F3.4: a canonical request drives one capture per camera (two PNGs plus two
    durable keyframe records) with the exact six-key source binding and a latency
    inside the bounded contract."""
    from simulation.tinker_sim_isaac.qualification_visual_capture import (
        MAX_CAPTURE_LATENCY_FRAMES,
    )

    backend = _FakeRenderBackend(physics_frame_index=102, simulation_time=1.02)
    capture = _make_capture(monkeypatch, tmp_path, backend)
    _write_visual_request(tmp_path, _canonical_visual_request())
    capture.poll()

    assert capture._errors == []
    pngs = sorted((tmp_path / "visual/source").glob("*.png"))
    assert len(pngs) == 2
    records = [
        json.loads(line)
        for line in (tmp_path / "visual-keyframes.jsonl")
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    ]
    assert len(records) == 2
    assert {record["camera"] for record in records} == {"overview", "manipulation_closeup"}
    for record in records:
        assert record["schema_version"] == 1
        assert record["gate"] == "g"
        assert record["event"] == "cancel-execution-start"
        assert record["request_sequence"] == 1
        assert record["execution_event_sequence"] == 7
        assert record["requested_simulated_timestamp"] == 1.0
        assert record["requested_physics_frame_index"] == 100
        assert record["capture_latency_frames"] == 2
        assert 0 <= record["capture_latency_frames"] <= MAX_CAPTURE_LATENCY_FRAMES
        assert (tmp_path / record["path"]).is_file()
        assert (tmp_path / record["path"]).stat().st_size > 0
    assert capture._handled_sequences == {1}


def test_f34_consumer_skips_executor_diagnostic_records(tmp_path, monkeypatch):
    """F3.4: executor diagnostic records (diagnostic-only, no canonical
    sequence/gate/event/timestamp) are co-tenanted but never capture-driving."""
    backend = _FakeRenderBackend()
    capture = _make_capture(monkeypatch, tmp_path, backend)
    _write_visual_request(
        tmp_path,
        {
            "schema_version": 1,
            "report_revision": 1,
            "scenario_id": "qualification-moveit-cancel",
            "phase": "before",
            "capture": {"kind": "gate-d-diagnostic", "target": None},
            "diagnostic_only": True,
        },
    )
    capture.poll()
    assert capture._errors == []
    assert capture._handled_sequences == set()
    assert not (tmp_path / "visual-keyframes.jsonl").exists()
    assert not list((tmp_path / "visual/source").glob("*.png"))


def test_f34_consumer_dedupes_malformed_requests(tmp_path, monkeypatch):
    """F3.4: repeated malformed records are reported exactly once (no error loop)."""
    backend = _FakeRenderBackend()
    capture = _make_capture(monkeypatch, tmp_path, backend)
    malformed = {"schema_version": 1, "event": "terminal"}  # missing sequence/timestamp
    _write_visual_request(tmp_path, malformed)
    _write_visual_request(tmp_path, malformed)
    capture.poll()
    capture.poll()
    assert len(capture._errors) == 1
    assert "unrecognized visual capture request record" in capture._errors[0]


def test_f34_consumer_defers_future_timestamp_requests(tmp_path, monkeypatch):
    """F3.4: a request whose simulated timestamp is still in the future is
    deferred silently (never an error, never a handled sequence)."""
    backend = _FakeRenderBackend(physics_frame_index=102, simulation_time=1.02)
    capture = _make_capture(monkeypatch, tmp_path, backend)
    _write_visual_request(
        tmp_path,
        _canonical_visual_request(simulated_timestamp=99.0),
    )
    capture.poll()
    assert capture._errors == []
    assert 1 not in capture._handled_sequences
    assert not (tmp_path / "visual-keyframes.jsonl").exists()


def test_f34_consumer_is_at_most_once_across_polls(tmp_path, monkeypatch):
    """F3.4: re-polling the same request journal never re-captures an already
    handled sequence."""
    backend = _FakeRenderBackend(physics_frame_index=102, simulation_time=1.02)
    capture = _make_capture(monkeypatch, tmp_path, backend)
    _write_visual_request(tmp_path, _canonical_visual_request())
    capture.poll()
    capture.poll()  # same file; sequence 1 already handled
    records = [
        json.loads(line)
        for line in (tmp_path / "visual-keyframes.jsonl")
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    ]
    assert len(records) == 2  # one per camera, never duplicated


def test_f34_consumer_durable_restart_seeds_handled_sequences(tmp_path, monkeypatch):
    """F3.4: a consumer process restart seeds its handled set from the durable
    keyframe journal, so an already-captured sequence is never re-captured."""
    backend = _FakeRenderBackend(physics_frame_index=102, simulation_time=1.02)
    # Seed BOTH cameras' durable keyframes for sequence 5 (a fully completed
    # capture before the crash/restart).
    durable_rows = []
    for camera in ("overview", "manipulation_closeup"):
        durable_rows.append(
            {
                "schema_version": 1,
                "gate": "g",
                "event": "cancel-trigger",
                "request_sequence": 5,
                "execution_event_sequence": 3,
                "camera": camera,
                "path": f"visual/source/0005-cancel-trigger-{camera}.png",
            }
        )
    (tmp_path / "visual-keyframes.jsonl").write_text(
        "\n".join(json.dumps(row, sort_keys=True) for row in durable_rows) + "\n",
        encoding="utf-8",
    )
    capture = _make_capture(monkeypatch, tmp_path, backend)
    _write_visual_request(tmp_path, _canonical_visual_request(sequence=5))
    capture.poll()
    assert capture._handled_sequences == {5}
    records = [
        json.loads(line)
        for line in (tmp_path / "visual-keyframes.jsonl")
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    ]
    assert len(records) == 2  # the pre-existing durable records, no re-capture


def test_f34_consumer_crash_after_camera1_restart_completes_camera2(tmp_path, monkeypatch):
    """F4.5: a crash after camera-1's keyframe but before camera-2's must not
    mark the request sequence durably complete.  On restart only the missing
    camera is captured once; camera-1 is never duplicated."""
    backend = _FakeRenderBackend(physics_frame_index=102, simulation_time=1.02)
    # Simulate the crash: only camera-1 (overview) durably recorded sequence 5
    # for the same canonical request (event cancel-execution-start) that will be
    # re-polled on restart.
    (tmp_path / "visual-keyframes.jsonl").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "gate": "g",
                "event": "cancel-execution-start",
                "request_sequence": 5,
                "execution_event_sequence": 3,
                "camera": "overview",
                "path": "visual/source/0005-cancel-execution-start-overview.png",
            }
        )
        + "\n",
        encoding="utf-8",
    )
    (tmp_path / "visual/source").mkdir(parents=True, exist_ok=True)
    (tmp_path / "visual/source/0005-cancel-execution-start-overview.png").write_bytes(
        _rgb_array().tobytes()
    )
    capture = _make_capture(monkeypatch, tmp_path, backend)
    _write_visual_request(tmp_path, _canonical_visual_request(sequence=5))
    capture.poll()
    # Sequence 5 is not durably complete until camera-2 is also captured.
    assert capture._handled_sequences == {5}
    records = [
        json.loads(line)
        for line in (tmp_path / "visual-keyframes.jsonl")
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    ]
    assert len(records) == 2  # camera-1 durable + camera-2 produced once
    by_camera = {record["camera"] for record in records}
    assert by_camera == {"overview", "manipulation_closeup"}
    overview_records = [record for record in records if record["camera"] == "overview"]
    assert len(overview_records) == 1  # camera-1 never duplicated
    # Camera-2 is captured under the request's canonical event.
    pngs = sorted(path.name for path in (tmp_path / "visual/source").glob("*.png"))
    assert pngs == [
        "0005-cancel-execution-start-manipulation_closeup.png",
        "0005-cancel-execution-start-overview.png",
    ]
    # Camera-2's new keyframe carries the real latency arithmetic.
    new_record = next(record for record in records if record["camera"] == "manipulation_closeup")
    assert new_record["requested_physics_frame_index"] == 100
    assert new_record["raw_frame_index"] == 102
    assert new_record["capture_latency_frames"] == 2
    assert new_record["event"] == "cancel-execution-start"


def test_f34_consumer_gate_mismatch_is_a_capture_failure(tmp_path, monkeypatch):
    """F3.4: a request for a different gate than the consumer's gate fails closed
    and is durably marked handled (never captured under the wrong gate)."""
    backend = _FakeRenderBackend(physics_frame_index=102, simulation_time=1.02)
    capture = _make_capture(monkeypatch, tmp_path, backend, gate="g")
    _write_visual_request(tmp_path, _canonical_visual_request(gate="other-gate"))
    capture.poll()
    assert len(capture._errors) == 1
    assert "does not match" in capture._errors[0]
    assert 1 in capture._handled_sequences


def test_f34_consumer_close_writes_summary(tmp_path, monkeypatch):
    """F3.4: close() writes the durable visual-keyframes.json summary carrying the
    bounded capture-latency contract, the recorded keyframes, and any errors."""
    from simulation.tinker_sim_isaac.qualification_visual_capture import (
        MAX_CAPTURE_LATENCY_FRAMES,
    )

    backend = _FakeRenderBackend(physics_frame_index=102, simulation_time=1.02)
    capture = _make_capture(monkeypatch, tmp_path, backend)
    _write_visual_request(tmp_path, _canonical_visual_request())
    capture.poll()
    capture.close()
    summary = json.loads((tmp_path / "visual-keyframes.json").read_text(encoding="utf-8"))
    assert summary["gate"] == "g"
    assert summary["capture_latency_contract"]["max_frames"] == MAX_CAPTURE_LATENCY_FRAMES
    assert summary["capture_latency_contract"]["unit"] == "physics_frames"
    assert len(summary["records"]) == 2
    assert summary["errors"] == []


def test_f34_consumer_from_environment_disabled(monkeypatch):
    """F3.4: from_environment returns None when visual evidence is not enabled."""
    monkeypatch.delenv("TINKER_SIM_VISUAL_EVIDENCE", raising=False)
    from simulation.tinker_sim_isaac.qualification_visual_capture import (
        QualificationVisualCapture,
    )

    assert QualificationVisualCapture.from_environment(app=object(), backend=object()) is None


def test_f34_consumer_from_environment_requires_attempt_and_gate(monkeypatch):
    """F3.4: enabled visual evidence without the attempt dir or gate fails closed."""
    monkeypatch.setenv("TINKER_SIM_VISUAL_EVIDENCE", "1")
    monkeypatch.delenv("TINKER_SIM_ATTEMPT_DIR", raising=False)
    monkeypatch.delenv("TINKER_SIM_QUALIFICATION_GATE", raising=False)
    from simulation.tinker_sim_isaac.qualification_visual_capture import (
        QualificationVisualCapture,
    )

    with pytest.raises(RuntimeError, match="TINKER_SIM_ATTEMPT_DIR"):
        QualificationVisualCapture.from_environment(app=object(), backend=object())


# ---------------------------------------------------------------------------
# F5.4: partial-camera stale-restart expiry + durable PNG-before-journal order.
#
# After camera 1 is durably captured, a restarted consumer may observe the
# original request more than MAX_CAPTURE_LATENCY_FRAMES late.  The sequence must
# expire fail-closed: one deduplicated terminal error, camera-1 evidence
# preserved, no camera-2 fabrication, no retry/error growth across polls or a
# process restart, and each PNG durably persisted before its keyframe row.
# ---------------------------------------------------------------------------


def test_f54_stale_partial_restart_terminal_no_retry_no_fabrication(tmp_path, monkeypatch):
    """F5.4: a partially captured sequence observed more than
    MAX_CAPTURE_LATENCY_FRAMES late on restart is terminal.  Exactly one
    deduplicated error is recorded, camera-1 evidence is preserved, camera-2 is
    never fabricated, and repeated polls/restarts never grow identical errors."""
    from simulation.tinker_sim_isaac.qualification_visual_capture import (
        MAX_CAPTURE_LATENCY_FRAMES,
    )

    # Durable camera-1 (overview) keyframe from a prior partial capture.
    (tmp_path / "visual-keyframes.jsonl").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "gate": "g",
                "event": "cancel-execution-start",
                "request_sequence": 5,
                "execution_event_sequence": 3,
                "camera": "overview",
                "path": "visual/source/0005-cancel-execution-start-overview.png",
            }
        )
        + "\n",
        encoding="utf-8",
    )
    (tmp_path / "visual/source").mkdir(parents=True, exist_ok=True)
    (tmp_path / "visual/source/0005-cancel-execution-start-overview.png").write_bytes(
        _rgb_array().tobytes()
    )
    # The restarted consumer observes the request more than MAX frames late.
    late_frames = MAX_CAPTURE_LATENCY_FRAMES + 5
    backend = _FakeRenderBackend(
        physics_frame_index=100 + late_frames,
        simulation_time=(100 + late_frames) / 100.0,
    )
    capture = _make_capture(monkeypatch, tmp_path, backend)
    _write_visual_request(tmp_path, _canonical_visual_request(sequence=5))
    capture.poll()

    # One terminal error, deduplicated, with the sequence marked handled.
    assert len(capture._errors) == 1
    assert "latency is outside the bounded contract" in capture._errors[0]
    assert 5 in capture._handled_sequences
    # Camera-1 evidence preserved; camera-2 never fabricated.
    pngs = sorted(path.name for path in (tmp_path / "visual/source").glob("*.png"))
    assert pngs == ["0005-cancel-execution-start-overview.png"]
    records = [
        json.loads(line)
        for line in (tmp_path / "visual-keyframes.jsonl")
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    ]
    assert [record["camera"] for record in records] == ["overview"]
    # The terminal decision is durable so a restart never retries it.
    terminal = json.loads((tmp_path / "visual-terminal.json").read_text(encoding="utf-8"))
    assert terminal["terminal_sequences"] == [5]
    # Repeated polls never grow identical errors.
    capture.poll()
    capture.poll()
    assert len(capture._errors) == 1
    # A process restart (new instance, same durable state) never retries and
    # never grows an error.
    restarted = _make_capture(monkeypatch, tmp_path, backend)
    restarted.poll()
    assert restarted._errors == []
    assert 5 in restarted._handled_sequences


def test_f54_in_range_partial_restart_still_completes_missing_camera(tmp_path, monkeypatch):
    """F5.4: when the restarted consumer still satisfies the latency contract,
    the round-4 behavior is preserved: only the missing camera is captured and
    camera-1 is never duplicated."""
    backend = _FakeRenderBackend(physics_frame_index=102, simulation_time=1.02)
    (tmp_path / "visual-keyframes.jsonl").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "gate": "g",
                "event": "cancel-execution-start",
                "request_sequence": 5,
                "execution_event_sequence": 3,
                "camera": "overview",
                "path": "visual/source/0005-cancel-execution-start-overview.png",
            }
        )
        + "\n",
        encoding="utf-8",
    )
    (tmp_path / "visual/source").mkdir(parents=True, exist_ok=True)
    (tmp_path / "visual/source/0005-cancel-execution-start-overview.png").write_bytes(
        _rgb_array().tobytes()
    )
    capture = _make_capture(monkeypatch, tmp_path, backend)
    _write_visual_request(tmp_path, _canonical_visual_request(sequence=5))
    capture.poll()
    assert capture._errors == []
    assert 5 in capture._handled_sequences
    records = [
        json.loads(line)
        for line in (tmp_path / "visual-keyframes.jsonl")
        .read_text(encoding="utf-8")
        .strip()
        .splitlines()
    ]
    by_camera = {record["camera"] for record in records}
    assert by_camera == {"overview", "manipulation_closeup"}
    overview_records = [record for record in records if record["camera"] == "overview"]
    assert len(overview_records) == 1  # camera-1 never duplicated
    assert not (tmp_path / "visual-terminal.json").exists()


def test_f54_image_persistence_failure_cannot_append_keyframe(tmp_path, monkeypatch):
    """F5.4: a failed image fsync/replace must never append a keyframe journal
    row.  A journal row is only durable after its referenced image bytes."""
    backend = _FakeRenderBackend(physics_frame_index=102, simulation_time=1.02)
    capture = _make_capture(monkeypatch, tmp_path, backend)
    _write_visual_request(tmp_path, _canonical_visual_request())

    real_replace = os.replace

    def _failing_replace(src, dst):
        if str(dst).endswith(".png"):
            raise OSError("simulated image replace failure")
        return real_replace(src, dst)

    monkeypatch.setattr("os.replace", _failing_replace)
    capture.poll()

    # No keyframe row may be durable before its referenced image bytes.
    keyframes_path = tmp_path / "visual-keyframes.jsonl"
    if keyframes_path.exists():
        assert keyframes_path.stat().st_size == 0
    # No final PNG is left behind and the temporary file is cleaned up.
    assert not list((tmp_path / "visual/source").glob("*.png"))
    assert not list((tmp_path / "visual/source").glob(".*.png.*"))
    # The failure is reported once and the sequence is not retried (no error loop).
    assert len(capture._errors) == 1
    assert "failed" in capture._errors[0]
    capture.poll()


def test_scene_callback_preserves_fixture_world_on_empty_diff():
    """RED: ``_make_scene_callback`` must preserve the prior fixture world fields
    when a diff message (``is_diff=True``) normalizes to empty owned/attached ids.

    Live defect: a MoveIt ``PlanningScene`` diff carries only changes; an empty
    ``world`` in a diff means "no world changes", not "world is now empty".
    The callback unconditionally replaces ``_latest_planning_scene`` with the
    normalized result, so a diff normalizing to ``owned_ids=[]`` erases the
    fixture world (owned ids, geometry, digest).  Desired: accept the newer
    ``scene_sequence``/``source`` and keep ``_planning_scene_invalid`` clear,
    while preserving the prior fixture world fields (``owned_ids``,
    ``fixture_geometry``, ``fixture_geometry_digest``).
    """
    from validation.integrated_gate_executor import IntegratedGateExecutor

    executor = object.__new__(IntegratedGateExecutor)
    executor._scene_sequence = 10
    executor.fixture_revision = "r1"
    executor.scenario = {
        "planning_scene_declaration": {
            "objects": [{"id": "fixture_a"}, {"id": "fixture_b"}],
            "diagnostic_objects": [],
        }
    }
    executor.ros = {"serialize_message": lambda _message: b"\x00\x01"}
    executor._latest_planning_scene = {
        "scene_sequence": 10,
        "scene_timestamp": 1.0,
        "owned_ids": ["fixture_a", "fixture_b"],
        "attached_ids": [],
        "attached_links": {},
        "touch_links": {},
        "fixture_revision": "r1",
        "fixture_geometry_digest": "FIXTURE_DIGEST",
        "fixture_geometry": [{"id": "fixture_a"}, {"id": "fixture_b"}],
        "scene_revision_digest": "scene1",
        "acm_digest": "acm1",
        "robot_state_digest": "rs1",
        "source": "/sim/status/planning_scene_fixture",
    }
    executor._planning_scene_invalid = False
    executor._scene_invalid_sequence = None

    # A diff message whose world carries no collision objects normalizes to
    # empty owned/attached ids but a newer sequence/source.
    message = types.SimpleNamespace(
        is_diff=True,
        world=types.SimpleNamespace(collision_objects=[]),
        robot_state=types.SimpleNamespace(attached_collision_objects=[]),
        allowed_collision_matrix=types.SimpleNamespace(),
    )

    callback = executor._make_scene_callback("/get_planning_scene")
    callback(message)

    cache = executor._latest_planning_scene
    assert cache["scene_sequence"] == 11, cache["scene_sequence"]
    assert cache["source"] == "/get_planning_scene", cache["source"]
    assert executor._planning_scene_invalid is False
    assert executor._scene_invalid_sequence is None
    # The prior fixture world fields survive the empty diff.
    assert cache["owned_ids"] == ["fixture_a", "fixture_b"], cache["owned_ids"]
    assert cache["fixture_geometry_digest"] == "FIXTURE_DIGEST", cache["fixture_geometry_digest"]
    assert cache["fixture_geometry"] == [{"id": "fixture_a"}, {"id": "fixture_b"}]


# ---------------------------------------------------------------------------
# Fix round 4 (F4.1): fixture geometry projection canonicalizes MoveIt's root
# ``world`` alias back to the declared frame before digest comparison.
# ---------------------------------------------------------------------------

def test_fixture_geometry_projection_canonicalizes_moveit_world_alias_to_declared_frame():
    """F4.1: MoveIt readback moves every object into its root ``world`` frame
    and shifts the transform into ``CollisionObject.pose``; the executor must
    canonicalize that exact alias to the declared frame before projecting, so
    the F3.3 geometry digest still equals the declared fixture geometry."""
    from validation.integrated_gate_executor import (
        IntegratedGateExecutor,
        expected_fixture_geometry_digest,
    )

    contract = scenario_report_contract("qualification-moveit-plan-joint")
    declaration = contract["planning_scene_declaration"]

    executor = object.__new__(IntegratedGateExecutor)
    executor.scenario = {"planning_scene_declaration": declaration}

    def _pose(x, y, z, qx, qy, qz, qw):
        return types.SimpleNamespace(
            position=types.SimpleNamespace(x=x, y=y, z=z),
            orientation=types.SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
        )

    # Build the message exactly as MoveIt readback canonicalizes it: every
    # collision object carries header.frame_id == 'world', the declared object
    # xyz/quaternion lives in CollisionObject.pose, one BOX SolidPrimitive with
    # the declared dimensions, and an identity local primitive pose.
    collision_objects = []
    for record in declaration["objects"]:
        collision_objects.append(
            types.SimpleNamespace(
                id=record["id"],
                header=types.SimpleNamespace(frame_id="world"),
                pose=_pose(*record["pose"]["xyz"], *record["pose"]["quaternion_xyzw"]),
                primitives=[
                    types.SimpleNamespace(
                        type=1, dimensions=list(record["primitive"]["dimensions"])
                    )
                ],
                primitive_poses=[_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)],
                meshes=[],
                mesh_poses=[],
            )
        )
    message = types.SimpleNamespace(
        world=types.SimpleNamespace(collision_objects=collision_objects)
    )

    digest, descriptors = executor._fixture_geometry_projection(message)

    # Every projected descriptor is bound to the declared frame ('base_link'),
    # never MoveIt's root 'world' alias.
    assert descriptors and [descriptor["frame_id"] for descriptor in descriptors] == [
        declaration["frame_id"] for _ in descriptors
    ]
    assert all(
        descriptor["frame_id"] == declaration["frame_id"] for descriptor in descriptors
    )
    assert all(descriptor["frame_id"] != "world" for descriptor in descriptors)
    # The canonicalized projection matches the declared fixture geometry digest.
    assert digest == expected_fixture_geometry_digest(declaration)


def test_s_post_clear_stability_tolerates_sim_floor_oscillation():
    """RED (S): the safety post-clear ``quiescent`` predicate must tolerate the
    sim's measured joint-state floor oscillation.

    Live Stage-D evidence (task66-ompl-stage-d-20260808T145207, safety
    attempt) shows Isaac's ``/isaac_joint_states`` topic reports a phantom
    ~1 Hz velocity bias (joint2 ~= -0.027 rad/s, joint4 ~= +0.016 rad/s)
    whenever the arm is at rest without an active controller command, while
    the arm POSITION stays frozen well inside ``safety_position_creep_rad``.
    The configured ``safety_stop_velocity_rad_s`` must sit above that floor so
    the executor reaches the ``quiescent`` journal anchor; the verifier's
    safety acceptance is position-based (target_frozen / position creep), so
    the phantom never represents a real resumed motion."""
    import time as _time
    import uuid as _uuid

    from validation.integrated_gate_executor import IntegratedGateExecutor

    config_path = ROOT / "simulation/qualification/integrated-ompl.json"
    with config_path.open() as fh:
        config = json.load(fh)
    thresholds = config["thresholds"]
    velocity_limit = float(thresholds["safety_stop_velocity_rad_s"])
    creep_limit = float(thresholds["safety_position_creep_rad"])

    # Measured post-clear joint-state frame (task66 safety attempt, t=5.33):
    # phantom velocity bias on a frozen arm; position creep well within limit.
    phantom_velocities = [0.001, -0.027, 0.001, 0.016, 0.007, -0.004, 0.003]
    clear_positions = [0.0008, 0.0059, 0.0002, -0.0045, -0.0014, 0.0011, 0.0]
    frame_positions = [0.0009, 0.0060, 0.0002, -0.0044, -0.0014, 0.0011, 0.0001]

    executor = object.__new__(IntegratedGateExecutor)
    executor._fjt_status_cache = []
    executor._joint_velocity_frames = [
        {
            "seq": 500 + index,
            "received_mono": float(_time.monotonic()),
            "velocities": list(phantom_velocities),
            "positions": list(frame_positions),
        }
        for index in range(6)
    ]
    executor._spinner = types.SimpleNamespace(spin_once=lambda *a, **k: None)

    baseline = {
        "fjt_seq": 0,
        "joint_seq": 499,
        "clear_positions": list(clear_positions),
    }
    result = executor._wait_for_post_clear_stability(
        0.2,
        baseline=baseline,
        known_goal_id=_uuid.uuid4().hex,
        velocity_limit=velocity_limit,
        creep_limit=creep_limit,
    )
    assert result["stable"] is True, (
        "safety post-clear stability rejected the sim floor oscillation "
        f"(velocities {phantom_velocities}, positions {frame_positions}) with "
        f"configured safety_stop_velocity_rad_s={velocity_limit}: "
        f"{result.get('reason')}"
    )
