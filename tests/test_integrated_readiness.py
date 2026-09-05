"""Task 6: ROS-free integrated readiness evaluator contract tests.

Exercises ``tinker_sim_bridge.integrated_readiness.evaluate_integrated_readiness``
with a complete ready snapshot and fail-closed mismatching snapshots, and proves
the public ``scenario-runner.json`` report validates against the real scenario
with launch-shaped expected values and matches the production-canonical schema.
The pure evaluator imports neither ROS nor Isaac Sim, so this test runs under the
simulator CPython 3.12 venv.
"""
from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Mapping

import pytest

ROOT = Path(__file__).resolve().parents[1]
import sys

sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.integrated_readiness import (  # noqa: E402
    FINAL_SIMULATION_STATE,
    INTEGRATED_ACTIONS,
    INTEGRATED_JOINT_STATE_NAMES,
    INTEGRATED_PUBLISHERS,
    INTEGRATED_SERVICES,
    INTEGRATED_TOUCH_LINKS,
    REPORT_REVISION,
    build_canonical_report,
    build_integrated_mapping,
    canonical_json,
    evaluate_integrated_readiness,
    json_safe_value,
    parse_canonical_report,
    public_integrated_mapping,
    serialize_report,
    sha256_bytes,
    sha256_json,
    validate_report,
)

SCENARIO_ID = "qualification-moveit-plan-joint"
SEED = 7
PLANNING_SCENE_REVISION = "2026-08-01-moveit-qualification-joint"
PLANNING_SCENE_REVISION_DIGEST = "77b26bb8bc35649f5b25e95c2d4a56c30cf6933d918a8419d956b1ca987d0510"
PLANNING_SCENE_OWNED_IDS = ["sim_fixture/pedestal", "sim_fixture/public_target"]
PLANNING_SCENE_TARGET_SOURCE_ID = "sim_fixture/public_target"
PLANNING_SCENE_TARGET_HANDOFF = "pick_and_place/object_mesh"
MODEL_FINGERPRINT = "2" * 64
PROVIDER_MANIFEST_SHA256 = "3" * 64
PROVIDER_MANIFEST_PATH = "/srv/tinker-sim/integration/provider-manifest.json"

SCENARIO_FILE = ROOT / "simulation/scenarios/qualification-moveit-plan-joint.json"


def _goal_service_type(action_type: str) -> str:
    """Derive the canonical ``_action/send_goal`` service type."""
    marker = "/action/"
    package, action = action_type.split(marker, 1)
    return "{}/action/{}_SendGoal".format(package, action)


def _real_planning_scene() -> dict[str, object]:
    """Load the real qualification planning-scene declaration (ROS-free)."""
    raw = json.loads(SCENARIO_FILE.read_text(encoding="utf-8"))
    scene = dict(raw["planning_scene"])
    # The launch supplies the derived declared-order owned ids, not a top-level
    # key (the real scenario has none).
    return scene


def provider_manifest() -> dict[str, object]:
    """Return a structurally complete provider-manifest mapping."""
    return {
        "schema_version": 1,
        "owner": "tinker_sim_bridge",
        "provider_manifest_sha256": PROVIDER_MANIFEST_SHA256,
        "cardinality_source": [
            "provider_manifest",
            "resolved_launch_graph",
            "process_lifecycle",
            "live_ros_graph",
            "typed_controller_manager",
            "publisher_endpoint_metadata",
        ],
        "persistent_nodes": [
            {
                "key": "move_group",
                "owner": "tinker_sim_bridge",
                "package": "moveit_ros_move_group",
                "executable": "move_group",
                "node": "/move_group",
                "cardinality": 1,
                "evidence": ["resolved_launch_graph", "live_ros_graph"],
            }
        ],
        "one_shot_processes": [
            {
                "key": "controller_reconciler",
                "owner": "tinker_sim_bridge",
                "package": "tinker_sim_bridge",
                "executable": "controller_reconciler",
                "node": "/tinker_controller_reconciler",
                "cardinality": 1,
                "arguments": ["joint_state_broadcaster", "xarm7_traj_controller"],
                "evidence": ["process_lifecycle", "typed_controller_manager"],
            }
        ],
        "controller_resources": [
            {
                "resource_name": "xarm7_traj_controller",
                "owner": "tinker_sim_bridge",
                "controller_type": "joint_trajectory_controller/JointTrajectoryController",
                "expected_state": "active",
                "cardinality": 1,
                "reconciler": "controller_reconciler",
                "evidence": ["typed_controller_manager", "live_ros_graph"],
            }
        ],
        "publishers": [
            {
                "topic": "/joint_states",
                "type": "sensor_msgs/msg/JointState",
                "source": "/controller_manager",
                "logical_resource": "joint_state_broadcaster",
                "cardinality": 1,
                "evidence": ["publisher_endpoint_metadata", "live_ros_graph"],
            }
        ],
    }


def contract() -> dict[str, object]:
    """Return the expected integrated readiness contract."""
    runtime_mapping = build_integrated_mapping()
    return {
        "schema_version": 1,
        "report_revision": REPORT_REVISION,
        "scenario_id": SCENARIO_ID,
        "seed": SEED,
        "scenario_declaration_sha256": sha256_json(
            {
                "id": SCENARIO_ID,
                "seed": SEED,
                "declaration": _declaration(),
            }
        ),
        "planning_scene_revision": PLANNING_SCENE_REVISION,
        "planning_scene_revision_digest": PLANNING_SCENE_REVISION_DIGEST,
        "planning_scene_owned_ids": list(PLANNING_SCENE_OWNED_IDS),
        "planning_scene_target_source_id": PLANNING_SCENE_TARGET_SOURCE_ID,
        "planning_scene_target_handoff": PLANNING_SCENE_TARGET_HANDOFF,
        "integrated_mapping": runtime_mapping,
        "public_integrated_mapping": public_integrated_mapping(),
        "integrated_sha256": sha256_json(public_integrated_mapping()),
        "runtime_contract_sha256": sha256_json(runtime_mapping),
        "model_fingerprint": MODEL_FINGERPRINT,
        "provider_manifest_path": PROVIDER_MANIFEST_PATH,
        "provider_manifest_sha256": PROVIDER_MANIFEST_SHA256,
        "actions": {endpoint: dict(spec) for endpoint, spec in INTEGRATED_ACTIONS.items()},
        "services": {endpoint: dict(spec) for endpoint, spec in INTEGRATED_SERVICES.items()},
        "publishers": INTEGRATED_PUBLISHERS,
        "controller_resources": {
            "joint_state_broadcaster": "active",
            "xarm7_traj_controller": "active",
        },
        "joint_names": list(INTEGRATED_JOINT_STATE_NAMES),
        "tf_parent": "base_link",
        "tf_child": "link_tcp",
        "touch_links": list(INTEGRATED_TOUCH_LINKS),
    }


def _declaration() -> dict[str, object]:
    raw = json.loads(SCENARIO_FILE.read_text(encoding="utf-8"))
    return {k: v for k, v in raw.items() if k not in {"id", "seed"}}


def _planning_scene() -> dict[str, object]:
    return _real_planning_scene()


def _shared_report_evidence() -> dict[str, object]:
    report = build_canonical_report(
        scenario_id=SCENARIO_ID,
        seed=SEED,
        declaration=_declaration(),
        planning_scene=_planning_scene(),
        integrated=public_integrated_mapping(),
        operations=[
            {
                "operation": "load_world",
                "accepted": True,
            },
            {
                "operation": "set_simulation_state",
                "accepted": True,
                "state": 1,
                "boundary": "PHYSICS_READY",
            },
        ],
        model_fingerprint=MODEL_FINGERPRINT,
        provider_manifest_sha256=PROVIDER_MANIFEST_SHA256,
        final_simulation_state=FINAL_SIMULATION_STATE,
    )
    data = serialize_report(report)
    return {
        "ready": True,
        "reasons": [],
        "scenario_report_sha256": sha256_bytes(data),
        "scenario_report_sha256_bytes": data,
        "scenario_report_sha256_matches": True,
        "final_simulation_state": FINAL_SIMULATION_STATE,
        "identities": report["identities"],
        "operations": report["operations"],
    }


def _actions() -> dict[str, object]:
    return {
        endpoint: {
            "count": 1,
            "type": spec["type"],
            "observed_types": [_goal_service_type(spec["type"])],
            "source": spec["source"],
            "ready": True,
            "reasons": [],
        }
        for endpoint, spec in INTEGRATED_ACTIONS.items()
    }


def _services() -> dict[str, object]:
    return {
        endpoint: {
            "count": 1,
            "type": spec["type"],
            "observed_types": [spec["type"]],
            "source": spec["source"],
            "ready": True,
            "reasons": [],
        }
        for endpoint, spec in INTEGRATED_SERVICES.items()
    }


def _publisher_metadata() -> dict[str, object]:
    return {
        topic: {
            "count": spec.get("cardinality", 1),
            "source": spec.get("source", ""),
            "sources": [spec.get("source", "")],
            "types": [spec.get("type", "")],
            "qos": {
                "reliability": spec.get("reliability", "RELIABLE"),
                "durability": spec.get("durability", "VOLATILE"),
                "depth": spec.get("depth", 10),
            },
        }
        for topic, spec in INTEGRATED_PUBLISHERS.items()
    }


def ready_snapshot() -> dict[str, object]:
    """Return a complete fail-closed-ready observation snapshot."""
    return {
        "model_preflight": {
            "ready": True,
            "reasons": [],
            "structural_fingerprint": MODEL_FINGERPRINT,
        },
        "shared_report": _shared_report_evidence(),
        "joint_states": {
            "ready": True,
            "reasons": [],
            "names": list(INTEGRATED_JOINT_STATE_NAMES),
            "positions": [0.0] * 8,
            "velocities": [0.0] * 8,
            "header_stamp_ns": 1_000_000_000,
            "publisher_source": "/controller_manager",
            "publisher_count": 1,
        },
        "tf": {
            "ready": True,
            "reasons": [],
            "parent": "base_link",
            "child": "link_tcp",
            "exists": True,
            "stamp_ns": 1_000_000_000,
        },
        "controller_resources": {
            "joint_state_broadcaster": {
                "state": "active",
                "ready": True,
                "reasons": [],
            },
            "xarm7_traj_controller": {
                "state": "active",
                "ready": True,
                "reasons": [],
                "action_server_count": 1,
            },
        },
        "operator_input": {
            "ready": True,
            "reasons": [],
            "value": False,
            "source": "/tinker_integrated_gate_executor",
            "count": 1,
        },
        "safety_stop": {
            "ready": True,
            "reasons": [],
            "value": False,
            "source": "/tinker_sim_safety_supervisor",
            "count": 1,
            "received_samples": 2,
        },
        "actions": _actions(),
        "services": _services(),
        "arm_joint_service": {
            "ready": True,
            "reasons": [],
            "count": 1,
            "source": "/pick_and_place",
            "type": "tinker_arm_msgs/srv/ArmJointService",
        },
        "fixture_status": {
            "ready": True,
            "reasons": [],
            "status": {
                "state": "FIXTURE_READY",
                "scenario": SCENARIO_ID,
                "revision": PLANNING_SCENE_REVISION,
                "revision_digest": PLANNING_SCENE_REVISION_DIGEST,
                "owned_ids": PLANNING_SCENE_OWNED_IDS,
                "target_source_id": PLANNING_SCENE_TARGET_SOURCE_ID,
                "target_handoff": PLANNING_SCENE_TARGET_HANDOFF,
                "sequence": 12,
            },
            "age_s": 0.05,
        },
        "publishers": _publisher_metadata(),
        "mapping_agreement": {
            "ready": True,
            "reasons": [],
            "observed": {
                "scenario_declaration_sha256": contract()["scenario_declaration_sha256"],
                "integrated_sha256": sha256_json(public_integrated_mapping()),
                "model_fingerprint": MODEL_FINGERPRINT,
                "provider_manifest_sha256": PROVIDER_MANIFEST_SHA256,
                "runtime_contract_sha256": sha256_json(build_integrated_mapping()),
            },
        },
        "provider_manifest": {
            "ready": True,
            "reasons": [],
            "path": PROVIDER_MANIFEST_PATH,
            "sha256": PROVIDER_MANIFEST_SHA256,
            "manifest": provider_manifest(),
            "observed_nodes": ["/move_group", "/tinker_controller_reconciler"],
            "observed_publishers": ["/joint_states", "/sim/safety/operator"],
            "observed_controllers": {
                "joint_state_broadcaster": "active",
                "xarm7_traj_controller": "active",
            },
        },
        "semantic_model": {
            "ready": True,
            "reasons": [],
            "kinematics_match": True,
            "touch_links": list(INTEGRATED_TOUCH_LINKS),
        },
        "collision_state": {
            "ready": True,
            "reasons": [],
            "value": False,
            "source": "/tinker_isaac_gateway",
        },
    }


def mismatching_snapshot(**overrides) -> dict[str, object]:
    """Return a ready snapshot with top-level fields overridden by *overrides*."""
    snapshot = ready_snapshot()
    snapshot.update(copy.deepcopy(overrides))
    return snapshot


def _evaluate(overrides: Mapping[str, object]):
    return evaluate_integrated_readiness(
        mismatching_snapshot(**dict(overrides)), contract()
    )


def test_ready_snapshot_passes() -> None:
    report = evaluate_integrated_readiness(ready_snapshot(), contract())
    assert report.ready is True
    assert report.reasons == ()


def test_readiness_status_with_report_bytes_is_json_serializable() -> None:
    report = evaluate_integrated_readiness(ready_snapshot(), contract())
    raw_bytes = report.evidence["shared_report"]["scenario_report_sha256_bytes"]
    status = {
        "schema_version": 1,
        "state": "pass" if report.ready else "fail",
        "ready": report.ready,
        "reasons": list(report.reasons),
        "evidence": report.evidence,
    }

    encoded = json.dumps(
        json_safe_value(status), sort_keys=True, separators=(",", ":")
    )
    decoded = json.loads(encoded)

    assert isinstance(raw_bytes, bytes)
    assert report.evidence["shared_report"]["scenario_report_sha256_bytes"] is raw_bytes
    assert decoded["evidence"]["shared_report"]["scenario_report_sha256_bytes"] == raw_bytes.hex()


def test_mismatching_snapshot_preserves_ready() -> None:
    assert _evaluate({"model_preflight": {"ready": False, "reasons": ["bad model"]}}).ready is False


def test_joint_state_content_fails() -> None:
    report = _evaluate(
        {
            "joint_states": {
                "ready": False,
                "reasons": ["joint names do not match expected"],
                "names": [],
            }
        }
    )
    assert report.ready is False
    assert any("joint_states" in reason for reason in report.reasons)


def test_shared_report_bad_digest_fails() -> None:
    evidence = _shared_report_evidence()
    evidence = dict(evidence)
    evidence["scenario_report_sha256"] = "9" * 64
    report = _evaluate({"shared_report": evidence})
    assert report.ready is False
    assert any("shared_report" in reason for reason in report.reasons)


def test_tf_missing_fails() -> None:
    report = _evaluate({"tf": {"ready": False, "reasons": ["composed lookup failed"], "exists": False}})
    assert report.ready is False
    assert any("tf" in reason for reason in report.reasons)


def test_controller_inactive_fails() -> None:
    resources = {
        "joint_state_broadcaster": {"state": "active", "ready": True, "reasons": []},
        "xarm7_traj_controller": {"state": "inactive", "ready": False, "reasons": ["state inactive"], "action_server_count": 1},
    }
    report = _evaluate({"controller_resources": resources})
    assert report.ready is False
    assert any("controller_resources" in reason for reason in report.reasons)


def test_traj_action_server_count_fails() -> None:
    resources = {
        "joint_state_broadcaster": {"state": "active", "ready": True, "reasons": []},
        "xarm7_traj_controller": {"state": "active", "ready": True, "reasons": [], "action_server_count": 0},
    }
    report = _evaluate({"controller_resources": resources})
    assert report.ready is False
    assert any("controller_resources" in reason for reason in report.reasons)


def test_operator_not_clear_fails() -> None:
    report = _evaluate(
        {"operator_input": {"ready": False, "reasons": ["sample value True != expected False"], "value": True}}
    )
    assert report.ready is False
    assert any("operator_input" in reason for reason in report.reasons)


def test_safety_stop_engaged_fails() -> None:
    report = _evaluate(
        {"safety_stop": {"ready": False, "reasons": ["effective safety stop is active"], "value": True}}
    )
    assert report.ready is False
    assert any("safety_stop" in reason for reason in report.reasons)


def test_action_server_count_fails() -> None:
    actions = _actions()
    actions["/move_action"] = {
        "count": 0,
        "type": INTEGRATED_ACTIONS["/move_action"]["type"],
        "observed_types": [_goal_service_type(INTEGRATED_ACTIONS["/move_action"]["type"])],
        "source": INTEGRATED_ACTIONS["/move_action"]["source"],
        "ready": False,
        "reasons": ["server count is 0"],
    }
    report = _evaluate({"actions": actions})
    assert report.ready is False
    assert any("action /move_action" in reason for reason in report.reasons)


def test_action_observed_type_fails() -> None:
    """A wrong observed goal-service type must fail, not self-confirm."""
    actions = _actions()
    actions["/move_action"]["observed_types"] = ["wrong_msgs/action/Wrong_SendGoal"]
    report = _evaluate({"actions": actions})
    assert report.ready is False
    assert any("action /move_action" in reason for reason in report.reasons)


def test_action_missing_observed_type_fails() -> None:
    actions = _actions()
    actions["/move_action"]["observed_types"] = []
    report = _evaluate({"actions": actions})
    assert report.ready is False
    assert any("action /move_action" in reason for reason in report.reasons)


def test_service_source_fails() -> None:
    services = _services()
    services["/get_planning_scene"] = {
        "count": 1,
        "type": INTEGRATED_SERVICES["/get_planning_scene"]["type"],
        "observed_types": [INTEGRATED_SERVICES["/get_planning_scene"]["type"]],
        "source": "/wrong_node",
        "ready": False,
        "reasons": ["source mismatch"],
    }
    report = _evaluate({"services": services})
    assert report.ready is False
    assert any("service /get_planning_scene" in reason for reason in report.reasons)


def test_service_observed_type_fails() -> None:
    services = _services()
    services["/get_planning_scene"]["observed_types"] = ["wrong_msgs/srv/Wrong"]
    report = _evaluate({"services": services})
    assert report.ready is False
    assert any("service /get_planning_scene" in reason for reason in report.reasons)


def test_arm_joint_service_fails() -> None:
    report = _evaluate({"arm_joint_service": {"ready": False, "reasons": ["no /pick_and_place server"], "count": 0}})
    assert report.ready is False
    assert any("arm_joint_service" in reason for reason in report.reasons)


def test_fixture_status_fails() -> None:
    report = _evaluate({"fixture_status": {"ready": False, "reasons": ["fixture state is not FIXTURE_READY"], "status": None}})
    assert report.ready is False
    assert any("fixture_status" in reason for reason in report.reasons)


def test_publisher_source_mismatch_fails() -> None:
    publishers = _publisher_metadata()
    publishers["/joint_states"]["source"] = "/wrong_publisher"
    report = _evaluate({"publishers": publishers})
    assert report.ready is False
    assert any("publishers" in reason and "/joint_states" in reason for reason in report.reasons)


def test_publisher_count_mismatch_fails() -> None:
    publishers = _publisher_metadata()
    publishers["/isaac_joint_commands"]["count"] = 2
    report = _evaluate({"publishers": publishers})
    assert report.ready is False
    assert any("publishers" in reason and "/isaac_joint_commands" in reason for reason in report.reasons)


def test_publisher_qos_mismatch_fails() -> None:
    publishers = _publisher_metadata()
    publishers["/sim/hardware/safety_stop"]["qos"]["reliability"] = "BEST_EFFORT"
    report = _evaluate({"publishers": publishers})
    assert report.ready is False
    assert any("publishers" in reason and "/sim/hardware/safety_stop" in reason for reason in report.reasons)


def test_publisher_durability_mismatch_fails() -> None:
    publishers = _publisher_metadata()
    publishers["/sim/safety/operator"]["qos"]["durability"] = "VOLATILE"
    report = _evaluate({"publishers": publishers})
    assert report.ready is False
    assert any("publishers" in reason and "/sim/safety/operator" in reason for reason in report.reasons)


def test_command_publisher_type_fails() -> None:
    publishers = _publisher_metadata()
    publishers["/sim/controller/gripper_commands"]["types"] = ["std_msgs/msg/Float64MultiArray"]
    report = _evaluate({"publishers": publishers})
    assert report.ready is False
    assert any("publishers" in reason and "/sim/controller/gripper_commands" in reason for reason in report.reasons)


def test_provider_manifest_live_agreement_fails() -> None:
    provider = dict(ready_snapshot()["provider_manifest"])
    provider["observed_nodes"] = []  # missing /move_group
    report = _evaluate({"provider_manifest": provider})
    assert report.ready is False
    assert any("provider_manifest" in reason for reason in report.reasons)


def test_provider_manifest_controller_agreement_fails() -> None:
    provider = dict(ready_snapshot()["provider_manifest"])
    provider["observed_controllers"] = {"xarm7_traj_controller": "inactive"}
    report = _evaluate({"provider_manifest": provider})
    assert report.ready is False
    assert any("provider_manifest" in reason for reason in report.reasons)


def test_mapping_agreement_fails() -> None:
    report = _evaluate({"mapping_agreement": {"ready": False, "reasons": ["integrated_sha256 does not match"]}})
    assert report.ready is False
    assert any("mapping_agreement" in reason for reason in report.reasons)


def test_runtime_contract_digest_mismatch_fails() -> None:
    mapping = dict(ready_snapshot()["mapping_agreement"])
    mapping = dict(mapping)
    mapping["ready"] = False
    mapping["reasons"] = ["runtime contract mapping sha256 does not match expected"]
    report = _evaluate({"mapping_agreement": mapping})
    assert report.ready is False
    assert any("mapping_agreement" in reason for reason in report.reasons)


def test_provider_manifest_fails() -> None:
    report = _evaluate({"provider_manifest": {"ready": False, "reasons": ["manifest bytes digest mismatch"]}})
    assert report.ready is False
    assert any("provider_manifest" in reason for reason in report.reasons)


def test_semantic_model_fails() -> None:
    report = _evaluate({"semantic_model": {"ready": False, "reasons": ["kinematics mismatch"], "kinematics_match": False}})
    assert report.ready is False
    assert any("semantic_model" in reason for reason in report.reasons)


def test_collision_state_fails() -> None:
    report = _evaluate({"collision_state": {"ready": False, "reasons": ["collision is active"], "value": True}})
    assert report.ready is False
    assert any("collision_state" in reason for reason in report.reasons)


def test_build_integrated_mapping_is_stable() -> None:
    first = build_integrated_mapping()
    second = build_integrated_mapping()
    assert sha256_json(first) == sha256_json(second)
    assert set(first) == {
        "report_revision",
        "actions",
        "services",
        "publishers",
        "joint_names",
        "touch_links",
        "tf",
        "controller_resources",
        "final_simulation_state",
    }


# ---------------------------------------------------------------------------
# Real-scenario report validation and production-canonical schema
# ---------------------------------------------------------------------------


def _launch_shaped_expected() -> dict[str, object]:
    """Build the launch-shaped expected contract (owned ids as a JSON string)."""
    return {
        "scenario_id": SCENARIO_ID,
        "seed": SEED,
        "scenario_declaration_sha256": sha256_json(
            {"id": SCENARIO_ID, "seed": SEED, "declaration": _declaration()}
        ),
        "planning_scene_revision": PLANNING_SCENE_REVISION,
        "planning_scene_revision_digest": PLANNING_SCENE_REVISION_DIGEST,
        "planning_scene_owned_ids": json.dumps(PLANNING_SCENE_OWNED_IDS),
        "planning_scene_target_source_id": PLANNING_SCENE_TARGET_SOURCE_ID,
        "planning_scene_target_handoff": PLANNING_SCENE_TARGET_HANDOFF,
        "integrated_mapping": public_integrated_mapping(),
        "integrated_sha256": sha256_json(public_integrated_mapping()),
        "model_fingerprint": MODEL_FINGERPRINT,
        "provider_manifest_sha256": PROVIDER_MANIFEST_SHA256,
    }


def test_real_scenario_report_validates() -> None:
    """A canonical report built from the real scenario passes validate_report
    with launch-shaped expected values (owned ids as a JSON-array wire string)."""
    report = build_canonical_report(
        scenario_id=SCENARIO_ID,
        seed=SEED,
        declaration=_declaration(),
        planning_scene=_planning_scene(),
        integrated=public_integrated_mapping(),
        operations=[
            {"operation": "set_simulation_state", "accepted": True, "state": 1, "boundary": "PHYSICS_READY"}
        ],
        model_fingerprint=MODEL_FINGERPRINT,
        provider_manifest_sha256=PROVIDER_MANIFEST_SHA256,
        final_simulation_state=FINAL_SIMULATION_STATE,
    )
    # The report planning-scene must derive the real owned ids in order.
    assert report["planning_scene"]["owned_ids"] == PLANNING_SCENE_OWNED_IDS
    data = serialize_report(report)
    parsed = parse_canonical_report(data)
    validation = validate_report(parsed, _launch_shaped_expected())
    assert validation["ready"] is True, validation["reasons"]


def test_real_scenario_report_mutations_rejected() -> None:
    """Mutating each field/digest in a real-scenario report is rejected."""
    def build():
        return build_canonical_report(
            scenario_id=SCENARIO_ID,
            seed=SEED,
            declaration=_declaration(),
            planning_scene=_planning_scene(),
            integrated=public_integrated_mapping(),
            operations=[
                {"operation": "set_simulation_state", "accepted": True, "state": 1, "boundary": "PHYSICS_READY"}
            ],
            model_fingerprint=MODEL_FINGERPRINT,
            provider_manifest_sha256=PROVIDER_MANIFEST_SHA256,
            final_simulation_state=FINAL_SIMULATION_STATE,
        )

    expected = _launch_shaped_expected()
    base = build()
    assert validate_report(base, expected)["ready"] is True

    mutations = {
        "scenario id": (("scenario", "id"), "wrong-scenario"),
        "seed": (("scenario", "seed"), 99),
        "planning_scene revision": (("planning_scene", "revision"), "wrong-revision"),
        "planning_scene owned ids": (("planning_scene", "owned_ids"), ["sim_fixture/foreign"]),
        "planning_scene target": (("planning_scene", "target_source_id"), "sim_fixture/foreign"),
        "integrated execution_profile": (("integrated", "execution_profile"), "hardware"),
        "scenario_declaration digest": (("identities", "scenario_declaration_sha256"), "9" * 64),
        "planning_scene digest": (("identities", "planning_scene_sha256"), "9" * 64),
        "integrated digest": (("identities", "integrated_sha256"), "9" * 64),
        "model fingerprint": (("identities", "model_fingerprint"), "9" * 64),
        "provider digest": (("identities", "provider_manifest_sha256"), "9" * 64),
    }
    for label, (path, value) in mutations.items():
        mutated = copy.deepcopy(base)
        node = mutated
        for key in path[:-1]:
            node = node[key]
        node[path[-1]] = value
        validation = validate_report(mutated, expected)
        assert validation["ready"] is False, "mutation {!r} was not rejected".format(label)


def test_public_report_schema_matches_production() -> None:
    """The public report has the eight exact top-level keys, the one-key
    integrated mapping, and self-consistent identities digest (production
    parse_scenario_status_json compatibility)."""
    report = build_canonical_report(
        scenario_id=SCENARIO_ID,
        seed=SEED,
        declaration=_declaration(),
        planning_scene=_planning_scene(),
        integrated=public_integrated_mapping(),
        operations=[
            {"operation": "set_simulation_state", "accepted": True, "state": 1, "boundary": "PHYSICS_READY"}
        ],
        model_fingerprint=MODEL_FINGERPRINT,
        provider_manifest_sha256=PROVIDER_MANIFEST_SHA256,
    )
    assert set(report) == {
        "schema_version",
        "report_revision",
        "scenario",
        "planning_scene",
        "integrated",
        "identities",
        "operations",
        "final_simulation_state",
    }
    assert report["integrated"] == {"execution_profile": "sim_ompl"}
    assert report["integrated"]["execution_profile"] == "sim_ompl"
    assert report["identities"]["integrated_sha256"] == sha256_json(
        {"execution_profile": "sim_ompl"}
    )
    assert report["identities"]["planning_scene_sha256"] == sha256_json(
        report["planning_scene"]
    )
    assert report["identities"]["scenario_declaration_sha256"] == sha256_json(
        report["scenario"]
    )
    # Final operation carries the exact identities and PHYSICS_READY boundary.
    last = report["operations"][-1]
    assert last["boundary"] == "PHYSICS_READY"
    assert last["state"] == 1
    for key in (
        "scenario_id",
        "seed",
        "scenario_declaration_sha256",
        "planning_scene_sha256",
        "integrated_sha256",
        "model_fingerprint",
        "provider_manifest_sha256",
    ):
        assert last[key] == report["identities"][key]


def test_public_vs_runtime_mapping_distinct() -> None:
    """The public report integrated mapping is distinct from the full runtime
    readiness contract; the runtime digest is carried separately."""
    public = public_integrated_mapping()
    runtime = build_integrated_mapping()
    assert set(public) == {"execution_profile"}
    assert "execution_profile" not in runtime
    assert sha256_json(public) != sha256_json(runtime)


def test_operator_subscription_qos_matches_transient_local_publisher() -> None:
    """RED (J): the integrated_readiness operator subscription must use the
    same TRANSIENT_LOCAL durability the executor publishes on
    ``/sim/safety/operator``.  A VOLATILE subscriber misses the latched False
    baseline on a cold start, so the readiness node reports ``operator_input:
    publisher count is 0`` / ``no sample received`` (execute-joint/cancel/
    safety).  The subscription spec must agree with the canonical publisher
    contract in ``INTEGRATED_PUBLISHERS``."""
    from tinker_sim_bridge.integrated_readiness import OPERATOR_SUB_QOS_SPEC

    contract = INTEGRATED_PUBLISHERS["/sim/safety/operator"]
    assert OPERATOR_SUB_QOS_SPEC["reliability"] == contract["reliability"].lower()
    assert OPERATOR_SUB_QOS_SPEC["durability"] == contract["durability"].lower()
    assert OPERATOR_SUB_QOS_SPEC["depth"] >= 1
    assert OPERATOR_SUB_QOS_SPEC["durability"] == "transient_local"


def test_r_joint_states_contract_is_volatile_matching_joint_state_broadcaster() -> None:
    """RED (R): the canonical ``/joint_states`` publisher contract must match the
    ros2_control ``joint_state_broadcaster`` which publishes with
    ``rclcpp::SystemDefaultsQoS()`` (RELIABLE / VOLATILE / KeepLast(10)).  The
    contract table previously claimed TRANSIENT_LOCAL; once the joint_state
    broadcaster is discovered the readiness publisher-metadata check would fail
    ``/joint_states durability 'VOLATILE' != expected 'TRANSIENT_LOCAL'`` and
    gate every Stage-D scenario (cartesian-retreat external C++ readiness)."""
    contract = INTEGRATED_PUBLISHERS["/joint_states"]
    assert contract["source"] == "/joint_state_broadcaster"
    assert contract["reliability"] == "RELIABLE"
    assert contract["durability"] == "VOLATILE", (
        "joint_state_broadcaster publishes /joint_states with "
        "rclcpp::SystemDefaultsQoS() = RELIABLE/VOLATILE/KeepLast(10); the "
        "readiness publisher contract must agree or the gate fails closed"
    )
    assert contract["depth"] >= 10
