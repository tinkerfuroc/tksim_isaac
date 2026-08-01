"""Task 6: ROS-free integrated readiness evaluator contract tests.

Exercises ``tinker_sim_bridge.integrated_readiness.evaluate_integrated_readiness``
with a complete ready snapshot and fail-closed mismatching snapshots.  The pure
evaluator imports neither ROS nor Isaac Sim, so this test runs under the
simulator CPython 3.12 venv.
"""
from __future__ import annotations

import copy
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
    evaluate_integrated_readiness,
    sha256_bytes,
    sha256_json,
)

SCENARIO_ID = "qualification-moveit-plan-joint"
SEED = 7
SCENARIO_DECLARATION_SHA256 = "1" * 64
PLANNING_SCENE_REVISION = "2026-08-01-moveit-qualification-joint"
PLANNING_SCENE_REVISION_DIGEST = "d684a3d2270ab6d935b8e5c94dd5d4512760e06a1d09a41582177680536ccd8d"
PLANNING_SCENE_OWNED_IDS = ["sim_fixture/pedestal", "sim_fixture/public_target"]
PLANNING_SCENE_TARGET_SOURCE_ID = "sim_fixture/public_target"
PLANNING_SCENE_TARGET_HANDOFF = "pick_and_place/object_mesh"
MODEL_FINGERPRINT = "2" * 64
PROVIDER_MANIFEST_SHA256 = "3" * 64
PROVIDER_MANIFEST_PATH = "/srv/tinker-sim/integration/provider-manifest.json"


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
    return {
        "schema_version": 1,
        "report_revision": REPORT_REVISION,
        "scenario_id": SCENARIO_ID,
        "seed": SEED,
        "scenario_declaration_sha256": SCENARIO_DECLARATION_SHA256,
        "planning_scene_revision": PLANNING_SCENE_REVISION,
        "planning_scene_revision_digest": PLANNING_SCENE_REVISION_DIGEST,
        "planning_scene_owned_ids": list(PLANNING_SCENE_OWNED_IDS),
        "planning_scene_target_source_id": PLANNING_SCENE_TARGET_SOURCE_ID,
        "planning_scene_target_handoff": PLANNING_SCENE_TARGET_HANDOFF,
        "integrated_mapping": build_integrated_mapping(),
        "integrated_sha256": sha256_json(build_integrated_mapping()),
        "model_fingerprint": MODEL_FINGERPRINT,
        "provider_manifest_path": PROVIDER_MANIFEST_PATH,
        "provider_manifest_sha256": PROVIDER_MANIFEST_SHA256,
        "actions": {endpoint: dict(spec) for endpoint, spec in INTEGRATED_ACTIONS.items()},
        "services": {endpoint: dict(spec) for endpoint, spec in INTEGRATED_SERVICES.items()},
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
    return {"schema_version": 2, "world": {"mode": "current"}}


def _planning_scene() -> dict[str, object]:
    return {
        "revision": PLANNING_SCENE_REVISION,
        "frame_id": "base_link",
        "target_source_id": PLANNING_SCENE_TARGET_SOURCE_ID,
        "target_handoff": PLANNING_SCENE_TARGET_HANDOFF,
        "objects": [
            {"id": "sim_fixture/pedestal", "class": "static", "primitive": {"type": "box", "dimensions": [0.7, 0.7, 0.85]}, "pose": {"xyz": [0.55, 0.0, 0.425], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}},
            {"id": "sim_fixture/public_target", "class": "target", "primitive": {"type": "box", "dimensions": [0.08, 0.08, 0.08]}, "pose": {"xyz": [0.55, 0.0, 0.89], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]}},
        ],
    }


def _shared_report_evidence() -> dict[str, object]:
    report = build_canonical_report(
        scenario_id=SCENARIO_ID,
        seed=SEED,
        declaration=_declaration(),
        planning_scene=_planning_scene(),
        integrated=build_integrated_mapping(),
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
    data = __import__("tinker_sim_bridge.integrated_readiness", fromlist=["serialize_report"]).serialize_report(report)
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
            "source": spec["source"],
            "ready": True,
            "reasons": [],
        }
        for endpoint, spec in INTEGRATED_SERVICES.items()
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
        "mapping_agreement": {
            "ready": True,
            "reasons": [],
            "observed": {
                "scenario_declaration_sha256": SCENARIO_DECLARATION_SHA256,
                "integrated_sha256": sha256_json(build_integrated_mapping()),
                "model_fingerprint": MODEL_FINGERPRINT,
                "provider_manifest_sha256": PROVIDER_MANIFEST_SHA256,
            },
        },
        "provider_manifest": {
            "ready": True,
            "reasons": [],
            "path": PROVIDER_MANIFEST_PATH,
            "sha256": PROVIDER_MANIFEST_SHA256,
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


def test_mismatching_snapshot_preserves_ready() -> None:
    # The override must actually flip a check, or the snapshot stays ready.
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
    report = _evaluate({"tf": {"ready": False, "reasons": ["base_link -> link_tcp not observed"], "exists": False}})
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
        "source": INTEGRATED_ACTIONS["/move_action"]["source"],
        "ready": False,
        "reasons": ["server count is 0"],
    }
    report = _evaluate({"actions": actions})
    assert report.ready is False
    assert any("action /move_action" in reason for reason in report.reasons)


def test_service_source_fails() -> None:
    services = _services()
    services["/get_planning_scene"] = {
        "count": 1,
        "type": INTEGRATED_SERVICES["/get_planning_scene"]["type"],
        "source": "/wrong_node",
        "ready": False,
        "reasons": ["source mismatch"],
    }
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


def test_mapping_agreement_fails() -> None:
    report = _evaluate({"mapping_agreement": {"ready": False, "reasons": ["integrated_sha256 does not match"]}})
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
