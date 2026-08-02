"""Integrated OMPL qualification Gate-C plan-only executor (Task 4).

This module is ROS-lazy: importing it under the simulator CPython 3.12 venv
never imports ``rclpy`` or any generated ROS message type.  All generated-message
imports happen inside :func:`_load_ros` or the goal-builder call paths, which the
Humble suite exercises under sourced ROS Humble Python 3.10.

Pure helpers (importable everywhere):

- endpoint/type/cardinality/QoS contract constants;
- ``expected_physics_ready_report`` / ``validate_physics_ready_snapshot``
  reconciled with the real canonical multi-operation public report;
- ``evaluate_executor_readiness`` with the scenario-declaration-bound fixture
  descriptor digest and operator freshness limit;
- ``build_journal_graph_projection`` matching the Task 3 journal
  ``planning_scene_journal.validate_graph_evidence`` schema.

The public report's ``integrated`` mapping is the production-canonical one-key
``{"execution_profile": "sim_ompl"}``; the full per-scenario ``integrated``
mapping stays in the scenario declaration and is bound by the scenario
declaration SHA-256 (never the public ``integrated_sha256``).  The
``fixture_descriptor_sha256`` is the real bridge descriptor digest over the full
planning-scene declaration (never a status-payload recompute).

Task 4 owns the Task 3 journal projection and artifacts: the live executor
instantiates ``PlanningSceneJournal``, feeds a graph projection produced by
:func:`build_journal_graph_projection`, and writes ``planning-scene.jsonl`` /
``planning-scene.json``.  It records diagnostic scene consistency only and never
supplies physical contact/force/object-pose/verdict fields.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import re
import sys
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.fixture_planning_scene import (  # noqa: E402
    fixture_descriptor_sha256,
    fixture_owned_ids,
)
from tinker_sim_bridge.integrated_readiness import (  # noqa: E402
    build_canonical_report,
    public_integrated_mapping,
    sha256_json,
)

REPORT_REVISION = "integrated-manipulation-v1"
FINAL_SIMULATION_STATE = "STATE_PLAYING"
PHYSICS_READY_BOUNDARY = "PHYSICS_READY"
SIMULATION_STATE_PLAYING = 1
INTEGRATED_EXECUTION_PROFILE = "sim_ompl"
OPERATOR_NODE = "/tinker_integrated_gate_executor"
FIXTURE_PUBLISHER_NODE = "/fixture_planning_scene"
SAFETY_SUPERVISOR_NODE = "/tinker_sim_safety_supervisor"
CONTROLLER_MANAGER_NODE = "/controller_manager"
GRIPPER_FACADE_NODE = "/tinker_sim_gripper_facade"
PICK_AND_PLACE_NODE = "/pick_and_place"
PHYSICS_READY_GATE_NODE = "/tinker_sim_physics_ready_gate"
PLANNING_SCENE_TOPIC = "/planning_scene"
MONITORED_PLANNING_SCENE_TOPIC = "/monitored_planning_scene"
FIXTURE_TOPIC = "/sim/status/planning_scene_fixture"
JOINT_STATES_TOPIC = "/joint_states"
OPERATOR_TOPIC = "/sim/safety/operator"
SAFETY_STOP_TOPIC = "/sim/hardware/safety_stop"
ISAAC_COMMAND_TOPIC = "/isaac_joint_commands"

#: Action endpoints and their exact generated types (one server each).
REQUIRED_ACTIONS: Mapping[str, str] = {
    "/move_action": "moveit_msgs/action/MoveGroup",
    "/execute_trajectory": "moveit_msgs/action/ExecuteTrajectory",
    "/xarm7_traj_controller/follow_joint_trajectory": "control_msgs/action/FollowJointTrajectory",
    "/xarm_gripper/gripper_action": "control_msgs/action/GripperCommand",
    "/pickup_action": "tinker_arm_msgs/action/Pick",
    "/place_action": "tinker_arm_msgs/action/Place",
    "/cartesian_move_action": "tinker_arm_msgs/action/CartesianMove",
    "/joint_move_action": "tinker_arm_msgs/action/JointMove",
    "/fold_action": "tinker_arm_msgs/action/Fold",
}

#: Service endpoints and their exact generated types (one server each).
REQUIRED_SERVICES: Mapping[str, str] = {
    "/controller_manager/list_controllers": "controller_manager_msgs/srv/ListControllers",
    "/controller_manager/load_controller": "controller_manager_msgs/srv/LoadController",
    "/controller_manager/configure_controller": "controller_manager_msgs/srv/ConfigureController",
    "/controller_manager/switch_controller": "controller_manager_msgs/srv/SwitchController",
    "/get_planning_scene": "moveit_msgs/srv/GetPlanningScene",
    "/apply_planning_scene": "moveit_msgs/srv/ApplyPlanningScene",
    "/check_state_validity": "moveit_msgs/srv/GetStateValidity",
    "/compute_cartesian_path": "moveit_msgs/srv/GetCartesianPath",
    "/arm_joint_service": "tinker_arm_msgs/srv/ArmJointService",
    "/sim/ready/physics": "std_srvs/srv/Trigger",
    "/sim/ready/fixture": "std_srvs/srv/Trigger",
}

#: Required topic graph contract (type/source/cardinality/QoS).  This is a graph
#: contract, not a fixture convenience: a valid payload can never mask graph
#: metadata and vice versa.
REQUIRED_TOPICS: Mapping[str, Mapping[str, object]] = {
    JOINT_STATES_TOPIC: {
        "type": "sensor_msgs/msg/JointState", "publisher_count": 1,
        "source_node": CONTROLLER_MANAGER_NODE,
        "qos": {"reliability": "reliable", "durability": "volatile", "depth": 10},
    },
    FIXTURE_TOPIC: {
        "type": "std_msgs/msg/String", "publisher_count": 1,
        "source_node": FIXTURE_PUBLISHER_NODE,
        "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
    },
    OPERATOR_TOPIC: {
        "type": "std_msgs/msg/Bool", "publisher_count": 1,
        "source_node": OPERATOR_NODE,
        "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
        "allowlist": [False, True],
    },
    SAFETY_STOP_TOPIC: {
        "type": "std_msgs/msg/Bool", "publisher_count": 1,
        "source_node": SAFETY_SUPERVISOR_NODE,
        "qos": {"reliability": "reliable", "durability": "transient_local", "depth": 1},
    },
}

_REQUIRED_ACTIONS = REQUIRED_ACTIONS
_REQUIRED_SERVICES = REQUIRED_SERVICES

#: Observed-graph provider for every required endpoint.  The
#: ``follow_joint_trajectory`` bridge identity is the logical
#: ``controller_resource:xarm7_traj_controller``; the observed graph node is
#: ``/controller_manager`` and is the value asserted here.
_REQUIRED_ENDPOINT_SOURCES: Mapping[str, str] = {
    "/move_action": "/move_group",
    "/execute_trajectory": "/move_group",
    "/xarm7_traj_controller/follow_joint_trajectory": CONTROLLER_MANAGER_NODE,
    "/xarm_gripper/gripper_action": GRIPPER_FACADE_NODE,
    "/pickup_action": PICK_AND_PLACE_NODE,
    "/place_action": PICK_AND_PLACE_NODE,
    "/cartesian_move_action": PICK_AND_PLACE_NODE,
    "/joint_move_action": PICK_AND_PLACE_NODE,
    "/fold_action": PICK_AND_PLACE_NODE,
    "/controller_manager/list_controllers": CONTROLLER_MANAGER_NODE,
    "/controller_manager/load_controller": CONTROLLER_MANAGER_NODE,
    "/controller_manager/configure_controller": CONTROLLER_MANAGER_NODE,
    "/controller_manager/switch_controller": CONTROLLER_MANAGER_NODE,
    "/get_planning_scene": "/move_group",
    "/apply_planning_scene": "/move_group",
    "/check_state_validity": "/move_group",
    "/compute_cartesian_path": "/move_group",
    "/arm_joint_service": PICK_AND_PLACE_NODE,
    "/sim/ready/physics": PHYSICS_READY_GATE_NODE,
    "/sim/ready/fixture": FIXTURE_PUBLISHER_NODE,
}

_REQUIRED_JOINTS: tuple[str, ...] = tuple(f"joint{index}" for index in range(1, 8)) + (
    "drive_joint",
)

DIGEST = re.compile(r"^(?!0{64}$)[0-9a-f]{64}$")
REPORT_KEYS = frozenset(
    {
        "schema_version", "report_revision", "scenario", "planning_scene",
        "integrated", "identities", "operations", "final_simulation_state",
    }
)
IDENTITY_KEYS = frozenset(
    {
        "scenario_id", "seed", "scenario_declaration_sha256",
        "planning_scene_sha256", "integrated_sha256", "model_fingerprint",
        "provider_manifest_sha256",
    }
)
#: The unique final ``PHYSICS_READY`` operation carries exactly this field set
#: (the report identities merged into the accepted set-simulation-state result).
OPERATION_KEYS = frozenset(
    {
        "operation", "accepted", "state", "boundary", "scenario_id", "seed",
        "scenario_declaration_sha256", "planning_scene_sha256", "integrated_sha256",
        "model_fingerprint", "provider_manifest_sha256",
    }
)
#: Optional fields an earlier accepted standard-operation record may carry
#: (``state`` / ``boundary`` on set-simulation-state, ``logical_id`` /
#: ``prim_path`` on spawn_entity).  ``operation`` and ``accepted`` are required.
EARLIER_OPERATION_OPTIONAL_FIELDS = frozenset(
    {"state", "boundary", "logical_id", "prim_path"}
)
#: Exact canonical fixture-status field set (matches bridge canonical status).
FIXTURE_STATUS_KEYS = frozenset(
    {
        "schema_version", "state", "scenario", "owner", "revision",
        "revision_digest", "sequence", "published_at", "owned_ids",
        "target_source_id", "target_handoff", "fixture_descriptor_sha256",
    }
)
FIXTURE_OWNER = "sim_fixture"
FIXTURE_TARGET_HANDOFF = "pick_and_place/object_mesh"

#: Journal graph projection QoS uses exact uppercase enum strings (Task 3
#: schema); readiness-snapshot QoS uses the existing lowercase representation.
JOURNAL_TOPIC_QOS: Mapping[str, object] = {
    "reliability": "RELIABLE",
    "durability": "TRANSIENT_LOCAL",
    "depth": 1,
}
JOURNAL_SERVICE_QOS: Mapping[str, object] = {
    "reliability": "RELIABLE",
    "durability": "VOLATILE",
}


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def _finite_vector(values: Sequence[float], *, length: int, name: str) -> list[float]:
    converted = [float(value) for value in values]
    if len(converted) != length or not all(math.isfinite(value) for value in converted):
        raise ValueError(f"{name} must contain exactly {length} finite values")
    return converted


def _validate_quaternion(quaternion) -> None:
    values = tuple(
        float(value)
        for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("pose quaternion must be finite")
    norm = math.sqrt(sum(value ** 2 for value in values))
    if abs(norm - 1.0) > 1.0e-3:
        raise ValueError("pose quaternion must be normalized within 1e-3")


def _as_mapping(value: object) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def _fresh(value: object, limit: object) -> bool:
    try:
        age = float(value)
        return math.isfinite(age) and 0.0 <= age <= float(limit)
    except (TypeError, ValueError):
        return False


def _finite_sequence(value: object, *, length: int) -> bool:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != length:
        return False
    try:
        return all(math.isfinite(float(item)) for item in value)
    except (TypeError, ValueError):
        return False


def _finite_number(value: object) -> bool:
    return type(value) in (int, float) and math.isfinite(float(value))


def _ordered_string_ids(value: object) -> bool:
    return (
        isinstance(value, Sequence)
        and not isinstance(value, (str, bytes))
        and all(isinstance(item, str) for item in value)
        and len(value) == len(set(value))
    )


def _valid_digest(value: object) -> bool:
    return isinstance(value, str) and DIGEST.fullmatch(value) is not None


def _strict_int(value: object) -> bool:
    return type(value) is int


def _canonical_digest(value: object) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _endpoint_failures(
    observed: object, required: Mapping[str, str], *, kind: str
) -> list[str]:
    endpoints = _as_mapping(observed)
    failures: list[str] = []
    for name, expected_type in required.items():
        endpoint = _as_mapping(endpoints.get(name))
        expected_source = _REQUIRED_ENDPOINT_SOURCES.get(name)
        if not (
            endpoint.get("type") == expected_type
            and endpoint.get("ready") is True
            and endpoint.get("server_count") == 1
            and endpoint.get("source_node") == expected_source
        ):
            failures.append(
                f"{kind} {name} is not exactly-one ready {expected_type} "
                f"owned by {expected_source}"
            )
    return failures


def _topic_failures(observed: object) -> list[str]:
    topics = _as_mapping(observed)
    failures: list[str] = []
    for name, expected in REQUIRED_TOPICS.items():
        topic = _as_mapping(topics.get(name))
        if (
            topic.get("type") != expected["type"]
            or topic.get("publisher_count") != expected["publisher_count"]
            or topic.get("source_node") != expected["source_node"]
            or topic.get("qos") != expected["qos"]
        ):
            failures.append(f"topic {name} has wrong type, cardinality, source, or QoS")
        if name == OPERATOR_TOPIC and topic.get("allowlist") != [False, True]:
            failures.append(f"topic {name} has an invalid Boolean allowlist")
    return failures


def _scenario_fixture_digest(scenario: Mapping[str, object]) -> str | None:
    """Return the real fixture descriptor digest over the scenario declaration."""
    declaration = _as_mapping(scenario.get("planning_scene_declaration"))
    if not declaration:
        return None
    return fixture_descriptor_sha256(declaration)


def _scenario_fixture_ids(scenario: Mapping[str, object]) -> list[str]:
    """Return declared-order owned fixture ids from the full declaration."""
    declaration = _as_mapping(scenario.get("planning_scene_declaration"))
    if declaration:
        return list(fixture_owned_ids(declaration))
    return list(_as_mapping(scenario.get("planning_scene")).get("owned_ids", ()))


def expected_physics_ready_report(
    *,
    scenario_mapping: Mapping[str, object],
    planning_scene: Mapping[str, object],
    integrated: Mapping[str, object],
    expected_identities: Mapping[str, object],
) -> dict[str, object]:
    """Build the expected real-shape multi-operation public report.

    ``planning_scene`` must be the full planning-scene declaration (so the
    bridge can derive the four-key public mapping and its digest).  The report
    carries the one-key public ``integrated`` mapping; the full ``integrated``
    mapping passed in is asserted to carry ``execution_profile == "sim_ompl"``
    and remains bound by the scenario declaration SHA-256.
    """
    identities = dict(expected_identities)
    if set(identities) != IDENTITY_KEYS:
        raise ValueError("expected PHYSICS_READY identities must be complete")
    if identities["scenario_id"] != str(scenario_mapping.get("id")):
        raise ValueError("expected identity scenario_id does not match scenario mapping")
    if int(identities["seed"]) != int(scenario_mapping.get("seed")):
        raise ValueError("expected identity seed does not match scenario mapping")
    if dict(integrated).get("execution_profile") != INTEGRATED_EXECUTION_PROFILE:
        raise ValueError("scenario integrated execution_profile must be sim_ompl")
    public_integrated = public_integrated_mapping()
    report = build_canonical_report(
        scenario_id=identities["scenario_id"],
        seed=identities["seed"],
        declaration=dict(_as_mapping(scenario_mapping.get("declaration"))),
        planning_scene=planning_scene,
        integrated=public_integrated,
        operations=[
            {"operation": "reset_spawned", "accepted": True},
            {
                "operation": "set_simulation_state",
                "accepted": True,
                "state": SIMULATION_STATE_PLAYING,
                "boundary": PHYSICS_READY_BOUNDARY,
            },
        ],
        model_fingerprint=identities["model_fingerprint"],
        provider_manifest_sha256=identities["provider_manifest_sha256"],
    )
    report = copy.deepcopy(dict(report))
    if report["identities"] != identities:
        raise ValueError("expected PHYSICS_READY report identities do not match")
    if report["integrated"] != public_integrated:
        raise ValueError(
            "expected PHYSICS_READY integrated mapping is not the public one-key mapping"
        )
    return report


def _validate_expected_report_structure(expected_report: Mapping[str, object]) -> None:
    """Reject boolean/non-integer numerics in the expected report positions.

    Boolean values in ``schema_version``, ``identities.seed``, or any
    operation ``state`` are structurally invalid; they fail closed before the
    observed-byte comparison so a caller passing a mutated expected report gets
    the specific strict-integer failure.
    """
    if not isinstance(expected_report, Mapping):
        raise ValueError("expected report must be a mapping")
    if not _strict_int(expected_report.get("schema_version")):
        raise ValueError("PHYSICS_READY schema_version must be a strict integer")
    identities = _as_mapping(expected_report.get("identities"))
    if not _strict_int(identities.get("seed")):
        raise ValueError("PHYSICS_READY identity seed must be a strict integer")
    operations = expected_report.get("operations")
    if not isinstance(operations, Sequence) or isinstance(operations, (str, bytes)):
        raise ValueError("PHYSICS_READY report has no operations")
    for operation in operations:
        if not isinstance(operation, Mapping):
            raise ValueError("PHYSICS_READY operations must be objects")
        if "state" in operation and not _strict_int(operation.get("state")):
            raise ValueError("PHYSICS_READY operation state must be a strict integer")


def validate_physics_ready_snapshot(
    snapshot: Mapping[str, object],
    scenario: Mapping[str, object],
    *,
    expected_report: Mapping[str, object] | None = None,
) -> None:
    """Validate the exact real-shape multi-operation physics-ready report.

    Keeps exact canonical byte/schema/revision checks and the exact top-level
    ``REPORT_KEYS``; compares ``scenario``/``planning_scene``/``integrated``/
    ``identities`` against the corrected expected contract; requires a non-empty
    operation list with exactly one final ``PHYSICS_READY`` operation carrying
    the exact ``OPERATION_KEYS`` and report identities; every earlier operation
    is an accepted standard-operation record with only the known optional fields.
    """
    if expected_report is not None:
        _validate_expected_report_structure(expected_report)
    state = _as_mapping(snapshot.get("scenario"))
    report_bytes = snapshot.get("scenario_report_bytes")
    if not isinstance(report_bytes, (bytes, bytearray)):
        raise ValueError("PHYSICS_READY exact report bytes are unavailable")
    exact_bytes = bytes(report_bytes)
    try:
        report = json.loads(exact_bytes.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("PHYSICS_READY report bytes are not valid UTF-8 JSON") from error
    if not isinstance(report, dict) or set(report) != REPORT_KEYS:
        raise ValueError("PHYSICS_READY report has the wrong canonical top-level schema")
    if not _strict_int(report.get("schema_version")):
        raise ValueError("PHYSICS_READY schema_version must be a strict integer")

    expected = (
        dict(expected_report)
        if expected_report is not None
        else expected_physics_ready_report(
            scenario_mapping=_as_mapping(scenario.get("scenario_mapping")),
            planning_scene=_as_mapping(scenario.get("planning_scene_declaration"))
            or _as_mapping(scenario.get("planning_scene")),
            integrated=_as_mapping(scenario.get("integrated")),
            expected_identities=_as_mapping(scenario.get("identities")),
        )
    )
    for key in ("scenario", "planning_scene", "integrated", "identities"):
        if report.get(key) != expected.get(key):
            raise ValueError(
                f"PHYSICS_READY report {key} does not match the "
                "scenario-specific expected contract"
            )
    canonical_bytes = json.dumps(
        report, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")
    if canonical_bytes != exact_bytes:
        raise ValueError("PHYSICS_READY report bytes are valid JSON but not the canonical serialization")
    if report["report_revision"] != REPORT_REVISION:
        raise ValueError("PHYSICS_READY report revision mismatch")

    identities = report["identities"]
    if not isinstance(identities, dict) or set(identities) != IDENTITY_KEYS:
        raise ValueError("PHYSICS_READY identity keys are not exact")
    if identities["scenario_id"] != scenario.get("id") or int(identities["seed"]) != int(
        scenario.get("seed")
    ):
        raise ValueError("PHYSICS_READY scenario identity mismatch")
    if not _strict_int(identities.get("seed")):
        raise ValueError("PHYSICS_READY identity seed must be a strict integer")
    if not _strict_int(_as_mapping(report.get("scenario")).get("seed")):
        raise ValueError("PHYSICS_READY scenario seed must be a strict integer")
    expected_identities = {
        "scenario_declaration_sha256": scenario.get("scenario_declaration_sha256"),
        "planning_scene_sha256": scenario.get("planning_scene_sha256"),
        "integrated_sha256": scenario.get("integrated_sha256"),
        "model_fingerprint": scenario.get("model_fingerprint"),
        "provider_manifest_sha256": scenario.get("provider_manifest_sha256"),
    }
    if any(identities[key] != value for key, value in expected_identities.items()):
        raise ValueError("PHYSICS_READY identity digests do not match the expected scenario")
    if identities["model_fingerprint"] != _as_mapping(snapshot.get("model")).get("fingerprint"):
        raise ValueError("PHYSICS_READY model fingerprint does not match the observed model")
    if identities["provider_manifest_sha256"] != snapshot.get("provider_manifest_sha256"):
        raise ValueError("PHYSICS_READY provider manifest digest does not match the observed manifest")
    for key in IDENTITY_KEYS - {"scenario_id", "seed"}:
        if not isinstance(identities[key], str) or DIGEST.fullmatch(identities[key]) is None:
            raise ValueError(f"PHYSICS_READY {key} is not a nonzero lowercase digest")
    for mapping_key, digest_key in (
        ("planning_scene", "planning_scene_sha256"),
        ("integrated", "integrated_sha256"),
    ):
        if identities[digest_key] != sha256_json(report[mapping_key]):
            raise ValueError(f"PHYSICS_READY {digest_key} is not the canonical mapping digest")
    if report["final_simulation_state"] != FINAL_SIMULATION_STATE:
        raise ValueError("PHYSICS_READY final simulation state is not STATE_PLAYING")

    operations = report["operations"]
    if not isinstance(operations, list) or not operations:
        raise ValueError("PHYSICS_READY report has no operations")
    # The real report is genuinely multi-operation: at least one accepted
    # standard-operation record (reset/spawn and related) precedes the unique
    # final PHYSICS_READY operation.  A fabricated single-operation report is
    # rejected, not compared against.
    if len(operations) < 2:
        raise ValueError(
            "PHYSICS_READY report must contain accepted standard-operation records "
            "before the final operation"
        )
    final = operations[-1]
    if not isinstance(final, dict) or set(final) != OPERATION_KEYS:
        raise ValueError("PHYSICS_READY final operation schema is not exact")
    physics_ready = [
        operation
        for operation in operations
        if isinstance(operation, dict) and operation.get("boundary") == PHYSICS_READY_BOUNDARY
    ]
    if len(physics_ready) != 1 or final is not physics_ready[0]:
        raise ValueError("PHYSICS_READY operation is not unique and final")
    for operation in operations[:-1]:
        if not isinstance(operation, dict):
            raise ValueError("PHYSICS_READY earlier operations must be objects")
        if not isinstance(operation.get("operation"), str) or not operation.get("operation"):
            raise ValueError("PHYSICS_READY earlier operations must carry an operation string")
        if operation.get("accepted") is not True:
            raise ValueError("PHYSICS_READY earlier operations must be accepted")
        if not set(operation) <= ({"operation", "accepted"} | EARLIER_OPERATION_OPTIONAL_FIELDS):
            raise ValueError("PHYSICS_READY earlier operations carry unknown fields")
    if (
        not _strict_int(final.get("state"))
        or final["state"] != SIMULATION_STATE_PLAYING
        or final["accepted"] is not True
        or not _strict_int(final.get("seed"))
    ):
        raise ValueError("PHYSICS_READY final operation is not accepted with integer state=1")
    for key in IDENTITY_KEYS:
        if final[key] != identities[key]:
            raise ValueError(f"PHYSICS_READY final operation {key} mismatch")

    expected_external_digest = state.get("scenario_report_sha256")
    if expected_external_digest != hashlib.sha256(exact_bytes).hexdigest():
        raise ValueError("PHYSICS_READY external report digest does not match exact report bytes")


def evaluate_executor_readiness(
    snapshot: Mapping[str, object],
    config: Mapping[str, object],
    scenario: Mapping[str, object],
) -> dict[str, object]:
    """Evaluate the genuine ready baseline; every negative test mutates exactly
    one contract and checks the specific failure reason."""
    thresholds = _as_mapping(config.get("thresholds"))
    integrated = _as_mapping(scenario.get("integrated"))
    declaration = _as_mapping(scenario.get("planning_scene_declaration"))
    expected_fixture = declaration or _as_mapping(scenario.get("planning_scene"))
    expected_ids = _scenario_fixture_ids(scenario)
    scenario_fixture_digest = _scenario_fixture_digest(scenario)
    reasons: list[str] = []

    if config.get("execution_profile") != INTEGRATED_EXECUTION_PROFILE:
        reasons.append("execution_profile must be sim_ompl")
    if integrated.get("execution_profile") != INTEGRATED_EXECUTION_PROFILE:
        reasons.append("scenario execution_profile must be sim_ompl")

    try:
        validate_physics_ready_snapshot(snapshot, scenario)
    except ValueError as error:
        reasons.append(str(error))
    if _as_mapping(snapshot.get("model")).get("fingerprint_match") is not True:
        reasons.append("robot-model fingerprint mismatch")

    tf = _as_mapping(snapshot.get("tf"))
    if tf.get("complete") is not True or not _fresh(tf.get("age_s"), thresholds.get("tf_fresh_s")):
        reasons.append("required TF chain is incomplete or stale")

    joint_state = _as_mapping(snapshot.get("joint_state"))
    names = joint_state.get("names")
    positions = joint_state.get("positions")
    velocities = joint_state.get("velocities")
    missing = [
        name
        for name in _REQUIRED_JOINTS
        if not isinstance(names, Sequence) or isinstance(names, (str, bytes)) or name not in names
    ]
    joint_ok = (
        list(names) == list(_REQUIRED_JOINTS)
        if isinstance(names, Sequence) and not isinstance(names, (str, bytes))
        else False
    )
    joint_ok = joint_ok and _finite_sequence(
        positions, length=len(_REQUIRED_JOINTS)
    ) and _finite_sequence(velocities, length=len(_REQUIRED_JOINTS))
    joint_ok = joint_ok and (
        type(joint_state.get("header_stamp_ns")) is int
        and joint_state.get("header_stamp_ns") > 0
        and _fresh(joint_state.get("age_s"), thresholds.get("joint_state_fresh_s"))
        and joint_state.get("publisher_count") == 1
        and joint_state.get("source_node") == CONTROLLER_MANAGER_NODE
        and joint_state.get("logical_controller") == "joint_state_broadcaster"
    )
    if not joint_ok:
        suffix = f"; missing {missing}" if missing else ""
        reasons.append(
            f"/joint_states joint state is incomplete, non-finite, stale, "
            f"unstamped, or wrongly owned{suffix}"
        )

    controllers = _as_mapping(snapshot.get("controllers"))
    controller_records = _as_mapping(controllers.get("logical_controllers"))
    if not (
        controllers.get("manager_healthy") is True
        and controllers.get("manager_source_node") == CONTROLLER_MANAGER_NODE
        and controllers.get("manager_publisher_count") == 1
        and set(controller_records) == {"joint_state_broadcaster", "xarm7_traj_controller"}
        and controller_records.get("joint_state_broadcaster") == {
            "state": "active", "source_node": CONTROLLER_MANAGER_NODE, "cardinality": 1
        }
        and controller_records.get("xarm7_traj_controller") == {
            "state": "active", "source_node": CONTROLLER_MANAGER_NODE, "cardinality": 1
        }
    ):
        reasons.append("controller manager or required logical controllers are unhealthy")

    topics = _as_mapping(snapshot.get("topics"))
    operator = _as_mapping(topics.get(OPERATOR_TOPIC))
    if not (
        operator.get("type") == "std_msgs/msg/Bool"
        and operator.get("publisher_count") == 1
        and operator.get("source_node") == OPERATOR_NODE
        and operator.get("qos") == REQUIRED_TOPICS[OPERATOR_TOPIC]["qos"]
        and operator.get("received") is True
        and operator.get("received_value") is False
        and type(operator.get("received_timestamp_ns")) is int
        and operator.get("received_timestamp_ns") > 0
        and _fresh(operator.get("received_age_s"), operator.get("freshness_limit_s"))
    ):
        reasons.append(
            "operator safety sample is missing, asserted, stale, invalid, or graph-mismatched"
        )

    safety = _as_mapping(snapshot.get("safety"))
    raw_safety_topic = _as_mapping(topics.get(SAFETY_STOP_TOPIC))
    if not (
        safety.get("stop") is False
        and raw_safety_topic.get("data") is False
        and _fresh(safety.get("age_s"), thresholds.get("joint_state_fresh_s"))
        and type(safety.get("sample_count")) is int
        and safety.get("sample_count") >= 2
        and safety.get("type") == "std_msgs/msg/Bool"
        and safety.get("publisher_count") == 1
        and safety.get("source_node") == SAFETY_SUPERVISOR_NODE
    ):
        reasons.append(
            "/sim/hardware/safety_stop safety heartbeat is not fresh, explicit, "
            "typed, or singly owned"
        )

    reasons.extend(_endpoint_failures(snapshot.get("actions"), _REQUIRED_ACTIONS, kind="action"))
    reasons.extend(_endpoint_failures(snapshot.get("services"), _REQUIRED_SERVICES, kind="service"))
    reasons.extend(_topic_failures(snapshot.get("topics")))

    fixture = _as_mapping(snapshot.get("fixture"))
    fixture_topic = _as_mapping(topics.get(FIXTURE_TOPIC))
    try:
        parsed_fixture_payload = json.loads(str(fixture_topic.get("payload", "")))
        fixture_payload = parsed_fixture_payload if isinstance(parsed_fixture_payload, Mapping) else {}
    except (TypeError, ValueError):
        fixture_payload = {}
    payload_ids = fixture_payload.get("owned_ids")
    observed_ids = fixture.get("owned_ids")
    target_id = fixture.get("target_source_id")
    fixture_payload_ok = (
        set(fixture_payload) == FIXTURE_STATUS_KEYS
        and set(fixture) >= FIXTURE_STATUS_KEYS
        and fixture_payload.get("schema_version") == 1
        and fixture_payload.get("state") == "FIXTURE_READY"
        and fixture_payload.get("owner") == FIXTURE_OWNER
        and fixture_payload.get("scenario") == scenario.get("id")
        and fixture_payload.get("revision") == fixture.get("revision")
        and _valid_digest(fixture_payload.get("revision_digest"))
        and fixture_payload.get("revision_digest") == fixture.get("revision_digest")
        and _strict_int(fixture_payload.get("sequence"))
        and _strict_int(fixture.get("sequence"))
        and _strict_int(fixture.get("previous_sequence"))
        and fixture_payload.get("sequence") == fixture.get("sequence")
        and fixture.get("sequence") > fixture.get("previous_sequence") >= 1
        and _finite_number(fixture_payload.get("published_at"))
        and _finite_number(fixture.get("published_at"))
        and abs(float(fixture_payload.get("published_at")) - float(fixture.get("published_at"))) <= 1.0e-6
        and _ordered_string_ids(payload_ids)
        and _ordered_string_ids(observed_ids)
        and _ordered_string_ids(expected_ids)
        and payload_ids == observed_ids == list(expected_ids)
        and isinstance(target_id, str)
        and payload_ids.count(target_id) == 1
        and fixture_payload.get("target_source_id") == target_id
        and fixture_payload.get("target_handoff") == FIXTURE_TARGET_HANDOFF
        and _valid_digest(fixture_payload.get("fixture_descriptor_sha256"))
        and fixture_payload.get("fixture_descriptor_sha256") == fixture.get("fixture_descriptor_sha256")
        and (scenario_fixture_digest is not None
             and fixture.get("fixture_descriptor_sha256") == scenario_fixture_digest)
    )
    if set(fixture_payload) != FIXTURE_STATUS_KEYS:
        reasons.append("fixture payload has extra or missing keys")

    fixture_ok = (
        fixture.get("schema_version") == 1
        and fixture.get("state") == "FIXTURE_READY"
        and fixture.get("owner") == FIXTURE_OWNER
        and fixture.get("scenario") == scenario.get("id")
        and fixture.get("revision") == expected_fixture.get("revision")
        and _valid_digest(fixture.get("revision_digest"))
        and fixture.get("revision_digest") == expected_fixture.get("revision_digest")
        and fixture.get("target_source_id") == expected_fixture.get("target_source_id")
        and fixture.get("target_handoff") == FIXTURE_TARGET_HANDOFF
        and _valid_digest(fixture.get("fixture_descriptor_sha256"))
        and (scenario_fixture_digest is not None
             and fixture.get("fixture_descriptor_sha256") == scenario_fixture_digest)
        and fixture.get("fixture_descriptor_sha256") == fixture_payload.get("fixture_descriptor_sha256")
        and _ordered_string_ids(fixture.get("owned_ids"))
        and _ordered_string_ids(expected_ids)
        and not any(item.startswith("pick_and_place/") for item in fixture.get("owned_ids", ()))
        and list(fixture.get("owned_ids")) == list(expected_ids)
        and _strict_int(fixture.get("sequence"))
        and _strict_int(fixture.get("previous_sequence"))
        and fixture.get("sequence") > fixture.get("previous_sequence") >= 1
        and _strict_int(fixture.get("sample_count"))
        and fixture.get("sample_count") >= 2
        and _fresh(fixture.get("age_s"), thresholds.get("fixture_fresh_s"))
        and fixture_topic.get("type") == "std_msgs/msg/String"
        and fixture_topic.get("publisher_count") == 1
        and fixture_topic.get("source_node") == FIXTURE_PUBLISHER_NODE
        and fixture_topic.get("qos") == REQUIRED_TOPICS[FIXTURE_TOPIC]["qos"]
        and fixture_payload_ok
    )
    if not fixture_ok:
        reasons.append("fixture heartbeat/revision/digest/ownership/sequence does not match")

    scene = _as_mapping(snapshot.get("planning_scene"))
    scene_owned_ids = list(scene.get("owned_ids", ()))
    attached_ids = list(scene.get("attached_ids", ()))
    source_id = expected_fixture.get("target_source_id")
    if not (
        scene_owned_ids == list(expected_ids)
        and source_id in scene_owned_ids
        and source_id not in attached_ids
        and len(scene_owned_ids) == len(set(scene_owned_ids))
        and not (set(scene_owned_ids) & set(attached_ids))
    ):
        reasons.append("PlanningScene does not contain the exact world-only fixture target contract")

    if snapshot.get("robot_in_collision", True):
        reasons.append("robot starts in collision")
    return {"ready": not reasons, "reasons": reasons}


def build_journal_graph_projection(
    *,
    fixture_payload: str,
) -> dict[str, object]:
    """Build the Task 3 graph projection consumed by ``PlanningSceneJournal``.

    Matches ``planning_scene_journal.validate_graph_evidence`` exactly: recorder
    identity ``node_name="/tinker_integrated_gate_executor"``, ``namespace="/"``,
    ``remap_table={}``; the exact topic/service key sets, types and uppercase
    QoS; the recorder node among all required subscribers/clients; exactly one
    fixture publisher ``/fixture_planning_scene``; the exact canonical compact
    payload string retained as ``payload`` (parsed data separate).
    """
    recorder: Mapping[str, str] = {"node": OPERATOR_NODE, "node_namespace": ""}
    move_group_publisher: Mapping[str, str] = {"node": "/move_group", "node_namespace": ""}
    fixture_publisher: Mapping[str, str] = {"node": FIXTURE_PUBLISHER_NODE, "node_namespace": ""}
    move_group_server: Mapping[str, str] = {"node": "/move_group", "node_namespace": ""}
    return {
        "node_name": OPERATOR_NODE,
        "namespace": "/",
        "remap_table": {},
        "topics": {
            PLANNING_SCENE_TOPIC: {
                "type": "moveit_msgs/msg/PlanningScene",
                "requested_qos": dict(JOURNAL_TOPIC_QOS),
                "offered_qos": dict(JOURNAL_TOPIC_QOS),
                "publishers": [dict(move_group_publisher)],
                "subscribers": [dict(recorder)],
            },
            MONITORED_PLANNING_SCENE_TOPIC: {
                "type": "moveit_msgs/msg/PlanningScene",
                "requested_qos": dict(JOURNAL_TOPIC_QOS),
                "offered_qos": dict(JOURNAL_TOPIC_QOS),
                "publishers": [dict(move_group_publisher)],
                "subscribers": [dict(recorder)],
            },
            FIXTURE_TOPIC: {
                "type": "std_msgs/msg/String",
                "requested_qos": dict(JOURNAL_TOPIC_QOS),
                "offered_qos": dict(JOURNAL_TOPIC_QOS),
                "publishers": [dict(fixture_publisher)],
                "subscribers": [dict(recorder)],
                "payload": str(fixture_payload),
            },
        },
        "services": {
            "/get_planning_scene": {
                "type": "moveit_msgs/srv/GetPlanningScene",
                "requested_qos": dict(JOURNAL_SERVICE_QOS),
                "offered_qos": dict(JOURNAL_SERVICE_QOS),
                "servers": [dict(move_group_server)],
                "clients": [dict(recorder)],
            },
            "/apply_planning_scene": {
                "type": "moveit_msgs/srv/ApplyPlanningScene",
                "requested_qos": dict(JOURNAL_SERVICE_QOS),
                "offered_qos": dict(JOURNAL_SERVICE_QOS),
                "servers": [dict(move_group_server)],
                "clients": [dict(recorder)],
            },
        },
    }


# ---------------------------------------------------------------------------
# ROS-lazy goal builders (import generated messages only at call time)
# ---------------------------------------------------------------------------

def build_joint_move_group_goal(
    joints: Sequence[float], *, plan_only: bool
):
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint

    values = _finite_vector(joints, length=7, name="joint goal")
    goal = MoveGroup.Goal()
    goal.request.group_name = "xarm7"
    goal.request.pipeline_id = "ompl"
    goal.request.num_planning_attempts = 3
    goal.request.allowed_planning_time = 3.0
    constraints = Constraints()
    constraints.joint_constraints = [
        JointConstraint(
            joint_name=f"joint{i}",
            position=value,
            tolerance_above=0.01,
            tolerance_below=0.01,
            weight=1.0,
        )
        for i, value in enumerate(values, start=1)
    ]
    goal.request.goal_constraints = [constraints]
    goal.planning_options.plan_only = bool(plan_only)
    goal.planning_options.replan = False
    return goal


def build_pose_move_group_goal(pose, *, plan_only: bool):
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import BoundingVolume, Constraints, OrientationConstraint, PositionConstraint
    from shape_msgs.msg import SolidPrimitive

    if pose.header.frame_id != "base_link":
        raise ValueError("pose goal must use base_link")
    _validate_quaternion(pose.pose.orientation)
    primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])
    position = PositionConstraint(
        header=pose.header,
        link_name="link_tcp",
        constraint_region=BoundingVolume(
            primitives=[primitive], primitive_poses=[pose.pose]
        ),
        weight=1.0,
    )
    orientation = OrientationConstraint(
        header=pose.header,
        link_name="link_tcp",
        orientation=pose.pose.orientation,
        absolute_x_axis_tolerance=0.05,
        absolute_y_axis_tolerance=0.05,
        absolute_z_axis_tolerance=0.05,
        weight=1.0,
    )
    goal = MoveGroup.Goal()
    goal.request.group_name = "xarm7"
    goal.request.pipeline_id = "ompl"
    goal.request.num_planning_attempts = 3
    goal.request.allowed_planning_time = 3.0
    goal.request.goal_constraints = [
        Constraints(
            position_constraints=[position], orientation_constraints=[orientation]
        )
    ]
    goal.planning_options.plan_only = bool(plan_only)
    goal.planning_options.replan = False
    return goal


def deterministic_cube_cloud(*, frame_id="base_link"):
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    offsets = (-0.04, -0.02, 0.0, 0.02, 0.04)
    points = [
        (0.65 + dx, 0.0 + dy, 0.60 + dz)
        for dx in offsets for dy in offsets for dz in offsets
    ]
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    header = Header(frame_id=frame_id)
    header.stamp.sec = 7
    cloud = point_cloud2.create_cloud(header, fields, points)
    cloud.height = 1
    cloud.width = 125
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = 1500
    cloud.is_dense = True
    return cloud


def build_pick_goal(
    *,
    target_pose,
    candidate_poses: Sequence,
    env_points,
    object_points,
    back_positions: Sequence[float],
    use_mesh: bool,
    stay: bool,
    two_stage_plan: bool = False,
):
    from tinker_arm_msgs.action import Pick

    back = _finite_vector(back_positions, length=7, name="back_positions")
    if not candidate_poses or candidate_poses[0] != target_pose:
        raise ValueError("candidate_poses must be non-empty and start with target_pose")
    _validate_quaternion(target_pose.orientation)
    goal = Pick.Goal()
    goal.target_pose = target_pose
    goal.candidate_poses = list(candidate_poses)
    goal.env_points = env_points
    goal.object_points = object_points
    goal.back_positions = back
    goal.two_stage_plan = bool(two_stage_plan)
    goal.use_mesh = bool(use_mesh)
    goal.stay = bool(stay)
    return goal


def build_place_goal(
    *,
    target_point,
    orientation,
    env_points,
    back_positions: Sequence[float],
):
    from tinker_arm_msgs.action import Place

    if target_point.header.frame_id != "base_link":
        raise ValueError("place target must use base_link")
    _validate_quaternion(orientation.orientation)
    goal = Place.Goal()
    goal.target_point = target_point
    goal.orientation = orientation
    goal.env_points = env_points
    goal.back_positions = _finite_vector(back_positions, length=7, name="back_positions")
    return goal


# ---------------------------------------------------------------------------
# Live ROS-lazy executor (imported/instantiated only under sourced Humble)
# ---------------------------------------------------------------------------

_ROS_IMPORTS: dict[str, Any] = {}


def _load_ros() -> dict[str, Any]:
    """Import ROS only at live execution time (Humble CPython 3.10)."""
    if _ROS_IMPORTS:
        return _ROS_IMPORTS
    import rclpy
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from controller_manager_msgs.srv import (
        ConfigureController,
        ListControllers,
        LoadController,
        SwitchController,
    )
    from moveit_msgs.action import ExecuteTrajectory, MoveGroup
    from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetStateValidity
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from tinker_arm_msgs.action import CartesianMove, Fold, JointMove, Pick, Place
    from tinker_arm_msgs.srv import ArmJointService

    _ROS_IMPORTS.update(locals())
    return _ROS_IMPORTS


def _operator_qos(ros: Mapping[str, Any]) -> Any:
    return ros["QoSProfile"](
        depth=1,
        reliability=ros["ReliabilityPolicy"].RELIABLE,
        durability=ros["DurabilityPolicy"].TRANSIENT_LOCAL,
    )


class IntegratedGateExecutor:
    """Live Gate-C plan-only executor node.

    Runs under sourced Humble Python 3.10 only.  Creates the node
    ``/tinker_integrated_gate_executor`` (namespace ``/``, no remaps), typed
    action/service clients for every required endpoint, the operator publisher
    ``/sim/safety/operator`` (``std_msgs/msg/Bool``, reliable/transient-local/
    depth 1), the PlanningScene journal recorder subscriptions/clients, and the
    Gate C plan-only flow.  It never sends ``/execute_trajectory`` in Gate C and
    never publishes ``/isaac_joint_commands`` or another command path during
    plan-only windows.  Plan-only evidence records remain ``diagnostic_only =
    true`` and never claim execution.
    """

    def __init__(
        self,
        *,
        scenario: Mapping[str, object],
        attempt_dir: Path,
        config: Mapping[str, object],
        ros_domain_id: int | str,
        journal: Any = None,
    ) -> None:
        domain_id = int(ros_domain_id)
        if domain_id < 0 or domain_id > 232:
            raise ValueError("ROS_DOMAIN_ID must be an integer in [0, 232]")
        ros = _load_ros()
        self.ros = ros
        self.scenario = scenario
        self.attempt_dir = Path(attempt_dir)
        self.config = config
        from rclpy import init as _init_rclpy

        _init_rclpy()
        self.node = ros["Node"](OPERATOR_NODE, namespace="/", cli_args=[])
        self.operator_publisher = self.node.create_publisher(
            ros["Bool"], OPERATOR_TOPIC, _operator_qos(ros)
        )
        self._create_clients()
        self._create_journal_recorder()
        self.journal = journal

    def _create_clients(self) -> None:
        ros = self.ros
        self._action_clients: dict[str, Any] = {}
        for name, action_type in REQUIRED_ACTIONS.items():
            self._action_clients[name] = ros["ActionClient"](self.node, getattr(ros, action_type.split("/")[-1]), name)
        self._service_clients: dict[str, Any] = {}
        for name, service_type in REQUIRED_SERVICES.items():
            message_type = _service_type_to_ros(service_type, ros)
            self._service_clients[name] = self.node.create_client(message_type, name)

    def _create_journal_recorder(self) -> None:
        ros = self.ros
        planning_scene_qos = ros["QoSProfile"](
            depth=1,
            reliability=ros["ReliabilityPolicy"].RELIABLE,
            durability=ros["DurabilityPolicy"].TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            ros["String"], FIXTURE_TOPIC, self._on_fixture_payload, planning_scene_qos
        )
        # The moveit planning-scene topics are recorded for diagnostic scene
        # consistency only; the recorder never emits physics fields.
        self.node.create_subscription(
            ros["String"], PLANNING_SCENE_TOPIC, self._on_planning_scene_diagnostic, planning_scene_qos
        )
        self.node.create_subscription(
            ros["String"], MONITORED_PLANNING_SCENE_TOPIC, self._on_planning_scene_diagnostic, planning_scene_qos
        )
        self._fixture_payload: str | None = None

    def _on_fixture_payload(self, message: Any) -> None:
        self._fixture_payload = str(getattr(message, "data", ""))

    def _on_planning_scene_diagnostic(self, message: Any) -> None:
        # Diagnostic scene consistency only; no physics field is recorded.
        return

    def publish_operator(self, value: bool) -> None:
        if value not in (False, True):
            raise ValueError("operator payload allowlist is [False, True]")
        message = self.ros["Bool"]()
        message.data = bool(value)
        self.operator_publisher.publish(message)

    def run_gate_c_plan_only(self, scenario_id: str, *, joints: Sequence[float] | None = None) -> dict[str, object]:
        """Run exactly one plan-only Gate C scenario through ``/move_action``."""
        ros = self.ros
        client = self._action_clients["/move_action"]
        goal = build_joint_move_group_goal(
            joints if joints is not None else [0.0] * 7, plan_only=True
        )
        future = client.send_goal_async(goal)
        deadline = time.monotonic() + float(
            _as_mapping(self.config.get("thresholds")).get("plan_timeout_s", 15.0)
        )
        while not future.done() and time.monotonic() < deadline:
            ros["rclpy"].spin_once(self.node, timeout_sec=0.05)
        if not future.done():
            future.cancel()
            return {"scenario_id": scenario_id, "status": "timeout", "diagnostic_only": True}
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {"scenario_id": scenario_id, "status": "goal-rejected", "diagnostic_only": True}
        result_future = goal_handle.get_result_async()
        while not result_future.done() and time.monotonic() < deadline:
            ros["rclpy"].spin_once(self.node, timeout_sec=0.05)
        if not result_future.done():
            goal_handle.cancel()
            return {"scenario_id": scenario_id, "status": "timeout", "diagnostic_only": True}
        result = result_future.result()
        return {
            "scenario_id": scenario_id,
            "status": "planned" if result.result.error_code.value == 1 else "planner-non-success",
            "error_code": result.result.error_code.value,
            "diagnostic_only": True,
        }

    def shutdown(self) -> None:
        self.node.destroy_node()


def _service_type_to_ros(service_type: str, ros: Mapping[str, Any]) -> Any:
    """Map a canonical service type string to the imported generated class."""
    message_name = service_type.rsplit("/", 1)[-1]
    if service_type.startswith("controller_manager_msgs/srv/"):
        mapping = {
            "ListControllers": ros["ListControllers"],
            "LoadController": ros["LoadController"],
            "ConfigureController": ros["ConfigureController"],
            "SwitchController": ros["SwitchController"],
        }
    elif service_type.startswith("moveit_msgs/srv/"):
        mapping = {
            "GetPlanningScene": ros["GetPlanningScene"],
            "ApplyPlanningScene": ros["ApplyPlanningScene"],
            "GetStateValidity": ros["GetStateValidity"],
            "GetCartesianPath": ros["GetCartesianPath"],
        }
    elif service_type == "std_srvs/srv/Trigger":
        mapping = {"Trigger": ros["Trigger"]}
    elif service_type == "tinker_arm_msgs/srv/ArmJointService":
        mapping = {"ArmJointService": ros["ArmJointService"]}
    else:
        raise ValueError(f"unsupported service type: {service_type}")
    if message_name not in mapping:
        raise ValueError(f"unsupported service type: {service_type}")
    return mapping[message_name]


__all__ = [
    "CONTROLLER_MANAGER_NODE",
    "DIGEST",
    "EARLIER_OPERATION_OPTIONAL_FIELDS",
    "FINAL_SIMULATION_STATE",
    "FIXTURE_OWNER",
    "FIXTURE_PUBLISHER_NODE",
    "FIXTURE_TARGET_HANDOFF",
    "FIXTURE_TOPIC",
    "IDENTITY_KEYS",
    "INTEGRATED_EXECUTION_PROFILE",
    "ISAAC_COMMAND_TOPIC",
    "JOINT_STATES_TOPIC",
    "JOURNAL_SERVICE_QOS",
    "JOURNAL_TOPIC_QOS",
    "OPERATION_KEYS",
    "OPERATOR_NODE",
    "OPERATOR_TOPIC",
    "PHYSICS_READY_BOUNDARY",
    "PHYSICS_READY_GATE_NODE",
    "REPORT_KEYS",
    "REPORT_REVISION",
    "REQUIRED_ACTIONS",
    "REQUIRED_SERVICES",
    "REQUIRED_TOPICS",
    "SAFETY_STOP_TOPIC",
    "SAFETY_SUPERVISOR_NODE",
    "SIMULATION_STATE_PLAYING",
    "IntegratedGateExecutor",
    "build_joint_move_group_goal",
    "build_journal_graph_projection",
    "build_pick_goal",
    "build_place_goal",
    "build_pose_move_group_goal",
    "deterministic_cube_cloud",
    "evaluate_executor_readiness",
    "expected_physics_ready_report",
    "validate_physics_ready_snapshot",
]
