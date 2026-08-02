"""Integrated OMPL qualification Gate-C plan-only executor (Task 4).

This module is ROS-lazy: importing it under the simulator CPython 3.12 venv
never imports ``rclpy`` or any generated ROS message type.  All generated-message
imports happen inside :func:`_load_ros` or the goal-builder call paths, which the
Humble suite exercises under sourced ROS Humble Python 3.10.

Pure helpers (importable everywhere):

- endpoint/type/cardinality/QoS contract constants;
- ``expected_physics_ready_report`` / ``validate_physics_ready_snapshot``
  reconciled with the real canonical multi-operation public report (one-key
  ``integrated`` mapping, scenario-declaration-bound fixture descriptor digest);
- ``evaluate_executor_readiness`` with the config-authoritative operator
  freshness threshold and the genuine positive-ready baseline;
- ``stage_c_dispatch`` validating the three Stage-C plan-only scenarios and
  returning a ROS-free dispatch spec;
- ``build_journal_graph_projection`` requiring an explicit observed-graph input
  (never fabricated publisher/server identities) and normalizing it for the
  Task-3 ``planning_scene_journal.validate_graph_evidence`` schema.

The live :class:`IntegratedGateExecutor` (Humble-only) constructs a valid
isolated rclpy node, subscribes to the real ``moveit_msgs/msg/PlanningScene``
topics, owns a :class:`~planning_scene_journal.PlanningSceneJournal`, gates
every goal on live readiness, dispatches the three Stage-C scenarios with
plan-only semantics, writes the Task-4 artifact set, and finalizes the journal.
It never calls ``/execute_trajectory`` in Gate C and never publishes
``/isaac_joint_commands``.  Task 7 later correlates physical truth; Task 4
records diagnostic scene consistency only and never supplies physical
contact/force/object-pose/verdict fields.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
import re
import sys
import tempfile
import time
import uuid as _uuid
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.fixture_contract import (  # noqa: E402
    geometry_signature_sha256,
    readback_geometry,
    spec_geometry,
)
from tinker_sim_bridge.fixture_planning_scene import (  # noqa: E402
    fixture_descriptor_sha256,
    fixture_owned_ids,
    fixture_to_specs,
    load_mesh_asset,
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
RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"

# rclpy node names are unqualified base names; the qualification identity is
# the fully qualified name ``/tinker_integrated_gate_executor`` (namespace
# ``/`` + basename).  ``use_global_arguments=False`` keeps launch/global remaps
# from changing the qualification identity.
NODE_BASENAME = "tinker_integrated_gate_executor"
OPERATOR_NODE = "/tinker_integrated_gate_executor"
OPERATOR_NODE_NAMESPACE = "/"
FIXTURE_PUBLISHER_NODE = "/fixture_planning_scene"
SAFETY_SUPERVISOR_NODE = "/tinker_sim_safety_supervisor"
CONTROLLER_MANAGER_NODE = "/controller_manager"
GRIPPER_FACADE_NODE = "/tinker_sim_gripper_facade"
PICK_AND_PLACE_NODE = "/pick_and_place"
PHYSICS_READY_GATE_NODE = "/tinker_sim_physics_ready_gate"
MOVE_GROUP_NODE = "/move_group"
PLANNING_SCENE_TOPIC = "/planning_scene"
MONITORED_PLANNING_SCENE_TOPIC = "/monitored_planning_scene"
FIXTURE_TOPIC = "/sim/status/planning_scene_fixture"
JOINT_STATES_TOPIC = "/joint_states"
OPERATOR_TOPIC = "/sim/safety/operator"
SAFETY_STOP_TOPIC = "/sim/hardware/safety_stop"
ISAAC_COMMAND_TOPIC = "/isaac_joint_commands"

#: Stage C is exactly these three plan-only scenarios.
STAGE_C_SCENARIOS: tuple[str, ...] = (
    "qualification-moveit-plan-joint",
    "qualification-moveit-plan-pose",
    "qualification-moveit-plan-blocked",
)

#: Canonical seven-joint outbound target for the Stage-C joint scenario.
Q_OUTBOUND: tuple[float, ...] = (0.20, -0.20, 0.15, 0.30, -0.15, 0.20, 0.15)

#: Task-4 Gate-C explicit journal contract (scenario JSON does not yet carry
#: journal fields).  This is a Stage-C-only derivation; later D/E tasks extend
#: their own explicit contracts.
GATE_C_REQUIRED_EVENT_ORDER: tuple[str, ...] = ("fixture-ready", "teardown")
GATE_C_FORBIDDEN_EVENTS: tuple[str, ...] = (
    "before-pick",
    "scene-attach",
    "lift-complete",
    "transport",
    "before-release",
    "scene-detach",
    "released-settled",
    "task-cleanup",
)
#: The eight Task-4 forbidden manipulation events apply unchanged to every D
#: journal; no attach/detach/release event is ever emitted in Gate D.
D_FORBIDDEN_EVENTS: tuple[str, ...] = GATE_C_FORBIDDEN_EVENTS

#: Stage D is exactly these six scenarios.
STAGE_D_SCENARIOS: tuple[str, ...] = (
    "qualification-moveit-execute-joint",
    "qualification-moveit-execute-pose",
    "qualification-moveit-cartesian-retreat",
    "qualification-moveit-gripper",
    "qualification-moveit-cancel",
    "qualification-moveit-safety",
)

#: D handler kind per exact scenario id.
STAGE_D_KIND: Mapping[str, str] = {
    "qualification-moveit-execute-joint": "execute-joint",
    "qualification-moveit-execute-pose": "execute-pose",
    "qualification-moveit-cartesian-retreat": "retreat",
    "qualification-moveit-gripper": "gripper",
    "qualification-moveit-cancel": "cancel",
    "qualification-moveit-safety": "safety",
}

#: Exact declared polarity per D scenario.
STAGE_D_EXPECTED_POLARITY: Mapping[str, str] = {
    "qualification-moveit-execute-joint": "positive",
    "qualification-moveit-execute-pose": "positive",
    "qualification-moveit-cartesian-retreat": "positive",
    "qualification-moveit-gripper": "positive",
    "qualification-moveit-cancel": "cancel",
    "qualification-moveit-safety": "safety",
}

#: Exact declared ``expected_physical`` list per D scenario.
STAGE_D_EXPECTED_PHYSICAL: Mapping[str, tuple[str, ...]] = {
    "qualification-moveit-execute-joint": ("joint_execution_tracks", "terminal_success"),
    "qualification-moveit-execute-pose": ("pose_execution_reaches_tcp", "terminal_success"),
    "qualification-moveit-cartesian-retreat": ("cartesian_retreat_collision_aware", "terminal_success"),
    "qualification-moveit-gripper": ("gripper_travel_predicates", "terminal_success"),
    "qualification-moveit-cancel": ("execute_goal_canceled", "quiescent_after_cancel", "no_later_stage"),
    "qualification-moveit-safety": ("safety_effective_stop", "target_frozen", "no_auto_resume"),
}

#: Scenario-specific D journal diagnostic event order (Task 3 graph projection
#: stays unchanged; these labels are diagnostics, not physical truth).
STAGE_D_REQUIRED_EVENT_ORDER: Mapping[str, tuple[str, ...]] = {
    "execute-joint": ("fixture-ready", "execution-start", "execution-terminal", "teardown"),
    "execute-pose": ("fixture-ready", "execution-start", "execution-terminal", "teardown"),
    "retreat": ("fixture-ready", "retreat-start", "retreat-terminal", "teardown"),
    "gripper": ("fixture-ready", "gripper-open-terminal", "gripper-close-terminal", "teardown"),
    "cancel": ("fixture-ready", "execution-start", "cancel-requested", "quiescent", "teardown"),
    "safety": ("fixture-ready", "execution-start", "effective-stop", "operator-clear", "quiescent", "teardown"),
}

#: ExecuteTrajectory / FJT split-path endpoints.
EXECUTE_TRAJECTORY_ENDPOINT = "/execute_trajectory"
FJT_ENDPOINT = "/xarm7_traj_controller/follow_joint_trajectory"
FJT_STATUS_TOPIC = "/xarm7_traj_controller/follow_joint_trajectory/_action/status"
GRIPPER_ENDPOINT = "/xarm_gripper/gripper_action"
CARTESIAN_MOVE_ENDPOINT = "/cartesian_move_action"

#: ``moveit_msgs/action/ExecuteTrajectory`` terminal action status ints
#: (``action_msgs/msg/GoalStatus``).  Unknown/malformed statuses never pass.
EXECUTE_STATUS_SUCCEEDED = 4
EXECUTE_STATUS_CANCELED = 5
EXECUTE_STATUS_ABORTED = 6
_EXECUTE_STATUS_NAMES: Mapping[int, str] = {
    EXECUTE_STATUS_SUCCEEDED: "succeeded",
    EXECUTE_STATUS_CANCELED: "canceled",
    EXECUTE_STATUS_ABORTED: "aborted",
}

#: Native gripper contract mirrors current production behavior; Task 5
#: qualification constants (config/scenario files are unchanged).
GRIPPER_OPEN_POSITION = 0.0
GRIPPER_CLOSE_POSITION = 0.85
GRIPPER_MAX_EFFORT = 10.0

#: Cartesian retreat geometry: top-down grasp, exactly ``RETREAT_DISTANCE_M``
#: along the declared ``base_link`` axis, preserving orientation
#: (``object_center_z=0.64``, ``grasp_tcp_z=0.72`` contract).
RETREAT_AXIS = "+z"
RETREAT_DISTANCE_M = 0.10

#: Bound on the cached FJT status entries retained by the executor.
FJT_STATUS_CACHE_LIMIT = 64

#: Gate-E negative identities for the fail-closed ``run_pick_place_negative``
#: stub (later E-stage task replaces the stub; Task 5 never pre-empts Gate E).
STAGE_E_CANCEL_NEGATIVES: Mapping[str, tuple[str, ...]] = {
    "qualification-pick-place-cancel-approach": ("approach-start", "cancel"),
    "cancel-approach": ("approach-start", "cancel"),
    "qualification-pick-place-cancel-transport": ("approach-start", "lift-complete", "cancel"),
    "cancel-transport": ("approach-start", "lift-complete", "cancel"),
}

TASK_NAMESPACE = "pick_and_place/"
TARGET_OBJECT_ID = "pick_and_place/object_mesh"

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
    "/move_action": MOVE_GROUP_NODE,
    "/execute_trajectory": MOVE_GROUP_NODE,
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
    "/get_planning_scene": MOVE_GROUP_NODE,
    "/apply_planning_scene": MOVE_GROUP_NODE,
    "/check_state_validity": MOVE_GROUP_NODE,
    "/compute_cartesian_path": MOVE_GROUP_NODE,
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
#: The two PlanningScene topics mirror the stock MoveIt2 Humble publisher's
#: plain depth-100 ``rclcpp::QoS`` (RELIABLE + VOLATILE); the fixture status
#: topic stays RELIABLE/TRANSIENT_LOCAL/depth 1 (F2.3).
JOURNAL_PLANNING_SCENE_TOPIC_QOS: Mapping[str, object] = {
    "reliability": "RELIABLE",
    "durability": "VOLATILE",
    "depth": 100,
}
JOURNAL_FIXTURE_TOPIC_QOS: Mapping[str, object] = {
    "reliability": "RELIABLE",
    "durability": "TRANSIENT_LOCAL",
    "depth": 1,
}
#: Backward-compatible alias retained for the fixture-status topic claim.
JOURNAL_TOPIC_QOS: Mapping[str, object] = dict(JOURNAL_FIXTURE_TOPIC_QOS)
JOURNAL_SERVICE_QOS: Mapping[str, object] = {
    "reliability": "RELIABLE",
    "durability": "VOLATILE",
}

#: MoveIt planning-stage non-success codes valid for the blocked diagnostic
#: (F2.4).  Only codes that unambiguously represent planner/IK non-success after
#: a valid plan-only request (PLANNING_FAILED=-1, INVALID_MOTION_PLAN=-2,
#: NO_IK_SOLUTION=5).  Request-level/configuration/timeout/transport errors are
#: never a blocked pass.
MOVEIT_SUCCESS_CODE = 1
MOVEIT_PLANNING_NON_SUCCESS_CODES: frozenset[int] = frozenset({-1, -2, 5})

#: Complete expected malformed-message exception set contained at the
#: PlanningScene callback boundary (F3.2): attribute/type/value and
#: serialization failures that arise from wrong-shaped input.  Process-control
#: exceptions (KeyboardInterrupt/SystemExit) and unrelated fatal errors are
#: never swallowed.
_SCENE_CALLBACK_EXCEPTIONS: tuple[type[Exception], ...] = (
    AttributeError,
    IndexError,
    KeyError,
    TypeError,
    ValueError,
)

#: Journal/evidence artifact names written per attempt.
ARTIFACT_JSONL_FILES: tuple[str, ...] = (
    "integrated-execution.jsonl",
    "moveit-plans.jsonl",
    "controller-results.jsonl",
    "visual-capture-requests.jsonl",
)


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


def _resolve_declared_mesh(mesh: Mapping[str, object]):
    """Resolve a declared mesh asset to (vertices, triangles) through the bridge."""
    return load_mesh_asset(mesh, project_root=ROOT)


def expected_fixture_geometry_digest(
    declaration: Mapping[str, object], *, resolve_mesh=None
) -> str:
    """Return the deterministic fixture-owned geometry projection digest (F3.3).

    The projection is the declared-order owned collision-body geometry in the
    same canonical descriptor form the bridge's ``readback_geometry`` produces
    for a received MoveIt ``CollisionObject``: frame, primitive type +
    dimensions + poses (and, for mesh fixtures, resolved vertices + triangles).
    The digest is ``geometry_signature_sha256`` over exactly those ordered
    descriptors, so it binds the exact declared IDs, order, frame, geometry, and
    pose — never unrelated full-scene serialization (robot state / ACM).

    Mesh fixtures require *resolve_mesh* (a callable mapping a
    ``{"uri", "sha256", "scale"}`` declaration to ``(vertices, triangles)``);
    the executor supplies the bridge asset resolver by default.
    """
    if resolve_mesh is None:
        resolve_mesh = _resolve_declared_mesh
    specs = fixture_to_specs(declaration)
    descriptors = [spec_geometry(spec, resolve_mesh=resolve_mesh) for spec in specs]
    return geometry_signature_sha256(descriptors)


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
    # F1.5: the configured operator freshness threshold is the authority.  If the
    # config has no dedicated ``operator_fresh_s``, the documented fallback is
    # ``thresholds.fixture_fresh_s`` (current value 0.25).  A snapshot-supplied
    # threshold is never trusted as authority.
    operator_fresh_limit = thresholds.get("operator_fresh_s", thresholds.get("fixture_fresh_s"))
    if not (
        operator.get("type") == "std_msgs/msg/Bool"
        and operator.get("publisher_count") == 1
        and operator.get("source_node") == OPERATOR_NODE
        and operator.get("qos") == REQUIRED_TOPICS[OPERATOR_TOPIC]["qos"]
        and operator.get("received") is True
        and operator.get("received_value") is False
        and type(operator.get("received_timestamp_ns")) is int
        and operator.get("received_timestamp_ns") > 0
        and _fresh(operator.get("received_age_s"), operator_fresh_limit)
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


# ---------------------------------------------------------------------------
# Observed graph validation / journal projection
# ---------------------------------------------------------------------------

def _validate_endpoint_entries(label: str, endpoints: object) -> list[dict[str, str]]:
    """Validate real endpoint metadata (never payload-only claims)."""
    if not isinstance(endpoints, (list, tuple)) or not endpoints:
        raise ValueError(f"{label} must have real endpoint metadata")
    normalized: list[dict[str, str]] = []
    for endpoint in endpoints:
        if isinstance(endpoint, Mapping):
            node = endpoint.get("node")
            if not isinstance(node, str) or not node:
                raise ValueError(f"{label} has an endpoint without a real node")
            node_namespace = endpoint.get("node_namespace")
            normalized.append(
                {
                    "node": node,
                    "node_namespace": str(node_namespace) if node_namespace is not None else "",
                }
            )
        else:
            raise ValueError(f"{label} has malformed endpoint metadata")
    return normalized


def _qos_exact(qos: object, expected: Mapping[str, object]) -> bool:
    if not isinstance(qos, Mapping) or set(qos) != set(expected):
        return False
    for key, expected_value in expected.items():
        value = qos.get(key)
        if key == "depth":
            if isinstance(value, bool) or not isinstance(value, int) or value != expected_value:
                return False
        elif value != expected_value:
            return False
    return True


def _validate_observed_graph(
    observed_graph: object,
) -> dict[str, object]:
    """Validate an observed graph and return the exact projection shape.

    The observed graph must be a mapping with the exact recorder identity, the
    exact journal topic/service interface sets, exact types/QoS, real endpoint
    metadata, the recorder among every required topic subscriber and service
    client, exactly one ``/fixture_planning_scene`` publisher, and no extra
    interfaces.  The fixture topic entry does not carry a payload; the payload
    is injected by :func:`build_journal_graph_projection`.
    """
    if not isinstance(observed_graph, Mapping):
        raise ValueError("observed graph must be a mapping")
    if observed_graph.get("node_name") != OPERATOR_NODE:
        raise ValueError("observed graph node_name must be /tinker_integrated_gate_executor")
    if observed_graph.get("namespace") != OPERATOR_NODE_NAMESPACE:
        raise ValueError("observed graph namespace must be /")
    remap_table = observed_graph.get("remap_table")
    if not isinstance(remap_table, Mapping) or len(remap_table) != 0:
        raise ValueError("observed graph remap_table must be empty")
    topics = observed_graph.get("topics")
    services = observed_graph.get("services")
    if not isinstance(topics, Mapping) or not isinstance(services, Mapping):
        raise ValueError("observed graph must include topics and services mappings")
    expected_topic_keys = {
        PLANNING_SCENE_TOPIC,
        MONITORED_PLANNING_SCENE_TOPIC,
        FIXTURE_TOPIC,
    }
    expected_service_keys = {"/get_planning_scene", "/apply_planning_scene"}
    if set(topics) != expected_topic_keys:
        raise ValueError(
            "observed graph topics must be exactly "
            f"{sorted(expected_topic_keys)}"
        )
    if set(services) != expected_service_keys:
        raise ValueError(
            "observed graph services must be exactly "
            f"{sorted(expected_service_keys)}"
        )

    normalized_topics: dict[str, dict[str, object]] = {}
    for name, expected_type in (
        (PLANNING_SCENE_TOPIC, "moveit_msgs/msg/PlanningScene"),
        (MONITORED_PLANNING_SCENE_TOPIC, "moveit_msgs/msg/PlanningScene"),
    ):
        entry = _as_mapping(topics.get(name))
        if entry.get("type") != expected_type:
            raise ValueError(f"observed topic {name} has wrong type {entry.get('type')!r}")
        if not _qos_exact(entry.get("requested_qos"), JOURNAL_PLANNING_SCENE_TOPIC_QOS):
            raise ValueError(f"observed topic {name} requested QoS must be RELIABLE/VOLATILE/depth 100")
        if not _qos_exact(entry.get("offered_qos"), JOURNAL_PLANNING_SCENE_TOPIC_QOS):
            raise ValueError(f"observed topic {name} offered QoS must be RELIABLE/VOLATILE/depth 100")
        publishers = _validate_endpoint_entries(f"observed topic {name} publishers", entry.get("publishers"))
        subscribers = _validate_endpoint_entries(f"observed topic {name} subscribers", entry.get("subscribers"))
        if not any(endpoint["node"] == OPERATOR_NODE for endpoint in subscribers):
            raise ValueError(f"observed topic {name} must be subscribed by {OPERATOR_NODE}")
        normalized_topics[name] = {
            "type": expected_type,
            "requested_qos": dict(JOURNAL_PLANNING_SCENE_TOPIC_QOS),
            "offered_qos": dict(JOURNAL_PLANNING_SCENE_TOPIC_QOS),
            "publishers": publishers,
            "subscribers": subscribers,
        }

    fixture_entry = _as_mapping(topics.get(FIXTURE_TOPIC))
    if fixture_entry.get("type") != "std_msgs/msg/String":
        raise ValueError(f"observed topic {FIXTURE_TOPIC} has wrong type")
    if not _qos_exact(fixture_entry.get("requested_qos"), JOURNAL_FIXTURE_TOPIC_QOS):
        raise ValueError(f"observed topic {FIXTURE_TOPIC} requested QoS must be RELIABLE/TRANSIENT_LOCAL/depth 1")
    if not _qos_exact(fixture_entry.get("offered_qos"), JOURNAL_FIXTURE_TOPIC_QOS):
        raise ValueError(f"observed topic {FIXTURE_TOPIC} offered QoS must be RELIABLE/TRANSIENT_LOCAL/depth 1")
    fixture_publishers = _validate_endpoint_entries(
        f"observed topic {FIXTURE_TOPIC} publishers", fixture_entry.get("publishers")
    )
    fixture_subscribers = _validate_endpoint_entries(
        f"observed topic {FIXTURE_TOPIC} subscribers", fixture_entry.get("subscribers")
    )
    if len(fixture_publishers) != 1:
        raise ValueError(f"observed topic {FIXTURE_TOPIC} must have exactly one publisher")
    if fixture_publishers[0]["node"] != FIXTURE_PUBLISHER_NODE:
        raise ValueError(f"observed topic {FIXTURE_TOPIC} publisher must be {FIXTURE_PUBLISHER_NODE}")
    if not any(endpoint["node"] == OPERATOR_NODE for endpoint in fixture_subscribers):
        raise ValueError(f"observed topic {FIXTURE_TOPIC} must be subscribed by {OPERATOR_NODE}")
    normalized_topics[FIXTURE_TOPIC] = {
        "type": "std_msgs/msg/String",
        "requested_qos": dict(JOURNAL_FIXTURE_TOPIC_QOS),
        "offered_qos": dict(JOURNAL_FIXTURE_TOPIC_QOS),
        "publishers": fixture_publishers,
        "subscribers": fixture_subscribers,
    }

    normalized_services: dict[str, dict[str, object]] = {}
    for name, expected_type in (
        ("/get_planning_scene", "moveit_msgs/srv/GetPlanningScene"),
        ("/apply_planning_scene", "moveit_msgs/srv/ApplyPlanningScene"),
    ):
        entry = _as_mapping(services.get(name))
        if entry.get("type") != expected_type:
            raise ValueError(f"observed service {name} has wrong type {entry.get('type')!r}")
        if not _qos_exact(entry.get("requested_qos"), JOURNAL_SERVICE_QOS):
            raise ValueError(f"observed service {name} requested QoS must be RELIABLE/VOLATILE")
        if not _qos_exact(entry.get("offered_qos"), JOURNAL_SERVICE_QOS):
            raise ValueError(f"observed service {name} offered QoS must be RELIABLE/VOLATILE")
        servers = _validate_endpoint_entries(f"observed service {name} servers", entry.get("servers"))
        clients = _validate_endpoint_entries(f"observed service {name} clients", entry.get("clients"))
        if not any(endpoint["node"] == OPERATOR_NODE for endpoint in clients):
            raise ValueError(f"observed service {name} must be called by {OPERATOR_NODE}")
        normalized_services[name] = {
            "type": expected_type,
            "requested_qos": dict(JOURNAL_SERVICE_QOS),
            "offered_qos": dict(JOURNAL_SERVICE_QOS),
            "servers": servers,
            "clients": clients,
        }

    return {
        "node_name": OPERATOR_NODE,
        "namespace": OPERATOR_NODE_NAMESPACE,
        "remap_table": {},
        "topics": normalized_topics,
        "services": normalized_services,
    }


def build_journal_graph_projection(
    *,
    fixture_payload: str,
    observed_graph: Mapping[str, object],
) -> dict[str, object]:
    """Build the Task-3 graph projection from an explicit observed graph.

    ``observed_graph`` must represent the active attempt's real endpoint
    observations (never fabricated constants): the recorder identity
    ``node_name="/tinker_integrated_gate_executor"``, ``namespace="/"``,
    ``remap_table={}``; the exact topic/service key sets, types and uppercase
    QoS; the recorder among all required subscribers/clients; exactly one
    ``/fixture_planning_scene`` fixture publisher.  The exact canonical compact
    fixture payload string is injected as ``payload`` (parsed data separate).
    Fails closed on missing/extra interfaces, wrong type/QoS/source/cardinality,
    or an absent recorder subscriber/client.
    """
    projection = _validate_observed_graph(observed_graph)
    projection = copy.deepcopy(projection)
    # Inject the exact canonical compact fixture payload into the fixture topic
    # entry; parsed data is retained separately by the journal validator.
    projection["topics"][FIXTURE_TOPIC]["payload"] = str(fixture_payload)
    return projection


def stage_c_dispatch(
    scenario_id: str,
    *,
    scenario: Mapping[str, object],
) -> dict[str, object]:
    """Validate a Stage-C plan-only scenario and return a ROS-free dispatch spec.

    Requires the exact scenario id, ``integrated.stage == "C"``,
    ``integrated.execution_profile == "sim_ompl"``, and
    ``integrated.acceptance.polarity == "plan-only"``.  Returns a spec with the
    goal ``kind`` (joint/pose/blocked), the expected diagnostic polarity
    (success/non-success), the seven-joint ``Q_OUTBOUND`` for joint, and the
    declared target-object pose for pose/blocked.
    """
    if not isinstance(scenario_id, str) or not scenario_id:
        raise ValueError("scenario_id must be a nonempty string")
    if scenario.get("id") != scenario_id:
        raise ValueError("scenario_id does not match the executor scenario mapping")
    integrated = _as_mapping(scenario.get("integrated"))
    if integrated.get("stage") != "C":
        raise ValueError("scenario integrated.stage must be C for Gate C")
    if integrated.get("execution_profile") != INTEGRATED_EXECUTION_PROFILE:
        raise ValueError("scenario integrated.execution_profile must be sim_ompl")
    acceptance = _as_mapping(integrated.get("acceptance"))
    if acceptance.get("polarity") != "plan-only":
        raise ValueError("scenario integrated.acceptance.polarity must be plan-only")
    if scenario_id not in STAGE_C_SCENARIOS:
        raise ValueError(f"scenario {scenario_id} is not one of the Stage-C plan-only scenarios")

    declaration = _as_mapping(scenario.get("planning_scene_declaration"))
    if not declaration:
        declaration = _as_mapping(scenario.get("planning_scene"))
    objects = declaration.get("objects")
    if not isinstance(objects, (list, tuple)):
        raise ValueError("scenario planning_scene has no objects list")
    target_source_id = declaration.get("target_source_id")
    target = next(
        (record for record in objects if isinstance(record, Mapping) and record.get("id") == target_source_id),
        None,
    )
    if target is None:
        raise ValueError("scenario declaration has no target object matching target_source_id")
    pose = _as_mapping(target.get("pose"))
    xyz = pose.get("xyz")
    quaternion = pose.get("quaternion_xyzw")
    if not (
        isinstance(xyz, (list, tuple))
        and len(xyz) == 3
        and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in xyz)
    ):
        raise ValueError("scenario target pose xyz must be three finite values")
    if not (
        isinstance(quaternion, (list, tuple))
        and len(quaternion) == 4
        and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in quaternion)
    ):
        raise ValueError("scenario target pose quaternion_xyzw must be four finite values")

    kind = {
        "qualification-moveit-plan-joint": "joint",
        "qualification-moveit-plan-pose": "pose",
        "qualification-moveit-plan-blocked": "blocked",
    }[scenario_id]
    return {
        "scenario_id": scenario_id,
        "kind": kind,
        "expectation": "non-success" if kind == "blocked" else "success",
        "joints": list(Q_OUTBOUND) if kind == "joint" else None,
        "target_pose": {
            "xyz": [float(value) for value in xyz],
            "quaternion_xyzw": [float(value) for value in quaternion],
        },
    }


def _normalize_goal_uuid(raw: object) -> str | None:
    """Normalize a goal-handle UUID to lowercase 16-byte hex, else ``None``.

    rclpy ``ClientGoalHandle.goal_id`` is the 16-byte ``unique_identifier_msgs``
    bytes; a normalized lowercase hex string is the canonical qualification
    identity.  Malformed/absent UUIDs never pass a split-path contract.
    """
    if isinstance(raw, bool):
        return None
    try:
        if isinstance(raw, (bytes, bytearray)):
            return _uuid.UUID(bytes=bytes(raw)).hex
        if isinstance(raw, str):
            return _uuid.UUID(str(raw)).hex
        candidate = getattr(raw, "uuid", None)
        if isinstance(candidate, bytes):
            return _uuid.UUID(bytes=bytes(candidate)).hex
        if isinstance(candidate, str):
            return _uuid.UUID(str(candidate)).hex
        return None
    except (ValueError, AttributeError, TypeError):
        return None


def _valid_goal_uuid(value: object) -> bool:
    return isinstance(value, str) and len(value) == 32 and all(
        character in "0123456789abcdef" for character in value
    )


def _execute_status_name(status: object) -> str:
    """Map an ``action_msgs/msg/GoalStatus`` int to the canonical string.

    Only the three ExecuteTrajectory terminal statuses are accepted:
    SUCCEEDED=4, CANCELED=5, ABORTED=6.  Unknown/malformed statuses raise so a
    caller can never pass on an unapproved result.
    """
    if isinstance(status, bool) or not isinstance(status, int):
        raise ValueError(f"execute status must be an integer, found {status!r}")
    name = _EXECUTE_STATUS_NAMES.get(status)
    if name is None:
        raise ValueError(f"unknown ExecuteTrajectory terminal status: {status}")
    return name


def _arm_velocity_within_limit(frames: object, limit: object) -> bool:
    """True when every measured frame's arm-joint velocities are bounded.

    Each *frame* is a sequence of absolute arm-joint (``joint1..joint7``)
    velocities in rad/s.  *limit* is the configured
    ``safety_stop_velocity_rad_s``.  A frame with non-finite velocity, the wrong
    arity, or a magnitude above the limit fails the effective-stop predicate.
    """
    try:
        velocity_limit = float(limit)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(velocity_limit) or velocity_limit < 0.0:
        return False
    if not isinstance(frames, Sequence) or isinstance(frames, (str, bytes)):
        return False
    for frame in frames:
        if not isinstance(frame, Sequence) or isinstance(frame, (str, bytes)):
            return False
        if len(frame) != 7:
            return False
        for value in frame:
            try:
                magnitude = abs(float(value))
            except (TypeError, ValueError):
                return False
            if not math.isfinite(magnitude) or magnitude > velocity_limit:
                return False
    return True


def derive_retreat_target_pose(
    source_pose: Mapping[str, object],
    *,
    distance_m: object,
    axis: str,
) -> dict[str, object]:
    """Derive the deterministic ``base_link`` retreat target from a TCP pose.

    Moves the observed TCP pose exactly *distance_m* along the declared
    *axis* (``+z`` for the top-down grasp geometry) preserving orientation.
    Fails closed on missing/wrong-frame/non-finite/invalid-quaternion input so
    a missing or stale provider can never produce a goal.
    """
    xyz = source_pose.get("xyz")
    quaternion = source_pose.get("quaternion_xyzw")
    frame_id = source_pose.get("frame_id")
    if frame_id not in (None, "base_link"):
        raise ValueError("retreat source pose must use base_link")
    if not (
        isinstance(xyz, (list, tuple))
        and len(xyz) == 3
        and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in xyz)
    ):
        raise ValueError("retreat source pose xyz must be three finite values")
    if not (
        isinstance(quaternion, (list, tuple))
        and len(quaternion) == 4
        and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in quaternion)
    ):
        raise ValueError("retreat source pose quaternion_xyzw must be four finite values")
    norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
    if abs(norm - 1.0) > 1.0e-3:
        raise ValueError("retreat source quaternion must be normalized within 1e-3")
    try:
        distance = float(distance_m)
    except (TypeError, ValueError):
        raise ValueError("retreat distance must be a finite number")
    if not math.isfinite(distance) or distance < 0.0:
        raise ValueError("retreat distance must be a finite non-negative number")
    if axis not in ("+x", "-x", "+y", "-y", "+z", "-z"):
        raise ValueError(f"retreat axis must be one of +x/-x/+y/-y/+z/-z, found {axis!r}")
    offset = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0), "-y": (0, -1, 0),
              "+z": (0, 0, 1), "-z": (0, 0, -1)}[axis]
    return {
        "frame_id": "base_link",
        "xyz": [
            float(xyz[0]) + distance * offset[0],
            float(xyz[1]) + distance * offset[1],
            float(xyz[2]) + distance * offset[2],
        ],
        "quaternion_xyzw": [float(value) for value in quaternion],
    }


def run_pick_place_negative(scenario_id: str) -> dict[str, object]:
    """Fail-closed Gate-E negative stub for the two cancellation identities.

    Preserves the brief's observable fake contract for later replacement without
    pre-empting Task 6/Gate E: always records ``release_stage_started=false``,
    ``released=false``, and zero ROS goals sent.  Rejects all other identities.
    """
    if not isinstance(scenario_id, str) or not scenario_id:
        raise ValueError("scenario_id must be a nonempty string")
    events = STAGE_E_CANCEL_NEGATIVES.get(scenario_id)
    if events is None:
        raise ValueError(f"unsupported Gate-E negative scenario: {scenario_id}")
    return {
        "scenario_id": scenario_id,
        "events": list(events),
        "release_stage_started": False,
        "released": False,
        "goals_sent": 0,
        "diagnostic_only": True,
        "isaac_joint_commands_published": False,
    }


def stage_d_dispatch(
    scenario_id: str,
    *,
    scenario: Mapping[str, object],
) -> dict[str, object]:
    """Validate a Stage-D scenario and return a ROS-free dispatch spec.

    Requires the exact scenario id, ``integrated.stage == "D"``,
    ``integrated.execution_profile == "sim_ompl"``, the exact declared polarity
    (``positive`` / ``cancel`` / ``safety``), the exact configured
    ``expected_physical`` list for that scenario, and
    ``forbidden_endpoints == ["/isaac_joint_commands"]``.  Unknown, C/E-stage,
    malformed, or mutated scenarios fail closed before any goal is created or
    sent.  Returns the D handler ``kind``, the declared polarity/expected
    physical list, the seven-joint ``Q_OUTBOUND`` for execute-joint, and the
    declared ``sim_fixture/public_target`` pose for execute-pose.
    """
    if not isinstance(scenario_id, str) or not scenario_id:
        raise ValueError("scenario_id must be a nonempty string")
    if scenario.get("id") != scenario_id:
        raise ValueError("scenario_id does not match the executor scenario mapping")
    integrated = _as_mapping(scenario.get("integrated"))
    if integrated.get("stage") != "D":
        raise ValueError("scenario integrated.stage must be D for Gate D")
    if integrated.get("execution_profile") != INTEGRATED_EXECUTION_PROFILE:
        raise ValueError("scenario integrated.execution_profile must be sim_ompl")
    if scenario_id not in STAGE_D_SCENARIOS:
        raise ValueError(f"scenario {scenario_id} is not one of the Stage-D scenarios")

    kind = STAGE_D_KIND[scenario_id]
    expected_polarity = STAGE_D_EXPECTED_POLARITY[scenario_id]
    acceptance = _as_mapping(integrated.get("acceptance"))
    declared_polarity = acceptance.get("polarity")
    if declared_polarity != expected_polarity:
        raise ValueError(
            f"scenario integrated.acceptance.polarity must be {expected_polarity!r}, "
            f"found {declared_polarity!r}"
        )
    declared_physical = integrated.get("expected_physical")
    expected_physical = list(STAGE_D_EXPECTED_PHYSICAL[scenario_id])
    if not (
        isinstance(declared_physical, (list, tuple))
        and not isinstance(declared_physical, (str, bytes))
        and list(declared_physical) == expected_physical
    ):
        raise ValueError(
            f"scenario integrated.expected_physical must be exactly {expected_physical}, "
            f"found {declared_physical!r}"
        )
    forbidden = integrated.get("forbidden_endpoints")
    if not (
        isinstance(forbidden, (list, tuple))
        and not isinstance(forbidden, (str, bytes))
        and list(forbidden) == [ISAAC_COMMAND_TOPIC]
    ):
        raise ValueError(
            f"scenario integrated.forbidden_endpoints must be exactly "
            f"[{ISAAC_COMMAND_TOPIC}], found {forbidden!r}"
        )

    target_pose: dict[str, object] | None = None
    if kind == "execute-pose":
        declaration = _as_mapping(
            scenario.get("planning_scene_declaration") or scenario.get("planning_scene")
        )
        objects = declaration.get("objects")
        if not isinstance(objects, (list, tuple)):
            raise ValueError("scenario planning_scene has no objects list")
        target_source_id = declaration.get("target_source_id")
        target = next(
            (
                record
                for record in objects
                if isinstance(record, Mapping) and record.get("id") == target_source_id
            ),
            None,
        )
        if target is None:
            raise ValueError("scenario declaration has no target object matching target_source_id")
        pose = _as_mapping(target.get("pose"))
        xyz = pose.get("xyz")
        quaternion = pose.get("quaternion_xyzw")
        if not (
            isinstance(xyz, (list, tuple))
            and len(xyz) == 3
            and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in xyz)
        ):
            raise ValueError("scenario target pose xyz must be three finite values")
        if not (
            isinstance(quaternion, (list, tuple))
            and len(quaternion) == 4
            and all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in quaternion)
        ):
            raise ValueError("scenario target pose quaternion_xyzw must be four finite values")
        norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
        if abs(norm - 1.0) > 1.0e-3:
            raise ValueError("scenario target pose quaternion must be normalized within 1e-3")
        target_pose = {
            "xyz": [float(value) for value in xyz],
            "quaternion_xyzw": [float(value) for value in quaternion],
        }

    return {
        "scenario_id": scenario_id,
        "kind": kind,
        "polarity": expected_polarity,
        "expected_physical": expected_physical,
        "joints": list(Q_OUTBOUND) if kind == "execute-joint" else None,
        "target_pose": target_pose,
        "forbidden_endpoints": [ISAAC_COMMAND_TOPIC],
    }


def _d_stage_event_order(scenario: Mapping[str, object]) -> tuple[str, ...]:
    """Return the scenario-specific D journal event order, else Gate C's."""
    integrated = _as_mapping(scenario.get("integrated"))
    if integrated.get("stage") != "D":
        return GATE_C_REQUIRED_EVENT_ORDER
    kind = STAGE_D_KIND.get(str(scenario.get("id", "")))
    if kind is None:
        return GATE_C_REQUIRED_EVENT_ORDER
    return STAGE_D_REQUIRED_EVENT_ORDER.get(kind, GATE_C_REQUIRED_EVENT_ORDER)


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
    """Build a Place goal.

    Qualification-only constraint (Task 4 plan): ``target_point.header.frame_id``
    must be ``base_link``.  The production ``Place`` server accepts an arbitrary
    frame and TF-transforms it, but Task 4 deliberately restricts to ``base_link``
    to keep the deterministic qualification geometry frame-local; later execute
    gates may broaden this.
    """
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


def build_execute_trajectory_goal(planned_trajectory):
    """Construct exactly one ``moveit_msgs/action/ExecuteTrajectory.Goal``.

    The returned ``planned_trajectory`` is assigned directly to
    ``goal.trajectory`` without mutation/replanning/round-trip reconstruction;
    the caller records the canonical ROS-serialized trajectory digest before and
    after assignment and requires them to match.  A ``None`` or empty trajectory
    fails closed.
    """
    from moveit_msgs.action import ExecuteTrajectory

    if planned_trajectory is None:
        raise ValueError("ExecuteTrajectory goal requires a non-empty planned trajectory")
    points = getattr(
        getattr(planned_trajectory, "joint_trajectory", None), "points", None
    )
    if not isinstance(points, (list, tuple)) or not points:
        raise ValueError("ExecuteTrajectory goal requires a non-empty planned trajectory")
    goal = ExecuteTrajectory.Goal()
    goal.trajectory = planned_trajectory
    return goal


def build_gripper_goal(position, *, max_effort):
    """Construct one native ``control_msgs/action/GripperCommand`` goal.

    Task-5 qualification constants: open position ``0.0``, close position
    ``0.85``, max effort ``10.0``.  A non-finite position/effort fails closed.
    """
    from control_msgs.action import GripperCommand

    try:
        position_value = float(position)
        effort_value = float(max_effort)
    except (TypeError, ValueError):
        raise ValueError("gripper position and max effort must be finite numbers")
    if not math.isfinite(position_value) or not math.isfinite(effort_value):
        raise ValueError("gripper position and max effort must be finite numbers")
    goal = GripperCommand.Goal()
    goal.command.position = position_value
    goal.command.max_effort = effort_value
    return goal


def build_cartesian_move_goal(target_pose, *, env_points=None):
    """Construct one ``tinker_arm_msgs/action/CartesianMove`` goal.

    ``target_pose`` is a ``geometry_msgs/msg/Pose`` in ``base_link``.  The goal
    carries the collision-aware production path (``env_points`` PointCloud2 when
    supplied).  A non-``base_link`` header (when a PoseStamped is supplied) or a
    non-finite target fails closed.
    """
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import PointCloud2
    from tinker_arm_msgs.action import CartesianMove

    if target_pose is None:
        raise ValueError("CartesianMove goal requires a target pose")
    pose = getattr(target_pose, "pose", target_pose)
    if getattr(pose, "position", None) is None:
        raise ValueError("CartesianMove goal requires a real Pose target")
    _validate_quaternion(pose.orientation)
    header_frame = getattr(target_pose, "header", None)
    if header_frame is not None and getattr(header_frame, "frame_id", "base_link") != "base_link":
        raise ValueError("cartesian retreat target must use base_link")
    goal = CartesianMove.Goal()
    if isinstance(target_pose, Pose):
        goal.target_pose = target_pose
    else:
        goal.target_pose = pose
    # The generated CartesianMove goal requires a PointCloud2 (never None) for
    # the collision-aware env_points field; an empty cloud is the neutral
    # default when no environment cloud is supplied.
    goal.env_points = env_points if env_points is not None else PointCloud2()
    return goal


# ---------------------------------------------------------------------------
# Live ROS-lazy executor (imported/instantiated only under sourced Humble)
# ---------------------------------------------------------------------------

_ROS_IMPORTS: dict[str, Any] = {}


def _load_ros() -> dict[str, Any]:
    """Import ROS only at live execution time (Humble CPython 3.10)."""
    if _ROS_IMPORTS:
        return _ROS_IMPORTS
    # F1.1: require or set the fast-DDS RMW before importing rclpy.
    configured_rmw = os.environ.get("RMW_IMPLEMENTATION")
    if configured_rmw is not None and configured_rmw != RMW_IMPLEMENTATION:
        raise RuntimeError(
            f"IntegratedGateExecutor requires RMW_IMPLEMENTATION={RMW_IMPLEMENTATION}; "
            f"found {configured_rmw}"
        )
    if configured_rmw is None:
        os.environ["RMW_IMPLEMENTATION"] = RMW_IMPLEMENTATION
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from controller_manager_msgs.srv import (
        ConfigureController,
        ListControllers,
        LoadController,
        SwitchController,
    )
    from moveit_msgs.action import ExecuteTrajectory, MoveGroup
    from moveit_msgs.msg import PlanningScene
    from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPlanningScene, GetStateValidity
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from tinker_arm_msgs.action import CartesianMove, Fold, JointMove, Pick, Place
    from tinker_arm_msgs.srv import ArmJointService

    if rclpy.get_rmw_implementation_identifier() != RMW_IMPLEMENTATION:
        raise RuntimeError(
            "rclpy loaded RMW "
            f"{rclpy.get_rmw_implementation_identifier()!r}; expected {RMW_IMPLEMENTATION}"
        )
    _ROS_IMPORTS.update(locals())
    return _ROS_IMPORTS


def _operator_qos(ros: Mapping[str, Any]) -> Any:
    return ros["QoSProfile"](
        depth=1,
        reliability=ros["ReliabilityPolicy"].RELIABLE,
        durability=ros["DurabilityPolicy"].TRANSIENT_LOCAL,
    )


def _planning_scene_qos(ros: Mapping[str, Any]) -> Any:
    """Stock MoveIt2 Humble PlanningScene contract: RELIABLE/VOLATILE/depth 100."""
    return ros["QoSProfile"](
        depth=100,
        reliability=ros["ReliabilityPolicy"].RELIABLE,
        durability=ros["DurabilityPolicy"].VOLATILE,
    )


def _fixture_qos(ros: Mapping[str, Any]) -> Any:
    """Fixture/safety/operator status contract: RELIABLE/TRANSIENT_LOCAL/depth 1."""
    return ros["QoSProfile"](
        depth=1,
        reliability=ros["ReliabilityPolicy"].RELIABLE,
        durability=ros["DurabilityPolicy"].TRANSIENT_LOCAL,
    )


def _joint_state_qos(ros: Mapping[str, Any]) -> Any:
    return ros["QoSProfile"](
        depth=10,
        reliability=ros["ReliabilityPolicy"].RELIABLE,
        durability=ros["DurabilityPolicy"].VOLATILE,
    )


def _fjt_status_qos(ros: Mapping[str, Any]) -> Any:
    """Stock Humble action status QoS (``rcl_action/default_qos.h``):
    RELIABLE / TRANSIENT_LOCAL / depth 1."""
    return ros["QoSProfile"](
        depth=1,
        reliability=ros["ReliabilityPolicy"].RELIABLE,
        durability=ros["DurabilityPolicy"].TRANSIENT_LOCAL,
    )


def _atomic_write_json(value: object, path: Path) -> None:
    """Write *value* canonically through temp-file + fsync + replace + dir fsync.

    Mirrors the strongest repository durability pattern (Task 3 journal) so a
    pass claim is never exposed before the parent directory entry is durable.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        try:
            dir_fd = os.open(path.parent, os.O_RDONLY)
        except OSError:
            pass
        else:
            try:
                os.fsync(dir_fd)
            finally:
                os.close(dir_fd)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class IntegratedGateExecutor:
    """Live Gate-C plan-only executor node.

    Runs under sourced Humble Python 3.10 only.  Creates a private
    ``rclpy.context.Context`` per executor, initializes it with the exact
    requested ``ros_domain_id``, and creates the node
    ``/tinker_integrated_gate_executor`` (basename ``tinker_integrated_gate_executor``,
    namespace ``/``, ``use_global_arguments=False``).  It creates typed
    action/service clients for every required endpoint, the operator publisher
    ``/sim/safety/operator`` (``std_msgs/msg/Bool``, reliable/transient-local/
    depth 1), real ``moveit_msgs/msg/PlanningScene`` subscriptions, an owned
    ``PlanningSceneJournal``, and the Gate C plan-only flow.

    Providers (Task 7/orchestration supplies them later; Task 4 defines the
    contracts and tests them):

    - ``join_key_provider`` -> exact raw/evaluator ``(frame_index, timestamp)``;
    - ``readiness_snapshot_provider`` -> the complete observed readiness
      snapshot evaluated by :func:`evaluate_executor_readiness`;
    - ``graph_observation_provider`` -> the observed graph for journal
      finalization.

    Gate C never sends ``/execute_trajectory`` and never publishes
    ``/isaac_joint_commands``.  Plan-only evidence records remain
    ``diagnostic_only = true`` and never claim execution.
    """

    def __init__(
        self,
        *,
        scenario: Mapping[str, object],
        attempt_dir: Path,
        config: Mapping[str, object],
        ros_domain_id: int | str,
        journal: Any = None,
        join_key_provider: Callable[[], tuple[int, float]] | None = None,
        readiness_snapshot_provider: Callable[[], Mapping[str, object]] | None = None,
        graph_observation_provider: Callable[[], Mapping[str, object]] | None = None,
    ) -> None:
        domain_id = self._validate_domain(ros_domain_id)
        if not isinstance(scenario, Mapping):
            raise ValueError("scenario must be a mapping")
        if not isinstance(config, Mapping):
            raise ValueError("config must be a mapping")
        self.scenario = scenario
        self.config = config
        self.attempt_dir = Path(attempt_dir).resolve()
        self.attempt_dir.mkdir(parents=True, exist_ok=True)
        self._reject_stale_attempt_evidence()

        declaration = _as_mapping(
            scenario.get("planning_scene_declaration") or scenario.get("planning_scene")
        )
        self.fixture_revision = str(declaration.get("revision", ""))
        if not self.fixture_revision:
            raise ValueError("scenario planning_scene has no revision")

        # Providers.
        self.join_key_provider = join_key_provider
        self.readiness_snapshot_provider = readiness_snapshot_provider
        self.graph_observation_provider = graph_observation_provider

        self.ros = _load_ros()
        self._context_initialized = False
        self.context = self.ros["Context"]()
        self._init_rclpy(domain_id)
        self._context_initialized = True
        self.node = self.ros["Node"](
            NODE_BASENAME,
            namespace=OPERATOR_NODE_NAMESPACE,
            cli_args=[],
            context=self.context,
            use_global_arguments=False,
        )
        self._spinner = self.ros["SingleThreadedExecutor"](context=self.context)
        self._spinner.add_node(self.node)
        self.operator_publisher = self.node.create_publisher(
            self.ros["Bool"], OPERATOR_TOPIC, _operator_qos(self.ros)
        )
        self._create_clients()
        self._create_subscriptions()

        # Journal ownership (F1.3): default to a real PlanningSceneJournal.
        if journal is not None:
            self.journal = journal
        else:
            from planning_scene_journal import PlanningSceneJournal, load_model_touch_contract

            contract = load_model_touch_contract()
            self.journal = PlanningSceneJournal(
                fixture_revision=self.fixture_revision,
                task_namespace=TASK_NAMESPACE,
                target_object_id=TARGET_OBJECT_ID,
                expected_attach_link=contract["link_tcp"],
                expected_touch_links=contract["touch_links"],
                required_event_order=_d_stage_event_order(self.scenario),
                forbidden_events=D_FORBIDDEN_EVENTS,
                jsonl_path=self.attempt_dir / "planning-scene.jsonl",
            )

        self._latest_planning_scene: dict[str, object] | None = None
        self._planning_scene_invalid = False
        self._scene_invalid_sequence: int | None = None
        self._fixture_payload: str | None = None
        self._fixture_payload_invalid = False
        self._latest_joint_state: Any = None
        self._latest_safety_stop: Any = None
        self._joint_velocity_frames: list[list[float]] = []
        self._fjt_status_cache: list[dict[str, object]] = []
        self._scene_sequence = 0
        self._last_join_key: tuple[int, float] | None = None

    # -- construction helpers ----------------------------------------------

    @staticmethod
    def _validate_domain(ros_domain_id: int | str) -> int:
        if isinstance(ros_domain_id, bool):
            raise ValueError("ROS_DOMAIN_ID must be an integer, not a boolean")
        try:
            domain_id = int(ros_domain_id)
        except (TypeError, ValueError):
            raise ValueError("ROS_DOMAIN_ID must be an integer in [0, 232]")
        if domain_id < 0 or domain_id > 232:
            raise ValueError("ROS_DOMAIN_ID must be an integer in [0, 232]")
        return domain_id

    def _init_rclpy(self, domain_id: int) -> None:
        self.ros["rclpy"].init(args=[], context=self.context, domain_id=domain_id)

    def _reject_stale_attempt_evidence(self) -> None:
        jsonl = self.attempt_dir / "planning-scene.jsonl"
        if jsonl.exists() and jsonl.stat().st_size > 0:
            raise ValueError(f"planning-scene.jsonl already contains records: {jsonl}")
        for name in ARTIFACT_JSONL_FILES:
            path = self.attempt_dir / name
            if path.exists() and path.stat().st_size > 0:
                raise ValueError(f"{name} already contains records: {path}")

    def _create_clients(self) -> None:
        ros = self.ros
        self._action_clients: dict[str, Any] = {}
        for name, action_type in REQUIRED_ACTIONS.items():
            action_class_name = action_type.split("/")[-1]
            action_class = ros.get(action_class_name)
            if action_class is None:
                raise RuntimeError(
                    f"missing imported action class {action_class_name} for {name}"
                )
            self._action_clients[name] = ros["ActionClient"](
                self.node, action_class, name
            )
        self._service_clients: dict[str, Any] = {}
        for name, service_type in REQUIRED_SERVICES.items():
            message_type = _service_type_to_ros(service_type, ros)
            self._service_clients[name] = self.node.create_client(message_type, name)
        if len(self._action_clients) != len(REQUIRED_ACTIONS):
            raise RuntimeError("not all required action clients were created")
        if len(self._service_clients) != len(REQUIRED_SERVICES):
            raise RuntimeError("not all required service clients were created")

    def _create_subscriptions(self) -> None:
        ros = self.ros
        self.node.create_subscription(
            ros["PlanningScene"],
            PLANNING_SCENE_TOPIC,
            self._make_scene_callback(PLANNING_SCENE_TOPIC),
            _planning_scene_qos(ros),
        )
        self.node.create_subscription(
            ros["PlanningScene"],
            MONITORED_PLANNING_SCENE_TOPIC,
            self._make_scene_callback(MONITORED_PLANNING_SCENE_TOPIC),
            _planning_scene_qos(ros),
        )
        self.node.create_subscription(
            ros["String"],
            FIXTURE_TOPIC,
            self._on_fixture_payload,
            _fixture_qos(ros),
        )
        self.node.create_subscription(
            ros["JointState"],
            JOINT_STATES_TOPIC,
            self._on_joint_state,
            _joint_state_qos(ros),
        )
        self.node.create_subscription(
            ros["Bool"],
            SAFETY_STOP_TOPIC,
            self._on_safety_stop,
            _fixture_qos(ros),
        )
        # D-stage FJT status observation.  ROS 2 actions do NOT publish an
        # ``_action/goal`` topic; the goal travels over the ``send_goal``
        # service.  Only the real ``_action/status`` subscription exists, with
        # the stock Humble action status QoS (RELIABLE/TRANSIENT_LOCAL/depth 1).
        # This subscription stays outside the Task-3 three-topic/two-service
        # journal graph projection.
        self.node.create_subscription(
            ros["GoalStatusArray"],
            FJT_STATUS_TOPIC,
            self._on_fjt_status,
            _fjt_status_qos(ros),
        )

    def _make_scene_callback(self, source: str):
        def callback(message: Any) -> None:
            try:
                normalized = self._normalize_planning_scene(message, source=source)
            except _SCENE_CALLBACK_EXCEPTIONS:
                # F3.2: a transient malformed message latches fail-closed but
                # never erases the last valid cached scene; a later valid
                # callback clears the latch.
                self._planning_scene_invalid = True
                self._scene_invalid_sequence = self._scene_sequence
            else:
                self._latest_planning_scene = normalized
                self._planning_scene_invalid = False
                self._scene_invalid_sequence = None

        return callback

    def _on_fixture_payload(self, message: Any) -> None:
        payload = str(getattr(message, "data", ""))
        try:
            self._validate_canonical_fixture_payload(payload)
        except ValueError:
            self._fixture_payload_invalid = True
            return
        self._fixture_payload_invalid = False
        self._fixture_payload = payload

    def _on_joint_state(self, message: Any) -> None:
        self._latest_joint_state = message
        frame = self._arm_velocity_frame(message)
        if frame is not None:
            self._joint_velocity_frames.append(frame)
            limit = int(self._thresholds().get("safety_stop_frames", 5))
            if limit < 1:
                limit = 1
            if len(self._joint_velocity_frames) > limit:
                del self._joint_velocity_frames[: len(self._joint_velocity_frames) - limit]

    @staticmethod
    def _arm_velocity_frame(message: Any) -> list[float] | None:
        """Extract the seven arm-joint absolute velocities from a JointState."""
        names = list(getattr(message, "name", ()))
        velocities = list(getattr(message, "velocity", ()))
        by_name = dict(zip(names, velocities))
        try:
            return [float(by_name[f"joint{index}"]) for index in range(1, 8)]
        except (KeyError, TypeError, ValueError):
            return None

    def _on_safety_stop(self, message: Any) -> None:
        self._latest_safety_stop = message

    def _on_fjt_status(self, message: Any) -> None:
        """Cache bounded, well-formed ``(goal_uuid, status)`` FJT status entries.

        Only well-formed entries with a valid lowercase 16-byte goal UUID and a
        strict integer status are retained; malformed entries are dropped.  The
        cache is bounded (the newest ``FJT_STATUS_CACHE_LIMIT`` entries) so a
        live controller cannot grow memory without bound.
        """
        status_list = getattr(message, "status_list", None)
        if not isinstance(status_list, (list, tuple)):
            return
        for status_entry in status_list:
            goal_info = getattr(status_entry, "goal_info", None)
            goal_id = getattr(goal_info, "goal_id", None)
            goal_uuid = _normalize_goal_uuid(goal_id)
            status = getattr(status_entry, "status", None)
            if goal_uuid is None or isinstance(status, bool) or not isinstance(status, int):
                continue
            self._fjt_status_cache.append(
                {"goal_uuid": goal_uuid, "status": int(status), "received": float(time.monotonic())}
            )
        if len(self._fjt_status_cache) > FJT_STATUS_CACHE_LIMIT:
            del self._fjt_status_cache[: len(self._fjt_status_cache) - FJT_STATUS_CACHE_LIMIT]

    def _fjt_status_entries(self) -> list[dict[str, object]]:
        return list(self._fjt_status_cache)

    def _newest_fjt_status(self) -> dict[str, object] | None:
        return self._fjt_status_cache[-1] if self._fjt_status_cache else None

    def _normalize_planning_scene(self, message: Any, *, source: str) -> dict[str, object]:
        ros = self.ros
        self._scene_sequence += 1
        owned_ids = [str(collision_object.id) for collision_object in message.world.collision_objects]
        attached = list(message.robot_state.attached_collision_objects)
        attached_ids = [str(attached_object.object.id) for attached_object in attached]
        attached_links = {
            str(attached_object.object.id): str(attached_object.link_name)
            for attached_object in attached
        }
        touch_links = {
            str(attached_object.object.id): [str(link) for link in attached_object.touch_links]
            for attached_object in attached
        }
        fixture_digest, fixture_geometry = self._fixture_geometry_projection(message)
        return {
            "scene_sequence": self._scene_sequence,
            "scene_timestamp": float(time.monotonic()),
            "owned_ids": owned_ids,
            "attached_ids": attached_ids,
            "attached_links": attached_links,
            "touch_links": touch_links,
            "fixture_revision": self.fixture_revision,
            "fixture_geometry_digest": fixture_digest,
            "fixture_geometry": fixture_geometry,
            "scene_revision_digest": self._digest(ros["serialize_message"](message)),
            "acm_digest": self._digest(ros["serialize_message"](message.allowed_collision_matrix)),
            "robot_state_digest": self._digest(ros["serialize_message"](message.robot_state)),
            "source": source,
        }

    def _fixture_declaration(self) -> Mapping[str, object]:
        return _as_mapping(
            self.scenario.get("planning_scene_declaration")
            or self.scenario.get("planning_scene")
        )

    def _expected_fixture_geometry_digest(self) -> str | None:
        declaration = self._fixture_declaration()
        if not declaration:
            return None
        try:
            return expected_fixture_geometry_digest(declaration)
        except Exception:
            return None

    def _fixture_geometry_projection(
        self, message: Any
    ) -> tuple[str, list[dict[str, object]]]:
        """F3.3: canonical fixture-owned geometry projection of the received scene.

        Extracts, in declared owned-ID order, the canonical bridge geometry
        descriptor for every scenario-owned collision object present in the
        message.  A missing or geometry-less owned object yields an empty
        descriptor so the digest cannot accidentally match; the projection never
        includes foreign objects or unrelated full-scene serialization (robot
        state / ACM).  Returns ``(digest, ordered_descriptors)``.
        """
        expected_ids = list(fixture_owned_ids(self._fixture_declaration()))
        by_id: dict[str, dict[str, object]] = {}
        for collision_object in message.world.collision_objects:
            object_id = str(getattr(collision_object, "id", ""))
            if object_id not in expected_ids:
                continue
            try:
                by_id[object_id] = dict(readback_geometry(collision_object))
            except Exception:
                # A geometry-less/malformed owned object can never match the
                # declared projection; record an empty descriptor so the digest
                # differs instead of failing normalization.
                by_id[object_id] = {
                    "id": object_id,
                    "frame_id": "",
                    "primitives": [],
                    "primitive_poses": [],
                    "meshes": [],
                    "mesh_poses": [],
                }
        ordered = [by_id[object_id] for object_id in expected_ids if object_id in by_id]
        return geometry_signature_sha256(ordered), ordered

    @staticmethod
    def _digest(data: bytes) -> str:
        return hashlib.sha256(bytes(data)).hexdigest()

    @staticmethod
    def _validate_canonical_fixture_payload(payload: str) -> dict[str, object]:
        if not isinstance(payload, str) or not payload:
            raise ValueError("fixture payload must be a nonempty canonical compact JSON string")
        try:
            parsed = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise ValueError("fixture payload must be parseable JSON") from exc
        if not isinstance(parsed, dict):
            raise ValueError("fixture payload must be a JSON object")
        canonical = json.dumps(parsed, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
        if canonical != payload:
            raise ValueError("fixture payload must be the canonical compact fixture-status encoding")
        if set(parsed) != FIXTURE_STATUS_KEYS:
            raise ValueError("fixture payload must be the exact canonical fixture-status field set")
        return parsed

    # -- operator publisher -------------------------------------------------

    def publish_operator(self, value: bool) -> None:
        if value not in (False, True):
            raise ValueError("operator payload allowlist is [False, True]")
        message = self.ros["Bool"]()
        message.data = bool(value)
        self.operator_publisher.publish(message)

    # -- providers / journal scene ------------------------------------------

    def _join_key(self) -> tuple[int, float] | None:
        if self.join_key_provider is None:
            return None
        try:
            key = self.join_key_provider()
        except Exception:
            return None
        if not isinstance(key, (tuple, list)) or len(key) != 2:
            return None
        frame_index = key[0]
        timestamp = key[1]
        if isinstance(frame_index, bool) or not isinstance(frame_index, int) or frame_index < 0:
            return None
        if isinstance(timestamp, bool):
            return None
        try:
            timestamp = float(timestamp)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(timestamp) or timestamp < 0.0:
            return None
        if self._last_join_key is not None:
            previous_frame, previous_timestamp = self._last_join_key
            if frame_index <= previous_frame or timestamp <= previous_timestamp:
                return None
        self._last_join_key = (frame_index, timestamp)
        return (frame_index, timestamp)

    def _journal_scene(self, join: tuple[int, float]) -> dict[str, object] | None:
        diagnostic = self._latest_planning_scene
        if diagnostic is None:
            return None
        frame_index, timestamp = join
        return {**dict(diagnostic), "frame_index": frame_index, "timestamp": timestamp}

    def _readiness(self) -> dict[str, object] | None:
        if self.readiness_snapshot_provider is None:
            return None
        try:
            snapshot = self.readiness_snapshot_provider()
        except Exception:
            return {"ready": False, "reasons": ["readiness_snapshot_provider raised"]}
        if not isinstance(snapshot, Mapping):
            return {"ready": False, "reasons": ["readiness_snapshot_provider returned a non-mapping"]}
        return evaluate_executor_readiness(snapshot, self.config, self.scenario)

    def _graph_observation(self) -> Mapping[str, object] | None:
        if self.graph_observation_provider is None:
            return None
        try:
            graph = self.graph_observation_provider()
        except Exception:
            return None
        return graph if isinstance(graph, Mapping) else None

    # -- goal construction ---------------------------------------------------

    def _pose_stamped_from_spec(self, spec: Mapping[str, object]):
        from geometry_msgs.msg import PoseStamped

        pose = _as_mapping(spec.get("target_pose"))
        xyz = pose.get("xyz")
        quaternion = pose.get("quaternion_xyzw")
        stamped = PoseStamped()
        stamped.header.frame_id = "base_link"
        stamped.pose.position.x = float(xyz[0])
        stamped.pose.position.y = float(xyz[1])
        stamped.pose.position.z = float(xyz[2])
        stamped.pose.orientation.x = float(quaternion[0])
        stamped.pose.orientation.y = float(quaternion[1])
        stamped.pose.orientation.z = float(quaternion[2])
        stamped.pose.orientation.w = float(quaternion[3])
        return stamped

    def _build_goal(self, spec: Mapping[str, object], joints: Sequence[float] | None):
        kind = spec.get("kind")
        if kind == "joint":
            target = list(joints) if joints is not None else list(spec.get("joints", Q_OUTBOUND))
            return build_joint_move_group_goal(target, plan_only=True)
        pose = self._pose_stamped_from_spec(spec)
        return build_pose_move_group_goal(pose, plan_only=True)

    # -- plan-only transaction -----------------------------------------------

    def _spin_once(self) -> None:
        self._spinner.spin_once(timeout_sec=0.05)

    def _wait_for_server(self, client: Any, timeout_s: float) -> bool:
        try:
            return bool(client.wait_for_server(timeout_sec=float(timeout_s)))
        except Exception:
            return False

    def _thresholds(self) -> Mapping[str, object]:
        return _as_mapping(self.config.get("thresholds"))

    def _send_plan_only_goal(
        self,
        scenario_id: str,
        goal: Any,
        spec: Mapping[str, object],
    ) -> dict[str, object]:
        """Send exactly one plan-only goal with bounded, correctly cancelled waits.

        F2.2/F2.8: every exceptional completion (server wait, send future,
        acceptance, result future, cancellation) is converted into a finite
        canonical diagnostic outcome with a stable reason code.  An unaccepted or
        indeterminate send future can never pass, and the evidence states that
        canceling a client future is not proof of server-side cancellation.
        """
        thresholds = self._thresholds()
        client = self._action_clients["/move_action"]
        server_timeout_s = float(thresholds.get("action_server_wait_s", 5.0))
        if not self._wait_for_server(client, server_timeout_s):
            return {
                "scenario_id": scenario_id,
                "status": "action-server-unavailable",
                "reason_code": "action-server-unavailable",
                "diagnostic_only": True,
                "error": "/move_action server was not available before send",
            }

        accept_timeout_s = float(thresholds.get("goal_accept_timeout_s", 5.0))
        send_future = client.send_goal_async(goal)
        accept_deadline = time.monotonic() + accept_timeout_s
        while not send_future.done() and time.monotonic() < accept_deadline:
            self._spin_once()
        if not send_future.done():
            # F2.8: canceling the client future is a client-side no-op and is not
            # proof of server-side cancellation; do not claim a cancel.
            try:
                send_future.cancel()
            except Exception:
                pass
            return {
                "scenario_id": scenario_id,
                "status": "goal-accept-timeout",
                "reason_code": "goal-accept-timeout",
                "diagnostic_only": True,
                "error": (
                    "goal acceptance timed out before a goal handle existed; "
                    "canceling the client send future is not proof of server-side cancellation"
                ),
                "send_future_cancelled": True,
            }
        try:
            goal_handle = send_future.result()
        except Exception as exc:  # F2.2: an exceptional send completion fails closed.
            return {
                "scenario_id": scenario_id,
                "status": "goal-send-exception",
                "reason_code": "goal-send-exception",
                "diagnostic_only": True,
                "error": f"send_goal future raised: {exc}",
            }
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            return {
                "scenario_id": scenario_id,
                "status": "goal-rejected",
                "reason_code": "goal-rejected",
                "diagnostic_only": True,
                "error": "send_goal returned no accepted goal handle",
            }

        result_timeout_s = float(thresholds.get("plan_result_timeout_s", 10.0))
        result_future = goal_handle.get_result_async()
        result_deadline = time.monotonic() + result_timeout_s
        while not result_future.done() and time.monotonic() < result_deadline:
            self._spin_once()
        if not result_future.done():
            cancel_response = self._cancel_goal(goal_handle)
            return {
                "scenario_id": scenario_id,
                "status": "timeout",
                "reason_code": "result-timeout",
                "diagnostic_only": True,
                "error": "planning result timed out",
                "cancel_response": cancel_response,
            }
        try:
            result = result_future.result()
        except Exception as exc:
            return {
                "scenario_id": scenario_id,
                "status": "malformed-result",
                "reason_code": "malformed-result",
                "diagnostic_only": True,
                "error": f"result future raised: {exc}",
            }
        if result is None or getattr(result, "result", None) is None:
            return {
                "scenario_id": scenario_id,
                "status": "malformed-result",
                "reason_code": "malformed-result",
                "diagnostic_only": True,
                "error": "result future returned no MoveGroup result",
            }
        return self._classify_plan_only_result(scenario_id, result, spec)

    def _cancel_goal(self, goal_handle: Any) -> str:
        cancel_timeout_s = float(self._thresholds().get("cancel_timeout_s", 3.0))
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception:
            return "cancel-failed"
        cancel_deadline = time.monotonic() + cancel_timeout_s
        while not cancel_future.done() and time.monotonic() < cancel_deadline:
            self._spin_once()
        return "completed" if cancel_future.done() else "timed-out"

    def _classify_plan_only_result(
        self,
        scenario_id: str,
        result: Any,
        spec: Mapping[str, object],
    ) -> dict[str, object]:
        result_object = getattr(result, "result", None)
        error_code = getattr(result_object, "error_code", None)
        error_value = getattr(error_code, "val", None) if error_code is not None else None
        if not _strict_int(error_value):
            return {
                "scenario_id": scenario_id,
                "status": "malformed-result",
                "reason_code": "malformed-result",
                "diagnostic_only": True,
                "error": "MoveGroup result error_code.val is not a strict integer",
            }
        planned = getattr(result_object, "planned_trajectory", None)
        points = (
            getattr(getattr(planned, "joint_trajectory", None), "points", None)
            if planned is not None
            else None
        )
        nonempty_plan = isinstance(points, (list, tuple)) and len(points) > 0
        trajectory_digest = (
            self._digest(self.ros["serialize_message"](planned)) if planned is not None else None
        )
        expectation = spec.get("expectation")
        success = bool(error_value == MOVEIT_SUCCESS_CODE)
        if expectation == "non-success":
            # F2.4/F3.4: the blocked scenario only passes on an explicit
            # planning-stage non-success after a valid request AND an empty
            # planned trajectory; unknown/request-level codes never pass, and an
            # allowlisted code with a contradictory non-empty trajectory is an
            # explicit contradiction, never a pass.
            if error_value == MOVEIT_SUCCESS_CODE:
                classification = "unexpected-success"
                passed = False
            elif error_value in MOVEIT_PLANNING_NON_SUCCESS_CODES:
                if nonempty_plan:
                    classification = "contradictory-nonempty-trajectory"
                    passed = False
                else:
                    classification = "planning-non-success"
                    passed = True
            else:
                classification = "request-level-or-unknown"
                passed = False
        else:
            passed = success and nonempty_plan
            classification = (
                "success"
                if passed
                else "success-with-empty-plan"
                if success
                else "non-success"
            )
        return {
            "scenario_id": scenario_id,
            "status": "diagnostic-pass" if passed else "diagnostic-fail",
            "diagnostic_only": True,
            "error_code": error_value,
            "error_code_classification": classification,
            "nonempty_plan": nonempty_plan,
            "trajectory_digest": trajectory_digest,
            "expectation": expectation,
        }

    # -- Gate C entry point ---------------------------------------------------

    def run_gate_c_plan_only(
        self, scenario_id: str, *, joints: Sequence[float] | None = None
    ) -> dict[str, object]:
        """Run exactly one plan-only Gate C scenario through ``/move_action``.

        Fail-dominant (F2.1): the authoritative final status is computed after
        the plan outcome *and* every required evidence finalization step.  Any
        readiness, journal event, graph projection, journal finalization, artifact
        serialization/write, or required-artifact-existence failure makes the
        public return and ``integrated-execution.json`` status ``evidence-invalid``;
        no artifact retains a pass claim for that attempt.  The raw planner
        outcome is preserved separately as ``planner_status``.

        Exceptional completion (F2.2): server wait, goal construction/serialization,
        ``send_goal_async``, send-future spin/result, goal acceptance, result-future
        spin/result, cancellation, provider calls, and artifact finalization are all
        converted into finite canonical diagnostic records with zero physical claim
        and exact zero-command/controller flags.  Once ``fixture-ready`` exists the
        executor always attempts teardown journal completion and failed finalization.
        No expected runtime/DDS/action failure escapes the public API.
        """
        fixture_ready_recorded = False
        try:
            try:
                spec = stage_c_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid(
                    scenario_id, "scenario-rejected", [str(exc)]
                )

            if self.join_key_provider is None:
                return self._evidence_invalid(
                    scenario_id,
                    "no-join-key",
                    ["join_key_provider is required before sending any goal"],
                )
            readiness = self._readiness()
            if readiness is None:
                return self._evidence_invalid(
                    scenario_id,
                    "readiness-unavailable",
                    ["readiness_snapshot_provider is required before sending any goal"],
                )
            if not readiness["ready"]:
                return self._evidence_invalid(
                    scenario_id, "readiness-failed", list(readiness["reasons"])
                )

            # F2.5: bounded self-spin to obtain a current fixture scene.
            acquire_error = self._acquire_scene(scenario_id)
            if acquire_error is not None:
                return acquire_error

            join = self._join_key()
            if join is None:
                return self._evidence_invalid(
                    scenario_id,
                    "no-join-key",
                    ["join_key_provider returned no valid strictly-increasing key"],
                )
            scene = self._journal_scene(join)
            if scene is None:
                return self._evidence_invalid(
                    scenario_id,
                    "no-planning-scene",
                    ["no valid PlanningScene cached before fixture-ready"],
                )
            # F2.6: fixture-ready must match the declared fixture contract.
            scene_error = self._fixture_scene_error(scene)
            if scene_error is not None:
                return self._evidence_invalid(
                    scenario_id, "fixture-scene-mismatch", [scene_error]
                )

            try:
                self.journal.record_diff("fixture-ready", scene)
            except (ValueError, TypeError) as exc:
                return self._evidence_invalid(
                    scenario_id, "journal-fixture-ready-rejected", [str(exc)]
                )
            fixture_ready_recorded = True

            # F2.7: `before` visual request is durably flushed before the goal send.
            self._append_visual_request("before", scenario_id, spec)

            try:
                goal = self._build_goal(spec, joints=joints)
                goal_digest = self._digest(self.ros["serialize_message"](goal))
            except Exception as exc:
                goal = None
                goal_digest = None
                outcome = {
                    "scenario_id": scenario_id,
                    "status": "goal-construction-exception",
                    "reason_code": "goal-construction-exception",
                    "diagnostic_only": True,
                    "error": f"goal construction/serialization raised: {exc}",
                }
            else:
                outcome = self._send_plan_only_goal(scenario_id, goal, spec)

            teardown_status = "not-recorded"
            later_join = self._join_key()
            if later_join is None:
                teardown_status = "no-join-key"
            else:
                try:
                    self.journal.snapshot(
                        "teardown", frame_index=later_join[0], timestamp=later_join[1]
                    )
                    teardown_status = "recorded"
                except (ValueError, TypeError) as exc:
                    teardown_status = f"rejected: {exc}"

            # F2.7: `after` visual request only in the post-transaction phase.
            self._append_visual_request("after", scenario_id, spec)

            # F2.1: authoritative fail-dominant final status after the plan outcome
            # and every evidence finalization step.
            planner_status = outcome.get("status")
            final_status = (
                "diagnostic-pass"
                if planner_status == "diagnostic-pass"
                else "diagnostic-fail"
                if planner_status == "diagnostic-fail"
                else "evidence-invalid"
            )
            graph_status = "unavailable"
            journal_finalize_error: str | None = None
            projection = None
            try:
                graph = self._graph_observation()
                if graph is None:
                    raise ValueError("observed graph evidence is unavailable")
                projection = build_journal_graph_projection(
                    fixture_payload=self._fixture_payload_for_graph(),
                    observed_graph=graph,
                )
                # F3.1: validate the graph BEFORE any artifact write (no durable
                # output yet), so a pass is never provisionally persisted before
                # the graph evidence is known to be valid.
                self.journal.finalize(final_status, graph=projection, json_path=None)
                graph_status = "validated"
            except Exception as exc:
                journal_finalize_error = str(exc)
                graph_status = f"invalid: {exc}"
                final_status = "evidence-invalid"
                # F2.1: always produce planning-scene.json as a canonical failure
                # artifact when journal finalization cannot validate the graph.
                if self.journal.record_count > 0:
                    self._finalize_failure_artifact(journal_finalize_error, graph_status)

            record = {
                **outcome,
                "planner_status": planner_status,
                "teardown": teardown_status,
                "graph": graph_status,
                "goal_digest": goal_digest,
                "diagnostic_only": True,
                "execute_trajectory_goal_sent": False,
                "isaac_joint_commands_published": False,
            }
            # F2.1: the fail-dominant status is authoritative in the public record.
            record["status"] = final_status
            if final_status == "evidence-invalid" and record.get("reason_code") is None:
                record["reason_code"] = (
                    "graph-evidence-invalid"
                    if journal_finalize_error is not None
                    else "evidence-invalid"
                )

            # F3.1: transactional finalization/write order.  All non-journal
            # artifacts are made durable first; the successful final journal
            # artifact (planning-scene.json) is deferred until every other
            # required artifact is durable.  If ANY required write fails after a
            # provisional planner/journal pass, every already-created
            # status-bearing artifact is downgraded to evidence-invalid (atomic
            # summaries rewritten, corrective ``row_kind="final"`` rows appended
            # to the JSONL lifecycle streams, and planning-scene.json written as
            # a failure artifact), so no persisted artifact retains a pass.
            try:
                self._write_artifacts(scenario_id, spec, goal, record, readiness, graph_status)
                if graph_status == "validated":
                    self.journal.finalize(
                        final_status,
                        graph=projection,
                        json_path=self.attempt_dir / "planning-scene.json",
                    )
            except Exception as exc:
                downgraded_from = record.get("status")
                record["status"] = "evidence-invalid"
                record["reason_code"] = "artifact-write-failed"
                record["artifact_error"] = str(exc)
                self._downgrade_persisted_evidence(
                    scenario_id,
                    record,
                    readiness,
                    graph_status,
                    planner_status,
                    downgraded_from=downgraded_from,
                    goal_kind=spec.get("kind"),
                )
            return record
        except Exception as exc:  # F2.2: no expected runtime failure escapes the API.
            if fixture_ready_recorded:
                return self._evidence_invalid_after_fixture_ready(
                    scenario_id, exc, spec, readiness
                )
            return self._evidence_invalid(
                scenario_id, "unexpected-exception", [str(exc)]
            )

    # -- Gate D ---------------------------------------------------------------
    #
    # D-stage scenarios reuse Task 4's required attempt paths and transactional
    # fail-dominant mechanics, with a separate D-stage record shape (never
    # changing Gate-C bytes).  Every D attempt stays ``diagnostic_only=true``
    # and ``isaac_joint_commands_published=false``; physical verdicts remain
    # verifier-owned.

    def _evidence_invalid_d(
        self, scenario_id: str, reason_code: str, reasons: Sequence[str]
    ) -> dict[str, object]:
        """D-stage evidence-invalid record with the D schema and durable rows."""
        record: dict[str, object] = {
            "scenario_id": scenario_id,
            "stage": "D",
            "diagnostic_only": True,
            "physical_verdict": None,
            "status": "evidence-invalid",
            "reason_code": reason_code,
            "reasons": list(reasons),
            "execute_trajectory_goal_sent": False,
            "controller_goal_sent": False,
            "isaac_joint_commands_published": False,
        }
        try:
            self._append_jsonl(
                self.attempt_dir / "integrated-execution.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "event": "gate-d",
                    "stage": "D",
                    "status": "evidence-invalid",
                    "reason_code": reason_code,
                    "reasons": list(reasons),
                    "diagnostic_only": True,
                    "execute_trajectory_goal_sent": False,
                    "controller_goal_sent": False,
                    "isaac_joint_commands_published": False,
                    "timestamp": float(time.monotonic()),
                },
            )
        except Exception:
            pass
        try:
            self._write_json_atomic(
                self.attempt_dir / "integrated-execution.json",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "stage": "D",
                    "diagnostic_only": True,
                    "physical_verdict": None,
                    "status": "evidence-invalid",
                    "reason_code": reason_code,
                    "reasons": list(reasons),
                    "execute_trajectory_goal_sent": False,
                    "controller_goal_sent": False,
                    "isaac_joint_commands_published": False,
                },
            )
        except Exception:
            pass
        # Fail-dominant D journal: when the attempt recorded fixture-ready (the
        # journal holds records), emit the canonical failure planning-scene.json.
        # A pre-fixture-ready failure leaves the journal empty; finalize_failure
        # rejects that and the helper swallows it, matching Gate C's no-scene
        # behavior.
        self._d_journal_failure(reason=reason_code, graph_diagnosis="D diagnostic journal")
        return record

    def _d_journal_pass(self, *, scenario_id: str, execute_error: str | None = None) -> str:
        """Finalize the D journal on the pass path with the observed graph.

        Returns ``"validated"`` on success or ``"invalid: <reason>"``; a failed
        finalize never leaves a dangling journal and writes the canonical failure
        planning-scene.json instead (the caller then downgrades the attempt).
        """
        try:
            graph = self._graph_observation()
            if graph is None:
                raise ValueError("observed graph evidence is unavailable")
            projection = build_journal_graph_projection(
                fixture_payload=self._fixture_payload_for_graph(),
                observed_graph=graph,
            )
            self.journal.finalize(
                "diagnostic-pass",
                graph=projection,
                json_path=self.attempt_dir / "planning-scene.json",
            )
            return "validated"
        except Exception as exc:
            try:
                self.journal.finalize_failure(
                    reason=execute_error or f"D journal finalize failed: {exc}",
                    graph_diagnosis=f"invalid: {exc}",
                    json_path=self.attempt_dir / "planning-scene.json",
                )
            except Exception:
                pass
            return f"invalid: {exc}"

    def _d_journal_failure(self, *, reason: str, graph_diagnosis: str) -> str:
        """Write the canonical D failure planning-scene.json, returning its status."""
        try:
            self.journal.finalize_failure(
                reason=reason,
                graph_diagnosis=graph_diagnosis,
                json_path=self.attempt_dir / "planning-scene.json",
            )
            return "written"
        except Exception as exc:
            return f"failed: {exc}"

    def _normalize_goal_id(self, goal_handle: Any) -> str | None:
        return _normalize_goal_uuid(getattr(goal_handle, "goal_id", None))

    def _build_d_goal(self, spec: Mapping[str, object], joints: Sequence[float] | None):
        kind = spec.get("kind")
        if kind == "execute-joint":
            target = (
                list(joints)
                if joints is not None
                else list(spec.get("joints", Q_OUTBOUND))
            )
            return build_joint_move_group_goal(target, plan_only=True)
        pose = self._pose_stamped_from_spec(spec)
        return build_pose_move_group_goal(pose, plan_only=True)

    def _send_plan_only_retaining_handle(
        self,
        scenario_id: str,
        goal: Any,
        spec: Mapping[str, object],
    ) -> dict[str, object]:
        """Send exactly one plan-only MoveGroup goal retaining handle/UUID/plan.

        Mirrors the Gate-C ``_send_plan_only_goal`` bounded lifecycle but keeps
        the accepted goal handle, its normalized lowercase planning UUID, and the
        complete generated ``planned_trajectory`` message for the split path.
        """
        thresholds = self._thresholds()
        client = self._action_clients["/move_action"]
        server_timeout_s = float(thresholds.get("action_server_wait_s", 5.0))
        if not self._wait_for_server(client, server_timeout_s):
            return {
                "scenario_id": scenario_id,
                "status": "action-server-unavailable",
                "reason_code": "action-server-unavailable",
                "diagnostic_only": True,
                "error": "/move_action server was not available before send",
            }
        accept_timeout_s = float(thresholds.get("goal_accept_timeout_s", 5.0))
        send_future = client.send_goal_async(goal)
        accept_deadline = time.monotonic() + accept_timeout_s
        while not send_future.done() and time.monotonic() < accept_deadline:
            self._spin_once()
        if not send_future.done():
            try:
                send_future.cancel()
            except Exception:
                pass
            return {
                "scenario_id": scenario_id,
                "status": "goal-accept-timeout",
                "reason_code": "goal-accept-timeout",
                "diagnostic_only": True,
                "error": "goal acceptance timed out before a goal handle existed",
            }
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            return {
                "scenario_id": scenario_id,
                "status": "goal-send-exception",
                "reason_code": "goal-send-exception",
                "diagnostic_only": True,
                "error": f"send_goal future raised: {exc}",
            }
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            return {
                "scenario_id": scenario_id,
                "status": "goal-rejected",
                "reason_code": "goal-rejected",
                "diagnostic_only": True,
                "error": "send_goal returned no accepted goal handle",
            }
        planning_goal_id = self._normalize_goal_id(goal_handle)
        result_timeout_s = float(thresholds.get("plan_result_timeout_s", 10.0))
        result_future = goal_handle.get_result_async()
        result_deadline = time.monotonic() + result_timeout_s
        while not result_future.done() and time.monotonic() < result_deadline:
            self._spin_once()
        if not result_future.done():
            cancel_response = self._cancel_goal(goal_handle)
            return {
                "scenario_id": scenario_id,
                "status": "timeout",
                "reason_code": "result-timeout",
                "diagnostic_only": True,
                "error": "planning result timed out",
                "cancel_response": cancel_response,
                "planning_goal_id": planning_goal_id,
                "goal_handle": goal_handle,
            }
        try:
            result = result_future.result()
        except Exception as exc:
            return {
                "scenario_id": scenario_id,
                "status": "malformed-result",
                "reason_code": "malformed-result",
                "diagnostic_only": True,
                "error": f"result future raised: {exc}",
                "planning_goal_id": planning_goal_id,
                "goal_handle": goal_handle,
            }
        if result is None or getattr(result, "result", None) is None:
            return {
                "scenario_id": scenario_id,
                "status": "malformed-result",
                "reason_code": "malformed-result",
                "diagnostic_only": True,
                "error": "result future returned no MoveGroup result",
                "planning_goal_id": planning_goal_id,
                "goal_handle": goal_handle,
            }
        planned = getattr(getattr(result, "result", None), "planned_trajectory", None)
        outcome = self._classify_plan_only_result(scenario_id, result, spec)
        outcome = dict(outcome)
        outcome["planning_goal_id"] = planning_goal_id
        outcome["goal_handle"] = goal_handle
        outcome["planned_trajectory"] = planned
        return outcome

    def _send_execute_trajectory(
        self,
        scenario_id: str,
        goal: Any,
        *,
        result_timeout_s: float,
        cancel_timeout_s: float,
        allow_cancel: bool = False,
    ) -> dict[str, object]:
        """Send exactly one ExecuteTrajectory goal and wait bounded for result.

        Returns the distinct execution UUID, the terminal action status int and
        string, and the accepted goal handle.  ``allow_cancel`` permits the
        caller to cancel the ExecuteTrajectory handle instead of timing out; a
        cancellation is never proof of controller-side quiescence by itself.
        """
        thresholds = self._thresholds()
        client = self._action_clients["/execute_trajectory"]
        server_timeout_s = float(thresholds.get("action_server_wait_s", 5.0))
        if not self._wait_for_server(client, server_timeout_s):
            return {
                "scenario_id": scenario_id,
                "status": "execute-server-unavailable",
                "reason_code": "execute-server-unavailable",
                "diagnostic_only": True,
                "error": "/execute_trajectory server was not available before send",
            }
        accept_timeout_s = float(thresholds.get("goal_accept_timeout_s", 5.0))
        send_future = client.send_goal_async(goal)
        accept_deadline = time.monotonic() + accept_timeout_s
        while not send_future.done() and time.monotonic() < accept_deadline:
            self._spin_once()
        if not send_future.done():
            try:
                send_future.cancel()
            except Exception:
                pass
            return {
                "scenario_id": scenario_id,
                "status": "execute-goal-accept-timeout",
                "reason_code": "execute-goal-accept-timeout",
                "diagnostic_only": True,
                "error": "execute goal acceptance timed out before a goal handle existed",
            }
        try:
            goal_handle = send_future.result()
        except Exception as exc:
            return {
                "scenario_id": scenario_id,
                "status": "execute-goal-send-exception",
                "reason_code": "execute-goal-send-exception",
                "diagnostic_only": True,
                "error": f"execute send_goal future raised: {exc}",
            }
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            return {
                "scenario_id": scenario_id,
                "status": "execute-goal-rejected",
                "reason_code": "execute-goal-rejected",
                "diagnostic_only": True,
                "error": "execute send_goal returned no accepted goal handle",
            }
        execute_goal_id = self._normalize_goal_id(goal_handle)
        result_future = goal_handle.get_result_async()
        result_deadline = time.monotonic() + float(result_timeout_s)
        while not result_future.done() and time.monotonic() < result_deadline:
            self._spin_once()
        if not result_future.done():
            if allow_cancel:
                cancel_response = self._cancel_goal(goal_handle)
                return {
                    "scenario_id": scenario_id,
                    "status": "execute-cancellation-issued",
                    "reason_code": "execute-cancellation-issued",
                    "diagnostic_only": True,
                    "execute_goal_id": execute_goal_id,
                    "goal_handle": goal_handle,
                    "cancel_response": cancel_response,
                }
            return {
                "scenario_id": scenario_id,
                "status": "execute-result-timeout",
                "reason_code": "execute-result-timeout",
                "diagnostic_only": True,
                "execute_goal_id": execute_goal_id,
                "goal_handle": goal_handle,
            }
        try:
            result = result_future.result()
        except Exception as exc:
            return {
                "scenario_id": scenario_id,
                "status": "execute-malformed-result",
                "reason_code": "execute-malformed-result",
                "diagnostic_only": True,
                "error": f"execute result future raised: {exc}",
                "execute_goal_id": execute_goal_id,
                "goal_handle": goal_handle,
            }
        action_status = getattr(result, "status", None)
        try:
            status_string = _execute_status_name(action_status)
        except ValueError:
            status_string = None
        return {
            "scenario_id": scenario_id,
            "status": "execute-terminal",
            "reason_code": "execute-terminal",
            "execute_goal_id": execute_goal_id,
            "goal_handle": goal_handle,
            "execute_result_status": action_status,
            "execute_result_status_string": status_string,
            "diagnostic_only": True,
        }

    def _validate_fjt_evidence(
        self,
        provider_evidence: object,
        *,
        expected_trajectory_digest: str | None,
    ) -> tuple[bool, str | None]:
        """Validate injected FJT transaction evidence and join it to status.

        The provider must return real observed controller-transaction evidence
        (endpoint exactly ``/xarm7_traj_controller/follow_joint_trajectory``,
        normalized FJT goal UUID, canonical trajectory digest equal to the
        unchanged ExecuteTrajectory digest, a finite timestamp/sequence, and a
        real observation source).  The provider UUID/status must join to the
        actual newest status-topic entry.  Missing, stale, mismatched, extra,
        malformed, or provider-exception evidence makes the attempt
        ``evidence-invalid``.
        """
        if not isinstance(provider_evidence, Mapping):
            return False, "fjt_transaction_provider returned a non-mapping"
        if provider_evidence.get("endpoint") != FJT_ENDPOINT:
            return False, f"fjt evidence endpoint must be {FJT_ENDPOINT}"
        goal_uuid = provider_evidence.get("goal_uuid")
        if not _valid_goal_uuid(goal_uuid):
            return False, "fjt evidence goal_uuid is not a valid 16-byte hex UUID"
        digest = provider_evidence.get("trajectory_digest")
        if not _valid_digest(digest):
            return False, "fjt evidence trajectory digest must be a valid nonzero digest"
        if expected_trajectory_digest is not None and digest != expected_trajectory_digest:
            return False, "fjt evidence trajectory digest does not match the ExecuteTrajectory trajectory"
        source = provider_evidence.get("source")
        if not isinstance(source, str) or not source:
            return False, "fjt evidence source must identify real controller introspection"
        sequence = provider_evidence.get("sequence")
        timestamp = provider_evidence.get("timestamp")
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            return False, "fjt evidence sequence must be a non-negative integer"
        if not _finite_number(timestamp):
            return False, "fjt evidence timestamp must be finite"
        provider_status = provider_evidence.get("status")
        if isinstance(provider_status, bool) or not isinstance(provider_status, int):
            return False, "fjt evidence status must be an integer"
        newest = self._newest_fjt_status()
        if newest is None:
            return False, "no FJT status-topic entry was observed for this transaction"
        if newest.get("goal_uuid") != goal_uuid:
            return False, "fjt evidence goal_uuid does not join to the newest status entry"
        if newest.get("status") != provider_status:
            return False, "fjt evidence status does not join to the newest status entry"
        return True, None

    def _journal_snapshot_d(self, event: str) -> str:
        """Append one D diagnostic journal snapshot, returning the join status."""
        later_join = self._join_key()
        if later_join is None:
            return "no-join-key"
        try:
            self.journal.snapshot(
                event, frame_index=later_join[0], timestamp=later_join[1]
            )
            return "recorded"
        except (ValueError, TypeError) as exc:
            return f"rejected: {exc}"

    def run_execute_sequence(
        self,
        scenario_id: str,
        *,
        joints: Sequence[float] | None = None,
        fjt_transaction_provider: Callable[[], Mapping[str, object]] | None = None,
    ) -> dict[str, object]:
        """Run one Stage-D split-path joint/pose execution.

        Mandatory MoveGroup plan-only -> ExecuteTrajectory boundary: exactly one
        plan-only ``/move_action`` goal, a required non-empty ``planned_trajectory``,
        exactly one ``/execute_trajectory`` goal whose trajectory is byte-identical
        to the planned one, distinct valid 16-byte plan/execute UUIDs, and FJT
        transaction evidence joining to the real status topic.
        """
        start_wall = time.monotonic()
        fixture_ready_recorded = False
        event_log: list[str] = []
        try:
            try:
                spec = stage_d_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid_d(scenario_id, "scenario-rejected", [str(exc)])
            if spec["kind"] not in ("execute-joint", "execute-pose"):
                return self._evidence_invalid_d(
                    scenario_id, "wrong-handler", [f"D handler is {spec['kind']!r}, not execute"]
                )
            if fjt_transaction_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-fjt-provider",
                    ["fjt_transaction_provider is required before sending any goal"],
                )
            if self.join_key_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-join-key",
                    ["join_key_provider is required before sending any goal"],
                )
            readiness = self._readiness()
            if readiness is None:
                return self._evidence_invalid_d(
                    scenario_id, "readiness-unavailable",
                    ["readiness_snapshot_provider is required before sending any goal"],
                )
            if not readiness["ready"]:
                return self._evidence_invalid_d(scenario_id, "readiness-failed", list(readiness["reasons"]))
            acquire_error = self._acquire_scene(scenario_id)
            if acquire_error is not None:
                return acquire_error
            join = self._join_key()
            if join is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-join-key", ["join_key_provider returned no valid key"]
                )
            scene = self._journal_scene(join)
            if scene is None:
                return self._evidence_invalid_d(scenario_id, "no-planning-scene", [])
            scene_error = self._fixture_scene_error(scene)
            if scene_error is not None:
                return self._evidence_invalid_d(scenario_id, "fixture-scene-mismatch", [scene_error])
            try:
                self.journal.record_diff("fixture-ready", scene)
            except (ValueError, TypeError) as exc:
                return self._evidence_invalid_d(scenario_id, "journal-fixture-ready-rejected", [str(exc)])
            fixture_ready_recorded = True
            event_log.append("fixture-ready")

            try:
                goal = self._build_d_goal(spec, joints=joints)
            except Exception as exc:
                return self._evidence_invalid_d(
                    scenario_id, "goal-construction-exception", [str(exc)]
                )
            plan_outcome = self._send_plan_only_retaining_handle(scenario_id, goal, spec)
            planner_status = plan_outcome.get("status")
            planning_goal_id = plan_outcome.get("planning_goal_id")
            planned = plan_outcome.get("planned_trajectory")
            if planner_status != "diagnostic-pass":
                final_status = (
                    "diagnostic-fail"
                    if planner_status == "diagnostic-fail"
                    else "evidence-invalid"
                )
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, final_status,
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                )
            if planned is None:
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "diagnostic-fail",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                )
            event_log.append("execution-start")
            self._journal_snapshot_d("execution-start")

            planned_digest_before = self._digest(self.ros["serialize_message"](planned))
            try:
                exec_goal = build_execute_trajectory_goal(planned)
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error=f"execute goal construction raised: {exc}",
                )
            executed_digest_after = self._digest(
                self.ros["serialize_message"](exec_goal.trajectory)
            )
            if executed_digest_after != planned_digest_before:
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_error="ExecuteTrajectory trajectory digest differs from the planned trajectory",
                )
            execute_timeout_s = float(self._thresholds().get("execute_timeout_s", 120.0))
            cancel_timeout_s = float(self._thresholds().get("cancel_timeout_s", 10.0))
            exec_outcome = self._send_execute_trajectory(
                scenario_id, exec_goal,
                result_timeout_s=execute_timeout_s,
                cancel_timeout_s=cancel_timeout_s,
            )
            execute_goal_id = exec_outcome.get("execute_goal_id")
            execute_result_status = exec_outcome.get("execute_result_status")
            if (
                not _valid_goal_uuid(planning_goal_id)
                or not _valid_goal_uuid(execute_goal_id)
                or planning_goal_id == execute_goal_id
            ):
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="plan/execute UUIDs must both be valid and distinct",
                )
            if execute_result_status != EXECUTE_STATUS_SUCCEEDED:
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "diagnostic-fail",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_outcome=exec_outcome,
                    execute_error="ExecuteTrajectory did not terminate SUCCEEDED",
                )
            try:
                provider_evidence = fjt_transaction_provider()
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_outcome=exec_outcome,
                    execute_error=f"fjt_transaction_provider raised: {exc}",
                )
            fjt_ok, fjt_reason = self._validate_fjt_evidence(
                provider_evidence, expected_trajectory_digest=executed_digest_after
            )
            if not fjt_ok:
                return self._finalize_d_attempt(
                    scenario_id, spec, plan_outcome, planner_status, "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_outcome=exec_outcome,
                    execute_error=fjt_reason or "fjt evidence invalid",
                )
            event_log.append("execution-terminal")
            self._journal_snapshot_d("execution-terminal")
            event_log.append("teardown")
            self._journal_snapshot_d("teardown")

            return self._finalize_d_attempt(
                scenario_id, spec, plan_outcome, planner_status, "diagnostic-pass",
                readiness, start_wall, event_log=event_log,
                planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                execute_goal_id=execute_goal_id,
                execute_outcome=exec_outcome,
                fjt_evidence=provider_evidence,
                trajectory_digest=executed_digest_after,
            )
        except Exception as exc:
            if fixture_ready_recorded:
                return self._evidence_invalid_d(
                    scenario_id, "unexpected-exception", [str(exc)]
                )
            return self._evidence_invalid_d(
                scenario_id, "unexpected-exception", [str(exc)]
            )

    def run_cancel_sequence(
        self,
        scenario_id: str,
        *,
        long_motion_provider: Callable[[], Mapping[str, object]] | None = None,
        fjt_transaction_provider: Callable[[], Mapping[str, object]] | None = None,
        planning_goal_id: str | None = None,
        execute_goal_id: str | None = None,
        timeout_s: float | None = None,
        execute_goal_handle: Any = None,
    ) -> dict[str, object]:
        """Run the Stage-D cancellation contract.

        Requires an explicit validated long-motion target (via
        *long_motion_provider* or the supplied plan/execute goal ids); missing
        target evidence fails closed before goal send.  When the orchestrator or
        focused test supplies the live ``/execute_trajectory``
        ``ClientGoalHandle`` via *execute_goal_handle*, ``cancel_goal_async()``
        is called on exactly that handle (never the completed MoveGroup planning
        handle).  Requires terminal CANCELED (5), requires controller
        quiescence, and never sends a later stage.
        """
        start_wall = time.monotonic()
        fixture_ready_recorded = False
        event_log: list[str] = []
        try:
            try:
                spec = stage_d_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid_d(scenario_id, "scenario-rejected", [str(exc)])
            if spec["kind"] != "cancel":
                return self._evidence_invalid_d(
                    scenario_id, "wrong-handler", [f"D handler is {spec['kind']!r}, not cancel"]
                )
            if (
                long_motion_provider is None
                and not (_valid_goal_uuid(planning_goal_id) and _valid_goal_uuid(execute_goal_id))
            ):
                return self._evidence_invalid_d(
                    scenario_id, "no-cancel-target",
                    ["cancel requires an explicit validated long-motion target/provider"],
                )
            if fjt_transaction_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-fjt-provider",
                    ["fjt_transaction_provider is required before sending any goal"],
                )
            if self.join_key_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-join-key",
                    ["join_key_provider is required before sending any goal"],
                )
            readiness = self._readiness()
            if readiness is None or not readiness["ready"]:
                return self._evidence_invalid_d(
                    scenario_id,
                    "readiness-unavailable" if readiness is None else "readiness-failed",
                    [] if readiness is None else list(readiness["reasons"]),
                )
            acquire_error = self._acquire_scene(scenario_id)
            if acquire_error is not None:
                return acquire_error
            join = self._join_key()
            if join is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            scene = self._journal_scene(join)
            if scene is None:
                return self._evidence_invalid_d(scenario_id, "no-planning-scene", [])
            scene_error = self._fixture_scene_error(scene)
            if scene_error is not None:
                return self._evidence_invalid_d(scenario_id, "fixture-scene-mismatch", [scene_error])
            try:
                self.journal.record_diff("fixture-ready", scene)
            except (ValueError, TypeError) as exc:
                return self._evidence_invalid_d(scenario_id, "journal-fixture-ready-rejected", [str(exc)])
            fixture_ready_recorded = True
            event_log.append("fixture-ready")

            if long_motion_provider is not None:
                try:
                    target_evidence = long_motion_provider()
                except Exception as exc:
                    return self._evidence_invalid_d(
                        scenario_id, "cancel-target-provider-raised", [str(exc)]
                    )
                if not isinstance(target_evidence, Mapping):
                    return self._evidence_invalid_d(
                        scenario_id, "cancel-target-invalid", ["long_motion_provider returned a non-mapping"]
                    )
                supplied_plan = _normalize_goal_uuid(target_evidence.get("planning_goal_id"))
                supplied_exec = _normalize_goal_uuid(target_evidence.get("execute_goal_id"))
                if not (_valid_goal_uuid(supplied_plan) and _valid_goal_uuid(supplied_exec)):
                    return self._evidence_invalid_d(
                        scenario_id, "cancel-target-invalid",
                        ["long_motion_provider must supply valid plan and execute UUIDs"],
                    )
                planning_goal_id = supplied_plan
                execute_goal_id = supplied_exec
            if planning_goal_id == execute_goal_id:
                return self._evidence_invalid_d(
                    scenario_id, "cancel-target-invalid", ["plan and execute UUIDs must differ"]
                )

            event_log.append("execution-start")
            self._journal_snapshot_d("execution-start")

            # The long-motion ExecuteTrajectory transaction is supplied by the
            # provider/target evidence (mid-flight).  Cancel only that exact
            # ExecuteTrajectory handle; the MoveGroup planning handle was already
            # completed and is never canceled.
            goals_canceling = [execute_goal_id]
            # §7: call ``cancel_goal_async()`` only on the ExecuteTrajectory
            # handle; never cancel the completed MoveGroup planning handle.
            cancel_response = "not-applicable"
            if execute_goal_handle is not None:
                cancel_response = self._cancel_goal(execute_goal_handle)
                if cancel_response != "completed":
                    return self._finalize_d_attempt(
                        scenario_id, spec, {"status": "diagnostic-pass", "diagnostic_only": True},
                        "diagnostic-pass", "evidence-invalid",
                        readiness, start_wall, event_log=event_log,
                        planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                        execute_goal_id=execute_goal_id,
                        execute_error=f"ExecuteTrajectory cancel did not complete: {cancel_response}",
                        goals_canceling=goals_canceling,
                    )
            event_log.append("cancel-requested")
            self._journal_snapshot_d("cancel-requested")

            # Quiescence: bounded wait until no active FJT status joins the
            # canceled controller goal.  The injected FJT evidence must join to
            # the newest status entry with the canceled UUID.
            try:
                provider_evidence = fjt_transaction_provider()
            except Exception as exc:
                return self._finalize_d_attempt(
                    scenario_id, spec, {"status": "diagnostic-pass", "diagnostic_only": True},
                    "diagnostic-pass", "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=f"fjt_transaction_provider raised: {exc}",
                    goals_canceling=goals_canceling,
                )
            fjt_ok, fjt_reason = self._validate_fjt_evidence(
                provider_evidence, expected_trajectory_digest=None
            )
            if not fjt_ok:
                return self._finalize_d_attempt(
                    scenario_id, spec, {"status": "diagnostic-pass", "diagnostic_only": True},
                    "diagnostic-pass", "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error=fjt_reason or "fjt evidence invalid",
                    goals_canceling=goals_canceling,
                )
            newest = self._newest_fjt_status()
            active = bool(
                newest is not None
                and newest.get("goal_uuid") == execute_goal_id
                and newest.get("status") not in (EXECUTE_STATUS_SUCCEEDED, EXECUTE_STATUS_CANCELED, EXECUTE_STATUS_ABORTED)
            )
            if active:
                return self._finalize_d_attempt(
                    scenario_id, spec, {"status": "diagnostic-pass", "diagnostic_only": True},
                    "diagnostic-pass", "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="FJT status is still active for the canceled controller goal",
                    goals_canceling=goals_canceling,
                )
            event_log.append("quiescent")
            self._journal_snapshot_d("quiescent")
            event_log.append("teardown")
            self._journal_snapshot_d("teardown")
            return self._finalize_d_attempt(
                scenario_id, spec, {"status": "diagnostic-pass", "diagnostic_only": True},
                "diagnostic-pass", "diagnostic-pass",
                readiness, start_wall, event_log=event_log,
                planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                execute_goal_id=execute_goal_id,
                goals_canceling=goals_canceling,
                terminal_status="canceled",
                fjt_evidence=provider_evidence,
            )
        except Exception as exc:
            return self._evidence_invalid_d(
                scenario_id, "unexpected-exception", [str(exc)]
            )

    def run_safety_sequence(
        self,
        scenario_id: str,
        *,
        long_motion_provider: Callable[[], Mapping[str, object]] | None = None,
        fjt_transaction_provider: Callable[[], Mapping[str, object]] | None = None,
        stop_timeout_s: float | None = None,
    ) -> dict[str, object]:
        """Run the Stage-D safety interruption contract.

        Uses the executor's real ``/sim/safety/operator`` publisher and the
        ``/sim/hardware/safety_stop``/``/joint_states`` callbacks: publishes
        operator ``True``, waits bounded for safety-stop ``True``, requires the
        old ExecuteTrajectory terminal status ABORTED (6), requires
        ``safety_stop_frames`` consecutive fresh joint-state velocity frames
        with every arm-joint absolute velocity bounded, publishes operator
        ``False`` after the effective-stop predicate, and sends no replacement/
        resume goal.
        """
        start_wall = time.monotonic()
        fixture_ready_recorded = False
        event_log: list[str] = []
        try:
            try:
                spec = stage_d_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid_d(scenario_id, "scenario-rejected", [str(exc)])
            if spec["kind"] != "safety":
                return self._evidence_invalid_d(
                    scenario_id, "wrong-handler", [f"D handler is {spec['kind']!r}, not safety"]
                )
            if long_motion_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-safety-target",
                    ["safety requires an explicit validated long-motion target/provider"],
                )
            if fjt_transaction_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-fjt-provider",
                    ["fjt_transaction_provider is required before sending any goal"],
                )
            if self.join_key_provider is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            readiness = self._readiness()
            if readiness is None or not readiness["ready"]:
                return self._evidence_invalid_d(
                    scenario_id,
                    "readiness-unavailable" if readiness is None else "readiness-failed",
                    [] if readiness is None else list(readiness["reasons"]),
                )
            acquire_error = self._acquire_scene(scenario_id)
            if acquire_error is not None:
                return acquire_error
            join = self._join_key()
            if join is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            scene = self._journal_scene(join)
            if scene is None:
                return self._evidence_invalid_d(scenario_id, "no-planning-scene", [])
            scene_error = self._fixture_scene_error(scene)
            if scene_error is not None:
                return self._evidence_invalid_d(scenario_id, "fixture-scene-mismatch", [scene_error])
            try:
                self.journal.record_diff("fixture-ready", scene)
            except (ValueError, TypeError) as exc:
                return self._evidence_invalid_d(scenario_id, "journal-fixture-ready-rejected", [str(exc)])
            fixture_ready_recorded = True
            event_log.append("fixture-ready")

            try:
                target_evidence = long_motion_provider()
            except Exception as exc:
                return self._evidence_invalid_d(scenario_id, "safety-target-provider-raised", [str(exc)])
            if not isinstance(target_evidence, Mapping):
                return self._evidence_invalid_d(scenario_id, "safety-target-invalid", [])
            planning_goal_id = _normalize_goal_uuid(target_evidence.get("planning_goal_id"))
            execute_goal_id = _normalize_goal_uuid(target_evidence.get("execute_goal_id"))
            if not (_valid_goal_uuid(planning_goal_id) and _valid_goal_uuid(execute_goal_id)):
                return self._evidence_invalid_d(
                    scenario_id, "safety-target-invalid",
                    ["safety target must supply valid plan and execute UUIDs"],
                )
            if planning_goal_id == execute_goal_id:
                return self._evidence_invalid_d(
                    scenario_id, "safety-target-invalid", ["plan and execute UUIDs must differ"]
                )

            event_log.append("execution-start")
            self._journal_snapshot_d("execution-start")

            # 3. publish operator True.
            try:
                self.publish_operator(True)
            except Exception as exc:
                return self._evidence_invalid_d(scenario_id, "operator-publish-failed", [str(exc)])
            # 4. wait bounded for safety-stop True.
            safety_stop_seen = self._wait_for_safety_stop(stop_timeout_s)
            if not safety_stop_seen:
                return self._finalize_d_attempt(
                    scenario_id, spec, {"status": "diagnostic-pass", "diagnostic_only": True},
                    "diagnostic-pass", "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="safety-stop True was not observed within the bounded wait",
                )
            event_log.append("effective-stop")
            self._journal_snapshot_d("effective-stop")

            # 5. the old ExecuteTrajectory action terminal status is ABORTED or
            # another explicitly non-success safety-abort result; never success.
            fjt_evidence = None
            try:
                fjt_evidence = fjt_transaction_provider()
            except Exception:
                pass
            fjt_ok, fjt_reason = self._validate_fjt_evidence(
                fjt_evidence, expected_trajectory_digest=None
            ) if fjt_evidence is not None else (False, "no fjt evidence")
            terminal_status = self._safety_terminal_status(fjt_evidence)
            if terminal_status != "aborted":
                return self._finalize_d_attempt(
                    scenario_id, spec, {"status": "diagnostic-pass", "diagnostic_only": True},
                    "diagnostic-pass", "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="safety did not produce a non-success ABORTED terminal status",
                    fjt_evidence=fjt_evidence,
                )

            # 6. safety_stop_frames consecutive fresh joint-state velocity frames.
            frames = self._safety_velocity_frames()
            velocity_limit = float(self._thresholds().get("safety_stop_velocity_rad_s", 0.02))
            required_frames = int(self._thresholds().get("safety_stop_frames", 5))
            if not _arm_velocity_within_limit(frames, velocity_limit) or len(frames) < required_frames:
                return self._finalize_d_attempt(
                    scenario_id, spec, {"status": "diagnostic-pass", "diagnostic_only": True},
                    "diagnostic-pass", "evidence-invalid",
                    readiness, start_wall, event_log=event_log,
                    planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                    execute_goal_id=execute_goal_id,
                    execute_error="safety velocity frames were not bounded or were insufficient",
                    fjt_evidence=fjt_evidence,
                )

            # 7. publish operator False only after the effective-stop predicate.
            try:
                self.publish_operator(False)
            except Exception as exc:
                return self._evidence_invalid_d(scenario_id, "operator-clear-publish-failed", [str(exc)])
            event_log.append("operator-clear")
            self._journal_snapshot_d("operator-clear")

            # 8. bounded post-clear stability: no fresh action/controller goal UUID.
            if self._newest_fjt_status() is not None:
                newest_uuid = self._newest_fjt_status().get("goal_uuid")
                if newest_uuid != execute_goal_id:
                    return self._finalize_d_attempt(
                        scenario_id, spec, {"status": "diagnostic-pass", "diagnostic_only": True},
                        "diagnostic-pass", "evidence-invalid",
                        readiness, start_wall, event_log=event_log,
                        planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                        execute_goal_id=execute_goal_id,
                        execute_error="a fresh goal UUID appeared after operator clear",
                        fjt_evidence=fjt_evidence,
                    )
            event_log.append("quiescent")
            self._journal_snapshot_d("quiescent")
            event_log.append("teardown")
            self._journal_snapshot_d("teardown")
            return self._finalize_d_attempt(
                scenario_id, spec, {"status": "diagnostic-pass", "diagnostic_only": True},
                "diagnostic-pass", "diagnostic-pass",
                readiness, start_wall, event_log=event_log,
                planning_goal_id=planning_goal_id, fixture_ready_recorded=fixture_ready_recorded,
                execute_goal_id=execute_goal_id,
                terminal_status="aborted",
                fjt_evidence=fjt_evidence,
            )
        except Exception as exc:
            return self._evidence_invalid_d(
                scenario_id, "unexpected-exception", [str(exc)]
            )

    def _wait_for_safety_stop(self, stop_timeout_s: float | None) -> bool:
        """Bounded spin until the newest safety-stop sample reads True."""
        timeout_s = float(
            stop_timeout_s if stop_timeout_s is not None
            else self._thresholds().get("safety_stop_wait_s", 0.25)
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            sample = self._latest_safety_stop
            if sample is not None and bool(getattr(sample, "data", False)) is True:
                return True
            self._spin_once()
        return False

    def _safety_terminal_status(self, fjt_evidence: object) -> str:
        """Derive the old ExecuteTrajectory safety terminal status, never success."""
        if isinstance(fjt_evidence, Mapping):
            status = fjt_evidence.get("status")
            if isinstance(status, int) and not isinstance(status, bool):
                try:
                    return _execute_status_name(status)
                except ValueError:
                    return "unknown"
        newest = self._newest_fjt_status()
        if newest is not None:
            status = newest.get("status")
            try:
                return _execute_status_name(status)
            except ValueError:
                return "unknown"
        return "unknown"

    def _safety_velocity_frames(self) -> list[list[float]]:
        """Return the cached bounded arm-velocity frames for the effective-stop
        predicate.

        The live controller streams fresh ``/joint_states`` and the executor
        caches up to ``safety_stop_frames`` consecutive frames through
        :meth:`_on_joint_state`; the effective-stop predicate requires
        ``safety_stop_frames`` consecutive frames with every arm-joint absolute
        velocity bounded.  In the offline/faked Humble suite the test feeds
        ``JointState`` messages through the callback directly.
        """
        return list(self._joint_velocity_frames)

    def run_cartesian_retreat(
        self,
        scenario_id: str,
        *,
        current_tcp_pose_provider: Callable[[], Mapping[str, object]] | None = None,
    ) -> dict[str, object]:
        """Run the Stage-D Cartesian retreat contract.

        Uses an injected ``current_tcp_pose_provider`` (never a TF listener
        embedded in this task) returning a fresh finite normalized ``base_link``
        TCP pose with observation identity/age.  Derives the deterministic
        ``+Z`` ``RETREAT_DISTANCE_M`` target preserving orientation and sends one
        collision-aware ``/cartesian_move_action`` goal.
        """
        start_wall = time.monotonic()
        event_log: list[str] = []
        try:
            try:
                spec = stage_d_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid_d(scenario_id, "scenario-rejected", [str(exc)])
            if spec["kind"] != "retreat":
                return self._evidence_invalid_d(
                    scenario_id, "wrong-handler", [f"D handler is {spec['kind']!r}, not retreat"]
                )
            if current_tcp_pose_provider is None:
                return self._evidence_invalid_d(
                    scenario_id, "no-tcp-pose-provider",
                    ["current_tcp_pose_provider is required before sending a retreat goal"],
                )
            if self.join_key_provider is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            readiness = self._readiness()
            if readiness is None or not readiness["ready"]:
                return self._evidence_invalid_d(
                    scenario_id,
                    "readiness-unavailable" if readiness is None else "readiness-failed",
                    [] if readiness is None else list(readiness["reasons"]),
                )
            acquire_error = self._acquire_scene(scenario_id)
            if acquire_error is not None:
                return acquire_error
            join = self._join_key()
            if join is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            scene = self._journal_scene(join)
            if scene is None:
                return self._evidence_invalid_d(scenario_id, "no-planning-scene", [])
            scene_error = self._fixture_scene_error(scene)
            if scene_error is not None:
                return self._evidence_invalid_d(scenario_id, "fixture-scene-mismatch", [scene_error])
            try:
                self.journal.record_diff("fixture-ready", scene)
            except (ValueError, TypeError) as exc:
                return self._evidence_invalid_d(scenario_id, "journal-fixture-ready-rejected", [str(exc)])
            event_log.append("fixture-ready")

            try:
                source = current_tcp_pose_provider()
            except Exception as exc:
                return self._evidence_invalid_d(scenario_id, "tcp-pose-provider-raised", [str(exc)])
            if not isinstance(source, Mapping):
                return self._evidence_invalid_d(scenario_id, "tcp-pose-invalid", ["provider returned a non-mapping"])
            identity = source.get("identity")
            age = source.get("age_s")
            if not (isinstance(identity, (int, str)) and str(identity)):
                return self._evidence_invalid_d(scenario_id, "tcp-pose-invalid", ["provider must supply observation identity"])
            if not _fresh(age, self._thresholds().get("tf_fresh_s", 0.25)):
                return self._evidence_invalid_d(scenario_id, "tcp-pose-stale", ["provider pose is stale"])
            try:
                target = derive_retreat_target_pose(
                    source, distance_m=RETREAT_DISTANCE_M, axis=RETREAT_AXIS
                )
            except ValueError as exc:
                return self._evidence_invalid_d(scenario_id, "retreat-derivation-failed", [str(exc)])

            event_log.append("retreat-start")
            self._journal_snapshot_d("retreat-start")
            from geometry_msgs.msg import Pose

            target_pose = Pose()
            target_pose.position.x = float(target["xyz"][0])
            target_pose.position.y = float(target["xyz"][1])
            target_pose.position.z = float(target["xyz"][2])
            target_pose.orientation.x = float(target["quaternion_xyzw"][0])
            target_pose.orientation.y = float(target["quaternion_xyzw"][1])
            target_pose.orientation.z = float(target["quaternion_xyzw"][2])
            target_pose.orientation.w = float(target["quaternion_xyzw"][3])
            try:
                cartesian_goal = build_cartesian_move_goal(target_pose)
            except Exception as exc:
                return self._evidence_invalid_d(scenario_id, "cartesian-goal-construction-failed", [str(exc)])

            client = self._action_clients["/cartesian_move_action"]
            server_timeout_s = float(self._thresholds().get("action_server_wait_s", 5.0))
            if not self._wait_for_server(client, server_timeout_s):
                return self._evidence_invalid_d(scenario_id, "cartesian-server-unavailable", [])
            accept_timeout_s = float(self._thresholds().get("goal_accept_timeout_s", 5.0))
            send_future = client.send_goal_async(cartesian_goal)
            accept_deadline = time.monotonic() + accept_timeout_s
            while not send_future.done() and time.monotonic() < accept_deadline:
                self._spin_once()
            if not send_future.done():
                return self._evidence_invalid_d(scenario_id, "cartesian-accept-timeout", [])
            try:
                goal_handle = send_future.result()
            except Exception as exc:
                return self._evidence_invalid_d(scenario_id, "cartesian-send-exception", [str(exc)])
            if goal_handle is None or not getattr(goal_handle, "accepted", False):
                return self._evidence_invalid_d(scenario_id, "cartesian-goal-rejected", [])
            retreat_goal_id = self._normalize_goal_id(goal_handle)
            result_timeout_s = float(self._thresholds().get("execute_timeout_s", 120.0))
            result_future = goal_handle.get_result_async()
            result_deadline = time.monotonic() + result_timeout_s
            while not result_future.done() and time.monotonic() < result_deadline:
                self._spin_once()
            if not result_future.done():
                return self._evidence_invalid_d(scenario_id, "cartesian-result-timeout", [])
            try:
                result = result_future.result()
            except Exception as exc:
                return self._evidence_invalid_d(scenario_id, "cartesian-result-exception", [str(exc)])
            action_status = getattr(result, "status", None)
            try:
                status_string = _execute_status_name(action_status)
            except ValueError:
                status_string = None
            if action_status != EXECUTE_STATUS_SUCCEEDED:
                return self._evidence_invalid_d(scenario_id, "cartesian-non-success", [])

            event_log.append("retreat-terminal")
            self._journal_snapshot_d("retreat-terminal")
            event_log.append("teardown")
            self._journal_snapshot_d("teardown")
            self._write_retreat_goal_artifact(scenario_id, source, target, retreat_goal_id)
            graph_status = self._d_journal_pass(scenario_id=scenario_id)
            if graph_status != "validated":
                return self._evidence_invalid_d(
                    scenario_id, "d-journal-finalize-failed", [graph_status]
                )
            return {
                "scenario_id": scenario_id,
                "handler": "retreat",
                "stage": "D",
                "polarity": spec.get("polarity"),
                "diagnostic_only": True,
                "physical_verdict": None,
                "status": "diagnostic-pass",
                "planner_status": "diagnostic-pass",
                "execute_trajectory_goal_sent": False,
                "controller_goal_sent": False,
                "isaac_joint_commands_published": False,
                "endpoint": CARTESIAN_MOVE_ENDPOINT,
                "target_frame": "base_link",
                "distance_m": RETREAT_DISTANCE_M,
                "axis": RETREAT_AXIS,
                "retreat_goal_id": retreat_goal_id,
                "source_pose": dict(source),
                "target_pose": target,
                "collision_checking": True,
                "command_gateway_bypassed": False,
                "graph": graph_status,
                "event_log": event_log,
                "elapsed_s": round(time.monotonic() - start_wall, 6),
            }
        except Exception as exc:
            return self._evidence_invalid_d(scenario_id, "unexpected-exception", [str(exc)])

    def _write_retreat_goal_artifact(
        self,
        scenario_id: str,
        source: Mapping[str, object],
        target: Mapping[str, object],
        retreat_goal_id: object,
    ) -> None:
        try:
            goal_path = self.attempt_dir / "goals" / f"{scenario_id}.json"
            goal_path.parent.mkdir(parents=True, exist_ok=True)
            _atomic_write_json(
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "handler": "retreat",
                    "stage": "D",
                    "diagnostic_only": True,
                    "physical_verdict": None,
                    "endpoint": CARTESIAN_MOVE_ENDPOINT,
                    "axis": RETREAT_AXIS,
                    "distance_m": RETREAT_DISTANCE_M,
                    "target_frame": "base_link",
                    "source_pose": dict(source),
                    "target_pose": dict(target),
                    "retreat_goal_id": retreat_goal_id,
                    "collision_checking": True,
                    "command_gateway_bypassed": False,
                    "isaac_joint_commands_published": False,
                },
                goal_path,
            )
        except Exception:
            pass

    def run_gripper_sequence(
        self,
        scenario_id: str,
        *,
        open_first: bool = True,
        gripper_open_position: float = GRIPPER_OPEN_POSITION,
        gripper_close_position: float = GRIPPER_CLOSE_POSITION,
        gripper_max_effort: float = GRIPPER_MAX_EFFORT,
    ) -> dict[str, object]:
        """Run the Stage-D native gripper contract.

        Sends open then close (or the reversed order) ``GripperCommand`` goals
        sequentially to ``/xarm_gripper/gripper_action`` with separate bounded
        acceptance/result deadlines; requires action success for each.
        """
        start_wall = time.monotonic()
        event_log: list[str] = []
        try:
            try:
                spec = stage_d_dispatch(scenario_id, scenario=self.scenario)
            except ValueError as exc:
                return self._evidence_invalid_d(scenario_id, "scenario-rejected", [str(exc)])
            if spec["kind"] != "gripper":
                return self._evidence_invalid_d(
                    scenario_id, "wrong-handler", [f"D handler is {spec['kind']!r}, not gripper"]
                )
            if self.join_key_provider is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            readiness = self._readiness()
            if readiness is None or not readiness["ready"]:
                return self._evidence_invalid_d(
                    scenario_id,
                    "readiness-unavailable" if readiness is None else "readiness-failed",
                    [] if readiness is None else list(readiness["reasons"]),
                )
            acquire_error = self._acquire_scene(scenario_id)
            if acquire_error is not None:
                return acquire_error
            join = self._join_key()
            if join is None:
                return self._evidence_invalid_d(scenario_id, "no-join-key", [])
            scene = self._journal_scene(join)
            if scene is None:
                return self._evidence_invalid_d(scenario_id, "no-planning-scene", [])
            scene_error = self._fixture_scene_error(scene)
            if scene_error is not None:
                return self._evidence_invalid_d(scenario_id, "fixture-scene-mismatch", [scene_error])
            try:
                self.journal.record_diff("fixture-ready", scene)
            except (ValueError, TypeError) as exc:
                return self._evidence_invalid_d(scenario_id, "journal-fixture-ready-rejected", [str(exc)])
            event_log.append("fixture-ready")

            commands = (
                [("open", gripper_open_position), ("close", gripper_close_position)]
                if open_first
                else [("close", gripper_close_position), ("open", gripper_open_position)]
            )
            client = self._action_clients["/xarm_gripper/gripper_action"]
            server_timeout_s = float(self._thresholds().get("action_server_wait_s", 5.0))
            if not self._wait_for_server(client, server_timeout_s):
                return self._evidence_invalid_d(scenario_id, "gripper-server-unavailable", [])
            accept_timeout_s = float(self._thresholds().get("goal_accept_timeout_s", 5.0))
            result_timeout_s = float(self._thresholds().get("execute_timeout_s", 120.0))
            command_records: list[dict[str, object]] = []
            goal_uuids: list[object] = []
            for name, position in commands:
                try:
                    gripper_goal = build_gripper_goal(position, max_effort=gripper_max_effort)
                except Exception as exc:
                    return self._evidence_invalid_d(scenario_id, "gripper-goal-construction-failed", [str(exc)])
                send_future = client.send_goal_async(gripper_goal)
                accept_deadline = time.monotonic() + accept_timeout_s
                while not send_future.done() and time.monotonic() < accept_deadline:
                    self._spin_once()
                if not send_future.done():
                    return self._evidence_invalid_d(scenario_id, "gripper-accept-timeout", [])
                try:
                    goal_handle = send_future.result()
                except Exception as exc:
                    return self._evidence_invalid_d(scenario_id, "gripper-send-exception", [str(exc)])
                if goal_handle is None or not getattr(goal_handle, "accepted", False):
                    return self._evidence_invalid_d(scenario_id, "gripper-goal-rejected", [])
                gripper_goal_id = self._normalize_goal_id(goal_handle)
                goal_uuids.append(gripper_goal_id)
                result_future = goal_handle.get_result_async()
                result_deadline = time.monotonic() + result_timeout_s
                while not result_future.done() and time.monotonic() < result_deadline:
                    self._spin_once()
                if not result_future.done():
                    return self._evidence_invalid_d(scenario_id, "gripper-result-timeout", [])
                try:
                    result = result_future.result()
                except Exception as exc:
                    return self._evidence_invalid_d(scenario_id, "gripper-result-exception", [str(exc)])
                action_status = getattr(result, "status", None)
                try:
                    status_string = _execute_status_name(action_status)
                except ValueError:
                    status_string = None
                if action_status != EXECUTE_STATUS_SUCCEEDED:
                    return self._evidence_invalid_d(scenario_id, "gripper-non-success", [])
                command_records.append(
                    {
                        "command": name,
                        "position": position,
                        "max_effort": gripper_max_effort,
                        "goal_id": gripper_goal_id,
                        "status": action_status,
                        "status_string": status_string,
                    }
                )
                event_log.append(f"gripper-{name}-terminal")
                self._journal_snapshot_d(f"gripper-{name}-terminal")

            event_log.append("teardown")
            self._journal_snapshot_d("teardown")
            self._write_gripper_artifact(scenario_id, command_records)
            graph_status = self._d_journal_pass(scenario_id=scenario_id)
            if graph_status != "validated":
                return self._evidence_invalid_d(
                    scenario_id, "d-journal-finalize-failed", [graph_status]
                )
            return {
                "scenario_id": scenario_id,
                "handler": "gripper",
                "stage": "D",
                "polarity": spec.get("polarity"),
                "diagnostic_only": True,
                "physical_verdict": None,
                "status": "diagnostic-pass",
                "planner_status": "diagnostic-pass",
                "execute_trajectory_goal_sent": False,
                "controller_goal_sent": True,
                "isaac_joint_commands_published": False,
                "endpoint": GRIPPER_ENDPOINT,
                "commands": [record["command"] for record in command_records],
                "command_records": command_records,
                "goal_uuids": goal_uuids,
                "native_action": True,
                "open_first": open_first,
                "graph": graph_status,
                "event_log": event_log,
                "elapsed_s": round(time.monotonic() - start_wall, 6),
            }
        except Exception as exc:
            return self._evidence_invalid_d(scenario_id, "unexpected-exception", [str(exc)])

    def _write_gripper_artifact(
        self, scenario_id: str, command_records: Sequence[Mapping[str, object]]
    ) -> None:
        try:
            goal_path = self.attempt_dir / "goals" / f"{scenario_id}.json"
            goal_path.parent.mkdir(parents=True, exist_ok=True)
            _atomic_write_json(
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "handler": "gripper",
                    "stage": "D",
                    "diagnostic_only": True,
                    "physical_verdict": None,
                    "endpoint": GRIPPER_ENDPOINT,
                    "commands": command_records,
                    "native_action": True,
                    "isaac_joint_commands_published": False,
                },
                goal_path,
            )
        except Exception:
            pass

    def run_pick_place_negative(self, scenario_id: str) -> dict[str, object]:
        """Fail-closed Gate-E negative stub (zero ROS goals sent).

        Preserves the brief's observable fake contract for later replacement
        without pre-empting Task 6/Gate E.  See the module-level pure
        :func:`run_pick_place_negative`.
        """
        return run_pick_place_negative(scenario_id)

    def _finalize_d_attempt(
        self,
        scenario_id: str,
        spec: Mapping[str, object],
        plan_outcome: Mapping[str, object],
        planner_status: object,
        final_status: str,
        readiness: Mapping[str, object],
        start_wall: float,
        *,
        event_log: Sequence[str],
        planning_goal_id: object,
        fixture_ready_recorded: bool,
        execute_goal_id: object = None,
        execute_outcome: Mapping[str, object] | None = None,
        execute_error: str | None = None,
        fjt_evidence: object = None,
        goals_canceling: Sequence[object] | None = None,
        terminal_status: str | None = None,
        trajectory_digest: str | None = None,
    ) -> dict[str, object]:
        """Fail-dominant D attempt finalization and artifact write.

        The authoritative ``status`` is computed after the transaction outcome
        *and* every required evidence finalization step.  A pass finalizes the
        D journal with the real observed-graph projection (Task-3 schema
        unchanged) and writes ``planning-scene.json``; a failure writes a
        canonical failure artifact.  Any write failure downgrades every
        already-created status-bearing D artifact.
        """
        # The flow methods record the complete D journal (including teardown)
        # on the pass path before finalization; a failed attempt records no
        # teardown and instead writes a canonical failure planning-scene.json.
        teardown_status = (
            "recorded"
            if event_log and list(event_log)[-1] == "teardown"
            else "not-recorded"
        )
        graph_status = "unavailable"
        if final_status == "diagnostic-pass":
            try:
                graph = self._graph_observation()
                if graph is None:
                    raise ValueError("observed graph evidence is unavailable")
                projection = build_journal_graph_projection(
                    fixture_payload=self._fixture_payload_for_graph(),
                    observed_graph=graph,
                )
                self.journal.finalize(
                    "diagnostic-pass",
                    graph=projection,
                    json_path=self.attempt_dir / "planning-scene.json",
                )
                graph_status = "validated"
            except Exception as exc:
                final_status = "evidence-invalid"
                graph_status = f"invalid: {exc}"
                if fixture_ready_recorded:
                    try:
                        self.journal.finalize_failure(
                            reason=execute_error or f"D journal finalize failed: {exc}",
                            graph_diagnosis=graph_status,
                            json_path=self.attempt_dir / "planning-scene.json",
                        )
                    except Exception:
                        pass
        elif fixture_ready_recorded:
            try:
                self.journal.finalize_failure(
                    reason=execute_error or "D attempt failed",
                    graph_diagnosis="D diagnostic journal",
                    json_path=self.attempt_dir / "planning-scene.json",
                )
            except Exception:
                pass
        executed_digest = trajectory_digest
        if execute_outcome is not None:
            result_status = execute_outcome.get("execute_result_status")
            try:
                result_status_string = _execute_status_name(result_status)
            except ValueError:
                result_status_string = None
        else:
            result_status = None
            result_status_string = None
        record: dict[str, object] = {
            "scenario_id": scenario_id,
            "handler": spec.get("kind"),
            "stage": "D",
            "polarity": spec.get("polarity"),
            "diagnostic_only": True,
            "physical_verdict": None,
            "status": final_status,
            "planner_status": planner_status,
            # The execute goal is sent only by the split-path execute handler
            # (once a valid ExecuteTrajectory goal was accepted).  Cancel and
            # safety handlers observe a mid-flight transaction and never send an
            # execute goal, and an execute attempt that fails before the goal
            # accept (plan fail / empty plan / construction / digest mismatch /
            # server-unavailable) never sent it either.  ``execute_goal_id``
            # existing in the execute outcome is the truthful sent marker.
            "execute_trajectory_goal_sent": bool(
                execute_outcome.get("execute_goal_id") if isinstance(execute_outcome, Mapping) else False
            ),
            "controller_goal_sent": False,
            "planning_goal_id": planning_goal_id,
            "execute_goal_id": execute_goal_id,
            "goals_canceling": list(goals_canceling) if goals_canceling is not None else [],
            "planned_trajectory_digest": plan_outcome.get("trajectory_digest"),
            "executed_trajectory_digest": executed_digest,
            "fjt_goal_digest": trajectory_digest,
            "fjt_goal_uuid": fjt_evidence.get("goal_uuid") if isinstance(fjt_evidence, Mapping) else None,
            "fjt_status": fjt_evidence.get("status") if isinstance(fjt_evidence, Mapping) else None,
            "execute_result_status": result_status,
            "execute_result_status_string": result_status_string,
            "terminal_status": terminal_status,
            "event_log": list(event_log),
            "elapsed_s": round(time.monotonic() - start_wall, 6),
            "teardown": teardown_status,
            "graph": graph_status,
            "isaac_joint_commands_published": False,
        }
        if execute_error is not None:
            record["execute_error"] = execute_error
        try:
            self._write_d_artifacts(scenario_id, spec, record, readiness)
        except Exception as exc:
            record["status"] = "evidence-invalid"
            record["reason_code"] = "artifact-write-failed"
            record["artifact_error"] = str(exc)
            self._downgrade_persisted_d_evidence(scenario_id, record, readiness, final_status)
        return record

    def _write_d_artifacts(
        self,
        scenario_id: str,
        spec: Mapping[str, object],
        record: Mapping[str, object],
        readiness: Mapping[str, object],
    ) -> None:
        """Write the D-stage artifact rows (separate shape; Gate-C bytes unchanged)."""
        self._append_jsonl(
            self.attempt_dir / "integrated-execution.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "event": "gate-d",
                "stage": "D",
                "handler": spec.get("kind"),
                "polarity": spec.get("polarity"),
                "status": record.get("status"),
                "reason_code": record.get("reason_code"),
                "planner_status": record.get("planner_status"),
                "row_kind": "lifecycle",
                "diagnostic_only": True,
                "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                "controller_goal_sent": record.get("controller_goal_sent"),
                "isaac_joint_commands_published": False,
                "timestamp": float(time.monotonic()),
            },
        )
        self._append_jsonl(
            self.attempt_dir / "moveit-plans.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "goal_kind": spec.get("kind"),
                "status": record.get("status"),
                "planner_status": record.get("planner_status"),
                "row_kind": "lifecycle",
                "planning_goal_id": record.get("planning_goal_id"),
                "execute_goal_id": record.get("execute_goal_id"),
                "trajectory_digest": record.get("planned_trajectory_digest"),
                "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                "diagnostic_only": True,
            },
        )
        self._append_jsonl(
            self.attempt_dir / "controller-results.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "controller_goal_sent": record.get("controller_goal_sent"),
                "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                "execute_result_status": record.get("execute_result_status"),
                "execute_result_status_string": record.get("execute_result_status_string"),
                "fjt_goal_uuid": record.get("fjt_goal_uuid"),
                "fjt_status": record.get("fjt_status"),
                "fjt_goal_digest": record.get("fjt_goal_digest"),
                "diagnostic_only": True,
            },
        )
        self._write_json_atomic(
            self.attempt_dir / "integrated-execution.json",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "stage": "D",
                "handler": spec.get("kind"),
                "polarity": spec.get("polarity"),
                "diagnostic_only": True,
                "physical_verdict": None,
                "status": record.get("status"),
                "reason_code": record.get("reason_code"),
                "planner_status": record.get("planner_status"),
                "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                "controller_goal_sent": record.get("controller_goal_sent"),
                "planning_goal_id": record.get("planning_goal_id"),
                "execute_goal_id": record.get("execute_goal_id"),
                "goals_canceling": record.get("goals_canceling"),
                "planned_trajectory_digest": record.get("planned_trajectory_digest"),
                "executed_trajectory_digest": record.get("executed_trajectory_digest"),
                "fjt_goal_digest": record.get("fjt_goal_digest"),
                "fjt_goal_uuid": record.get("fjt_goal_uuid"),
                "fjt_status": record.get("fjt_status"),
                "execute_result_status": record.get("execute_result_status"),
                "execute_result_status_string": record.get("execute_result_status_string"),
                "terminal_status": record.get("terminal_status"),
                "event_log": record.get("event_log"),
                "elapsed_s": record.get("elapsed_s"),
                "isaac_joint_commands_published": False,
            },
        )

    def _write_d_fail_dominant_execution_json(
        self,
        scenario_id: str,
        record: Mapping[str, object],
        readiness: Mapping[str, object],
        *,
        downgraded_from: object,
    ) -> None:
        self._write_json_atomic(
            self.attempt_dir / "integrated-execution.json",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "stage": "D",
                "diagnostic_only": True,
                "physical_verdict": None,
                "status": "evidence-invalid",
                "reason_code": record.get("reason_code", "artifact-write-failed"),
                "reasons": [str(record.get("artifact_error") or "D artifact final output failed")],
                "planner_status": record.get("planner_status"),
                "readiness": {
                    "ready": readiness.get("ready", False),
                    "reasons": readiness.get("reasons", []),
                },
                "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                "controller_goal_sent": record.get("controller_goal_sent"),
                "downgraded_from": downgraded_from,
                "isaac_joint_commands_published": False,
            },
        )

    def _downgrade_persisted_d_evidence(
        self,
        scenario_id: str,
        record: Mapping[str, object],
        readiness: Mapping[str, object],
        downgraded_from: object,
    ) -> None:
        """F3.1 analog: after a D artifact write failure, downgrade every already
        created status-bearing D artifact to evidence-invalid."""
        try:
            self._write_d_fail_dominant_execution_json(
                scenario_id, record, readiness, downgraded_from=downgraded_from
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "integrated-execution.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "event": "gate-d",
                    "stage": "D",
                    "status": "evidence-invalid",
                    "reason_code": record.get("reason_code"),
                    "planner_status": record.get("planner_status"),
                    "row_kind": "final",
                    "downgraded_from": downgraded_from,
                    "error": record.get("artifact_error"),
                    "diagnostic_only": True,
                    "execute_trajectory_goal_sent": record.get("execute_trajectory_goal_sent"),
                    "controller_goal_sent": record.get("controller_goal_sent"),
                    "isaac_joint_commands_published": False,
                    "timestamp": float(time.monotonic()),
                },
            )
        except Exception:
            pass

    def _acquire_scene(self, scenario_id: str) -> dict[str, object] | None:
        """F2.5/F3.2: bounded pre-goal scene acquisition through the private spinner.

        Acquisition requires a valid observation received after the last
        normalization failure (proved by ``scene_sequence`` exceeding the last
        invalid sequence), so a stale pre-invalid cached scene is never used.
        When the newest observation is invalid the executor spins up to the
        finite timeout to try to recover with a newer valid scene; timeout with
        no valid-after-invalid observation fails closed with zero goals.
        """
        timeout_s = float(self._thresholds().get("scene_acquire_timeout_s", 5.0))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if not self._planning_scene_invalid:
                scene = self._latest_planning_scene
                if scene is not None and (
                    self._scene_invalid_sequence is None
                    or int(scene["scene_sequence"]) > self._scene_invalid_sequence
                ):
                    return None
            self._spin_once()
        if self._planning_scene_invalid:
            return self._evidence_invalid(
                scenario_id,
                "planning-scene-invalid",
                [
                    "a received PlanningScene failed normalization and no valid "
                    "scene arrived after it within the acquisition timeout"
                ],
            )
        return self._evidence_invalid(
            scenario_id,
            "no-planning-scene",
            [f"no valid PlanningScene cached within {timeout_s:.3f}s of self-spin"],
        )

    def _fixture_scene_error(self, scene: Mapping[str, object]) -> str | None:
        """F2.6/F3.3: return a reason when *scene* does not match the fixture contract.

        Beyond the exact ordered owned-ID set and empty attached set, the scene
        must carry the exact declared fixture geometry projection digest
        (primitive/mesh geometry, dimensions/scales, frame, and poses), so a
        stale full scene with the same IDs but an old cube pose is never
        labeled fixture-ready.
        """
        declaration = _as_mapping(
            self.scenario.get("planning_scene_declaration") or self.scenario.get("planning_scene")
        )
        expected_ids = list(fixture_owned_ids(declaration))
        owned_ids = list(scene.get("owned_ids", []))
        if owned_ids != expected_ids:
            return (
                "fixture-ready owned_ids must equal the declared ordered fixture ids: "
                f"scene {owned_ids} != declared {expected_ids}"
            )
        attached_ids = list(scene.get("attached_ids", []))
        if attached_ids:
            return f"fixture-ready scene must not carry attached objects: {attached_ids}"
        expected_digest = self._expected_fixture_geometry_digest()
        observed_digest = scene.get("fixture_geometry_digest")
        if expected_digest is None or observed_digest != expected_digest:
            return (
                "fixture-ready scene geometry/pose must match the declared fixture "
                f"projection: scene {observed_digest} != declared {expected_digest}"
            )
        return None

    def _append_visual_request(
        self, phase: str, scenario_id: str, spec: Mapping[str, object]
    ) -> None:
        """F2.7: durably append one visual-capture request record at the truthful phase."""
        self._append_jsonl(
            self.attempt_dir / "visual-capture-requests.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "phase": phase,
                "capture": {"kind": "plan-only", "target": spec.get("target_pose")},
                "diagnostic_only": True,
            },
        )

    def _finalize_failure_artifact(self, reason: str, graph_diagnosis: str) -> str:
        """F2.1: write planning-scene.json as a canonical failure artifact."""
        try:
            self.journal.finalize_failure(
                reason=reason,
                graph_diagnosis=graph_diagnosis,
                json_path=self.attempt_dir / "planning-scene.json",
            )
            return "written"
        except Exception as exc:
            return f"failed: {exc}"

    def _evidence_invalid_after_fixture_ready(
        self,
        scenario_id: str,
        exc: Exception,
        spec: Mapping[str, object],
        readiness: Mapping[str, object],
    ) -> dict[str, object]:
        """F2.2: complete evidence for a failure after fixture-ready was recorded."""
        teardown_status = "not-recorded"
        later_join = self._join_key()
        if later_join is not None:
            try:
                self.journal.snapshot(
                    "teardown", frame_index=later_join[0], timestamp=later_join[1]
                )
                teardown_status = "recorded"
            except (ValueError, TypeError) as exc2:
                teardown_status = f"rejected: {exc2}"
        reason = f"unexpected-exception: {exc}"
        graph_status = "unavailable"
        self._finalize_failure_artifact(reason, graph_status)
        record = {
            "scenario_id": scenario_id,
            "status": "evidence-invalid",
            "reason_code": "unexpected-exception",
            "reasons": [reason],
            "planner_status": None,
            "teardown": teardown_status,
            "graph": graph_status,
            "goal_digest": None,
            "diagnostic_only": True,
            "execute_trajectory_goal_sent": False,
            "isaac_joint_commands_published": False,
        }
        try:
            self._write_artifacts(scenario_id, spec, None, record, readiness, graph_status)
        except Exception:
            # F2.2: artifact output must never escape; fall back to the durable
            # fail-dominant summary only.
            try:
                self._write_fail_dominant_execution_json(
                    scenario_id,
                    record,
                    readiness,
                    graph_status,
                    planner_status=None,
                    reason="artifact output failed after an unexpected exception",
                )
            except Exception:
                pass
        return record

    def _fixture_payload_for_graph(self) -> str:
        if self._fixture_payload is not None:
            return self._fixture_payload
        raise ValueError(
            "no canonical fixture payload was cached before journal finalization"
        )

    # -- artifacts -----------------------------------------------------------

    def _append_jsonl(self, path: Path, record: Mapping[str, object]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(dict(record), sort_keys=True, separators=(",", ":")) + "\n")
            stream.flush()
            os.fsync(stream.fileno())

    def _evidence_invalid(
        self, scenario_id: str, reason_code: str, reasons: Sequence[str]
    ) -> dict[str, object]:
        record = {
            "scenario_id": scenario_id,
            "status": "evidence-invalid",
            "reason_code": reason_code,
            "reasons": list(reasons),
            "diagnostic_only": True,
            "execute_trajectory_goal_sent": False,
            "isaac_joint_commands_published": False,
        }
        try:
            self._append_jsonl(
                self.attempt_dir / "integrated-execution.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "event": "gate-c-plan-only",
                    "status": "evidence-invalid",
                    "reason_code": reason_code,
                    "reasons": list(reasons),
                    "diagnostic_only": True,
                    "timestamp": float(time.monotonic()),
                },
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "controller-results.jsonl",
                {
                    "scenario_id": scenario_id,
                    "controller_goal_sent": False,
                    "execute_trajectory_goal_sent": False,
                    "diagnostic_only": True,
                },
            )
        except Exception:
            pass
        try:
            self._write_json_atomic(
                self.attempt_dir / "integrated-execution.json",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "diagnostic_only": True,
                    "status": "evidence-invalid",
                    "reason_code": reason_code,
                    "reasons": list(reasons),
                    "execute_trajectory_goal_sent": False,
                    "isaac_joint_commands_published": False,
                    "physical_verdict": None,
                },
            )
        except Exception:
            pass
        return record

    def _write_json_atomic(self, path: Path, value: Mapping[str, object]) -> None:
        _atomic_write_json(value, path)

    def _write_fail_dominant_execution_json(
        self,
        scenario_id: str,
        record: Mapping[str, object],
        readiness: Mapping[str, object],
        graph_status: str,
        *,
        planner_status: str | None,
        reason: str,
    ) -> None:
        """F2.1: durable fail-dominant execution summary when artifact output fails."""
        self._write_json_atomic(
            self.attempt_dir / "integrated-execution.json",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "diagnostic_only": True,
                "status": "evidence-invalid",
                "reason_code": record.get("reason_code", "artifact-write-failed"),
                "reasons": [reason],
                "planner_status": planner_status,
                "readiness": {
                    "ready": readiness.get("ready", False),
                    "reasons": readiness.get("reasons", []),
                },
                "graph": graph_status,
                "execute_trajectory_goal_sent": False,
                "isaac_joint_commands_published": False,
                "physical_verdict": None,
            },
        )

    def _downgrade_persisted_evidence(
        self,
        scenario_id: str,
        record: Mapping[str, object],
        readiness: Mapping[str, object],
        graph_status: str,
        planner_status: str | None,
        *,
        downgraded_from: object,
        goal_kind: object,
    ) -> None:
        """F3.1: after an artifact write fails post-provisional-pass, downgrade
        every already-created status-bearing artifact to evidence-invalid.

        Rewrites the atomic execution summary fail-dominantly, appends a
        corrective ``row_kind="final"`` row to each JSONL lifecycle stream so an
        early provisional row can never be mistaken for a completed pass, and
        writes planning-scene.json as a canonical failure artifact.  Every step
        is individually contained so no downgrade failure can escape the API.
        """
        try:
            self._write_fail_dominant_execution_json(
                scenario_id,
                record,
                readiness,
                graph_status,
                planner_status=planner_status,
                reason=str(record.get("artifact_error") or "artifact final output failed"),
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "integrated-execution.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "event": "gate-c-plan-only",
                    "status": "evidence-invalid",
                    "reason_code": record.get("reason_code"),
                    "planner_status": planner_status,
                    "row_kind": "final",
                    "downgraded_from": downgraded_from,
                    "error": record.get("artifact_error"),
                    "diagnostic_only": True,
                    "execute_trajectory_goal_sent": False,
                    "isaac_joint_commands_published": False,
                    "timestamp": float(time.monotonic()),
                },
            )
        except Exception:
            pass
        try:
            self._append_jsonl(
                self.attempt_dir / "moveit-plans.jsonl",
                {
                    "schema_version": 1,
                    "report_revision": REPORT_REVISION,
                    "scenario_id": scenario_id,
                    "goal_kind": goal_kind,
                    "status": "evidence-invalid",
                    "planner_status": planner_status,
                    "row_kind": "final",
                    "downgraded_from": downgraded_from,
                    "error_code": record.get("error_code"),
                    "error_code_classification": record.get("error_code_classification"),
                    "nonempty_plan": record.get("nonempty_plan"),
                    "goal_digest": record.get("goal_digest"),
                    "trajectory_digest": record.get("trajectory_digest"),
                    "error": record.get("artifact_error"),
                    "diagnostic_only": True,
                },
            )
        except Exception:
            pass
        try:
            if self.journal.record_count > 0:
                self._finalize_failure_artifact(
                    str(record.get("artifact_error") or "artifact final output failed"),
                    graph_status,
                )
        except Exception:
            pass

    def _write_artifacts(
        self,
        scenario_id: str,
        spec: Mapping[str, object],
        goal: Any,
        record: Mapping[str, object],
        readiness: Mapping[str, object],
        graph_status: str,
    ) -> None:
        goal_digest = record.get("goal_digest")
        self._append_jsonl(
            self.attempt_dir / "integrated-execution.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "event": "gate-c-plan-only",
                "status": record.get("status"),
                "reason_code": record.get("reason_code"),
                "planner_status": record.get("planner_status"),
                "row_kind": "lifecycle",
                "diagnostic_only": True,
                "readiness": {
                    "ready": readiness.get("ready", False),
                    "reasons": readiness.get("reasons", []),
                },
                "graph": graph_status,
                "execute_trajectory_goal_sent": False,
                "isaac_joint_commands_published": False,
                "timestamp": float(time.monotonic()),
            },
        )
        self._append_jsonl(
            self.attempt_dir / "moveit-plans.jsonl",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "goal_kind": spec.get("kind"),
                "status": record.get("status"),
                "planner_status": record.get("planner_status"),
                "row_kind": "lifecycle",
                "error_code": record.get("error_code"),
                "error_code_classification": record.get("error_code_classification"),
                "nonempty_plan": record.get("nonempty_plan"),
                "goal_digest": goal_digest,
                "trajectory_digest": record.get("trajectory_digest"),
                "diagnostic_only": True,
            },
        )
        self._append_jsonl(
            self.attempt_dir / "controller-results.jsonl",
            {
                "scenario_id": scenario_id,
                "controller_goal_sent": False,
                "execute_trajectory_goal_sent": False,
                "diagnostic_only": True,
            },
        )
        goal_path = self.attempt_dir / "goals" / f"{scenario_id}.json"
        goal_path.parent.mkdir(parents=True, exist_ok=True)
        _atomic_write_json(
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "kind": spec.get("kind"),
                "group_name": "xarm7",
                "pipeline_id": "ompl",
                "num_planning_attempts": 3,
                "allowed_planning_time": 3.0,
                "plan_only": True,
                "replan": False,
                "joints": spec.get("joints"),
                "target_pose": spec.get("target_pose"),
                "goal_digest": goal_digest,
                "diagnostic_only": True,
            },
            goal_path,
        )
        self._write_json_atomic(
            self.attempt_dir / "integrated-execution.json",
            {
                "schema_version": 1,
                "report_revision": REPORT_REVISION,
                "scenario_id": scenario_id,
                "diagnostic_only": True,
                "status": record.get("status"),
                "reason_code": record.get("reason_code"),
                "planner_status": record.get("planner_status"),
                "readiness": {
                    "ready": readiness.get("ready", False),
                    "reasons": readiness.get("reasons", []),
                },
                "goal": {
                    "kind": spec.get("kind"),
                    "group_name": "xarm7",
                    "pipeline_id": "ompl",
                    "num_planning_attempts": 3,
                    "allowed_planning_time": 3.0,
                    "plan_only": True,
                    "replan": False,
                    "goal_digest": goal_digest,
                },
                "result": {
                    "error_code": record.get("error_code"),
                    "error_code_classification": record.get("error_code_classification"),
                    "nonempty_plan": record.get("nonempty_plan"),
                    "trajectory_digest": record.get("trajectory_digest"),
                },
                "journal": {
                    "jsonl": str(self.attempt_dir / "planning-scene.jsonl"),
                    "json": str(self.attempt_dir / "planning-scene.json"),
                },
                "graph": graph_status,
                "execute_trajectory_goal_sent": False,
                "isaac_joint_commands_published": False,
                "physical_verdict": None,
            },
        )

    # -- teardown ------------------------------------------------------------

    def shutdown(self) -> None:
        """Idempotently destroy the node and shut down the executor-owned context."""
        if not self._context_initialized:
            return
        try:
            self._spinner.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            self.ros["rclpy"].shutdown(context=self.context)
        except Exception:
            pass
        self._context_initialized = False


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
    "ARTIFACT_JSONL_FILES",
    "CARTESIAN_MOVE_ENDPOINT",
    "CONTROLLER_MANAGER_NODE",
    "DIGEST",
    "D_FORBIDDEN_EVENTS",
    "EARLIER_OPERATION_OPTIONAL_FIELDS",
    "EXECUTE_STATUS_ABORTED",
    "EXECUTE_STATUS_CANCELED",
    "EXECUTE_STATUS_SUCCEEDED",
    "EXECUTE_TRAJECTORY_ENDPOINT",
    "FINAL_SIMULATION_STATE",
    "FIXTURE_OWNER",
    "FIXTURE_PUBLISHER_NODE",
    "FIXTURE_TARGET_HANDOFF",
    "FIXTURE_TOPIC",
    "FJT_ENDPOINT",
    "FJT_STATUS_CACHE_LIMIT",
    "FJT_STATUS_TOPIC",
    "GATE_C_FORBIDDEN_EVENTS",
    "GATE_C_REQUIRED_EVENT_ORDER",
    "GRIPPER_CLOSE_POSITION",
    "GRIPPER_ENDPOINT",
    "GRIPPER_MAX_EFFORT",
    "GRIPPER_OPEN_POSITION",
    "IDENTITY_KEYS",
    "INTEGRATED_EXECUTION_PROFILE",
    "ISAAC_COMMAND_TOPIC",
    "JOINT_STATES_TOPIC",
    "JOURNAL_FIXTURE_TOPIC_QOS",
    "JOURNAL_PLANNING_SCENE_TOPIC_QOS",
    "JOURNAL_SERVICE_QOS",
    "JOURNAL_TOPIC_QOS",
    "MOVEIT_PLANNING_NON_SUCCESS_CODES",
    "MOVEIT_SUCCESS_CODE",
    "MOVE_GROUP_NODE",
    "NODE_BASENAME",
    "OPERATION_KEYS",
    "OPERATOR_NODE",
    "OPERATOR_NODE_NAMESPACE",
    "OPERATOR_TOPIC",
    "PHYSICS_READY_BOUNDARY",
    "PHYSICS_READY_GATE_NODE",
    "Q_OUTBOUND",
    "REPORT_KEYS",
    "REPORT_REVISION",
    "REQUIRED_ACTIONS",
    "REQUIRED_SERVICES",
    "REQUIRED_TOPICS",
    "RETREAT_AXIS",
    "RETREAT_DISTANCE_M",
    "RMW_IMPLEMENTATION",
    "SAFETY_STOP_TOPIC",
    "SAFETY_SUPERVISOR_NODE",
    "SIMULATION_STATE_PLAYING",
    "STAGE_C_SCENARIOS",
    "STAGE_D_EXPECTED_PHYSICAL",
    "STAGE_D_EXPECTED_POLARITY",
    "STAGE_D_KIND",
    "STAGE_D_REQUIRED_EVENT_ORDER",
    "STAGE_D_SCENARIOS",
    "STAGE_E_CANCEL_NEGATIVES",
    "TARGET_OBJECT_ID",
    "TASK_NAMESPACE",
    "IntegratedGateExecutor",
    "build_cartesian_move_goal",
    "build_execute_trajectory_goal",
    "build_gripper_goal",
    "build_joint_move_group_goal",
    "build_journal_graph_projection",
    "build_pick_goal",
    "build_place_goal",
    "build_pose_move_group_goal",
    "derive_retreat_target_pose",
    "deterministic_cube_cloud",
    "evaluate_executor_readiness",
    "expected_fixture_geometry_digest",
    "expected_physics_ready_report",
    "run_pick_place_negative",
    "stage_c_dispatch",
    "stage_d_dispatch",
    "validate_physics_ready_snapshot",
]
