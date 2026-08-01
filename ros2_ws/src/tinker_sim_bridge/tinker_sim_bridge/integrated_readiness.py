"""ROS-free canonical scenario-report schema and integrated readiness evaluator.

This module defines the shared canonical ``scenario-runner.json`` report schema,
the deterministic mapping/digest helpers used by the launch, the scenario
runner, the physics-ready gate, and the integrated readiness node, and the pure
fail-closed readiness evaluator :func:`evaluate_integrated_readiness`.  It
imports neither ROS, Isaac Sim, nor simulator-extension packages and runs under
both simulator CPython 3.12 and system Humble CPython 3.10.

The canonical report schema (``report_revision`` ``"integrated-manipulation-v1"``)
carries the exact scenario, planning-scene, and full integrated mappings together
with an ``identities`` object whose digests agree with those mappings, the
operation list whose final ``PHYSICS_READY`` operation carries the same
identities, and ``final_simulation_state="STATE_PLAYING"``.  ``parse_canonical_report``
and :func:`validate_report` evaluate a parsed report fail-closed; the
readiness evaluator consumes a complete observation snapshot and an expected
contract and returns an immutable :class:`ReadinessReport`.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from typing import Mapping, Sequence

from .fixture_planning_scene import fixture_owned_ids

REPORT_SCHEMA_VERSION = 1
REPORT_REVISION = "integrated-manipulation-v1"
FINAL_SIMULATION_STATE = "STATE_PLAYING"
PHYSICS_READY_BOUNDARY = "PHYSICS_READY"
SIMULATION_STATE_PLAYING = 1
INTEGRATED_EXECUTION_PROFILE = "sim_ompl"

REPORT_REQUIRED_TOP_LEVEL = frozenset(
    {
        "schema_version",
        "report_revision",
        "scenario",
        "planning_scene",
        "integrated",
        "identities",
        "operations",
        "final_simulation_state",
    }
)
IDENTITIES_KEYS = frozenset(
    {
        "scenario_id",
        "seed",
        "scenario_declaration_sha256",
        "planning_scene_sha256",
        "integrated_sha256",
        "model_fingerprint",
        "provider_manifest_sha256",
    }
)
PLANNING_SCENE_REPORT_KEYS = (
    "revision",
    "owned_ids",
    "target_source_id",
    "target_handoff",
)

# Canonical integrated action endpoints (exactly one server each).
INTEGRATED_ACTIONS: Mapping[str, Mapping[str, str]] = {
    "/move_action": {
        "type": "moveit_msgs/action/MoveGroup",
        "source": "/move_group",
    },
    "/execute_trajectory": {
        "type": "moveit_msgs/action/ExecuteTrajectory",
        "source": "/move_group",
    },
    "/xarm_gripper/gripper_action": {
        "type": "control_msgs/action/GripperCommand",
        "source": "/tinker_sim_gripper_facade",
    },
    "/xarm7_traj_controller/follow_joint_trajectory": {
        "type": "control_msgs/action/FollowJointTrajectory",
        "source": "controller_resource:xarm7_traj_controller",
    },
    "/pickup_action": {
        "type": "tinker_arm_msgs/action/Pick",
        "source": "/pick_and_place",
    },
    "/place_action": {
        "type": "tinker_arm_msgs/action/Place",
        "source": "/pick_and_place",
    },
    "/cartesian_move_action": {
        "type": "tinker_arm_msgs/action/CartesianMove",
        "source": "/pick_and_place",
    },
    "/joint_move_action": {
        "type": "tinker_arm_msgs/action/JointMove",
        "source": "/pick_and_place",
    },
    "/fold_action": {
        "type": "tinker_arm_msgs/action/Fold",
        "source": "/pick_and_place",
    },
}

# Canonical integrated service endpoints (exactly one server each).
INTEGRATED_SERVICES: Mapping[str, Mapping[str, str]] = {
    "/controller_manager/list_controllers": {
        "type": "controller_manager_msgs/srv/ListControllers",
        "source": "/controller_manager",
    },
    "/controller_manager/load_controller": {
        "type": "controller_manager_msgs/srv/LoadController",
        "source": "/controller_manager",
    },
    "/controller_manager/configure_controller": {
        "type": "controller_manager_msgs/srv/ConfigureController",
        "source": "/controller_manager",
    },
    "/controller_manager/switch_controller": {
        "type": "controller_manager_msgs/srv/SwitchController",
        "source": "/controller_manager",
    },
    "/get_planning_scene": {
        "type": "moveit_msgs/srv/GetPlanningScene",
        "source": "/move_group",
    },
    "/apply_planning_scene": {
        "type": "moveit_msgs/srv/ApplyPlanningScene",
        "source": "/move_group",
    },
    "/check_state_validity": {
        "type": "moveit_msgs/srv/GetStateValidity",
        "source": "/move_group",
    },
    "/compute_cartesian_path": {
        "type": "moveit_msgs/srv/GetCartesianPath",
        "source": "/move_group",
    },
    "/arm_joint_service": {
        "type": "tinker_arm_msgs/srv/ArmJointService",
        "source": "/pick_and_place",
    },
    "/sim/ready/physics": {
        "type": "std_srvs/srv/Trigger",
        "source": "/tinker_sim_physics_ready_gate",
    },
    "/sim/ready/fixture": {
        "type": "std_srvs/srv/Trigger",
        "source": "/fixture_planning_scene",
    },
}

# Canonical integrated publisher endpoints and their freshness/stamp/QoS contracts.
INTEGRATED_PUBLISHERS: Mapping[str, Mapping[str, object]] = {
    "/joint_states": {
        "type": "sensor_msgs/msg/JointState",
        "source": "/controller_manager",
        "logical_resource": "joint_state_broadcaster",
        "cardinality": 1,
        "max_age_s": 0.25,
        "durability": "VOLATILE",
        "reliability": "RELIABLE",
        "depth": 10,
    },
    "/isaac_joint_commands": {
        "type": "sensor_msgs/msg/JointState",
        "source": "/tinker_sim_command_gateway",
        "cardinality": 1,
        "max_age_s": None,
        "durability": "VOLATILE",
        "reliability": "RELIABLE",
        "depth": 10,
    },
    "/sim/controller/gripper_commands": {
        "type": "sensor_msgs/msg/JointState",
        "source": "/tinker_sim_gripper_facade",
        "cardinality": 1,
        "max_age_s": None,
        "durability": "VOLATILE",
        "reliability": "RELIABLE",
        "depth": 20,
    },
    "/sim/safety/operator": {
        "type": "std_msgs/msg/Bool",
        "source": "/tinker_integrated_gate_executor",
        "cardinality": 1,
        "max_age_s": 0.25,
        "durability": "TRANSIENT_LOCAL",
        "reliability": "RELIABLE",
        "depth": 1,
        "clear_value": False,
    },
    "/sim/hardware/safety_stop": {
        "type": "std_msgs/msg/Bool",
        "source": "/tinker_sim_safety_supervisor",
        "cardinality": 1,
        "max_age_s": 0.25,
        "durability": "TRANSIENT_LOCAL",
        "reliability": "RELIABLE",
        "depth": 1,
        "min_samples": 2,
        "clear_value": False,
    },
    "/sim/status/planning_scene_fixture": {
        "type": "std_msgs/msg/String",
        "source": "/fixture_planning_scene",
        "cardinality": 1,
        "max_age_s": 0.25,
        "durability": "TRANSIENT_LOCAL",
        "reliability": "RELIABLE",
        "depth": 1,
    },
    "/sim/status/integrated_manipulation": {
        "type": "std_msgs/msg/String",
        "source": "/integrated_readiness",
        "cardinality": 1,
        "max_age_s": 0.25,
        "durability": "TRANSIENT_LOCAL",
        "reliability": "RELIABLE",
        "depth": 1,
    },
}

INTEGRATED_JOINT_STATE_NAMES = tuple(f"joint{index}" for index in range(1, 8)) + (
    "drive_joint",
)
INTEGRATED_TOUCH_LINKS = (
    "xarm_gripper_base_link",
    "left_outer_knuckle",
    "left_finger",
    "left_inner_knuckle",
    "right_inner_knuckle",
    "right_outer_knuckle",
    "right_finger",
    "link_tcp",
)
TF_PARENT = "base_link"
TF_CHILD = "link_tcp"


def build_integrated_mapping() -> dict[str, object]:
    """Return the canonical full integrated composition mapping.

    The mapping carries the exact typed action/service/publisher endpoint
    contract, the eight-joint state set, the eight-link touch set, the
    ``base_link -> link_tcp`` TF pair, the logical controller resources with
    their expected active state, and the final simulation state.  This is the
    full runtime readiness contract, carried separately from the public
    scenario report's one-key ``integrated`` mapping (see
    :func:`public_integrated_mapping`).  ``runtime_contract_sha256`` is the
    digest of this full mapping.
    """
    return {
        "report_revision": REPORT_REVISION,
        "actions": {endpoint: dict(spec) for endpoint, spec in INTEGRATED_ACTIONS.items()},
        "services": {endpoint: dict(spec) for endpoint, spec in INTEGRATED_SERVICES.items()},
        "publishers": {
            name: dict(spec) for name, spec in INTEGRATED_PUBLISHERS.items()
        },
        "joint_names": list(INTEGRATED_JOINT_STATE_NAMES),
        "touch_links": list(INTEGRATED_TOUCH_LINKS),
        "tf": {"parent": TF_PARENT, "child": TF_CHILD},
        "controller_resources": {
            "joint_state_broadcaster": "active",
            "xarm7_traj_controller": "active",
        },
        "final_simulation_state": FINAL_SIMULATION_STATE,
    }


def public_integrated_mapping(
    execution_profile: str = INTEGRATED_EXECUTION_PROFILE,
) -> dict[str, str]:
    """Return the production-canonical public report ``integrated`` mapping.

    The shipped ``pick_and_place`` canonical parser requires the report's
    ``integrated`` field to be exactly ``{"execution_profile": "sim_ompl"}``
    with no other keys.  The full typed runtime contract
    (:func:`build_integrated_mapping`) is carried separately by the physics
    gate / readiness / provider evidence and never enters the public report.
    """
    return {"execution_profile": str(execution_profile)}


def normalize_qos_value(value) -> str:
    """Normalize a Humble QoS enum value to its short uppercase name.

    ``str()`` on rclpy Humble enums yields ``"ReliabilityPolicy.RELIABLE"`` /
    ``"DurabilityPolicy.TRANSIENT_LOCAL"``.  Normalizing to the final segment
    (``"RELIABLE"`` / ``"TRANSIENT_LOCAL"``) makes observed and expected values
    comparable deterministically.
    """
    text = str(value)
    if "." in text:
        text = text.rsplit(".", 1)[-1]
    return text.strip().upper()


class ReportValidationError(ValueError):
    """Typed failure raised when a canonical report cannot be parsed."""


def canonical_json(value) -> bytes:
    """Return compact canonical JSON bytes (sorted keys, minimal separators)."""
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")


def sha256_json(value) -> str:
    """Return the lowercase SHA-256 of the canonical JSON bytes of *value*."""
    return hashlib.sha256(canonical_json(value)).hexdigest()


def sha256_bytes(data: bytes) -> str:
    """Return the lowercase SHA-256 of raw *data* bytes."""
    return hashlib.sha256(data).hexdigest()


def scenario_mapping(
    scenario_id: str, seed: int, declaration: Mapping[str, object]
) -> Mapping[str, object]:
    """Build the canonical report ``scenario`` mapping.

    The declaration is the public scenario spec without its identity keys; the
    mapping keeps ``id``/``seed`` at the top level so ``scenario_declaration_sha256``
    is stable and self-contained.
    """
    return {"id": str(scenario_id), "seed": int(seed), "declaration": dict(declaration)}


def planning_scene_mapping(planning_scene: Mapping[str, object]) -> Mapping[str, object]:
    """Build the canonical report ``planning_scene`` mapping.

    Only the shared identity keys are retained (the digest is computed over this
    same subset).  ``owned_ids`` is derived through the authoritative Task 5
    ``fixture_owned_ids`` helper in declaration order, never read from a
    nonexistent top-level key.
    """
    return {
        "revision": str(planning_scene["revision"]),
        "owned_ids": [str(item) for item in fixture_owned_ids(planning_scene)],
        "target_source_id": str(planning_scene["target_source_id"]),
        "target_handoff": str(planning_scene["target_handoff"]),
    }


def planning_scene_digest(planning_scene: Mapping[str, object]) -> str:
    """Return the canonical digest of a planning-scene declaration.

    The digest covers the full declaration with any ``revision_digest`` key
    excluded, matching ``fixture_contract.revision_digest`` and the report
    ``planning_scene_sha256``.
    """
    payload = {
        key: value
        for key, value in planning_scene.items()
        if key != "revision_digest"
    }
    return sha256_json(payload)


def report_identities(
    *,
    scenario_id: str,
    seed: int,
    declaration: Mapping[str, object],
    planning_scene: Mapping[str, object],
    integrated: Mapping[str, object],
    model_fingerprint: str,
    provider_manifest_sha256: str,
) -> Mapping[str, object]:
    """Build the canonical report ``identities`` mapping.

    Every digest agrees with the corresponding report mapping, so a consumer can
    recompute and compare each identity from the unchanged report fields.
    """
    scenario = scenario_mapping(scenario_id, seed, declaration)
    plan = planning_scene_mapping(planning_scene)
    return {
        "scenario_id": str(scenario_id),
        "seed": int(seed),
        "scenario_declaration_sha256": sha256_json(scenario),
        "planning_scene_sha256": sha256_json(plan),
        "integrated_sha256": sha256_json(dict(integrated)),
        "model_fingerprint": str(model_fingerprint),
        "provider_manifest_sha256": str(provider_manifest_sha256),
    }


def _final_operation(
    identities: Mapping[str, object],
) -> Mapping[str, object]:
    return {
        "operation": "set_simulation_state",
        "accepted": True,
        "state": SIMULATION_STATE_PLAYING,
        "boundary": PHYSICS_READY_BOUNDARY,
        **dict(identities),
    }


def build_canonical_report(
    *,
    scenario_id: str,
    seed: int,
    declaration: Mapping[str, object],
    planning_scene: Mapping[str, object],
    integrated: Mapping[str, object],
    operations: Sequence[Mapping[str, object]],
    model_fingerprint: str,
    provider_manifest_sha256: str,
    final_simulation_state: str = FINAL_SIMULATION_STATE,
) -> Mapping[str, object]:
    """Build the complete canonical shared scenario report mapping.

    The returned mapping's ``identities`` digests agree with its scenario,
    planning-scene, and integrated mappings, and its final operation is the
    accepted ``PHYSICS_READY`` boundary carrying the same identities.
    """
    identities = report_identities(
        scenario_id=scenario_id,
        seed=seed,
        declaration=declaration,
        planning_scene=planning_scene,
        integrated=integrated,
        model_fingerprint=model_fingerprint,
        provider_manifest_sha256=provider_manifest_sha256,
    )
    operations_list = [dict(operation) for operation in operations]
    if operations_list:
        operations_list[-1] = {
            **operations_list[-1],
            **dict(identities),
        }
    else:
        operations_list.append(dict(_final_operation(identities)))
    return {
        "schema_version": REPORT_SCHEMA_VERSION,
        "report_revision": REPORT_REVISION,
        "scenario": scenario_mapping(scenario_id, seed, declaration),
        "planning_scene": planning_scene_mapping(planning_scene),
        "integrated": dict(integrated),
        "identities": identities,
        "operations": operations_list,
        "final_simulation_state": str(final_simulation_state),
    }


def serialize_report(report: Mapping[str, object]) -> bytes:
    """Serialize a canonical report to compact canonical JSON bytes."""
    return canonical_json(report)


def parse_canonical_report(data: bytes) -> Mapping[str, object]:
    """Parse and structurally validate canonical report bytes, fail-closed.

    Raises :class:`ReportValidationError` when the bytes are not the canonical
    integrated-manipulation report schema.
    """
    try:
        report = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ReportValidationError("report is not valid JSON: {}".format(exc)) from exc
    if not isinstance(report, dict):
        raise ReportValidationError("report must contain a JSON object")
    missing = sorted(REPORT_REQUIRED_TOP_LEVEL - set(report))
    if missing:
        raise ReportValidationError("report missing required fields {}".format(missing))
    if report.get("schema_version") != REPORT_SCHEMA_VERSION:
        raise ReportValidationError(
            "unsupported report schema_version {}".format(report.get("schema_version"))
        )
    if report.get("report_revision") != REPORT_REVISION:
        raise ReportValidationError(
            "unexpected report_revision {!r}".format(report.get("report_revision"))
        )
    scenario = report["scenario"]
    planning_scene = report["planning_scene"]
    integrated = report["integrated"]
    identities = report["identities"]
    if not isinstance(scenario, dict) or not isinstance(planning_scene, dict):
        raise ReportValidationError("report scenario/planning_scene must be objects")
    if not isinstance(integrated, dict) or not isinstance(identities, dict):
        raise ReportValidationError("report integrated/identities must be objects")
    missing_ids = sorted(IDENTITIES_KEYS - set(identities))
    if missing_ids:
        raise ReportValidationError("report identities missing {}".format(missing_ids))
    operations = report.get("operations")
    if not isinstance(operations, list) or not operations:
        raise ReportValidationError("report operations must be a nonempty array")
    if report.get("final_simulation_state") != FINAL_SIMULATION_STATE:
        raise ReportValidationError(
            "report final_simulation_state must be {}".format(FINAL_SIMULATION_STATE)
        )
    expected_plan_keys = set(PLANNING_SCENE_REPORT_KEYS)
    if set(planning_scene) != expected_plan_keys:
        raise ReportValidationError(
            "report planning_scene keys {!r} != {!r}".format(
                sorted(planning_scene), sorted(expected_plan_keys)
            )
        )
    if set(integrated) != {"execution_profile"}:
        raise ReportValidationError(
            "report integrated keys {!r} != {{'execution_profile'}}".format(
                sorted(integrated)
            )
        )
    if not isinstance(integrated.get("execution_profile"), str):
        raise ReportValidationError("report integrated execution_profile must be a string")
    return report


def _coerce_owned_ids(value: object) -> list[str]:
    """Coerce an in-memory sequence or JSON-array wire string to a string list.

    Malformed/non-string/non-``sim_fixture/*``/duplicate values are not silently
    dropped here; the caller's mapping comparison then rejects the report
    fail-closed.
    """
    if isinstance(value, str):
        try:
            parsed = json.loads(value)
        except json.JSONDecodeError:
            return []
        if not isinstance(parsed, list):
            return []
        return [str(item) for item in parsed if isinstance(item, str)]
    if isinstance(value, Sequence):
        return [str(item) for item in value]
    return []


def _expected_planning_scene_contract(
    expected: Mapping[str, object],
) -> Mapping[str, object]:
    """Extract the expected planning-scene report mapping from a contract."""
    return {
        "revision": str(expected.get("planning_scene_revision", "")),
        "owned_ids": _coerce_owned_ids(expected.get("planning_scene_owned_ids", ())),
        "target_source_id": str(expected.get("planning_scene_target_source_id", "")),
        "target_handoff": str(expected.get("planning_scene_target_handoff", "")),
    }


def validate_report(
    report: Mapping[str, object],
    expected: Mapping[str, object],
) -> dict[str, object]:
    """Fail-closed validation of a parsed report against an expected contract.

    *expected* carries the exact scenario id/seed, the declaration digest, the
    planning-scene identities, the full integrated mapping and digest, the model
    fingerprint, and the provider-manifest digest the report must match.
    """
    reasons: list[str] = []
    observed: dict[str, object] = {}
    try:
        parsed = parse_canonical_report(serialize_report(report))
    except ReportValidationError as exc:
        return {"ready": False, "reasons": ["report: {}".format(exc)], "observed": {}}
    scenario = parsed["scenario"]
    plan = parsed["planning_scene"]
    integrated = parsed["integrated"]
    identities = parsed["identities"]
    expected_plan = _expected_planning_scene_contract(expected)
    # The public report's ``integrated`` field is the production-canonical
    # one-key mapping.  The full runtime contract is carried separately (as the
    # readiness ``integrated_mapping``), so ``validate_report`` compares the
    # public field against ``public_integrated_mapping`` when supplied and falls
    # back to a caller that only provides the public mapping as
    # ``integrated_mapping`` (e.g. the scenario runner).
    expected_integrated = expected.get(
        "public_integrated_mapping", expected.get("integrated_mapping")
    )
    observed.update(
        {
            "scenario_id": scenario.get("id"),
            "seed": scenario.get("seed"),
            "planning_scene": plan,
            "integrated": integrated,
            "identities": identities,
            "final_simulation_state": parsed.get("final_simulation_state"),
        }
    )
    if scenario.get("id") != str(expected.get("scenario_id", "")):
        reasons.append(
            "report scenario id {!r} != expected {!r}".format(
                scenario.get("id"), expected.get("scenario_id")
            )
        )
    if scenario.get("seed") != int(expected.get("seed", -1)):
        reasons.append(
            "report seed {!r} != expected {!r}".format(
                scenario.get("seed"), expected.get("seed")
            )
        )
    if identities.get("scenario_declaration_sha256") != str(
        expected.get("scenario_declaration_sha256", "")
    ):
        reasons.append("report scenario_declaration_sha256 does not match expected")
    if identities.get("scenario_declaration_sha256") != sha256_json(scenario):
        reasons.append("report scenario_declaration_sha256 does not match its mapping")
    if identities.get("planning_scene_sha256") != sha256_json(plan):
        reasons.append("report planning_scene_sha256 does not match its mapping")
    if plan != expected_plan:
        reasons.append("report planning_scene mapping does not match expected")
    if identities.get("integrated_sha256") != sha256_json(integrated):
        reasons.append("report integrated_sha256 does not match its mapping")
    if (
        expected_integrated is not None
        and integrated != dict(expected_integrated)
    ):
        reasons.append("report integrated mapping does not match expected")
    if identities.get("model_fingerprint") != str(expected.get("model_fingerprint", "")):
        reasons.append("report model_fingerprint does not match expected")
    if identities.get("provider_manifest_sha256") != str(
        expected.get("provider_manifest_sha256", "")
    ):
        reasons.append("report provider_manifest_sha256 does not match expected")
    final_operation = parsed["operations"][-1]
    if not isinstance(final_operation, dict):
        reasons.append("report final operation must be an object")
    else:
        if final_operation.get("operation") != "set_simulation_state":
            reasons.append("report final operation must be set_simulation_state")
        if final_operation.get("accepted") is not True:
            reasons.append("report final operation must be accepted")
        if final_operation.get("state") != SIMULATION_STATE_PLAYING:
            reasons.append("report final operation state must be {}".format(SIMULATION_STATE_PLAYING))
        if final_operation.get("boundary") != PHYSICS_READY_BOUNDARY:
            reasons.append("report final operation boundary must be {}".format(PHYSICS_READY_BOUNDARY))
        for key in sorted(IDENTITIES_KEYS):
            if final_operation.get(key) != identities.get(key):
                reasons.append(
                    "report final operation identity {!r} does not match".format(key)
                )
    return {"ready": not reasons, "reasons": reasons, "observed": observed}


@dataclass(frozen=True)
class ReadinessReport:
    """Immutable fail-closed integrated readiness result."""

    ready: bool
    reasons: tuple[str, ...] = field(default_factory=tuple)
    evidence: Mapping[str, object] = field(default_factory=dict)


def _append_reason(reasons: list[str], prefix: str, ready: bool, check_reasons: Sequence[str]) -> None:
    if not ready:
        reasons.extend(
            "{}: {}".format(prefix, reason) if reason else prefix
            for reason in (check_reasons or ("not ready",))
        )


def _derive_goal_service_type(action_type: str) -> str:
    """Derive the canonical ``_action/send_goal`` service type for an action type.

    The production endpoint contract derives
    ``{package}/action/{Action}_SendGoal`` from ``{package}/action/{Action}``.
    """
    marker = "/action/"
    if marker not in action_type:
        return ""
    package, action = action_type.split(marker, 1)
    return "{}/action/{}_SendGoal".format(package, action)


def _check_observed_types(
    observed_types: Sequence[object], expected_type: str, endpoint: str, kind: str
) -> str | None:
    """Return a reason when *expected_type* is not the sole observed type."""
    normalized = [str(value) for value in observed_types]
    if not normalized:
        return "{} {} observed no types".format(kind, endpoint)
    if len(set(normalized)) != 1 or normalized[0] != expected_type:
        return "{} {} observed types {!r} != expected {!r}".format(
            kind, endpoint, sorted(set(normalized)), expected_type
        )
    return None


def evaluate_publisher_metadata(
    observed: Mapping[str, object],
    contract_publishers: Mapping[str, object],
) -> tuple[bool, list[str]]:
    """Fail-closed graph metadata agreement for every typed publisher endpoint.

    Compares cardinality, source, observed message type, and (when the contract
    specifies them) reliability/durability/depth against the observed publisher
    metadata.  Content/freshness is evaluated separately by the message-specific
    evidence entries.
    """
    reasons: list[str] = []
    for topic, spec in contract_publishers.items():
        entry = observed.get(topic, {})
        count = entry.get("count", 0)
        if count != spec.get("cardinality", 1):
            reasons.append(
                "publisher {} count {} != {}".format(
                    topic, count, spec.get("cardinality", 1)
                )
            )
            continue
        expected_type = str(spec.get("type", ""))
        observed_types = [str(value) for value in entry.get("types", [])]
        if expected_type and observed_types and expected_type not in observed_types:
            reasons.append(
                "publisher {} observed types {!r} do not include {!r}".format(
                    topic, sorted(set(observed_types)), expected_type
                )
            )
        expected_source = str(spec.get("source", ""))
        if expected_source and entry.get("source") != expected_source:
            reasons.append(
                "publisher {} source {!r} != {!r}".format(
                    topic, entry.get("source"), expected_source
                )
            )
        qos = entry.get("qos") or {}
        for dimension in ("reliability", "durability"):
            expected_qos = spec.get(dimension)
            if expected_qos is None:
                continue
            observed_qos = normalize_qos_value(qos.get(dimension, ""))
            if observed_qos != normalize_qos_value(str(expected_qos)):
                reasons.append(
                    "publisher {} {} {!r} != expected {!r}".format(
                        topic, dimension, observed_qos, expected_qos
                    )
                )
        expected_depth = spec.get("depth")
        if expected_depth is not None:
            observed_depth = qos.get("depth")
            try:
                depth_value = int(observed_depth)
            except (TypeError, ValueError):
                depth_value = 0
            # Humble ``PublishersInfo.qos_profile`` never reports depth/history
            # (always 0/UNKNOWN), so depth is compared only when a publisher
            # actually reports a positive value; reliability/durability are
            # reported and compared strictly above.
            if depth_value > 0 and depth_value != int(expected_depth):
                reasons.append(
                    "publisher {} depth {!r} != expected {!r}".format(
                        topic, observed_depth, expected_depth
                    )
                )
    return (not reasons, reasons)


def _provider_manifest_agreement(
    manifest: Mapping[str, object],
    observed_nodes: Sequence[str],
    observed_publishers: Sequence[str],
    observed_controllers: Mapping[str, str],
) -> list[str]:
    """Fail-closed manifest-vs-live agreement for the integrated composition.

    ``/sim/safety/operator`` is intentionally provided later by the integrated
    qualification executor (``/tinker_integrated_gate_executor``); it is not a
    Task 6 persistent provider and its readiness is enforced separately by the
    operator-input sample evidence, so it is excluded from the strict
    live-presence publisher comparison here.
    """
    reasons: list[str] = []
    live_nodes = set(observed_nodes)
    for entry in manifest.get("persistent_nodes", []):
        node = str(entry.get("node", ""))
        cardinality = int(entry.get("cardinality", 1))
        present = sum(1 for live in observed_nodes if live == node)
        if present < cardinality:
            reasons.append(
                "manifest persistent node {!r} not live (found {})".format(node, present)
            )
    for entry in manifest.get("publishers", []):
        topic = str(entry.get("topic", ""))
        if topic == "/sim/safety/operator":
            continue
        cardinality = int(entry.get("cardinality", 1))
        present = sum(1 for live in observed_publishers if live == topic)
        if present < cardinality:
            reasons.append(
                "manifest publisher {!r} not live (found {})".format(topic, present)
            )
    for entry in manifest.get("controller_resources", []):
        name = str(entry.get("resource_name", ""))
        expected_state = str(entry.get("expected_state", ""))
        state = observed_controllers.get(name)
        if state != expected_state:
            reasons.append(
                "manifest controller {!r} state {!r} != expected {!r}".format(
                    name, state, expected_state
                )
            )
    return reasons


def evaluate_integrated_readiness(
    snapshot: Mapping[str, object], contract: Mapping[str, object]
) -> ReadinessReport:
    """Evaluate a complete observation snapshot against an expected contract.

    Every entry is checked independently and fail-closed; the returned
    :class:`ReadinessReport` carries the aggregate ``ready`` flag, all reasons,
    and the complete observation evidence.
    """
    reasons: list[str] = []
    evidence: dict[str, object] = {}

    # 1. Model preflight.
    preflight = snapshot.get("model_preflight", {})
    preflight_ready = bool(preflight.get("ready"))
    evidence["model_preflight"] = preflight
    _append_reason(reasons, "model_preflight", preflight_ready, preflight.get("reasons", ()))

    # 2. Shared report PHYSICS_READY evidence.
    report_evidence = snapshot.get("shared_report", {})
    report_ready = bool(report_evidence.get("ready"))
    if report_evidence.get("scenario_report_sha256"):
        recomputed = sha256_bytes(report_evidence.get("scenario_report_sha256_bytes", b""))
        matches = report_evidence.get("scenario_report_sha256") == recomputed
    else:
        matches = bool(report_evidence.get("scenario_report_sha256_matches"))
    if not matches:
        report_ready = False
    report_reasons = list(report_evidence.get("reasons", ()))
    if not matches:
        report_reasons.append("scenario_report_sha256 does not match the report bytes")
    if report_evidence.get("final_simulation_state") != FINAL_SIMULATION_STATE:
        report_ready = False
        report_reasons.append(
            "final_simulation_state must be {}".format(FINAL_SIMULATION_STATE)
        )
    evidence["shared_report"] = report_evidence
    _append_reason(reasons, "shared_report", report_ready, report_reasons)

    # 3. Exact joint state content/stamp/age/source.
    joint_evidence = snapshot.get("joint_states", {})
    joint_ready = bool(joint_evidence.get("ready"))
    evidence["joint_states"] = joint_evidence
    _append_reason(reasons, "joint_states", joint_ready, joint_evidence.get("reasons", ()))

    # 4. base_link -> link_tcp TF.
    tf_evidence = snapshot.get("tf", {})
    tf_ready = bool(tf_evidence.get("ready"))
    evidence["tf"] = tf_evidence
    _append_reason(reasons, "tf", tf_ready, tf_evidence.get("reasons", ()))

    # 5. Active trajectory controller logical-resource identity.
    controllers = snapshot.get("controller_resources", {})
    controller_reasons: list[str] = []
    controller_observed: dict[str, object] = {}
    for name, expected_state in contract.get("controller_resources", {}).items():
        entry = controllers.get(name, {})
        state = entry.get("state")
        controller_observed[name] = entry
        if state != expected_state:
            controller_reasons.append(
                "controller {!r} state {!r} != expected {!r}".format(name, state, expected_state)
            )
    if contract.get("controller_resources", {}).get("xarm7_traj_controller"):
        action_server = controllers.get("xarm7_traj_controller", {}).get("action_server_count")
        if action_server != 1:
            controller_reasons.append(
                "xarm7_traj_controller follow_joint_trajectory server count is {}, expected 1".format(action_server)
            )
    evidence["controller_resources"] = controller_observed
    _append_reason(reasons, "controller_resources", not controller_reasons, controller_reasons)

    # 6. Operator input (fresh false clear) and effective safety output.
    operator = snapshot.get("operator_input", {})
    operator_ready = bool(operator.get("ready"))
    evidence["operator_input"] = operator
    _append_reason(reasons, "operator_input", operator_ready, operator.get("reasons", ()))
    safety = snapshot.get("safety_stop", {})
    safety_ready = bool(safety.get("ready"))
    evidence["safety_stop"] = safety
    _append_reason(reasons, "safety_stop", safety_ready, safety.get("reasons", ()))

    # 7. Every typed action, exactly one server each, with observed-type proof.
    observed_actions = snapshot.get("actions", {})
    action_reasons: list[str] = []
    action_observed: dict[str, object] = {}
    for endpoint, expected in contract.get("actions", {}).items():
        entry = observed_actions.get(endpoint, {})
        action_observed[endpoint] = entry
        if entry.get("count") != expected.get("cardinality", 1):
            action_reasons.append(
                "action {} server count {} != {}".format(
                    endpoint, entry.get("count"), expected.get("cardinality", 1)
                )
            )
        observed_types = entry.get("observed_types") or entry.get("types") or []
        goal_type = _derive_goal_service_type(str(expected.get("type", "")))
        type_reason = None
        if goal_type:
            type_reason = _check_observed_types(observed_types, goal_type, endpoint, "action")
        elif observed_types and observed_types != [expected.get("type")]:
            type_reason = "action {} observed types {!r} != expected {!r}".format(
                endpoint, sorted({str(v) for v in observed_types}), expected.get("type")
            )
        if type_reason:
            action_reasons.append(type_reason)
        source = entry.get("source")
        expected_source = expected.get("source")
        if source != expected_source and not (
            expected_source
            and expected_source.startswith("controller_resource:")
            and source == expected_source
        ):
            action_reasons.append(
                "action {} source {!r} != {!r}".format(endpoint, source, expected_source)
            )
    evidence["actions"] = action_observed
    _append_reason(reasons, "actions", not action_reasons, action_reasons)

    # 8. Every typed service, exactly one server each, with observed-type proof.
    observed_services = snapshot.get("services", {})
    service_reasons: list[str] = []
    service_observed: dict[str, object] = {}
    for endpoint, expected in contract.get("services", {}).items():
        entry = observed_services.get(endpoint, {})
        service_observed[endpoint] = entry
        if entry.get("count") != expected.get("cardinality", 1):
            service_reasons.append(
                "service {} server count {} != {}".format(
                    endpoint, entry.get("count"), expected.get("cardinality", 1)
                )
            )
        observed_types = entry.get("observed_types") or entry.get("types") or []
        type_reason = _check_observed_types(
            observed_types, str(expected.get("type", "")), endpoint, "service"
        )
        if type_reason:
            service_reasons.append(type_reason)
        if entry.get("source") != expected.get("source"):
            service_reasons.append(
                "service {} source {!r} != {!r}".format(endpoint, entry.get("source"), expected.get("source"))
            )
    evidence["services"] = service_observed
    _append_reason(reasons, "services", not service_reasons, service_reasons)

    # 9. /arm_joint_service with exactly one /pick_and_place server.
    arm_service = snapshot.get("arm_joint_service", {})
    arm_ready = bool(arm_service.get("ready"))
    evidence["arm_joint_service"] = arm_service
    _append_reason(reasons, "arm_joint_service", arm_ready, arm_service.get("reasons", ()))

    # 10. Canonical fixture status exact fields.
    fixture = snapshot.get("fixture_status", {})
    fixture_ready = bool(fixture.get("ready"))
    evidence["fixture_status"] = fixture
    _append_reason(reasons, "fixture_status", fixture_ready, fixture.get("reasons", ()))

    # 11. Full scenario/planning_scene/integrated mapping and digest agreement.
    mapping = snapshot.get("mapping_agreement", {})
    mapping_ready = bool(mapping.get("ready"))
    evidence["mapping_agreement"] = mapping
    _append_reason(reasons, "mapping_agreement", mapping_ready, mapping.get("reasons", ()))

    # 12. Publisher graph metadata agreement for every typed endpoint.
    publisher_metadata = snapshot.get("publishers", {})
    publisher_reasons = []
    if contract.get("publishers"):
        publisher_ready, publisher_reasons = evaluate_publisher_metadata(
            publisher_metadata, contract["publishers"]
        )
    else:
        publisher_ready = not publisher_reasons
    evidence["publishers"] = publisher_metadata
    _append_reason(reasons, "publishers", publisher_ready, publisher_reasons)

    # 13. Provider-manifest path/digest and resolved/live agreement.
    provider = snapshot.get("provider_manifest", {})
    provider_ready = bool(provider.get("ready"))
    provider_reasons = list(provider.get("reasons", []))
    manifest = provider.get("manifest")
    if provider_ready and isinstance(manifest, dict):
        manifest_reasons = _provider_manifest_agreement(
            manifest,
            [str(name) for name in provider.get("observed_nodes", [])],
            [str(topic) for topic in provider.get("observed_publishers", [])],
            {
                str(name): str(state)
                for name, state in provider.get("observed_controllers", {}).items()
            },
        )
        provider_reasons.extend(manifest_reasons)
        provider_ready = not provider_reasons
    evidence["provider_manifest"] = provider
    _append_reason(reasons, "provider_manifest", provider_ready, provider_reasons)

    # 14. Semantic model/kinematics equality.
    semantic = snapshot.get("semantic_model", {})
    semantic_ready = bool(semantic.get("ready"))
    evidence["semantic_model"] = semantic
    _append_reason(reasons, "semantic_model", semantic_ready, semantic.get("reasons", ()))

    # 15. Initial collision state.
    collision = snapshot.get("collision_state", {})
    collision_ready = bool(collision.get("ready"))
    evidence["collision_state"] = collision
    _append_reason(reasons, "collision_state", collision_ready, collision.get("reasons", ()))

    return ReadinessReport(ready=not reasons, reasons=tuple(reasons), evidence=evidence)


__all__ = [
    "FINAL_SIMULATION_STATE",
    "IDENTITIES_KEYS",
    "INTEGRATED_ACTIONS",
    "INTEGRATED_EXECUTION_PROFILE",
    "INTEGRATED_JOINT_STATE_NAMES",
    "INTEGRATED_PUBLISHERS",
    "INTEGRATED_SERVICES",
    "INTEGRATED_TOUCH_LINKS",
    "PHYSICS_READY_BOUNDARY",
    "PLANNING_SCENE_REPORT_KEYS",
    "REPORT_REVISION",
    "REPORT_SCHEMA_VERSION",
    "ReadinessReport",
    "ReportValidationError",
    "SIMULATION_STATE_PLAYING",
    "TF_CHILD",
    "TF_PARENT",
    "build_canonical_report",
    "build_integrated_mapping",
    "canonical_json",
    "evaluate_integrated_readiness",
    "normalize_qos_value",
    "parse_canonical_report",
    "planning_scene_digest",
    "planning_scene_mapping",
    "public_integrated_mapping",
    "report_identities",
    "scenario_mapping",
    "serialize_report",
    "sha256_bytes",
    "sha256_json",
    "validate_report",
]
