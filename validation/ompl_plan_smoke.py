"""Deterministic OMPL plan-only smoke (Task 7, consolidated fix round 1).

This module is ROS-free at import time and runs under simulator CPython 3.12:
it imports neither ``rclpy``, nor ``rclpy.action``, nor ``moveit_msgs``, nor
any generated message type at module scope.  ``main()`` is the live Humble
client seam: it imports ``rclpy`` and constructs
:class:`OmplPlanSmokeClient`, whose methods import ``rclpy`` / ``moveit_msgs``
only inside their bodies (the Humble-only live seam).

Live behavior (only when the Task 6 overlay is running and ready):

1. Wait boundedly for a **canonical** fresh ``pass`` on
   ``/sim/status/integrated_manipulation``: exact Task 6 status schema, exactly
   one ``std_msgs/msg/String`` publisher from ``/integrated_readiness`` with
   RELIABLE + TRANSIENT_LOCAL QoS, and scenario/revision/digest identity
   agreement against the selected local scenario.
2. Prove the ``/isaac_joint_commands`` channel: boundedly wait for publisher
   discovery and require exactly one ``sensor_msgs/msg/JointState`` publisher
   from ``/tinker_sim_command_gateway`` (RELIABLE + VOLATILE).
3. Re-verify readiness freshness immediately before send; open the command
   observation window across goal send, result/cancel terminal handling, and a
   fixed post-result tail, requiring exactly zero command samples.
4. Probe ``/move_action`` and require exactly one ``moveit_msgs/action/MoveGroup``
   action server with observed kind/type/source/cardinality and an exact
   ``_action/get_result`` type.
5. Send a goal with ``request.pipeline_id="ompl"`` and
   ``planning_options.plan_only=true``.  Joint/pose require an accepted goal,
   terminal ``STATUS_SUCCEEDED``, ``MoveItErrorCodes.SUCCESS``, and a nonempty
   trajectory.  Blocked mode requires an accepted goal, a typed result, a
   terminal status in the documented planning-failure statuses, and an explicit
   MoveIt planning/collision/constraint failure code from the documented
   allowlist — never a rejection/transport/cancel/default-code outcome.
6. Write compact canonical JSON acceptance evidence atomically.

When the readiness gate cannot be reached, or any proof fails, the client
writes a canonical fail-closed report recording the exact blocker and exits
nonzero.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Mapping, Sequence

from ompl_goal_builders import (
    DEFAULT_ALLOWED_PLANNING_TIME,
    DEFAULT_PLANNER_ID,
    DEFAULT_POSITION_TOLERANCE,
    GROUP_NAME,
    POSE_APPROACH_Z_OFFSET,
    build_joint_goal,
    build_pose_goal,
    goal_kind,
    goal_to_dict,
)

REPO_ROOT = Path(__file__).resolve().parents[1]

MOVE_ACTION = "/move_action"
MOVE_ACTION_TYPE = "moveit_msgs/action/MoveGroup"
MOVE_ACTION_SOURCE = "/move_group"
READINESS_TOPIC = "/sim/status/integrated_manipulation"
READINESS_SOURCE = "/integrated_readiness"
READINESS_TYPE = "std_msgs/msg/String"
READINESS_SCHEMA_VERSION = 1
READINESS_STATE_PASS = "pass"
READINESS_STATE_FAIL = "fail"
COMMAND_TOPIC = "/isaac_joint_commands"
COMMAND_SOURCE = "/tinker_sim_command_gateway"
COMMAND_TYPE = "sensor_msgs/msg/JointState"

PIPELINE_ID = "ompl"
PLAN_ONLY = True
SUCCESS_ERROR_CODE = 1

# The action kind is derived from the canonical action type contract, never an
# independent drift-prone literal (adversarial M2 / spec M2).
MOVE_ACTION_KIND = "MoveGroup"

# action_msgs/msg/GoalStatus constants (Humble).
STATUS_UNKNOWN = 0
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

GOAL_STATUS_NAMES: Mapping[int, str] = {
    STATUS_UNKNOWN: "STATUS_UNKNOWN",
    STATUS_ACCEPTED: "STATUS_ACCEPTED",
    STATUS_EXECUTING: "STATUS_EXECUTING",
    STATUS_CANCELING: "STATUS_CANCELING",
    STATUS_SUCCEEDED: "STATUS_SUCCEEDED",
    STATUS_CANCELED: "STATUS_CANCELED",
    STATUS_ABORTED: "STATUS_ABORTED",
}

# moveit_msgs/msg/MoveItErrorCodes names (Humble).
MOVEIT_ERROR_CODES: Mapping[int, str] = {
    1: "SUCCESS",
    99999: "FAILURE",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
    -6: "TIMED_OUT",
    -7: "PREEMPTED",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -15: "INVALID_GROUP_NAME",
    -16: "INVALID_GOAL_CONSTRAINTS",
    -17: "INVALID_ROBOT_STATE",
    -18: "INVALID_LINK_NAME",
    -19: "INVALID_OBJECT_NAME",
    -21: "FRAME_TRANSFORM_FAILURE",
    -22: "COLLISION_CHECKING_UNAVAILABLE",
    -23: "ROBOT_STATE_STALE",
    -24: "SENSOR_INFO_STALE",
    -25: "COMMUNICATION_FAILURE",
    -26: "START_STATE_INVALID",
    -27: "GOAL_STATE_INVALID",
    -28: "UNRECOGNIZED_GOAL_TYPE",
    -29: "CRASH",
    -30: "ABORT",
    -31: "NO_IK_SOLUTION",
}

# Documented planning/collision/constraint failure codes accepted for blocked
# mode.  Restricted to genuine planning/collision/constraint outcomes only:
# request/state/config-validation codes (INVALID_GROUP_NAME -15,
# INVALID_GOAL_CONSTRAINTS -16, INVALID_ROBOT_STATE -17, START_STATE_INVALID
# -26, GOAL_STATE_INVALID -27, UNRECOGNIZED_GOAL_TYPE -28) and every control/
# execution/transport/setup/configuration failure are deliberately excluded so
# a misconfigured overlay cannot pass blocked mode (fix1 adversarial A).
MOVEIT_PLANNING_FAILURE_CODES = frozenset(
    {
        -1,   # PLANNING_FAILED
        -2,   # INVALID_MOTION_PLAN
        -3,   # MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE
        -10,  # START_STATE_IN_COLLISION
        -11,  # START_STATE_VIOLATES_PATH_CONSTRAINTS
        -12,  # GOAL_IN_COLLISION
        -13,  # GOAL_VIOLATES_PATH_CONSTRAINTS
        -14,  # GOAL_CONSTRAINTS_VIOLATED
        -31,  # NO_IK_SOLUTION
    }
)

# Explicitly-rejected MoveItErrorCodes used as documentation/evidence: the
# request/state/config-validation codes (blocked-mode masquerades) plus the
# control/execution/transport/setup codes already excluded by the allowlist.
MOVEIT_CONFIGURATION_FAILURE_CODES = frozenset(
    {
        -4,    # CONTROL_FAILED
        -5,    # UNABLE_TO_AQUIRE_SENSOR_DATA
        -6,    # TIMED_OUT
        -7,    # PREEMPTED
        -15,   # INVALID_GROUP_NAME
        -16,   # INVALID_GOAL_CONSTRAINTS
        -17,   # INVALID_ROBOT_STATE
        -21,   # FRAME_TRANSFORM_FAILURE
        -22,   # COLLISION_CHECKING_UNAVAILABLE
        -23,   # ROBOT_STATE_STALE
        -24,   # SENSOR_INFO_STALE
        -25,   # COMMUNICATION_FAILURE
        -26,   # START_STATE_INVALID
        -27,   # GOAL_STATE_INVALID
        -28,   # UNRECOGNIZED_GOAL_TYPE
        -29,   # CRASH
        -30,   # ABORT
        99999,  # FAILURE
    }
)

# Blocked-mode acceptance permits an accepted goal that ended with the action
# terminal status STATUS_ABORTED (the normal planning-failure terminal); a
# STATUS_SUCCEEDED terminal is allowed only when the server delivers a
# non-success MoveIt result that way.  The generic error-code allowlist check
# rejects SUCCESS in that case.
BLOCKED_ALLOWED_TERMINAL_STATUSES = frozenset({STATUS_ABORTED, STATUS_SUCCEEDED})

MODES = ("joint", "pose", "blocked")
MODE_SCENARIOS: Mapping[str, str] = {
    "joint": "qualification-moveit-plan-joint",
    "pose": "qualification-moveit-plan-pose",
    "blocked": "qualification-moveit-plan-blocked",
}

DEFAULT_REPORT_PATH = str(Path("outputs/ompl-plan-smoke/ompl-plan-smoke.json"))
DEFAULT_ACTION_TIMEOUT = 30.0
DEFAULT_READINESS_TIMEOUT = 60.0
DEFAULT_READINESS_MAX_AGE_S = 1.0
DEFAULT_TRAJECTORY_MIN_POINTS = 1
DEFAULT_POST_RESULT_TAIL_S = 0.25
DEFAULT_DISCOVERY_SETTLE_S = 0.5

# Keyword arguments accepted by ``ompl_goal_builders.build_joint_goal``.  Used by
# :func:`build_goal` to forward only joint-relevant overrides in joint mode.
_JOINT_BUILDER_KWARGS = frozenset(
    {
        "joint_positions",
        "group_name",
        "tolerances",
        "pipeline_id",
        "plan_only",
        "planner_id",
        "allowed_planning_time",
        "num_planning_attempts",
    }
)


# ---------------------------------------------------------------------------
# Deterministic JSON helpers
# ---------------------------------------------------------------------------


def canonical_json(value: object) -> bytes:
    """Return compact canonical JSON bytes (sorted keys, minimal separators)."""
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")


def sha256_json(value: object) -> str:
    """Return the lowercase SHA-256 of the canonical JSON bytes of *value*."""
    return hashlib.sha256(canonical_json(value)).hexdigest()


def serialize_report(report: Mapping[str, object]) -> bytes:
    """Serialize a smoke report to compact canonical JSON bytes."""
    return canonical_json(report)


def write_report_atomic(path: str | Path, data: bytes) -> Path:
    """Atomically write *data* to *path* (sibling temp + ``os.replace``).

    On any write failure the sibling ``.tmp`` is removed before re-raising, so
    no temp artifact is left behind.
    """
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    try:
        temporary.write_bytes(data)
        os.replace(temporary, target)
    except Exception:
        try:
            if temporary.exists():
                temporary.unlink()
        except OSError:
            pass
        raise
    return target


def _shell_meta() -> dict[str, object]:
    """Build environment metadata without importing rclpy (exception-safe)."""
    return {
        "python": "{}.{}.{}".format(*sys.version_info[:3]),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
        "domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
        "dds_profile": os.environ.get("TINKER_SIM_DDS_PROFILE", ""),
        "simulator_root": str(REPO_ROOT),
    }


def _exception_meta() -> dict[str, object]:
    """Exception-path meta; includes the rclpy version when rclpy imported
    successfully before the failure (fix1 spec M1)."""
    meta = _shell_meta()
    if "rclpy" in sys.modules:
        try:
            import rclpy

            meta["rclpy"] = str(getattr(rclpy, "__version__", ""))
        except Exception:  # noqa: BLE001 - never let meta collection mask the failure
            meta["rclpy"] = ""
    return meta


# ---------------------------------------------------------------------------
# Scenario helpers (ROS-free)
# ---------------------------------------------------------------------------


def load_scenario(path: str | Path) -> dict[str, object]:
    """Load and structurally validate a scenario declaration file."""
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError("scenario unreadable at {}: {}".format(path, exc)) from exc
    if not isinstance(raw, dict):
        raise ValueError("scenario declaration must be a JSON object")
    return raw


def scenario_qualification_gate(scenario: Mapping[str, object]) -> str:
    """Return the scenario's ``qualification_gate`` string."""
    return str(scenario.get("qualification_gate", ""))


def _pose_tuple(pose: object) -> tuple[list[float], list[float]]:
    if not isinstance(pose, dict):
        raise ValueError("scenario object pose must be an object")
    xyz = pose.get("xyz")
    quat = pose.get("quaternion_xyzw")
    if not isinstance(xyz, Sequence) or len(xyz) != 3:
        raise ValueError("scenario object pose must carry xyz (3 values)")
    if not isinstance(quat, Sequence) or len(quat) != 4:
        raise ValueError("scenario object pose must carry quaternion_xyzw (4 values)")
    return [float(value) for value in xyz], [float(value) for value in quat]


def scenario_target_pose(
    scenario: Mapping[str, object],
) -> tuple[list[float], list[float]]:
    """Return the ``class == "target"`` object's xyz and quaternion_xyzw."""
    objects = (scenario.get("planning_scene") or {}).get("objects") or []
    for obj in objects:
        if isinstance(obj, dict) and obj.get("class") == "target":
            return _pose_tuple(obj.get("pose"))
    raise ValueError("scenario has no 'target' object")


def scenario_blocker_pose(
    scenario: Mapping[str, object],
) -> tuple[list[float], list[float]]:
    """Return the ``class == "blocker"`` object's xyz and quaternion_xyzw."""
    objects = (scenario.get("planning_scene") or {}).get("objects") or []
    for obj in objects:
        if isinstance(obj, dict) and obj.get("class") == "blocker":
            return _pose_tuple(obj.get("pose"))
    raise ValueError("scenario has no 'blocker' object")


def build_goal(mode: str, scenario: Mapping[str, object], **overrides: object) -> object:
    """Build a deterministic plain-data goal from a scenario and mode.

    The scenario's ``qualification_gate`` must equal ``moveit-plan-<mode>``
    (fail-closed mode/scenario consistency).  Joint mode builds a small reach
    from a vertical arm; pose mode hovers ``POSE_APPROACH_Z_OFFSET`` above the
    target object; blocked mode targets the interior of the ``blocker`` object
    so every goal sample is in collision, yielding a deterministic non-success.
    """
    if mode not in MODES:
        raise ValueError("unknown mode {!r}; expected one of {}".format(mode, MODES))
    gate = scenario_qualification_gate(scenario)
    expected_gate = "moveit-plan-{}".format(mode)
    if gate != expected_gate:
        raise ValueError(
            "scenario qualification_gate {!r} does not match mode {!r}".format(gate, mode)
        )
    if mode == "joint":
        # Only joint-relevant keyword overrides are forwarded so pose-only
        # options (e.g. position_tolerance) never leak into the joint builder.
        joint_kwargs = {
            key: value
            for key, value in overrides.items()
            if key in _JOINT_BUILDER_KWARGS
        }
        return build_joint_goal(**joint_kwargs)
    target_xyz, target_quat = scenario_target_pose(scenario)
    if mode == "pose":
        return build_pose_goal(
            [target_xyz[0], target_xyz[1], target_xyz[2] + POSE_APPROACH_Z_OFFSET],
            target_quat,
            **overrides,
        )
    blocked_xyz, blocked_quat = scenario_blocker_pose(scenario)
    return build_pose_goal(blocked_xyz, blocked_quat, **overrides)


def scenario_expected_identities(scenario: Mapping[str, object]) -> dict[str, object]:
    """Compute the expected identity values for a local scenario declaration.

    Uses the Task 6 canonical helpers (``scenario_mapping``,
    ``planning_scene_mapping``, ``fixture_owned_ids``, ``sha256_json``) so the
    smoke and the readiness node agree on digest semantics.  ROS-free (the
    bridge helpers import no ROS), but imported lazily so this module stays
    importable without the bridge package.
    """
    from tinker_sim_bridge.integrated_readiness import (
        planning_scene_mapping,
        scenario_mapping,
        sha256_json,
    )
    from tinker_sim_bridge.fixture_planning_scene import fixture_owned_ids

    scenario_id = str(scenario["id"])
    seed = int(scenario["seed"])
    declaration = {
        str(key): value for key, value in scenario.items() if key not in ("id", "seed")
    }
    planning_scene = scenario["planning_scene"]
    return {
        "scenario_id": scenario_id,
        "seed": seed,
        "scenario_declaration_sha256": sha256_json(
            scenario_mapping(scenario_id, seed, declaration)
        ),
        "planning_scene_sha256": sha256_json(planning_scene_mapping(planning_scene)),
        "planning_scene_revision": str(planning_scene["revision"]),
        "planning_scene_revision_digest": str(
            planning_scene.get("revision_digest", "")
        ),
        "planning_scene_owned_ids": [str(item) for item in fixture_owned_ids(planning_scene)],
        "planning_scene_target_source_id": str(
            planning_scene.get("target_source_id", "")
        ),
        "planning_scene_target_handoff": str(
            planning_scene.get("target_handoff", "")
        ),
    }


# ---------------------------------------------------------------------------
# Endpoint / status / error derivation (ROS-free)
# ---------------------------------------------------------------------------


def derive_goal_service_type(action_type: str) -> str:
    """Derive the canonical ``_action/send_goal`` service type for an action."""
    marker = "/action/"
    if marker not in action_type:
        return ""
    package, action = action_type.split(marker, 1)
    return "{}/action/{}_SendGoal".format(package, action)


def derive_result_service_type(action_type: str) -> str:
    """Derive the canonical ``_action/get_result`` service type for an action."""
    marker = "/action/"
    if marker not in action_type:
        return ""
    package, action = action_type.split(marker, 1)
    return "{}/action/{}_GetResult".format(package, action)


def derive_action_kind(action_type: str) -> str:
    """Extract the action kind from a canonical action type (``MoveGroup``)."""
    marker = "/action/"
    if marker not in action_type:
        return ""
    return action_type.split(marker, 1)[1]


def _action_name_from_type(observed_type: str) -> str:
    """Extract the action kind from an observed ROS interface type string.

    ``moveit_msgs/action/MoveGroup_SendGoal`` -> ``"MoveGroup"``.  The kind is
    observed (derived from live graph metadata), never stamped from an expected
    literal.
    """
    text = str(observed_type)
    marker = "/action/"
    if marker not in text:
        return ""
    base = text.split(marker, 1)[1]
    for suffix in ("_SendGoal", "_GetResult", "_CancelGoal", "_Feedback"):
        if base.endswith(suffix):
            base = base[: -len(suffix)]
            break
    return base


def goal_status_name(status: object) -> str:
    """Return the canonical ``GoalStatus`` constant name for an int status."""
    try:
        value = int(status)
    except (TypeError, ValueError):
        return "UNKNOWN_STATUS"
    return GOAL_STATUS_NAMES.get(value, "STATUS_{}".format(value))


def moveit_error_code_name(code: object) -> str:
    """Return the canonical ``MoveItErrorCodes`` constant name for an int code."""
    try:
        value = int(code)
    except (TypeError, ValueError):
        return "CODE_{}".format(code)
    return MOVEIT_ERROR_CODES.get(value, "CODE_{}".format(value))


def _qos_short(value: object) -> str:
    """Normalize a Humble QoS enum value to its short uppercase name."""
    text = str(value)
    if "." in text:
        text = text.rsplit(".", 1)[-1]
    return text.strip().upper()


# ---------------------------------------------------------------------------
# Readiness payload parsing / identity (ROS-free)
# ---------------------------------------------------------------------------


def parse_readiness_payload(raw_text: str) -> dict[str, object]:
    """Parse and structurally validate a readiness ``std_msgs/msg/String`` payload.

    Returns ``{"valid": bool, "payload": dict|None, "reason": str}``.  A
    payload is valid only when it matches the exact Task 6 status schema:
    ``schema_version=1``, ``state in {pass, fail}``, ``ready`` matching state,
    ``reasons`` a list (empty for pass), finite ``published_at``, and
    mapping-shaped ``evidence``.  Malformed JSON is never silently ignored.
    """
    try:
        payload = json.loads(raw_text)
    except (json.JSONDecodeError, TypeError, ValueError) as exc:
        return {"valid": False, "payload": None, "reason": "malformed JSON: {}".format(exc)}
    if not isinstance(payload, dict):
        return {"valid": False, "payload": None, "reason": "payload must be a JSON object"}
    reasons: list[str] = []
    if payload.get("schema_version") != READINESS_SCHEMA_VERSION:
        reasons.append(
            "schema_version {!r} != {}".format(
                payload.get("schema_version"), READINESS_SCHEMA_VERSION
            )
        )
    state = payload.get("state")
    if state not in (READINESS_STATE_PASS, READINESS_STATE_FAIL):
        reasons.append("state {!r} not in (pass, fail)".format(state))
    ready = payload.get("ready")
    if not isinstance(ready, bool):
        reasons.append("ready must be a boolean")
    elif state == READINESS_STATE_PASS and ready is not True:
        reasons.append("ready must be true when state is pass")
    elif state == READINESS_STATE_FAIL and ready is not False:
        reasons.append("ready must be false when state is fail")
    canonical_reasons = payload.get("reasons")
    if not isinstance(canonical_reasons, list):
        reasons.append("reasons must be a list")
    elif state == READINESS_STATE_PASS and canonical_reasons:
        reasons.append("pass payload must carry an empty reasons list")
    published_at = payload.get("published_at")
    if (
        isinstance(published_at, bool)
        or not isinstance(published_at, (int, float))
        or not math.isfinite(float(published_at))
    ):
        reasons.append("published_at must be a finite number")
    evidence = payload.get("evidence")
    if not isinstance(evidence, dict):
        reasons.append("evidence must be an object")
    if reasons:
        return {
            "valid": False,
            "payload": dict(payload),
            "reason": "; ".join(reasons),
        }
    return {"valid": True, "payload": dict(payload), "reason": ""}


def readiness_identity_reasons(
    payload: Mapping[str, object], expected: Mapping[str, object]
) -> list[str]:
    """Fail-closed identity agreement between a canonical pass and the scenario.

    Compares ``evidence.shared_report.identities`` (scenario id, seed,
    scenario declaration digest, planning-scene digest) and
    ``evidence.fixture_status.status`` (scenario, revision, revision_digest,
    declared-order owned ids, target source, target handoff) against the local
    scenario's expected values.
    """
    reasons: list[str] = []
    evidence = payload.get("evidence") or {}
    if not isinstance(evidence, dict):
        return ["readiness evidence is not an object"]
    identities = (evidence.get("shared_report") or {}).get("identities") or {}
    if not isinstance(identities, dict):
        reasons.append("shared_report identities is not an object")
        identities = {}
    checks = (
        ("scenario_id", "scenario_id"),
        ("scenario_declaration_sha256", "scenario_declaration_sha256"),
        ("planning_scene_sha256", "planning_scene_sha256"),
    )
    for observed_key, expected_key in checks:
        observed = identities.get(observed_key)
        wanted = expected.get(expected_key)
        if str(observed) != str(wanted):
            reasons.append(
                "readiness {} {!r} != expected {!r}".format(
                    observed_key, observed, wanted
                )
            )
    seed_observed = identities.get("seed")
    seed_wanted = expected.get("seed")
    try:
        if int(seed_observed) != int(seed_wanted):
            reasons.append(
                "readiness seed {!r} != expected {!r}".format(seed_observed, seed_wanted)
            )
    except (TypeError, ValueError):
        reasons.append(
            "readiness seed {!r} != expected {!r}".format(seed_observed, seed_wanted)
        )
    fixture = (evidence.get("fixture_status") or {}).get("status") or {}
    if not isinstance(fixture, dict):
        reasons.append("fixture_status.status is not an object")
        fixture = {}
    fixture_checks = (
        ("scenario", "scenario_id"),
        ("revision", "planning_scene_revision"),
        ("revision_digest", "planning_scene_revision_digest"),
        ("target_source_id", "planning_scene_target_source_id"),
        ("target_handoff", "planning_scene_target_handoff"),
    )
    for observed_key, expected_key in fixture_checks:
        observed = fixture.get(observed_key)
        wanted = expected.get(expected_key)
        if str(observed) != str(wanted):
            reasons.append(
                "readiness fixture {} {!r} != expected {!r}".format(
                    observed_key, observed, wanted
                )
            )
    observed_owned = tuple(str(item) for item in (fixture.get("owned_ids") or ()))
    wanted_owned = tuple(
        str(item) for item in (expected.get("planning_scene_owned_ids") or ())
    )
    if observed_owned != wanted_owned:
        reasons.append(
            "readiness fixture owned_ids {!r} != expected {!r}".format(
                observed_owned, wanted_owned
            )
        )
    return reasons


# ---------------------------------------------------------------------------
# Pure evaluator
# ---------------------------------------------------------------------------


def _check_publisher_metadata(
    entry: Mapping[str, object],
    *,
    expected_type: str,
    expected_source: str,
    expected_reliability: str,
    expected_durability: str,
    label: str,
) -> list[str]:
    """Fail-closed publisher graph metadata agreement for one topic."""
    reasons: list[str] = []
    if entry.get("count") != 1:
        reasons.append("{} publisher count {!r} != 1".format(label, entry.get("count")))
    observed_type = entry.get("type")
    if observed_type != expected_type:
        reasons.append(
            "{} publisher type {!r} != {!r}".format(label, observed_type, expected_type)
        )
    observed_source = entry.get("source")
    if observed_source != expected_source:
        reasons.append(
            "{} publisher source {!r} != {!r}".format(label, observed_source, expected_source)
        )
    qos = entry.get("qos") or {}
    if _qos_short(qos.get("reliability", "")) != _qos_short(expected_reliability):
        reasons.append(
            "{} publisher reliability {!r} != {!r}".format(
                label, qos.get("reliability"), expected_reliability
            )
        )
    if _qos_short(qos.get("durability", "")) != _qos_short(expected_durability):
        reasons.append(
            "{} publisher durability {!r} != {!r}".format(
                label, qos.get("durability"), expected_durability
            )
        )
    depth = qos.get("depth")
    try:
        depth_value = int(depth)
    except (TypeError, ValueError):
        depth_value = 0
    expected_depth = entry.get("expected_depth")
    if expected_depth is not None and depth_value > 0 and depth_value != int(expected_depth):
        reasons.append(
            "{} publisher depth {!r} != expected {!r}".format(label, depth, expected_depth)
        )
    return reasons


def _evaluate_readiness(
    readiness: Mapping[str, object], expected: Mapping[str, object]
) -> list[str]:
    reasons: list[str] = []
    if readiness.get("valid") is not True:
        reasons.append("readiness is not a valid canonical pass")
    if readiness.get("state") != READINESS_STATE_PASS:
        reasons.append(
            "readiness state {!r} != {!r}".format(
                readiness.get("state"), READINESS_STATE_PASS
            )
        )
    if readiness.get("ready") is not True:
        reasons.append("readiness ready flag is not true")
    if readiness.get("schema_version") != READINESS_SCHEMA_VERSION:
        reasons.append(
            "readiness schema_version {!r} != {}".format(
                readiness.get("schema_version"), READINESS_SCHEMA_VERSION
            )
        )
    canonical_reasons = readiness.get("reasons")
    if canonical_reasons:
        reasons.append(
            "readiness payload carries reasons {!r}".format(canonical_reasons)
        )
    received_at = readiness.get("received_at_s")
    now_s = readiness.get("now_s")
    max_age = float(expected.get("readiness_max_age_s", DEFAULT_READINESS_MAX_AGE_S))
    if isinstance(received_at, (int, float)) and isinstance(now_s, (int, float)):
        consumer_age = float(now_s) - float(received_at)
        if consumer_age < 0.0 or consumer_age > max_age:
            reasons.append(
                "readiness consumer age {:.3f} s exceeds {:.3f} s".format(
                    consumer_age, max_age
                )
            )
    else:
        reasons.append("readiness receive timestamp missing")
    published_at = readiness.get("published_at")
    if isinstance(published_at, (int, float)) and isinstance(now_s, (int, float)):
        producer_age = float(now_s) - float(published_at)
        if producer_age < 0.0 or producer_age > max_age:
            reasons.append(
                "readiness producer age {:.3f} s exceeds {:.3f} s".format(
                    producer_age, max_age
                )
            )
    else:
        reasons.append("readiness published_at missing")
    if readiness.get("gate_refreshed") is not True:
        reasons.append("readiness gate not re-verified immediately before send")
    reasons.extend(
        _check_publisher_metadata(
            readiness.get("publisher") or {},
            expected_type=READINESS_TYPE,
            expected_source=READINESS_SOURCE,
            expected_reliability="RELIABLE",
            expected_durability="TRANSIENT_LOCAL",
            label=READINESS_TOPIC,
        )
    )
    identity = readiness.get("identity") or {}
    if not isinstance(identity, dict) or not identity:
        reasons.append("readiness identity evidence missing")
    else:
        for field in sorted(identity):
            entry = identity[field]
            if isinstance(entry, dict) and entry.get("ok") is not True:
                reasons.append("readiness identity {} does not match".format(field))
            elif not isinstance(entry, dict):
                reasons.append("readiness identity {} is not an object".format(field))
    window_counts = readiness.get("window_counts") or {}
    if window_counts.get("fail"):
        reasons.append("readiness degraded to fail during the observation window")
    if window_counts.get("malformed"):
        reasons.append("readiness produced a malformed payload during the window")
    if window_counts.get("identity_invalid"):
        reasons.append(
            "readiness produced a wrong-identity pass during the observation window"
        )
    if readiness.get("any_fail_in_window"):
        reasons.append("readiness observed fail during the window")
    if readiness.get("any_malformed_in_window"):
        reasons.append("readiness observed malformed payload during the window")
    if readiness.get("any_identity_invalid_in_window"):
        reasons.append("readiness observed wrong-identity pass during the window")
    if readiness.get("publisher_graph_changed"):
        reasons.append("readiness publisher graph changed during the window")
    return reasons


def _evaluate_endpoint(
    endpoint: Mapping[str, object], expected: Mapping[str, object]
) -> list[str]:
    reasons: list[str] = []
    if endpoint.get("count") != 1:
        reasons.append(
            "endpoint {} server count {!r} != 1".format(
                MOVE_ACTION, endpoint.get("count")
            )
        )
    goal_types = sorted({str(value) for value in (endpoint.get("observed_types") or [])})
    expected_goal_type = derive_goal_service_type(
        str(expected.get("action_type", MOVE_ACTION_TYPE))
    )
    if goal_types != [expected_goal_type]:
        reasons.append(
            "endpoint {} observed goal-service types {!r} != expected {!r}".format(
                MOVE_ACTION, goal_types, expected_goal_type
            )
        )
    expected_kind = derive_action_kind(
        str(expected.get("action_type", MOVE_ACTION_TYPE))
    )
    observed_kind = endpoint.get("kind")
    if observed_kind != expected_kind:
        reasons.append(
            "endpoint kind {!r} != {!r}".format(observed_kind, expected_kind)
        )
    observed_source = endpoint.get("source")
    expected_source = str(expected.get("source", MOVE_ACTION_SOURCE))
    if observed_source != expected_source:
        reasons.append(
            "endpoint source {!r} != {!r}".format(observed_source, expected_source)
        )
    result_types = sorted({str(value) for value in (endpoint.get("result_service_types") or [])})
    expected_result_type = derive_result_service_type(
        str(expected.get("action_type", MOVE_ACTION_TYPE))
    )
    if result_types != [expected_result_type]:
        reasons.append(
            "endpoint {} observed result-service types {!r} != expected {!r}".format(
                MOVE_ACTION, result_types, expected_result_type
            )
        )
    return reasons


def _evaluate_commands(
    commands: Mapping[str, object],
) -> list[str]:
    reasons: list[str] = []
    reasons.extend(
        _check_publisher_metadata(
            commands.get("publisher") or {},
            expected_type=COMMAND_TYPE,
            expected_source=COMMAND_SOURCE,
            expected_reliability="RELIABLE",
            expected_durability="VOLATILE",
            label=COMMAND_TOPIC,
        )
    )
    if commands.get("settled") is not True:
        reasons.append("{} publisher discovery did not settle".format(COMMAND_TOPIC))
    samples = commands.get("samples")
    if not isinstance(samples, int) or isinstance(samples, bool) or samples < 0:
        reasons.append("command observation sample count is missing/invalid")
    elif samples != 0:
        reasons.append(
            "observed {} command sample(s) on {}; expected exactly 0".format(
                samples, COMMAND_TOPIC
            )
        )
    start_s = commands.get("window_start_s")
    end_s = commands.get("window_end_s")
    if isinstance(start_s, (int, float)) and isinstance(end_s, (int, float)):
        if not (float(end_s) > float(start_s)):
            reasons.append("command observation window is empty or inverted")
    else:
        reasons.append("command observation window timestamps missing")
    if commands.get("publisher_graph_changed"):
        reasons.append(
            "{} publisher graph changed during the window".format(COMMAND_TOPIC)
        )
    return reasons


def _evaluate_outcome(
    outcome: Mapping[str, object], mode: str, expected: Mapping[str, object]
) -> list[str]:
    reasons: list[str] = []
    kind = outcome.get("kind")
    goal_accepted = outcome.get("goal_accepted") is True
    terminal_status = outcome.get("terminal_status")
    result_received = outcome.get("result_received") is True
    error_code = outcome.get("error_code")
    if mode in ("joint", "pose"):
        if kind != "success":
            reasons.append(
                "expected a successful plan, got outcome {!r}".format(kind)
            )
        elif not goal_accepted:
            reasons.append("success requires an accepted goal")
        elif terminal_status != STATUS_SUCCEEDED:
            reasons.append(
                "success requires terminal STATUS_SUCCEEDED, got {!r}".format(
                    outcome.get("terminal_status_name")
                )
            )
        elif not result_received:
            reasons.append("success requires a typed result")
        elif error_code != int(expected.get("success_error_code", SUCCESS_ERROR_CODE)):
            reasons.append(
                "success outcome error_code {!r} != {}".format(
                    error_code, expected.get("success_error_code", SUCCESS_ERROR_CODE)
                )
            )
        else:
            point_count = outcome.get("trajectory_point_count")
            min_points = int(
                expected.get("trajectory_min_points", DEFAULT_TRAJECTORY_MIN_POINTS)
            )
            if (
                not isinstance(point_count, int)
                or isinstance(point_count, bool)
                or point_count < min_points
            ):
                reasons.append(
                    "successful plan has empty trajectory (point_count={})".format(
                        point_count
                    )
                )
            joint_names = outcome.get("trajectory_joint_names") or []
            if not joint_names:
                reasons.append("successful plan has no trajectory joint names")
    else:  # blocked
        if kind != "non_success":
            reasons.append(
                "blocked mode requires a deterministic non-success result, got outcome {!r}".format(
                    kind
                )
            )
        if not goal_accepted:
            reasons.append("blocked mode requires an accepted goal")
        if not result_received:
            reasons.append("blocked mode requires a typed result")
        if terminal_status not in BLOCKED_ALLOWED_TERMINAL_STATUSES:
            reasons.append(
                "blocked mode terminal status {!r} not in allowed planning-failure statuses".format(
                    outcome.get("terminal_status_name")
                )
            )
        if error_code is None or error_code == 0:
            reasons.append(
                "blocked mode error_code must be an explicit integer, got {!r}".format(
                    error_code
                )
            )
        elif error_code == int(expected.get("success_error_code", SUCCESS_ERROR_CODE)):
            reasons.append("blocked mode returned unexpected success (error_code {})".format(error_code))
        elif error_code not in MOVEIT_PLANNING_FAILURE_CODES:
            reasons.append(
                "blocked mode error_code {!r} is not a documented planning failure".format(
                    error_code
                )
            )
    return reasons


def evaluate_smoke(
    observations: Mapping[str, object], expected: Mapping[str, object]
) -> tuple[bool, list[str]]:
    """Fail-closed evaluation of a complete smoke observation snapshot.

    Returns ``(ready, reasons)``.  Joint/pose modes require an accepted goal,
    terminal ``STATUS_SUCCEEDED``, ``MoveItErrorCodes.SUCCESS``, and a nonempty
    trajectory; blocked mode requires an accepted goal, a typed result, a
    terminal status in the documented planning-failure statuses, and an
    explicit MoveIt planning failure code from the allowlist.  Every mode
    additionally requires a fresh canonical pass with exact publisher metadata
    and scenario identity, an exact ``/move_action`` MoveGroup server with an
    exact result-service type, and zero ``/isaac_joint_commands`` samples
    across the send/result/tail window with the exact publisher proven.
    """
    reasons: list[str] = []
    mode = str(expected.get("mode", ""))
    if mode not in MODES:
        return (False, ["unknown expected mode {!r}".format(mode)])

    reasons.extend(_evaluate_readiness(observations.get("readiness") or {}, expected))
    reasons.extend(_evaluate_endpoint(observations.get("endpoint") or {}, expected))
    reasons.extend(_evaluate_commands(observations.get("command_observations") or {}))
    reasons.extend(_evaluate_outcome(observations.get("outcome") or {}, mode, expected))

    return (not reasons, reasons)


# ---------------------------------------------------------------------------
# Report builders (deterministic)
# ---------------------------------------------------------------------------


def build_report(
    observations: Mapping[str, object],
    evaluation: tuple[bool, list[str]],
    meta: Mapping[str, object],
) -> dict[str, object]:
    """Assemble the compact canonical acceptance evidence mapping.

    The returned mapping is a deterministic function of *observations*,
    *evaluation*, and *meta*; :func:`serialize_report` produces identical bytes
    for identical inputs.
    """
    ready, reasons = evaluation
    return {
        "schema_version": 1,
        "tool": "ompl_plan_smoke",
        "mode": str(meta.get("mode", "")),
        "scenario": str(meta.get("scenario", "")),
        "goal": dict(meta.get("goal", {}) or {}),
        "pipeline_id": str(meta.get("pipeline_id", PIPELINE_ID)),
        "plan_only": bool(meta.get("plan_only", PLAN_ONLY)),
        "readiness": dict(observations.get("readiness", {}) or {}),
        "endpoint": dict(observations.get("endpoint", {}) or {}),
        "command_observations": dict(observations.get("command_observations", {}) or {}),
        "outcome": dict(observations.get("outcome", {}) or {}),
        "evaluation": {"ready": bool(ready), "reasons": sorted(str(r) for r in reasons)},
        "meta": dict(meta.get("meta", {}) or {}),
    }


def fail_closed_report(
    *, mode: str, scenario: str, blocker: str, meta: Mapping[str, object]
) -> dict[str, object]:
    """Build a bounded fail-closed report when the live attempt cannot run."""
    return {
        "schema_version": 1,
        "tool": "ompl_plan_smoke",
        "mode": str(mode),
        "scenario": str(scenario),
        "blocker": str(blocker),
        "evaluation": {"ready": False, "reasons": ["blocked: " + str(blocker)]},
        "meta": dict(meta),
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse live-client command-line arguments (ROS-free)."""
    parser = argparse.ArgumentParser(
        description="Deterministic OMPL plan-only smoke against the integrated overlay."
    )
    parser.add_argument(
        "--mode",
        choices=list(MODES),
        required=True,
        help="smoke mode: joint, pose, or blocked (selects the scenario)",
    )
    parser.add_argument(
        "--report",
        default=DEFAULT_REPORT_PATH,
        help="compact JSON acceptance evidence path",
    )
    parser.add_argument(
        "--scenario",
        default=None,
        help="scenario JSON path (default derived from mode)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_ACTION_TIMEOUT,
        help="bounded action deadline in seconds (default %(default).0f)",
    )
    parser.add_argument(
        "--readiness-timeout",
        type=float,
        default=DEFAULT_READINESS_TIMEOUT,
        help="bounded readiness wait in seconds (default %(default).0f)",
    )
    parser.add_argument(
        "--readiness-max-age",
        type=float,
        default=DEFAULT_READINESS_MAX_AGE_S,
        help="maximum acceptable readiness sample age in seconds",
    )
    parser.add_argument(
        "--post-result-tail",
        type=float,
        default=DEFAULT_POST_RESULT_TAIL_S,
        help="post-result command-observation tail in seconds (default %(default).2f)",
    )
    parser.add_argument(
        "--allowed-planning-time",
        type=float,
        default=DEFAULT_ALLOWED_PLANNING_TIME,
        help="MotionPlanRequest.allowed_planning_time in seconds",
    )
    parser.add_argument(
        "--group-name", default=GROUP_NAME, help="MoveGroup planning group"
    )
    parser.add_argument(
        "--planner-id", default=DEFAULT_PLANNER_ID, help="optional planner id"
    )
    parser.add_argument(
        "--position-tolerance",
        type=float,
        default=DEFAULT_POSITION_TOLERANCE,
        help="pose goal position tolerance in meters",
    )
    arguments = parser.parse_args(argv)
    if arguments.timeout <= 0 or arguments.readiness_timeout <= 0:
        parser.error("--timeout and --readiness-timeout must be positive")
    if arguments.readiness_max_age <= 0:
        parser.error("--readiness-max-age must be positive")
    if arguments.post_result_tail < 0:
        parser.error("--post-result-tail must be nonnegative")
    return arguments


def scenario_path_for(args: argparse.Namespace) -> Path:
    """Return the scenario JSON path, defaulting from ``--mode``."""
    if args.scenario:
        return Path(args.scenario)
    return REPO_ROOT / "simulation" / "scenarios" / "{}.json".format(
        MODE_SCENARIOS[args.mode]
    )


# ---------------------------------------------------------------------------
# Humble-only live seam
# ---------------------------------------------------------------------------


def _endpoint_label(node_name: str, node_namespace: str) -> str:
    namespace = str(node_namespace or "").rstrip("/")
    if namespace:
        return "{}/{}".format(namespace, str(node_name))
    return "/" + str(node_name)


class OmplPlanSmokeClient:
    """Bounded live smoke client.  All ROS imports occur inside methods.

    This class is the Humble-only live seam: importing the module never
    executes these imports, so pure collection under simulator CPython 3.12
    stays ROS-free.
    """

    def __init__(self, node: object, args: argparse.Namespace) -> None:
        self._node = node
        self._args = args
        self._readiness_obs: dict[str, object] | None = None
        self._readiness_valid = False
        self._readiness_received_at_s: float | None = None
        self._readiness_gate_refreshed_at: float | None = None
        self._readiness_expected: dict[str, object] | None = None
        self._readiness_counts = {"pass": 0, "fail": 0, "malformed": 0}
        self._readiness_window_open = False
        self._readiness_window_counts = {
            "pass": 0,
            "fail": 0,
            "malformed": 0,
            "identity_invalid": 0,
        }
        self._readiness_window: dict[str, object] = {"start_s": None, "end_s": None}
        self._readiness_publisher: dict[str, object] = {}
        self._readiness_pub_changed = False
        self._command_samples = 0
        self._command_sample_events: list[dict[str, object]] = []
        self._window_open = False
        self._command_window: dict[str, object] = {
            "start_s": None,
            "end_s": None,
            "send_s": None,
            "result_s": None,
            "tail_s": None,
        }
        self._command_publisher: dict[str, object] = {}
        self._command_pub_changed = False
        self._setup_subscriptions()

    # -- subscriptions ----------------------------------------------------

    def _setup_subscriptions(self) -> None:
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String

        readiness_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self._node.create_subscription(
            String, READINESS_TOPIC, self._on_readiness, readiness_qos
        )
        self._node.create_subscription(
            JointState, COMMAND_TOPIC, self._on_command, command_qos
        )

    def _on_readiness(self, message: object) -> None:
        try:
            text = str(getattr(message, "data", message))
        except Exception:  # noqa: BLE001 - a malformed message is a malformed sample
            text = ""
        parsed = parse_readiness_payload(text)
        valid = bool(parsed.get("valid"))
        payload = parsed.get("payload") or {}
        state = payload.get("state")
        if valid and state in (READINESS_STATE_PASS, READINESS_STATE_FAIL):
            self._readiness_obs = dict(payload)
            self._readiness_received_at_s = time.monotonic()
        else:
            self._readiness_obs = dict(payload) if isinstance(payload, dict) else None
            self._readiness_received_at_s = None
        self._readiness_valid = valid and state == READINESS_STATE_PASS
        if not valid:
            self._readiness_counts["malformed"] += 1
        elif state == READINESS_STATE_PASS:
            self._readiness_counts["pass"] += 1
        elif state == READINESS_STATE_FAIL:
            self._readiness_counts["fail"] += 1
        if self._readiness_window_open:
            if not valid:
                self._readiness_window_counts["malformed"] += 1
            elif state == READINESS_STATE_FAIL:
                self._readiness_window_counts["fail"] += 1
            elif state == READINESS_STATE_PASS:
                self._readiness_window_counts["pass"] += 1
                expected = self._readiness_expected
                if expected and readiness_identity_reasons(payload, expected):
                    self._readiness_window_counts["identity_invalid"] += 1

    def _on_command(self, message: object) -> None:
        if not self._window_open:
            return
        stamp_ns = 0
        try:
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            if stamp is not None:
                stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            stamp_ns = 0
        self._command_samples += 1
        self._command_sample_events.append(
            {"stamp_ns": stamp_ns, "received_at_s": time.monotonic()}
        )

    # -- publisher metadata ------------------------------------------------

    def _read_publisher_metadata(self, topic: str) -> dict[str, object]:
        publishers = self._node.get_publishers_info_by_topic(topic)
        labels = [
            _endpoint_label(p.node_name, p.node_namespace) for p in publishers
        ]
        types = sorted({str(getattr(p, "topic_type", "")) for p in publishers})
        qos = None
        if publishers:
            profile = getattr(publishers[0], "qos_profile", None)
            qos = {
                "reliability": _qos_short(getattr(profile, "reliability", "")),
                "durability": _qos_short(getattr(profile, "durability", "")),
                "depth": int(getattr(profile, "depth", 0)),
            }
        return {
            "topic": topic,
            "count": len(publishers),
            "source": labels[0] if len(labels) == 1 else sorted(labels),
            "sources": sorted(labels),
            "types": types,
            "type": types[0] if types else "",
            "qos": qos,
        }

    def _wait_publisher_discovery(
        self,
        topic: str,
        *,
        timeout_s: float,
        expected_type: str,
        expected_source: str,
    ) -> dict[str, object]:
        """Boundedly wait for a stable exact publisher observation.

        Requires two consecutive identical stable metadata observations (a
        bounded discovery-settle rule).  The node is spun while waiting so
        graph/callback state advances.  Returns the metadata plus ``settled``.
        """
        import rclpy

        expected = (expected_type, expected_source)
        deadline = time.monotonic() + float(timeout_s)
        first: dict[str, object] | None = None
        first_time: float | None = None
        while time.monotonic() < deadline:
            obs = self._read_publisher_metadata(topic)
            if first is not None and obs == first:
                settled = (
                    obs.get("type") == expected[0]
                    and obs.get("source") == expected[1]
                )
                if settled:
                    return {
                        **obs,
                        "settled": True,
                        "settle_seconds": (
                            time.monotonic() - first_time
                            if first_time is not None
                            else 0.0
                        ),
                    }
            first = obs
            first_time = time.monotonic()
            rclpy.spin_once(self._node, timeout_sec=0.1)
        obs = self._read_publisher_metadata(topic)
        return {
            **obs,
            "settled": False,
            "detail": "publisher discovery did not settle for {}".format(topic),
        }

    def _probe_readiness_publisher(self, timeout_s: float) -> dict[str, object]:
        obs = self._wait_publisher_discovery(
            READINESS_TOPIC,
            timeout_s=timeout_s,
            expected_type=READINESS_TYPE,
            expected_source=READINESS_SOURCE,
        )
        obs["expected_depth"] = 1
        return obs

    def _probe_command_publisher(self, timeout_s: float) -> dict[str, object]:
        obs = self._wait_publisher_discovery(
            COMMAND_TOPIC,
            timeout_s=timeout_s,
            expected_type=COMMAND_TYPE,
            expected_source=COMMAND_SOURCE,
        )
        # Verified gateway source truth: command_gateway.py creates
        # /isaac_joint_commands with RELIABLE/KEEP_LAST depth=50.
        obs["expected_depth"] = 50
        return obs

    # -- readiness gating ---------------------------------------------------

    def _wait_for_readiness(self, timeout_s: float, expected: Mapping[str, object]) -> bool:
        import rclpy

        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            obs = self._readiness_obs
            if obs is None or not self._readiness_valid or obs.get("state") != READINESS_STATE_PASS:
                continue
            if not readiness_identity_reasons(obs, expected):
                return True
        return False

    def _ensure_fresh_readiness(
        self, max_age_s: float, timeout_s: float, expected: Mapping[str, object]
    ) -> bool:
        """Re-verify a fresh canonical pass immediately before send."""
        import rclpy

        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            obs = self._readiness_obs
            if obs is None or not self._readiness_valid or obs.get("state") != READINESS_STATE_PASS:
                continue
            now = time.monotonic()
            received_at = self._readiness_received_at_s
            age = now - float(received_at) if isinstance(received_at, (int, float)) else float("inf")
            if age <= float(max_age_s) and not readiness_identity_reasons(obs, expected):
                self._readiness_gate_refreshed_at = now
                return True
        return False

    # -- graph probes -------------------------------------------------------

    def _service_servers(self) -> dict[str, list[tuple[str, list[str]]]]:
        servers: dict[str, list[tuple[str, list[str]]]] = {}
        for node_name, node_namespace in self._node.get_node_names_and_namespaces():
            label = _endpoint_label(node_name, node_namespace)
            try:
                by_node = self._node.get_service_names_and_types_by_node(
                    node_name, node_namespace
                )
            except Exception:  # noqa: BLE001 - transient graph reads must not crash
                continue
            for service_name, types in by_node:
                servers.setdefault(str(service_name), []).append(
                    (label, [str(t) for t in types])
                )
        return servers

    def _probe_move_action(self) -> dict[str, object]:
        goal_service = "{}/_action/send_goal".format(MOVE_ACTION)
        result_service = "{}/_action/get_result".format(MOVE_ACTION)
        servers = self._service_servers()
        goal_servers = servers.get(goal_service, [])
        result_servers = servers.get(result_service, [])
        count = len(goal_servers)
        goal_types = sorted({t for _label, tlist in goal_servers for t in tlist})
        sources = sorted({label for label, _tlist in goal_servers})
        source = sources[0] if len(sources) == 1 else ""
        observed_type = goal_types[0] if goal_types else ""
        return {
            "action": MOVE_ACTION,
            "count": count,
            "source": source,
            "sources": sources,
            "observed_types": goal_types,
            "kind": _action_name_from_type(observed_type),
            "result_service_present": len(result_servers) >= 1,
            "result_service_types": sorted(
                {t for _label, tlist in result_servers for t in tlist}
            ),
        }

    def _endpoint_ok(self, endpoint: Mapping[str, object]) -> bool:
        """Endpoint sanity used to decide whether to run the goal.

        Requires the exact observed ``_action/get_result`` type before sending:
        a goal is never sent against a malformed result endpoint merely to
        reject afterward (fix1 spec M2).
        """
        expected_result_type = derive_result_service_type(MOVE_ACTION_TYPE)
        return (
            endpoint.get("count") == 1
            and endpoint.get("source") == MOVE_ACTION_SOURCE
            and endpoint.get("kind") == MOVE_ACTION_KIND
            and endpoint.get("result_service_present") is True
            and sorted(
                str(value) for value in (endpoint.get("result_service_types") or [])
            )
            == [expected_result_type]
        )

    # -- goal conversion -----------------------------------------------------

    def _to_moveit_goal(self, goal: object) -> object:
        from geometry_msgs.msg import Pose, Vector3
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import (
            BoundingVolume,
            Constraints,
            JointConstraint,
            MotionPlanRequest,
            OrientationConstraint,
            PlanningOptions,
            PositionConstraint,
        )
        from shape_msgs.msg import SolidPrimitive

        request = MotionPlanRequest()
        request.group_name = str(goal.group_name)
        request.pipeline_id = str(goal.pipeline_id)
        request.planner_id = str(goal.planner_id)
        request.allowed_planning_time = float(goal.allowed_planning_time)
        request.num_planning_attempts = int(goal.num_planning_attempts)
        request.max_velocity_scaling_factor = 1.0
        request.max_acceleration_scaling_factor = 1.0

        if goal_kind(goal) == "joint":
            constraints = Constraints()
            constraints.name = "joint_goal"
            for name, position in goal.joint_positions.items():
                tolerance = float(goal.tolerances.get(name, 0.02))
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = str(name)
                joint_constraint.position = float(position)
                joint_constraint.tolerance_above = tolerance
                joint_constraint.tolerance_below = tolerance
                joint_constraint.weight = 1.0
                constraints.joint_constraints.append(joint_constraint)
            request.goal_constraints.append(constraints)
        else:
            constraints = Constraints()
            constraints.name = "pose_goal"
            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = str(goal.frame_id)
            position_constraint.link_name = str(goal.link_name)
            position_constraint.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
            position_constraint.weight = 1.0
            region = BoundingVolume()
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            half = float(goal.position_tolerance)
            primitive.dimensions = [half * 2.0, half * 2.0, half * 2.0]
            region.primitives.append(primitive)
            pose = Pose()
            pose.position.x = float(goal.position_xyz[0])
            pose.position.y = float(goal.position_xyz[1])
            pose.position.z = float(goal.position_xyz[2])
            pose.orientation.x = float(goal.orientation_xyzw[0])
            pose.orientation.y = float(goal.orientation_xyzw[1])
            pose.orientation.z = float(goal.orientation_xyzw[2])
            pose.orientation.w = float(goal.orientation_xyzw[3])
            region.primitive_poses.append(pose)
            position_constraint.constraint_region = region
            constraints.position_constraints.append(position_constraint)
            if goal.use_orientation:
                orientation_constraint = OrientationConstraint()
                orientation_constraint.header.frame_id = str(goal.frame_id)
                orientation_constraint.link_name = str(goal.link_name)
                orientation_constraint.orientation.x = float(goal.orientation_xyzw[0])
                orientation_constraint.orientation.y = float(goal.orientation_xyzw[1])
                orientation_constraint.orientation.z = float(goal.orientation_xyzw[2])
                orientation_constraint.orientation.w = float(goal.orientation_xyzw[3])
                tolerance = float(goal.orientation_tolerance)
                orientation_constraint.absolute_x_axis_tolerance = tolerance
                orientation_constraint.absolute_y_axis_tolerance = tolerance
                orientation_constraint.absolute_z_axis_tolerance = tolerance
                orientation_constraint.parameterization = 0  # XYZ_EULER_ANGLES
                orientation_constraint.weight = 1.0
                constraints.orientation_constraints.append(orientation_constraint)
            request.goal_constraints.append(constraints)

        move_group_goal = MoveGroup.Goal()
        move_group_goal.request = request
        move_group_goal.planning_options = PlanningOptions()
        move_group_goal.planning_options.plan_only = bool(goal.plan_only)
        return move_group_goal

    # -- action execution ----------------------------------------------------

    @staticmethod
    def _make_outcome(
        kind: str,
        detail: str,
        *,
        goal_accepted: bool = False,
        terminal_status: object | None = None,
        error_code: object | None = None,
        result_received: bool = False,
        trajectory_point_count: int = 0,
        trajectory_joint_names: Sequence[str] = (),
        planning_time: object | None = None,
        cancel_requested: bool = False,
        cancel_accepted: bool = False,
        cancel_confirmed: bool = False,
        cancel_terminal_status: object | None = None,
    ) -> dict[str, object]:
        return {
            "kind": kind,
            "detail": detail,
            "goal_accepted": bool(goal_accepted),
            "terminal_status": int(terminal_status)
            if terminal_status is not None
            else None,
            "terminal_status_name": goal_status_name(terminal_status),
            "error_code": int(error_code) if error_code is not None else None,
            "error_code_name": moveit_error_code_name(error_code),
            "result_received": bool(result_received),
            "trajectory_point_count": int(trajectory_point_count),
            "trajectory_joint_names": [str(name) for name in trajectory_joint_names],
            "planning_time": float(planning_time)
            if planning_time is not None
            else None,
            "cancel_requested": bool(cancel_requested),
            "cancel_accepted": bool(cancel_accepted),
            "cancel_confirmed": bool(cancel_confirmed),
            "cancel_terminal_status": int(cancel_terminal_status)
            if cancel_terminal_status is not None
            else None,
            "cancel_terminal_status_name": goal_status_name(cancel_terminal_status),
        }

    def _run_action(
        self, client: object, goal: object, deadline: float
    ) -> dict[str, object]:
        """Send the goal, wait for result, and handle timeout/cancel.

        Returns an outcome dict.  The command observation window is managed by
        the caller :meth:`_execute_goal`; this method records send/result times
        and keeps the window open through every terminal path.
        """
        import rclpy

        from action_msgs.msg import GoalStatus

        if not client.wait_for_server(timeout_sec=min(self._args.timeout, 10.0)):
            return self._make_outcome(
                "invalid", "action server unavailable on {}".format(MOVE_ACTION)
            )
        goal_message = self._to_moveit_goal(goal)
        self._command_window["send_s"] = time.monotonic()
        send_future = client.send_goal_async(goal_message)
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return self._make_outcome("timeout", "goal not accepted")
        rclpy.spin_until_future_complete(self._node, send_future, timeout_sec=remaining)
        if not send_future.done():
            return self._make_outcome("timeout", "goal not accepted")
        goal_handle = send_future.result()
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            return self._make_outcome("rejected", "goal rejected by the action server")
        result_future = goal_handle.get_result_async()
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return self._cancel_path(goal_handle, "planning deadline")
        rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=remaining)
        if not result_future.done():
            return self._cancel_path(goal_handle, "planning deadline")
        self._command_window["result_s"] = time.monotonic()
        response = result_future.result()
        if response is None:
            return self._make_outcome(
                "invalid",
                "empty action result",
                goal_accepted=True,
            )
        status = int(response.status)
        move_result = response.result
        error_code = int(move_result.error_code.val)
        trajectory = move_result.planned_trajectory
        joint_names = list(trajectory.joint_trajectory.joint_names)
        point_count = len(trajectory.joint_trajectory.points) + len(
            trajectory.multi_dof_joint_trajectory.points
        )
        if (
            status == GoalStatus.STATUS_SUCCEEDED
            and error_code == SUCCESS_ERROR_CODE
        ):
            kind = "success"
        else:
            kind = "non_success"
        return self._make_outcome(
            kind,
            "",
            goal_accepted=True,
            terminal_status=status,
            error_code=error_code,
            result_received=True,
            trajectory_point_count=point_count,
            trajectory_joint_names=joint_names,
            planning_time=float(move_result.planning_time),
        )

    def _cancel_path(self, goal_handle: object, detail: str) -> dict[str, object]:
        """Boundedly request cancel, confirm, and return a fail-closed timeout.

        The command window stays open (the caller closes it after the tail).
        ``cancel_confirmed`` means the goal actually reached ``STATUS_CANCELED``;
        if it instead ended ``SUCCEEDED``/``ABORTED`` concurrently, that terminal
        completion is recorded separately (``cancel_terminal_status``) rather
        than mislabeled as confirmed (fix1 adv D).  All unresolved cancellation
        paths remain fail-closed.
        """
        import rclpy

        from action_msgs.msg import GoalStatus

        cancel_requested = True
        cancel_accepted = False
        cancel_confirmed = False
        cancel_terminal_status: object | None = None
        try:
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self._node, cancel_future, timeout_sec=2.0)
            if cancel_future.done():
                cancel_response = cancel_future.result()
                cancel_accepted = bool(
                    getattr(cancel_response, "accepted", False)
                ) if cancel_response is not None else False
            if cancel_accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(
                    self._node, result_future, timeout_sec=2.0
                )
                if result_future.done():
                    response = result_future.result()
                    if response is not None:
                        cancel_terminal_status = int(response.status)
                        cancel_confirmed = (
                            cancel_terminal_status == GoalStatus.STATUS_CANCELED
                        )
        except Exception:  # noqa: BLE001 - unresolved cancellation stays fail-closed
            cancel_accepted = cancel_accepted
        return self._make_outcome(
            "timeout",
            detail,
            goal_accepted=True,
            cancel_requested=cancel_requested,
            cancel_accepted=cancel_accepted,
            cancel_confirmed=cancel_confirmed,
            cancel_terminal_status=cancel_terminal_status,
        )

    def _execute_goal(self, goal: object) -> dict[str, object]:
        """Run the goal with the command observation window open across the
        full send/result/cancel/tail span.
        """
        import rclpy

        from moveit_msgs.action import MoveGroup
        from rclpy.action import ActionClient

        client = ActionClient(self._node, MoveGroup, MOVE_ACTION)
        deadline = time.monotonic() + float(self._args.timeout)
        tail_s = float(self._args.post_result_tail)
        self._command_window["start_s"] = time.monotonic()
        self._command_window["send_s"] = None
        self._command_window["result_s"] = None
        self._command_window["tail_s"] = None
        self._window_open = True
        try:
            return self._run_action(client, goal, deadline)
        finally:
            # Fixed documented post-result tail: keep the window open while
            # spinning so a command emitted during teardown cannot escape.
            tail_end = time.monotonic() + tail_s
            while time.monotonic() < tail_end:
                rclpy.spin_once(self._node, timeout_sec=0.05)
            # Final bounded nonblocking executor drain while the window is
            # still open, then snapshot and close; this eliminates the
            # unobserved close sliver as far as rclpy permits (fix1 adv C).
            rclpy.spin_once(self._node, timeout_sec=0.0)
            self._command_window["tail_s"] = time.monotonic()
            self._window_open = False
            self._command_window["end_s"] = time.monotonic()
            try:
                client.destroy()
            except Exception:  # noqa: BLE001 - destroy must never mask the result
                pass

    # -- observations ---------------------------------------------------------

    def _assemble_readiness_obs(
        self, expected: Mapping[str, object], gate_refreshed: bool
    ) -> dict[str, object]:
        payload = self._readiness_obs or {}
        now_s = time.monotonic()
        received_at = self._readiness_received_at_s
        identity: dict[str, object] = {}
        if payload:
            evidence = payload.get("evidence") or {}
            identities = (evidence.get("shared_report") or {}).get("identities") or {}
            fixture = (evidence.get("fixture_status") or {}).get("status") or {}
            identity = {
                "scenario_id": {
                    "ok": str(identities.get("scenario_id"))
                    == str(expected.get("scenario_id", "")),
                    "observed": identities.get("scenario_id"),
                    "expected": expected.get("scenario_id"),
                },
                "seed": {
                    "ok": _intish(identities.get("seed"))
                    == _intish(expected.get("seed", -1)),
                    "observed": identities.get("seed"),
                    "expected": expected.get("seed"),
                },
                "scenario_declaration_sha256": {
                    "ok": str(identities.get("scenario_declaration_sha256"))
                    == str(expected.get("scenario_declaration_sha256", "")),
                    "observed": identities.get("scenario_declaration_sha256"),
                    "expected": expected.get("scenario_declaration_sha256"),
                },
                "planning_scene_sha256": {
                    "ok": str(identities.get("planning_scene_sha256"))
                    == str(expected.get("planning_scene_sha256", "")),
                    "observed": identities.get("planning_scene_sha256"),
                    "expected": expected.get("planning_scene_sha256"),
                },
                "fixture_scenario": {
                    "ok": str(fixture.get("scenario"))
                    == str(expected.get("scenario_id", "")),
                    "observed": fixture.get("scenario"),
                    "expected": expected.get("scenario_id"),
                },
                "fixture_revision": {
                    "ok": str(fixture.get("revision"))
                    == str(expected.get("planning_scene_revision", "")),
                    "observed": fixture.get("revision"),
                    "expected": expected.get("planning_scene_revision"),
                },
                "fixture_revision_digest": {
                    "ok": str(fixture.get("revision_digest"))
                    == str(expected.get("planning_scene_revision_digest", "")),
                    "observed": fixture.get("revision_digest"),
                    "expected": expected.get("planning_scene_revision_digest"),
                },
                "fixture_owned_ids": {
                    "ok": tuple(str(x) for x in (fixture.get("owned_ids") or ()))
                    == tuple(
                        str(x) for x in (expected.get("planning_scene_owned_ids") or ())
                    ),
                    "observed": list(fixture.get("owned_ids") or ()),
                    "expected": list(expected.get("planning_scene_owned_ids") or ()),
                },
                "fixture_target_source_id": {
                    "ok": str(fixture.get("target_source_id"))
                    == str(expected.get("planning_scene_target_source_id", "")),
                    "observed": fixture.get("target_source_id"),
                    "expected": expected.get("planning_scene_target_source_id"),
                },
                "fixture_target_handoff": {
                    "ok": str(fixture.get("target_handoff"))
                    == str(expected.get("planning_scene_target_handoff", "")),
                    "observed": fixture.get("target_handoff"),
                    "expected": expected.get("planning_scene_target_handoff"),
                },
                # Record-only identities: no local independent value exists.
                "integrated_sha256": {
                    "ok": True,
                    "observed": identities.get("integrated_sha256"),
                    "expected": None,
                },
                "model_fingerprint": {
                    "ok": True,
                    "observed": identities.get("model_fingerprint"),
                    "expected": None,
                },
                "provider_manifest_sha256": {
                    "ok": True,
                    "observed": identities.get("provider_manifest_sha256"),
                    "expected": None,
                },
            }
        producer_age = None
        published_at = payload.get("published_at")
        if isinstance(published_at, (int, float)) and not isinstance(published_at, bool):
            producer_age = now_s - float(published_at)
        consumer_age = (
            now_s - float(received_at) if isinstance(received_at, (int, float)) else None
        )
        return {
            "valid": bool(self._readiness_valid),
            "state": payload.get("state"),
            "ready": payload.get("ready"),
            "schema_version": payload.get("schema_version"),
            "reasons": payload.get("reasons"),
            "published_at": payload.get("published_at"),
            "received_at_s": received_at,
            "now_s": now_s,
            "producer_age_s": producer_age,
            "consumer_age_s": consumer_age,
            "gate_refreshed": bool(gate_refreshed),
            "gate_refreshed_at_s": self._readiness_gate_refreshed_at,
            "publisher": self._readiness_publisher,
            "identity": identity,
            "counts": dict(self._readiness_counts),
            "window_counts": dict(self._readiness_window_counts),
            "any_fail_in_window": self._readiness_window_counts["fail"] > 0,
            "any_malformed_in_window": self._readiness_window_counts["malformed"] > 0,
            "any_identity_invalid_in_window": self._readiness_window_counts[
                "identity_invalid"
            ]
            > 0,
            "publisher_graph_changed": bool(self._readiness_pub_changed),
            "window": {
                "start_s": self._readiness_window.get("start_s"),
                "end_s": self._readiness_window.get("end_s"),
            },
        }

    def _assemble_command_obs(self) -> dict[str, object]:
        start_s = self._command_window.get("start_s")
        end_s = self._command_window.get("end_s")
        send_s = self._command_window.get("send_s")
        result_s = self._command_window.get("result_s")
        tail_s = self._command_window.get("tail_s")
        return {
            "topic": COMMAND_TOPIC,
            "samples": self._command_samples,
            "sample_events": list(self._command_sample_events),
            "window_start_s": start_s,
            "window_end_s": end_s,
            "send_time_s": send_s,
            "result_time_s": result_s,
            "tail_time_s": tail_s,
            "tail_s": float(self._args.post_result_tail),
            "duration_s": (
                float(end_s) - float(start_s)
                if isinstance(start_s, (int, float)) and isinstance(end_s, (int, float))
                else None
            ),
            "publisher": self._command_publisher,
            "settled": bool(self._command_publisher.get("settled")),
            "publisher_graph_changed": bool(self._command_pub_changed),
        }

    # -- run -----------------------------------------------------------------

    def run(self) -> dict[str, object]:
        args = self._args
        meta_info = self._environment_meta()
        try:
            scenario = load_scenario(scenario_path_for(args))
            scenario_id = str(scenario.get("id", ""))
            goal = build_goal(
                args.mode,
                scenario,
                group_name=args.group_name,
                planner_id=args.planner_id,
                allowed_planning_time=args.allowed_planning_time,
                position_tolerance=args.position_tolerance,
            )
            expected = dict(scenario_expected_identities(scenario))
        except ValueError as exc:
            return fail_closed_report(
                mode=args.mode,
                scenario="",
                blocker="scenario/goal validation failed: {}".format(exc),
                meta=meta_info,
            )
        expected.update(
            {
                "mode": args.mode,
                "action_type": MOVE_ACTION_TYPE,
                "source": MOVE_ACTION_SOURCE,
                "readiness_max_age_s": args.readiness_max_age,
                "success_error_code": SUCCESS_ERROR_CODE,
                "trajectory_min_points": DEFAULT_TRAJECTORY_MIN_POINTS,
            }
        )
        # Store the identity contract so the readiness callback can classify
        # wrong-identity pass samples observed during the window.
        self._readiness_expected = expected

        # 1. Canonical pass with identity agreement.
        if not self._wait_for_readiness(args.readiness_timeout, expected):
            return fail_closed_report(
                mode=args.mode,
                scenario=scenario_id,
                blocker="integrated readiness did not reach a canonical pass on {} within {:.1f} s".format(
                    READINESS_TOPIC, args.readiness_timeout
                ),
                meta=meta_info,
            )

        # 2. Readiness publisher graph proof.
        self._readiness_publisher = self._probe_readiness_publisher(
            min(args.readiness_timeout, 15.0)
        )
        if self._readiness_publisher.get("settled") is not True:
            return fail_closed_report(
                mode=args.mode,
                scenario=scenario_id,
                blocker="readiness publisher proof failed: {}".format(
                    self._readiness_publisher.get("detail", "not settled")
                ),
                meta=meta_info,
            )

        # 3. Command publisher graph proof.
        self._command_publisher = self._probe_command_publisher(
            min(args.readiness_timeout, 15.0)
        )
        if self._command_publisher.get("settled") is not True:
            return fail_closed_report(
                mode=args.mode,
                scenario=scenario_id,
                blocker="command publisher proof failed: {}".format(
                    self._command_publisher.get("detail", "not settled")
                ),
                meta=meta_info,
            )

        # 4. Endpoint probe.
        endpoint_obs = self._probe_move_action()

        # 5. Re-verify readiness freshness as the final check immediately
        # before opening the window/sending (after action + command publisher
        # discovery).  A stale/wrong-identity pass at this point rejects.
        gate_refreshed = self._ensure_fresh_readiness(
            args.readiness_max_age, min(args.timeout, 10.0), expected
        )
        if not gate_refreshed:
            return fail_closed_report(
                mode=args.mode,
                scenario=scenario_id,
                blocker="integrated readiness was not fresh immediately before send",
                meta=meta_info,
            )

        # 6. Open the readiness observation window around the request/result/tail.
        self._readiness_window_open = True
        self._readiness_window["start_s"] = time.monotonic()
        self._readiness_window_counts = {
            "pass": 0,
            "fail": 0,
            "malformed": 0,
            "identity_invalid": 0,
        }
        try:
            if not self._endpoint_ok(endpoint_obs):
                outcome_obs = self._make_outcome(
                    "invalid",
                    "endpoint metadata does not match: {}".format(
                        json.dumps(endpoint_obs, sort_keys=True, separators=(",", ":"))
                    ),
                )
                self._command_window["start_s"] = time.monotonic()
                self._command_window["send_s"] = None
                self._command_window["result_s"] = None
                self._command_window["tail_s"] = None
                self._command_window["end_s"] = time.monotonic()
            else:
                outcome_obs = self._execute_goal(goal)
        finally:
            self._readiness_window_open = False
            self._readiness_window["end_s"] = time.monotonic()
            # Publisher-graph stability across the window: a bounded quick
            # snapshot (single read, no settle wait) detects any graph change.
            tail_readiness = self._read_publisher_metadata(READINESS_TOPIC)
            tail_command = self._read_publisher_metadata(COMMAND_TOPIC)
            self._readiness_pub_changed = not self._publisher_same(
                self._readiness_publisher, tail_readiness
            )
            self._command_pub_changed = not self._publisher_same(
                self._command_publisher, tail_command
            )

        readiness_obs = self._assemble_readiness_obs(expected, gate_refreshed)
        command_obs = self._assemble_command_obs()
        observations: dict[str, object] = {
            "readiness": readiness_obs,
            "endpoint": endpoint_obs,
            "command_observations": command_obs,
            "outcome": outcome_obs,
        }
        evaluation = evaluate_smoke(observations, expected)
        return build_report(
            observations,
            evaluation,
            {
                "mode": args.mode,
                "scenario": scenario_id,
                "goal": goal_to_dict(goal),
                "pipeline_id": PIPELINE_ID,
                "plan_only": PLAN_ONLY,
                "meta": meta_info,
            },
        )

    @staticmethod
    def _publisher_same(
        first: Mapping[str, object], second: Mapping[str, object]
    ) -> bool:
        if first.get("count") != second.get("count"):
            return False
        if first.get("source") != second.get("source"):
            return False
        if first.get("type") != second.get("type"):
            return False
        if first.get("qos") != second.get("qos"):
            return False
        return True

    def _environment_meta(self) -> dict[str, object]:
        import rclpy

        meta = _shell_meta()
        meta["rclpy"] = str(getattr(rclpy, "__version__", ""))
        return meta


def _intish(value: object) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return -1


def main(argv: Sequence[str] | None = None) -> int:
    """Live Humble entry point.  All ROS imports happen inside this function."""
    # Parse first so ``--help`` exits without importing rclpy (argparse raises
    # SystemExit before the Humble-only seam is reached).
    arguments = parse_args(argv)
    try:
        import rclpy

        rclpy.init(args=argv)
        node = rclpy.create_node("ompl_plan_smoke")
        try:
            client = OmplPlanSmokeClient(node, arguments)
            report = client.run()
        finally:
            try:
                node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
    except KeyboardInterrupt:
        report = fail_closed_report(
            mode=arguments.mode,
            scenario="",
            blocker="interrupted",
            meta=_exception_meta(),
        )
    except Exception as exc:  # noqa: BLE001 - every representable failure writes a report
        report = fail_closed_report(
            mode=arguments.mode,
            scenario="",
            blocker="live attempt failed: {}".format(exc),
            meta=_exception_meta(),
        )
    try:
        data = serialize_report(report)
        write_report_atomic(arguments.report, data)
    except Exception as exc:  # noqa: BLE001 - unwritable path gets a stderr diagnostic
        target = Path(arguments.report)
        temporary = target.with_suffix(target.suffix + ".tmp")
        try:
            if temporary.exists():
                temporary.unlink()
        except OSError:
            pass
        print(
            "ERROR: cannot write report to {}: {}".format(arguments.report, exc),
            file=sys.stderr,
        )
        return 1
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0 if report["evaluation"]["ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
