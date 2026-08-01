"""Deterministic OMPL plan-only smoke (Task 7).

This module is ROS-free at import time and runs under simulator CPython 3.12:
it imports neither ``rclpy``, nor ``rclpy.action``, nor ``moveit_msgs``, nor
any generated message type at module scope.  ``main()`` is the live Humble
client seam: it imports ``rclpy`` and constructs
:class:`OmplPlanSmokeClient`, whose methods import ``rclpy`` / ``moveit_msgs``
only inside their bodies (the Humble-only live seam).

Live behavior (only when the Task 6 overlay is running and ready):

1. Wait for a fresh ``pass`` on ``/sim/status/integrated_manipulation``.
2. Probe ``/move_action`` and require exactly one ``moveit_msgs/action/MoveGroup``
   action server with observed action-kind/type/cardinality/source metadata.
3. Send a goal with ``request.pipeline_id="ompl"`` and
   ``planning_options.plan_only=true``, observing ``/isaac_joint_commands`` for
   zero command samples across the full request/result window.  The MoveGroup
   action client is the only action client; no execute-trajectory, controller,
   or task action client is constructed.
4. Joint/pose modes require a successful MoveIt result with a nonempty
   trajectory; blocked mode requires a deterministic non-success result.
5. Write compact canonical JSON acceptance evidence atomically.

When the readiness gate cannot be reached within the bounded readiness timeout
the client writes a fail-closed report recording the exact blocker and exits
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
MOVE_ACTION_KIND = "MoveGroup"
READINESS_TOPIC = "/sim/status/integrated_manipulation"
READINESS_STATE_PASS = "pass"
COMMAND_TOPIC = "/isaac_joint_commands"

PIPELINE_ID = "ompl"
PLAN_ONLY = True
SUCCESS_ERROR_CODE = 1

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
    """Atomically write *data* to *path* (sibling temp + ``os.replace``)."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_bytes(data)
    os.replace(temporary, target)
    return target


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


# ---------------------------------------------------------------------------
# Endpoint type derivation
# ---------------------------------------------------------------------------


def derive_goal_service_type(action_type: str) -> str:
    """Derive the canonical ``_action/send_goal`` service type for an action."""
    marker = "/action/"
    if marker not in action_type:
        return ""
    package, action = action_type.split(marker, 1)
    return "{}/action/{}_SendGoal".format(package, action)


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


# ---------------------------------------------------------------------------
# Pure evaluator
# ---------------------------------------------------------------------------


def evaluate_smoke(
    observations: Mapping[str, object], expected: Mapping[str, object]
) -> tuple[bool, list[str]]:
    """Fail-closed evaluation of a complete smoke observation snapshot.

    Returns ``(ready, reasons)``.  Joint/pose modes require a successful,
    nonempty plan; blocked mode requires a deterministic non-success result;
    every mode additionally requires fresh ``pass`` readiness, exactly one
    ``/move_action`` MoveGroup server with observed type/kind/source metadata,
    and zero ``/isaac_joint_commands`` samples during the request/result
    window.
    """
    reasons: list[str] = []
    mode = str(expected.get("mode", ""))
    if mode not in MODES:
        return (False, ["unknown expected mode {!r}".format(mode)])

    # Readiness: fresh canonical pass.
    readiness = observations.get("readiness", {}) or {}
    state = readiness.get("state")
    if state != READINESS_STATE_PASS:
        reasons.append(
            "readiness state {!r} != {!r}".format(state, READINESS_STATE_PASS)
        )
    received_at = readiness.get("received_at_s")
    now_s = readiness.get("now_s")
    if isinstance(received_at, (int, float)) and isinstance(now_s, (int, float)):
        age = float(now_s) - float(received_at)
        max_age = float(expected.get("readiness_max_age_s", DEFAULT_READINESS_MAX_AGE_S))
        if age < 0.0 or age > max_age:
            reasons.append(
                "readiness is stale (age {:.3f} s > {:.3f} s)".format(age, max_age)
            )
    else:
        reasons.append("readiness timestamp missing")

    # Endpoint: exactly one MoveGroup action server with observed metadata.
    endpoint = observations.get("endpoint", {}) or {}
    count = endpoint.get("count")
    if count != 1:
        reasons.append(
            "endpoint {} server count {} != 1".format(MOVE_ACTION, count)
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
    kind = endpoint.get("kind")
    if kind != MOVE_ACTION_KIND:
        reasons.append(
            "endpoint kind {!r} != {!r}".format(kind, MOVE_ACTION_KIND)
        )
    source = endpoint.get("source")
    expected_source = str(expected.get("source", MOVE_ACTION_SOURCE))
    if source != expected_source:
        reasons.append(
            "endpoint source {!r} != {!r}".format(source, expected_source)
        )
    if endpoint.get("result_service_present") is not True:
        reasons.append("endpoint {} result service is missing".format(MOVE_ACTION))

    # Command observations: exactly zero samples across the request/result window.
    commands = observations.get("command_observations", {}) or {}
    samples = commands.get("samples")
    if not isinstance(samples, int) or isinstance(samples, bool) or samples < 0:
        reasons.append("command observation sample count is missing/invalid")
    elif samples != 0:
        reasons.append(
            "observed {} command sample(s) on {}; expected exactly 0".format(
                samples, COMMAND_TOPIC
            )
        )

    # Outcome.
    outcome = observations.get("outcome", {}) or {}
    outcome_kind = outcome.get("kind")
    error_code = outcome.get("error_code")
    if mode in ("joint", "pose"):
        if outcome_kind != "success":
            reasons.append(
                "expected a successful plan, got outcome {!r}".format(outcome_kind)
            )
        else:
            if error_code != int(expected.get("success_error_code", SUCCESS_ERROR_CODE)):
                reasons.append(
                    "success outcome error_code {} != {}".format(
                        error_code, expected.get("success_error_code", SUCCESS_ERROR_CODE)
                    )
                )
            point_count = outcome.get("trajectory_point_count")
            min_points = int(expected.get("trajectory_min_points", DEFAULT_TRAJECTORY_MIN_POINTS))
            if not isinstance(point_count, int) or isinstance(point_count, bool) or point_count < min_points:
                reasons.append(
                    "successful plan has empty trajectory (point_count={})".format(point_count)
                )
            if outcome.get("nonempty") is not True:
                reasons.append("successful plan is not nonempty")
    else:  # blocked
        if outcome_kind != "non_success":
            reasons.append(
                "blocked mode requires a deterministic non-success result, got outcome {!r}".format(
                    outcome_kind
                )
            )
        elif error_code is not None and error_code == int(
            expected.get("success_error_code", SUCCESS_ERROR_CODE)
        ):
            reasons.append(
                "blocked mode returned unexpected success (error_code {})".format(error_code)
            )

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


class OmplPlanSmokeClient:
    """Bounded live smoke client.  All ROS imports occur inside methods.

    This class is the Humble-only live seam: importing the module never
    executes these imports, so pure collection under simulator CPython 3.12
    stays ROS-free.
    """

    def __init__(self, node: object, args: argparse.Namespace) -> None:
        self._node = node
        self._args = args
        self._readiness: dict[str, object] = {"state": None, "received_at_s": None}
        self._command_samples = 0
        self._window_open = False
        self._command_window: dict[str, object] = {"start_s": None, "end_s": None}
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
            payload = json.loads(str(message.data))
        except (json.JSONDecodeError, AttributeError):
            return
        if isinstance(payload, dict):
            self._readiness["state"] = payload.get("state")
            self._readiness["received_at_s"] = time.monotonic()

    def _on_command(self, message: object) -> None:
        del message  # any published JointState is a command sample
        if self._window_open:
            self._command_samples += 1

    # -- spinning ----------------------------------------------------------

    def _spin_until(self, predicate: object, timeout_s: float) -> bool:
        import rclpy

        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def _wait_for_readiness(self, timeout_s: float) -> bool:
        import rclpy

        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if self._readiness.get("state") == READINESS_STATE_PASS:
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

    # -- goal conversion -----------------------------------------------------

    def _to_moveit_goal(self, goal: object) -> object:
        from geometry_msgs.msg import Pose, Quaternion, Vector3
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
    def _timeout_outcome(detail: str) -> dict[str, object]:
        """Return a deterministic bounded-timeout outcome observation."""
        return {
            "kind": "timeout",
            "error_code": None,
            "detail": detail,
            "trajectory_point_count": 0,
            "nonempty": False,
        }

    def _execute_goal(self, goal: object) -> dict[str, object]:
        import rclpy

        from moveit_msgs.action import MoveGroup
        from rclpy.action import ActionClient

        client = ActionClient(self._node, MoveGroup, MOVE_ACTION)
        deadline = time.monotonic() + float(self._args.timeout)
        try:
            if not client.wait_for_server(timeout_sec=min(self._args.timeout, 10.0)):
                return {
                    "kind": "invalid",
                    "error_code": None,
                    "detail": "action server unavailable on {}".format(MOVE_ACTION),
                    "trajectory_point_count": 0,
                    "nonempty": False,
                }
            goal_message = self._to_moveit_goal(goal)
            self._command_window["start_s"] = time.monotonic()
            self._window_open = True
            try:
                send_future = client.send_goal_async(goal_message)
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return self._timeout_outcome("goal not accepted")
                rclpy.spin_until_future_complete(
                    self._node, send_future, timeout_sec=remaining
                )
                if not send_future.done():
                    return self._timeout_outcome("goal not accepted")
                goal_handle = send_future.result()
                if goal_handle is None or not getattr(goal_handle, "accepted", False):
                    return {
                        "kind": "non_success",
                        "error_code": None,
                        "detail": "goal rejected by the action server",
                        "trajectory_point_count": 0,
                        "nonempty": False,
                    }
                result_future = goal_handle.get_result_async()
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return self._timeout_outcome("planning deadline")
                rclpy.spin_until_future_complete(
                    self._node, result_future, timeout_sec=remaining
                )
                if not result_future.done():
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(
                        self._node, cancel_future, timeout_sec=2.0
                    )
                    return self._timeout_outcome("planning deadline")
                response = result_future.result()
                if response is None:
                    return {
                        "kind": "invalid",
                        "error_code": None,
                        "detail": "empty action result",
                        "trajectory_point_count": 0,
                        "nonempty": False,
                    }
                move_result = response.result
                error_code = int(move_result.error_code.val)
                trajectory = move_result.planned_trajectory
                point_count = len(trajectory.joint_trajectory.points) + len(
                    trajectory.multi_dof_joint_trajectory.points
                )
                return {
                    "kind": "success" if error_code == SUCCESS_ERROR_CODE else "non_success",
                    "error_code": error_code,
                    "detail": "",
                    "trajectory_point_count": int(point_count),
                    "nonempty": point_count > 0,
                    "planning_time": float(move_result.planning_time),
                }
            finally:
                self._window_open = False
                self._command_window["end_s"] = time.monotonic()
        finally:
            try:
                client.destroy()
            except Exception:  # noqa: BLE001 - destroy must never mask the result
                pass

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
        except ValueError as exc:
            return fail_closed_report(
                mode=args.mode,
                scenario="",
                blocker="scenario/goal validation failed: {}".format(exc),
                meta=meta_info,
            )

        if not self._wait_for_readiness(args.readiness_timeout):
            return fail_closed_report(
                mode=args.mode,
                scenario=scenario_id,
                blocker="integrated readiness did not reach a fresh pass on {} within {:.1f} s".format(
                    READINESS_TOPIC, args.readiness_timeout
                ),
                meta=meta_info,
            )

        endpoint_obs = self._probe_move_action()
        if endpoint_obs.get("count") != 1 or endpoint_obs.get("result_service_present") is not True:
            outcome_obs: dict[str, object] = {
                "kind": "invalid",
                "error_code": None,
                "detail": "endpoint metadata does not match: {}".format(
                    json.dumps(endpoint_obs, sort_keys=True, separators=(",", ":"))
                ),
                "trajectory_point_count": 0,
                "nonempty": False,
            }
            self._command_window["start_s"] = time.monotonic()
            self._command_window["end_s"] = time.monotonic()
        else:
            outcome_obs = self._execute_goal(goal)

        now_s = time.monotonic()
        readiness_obs: dict[str, object] = {
            "state": self._readiness.get("state"),
            "received_at_s": self._readiness.get("received_at_s"),
            "now_s": now_s,
        }
        command_obs: dict[str, object] = {
            "topic": COMMAND_TOPIC,
            "samples": self._command_samples,
            "window_start_s": self._command_window.get("start_s"),
            "window_end_s": self._command_window.get("end_s"),
        }
        observations: dict[str, object] = {
            "readiness": readiness_obs,
            "endpoint": endpoint_obs,
            "command_observations": command_obs,
            "outcome": outcome_obs,
        }
        expected: dict[str, object] = {
            "mode": args.mode,
            "action_type": MOVE_ACTION_TYPE,
            "source": MOVE_ACTION_SOURCE,
            "readiness_max_age_s": args.readiness_max_age,
            "success_error_code": SUCCESS_ERROR_CODE,
            "trajectory_min_points": DEFAULT_TRAJECTORY_MIN_POINTS,
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

    def _environment_meta(self) -> dict[str, object]:
        import rclpy

        return {
            "python": "{}.{}.{}".format(*sys.version_info[:3]),
            "rclpy": str(getattr(rclpy, "__version__", "")),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
            "domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "dds_profile": os.environ.get("TINKER_SIM_DDS_PROFILE", ""),
            "simulator_root": str(REPO_ROOT),
        }


def _endpoint_label(node_name: str, node_namespace: str) -> str:
    namespace = str(node_namespace or "").rstrip("/")
    if namespace:
        return "{}/{}".format(namespace, str(node_name))
    return "/" + str(node_name)


def main(argv: Sequence[str] | None = None) -> int:
    """Live Humble entry point.  All ROS imports happen inside this function."""
    # Parse first so ``--help`` exits without importing rclpy (argparse raises
    # SystemExit before the Humble-only seam is reached).
    arguments = parse_args(argv)
    import rclpy

    rclpy.init(args=argv)
    node = rclpy.create_node("ompl_plan_smoke")
    try:
        client = OmplPlanSmokeClient(node, arguments)
        report = client.run()
    except KeyboardInterrupt:
        report = fail_closed_report(
            mode=arguments.mode,
            scenario="",
            blocker="interrupted",
            meta={"python": "{}.{}.{}".format(*sys.version_info[:3])},
        )
    except Exception as exc:  # noqa: BLE001 - every failure writes a report
        report = fail_closed_report(
            mode=arguments.mode,
            scenario="",
            blocker="live attempt failed: {}".format(exc),
            meta={"python": "{}.{}.{}".format(*sys.version_info[:3])},
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    data = serialize_report(report)
    write_report_atomic(arguments.report, data)
    print(json.dumps(report, sort_keys=True, separators=(",", ":")))
    return 0 if report["evaluation"]["ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
