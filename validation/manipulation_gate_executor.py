#!/usr/bin/env python3
"""Execute the six ROS manipulation qualification gates.

The fixture builders and event journal deliberately have no ROS imports.  ROS
packages are loaded only when :func:`main` constructs a live executor, which
keeps this module usable for offline validation and unit tests.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

SCHEMA_VERSION = 1
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))
TRAJECTORY_ACTION = "/xarm7_traj_controller/follow_joint_trajectory"
GRIPPER_ACTION = "/xarm_gripper/gripper_action"
SAFETY_TOPIC = "/sim/safety/operator"
EFFECTIVE_SAFETY_TOPIC = "/sim/hardware/safety_stop"
CLOCK_TOPIC = "/clock"
JOINT_STATE_TOPIC = "/isaac_joint_states"
GATES = (
    "free-space-fjt",
    "safety-stop",
    "free-gripper",
    "obstructed-gripper",
    "arm-collision",
    "retention",
)
VISUAL_CHECKPOINT_EVENTS = {
    "free-space-fjt": ("start", "outbound-apex", "return-arrival", "terminal"),
    "safety-stop": ("moving", "effective-stop", "velocity-compliant", "post-clear"),
    "free-gripper": ("open-start", "closed", "reopening", "open-terminal"),
    "obstructed-gripper": ("pre-close", "bilateral-contact", "stalled-result", "terminal"),
    "arm-collision": ("approach", "first-contact", "velocity-compliant", "terminal"),
    "retention": ("bilateral-grasp", "lift-threshold", "translation-threshold", "stable-hold"),
}
Q_GRASP = (
    0.000002,
    0.836049,
    0.000005,
    2.040282,
    0.000003,
    1.189480,
    0.000006,
)
Q_LIFT = (
    0.000004,
    1.262552,
    0.000013,
    2.714944,
    -0.000002,
    0.616009,
    0.000006,
)
Q_OUTBOUND = (0.20, -0.20, 0.15, 0.30, -0.15, 0.20, 0.15)
FREE_SPACE_OUTBOUND_S = 4.0
FREE_SPACE_RETURN_S = 8.0
GRASP_MOTION_S = 8.0
LIFT_MOTION_S = 8.0
FJT_GOAL_TIME_TOLERANCE_S = 1.0
ACTION_RESULT_TIMEOUT_S = 90.0
SAFETY_CANCEL_RESPONSE_TIMEOUT_S = 10.0
SAFETY_ACTION_RESULT_TIMEOUT_S = 2.0
SAFETY_CLEAR_TRUTH_TIMEOUT_S = 10.0
SAFETY_HOLD_AFTER_COMPLIANCE_S = 0.65
SAFETY_POST_CLEAR_SETTLE_S = 1.1
PHYSICS_TRUTH_TAIL_BYTES = 256 * 1024


def _finite_vector(values: Sequence[float], *, name: str, size: int = 7) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


@dataclass(frozen=True)
class TrajectoryPointSpec:
    time_from_start_s: float
    positions: tuple[float, ...]
    velocities: tuple[float, ...]

    def __post_init__(self) -> None:
        if self.time_from_start_s < 0 or not math.isfinite(self.time_from_start_s):
            raise ValueError("trajectory point time must be finite and non-negative")
        object.__setattr__(self, "positions", _finite_vector(self.positions, name="positions"))
        object.__setattr__(self, "velocities", _finite_vector(self.velocities, name="velocities"))

    def as_dict(self) -> dict[str, Any]:
        return {
            "time_from_start_s": self.time_from_start_s,
            "positions": list(self.positions),
            "velocities": list(self.velocities),
        }


@dataclass(frozen=True)
class TrajectorySpec:
    joint_names: tuple[str, ...]
    points: tuple[TrajectoryPointSpec, ...]
    start_offset_s: float = 0.5

    def __post_init__(self) -> None:
        if self.joint_names != JOINT_NAMES:
            raise ValueError("manipulation goals must contain joint1..joint7 in order")
        if not self.points or any(
            right.time_from_start_s <= left.time_from_start_s
            for left, right in zip(self.points, self.points[1:])
        ):
            raise ValueError("trajectory points must be non-empty and strictly ordered")
        if self.start_offset_s != 0.5:
            raise ValueError("manipulation goals require a +0.5 simulated-second start offset")

    def as_dict(self) -> dict[str, Any]:
        return {
            "joint_names": list(self.joint_names),
            "points": [point.as_dict() for point in self.points],
            "start_offset_s": self.start_offset_s,
        }


@dataclass(frozen=True)
class ActionSpec:
    endpoint: str
    goal: Mapping[str, Any]
    wait_after_s: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        return {"endpoint": self.endpoint, "goal": dict(self.goal), "wait_after_s": self.wait_after_s}


def trajectory_spec(start: Sequence[float], points: Iterable[tuple[float, Sequence[float]]]) -> TrajectorySpec:
    del start  # The first point is explicit in every fixture and is retained in points.
    specs = tuple(
        TrajectoryPointSpec(time_from_start_s=time, positions=_finite_vector(position, name="positions"), velocities=(0.0,) * 7)
        for time, position in points
    )
    return TrajectorySpec(joint_names=JOINT_NAMES, points=specs)


def _motion(start: Sequence[float], target: Sequence[float], duration_s: float) -> TrajectorySpec:
    start = _finite_vector(start, name="start")
    target = _finite_vector(target, name="target")
    return trajectory_spec(start, ((0.0, start), (duration_s, target)))


def build_free_space_fixture(q0: Sequence[float]) -> TrajectorySpec:
    q0 = _finite_vector(q0, name="q0")
    return trajectory_spec(
        q0,
        (
            (0.0, q0),
            (FREE_SPACE_OUTBOUND_S, Q_OUTBOUND),
            (FREE_SPACE_RETURN_S, q0),
        ),
    )


def build_safety_fixture(q0: Sequence[float]) -> TrajectorySpec:
    q0 = _finite_vector(q0, name="q0")
    return trajectory_spec(q0, ((0.0, q0), (4.0, Q_OUTBOUND), (8.0, q0)))


def build_grasp_fixture(q0: Sequence[float]) -> TrajectorySpec:
    return _motion(q0, Q_GRASP, GRASP_MOTION_S)


def build_lift_fixture() -> TrajectorySpec:
    return _motion(Q_GRASP, Q_LIFT, LIFT_MOTION_S)


def gripper_goal(position: float) -> dict[str, float]:
    if not math.isfinite(position) or not 0.0 <= position <= 0.85:
        raise ValueError("gripper position must be within [0, 0.85]")
    return {"position": float(position), "max_effort": 20.0}


def build_gate_actions(gate: str, q0: Sequence[float]) -> tuple[ActionSpec, ...]:
    """Return the pure-Python action sequence for a gate."""
    q0 = _finite_vector(q0, name="q0")
    if gate == "free-space-fjt":
        return (ActionSpec(TRAJECTORY_ACTION, build_free_space_fixture(q0).as_dict()),)
    if gate == "safety-stop":
        return (ActionSpec(TRAJECTORY_ACTION, build_safety_fixture(q0).as_dict()),)
    if gate == "free-gripper":
        return (
            ActionSpec(GRIPPER_ACTION, gripper_goal(0.83)),
            ActionSpec(GRIPPER_ACTION, gripper_goal(0.0)),
        )
    if gate == "obstructed-gripper":
        return (
            ActionSpec(TRAJECTORY_ACTION, build_grasp_fixture(q0).as_dict()),
            ActionSpec(GRIPPER_ACTION, gripper_goal(0.83)),
        )
    if gate == "retention":
        return (
            ActionSpec(TRAJECTORY_ACTION, build_grasp_fixture(q0).as_dict()),
            ActionSpec(GRIPPER_ACTION, gripper_goal(0.83)),
            ActionSpec(TRAJECTORY_ACTION, build_lift_fixture().as_dict(), wait_after_s=1.0),
        )
    if gate == "arm-collision":
        return (ActionSpec(TRAJECTORY_ACTION, build_grasp_fixture(q0).as_dict()),)
    raise ValueError(f"unknown manipulation gate: {gate}")


@dataclass(frozen=True)
class EventRecord:
    sequence: int
    event: str
    gate: str
    simulated_timestamp: float | None = None
    wall_timestamp: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    action_endpoint: str | None = None
    phase: str | None = None
    accepted: bool | None = None
    feedback: Any = None
    result: Any = None
    canceled: bool | None = None
    aborted: bool | None = None
    goals: Any = None
    error: str | None = None
    schema_version: int = SCHEMA_VERSION

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class EventJournal:
    """Append JSONL events and atomically replace the final summary."""

    def __init__(self, attempt_dir: Path, gate: str) -> None:
        self.attempt_dir = Path(attempt_dir)
        self.attempt_dir.mkdir(parents=True, exist_ok=True)
        self.gate = gate
        self.path = self.attempt_dir / "gate-execution.jsonl"
        self.visual_path = self.attempt_dir / "visual-capture-requests.jsonl"
        self.summary_path = self.attempt_dir / "gate-execution.json"
        self.sequence = 0
        self.visual_sequence = 0
        self.records: list[dict[str, Any]] = []
        self.visual_requests: list[dict[str, Any]] = []

    def record(self, event: str, *, simulated_timestamp: float | None = None, action_endpoint: str | None = None, phase: str | None = None, accepted: bool | None = None, feedback: Any = None, result: Any = None, canceled: bool | None = None, aborted: bool | None = None, goals: Any = None, error: str | None = None) -> dict[str, Any]:
        self.sequence += 1
        item = EventRecord(
            sequence=self.sequence,
            event=event,
            gate=self.gate,
            simulated_timestamp=simulated_timestamp,
            action_endpoint=action_endpoint,
            phase=phase,
            accepted=accepted,
            feedback=feedback,
            result=result,
            canceled=canceled,
            aborted=aborted,
            goals=goals,
            error=error,
        ).as_dict()
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(item, sort_keys=True, separators=(",", ":")) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        self.records.append(item)
        if event in VISUAL_CHECKPOINT_EVENTS.get(self.gate, ()):
            self.visual_sequence += 1
            request = {
                "schema_version": SCHEMA_VERSION,
                "sequence": self.visual_sequence,
                "gate": self.gate,
                "event": event,
                "simulated_timestamp": simulated_timestamp,
                "source_execution_event_sequence": item["sequence"],
            }
            with self.visual_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(request, sort_keys=True, separators=(",", ":")) + "\n")
                stream.flush()
                os.fsync(stream.fileno())
            self.visual_requests.append(request)
        return item

    def finalize(self, *, success: bool, error: str | None = None) -> dict[str, Any]:
        summary = {
            "schema_version": SCHEMA_VERSION,
            "gate": self.gate,
            "success": bool(success),
            "error": error,
            "event_count": len(self.records),
            "events": self.records,
            "requested_visual_events": [request["event"] for request in self.visual_requests],
            "visual_capture_requests": self.visual_requests,
        }
        fd, temporary = tempfile.mkstemp(prefix="gate-execution.", suffix=".json.tmp", dir=self.attempt_dir)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(summary, stream, sort_keys=True, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.summary_path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return summary


def _clock_seconds(message: Any) -> float:
    clock = message.clock
    return float(clock.sec) + float(clock.nanosec) * 1e-9


def _duration_message(seconds: float, Duration: Any) -> Any:
    return Duration(seconds=float(seconds)).to_msg()


def _goal_spec_to_message(spec: TrajectorySpec, FollowJointTrajectory: Any, JointTrajectoryPoint: Any, stamp: Any) -> Any:
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(spec.joint_names)
    goal.trajectory.header.stamp = stamp
    goal.goal_time_tolerance = _duration_message(FJT_GOAL_TIME_TOLERANCE_S, _ROS_IMPORTS["Duration"])
    for item in spec.points:
        point = JointTrajectoryPoint()
        point.positions = list(item.positions)
        point.velocities = list(item.velocities)
        point.time_from_start = _duration_message(item.time_from_start_s, _ROS_IMPORTS["Duration"])
        goal.trajectory.points.append(point)
    return goal


_ROS_IMPORTS: dict[str, Any] = {}


def _load_ros() -> dict[str, Any]:
    """Import ROS only at live execution time."""
    if _ROS_IMPORTS:
        return _ROS_IMPORTS
    import rclpy
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from trajectory_msgs.msg import JointTrajectoryPoint
    from rosgraph_msgs.msg import Clock

    _ROS_IMPORTS.update(locals())
    return _ROS_IMPORTS


class RosGateExecutor:
    """Small ROS boundary for the pure gate plans."""

    def __init__(self, gate: str, attempt_dir: Path, config: Mapping[str, Any]) -> None:
        ros = _load_ros()
        self.ros = ros
        self.gate = gate
        self.config = config
        self.journal = EventJournal(attempt_dir, gate)
        self.physics_truth_path = Path(attempt_dir) / "physics_truth.jsonl"
        self.node = ros["Node"](f"tinker_manipulation_gate_{gate.replace('-', '_')}")
        self.clock = None
        self.effective_safety: bool | None = None
        self.joint_state: dict[str, float] = {}
        self.joint_velocity: dict[str, float] = {}
        self.collision = False
        self._trajectory_phase: str | None = None
        self._gripper_phase: str | None = None
        self.node.create_subscription(ros["Clock"], CLOCK_TOPIC, self._on_clock, 10)
        self.node.create_subscription(ros["JointState"], JOINT_STATE_TOPIC, self._on_joint_state, 20)
        self.node.create_subscription(ros["Bool"], "/sim/safety/collision", self._on_collision, 10)
        safety_qos = ros["QoSProfile"](
            depth=1,
            reliability=ros["ReliabilityPolicy"].RELIABLE,
            durability=ros["DurabilityPolicy"].TRANSIENT_LOCAL,
        )
        self.node.create_subscription(ros["Bool"], EFFECTIVE_SAFETY_TOPIC, self._on_effective_safety, safety_qos)
        self.safety_publisher = self.node.create_publisher(ros["Bool"], SAFETY_TOPIC, safety_qos)
        self.trajectory_client = ros["ActionClient"](self.node, ros["FollowJointTrajectory"], TRAJECTORY_ACTION)
        self.gripper_client = ros["ActionClient"](self.node, ros["GripperCommand"], GRIPPER_ACTION)

    def _on_clock(self, message: Any) -> None:
        self.clock = _clock_seconds(message)

    def _on_joint_state(self, message: Any) -> None:
        self.joint_state.update({name: float(position) for name, position in zip(message.name, message.position)})
        self.joint_velocity.update({name: float(velocity) for name, velocity in zip(message.name, message.velocity)})

    def _on_collision(self, message: Any) -> None:
        self.collision = bool(message.data)

    def _on_effective_safety(self, message: Any) -> None:
        self.effective_safety = bool(message.data)

    def _spin_until(self, predicate: Any, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            self.ros["rclpy"].spin_once(self.node, timeout_sec=0.05)
        return bool(predicate())

    def wait_clock(self, target: float, timeout_s: float = 30.0) -> None:
        if not self._spin_until(lambda: self.clock is not None and self.clock >= target, timeout_s):
            raise RuntimeError(f"timed out waiting for /clock >= {target}")

    def q0(self) -> tuple[float, ...]:
        if not self._spin_until(lambda: all(name in self.joint_state for name in JOINT_NAMES), 30.0):
            raise RuntimeError("timed out waiting for all seven joint states")
        return tuple(self.joint_state[name] for name in JOINT_NAMES)

    def _stamp_after(self, offset_s: float = 0.5) -> Any:
        if self.clock is None:
            raise RuntimeError("simulated clock is not ready")
        total_ns = int(round((self.clock + offset_s) * 1e9))
        stamp = self.ros["Time"](nanoseconds=total_ns).to_msg()
        return stamp

    def _send_trajectory(self, spec: TrajectorySpec, *, phase: str) -> tuple[Any, Any]:
        if not self.trajectory_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError(f"action server unavailable: {TRAJECTORY_ACTION}")
        goal = _goal_spec_to_message(
            spec,
            self.ros["FollowJointTrajectory"],
            self.ros["JointTrajectoryPoint"],
            self._stamp_after(spec.start_offset_s),
        )
        self._trajectory_phase = phase
        self.journal.record("action_goal_sent", simulated_timestamp=self.clock, action_endpoint=TRAJECTORY_ACTION, phase=phase, goals=spec.as_dict())
        goal_future = self.trajectory_client.send_goal_async(goal, feedback_callback=self._trajectory_feedback)
        self.ros["rclpy"].spin_until_future_complete(self.node, goal_future, timeout_sec=30.0)
        handle = goal_future.result()
        accepted = bool(handle and handle.accepted)
        self.journal.record("action_goal_response", simulated_timestamp=self.clock, action_endpoint=TRAJECTORY_ACTION, phase=phase, accepted=accepted, goals=spec.as_dict(), error=None if accepted else "goal rejected")
        if not accepted:
            raise RuntimeError("trajectory goal rejected")
        return handle, handle.get_result_async()

    def _trajectory_feedback(self, message: Any) -> None:
        feedback = getattr(message, "feedback", message)
        self.journal.record("action_feedback", simulated_timestamp=self.clock, action_endpoint=TRAJECTORY_ACTION, phase=self._trajectory_phase, feedback=str(feedback))

    def _wait_result(
        self,
        result_future: Any,
        endpoint: str,
        *,
        phase: str | None = None,
        goals: Mapping[str, Any] | None = None,
        timeout_s: float = ACTION_RESULT_TIMEOUT_S,
        allow_unresolved: bool = False,
        canceled: bool | None = None,
    ) -> Any:
        self.ros["rclpy"].spin_until_future_complete(self.node, result_future, timeout_sec=timeout_s)
        if not result_future.done():
            if allow_unresolved:
                error = f"action result future unresolved after cancellation: {endpoint}"
                self.journal.record(
                    "action_result",
                    simulated_timestamp=self.clock,
                    action_endpoint=endpoint,
                    phase=phase,
                    result={
                        "resolved": False,
                        "status": None,
                        "message": error,
                    },
                    canceled=canceled,
                    goals=dict(goals or {}),
                    error=error,
                )
                return None
            raise RuntimeError(f"timed out waiting for action result: {endpoint}")
        wrapped = result_future.result()
        status = getattr(wrapped, "status", None)
        result = getattr(wrapped, "result", wrapped)
        result_record = {
            "status": status,
            "success": status == 4,
            "message": str(result),
        }
        for name in ("reached_goal", "stalled", "error_code", "error_string"):
            if hasattr(result, name):
                result_record[name] = getattr(result, name)
        self.journal.record(
            "action_result",
            simulated_timestamp=self.clock,
            action_endpoint=endpoint,
            phase=phase,
            result=result_record,
            canceled=status == 5,
            aborted=status == 6,
            goals=dict(goals or {}),
        )
        return wrapped

    @staticmethod
    def _goal_id_token(goal_id: Any) -> tuple[int, ...] | None:
        raw = getattr(goal_id, "uuid", goal_id)
        if raw is None or isinstance(raw, str):
            return None
        try:
            values = tuple(int(value) for value in raw)
        except (TypeError, ValueError):
            return None
        return values or None

    def _cancel_trajectory_goal(self, handle: Any) -> bool | None:
        """Request cancellation and journal the server's actual response."""
        cancel_goal = getattr(handle, "cancel_goal_async", None)
        if cancel_goal is None:
            error = "trajectory goal handle has no cancel_goal_async"
            self.journal.record(
                "action_cancel_requested",
                simulated_timestamp=self.clock,
                action_endpoint=TRAJECTORY_ACTION,
                canceled=None,
                result={"resolved": False, "accepted": False},
                error=error,
            )
            return None
        try:
            cancel_future = cancel_goal()
        except Exception as error:
            message = f"cancel request failed: {type(error).__name__}: {error}"
            self.journal.record(
                "action_cancel_requested",
                simulated_timestamp=self.clock,
                action_endpoint=TRAJECTORY_ACTION,
                canceled=None,
                result={"resolved": False, "accepted": False},
                error=message,
            )
            return None

        self.ros["rclpy"].spin_until_future_complete(
            self.node,
            cancel_future,
            timeout_sec=SAFETY_CANCEL_RESPONSE_TIMEOUT_S,
        )
        if not cancel_future.done():
            error = "cancel response future unresolved"
            self.journal.record(
                "action_cancel_requested",
                simulated_timestamp=self.clock,
                action_endpoint=TRAJECTORY_ACTION,
                canceled=None,
                result={
                    "resolved": False,
                    "accepted": False,
                    "timeout_s": SAFETY_CANCEL_RESPONSE_TIMEOUT_S,
                },
                error=error,
            )
            return None

        try:
            response = cancel_future.result()
        except Exception as error:
            message = f"cancel response failed: {type(error).__name__}: {error}"
            self.journal.record(
                "action_cancel_requested",
                simulated_timestamp=self.clock,
                action_endpoint=TRAJECTORY_ACTION,
                canceled=None,
                result={"resolved": False, "accepted": False},
                error=message,
            )
            return None

        goal_ids: list[list[int]] = []
        for goal_info in getattr(response, "goals_canceling", ()) or ():
            token = self._goal_id_token(getattr(goal_info, "goal_id", goal_info))
            if token is not None:
                goal_ids.append(list(token))
        requested_id = self._goal_id_token(getattr(handle, "goal_id", None))
        goal_id_match = bool(goal_ids) and (
            requested_id is None or list(requested_id) in goal_ids
        )
        raw_return_code = getattr(response, "return_code", None)
        return_code_valid = raw_return_code is not None
        try:
            return_code = int(raw_return_code) if raw_return_code is not None else None
        except (TypeError, ValueError):
            return_code = None
            return_code_valid = False
        accepted = bool(goal_id_match and return_code_valid and return_code == 0)
        details = {
            "resolved": True,
            "accepted": accepted,
            "return_code": return_code,
            "return_code_valid": return_code_valid,
            "goals_canceling_count": len(goal_ids),
            "goal_ids": goal_ids,
            "goal_id_match": goal_id_match,
        }
        error = None
        if not accepted:
            error = "cancel response did not confirm the requested goal"
            if not return_code_valid:
                error = "cancel response did not contain a valid return_code"
            elif return_code != 0:
                error = f"cancel response rejected the goal (return_code={return_code})"
        self.journal.record(
            "action_cancel_requested",
            simulated_timestamp=self.clock,
            action_endpoint=TRAJECTORY_ACTION,
            accepted=accepted,
            canceled=accepted,
            result=details,
            error=error,
        )
        return accepted

    def _send_gripper(self, position: float) -> Any:
        if not self.gripper_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError(f"action server unavailable: {GRIPPER_ACTION}")
        goal = self.ros["GripperCommand"].Goal()
        goal.command.position = position
        goal.command.max_effort = 20.0
        goal_spec = gripper_goal(position)
        phase = "close" if position > 0.4 else "open"
        self._gripper_phase = phase
        self.journal.record("action_goal_sent", simulated_timestamp=self.clock, action_endpoint=GRIPPER_ACTION, phase=phase, goals=goal_spec)
        future = self.gripper_client.send_goal_async(goal, feedback_callback=self._gripper_feedback)
        self.ros["rclpy"].spin_until_future_complete(self.node, future, timeout_sec=30.0)
        handle = future.result()
        accepted = bool(handle and handle.accepted)
        self.journal.record("action_goal_response", simulated_timestamp=self.clock, action_endpoint=GRIPPER_ACTION, phase=phase, accepted=accepted, goals=goal_spec, error=None if accepted else "goal rejected")
        if not accepted:
            raise RuntimeError("gripper goal rejected")
        try:
            return self._wait_result(
                handle.get_result_async(),
                GRIPPER_ACTION,
                phase=phase,
                goals=goal_spec,
            )
        finally:
            self._gripper_phase = None

    def _gripper_feedback(self, message: Any) -> None:
        feedback = getattr(message, "feedback", message)
        structured = {
            name: getattr(feedback, name)
            for name in ("position", "effort", "reached_goal", "stalled")
            if hasattr(feedback, name)
        }
        self.journal.record(
            "action_feedback",
            simulated_timestamp=self.clock,
            action_endpoint=GRIPPER_ACTION,
            phase=self._gripper_phase,
            feedback=structured or str(feedback),
        )

    def _publish_safety(self, active: bool) -> None:
        message = self.ros["Bool"]()
        message.data = bool(active)
        self.safety_publisher.publish(message)
        self.journal.record("safety_asserted" if active else "safety_cleared", simulated_timestamp=self.clock, action_endpoint=SAFETY_TOPIC, result={"active": active})

    def _arm_velocity_compliant(self) -> bool:
        threshold = float(self.config.get("thresholds", {}).get("safety_stop_velocity_rad_s", 0.02))
        return all(name in self.joint_velocity for name in JOINT_NAMES) and max(
            abs(self.joint_velocity[name]) for name in JOINT_NAMES
        ) <= threshold

    def _latest_physics_truth(self) -> dict[str, Any]:
        """Read the evaluator-owned latest complete truth record.

        This deliberately reads the qualification evidence file instead of
        subscribing to the raw-truth ROS topic.  A missing, malformed, or
        temporally stale record is an execution failure, not a false predicate.
        """
        try:
            with self.physics_truth_path.open("rb") as stream:
                stream.seek(0, os.SEEK_END)
                end = stream.tell()
                if end <= 0:
                    raise RuntimeError(f"physics truth evidence is empty: {self.physics_truth_path}")
                stream.seek(end - 1)
                if stream.read(1) != b"\n":
                    raise RuntimeError("physics truth evidence ends with an incomplete record")
                start = max(0, end - PHYSICS_TRUTH_TAIL_BYTES)
                stream.seek(start)
                tail = stream.read(end - start)
        except (OSError, UnicodeError) as exc:
            raise RuntimeError(f"physics truth evidence unavailable: {self.physics_truth_path}") from exc
        nonblank = [line for line in tail.split(b"\n") if line.strip()]
        if not nonblank:
            raise RuntimeError(f"physics truth evidence is empty: {self.physics_truth_path}")
        try:
            truth = json.loads(nonblank[-1].decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("latest physics truth record is malformed") from exc
        if not isinstance(truth, dict):
            raise RuntimeError("latest physics truth record is not an object")
        timestamp = truth.get("timestamp")
        frame_index = truth.get("frame_index")
        if isinstance(timestamp, bool) or not isinstance(timestamp, (int, float)) or not math.isfinite(float(timestamp)):
            raise RuntimeError("latest physics truth record has no finite timestamp")
        if isinstance(frame_index, bool) or not isinstance(frame_index, int) or frame_index < 0:
            raise RuntimeError("latest physics truth record has no valid frame_index")
        if self.clock is None:
            raise RuntimeError("simulation clock is unavailable for physics truth freshness")
        physics_hz = float(self.config.get("physics", {}).get("hz", 120.0))
        if not math.isfinite(physics_hz) or physics_hz <= 0.0:
            raise RuntimeError("physics frequency is invalid for physics truth freshness")
        max_age = max(4.0 / physics_hz, 0.05)
        age = float(self.clock) - float(timestamp)
        if age < -max_age or age > max_age:
            raise RuntimeError(f"physics truth record is stale by {age:.6f}s")
        return truth

    def _truth_contact_sides(self) -> tuple[bool, bool]:
        truth = self._latest_physics_truth()
        contacts = truth.get("contacts", truth.get("contact_pairs", []))
        left = right = False
        for contact in contacts if isinstance(contacts, list) else ():
            if not isinstance(contact, Mapping):
                continue
            bodies = {str(contact.get(key, "")).lower().replace("/", "_") for key in ("body_a", "body_b")}
            if not any("qualification_cube" in body for body in bodies):
                continue
            left = left or any("left_finger" in body for body in bodies)
            right = right or any("right_finger" in body for body in bodies)
        return left, right

    def _truth_cube_pose(self) -> tuple[float, float, float] | None:
        truth = self._latest_physics_truth()
        objects = truth.get("objects")
        if objects is None and truth.get("object") is not None:
            objects = [truth["object"]]
        for item in objects if isinstance(objects, list) else ():
            if not isinstance(item, Mapping) or str(item.get("id", item.get("object_id", ""))) != "qualification_cube":
                continue
            pose = item.get("pose")
            xyz = pose.get("xyz") if isinstance(pose, Mapping) else None
            if isinstance(xyz, Sequence) and len(xyz) >= 3:
                return tuple(float(value) for value in xyz[:3])
        return None

    def _truth_safety_stop(self) -> bool:
        """Return the raw safety state from the evaluator-owned truth record."""
        truth = self._latest_physics_truth()
        value = truth.get("safety_stop")
        if value is None:
            robot = truth.get("robot")
            value = robot.get("safety_stop") if isinstance(robot, Mapping) else None
        if not isinstance(value, bool):
            raise RuntimeError("latest physics truth record has no boolean safety_stop")
        return value

    def _wait_for_truth_safety_stop(self, expected: bool, timeout_s: float) -> None:
        """Poll evaluator-owned truth, keeping transient evidence failures fail-closed."""
        def predicate() -> bool:
            try:
                return self._truth_safety_stop() is expected
            except RuntimeError:
                return False

        if not self._spin_until(predicate, timeout_s):
            raise RuntimeError(
                f"raw physics truth safety_stop did not become {expected} before timeout"
            )

    def _wait_simulated_duration(self, duration_s: float, *, timeout_s: float = 30.0) -> None:
        """Wait on simulation time with a bounded wall-clock escape hatch."""
        if not math.isfinite(duration_s) or duration_s < 0.0:
            raise ValueError("simulated wait duration must be finite and non-negative")
        if self.clock is None:
            raise RuntimeError("simulated clock is unavailable for timed wait")
        self.wait_clock(self.clock + duration_s, timeout_s=timeout_s)

    def _checkpoint(self, event: str, *, predicate: Any | None = None, timeout_s: float = 30.0) -> None:
        if predicate is not None and not self._spin_until(predicate, timeout_s):
            raise RuntimeError(f"physical predicate was not satisfied before checkpoint {event}")
        self.journal.record(event, simulated_timestamp=self.clock)
        self.wait_clock((self.clock or 0.0) + float(self.config.get("physics", {}).get("hz", 120.0)) ** -1)

    def _run_trajectory(self, spec: TrajectorySpec, *, phase: str, safety_at_s: float | None = None, checkpoint_events: Sequence[tuple[str, float]] = ()) -> None:
        trajectory_start = self.clock or 0.0
        if checkpoint_events:
            self._checkpoint(checkpoint_events[0][0])
        handle, result_future = self._send_trajectory(spec, phase=phase)
        cancel_accepted: bool | None = None
        for event, offset_s in checkpoint_events[1:]:
            self.wait_clock(trajectory_start + spec.start_offset_s + offset_s)
            self._checkpoint(event)
        if safety_at_s is not None:
            self.wait_clock(trajectory_start + spec.start_offset_s + safety_at_s)
            self._publish_safety(True)
            if not self._spin_until(lambda: self.effective_safety is True, 10.0):
                raise RuntimeError("operator stop did not reach /sim/hardware/safety_stop")
            self._checkpoint("effective-stop")
            cancel_accepted = self._cancel_trajectory_goal(handle)
            if not self._spin_until(self._arm_velocity_compliant, 30.0):
                raise RuntimeError("arm velocity did not become compliant after safety stop")
            self._checkpoint("velocity-compliant")
            self._wait_simulated_duration(
                SAFETY_HOLD_AFTER_COMPLIANCE_S,
                timeout_s=SAFETY_CLEAR_TRUTH_TIMEOUT_S,
            )
            self._publish_safety(False)
            if not self._spin_until(lambda: self.effective_safety is False, 10.0):
                raise RuntimeError("operator stop did not clear /sim/hardware/safety_stop")
            self._wait_for_truth_safety_stop(False, SAFETY_CLEAR_TRUTH_TIMEOUT_S)
            self._checkpoint("post-clear")
            self._wait_simulated_duration(
                SAFETY_POST_CLEAR_SETTLE_S,
                timeout_s=SAFETY_CLEAR_TRUTH_TIMEOUT_S,
            )
        if phase == "collision":
            self._checkpoint("first-contact", predicate=lambda: self.collision)
            self._checkpoint("velocity-compliant", predicate=self._arm_velocity_compliant)
        self._wait_result(
            result_future,
            TRAJECTORY_ACTION,
            phase=phase,
            goals={
                "expected_final_positions": list(spec.points[-1].positions),
                "trajectory": spec.as_dict(),
            },
            timeout_s=(
                SAFETY_ACTION_RESULT_TIMEOUT_S
                if safety_at_s is not None
                else ACTION_RESULT_TIMEOUT_S
            ),
            allow_unresolved=safety_at_s is not None,
            canceled=cancel_accepted,
        )

    def run(self) -> dict[str, Any]:
        q0 = self.q0()
        self.journal.record("gate_started", simulated_timestamp=self.clock, goals={"q0": list(q0), "actions": [item.as_dict() for item in build_gate_actions(self.gate, q0)]})
        if self.gate == "free-space-fjt":
            self._run_trajectory(
                build_free_space_fixture(q0),
                phase="free-space",
                checkpoint_events=(
                    ("start", 0.0),
                    ("outbound-apex", FREE_SPACE_OUTBOUND_S),
                    ("return-arrival", FREE_SPACE_RETURN_S),
                ),
            )
            self._checkpoint("terminal")
        elif self.gate == "safety-stop":
            self._run_trajectory(build_safety_fixture(q0), phase="safety", safety_at_s=1.5, checkpoint_events=(("moving", 0.0),))
        elif self.gate == "free-gripper":
            self._checkpoint("open-start")
            self._send_gripper(0.83)
            self._checkpoint("closed")
            self._checkpoint("reopening")
            self._send_gripper(0.0)
            self._checkpoint("open-terminal")
        elif self.gate == "obstructed-gripper":
            self._run_trajectory(build_grasp_fixture(q0), phase="grasp")
            self._checkpoint("pre-close")
            self._send_gripper(0.83)
            self._checkpoint("bilateral-contact", predicate=lambda: self._truth_contact_sides() == (True, True))
            self._checkpoint("stalled-result")
            self._checkpoint("terminal")
        elif self.gate == "retention":
            self._run_trajectory(build_grasp_fixture(q0), phase="grasp")
            self._send_gripper(0.83)
            self._checkpoint("bilateral-grasp", predicate=lambda: self._truth_contact_sides() == (True, True))
            initial = self._truth_cube_pose()
            if initial is None:
                raise RuntimeError("retention requires qualification_cube pose truth")
            self._run_trajectory(build_lift_fixture(), phase="lift")
            self._checkpoint("lift-threshold", predicate=lambda: self._truth_cube_pose() is not None and self._truth_cube_pose()[2] - initial[2] >= 0.10)
            self._checkpoint("translation-threshold", predicate=lambda: self._truth_cube_pose() is not None and math.dist(self._truth_cube_pose(), initial) >= 0.20)
            self.wait_clock((self.clock or 0.0) + 1.0)
            self._checkpoint("stable-hold")
        elif self.gate == "arm-collision":
            self._run_trajectory(build_grasp_fixture(q0), phase="collision", checkpoint_events=(("approach", 0.0),))
            self._checkpoint("terminal")
        else:
            raise ValueError(f"unknown manipulation gate: {self.gate}")
        return self.journal.finalize(success=True)

    def close(self) -> None:
        self.node.destroy_node()


def load_config(path: Path) -> dict[str, Any]:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if raw.get("schema_version") != 2:
        raise ValueError("manipulation qualification config schema_version must be 2")
    return raw


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gate", required=True, choices=GATES)
    parser.add_argument("--attempt-dir", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    journal: EventJournal | None = None
    ros: dict[str, Any] | None = None
    initialized = False
    try:
        config = load_config(args.config)
        configured = config.get("gates", [])
        if args.gate not in configured:
            raise ValueError(f"gate {args.gate!r} is not enabled by {args.config}")
        ros = _load_ros()
        ros["rclpy"].init(args=None)
        initialized = True
        executor = RosGateExecutor(args.gate, args.attempt_dir, config)
        journal = executor.journal
        try:
            executor.run()
        finally:
            executor.close()
            if initialized and ros["rclpy"].ok():
                ros["rclpy"].shutdown()
        return 0
    except Exception as exc:  # CLI failures must be visible in the final artifact.
        if initialized and ros is not None and ros["rclpy"].ok():
            ros["rclpy"].shutdown()
        if journal is None:
            journal = EventJournal(args.attempt_dir, args.gate)
        journal.record("executor_error", simulated_timestamp=None, error=f"{type(exc).__name__}: {exc}")
        journal.finalize(success=False, error=f"{type(exc).__name__}: {exc}")
        print(f"manipulation gate executor failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
