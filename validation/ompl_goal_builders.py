"""ROS-free deterministic plain-data goal builders (Task 7).

This module runs under simulator CPython 3.12.  It imports neither ROS, nor
``rclpy``, nor ``moveit_msgs``, nor any generated message type: it builds plain
frozen dataclasses that the live Humble client in :mod:`ompl_plan_smoke`
converts into ``moveit_msgs/action/MoveGroup`` goals with
``pipeline_id="ompl"`` and ``planning_options.plan_only=true``.

``build_joint_goal`` and ``build_pose_goal`` are deterministic: every optional
parameter has a fixed default, all numeric values are validated finite, and a
zero-norm quaternion / empty joint set / nonfinite position is rejected
fail-closed.  ``goal_to_dict`` yields a stable plain-data dictionary used for
report evidence and ``goal_kind`` labels the mode.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping, Sequence, Tuple

GROUP_NAME = "xarm7"
PIPELINE_ID = "ompl"
PLAN_ONLY = True
DEFAULT_PLANNER_ID = ""
DEFAULT_ALLOWED_PLANNING_TIME = 5.0
DEFAULT_NUM_PLANNING_ATTEMPTS = 1

DEFAULT_POSE_FRAME_ID = "base_link"
DEFAULT_POSE_LINK_NAME = "link_tcp"
DEFAULT_POSITION_TOLERANCE = 0.02
DEFAULT_ORIENTATION_TOLERANCE = 0.1
DEFAULT_USE_ORIENTATION = False

ARM_JOINT_NAMES: Tuple[str, ...] = tuple("joint{}".format(i) for i in range(1, 8))

# Deterministic joint target used by joint mode.  All values are within the real
# xArm7 hardware limits (joint1/3/5/7 +/-2pi, joint2 [-2.059, 2.0944],
# joint4 [-0.192, 3.927], joint6 [-1.693, 3.142]) and are a small reach from a
# home/vertical arm, keeping the goal collision-free relative to the fixture
# pedestal at x >= 0.2.  OMPL produces a short nonempty plan to this target.
JOINT_TARGET_POSITIONS: Mapping[str, float] = {
    "joint1": 0.0,
    "joint2": 0.0,
    "joint3": 0.0,
    "joint4": 0.2,
    "joint5": 0.0,
    "joint6": 0.3,
    "joint7": 0.0,
}
JOINT_TARGET_TOLERANCES: Mapping[str, float] = {
    name: 0.02 for name in ARM_JOINT_NAMES
}

# Vertical approach offset applied on top of a scenario target object's pose so
# the link_tcp position constraint region is above (not inside) the 8 cm target
# box, keeping goal sampling collision-free in the free-space scenarios.
POSE_APPROACH_Z_OFFSET = 0.10

# Overhead z-down approach quaternion (Rz45*Rx180, sign-equivalent accepted)
# applied to generated pose goals so the TCP approaches the target from above
# instead of pointing at the collision-box center.  The fixture declaration
# stays yaw-only; only the generated goals carry this orientation.
POSE_APPROACH_QUATERNION_XYZW: Tuple[float, float, float, float] = (
    0.9238795,
    0.3826834,
    0.0,
    0.0,
)


@dataclass(frozen=True)
class JointGoal:
    """Plain-data joint-space goal (``group_name`` + joint positions)."""

    group_name: str
    joint_positions: Mapping[str, float]
    tolerances: Mapping[str, float]
    pipeline_id: str
    plan_only: bool
    planner_id: str
    allowed_planning_time: float
    num_planning_attempts: int


@dataclass(frozen=True)
class PoseGoal:
    """Plain-data Cartesian pose goal for a named TCP link."""

    group_name: str
    link_name: str
    frame_id: str
    position_xyz: Tuple[float, float, float]
    orientation_xyzw: Tuple[float, float, float, float]
    position_tolerance: float
    orientation_tolerance: float
    use_orientation: bool
    pipeline_id: str
    plan_only: bool
    planner_id: str
    allowed_planning_time: float
    num_planning_attempts: int


Goal = object  # JointGoal | PoseGoal (module-level alias for annotation below)


def goal_kind(goal: object) -> str:
    """Return ``"joint"`` or ``"pose"`` for a builder-produced goal."""
    if isinstance(goal, JointGoal):
        return "joint"
    if isinstance(goal, PoseGoal):
        return "pose"
    raise TypeError("goal must be a JointGoal or PoseGoal, got {!r}".format(type(goal)))


def goal_to_dict(goal: object) -> dict[str, object]:
    """Return a stable plain-data dictionary for report evidence."""
    common = {
        "kind": goal_kind(goal),
        "group_name": str(goal.group_name),
        "pipeline_id": str(goal.pipeline_id),
        "plan_only": bool(goal.plan_only),
        "planner_id": str(goal.planner_id),
        "allowed_planning_time": float(goal.allowed_planning_time),
        "num_planning_attempts": int(goal.num_planning_attempts),
    }
    if isinstance(goal, JointGoal):
        return {
            **common,
            "joint_positions": {
                str(name): float(value) for name, value in goal.joint_positions.items()
            },
            "tolerances": {
                str(name): float(value) for name, value in goal.tolerances.items()
            },
        }
    return {
        **common,
        "link_name": str(goal.link_name),
        "frame_id": str(goal.frame_id),
        "position_xyz": [float(value) for value in goal.position_xyz],
        "orientation_xyzw": [float(value) for value in goal.orientation_xyzw],
        "position_tolerance": float(goal.position_tolerance),
        "orientation_tolerance": float(goal.orientation_tolerance),
        "use_orientation": bool(goal.use_orientation),
    }


def _require_finite(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("{} must be a finite number, got {!r}".format(label, value))
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("{} must be finite, got {!r}".format(label, value))
    return number


def _require_positive(value: object, label: str) -> float:
    number = _require_finite(value, label)
    if number <= 0.0:
        raise ValueError("{} must be positive, got {!r}".format(label, value))
    return number


def build_joint_goal(
    joint_positions: Mapping[str, float] | None = None,
    *,
    group_name: str = GROUP_NAME,
    tolerances: Mapping[str, float] | None = None,
    pipeline_id: str = PIPELINE_ID,
    plan_only: bool = PLAN_ONLY,
    planner_id: str = DEFAULT_PLANNER_ID,
    allowed_planning_time: float = DEFAULT_ALLOWED_PLANNING_TIME,
    num_planning_attempts: int = DEFAULT_NUM_PLANNING_ATTEMPTS,
) -> JointGoal:
    """Build a deterministic joint-space goal (ROS-free).

    ``joint_positions`` defaults to the reachable preset
    :data:`JOINT_TARGET_POSITIONS`.  Every position must be finite and the set
    nonempty; every tolerance must be positive.  The returned dataclass carries
    ``pipeline_id="ompl"`` and ``plan_only=True`` by default.
    """
    positions = dict(joint_positions if joint_positions is not None else JOINT_TARGET_POSITIONS)
    if not positions:
        raise ValueError("joint_positions must be nonempty")
    normalized: dict[str, float] = {}
    for name, value in positions.items():
        if not isinstance(name, str) or not name:
            raise ValueError("joint names must be nonempty strings")
        normalized[str(name)] = _require_finite(value, "joint {!r} position".format(name))
    if tolerances is None:
        tolerance_map = {
            name: JOINT_TARGET_TOLERANCES.get(name, 0.02) for name in normalized
        }
    else:
        tolerance_map = dict(tolerances)
    normalized_tolerances: dict[str, float] = {}
    for name, value in tolerance_map.items():
        normalized_tolerances[str(name)] = _require_positive(
            value, "joint {!r} tolerance".format(name)
        )
    return JointGoal(
        group_name=str(group_name),
        joint_positions=normalized,
        tolerances=normalized_tolerances,
        pipeline_id=str(pipeline_id),
        plan_only=bool(plan_only),
        planner_id=str(planner_id),
        allowed_planning_time=_require_positive(allowed_planning_time, "allowed_planning_time"),
        num_planning_attempts=int(num_planning_attempts),
    )


def build_pose_goal(
    position_xyz: Sequence[float],
    orientation_xyzw: Sequence[float] | None = None,
    *,
    group_name: str = GROUP_NAME,
    link_name: str = DEFAULT_POSE_LINK_NAME,
    frame_id: str = DEFAULT_POSE_FRAME_ID,
    position_tolerance: float = DEFAULT_POSITION_TOLERANCE,
    orientation_tolerance: float = DEFAULT_ORIENTATION_TOLERANCE,
    use_orientation: bool = DEFAULT_USE_ORIENTATION,
    pipeline_id: str = PIPELINE_ID,
    plan_only: bool = PLAN_ONLY,
    planner_id: str = DEFAULT_PLANNER_ID,
    allowed_planning_time: float = DEFAULT_ALLOWED_PLANNING_TIME,
    num_planning_attempts: int = DEFAULT_NUM_PLANNING_ATTEMPTS,
) -> PoseGoal:
    """Build a deterministic Cartesian pose goal (ROS-free).

    ``position_xyz`` must be exactly three finite numbers; ``orientation_xyzw``
    (default identity) must be four finite numbers with nonzero norm and is
    normalized to unit length so the goal is deterministic under scale.
    """
    xyz = [_require_finite(value, "position component") for value in position_xyz]
    if len(xyz) != 3:
        raise ValueError("position_xyz must contain exactly three components")
    quat_source = (
        list(orientation_xyzw) if orientation_xyzw is not None else [0.0, 0.0, 0.0, 1.0]
    )
    quat = [_require_finite(value, "quaternion component") for value in quat_source]
    if len(quat) != 4:
        raise ValueError("orientation_xyzw must contain exactly four components")
    norm = math.sqrt(sum(value * value for value in quat))
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("orientation_xyzw must have nonzero norm")
    unit_quat = tuple(value / norm for value in quat)
    return PoseGoal(
        group_name=str(group_name),
        link_name=str(link_name),
        frame_id=str(frame_id),
        position_xyz=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
        orientation_xyzw=unit_quat,
        position_tolerance=_require_positive(position_tolerance, "position_tolerance"),
        orientation_tolerance=_require_positive(orientation_tolerance, "orientation_tolerance"),
        use_orientation=bool(use_orientation),
        pipeline_id=str(pipeline_id),
        plan_only=bool(plan_only),
        planner_id=str(planner_id),
        allowed_planning_time=_require_positive(allowed_planning_time, "allowed_planning_time"),
        num_planning_attempts=int(num_planning_attempts),
    )


__all__ = [
    "ARM_JOINT_NAMES",
    "DEFAULT_ALLOWED_PLANNING_TIME",
    "DEFAULT_NUM_PLANNING_ATTEMPTS",
    "DEFAULT_ORIENTATION_TOLERANCE",
    "DEFAULT_PLANNER_ID",
    "DEFAULT_POSE_FRAME_ID",
    "DEFAULT_POSE_LINK_NAME",
    "DEFAULT_POSITION_TOLERANCE",
    "DEFAULT_USE_ORIENTATION",
    "GROUP_NAME",
    "JOINT_TARGET_POSITIONS",
    "JOINT_TARGET_TOLERANCES",
    "PIPELINE_ID",
    "PLAN_ONLY",
    "POSE_APPROACH_QUATERNION_XYZW",
    "POSE_APPROACH_Z_OFFSET",
    "JointGoal",
    "PoseGoal",
    "build_joint_goal",
    "build_pose_goal",
    "goal_kind",
    "goal_to_dict",
]
