"""Per-trial GPSR bench reset: stop motion, put the base back at spawn,
re-localize, re-home arm and head.

Run by ``scripts/gpsr-run-reset`` (the ``--reset-cmd`` of every tier-2
battery). The base is restored with the sim's scoped STATE reset
(``/reset_simulation`` scope 2, ``simulation/tinker_sim_isaac/scoped_reset.py``),
which writes the root pose through the live PhysX articulation view. Never
use SetEntityState for the robot: it writes a USD layer the articulation
never reads back (commit 5fa9186).
"""
from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Protocol

NAV_ACTIONS = ("/navigate_to_pose", "/go_to_approach")
# IdleReset rejects a reset while either of these has an active goal.
ARM_ACTIONS = (
    "/xarm7_traj_controller/follow_joint_trajectory",
    "/xarm_gripper/gripper_action",
)
ACTIVE_GOAL_STATUSES = (1, 2, 3)  # ACCEPTED, EXECUTING, CANCELING
SCOPE_STATE = 2
# The scenario pose is the spawn *request*; the settled spawn sits ~0.07 m /
# 1.3 deg from it. This bound catches a dead write; the 0.05 m / 2 deg
# trial-to-trial bound is checked across a batch with --summarize.
SPAWN_TOLERANCE_M = 0.15
SPAWN_TOLERANCE_DEG = 5.0
AMCL_TOLERANCE_M = 0.15
AMCL_TOLERANCE_DEG = 5.0
STATIONARY_LINEAR_M_S = 0.01
STATIONARY_ANGULAR_RAD_S = 0.02
STATIONARY_HOLD_S = 1.0


class ResetError(RuntimeError):
    """The reset could not establish a clean trial start."""


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ResetOutcome:
    truth: Pose2D
    amcl: Pose2D
    error_m: float
    error_deg: float


class ResetClient(Protocol):
    def cancel_all_goals(self, action: str) -> bool: ...
    def active_goals(self, action: str) -> int: ...
    def base_speed(self) -> tuple[float, float]: ...
    def truth_pose(self) -> Pose2D: ...
    def reset_state(self) -> tuple[bool, str]: ...
    def seed_localization(self, pose: Pose2D) -> Optional[Pose2D]: ...
    def clear_costmaps(self) -> None: ...
    def home_arm_and_head(self) -> None: ...
    def sleep(self, seconds: float) -> None: ...
    def now(self) -> float: ...


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_error(a: Pose2D, b: Pose2D) -> tuple[float, float]:
    distance = math.hypot(a.x - b.x, a.y - b.y)
    dyaw = math.atan2(math.sin(a.yaw - b.yaw), math.cos(a.yaw - b.yaw))
    return distance, abs(math.degrees(dyaw))


def scenario_spawn(path: Path) -> Pose2D:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    pose = data.get("robot", {}).get("initial_pose")
    if not isinstance(pose, list) or len(pose) < 3:
        raise ResetError(f"{path}: robot.initial_pose [x, y, yaw] missing")
    return Pose2D(float(pose[0]), float(pose[1]), float(pose[2]))


def _log(message: str) -> None:
    print(f"gpsr-run-reset: {message}", file=sys.stderr, flush=True)


def _active(client: ResetClient, actions) -> dict[str, int]:
    counts = {action: client.active_goals(action) for action in actions}
    return {action: n for action, n in counts.items() if n}


def _wait_until(client: ResetClient, predicate: Callable[[], bool], timeout_s: float,
                poll_s: float, failure: Callable[[], str], *, hold_s: float = 0.0) -> None:
    deadline = client.now() + timeout_s
    held_since = None
    while True:
        if predicate():
            if held_since is None:
                held_since = client.now()
            if client.now() - held_since >= hold_s:
                return
        else:
            held_since = None
        if client.now() >= deadline:
            raise ResetError(failure())
        client.sleep(poll_s)


def _stationary(speed: tuple[float, float]) -> bool:
    linear, angular = speed
    return linear <= STATIONARY_LINEAR_M_S and angular <= STATIONARY_ANGULAR_RAD_S


def _fmt(pose: Pose2D) -> str:
    return f"({pose.x:.3f}, {pose.y:.3f}, {math.degrees(pose.yaw):.1f} deg)"


def run_reset(client: ResetClient, spawn: Pose2D, *, idle_timeout_s: float = 20.0,
              poll_s: float = 0.2, settle_s: float = 9.0,
              log: Callable[[str], None] = _log) -> ResetOutcome:
    actions = NAV_ACTIONS + ARM_ACTIONS
    # A TIMEOUT-killed orchestrator leaves its last goal driving the robot.
    for action in actions:
        if not client.cancel_all_goals(action):
            log(f"{action}: no action server, nothing to cancel")
    _wait_until(client, lambda: not _active(client, actions), idle_timeout_s, poll_s,
                lambda: f"goals still active after cancel: {_active(client, actions)}")
    # IdleReset needs a stationary robot and only waits 12 s for one.
    _wait_until(client, lambda: _stationary(client.base_speed()), idle_timeout_s, poll_s,
                lambda: "base still moving after goals were cancelled",
                hold_s=STATIONARY_HOLD_S)

    ok, detail = client.reset_state()
    if not ok:
        hint = ""
        if "reset stop remains asserted" in detail:
            hint = (" -- the sim is paused with the reset stop asserted;"
                    " restart the stack (scripts/gpsr-stack down, then up)")
        raise ResetError(f"STATE reset failed: {detail}{hint}")

    truth = client.truth_pose()
    error_m, error_deg = pose_error(truth, spawn)
    if error_m > SPAWN_TOLERANCE_M or error_deg > SPAWN_TOLERANCE_DEG:
        raise ResetError(
            f"base at {_fmt(truth)} after STATE reset, {error_m:.2f} m / {error_deg:.1f} deg"
            f" from spawn {_fmt(spawn)}: the root write did not reach physics"
        )

    # Odom integrates wheel velocities, which the reset zeroes, so only
    # map->odom moved. Seed AMCL with the truth pose (world == map).
    amcl = client.seed_localization(truth)
    if amcl is None:
        raise ResetError("AMCL did not publish a pose after /initialpose")
    amcl_m, amcl_deg = pose_error(amcl, truth)
    if amcl_m > AMCL_TOLERANCE_M or amcl_deg > AMCL_TOLERANCE_DEG:
        raise ResetError(
            f"AMCL settled at {_fmt(amcl)}, {amcl_m:.2f} m / {amcl_deg:.1f} deg from truth {_fmt(truth)}"
        )

    client.clear_costmaps()
    # STATE restores the stack-up arm pose, which blocks the head camera.
    client.home_arm_and_head()
    client.sleep(settle_s)

    leftover = _active(client, NAV_ACTIONS)
    if leftover:
        raise ResetError(f"a Nav2 goal started during reset: {leftover}")
    log(f"base at {_fmt(truth)} ({error_m:.3f} m / {error_deg:.1f} deg from scenario spawn),"
        f" AMCL {_fmt(amcl)}")
    return ResetOutcome(truth=truth, amcl=amcl, error_m=error_m, error_deg=error_deg)
