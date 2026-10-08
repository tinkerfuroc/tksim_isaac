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

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, replace
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
# 1.3 deg from it. This coarse bound catches a gross dead write; the
# 0.05 m / 2 deg trial-start bound is checked against the stack session's
# first reset (--tolerance-m / --tolerance-deg).
SPAWN_TOLERANCE_M = 0.15
SPAWN_TOLERANCE_DEG = 5.0
# Live sim AMCL sits 7-17 cm off truth at spawn (front-only +/-82.5 deg
# scan, 2026-10-08); this catches a re-seed that did not take, not that.
AMCL_TOLERANCE_M = 0.25
AMCL_TOLERANCE_DEG = 5.0
STATIONARY_LINEAR_M_S = 0.01
STATIONARY_ANGULAR_RAD_S = 0.02
STATIONARY_HOLD_S = 1.0
# Endpoint discovery on a graph with dozens of participants takes seconds.
DISCOVERY_S = 3.0


class ResetError(RuntimeError):
    """The reset could not establish a clean trial start."""


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ResetOutcome:
    truth: Pose2D  # settled trial-start pose, read after the arm swing
    amcl: Pose2D
    error_m: float  # truth vs scenario spawn
    error_deg: float
    before: Optional[Pose2D] = None
    session: Optional[str] = None


class ResetClient(Protocol):
    def list_actions(self) -> list[str]: ...
    def stack_session(self) -> Optional[str]: ...
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


def _check_spawn(pose: Pose2D, spawn: Pose2D, when: str) -> tuple[float, float]:
    error_m, error_deg = pose_error(pose, spawn)
    if error_m > SPAWN_TOLERANCE_M or error_deg > SPAWN_TOLERANCE_DEG:
        raise ResetError(
            f"base at {_fmt(pose)} {when}, {error_m:.2f} m / {error_deg:.1f} deg"
            f" from spawn {_fmt(spawn)}: the root write did not reach physics"
        )
    return error_m, error_deg


def run_reset(client: ResetClient, spawn: Pose2D, *, reference: Optional[Pose2D] = None,
              tolerance_m: float = 0.05, tolerance_deg: float = 2.0,
              idle_timeout_s: float = 20.0, poll_s: float = 0.2, settle_s: float = 9.0,
              log: Callable[[str], None] = _log) -> ResetOutcome:
    # Every action on the graph, not only the known ones: in live
    # manipulation the arm_api servers (start_grasp, ...) keep sending arm
    # goals after a TIMEOUT-killed orchestrator.
    actions = tuple(dict.fromkeys(NAV_ACTIONS + ARM_ACTIONS + tuple(client.list_actions())))
    # A TIMEOUT-killed orchestrator leaves its last goal driving the robot.
    for action in actions:
        if not client.cancel_all_goals(action):
            log(f"{action}: no action server, nothing to cancel")

    def idle() -> bool:
        active = _active(client, actions)
        # A cancel can be lost to discovery lag; resend while goals remain.
        for action in active:
            client.cancel_all_goals(action)
        # IdleReset needs a stationary robot and only waits 12 s for one.
        return not active and _stationary(client.base_speed())

    def not_idle() -> str:
        active = _active(client, actions)
        if active:
            return f"goals still active after cancel: {active}"
        return "base still moving after goals were cancelled"

    _wait_until(client, idle, idle_timeout_s, poll_s, not_idle, hold_s=STATIONARY_HOLD_S)
    before = client.truth_pose()
    # IdleReset fails *after* asserting its stop if an action starts inside
    # its window, which leaves the sim paused; never start one in that state.
    active = _active(client, actions)
    if active:
        raise ResetError(f"goal became active just before the STATE reset, not resetting: {active}")

    ok, detail = client.reset_state()
    if not ok:
        hint = ""
        if "reset stop remains asserted" in detail:
            hint = (" -- the sim is paused with the reset stop asserted;"
                    " restart the stack (scripts/gpsr-stack down, then up)")
        raise ResetError(f"STATE reset failed: {detail}{hint}")

    restored = client.truth_pose()
    _check_spawn(restored, spawn, "after STATE reset")
    moved_m, _ = pose_error(restored, before)
    log(f"STATE reset moved the base {moved_m:.3f} m from {_fmt(before)}")

    # Odom integrates wheel velocities, which the reset zeroes, so only
    # map->odom moved. Seed AMCL with the truth pose (world == map).
    amcl = client.seed_localization(restored)
    if amcl is None:
        raise ResetError("AMCL did not publish a pose after /initialpose")
    amcl_m, amcl_deg = pose_error(amcl, restored)
    if amcl_m > AMCL_TOLERANCE_M or amcl_deg > AMCL_TOLERANCE_DEG:
        raise ResetError(
            f"AMCL settled at {_fmt(amcl)}, {amcl_m:.2f} m / {amcl_deg:.1f} deg from truth {_fmt(restored)}"
        )

    client.clear_costmaps()
    # STATE restores the stack-up arm pose, which blocks the head camera.
    client.home_arm_and_head()
    client.sleep(settle_s)

    leftover = _active(client, actions)
    if leftover:
        raise ResetError(f"a Nav2 goal started during reset: {leftover}")
    truth = client.truth_pose()
    error_m, error_deg = _check_spawn(truth, spawn, "after the settle")
    if reference is not None:
        ref_m, ref_deg = pose_error(truth, reference)
        if ref_m > tolerance_m or ref_deg > tolerance_deg:
            raise ResetError(
                f"trial start {_fmt(truth)} is {ref_m:.3f} m / {ref_deg:.1f} deg from this stack"
                f" session's first reset {_fmt(reference)} (tolerance {tolerance_m} m /"
                f" {tolerance_deg} deg)"
            )
    log(f"base at {_fmt(truth)} ({error_m:.3f} m / {error_deg:.1f} deg from scenario spawn),"
        f" AMCL {_fmt(amcl)}")
    return ResetOutcome(truth=truth, amcl=amcl, error_m=error_m, error_deg=error_deg,
                        before=before)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RECORD = ROOT / "gpsr_stack_logs" / "run-reset.jsonl"


def outcome_record(outcome: ResetOutcome, spawn: Pose2D) -> dict:
    return {
        "wall_time": time.time(),
        "truth": [outcome.truth.x, outcome.truth.y, outcome.truth.yaw],
        "amcl": [outcome.amcl.x, outcome.amcl.y, outcome.amcl.yaw],
        "spawn": [spawn.x, spawn.y, spawn.yaw],
        "error_m": outcome.error_m,
        "error_deg": outcome.error_deg,
        "before": (None if outcome.before is None
                   else [outcome.before.x, outcome.before.y, outcome.before.yaw]),
        "session": outcome.session,
    }


def read_records(path: Path) -> list[dict]:
    if not Path(path).exists():
        return []
    records = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError:
            continue  # a torn write from a killed reset
    return records


def session_reference(records: list[dict], session: Optional[str]) -> Optional[Pose2D]:
    """Where this stack session's first reset put the base: every later
    trial start must match it. A new stack captures a new baseline."""
    if session is None:
        return None
    for record in records:
        if record.get("session") == session:
            return Pose2D(*record["truth"])
    return None


def start_pose_spread(records: list[dict]) -> tuple[float, float]:
    """Worst trial start against the first reset of the same stack session."""
    firsts: dict = {}
    worst_m = worst_deg = 0.0
    for record in records:
        pose = Pose2D(*record["truth"])
        first = firsts.setdefault(record.get("session"), pose)
        distance, degrees = pose_error(pose, first)
        worst_m, worst_deg = max(worst_m, distance), max(worst_deg, degrees)
    return worst_m, worst_deg


def _summarize(path: Path, tolerance_m: float, tolerance_deg: float) -> int:
    records = read_records(path)
    if not records:
        print(f"{path}: no reset records")
        return 1
    distance, degrees = start_pose_spread(records)
    ok = distance <= tolerance_m and degrees <= tolerance_deg
    print(f"{len(records)} resets; worst start vs first: {distance:.3f} m / {degrees:.2f} deg"
          f" (tolerance {tolerance_m} m / {tolerance_deg} deg): {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


class RosResetClient:
    """The only place this module imports rclpy, constructed lazily so every
    test runs without ROS (same pattern as tools/gpsr_spawn.py)."""

    def __init__(self) -> None:
        import rclpy
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from tinker_sim_interfaces.msg import RobotTruth

        self._rclpy = rclpy
        rclpy.init(args=None)
        # Sim time so the /initialpose stamp matches the TF AMCL looks up.
        self.node = Node("gpsr_run_reset",
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._GoalStatusArray = GoalStatusArray
        self._latched = latched
        self._status: dict[str, object] = {}
        self._cancel_clients: dict[str, object] = {}
        for action in NAV_ACTIONS + ARM_ACTIONS:
            self._watch(action)
        self._truth = None
        self._truth_count = 0
        self.node.create_subscription(RobotTruth, "/sim/truth/robot_state", self._on_truth, 10)
        self._amcl = None
        self._amcl_count = 0
        self.node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, latched)
        self._initial_pose_pub = self.node.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", latched)
        self._PoseWithCovarianceStamped = PoseWithCovarianceStamped
        from nav2_msgs.srv import ClearEntireCostmap
        from simulation_interfaces.srv import ResetSimulation
        self._reset_client = self.node.create_client(ResetSimulation, "/reset_simulation")
        self._costmap_clients = [
            (name, self.node.create_client(ClearEntireCostmap, name))
            for name in ("/global_costmap/clear_entirely_global_costmap",
                         "/local_costmap/clear_entirely_local_costmap")]
        # A request sent before discovery completes is lost (the server's
        # response writer has not matched a brand-new client), and a status
        # topic that has not arrived yet reads as idle.
        self._spin_until(lambda: False, DISCOVERY_S)

    def _watch(self, action: str) -> None:
        if action in self._cancel_clients:
            return
        from action_msgs.srv import CancelGoal
        self.node.create_subscription(
            self._GoalStatusArray, f"{action}/_action/status",
            lambda message, key=action: self._status.__setitem__(key, message), self._latched)
        self._cancel_clients[action] = self.node.create_client(
            CancelGoal, f"{action}/_action/cancel_goal")

    def list_actions(self) -> list[str]:
        suffix = "/_action/status"
        names = sorted(name[: -len(suffix)] for name, _ in self.node.get_topic_names_and_types()
                       if name.endswith(suffix))
        new = [name for name in names if name not in self._cancel_clients]
        for name in new:
            self._watch(name)
        if new:
            self._spin_until(lambda: False, DISCOVERY_S)  # match the new cancel clients
        return names

    def stack_session(self) -> Optional[str]:
        """The sim gateway's /sim/status/isaac publisher GID: unique per sim
        process, so one value per stack bring-up. (services_ready_since is
        sim time and reads 0.0 on every stack.)"""
        topic = "/sim/status/isaac"
        if not self._spin_until(lambda: self.node.get_publishers_info_by_topic(topic), 5.0):
            _log(f"{topic}: no publisher; trial-start reference check skipped")
            return None
        info = self.node.get_publishers_info_by_topic(topic)[0]
        return bytes(info.endpoint_gid).hex()

    def _on_truth(self, message) -> None:
        self._truth = message
        self._truth_count += 1

    def _on_amcl(self, message) -> None:
        self._amcl = message
        self._amcl_count += 1

    def _spin_until(self, predicate: Callable[[], bool], timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    def _call(self, client, name: str, request, *, wait_s: float, call_s: float):
        if not client.wait_for_service(timeout_sec=wait_s):
            return None
        future = client.call_async(request)
        if not self._spin_until(future.done, call_s):
            raise ResetError(f"{name}: no response after {call_s:.0f}s")
        return future.result()

    def _fresh_truth(self, timeout_s: float = 5.0):
        seen = self._truth_count
        if not self._spin_until(lambda: self._truth_count > seen, timeout_s):
            raise ResetError("/sim/truth/robot_state: no message (is truth_evaluator running?)")
        return self._truth

    def cancel_all_goals(self, action: str) -> bool:
        from action_msgs.srv import CancelGoal
        self._watch(action)
        client = self._cancel_clients[action]
        if not client.wait_for_service(timeout_sec=1.0):
            return False
        # An all-zero goal id and stamp cancels every goal on the server.
        future = client.call_async(CancelGoal.Request())
        if not self._spin_until(future.done, 5.0):
            raise ResetError(f"{action}: cancel got no response after 5s")
        if future.result().return_code == CancelGoal.Response.ERROR_REJECTED:
            _log(f"{action}: server rejected the cancel")
        return True

    def active_goals(self, action: str) -> int:
        self._rclpy.spin_once(self.node, timeout_sec=0.05)
        message = self._status.get(action)
        if message is None:
            return 0
        return sum(1 for item in message.status_list if item.status in ACTIVE_GOAL_STATUSES)

    def base_speed(self) -> tuple[float, float]:
        twist = self._fresh_truth().base_twist
        return math.hypot(twist.linear.x, twist.linear.y), abs(twist.angular.z)

    def truth_pose(self) -> Pose2D:
        pose = self._fresh_truth().base_pose
        q = pose.orientation
        return Pose2D(pose.position.x, pose.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def reset_state(self) -> tuple[bool, str]:
        from simulation_interfaces.msg import Result
        from simulation_interfaces.srv import ResetSimulation
        request = ResetSimulation.Request()
        request.scope = SCOPE_STATE
        response = self._call(self._reset_client, "/reset_simulation", request,
                              wait_s=10.0, call_s=90.0)
        if response is None:
            return False, "/reset_simulation not available"
        return response.result.result == Result.RESULT_OK, response.result.error_message

    def seed_localization(self, pose: Pose2D) -> Optional[Pose2D]:
        if not self._spin_until(lambda: self.node.get_clock().now().nanoseconds > 0, 5.0):
            raise ResetError("/clock not received")
        message = self._PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        # Truth is known: 5 cm and ~2 deg standard deviation.
        message.pose.covariance[0] = 0.0025
        message.pose.covariance[7] = 0.0025
        message.pose.covariance[35] = 0.0012
        seed_ns = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        seen = self._amcl_count

        def answered() -> bool:
            if self._amcl_count <= seen:
                return False
            stamp = self._amcl.header.stamp
            return stamp.sec * 10**9 + stamp.nanosec >= seed_ns

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            self._initial_pose_pub.publish(message)
            if self._spin_until(answered, 0.5):
                amcl = self._amcl.pose.pose
                q = amcl.orientation
                return Pose2D(amcl.position.x, amcl.position.y,
                              yaw_from_quaternion(q.x, q.y, q.z, q.w))
        return None

    def clear_costmaps(self) -> None:
        from nav2_msgs.srv import ClearEntireCostmap
        for name, client in self._costmap_clients:
            if self._call(client, name, ClearEntireCostmap.Request(),
                          wait_s=2.0, call_s=5.0) is None:
                _log(f"{name}: not available, skipped")

    def home_arm_and_head(self) -> None:
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
        from tinker_vision_msgs_26.msg import PanTiltCommand
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        # Volatile to match the facade and ros2_control subscriptions during
        # graph churn; wait for both endpoints so a missing head facade
        # cannot leave the arm half-reset.
        qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        arm_pub = self.node.create_publisher(
            JointTrajectory, "/xarm7_traj_controller/joint_trajectory", qos)
        head_pub = self.node.create_publisher(PanTiltCommand, "/pan_tilt_controller/cmd", qos)
        endpoints = [("arm reset (/xarm7_traj_controller/joint_trajectory)", arm_pub),
                     ("head reset (/pan_tilt_controller/cmd)", head_pub)]
        if not self._spin_until(
                lambda: all(p.get_subscription_count() > 0 for _, p in endpoints), 30.0):
            missing = [label for label, p in endpoints if p.get_subscription_count() < 1]
            raise ResetError(f"no matching subscriber after 30s: {', '.join(missing)}")
        trajectory = JointTrajectory()
        trajectory.joint_names = [f"joint{i}" for i in range(1, 8)]
        point = JointTrajectoryPoint()
        point.positions = [0.0] * 7
        # 8 s: a 2 s swing trips the controller's 1.0 rad state tolerance.
        point.time_from_start.sec = 8
        trajectory.points = [point]
        head = PanTiltCommand()
        head.mode = 0
        head.pan_rad = 0.0
        head.tilt_rad = 0.0
        arm_pub.publish(trajectory)
        head_pub.publish(head)
        self._spin_until(lambda: False, 0.5)  # bounded chance for DDS delivery

    def sleep(self, seconds: float) -> None:
        self._spin_until(lambda: False, seconds)

    def now(self) -> float:
        return time.monotonic()

    def close(self) -> None:
        self.node.destroy_node()
        self._rclpy.try_shutdown()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scenario", type=Path,
                        default=ROOT / "simulation/scenarios/gpsr-rcw2026-bench.json")
    parser.add_argument("--record", type=Path,
                        default=Path(os.environ.get("GPSR_RESET_RECORD", DEFAULT_RECORD)))
    parser.add_argument("--summarize", type=Path, metavar="RECORD",
                        help="check a batch's reset records instead of resetting")
    parser.add_argument("--tolerance-m", type=float, default=0.05)
    parser.add_argument("--tolerance-deg", type=float, default=2.0)
    return parser


def main(argv: Optional[list[str]] = None,
         client_factory: Optional[Callable[[], ResetClient]] = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    if args.summarize is not None:
        return _summarize(args.summarize, args.tolerance_m, args.tolerance_deg)
    client = None
    try:
        spawn = scenario_spawn(args.scenario)
        client = (client_factory or RosResetClient)()
        session = client.stack_session()
        reference = session_reference(read_records(args.record), session)
        outcome = replace(run_reset(client, spawn, reference=reference,
                                    tolerance_m=args.tolerance_m,
                                    tolerance_deg=args.tolerance_deg), session=session)
    except ResetError as error:
        _log(str(error))
        return 1
    finally:
        close = getattr(client, "close", None)
        if close is not None:
            close()
    args.record.parent.mkdir(parents=True, exist_ok=True)
    with args.record.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(outcome_record(outcome, spawn)) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
