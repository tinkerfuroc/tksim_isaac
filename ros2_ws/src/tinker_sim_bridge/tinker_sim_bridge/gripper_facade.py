from __future__ import annotations

import math
import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import WrenchStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class GripperFacade(Node):
    def __init__(self) -> None:
        super().__init__("tinker_sim_gripper_facade")
        self.declare_parameter("joint", "drive_joint")
        self.declare_parameter("position_tolerance", 0.002)
        self.declare_parameter("simulation_timeout_s", 5.0)
        self.declare_parameter("wall_watchdog_s", 30.0)
        self.declare_parameter("contact_force_n", 1.0)
        self.declare_parameter("contact_max_age_s", 0.1)
        # Contact-free stall detection.  A finger commanded past an obstacle
        # stops advancing toward the target while still short of it; that is a
        # physical stall, exactly what a real gripper driver reports as a
        # closed grasp -- and it needs no contact telemetry, so it holds in
        # profiles that run the backend without contact reporting (sensor-rich,
        # where /sim/parity/finger_contact is structurally silent).  A close
        # that has not improved its best distance-to-target by ``stall_epsilon``
        # for ``stall_dwell_s`` while still outside ``position_tolerance`` is
        # declared stalled.  Set ``stall_dwell_s`` <= 0 to disable this path and
        # fall back to contact-only stall detection.
        self.declare_parameter("stall_dwell_s", 0.3)
        self.declare_parameter("stall_epsilon", 0.0005)
        self.declare_parameter("safety_timeout_s", 1.0)
        self._joint = str(self.get_parameter("joint").value)
        self._position = 0.0
        self._effort = 0.0
        self._contact_force = 0.0
        self._contact_received_at: float | None = None
        # Discovery is not proof of an effective safety-clear state. Reject
        # goals and hold commands until an explicit false sample arrives.
        self._stopped = True
        self._safety_timeout_s = float(self.get_parameter("safety_timeout_s").value)
        if self._safety_timeout_s <= 0.0:
            raise ValueError("safety timeout must be positive")
        self._safety_last_sample_at: float | None = None
        self._active_goal_reserved = False
        self._active_goal_handle = None
        self._cancel_requested = False
        # A settled grasp must keep clamping.  The GripperCommand action returns
        # once the close has settled, but the physical grip has to persist after
        # that -- and the simulator's command mux drops any source it has not
        # heard from within 0.5 s, replacing it with a zero-effort measured hold
        # (the object slides out at ~5 N).  So once a grasp settles we latch the
        # clamp command here and a steady-time timer republishes it faster than
        # that watchdog, holding full effort until the next goal or a safety stop.
        self._keepalive_command: JointState | None = None
        self._lock = threading.Lock()
        self._commands = self.create_publisher(
            JointState, "/sim/controller/gripper_commands", 20
        )
        callbacks = ReentrantCallbackGroup()
        self.create_subscription(
            JointState,
            "/isaac_joint_states",
            self._state,
            20,
            callback_group=callbacks,
        )
        self.create_subscription(
            WrenchStamped,
            "/sim/parity/finger_contact",
            self._contact,
            20,
            callback_group=callbacks,
        )
        safety_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            "/sim/hardware/safety_stop",
            self._safety,
            safety_qos,
            callback_group=callbacks,
        )
        self.create_timer(
            min(0.1, self._safety_timeout_s / 4.0),
            self._enforce_safety_deadline,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
            callback_group=callbacks,
        )
        # Republish a latched grasp clamp on wall-clock cadence (20 Hz keeps a
        # 10x margin under the mux's 0.5 s source watchdog).  Steady-time, not
        # sim-time: the watchdog measures wall seconds, so a sim-time timer would
        # slow past the deadline whenever RTF drops under load.
        self.create_timer(
            0.05,
            self._keepalive_tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
            callback_group=callbacks,
        )
        self._server = ActionServer(
            self,
            GripperCommand,
            "/xarm_gripper/gripper_action",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=callbacks,
        )

    def _state(self, message: JointState) -> None:
        try:
            index = message.name.index(self._joint)
        except ValueError:
            return
        with self._lock:
            if len(message.position) > index:
                self._position = float(message.position[index])
            if len(message.effort) > index:
                self._effort = float(message.effort[index])

    def _contact(self, message: WrenchStamped) -> None:
        force = message.wrench.force
        with self._lock:
            self._contact_force = (
                force.x * force.x + force.y * force.y + force.z * force.z
            ) ** 0.5
            self._contact_received_at = time.monotonic()

    def _safety(self, message: Bool) -> None:
        hold_position = None
        with self._lock:
            self._safety_last_sample_at = time.monotonic()
            self._stopped = bool(message.data)
            if self._stopped:
                # A safety stop overrides any latched grasp clamp: drop the
                # keepalive and fall back to a zero-effort measured hold.
                self._keepalive_command = None
                hold_position = self._position
        if hold_position is not None:
            self._publish_hold(hold_position)

    def _enforce_safety_deadline(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else float(now)
        hold_position = None
        with self._lock:
            last = self._safety_last_sample_at
            if last is not None and now - last < self._safety_timeout_s:
                return
            if self._stopped:
                return
            self._stopped = True
            # Losing the safety heartbeat is a stop: drop any latched clamp and
            # hold at the measured position with zero effort.
            self._keepalive_command = None
            hold_position = self._position
        if hold_position is not None:
            self._publish_hold(hold_position)

    def _cancel(self, _goal_handle) -> CancelResponse:
        hold_position = None
        with self._lock:
            if self._active_goal_reserved:
                self._cancel_requested = True
                hold_position = self._position
        if hold_position is not None:
            self._publish_hold(hold_position)
        return CancelResponse.ACCEPT

    def _goal(self, _goal) -> GoalResponse:
        self._enforce_safety_deadline()
        with self._lock:
            if self._stopped or self._active_goal_reserved:
                return GoalResponse.REJECT
            self._active_goal_reserved = True
            self._cancel_requested = False
            # A new grasp owns the command stream; retire any prior clamp so a
            # cancel/abort of this goal cannot resume the old hold.
            self._keepalive_command = None
            return GoalResponse.ACCEPT

    def _keepalive_tick(self) -> None:
        # Decide + snapshot under the lock, then publish OUTSIDE it. This timer
        # runs at 20 Hz continuously after any grasp; publishing while holding
        # self._lock can lock-invert with rclpy internals and freeze every
        # callback that needs the lock -- goal_callback included -- hanging the
        # action server (observed: all executor threads parked in futex_wait,
        # no goals accepted).
        publish_command = None
        hold_position = None
        with self._lock:
            # An in-flight goal owns the command stream (its execute loop
            # publishes); the keepalive only runs between goals.
            if self._active_goal_handle is not None or self._active_goal_reserved:
                return
            if self._keepalive_command is None:
                return
            if self._stopped:
                self._keepalive_command = None
                hold_position = self._position
            else:
                publish_command = self._keepalive_command
        if publish_command is not None:
            self._commands.publish(publish_command)
        elif hold_position is not None:
            self._publish_hold(hold_position)

    def _publish_hold(self, position: float) -> None:
        # Publish a zero-effort measured-position hold. MUST be called WITHOUT
        # self._lock held: a publish under the lock can lock-invert with rclpy
        # internals and hang every lock-waiting callback (see _keepalive_tick).
        # Callers snapshot the position under the lock, then call this outside.
        command = JointState()
        command.name = [self._joint]
        command.position = [float(position)]
        command.effort = [0.0]
        self._commands.publish(command)

    def _release_goal(self, goal_handle) -> None:
        with self._lock:
            if self._active_goal_handle is goal_handle or self._active_goal_handle is None:
                self._active_goal_handle = None
                self._active_goal_reserved = False
                self._cancel_requested = False

    def _log_execute_outcome(
        self,
        outcome: str,
        *,
        position: float,
        effort: float,
        stalled: bool,
        reached_goal: bool,
        start_sim: float,
        start_wall: float,
    ) -> None:
        # The facade otherwise logs nothing: every diagnostic fact about a
        # close (when it started, which predicate fired, how far the drive
        # had actually moved) had to be reconstructed indirectly from a
        # client's own logs and the truth evaluator (see #27). One line per
        # terminal outcome makes that a single grep.
        elapsed_sim = self.get_clock().now().nanoseconds * 1.0e-9 - start_sim
        elapsed_wall = time.monotonic() - start_wall
        self.get_logger().info(
            "gripper close outcome=%s position=%.6f effort=%.6f stalled=%s "
            "reached_goal=%s elapsed_sim_s=%.3f elapsed_wall_s=%.3f"
            % (outcome, position, effort, stalled, reached_goal, elapsed_sim, elapsed_wall)
        )

    def _execute(self, goal_handle):
        target = float(goal_handle.request.command.position)
        requested_effort = float(goal_handle.request.command.max_effort)
        result = GripperCommand.Result()
        with self._lock:
            active = self._active_goal_handle is not None
            if not active:
                reserved = self._active_goal_reserved
                self._active_goal_reserved = True
                self._active_goal_handle = goal_handle
                if not reserved:
                    self._cancel_requested = False
        if active:
            goal_handle.abort()
            return result
        if not math.isfinite(target) or not math.isfinite(requested_effort) or requested_effort < 0.0:
            try:
                goal_handle.abort()
                return result
            finally:
                self._release_goal(goal_handle)
        max_effort = requested_effort
        command = JointState()
        command.name = [self._joint]
        command.position = [target]
        command.effort = [max_effort]
        try:
            start_sim = self.get_clock().now().nanoseconds * 1.0e-9
            start_wall = time.monotonic()
            tolerance = float(self.get_parameter("position_tolerance").value)
            simulation_timeout = float(
                self.get_parameter("simulation_timeout_s").value
            )
            wall_watchdog = float(self.get_parameter("wall_watchdog_s").value)
            contact_threshold = float(self.get_parameter("contact_force_n").value)
            contact_max_age = float(self.get_parameter("contact_max_age_s").value)
            stall_dwell = float(self.get_parameter("stall_dwell_s").value)
            stall_epsilon = float(self.get_parameter("stall_epsilon").value)
            best_error: float | None = None
            # Sim seconds, not wall seconds: the node runs with use_sim_time
            # and the drive advances in SIM time, so the no-progress dwell
            # (stall_dwell) must be measured against the same clock or it can
            # fire before the drive has had any chance to move under RTF<1
            # (see #27 -- bench RTF 0.27 fired a false stall at 4% stroke).
            last_progress_at = start_sim
            contact_since: float | None = None
            with self._lock:
                position = self._position
                effort = self._effort
            while rclpy.ok():
                self._enforce_safety_deadline()
                with self._lock:
                    position = self._position
                    effort = self._effort
                    contact = self._contact_force
                    contact_received_at = self._contact_received_at
                    stopped = self._stopped
                    cancel_requested = (
                        self._cancel_requested or goal_handle.is_cancel_requested
                    )
                if stopped:
                    self._publish_hold(position)
                    goal_handle.abort()
                    self._log_execute_outcome(
                        "aborted_safety_stop",
                        position=position,
                        effort=effort,
                        stalled=False,
                        reached_goal=False,
                        start_sim=start_sim,
                        start_wall=start_wall,
                    )
                    break
                if cancel_requested:
                    self._publish_hold(position)
                    goal_handle.canceled()
                    self._log_execute_outcome(
                        "canceled",
                        position=position,
                        effort=effort,
                        stalled=False,
                        reached_goal=False,
                        start_sim=start_sim,
                        start_wall=start_wall,
                    )
                    break
                with self._lock:
                    stopped = self._stopped
                    cancel_requested = (
                        self._cancel_requested or goal_handle.is_cancel_requested
                    )
                    should_publish = not stopped and not cancel_requested
                    if not should_publish:
                        position = self._position
                if should_publish:
                    # Publish OUTSIDE the lock (see _publish_hold): holding
                    # self._lock across a publish risks a lock-inversion hang.
                    self._commands.publish(command)
                if stopped:
                    self._publish_hold(position)
                    goal_handle.abort()
                    self._log_execute_outcome(
                        "aborted_safety_stop",
                        position=position,
                        effort=effort,
                        stalled=False,
                        reached_goal=False,
                        start_sim=start_sim,
                        start_wall=start_wall,
                    )
                    break
                if cancel_requested:
                    self._publish_hold(position)
                    goal_handle.canceled()
                    self._log_execute_outcome(
                        "canceled",
                        position=position,
                        effort=effort,
                        stalled=False,
                        reached_goal=False,
                        start_sim=start_sim,
                        start_wall=start_wall,
                    )
                    break
                error = abs(target - position)
                contact_is_fresh = (
                    contact_received_at is not None
                    and contact_received_at >= start_wall
                    and time.monotonic() - contact_received_at <= contact_max_age
                )
                # SIM seconds (see the comment on last_progress_at above) --
                # the dwell must track the clock the drive actually moves in,
                # not wall time, or it fires before RTF<1 has given the drive
                # a chance to move.
                now_sim = self.get_clock().now().nanoseconds * 1.0e-9
                if best_error is None or error < best_error - stall_epsilon:
                    best_error = error
                    last_progress_at = now_sim
                position_stalled = (
                    stall_dwell > 0.0
                    and error > tolerance
                    and now_sim - last_progress_at >= stall_dwell
                )
                # Success is a DRIVE STALL (or reached_goal), not first touch --
                # hardware parity with the real xArm gripper action, which only
                # returns once the drive stalls, reaches its target, or hits its
                # effort limit.  A fresh contact is not on its own proof of that:
                # a finger can brush an object, or squish into a compliant one,
                # while the drive keeps advancing for seconds afterward (a
                # validation round saw the drive close from 0.162 rad to 0.569
                # rad -- 19% to 67% of stroke -- after a contact-only success had
                # already returned).  So contact_stalled is gated by the SAME
                # non-advancing test as the position path: it can only confirm a
                # stall that position_stalled would already report on its own,
                # never trigger one while the drive is still closing.  With
                # stall_dwell <= 0 (the isolation-test / contact-only fallback)
                # both dwell checks are satisfied immediately, so this reduces to
                # the original instantaneous contact stall.
                if contact_is_fresh and contact >= contact_threshold:
                    if contact_since is None:
                        contact_since = now_sim
                else:
                    contact_since = None
                contact_stalled = (
                    contact_since is not None
                    and error > tolerance
                    and now_sim - contact_since >= stall_dwell
                    and now_sim - last_progress_at >= stall_dwell
                )
                feedback = GripperCommand.Feedback()
                feedback.position = position
                feedback.effort = effort
                feedback.reached_goal = error <= tolerance
                feedback.stalled = position_stalled or contact_stalled
                goal_handle.publish_feedback(feedback)
                if feedback.reached_goal or feedback.stalled:
                    # Latch the clamp so it survives past the action's return:
                    # the keepalive timer keeps streaming this command (full
                    # effort at the commanded target) until the next goal or a
                    # safety stop, holding the grasp instead of decaying to the
                    # mux's zero-effort measured hold.
                    with self._lock:
                        self._keepalive_command = command
                    goal_handle.succeed()
                    result.reached_goal = feedback.reached_goal
                    result.stalled = feedback.stalled
                    self._log_execute_outcome(
                        "reached_goal" if feedback.reached_goal else "stalled",
                        position=position,
                        effort=effort,
                        stalled=feedback.stalled,
                        reached_goal=feedback.reached_goal,
                        start_sim=start_sim,
                        start_wall=start_wall,
                    )
                    break
                sim_elapsed = (
                    self.get_clock().now().nanoseconds * 1.0e-9 - start_sim
                )
                wall_elapsed = time.monotonic() - start_wall
                if sim_elapsed >= simulation_timeout or wall_elapsed >= wall_watchdog:
                    self._publish_hold(position)
                    goal_handle.abort()
                    self._log_execute_outcome(
                        "timeout_wall" if wall_elapsed >= wall_watchdog else "timeout_sim",
                        position=position,
                        effort=effort,
                        stalled=False,
                        reached_goal=False,
                        start_sim=start_sim,
                        start_wall=start_wall,
                    )
                    break
                time.sleep(0.01)
            result.position = position
            result.effort = effort
            return result
        finally:
            self._release_goal(goal_handle)


def main() -> None:
    rclpy.init()
    node = GripperFacade()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
