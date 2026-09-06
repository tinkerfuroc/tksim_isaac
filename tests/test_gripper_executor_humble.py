from __future__ import annotations

import threading
import time

import pytest


rclpy = pytest.importorskip("rclpy")
pytest.importorskip("control_msgs")
pytest.importorskip("geometry_msgs")
pytest.importorskip("sensor_msgs")
pytest.importorskip("std_msgs")

from control_msgs.action import GripperCommand  # noqa: E402
from geometry_msgs.msg import WrenchStamped  # noqa: E402
from rclpy.action import GoalResponse  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from tinker_sim_bridge.gripper_facade import GripperFacade  # noqa: E402


class _GoalHandle:
    def __init__(self, target: float = 0.5) -> None:
        self.request = GripperCommand.Goal()
        self.request.command.position = target
        self.request.command.max_effort = 1.0
        self.is_cancel_requested = False
        self.outcome = None
        self.finished = threading.Event()
        self.feedback = []

    def abort(self) -> None:
        self.outcome = "aborted"
        self.finished.set()

    def canceled(self) -> None:
        self.outcome = "canceled"
        self.finished.set()

    def succeed(self) -> None:
        self.outcome = "succeeded"
        self.finished.set()

    def publish_feedback(self, feedback) -> None:
        self.feedback.append(feedback)


def _run_goal_with_timer(node: GripperFacade, goal: _GoalHandle):
    timer = None

    def run() -> None:
        timer.cancel()
        node._execute(goal)

    timer = node.create_timer(0.05, run)
    return timer


def _clear_safety(node: GripperFacade) -> None:
    message = Bool()
    message.data = False
    node._safety(message)


def _disable_position_stall(node: GripperFacade) -> None:
    # Isolate the property under test from the contact-free position-stall
    # path: a finger parked away from its target is a stall by design, so
    # tests that deliberately hold it there (cancel, safety, stale-contact)
    # disable that path to exercise only their own concern.
    node.set_parameters([Parameter("stall_dwell_s", Parameter.Type.DOUBLE, 0.0)])


def _spin(node: Node, source: Node):
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(source)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    return executor, spin_thread


@pytest.fixture
def ros_context():
    if rclpy.ok():
        yield
        return
    rclpy.init()
    try:
        yield
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_joint_state_callback_progresses_during_execute(ros_context) -> None:
    node = GripperFacade()
    _clear_safety(node)
    source = Node("gripper_executor_state_source")
    publisher = source.create_publisher(JointState, "/isaac_joint_states", 20)
    goal = _GoalHandle()
    _run_goal_with_timer(node, goal)

    def publish_state() -> None:
        message = JointState()
        message.name = ["drive_joint"]
        message.position = [0.5]
        publisher.publish(message)

    state_timer = source.create_timer(0.1, publish_state)
    executor, spin_thread = _spin(node, source)
    try:
        assert goal.finished.wait(2.0)
        assert goal.outcome == "succeeded"
    finally:
        state_timer.cancel()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        source.destroy_node()


def test_safety_callback_aborts_execute(ros_context) -> None:
    node = GripperFacade()
    _clear_safety(node)
    _disable_position_stall(node)
    source = Node("gripper_executor_safety_source")
    command_messages = []
    source.create_subscription(
        JointState,
        "/sim/controller/gripper_commands",
        command_messages.append,
        20,
    )
    safety_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = source.create_publisher(
        Bool, "/sim/hardware/safety_stop", safety_qos
    )
    goal = _GoalHandle(target=1.0)
    _run_goal_with_timer(node, goal)

    def publish_stop() -> None:
        message = Bool()
        message.data = True
        publisher.publish(message)

    stop_timer = source.create_timer(0.1, publish_stop)
    executor, spin_thread = _spin(node, source)
    try:
        assert goal.finished.wait(2.0)
        assert goal.outcome == "aborted"
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not any(
            message.effort == pytest.approx([0.0]) for message in command_messages
        ):
            time.sleep(0.01)
        assert any(
            message.effort == pytest.approx([0.0]) for message in command_messages
        )
    finally:
        stop_timer.cancel()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        source.destroy_node()


def test_gripper_rejects_goal_before_first_effective_clear_sample(ros_context) -> None:
    node = GripperFacade()
    source = Node("gripper_startup_safety_source")
    try:
        assert node._stopped
        goal = _GoalHandle(target=0.5)
        assert node._goal(goal.request) == GoalResponse.REJECT

        clear = Bool()
        clear.data = False
        node._safety(clear)
        assert not node._stopped
        assert node._goal(goal.request) == GoalResponse.ACCEPT
    finally:
        node.destroy_node()
        source.destroy_node()


def test_gripper_safety_heartbeat_timeout_retires_goal(ros_context) -> None:
    node = GripperFacade()
    source = Node("gripper_safety_deadline_source")
    try:
        _clear_safety(node)
        goal = _GoalHandle(target=0.5)
        assert node._goal(goal.request) == GoalResponse.ACCEPT
        with node._lock:
            node._safety_last_sample_at = 10.0

        node._enforce_safety_deadline(now=11.0)

        assert node._stopped
        assert node._active_goal_reserved
    finally:
        node.destroy_node()
        source.destroy_node()


def test_rejects_second_goal_while_first_is_active(ros_context) -> None:
    node = GripperFacade()
    _clear_safety(node)
    source = Node("gripper_executor_concurrency_source")
    first = _GoalHandle(target=1.0)
    assert node._goal(first.request) == GoalResponse.ACCEPT
    second = _GoalHandle(target=0.5)
    assert node._goal(second.request) == GoalResponse.REJECT
    first.is_cancel_requested = True
    _run_goal_with_timer(node, first)
    executor, spin_thread = _spin(node, source)
    try:
        assert first.finished.wait(2.0)
        assert first.outcome == "canceled"
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        source.destroy_node()


def test_cancel_publishes_measured_zero_effort_hold(ros_context) -> None:
    node = GripperFacade()
    _clear_safety(node)
    _disable_position_stall(node)
    source = Node("gripper_executor_cancel_source")
    state_publisher = source.create_publisher(JointState, "/isaac_joint_states", 20)
    command_messages = []
    command_observer = source.create_subscription(
        JointState,
        "/sim/controller/gripper_commands",
        command_messages.append,
        20,
    )
    goal = _GoalHandle(target=1.0)
    _run_goal_with_timer(node, goal)
    executor, spin_thread = _spin(node, source)
    try:
        deadline = time.monotonic() + 1.0
        while (
            time.monotonic() < deadline
            and state_publisher.get_subscription_count() < 1
        ):
            time.sleep(0.01)
        state = JointState()
        state.name = ["drive_joint"]
        state.position = [0.25]
        state_publisher.publish(state)
        time_limit = time.monotonic() + 1.0
        while time.monotonic() < time_limit:
            with node._lock:
                measured = node._position
            if abs(measured - 0.25) < 0.01:
                break
            time.sleep(0.01)
        goal.is_cancel_requested = True
        assert goal.finished.wait(2.0)
        assert goal.outcome == "canceled"
        time_limit = time.monotonic() + 1.0
        while time.monotonic() < time_limit and not any(
            message.effort == pytest.approx([0.0]) for message in command_messages
        ):
            time.sleep(0.01)
        retirement = next(
            message
            for message in reversed(command_messages)
            if message.effort == pytest.approx([0.0])
        )
        assert retirement.position == pytest.approx([0.25], abs=0.02)
        assert retirement.effort == pytest.approx([0.0])
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        source.destroy_node()


def test_stale_contact_from_previous_goal_does_not_stall(ros_context) -> None:
    node = GripperFacade()
    _clear_safety(node)
    _disable_position_stall(node)
    source = Node("gripper_executor_stale_contact_source")
    contact_publisher = source.create_publisher(
        WrenchStamped, "/sim/parity/finger_contact", 20
    )
    goal = _GoalHandle(target=1.0)
    contact = WrenchStamped()
    contact.wrench.force.z = 10.0
    executor, spin_thread = _spin(node, source)
    try:
        deadline = time.monotonic() + 1.0
        while (
            time.monotonic() < deadline
            and contact_publisher.get_subscription_count() < 1
        ):
            time.sleep(0.01)
        contact_publisher.publish(contact)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            with node._lock:
                received_at = node._contact_received_at
            if received_at is not None:
                break
            time.sleep(0.01)
        assert received_at is not None
        assert node._goal(goal.request) == GoalResponse.ACCEPT
        _run_goal_with_timer(node, goal)
        time.sleep(0.2)
        assert not goal.finished.is_set()
        goal.is_cancel_requested = True
        assert goal.finished.wait(2.0)
        assert goal.outcome == "canceled"
        assert not any(feedback.stalled for feedback in goal.feedback)
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        source.destroy_node()


def test_keepalive_publishes_without_holding_lock(ros_context) -> None:
    # Regression: the 20 Hz keepalive timer must publish OUTSIDE self._lock.
    # Publishing while holding the lock lock-inverts with rclpy internals and
    # hangs every lock-waiting callback (goal_callback included), freezing the
    # action server (all executor threads parked in futex_wait, no goals
    # accepted). This asserts the lock is free at the moment of publish.
    node = GripperFacade()
    _clear_safety(node)

    lock_free_at_publish = []

    class _SpyPublisher:
        def __init__(self, lock) -> None:
            self._lock = lock

        def publish(self, _message) -> None:
            acquired = self._lock.acquire(blocking=False)
            lock_free_at_publish.append(acquired)
            if acquired:
                self._lock.release()

    node._commands = _SpyPublisher(node._lock)

    command = JointState()
    command.name = ["drive_joint"]
    command.position = [0.85]
    command.effort = [50.0]
    with node._lock:
        node._keepalive_command = command
        node._active_goal_handle = None
        node._active_goal_reserved = False
        node._stopped = False

    try:
        node._keepalive_tick()
        assert lock_free_at_publish == [True]  # published, and the lock was free
    finally:
        node.destroy_node()


def test_settled_grasp_keeps_clamping_after_success(ros_context) -> None:
    # The GripperCommand action returns once the close settles, but the physical
    # grip must persist -- the simulator's command mux drops any source it has
    # not heard from within 0.5 s and replaces it with a zero-effort measured
    # hold (the object slides out at ~5 N).  So after a settled/stalled success
    # the facade must keep streaming the full-effort clamp, not go silent.
    node = GripperFacade()
    _clear_safety(node)
    node.set_parameters([Parameter("stall_dwell_s", Parameter.Type.DOUBLE, 0.15)])
    source = Node("gripper_executor_keepalive_source")
    state_publisher = source.create_publisher(JointState, "/isaac_joint_states", 20)
    command_messages = []
    source.create_subscription(
        JointState,
        "/sim/controller/gripper_commands",
        command_messages.append,
        20,
    )
    goal = _GoalHandle(target=1.0)  # max_effort defaults to 1.0
    _run_goal_with_timer(node, goal)

    def publish_state() -> None:
        message = JointState()
        message.name = ["drive_joint"]
        message.position = [0.4]  # parked short -> position stall -> settled
        state_publisher.publish(message)

    state_timer = source.create_timer(0.02, publish_state)
    executor, spin_thread = _spin(node, source)
    try:
        assert goal.finished.wait(3.0)
        assert goal.outcome == "succeeded"

        def clamp_count() -> int:
            return sum(
                1
                for message in list(command_messages)
                if message.effort == pytest.approx([1.0])
            )

        count_at_success = clamp_count()
        time.sleep(0.4)  # many multiples of the 0.05 s keepalive period
        # The clamp keeps being published after the action already returned.
        assert clamp_count() >= count_at_success + 3
        clamp = next(
            message
            for message in reversed(command_messages)
            if message.effort == pytest.approx([1.0])
        )
        assert clamp.position == pytest.approx([1.0])  # commanded target, full effort
    finally:
        state_timer.cancel()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        source.destroy_node()


def test_sustained_contact_does_not_succeed_while_drive_still_advancing(
    ros_context,
) -> None:
    # Hardware parity (#23): the real xArm gripper action returns only once the
    # drive stalls, reaches its target, or hits its effort limit -- never on
    # contact alone.  A fresh, sustained contact (well past stall_dwell_s) must
    # NOT trigger success while the measured position is still advancing toward
    # the target (a validation round saw the facade return success at 0.162 rad
    # while the real drive kept closing to 0.569 rad over the next ~5 s).  Once
    # the drive stops advancing, the SAME contact is allowed to confirm the
    # stall that the position path would already report.
    node = GripperFacade()
    _clear_safety(node)
    stall_dwell = 0.15
    node.set_parameters(
        [Parameter("stall_dwell_s", Parameter.Type.DOUBLE, stall_dwell)]
    )
    source = Node("gripper_executor_contact_dwell_source")
    state_publisher = source.create_publisher(JointState, "/isaac_joint_states", 20)
    contact_publisher = source.create_publisher(
        WrenchStamped, "/sim/parity/finger_contact", 20
    )
    goal = _GoalHandle(target=1.0)
    _run_goal_with_timer(node, goal)

    # The measured position keeps improving toward the target for many times
    # the stall dwell while contact stays sustained above threshold -- this is
    # the "still closing" / compliant-contact case the fix must not latch on.
    progress = {"p": 0.0, "advancing": True}

    def publish_state() -> None:
        if progress["advancing"]:
            progress["p"] = min(0.9, progress["p"] + 0.01)
        message = JointState()
        message.name = ["drive_joint"]
        message.position = [progress["p"]]
        state_publisher.publish(message)

    def publish_contact() -> None:
        contact = WrenchStamped()
        contact.wrench.force.z = 10.0  # well above the 1 N threshold
        contact_publisher.publish(contact)

    state_timer = source.create_timer(0.02, publish_state)
    contact_timer = source.create_timer(0.02, publish_contact)
    executor, spin_thread = _spin(node, source)
    try:
        # Sustained contact alone, several dwell periods long, must NOT succeed
        # while the drive is still advancing (progress keeps climbing toward
        # 0.9 over ~1.8 s -- far past the 0.15 s dwell).
        assert not goal.finished.wait(4 * stall_dwell)
        assert not goal.finished.is_set()
        # Now let the drive plateau (stop advancing) with contact still held:
        # the position-stall path fires, and the gated contact_stalled can only
        # ever confirm that same stall, never precede it.
        progress["advancing"] = False
        assert goal.finished.wait(3.0)
        assert goal.outcome == "succeeded"
        assert any(feedback.stalled for feedback in goal.feedback)
    finally:
        state_timer.cancel()
        contact_timer.cancel()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        source.destroy_node()


class _RecordingGoalHandle(_GoalHandle):
    """A _GoalHandle that timestamps each feedback publish against a shared,
    externally-driven simulated-clock reading, so the test can tell whether a
    stall/success was reported before or after a given amount of SIM progress
    -- independent of how much WALL time has actually elapsed."""

    def __init__(self, target: float, sim_time_ref: dict, sim_time_lock: threading.Lock) -> None:
        super().__init__(target)
        self._sim_time_ref = sim_time_ref
        self._sim_time_lock = sim_time_lock
        self.feedback_log: list[tuple[float, object]] = []

    def publish_feedback(self, feedback) -> None:
        with self._sim_time_lock:
            sim_t = self._sim_time_ref["sim_t"]
        self.feedback_log.append((sim_t, feedback))
        super().publish_feedback(feedback)


def test_position_stall_dwell_uses_sim_clock_not_wall_clock_at_low_rtf(
    ros_context,
) -> None:
    # Task #27: bench round agv saw the close goal report stalled=1 at 4% of
    # stroke with no contact, at bench RTF ~0.27 -- the arm then lifted air.
    # Root cause: the stall dwell (stall_dwell_s=0.3) was measured with
    # time.monotonic() (WALL seconds) even though the node runs with
    # use_sim_time=True. At RTF 0.27, 0.3s of WALL time is only ~0.08s of SIM
    # time -- far less than the ~0.2s SIM actuation latency before the drive
    # even starts moving -- so the dwell fires before the drive has had any
    # chance to make progress.
    #
    # This test drives a synthetic /clock feed at RTF ~0.27 (sim advances
    # ~0.27s of sim time per second of wall time) alongside /isaac_joint_states
    # samples where the drive stays parked at 0.0 for the first 0.2s of SIM
    # time, then genuinely advances for 0.15s of SIM time before genuinely
    # plateauing (a real stall). It asserts the facade does not report
    # stalled/succeeded before the drive could have made SIM-time progress,
    # but does report the genuine stall once 0.3s of SIM time has passed with
    # no further progress.
    from rosgraph_msgs.msg import Clock

    node = GripperFacade()
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    _clear_safety(node)
    source = Node("gripper_executor_sim_clock_source")
    clock_publisher = source.create_publisher(Clock, "/clock", 1)
    joint_publisher = source.create_publisher(JointState, "/isaac_joint_states", 20)
    safety_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    safety_publisher = source.create_publisher(
        Bool, "/sim/hardware/safety_stop", safety_qos
    )

    sim_time_lock = threading.Lock()
    sim_time = {"sim_t": 0.0}
    goal = _RecordingGoalHandle(1.0, sim_time, sim_time_lock)
    # NOT _run_goal_with_timer: that helper drives _execute() from a
    # node.create_timer() callback with no explicit callback_group, which
    # defaults to the node's DEFAULT MutuallyExclusiveCallbackGroup -- the
    # SAME group rclpy's internal TimeSource creates its own /clock
    # subscription in. _execute()'s while-loop then occupies that group for
    # the whole goal, starving the /clock callback and freezing sim time
    # (reproduced in isolation: node.get_clock().now() stalls for the entire
    # goal, then jumps once it finishes). The real ActionServer does not have
    # this problem -- it runs execute_callback in its own explicit
    # ReentrantCallbackGroup, disjoint from the default group. A plain
    # background thread reproduces that disjointness for this test.
    threading.Thread(target=lambda: node._execute(goal), daemon=True).start()

    wall_dt = 0.05
    sim_dt = wall_dt * 0.27  # matches the bench's ~0.27 RTF (round agv)

    def tick() -> None:
        # Wall-clock safety heartbeat (safety_timeout_s defaults to 1.0s WALL,
        # independent of the sim-clock bug under test): keep it fed on every
        # tick, faster than 1.0s, or the goal aborts on a safety timeout
        # before the stall-dwell behavior under test ever gets exercised.
        clear = Bool()
        clear.data = False
        safety_publisher.publish(clear)
        with sim_time_lock:
            sim_time["sim_t"] += sim_dt
            t = sim_time["sim_t"]
        clock_message = Clock()
        clock_message.clock.sec = int(t)
        clock_message.clock.nanosec = int((t - int(t)) * 1.0e9)
        clock_publisher.publish(clock_message)
        if t < 0.2:
            position = 0.0  # parked: actuation latency, drive hasn't moved yet
        elif t < 0.35:
            position = (t - 0.2) / 0.15 * 0.05  # genuine progress
        else:
            position = 0.05  # genuine plateau -> genuine stall after 0.3s SIM
        message = JointState()
        message.name = ["drive_joint"]
        message.position = [position]
        joint_publisher.publish(message)

    clock_timer = source.create_timer(wall_dt, tick)
    executor, spin_thread = _spin(node, source)
    try:
        # At wall=1.4s, sim_t ~= 0.38s: the drive has genuinely started moving
        # (past the 0.2s SIM latency) but has not yet been stalled for a
        # genuine 0.3s of SIM time (plateau starts at sim_t=0.35, so the
        # genuine stall cannot fire before sim_t=0.65, ~2.4s wall). On main,
        # the WALL-clock dwell fires within ~0.3-0.5s wall -- long before this
        # check -- so this assertion fails on main.
        time.sleep(1.4)
        assert not goal.finished.is_set(), (
            "goal finished before the drive could have genuinely stalled in "
            "SIM time -- the stall dwell is still running on WALL time"
        )
        assert not any(feedback.stalled for _, feedback in goal.feedback_log), (
            "position_stalled fired before 0.3s of SIM no-progress time had "
            "elapsed"
        )
        assert goal.finished.wait(4.0)
        assert goal.outcome == "succeeded"
        assert any(feedback.stalled for _, feedback in goal.feedback_log)
        assert not any(feedback.reached_goal for _, feedback in goal.feedback_log)
    finally:
        clock_timer.cancel()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        source.destroy_node()


def test_finger_parked_short_of_target_stalls_without_contact(ros_context) -> None:
    # A close that stops advancing toward its target while still short of it --
    # with no contact telemetry at all -- is a physical stall and must report a
    # successful (stalled) grasp, as a real gripper driver does.  This is the
    # contact-free path that keeps grasps working in profiles that run the
    # backend without contact reporting (sensor-rich).
    node = GripperFacade()
    _clear_safety(node)
    node.set_parameters([Parameter("stall_dwell_s", Parameter.Type.DOUBLE, 0.15)])
    source = Node("gripper_executor_position_stall_source")
    publisher = source.create_publisher(JointState, "/isaac_joint_states", 20)
    goal = _GoalHandle(target=1.0)
    _run_goal_with_timer(node, goal)

    def publish_state() -> None:
        message = JointState()
        message.name = ["drive_joint"]
        message.position = [0.4]  # parked well short of the 1.0 target
        publisher.publish(message)

    state_timer = source.create_timer(0.02, publish_state)
    executor, spin_thread = _spin(node, source)
    try:
        assert goal.finished.wait(3.0)
        assert goal.outcome == "succeeded"
        assert any(feedback.stalled for feedback in goal.feedback)
        assert not any(feedback.reached_goal for feedback in goal.feedback)
    finally:
        state_timer.cancel()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        source.destroy_node()
