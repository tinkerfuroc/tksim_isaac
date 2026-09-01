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
