"""The pan_tilt facade's hold/keepalive republish must be RTF-invariant.

Task #41: ``PanTiltFacade._hold_target`` republishes the held pan/tilt target
every 0.2 s, but the timer had no explicit ``clock=`` argument, so it ran on
the node's default clock -- ROS_TIME under the bridge's ``use_sim_time=True``
launch, i.e. paced in SIM seconds. The consumer (``CommandGateway``'s
pan_tilt ``CommandSource``, 0.5 s timeout) judges staleness against
``time.monotonic()`` -- WALL seconds. At RTF < 0.4 a 0.2 sim-s republish
costs more than 0.5 wall-s to land, so every tick arrived after the
deadline: continuous stale_hold/stale_hold_cleared churn (2835 pairs in one
bench round), and a real in-progress head sweep risks being clamped to a
stale mid-sweep measured position. The fix mirrors ``command_gateway.py``'s
own 150 Hz timer and ``gripper_facade.py``'s 20 Hz keepalive: pin the hold
timer to ``Clock(clock_type=ClockType.STEADY_TIME)`` so its cadence tracks
wall time regardless of RTF.
"""

from __future__ import annotations

import threading
import time
from unittest import mock

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs")
pytest.importorskip("tinker_vision_msgs_26")

from rclpy.clock import ClockType  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from tinker_sim_bridge.pan_tilt_facade import PanTiltFacade  # noqa: E402


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


def _spin(node: Node, source: Node):
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(source)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    return executor, spin_thread


def test_hold_timer_created_with_steady_clock(ros_context) -> None:
    # Recording double on Node.create_timer: capture the clock= kwarg the
    # _hold_target timer is actually constructed with, rather than trusting
    # a string match against the source.
    recorded = []
    original_create_timer = Node.create_timer

    def spy(self, period, callback, *args, **kwargs):
        timer = original_create_timer(self, period, callback, *args, **kwargs)
        if getattr(callback, "__name__", None) == "_hold_target":
            recorded.append((period, kwargs.get("clock")))
        return timer

    node = None
    try:
        with mock.patch.object(Node, "create_timer", spy):
            node = PanTiltFacade()
        assert len(recorded) == 1, "expected exactly one _hold_target timer"
        period, clock = recorded[0]
        assert period == 0.2
        assert clock is not None, "_hold_target timer must pass an explicit clock="
        assert clock.clock_type == ClockType.STEADY_TIME
    finally:
        if node is not None:
            node.destroy_node()


def test_hold_republish_cadence_is_wall_clock_bounded_at_low_rtf(ros_context) -> None:
    # Drive a synthetic /clock feed at RTF ~0.25 (matches the bench round that
    # found this: RTF 0.17-0.6, well under the 0.4 breakeven for a 0.2 sim-s /
    # 0.5 wall-s deadline) and record the WALL-clock arrival time of each
    # /sim/controller/pan_tilt_commands publish. Pre-fix, the hold timer ticks
    # every 0.2 SIM-s -> 0.2 / 0.25 = 0.8 WALL-s between publishes, blowing
    # the mux's 0.5 s watchdog every cycle. Post-fix, the timer is
    # STEADY_TIME and keeps its ~0.2 WALL-s cadence regardless of RTF.
    from rosgraph_msgs.msg import Clock

    node = PanTiltFacade()
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    source = Node("pan_tilt_facade_sim_clock_source")
    clock_publisher = source.create_publisher(Clock, "/clock", 1)

    received_at: list[float] = []
    source.create_subscription(
        JointState,
        "/sim/controller/pan_tilt_commands",
        lambda _message: received_at.append(time.monotonic()),
        20,
    )

    sim_time = {"t": 0.0}
    wall_dt = 0.05
    sim_dt = wall_dt * 0.25  # RTF ~0.25

    def tick() -> None:
        sim_time["t"] += sim_dt
        t = sim_time["t"]
        message = Clock()
        message.clock.sec = int(t)
        message.clock.nanosec = int((t - int(t)) * 1.0e9)
        clock_publisher.publish(message)

    clock_timer = source.create_timer(wall_dt, tick)
    executor, spin_thread = _spin(node, source)
    try:
        time.sleep(3.0)
        # 3 wall-s at a STEADY_TIME 0.2 s period is ~15 ticks; require enough
        # samples that a single scheduling jitter cannot pass the assertion.
        assert len(received_at) >= 8, (
            f"only {len(received_at)} republishes in 3 wall-s -- cadence is "
            "not wall-clock paced"
        )
        gaps = [b - a for a, b in zip(received_at, received_at[1:])]
        # Pre-fix this bound fails: 0.2 sim-s / 0.25 RTF = 0.8 wall-s per gap.
        assert max(gaps) <= 0.35, (
            f"max wall gap {max(gaps):.3f}s exceeds the 0.5s CommandSource "
            "timeout margin -- keepalive is still RTF-dependent"
        )
    finally:
        clock_timer.cancel()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        source.destroy_node()
