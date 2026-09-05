"""The command gateway re-sends an unchanged snapshot only at keepalive cadence.

Every packet the simulator receives costs it main-loop time (GIL hand-off to
its executor thread plus a PhysX target write).  Resending four identical
packets at the full 150 Hz tick was measured on 2026-08-21 to cut the
simulator's real-time factor from ~0.8 to ~0.23, so the tick now publishes a
full snapshot only when it differs from the last one sent, or when the
keepalive period (well inside the simulator's 0.5 s command-stream watchdog)
has elapsed.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs")

from tinker_sim_bridge.command_gateway import CommandGateway  # noqa: E402
from tinker_sim_core.command_mux import (  # noqa: E402
    CommandSource,
    JointCommand,
    JointCommandMux,
)


class _Recorder:
    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, message) -> None:
        self.messages.append(message)


def _gateway() -> CommandGateway:
    gateway = object.__new__(CommandGateway)
    gateway._mux = JointCommandMux(
        {"arm": CommandSource(frozenset({"joint1"}), 1.0)}
    )
    gateway._rejected = {}
    gateway._safety_active = False
    gateway._mux.stop(False)
    gateway._safety_timeout_s = 1.0
    gateway._safety_last_sample_at = time.monotonic() + 3600.0
    gateway._command_epoch = 0
    gateway._snapshot_id = 0
    gateway._publisher = _Recorder()
    gateway._status = _Recorder()
    gateway.get_logger = lambda: SimpleNamespace(error=lambda *a, **k: None)
    gateway._mux.accept(
        "arm", JointCommand(("joint1",), positions=(0.25,)), time.monotonic()
    )
    return gateway


def test_unchanged_snapshot_is_not_resent_within_keepalive_period() -> None:
    gateway = _gateway()
    gateway._publish()
    packets = len(gateway._publisher.messages)
    assert packets == 1
    assert len(gateway._status.messages) == 1

    for _ in range(10):
        gateway._publish()
    assert len(gateway._publisher.messages) == packets
    assert len(gateway._status.messages) == 1
    assert gateway._snapshot_id == 1


def test_unchanged_snapshot_is_resent_after_keepalive_period() -> None:
    gateway = _gateway()
    gateway._publish()
    gateway._last_publish_at -= CommandGateway.KEEPALIVE_PERIOD_S
    gateway._publish()
    assert len(gateway._publisher.messages) == 2
    assert gateway._snapshot_id == 2


def test_changed_command_is_sent_immediately() -> None:
    gateway = _gateway()
    gateway._publish()
    gateway._last_publish_at -= CommandGateway.MIN_PUBLISH_PERIOD_S
    gateway._mux.accept(
        "arm", JointCommand(("joint1",), positions=(0.5,)), time.monotonic()
    )
    gateway._publish()
    assert len(gateway._publisher.messages) == 2
    assert list(gateway._publisher.messages[-1].position) == [0.5]


def test_changes_are_rate_limited_to_the_control_rate() -> None:
    gateway = _gateway()
    gateway._publish()
    for i in range(5):  # five changes inside one 60 Hz period
        gateway._mux.accept(
            "arm", JointCommand(("joint1",), positions=(0.3 + 0.01 * i,)), time.monotonic()
        )
        gateway._publish()
    assert len(gateway._publisher.messages) == 1
    gateway._last_publish_at -= CommandGateway.MIN_PUBLISH_PERIOD_S
    gateway._publish()
    # The latest value goes out, not the intermediate ones.
    assert len(gateway._publisher.messages) == 2
    assert abs(gateway._publisher.messages[-1].position[0] - 0.34) < 1e-9
    assert CommandGateway.MIN_PUBLISH_PERIOD_S <= 1.0 / 60.0 + 1e-12


def test_safety_stop_transition_is_sent_immediately() -> None:
    gateway = _gateway()
    gateway._publish()
    gateway._mux.stop(True)
    gateway._publish()  # inside the rate-limit window: still sent at once
    assert len(gateway._publisher.messages) == 2


def test_keepalive_is_well_inside_simulator_command_watchdog() -> None:
    # ros_gateway.COMMAND_STREAM_TIMEOUT_S is 0.5 s; keep a wide margin.
    assert CommandGateway.KEEPALIVE_PERIOD_S <= 0.1


def test_epoch_bump_forces_next_publish_even_if_commands_unchanged() -> None:
    gateway = _gateway()
    gateway._publish()
    # Simulate the bookkeeping every epoch-bump site performs.
    gateway._command_epoch += 1
    gateway._snapshot_id = 0
    gateway._last_published_commands = None
    gateway._publish()
    assert len(gateway._publisher.messages) == 2
    assert gateway._publisher.messages[-1].header.frame_id != (
        gateway._publisher.messages[0].header.frame_id
    )
