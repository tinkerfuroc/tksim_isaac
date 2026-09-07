"""#33 observability: the command gateway must announce safety-gate
transitions and dropped _accept messages, without changing any command,
rejection dict entry, or publish behaviour those paths already had.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import patch

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs")

from tinker_sim_bridge.command_gateway import CommandGateway  # noqa: E402
from tinker_sim_core.command_mux import (  # noqa: E402
    CommandSource,
    JointCommandMux,
)


class _LogRecorder:
    def __init__(self) -> None:
        self.info_lines: list[str] = []
        self.error_lines: list[str] = []

    def info(self, message: str) -> None:
        self.info_lines.append(message)

    def error(self, message: str) -> None:
        self.error_lines.append(message)

    def warning(self, message: str) -> None:
        pass


def _gateway(logger: _LogRecorder, sample_at: float) -> CommandGateway:
    gateway = object.__new__(CommandGateway)
    gateway._mux = JointCommandMux(
        {"gripper": CommandSource(frozenset({"drive_joint"}), 0.5)}
    )
    gateway._rejected = {}
    gateway._rejected_log_state = {}
    gateway._safety_active = False
    gateway._safety_armed_at = None
    gateway._safety_timeout_s = 1.0
    gateway._safety_last_sample_at = sample_at
    gateway._command_epoch = 0
    gateway._snapshot_id = 0
    gateway._last_published_commands = None
    gateway.get_logger = lambda: logger
    return gateway


def test_timeout_arm_then_rejected_then_sample_clear_are_logged() -> None:
    clock = {"t": 100.0}

    def fake_monotonic() -> float:
        return clock["t"]

    with patch("tinker_sim_bridge.command_gateway.time.monotonic", fake_monotonic):
        logger = _LogRecorder()
        gateway = _gateway(logger, sample_at=clock["t"])

        # A heartbeat sample arrives while already clear: no transition, no
        # safety_gate line, but it resets the deadline reference.
        gateway._safety_stop(SimpleNamespace(data=False))
        assert gateway._safety_active is False
        assert logger.info_lines == []

        # 1.1 s pass with no further sample -> the deadline check arms the
        # gate via timeout.
        clock["t"] += 1.1
        gateway._enforce_safety_deadline()
        assert gateway._safety_active is True
        armed = [m for m in logger.info_lines if m.startswith("safety_gate armed")]
        assert len(armed) == 1
        assert armed[0] == "safety_gate armed reason=timeout gap_s=1.100"

        # A gripper command while armed is rejected and logged exactly once.
        gateway._accept("gripper", None)
        rejected = [m for m in logger.info_lines if m.startswith("command_rejected")]
        assert len(rejected) == 1
        assert rejected[0] == (
            "command_rejected source=gripper reason=blocked by safety stop count=1"
        )
        assert gateway._rejected["gripper"] == "blocked by safety stop"

        # A second drop inside the 1 s rate-limit window is counted but not
        # logged again yet.
        gateway._accept("gripper", None)
        rejected = [m for m in logger.info_lines if m.startswith("command_rejected")]
        assert len(rejected) == 1
        assert gateway._rejected_log_state["gripper"]["count"] == 1

        # A fresh sample clears the gate and reports how long it was armed.
        clock["t"] += 0.05
        gateway._safety_stop(SimpleNamespace(data=False))
        assert gateway._safety_active is False
        cleared = [m for m in logger.info_lines if m.startswith("safety_gate cleared")]
        assert len(cleared) == 1
        # Armed at t=101.1 (the timeout check above), cleared 0.05 s later:
        # gap_s reports how long the gate was held, not the sample interval.
        assert cleared[0] == "safety_gate cleared reason=sample gap_s=0.050"


def test_explicit_active_sample_logs_armed_reason_sample_without_gap() -> None:
    with patch("tinker_sim_bridge.command_gateway.time.monotonic", lambda: 5.0):
        logger = _LogRecorder()
        gateway = _gateway(logger, sample_at=5.0)

        gateway._safety_stop(SimpleNamespace(data=True))

        assert gateway._safety_active is True
        armed = [m for m in logger.info_lines if m.startswith("safety_gate armed")]
        assert armed == ["safety_gate armed reason=sample"]


def test_command_rejected_rate_limit_reopens_after_the_window() -> None:
    clock = {"t": 0.0}

    with patch("tinker_sim_bridge.command_gateway.time.monotonic", lambda: clock["t"]):
        logger = _LogRecorder()
        gateway = _gateway(logger, sample_at=clock["t"])
        gateway._safety_active = True  # armed for the whole test

        gateway._accept("gripper", None)
        clock["t"] += 1.2
        gateway._accept("gripper", None)

        rejected = [m for m in logger.info_lines if m.startswith("command_rejected")]
        assert len(rejected) == 2
        assert rejected[0].endswith("count=1")
        assert rejected[1].endswith("count=1")
