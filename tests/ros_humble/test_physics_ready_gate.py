"""ROS-level regression tests for the integrated physics-ready gate."""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="requires Humble ROS Python runtime")
pytest.importorskip("std_srvs", reason="requires Humble-generated std_srvs interfaces")

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from rclpy.parameter import Parameter  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from tinker_sim_bridge.physics_ready_gate import PhysicsReadyGate  # noqa: E402


def test_gate_parses_report_before_serving_readiness(monkeypatch, tmp_path) -> None:
    if not rclpy.ok():
        rclpy.init(args=[])

    parse_calls: list[str] = []

    def mark_ready(self: PhysicsReadyGate) -> None:
        parse_calls.append("parse")
        self._report = {"scenario": {"id": "qualification-moveit-plan-joint"}}
        self._state = "PHYSICS_READY"

    monkeypatch.setattr(PhysicsReadyGate, "_parse_report", mark_ready)
    node = PhysicsReadyGate(
        parameter_overrides=[
            Parameter("report_path", value=str(tmp_path / "scenario-runner.json")),
            Parameter("physics_ready_path", value=str(tmp_path / "physics-ready.json")),
        ]
    )
    try:
        response = node._on_ready(Trigger.Request(), Trigger.Response())
        assert parse_calls == ["parse"]
        assert response.success is True
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
