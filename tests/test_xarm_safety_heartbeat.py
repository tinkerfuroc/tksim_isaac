from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
XARM = ROOT / "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/xarm_facade.py"


def _source() -> str:
    return XARM.read_text(encoding="utf-8")


def _method(tree: ast.AST, name: str) -> ast.FunctionDef:
    return next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def test_xarm_safety_heartbeat_uses_steady_clock_and_beats_supervisor_deadline() -> None:
    source = _source()
    assert "from rclpy.clock import Clock, ClockType" in source
    assert "SAFETY_HEARTBEAT_PERIOD_S = 0.25" in source
    assert "clock=Clock(clock_type=ClockType.STEADY_TIME)" in source
    supervisor = (
        ROOT
        / "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/safety_supervisor.py"
    ).read_text(encoding="utf-8")
    assert 'declare_parameter("required_source_deadline_s", 1.0)' in supervisor

    tree = ast.parse(source)
    period = next(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name)
            and target.id == "SAFETY_HEARTBEAT_PERIOD_S"
            for target in node.targets
        )
    )
    assert isinstance(period, ast.Constant)
    assert 0.0 < period.value <= 0.5
    init = _method(tree, "__init__")
    timer = next(
        node
        for node in ast.walk(init)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_timer"
    )
    assert isinstance(timer.args[1], ast.Attribute)
    assert timer.args[1].attr == "_publish_safety"


def test_xarm_state_changes_still_publish_immediately() -> None:
    tree = ast.parse(_source())
    for name in ("__init__", "_motion_enable", "_set_state"):
        method = _method(tree, name)
        assert any(
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "_publish_safety"
            for node in ast.walk(method)
        ), name
