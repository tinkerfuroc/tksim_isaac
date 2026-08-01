from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest


pytest.importorskip(
    "rclpy",
    reason="scenario runner tests require the Humble ROS Python runtime",
)
pytest.importorskip(
    "simulation_interfaces",
    reason="scenario runner tests require Humble-generated simulation interfaces",
)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge import scenario_runner


class _StrictLogger:
    def __init__(self, warnings: list[str]) -> None:
        self._warnings = warnings

    def warning(self, message: str) -> None:
        self._warnings.append(message)


class _Runner:
    _initial_reset = scenario_runner.ScenarioRunner._initial_reset

    def __init__(self, outcomes, attempts=3):
        self.outcomes = iter(outcomes)
        self._reset = object()
        self.reset_attempts = attempts
        self.reset_retry_delay_s = 0
        self.calls = 0
        self.warnings = []

    def call(self, _client, _request):
        self.calls += 1
        outcome = next(self.outcomes)
        if isinstance(outcome, BaseException):
            raise outcome
        return outcome

    def get_logger(self):
        return _StrictLogger(self.warnings)


def test_initial_reset_retries_bounded_service_startup_failures() -> None:
    runner = _Runner(
        [
            scenario_runner._RetryableServiceError("unavailable"),
            scenario_runner._RetryableServiceError("timed out"),
            "reset-ok",
        ]
    )

    with patch.object(scenario_runner.time, "sleep") as sleep:
        assert runner._initial_reset(object()) == "reset-ok"

    assert runner.calls == 3
    assert runner.warnings == [
        "initial reset attempt 1/3 did not complete: unavailable; retrying",
        "initial reset attempt 2/3 did not complete: timed out; retrying",
    ]
    sleep.assert_not_called()


def test_initial_reset_does_not_retry_strict_server_rejection() -> None:
    runner = _Runner([RuntimeError("rejected")])

    with pytest.raises(RuntimeError, match="rejected"):
        runner._initial_reset(object())

    assert runner.calls == 1


def test_initial_reset_exhaustion_is_bounded_and_preserves_failure_context() -> None:
    runner = _Runner(
        [scenario_runner._RetryableServiceError("not ready")] * 2,
        attempts=2,
    )

    with pytest.raises(RuntimeError, match="initial reset failed after 2 attempts"):
        runner._initial_reset(object())

    assert runner.calls == 2


def test_spawn_failure_is_not_retried() -> None:
    runner = object.__new__(scenario_runner.ScenarioRunner)
    runner._spawn = object()
    calls = 0

    def call(_client, _request):
        nonlocal calls
        calls += 1
        raise scenario_runner._RetryableServiceError("spawn service timed out")

    runner.call = call
    operation = SimpleNamespace(
        kind="spawn_entity",
        payload={
            "name": "/World/Scenario/object",
            "uri": "object.usda",
            "entity_namespace": "Scenario",
            "frame_id": "world",
            "xyz": (0.0, 0.0, 0.0),
            "quaternion_xyzw": (0.0, 0.0, 0.0, 1.0),
            "prim_path": "/World/Scenario/object",
            "logical_id": "object",
        },
    )

    with pytest.raises(scenario_runner._RetryableServiceError):
        runner.execute([operation])

    assert calls == 1


def test_planning_scene_scenario_compiles_without_extra_spawn_operations() -> None:
    """A schema-2 scenario with an optional planning_scene compiles cleanly.

    The fixture PlanningScene is applied by the dedicated
    ``fixture_planning_scene`` node, so the standard scenario runner must not
    synthesize additional spawn operations from ``planning_scene`` and must
    carry the scenario id/seed into the final PLAYING boundary.
    """
    from tinker_sim_core.orchestration import standard_operations
    from tinker_sim_core.scenario import load_named_scenario

    root = Path(__file__).resolve().parents[1]
    scenario = load_named_scenario(
        root, "qualification-moveit-plan-blocked"
    )
    assert scenario.planning_scene is not None
    operations = standard_operations(root, scenario, seed=7)
    kinds = [operation.kind for operation in operations]
    assert kinds == [
        "reset_spawned",
        "set_simulation_state",
        "set_simulation_state",
    ]
    assert all("spawn_entity" not in kind for kind in kinds)
    final_state = operations[-1].payload
    assert final_state["state"] == 1
    assert final_state["boundary"] == "PHYSICS_READY"
    assert final_state["scenario"] == "qualification-moveit-plan-blocked"
    assert final_state["seed"] == 7
