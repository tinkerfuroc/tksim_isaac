from __future__ import annotations

import io
import json
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
        self._state = object()
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


def test_set_simulation_state_retries_bounded_service_startup_failures() -> None:
    """RED (F5): /set_simulation_state is a transient startup race (the live
    cancel run died on ``standard service timed out: /set_simulation_state``) and
    must be retried with the same bounded budget as the initial reset."""
    runner = _Runner(
        [
            scenario_runner._RetryableServiceError("timed out"),
            scenario_runner._RetryableServiceError("unavailable"),
            "state-ok",
        ]
    )

    with patch.object(scenario_runner.time, "sleep") as sleep:
        assert (
            scenario_runner.ScenarioRunner._call_state(runner, object()) == "state-ok"
        )

    assert runner.calls == 3
    assert runner.warnings == [
        "set_simulation_state attempt 1/3 did not complete: timed out; retrying",
        "set_simulation_state attempt 2/3 did not complete: unavailable; retrying",
    ]
    sleep.assert_not_called()


def test_set_simulation_state_exhaustion_is_bounded_and_preserves_context() -> None:
    """F5: a persistently failing set_simulation_state fails closed after the
    bounded attempts with the last retryable error preserved."""
    runner = _Runner(
        [scenario_runner._RetryableServiceError("not ready")] * 2,
        attempts=2,
    )

    with pytest.raises(RuntimeError, match="set_simulation_state failed after 2 attempts"):
        scenario_runner.ScenarioRunner._call_state(runner, object())

    assert runner.calls == 2


def test_set_simulation_state_does_not_retry_strict_server_rejection() -> None:
    """F5: a genuine non-retryable server rejection is not retried."""
    runner = _Runner([RuntimeError("rejected")])

    with pytest.raises(RuntimeError, match="rejected"):
        scenario_runner.ScenarioRunner._call_state(runner, object())

    assert runner.calls == 1


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
        root, "qualification-moveit-plan-pose"
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
    # The final operation carries the immutable scenario mapping plus the
    # planning-scene and integrated mappings (Task 1), alongside seed.
    assert final_state["scenario"]["id"] == "qualification-moveit-plan-pose"
    assert final_state["scenario"]["seed"] == 7
    # The identity-free declaration carries the immutable spec (including the
    # planning-scene and integrated mappings) but never the id/seed identity keys.
    assert "id" not in final_state["scenario"]["declaration"]
    assert "seed" not in final_state["scenario"]["declaration"]
    assert final_state["scenario"]["declaration"]["integrated"]["stage"] == "C"
    assert final_state["seed"] == 7
    assert final_state["planning_scene"]["revision"] == "2026-08-08-moveit-qualification-pose"
    assert final_state["integrated"]["stage"] == "C"


class _FakeResultsNode:
    """A minimal stand-in for the real ScenarioRunner graph node.

    ``main()`` builds the legacy/canonical report from ``execute(operations)``;
    the fake node returns canned results and records construction arguments.
    """

    def __init__(self, *args, **kwargs) -> None:
        del args, kwargs

    def execute(self, operations):
        del operations
        return [
            {"operation": "reset_spawned", "accepted": True},
            {
                "operation": "set_simulation_state",
                "accepted": True,
                "state": 1,
                "boundary": "PHYSICS_READY",
            },
        ]

    def destroy_node(self) -> None:
        return None


def _patch_rclpy_and_runner(monkeypatch):
    import rclpy

    monkeypatch.setattr(scenario_runner.rclpy, "init", lambda args=None: None)
    monkeypatch.setattr(scenario_runner.rclpy, "ok", lambda: False)
    monkeypatch.setattr(
        scenario_runner.rclpy.utilities, "remove_ros_args", lambda args: args
    )
    monkeypatch.setattr(scenario_runner, "ScenarioRunner", _FakeResultsNode)


def _legacy_argv(tmp_path, *, report: str | None) -> list[str]:
    argv = [
        "tinker_sim_scenario_runner",
        "--root", str(ROOT),
        "--scenario", "qualification-moveit-plan-joint",
        "--seed", "7",
        "--timeout", "20",
    ]
    if report is not None:
        argv += ["--report", str(tmp_path / "scenario-runner.json")]
    return argv


def test_legacy_path_without_report_prints_previous_shape(capsys, monkeypatch, tmp_path) -> None:
    """The non-overlay branch prints the exact previous report shape and does not
    require a --report file."""
    _patch_rclpy_and_runner(monkeypatch)
    scenario_runner.main(_legacy_argv(tmp_path, report=None))
    captured = capsys.readouterr()
    report = json.loads(captured.out)
    assert report["schema_version"] == 1
    assert report["scenario"] == "qualification-moveit-plan-joint"
    assert report["seed"] == 7
    assert report["control_api"] == "simulation_interfaces"
    assert report["custom_control_services"] is False
    assert report["operations"][-1]["boundary"] == "PHYSICS_READY"
    assert not (tmp_path / "scenario-runner.json").exists()


def test_legacy_path_with_report_writes_atomic_previous_shape(capsys, monkeypatch, tmp_path) -> None:
    """The non-overlay branch honors --report with the exact previous report
    shape, written atomically to the requested path."""
    _patch_rclpy_and_runner(monkeypatch)
    scenario_runner.main(_legacy_argv(tmp_path, report="scenario-runner.json"))
    captured = capsys.readouterr()
    printed = json.loads(captured.out)
    written = json.loads((tmp_path / "scenario-runner.json").read_text(encoding="utf-8"))
    assert written == printed
    assert written["schema_version"] == 1
    assert written["scenario"] == "qualification-moveit-plan-joint"
    assert written["seed"] == 7
    assert written["control_api"] == "simulation_interfaces"
    assert written["custom_control_services"] is False
    assert written["operations"][-1]["boundary"] == "PHYSICS_READY"
    # No canonical compact report keys leak into the legacy payload.
    assert "report_revision" not in written
    assert "integrated" not in written
