from __future__ import annotations

import sys

import pytest
import tinker_sim_bridge.controller_reconciler as reconciler

from tinker_sim_bridge.controller_reconciler import (
    ControllerState,
    ReconciliationError,
    reconcile_controller,
    set_remote_parameter,
)


class FakeControllerManager:
    def __init__(self, state: str | None = None):
        self.state = state
        self.load_results: list[bool] = [True]
        self.configure_results: list[bool] = [True]
        self.activate_results: list[bool] = [True]
        self.load_timeout = False
        self.calls: list[str] = []

    def list_controllers(self):
        self.calls.append("list")
        return {} if self.state is None else {
            "joint_state_broadcaster": ControllerState(
                "joint_state_broadcaster", self.state
            )
        }

    def _result(self, operation: str, results: list[bool]) -> bool:
        self.calls.append(operation)
        result = results[0]
        if len(results) > 1:
            results.pop(0)
        return result

    def load_controller(self, name: str) -> bool:
        if self.load_timeout:
            self.calls.append("load")
            self.state = "unconfigured"
            raise TimeoutError("simulated load timeout")
        result = self._result("load", self.load_results)
        if result:
            self.state = "unconfigured"
        return result

    def configure_controller(self, name: str) -> bool:
        result = self._result("configure", self.configure_results)
        if result:
            self.state = "inactive"
        return result

    def activate_controller(self, name: str) -> bool:
        result = self._result("activate", self.activate_results)
        if result:
            self.state = "active"
        return result


def test_loaded_controller_is_reconciled_to_active() -> None:
    manager = FakeControllerManager("unconfigured")

    result = reconcile_controller(manager, "joint_state_broadcaster")

    assert result.state == "active"
    assert manager.calls == ["list", "configure", "list", "activate", "list"]


def test_load_timeout_is_recovered_when_manager_reports_loaded() -> None:
    manager = FakeControllerManager()
    manager.load_timeout = True

    result = reconcile_controller(manager, "joint_state_broadcaster")

    assert result.state == "active"
    assert manager.calls == ["list", "load", "list", "configure", "list", "activate", "list"]


def test_active_controller_is_idempotent() -> None:
    manager = FakeControllerManager("active")

    result = reconcile_controller(manager, "joint_state_broadcaster")

    assert result.state == "active"
    assert manager.calls == ["list"]


def test_actual_activation_failure_is_strict() -> None:
    manager = FakeControllerManager("inactive")
    manager.activate_results = [False]

    with pytest.raises(ReconciliationError, match="activate failed"):
        reconcile_controller(manager, "joint_state_broadcaster")


def test_loaded_but_unsupported_state_is_strict() -> None:
    manager = FakeControllerManager("finalized")

    with pytest.raises(ReconciliationError, match="unsupported state"):
        reconcile_controller(manager, "joint_state_broadcaster")


def test_launches_use_reconciler_and_keep_return_code_gate() -> None:
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    for name in ("manipulation.launch.py", "whole_robot.launch.py"):
        launch = (
            root / "ros2_ws/src/tinker_sim_bridge/launch" / name
        ).read_text(encoding="utf-8")
        assert 'package="tinker_sim_bridge"' in launch
        assert 'executable="controller_reconciler"' in launch
        assert 'executable="spawner"' not in launch
        assert '"--controller-manager", "/controller_manager"' in launch
        assert "event.returncode == 0" in launch
        assert "OnProcessExit" in launch


def test_actual_load_failure_is_strict() -> None:
    manager = FakeControllerManager()
    manager.load_results = [False]

    with pytest.raises(ReconciliationError, match="was not loaded"):
        reconcile_controller(manager, "joint_state_broadcaster")


def test_main_removes_ros_args_but_keeps_application_args(monkeypatch) -> None:
    captured = {}
    monkeypatch.setattr(
        reconciler,
        "_remove_ros_args",
        lambda argv: argv[: argv.index("--ros-args")],
    )
    monkeypatch.setattr(
        reconciler,
        "_run",
        lambda args: captured.update(vars(args)) or 0,
    )

    monkeypatch.setattr(
        sys,
        "argv",
        [
            "controller_reconciler",
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--ros-args",
            "-r",
            "__node:=qualification_reconciler",
        ],
    )
    result = reconciler.main()

    assert result == 0
    assert captured == {
        "controllers": ["joint_state_broadcaster"],
        "controller_manager": "/controller_manager",
        "service_timeout": 5.0,
        "attempts": 3,
        "ready_node": None,
        "ready_parameter": None,
        "ready_value": True,
        "ready_timeout": 15.0,
    }


def test_main_does_not_mask_application_argument_typos(monkeypatch) -> None:
    monkeypatch.setattr(reconciler, "_remove_ros_args", lambda argv: argv)
    monkeypatch.setattr(reconciler, "_run", lambda args: 0)

    with pytest.raises(SystemExit):
        reconciler.main(["joint_state_broadcaster", "--not-an-app-option"])


class _ParameterFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _ParameterClient:
    def __init__(self, discovery_results, response):
        self.discovery_results = list(discovery_results)
        self.response = response
        self.requests = []

    def wait_for_service(self, timeout_sec):
        if len(self.discovery_results) > 1:
            return self.discovery_results.pop(0)
        return self.discovery_results[0]

    def call_async(self, request):
        self.requests.append(request)
        return _ParameterFuture(self.response)


def test_remote_parameter_retries_discovery_then_proves_success() -> None:
    result = type("Result", (), {"successful": True, "reason": ""})()
    client = _ParameterClient(
        [False, True], type("Response", (), {"results": [result]})()
    )

    created = {}
    set_remote_parameter(
        object(),
        "tinker_sim_safety_supervisor",
        "controller_management_ready",
        True,
        timeout=1.0,
        client_factory=lambda node, name: created.__setitem__("name", name) or client,
        parameter_factory=lambda name, value: (name, value),
        request_factory=lambda parameters: parameters,
    )

    assert created["name"] == "/tinker_sim_safety_supervisor"
    assert client.requests == [[("controller_management_ready", True)]]


def test_remote_parameter_fails_closed_after_bounded_discovery_timeout() -> None:
    client = _ParameterClient([False], [])

    with pytest.raises(ReconciliationError, match="within 0.0s"):
        set_remote_parameter(
            object(),
            "/tinker_sim_safety_supervisor",
            "controller_management_ready",
            True,
            timeout=0.001,
            client_factory=lambda node, name: client,
            parameter_factory=lambda name, value: (name, value),
            request_factory=lambda parameters: parameters,
        )
