from __future__ import annotations

import sys
import threading

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
    monkeypatch.delenv(reconciler.TEARDOWN_TIMEOUT_ENV_VAR, raising=False)
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
        "teardown_timeout_s": reconciler.DEFAULT_TEARDOWN_TIMEOUT_S,
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


class _RecordingLogger:
    def __init__(self):
        self.infos: list[str] = []
        self.errors: list[str] = []

    def info(self, message: str) -> None:
        self.infos.append(message)

    def error(self, message: str) -> None:
        self.errors.append(message)


def test_bounded_teardown_returns_normally_when_teardown_is_prompt() -> None:
    calls: list[int] = []
    logger = _RecordingLogger()

    def prompt_teardown() -> None:
        calls.append(1)

    completed = reconciler.bounded_teardown(
        prompt_teardown,
        1.0,
        logger=logger,
        exit_code=0,
        exit_fn=lambda code: pytest.fail(
            f"exit_fn must not be called on a prompt teardown (got code={code})"
        ),
    )

    assert completed is True
    assert calls == [1]
    assert logger.errors == []
    assert any("starting teardown" in message for message in logger.infos)
    assert any("teardown completed" in message for message in logger.infos)


def test_bounded_teardown_forces_exit_when_teardown_hangs() -> None:
    forced: list[int] = []
    started = threading.Event()
    release = threading.Event()
    logger = _RecordingLogger()

    def hanging_teardown() -> None:
        started.set()
        # Never sets `release`; blocks forever (this thread is a daemon, so
        # it is abandoned -- not joined -- once the test process exits).
        release.wait()

    completed = reconciler.bounded_teardown(
        hanging_teardown,
        0.05,
        logger=logger,
        exit_code=0,
        exit_fn=lambda code: forced.append(code),
    )

    assert started.wait(timeout=1.0), "teardown thread never started"
    assert completed is False
    assert forced == [0]
    assert len(logger.errors) == 1
    assert (
        "controller_reconciler: teardown did not complete within"
        in logger.errors[0]
    )
    assert "after success; forcing exit (rc=0)" in logger.errors[0]


def test_bounded_teardown_preserves_nonzero_exit_code_on_timeout() -> None:
    forced: list[int] = []
    release = threading.Event()
    logger = _RecordingLogger()

    reconciler.bounded_teardown(
        release.wait,
        0.05,
        logger=logger,
        exit_code=1,
        exit_fn=lambda code: forced.append(code),
    )

    assert forced == [1]
    assert "after failure; forcing exit (rc=1)" in logger.errors[0]


def test_teardown_timeout_default_reads_env_var(monkeypatch) -> None:
    monkeypatch.setenv(reconciler.TEARDOWN_TIMEOUT_ENV_VAR, "12.5")

    assert reconciler._teardown_timeout_default() == 12.5


def test_teardown_timeout_default_falls_back_when_env_var_is_invalid(
    monkeypatch,
) -> None:
    monkeypatch.setenv(reconciler.TEARDOWN_TIMEOUT_ENV_VAR, "not-a-number")

    assert (
        reconciler._teardown_timeout_default()
        == reconciler.DEFAULT_TEARDOWN_TIMEOUT_S
    )


def test_teardown_timeout_default_falls_back_when_env_var_is_unset(
    monkeypatch,
) -> None:
    monkeypatch.delenv(reconciler.TEARDOWN_TIMEOUT_ENV_VAR, raising=False)

    assert (
        reconciler._teardown_timeout_default()
        == reconciler.DEFAULT_TEARDOWN_TIMEOUT_S
    )
