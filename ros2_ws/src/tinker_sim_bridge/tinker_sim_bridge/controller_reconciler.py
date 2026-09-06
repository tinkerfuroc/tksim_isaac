"""Bounded, idempotent controller-manager startup reconciliation."""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Protocol

# Task #25: node.destroy_node()/rclpy.shutdown() were observed to hang
# indefinitely (SIGINT-immune, no traceback) after a successful
# reconciliation during a live GPSR run, wedging the OnProcessExit launch
# chain that loads xarm7_traj_controller. bounded_teardown() below caps that
# call so a wedge there can no longer stall the whole robot.
DEFAULT_TEARDOWN_TIMEOUT_S = 5.0
TEARDOWN_TIMEOUT_ENV_VAR = "TINKER_RECONCILER_TEARDOWN_TIMEOUT_S"


class ReconciliationError(RuntimeError):
    """Raised when a requested controller cannot be proven active."""


def _parse_bool(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value!r}")


def _teardown_timeout_default() -> float:
    """Resolve the default --teardown-timeout-s from TEARDOWN_TIMEOUT_ENV_VAR.

    Falls back to DEFAULT_TEARDOWN_TIMEOUT_S when the env var is unset,
    blank, or not a valid float, so a malformed env var cannot make the
    reconciler fail to start.
    """
    raw = os.environ.get(TEARDOWN_TIMEOUT_ENV_VAR)
    if raw is None or not raw.strip():
        return DEFAULT_TEARDOWN_TIMEOUT_S
    try:
        return float(raw)
    except ValueError:
        return DEFAULT_TEARDOWN_TIMEOUT_S


def bounded_teardown(
    teardown: Callable[[], None],
    timeout_s: float,
    *,
    logger=None,
    exit_code: int = 0,
    exit_fn: Callable[[int], None] = os._exit,
) -> bool:
    """Run *teardown* in a daemon thread, bounded by *timeout_s* seconds.

    node.destroy_node()/rclpy.shutdown() have been observed (task #25) to
    hang indefinitely -- SIGINT-immune, no traceback -- under heavy DDS
    discovery load right after a successful reconciliation, which wedges
    every launch file chained on this process's exit. This keeps that call
    from stalling the process forever: if *teardown* has not finished within
    *timeout_s*, this logs one clear line, flushes stdout/stderr, and
    force-exits the process via *exit_fn* (os._exit by default, which does
    not run atexit/finally handlers or return) instead of waiting on it.

    Returns True if *teardown* completed within the bound (the normal path
    today). A timeout never returns: *exit_fn* is expected to terminate the
    process; it is only injectable so tests can observe the call instead of
    actually exiting.
    """

    def _log_info(message: str) -> None:
        if logger is not None:
            logger.info(message)
        else:
            print(message)

    def _log_error(message: str) -> None:
        if logger is not None:
            logger.error(message)
        else:
            print(message, file=sys.stderr)

    _log_info(f"controller_reconciler: starting teardown (bound {timeout_s:g}s)")
    thread = threading.Thread(target=teardown, daemon=True)
    thread.start()
    thread.join(timeout_s)
    if thread.is_alive():
        outcome = "success" if exit_code == 0 else "failure"
        _log_error(
            "controller_reconciler: teardown did not complete within "
            f"{timeout_s:g}s after {outcome}; forcing exit (rc={exit_code})"
        )
        sys.stdout.flush()
        sys.stderr.flush()
        exit_fn(exit_code)
        return False
    _log_info("controller_reconciler: teardown completed")
    return True


@dataclass(frozen=True)
class ControllerState:
    name: str
    state: str


class ControllerManagerApi(Protocol):
    def list_controllers(self) -> dict[str, ControllerState]: ...

    def load_controller(self, name: str) -> bool: ...

    def configure_controller(self, name: str) -> bool: ...

    def activate_controller(self, name: str) -> bool: ...


def _state(api: ControllerManagerApi, name: str) -> ControllerState | None:
    return api.list_controllers().get(name)


def reconcile_controller(
    api: ControllerManagerApi,
    name: str,
    *,
    attempts: int = 3,
) -> ControllerState:
    """Load, configure, and activate *name*, proving each result by listing.

    A timeout or false service result is recoverable only when a fresh list
    call proves that controller-manager completed the operation anyway.
    """
    if attempts < 1:
        raise ValueError("attempts must be positive")

    last_error = "no controller-manager state"
    for _ in range(attempts):
        try:
            current = _state(api, name)
            if current is None:
                try:
                    api.load_controller(name)
                except TimeoutError as exc:
                    last_error = f"load timed out: {exc}"
                current = _state(api, name)
                if current is None:
                    last_error = f"controller {name!r} was not loaded"
                    continue

            if current.state == "active":
                return current
            if current.state == "unconfigured":
                try:
                    configured = api.configure_controller(name)
                except TimeoutError as exc:
                    configured = False
                    last_error = f"configure timed out: {exc}"
                current = _state(api, name)
                if current is None:
                    last_error = f"controller {name!r} disappeared after configure"
                    continue
                if current.state == "active":
                    return current
                if not configured and current.state not in {"inactive", "active"}:
                    last_error = (
                        f"controller {name!r} configure failed in state "
                        f"{current.state!r}"
                    )
                    continue

            if current.state == "inactive":
                try:
                    activated = api.activate_controller(name)
                except TimeoutError as exc:
                    activated = False
                    last_error = f"activate timed out: {exc}"
                current = _state(api, name)
                if current is not None and current.state == "active":
                    return current
                if not activated:
                    last_error = f"controller {name!r} activate failed"
                elif current is None:
                    last_error = f"controller {name!r} disappeared after activate"
                else:
                    last_error = (
                        f"controller {name!r} remained in state {current.state!r}"
                    )
                continue

            last_error = f"controller {name!r} is in unsupported state {current.state!r}"
        except TimeoutError as exc:
            last_error = str(exc)

    raise ReconciliationError(last_error)


class RosControllerManagerApi:
    """Small rclpy adapter kept separate from the deterministic reconciler."""

    def __init__(self, node, manager: str, timeout: float):
        from controller_manager_msgs.srv import (
            ConfigureController,
            ListControllers,
            LoadController,
            SwitchController,
        )

        self._node = node
        self._timeout = timeout
        prefix = "/" + manager.strip("/")
        self._list = node.create_client(ListControllers, prefix + "/list_controllers")
        self._load = node.create_client(LoadController, prefix + "/load_controller")
        self._configure = node.create_client(
            ConfigureController, prefix + "/configure_controller"
        )
        self._switch = node.create_client(SwitchController, prefix + "/switch_controller")
        self._switch_type = SwitchController

    def _call(self, client, request):
        deadline = time.monotonic() + self._timeout
        if not client.wait_for_service(timeout_sec=max(0.0, deadline - time.monotonic())):
            raise TimeoutError(f"service {client.srv_name} unavailable")
        future = client.call_async(request)
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"service {client.srv_name} timed out")
            import rclpy

            rclpy.spin_once(self._node, timeout_sec=min(0.1, remaining))
        return future.result()

    def list_controllers(self) -> dict[str, ControllerState]:
        from controller_manager_msgs.srv import ListControllers

        response = self._call(self._list, ListControllers.Request())
        return {
            item.name: ControllerState(name=item.name, state=item.state)
            for item in response.controller
        }

    def load_controller(self, name: str) -> bool:
        from controller_manager_msgs.srv import LoadController

        response = self._call(self._load, LoadController.Request(name=name))
        return bool(response.ok)

    def configure_controller(self, name: str) -> bool:
        from controller_manager_msgs.srv import ConfigureController

        response = self._call(self._configure, ConfigureController.Request(name=name))
        return bool(response.ok)

    def activate_controller(self, name: str) -> bool:
        from builtin_interfaces.msg import Duration

        request = self._switch_type.Request()
        request.activate_controllers = [name]
        request.deactivate_controllers = []
        request.strictness = self._switch_type.Request.STRICT
        request.activate_asap = True
        request.timeout = Duration(
            sec=max(0, int(self._timeout)),
            nanosec=max(0, int((self._timeout % 1) * 1_000_000_000)),
        )
        response = self._call(self._switch, request)
        return bool(response.ok)


def set_remote_parameter(
    node,
    remote_node: str,
    parameter_name: str,
    value: bool,
    *,
    timeout: float,
    client_factory: Callable | None = None,
    parameter_factory: Callable | None = None,
    request_factory: Callable | None = None,
) -> None:
    """Set a remote parameter after discovery, within one bounded deadline.

    This keeps launch sequencing inside a ROS node instead of depending on a
    one-shot CLI discovery request. Every failed discovery or set operation is
    retried until the deadline, and the caller receives an error if success was
    never proven.
    """
    if timeout <= 0.0:
        raise ValueError("readiness timeout must be positive")
    if not remote_node.startswith("/"):
        remote_node = "/" + remote_node

    if client_factory is None:
        from rcl_interfaces.srv import SetParameters

        client_factory = lambda current_node, node_name: current_node.create_client(
            SetParameters, node_name.rstrip("/") + "/set_parameters"
        )
    if parameter_factory is None:
        from rclpy.parameter import Parameter

        parameter_factory = lambda name, enabled: Parameter(
            name, Parameter.Type.BOOL, enabled
        ).to_parameter_msg()
    if request_factory is None:
        from rcl_interfaces.srv import SetParameters

        request_factory = lambda parameters: SetParameters.Request(
            parameters=parameters
        )

    client = client_factory(node, remote_node)
    deadline = time.monotonic() + timeout
    last_error = f"parameter service for {remote_node} was not discovered"
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            if not client.wait_for_service(timeout_sec=min(0.25, remaining)):
                continue
            future = client.call_async(
                request_factory(
                    [parameter_factory(parameter_name, bool(value))]
                )
            )
            while not future.done():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    last_error = "parameter update timed out"
                    break
                import rclpy

                rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
            if not future.done():
                continue
            response = future.result()
            results = getattr(response, "results", response)
            if results and all(result.successful for result in results):
                return
            reason = results[0].reason if results else "empty parameter response"
            last_error = f"parameter update rejected: {reason}"
        except (RuntimeError, TimeoutError) as exc:
            last_error = str(exc)
    raise ReconciliationError(
        f"could not set {remote_node}.{parameter_name} within {timeout:.1f}s: {last_error}"
    )


def _parse_args(argv: list[str] | None = None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("controllers", nargs="+", help="controller names")
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument("--service-timeout", type=float, default=5.0)
    parser.add_argument("--attempts", type=int, default=3)
    parser.add_argument("--ready-node")
    parser.add_argument("--ready-parameter")
    parser.add_argument("--ready-value", type=_parse_bool, default=True)
    parser.add_argument("--ready-timeout", type=float, default=15.0)
    parser.add_argument(
        "--teardown-timeout-s",
        type=float,
        default=_teardown_timeout_default(),
        help=(
            "Bound, in seconds, on node.destroy_node()/rclpy.shutdown() "
            "after reconciliation finishes; the process force-exits if this "
            f"is exceeded (default {DEFAULT_TEARDOWN_TIMEOUT_S:g}, "
            f"overridable via {TEARDOWN_TIMEOUT_ENV_VAR})."
        ),
    )

    raw_argv = list(sys.argv) if argv is None else ["controller_reconciler", *argv]
    application_argv = _remove_ros_args(raw_argv)[1:]
    return parser.parse_args(application_argv)


def _remove_ros_args(argv: list[str]) -> list[str]:
    """Remove ROS arguments using Humble's utility at runtime.

    Keeping this import behind a function lets generic parser tests inject a
    deterministic remover without loading Humble's native rclpy extension.
    """
    from rclpy.utilities import remove_ros_args

    return remove_ros_args(argv)


def _run(args) -> int:
    import rclpy

    rclpy.init()
    node = rclpy.create_node("tinker_controller_reconciler")
    rc = 1
    try:
        api = RosControllerManagerApi(node, args.controller_manager, args.service_timeout)
        for name in args.controllers:
            state = reconcile_controller(api, name, attempts=args.attempts)
            node.get_logger().info(f"controller {name} is {state.state}")
        if args.ready_node:
            if not args.ready_parameter:
                raise ValueError("--ready-parameter is required with --ready-node")
            set_remote_parameter(
                node,
                args.ready_node,
                args.ready_parameter,
                args.ready_value,
                timeout=args.ready_timeout,
            )
            node.get_logger().info(
                f"set {args.ready_node}.{args.ready_parameter}={args.ready_value}"
            )
        rc = 0
        return rc
    except (ReconciliationError, ValueError) as exc:
        node.get_logger().error(str(exc))
        rc = 1
        return rc
    finally:
        # Same teardown that ran here unconditionally before task #25, now
        # bounded: on success (rc=0) a timeout force-exits 0 (the reconciled
        # controller is already proven active; do not fail the launch chain
        # over a teardown-only hang) and logs it clearly. The same bound
        # applies on the failure path (rc=1) and preserves that non-zero
        # code, since this is the single teardown for both branches.
        def _teardown() -> None:
            node.destroy_node()
            rclpy.shutdown()

        bounded_teardown(
            _teardown,
            args.teardown_timeout_s,
            logger=node.get_logger(),
            exit_code=rc,
        )


def main(argv: list[str] | None = None) -> int:
    return _run(_parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
