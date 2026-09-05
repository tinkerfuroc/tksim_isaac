"""Task 6: real Humble readiness-waiter test.

Runs ``tinker_sim_bridge.readiness_waiter.wait_for_trigger`` against a live
local ``std_srvs/srv/Trigger`` server with a real executor, proving:

- success: the waiter exits 0 for a typed ``success=true`` response;
- bounded timeout: the waiter returns nonzero within the deadline when no
  server exists (service discovery bounds the process lifetime);
- false response: the waiter rejects a ``success=false`` response.

These tests require the Humble ROS Python runtime and are skipped under the
simulator CPython 3.12 venv.
"""
from __future__ import annotations

import json
import sys
import threading
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy", reason="requires Humble ROS Python runtime")
pytest.importorskip(
    "std_srvs", reason="requires Humble-generated std_srvs interfaces"
)

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.readiness_waiter import (  # noqa: E402
    EXIT_CALL_FAILED,
    EXIT_CALL_TIMEOUT,
    EXIT_SERVICE_UNAVAILABLE,
    EXIT_SUCCESS,
    wait_for_trigger,
)

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402


@pytest.fixture(scope="module")
def rclpy_context():
    if not rclpy.ok():
        rclpy.init(args=[])
    yield
    if rclpy.ok():
        rclpy.shutdown()


def _spin_server_thread(executor: SingleThreadedExecutor) -> threading.Thread:
    """Spin an executor on a daemon thread so the server can answer while the
    waiter (on the test thread) blocks inside ``wait_for_trigger``."""
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return thread


def _stop_server(executor: SingleThreadedExecutor, thread: threading.Thread) -> None:
    executor.shutdown()
    thread.join(timeout=2.0)


def test_waiter_succeeds_for_true_response(rclpy_context) -> None:
    server = Node("waiter_test_server")

    def handle(request: Trigger.Request, response: Trigger.Response):
        del request
        response.success = True
        response.message = "ready"
        return response

    server.create_service(Trigger, "/waiter_test/trigger", handle)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    thread = _spin_server_thread(executor)
    try:
        result = wait_for_trigger(
            service_name="/waiter_test/trigger",
            deadline_s=5.0,
            node_name="waiter_test_client_success",
            owns_rclpy=True,
        )
        assert result == EXIT_SUCCESS
    finally:
        _stop_server(executor, thread)
        server.destroy_node()


def test_waiter_retries_structured_pending_response(rclpy_context) -> None:
    server = Node("waiter_test_server_pending")
    calls = 0

    def handle(request: Trigger.Request, response: Trigger.Response):
        nonlocal calls
        del request
        calls += 1
        response.success = calls >= 3
        response.message = json.dumps(
            {"state": "FIXTURE_READY" if response.success else "FIXTURE_PENDING"}
        )
        return response

    server.create_service(Trigger, "/waiter_test/pending", handle)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    thread = _spin_server_thread(executor)
    try:
        result = wait_for_trigger(
            service_name="/waiter_test/pending",
            deadline_s=5.0,
            node_name="waiter_test_client_pending",
            owns_rclpy=True,
        )
        assert result == EXIT_SUCCESS
        assert calls == 3
    finally:
        _stop_server(executor, thread)
        server.destroy_node()


def test_waiter_bounded_timeout_with_no_server(rclpy_context) -> None:
    started = __import__("time").monotonic()
    result = wait_for_trigger(
        service_name="/waiter_test/absent",
        deadline_s=0.5,
        node_name="waiter_test_client_timeout",
        owns_rclpy=True,
    )
    elapsed = __import__("time").monotonic() - started
    assert result == EXIT_SERVICE_UNAVAILABLE
    assert elapsed < 3.0, "waiter was not bounded: {:.2f}s".format(elapsed)


def test_waiter_retries_call_that_exceeds_per_call_budget(rclpy_context) -> None:
    """RED: a slow first Trigger response must not consume the whole deadline.

    The first request's response arrives only after the per-call budget
    (``call_timeout_s``) elapses; the waiter must abandon that call, retry, and
    return EXIT_SUCCESS before the total deadline.  Today ``wait_for_trigger``
    spins on the first call for the entire remaining deadline and has no
    ``call_timeout_s`` parameter, so the second (successful) request is never
    given a chance -> the response is effectively lost.
    """
    import time as _time

    server = Node("waiter_test_server_slow")
    calls = 0

    def handle(request: Trigger.Request, response: Trigger.Response):
        nonlocal calls
        del request
        calls += 1
        if calls == 1:
            # First response arrives after the per-call budget (0.1 s) but far
            # inside the total deadline (2.0 s).
            _time.sleep(0.3)
            response.success = False
            response.message = json.dumps({"state": "FIXTURE_PENDING"})
        else:
            response.success = True
            response.message = json.dumps({"state": "FIXTURE_READY"})
        return response

    server.create_service(Trigger, "/waiter_test/slow", handle)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    thread = _spin_server_thread(executor)
    try:
        started = _time.monotonic()
        result = wait_for_trigger(
            service_name="/waiter_test/slow",
            deadline_s=2.0,
            call_timeout_s=0.1,
            node_name="waiter_test_client_slow",
            owns_rclpy=True,
        )
        elapsed = _time.monotonic() - started
        assert result == EXIT_SUCCESS
        assert calls >= 2
        assert elapsed < 1.5, "waiter did not retry before the total deadline: {:.2f}s".format(elapsed)
    finally:
        _stop_server(executor, thread)
        server.destroy_node()


def test_waiter_rejects_false_response(rclpy_context) -> None:
    server = Node("waiter_test_server_false")

    def handle(request: Trigger.Request, response: Trigger.Response):
        del request
        response.success = False
        response.message = "not ready"
        return response

    server.create_service(Trigger, "/waiter_test/false", handle)
    executor = SingleThreadedExecutor()
    executor.add_node(server)
    thread = _spin_server_thread(executor)
    try:
        result = wait_for_trigger(
            service_name="/waiter_test/false",
            deadline_s=5.0,
            node_name="waiter_test_client_false",
            owns_rclpy=True,
        )
        assert result == EXIT_CALL_FAILED
    finally:
        _stop_server(executor, thread)
        server.destroy_node()
