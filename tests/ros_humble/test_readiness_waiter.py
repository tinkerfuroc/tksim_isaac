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
