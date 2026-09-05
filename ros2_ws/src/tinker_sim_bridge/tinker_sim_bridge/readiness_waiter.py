"""Bounded ``std_srvs/srv/Trigger`` readiness waiter (Task 6).

The integrated launch stages the physics and fixture gates behind one-shot
processes that exit 0 only when the corresponding Trigger service answers
``success=true`` within the deadline.  This module is the installed, testable
implementation of that waiter::

    python3 -m tinker_sim_bridge.readiness_waiter --service /sim/ready/physics --deadline 120

Service discovery, the call, the response, retries, and the total process
lifetime all respect the bounded deadline.  Humble rclpy only completes a
``Client.call_async`` future when an executor drains the client node's wait
set, so the future is serviced by ``rclpy.spin_until_future_complete`` (itself
a bounded spin).  The process exits 0 only for a typed Trigger response with
``success=true``.  Structured ``*_PENDING`` responses are retried within the
same deadline; timeout, exception, service disappearance, malformed responses,
and terminal ``success=false`` responses exit nonzero.
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

EXIT_SUCCESS = 0
EXIT_CALL_FAILED = 1
EXIT_SERVICE_UNAVAILABLE = 2
EXIT_CALL_TIMEOUT = 3
_PENDING_RETRY_S = 0.05
#: Default per-call spin budget.  A single Trigger request is abandoned (and a
#: fresh request retried) after this long, so one slow/lost response never
#: consumes the whole total deadline.
DEFAULT_CALL_TIMEOUT_S = 5.0


def _response_is_pending(response: Trigger.Response) -> bool:
    try:
        payload = json.loads(response.message)
    except (TypeError, json.JSONDecodeError):
        return False
    state = payload.get("state") if isinstance(payload, dict) else None
    return isinstance(state, str) and (state == "PENDING" or state.endswith("_PENDING"))


def wait_for_trigger(
    *,
    service_name: str,
    deadline_s: float,
    node_name: str = "tinker_sim_readiness_waiter",
    owns_rclpy: bool = False,
    call_timeout_s: float = DEFAULT_CALL_TIMEOUT_S,
) -> int:
    """Block until *service_name* answers a Trigger ``success=true``.

    Returns one of :data:`EXIT_SUCCESS`, :data:`EXIT_CALL_FAILED`,
    :data:`EXIT_SERVICE_UNAVAILABLE`, :data:`EXIT_CALL_TIMEOUT`.

    Each Trigger request is spun for at most ``min(remaining, call_timeout_s)``
    seconds.  If the request is not answered within that per-call budget, or the
    future yields no response, a fresh request is retried while the total
    deadline remains, so one slow/lost response never consumes the whole
    deadline.  A typed ``success=true`` returns immediately; a terminal
    ``success=false`` (or malformed/unparseable) response fails immediately.

    When *owns_rclpy* is false (default) this function initializes rclpy and
    shuts it down on every path.  When true the caller already owns
    ``rclpy.init()`` (test embedding) and this function never shuts down rclpy.
    """
    own_init = not owns_rclpy
    if own_init:
        rclpy.init(args=[])
    node = Node(node_name)
    try:
        client = node.create_client(Trigger, service_name)
        deadline = time.monotonic() + float(deadline_s)
        # Discovery must also respect the deadline.
        if not client.wait_for_service(timeout_sec=float(deadline_s)):
            return EXIT_SERVICE_UNAVAILABLE
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return EXIT_CALL_TIMEOUT
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=min(remaining, float(call_timeout_s))
            )
            if not future.done():
                # Per-call budget exhausted; retry a fresh request while the
                # total deadline remains.
                continue
            response = future.result()
            if response is None:
                # Lost/empty response; retry a fresh request.
                continue
            if response.success:
                return EXIT_SUCCESS
            if not _response_is_pending(response):
                return EXIT_CALL_FAILED
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return EXIT_CALL_TIMEOUT
            time.sleep(min(_PENDING_RETRY_S, remaining))
    except Exception:  # noqa: BLE001 - every failure is a nonzero exit
        return EXIT_CALL_FAILED
    finally:
        node.destroy_node()
        if own_init and rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Wait for a std_srvs/srv/Trigger readiness service."
    )
    parser.add_argument("--service", required=True, help="Trigger service name")
    parser.add_argument(
        "--deadline", type=float, default=120.0, help="bounded wait deadline in seconds"
    )
    arguments = parser.parse_args(
        rclpy.utilities.remove_ros_args(
            args=sys.argv if argv is None else argv
        )[1:]
    )
    code = wait_for_trigger(
        service_name=arguments.service, deadline_s=arguments.deadline
    )
    raise SystemExit(code)


if __name__ == "__main__":
    main()
