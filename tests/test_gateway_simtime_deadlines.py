"""Gateway liveness deadlines must judge staleness in both clocks.

The safety heartbeat (0.25 s wall cadence) and the command stream come from
separate bridge processes that keep publishing while the simulation loop
stalls -- an RTX render stride or a loaded box blocks the stepping thread
for multiple wall seconds with healthy samples queued in DDS.  A wall-only
deadline then re-latched the limp safety hold on every stride and
invalidated the command stream (observed 2026-08-31 on the grasp-benchmark
stack; the same 1.0 s wall deadline is the prime suspect for the GPSR
battery's post-abort "arm ignores trajectories" state).

Simulation time freezes exactly when the loop stalls, so a deadline that
requires the newest sample to be stale in BOTH wall time and simulation
time cannot punish the loop for its own stall, still trips within one
simulated timeout when a publisher actually dies while the sim steps, and
cannot trip early under faster-than-realtime stepping (wall age still
gates).  Receipts without a simulation stamp keep the wall-only behavior,
which is what every pre-existing deadline test exercises.
"""
from __future__ import annotations

import queue
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.ros_gateway import RosStandardGateway  # noqa: E402


class _Backend:
    def __init__(self, simulation_time: float) -> None:
        self.simulation_time = simulation_time
        self.stops: list[bool] = []
        self.safety_stopped = False

    def set_safety_stop(self, active: bool) -> None:
        self.stops.append(active)
        self.safety_stopped = active


def _gateway(backend: _Backend) -> RosStandardGateway:
    gateway = object.__new__(RosStandardGateway)
    gateway.backend = backend
    gateway._incoming_events = queue.SimpleQueue()
    gateway._last_command_error = None
    gateway._safety_active = False
    gateway._command_epoch = 3
    gateway._last_snapshot_id = 44
    gateway._last_logical_snapshot_id = 44
    gateway._safety_timeout_s = 1.0
    gateway._command_stream_timeout_s = 0.5
    gateway._command_stream_lost = False
    return gateway


class SafetyDeadlineSimTime(unittest.TestCase):
    def test_loop_stall_does_not_relatch_the_hold(self) -> None:
        # Sample received at wall 10.0 / sim 5.0; the loop then stalls for
        # ten wall seconds during which simulation time cannot advance.
        backend = _Backend(simulation_time=5.0)
        gateway = _gateway(backend)
        gateway._safety_last_sample_at = 10.0
        gateway._safety_last_sample_sim_at = 5.0

        gateway._enforce_safety_deadline(now=20.0)

        self.assertFalse(gateway._safety_active)
        self.assertEqual(backend.stops, [])

    def test_dead_publisher_still_trips_while_sim_steps(self) -> None:
        backend = _Backend(simulation_time=6.5)
        gateway = _gateway(backend)
        gateway._safety_last_sample_at = 10.0
        gateway._safety_last_sample_sim_at = 5.0

        gateway._enforce_safety_deadline(now=11.5)

        self.assertTrue(gateway._safety_active)
        self.assertEqual(backend.stops, [True])
        self.assertEqual(gateway._last_command_error, "safety heartbeat expired")

    def test_receipt_without_sim_stamp_keeps_wall_only_behavior(self) -> None:
        backend = _Backend(simulation_time=5.0)
        gateway = _gateway(backend)
        gateway._safety_last_sample_at = 10.0

        gateway._enforce_safety_deadline(now=11.5)

        self.assertTrue(gateway._safety_active)

    def test_fresh_wall_age_never_trips_regardless_of_sim_age(self) -> None:
        # Faster-than-realtime stepping: sim advanced 3 s while only 0.2
        # wall seconds passed.  The wall gate must keep the hold released.
        backend = _Backend(simulation_time=8.0)
        gateway = _gateway(backend)
        gateway._safety_last_sample_at = 10.0
        gateway._safety_last_sample_sim_at = 5.0

        gateway._enforce_safety_deadline(now=10.2)

        self.assertFalse(gateway._safety_active)
        self.assertEqual(backend.stops, [])


class CommandDeadlineSimTime(unittest.TestCase):
    def test_loop_stall_does_not_expire_the_command_stream(self) -> None:
        backend = _Backend(simulation_time=5.0)
        gateway = _gateway(backend)
        gateway._last_command_received_at = 10.0
        gateway._last_command_received_sim_at = 5.0

        gateway._enforce_command_deadline(now=20.0)

        self.assertFalse(gateway._command_stream_lost)
        self.assertEqual(backend.stops, [])

    def test_dead_command_stream_still_expires_while_sim_steps(self) -> None:
        backend = _Backend(simulation_time=6.0)
        gateway = _gateway(backend)
        gateway._last_command_received_at = 10.0
        gateway._last_command_received_sim_at = 5.0

        gateway._enforce_command_deadline(now=11.0)

        self.assertTrue(gateway._command_stream_lost)
        self.assertEqual(backend.stops, [True])
        self.assertEqual(gateway._last_command_error, "command stream expired")

    def test_receipt_without_sim_stamp_keeps_wall_only_behavior(self) -> None:
        backend = _Backend(simulation_time=5.0)
        gateway = _gateway(backend)
        gateway._last_command_received_at = 10.0

        gateway._enforce_command_deadline(now=11.0)

        self.assertTrue(gateway._command_stream_lost)


if __name__ == "__main__":
    unittest.main()
