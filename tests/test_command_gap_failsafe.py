"""A lost command packet must stop the base, not freeze its last velocity.

Observed 2026-10-11 (gpsr_stack_logs/20261011T081535, route run near map
(3.3, 0.15)): a ~1 s sim-loop stall overflowed the depth-50
``/isaac_joint_commands`` reader, a snapshot was lost, and strict contiguity
then rejected every later snapshot as non-contiguous.  The backend kept the
last applied wheel velocity (0.35 m/s) for the whole 5 sim-s command-stream
timeout -- through RPP's collision-ahead zero ``/cmd_vel`` -- and drove the
robot ~1 m into furniture.  A forward gap is now treated as a lost stream:
stop immediately, then recover through the existing fresh-clear baseline.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "tests"))

from tinker_sim_core.command_mux import (  # noqa: E402
    encode_command_frame,
    encode_snapshot_packet,
)
from tinker_sim_isaac.backend import WHEEL_VELOCITY_SLEW_RAD_S2  # noqa: E402
from test_manipulation_runtime import _backend, _protocol_gateway  # noqa: E402

EPOCH = 404


def _send(gateway, logical: int, velocity: float, *, count: int = 1, index: int = 1) -> None:
    gateway._joint_command(
        SimpleNamespace(
            header=SimpleNamespace(
                frame_id=encode_command_frame(
                    EPOCH, encode_snapshot_packet(logical, count, index)
                )
            ),
            name=("joint1",),
            position=(),
            velocity=(velocity,),
            effort=(),
        )
    )
    gateway.spin_once()


def _joint1_velocity(gateway) -> float:
    return float(gateway.backend._velocity_targets[0, 1])


class CommandGapFailSafeTest(unittest.TestCase):
    def _driving_gateway(self):
        gateway = _protocol_gateway(epoch=EPOCH)
        _send(gateway, 0, 1.5)
        _send(gateway, 1, 1.5)
        self.assertFalse(gateway._command_stream_lost)
        self.assertEqual(gateway._last_logical_snapshot_id, 1)
        self.assertAlmostEqual(_joint1_velocity(gateway), 1.5)
        return gateway

    def test_skipped_snapshot_stops_instead_of_holding_velocity(self) -> None:
        gateway = self._driving_gateway()

        _send(gateway, 3, 1.5)  # snapshot 2 was lost in transport

        self.assertIn("non-contiguous", gateway._last_command_error)
        self.assertTrue(gateway._command_stream_lost)
        self.assertTrue(gateway.backend._safety_stopped)
        self.assertEqual(_joint1_velocity(gateway), 0.0)

    def test_lost_first_packet_of_next_snapshot_stops(self) -> None:
        gateway = self._driving_gateway()

        _send(gateway, 2, 1.5, count=2, index=2)  # packet 1 of snapshot 2 lost

        self.assertIn("packet one", gateway._last_command_error)
        self.assertTrue(gateway._command_stream_lost)
        self.assertEqual(_joint1_velocity(gateway), 0.0)

    def test_skipped_packet_within_snapshot_stops(self) -> None:
        gateway = self._driving_gateway()
        _send(gateway, 2, 1.5, count=3, index=1)

        _send(gateway, 2, 1.5, count=3, index=3)  # packet 2 lost

        self.assertTrue(gateway._command_stream_lost)
        self.assertEqual(_joint1_velocity(gateway), 0.0)

    def test_stale_or_duplicate_packet_is_dropped_without_stopping(self) -> None:
        gateway = self._driving_gateway()

        _send(gateway, 0, 9.0)  # delayed copy of an older snapshot
        self.assertIn("old command snapshot", gateway._last_command_error)
        _send(gateway, 1, 9.0)  # duplicate of the current snapshot
        self.assertIn("duplicate", gateway._last_command_error)

        self.assertFalse(gateway._command_stream_lost)
        self.assertFalse(gateway.backend._safety_stopped)
        self.assertAlmostEqual(_joint1_velocity(gateway), 1.5)

    def test_stream_recovers_after_fresh_safety_clear(self) -> None:
        gateway = self._driving_gateway()
        _send(gateway, 3, 1.5)
        self.assertTrue(gateway._command_stream_lost)

        # Without a fresh clear sample the stop holds.
        _send(gateway, 4, 0.5)
        self.assertTrue(gateway._command_stream_lost)
        self.assertEqual(_joint1_velocity(gateway), 0.0)

        gateway._safety_stop(SimpleNamespace(data=False))  # 0.25 s heartbeat
        gateway.spin_once()
        _send(gateway, 5, 0.5)

        self.assertFalse(gateway._command_stream_lost)
        self.assertFalse(gateway.backend._safety_stopped)
        self.assertEqual(gateway._last_logical_snapshot_id, 5)
        self.assertAlmostEqual(_joint1_velocity(gateway), 0.5)
        _send(gateway, 6, 0.25)
        self.assertAlmostEqual(_joint1_velocity(gateway), 0.25)


class WheelSlewKeepsCommandTest(unittest.TestCase):
    """One command packet must be tracked to completion by the per-step slew.

    The slew used to write its intermediate value back into the command
    buffer, so the next step slewed toward that value instead of the command:
    a wheel moved one slew step per *received packet*, and a stop decelerated
    only as fast as keepalive packets arrived.
    """

    def _wheel_backend(self, applied: float):
        backend = _backend()
        backend.dt = 1.0 / 60.0
        backend._wheel_indices = (1,)
        backend._wheel_index_tensor = torch.tensor([1], dtype=torch.long)
        backend._applied_wheel_velocities = {1: applied}
        return backend

    def test_single_stop_command_decelerates_to_zero(self) -> None:
        backend = self._wheel_backend(applied=6.0)
        backend._velocity_targets[0, 1] = 0.0  # one zero-velocity packet
        steps = int(6.0 / (WHEEL_VELOCITY_SLEW_RAD_S2 * backend.dt)) + 1

        written = []
        for _ in range(steps):
            written.append(float(backend._slew_wheel_targets()[0, 1]))

        self.assertAlmostEqual(written[0], 6.0 - WHEEL_VELOCITY_SLEW_RAD_S2 * backend.dt, places=5)
        self.assertEqual(written[-1], 0.0)
        self.assertEqual(float(backend._velocity_targets[0, 1]), 0.0)

    def test_single_drive_command_accelerates_to_target(self) -> None:
        backend = self._wheel_backend(applied=0.0)
        backend._velocity_targets[0, 1] = 3.0
        for _ in range(4):
            output = backend._slew_wheel_targets()
        self.assertAlmostEqual(float(output[0, 1]), 3.0, places=5)
        self.assertEqual(backend._applied_wheel_velocities[1], 3.0)


if __name__ == "__main__":
    unittest.main()
