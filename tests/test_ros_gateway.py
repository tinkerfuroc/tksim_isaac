from __future__ import annotations

import queue
import sys
import time
import unittest
from types import SimpleNamespace
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.command_mux import (  # noqa: E402
    JointCommand,
    encode_command_epoch,
    encode_command_frame,
    encode_snapshot_packet,
)
from tinker_sim_isaac.ros_gateway import RosStandardGateway  # noqa: E402


class _SnapshotBackend:
    def __init__(self) -> None:
        self.safety_stopped = True
        self.position_target = -0.4
        self.pending: list[object] = []
        self.pending_count = 0
        self.pending_index = 0
        self.begin_calls: list[int] = []

    def set_safety_stop(self, active: bool) -> None:
        self.safety_stopped = bool(active)
        if active:
            self.pending.clear()
            self.pending_count = 0
            self.pending_index = 0

    def begin_command_snapshot(self, snapshot: int) -> None:
        from tinker_sim_core.command_mux import decode_snapshot_packet

        logical_id, packet_count, packet_index = decode_snapshot_packet(snapshot)
        self.begin_calls.append(snapshot)
        if packet_index == 1:
            self.pending = []
        self.pending_count = packet_count
        self.pending_index = packet_index

    def command_joints(self, command) -> bool:
        if self.safety_stopped:
            return False
        if self.pending_count and self.pending_index < self.pending_count:
            self.pending.append(command)
            return True
        if self.pending_count:
            self.pending.append(command)
            command = self.pending[-1]
            self.pending.clear()
            self.pending_count = 0
            self.pending_index = 0
        if command.positions:
            self.position_target = float(command.positions[0])
        return True


class _FaultInjectingBackend(_SnapshotBackend):
    def __init__(
        self,
        *,
        begin_error: bool = False,
        command_failure_index: int | None = None,
        command_failure_mode: str = "reject",
        clear_error: bool = False,
    ) -> None:
        super().__init__()
        self.hold_position_target = self.position_target
        self.begin_error = begin_error
        self.command_failure_index = command_failure_index
        self.command_failure_mode = command_failure_mode
        self.clear_error = clear_error
        self.stop_calls: list[bool] = []
        self.begin_count = 0
        self.command_count = 0
        self.committed_command_count = 0

    def set_safety_stop(self, active: bool) -> None:
        self.stop_calls.append(bool(active))
        if not active and self.clear_error:
            raise RuntimeError("injected safety clear failure")
        super().set_safety_stop(active)
        if active:
            self.position_target = self.hold_position_target

    def begin_command_snapshot(self, snapshot: int) -> None:
        self.begin_count += 1
        if self.begin_error:
            raise RuntimeError("injected begin snapshot failure")
        super().begin_command_snapshot(snapshot)

    def command_joints(self, command) -> bool:
        self.command_count += 1
        if not self.safety_stopped:
            self.committed_command_count += 1
        if self.command_count == self.command_failure_index:
            if command.positions:
                # Exercise rollback even if a backend mutates before reporting
                # its failure.  The stop transition must remove this target.
                self.position_target = float(command.positions[0])
            if self.command_failure_mode == "exception":
                raise RuntimeError("injected command exception")
            return False
        return super().command_joints(command)


def _gateway(backend: _SnapshotBackend | None = None) -> RosStandardGateway:
    backend = backend or _SnapshotBackend()
    gateway = object.__new__(RosStandardGateway)
    gateway.backend = backend
    gateway._incoming_events = queue.SimpleQueue()
    gateway._last_command_error = None
    gateway._safety_active = True
    gateway._session_protocol_enabled = True
    gateway._command_epoch = None
    gateway._retired_command_epochs = set()
    gateway._retired_command_sessions = set()
    gateway._known_command_session = None
    gateway._known_command_generation = None
    gateway._last_logical_snapshot_id = -1
    gateway._last_snapshot_packet_count = 0
    gateway._last_snapshot_packet_index = 0
    gateway._last_snapshot_id = -1
    gateway._snapshot_baseline_pending = True
    gateway._command_stream_timeout_s = 0.5
    gateway._last_command_received_at = None
    gateway._command_stream_lost = True
    gateway._command_loss_at = time.monotonic()
    gateway._safety_sample_sequence = 1
    gateway._last_safety_clear_sequence = 1
    gateway._last_epoch_adoption_clear_sequence = -1
    gateway._command_loss_safety_sequence = 0
    gateway._safety_timeout_s = 1.0
    gateway._safety_last_sample_at = time.monotonic()
    gateway._last_safety_clear_at = gateway._safety_last_sample_at
    return gateway


def _message(epoch: int, snapshot: int, position: float):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=encode_command_frame(epoch, snapshot)
        ),
        name=("joint1",),
        position=(position,),
        velocity=(),
        effort=(),
    )


class RosGatewayOrderingTest(unittest.TestCase):
    def test_packet_one_before_clear_packet_two_after_clear_then_recover(self) -> None:
        gateway = _gateway()
        epoch = encode_command_epoch(53, 1)

        gateway._joint_command(_message(epoch, encode_snapshot_packet(20, 2, 1), 0.25))
        gateway.spin_once()
        self.assertEqual(gateway.backend.position_target, -0.4)

        gateway._safety_stop(SimpleNamespace(data=False))
        gateway.spin_once()

        # Packet two is never accepted or staged; the expected cross-topic race
        # is silent while the gateway remains fail-closed.
        gateway._joint_command(_message(epoch, encode_snapshot_packet(20, 2, 2), 0.75))
        gateway.spin_once()
        self.assertIsNone(gateway._last_command_error)
        self.assertTrue(gateway.backend.safety_stopped)
        self.assertEqual(gateway.backend.position_target, -0.4)
        self.assertEqual(gateway.backend.begin_calls, [])
        self.assertTrue(gateway._snapshot_baseline_pending)

        # A later complete snapshot is required before release and mutation.
        gateway._joint_command(_message(epoch, encode_snapshot_packet(21, 2, 1), 0.6))
        gateway.spin_once()
        self.assertTrue(gateway.backend.safety_stopped)
        self.assertEqual(gateway.backend.position_target, -0.4)
        gateway._joint_command(_message(epoch, encode_snapshot_packet(21, 2, 2), 0.8))
        gateway.spin_once()

        self.assertFalse(gateway.backend.safety_stopped)
        self.assertFalse(gateway._snapshot_baseline_pending)
        self.assertEqual(gateway.backend.position_target, 0.8)

    def test_expired_resync_window_rejects_packet_two_without_backend_mutation(self) -> None:
        gateway = _gateway()
        epoch = encode_command_epoch(54, 1)
        gateway._safety_stop(SimpleNamespace(data=False))
        gateway.spin_once()
        gateway._command_epoch = epoch
        gateway._last_epoch_adoption_clear_sequence = (
            gateway._last_safety_clear_sequence
        )
        gateway._baseline_resync_until = time.monotonic() - 1.0

        gateway._joint_command(_message(epoch, encode_snapshot_packet(22, 2, 2), 0.9))
        gateway.spin_once()

        self.assertIn("session baseline", gateway._last_command_error)
        self.assertTrue(gateway.backend.safety_stopped)
        self.assertEqual(gateway.backend.position_target, -0.4)
        self.assertEqual(gateway.backend.begin_calls, [])

    def _run_baseline_failure(
        self,
        backend: _FaultInjectingBackend,
        *,
        packet_count: int = 3,
    ) -> RosStandardGateway:
        gateway = _gateway(backend)
        epoch = encode_command_epoch(55, 1)
        gateway._safety_stop(SimpleNamespace(data=False))
        gateway.spin_once()
        for packet_index in range(1, packet_count + 1):
            gateway._joint_command(
                _message(
                    epoch,
                    encode_snapshot_packet(30, packet_count, packet_index),
                    0.1 * packet_index,
                )
            )
            gateway.spin_once()
        return gateway

    def _assert_failed_baseline_is_stopped(
        self, gateway: RosStandardGateway
    ) -> None:
        self.assertTrue(gateway._safety_active)
        self.assertTrue(gateway._command_stream_lost)
        self.assertTrue(gateway._snapshot_baseline_pending)
        self.assertTrue(gateway.backend.safety_stopped)
        self.assertEqual(gateway.backend.position_target, -0.4)
        self.assertFalse(
            gateway.backend.command_joints(
                JointCommand(("joint1",), positions=(9.0,))
            )
        )
        self.assertEqual(gateway.backend.position_target, -0.4)

    def test_begin_snapshot_exception_keeps_baseline_stopped(self) -> None:
        backend = _FaultInjectingBackend(begin_error=True)
        gateway = self._run_baseline_failure(backend)

        self._assert_failed_baseline_is_stopped(gateway)
        self.assertNotIn(False, backend.stop_calls)
        self.assertIn("injected begin snapshot failure", gateway._last_command_error)
        self.assertEqual(backend.committed_command_count, 0)

    def test_middle_packet_rejection_reasserts_stop_and_discards_partial_snapshot(self) -> None:
        backend = _FaultInjectingBackend(
            command_failure_index=2,
            command_failure_mode="reject",
        )
        gateway = self._run_baseline_failure(backend)

        self._assert_failed_baseline_is_stopped(gateway)
        self.assertEqual(backend.committed_command_count, 2)
        self.assertEqual(backend.stop_calls[-2:], [False, True])

    def test_middle_packet_exception_reasserts_stop_and_discards_partial_snapshot(self) -> None:
        backend = _FaultInjectingBackend(
            command_failure_index=2,
            command_failure_mode="exception",
        )
        gateway = self._run_baseline_failure(backend)

        self._assert_failed_baseline_is_stopped(gateway)
        self.assertIn("injected command exception", gateway._last_command_error)
        self.assertEqual(backend.committed_command_count, 2)

    def test_final_packet_rejection_reasserts_stop_and_discards_partial_snapshot(self) -> None:
        backend = _FaultInjectingBackend(
            command_failure_index=3,
            command_failure_mode="reject",
        )
        gateway = self._run_baseline_failure(backend)

        self._assert_failed_baseline_is_stopped(gateway)
        self.assertEqual(backend.committed_command_count, 3)

    def test_final_packet_exception_reasserts_stop_and_discards_partial_snapshot(self) -> None:
        backend = _FaultInjectingBackend(
            command_failure_index=3,
            command_failure_mode="exception",
        )
        gateway = self._run_baseline_failure(backend)

        self._assert_failed_baseline_is_stopped(gateway)
        self.assertIn("injected command exception", gateway._last_command_error)
        self.assertEqual(backend.committed_command_count, 3)

    def test_clear_exception_keeps_complete_baseline_unexecutable(self) -> None:
        backend = _FaultInjectingBackend(clear_error=True)
        gateway = self._run_baseline_failure(backend, packet_count=1)

        self._assert_failed_baseline_is_stopped(gateway)
        self.assertIn("injected safety clear failure", gateway._last_command_error)
        self.assertEqual(backend.committed_command_count, 0)

    def test_complete_baseline_commits_before_gateway_acceptance(self) -> None:
        backend = _FaultInjectingBackend()
        gateway = self._run_baseline_failure(backend)

        self.assertFalse(gateway._safety_active)
        self.assertFalse(gateway._command_stream_lost)
        self.assertFalse(gateway._snapshot_baseline_pending)
        self.assertFalse(backend.safety_stopped)
        self.assertAlmostEqual(backend.position_target, 0.3)
        self.assertEqual(gateway._last_logical_snapshot_id, 30)
        self.assertEqual(gateway._last_snapshot_packet_count, 3)
        self.assertEqual(gateway._last_snapshot_packet_index, 3)
        self.assertEqual(backend.command_count, 3)
        self.assertEqual(backend.stop_calls[-2:], [True, False])


if __name__ == "__main__":
    unittest.main()
