"""A multi-packet baseline must survive the gateway's preflight pass.

`RosStandardGateway._commit_staged_baseline` validates a staged baseline in
two passes over the same packet list: a preflight pass under the physical
stop, then the real commit pass.  Both passes call
`backend.begin_command_snapshot()` for every packet, and the real backend
enforces strict packet ordering -- packet N+1 must follow packet N.

The preflight pass therefore leaves the backend's staging index at
`packet_count`, and the commit pass restarts at packet 1.  The gateway
intends to clear that staging between the passes (ros_gateway.py: "begin_*
may stage packet ordering internally, so reset that staging before the real
commit pass below") by calling `set_safety_stop(True)` a second time -- but
the backend early-returns on a repeated identical sample to protect the
acceleration-limited wheel state (backend.py, see tests/test_base_velocity_slew.py),
so that reset never runs.

Observed live on 2026-08-20 (domain 71, gpsr-rcw2026): every base command was
refused with `expected command snapshot packet 3, got 1` for a 2-packet
baseline and `expected command snapshot packet 5, got 1` for a 4-packet one --
i.e. exactly `packet_count + 1` -- and the robot never moved.  Nav2 kept
publishing /cmd_vel and the base facade kept publishing wheel velocities;
the commands died at the gateway/backend boundary.

`tests/test_ros_gateway.py::_SnapshotBackend` hides this: its
`begin_command_snapshot` resets staging whenever `packet_index == 1`, so the
double accepts a restart the real backend rejects.  These tests drive the
real ordering logic instead.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.command_mux import encode_snapshot_packet  # noqa: E402
from tinker_sim_isaac.backend import IsaacWholeRobotBackend  # noqa: E402
from tinker_sim_isaac.ros_gateway import RosStandardGateway  # noqa: E402

GATEWAY = ROOT / "simulation/tinker_sim_isaac/ros_gateway.py"


class _Targets:
    """Minimal stand-in for the torch command buffers."""

    def __init__(self) -> None:
        self.zeroed = 0

    def zero_(self) -> None:
        self.zeroed += 1


class _StagingOnlyBackend:
    """The real snapshot-ordering methods over plain attributes.

    `begin_command_snapshot` and the staging discard touch only the
    `_pending_snapshot_*` / `_command_snapshot_id` fields and the command
    buffers, so they can be exercised without Isaac or torch.
    """

    def __init__(self) -> None:
        self._command_snapshot_id = None
        self._pending_snapshot_id = None
        self._pending_snapshot_count = 0
        self._pending_snapshot_index = 0
        self._pending_snapshot_commands = []
        self._velocity_targets = _Targets()
        self._effort_targets = _Targets()

    begin_command_snapshot = IsaacWholeRobotBackend.begin_command_snapshot
    discard_command_snapshot_staging = (
        IsaacWholeRobotBackend.discard_command_snapshot_staging
    )


def _packets(logical_id: int, count: int) -> list[int]:
    return [encode_snapshot_packet(logical_id, count, i) for i in range(1, count + 1)]


class BaselineCommitSnapshotStagingTest(unittest.TestCase):
    def test_preflight_leaves_staging_at_packet_count(self):
        """Characterises the state the commit pass has to recover from."""
        backend = _StagingOnlyBackend()
        for packet in _packets(7, 2):
            backend.begin_command_snapshot(packet)
        self.assertEqual(backend._pending_snapshot_index, 2)
        self.assertEqual(backend._pending_snapshot_id, 7)

    def test_second_pass_over_the_same_packets_is_refused_without_a_discard(self):
        """The live failure, reproduced: 2 packets -> 'expected 3, got 1'."""
        backend = _StagingOnlyBackend()
        packets = _packets(7, 2)
        for packet in packets:
            backend.begin_command_snapshot(packet)
        with self.assertRaises(ValueError) as caught:
            backend.begin_command_snapshot(packets[0])
        self.assertIn("expected command snapshot packet 3, got 1", str(caught.exception))

    def test_discard_lets_the_commit_pass_replay_the_same_baseline(self):
        backend = _StagingOnlyBackend()
        packets = _packets(7, 2)
        for packet in packets:
            backend.begin_command_snapshot(packet)
        backend.discard_command_snapshot_staging()
        for packet in packets:  # must not raise
            backend.begin_command_snapshot(packet)
        self.assertEqual(backend._pending_snapshot_index, 2)

    def test_four_packet_baseline_reports_packet_five(self):
        """The other message seen live, pinned to packet_count + 1."""
        backend = _StagingOnlyBackend()
        packets = _packets(11, 4)
        for packet in packets:
            backend.begin_command_snapshot(packet)
        with self.assertRaises(ValueError) as caught:
            backend.begin_command_snapshot(packets[0])
        self.assertIn("expected command snapshot packet 5, got 1", str(caught.exception))

    def test_discard_is_idempotent_and_safe_when_nothing_is_staged(self):
        backend = _StagingOnlyBackend()
        backend.discard_command_snapshot_staging()
        backend.discard_command_snapshot_staging()
        self.assertIsNone(backend._pending_snapshot_id)
        self.assertEqual(backend._pending_snapshot_index, 0)

    def test_discard_does_not_zero_the_command_buffers(self):
        """Discarding staging is a bookkeeping reset, not a physical stop.

        Zeroing velocity/effort targets here would duplicate what
        `set_safety_stop` already owns, and would clear the
        acceleration-limited wheel state the early return protects.
        """
        backend = _StagingOnlyBackend()
        for packet in _packets(7, 2):
            backend.begin_command_snapshot(packet)
        backend.discard_command_snapshot_staging()
        self.assertEqual(backend._velocity_targets.zeroed, 0)
        self.assertEqual(backend._effort_targets.zeroed, 0)

    def test_discard_preserves_the_last_completed_snapshot_id(self):
        """Anti-replay must survive a discard: only staging is dropped."""
        backend = _StagingOnlyBackend()
        backend._command_snapshot_id = 5
        for packet in _packets(7, 2):
            backend.begin_command_snapshot(packet)
        backend.discard_command_snapshot_staging()
        self.assertEqual(backend._command_snapshot_id, 5)


class _StrictBackend(_StagingOnlyBackend):
    """A backend double that enforces the real packet ordering.

    `tests/test_ros_gateway.py::_SnapshotBackend` resets staging on
    `packet_index == 1`, which silently accepts the replay the real backend
    refuses.  This double inherits the genuine methods instead.
    """

    def __init__(self) -> None:
        super().__init__()
        self.safety_stopped = True
        self.commands: list[object] = []

    def set_safety_stop(self, active: bool) -> None:
        # Mirrors the real early return that makes the second
        # `set_safety_stop(True)` a no-op.
        if bool(active) == self.safety_stopped:
            return
        self.safety_stopped = bool(active)
        if active:
            self.discard_command_snapshot_staging()

    def command_joints(self, command) -> bool:
        if self.safety_stopped:
            return False
        self.commands.append(command)
        return True


class _StubGateway:
    """The real baseline methods over the minimum surrounding state."""

    _commit_staged_baseline = RosStandardGateway._commit_staged_baseline
    _discard_backend_snapshot_staging = (
        RosStandardGateway._discard_backend_snapshot_staging
    )
    _reject_staged_baseline = RosStandardGateway._reject_staged_baseline

    def __init__(self, backend) -> None:
        self.backend = backend
        self.node = None
        self._command_stream_lost = True
        self._safety_active = False
        self._command_loss_at = None
        self._command_loss_safety_sequence = 0
        self._last_safety_clear_sequence = 0
        self._snapshot_baseline_pending = False
        self._pending_baseline_packets = []
        self._last_command_error = None
        self.validated: list[object] = []

    def _validate_staged_command(self, command) -> None:
        self.validated.append(command)


class BaselineCommitGatewayTest(unittest.TestCase):
    def _staged(self, logical_id: int, count: int):
        return [
            (object(), packet, 0.0) for packet in _packets(logical_id, count)
        ]

    def test_multi_packet_baseline_commits(self):
        """The live failure, end to end: this returned False before the fix."""
        backend = _StrictBackend()
        gateway = _StubGateway(backend)
        self.assertTrue(
            gateway._commit_staged_baseline(self._staged(7, 2)),
            f"baseline was rejected: {gateway._last_command_error}",
        )
        self.assertIsNone(gateway._last_command_error)
        self.assertFalse(gateway._command_stream_lost)
        self.assertEqual(len(backend.commands), 2, "both packets must be applied")
        self.assertFalse(backend.safety_stopped, "commit must leave the stop cleared")

    def test_four_packet_baseline_commits(self):
        backend = _StrictBackend()
        gateway = _StubGateway(backend)
        self.assertTrue(gateway._commit_staged_baseline(self._staged(11, 4)))
        self.assertEqual(len(backend.commands), 4)

    def test_single_packet_baseline_still_commits(self):
        """The path that worked before must keep working."""
        backend = _StrictBackend()
        gateway = _StubGateway(backend)
        self.assertTrue(gateway._commit_staged_baseline(self._staged(3, 1)))
        self.assertEqual(len(backend.commands), 1)

    def test_every_packet_is_validated_once_during_preflight(self):
        backend = _StrictBackend()
        gateway = _StubGateway(backend)
        staged = self._staged(7, 3)
        gateway._commit_staged_baseline(staged)
        self.assertEqual(len(gateway.validated), 3)

    def test_a_rejected_baseline_leaves_no_staging_behind(self):
        """A failed attempt must not poison the next one."""
        backend = _StrictBackend()
        gateway = _StubGateway(backend)

        def _boom(command):
            raise RuntimeError("validation failed")

        gateway._validate_staged_command = _boom
        self.assertFalse(gateway._commit_staged_baseline(self._staged(7, 2)))
        self.assertIsNone(backend._pending_snapshot_id)
        self.assertEqual(backend._pending_snapshot_index, 0)

        # The retry, with validation working again, must now succeed.
        gateway2 = _StubGateway(backend)
        self.assertTrue(
            gateway2._commit_staged_baseline(self._staged(8, 2)),
            f"retry after abort was rejected: {gateway2._last_command_error}",
        )


if __name__ == "__main__":
    unittest.main()
