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
        self._contact_pairs: list[dict[str, str]] = []

    def set_safety_stop(self, active: bool) -> None:
        self.safety_stopped = bool(active)
        if active:
            self.pending.clear()
            self.pending_count = 0
            self.pending_index = 0

    def contact_pairs(self) -> list[dict[str, str]]:
        return list(self._contact_pairs)

    def arm_scenario_collision(self) -> bool:
        from tinker_sim_isaac.backend import IsaacWholeRobotBackend

        return IsaacWholeRobotBackend.is_arm_scenario_collision(self.contact_pairs())

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


class CollisionHeartbeatTest(unittest.TestCase):
    """RED: collision must keep publishing while Isaac is paused.

    /sim/safety/collision is the supervisor's required source with a 1.0s
    deadline.  The physics-tick publish() is gated by is_playing(); on the
    first cold start the timeline is still paused (world load/spawn) so the
    collision source goes stale >1s, the supervisor asserts stop, and it
    issues a STRICT deactivate of the freshly-activated trajectory controller
    that the controller manager cannot satisfy — the joint-scenario
    controller-switch storm.  A pause-independent wall-clock heartbeat must
    keep the collision Bool fresh regardless of the timeline state.
    """

    def _gateway_with_capture(self, backend: _SnapshotBackend):
        gateway = _gateway(backend)
        gateway._Bool = _TestBool
        gateway.collision_pub = _CollisionCapture()
        return gateway

    def test_heartbeat_publishes_false_when_no_collision(self) -> None:
        backend = _SnapshotBackend()
        gateway = self._gateway_with_capture(backend)
        gateway.collision_pub.captured.clear()

        gateway.publish_safety_heartbeat()

        self.assertEqual(
            gateway.collision_pub.captured,
            [False],
            "pause-independent heartbeat must publish the collision Bool",
        )

    def test_heartbeat_publishes_true_when_arm_touches_scenario(self) -> None:
        backend = _SnapshotBackend()
        backend._contact_pairs = [
            {
                "body_a": "/World/Tinker/link1",
                "body_b": "/World/Scenario/sim_fixture/public_target",
            }
        ]
        gateway = self._gateway_with_capture(backend)

        gateway.publish_safety_heartbeat()

        self.assertEqual(
            gateway.collision_pub.captured,
            [True],
            "arm/scenario contact must surface as an active collision sample",
        )

    def test_heartbeat_ignores_arm_arm_contact(self) -> None:
        backend = _SnapshotBackend()
        backend._contact_pairs = [
            {"body_a": "/World/Tinker/link_tcp", "body_b": "/World/Tinker/link7"}
        ]
        gateway = self._gateway_with_capture(backend)

        gateway.publish_safety_heartbeat()

        self.assertEqual(gateway.collision_pub.captured, [False])


class _CollisionCapture:
    """Stand-in for the gateway's rclpy collision publisher."""

    def __init__(self) -> None:
        self.captured: list[bool] = []

    def publish(self, message: object) -> None:
        self.captured.append(bool(message.data))


class _TestBool:
    """Minimal stand-in for std_msgs/msg/Bool used by object.__new__ harness."""

    def __init__(self) -> None:
        self.data = False


class _CloudBackend:
    """Minimal backend for the development-lidar publish path."""

    def __init__(self, occupancy: object | None) -> None:
        self.occupancy = occupancy
        self.root_state_calls = 0

    def root_state(self) -> dict[str, object]:
        self.root_state_calls += 1
        return {
            "position": [0.0, 0.0, 0.0],
            "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        }


def _cloud_gateway(*, development_lidar: bool, occupancy: object | None) -> RosStandardGateway:
    from builtin_interfaces.msg import Time  # type: ignore[import-untyped]
    from sensor_msgs.msg import PointCloud2, PointField  # type: ignore[import-untyped]

    gateway = object.__new__(RosStandardGateway)
    gateway.backend = _CloudBackend(occupancy)
    gateway.development_lidar = development_lidar
    gateway._tick = 0
    gateway._lidar_stride = 1
    gateway._PointCloud2 = PointCloud2
    gateway._PointField = PointField
    gateway._stamp = lambda: Time(sec=0, nanosec=0)
    return gateway


def _cloud_stamp() -> object:
    from builtin_interfaces.msg import Time  # type: ignore[import-untyped]

    return Time(sec=0, nanosec=0)


class RosDevelopmentLidarTest(unittest.TestCase):
    def test_development_lidar_publishes_without_occupancy(self) -> None:
        """RED (R2): the development lidar cloud must publish even when the
        backend has no occupancy map.  Live rerun-5 raised
        ``environment_cloud_provider raised: no live environment PointCloud2 is
        available`` because the observer received zero ``/livox/lidar`` clouds;
        the cloud-publish gate hard-required ``backend.occupancy is not None``
        (manipulation-core qualification has no PGM map), so the dev lidar never
        fired.  The gate must not depend on occupancy, and
        ``_development_point_cloud`` must emit a non-empty finite cloud from a
        deterministic fallback."""
        gateway = _cloud_gateway(development_lidar=True, occupancy=None)
        self.assertTrue(
            gateway._cloud_publish_enabled(),
            "development lidar must publish without an occupancy map",
        )
        cloud = gateway._development_point_cloud(_cloud_stamp())
        self.assertGreaterEqual(
            cloud.width, 1, "development cloud must be non-empty when occupancy is None"
        )
        self.assertEqual(cloud.height, 1)
        self.assertTrue(cloud.is_dense)
        self.assertGreaterEqual(len(cloud.data), 12, "non-empty cloud must carry point data")

    def test_development_lidar_uses_occupancy_raycast_when_present(self) -> None:
        """The occupancy raycast path stays the primary source when a map
        exists. The sensor origin's own cell must be free (unlike
        ``test_development_lidar_empty_when_origin_occupied``) so this
        exercises the raycast path rather than the occupied-origin guard.

        This asserts a geometry-derived point, not just non-emptiness: a
        regression that reinstated the raycast-floor ring (every ray at the
        0.3 m minimum) would still satisfy a bare ``width >= 1`` check with
        181 fake points, so that alone is not a real guard on the dev-lidar
        occupied-origin fix (``0a42eec``).

        Geometry, independent of the implementation under test:
        ``_CloudBackend.root_state`` fixes position (0, 0, 0) and an identity
        orientation, so yaw = 0 and the sensor origin sits 0.12 m ahead of
        the robot at world (0.12, 0.0) -- see the 0.12 m mount offset in
        ``_development_point_cloud``. The fixture's ``OccupancyMap`` is 4x4
        at 1.0 m resolution with origin (-2.0, -2.0), free (not occupied)
        only at grid cell (gx=2, gy=2) -- i.e. world x in [0, 1), y in
        [0, 1) -- and occupied everywhere else (including out of bounds).
        The sensor origin (0.12, 0.0) falls inside that one free cell.

        The straight-ahead ray (local angle 0, i.e. index 90 of the 181 rays
        for degrees -90..90) walks outward from ``minimum=0.3`` in
        ``step = resolution / 2 = 0.5`` increments along +x: 0.3 -> world
        x=0.42 (still inside the free cell, x in [0,1)) -> 0.8 -> world
        x=0.92 (still inside) -> 1.3 -> world x=1.42, grid cell (gx=3, gy=2),
        which is occupied. So the ray must stop at distance 1.3 m, landing
        the point at local (1.3, 0.0, 0.0) in the sensor frame.
        """
        import struct

        from tinker_sim_core.occupancy import OccupancyMap

        rows = tuple(
            tuple(not (gy == 2 and gx == 2) for gx in range(4)) for gy in range(4)
        )
        occ = OccupancyMap(4, 4, 1.0, -2.0, -2.0, rows)
        gateway = _cloud_gateway(development_lidar=True, occupancy=occ)
        cloud = gateway._development_point_cloud(_cloud_stamp())
        self.assertEqual(cloud.width, 181, "every ray must resolve inside the 4x4 grid")

        straight_ahead_index = 90  # degrees == 0 within range(-90, 91)
        offset = straight_ahead_index * cloud.point_step
        px, py, pz = struct.unpack_from("<fff", cloud.data, offset)
        self.assertAlmostEqual(px, 1.3, places=5)
        self.assertAlmostEqual(py, 0.0, places=5)
        self.assertAlmostEqual(pz, 0.0, places=5)

    def test_cloud_disabled_without_development_lidar(self) -> None:
        gateway = _cloud_gateway(development_lidar=False, occupancy=None)
        self.assertFalse(
            gateway._cloud_publish_enabled(),
            "cloud must not publish when the development lidar flag is off",
        )

    def test_development_lidar_empty_when_origin_occupied(self) -> None:
        """A sensor origin inside an occupied cell has no valid returns; the
        cloud must be empty rather than a fake ring at the raycast floor."""

        class _OccupiedEverywhere:
            def occupied_at_world(self, x: float, y: float) -> bool:
                return True

            def raycast(
                self,
                x: float,
                y: float,
                angle: float,
                minimum: float = 0.3,
                maximum: float = 40.0,
            ) -> float:
                return minimum

        gateway = _cloud_gateway(
            development_lidar=True, occupancy=_OccupiedEverywhere()
        )
        message = gateway._development_point_cloud(_cloud_stamp())
        self.assertEqual(message.width, 0)

    def test_cloud_publish_gate_respects_stride(self) -> None:
        gateway = _cloud_gateway(development_lidar=True, occupancy=None)
        gateway._tick = 1
        gateway._lidar_stride = 2
        self.assertFalse(gateway._cloud_publish_enabled())
        gateway._tick = 2
        self.assertTrue(gateway._cloud_publish_enabled())


if __name__ == "__main__":
    unittest.main()
