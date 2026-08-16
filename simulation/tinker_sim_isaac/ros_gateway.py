from __future__ import annotations

import array
from collections import deque
import json
import math
import os
import queue
import struct
import threading
import time
from pathlib import Path
from typing import Any, Mapping

from tinker_sim_core.command_mux import (
    command_from_sequences,
    decode_command_epoch,
    decode_command_frame,
    decode_snapshot_packet,
)
from tinker_sim_isaac.camera_rig import (
    camera_info_fields,
    depth_to_16uc1_mm,
    pack_registered_cloud,
    rgb8_array,
)


SAFETY_HEARTBEAT_TIMEOUT_S = 1.0
COMMAND_STREAM_TIMEOUT_S = 0.5
MAX_RETIRED_COMMAND_EPOCHS = 64
# R2: fixed range for the deterministic development-lidar fallback ring emitted
# when the backend carries no occupancy map.  Finite and inside the 40 m lidar
# bound so the qualification cloud consumer always receives a non-empty cloud.
_FALLBACK_LIDAR_RANGE_M = 1.0
# Safety and command messages use separate ROS topics.  Tolerate only a short
# bounded packet gap at that boundary; no packet is applied while resyncing.
BASELINE_RESYNC_WINDOW_S = 0.25
MAX_BASELINE_RESYNC_PACKETS = 8


class PhysicsTruthJsonlWriter:
    """Append raw evaluator frames to a crash-readable JSONL artifact.

    The writer is deliberately independent of ROS so it can be tested and
    used by the Isaac process without adding another truth consumer.  A
    missing path disables the artifact while keeping serialization identical
    to the ROS payload.
    """

    def __init__(self, path: str | os.PathLike[str] | None) -> None:
        self.path = Path(path).expanduser() if path else None
        self._stream = None
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._stream = self.path.open("a", encoding="utf-8", buffering=1)

    @classmethod
    def from_environment(cls) -> "PhysicsTruthJsonlWriter":
        return cls(os.environ.get("TINKER_SIM_TRUTH_JSONL"))

    @staticmethod
    def serialize(frame: Mapping[str, Any]) -> str:
        return json.dumps(frame, sort_keys=True, allow_nan=False)

    def append(self, frame: Mapping[str, Any]) -> str:
        serialized = self.serialize(frame)
        if self._stream is not None:
            self._stream.write(serialized)
            self._stream.write("\n")
            self._stream.flush()
        return serialized

    def close(self) -> None:
        if self._stream is not None:
            self._stream.close()
            self._stream = None


class RosStandardGateway:
    """Python 3.12 Isaac endpoint using only standard ROS messages.

    Simulation lifecycle services are intentionally absent here: NVIDIA's
    isaacsim.ros2.sim_control extension owns the standard simulation_interfaces
    API. All actuator commands arrive through one JointState topic.
    """

    def __init__(
        self,
        backend: Any,
        *,
        development_lidar: bool = False,
        camera_rig: Any | None = None,
        camera_pointcloud: bool = False,
    ) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from rclpy.signals import SignalHandlerOptions
        from geometry_msgs.msg import WrenchStamped
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, JointState, PointCloud2, PointField
        from std_msgs.msg import Bool, String

        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.rclpy = rclpy
        self.backend = backend
        self.development_lidar = development_lidar
        self.node = Node("tinker_isaac_gateway")
        # Keep commands and safety transitions in one FIFO.  Draining separate
        # queues by category can apply an old command after a stop has been
        # cleared, even when the callbacks arrived in the opposite order.
        self._incoming_events: queue.SimpleQueue[tuple[str, Any]] = queue.SimpleQueue()
        self._last_command_error: str | None = None
        # The backend starts stopped. A clear transition is valid only after
        # the supervisor delivers an explicit false sample.
        self._safety_active = True
        self._safety_timeout_s = SAFETY_HEARTBEAT_TIMEOUT_S
        self._safety_last_sample_at: float | None = None
        self.backend.set_safety_stop(True)
        # Isaac does not mint or increment epochs.  The gateway owns the
        # session/generation token and Isaac adopts only a fresh session's
        # first snapshot after a safety-clear sample.
        self._command_epoch: int | None = None
        self._session_protocol_enabled = True
        self._retired_command_epochs = deque(maxlen=MAX_RETIRED_COMMAND_EPOCHS)
        self._retired_command_sessions = deque(maxlen=MAX_RETIRED_COMMAND_EPOCHS)
        self._known_command_session: int | None = None
        self._known_command_generation: int | None = None
        self._last_epoch_adoption_clear_sequence = -1
        self._last_safety_clear_at: float | None = None
        self._snapshot_baseline_pending = True
        self._last_logical_snapshot_id = -1
        self._last_snapshot_packet_count = 0
        self._last_snapshot_packet_index = 0
        self._last_snapshot_id = -1
        self._snapshot_baseline_pending = True
        self._pending_baseline_packets: list[tuple[Any, int, float]] = []
        self._baseline_resync_until: float | None = None
        self._baseline_resync_packets_remaining = 0
        self._command_stream_timeout_s = COMMAND_STREAM_TIMEOUT_S
        self._last_command_received_at: float | None = None
        self._command_stream_lost = True
        self._command_loss_at = time.monotonic()
        self._safety_sample_sequence = 0
        self._last_safety_clear_sequence = -1
        self._command_loss_safety_sequence = 0
        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self._Clock = Clock
        self._JointState = JointState
        self._Imu = Imu
        self._PointCloud2 = PointCloud2
        self._PointField = PointField
        self._Bool = Bool
        self._String = String
        self._WrenchStamped = WrenchStamped
        self.clock_pub = self.node.create_publisher(Clock, "/clock", reliable)
        self.joint_pub = self.node.create_publisher(
            JointState, "/isaac_joint_states", reliable
        )
        self.imu_pub = self.node.create_publisher(
            Imu, "/livox/imu", qos_profile_sensor_data
        )
        self.cloud_pub = self.node.create_publisher(
            PointCloud2, "/livox/lidar", qos_profile_sensor_data
        )
        self.status_pub = self.node.create_publisher(
            String, "/sim/status/isaac", reliable
        )
        self.physics_truth_pub = self.node.create_publisher(
            String, "/sim/internal/physics_truth", reliable
        )
        collision_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.collision_pub = self.node.create_publisher(
            Bool, "/sim/safety/collision", collision_qos
        )
        self.contact_pub = self.node.create_publisher(
            WrenchStamped, "/sim/parity/finger_contact", reliable
        )
        self._camera_rig = camera_rig
        self.camera_skipped_frames = 0
        self._camera_streams: list[dict[str, Any]] = []
        self._camera_cloud_pub = None
        if camera_rig is not None:
            from sensor_msgs.msg import CameraInfo, Image

            self._Image = Image
            self._CameraInfo = CameraInfo
            # The real drivers publish RELIABLE + VOLATILE + KEEP_LAST(10)
            # (tk26_vision realsense_qos.yaml).  Every tk26_vision CameraInfo
            # subscription is RELIABLE; a best-effort publisher would deliver
            # zero messages to them, silently.
            camera_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            for spec in camera_rig.specs:
                self._camera_streams.append(
                    {
                        "spec": spec,
                        "info_fields": camera_info_fields(spec),
                        "color_pub": self.node.create_publisher(
                            Image, spec.color_topic, camera_qos
                        ),
                        "depth_pub": self.node.create_publisher(
                            Image, spec.depth_topic, camera_qos
                        ),
                        "info_pubs": [
                            self.node.create_publisher(CameraInfo, topic, camera_qos)
                            for topic in spec.camera_info_topics
                        ],
                    }
                )
            if camera_pointcloud:
                self._camera_cloud_pub = self.node.create_publisher(
                    PointCloud2, "/camera/depth_registered/points", camera_qos
                )
        initial_collision = Bool()
        initial_collision.data = False
        self.collision_pub.publish(initial_collision)
        self.node.create_subscription(
            JointState, "/isaac_joint_commands", self._joint_command, reliable
        )
        safety_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(
            Bool, "/sim/hardware/safety_stop", self._safety_stop, safety_qos
        )
        # NVIDIA's simulation-control extension owns a MultiThreadedExecutor.
        # Never use rclpy's global executor here: spin_once may execute
        # unrelated work and block the Kit/physics thread.  DDS callbacks run
        # in this private executor and only enqueue immutable commands.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._executor_thread = threading.Thread(
            target=self._spin_executor,
            name="tinker-isaac-ros-gateway",
            daemon=True,
        )
        self._executor_thread.start()
        self._state_stride = max(1, round((1.0 / 50.0) / backend.dt))
        self._lidar_stride = max(1, round((1.0 / 10.0) / backend.dt))
        self._imu_stride = max(1, round((1.0 / 200.0) / backend.dt))
        self._status_stride = max(1, round((1.0 / 2.0) / backend.dt))
        self._tick = 0

    def _spin_executor(self) -> None:
        from rclpy.executors import ExternalShutdownException
        from rclpy._rclpy_pybind11 import RCLError

        try:
            self._executor.spin()
        except ExternalShutdownException:
            # The standard simulation-control node installs the process signal
            # handler and may shut down the shared rclpy context first.
            pass
        except RCLError:
            # rcl_shutdown() can also invalidate the context between wait-set
            # rebuilds without raising ExternalShutdownException.
            if self.node.context.ok():
                raise

    def _stamp(self):
        from builtin_interfaces.msg import Time

        value = self.backend.simulation_time
        stamp = Time()
        stamp.sec = int(value)
        stamp.nanosec = int(round((value - int(value)) * 1.0e9))
        if stamp.nanosec >= 1_000_000_000:
            stamp.sec += 1
            stamp.nanosec -= 1_000_000_000
        return stamp

    def _joint_command(self, message) -> None:
        try:
            frame_id = getattr(getattr(message, "header", None), "frame_id", "")
            epoch, snapshot = decode_command_frame(frame_id)
            command = command_from_sequences(
                message.name, message.position, message.velocity, message.effort
            )
            self._incoming_events.put(
                ("command", (command, epoch, snapshot, time.monotonic()))
            )
        except Exception as error:
            self._last_command_error = str(error)
            self.node.get_logger().error(f"rejected joint command: {error}")

    def _safety_stop(self, message) -> None:
        try:
            received_at = time.monotonic()
            self._safety_last_sample_at = received_at
            self._safety_sample_sequence = (
                getattr(self, "_safety_sample_sequence", 0) + 1
            )
            self._incoming_events.put(
                (
                    "safety_stop",
                    (bool(message.data), received_at, self._safety_sample_sequence),
                )
            )
            self._last_command_error = None
        except Exception as error:
            self._last_command_error = str(error)
            node = getattr(self, "node", None)
            if node is not None:
                node.get_logger().error(f"rejected safety-stop message: {error}")

    def _apply_safety_stop(self, active: bool) -> None:
        if not getattr(self, "_session_protocol_enabled", False):
            if active == self._safety_active:
                return
            self._safety_active = active
            self._command_epoch += 1
            if active:
                self._last_snapshot_id = -1
            self.backend.set_safety_stop(active)
            return
        if active:
            self._baseline_resync_until = None
            self._baseline_resync_packets_remaining = 0
            self._pending_baseline_packets = []
            self._safety_active = True
            self._command_stream_lost = True
            self._command_loss_at = time.monotonic()
            self._command_loss_safety_sequence = getattr(
                self, "_safety_sample_sequence", 0
            )
            self._recoverable_command_epoch = None
            self._retired_command_epochs = getattr(
                self, "_retired_command_epochs", set()
            )
            self._retire_command_epoch()
            self._last_snapshot_id = -1
            self.backend.set_safety_stop(True)
            return
        self._safety_active = False
        self._arm_baseline_resynchronization()
        # A clear sample alone never releases the actuator hold.  A valid,
        # post-boundary command must arrive as well.
        if not getattr(self, "_command_stream_lost", False):
            self.backend.set_safety_stop(False)

    def _retire_command_epoch(
        self, *, retire: bool = True, reset_snapshot: bool = True
    ) -> None:
        self._retired_command_epochs = getattr(
            self, "_retired_command_epochs", set()
        )
        epoch = getattr(self, "_command_epoch", None)
        if retire and epoch is not None:
            if hasattr(self._retired_command_epochs, "append"):
                self._retired_command_epochs.append(epoch)
            else:
                self._retired_command_epochs.add(epoch)
                if len(self._retired_command_epochs) > MAX_RETIRED_COMMAND_EPOCHS:
                    self._retired_command_epochs.pop()
        self._command_epoch = None
        if reset_snapshot:
            self._last_logical_snapshot_id = -1
            self._last_snapshot_packet_count = 0
            self._last_snapshot_packet_index = 0
            self._last_snapshot_id = -1
            self._snapshot_baseline_pending = True
            self._snapshot_recovery_floor = None
            self._pending_baseline_packets = []
            self._baseline_resync_until = None
            self._baseline_resync_packets_remaining = 0

    def _arm_baseline_resynchronization(self) -> None:
        """Bound tolerance for packets following a lost packet-one boundary."""
        if not getattr(self, "_snapshot_baseline_pending", False):
            return
        boundary = getattr(self, "_last_safety_clear_at", None)
        if boundary is None:
            boundary = time.monotonic()
        self._baseline_resync_until = float(boundary) + BASELINE_RESYNC_WINDOW_S
        self._baseline_resync_packets_remaining = MAX_BASELINE_RESYNC_PACKETS

    def _consume_baseline_resynchronization(self, received_at: float) -> bool:
        """Drop one non-initial packet without mutating the backend."""
        deadline = getattr(self, "_baseline_resync_until", None)
        remaining = getattr(self, "_baseline_resync_packets_remaining", 0)
        if deadline is None or remaining <= 0 or float(received_at) > deadline:
            return False
        self._baseline_resync_packets_remaining = remaining - 1
        self._last_command_error = None
        return True

    def _validate_staged_command(self, command: Any) -> None:
        """Validate one command without making it executable.

        Isaac's backend deliberately rejects ``command_joints`` while stopped,
        so the gateway cannot use that method as a preflight API.  Prefer the
        backend's validation hook when it exposes one; the command-mux object
        still provides the common structural validation for lightweight test
        and compatibility backends.
        """
        validator = getattr(self.backend, "validate_command", None)
        if validator is None:
            validator = getattr(self.backend, "_validate_backend_command", None)
        if validator is not None:
            validator(command)
            return
        validate = getattr(command, "validate", None)
        if validate is not None:
            validate()

    def _reject_staged_baseline(self, error: BaseException) -> None:
        """Abort a baseline and leave both gateway and backend fail-closed."""
        self._safety_active = True
        self._command_stream_lost = True
        self._command_loss_at = time.monotonic()
        # A failed transaction is recoverable only after the next explicit
        # safety-clear boundary.  Keep the adopted epoch/session and all
        # anti-replay counters unchanged.
        self._command_loss_safety_sequence = max(
            0, getattr(self, "_last_safety_clear_sequence", 0)
        )
        self._snapshot_baseline_pending = True
        self._pending_baseline_packets = []
        try:
            # Isaac's stop transition also discards any backend snapshot
            # staging and restores the physical hold target.  This is the
            # rollback boundary for a backend without a stopped-state command
            # transaction API.
            self.backend.set_safety_stop(True)
        except Exception as stop_error:
            error = RuntimeError(
                f"{error}; failed to reassert safety stop: {stop_error}"
            )
        self._last_command_error = str(error)
        node = getattr(self, "node", None)
        get_logger = getattr(node, "get_logger", None)
        if get_logger is not None:
            get_logger().error(f"rejected command baseline: {error}")

    def _commit_staged_baseline(
        self, packets_to_apply: list[tuple[Any, int, float]]
    ) -> bool:
        """Validate a complete baseline, then apply it with bounded rollback.

        The Isaac backend has no public API for accepting commands while its
        physical stop is active.  Snapshot metadata and backend-specific
        command validation therefore happen under the stop first.  The actual
        packet application is contiguous in this gateway turn; any clear,
        begin, or command failure immediately reasserts the stop, which also
        discards the backend's partial snapshot.  Gateway acceptance state is
        committed only after every packet returns success.
        """
        begin_snapshot = getattr(self.backend, "begin_command_snapshot", None)
        if begin_snapshot is None:
            self._reject_staged_baseline(
                RuntimeError("rejected command: backend lacks snapshot boundary")
            )
            return False

        try:
            # The backend must be stopped throughout preflight.  begin_* may
            # stage packet ordering internally, so reset that staging before
            # the real commit pass below.
            self.backend.set_safety_stop(True)
            for staged_command, staged_snapshot, _ in packets_to_apply:
                begin_snapshot(staged_snapshot)
                self._validate_staged_command(staged_command)
            self.backend.set_safety_stop(True)

            # This is the only non-atomic portion for the legacy backend.  No
            # physics step can interleave with this single gateway turn, and
            # every failure path below restores the physical stop before the
            # command stream can be considered accepted.
            self.backend.set_safety_stop(False)
            for staged_command, staged_snapshot, _ in packets_to_apply:
                begin_snapshot(staged_snapshot)
                accepted = self.backend.command_joints(staged_command)
                if accepted is False:
                    raise RuntimeError("command rejected during baseline commit")
        except Exception as error:
            self._reject_staged_baseline(error)
            return False

        self._command_stream_lost = False
        return True

    def _fresh_clear_boundary(self, received_at: float) -> bool:
        clear_sequence = getattr(self, "_last_safety_clear_sequence", -1)
        last_adoption = getattr(
            self, "_last_epoch_adoption_clear_sequence", -1
        )
        clear_at = getattr(self, "_last_safety_clear_at", None)
        return (
            clear_sequence > last_adoption
            and clear_at is not None
            and received_at >= clear_at
        )

    def _adopt_command_epoch(
        self, epoch: int, *, received_at: float, new_session: bool
    ) -> None:
        """Atomically retire the prior epoch before accepting a new baseline."""
        session_id, generation = decode_command_epoch(epoch)
        current = getattr(self, "_command_epoch", None)
        if current is not None and current != epoch:
            self._retire_command_epoch()
        if new_session:
            known_session = getattr(self, "_known_command_session", None)
            if known_session is not None and known_session != session_id:
                sessions = getattr(self, "_retired_command_sessions", deque())
                if hasattr(sessions, "append"):
                    sessions.append(known_session)
                else:
                    sessions.add(known_session)
                self._retired_command_sessions = sessions
        self.backend.set_safety_stop(True)
        self._command_stream_lost = True
        self._command_loss_at = getattr(
            self, "_last_safety_clear_at", received_at
        )
        self._command_loss_safety_sequence = (
            getattr(self, "_last_safety_clear_sequence", 0) - 1
        )
        self._recoverable_command_epoch = None
        self._command_epoch = epoch
        self._known_command_session = session_id
        self._known_command_generation = generation
        self._last_epoch_adoption_clear_sequence = getattr(
            self, "_last_safety_clear_sequence", -1
        )
        self._last_logical_snapshot_id = -1
        self._last_snapshot_packet_count = 0
        self._last_snapshot_packet_index = 0
        self._last_snapshot_id = -1
        self._snapshot_baseline_pending = True
        self._pending_baseline_packets = []
        self._snapshot_recovery_floor: int | None = None
        self._arm_baseline_resynchronization()

    def _enter_command_stream_lost(self, now: float) -> None:
        if getattr(self, "_command_stream_lost", False):
            return
        self._command_stream_lost = True
        self._command_loss_at = now
        self._command_loss_safety_sequence = getattr(
            self, "_safety_sample_sequence", 0
        )
        self._recoverable_command_epoch = getattr(self, "_command_epoch", None)
        self._retire_command_epoch(retire=False, reset_snapshot=False)
        self._snapshot_baseline_pending = True
        self._snapshot_recovery_floor = self._last_logical_snapshot_id
        self.backend.set_safety_stop(True)
        self._last_command_error = "command stream expired"

    def _enforce_command_deadline(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else float(now)
        if getattr(self, "_command_stream_lost", False):
            return
        last = getattr(self, "_last_command_received_at", None)
        timeout = getattr(
            self, "_command_stream_timeout_s", COMMAND_STREAM_TIMEOUT_S
        )
        if last is not None and now - last >= timeout:
            self._enter_command_stream_lost(now)

    def _enforce_safety_deadline(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else float(now)
        if not hasattr(self, "_safety_last_sample_at"):
            return
        last = self._safety_last_sample_at
        timeout = getattr(self, "_safety_timeout_s", SAFETY_HEARTBEAT_TIMEOUT_S)
        if last is not None and now - last < timeout:
            return
        if self._safety_active:
            return
        try:
            self._apply_safety_stop(True)
            self._last_command_error = "safety heartbeat expired"
        except Exception as error:
            self._last_command_error = str(error)
            node = getattr(self, "node", None)
            if node is not None:
                node.get_logger().error(
                    f"failed to apply safety heartbeat timeout: {error}"
                )

    def spin_once(self) -> None:
        self._enforce_safety_deadline()
        self._enforce_command_deadline()
        while True:
            try:
                event_type, payload = self._incoming_events.get_nowait()
            except queue.Empty:
                return
            if event_type == "safety_stop":
                if isinstance(payload, tuple):
                    active, received_at = payload[:2]
                    sequence = payload[2] if len(payload) > 2 else None
                    if (
                        time.monotonic() - float(received_at)
                        >= getattr(
                            self,
                            "_safety_timeout_s",
                            SAFETY_HEARTBEAT_TIMEOUT_S,
                        )
                    ):
                        continue
                else:
                    active = bool(payload)
                    sequence = None
                if active and self._safety_active:
                    continue
                try:
                    if not active and sequence is not None:
                        self._last_safety_clear_sequence = int(sequence)
                        self._last_safety_clear_at = float(received_at)
                    self._apply_safety_stop(active)
                    self._last_command_error = None
                except Exception as error:
                    self._last_command_error = str(error)
                    self.node.get_logger().error(
                        f"failed to apply safety-stop: {error}"
                    )
                continue
            command, epoch, snapshot, received_at = payload
            if not getattr(self, "_session_protocol_enabled", False):
                if epoch != self._command_epoch:
                    self._last_command_error = (
                        f"rejected command epoch {epoch}; current epoch is "
                        f"{self._command_epoch}"
                    )
                    continue
                if snapshot < self._last_snapshot_id:
                    self._last_command_error = (
                        f"rejected old command snapshot {snapshot}; last snapshot is "
                        f"{self._last_snapshot_id}"
                    )
                    continue
                if snapshot > self._last_snapshot_id:
                    begin_snapshot = getattr(
                        self.backend, "begin_command_snapshot", None
                    )
                    if begin_snapshot is None:
                        self._last_command_error = (
                            "rejected command: backend lacks snapshot boundary"
                        )
                        continue
                    begin_snapshot(snapshot)
                    self._last_snapshot_id = snapshot
                accepted = self.backend.command_joints(command)
                if accepted is False:
                    self._last_command_error = (
                        "command ignored while safety stop is active"
                    )
                else:
                    self._last_command_error = None
                continue
            if self._safety_active:
                self._last_command_error = "command ignored while safety stop is active"
                continue
            if float(received_at) <= getattr(self, "_command_loss_at", -math.inf):
                self._last_command_error = "rejected command received before stream boundary"
                continue
            if getattr(self, "_command_stream_lost", False) and (
                self._last_safety_clear_sequence
                <= getattr(self, "_command_loss_safety_sequence", -1)
            ):
                self._last_command_error = "rejected command before fresh safety clear"
                continue
            session_id, generation = decode_command_epoch(epoch)
            if session_id in getattr(self, "_retired_command_sessions", ()):
                self._last_command_error = f"rejected retired command session {session_id}"
                continue
            if epoch in getattr(self, "_retired_command_epochs", set()):
                self._last_command_error = f"rejected retired command epoch {epoch}"
                continue
            try:
                logical_snapshot, packet_count, packet_index = decode_snapshot_packet(
                    snapshot
                )
            except ValueError as error:
                self._last_command_error = str(error)
                continue
            current_epoch = getattr(self, "_command_epoch", None)
            if current_epoch is not None and epoch != current_epoch:
                current_session, current_generation = decode_command_epoch(
                    current_epoch
                )
                new_session = session_id != current_session
                if new_session:
                    reason = "session change"
                elif generation <= current_generation:
                    reason = "old or lower command generation"
                else:
                    reason = "command generation change"
                if (
                    (not new_session and generation <= current_generation)
                    or not self._fresh_clear_boundary(float(received_at))
                ):
                    self._last_command_error = (
                        f"rejected {reason} without a fresh safety-clear boundary"
                    )
                    continue
                self._adopt_command_epoch(
                    epoch, received_at=float(received_at), new_session=new_session
                )
            elif current_epoch is None:
                recoverable_epoch = getattr(
                    self, "_recoverable_command_epoch", None
                )
                same_session_recovery = epoch == recoverable_epoch
                known_session = getattr(self, "_known_command_session", None)
                known_generation = getattr(self, "_known_command_generation", None)
                if same_session_recovery:
                    self._command_epoch = epoch
                elif session_id in getattr(self, "_retired_command_sessions", ()):
                    self._last_command_error = (
                        f"rejected retired command session {session_id}"
                    )
                    continue
                else:
                    if not self._fresh_clear_boundary(float(received_at)):
                        self._last_command_error = (
                            "rejected command session without a fresh safety-clear boundary"
                        )
                        continue
                    if (
                        known_session == session_id
                        and known_generation is not None
                        and generation <= known_generation
                    ):
                        self._last_command_error = (
                            "rejected old or lower command generation"
                        )
                        continue
                    self._adopt_command_epoch(
                        epoch,
                        received_at=float(received_at),
                        new_session=known_session not in (None, session_id),
                    )
            packets_to_apply: list[tuple[Any, int, float]] = [
                (command, snapshot, float(received_at))
            ]
            if self._snapshot_baseline_pending:
                if packet_index != 1:
                    pending = getattr(self, "_pending_baseline_packets", [])
                    if pending:
                        _, first_snapshot, _ = pending[0]
                        first_logical, first_count, first_index = decode_snapshot_packet(
                            first_snapshot
                        )
                        if (
                            logical_snapshot == first_logical
                            and packet_count == first_count
                            and packet_index == first_index + len(pending)
                        ):
                            packets_to_apply = pending + packets_to_apply
                            self._pending_baseline_packets = []
                            if packet_index < packet_count:
                                self._pending_baseline_packets = packets_to_apply
                                self._last_command_error = None
                                continue
                        else:
                            if self._consume_baseline_resynchronization(float(received_at)):
                                continue
                            self._last_command_error = (
                                "rejected session baseline that does not start at packet one"
                            )
                            continue
                    elif self._consume_baseline_resynchronization(float(received_at)):
                        continue
                    else:
                        self._last_command_error = (
                            "rejected session baseline that does not start at packet one"
                        )
                        continue
                recovery_floor = getattr(self, "_snapshot_recovery_floor", None)
                if recovery_floor is not None and logical_snapshot <= recovery_floor:
                    self._last_command_error = (
                        f"rejected recovery snapshot {logical_snapshot}; "
                        f"expected a value greater than {recovery_floor}"
                    )
                    continue
                if packet_count > 1 and packet_index == 1:
                    # Hold the first packet at the gateway boundary.  The
                    # backend cannot safely receive it until the full snapshot
                    # is present because releasing its physical stop early
                    # would make the transition fail open.
                    self._pending_baseline_packets = packets_to_apply
                    self._last_command_error = None
                    continue
            else:
                if logical_snapshot < self._last_logical_snapshot_id:
                    self._last_command_error = (
                        f"rejected old command snapshot {logical_snapshot}; last snapshot is "
                        f"{self._last_logical_snapshot_id}"
                    )
                    continue
                if logical_snapshot == self._last_logical_snapshot_id:
                    if (
                        packet_count != self._last_snapshot_packet_count
                        or packet_index != self._last_snapshot_packet_index + 1
                    ):
                        self._last_command_error = "rejected duplicate or out-of-order snapshot packet"
                        continue
                elif packet_index != 1:
                    self._last_command_error = "rejected snapshot that does not start at packet one"
                    continue
                elif logical_snapshot != self._last_logical_snapshot_id + 1:
                    self._last_command_error = (
                        f"rejected non-contiguous command snapshot {logical_snapshot}; "
                        f"expected {self._last_logical_snapshot_id + 1}"
                    )
                    continue
            if self._snapshot_baseline_pending:
                if not self._commit_staged_baseline(packets_to_apply):
                    continue
                self._last_logical_snapshot_id = logical_snapshot
                self._last_snapshot_packet_count = packet_count
                self._last_snapshot_packet_index = packet_index
                self._last_snapshot_id = logical_snapshot
                self._snapshot_baseline_pending = False
                self._snapshot_recovery_floor = None
                self._last_command_received_at = time.monotonic()
                self._last_command_error = None
                continue

            begin_snapshot = getattr(self.backend, "begin_command_snapshot", None)
            if begin_snapshot is None:
                self._last_command_error = "rejected command: backend lacks snapshot boundary"
                continue
            try:
                for staged_command, staged_snapshot, _ in packets_to_apply:
                    begin_snapshot(staged_snapshot)
                    if self._command_stream_lost:
                        self.backend.set_safety_stop(False)
                        self._command_stream_lost = False
                    accepted = self.backend.command_joints(staged_command)
                    if accepted is False:
                        self._command_stream_lost = True
                        self._last_command_error = (
                            "command ignored while safety stop is active"
                        )
                        break
                else:
                    self._last_logical_snapshot_id = logical_snapshot
                    self._last_snapshot_packet_count = packet_count
                    self._last_snapshot_packet_index = packet_index
                    self._last_snapshot_id = logical_snapshot
                    self._snapshot_baseline_pending = False
                    self._snapshot_recovery_floor = None
                    self._last_command_received_at = time.monotonic()
                    self._last_command_error = None
            except Exception as error:
                self._last_command_error = str(error)
                self.node.get_logger().error(f"rejected joint command: {error}")

    def publish(self) -> None:
        stamp = self._stamp()
        clock = self._Clock()
        clock.clock = stamp
        self.clock_pub.publish(clock)
        if self._tick % self._state_stride == 0:
            names, positions, velocities, efforts = self.backend.joint_state()
            message = self._JointState()
            message.header.stamp = stamp
            message.name = list(names)
            message.position = positions
            message.velocity = velocities
            message.effort = efforts
            self.joint_pub.publish(message)
        if self._tick % self._imu_stride == 0:
            state = self.backend.root_state()
            message = self._Imu()
            message.header.stamp = stamp
            message.header.frame_id = "livox360"
            message.orientation_covariance[0] = -1.0
            angular = state["angular_velocity_world"]
            (
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ) = angular
            self.imu_pub.publish(message)
        if self._cloud_publish_enabled():
            self.cloud_pub.publish(self._development_point_cloud(stamp))
        if self._tick % self._status_stride == 0:
            status = {
                "physics_device": self.backend.physics_device,
                "standard_simulation_control": True,
                "joint_command_topic": "/isaac_joint_commands",
                "last_command_error": self._last_command_error,
                "development_lidar": self.development_lidar,
                "safety_stop": bool(self.backend.safety_stopped),
            }
            if self._camera_rig is not None:
                status["camera_skipped_frames"] = self.camera_skipped_frames
            message = self._String()
            message.data = json.dumps(status, sort_keys=True)
            self.status_pub.publish(message)
            contacts = self.backend.contact_state()
            force = sum(
                float(item["force"])
                for name, item in contacts.items()
                if name in {"left_finger", "right_finger"}
            )
            contact = self._WrenchStamped()
            contact.header.stamp = stamp
            contact.header.frame_id = "link_tcp"
            contact.wrench.force.z = float(force)
            self.contact_pub.publish(contact)
        physics_truth = self._String()
        frame = dict(self.backend.physics_truth_frame(self.backend.TRUTH_TOKEN))
        frame["command_gateway"] = {
            "last_command_error": self._last_command_error,
            "command_stream_lost": bool(self._command_stream_lost),
            "active_epoch": self._command_epoch,
            "last_snapshot_id": self._last_logical_snapshot_id,
        }
        physics_truth.data = json.dumps(
            frame, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        self.physics_truth_pub.publish(physics_truth)
        self._tick += 1

    def publish_cameras(self) -> None:
        """Publish one same-stamp color+depth+info set per camera.

        A camera whose annotator has no frame this tick is skipped and counted
        rather than fabricated; the counter keeps stalls observable.
        """
        if self._camera_rig is None:
            return
        stamp = self._stamp()
        frames = self._camera_rig.capture()
        for entry in self._camera_streams:
            spec = entry["spec"]
            rgb, depth = frames.get(spec.name, (None, None))
            if rgb is None or depth is None:
                self.camera_skipped_frames += 1
                continue
            color_array = rgb8_array(rgb, spec.height, spec.width)
            depth_array = depth_to_16uc1_mm(depth)
            if depth_array.shape != (spec.height, spec.width):
                raise RuntimeError(
                    f"{spec.name} depth resolution {depth_array.shape} does not "
                    f"match the contract ({spec.height}, {spec.width})"
                )

            color = self._Image()
            color.header.stamp = stamp
            color.header.frame_id = spec.frame_id
            color.height = spec.height
            color.width = spec.width
            color.encoding = "rgb8"
            color.is_bigendian = 0
            color.step = spec.width * 3
            # array.array('B') takes rclpy's validated fast path; raw bytes trigger a per-element __debug__ scan that costs seconds per frame at 720p.
            color.data = array.array("B", color_array.tobytes())
            entry["color_pub"].publish(color)

            depth_msg = self._Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = spec.frame_id
            depth_msg.height = spec.height
            depth_msg.width = spec.width
            depth_msg.encoding = "16UC1"
            depth_msg.is_bigendian = 0
            depth_msg.step = spec.width * 2
            depth_msg.data = array.array("B", depth_array.tobytes())
            entry["depth_pub"].publish(depth_msg)

            fields = entry["info_fields"]
            info = self._CameraInfo()
            info.header.stamp = stamp
            info.header.frame_id = spec.frame_id
            info.height = fields["height"]
            info.width = fields["width"]
            info.distortion_model = fields["distortion_model"]
            info.d = list(fields["d"])
            info.k = fields["k"]
            info.r = fields["r"]
            info.p = fields["p"]
            for publisher in entry["info_pubs"]:
                publisher.publish(info)

            if self._camera_cloud_pub is not None and spec.name == "head_camera":
                cloud = self._PointCloud2()
                cloud.header.stamp = stamp
                cloud.header.frame_id = spec.frame_id
                cloud.height = spec.height
                cloud.width = spec.width
                cloud.fields = [
                    self._PointField(
                        name=name,
                        offset=offset,
                        datatype=self._PointField.FLOAT32,
                        count=1,
                    )
                    for name, offset in (("x", 0), ("y", 4), ("z", 8))
                ]
                cloud.is_bigendian = False
                cloud.point_step = 16
                cloud.row_step = 16 * spec.width
                cloud.is_dense = False
                cloud.data = array.array(
                    "B",
                    pack_registered_cloud(
                        depth,
                        fx=fields["k"][0],
                        fy=fields["k"][4],
                        cx=fields["k"][2],
                        cy=fields["k"][5],
                    ),
                )
                self._camera_cloud_pub.publish(cloud)

    def publish_safety_heartbeat(self) -> None:
        """Publish the collision source on a wall-clock cadence, pause-safe.

        ``publish()`` only runs while the timeline is playing, so the
        supervisor's required collision source would go stale during a pause
        (world load/spawn on the first cold start) and trip a spurious stop +
        controller deactivate.  This method republishes the current collision
        classification regardless of the timeline state and is throttled to a
        wall-clock period by the caller (``run_sim``).
        """
        collision = self._Bool()
        collision.data = bool(self.backend.arm_scenario_collision())
        try:
            self.collision_pub.publish(collision)
        except Exception:
            # Signal-driven rcl_shutdown() can invalidate the context between
            # the caller's loop check and this publish; a heartbeat is moot
            # once shutdown began, but a live-context failure is real.
            if self.node.context.ok():
                raise

    def _cloud_publish_enabled(self) -> bool:
        """Whether the development lidar cloud should publish on this tick.

        R2: the qualification development-lidar cloud must not depend on
        ``backend.occupancy`` being present.  The manipulation-core qualification
        profile carries no PGM map; the legacy gate hard-required ``occupancy is
        not None``, so ``/livox/lidar`` never fired and the cartesian-retreat
        ``environment_cloud_provider`` failed closed ("no live environment
        PointCloud2 is available").  Occupancy (when present) only shapes the
        raycast in :meth:`_development_point_cloud`; the dev lidar itself is the
        qualification sensor source and must always publish.
        """
        return bool(self.development_lidar) and self._tick % self._lidar_stride == 0

    def _development_point_cloud(self, stamp):
        state = self.backend.root_state()
        x, y, _ = state["position"]
        qw, qx, qy, qz = state["quaternion_wxyz"]
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        occupancy = getattr(self.backend, "occupancy", None)
        points = []
        for degrees in range(-90, 91):
            local = math.radians(degrees)
            if occupancy is not None:
                distance = occupancy.raycast(
                    x + 0.12 * math.cos(yaw),
                    y + 0.12 * math.sin(yaw),
                    yaw + local,
                )
            else:
                # R2 deterministic fallback: a fixed keep-out ring so the cloud
                # stays non-empty and finite for qualification consumers even
                # when no occupancy map exists.
                distance = _FALLBACK_LIDAR_RANGE_M
            if math.isfinite(distance):
                points.append(
                    (distance * math.cos(local), distance * math.sin(local), 0.0)
                )
        message = self._PointCloud2()
        message.header.stamp = stamp
        message.header.frame_id = "livox360"
        message.height = 1
        message.width = len(points)
        message.is_bigendian = False
        message.is_dense = True
        for index, name in enumerate(("x", "y", "z")):
            field = self._PointField()
            field.name = name
            field.offset = 4 * index
            field.datatype = self._PointField.FLOAT32
            field.count = 1
            message.fields.append(field)
        message.point_step = 12
        message.row_step = 12 * len(points)
        message.data = array.array(
            "B", b"".join(struct.pack("<fff", *point) for point in points)
        )
        return message

    def close(self) -> None:
        self._executor.shutdown()
        self._executor_thread.join(timeout=2.0)
        self._executor.remove_node(self.node)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()
