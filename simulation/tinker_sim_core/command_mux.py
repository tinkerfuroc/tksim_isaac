from __future__ import annotations

import math
import secrets
from dataclasses import dataclass
from typing import Mapping, Sequence


COMMAND_FRAME_PREFIX = "tinker_command_epoch:"
COMMAND_SNAPSHOT_PREFIX = "snapshot:"
SNAPSHOT_PACKET_BITS = 16
SNAPSHOT_PACKET_MASK = (1 << SNAPSHOT_PACKET_BITS) - 1
SNAPSHOT_ATOMIC_MARKER = 1 << 63
# Bits 32..62 carry the logical ID; bit 63 is reserved as the atomic marker.
MAX_LOGICAL_SNAPSHOT_ID = (1 << 31) - 1
COMMAND_SESSION_BITS = 32
COMMAND_GENERATION_BITS = 32
COMMAND_GENERATION_MASK = (1 << COMMAND_GENERATION_BITS) - 1
COMMAND_SESSION_MASK = (1 << COMMAND_SESSION_BITS) - 1


def new_command_session() -> int:
    """Return a non-zero session nonce owned by one gateway process."""
    return secrets.randbits(COMMAND_SESSION_BITS) or 1


def encode_command_epoch(session_id: int, generation: int) -> int:
    """Pack the gateway session and its stop/recovery generation.

    The packed value remains an integer so the ROS frame stays compatible with
    the existing wire format. Isaac treats it as opaque; only the gateway may
    create a new value.
    """
    if (
        isinstance(session_id, bool)
        or isinstance(generation, bool)
        or not isinstance(session_id, int)
        or not isinstance(generation, int)
        or session_id < 1
        or session_id > COMMAND_SESSION_MASK
        or generation < 0
        or generation > COMMAND_GENERATION_MASK
    ):
        raise ValueError("command session and generation are invalid")
    return (session_id << COMMAND_GENERATION_BITS) | generation


def decode_command_epoch(epoch: int) -> tuple[int, int]:
    """Decode a packed epoch, retaining support for legacy test integers."""
    if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 0:
        raise ValueError("command epoch must be a non-negative integer")
    if epoch <= COMMAND_GENERATION_MASK:
        return 0, epoch
    return epoch >> COMMAND_GENERATION_BITS, epoch & COMMAND_GENERATION_MASK


def encode_snapshot_packet(
    snapshot_id: int, packet_count: int, packet_index: int
) -> int:
    """Encode atomic snapshot metadata into one monotonic frame integer.

    ``snapshot_id`` is limited to 31 bits because the high marker bit and the
    two 16-bit packet fields occupy the remaining bits. Plain legacy IDs are
    still decoded as one complete packet.
    """
    if (
        isinstance(snapshot_id, bool)
        or isinstance(packet_count, bool)
        or isinstance(packet_index, bool)
        or not isinstance(snapshot_id, int)
        or not isinstance(packet_count, int)
        or not isinstance(packet_index, int)
        or snapshot_id < 0
        or snapshot_id > MAX_LOGICAL_SNAPSHOT_ID
        or packet_count < 1
        or packet_count > SNAPSHOT_PACKET_MASK
        or packet_index < 1
        or packet_index > packet_count
    ):
        raise ValueError("command snapshot packet metadata is invalid")
    return (
        SNAPSHOT_ATOMIC_MARKER
        | (snapshot_id << (2 * SNAPSHOT_PACKET_BITS))
        | (packet_count << SNAPSHOT_PACKET_BITS)
        | packet_index
    )


def decode_snapshot_packet(snapshot_id: int) -> tuple[int, int, int]:
    """Decode logical snapshot ID, packet count, and one-based packet index."""
    if (
        isinstance(snapshot_id, bool)
        or not isinstance(snapshot_id, int)
        or snapshot_id < 0
    ):
        raise ValueError("command snapshot ID must be a non-negative integer")
    if not snapshot_id & SNAPSHOT_ATOMIC_MARKER:
        return snapshot_id, 1, 1
    payload = snapshot_id ^ SNAPSHOT_ATOMIC_MARKER
    packet_index = payload & SNAPSHOT_PACKET_MASK
    packet_count = (payload >> SNAPSHOT_PACKET_BITS) & SNAPSHOT_PACKET_MASK
    logical_id = payload >> (2 * SNAPSHOT_PACKET_BITS)
    if (
        logical_id > MAX_LOGICAL_SNAPSHOT_ID
        or packet_count < 1
        or packet_index < 1
        or packet_index > packet_count
    ):
        raise ValueError("command snapshot packet metadata is invalid")
    return logical_id, packet_count, packet_index


def encode_command_frame(epoch: int, snapshot: int) -> str:
    """Encode the command epoch and complete-snapshot sequence strictly."""
    if (
        isinstance(epoch, bool)
        or isinstance(snapshot, bool)
        or not isinstance(epoch, int)
        or not isinstance(snapshot, int)
        or epoch < 0
        or snapshot < 0
    ):
        raise ValueError("command epoch and snapshot must be non-negative integers")
    return f"{COMMAND_FRAME_PREFIX}{epoch};{COMMAND_SNAPSHOT_PREFIX}{snapshot}"


def decode_command_frame(frame_id: str) -> tuple[int, int]:
    """Decode a command frame ID, rejecting ambiguous or malformed metadata."""
    if not isinstance(frame_id, str):
        raise ValueError("command frame ID must be a string")
    parts = frame_id.split(";")
    if len(parts) != 2 or not parts[0].startswith(COMMAND_FRAME_PREFIX):
        raise ValueError("command frame ID has an invalid epoch prefix")
    epoch_text = parts[0][len(COMMAND_FRAME_PREFIX) :]
    if not parts[1].startswith(COMMAND_SNAPSHOT_PREFIX):
        raise ValueError("command frame ID has an invalid snapshot prefix")
    snapshot_text = parts[1][len(COMMAND_SNAPSHOT_PREFIX) :]
    if (
        not epoch_text
        or not snapshot_text
        or not epoch_text.isascii()
        or not snapshot_text.isascii()
        or not epoch_text.isdecimal()
        or not snapshot_text.isdecimal()
    ):
        raise ValueError("command frame ID values must be ASCII decimal integers")
    epoch = int(epoch_text)
    snapshot = int(snapshot_text)
    if encode_command_frame(epoch, snapshot) != frame_id:
        raise ValueError("command frame ID is not in canonical form")
    return epoch, snapshot


@dataclass(frozen=True)
class JointCommand:
    names: tuple[str, ...]
    positions: tuple[float, ...] = ()
    velocities: tuple[float, ...] = ()
    efforts: tuple[float, ...] = ()

    def validate(self) -> None:
        if not self.names or len(self.names) != len(set(self.names)):
            raise ValueError("joint command names must be non-empty and unique")
        for label, values in (
            ("positions", self.positions),
            ("velocities", self.velocities),
            ("efforts", self.efforts),
        ):
            if values and len(values) != len(self.names):
                raise ValueError(f"{label} length must match joint names")
            if any(not math.isfinite(value) for value in values):
                raise ValueError(f"{label} must contain only finite values")


@dataclass(frozen=True)
class CommandSource:
    joints: frozenset[str]
    timeout_s: float

    def __post_init__(self) -> None:
        if not self.joints:
            raise ValueError("a command source must own at least one joint")
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0.0:
            raise ValueError("command timeout must be finite and positive")


class JointCommandMux:
    """Deterministic, single-publisher joint command arbitration.

    Each joint has exactly one configured source. Source timeouts replace
    velocity and effort commands with zero while holding the measured position
    captured at the watchdog transition.
    A safety stop forces the same safe output for every source.
    """

    def __init__(self, sources: Mapping[str, CommandSource]) -> None:
        if not sources:
            raise ValueError("at least one command source is required")
        owners: dict[str, str] = {}
        for source, policy in sources.items():
            for joint in policy.joints:
                if joint in owners:
                    raise ValueError(
                        f"joint {joint!r} has conflicting owners "
                        f"{owners[joint]!r} and {source!r}"
                    )
                owners[joint] = source
        self.sources = dict(sources)
        self.owners = owners
        self._latest: dict[str, tuple[float, JointCommand]] = {}
        self._measured_positions: dict[str, float] = {}
        self._stale_position_holds: dict[str, dict[str, float]] = {}
        self._velocity_controlled: dict[str, set[str]] = {
            source: set() for source in self.sources
        }
        self._retired_velocities: dict[str, set[str]] = {
            source: set() for source in self.sources
        }
        self._stopped_packets: tuple[JointCommand, ...] = ()
        self.safety_stop = False

    def accept(self, source: str, command: JointCommand, steady_time: float) -> None:
        if source not in self.sources:
            raise KeyError(f"unknown command source: {source}")
        if not math.isfinite(steady_time):
            raise ValueError("steady time must be finite")
        if self.safety_stop:
            raise RuntimeError("commands are rejected while the safety stop is active")
        command.validate()
        foreign = set(command.names) - self.sources[source].joints
        if foreign:
            raise ValueError(f"{source} attempted to command unowned joints: {sorted(foreign)}")
        velocity_controlled = self._velocity_controlled[source]
        retired_velocities = self._retired_velocities[source]
        if command.velocities:
            velocity_controlled.update(command.names)
            retired_velocities.difference_update(command.names)
        else:
            retired_velocities.update(velocity_controlled.intersection(command.names))
            velocity_controlled.difference_update(command.names)
        self._stale_position_holds.pop(source, None)
        self._latest[source] = (steady_time, command)

    def observe_positions(
        self, names: Sequence[str], positions: Sequence[float]
    ) -> None:
        if len(names) != len(positions):
            raise ValueError("observed position length must match joint names")
        for name, position in zip(names, positions):
            value = float(position)
            if name in self.owners and math.isfinite(value):
                self._measured_positions[name] = value

    def stop(self, active: bool = True) -> None:
        active = bool(active)
        if active and not self.safety_stop:
            self._stopped_packets = self._compose_packets(
                steady_time=math.inf, force_safe=True
            )
            self._latest.clear()
            for joints in self._velocity_controlled.values():
                joints.clear()
            for joints in self._retired_velocities.values():
                joints.clear()
        elif not active and self.safety_stop:
            self._stopped_packets = ()
        self.safety_stop = active

    def compose(self, steady_time: float) -> tuple[JointCommand, ...]:
        if not math.isfinite(steady_time):
            raise ValueError("steady time must be finite")
        if self.safety_stop:
            return self._stopped_packets
        return self._compose_packets(steady_time=steady_time, force_safe=False)

    def _compose_packets(
        self, *, steady_time: float, force_safe: bool
    ) -> tuple[JointCommand, ...]:
        packets: list[JointCommand] = []
        for source, policy in self.sources.items():
            record = self._latest.get(source)
            if record is None:
                if force_safe:
                    names = tuple(
                        sorted(
                            name
                            for name in policy.joints
                            if name in self._measured_positions
                        )
                    )
                    if names:
                        packet = JointCommand(
                            names,
                            positions=tuple(
                                self._measured_positions[name] for name in names
                            ),
                        )
                        packet.validate()
                        packets.append(packet)
                continue
            stale = force_safe or steady_time - record[0] > policy.timeout_s
            command = record[1]
            if stale and not force_safe:
                # Freeze the measured hold at the watchdog transition. Later
                # observations must not make a timed-out source move again.
                self._stale_position_holds.setdefault(
                    source,
                    {
                        name: self._measured_positions[name]
                        for name in command.names
                        if name in self._measured_positions
                    },
                )
            stale_hold = self._stale_position_holds.get(source, {})
            if command.positions:
                hold = stale_hold if stale and not force_safe else self._measured_positions
                positions = (
                    tuple(hold[name] for name in command.names)
                    if stale and all(name in hold for name in command.names)
                    else ()
                    if stale
                    else command.positions
                )
            elif stale and all(
                name in (stale_hold if not force_safe else self._measured_positions)
                for name in command.names
            ):
                # A velocity/effort-only source still needs a finite position
                # hold once its watchdog expires or the safety stop is active.
                hold = stale_hold if not force_safe else self._measured_positions
                positions = tuple(hold[name] for name in command.names)
            else:
                positions = ()
            velocities = (
                tuple(0.0 for _ in command.names)
                if stale
                and (
                    command.velocities
                    or self._retired_velocities[source].intersection(command.names)
                )
                else (
                    tuple(0.0 for _ in command.names)
                    if not command.velocities
                    and self._retired_velocities[source].intersection(command.names)
                    else command.velocities
                )
            )
            efforts = (
                tuple(0.0 for _ in command.names)
                if stale and command.efforts
                else command.efforts
            )
            packet = JointCommand(command.names, positions, velocities, efforts)
            packet.validate()
            packets.append(packet)
        return tuple(packets)


def command_from_sequences(
    names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float],
    efforts: Sequence[float],
) -> JointCommand:
    command = JointCommand(
        tuple(names),
        tuple(float(value) for value in positions),
        tuple(float(value) for value in velocities),
        tuple(float(value) for value in efforts),
    )
    command.validate()
    return command
