from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.command_mux import (
    MAX_LOGICAL_SNAPSHOT_ID,
    CommandSource,
    decode_snapshot_packet,
    JointCommand,
    JointCommandMux,
    decode_command_epoch,
    decode_command_frame,
    encode_command_epoch,
    encode_command_frame,
    encode_snapshot_packet,
    new_command_session,
)


class JointCommandMuxTest(unittest.TestCase):
    def setUp(self) -> None:
        self.mux = JointCommandMux(
            {
                "base": CommandSource(frozenset({"left", "right"}), 0.25),
                "ros2_control": CommandSource(frozenset({"joint1", "joint2"}), 0.5),
            }
        )

    def test_rejects_conflicting_ownership(self) -> None:
        with self.assertRaises(ValueError):
            JointCommandMux(
                {
                    "one": CommandSource(frozenset({"joint1"}), 1.0),
                    "two": CommandSource(frozenset({"joint1"}), 1.0),
                }
            )

    def test_rejects_foreign_joint(self) -> None:
        with self.assertRaises(ValueError):
            self.mux.accept("base", JointCommand(("joint1",), velocities=(1.0,)), 1.0)

    def test_command_frame_epoch_and_snapshot_metadata_is_strict(self) -> None:
        frame_id = encode_command_frame(3, 17)
        self.assertEqual(frame_id, "tinker_command_epoch:3;snapshot:17")
        self.assertEqual(decode_command_frame(frame_id), (3, 17))
        for malformed in (
            "",
            "tinker_command_epoch:3",
            "tinker_command_epoch:3;snapshot:",
            "tinker_command_epoch:-1;snapshot:1",
            "tinker_command_epoch:03;snapshot:1",
            "tinker_command_epoch:3;snapshot:1;extra",
        ):
            with self.assertRaises(ValueError):
                decode_command_frame(malformed)

    def test_command_epoch_uses_one_gateway_session_and_generation(self) -> None:
        session = new_command_session()
        epoch = encode_command_epoch(session, 7)
        self.assertEqual(decode_command_epoch(epoch), (session, 7))
        alternate = 1 if session != 1 else 2
        self.assertNotEqual(epoch, encode_command_epoch(alternate, 7))

    def test_atomic_snapshot_metadata_round_trips_at_logical_id_boundaries(self) -> None:
        for logical_id, count, index in (
            (0, 1, 1),
            (MAX_LOGICAL_SNAPSHOT_ID, 65535, 65535),
        ):
            encoded = encode_snapshot_packet(logical_id, count, index)
            self.assertEqual(
                decode_snapshot_packet(encoded), (logical_id, count, index)
            )

        self.assertEqual(decode_snapshot_packet(17), (17, 1, 1))
        with self.assertRaises(ValueError):
            encode_snapshot_packet(MAX_LOGICAL_SNAPSHOT_ID + 1, 1, 1)
        with self.assertRaises(ValueError):
            encode_snapshot_packet(0, 65536, 1)
        with self.assertRaises(ValueError):
            encode_snapshot_packet(0, 2, 3)

    def test_merges_sources_and_stops_stale_velocity(self) -> None:
        self.mux.accept("base", JointCommand(("left", "right"), velocities=(2.0, 3.0)), 1.0)
        self.mux.accept(
            "ros2_control",
            JointCommand(("joint1", "joint2"), positions=(0.1, 0.2), velocities=(0.3, 0.4)),
            1.0,
        )
        current = self.mux.compose(1.1)
        self.assertEqual(len(current), 2)
        velocity = {
            name: value
            for packet in current
            for name, value in zip(packet.names, packet.velocities)
        }
        self.assertEqual(velocity["left"], 2.0)
        self.assertEqual(velocity["joint2"], 0.4)
        stale = self.mux.compose(1.3)
        stale_velocity = {
            name: value
            for packet in stale
            for name, value in zip(packet.names, packet.velocities)
        }
        self.assertEqual(stale_velocity["left"], 0.0)
        self.assertEqual(stale_velocity["joint1"], 0.3)

    def test_safety_stop_zeroes_motion_and_holds_positions(self) -> None:
        self.mux.accept(
            "ros2_control",
            JointCommand(("joint1", "joint2"), positions=(0.1, 0.2), velocities=(1.0, 1.0)),
            1.0,
        )
        self.mux.observe_positions(("joint1", "joint2"), (0.12, 0.18))
        self.mux.stop()
        stopped = self.mux.compose(1.01)
        self.assertEqual(len(stopped), 1)
        self.assertEqual(stopped[0].positions, (0.12, 0.18))
        self.assertEqual(stopped[0].velocities, (0.0, 0.0))
        with self.assertRaises(RuntimeError):
            self.mux.accept(
                "ros2_control",
                JointCommand(("joint1", "joint2"), positions=(0.4, 0.5)),
                1.02,
            )
        self.mux.stop(False)
        self.assertEqual(self.mux.compose(1.03), ())

    def test_mixed_interfaces_are_finite_separate_packets(self) -> None:
        self.mux.accept(
            "base", JointCommand(("left", "right"), velocities=(1.0, 1.0)), 1.0
        )
        self.mux.accept(
            "ros2_control",
            JointCommand(
                ("joint1", "joint2"),
                positions=(0.1, 0.2),
                velocities=(0.3, 0.4),
            ),
            1.0,
        )
        packets = self.mux.compose(1.1)
        self.assertEqual(len(packets), 2)
        for packet in packets:
            packet.validate()
        self.assertFalse(packets[0].positions)
        self.assertEqual(packets[1].positions, (0.1, 0.2))

    def test_base_arm_and_gripper_packets_never_use_nan(self) -> None:
        mux = JointCommandMux(
            {
                "base": CommandSource(frozenset({"left", "right"}), 0.25),
                "arm": CommandSource(frozenset({"joint1", "joint2"}), 0.5),
                "gripper": CommandSource(frozenset({"drive_joint"}), 0.5),
            }
        )
        mux.accept("base", JointCommand(("left", "right"), velocities=(1.0, -1.0)), 1.0)
        mux.accept("arm", JointCommand(("joint1", "joint2"), positions=(0.1, 0.2)), 1.0)
        mux.accept("gripper", JointCommand(("drive_joint",), positions=(0.4,), efforts=(0.0,)), 1.0)

        packets = mux.compose(1.1)

        self.assertEqual(len(packets), 3)
        for packet in packets:
            packet.validate()
            for values in (packet.positions, packet.velocities, packet.efforts):
                self.assertTrue(all(math.isfinite(value) for value in values))

    def test_stale_velocity_source_holds_measured_positions(self) -> None:
        self.mux.accept(
            "base", JointCommand(("left", "right"), velocities=(1.0, 1.0)), 1.0
        )
        self.mux.observe_positions(("left", "right"), (0.25, -0.5))

        packets = self.mux.compose(1.26)

        self.assertEqual(packets[0].positions, (0.25, -0.5))
        self.assertEqual(packets[0].velocities, (0.0, 0.0))

    def test_stale_position_command_holds_observation_not_command_target(self) -> None:
        self.mux.accept(
            "ros2_control", JointCommand(("joint1",), positions=(1.1,)), 1.0
        )
        self.mux.observe_positions(("joint1",), (0.2,))

        packets = self.mux.compose(1.51)

        self.assertEqual(packets[0].positions, (0.2,))

    def test_stale_command_without_observation_does_not_invent_a_hold(self) -> None:
        self.mux.accept(
            "ros2_control", JointCommand(("joint1",), velocities=(1.0,)), 1.0
        )

        packets = self.mux.compose(1.51)

        self.assertEqual(packets[0].positions, ())
        self.assertEqual(packets[0].velocities, (0.0,))

    def test_position_after_velocity_retires_velocity_in_active_and_stale_output(self) -> None:
        self.mux.accept(
            "ros2_control", JointCommand(("joint1",), velocities=(1.0,)), 1.0
        )
        self.mux.accept(
            "ros2_control", JointCommand(("joint1",), positions=(0.8,)), 1.1
        )
        self.mux.observe_positions(("joint1",), (0.2,))

        active = self.mux.compose(1.2)[0]
        stale = self.mux.compose(1.61)[0]

        self.assertEqual(active.positions, (0.8,))
        self.assertEqual(active.velocities, (0.0,))
        self.assertEqual(stale.positions, (0.2,))
        self.assertEqual(stale.velocities, (0.0,))

    def test_stale_hold_is_frozen_until_source_accepts_new_command(self) -> None:
        self.mux.accept(
            "base", JointCommand(("left", "right"), velocities=(1.0, 1.0)), 1.0
        )
        self.mux.observe_positions(("left", "right"), (0.25, -0.5))
        first_stale = self.mux.compose(1.26)
        self.assertEqual(first_stale[0].positions, (0.25, -0.5))

        self.mux.observe_positions(("left", "right"), (0.75, -0.8))
        later_stale = self.mux.compose(1.4)
        self.assertEqual(later_stale[0].positions, (0.25, -0.5))

        self.mux.accept(
            "base", JointCommand(("left", "right"), velocities=(2.0, 2.0)), 1.5
        )
        self.mux.observe_positions(("left", "right"), (0.9, -1.0))
        refreshed_stale = self.mux.compose(1.76)
        self.assertEqual(refreshed_stale[0].positions, (0.9, -1.0))

    def test_clear_does_not_resume_pre_stop_commands(self) -> None:
        self.mux.accept(
            "ros2_control", JointCommand(("joint1",), positions=(0.2,)), 1.0
        )
        self.mux.stop()
        self.mux.stop(False)

        self.assertEqual(self.mux.compose(1.01), ())
        self.mux.accept(
            "ros2_control", JointCommand(("joint1",), positions=(0.3,)), 1.02
        )
        self.assertEqual(self.mux.compose(1.03)[0].positions, (0.3,))

    def test_stop_holds_observed_source_without_latest_command(self) -> None:
        self.mux.observe_positions(("left", "right"), (0.4, -0.2))
        self.mux.stop()

        self.assertEqual(self.mux.compose(1.0)[0].names, ("left", "right"))
        self.assertEqual(self.mux.compose(1.0)[0].positions, (0.4, -0.2))


if __name__ == "__main__":
    unittest.main()
