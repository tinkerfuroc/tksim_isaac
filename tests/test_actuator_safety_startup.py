from __future__ import annotations

import time
from types import SimpleNamespace

import pytest


pytest.importorskip("rclpy")
pytest.importorskip("sensor_msgs")

from tinker_sim_bridge.command_gateway import CommandGateway  # noqa: E402
from tinker_sim_core.command_mux import CommandSource, JointCommandMux  # noqa: E402


def _gateway() -> CommandGateway:
    gateway = object.__new__(CommandGateway)
    gateway._mux = JointCommandMux(
        {"arm": CommandSource(frozenset({"joint1"}), 1.0)}
    )
    gateway._rejected = {}
    gateway._safety_active = True
    gateway._command_epoch = 0
    gateway._safety_timeout_s = 1.0
    gateway._safety_last_sample_at = None
    gateway._snapshot_id = 0
    return gateway


def test_command_gateway_rejects_before_first_effective_clear_sample() -> None:
    gateway = _gateway()
    message = SimpleNamespace(
        name=["joint1"], position=[0.4], velocity=[], effort=[]
    )

    gateway._accept("arm", message)
    assert "arm" in gateway._rejected
    assert not gateway._mux._latest

    gateway._safety_stop(SimpleNamespace(data=False))
    gateway._accept("arm", message)
    assert gateway._command_epoch == 1
    assert gateway._mux._latest["arm"][1].positions == (0.4,)


def test_command_gateway_timeout_reasserts_stop_and_invalidates_snapshots() -> None:
    gateway = _gateway()
    gateway._safety_active = False
    gateway._mux.stop(False)
    gateway._command_epoch = 1
    gateway._snapshot_id = 22
    gateway._safety_last_sample_at = 10.0

    gateway._enforce_safety_deadline(now=11.0)

    assert gateway._safety_active
    assert gateway._mux.safety_stop
    assert gateway._command_epoch == 2
    assert gateway._snapshot_id == 0
    assert gateway._rejected["safety"] == "safety heartbeat expired"


def _live_plugin_message() -> SimpleNamespace:
    """The vendored topic_based_ros2_control publishes this exact shape.

    drive_joint is intentional state-only, so names carries all eight joints
    while positions and velocities carry only the seven arm joints.
    """
    return SimpleNamespace(
        name=[f"joint{i}" for i in range(1, 8)] + ["drive_joint"],
        position=[0.1 * i for i in range(1, 8)],
        velocity=[0.2 * i for i in range(1, 8)],
        effort=[],
    )


def _cleared_gateway() -> CommandGateway:
    gateway = object.__new__(CommandGateway)
    gateway._mux = JointCommandMux(
        {
            "ros2_control": CommandSource(
                frozenset({f"joint{i}" for i in range(1, 8)}), 0.5
            ),
            "gripper": CommandSource(frozenset({"drive_joint"}), 0.5),
        }
    )
    gateway._rejected = {}
    gateway._safety_active = False
    gateway._mux.stop(False)
    gateway._safety_timeout_s = 1.0
    # Offset the last sample far into the future so the safety deadline can
    # never expire mid-test and flip _safety_active back to fail-closed.
    gateway._safety_last_sample_at = time.monotonic() + 3600.0
    gateway._snapshot_id = 0
    # object.__new__ skips Node.__init__, so _accept's rejection path cannot
    # reach a real logger; stub one so the validation result is observable.
    gateway.get_logger = lambda: SimpleNamespace(error=lambda *a, **k: None)
    return gateway


def test_command_gateway_drops_state_only_drive_joint_and_forwards_arm_commands() -> None:
    gateway = _cleared_gateway()

    gateway._accept("ros2_control", _live_plugin_message())

    assert gateway._rejected == {}
    command = gateway._mux._latest["ros2_control"][1]
    assert command.names == tuple(f"joint{i}" for i in range(1, 8))
    assert command.positions == tuple(0.1 * i for i in range(1, 8))
    assert command.velocities == tuple(0.2 * i for i in range(1, 8))


def test_command_gateway_index_filters_unowned_drive_joint_value() -> None:
    gateway = _cleared_gateway()
    # Canonical parallel arrays: value[i] belongs to name[i], so the unowned
    # drive_joint value (0.8 position, 1.6 velocity) must be dropped with the
    # name while the seven arm values are forwarded unchanged.
    message = SimpleNamespace(
        name=[f"joint{i}" for i in range(1, 8)] + ["drive_joint"],
        position=[0.1 * i for i in range(1, 9)],
        velocity=[0.2 * i for i in range(1, 9)],
        effort=[],
    )

    gateway._accept("ros2_control", message)

    assert gateway._rejected == {}
    command = gateway._mux._latest["ros2_control"][1]
    assert command.names == tuple(f"joint{i}" for i in range(1, 8))
    assert command.positions == tuple(0.1 * i for i in range(1, 8))
    assert command.velocities == tuple(0.2 * i for i in range(1, 8))


def test_command_gateway_rejects_malformed_owned_arm_shape() -> None:
    gateway = _cleared_gateway()
    message = SimpleNamespace(
        name=[f"joint{i}" for i in range(1, 8)],
        position=[0.1 * i for i in range(1, 7)],
        velocity=[],
        effort=[],
    )

    gateway._accept("ros2_control", message)

    assert "ros2_control" in gateway._rejected
    assert not gateway._mux._latest
