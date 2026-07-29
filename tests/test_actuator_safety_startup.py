from __future__ import annotations

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
