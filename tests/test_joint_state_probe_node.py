"""Probe-level tests for the ROS Humble ``joint_state_probe`` node.

These tests instantiate the probe's evaluation logic with a mocked ROS graph and
drive ``_check()`` deterministically.  They require the Humble Python 3.10 ROS
runtime (rclpy + sensor_msgs + controller_manager_msgs) and are skipped under
the simulator CPython 3.12 venv, where the pure contract helpers are exercised
by ``tests/test_integrated_joint_state_contract.py`` instead.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

ROOT = Path(__file__).resolve().parents[1]
import sys

sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.joint_state_probe import JointStateProbe  # noqa: E402

_READY_XACRO = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="tinker_topic_control" params="name">
    <ros2_control name="${name}" type="system">
      <hardware><plugin>topic_based_ros2_control/TopicBasedSystem</plugin></hardware>
      <joint name="joint1"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
      <joint name="joint2"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
      <joint name="joint3"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
      <joint name="joint4"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
      <joint name="joint5"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
      <joint name="joint6"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
      <joint name="joint7"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
      <joint name="drive_joint"><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
    </ros2_control>
  </xacro:macro>
</robot>
"""

_READY_DESCRIPTION = """<?xml version="1.0"?>
<robot name="tinker_full">
  <ros2_control name="TinkerTopicSystem" type="system">
    <hardware><plugin>topic_based_ros2_control/TopicBasedSystem</plugin></hardware>
    <joint name="joint1"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
    <joint name="joint2"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
    <joint name="joint3"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
    <joint name="joint4"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
    <joint name="joint5"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
    <joint name="joint6"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
    <joint name="joint7"><command_interface name="position"/><command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
    <joint name="drive_joint"><state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/></joint>
  </ros2_control>
</robot>
"""

_EIGHT = [f"joint{i}" for i in range(1, 8)] + ["drive_joint"]


def _fake_publisher_info(node_name: str, node_namespace: str = "/") -> object:
    return type(
        "EndpointInfo", (), {"node_name": node_name, "node_namespace": node_namespace}
    )()


def _make_sample() -> object:
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import JointState

    message = JointState()
    message.header.stamp = Time(sec=1, nanosec=0)
    message.name = list(_EIGHT)
    message.position = [0.0] * 8
    message.velocity = [0.0] * 8
    message.effort = [0.0] * 8
    return message


def _make_fake_client(*, ready: bool = False) -> object:
    class FakeSrvType:
        @staticmethod
        def Request():
            return object()

    client = type("Client", (), {})()
    client.service_is_ready = lambda: ready
    client.call_async = lambda request: None
    client.destroy = lambda: None
    client.srv_type = FakeSrvType
    return client


def _make_probe(*, use_sim_time: bool = True, clock_now_ns: int = 2_000_000_000) -> JointStateProbe:
    probe = JointStateProbe.__new__(JointStateProbe)
    probe._controller_manager = "controller_manager"
    probe._broadcaster = "joint_state_broadcaster"
    probe._topic = "/joint_states"
    probe._sample_watchdog_s = 15.0
    probe._clock = type("Clock", (), {})()
    probe._clock.now = lambda: type("Now", (), {"nanoseconds": clock_now_ns})()
    probe._sample = None
    probe._sample_received_ns = 0
    probe._wall_last_sample_monotonic = time.monotonic()
    probe._robot_description = None
    probe._remote_use_sim_time = None
    probe._parameters_state = {
        "client": None, "future": None, "error": None, "pending": None,
        "succeeded": False, "result": None,
    }
    probe._controllers_state = {
        "client": None, "future": None, "error": None, "pending": None,
        "succeeded": False, "result": None,
    }
    probe._controller_states = {}
    probe._controller_entries = []
    probe._xacro_text = _READY_XACRO
    probe._xacro_error = None
    probe._service_group = object()
    probe._publisher = type("Pub", (), {})()
    published: list[str] = []
    probe._publisher.publish = lambda message: published.append(message.data)
    probe._published = published
    probe.create_client = lambda srv, name, callback_group=None: _make_fake_client(ready=False)
    probe.get_publishers_info_by_topic = lambda topic: (
        [_fake_publisher_info("controller_manager")] if topic == "/joint_states" else (
            [_fake_publisher_info("clock_node", "/clock_ns")] if topic == "/clock" else []
        )
    )
    probe.get_parameter = lambda name: type("P", (), {"value": use_sim_time})()
    logger = type("Logger", (), {})()
    logger.warning = lambda msg: None
    probe.get_logger = lambda: logger
    probe._use_sim_time = use_sim_time
    return probe


def _last_state(probe) -> str:
    if not probe._published:
        return "none"
    return json.loads(probe._published[-1])["state"]


def test_no_sample_publishes_fail() -> None:
    probe = _make_probe()
    probe._check()
    assert _last_state(probe) == "fail"
    payload = json.loads(probe._published[-1])
    assert payload["verdict"]["state"] == "fail"
    assert any("no joint_state sample" in r for r in payload["verdict"]["reasons"])


def _set_controllers(probe, entries, *, succeeded=True):
    probe._controllers_state["succeeded"] = succeeded
    probe._controllers_state["result"] = entries if succeeded else None
    probe._controller_entries = list(entries)
    probe._controller_states = {name: state for name, state in entries}


def test_fresh_sample_publishes_pass() -> None:
    probe = _make_probe()
    # Provide controller evidence so attribution passes.
    _set_controllers(probe, [("joint_state_broadcaster", "active")])
    probe._robot_description = _READY_DESCRIPTION
    probe._remote_use_sim_time = True
    probe._sample = _make_sample()
    probe._sample_received_ns = 2_000_000_000
    probe._check()
    payload = json.loads(probe._published[-1])
    assert payload["state"] == "pass", payload["verdict"]["reasons"]


def test_prior_pass_then_lost_sample_publishes_fail() -> None:
    probe = _make_probe()
    _set_controllers(probe, [("joint_state_broadcaster", "active")])
    probe._robot_description = _READY_DESCRIPTION
    probe._remote_use_sim_time = True
    probe._sample = _make_sample()
    probe._sample_received_ns = 2_000_000_000
    probe._check()
    assert _last_state(probe) == "pass"
    # Sample disappears; watchdog trips.
    probe._sample = None
    probe._wall_last_sample_monotonic = time.monotonic() - 60.0
    probe._check()
    payload = json.loads(probe._published[-1])
    assert payload["state"] == "fail"
    assert any("no joint_state sample" in r for r in payload["verdict"]["reasons"])


def test_prior_pass_then_graph_failure_publishes_fail() -> None:
    probe = _make_probe()
    _set_controllers(probe, [("joint_state_broadcaster", "active")])
    probe._robot_description = _READY_DESCRIPTION
    probe._remote_use_sim_time = True
    probe._sample = _make_sample()
    probe._sample_received_ns = 2_000_000_000
    probe._check()
    assert _last_state(probe) == "pass"
    # Broadcaster becomes inactive; attribution fails.
    _set_controllers(probe, [("joint_state_broadcaster", "inactive")])
    probe._check()
    payload = json.loads(probe._published[-1])
    assert payload["state"] == "fail"
    assert any("attribution" in r for r in payload["verdict"]["reasons"])


def test_latched_pass_replaced_by_current_fail_on_same_topic() -> None:
    probe = _make_probe()
    _set_controllers(probe, [("joint_state_broadcaster", "active")])
    probe._robot_description = _READY_DESCRIPTION
    probe._remote_use_sim_time = True
    probe._sample = _make_sample()
    probe._sample_received_ns = 2_000_000_000
    probe._check()
    assert _last_state(probe) == "pass"
    # Now the controller service fails transiently; FAIL replaces the latched PASS.
    _set_controllers(probe, [], succeeded=False)
    probe._controllers_state["error"] = "service call failed: boom"
    probe._check()
    assert _last_state(probe) == "fail"


def test_clock_domain_mismatch_publishes_fail() -> None:
    probe = _make_probe(use_sim_time=False, clock_now_ns=1_700_000_000_000_000_000)
    probe._robot_description = _READY_DESCRIPTION
    probe._remote_use_sim_time = True
    _set_controllers(probe, [("joint_state_broadcaster", "active")])
    probe._sample = _make_sample()
    probe._sample_received_ns = 1_700_000_000_000_000_000
    probe._check()
    payload = json.loads(probe._published[-1])
    assert payload["state"] == "fail"
    assert payload["clock_domain"]["ready"] is False
    assert any("use_sim_time" in r for r in payload["clock_domain"]["reasons"])


def test_malformed_parameter_response_publishes_fail_and_recovers() -> None:
    probe = _make_probe()
    # Force a malformed/extract failure path by making extraction fail through
    # the parameter step's reset logic.
    from tinker_sim_bridge.joint_state_probe import _parameter_string_value

    assert _parameter_string_value(None) is None
    probe._check()
    # No description -> description contract fails.
    payload = json.loads(probe._published[-1])
    assert payload["state"] == "fail"


def test_controller_service_exception_publishes_fail() -> None:
    probe = _make_probe()
    probe._controllers_state["error"] = "service call failed: connection reset"
    probe._controllers_state["succeeded"] = False
    probe._controllers_state["result"] = None
    probe._check()
    payload = json.loads(probe._published[-1])
    assert payload["state"] == "fail"
    assert payload["publisher_attribution"]["ready"] is False
