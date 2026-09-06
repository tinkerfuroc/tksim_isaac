"""Task 4: integrated eight-joint state contract.

The integrated ``/joint_states`` contract is exactly eight joints
(``joint1``..``joint7`` + ``drive_joint``), where ``drive_joint`` is state-only
(``position``/``velocity``/``effort``, zero command interfaces) in both the
checked-in xacro and the live controller description produced by
``tinker_sim_deploy.runtime.topic_control_description``.

The contract tests exercise the real complete robot URDF through the live
runtime transformer (never a source-text count or a synthetic dead helper), then
evaluate the same drive_joint contract through the pure ROS-free helpers that
the ROS Humble live probe also uses.
"""
from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.contract_guard import (  # noqa: E402
    INTEGRATED_JOINT_STATE_NAMES,
    JOINT_STATE_BROADCASTER,
    JOINT_STATE_CLOCK_DOMAIN_THRESHOLD_NS,
    derive_logical_joint_state_publishers,
    evaluate_clock_domain,
    evaluate_integrated_cardinality,
    evaluate_joint_state_evidence_pair,
    evaluate_joint_state_sample,
    evaluate_probe_verdict,
    evaluate_robot_description_contract,
    evaluate_sample_freshness,
    evaluate_xacro_contract,
    step_service,
)
from tinker_sim_deploy.runtime import (  # noqa: E402
    resolve_current_artifact,
    topic_control_description,
)
from tinker_sim_deploy.workspace import canonicalize_urdf  # noqa: E402


def _source_robot_urdf() -> bytes:
    """Build a complete robot URDF mirroring the production Tinker 2 source.

    The graph is a full actuated robot: world/base/arm links, arm joints
    ``joint1``..``joint7``, a gripper mount, the physical ``drive_joint``, a
    pre-existing ``ros2_control`` block (which the runtime transformer must
    replace), transmissions, and root metadata.
    """
    root = ET.Element("robot", {"name": "tinker_full"})
    for name in [
        "base_link",
        "link_base",
        *[f"link{index}" for index in range(1, 8)],
        "gripper_base",
        "left_outer",
    ]:
        link = ET.SubElement(root, "link", {"name": name})
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass", {"value": "1.0"})
        visual = ET.SubElement(link, "visual")
        ET.SubElement(visual, "geometry").append(ET.Element("box", {"size": "1 2 3"}))
        collision = ET.SubElement(link, "collision")
        ET.SubElement(collision, "geometry").append(ET.Element("box", {"size": "1 2 3"}))

    mount = ET.SubElement(root, "joint", {"name": "world_joint", "type": "fixed"})
    ET.SubElement(mount, "parent", {"link": "base_link"})
    ET.SubElement(mount, "child", {"link": "link_base"})
    ET.SubElement(mount, "origin", {"xyz": "-0.03 0 0.527", "rpy": "0 0 0"})

    parent = "link_base"
    for index in range(1, 8):
        joint = ET.SubElement(root, "joint", {"name": f"joint{index}", "type": "revolute"})
        ET.SubElement(joint, "parent", {"link": parent})
        child = f"link{index}"
        ET.SubElement(joint, "child", {"link": child})
        ET.SubElement(joint, "axis", {"xyz": "0 0 1"})
        ET.SubElement(joint, "limit", {"lower": "-1", "upper": "1", "effort": "2", "velocity": "3"})
        parent = child

    fixed = ET.SubElement(root, "joint", {"name": "gripper_mount", "type": "fixed"})
    ET.SubElement(fixed, "parent", {"link": "link7"})
    ET.SubElement(fixed, "child", {"link": "gripper_base"})
    drive = ET.SubElement(root, "joint", {"name": "drive_joint", "type": "revolute"})
    ET.SubElement(drive, "parent", {"link": "gripper_base"})
    ET.SubElement(drive, "child", {"link": "left_outer"})
    ET.SubElement(drive, "axis", {"xyz": "1 0 0"})
    ET.SubElement(drive, "limit", {"lower": "0", "upper": "1", "effort": "2", "velocity": "3"})

    control = ET.SubElement(root, "ros2_control", {"name": "source-system", "type": "system"})
    hardware = ET.SubElement(control, "hardware")
    ET.SubElement(hardware, "plugin").text = "uf_robot_hardware/UFRobotSystemHardware"
    ET.SubElement(hardware, "param", {"name": "add_gripper"}).text = "False"
    for name in [f"joint{index}" for index in range(1, 8)]:
        joint = ET.SubElement(control, "joint", {"name": name})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        ET.SubElement(joint, "command_interface", {"name": "velocity"})
        ET.SubElement(joint, "state_interface", {"name": "position"})
        ET.SubElement(joint, "state_interface", {"name": "velocity"})
    transmission = ET.SubElement(root, "transmission", {"name": "fixture-transmission"})
    ET.SubElement(transmission, "type").text = "fixture/SimpleTransmission"
    ET.SubElement(root, "root_metadata").text = "root text"
    return ET.tostring(root, encoding="utf-8")


def _canonical_robot_urdf() -> bytes:
    """Produce the canonical robot URDF exactly as the artifact pipeline does."""
    return canonicalize_urdf(_source_robot_urdf())


def _live_description() -> str:
    """Produce the live controller description from the complete real URDF."""
    return topic_control_description(_canonical_robot_urdf())


def _checked_in_xacro() -> str:
    path = ROOT / "ros2_ws/src/tinker_sim_bridge/config/tinker_topic_control.ros2_control.xacro"
    return path.read_text(encoding="utf-8")


# ---------------------------------------------------------------------------
# Live runtime transformer on the complete real robot URDF
# ---------------------------------------------------------------------------


def test_topic_control_description_has_exactly_one_state_only_drive_joint() -> None:
    root = ET.fromstring(_live_description())
    controls = root.findall("ros2_control")
    assert len(controls) == 1
    joints = controls[0].findall("joint")
    names = [joint.get("name") for joint in joints]
    assert names == [f"joint{index}" for index in range(1, 8)] + ["drive_joint"]
    drives = [joint for joint in joints if joint.get("name") == "drive_joint"]
    assert len(drives) == 1
    drive = drives[0]
    assert [item.get("name") for item in drive.findall("command_interface")] == []
    assert [item.get("name") for item in drive.findall("state_interface")] == [
        "position",
        "velocity",
        "effort",
    ]


def mock_request(client):
    return client.srv_type.Request()


class _FakeFuture:
    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done

    def done(self) -> bool:
        return self._done

    def result(self):
        if isinstance(self._result, Exception):
            raise self._result
        return self._result


def mock_client(*, ready: bool, result="payload", done: bool = True):
    """Build a fake ROS service client exposing the step_service contract."""
    class FakeSrvType:
        @staticmethod
        def Request():
            return object()

    client = type("FakeClient", (), {})()
    client.service_is_ready = lambda: ready
    client.srv_type = FakeSrvType
    client.future = _FakeFuture(result=result, done=done)
    client.call_async = lambda request: client.future
    client.destroy = lambda: None
    return client


def test_topic_control_description_keeps_arm_joint_interfaces() -> None:
    root = ET.fromstring(_live_description())
    control = root.find("ros2_control")
    assert control is not None
    for index in range(1, 8):
        name = f"joint{index}"
        joint = next(item for item in control.findall("joint") if item.get("name") == name)
        assert [item.get("name") for item in joint.findall("command_interface")] == [
            "position",
            "velocity",
        ]
        assert [item.get("name") for item in joint.findall("state_interface")] == [
            "position",
            "velocity",
            "effort",
        ]


def test_topic_control_description_preserves_topic_hardware_contract() -> None:
    root = ET.fromstring(_live_description())
    control = root.find("ros2_control")
    assert control is not None
    assert control.get("name") == "TinkerTopicSystem"
    plugin = control.findtext("hardware/plugin")
    assert plugin == "topic_based_ros2_control/TopicBasedSystem"
    params = {param.get("name"): param.text for param in control.findall("hardware/param")}
    assert params["joint_commands_topic"] == "/sim/controller/ros2_control_commands"
    assert params["joint_states_topic"] == "/isaac_joint_states"
    assert params["trigger_joint_command_threshold"] == "-1"


# ---------------------------------------------------------------------------
# evaluate_robot_description_contract (live controller_manager parameter)
# ---------------------------------------------------------------------------


def test_robot_description_contract_ready_for_live_description() -> None:
    result = evaluate_robot_description_contract(_live_description())
    assert result["ready"] is True, result["reasons"]
    assert result["reasons"] == []
    observed = result["observed"]
    assert observed["ros2_control_joint_names"] == list(INTEGRATED_JOINT_STATE_NAMES)
    assert observed["drive_joint"] == {
        "command_interfaces": [],
        "state_interfaces": ["position", "velocity", "effort"],
    }


def test_robot_description_contract_rejects_missing_drive_joint() -> None:
    description = ET.fromstring(_live_description())
    control = description.find("ros2_control")
    assert control is not None
    for joint in control.findall("joint"):
        if joint.get("name") == "drive_joint":
            control.remove(joint)
    result = evaluate_robot_description_contract(ET.tostring(description, encoding="unicode"))
    assert result["ready"] is False
    assert any("drive_joint" in reason for reason in result["reasons"])
    assert result["observed"]["drive_joint"] is None


def test_robot_description_contract_rejects_drive_joint_command_interface() -> None:
    description = ET.fromstring(_live_description())
    control = description.find("ros2_control")
    assert control is not None
    drive = next(item for item in control.findall("joint") if item.get("name") == "drive_joint")
    ET.SubElement(drive, "command_interface", {"name": "position"})
    result = evaluate_robot_description_contract(ET.tostring(description, encoding="unicode"))
    assert result["ready"] is False
    assert any("command interfaces" in reason for reason in result["reasons"])


def test_robot_description_contract_rejects_malformed_xml() -> None:
    result = evaluate_robot_description_contract("<robot")
    assert result["ready"] is False
    assert any("not well-formed" in reason for reason in result["reasons"])


def test_robot_description_contract_rejects_wrong_joint_names() -> None:
    description = ET.fromstring(_live_description())
    control = description.find("ros2_control")
    assert control is not None
    drive = next(item for item in control.findall("joint") if item.get("name") == "drive_joint")
    drive.set("name", "wrong_joint")
    result = evaluate_robot_description_contract(ET.tostring(description, encoding="unicode"))
    assert result["ready"] is False
    assert any("joint names" in reason for reason in result["reasons"])


# ---------------------------------------------------------------------------
# evaluate_xacro_contract (checked-in source)
# ---------------------------------------------------------------------------


def test_xacro_contract_ready_for_checked_in_source() -> None:
    result = evaluate_xacro_contract(_checked_in_xacro())
    assert result["ready"] is True, result["reasons"]
    assert result["observed"]["drive_joint"] == {
        "command_interfaces": [],
        "state_interfaces": ["position", "velocity", "effort"],
    }
    assert result["observed"]["ros2_control_joint_names"] == list(INTEGRATED_JOINT_STATE_NAMES)


def test_xacro_contract_rejects_missing_drive_joint() -> None:
    text = _checked_in_xacro().replace("<joint name=\"drive_joint\">", "<joint name=\"removed_joint\">")
    result = evaluate_xacro_contract(text)
    assert result["ready"] is False
    assert any("drive_joint" in reason for reason in result["reasons"])


def test_xacro_contract_rejects_malformed_xml() -> None:
    result = evaluate_xacro_contract("<robot")
    assert result["ready"] is False


# ---------------------------------------------------------------------------
# Source-xacro and live-parameter evidence compared together
# ---------------------------------------------------------------------------


def test_evidence_pair_ready_when_xacro_and_live_agree() -> None:
    result = evaluate_joint_state_evidence_pair(
        xacro_contract=evaluate_xacro_contract(_checked_in_xacro()),
        description_contract=evaluate_robot_description_contract(_live_description()),
    )
    assert result["ready"] is True, result["reasons"]
    assert result["observed"]["xacro_drive_joint"] == result["observed"]["description_drive_joint"]


def test_evidence_pair_rejects_live_drive_joint_command_interface() -> None:
    description = ET.fromstring(_live_description())
    control = description.find("ros2_control")
    assert control is not None
    drive = next(item for item in control.findall("joint") if item.get("name") == "drive_joint")
    ET.SubElement(drive, "command_interface", {"name": "position"})
    broken = evaluate_robot_description_contract(ET.tostring(description, encoding="unicode"))
    result = evaluate_joint_state_evidence_pair(
        xacro_contract=evaluate_xacro_contract(_checked_in_xacro()),
        description_contract=broken,
    )
    assert result["ready"] is False
    assert any("not ready" in reason for reason in result["reasons"])


# ---------------------------------------------------------------------------
# evaluate_integrated_cardinality (graph metadata)
# ---------------------------------------------------------------------------


def test_integrated_cardinality_ready_for_single_broadcaster() -> None:
    result = evaluate_integrated_cardinality(joint_state_publishers=[JOINT_STATE_BROADCASTER])
    assert result["ready"] is True, result["reasons"]
    assert result["observed"]["joint_state_publisher_count"] == 1
    assert result["observed"]["joint_state_publisher_source"] == JOINT_STATE_BROADCASTER
    assert result["observed"]["joint_state_publishers"] == [JOINT_STATE_BROADCASTER]


def test_integrated_cardinality_rejects_zero_publishers() -> None:
    result = evaluate_integrated_cardinality(joint_state_publishers=[])
    assert result["ready"] is False
    assert result["observed"]["joint_state_publisher_count"] == 0
    assert result["observed"]["joint_state_publisher_source"] is None


def test_integrated_cardinality_rejects_duplicate_publishers() -> None:
    result = evaluate_integrated_cardinality(
        joint_state_publishers=[JOINT_STATE_BROADCASTER, JOINT_STATE_BROADCASTER]
    )
    assert result["ready"] is False
    assert result["observed"]["joint_state_publisher_count"] == 2


def test_integrated_cardinality_rejects_wrong_source() -> None:
    result = evaluate_integrated_cardinality(joint_state_publishers=["/controller_manager"])
    assert result["ready"] is False
    assert any("source" in reason for reason in result["reasons"])
    assert result["observed"]["joint_state_publisher_source"] == "/controller_manager"


def test_integrated_cardinality_reports_exact_observed_values() -> None:
    result = evaluate_integrated_cardinality(joint_state_publishers=[JOINT_STATE_BROADCASTER])
    assert result["observed"] == {
        "joint_state_publishers": [JOINT_STATE_BROADCASTER],
        "joint_state_publisher_count": 1,
        "joint_state_publisher_source": JOINT_STATE_BROADCASTER,
    }


# ---------------------------------------------------------------------------
# evaluate_joint_state_sample (actual JointState content)
# ---------------------------------------------------------------------------

_ARM = [f"joint{index}" for index in range(1, 8)]
_EIGHT = _ARM + ["drive_joint"]
_ZERO_EIGHT = [0.0] * 8


def _ready_sample(**overrides) -> dict[str, object]:
    values: dict[str, object] = {
        "publisher_node": JOINT_STATE_BROADCASTER,
        "publisher_count": 1,
        "names": _EIGHT,
        "positions": list(_ZERO_EIGHT),
        "velocities": list(_ZERO_EIGHT),
        "header_stamp_ns": 1_000_000_000,
        "received_at_ns": 1_500_000_000,
        "now_ns": 2_000_000_000,
    }
    values.update(overrides)
    return evaluate_joint_state_sample(**values)


def test_joint_state_sample_ready_for_exact_contract() -> None:
    result = _ready_sample()
    assert result["ready"] is True, result["reasons"]
    assert result["reasons"] == []
    observed = result["observed"]
    assert observed["names"] == _EIGHT
    assert observed["age_ns"] == 1_000_000_000
    assert observed["transport_ns"] == 500_000_000


def test_joint_state_sample_rejects_wrong_names() -> None:
    result = _ready_sample(names=_ARM)
    assert result["ready"] is False
    assert any("joint names" in reason for reason in result["reasons"])


def test_joint_state_sample_rejects_wrong_name_order() -> None:
    result = _ready_sample(names=["drive_joint", *_ARM])
    assert result["ready"] is False
    assert any("joint names" in reason for reason in result["reasons"])


def test_joint_state_sample_rejects_nonfinite_positions() -> None:
    positions = list(_ZERO_EIGHT)
    positions[0] = float("nan")
    result = _ready_sample(positions=positions)
    assert result["ready"] is False
    assert any("positions" in reason for reason in result["reasons"])


def test_joint_state_sample_rejects_nonfinite_velocities() -> None:
    velocities = list(_ZERO_EIGHT)
    velocities[3] = float("inf")
    result = _ready_sample(velocities=velocities)
    assert result["ready"] is False
    assert any("velocities" in reason for reason in result["reasons"])


def test_joint_state_sample_rejects_wrong_array_length() -> None:
    result = _ready_sample(positions=[0.0] * 7)
    assert result["ready"] is False
    assert any("positions length" in reason for reason in result["reasons"])


def test_joint_state_sample_rejects_zero_header_stamp() -> None:
    result = _ready_sample(header_stamp_ns=0)
    assert result["ready"] is False
    assert any("zero" in reason for reason in result["reasons"])


def test_joint_state_sample_rejects_stale_age() -> None:
    result = _ready_sample(now_ns=10_000_000_000)
    assert result["ready"] is False
    assert any("stale" in reason for reason in result["reasons"])
    assert result["observed"]["age_ns"] == 9_000_000_000


def test_joint_state_sample_rejects_future_stamp() -> None:
    result = _ready_sample(now_ns=500_000_000)
    assert result["ready"] is False
    assert any("future" in reason for reason in result["reasons"])


def test_joint_state_sample_rejects_wrong_publisher() -> None:
    result = _ready_sample(publisher_node="/controller_manager")
    assert result["ready"] is False
    assert any("publisher" in reason for reason in result["reasons"])


def test_joint_state_sample_rejects_wrong_publisher_count() -> None:
    result = _ready_sample(publisher_count=2)
    assert result["ready"] is False
    assert any("publisher count" in reason for reason in result["reasons"])


def test_joint_state_sample_rejects_negative_transport() -> None:
    result = _ready_sample(received_at_ns=500_000_000)
    assert result["ready"] is False
    assert any("transport" in reason for reason in result["reasons"])


def test_joint_state_sample_reports_complete_observed_mapping() -> None:
    result = _ready_sample()
    assert set(result["observed"]) == {
        "publisher_node",
        "publisher_count",
        "names",
        "positions",
        "velocities",
        "header_stamp_ns",
        "received_at_ns",
        "now_ns",
        "age_ns",
        "transport_ns",
    }
    assert all(math.isfinite(value) for value in result["observed"]["positions"])
    assert all(math.isfinite(value) for value in result["observed"]["velocities"])
    assert result["observed"]["header_stamp_ns"] != 0


# ---------------------------------------------------------------------------
# Real selected production artifact (must not skip when artifacts are provisioned)
# ---------------------------------------------------------------------------


def test_real_selected_artifact_state_only_drive_joint_contract() -> None:
    """Feed the actual selected ``robot.urdf`` through the live transformer.

    Resolves ``artifacts/robot/tinker2/current.json`` through the authoritative
    Task 3 resolver and asserts the exact eight-joint ordering plus the single
    state-only ``drive_joint``.  Skips only when the gitignored artifact tree is
    not provisioned in this checkout.
    """
    try:
        resolved = resolve_current_artifact(ROOT)
    except Exception as exc:  # noqa: BLE001 - artifact tree absent in some checkouts
        pytest.skip(
            "real Tinker 2 artifact is not provisioned in this checkout: {}".format(exc)
        )
    urdf = resolved.robot_urdf
    assert urdf
    description = topic_control_description(urdf)
    result = evaluate_robot_description_contract(description)
    assert result["ready"], result["reasons"]
    assert result["observed"]["ros2_control_joint_names"] == list(
        INTEGRATED_JOINT_STATE_NAMES
    )
    assert result["observed"]["drive_joint"] == {
        "command_interfaces": [],
        "state_interfaces": ["position", "velocity", "effort"],
    }
    root = ET.fromstring(description)
    control = root.find("ros2_control")
    assert control is not None
    drives = [joint for joint in control.findall("joint") if joint.get("name") == "drive_joint"]
    assert len(drives) == 1
    assert [item.get("name") for item in drives[0].findall("command_interface")] == []
    assert [item.get("name") for item in drives[0].findall("state_interface")] == [
        "position",
        "velocity",
        "effort",
    ]


# ---------------------------------------------------------------------------
# Clock-domain helpers
# ---------------------------------------------------------------------------


def test_clock_domain_ready_for_matching_sim_time() -> None:
    result = evaluate_clock_domain(
        local_use_sim_time=True,
        remote_use_sim_time=True,
        sim_clock_active=True,
        clock_now_ns=3_000_000_000,
    )
    assert result["ready"] is True, result["reasons"]
    assert result["observed"]["clock_domain"] == "sim"


def test_clock_domain_rejects_wall_vs_sim_mismatch() -> None:
    result = evaluate_clock_domain(
        local_use_sim_time=False,
        remote_use_sim_time=True,
        sim_clock_active=False,
        clock_now_ns=1_700_000_000_000_000_000,
    )
    assert result["ready"] is False
    assert any("use_sim_time" in reason for reason in result["reasons"])


def test_clock_domain_rejects_unknown_remote() -> None:
    result = evaluate_clock_domain(
        local_use_sim_time=True,
        remote_use_sim_time=None,
        sim_clock_active=True,
        clock_now_ns=3_000_000_000,
    )
    assert result["ready"] is False
    assert any("unknown" in reason for reason in result["reasons"])


def test_clock_domain_rejects_zero_not_started_sim_clock() -> None:
    result = evaluate_clock_domain(
        local_use_sim_time=True,
        remote_use_sim_time=True,
        sim_clock_active=True,
        clock_now_ns=0,
    )
    assert result["ready"] is False
    assert any("past zero" in reason for reason in result["reasons"])


def test_clock_domain_rejects_missing_clock_publisher() -> None:
    result = evaluate_clock_domain(
        local_use_sim_time=True,
        remote_use_sim_time=True,
        sim_clock_active=False,
        clock_now_ns=3_000_000_000,
    )
    assert result["ready"] is False
    assert any("/clock" in reason for reason in result["reasons"])


def test_clock_domain_rejects_no_sample_received_yet() -> None:
    # Task #21: the readiness gate must distinguish "no clock sample
    # received" (None) from "value is zero" -- an epoch-anchored /clock
    # (TINKER_SIM_CLOCK_EPOCH default wall-clock) never legitimately
    # publishes exactly 0, so None is the only way a caller that knows it
    # has never received a sample can express that.
    result = evaluate_clock_domain(
        local_use_sim_time=True,
        remote_use_sim_time=True,
        sim_clock_active=True,
        clock_now_ns=None,
    )
    assert result["ready"] is False
    assert any(
        "no clock sample has been received yet" in reason
        for reason in result["reasons"]
    )


def test_clock_domain_ready_for_large_boot_epoch_value() -> None:
    # A valid large epoch value (e.g. a wall-clock TINKER_SIM_CLOCK_EPOCH
    # anchor, ~1.7e18 ns for a 2026 wall-clock second count) is ready, same
    # as any other nonzero sample.
    result = evaluate_clock_domain(
        local_use_sim_time=True,
        remote_use_sim_time=True,
        sim_clock_active=True,
        clock_now_ns=1_798_000_000_000_000_000,
    )
    assert result["ready"] is True, result["reasons"]


# ---------------------------------------------------------------------------
# evaluate_probe_verdict (fail-closed aggregation seam)
# ---------------------------------------------------------------------------


def _verdict(**flags) -> dict[str, object]:
    defaults = {
        "sample_ready": True,
        "sample_reasons": [],
        "cardinality_ready": True,
        "attribution_ready": True,
        "description_ready": True,
        "xacro_ready": True,
        "evidence_pair_ready": True,
        "clock_domain_ready": True,
    }
    defaults.update(flags)
    return evaluate_probe_verdict(**defaults)


def test_verdict_no_sample_is_fail() -> None:
    result = _verdict(sample_ready=False, sample_reasons=["no joint_state sample received yet"])
    assert result["state"] == "fail"
    assert any("sample" in reason for reason in result["reasons"])


def test_verdict_fresh_valid_sample_is_pass() -> None:
    result = _verdict()
    assert result["state"] == "pass"
    assert result["reasons"] == []


def test_verdict_stale_sample_after_prior_pass_is_fail() -> None:
    result = _verdict(
        sample_ready=False,
        sample_reasons=["header stamp is stale by 9000000000 ns (bound 5000000000)"],
    )
    assert result["state"] == "fail"


def test_verdict_graph_failure_after_prior_pass_is_fail() -> None:
    result = _verdict(cardinality_ready=False)
    assert result["state"] == "fail"
    assert any("cardinality" in reason for reason in result["reasons"])


def test_verdict_attribution_failure_is_fail() -> None:
    result = _verdict(attribution_ready=False)
    assert result["state"] == "fail"
    assert any("attribution" in reason for reason in result["reasons"])


def test_verdict_clock_domain_failure_is_fail() -> None:
    result = _verdict(clock_domain_ready=False)
    assert result["state"] == "fail"
    assert any("clock" in reason for reason in result["reasons"])


def test_verdict_reports_observed_flags() -> None:
    result = _verdict(sample_ready=False)
    assert result["observed"]["sample_ready"] is False
    assert result["observed"]["cardinality_ready"] is True


# ---------------------------------------------------------------------------
# evaluate_sample_freshness (watchdog)
# ---------------------------------------------------------------------------


def test_freshness_ready_for_recent_sample() -> None:
    result = evaluate_sample_freshness(
        sample_present=True, wall_age_s=1.0, wall_watchdog_s=15.0
    )
    assert result["ready"] is True, result["reasons"]


def test_freshness_rejects_missing_sample() -> None:
    result = evaluate_sample_freshness(
        sample_present=False, wall_age_s=None, wall_watchdog_s=15.0
    )
    assert result["ready"] is False
    assert any("no joint_state sample" in reason for reason in result["reasons"])


def test_freshness_rejects_stalled_sample_after_prior_sample() -> None:
    result = evaluate_sample_freshness(
        sample_present=True, wall_age_s=30.0, wall_watchdog_s=15.0
    )
    assert result["ready"] is False
    assert any("no new joint_state sample" in reason for reason in result["reasons"])


# ---------------------------------------------------------------------------
# evaluate_joint_state_sample clock-domain mismatch diagnostic
# ---------------------------------------------------------------------------


def test_sample_reports_epoch_scale_clock_domain_mismatch() -> None:
    result = _ready_sample(
        header_stamp_ns=500_000_000,
        now_ns=1_700_000_000_000_000_000,
        received_at_ns=1_700_000_000_000_000_000,
    )
    assert result["ready"] is False
    assert any("clock-domain mismatch" in reason for reason in result["reasons"])
    assert any("stale" in reason for reason in result["reasons"])
    assert any("transport latency" in reason for reason in result["reasons"])
    assert abs(result["observed"]["age_ns"]) > JOINT_STATE_CLOCK_DOMAIN_THRESHOLD_NS


# ---------------------------------------------------------------------------
# derive_logical_joint_state_publishers (proven attribution)
# ---------------------------------------------------------------------------


def test_attribution_controller_manager_hosted_active_broadcaster() -> None:
    labels, reasons = derive_logical_joint_state_publishers(
        raw_labels=["/controller_manager"],
        controller_manager="controller_manager",
        broadcaster_controller="joint_state_broadcaster",
        controller_entries=[("joint_state_broadcaster", "active")],
    )
    assert labels == ["joint_state_broadcaster"]
    assert reasons == []


def test_attribution_standalone_exact_broadcaster() -> None:
    labels, reasons = derive_logical_joint_state_publishers(
        raw_labels=["/joint_state_broadcaster"],
        controller_manager="controller_manager",
        broadcaster_controller="joint_state_broadcaster",
        controller_entries=[],
    )
    assert labels == ["joint_state_broadcaster"]
    assert reasons == []


def test_attribution_missing_broadcaster_controller() -> None:
    labels, reasons = derive_logical_joint_state_publishers(
        raw_labels=["/controller_manager"],
        controller_manager="controller_manager",
        broadcaster_controller="joint_state_broadcaster",
        controller_entries=[],
    )
    assert labels == ["/controller_manager"]
    assert any("cannot be attributed" in reason for reason in reasons)


def test_attribution_inactive_broadcaster_controller() -> None:
    labels, reasons = derive_logical_joint_state_publishers(
        raw_labels=["/controller_manager"],
        controller_manager="controller_manager",
        broadcaster_controller="joint_state_broadcaster",
        controller_entries=[("joint_state_broadcaster", "inactive")],
    )
    assert labels == ["/controller_manager"]
    assert any("cannot be attributed" in reason for reason in reasons)


def test_attribution_renamed_broadcaster_controller() -> None:
    labels, reasons = derive_logical_joint_state_publishers(
        raw_labels=["/controller_manager"],
        controller_manager="controller_manager",
        broadcaster_controller="joint_state_broadcaster",
        controller_entries=[("joint_state_broadcaster2", "active")],
    )
    assert labels == ["/controller_manager"]
    assert any("cannot be attributed" in reason for reason in reasons)


def test_attribution_duplicate_broadcaster_controllers() -> None:
    labels, reasons = derive_logical_joint_state_publishers(
        raw_labels=["/controller_manager"],
        controller_manager="controller_manager",
        broadcaster_controller="joint_state_broadcaster",
        controller_entries=[
            ("joint_state_broadcaster", "active"),
            ("joint_state_broadcaster", "active"),
        ],
    )
    assert labels == ["/controller_manager"]
    assert any("cannot be attributed" in reason for reason in reasons)


def test_attribution_namespaced_node_not_converted() -> None:
    labels, reasons = derive_logical_joint_state_publishers(
        raw_labels=["/some_ns/joint_state_broadcaster"],
        controller_manager="controller_manager",
        broadcaster_controller="joint_state_broadcaster",
        controller_entries=[],
    )
    assert labels == ["/some_ns/joint_state_broadcaster"]
    assert reasons == []


def test_attribution_multiple_raw_publishers_preserved() -> None:
    labels, reasons = derive_logical_joint_state_publishers(
        raw_labels=["/controller_manager", "/other"],
        controller_manager="controller_manager",
        broadcaster_controller="joint_state_broadcaster",
        controller_entries=[("joint_state_broadcaster", "active")],
    )
    assert labels == ["joint_state_broadcaster", "/other"]
    assert reasons == []


# ---------------------------------------------------------------------------
# step_service (bounded, recoverable async service state machine)
# ---------------------------------------------------------------------------


def _make_service_state() -> dict[str, object]:
    return {
        "client": None,
        "future": None,
        "error": None,
        "pending": None,
        "succeeded": False,
        "result": None,
        "succeeded_at": None,
        "started_at": None,
    }


def test_step_service_success_path() -> None:
    state = _make_service_state()
    client = mock_client(ready=True)
    step_service(
        state,
        create_client=lambda: client,
        request=lambda c: mock_request(c),
        extract=lambda response: {"ok": response} if response == "payload" else None,
        reset_client=lambda s: s.update(client=None, future=None),
    )
    # First call discovers service and issues request.
    assert state["pending"] is None
    assert state["future"] is client.future
    step_service(
        state,
        create_client=lambda: client,
        request=lambda c: mock_request(c),
        extract=lambda response: {"ok": response} if response == "payload" else None,
        reset_client=lambda s: s.update(client=None, future=None),
    )
    assert state["succeeded"] is True
    assert state["result"] == {"ok": "payload"}


def test_step_service_service_not_ready_pending() -> None:
    state = _make_service_state()
    client = mock_client(ready=False)
    step_service(
        state,
        create_client=lambda: client,
        request=lambda c: mock_request(c),
        extract=lambda response: response,
        reset_client=lambda s: s.update(client=None, future=None),
    )
    assert state["succeeded"] is False
    assert state["pending"] == "service not ready"


def test_step_service_in_flight_pending() -> None:
    state = _make_service_state()
    client = mock_client(ready=True, done=False)
    step_service(
        state,
        create_client=lambda: client,
        request=lambda c: mock_request(c),
        extract=lambda response: response,
        reset_client=lambda s: s.update(client=None, future=None),
    )
    step_service(
        state,
        create_client=lambda: client,
        request=lambda c: mock_request(c),
        extract=lambda response: response,
        reset_client=lambda s: s.update(client=None, future=None),
    )
    assert state["succeeded"] is False
    assert state["pending"] == "request in flight"


def test_step_service_exception_recovers_and_succeeds() -> None:
    state = _make_service_state()
    first = mock_client(ready=True, result=RuntimeError("boom"))
    second = mock_client(ready=True, result="payload")
    sequence = iter([first, second])

    def create():
        return next(sequence)

    def extract(response):
        return response

    reset_client = lambda s: s.update(client=None, future=None)
    step_service(state, create_client=create, request=lambda c: mock_request(c), extract=extract, reset_client=reset_client)
    step_service(state, create_client=create, request=lambda c: mock_request(c), extract=extract, reset_client=reset_client)
    assert state["succeeded"] is False
    assert "service call failed" in str(state["error"])
    # Recovery on the next client.
    step_service(state, create_client=create, request=lambda c: mock_request(c), extract=extract, reset_client=reset_client)
    step_service(state, create_client=create, request=lambda c: mock_request(c), extract=extract, reset_client=reset_client)
    assert state["succeeded"] is True
    assert state["result"] == "payload"


def test_step_service_malformed_response_recovers() -> None:
    state = _make_service_state()
    client = mock_client(ready=True, result="malformed")
    step_service(
        state,
        create_client=lambda: client,
        request=lambda c: mock_request(c),
        extract=lambda response: None if response == "malformed" else response,
        reset_client=lambda s: s.update(client=None, future=None),
    )
    step_service(
        state,
        create_client=lambda: client,
        request=lambda c: mock_request(c),
        extract=lambda response: None if response == "malformed" else response,
        reset_client=lambda s: s.update(client=None, future=None),
    )
    assert state["succeeded"] is False
    assert "no usable response" in str(state["error"])


def test_step_service_repeated_failures_never_raise() -> None:
    state = _make_service_state()
    for _ in range(50):
        client = mock_client(ready=True, result=RuntimeError("flaky"))
        step_service(
            state,
            create_client=lambda: client,
            request=lambda c: mock_request(c),
            extract=lambda response: response,
            reset_client=lambda s: s.update(client=None, future=None),
        )
    assert state["succeeded"] is False
    assert "service call failed" in str(state["error"])


def test_step_service_extract_never_crashes_callback() -> None:
    state = _make_service_state()

    def bad_extract(_response):
        raise ValueError("parse error")

    client = mock_client(ready=True, result="anything")
    step_service(
        state,
        create_client=lambda: client,
        request=lambda c: mock_request(c),
        extract=bad_extract,
        reset_client=lambda s: s.update(client=None, future=None),
    )
    step_service(
        state,
        create_client=lambda: client,
        request=lambda c: mock_request(c),
        extract=bad_extract,
        reset_client=lambda s: s.update(client=None, future=None),
    )
    assert state["succeeded"] is False
    assert "malformed" in str(state["error"])


# ---------------------------------------------------------------------------
# step_service TTL / freshness of successful evidence
# ---------------------------------------------------------------------------


def test_step_service_success_stays_fresh_within_ttl() -> None:
    state = _make_service_state()
    client = mock_client(ready=True, result="payload")
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=100.0, ttl_s=30.0, timeout_s=5.0,
    )
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=100.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is True
    assert state["result"] == "payload"
    assert state["succeeded_at"] == 100.0
    # Within TTL: the success latch is not re-polled.
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=120.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is True
    assert state["result"] == "payload"


def test_step_service_ttl_expiry_revokes_success_and_repolls() -> None:
    state = _make_service_state()
    client = mock_client(ready=True, result="payload")
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=100.0, ttl_s=30.0, timeout_s=5.0,
    )
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=100.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is True
    # TTL expired: the latch is revoked so the caller publishes FAIL.
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=131.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is False
    assert state["result"] is None
    assert state["succeeded_at"] is None
    assert state["future"] is client.future  # re-issued on the same client
    # A fresh success restores the latch.
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=131.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is True
    assert state["succeeded_at"] == 131.0


def _refreshing_client(result):
    """A fake client whose ``call_async`` returns a fresh future each call."""
    class FakeSrvType:
        @staticmethod
        def Request():
            return object()

    client = type("RefreshingClient", (), {})()
    client.service_is_ready = lambda: True
    client.srv_type = FakeSrvType
    client._result = result
    client.call_async = lambda request: _FakeFuture(result=client._result, done=True)
    client.destroy = lambda: None
    return client


def test_step_service_parameter_content_change_after_restart() -> None:
    state = _make_service_state()
    client = _refreshing_client("old")
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=100.0, ttl_s=30.0, timeout_s=5.0,
    )
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=100.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["result"] == "old"
    # Restart changes the served content; TTL expiry re-polls and observes it.
    client._result = "new"
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=131.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is False and state["result"] is None
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=131.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is True
    assert state["result"] == "new"


def test_step_service_ttl_expiry_with_unavailable_service_pends() -> None:
    state = _make_service_state()
    client = _refreshing_client("old")
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=100.0, ttl_s=30.0, timeout_s=5.0,
    )
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=100.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is True
    # Controller goes down at the TTL boundary: evidence revoked, no re-poll.
    client.service_is_ready = lambda: False
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=131.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is False
    assert state["pending"] == "service not ready"
    # Controller returns: re-poll succeeds and the fresh result is adopted.
    client.service_is_ready = lambda: True
    client._result = "new"
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=132.0, ttl_s=30.0, timeout_s=5.0,
    )
    step_service(
        state, create_client=lambda: client, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=132.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is True
    assert state["result"] == "new"


# ---------------------------------------------------------------------------
# step_service in-flight timeout / generation-safe retry
# ---------------------------------------------------------------------------


def test_step_service_in_flight_timeout_resets_and_recovers() -> None:
    state = _make_service_state()
    slow = mock_client(ready=True, result="payload", done=False)
    step_service(
        state, create_client=lambda: slow, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=0.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["future"] is slow.future
    assert state["started_at"] == 0.0
    # Still in flight but under the deadline.
    step_service(
        state, create_client=lambda: slow, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=3.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["pending"] == "request in flight"
    assert state["client"] is slow
    # Deadline exceeded: abandon the future and reset the client.
    step_service(
        state, create_client=lambda: slow, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=6.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert "timed out" in str(state["error"])
    assert state["client"] is None
    assert state["future"] is None
    assert state["started_at"] is None
    # A new generation recovers.
    fast = mock_client(ready=True, result="payload")
    step_service(
        state, create_client=lambda: fast, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=7.0, ttl_s=30.0, timeout_s=5.0,
    )
    step_service(
        state, create_client=lambda: fast, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=7.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is True
    assert state["result"] == "payload"


def test_step_service_repeated_timeouts_then_recovery() -> None:
    state = _make_service_state()
    now = 0.0
    for _ in range(3):
        slow = mock_client(ready=True, result=None, done=False)
        step_service(
            state, create_client=lambda: slow, request=lambda c: mock_request(c),
            extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
            now_s=now, ttl_s=30.0, timeout_s=5.0,
        )
        step_service(
            state, create_client=lambda: slow, request=lambda c: mock_request(c),
            extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
            now_s=now + 6.0, ttl_s=30.0, timeout_s=5.0,
        )
        assert "timed out" in str(state["error"])
        now += 10.0
    fast = mock_client(ready=True, result="payload")
    step_service(
        state, create_client=lambda: fast, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=now, ttl_s=30.0, timeout_s=5.0,
    )
    step_service(
        state, create_client=lambda: fast, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=now, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is True
    assert state["result"] == "payload"


def test_step_service_exception_after_timeout() -> None:
    state = _make_service_state()
    slow = mock_client(ready=True, result=RuntimeError("boom"), done=False)
    step_service(
        state, create_client=lambda: slow, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=0.0, ttl_s=30.0, timeout_s=5.0,
    )
    step_service(
        state, create_client=lambda: slow, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=6.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert "timed out" in str(state["error"])
    boom = mock_client(ready=True, result=RuntimeError("connection reset"))
    step_service(
        state, create_client=lambda: boom, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=7.0, ttl_s=30.0, timeout_s=5.0,
    )
    step_service(
        state, create_client=lambda: boom, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=7.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert "service call failed" in str(state["error"])
    assert state["succeeded"] is False


def test_step_service_no_stale_result_from_old_generation() -> None:
    state = _make_service_state()
    old = mock_client(ready=True, result="old", done=False)
    step_service(
        state, create_client=lambda: old, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=0.0, ttl_s=30.0, timeout_s=5.0,
    )
    step_service(
        state, create_client=lambda: old, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=6.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["succeeded"] is False and state["result"] is None
    new = mock_client(ready=True, result="new")
    step_service(
        state, create_client=lambda: new, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=7.0, ttl_s=30.0, timeout_s=5.0,
    )
    step_service(
        state, create_client=lambda: new, request=lambda c: mock_request(c),
        extract=lambda r: r, reset_client=lambda s: s.update(client=None, future=None),
        now_s=7.0, ttl_s=30.0, timeout_s=5.0,
    )
    assert state["result"] == "new"


# ---------------------------------------------------------------------------
# derive_logical_joint_state_publishers endpoint/source edge cases
# ---------------------------------------------------------------------------


def test_attribution_standalone_bare_name_matches() -> None:
    labels, reasons = derive_logical_joint_state_publishers(
        raw_labels=["joint_state_broadcaster"],
        controller_manager="controller_manager",
        broadcaster_controller="joint_state_broadcaster",
        controller_entries=[],
    )
    assert labels == ["joint_state_broadcaster"]
    assert reasons == []


def test_attribution_standalone_renamed_preserved() -> None:
    labels, reasons = derive_logical_joint_state_publishers(
        raw_labels=["/joint_state_broadcaster2"],
        controller_manager="controller_manager",
        broadcaster_controller="joint_state_broadcaster",
        controller_entries=[],
    )
    assert labels == ["/joint_state_broadcaster2"]
    assert reasons == []


def test_attribution_nested_namespace_controller_manager_preserved() -> None:
    labels, reasons = derive_logical_joint_state_publishers(
        raw_labels=["/some_ns/controller_manager"],
        controller_manager="controller_manager",
        broadcaster_controller="joint_state_broadcaster",
        controller_entries=[("joint_state_broadcaster", "active")],
    )
    assert labels == ["/some_ns/controller_manager"]
    assert reasons == []


def test_attribution_standalone_exact_beats_stale_controller_entries() -> None:
    # A standalone exact-name broadcaster satisfies attribution even when the
    # controller-manager list is stale/absent; only controller-manager-hosted
    # publishers require exact active-controller proof.
    labels, reasons = derive_logical_joint_state_publishers(
        raw_labels=["/joint_state_broadcaster"],
        controller_manager="controller_manager",
        broadcaster_controller="joint_state_broadcaster",
        controller_entries=[("foo_broadcaster", "active")],
    )
    assert labels == ["joint_state_broadcaster"]
    assert reasons == []
