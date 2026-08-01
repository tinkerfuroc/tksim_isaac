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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.contract_guard import (  # noqa: E402
    INTEGRATED_JOINT_STATE_NAMES,
    JOINT_STATE_BROADCASTER,
    evaluate_integrated_cardinality,
    evaluate_joint_state_evidence_pair,
    evaluate_joint_state_sample,
    evaluate_robot_description_contract,
    evaluate_xacro_contract,
)
from tinker_sim_deploy.runtime import topic_control_description  # noqa: E402
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
