from __future__ import annotations

import json
import math
import time
import xml.etree.ElementTree as ET
from collections.abc import Iterable, Mapping, Sequence

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - pure contract helpers stay ROS-free
    rclpy = None
    Node = object  # type: ignore[assignment,misc]
    DurabilityPolicy = None  # type: ignore[assignment,misc]
    QoSProfile = None  # type: ignore[assignment,misc]
    ReliabilityPolicy = None  # type: ignore[assignment,misc]
    String = None  # type: ignore[assignment,misc]


NAVIGATION_REQUIRED_TOPICS = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/isaac_joint_states": "sensor_msgs/msg/JointState",
    "/isaac_joint_commands": "sensor_msgs/msg/JointState",
    "/joint_states": "sensor_msgs/msg/JointState",
    "/tracer/odom": "nav_msgs/msg/Odometry",
    "/livox/lidar": "sensor_msgs/msg/PointCloud2",
    "/livox/imu": "sensor_msgs/msg/Imu",
    "/scan": "sensor_msgs/msg/LaserScan",
}

MANIPULATION_REQUIRED_TOPICS = {
    "/clock": "rosgraph_msgs/msg/Clock",
    "/isaac_joint_states": "sensor_msgs/msg/JointState",
    "/isaac_joint_commands": "sensor_msgs/msg/JointState",
    "/sim/internal/physics_truth": "std_msgs/msg/String",
    "/sim/truth/robot_state": "tinker_sim_interfaces/msg/RobotTruth",
    "/sim/truth/object_state": "tinker_sim_interfaces/msg/ObjectTruth",
    "/sim/truth/contacts": "tinker_sim_interfaces/msg/ContactTruth",
    "/sim/truth/task_state": "tinker_sim_interfaces/msg/TaskTruth",
    "/sim/hardware/safety_stop": "std_msgs/msg/Bool",
    "/sim/safety/xarm": "std_msgs/msg/Bool",
    "/sim/safety/collision": "std_msgs/msg/Bool",
}

SAFETY_SUPERVISOR_NODE = "tinker_sim_safety_supervisor"

TOPIC_PROFILES = {
    "navigation": NAVIGATION_REQUIRED_TOPICS,
    "manipulation": MANIPULATION_REQUIRED_TOPICS,
}
# Kept as a compatibility alias for callers that imported the old navigation
# contract directly.
EXPECTED_TYPES = NAVIGATION_REQUIRED_TOPICS

FORBIDDEN_SERVICES = {
    "/sim/control/reset",
    "/sim/control/pause",
    "/sim/control/step",
    "/sim/control/set_seed",
    "/sim/scenario/load",
    "/sim/scenario/status",
}

REQUIRED_STANDARD_SERVICES = {
    "/get_simulation_state",
    "/set_simulation_state",
    "/reset_simulation",
    "/step_simulation",
    "/load_world",
    "/spawn_entity",
    "/set_entity_state",
}


def _cardinality_state(
    actual: int,
    expected: int,
    startup_grace_elapsed: bool,
) -> str:
    if actual == expected:
        return "pass"
    if actual == 0 and expected == 1 and not startup_grace_elapsed:
        return "starting"
    return "fail"


def evaluate_cardinality(
    profile: str,
    command_publisher_count: int,
    raw_truth_subscriber_count: int,
    startup_grace_elapsed: bool,
    safety_stop_publisher_count: int = 0,
    xarm_source_publisher_count: int = 0,
    collision_source_publisher_count: int = 0,
) -> dict[str, dict[str, object]]:
    """Classify endpoint cardinality, including DDS discovery startup state."""
    if profile not in TOPIC_PROFILES:
        raise ValueError(f"unsupported contract profile: {profile!r}")
    raw_truth_expected = 1 if profile == "manipulation" else 0
    command_state = _cardinality_state(
        command_publisher_count, 1, startup_grace_elapsed
    )
    raw_truth_state = _cardinality_state(
        raw_truth_subscriber_count, raw_truth_expected, startup_grace_elapsed
    )
    if profile == "manipulation":
        safety_stop_state = _cardinality_state(
            safety_stop_publisher_count, 1, startup_grace_elapsed
        )
        xarm_source_state = _cardinality_state(
            xarm_source_publisher_count, 1, startup_grace_elapsed
        )
        collision_source_state = _cardinality_state(
            collision_source_publisher_count, 1, startup_grace_elapsed
        )
    else:
        safety_stop_state = xarm_source_state = collision_source_state = "not-applicable"
    return {
        "command_publisher": {
            "expected": 1,
            "actual": command_publisher_count,
            "state": command_state,
            "ok": command_state == "pass",
        },
        "raw_truth_subscriber": {
            "expected": raw_truth_expected,
            "actual": raw_truth_subscriber_count,
            "state": raw_truth_state,
            "ok": raw_truth_state == "pass",
        },
        "safety_stop_publisher": {
            "expected": 1 if profile == "manipulation" else 0,
            "actual": safety_stop_publisher_count,
            "state": safety_stop_state,
            "ok": safety_stop_state == "pass" or profile != "manipulation",
        },
        "xarm_source_publisher": {
            "expected": 1 if profile == "manipulation" else 0,
            "actual": xarm_source_publisher_count,
            "state": xarm_source_state,
            "ok": xarm_source_state == "pass" or profile != "manipulation",
        },
        "collision_source_publisher": {
            "expected": 1 if profile == "manipulation" else 0,
            "actual": collision_source_publisher_count,
            "state": collision_source_state,
            "ok": collision_source_state == "pass" or profile != "manipulation",
        },
    }


def evaluate_contract(
    profile: str,
    topic_types: Mapping[str, Sequence[str]],
    services: Iterable[str],
    command_publishers: Iterable[str],
    raw_truth_subscribers: Iterable[str] = (),
    tf_publishers: Iterable[str] = (),
    startup_grace_elapsed: bool = True,
    safety_stop_publishers: Iterable[str] = (),
    safety_source_publishers: Mapping[str, Iterable[str]] | None = None,
) -> dict[str, object]:
    """Evaluate a graph snapshot without requiring a live ROS graph."""
    command_publishers = tuple(command_publishers)
    raw_truth_subscribers = tuple(raw_truth_subscribers)
    safety_stop_publishers = tuple(safety_stop_publishers)
    safety_source_publishers = safety_source_publishers or {}
    try:
        expected_topics = TOPIC_PROFILES[profile]
    except KeyError as error:
        raise ValueError(f"unsupported contract profile: {profile!r}") from error
    missing_topics = sorted(topic for topic in expected_topics if not topic_types.get(topic))
    wrong_types = []
    for topic, expected in expected_topics.items():
        actual = topic_types.get(topic, ())
        actual_types = (actual,) if isinstance(actual, str) else tuple(actual)
        if actual_types and expected not in actual_types:
            wrong_types.append(f"{topic}:{list(actual_types)}")
    service_names = set(services)
    missing_services = sorted(REQUIRED_STANDARD_SERVICES - service_names)
    forbidden_services = sorted(FORBIDDEN_SERVICES & service_names)
    wrong_command_publishers = sorted(
        owner for owner in command_publishers
        if owner.rsplit("/", 1)[-1] != "tinker_sim_command_gateway"
    )
    forbidden_truth_subscribers = sorted(
        owner for owner in raw_truth_subscribers
        if owner.rsplit("/", 1)[-1] != "tinker_truth_evaluator"
    )
    wrong_safety_stop_publishers = sorted(
        owner
        for owner in safety_stop_publishers
        if owner.rsplit("/", 1)[-1] != SAFETY_SUPERVISOR_NODE
    )
    xarm_source_publishers = tuple(safety_source_publishers.get("xarm", ()))
    collision_source_publishers = tuple(safety_source_publishers.get("collision", ()))
    forbidden_tf_publishers = sorted(tf_publishers)
    command_publisher_count = len(command_publishers)
    raw_truth_subscriber_count = len(raw_truth_subscribers)
    cardinality = evaluate_cardinality(
        profile,
        command_publisher_count,
        raw_truth_subscriber_count,
        startup_grace_elapsed,
        len(safety_stop_publishers),
        len(xarm_source_publishers),
        len(collision_source_publishers),
    )
    return {
        "profile": profile,
        "missing_topics": missing_topics,
        "missing_standard_services": missing_services,
        "forbidden_services": forbidden_services,
        "wrong_types": wrong_types,
        "wrong_command_publishers": wrong_command_publishers,
        "forbidden_truth_subscribers": forbidden_truth_subscribers,
        "wrong_safety_stop_publishers": wrong_safety_stop_publishers,
        "forbidden_tf_publishers": forbidden_tf_publishers,
        "command_publisher_count": command_publisher_count,
        "raw_truth_subscriber_count": raw_truth_subscriber_count,
        "command_publisher_cardinality": cardinality["command_publisher"],
        "raw_truth_subscriber_cardinality": cardinality["raw_truth_subscriber"],
        "safety_stop_publisher_count": len(safety_stop_publishers),
        "xarm_source_publisher_count": len(xarm_source_publishers),
        "collision_source_publisher_count": len(collision_source_publishers),
        "safety_stop_publisher_cardinality": cardinality["safety_stop_publisher"],
        "xarm_source_publisher_cardinality": cardinality["xarm_source_publisher"],
        "collision_source_publisher_cardinality": cardinality["collision_source_publisher"],
    }


def evaluate_overall_state(
    contract: Mapping[str, object],
    startup_grace_elapsed: bool,
) -> str:
    """Resolve the published state with failures taking precedence."""
    if (
        contract["wrong_types"]
        or contract["forbidden_tf_publishers"]
        or contract["forbidden_services"]
        or contract["wrong_command_publishers"]
        or contract["forbidden_truth_subscribers"]
        or contract.get("wrong_safety_stop_publishers", [])
    ):
        return "fail"

    cardinality_states = (
        contract["command_publisher_cardinality"]["state"],  # type: ignore[index]
        contract["raw_truth_subscriber_cardinality"]["state"],  # type: ignore[index]
    )
    safety_cardinality_states = (
        contract.get("safety_stop_publisher_cardinality", {"state": "not-applicable"})["state"],  # type: ignore[index]
        contract.get("xarm_source_publisher_cardinality", {"state": "not-applicable"})["state"],  # type: ignore[index]
        contract.get("collision_source_publisher_cardinality", {"state": "not-applicable"})["state"],  # type: ignore[index]
    )
    if "fail" in cardinality_states:
        return "fail"
    if "fail" in safety_cardinality_states:
        return "fail"
    if "starting" in cardinality_states or "starting" in safety_cardinality_states:
        return "fail" if startup_grace_elapsed else "starting"
    if contract["missing_topics"] or contract["missing_standard_services"]:
        return "fail" if startup_grace_elapsed else "starting"
    return "pass"


INTEGRATED_JOINT_STATE_NAMES = tuple(f"joint{index}" for index in range(1, 8)) + (
    "drive_joint",
)
JOINT_STATE_BROADCASTER = "joint_state_broadcaster"
JOINT_STATE_MAX_AGE_NS = 5_000_000_000
JOINT_STATE_MAX_TRANSPORT_NS = 2_000_000_000
# An epoch-scale difference (≳ 11.6 days) cannot be ordinary latency; it is the
# signature of a wall-clock-vs-sim-clock domain mismatch.
JOINT_STATE_CLOCK_DOMAIN_THRESHOLD_NS = 1_000_000_000_000_000
JOINT_STATE_DEFAULT_WATCHDOG_S = 15.0
# Successful ``GetParameters``/``ListControllers`` evidence is re-polled after
# this wall-clock TTL so a controller_manager restart or parameter change cannot
# be masked by latched evidence.  An in-flight request older than
# ``JOINT_STATE_SERVICE_TIMEOUT_S`` is abandoned and the client reset so the
# probe retries on a bounded cadence instead of waiting forever.
JOINT_STATE_SERVICE_TTL_S = 30.0
JOINT_STATE_SERVICE_TIMEOUT_S = 5.0
_EXPECTED_ARM_COMMAND_INTERFACES = ("position", "velocity")
_EXPECTED_ARM_STATE_INTERFACES = ("position", "velocity", "effort")
_EXPECTED_DRIVE_STATE_INTERFACES = ("position", "velocity", "effort")


def evaluate_integrated_cardinality(
    *,
    joint_state_publishers: Sequence[str],
) -> dict[str, object]:
    """Classify the integrated ``/joint_states`` publisher cardinality.

    *joint_state_publishers* carries the logical publisher source labels derived
    from ROS graph endpoint metadata; cardinality one and the sole source
    ``joint_state_broadcaster`` are required.  This helper never inspects source
    text and never relies on an implicit topic name.
    """
    publishers = tuple(joint_state_publishers)
    reasons: list[str] = []
    if len(publishers) != 1:
        reasons.append(
            f"joint_state publisher count is {len(publishers)}, expected 1"
        )
    if publishers != (JOINT_STATE_BROADCASTER,):
        reasons.append(
            "joint_state publisher source is {!r}, expected {!r}".format(
                list(publishers), JOINT_STATE_BROADCASTER
            )
        )
    return {
        "ready": not reasons,
        "reasons": reasons,
        "observed": {
            "joint_state_publishers": list(publishers),
            "joint_state_publisher_count": len(publishers),
            "joint_state_publisher_source": publishers[0] if publishers else None,
        },
    }


def evaluate_joint_state_sample(
    *,
    publisher_node: str,
    publisher_count: int,
    names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float],
    header_stamp_ns: int,
    received_at_ns: int,
    now_ns: int,
) -> dict[str, object]:
    """Classify one actual ``sensor_msgs/msg/JointState`` sample.

    Every time input is explicit: *header_stamp_ns* (message header), the
    *received_at_ns* observation time, and *now_ns* evaluation time.  The
    contract requires the exact eight integrated joint names, a single
    ``joint_state_broadcaster`` publisher, a nonzero header stamp, finite
    eight-element position/velocity arrays, and bounded age/transport latency.
    """
    reasons: list[str] = []
    observed_names = tuple(names)
    observed_positions = tuple(float(value) for value in positions)
    observed_velocities = tuple(float(value) for value in velocities)
    if observed_names != INTEGRATED_JOINT_STATE_NAMES:
        reasons.append(
            "joint names are {!r}, expected {!r}".format(
                list(observed_names), list(INTEGRATED_JOINT_STATE_NAMES)
            )
        )
    if publisher_count != 1:
        reasons.append(f"joint_state publisher count is {publisher_count}, expected 1")
    if publisher_node != JOINT_STATE_BROADCASTER:
        reasons.append(
            "joint_state publisher is {!r}, expected {!r}".format(
                publisher_node, JOINT_STATE_BROADCASTER
            )
        )
    if not header_stamp_ns:
        reasons.append(f"header stamp is zero ({header_stamp_ns} ns)")
    if len(observed_positions) != len(INTEGRATED_JOINT_STATE_NAMES):
        reasons.append(
            "positions length is {}, expected {}".format(
                len(observed_positions), len(INTEGRATED_JOINT_STATE_NAMES)
            )
        )
    elif not all(math.isfinite(value) for value in observed_positions):
        reasons.append("positions contain non-finite values")
    if len(observed_velocities) != len(INTEGRATED_JOINT_STATE_NAMES):
        reasons.append(
            "velocities length is {}, expected {}".format(
                len(observed_velocities), len(INTEGRATED_JOINT_STATE_NAMES)
            )
        )
    elif not all(math.isfinite(value) for value in observed_velocities):
        reasons.append("velocities contain non-finite values")
    age_ns = now_ns - header_stamp_ns
    transport_ns = received_at_ns - header_stamp_ns
    if age_ns < 0:
        reasons.append(f"header stamp is in the future by {-age_ns} ns")
    elif age_ns > JOINT_STATE_MAX_AGE_NS:
        reasons.append(
            "header stamp is stale by {} ns (bound {})".format(
                age_ns, JOINT_STATE_MAX_AGE_NS
            )
        )
    if transport_ns < 0:
        reasons.append(f"transport latency is negative ({transport_ns} ns)")
    elif transport_ns > JOINT_STATE_MAX_TRANSPORT_NS:
        reasons.append(
            "transport latency {} ns exceeds bound {}".format(
                transport_ns, JOINT_STATE_MAX_TRANSPORT_NS
            )
        )
    if abs(age_ns) > JOINT_STATE_CLOCK_DOMAIN_THRESHOLD_NS:
        # Additional, non-replacing diagnostic: an epoch-scale difference is a
        # probable wall-vs-sim clock-domain mismatch, not ordinary staleness.
        reasons.append(
            "header and evaluation clock differ by {} ns; probable use_sim_time "
            "clock-domain mismatch".format(age_ns)
        )
    return {
        "ready": not reasons,
        "reasons": reasons,
        "observed": {
            "publisher_node": publisher_node,
            "publisher_count": publisher_count,
            "names": list(observed_names),
            "positions": list(observed_positions),
            "velocities": list(observed_velocities),
            "header_stamp_ns": header_stamp_ns,
            "received_at_ns": received_at_ns,
            "now_ns": now_ns,
            "age_ns": age_ns,
            "transport_ns": transport_ns,
        },
    }


def _joint_records_from_description(
    description: str | bytes,
) -> tuple[list[str], dict[str, dict[str, list[str]]], str | None]:
    """Parse ``ros2_control`` joint interface records from a description string.

    Returns ``(joint_names, records_by_name, error)`` where *error* is a
    human-readable failure reason when the description is malformed or does not
    contain exactly one ``ros2_control`` block.
    """
    try:
        raw = description.decode("utf-8") if isinstance(description, bytes) else description
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        return [], {}, f"description is not well-formed XML: {exc}"
    controls = root.findall("ros2_control")
    if len(controls) != 1:
        return [], {}, f"expected exactly one ros2_control block, found {len(controls)}"
    records: dict[str, dict[str, list[str]]] = {}
    for joint in controls[0].findall("joint"):
        name = joint.get("name")
        commands = [
            item.get("name")
            for item in joint.findall("command_interface")
            if item.get("name")
        ]
        states = [
            item.get("name")
            for item in joint.findall("state_interface")
            if item.get("name")
        ]
        records[name] = {"command_interfaces": commands, "state_interfaces": states}
    return list(records), records, None


def _joint_state_contract(
    records: dict[str, dict[str, list[str]]],
) -> tuple[list[str], dict[str, object]]:
    """Evaluate the eight-joint state-only drive_joint contract on parsed records."""
    reasons: list[str] = []
    names = list(records)
    if names != list(INTEGRATED_JOINT_STATE_NAMES):
        reasons.append(
            "ros2_control joint names are {!r}, expected {!r}".format(
                names, list(INTEGRATED_JOINT_STATE_NAMES)
            )
        )
    for arm in [f"joint{index}" for index in range(1, 8)]:
        record = records.get(arm)
        if record is None:
            reasons.append(f"arm joint {arm} is missing from ros2_control")
        elif record["command_interfaces"] != list(_EXPECTED_ARM_COMMAND_INTERFACES):
            reasons.append(
                "arm joint {0} command interfaces are {1!r}, expected {2!r}".format(
                    arm, record["command_interfaces"], list(_EXPECTED_ARM_COMMAND_INTERFACES)
                )
            )
        elif record["state_interfaces"] != list(_EXPECTED_ARM_STATE_INTERFACES):
            reasons.append(
                "arm joint {0} state interfaces are {1!r}, expected {2!r}".format(
                    arm, record["state_interfaces"], list(_EXPECTED_ARM_STATE_INTERFACES)
                )
            )
    drive = records.get("drive_joint")
    if drive is None:
        reasons.append("drive_joint is missing from ros2_control")
    else:
        if drive["command_interfaces"] != []:
            reasons.append(
                "drive_joint command interfaces are {!r}, expected []".format(
                    drive["command_interfaces"]
                )
            )
        if drive["state_interfaces"] != list(_EXPECTED_DRIVE_STATE_INTERFACES):
            reasons.append(
                "drive_joint state interfaces are {!r}, expected {!r}".format(
                    drive["state_interfaces"], list(_EXPECTED_DRIVE_STATE_INTERFACES)
                )
            )
    observed: dict[str, object] = {
        "ros2_control_joint_names": names,
        "joint_records": records,
        "arm_joints": {
            arm: records.get(arm) for arm in [f"joint{index}" for index in range(1, 8)]
        },
        "drive_joint": drive,
    }
    return reasons, observed


def evaluate_robot_description_contract(description: str | bytes) -> dict[str, object]:
    """Evaluate the drive_joint state-only contract in a live robot_description.

    The ROS Humble live probe feeds the ``robot_description`` parameter received
    by ``/controller_manager`` through this helper; the same helper is exercised
    deterministically on the complete real robot URDF through the runtime
    transformer in the contract tests.
    """
    names, records, error = _joint_records_from_description(description)
    if error is not None:
        return {"ready": False, "reasons": [error], "observed": {}}
    reasons, observed = _joint_state_contract(records)
    if not names:
        reasons.append("ros2_control contains no joints")
    return {"ready": not reasons, "reasons": reasons, "observed": observed}


_XACRO_NAMESPACES = {"xacro": "http://www.ros.org/wiki/xacro"}


def evaluate_xacro_contract(xacro_text: str) -> dict[str, object]:
    """Evaluate the same drive_joint contract in the checked-in xacro source.

    The live probe compares this evidence with the controller_manager
    ``robot_description`` evidence; both must agree on the state-only
    ``drive_joint``.
    """
    try:
        root = ET.fromstring(xacro_text)
    except ET.ParseError as exc:
        return {"ready": False, "reasons": [f"xacro is not well-formed XML: {exc}"], "observed": {}}
    macro = root.find("xacro:macro", _XACRO_NAMESPACES)
    control = macro.find("ros2_control") if macro is not None else None
    if control is None:
        control = root.find("ros2_control")
    if control is None:
        return {
            "ready": False,
            "reasons": ["xacro contains no ros2_control or xacro:macro container"],
            "observed": {},
        }
    records: dict[str, dict[str, list[str]]] = {}
    for joint in control.findall("joint"):
        name = joint.get("name")
        commands = [
            item.get("name")
            for item in joint.findall("command_interface")
            if item.get("name")
        ]
        states = [
            item.get("name")
            for item in joint.findall("state_interface")
            if item.get("name")
        ]
        records[name] = {"command_interfaces": commands, "state_interfaces": states}
    reasons, observed = _joint_state_contract(records)
    if not records:
        reasons.append("ros2_control contains no joints")
    return {"ready": not reasons, "reasons": reasons, "observed": observed}


def evaluate_joint_state_evidence_pair(
    *,
    xacro_contract: Mapping[str, object],
    description_contract: Mapping[str, object],
) -> dict[str, object]:
    """Compare the checked-in xacro and live robot_description drive_joint evidence.

    Source-xacro and live-parameter evidence are recorded together so the live
    probe (and its deterministic test seam) prove they agree on the exact
    state-only ``drive_joint`` contract.
    """
    reasons: list[str] = []
    xacro_observed = xacro_contract.get("observed")
    description_observed = description_contract.get("observed")
    xacro_drive = (
        xacro_observed.get("drive_joint")
        if isinstance(xacro_observed, dict)
        else None
    )
    description_drive = (
        description_observed.get("drive_joint")
        if isinstance(description_observed, dict)
        else None
    )
    if not xacro_contract.get("ready"):
        reasons.append("checked-in xacro drive_joint contract is not ready")
    if not description_contract.get("ready"):
        reasons.append("controller_manager robot_description drive_joint contract is not ready")
    if xacro_drive != description_drive:
        reasons.append(
            "checked-in xacro drive_joint {!r} differs from live robot_description drive_joint {!r}".format(
                xacro_drive, description_drive
            )
        )
    return {
        "ready": not reasons,
        "reasons": reasons,
        "observed": {
            "xacro_drive_joint": xacro_drive,
            "description_drive_joint": description_drive,
        },
    }


def evaluate_sample_freshness(
    *,
    sample_present: bool,
    wall_age_s: float | None,
    wall_watchdog_s: float,
) -> dict[str, object]:
    """Classify sample presence and wall-clock freshness.

    ``wall_age_s`` is the wall-clock age of the most recent sample (``None`` when
    no sample exists).  A missing sample, or a later wall-clock gap beyond the
    watchdog, is a fail-closed condition: the probe must not affirm a healthy
    ``/joint_states`` endpoint on evidence that stopped arriving.
    """
    reasons: list[str] = []
    if not sample_present:
        reasons.append("no joint_state sample received yet")
    elif wall_age_s is None or wall_age_s > wall_watchdog_s:
        reasons.append(
            "no new joint_state sample for {:.1f} s (watchdog {:.1f} s)".format(
                wall_age_s if wall_age_s is not None else float("inf"),
                wall_watchdog_s,
            )
        )
    return {
        "ready": not reasons,
        "reasons": reasons,
        "observed": {
            "sample_present": bool(sample_present),
            "wall_age_s": wall_age_s,
            "wall_watchdog_s": wall_watchdog_s,
        },
    }


def evaluate_clock_domain(
    *,
    local_use_sim_time: bool,
    remote_use_sim_time: bool | None,
    sim_clock_active: bool,
    clock_now_ns: int | None,
) -> dict[str, object]:
    """Classify probe/controller clock-domain agreement and sim-clock readiness.

    The probe and the controller_manager must agree on ``use_sim_time``; when
    running on the sim clock the probe additionally requires an active ``/clock``
    that has produced a sample.  A mismatch is a typed FAIL with a probable
    ``use_sim_time`` explanation rather than a bare stale/transport verdict.

    ``clock_now_ns`` is ``None`` when the caller itself knows no ``/clock``
    sample has been received yet -- the primary "not ready" signal, checked
    first and independent of the numeric value. When a numeric value *is*
    given, an exact ``0`` is additionally treated as not-ready, matching
    rclpy's own ``TimeSource`` convention where a not-yet-set sim clock reads
    exactly ``0`` ("Zero time is a special value that means time is
    uninitialized", ``rclpy/time_source.py``) -- this covers a caller that
    can only observe the numeric value (e.g. a raw ``Clock.now()`` read
    before any ``/clock`` message has ever arrived) and has no independent
    way to produce ``None``. This is an exact-zero check, not ``<= 0``: task
    #21's ``resolve_clock_epoch`` rejects negative ``TINKER_SIM_CLOCK_EPOCH``
    values specifically so ``ros_clock_time`` (``simulation_time + epoch``)
    can never be negative and reads exactly ``0`` only in the legacy
    zero-based clock (``TINKER_SIM_CLOCK_EPOCH=0``) at true start -- the
    contract the exact-zero branch exists to serve. Under the default
    wall-clock epoch, a real running sim's first sample is a large nonzero
    value, never ``0``, so in practice this branch only fires in legacy mode
    or before any sample has arrived.
    """
    reasons: list[str] = []
    if remote_use_sim_time is None:
        reasons.append("controller_manager use_sim_time parameter is unknown")
    elif local_use_sim_time != remote_use_sim_time:
        reasons.append(
            "probe use_sim_time={} does not match controller_manager use_sim_time={} "
            "(probable use_sim_time clock-domain mismatch)".format(
                local_use_sim_time, remote_use_sim_time
            )
        )
    if not local_use_sim_time:
        reasons.append("probe is not running on the sim clock (use_sim_time=false)")
    elif not sim_clock_active:
        reasons.append("use_sim_time=true but /clock is not published")
    elif clock_now_ns is None:
        reasons.append("sim clock is active but no clock sample has been received yet")
    elif clock_now_ns == 0:
        reasons.append("sim clock is active but has not advanced past zero")
    return {
        "ready": not reasons,
        "reasons": reasons,
        "observed": {
            "local_use_sim_time": bool(local_use_sim_time),
            "remote_use_sim_time": remote_use_sim_time,
            "sim_clock_active": bool(sim_clock_active),
            "clock_now_ns": clock_now_ns,
            "clock_domain": "sim" if local_use_sim_time else "wall",
        },
    }


def _standalone_broadcaster_label(label: str, broadcaster: str) -> str | None:
    """Return the normalized broadcaster label for a standalone broadcaster node.

    A raw label is the standalone broadcaster only when its name is exactly
    *broadcaster* at the root namespace (``/joint_state_broadcaster`` or the
    bare ``joint_state_broadcaster``).  A namespaced node such as
    ``/ns/joint_state_broadcaster`` is a different node and is not converted.
    """
    stripped = label.strip("/")
    if stripped == broadcaster and "/" not in stripped:
        return stripped
    return None


def derive_logical_joint_state_publishers(
    *,
    raw_labels: Sequence[str],
    controller_manager: str,
    broadcaster_controller: str,
    controller_entries: Sequence[tuple[str, str]],
) -> tuple[list[str], list[str]]:
    """Derive logical ``/joint_states`` publisher labels from graph + controller evidence.

    *controller_entries* is the ordered ``(name, state)`` list reported by
    ``controller_manager/list_controllers`` (duplicates preserved).  A raw
    publisher is labeled *broadcaster_controller* only when the evidence proves
    that exact controller is the source:

    - a standalone node named exactly *broadcaster_controller*, or
    - a controller-manager-hosted publisher with exactly one controller named
      *broadcaster_controller* in the ``active`` state.

    Otherwise the raw label is preserved and a reason records the attribution
    gap, so the caller fails honestly instead of relabeling an arbitrary
    controller-manager publisher.
    """
    reasons: list[str] = []
    logical: list[str] = []
    manager = "/" + controller_manager.strip("/")
    matching = [
        state for name, state in controller_entries if name == broadcaster_controller
    ]
    for label in raw_labels:
        standalone = _standalone_broadcaster_label(label, broadcaster_controller)
        if standalone is not None:
            logical.append(standalone)
            continue
        if label == manager:
            if len(matching) == 1 and matching[0] == "active":
                logical.append(broadcaster_controller)
                continue
            reasons.append(
                "controller-manager-hosted publisher cannot be attributed to "
                "broadcaster controller {!r}: need exactly one active controller "
                "of that exact name, found {!r}".format(broadcaster_controller, matching)
            )
        logical.append(label)
    return logical, reasons


def evaluate_probe_verdict(
    *,
    sample_ready: bool,
    sample_reasons: Sequence[str],
    cardinality_ready: bool,
    attribution_ready: bool,
    description_ready: bool,
    xacro_ready: bool,
    evidence_pair_ready: bool,
    clock_domain_ready: bool,
) -> dict[str, object]:
    """Aggregate probe evidence into a single fail-closed verdict.

    Sample readiness participates unconditionally: no sample, stale sample,
    malformed sample, or later loss/staleness produces FAIL with explicit
    evidence.  Because every evaluation tick recomputes the verdict from current
    inputs, a latched old PASS is replaced by the current failure status.
    """
    reasons: list[str] = []
    if not sample_ready:
        if sample_reasons:
            reasons.extend("sample: {}".format(reason) for reason in sample_reasons)
        else:
            reasons.append("sample: no joint_state evidence")
    if not cardinality_ready:
        reasons.append("publisher cardinality not ready")
    if not attribution_ready:
        reasons.append("publisher attribution not ready")
    if not description_ready:
        reasons.append("controller_manager robot_description contract not ready")
    if not xacro_ready:
        reasons.append("checked-in xacro drive_joint contract not ready")
    if not evidence_pair_ready:
        reasons.append("checked-in xacro and live robot_description evidence differ")
    if not clock_domain_ready:
        reasons.append("clock domain evidence not ready")
    return {
        "state": "pass" if not reasons else "fail",
        "ready": not reasons,
        "reasons": reasons,
        "observed": {
            "sample_ready": bool(sample_ready),
            "cardinality_ready": bool(cardinality_ready),
            "attribution_ready": bool(attribution_ready),
            "description_ready": bool(description_ready),
            "xacro_ready": bool(xacro_ready),
            "evidence_pair_ready": bool(evidence_pair_ready),
            "clock_domain_ready": bool(clock_domain_ready),
        },
    }


def step_service(
    state: dict[str, object],
    *,
    create_client,
    request,
    extract,
    reset_client,
    now_s: float | None = None,
    ttl_s: float | None = None,
    timeout_s: float | None = None,
) -> dict[str, object]:
    """Advance one bounded, recoverable async ROS service request.

    *state* carries ``client``, ``future``, ``error``, ``pending``,
    ``succeeded``, and ``result`` keys (plus ``succeeded_at`` and ``started_at``)
    and is mutated in place (the same dict is returned for convenience).  The
    step never raises: discovery-pending and in-flight requests leave the state
    unchanged (the caller publishes FAIL that tick), while a completed-with-
    exception or malformed response records an error, resets the client via
    *reset_client*, and retries on the next step.  A successful *extract* marks
    the request succeeded with its result.

    *now_s* is an explicit monotonic-clock input (seconds) so the freshness/TTL
    and timeout logic is deterministic in tests; it defaults to
    ``time.monotonic()``.  When *ttl_s* is given, a successful result expires
    once ``now_s - succeeded_at > ttl_s``: the success latch is revoked (so the
    caller publishes FAIL until a fresh response arrives) and the service is
    re-polled, which is how a controller_manager restart or parameter change is
    re-verified on a bounded cadence.  When *timeout_s* is given, an in-flight
    request older than the deadline is abandoned, the client reset via
    *reset_client*, and the request retried on the next step without leaking a
    client or keeping a stale future.
    """
    now = time.monotonic() if now_s is None else now_s
    if state.get("succeeded"):
        succeeded_at = state.get("succeeded_at")
        if (
            ttl_s is not None
            and succeeded_at is not None
            and now - succeeded_at > ttl_s
        ):
            # Successful evidence is stale: revoke the latch so it cannot
            # contribute readiness until a fresh successful response arrives.
            state["succeeded"] = False
            state["result"] = None
            state["succeeded_at"] = None
        else:
            return state
    client = state.get("client")
    if client is None:
        client = create_client()
        state["client"] = client
        state["future"] = None
        state["started_at"] = None
    future = state.get("future")
    if future is None:
        if not client.service_is_ready():
            state["pending"] = "service not ready"
            return state
        state["pending"] = None
        state["future"] = client.call_async(request(client))
        state["started_at"] = now
        return state
    if not future.done():
        started_at = state.get("started_at")
        if (
            timeout_s is not None
            and started_at is not None
            and now - started_at > timeout_s
        ):
            state["error"] = "service request timed out after {:.1f} s".format(
                timeout_s
            )
            state["pending"] = None
            state["future"] = None
            state["started_at"] = None
            reset_client(state)
            return state
        state["pending"] = "request in flight"
        return state
    state["pending"] = None
    state["started_at"] = None
    try:
        response = future.result()
    except Exception as exc:  # noqa: BLE001 - transient service failures must recover
        state["error"] = "service call failed: {}".format(exc)
        state["future"] = None
        reset_client(state)
        return state
    state["future"] = None
    try:
        result = extract(response)
    except Exception as exc:  # noqa: BLE001 - malformed responses must not crash
        state["error"] = "service response is malformed: {}".format(exc)
        reset_client(state)
        return state
    if result is None:
        state["error"] = "service returned no usable response"
        reset_client(state)
        return state
    state["error"] = None
    state["succeeded"] = True
    state["result"] = result
    state["succeeded_at"] = now
    return state


def _endpoint_label(info: object) -> str:
    if isinstance(info, str):
        return info
    namespace = str(getattr(info, "node_namespace", ""))
    name = str(getattr(info, "node_name", ""))
    return f"{namespace.rstrip('/')}/{name}" if namespace else name


class ContractGuard(Node):
    def __init__(self) -> None:
        super().__init__("tinker_sim_contract_guard")
        self.declare_parameter("startup_grace_s", 20.0)
        self.declare_parameter("profile", "navigation")
        self._started = time.monotonic()
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            String, "/sim/status/contract", status_qos
        )
        self.create_timer(2.0, self._check)

    def _check(self) -> None:
        graph = dict(self.get_topic_names_and_types())
        profile = str(self.get_parameter("profile").value)
        command_publishers = self.get_publishers_info_by_topic("/isaac_joint_commands")
        raw_truth_subscribers = self.get_subscriptions_info_by_topic(
            "/sim/internal/physics_truth"
        )
        safety_stop_publishers = self.get_publishers_info_by_topic(
            "/sim/hardware/safety_stop"
        )
        safety_source_publishers = {
            name: self.get_publishers_info_by_topic(topic)
            for name, topic in {
                "xarm": "/sim/safety/xarm",
                "collision": "/sim/safety/collision",
            }.items()
        }
        forbidden_tf = []
        for info in self.get_publishers_info_by_topic("/tf"):
            if info.node_name in {"tinker_isaac_gateway", "tinker_base_facade"}:
                forbidden_tf.append(info.node_name)
        services = {name for name, _types in self.get_service_names_and_types()}
        missing_services = sorted(REQUIRED_STANDARD_SERVICES - services)
        forbidden_services = sorted(FORBIDDEN_SERVICES & services)
        startup_grace_elapsed = (
            time.monotonic() - self._started
            > float(self.get_parameter("startup_grace_s").value)
        )
        contract = evaluate_contract(
            profile,
            graph,
            services,
            (_endpoint_label(item) for item in command_publishers),
            (_endpoint_label(item) for item in raw_truth_subscribers),
            forbidden_tf,
            startup_grace_elapsed,
            safety_stop_publishers=(
                _endpoint_label(item) for item in safety_stop_publishers
            ),
            safety_source_publishers={
                name: (_endpoint_label(item) for item in infos)
                for name, infos in safety_source_publishers.items()
            },
        )
        state = evaluate_overall_state(contract, startup_grace_elapsed)
        message = String()
        message.data = json.dumps(
            {
                "state": state,
                **contract,
            },
            sort_keys=True,
        )
        self._publisher.publish(message)
        if state == "fail":
            self.get_logger().error(message.data)


def main() -> None:
    if rclpy is None:
        raise RuntimeError("contract_guard requires a sourced ROS 2 Humble environment")
    rclpy.init(); node = ContractGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
