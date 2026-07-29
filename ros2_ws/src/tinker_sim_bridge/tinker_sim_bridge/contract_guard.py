from __future__ import annotations

import json
import time
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
