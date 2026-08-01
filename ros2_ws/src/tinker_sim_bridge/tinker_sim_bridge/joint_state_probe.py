"""ROS Humble live probe for the integrated eight-joint state contract.

The probe collects three independent pieces of live evidence and evaluates each
through the pure ROS-free helpers in :mod:`contract_guard`:

1. the ``robot_description`` parameter received by ``/controller_manager``,
   evaluated by :func:`contract_guard.evaluate_robot_description_contract`;
2. the checked-in ``tinker_topic_control.ros2_control.xacro`` source, evaluated
   by :func:`contract_guard.evaluate_xacro_contract` and compared with the live
   parameter through :func:`contract_guard.evaluate_joint_state_evidence_pair`;
3. the actual ``sensor_msgs/msg/JointState`` endpoint: graph publisher metadata
   through :func:`contract_guard.evaluate_integrated_cardinality` and one
   received sample through :func:`contract_guard.evaluate_joint_state_sample`.

The verdict is published as compact JSON on ``/sim/status/joint_state_contract``
(latched, reliable).  The simulator itself is only launched in a later overlay
task; this probe is the Task 6 evidence path, and every evaluation it performs is
a deterministic pure helper that the contract tests exercise now.
"""
from __future__ import annotations

import json
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .contract_guard import (
    evaluate_integrated_cardinality,
    evaluate_joint_state_evidence_pair,
    evaluate_joint_state_sample,
    evaluate_robot_description_contract,
    evaluate_xacro_contract,
)

_XACRO_FILENAME = "tinker_topic_control.ros2_control.xacro"


class JointStateProbe(Node):
    def __init__(self) -> None:
        super().__init__("tinker_sim_joint_state_probe")
        # ``use_sim_time`` is declared automatically by rclpy on every node;
        # set it through the launch/CLI parameter interface instead.
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("broadcaster", "joint_state_broadcaster")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("check_period_s", 5.0)
        self.declare_parameter("xacro_path", "")
        self._controller_manager = str(self.get_parameter("controller_manager").value).strip("/")
        self._broadcaster = str(self.get_parameter("broadcaster").value)
        self._topic = str(self.get_parameter("joint_state_topic").value)
        period = float(self.get_parameter("check_period_s").value)

        self._sample: JointState | None = None
        self._sample_received_ns = 0
        self._robot_description: str | None = None
        self._description_error: str | None = None
        self._description_future = None
        self._description_client = None
        self._service_group = ReentrantCallbackGroup()
        try:
            self._xacro_text = self._load_xacro()
        except OSError as exc:
            self._xacro_text = None
            self._xacro_error = str(exc)
        else:
            self._xacro_error = None

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            String, "/sim/status/joint_state_contract", status_qos
        )
        sample_qos = QoSProfile(
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            JointState, self._topic, self._on_joint_state, sample_qos
        )
        self.create_timer(period, self._check)

    def _load_xacro(self) -> str:
        configured = str(self.get_parameter("xacro_path").value)
        if configured:
            return Path(configured).read_text(encoding="utf-8")
        from ament_index_python.packages import get_package_share_directory

        try:
            share = Path(get_package_share_directory("tinker_sim_bridge"))
            candidate = share / "config" / _XACRO_FILENAME
            if candidate.is_file():
                return candidate.read_text(encoding="utf-8")
        except (ImportError, KeyError):
            pass
        module_dir = Path(__file__).resolve().parents[1]
        candidate = module_dir / "config" / _XACRO_FILENAME
        if candidate.is_file():
            return candidate.read_text(encoding="utf-8")
        raise FileNotFoundError(
            "cannot locate {}".format(_XACRO_FILENAME)
        )

    def _on_joint_state(self, message: JointState) -> None:
        self._sample = message
        self._sample_received_ns = self.get_clock().now().nanoseconds

    def _ensure_robot_description(self) -> None:
        if self._robot_description is not None or self._description_error is not None:
            return
        if self._description_future is None:
            if self._description_client is None:
                from rcl_interfaces.srv import GetParameters

                self._description_client = self.create_client(
                    GetParameters,
                    "/{}/get_parameters".format(self._controller_manager),
                    callback_group=self._service_group,
                )
            if not self._description_client.service_is_ready():
                return
            request = self._description_client.srv_type.Request()
            request.names = ["robot_description"]
            self._description_future = self._description_client.call_async(request)
            return
        if not self._description_future.done():
            return
        response = self._description_future.result()
        self._description_client.destroy()
        self._description_client = None
        self._description_future = None
        if response is not None and response.values:
            value = response.values[0]
            if value.string_value:
                self._robot_description = value.string_value
                return
        self._description_error = (
            "controller_manager robot_description parameter is unset or not a string"
        )

    def _logical_publisher_labels(self, publishers) -> list[str]:
        manager = "/" + self._controller_manager
        labels: list[str] = []
        for info in publishers:
            node_label = "/{}/{}".format(
                (info.node_namespace or "").rstrip("/"),
                info.node_name or "",
            ).replace("//", "/")
            if node_label == manager:
                # ros2_control controllers publish from the controller-manager
                # node; label the sole expected source with the broadcaster
                # controller identity.
                labels.append(self._broadcaster)
            else:
                labels.append(node_label)
        return labels

    def _check(self) -> None:
        self._ensure_robot_description()
        publishers = self.get_publishers_info_by_topic(self._topic)
        raw_labels = [
            "/{}/{}".format(
                (info.node_namespace or "").rstrip("/"),
                info.node_name or "",
            ).replace("//", "/")
            for info in publishers
        ]
        logical_labels = self._logical_publisher_labels(publishers)
        cardinality = evaluate_integrated_cardinality(
            joint_state_publishers=logical_labels
        )

        description_contract = evaluate_robot_description_contract(
            self._robot_description
            if self._robot_description is not None
            else "<unavailable: {}>".format(self._description_error or "not read yet")
        )
        xacro_contract = (
            evaluate_xacro_contract(self._xacro_text)
            if self._xacro_text is not None
            else {
                "ready": False,
                "reasons": ["xacro unavailable: {}".format(self._xacro_error)],
                "observed": {},
            }
        )
        evidence_pair = evaluate_joint_state_evidence_pair(
            xacro_contract=xacro_contract,
            description_contract=description_contract,
        )

        sample_result: dict[str, object] = {
            "ready": False,
            "reasons": ["no joint_state sample received yet"],
            "observed": {},
        }
        if self._sample is not None:
            stamp = self._sample.header.stamp
            header_stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            sample_result = evaluate_joint_state_sample(
                publisher_node=logical_labels[0] if len(logical_labels) == 1 else "",
                publisher_count=len(logical_labels),
                names=list(self._sample.name),
                positions=list(self._sample.position),
                velocities=list(self._sample.velocity),
                header_stamp_ns=header_stamp_ns,
                received_at_ns=self._sample_received_ns,
                now_ns=self.get_clock().now().nanoseconds,
            )

        ready_flags = [
            cardinality["ready"],
            description_contract["ready"],
            xacro_contract["ready"],
            evidence_pair["ready"],
        ]
        if self._sample is not None:
            ready_flags.append(sample_result["ready"])
        state = "pass" if all(ready_flags) else "fail"
        description_text = self._robot_description or ""
        evidence = {
            "state": state,
            "controller_manager": "/" + self._controller_manager,
            "broadcaster": self._broadcaster,
            "joint_state_topic": self._topic,
            "description_contract": description_contract,
            "xacro_contract": xacro_contract,
            "evidence_pair": evidence_pair,
            "cardinality": cardinality,
            "joint_state_sample": sample_result,
            "graph": {
                "raw_publisher_labels": raw_labels,
                "logical_publisher_labels": logical_labels,
            },
            "source_text_diagnostics": {
                "xacro_drive_joint_occurrences": (
                    self._xacro_text.count("drive_joint") if self._xacro_text else None
                ),
                "description_drive_joint_occurrences": description_text.count(
                    "drive_joint"
                ),
            },
        }
        message = String()
        message.data = json.dumps(evidence, sort_keys=True)
        self._publisher.publish(message)
        if state == "fail":
            self.get_logger().warning(
                "joint_state contract fail: {}".format(
                    json.dumps(
                        {
                            "cardinality": cardinality["reasons"],
                            "description": description_contract["reasons"],
                            "sample": sample_result["reasons"],
                        }
                    )
                )
            )


def main() -> None:
    rclpy.init()
    node = JointStateProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
