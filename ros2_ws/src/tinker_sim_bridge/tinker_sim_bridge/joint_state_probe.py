"""ROS Humble live probe for the integrated eight-joint state contract.

The probe collects independent pieces of live evidence and evaluates each
through the pure ROS-free helpers in :mod:`contract_guard`:

1. the ``robot_description`` parameter received by ``/controller_manager``
   (together with its ``use_sim_time``), evaluated by
   :func:`contract_guard.evaluate_robot_description_contract` and
   :func:`contract_guard.evaluate_clock_domain`;
2. the checked-in ``tinker_topic_control.ros2_control.xacro`` source, evaluated
   by :func:`contract_guard.evaluate_xacro_contract` and compared with the live
   parameter through :func:`contract_guard.evaluate_joint_state_evidence_pair`;
3. the actual ``sensor_msgs/msg/JointState`` endpoint: graph publisher metadata
   plus ``/controller_manager/list_controllers`` evidence (proven attribution via
   :func:`contract_guard.derive_logical_joint_state_publishers`), a received
   sample through :func:`contract_guard.evaluate_joint_state_sample`, and a
   wall-clock freshness watchdog.

The verdict is fail-closed: no sample, stale/no-new sample, clock-domain
mismatch, unproven publisher source, or any service failure produces FAIL with
explicit evidence, and a latched old PASS is replaced by the current status on
every tick.  It is published as compact JSON on ``/sim/status/joint_state_contract``
(latched, reliable).  The simulator itself is only launched in a later overlay
task; this probe is the Task 6 evidence path, and every evaluation it performs
is a deterministic pure helper that the contract tests exercise now.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import ParameterType
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .contract_guard import (
    JOINT_STATE_BROADCASTER,
    JOINT_STATE_DEFAULT_WATCHDOG_S,
    JOINT_STATE_SERVICE_TIMEOUT_S,
    JOINT_STATE_SERVICE_TTL_S,
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

_XACRO_FILENAME = "tinker_topic_control.ros2_control.xacro"
_PARAMETER_NAMES = ["robot_description", "use_sim_time"]


def _parameter_string_value(value) -> str | None:
    """Read a ``PARAMETER_STRING`` from a real Humble ``ParameterValue``.

    The type constants live on ``rcl_interfaces.msg.ParameterType``; generated
    ``ParameterValue`` instances expose no such attributes, so the comparison is
    made against ``ParameterType`` directly.
    """
    try:
        if value.type == ParameterType.PARAMETER_STRING:
            return str(value.string_value)
    except AttributeError:
        return None
    return None


def _parameter_bool_value(value) -> bool | None:
    try:
        if value.type == ParameterType.PARAMETER_BOOL:
            return bool(value.bool_value)
    except AttributeError:
        return None
    return None


def _endpoint_label_from_info(info: object) -> str:
    """Build a normalized ``/namespace/name`` label without a double slash.

    A root-namespace publisher (``node_namespace == "/"``) becomes
    ``"/<name>"``, matching the labels the pure attribution helpers expect.
    """
    namespace = str(getattr(info, "node_namespace", ""))
    name = str(getattr(info, "node_name", ""))
    namespace = namespace.rstrip("/")
    if namespace:
        return "{}/{}".format(namespace, name)
    return "/" + name


class JointStateProbe(Node):
    def __init__(self) -> None:
        super().__init__("tinker_sim_joint_state_probe")
        # ``use_sim_time`` is declared automatically by rclpy on every node and
        # must be supplied at launch: ``--ros-args -p use_sim_time:=true`` so
        # the probe's clock matches the sim-time controller_manager.
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("broadcaster", JOINT_STATE_BROADCASTER)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("check_period_s", 5.0)
        self.declare_parameter("sample_watchdog_s", JOINT_STATE_DEFAULT_WATCHDOG_S)
        self.declare_parameter("xacro_path", "")
        self._controller_manager = str(self.get_parameter("controller_manager").value).strip("/")
        self._broadcaster = str(self.get_parameter("broadcaster").value)
        self._topic = str(self.get_parameter("joint_state_topic").value)
        period = float(self.get_parameter("check_period_s").value)
        self._sample_watchdog_s = float(self.get_parameter("sample_watchdog_s").value)
        self._clock = self.get_clock()

        self._sample: JointState | None = None
        self._sample_received_ns = 0
        self._wall_last_sample_monotonic = time.monotonic()
        self._robot_description: str | None = None
        self._remote_use_sim_time: bool | None = None
        self._parameters_state: dict[str, object] = {
            "client": None,
            "future": None,
            "error": None,
            "pending": None,
            "succeeded": False,
            "result": None,
        }
        self._controllers_state: dict[str, object] = {
            "client": None,
            "future": None,
            "error": None,
            "pending": None,
            "succeeded": False,
            "result": None,
        }
        self._controller_states: dict[str, str] = {}
        self._controller_entries: list[tuple[str, str]] = []
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
        raise FileNotFoundError("cannot locate {}".format(_XACRO_FILENAME))

    def _on_joint_state(self, message: JointState) -> None:
        self._sample = message
        self._sample_received_ns = self._clock.now().nanoseconds
        self._wall_last_sample_monotonic = time.monotonic()

    def _step_parameters(self) -> None:
        """Advance the bounded controller_manager GetParameters request."""
        def create_client():
            from rcl_interfaces.srv import GetParameters

            return self.create_client(
                GetParameters,
                "/{}/get_parameters".format(self._controller_manager),
                callback_group=self._service_group,
            )

        def request(client):
            req = client.srv_type.Request()
            req.names = list(_PARAMETER_NAMES)
            return req

        def extract(response):
            if response is None:
                return None
            values = getattr(response, "values", None)
            if not values or len(values) < 1:
                return None
            description = _parameter_string_value(values[0])
            if description is None:
                return None
            remote = _parameter_bool_value(values[1]) if len(values) > 1 else None
            return (description, remote)

        def reset_client(state):
            client = state.get("client")
            if client is not None:
                try:
                    client.destroy()
                except Exception:  # noqa: BLE001 - destroy must never raise here
                    pass
            state["client"] = None
            state["future"] = None

        step_service(
            self._parameters_state,
            create_client=create_client,
            request=request,
            extract=extract,
            reset_client=reset_client,
            now_s=time.monotonic(),
            ttl_s=JOINT_STATE_SERVICE_TTL_S,
            timeout_s=JOINT_STATE_SERVICE_TIMEOUT_S,
        )
        if self._parameters_state.get("succeeded"):
            description, remote = self._parameters_state["result"]  # type: ignore[misc]
            self._robot_description = description
            self._remote_use_sim_time = remote

    def _step_controllers(self) -> None:
        """Advance the bounded controller_manager ListControllers request."""
        def create_client():
            from controller_manager_msgs.srv import ListControllers

            return self.create_client(
                ListControllers,
                "/{}/list_controllers".format(self._controller_manager),
                callback_group=self._service_group,
            )

        def request(client):
            return client.srv_type.Request()

        def extract(response):
            if response is None or not hasattr(response, "controller"):
                return None
            entries: list[tuple[str, str]] = []
            for item in response.controller:
                entries.append((item.name, item.state))
            return entries

        def reset_client(state):
            client = state.get("client")
            if client is not None:
                try:
                    client.destroy()
                except Exception:  # noqa: BLE001 - destroy must never raise here
                    pass
            state["client"] = None
            state["future"] = None

        step_service(
            self._controllers_state,
            create_client=create_client,
            request=request,
            extract=extract,
            reset_client=reset_client,
            now_s=time.monotonic(),
            ttl_s=JOINT_STATE_SERVICE_TTL_S,
            timeout_s=JOINT_STATE_SERVICE_TIMEOUT_S,
        )
        result = self._controllers_state.get("result")
        if isinstance(result, list):
            self._controller_entries = [(name, state) for name, state in result]
            states: dict[str, str] = {}
            for name, state in self._controller_entries:
                states[name] = state
            self._controller_states = states
        else:
            self._controller_entries = []
            self._controller_states = {}

    def _check(self) -> None:
        self._step_parameters()
        self._step_controllers()

        raw_infos = self.get_publishers_info_by_topic(self._topic)
        raw_labels = [_endpoint_label_from_info(info) for info in raw_infos]
        controller_states = dict(self._controller_states)
        logical_labels, attribution_reasons = derive_logical_joint_state_publishers(
            raw_labels=raw_labels,
            controller_manager=self._controller_manager,
            broadcaster_controller=self._broadcaster,
            controller_entries=list(self._controller_entries),
        )
        # A controller-manager-hosted publisher requires fresh exact active
        # controller proof; a standalone exact-name broadcaster satisfies
        # attribution without any controller-manager list, so the manager state
        # gate only applies when the graph actually shows a manager publisher.
        manager_label = "/" + self._controller_manager
        manager_hosted = any(label == manager_label for label in raw_labels)
        if manager_hosted and not self._controllers_state.get("succeeded"):
            attribution_reasons.append(
                "controller manager state is unavailable: {}".format(
                    self._controllers_state.get("error")
                    or self._controllers_state.get("pending")
                    or "not queried"
                )
            )

        cardinality = evaluate_integrated_cardinality(
            joint_state_publishers=logical_labels
        )

        clock_now_ns = self._clock.now().nanoseconds
        sim_clock_active = bool(self.get_publishers_info_by_topic("/clock"))
        local_use_sim_time = bool(self.get_parameter("use_sim_time").value)
        # Successful parameter evidence is only fresh within its TTL; once the
        # success latch is revoked the stale description/use_sim_time values must
        # not contribute readiness until a fresh response arrives.
        parameters_fresh = bool(self._parameters_state.get("succeeded"))
        clock_domain = evaluate_clock_domain(
            local_use_sim_time=local_use_sim_time,
            remote_use_sim_time=(
                self._remote_use_sim_time if parameters_fresh else None
            ),
            sim_clock_active=sim_clock_active,
            clock_now_ns=clock_now_ns,
        )

        description_contract = (
            evaluate_robot_description_contract(self._robot_description)
            if parameters_fresh and self._robot_description is not None
            else {
                "ready": False,
                "reasons": [
                    "controller_manager robot_description is stale or unavailable: {}".format(
                        self._parameters_state.get("error")
                        or self._parameters_state.get("pending")
                        or "not read yet"
                    )
                ],
                "observed": {},
            }
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

        freshness = evaluate_sample_freshness(
            sample_present=self._sample is not None,
            wall_age_s=(
                time.monotonic() - self._wall_last_sample_monotonic
                if self._sample is not None
                else None
            ),
            wall_watchdog_s=self._sample_watchdog_s,
        )
        sample_effort_supplemental: dict[str, object] | None = None
        if self._sample is not None:
            stamp = self._sample.header.stamp
            header_stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            content = evaluate_joint_state_sample(
                publisher_node=logical_labels[0] if len(logical_labels) == 1 else "",
                publisher_count=len(logical_labels),
                names=list(self._sample.name),
                positions=list(self._sample.position),
                velocities=list(self._sample.velocity),
                header_stamp_ns=header_stamp_ns,
                received_at_ns=self._sample_received_ns,
                now_ns=clock_now_ns,
            )
            effort = list(self._sample.effort)
            sample_effort_supplemental = {
                "length": len(effort),
                "all_finite": all(math.isfinite(float(value)) for value in effort),
            }
            sample_content: dict[str, object] = content
        else:
            sample_content = {
                "ready": False,
                "reasons": ["no joint_state sample received yet"],
                "observed": {},
            }
        sample_ready = bool(sample_content["ready"]) and bool(freshness["ready"])
        sample_reasons = list(sample_content["reasons"]) + list(freshness["reasons"])

        verdict = evaluate_probe_verdict(
            sample_ready=sample_ready,
            sample_reasons=sample_reasons,
            cardinality_ready=bool(cardinality["ready"]),
            attribution_ready=not attribution_reasons,
            description_ready=bool(description_contract["ready"]),
            xacro_ready=bool(xacro_contract["ready"]),
            evidence_pair_ready=bool(evidence_pair["ready"]),
            clock_domain_ready=bool(clock_domain["ready"]),
        )
        state = str(verdict["state"])
        description_text = self._robot_description or ""
        evidence = {
            "state": state,
            "controller_manager": "/" + self._controller_manager,
            "broadcaster": self._broadcaster,
            "joint_state_topic": self._topic,
            "verdict": verdict,
            "clock_domain": clock_domain,
            "description_contract": description_contract,
            "xacro_contract": xacro_contract,
            "evidence_pair": evidence_pair,
            "cardinality": cardinality,
            "publisher_attribution": {
                "ready": not attribution_reasons,
                "reasons": attribution_reasons,
                "observed": {
                    "raw_labels": raw_labels,
                    "logical_labels": logical_labels,
                },
            },
            "controller_evidence": {
                "succeeded": bool(self._controllers_state.get("succeeded")),
                "fresh": bool(self._controllers_state.get("succeeded")),
                "error": self._controllers_state.get("error"),
                "pending": self._controllers_state.get("pending"),
                "ttl_s": JOINT_STATE_SERVICE_TTL_S,
                "timeout_s": JOINT_STATE_SERVICE_TIMEOUT_S,
                "succeeded_at": self._controllers_state.get("succeeded_at"),
                "controller_states": controller_states,
            },
            "parameter_evidence": {
                "succeeded": parameters_fresh,
                "fresh": parameters_fresh,
                "error": self._parameters_state.get("error"),
                "pending": self._parameters_state.get("pending"),
                "ttl_s": JOINT_STATE_SERVICE_TTL_S,
                "timeout_s": JOINT_STATE_SERVICE_TIMEOUT_S,
                "succeeded_at": self._parameters_state.get("succeeded_at"),
                "remote_use_sim_time": (
                    self._remote_use_sim_time if parameters_fresh else None
                ),
            },
            "joint_state_sample": sample_content,
            "sample_freshness": freshness,
            "sample_effort_supplemental": sample_effort_supplemental,
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
        message.data = json.dumps(evidence, sort_keys=True, separators=(",", ":"))
        self._publisher.publish(message)
        if state == "fail":
            self.get_logger().warning(
                "joint_state contract fail: {}".format(
                    json.dumps(
                        {
                            "verdict": verdict["reasons"],
                            "cardinality": cardinality["reasons"],
                            "attribution": attribution_reasons,
                            "description": description_contract["reasons"],
                            "clock_domain": clock_domain["reasons"],
                            "sample": sample_reasons,
                        },
                        sort_keys=True,
                        separators=(",", ":"),
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
