"""Evaluate Isaac physics truth at the external simulation boundary.

The evaluator deliberately accepts only the versioned JSON contract published
on ``/sim/internal/physics_truth``.  The pure ``TruthEvaluatorCore`` is usable
without ROS, which keeps the qualification rules deterministic and makes it
possible to test the metrics independently of a running simulator.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

# The evaluator's output contract is independent from the raw simulator
# contract.  Raw physics truth is currently published as either v1 or v2;
# v2 adds fields such as command_targets without changing the fields parsed
# below.
EVALUATOR_SCHEMA_VERSION = 1
SUPPORTED_RAW_SCHEMA_VERSIONS = frozenset({1, 2})
# Keep the historical name available to callers that use it for the
# evaluator output schema.
SCHEMA_VERSION = EVALUATOR_SCHEMA_VERSION
DEFAULT_THRESHOLDS: dict[str, float] = {
    "lift_m": 0.10,
    "translation_m": 0.20,
    "hold_s": 1.0,
    "drift_m": 0.02,
    "drift_deg": 5.0,
    "stable_speed_mps": 0.02,
    "contact_force_n": 0.0,
}


def _finite(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _vec(value: Any, size: int, name: str) -> tuple[float, ...]:
    if isinstance(value, Mapping):
        keys = ("x", "y", "z", "w")[:size]
        value = [value.get(key) for key in keys]
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{name} must be an array")
    if len(value) != size:
        raise ValueError(f"{name} must contain {size} values")
    return tuple(_finite(item, name) for item in value)


def _pose(value: Any, name: str) -> dict[str, tuple[float, ...]]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    position = value.get("position", value.get("xyz"))
    orientation = value.get("orientation", value.get("quaternion_xyzw"))
    return {
        "position": _vec(position, 3, f"{name}.position"),
        "orientation": _quat(orientation, f"{name}.orientation"),
    }


def _quat(value: Any, name: str) -> tuple[float, float, float, float]:
    quaternion = _vec(value, 4, name)
    norm = math.sqrt(sum(item * item for item in quaternion))
    if norm <= 1.0e-12:
        raise ValueError(f"{name} must not be zero")
    return tuple(item / norm for item in quaternion)  # type: ignore[return-value]


def _twist(value: Any, name: str) -> dict[str, tuple[float, ...]]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be an object")
    linear = value.get("linear", value.get("linear_velocity", [0.0, 0.0, 0.0]))
    angular = value.get("angular", value.get("angular_velocity", [0.0, 0.0, 0.0]))
    return {
        "linear": _vec(linear, 3, f"{name}.linear"),
        "angular": _vec(angular, 3, f"{name}.angular"),
    }


def _stamp(value: Any, default: float = 0.0) -> float:
    if value is None:
        return default
    if isinstance(value, Mapping):
        return _finite(value.get("sec", 0), "stamp.sec") + _finite(
            value.get("nanosec", 0), "stamp.nanosec"
        ) * 1.0e-9
    return _finite(value, "stamp")


def _distance(first: Sequence[float], second: Sequence[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def _q_inverse(q: Sequence[float]) -> tuple[float, float, float, float]:
    return (-q[0], -q[1], -q[2], q[3])


def _q_multiply(
    first: Sequence[float], second: Sequence[float]
) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _q_rotate(q: Sequence[float], vector: Sequence[float]) -> tuple[float, float, float]:
    """Rotate a vector with an XYZW quaternion."""
    rotated = _q_multiply(_q_multiply(q, (*vector, 0.0)), _q_inverse(q))
    return rotated[:3]


def _q_angle_deg(first: Sequence[float], second: Sequence[float]) -> float:
    dot = abs(sum(a * b for a, b in zip(first, second)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def _body_side(body: str) -> str | None:
    normalized = body.lower().replace("-", "_")
    if any(token in normalized for token in ("left_finger", "finger_left", "left_gripper")):
        return "left"
    if any(token in normalized for token in ("right_finger", "finger_right", "right_gripper")):
        return "right"
    return None


@dataclass(frozen=True)
class TruthFrame:
    timestamp: float
    scenario: str
    task: str
    robot: Mapping[str, Any]
    object: Mapping[str, Any] | None
    objects: tuple[Mapping[str, Any], ...]
    contacts: tuple[Mapping[str, Any], ...]
    raw: Mapping[str, Any]

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "TruthFrame":
        version = payload.get("schema_version", payload.get("version"))
        if version not in SUPPORTED_RAW_SCHEMA_VERSIONS:
            raise ValueError(f"unsupported physics truth schema version: {version!r}")
        robot_raw = payload.get("robot")
        if not isinstance(robot_raw, Mapping):
            raise ValueError("physics truth requires a robot mapping")
        robot = dict(robot_raw)
        robot["base_pose"] = _pose(robot.get("base_pose", {"xyz": [0, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]}), "robot.base_pose")
        robot["tcp_pose"] = _pose(robot.get("tcp_pose", robot["base_pose"]), "robot.tcp_pose")
        robot["base_twist"] = _twist(robot.get("base_twist", {}), "robot.base_twist")
        robot["joint_positions"] = tuple(_finite(item, "robot.joint_positions") for item in robot.get("joint_positions", []))
        robot["joint_velocities"] = tuple(_finite(item, "robot.joint_velocities") for item in robot.get("joint_velocities", []))
        robot["joint_efforts"] = tuple(_finite(item, "robot.joint_efforts") for item in robot.get("joint_efforts", []))

        def _parse_object(raw: Any, name: str) -> Mapping[str, Any]:
            if not isinstance(raw, Mapping):
                raise ValueError(f"{name} must be a mapping")
            parsed_object = dict(raw)
            parsed_object["pose"] = _pose(parsed_object.get("pose"), f"{name}.pose")
            parsed_object["twist"] = _twist(parsed_object.get("twist", {}), f"{name}.twist")
            return parsed_object

        # The public/plural contract is "objects": declared objects first,
        # spawned entities (e.g. via /spawn_entity) appended after -- see
        # backend.py truth_state(). Older payloads (and any producer that
        # still only sets the legacy singular "object") are supported by
        # falling back to a single-element list built from "object".
        objects_raw = payload.get("objects")
        objects_value: tuple[Mapping[str, Any], ...]
        if objects_raw is not None:
            if not isinstance(objects_raw, Sequence) or isinstance(objects_raw, (str, bytes)):
                raise ValueError("objects must be an array")
            objects_value = tuple(
                _parse_object(raw, f"objects[{index}]") for index, raw in enumerate(objects_raw)
            )
        else:
            object_raw = payload.get("object")
            if object_raw is None:
                objects_value = ()
            elif isinstance(object_raw, Mapping):
                objects_value = (_parse_object(object_raw, "object"),)
            else:
                raise ValueError("physics truth object must be a mapping or null")
        # Back-compat alias: "object" (singular) is always objects[0] if any
        # object is present, so existing single-object consumers (and the
        # per-object retention metrics in TruthEvaluatorCore) are unchanged.
        object_value: Mapping[str, Any] | None = objects_value[0] if objects_value else None
        contacts_value = payload.get("contacts", [])
        if not isinstance(contacts_value, Sequence) or isinstance(contacts_value, (str, bytes)):
            raise ValueError("contacts must be an array")
        contacts: list[Mapping[str, Any]] = []
        for index, raw_contact in enumerate(contacts_value):
            if not isinstance(raw_contact, Mapping):
                raise ValueError(f"contacts[{index}] must be an object")
            contact = dict(raw_contact)
            contact["body_a"] = str(contact.get("body_a", ""))
            contact["body_b"] = str(contact.get("body_b", ""))
            contact["normal_force"] = _finite(contact.get("normal_force", 0.0), "normal_force")
            contact["point"] = _vec(contact.get("point", [0, 0, 0]), 3, "contact.point")
            contact["normal"] = _vec(contact.get("normal", [0, 0, 1]), 3, "contact.normal")
            contacts.append(contact)
        return cls(
            timestamp=_stamp(payload.get("timestamp", payload.get("stamp"))),
            scenario=str(payload.get("scenario", "")),
            task=str(payload.get("task", "")),
            robot=robot,
            object=object_value,
            objects=objects_value,
            contacts=tuple(contacts),
            raw=payload,
        )


@dataclass(frozen=True)
class RetentionMetrics:
    object_present: bool
    bilateral_contact: bool
    left_contact: bool
    right_contact: bool
    lift_m: float
    translation_m: float
    drift_m: float
    drift_deg: float
    object_speed_mps: float
    stable_window_s: float
    stable: bool
    retained: bool

    def as_dict(self) -> dict[str, Any]:
        return {
            "object_present": self.object_present,
            "bilateral_contact": self.bilateral_contact,
            "left_contact": self.left_contact,
            "right_contact": self.right_contact,
            "lift_m": self.lift_m,
            "translation_m": self.translation_m,
            "drift_m": self.drift_m,
            "drift_deg": self.drift_deg,
            "object_speed_mps": self.object_speed_mps,
            "stable_window_s": self.stable_window_s,
            "stable": self.stable,
            "retained": self.retained,
        }


class JsonlWriter:
    """Append evaluator records without changing or replacing an attempt."""

    def __init__(self, path: str | Path | None) -> None:
        self.path = Path(path) if path else None
        self._stream = None
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._stream = self.path.open("a", encoding="utf-8")

    def write(self, record: Mapping[str, Any]) -> None:
        if self._stream is None:
            return
        self._stream.write(json.dumps(record, sort_keys=True, allow_nan=False) + "\n")
        self._stream.flush()

    def close(self) -> None:
        if self._stream is not None:
            self._stream.close()
            self._stream = None


class TruthEvaluatorCore:
    """Stateful evaluator for physics frames; action results are not inputs."""

    def __init__(self, thresholds: Mapping[str, float] | None = None) -> None:
        self.thresholds = {**DEFAULT_THRESHOLDS, **(thresholds or {})}
        self._initial_object_position: tuple[float, ...] | None = None
        self._relative_reference: tuple[tuple[float, ...], tuple[float, ...]] | None = None
        self._stable_start: float | None = None
        self._last_timestamp: float | None = None
        self._last_scenario: str | None = None
        self._last_task: str | None = None
        self._last_object_id: str | None = None

    def _reset_retention_state(self) -> None:
        self._initial_object_position = None
        self._relative_reference = None
        self._stable_start = None

    @staticmethod
    def _object_id(frame: TruthFrame) -> str | None:
        if frame.object is None:
            return None
        return str(frame.object.get("id", frame.object.get("object_id", "")))

    def _contacts(self, frame: TruthFrame) -> tuple[bool, bool]:
        if frame.object is None:
            return False, False
        object_id = str(frame.object.get("id", frame.object.get("object_id", ""))).lower()
        left = right = False
        for contact in frame.contacts:
            if contact["normal_force"] < self.thresholds["contact_force_n"]:
                continue
            bodies = (str(contact["body_a"]), str(contact["body_b"]))
            has_object = any(object_id and object_id in body.lower() for body in bodies)
            if not has_object:
                has_object = any("cube" in body.lower() or "object" in body.lower() for body in bodies)
            if not has_object:
                continue
            sides = {_body_side(body) for body in bodies}
            left = left or "left" in sides
            right = right or "right" in sides
        return left, right

    def evaluate(self, frame: TruthFrame) -> RetentionMetrics:
        object_id = self._object_id(frame)
        timestamp_rollback = (
            self._last_timestamp is not None and frame.timestamp < self._last_timestamp
        )
        context_changed = (
            self._last_scenario is not None
            and frame.scenario != self._last_scenario
        ) or (
            self._last_task is not None
            and frame.task != self._last_task
        ) or (
            self._last_object_id is not None
            and object_id != self._last_object_id
        )
        if timestamp_rollback or context_changed:
            self._reset_retention_state()
        self._last_timestamp = frame.timestamp
        self._last_scenario = frame.scenario
        self._last_task = frame.task
        self._last_object_id = object_id
        if frame.object is None:
            self._reset_retention_state()
            return RetentionMetrics(
                object_present=False,
                bilateral_contact=False,
                left_contact=False,
                right_contact=False,
                lift_m=0.0,
                translation_m=0.0,
                drift_m=0.0,
                drift_deg=0.0,
                object_speed_mps=0.0,
                stable_window_s=0.0,
                stable=False,
                retained=False,
            )
        object_pose = frame.object["pose"]
        tcp_pose = frame.robot["tcp_pose"]
        object_position = object_pose["position"]
        if self._initial_object_position is None:
            self._initial_object_position = object_position
        left, right = self._contacts(frame)
        if left and right and self._relative_reference is None:
            tcp_inverse = _q_inverse(tcp_pose["orientation"])
            world_delta = tuple(a - b for a, b in zip(object_position, tcp_pose["position"]))
            relative_position = _q_rotate(tcp_inverse, world_delta)
            relative_orientation = _q_multiply(tcp_inverse, object_pose["orientation"])
            self._relative_reference = (relative_position, relative_orientation)
        if self._relative_reference is None:
            drift_m = 0.0
            drift_deg = 0.0
        else:
            reference_position, reference_orientation = self._relative_reference
            tcp_inverse = _q_inverse(tcp_pose["orientation"])
            world_delta = tuple(a - b for a, b in zip(object_position, tcp_pose["position"]))
            current_position = _q_rotate(tcp_inverse, world_delta)
            drift_m = _distance(current_position, reference_position)
            current_orientation = _q_multiply(tcp_inverse, object_pose["orientation"])
            drift_deg = _q_angle_deg(current_orientation, reference_orientation)
        lift_m = object_position[2] - self._initial_object_position[2]
        translation_m = _distance(object_position, self._initial_object_position)
        linear = frame.object["twist"]["linear"]
        object_speed = math.sqrt(sum(item * item for item in linear))
        tolerance = 1.0e-12
        stable_now = (
            left
            and right
            and lift_m + tolerance >= self.thresholds["lift_m"]
            and translation_m + tolerance >= self.thresholds["translation_m"]
            and drift_m <= self.thresholds["drift_m"] + tolerance
            and drift_deg <= self.thresholds["drift_deg"] + tolerance
            and object_speed <= self.thresholds["stable_speed_mps"] + tolerance
            and not bool(frame.robot.get("safety_stop", False))
        )
        if stable_now:
            if self._stable_start is None:
                self._stable_start = frame.timestamp
            stable_window_s = max(0.0, frame.timestamp - self._stable_start)
        else:
            self._stable_start = None
            stable_window_s = 0.0
        stable = stable_window_s >= self.thresholds["hold_s"]
        return RetentionMetrics(
            object_present=True,
            bilateral_contact=left and right,
            left_contact=left,
            right_contact=right,
            lift_m=lift_m,
            translation_m=translation_m,
            drift_m=drift_m,
            drift_deg=drift_deg,
            object_speed_mps=object_speed,
            stable_window_s=stable_window_s,
            stable=stable,
            retained=stable,
        )

    def process(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        frame = TruthFrame.from_mapping(payload)
        metrics = self.evaluate(frame)
        detail = json.dumps(metrics.as_dict(), sort_keys=True, allow_nan=False)
        state = (
            "retained"
            if metrics.retained
            else "holding"
            if metrics.bilateral_contact
            else "tracking"
            if metrics.object_present
            else "no-object"
        )
        return {
            "evaluator_version": EVALUATOR_SCHEMA_VERSION,
            "timestamp": frame.timestamp,
            "scenario": frame.scenario,
            "task": frame.task,
            "metrics": metrics.as_dict(),
            "task_truth": {
                "state": state,
                "postcondition_satisfied": metrics.retained,
                "detail": detail,
            },
            "frame": frame.raw,
        }


def _set_time(message: Any, seconds: float) -> None:
    whole = int(seconds)
    nanoseconds = int(round((seconds - whole) * 1.0e9))
    message.stamp.sec = whole
    message.stamp.nanosec = nanoseconds


def _set_pose(message: Any, pose: Mapping[str, Any]) -> None:
    position = pose["position"]
    orientation = pose["orientation"]
    message.position.x, message.position.y, message.position.z = position
    message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = orientation


def _set_twist(message: Any, twist: Mapping[str, Any]) -> None:
    linear = twist["linear"]
    angular = twist["angular"]
    message.linear.x, message.linear.y, message.linear.z = linear
    message.angular.x, message.angular.y, message.angular.z = angular


try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from tinker_sim_interfaces.msg import ContactTruth, ObjectTruth, RobotTruth, TaskTruth
except ImportError:  # pragma: no cover - exercised only in non-ROS unit tests
    rclpy = None


if rclpy is not None:

    class TruthEvaluatorNode(Node):
        """ROS adapter; the only subscriber to raw physics truth."""

        def __init__(self) -> None:
            super().__init__("tinker_truth_evaluator")
            self.declare_parameter("physics_truth_topic", "/sim/internal/physics_truth")
            self.declare_parameter("jsonl_path", "")
            self.declare_parameter("raw_jsonl_path", "")
            self.declare_parameter("scenario", "")
            self.declare_parameter("task", "")
            self.declare_parameter("thresholds_json", "")
            thresholds_raw = str(self.get_parameter("thresholds_json").value)
            thresholds = json.loads(thresholds_raw) if thresholds_raw else None
            self._core = TruthEvaluatorCore(thresholds)
            self._writer = JsonlWriter(str(self.get_parameter("jsonl_path").value) or None)
            self._raw_writer = JsonlWriter(
                str(self.get_parameter("raw_jsonl_path").value) or None
            )
            reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
            self._robot_pub = self.create_publisher(RobotTruth, "/sim/truth/robot_state", reliable)
            self._object_pub = self.create_publisher(ObjectTruth, "/sim/truth/object_state", reliable)
            self._contact_pub = self.create_publisher(ContactTruth, "/sim/truth/contacts", reliable)
            self._task_pub = self.create_publisher(TaskTruth, "/sim/truth/task_state", reliable)
            self._subscription = self.create_subscription(
                String,
                str(self.get_parameter("physics_truth_topic").value),
                self._on_truth,
                reliable,
            )

        def _on_truth(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, Mapping):
                    raise ValueError("physics truth payload must be an object")
                payload = dict(payload)
                payload.setdefault("scenario", str(self.get_parameter("scenario").value))
                payload.setdefault("task", str(self.get_parameter("task").value))
                record = self._core.process(payload)
                frame = TruthFrame.from_mapping(payload)
                metrics = record["metrics"]
                robot = RobotTruth()
                _set_time(robot, frame.timestamp)
                _set_pose(robot.base_pose, frame.robot["base_pose"])
                _set_twist(robot.base_twist, frame.robot["base_twist"])
                robot.joint_names = [str(item) for item in frame.robot.get("joint_names", [])]
                robot.joint_positions = list(frame.robot["joint_positions"])
                robot.joint_velocities = list(frame.robot["joint_velocities"])
                robot.joint_efforts = list(frame.robot["joint_efforts"])
                robot.safety_stop = bool(frame.robot.get("safety_stop", False))
                self._robot_pub.publish(robot)
                for obj in frame.objects:
                    object_message = ObjectTruth()
                    _set_time(object_message, frame.timestamp)
                    object_message.object_id = str(obj.get("id", obj.get("object_id", "")))
                    object_message.class_name = str(obj.get("class_name", ""))
                    _set_pose(object_message.pose, obj["pose"])
                    _set_twist(object_message.twist, obj["twist"])
                    # Retention metrics are evaluated only for the primary
                    # (objects[0] / frame.object) task object; spawned
                    # entities past objects[0] are republished for
                    # visibility but are not the retention target.
                    object_message.retained_by_gripper = bool(metrics["retained"]) if obj is frame.object else False
                    self._object_pub.publish(object_message)
                for contact in frame.contacts:
                    contact_message = ContactTruth()
                    _set_time(contact_message, frame.timestamp)
                    contact_message.body_a = str(contact["body_a"])
                    contact_message.body_b = str(contact["body_b"])
                    contact_message.point.x, contact_message.point.y, contact_message.point.z = contact["point"]
                    contact_message.normal.x, contact_message.normal.y, contact_message.normal.z = contact["normal"]
                    contact_message.normal_force = float(contact["normal_force"])
                    self._contact_pub.publish(contact_message)
                task_message = TaskTruth()
                _set_time(task_message, frame.timestamp)
                task_message.scenario = frame.scenario
                task_message.task = frame.task
                task_message.state = record["task_truth"]["state"]
                task_message.postcondition_satisfied = bool(record["task_truth"]["postcondition_satisfied"])
                task_message.detail = record["task_truth"]["detail"]
                self._task_pub.publish(task_message)
                self._writer.write(record)
                self._raw_writer.write(payload)
            except (TypeError, ValueError, json.JSONDecodeError) as error:
                self.get_logger().error(f"invalid physics truth frame: {error}")

        def close(self) -> None:
            self._raw_writer.close()
            self._writer.close()


else:

    TruthEvaluatorNode = None  # type: ignore[assignment,misc]


def main() -> int:
    if rclpy is None or TruthEvaluatorNode is None:
        raise RuntimeError("truth_evaluator requires a sourced ROS 2 Humble environment")
    rclpy.init()
    node = TruthEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
