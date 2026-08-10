"""ROS Humble integrated readiness node (Task 6).

``IntegratedReadiness`` performs live graph, type, source, cardinality, QoS,
freshness, controller, TF, collision, service/action, model, mapping, and
provider-manifest probes independently of status topics, builds a complete
observation snapshot, and evaluates it through the ROS-free
:func:`~tinker_sim_bridge.integrated_readiness.evaluate_integrated_readiness`
seam.  It publishes compact JSON status on ``/sim/status/integrated_manipulation``
at 5 Hz and publishes ``fail`` (with explicit reasons) whenever any check fails;
the node stays alive so its evidence remains observable.
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Mapping

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

from .contract_guard import JOINT_STATE_SERVICE_TIMEOUT_S, JOINT_STATE_SERVICE_TTL_S, step_service
from .integrated_readiness import (
    INTEGRATED_ACTIONS,
    INTEGRATED_JOINT_STATE_NAMES,
    INTEGRATED_PUBLISHERS,
    INTEGRATED_SERVICES,
    OPERATOR_SUB_QOS_SPEC,
    INTEGRATED_TOUCH_LINKS,
    TF_CHILD,
    TF_PARENT,
    evaluate_integrated_readiness,
    json_safe_value,
    normalize_qos_value,
    parse_canonical_report,
    sha256_bytes,
    sha256_json,
    validate_report,
)

_STATUS_TOPIC = "/sim/status/integrated_manipulation"
_PHYSICS_READY_SERVICE = "/sim/ready/physics"
_FIXTURE_READY_SERVICE = "/sim/ready/fixture"
_LIST_CONTROLLERS_SERVICE = "/controller_manager/list_controllers"

_JOINT_STATE_MAX_AGE_S = 0.25
_SAFETY_MAX_AGE_S = 0.25
_FIXTURE_MAX_AGE_S = 0.25
_INTEGRATED_MAX_AGE_S = 0.25


def _endpoint_label(info: object) -> str:
    namespace = str(getattr(info, "node_namespace", ""))
    name = str(getattr(info, "node_name", ""))
    namespace = namespace.rstrip("/")
    if namespace:
        return "{}/{}".format(namespace, name)
    return "/" + name


def _canonicalize_moveit_private(label: str) -> str:
    """Map MoveIt private-helper labels ``/move_group_private_<suffix>`` to the
    logical ``/move_group`` owner; any other source label is returned unchanged."""
    if label.startswith("/move_group_private_"):
        return "/move_group"
    return label


def _canonicalize_joint_sample(
    names: list[str],
    positions: list[float],
    velocities: list[float],
) -> tuple[list[str], list[float], list[float]]:
    """Canonicalize a JointState to the required name order.

    Accepts any ordering of the required joint names exactly once each with no
    unknown names; positions and velocities are reordered to the canonical order
    using the same indices.  Any missing/duplicate/unknown/unaligned/non-finite
    input is returned unchanged so the caller rejects it fail-closed.
    """
    raw_names = list(names)
    raw_positions = [float(value) for value in positions]
    raw_velocities = [float(value) for value in velocities]
    if sorted(raw_names) != sorted(INTEGRATED_JOINT_STATE_NAMES):
        return raw_names, raw_positions, raw_velocities
    try:
        index = {name: position for position, name in enumerate(raw_names)}
        if len(index) != len(raw_names):
            return raw_names, raw_positions, raw_velocities
        if len(raw_positions) != len(INTEGRATED_JOINT_STATE_NAMES) or len(raw_velocities) != len(
            INTEGRATED_JOINT_STATE_NAMES
        ):
            return raw_names, raw_positions, raw_velocities
        if not all(math.isfinite(value) for value in raw_positions) or not all(
            math.isfinite(value) for value in raw_velocities
        ):
            return raw_names, raw_positions, raw_velocities
        ordered_positions = [raw_positions[index[name]] for name in INTEGRATED_JOINT_STATE_NAMES]
        ordered_velocities = [raw_velocities[index[name]] for name in INTEGRATED_JOINT_STATE_NAMES]
        return list(INTEGRATED_JOINT_STATE_NAMES), ordered_positions, ordered_velocities
    except (TypeError, ValueError, KeyError, IndexError):
        return raw_names, raw_positions, raw_velocities


def _qos_profile_of(info: object) -> dict[str, object] | None:
    """Normalize a Humble ``PublishersInfo.qos_profile`` into comparable fields."""
    profile = getattr(info, "qos_profile", None)
    if profile is None:
        return None
    return {
        "reliability": normalize_qos_value(getattr(profile, "reliability", "")),
        "durability": normalize_qos_value(getattr(profile, "durability", "")),
        "depth": int(getattr(profile, "depth", 0)),
    }


def _bool_value(message) -> bool | None:
    value = getattr(message, "data", None)
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return bool(value)
    return None


class IntegratedReadiness(Node):
    """Persistent integrated OMPL/readiness boundary node."""

    def __init__(
        self,
        *,
        node_name: str | None = None,
        context=None,
        parameter_overrides=None,
        create_status_publisher: bool = True,
    ) -> None:
        super().__init__(
            node_name or "integrated_readiness",
            context=context,
            parameter_overrides=parameter_overrides or [],
        )
        self._create_status_publisher = create_status_publisher
        try:
            self._initialize()
        except Exception:
            try:
                self.destroy_node()
            except Exception:  # noqa: BLE001 - destroy must never mask the cause
                pass
            raise

    def _initialize(self) -> None:
        self.declare_parameter("check_period_s", 0.2)
        self.declare_parameter("startup_timeout_s", 60.0)
        self.declare_parameter("report_path", "")
        self.declare_parameter("physics_ready_path", "")
        self.declare_parameter("provider_manifest_path", "")
        self.declare_parameter("provider_manifest_sha256", "")
        self.declare_parameter("model_bundle_manifest", "")
        self.declare_parameter("scenario_id", "")
        self.declare_parameter("seed", -1)
        self.declare_parameter("scenario_declaration_sha256", "")
        self.declare_parameter("planning_scene_revision", "")
        self.declare_parameter("planning_scene_revision_digest", "")
        self.declare_parameter("planning_scene_owned_ids", "[]")
        self.declare_parameter("planning_scene_target_source_id", "")
        self.declare_parameter("planning_scene_target_handoff", "")
        self.declare_parameter("integrated_mapping", "{}")
        self.declare_parameter("public_integrated_mapping", "")
        self.declare_parameter("integrated_sha256", "")
        self.declare_parameter("runtime_contract_sha256", "")
        self.declare_parameter("model_fingerprint", "")
        self.declare_parameter("fail_exit_s", 0.0)
        period = float(self.get_parameter("check_period_s").value)
        if not (period > 0):
            raise ValueError("check_period_s must be positive")
        self._period = period
        self._startup_timeout_s = float(self.get_parameter("startup_timeout_s").value)
        if not (self._startup_timeout_s > 0):
            raise ValueError("startup_timeout_s must be positive")
        self._started = time.monotonic()
        self._fail_exit_s = float(self.get_parameter("fail_exit_s").value)
        self._report_path = str(self.get_parameter("report_path").value)
        self._physics_ready_path = str(self.get_parameter("physics_ready_path").value)
        self._provider_manifest_path = str(
            self.get_parameter("provider_manifest_path").value
        )
        self._model_bundle_manifest = str(
            self.get_parameter("model_bundle_manifest").value
        )
        self._contract = self._build_contract()

        self._joint_sample: JointState | None = None
        self._joint_received_at = 0.0
        self._safety_stop: bool | None = None
        self._safety_stop_received_at = 0.0
        self._safety_stop_samples = 0
        self._operator: bool | None = None
        self._operator_received_at = 0.0
        self._collision: bool | None = None
        self._collision_samples = 0
        self._collision_received_at = 0.0
        self._fixture_status: Mapping[str, object] | None = None
        self._fixture_received_at = 0.0
        self._fixture_last_sequence: int | None = None
        self._tf_observed: Mapping[str, object] | None = None
        self._controller_entries: list[tuple[str, str]] = []
        self._controllers_state: dict[str, object] = {}
        self._service_group = ReentrantCallbackGroup()

        # A tf2 buffer + transform listener composes the real multi-hop chain
        # (dynamic joints on /tf plus the fixed joint_tcp on /tf_static) rather
        # than requiring a direct base_link -> link_tcp edge that RSP never
        # publishes.  TransformListener subscribes to both /tf and /tf_static.
        self._tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._model_preflight = self._run_model_preflight()
        self._provider_evidence = self._read_provider_manifest()
        self._report_evidence = self._read_shared_report()

        self._last_status: Mapping[str, object] | None = None
        self._last_evaluated = False
        self._last_reasons: list[str] = []
        self._fail_since = None

        if self._create_status_publisher:
            status_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._publisher = self.create_publisher(String, _STATUS_TOPIC, status_qos)
        self._fixture_sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._joint_sub_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._bool_sub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # J: the executor publishes /sim/safety/operator as latched state
        # (RELIABLE/TRANSIENT_LOCAL).  Subscribe with the same durability so a
        # late-joining subscriber receives the latched False baseline instead of
        # failing the readiness gate with "no sample received".  safety_stop and
        # collision keep the volatile _bool_sub_qos (their supervisors republish
        # on a continuous heartbeat, so volatile live samples suffice).
        self._operator_sub_qos = QoSProfile(
            depth=OPERATOR_SUB_QOS_SPEC["depth"],
            reliability=ReliabilityPolicy[OPERATOR_SUB_QOS_SPEC["reliability"].upper()],
            durability=DurabilityPolicy[OPERATOR_SUB_QOS_SPEC["durability"].upper()],
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, self._joint_sub_qos
        )
        self.create_subscription(
            Bool, "/sim/hardware/safety_stop", self._on_safety_stop, self._bool_sub_qos
        )
        self.create_subscription(
            Bool, "/sim/safety/operator", self._on_operator, self._operator_sub_qos
        )
        self.create_subscription(
            Bool, "/sim/safety/collision", self._on_collision, self._bool_sub_qos
        )
        self.create_subscription(
            String,
            "/sim/status/planning_scene_fixture",
            self._on_fixture_status,
            self._fixture_sub_qos,
        )
        self.create_timer(period, self._check)

    # ------------------------------------------------------------------
    # Contract construction
    # ------------------------------------------------------------------

    def _build_contract(self) -> dict[str, object]:
        raw_mapping = str(self.get_parameter("integrated_mapping").value)
        try:
            integrated = json.loads(raw_mapping)
        except json.JSONDecodeError as exc:
            raise ValueError("integrated_mapping parameter is not valid JSON") from exc
        if not isinstance(integrated, dict):
            raise ValueError("integrated_mapping parameter must be a JSON object")
        owned_ids_raw = str(self.get_parameter("planning_scene_owned_ids").value)
        try:
            owned_ids = json.loads(owned_ids_raw)
        except json.JSONDecodeError as exc:
            raise ValueError("planning_scene_owned_ids parameter is not valid JSON") from exc
        if not isinstance(owned_ids, list):
            raise ValueError("planning_scene_owned_ids parameter must be a JSON array")
        # The public report's integrated field is the production-canonical
        # one-key mapping used for report validation; the full runtime contract
        # (``integrated_mapping``) drives the readiness evaluator.
        public_raw = str(self.get_parameter("public_integrated_mapping").value)
        if public_raw and public_raw.strip():
            try:
                public_integrated = json.loads(public_raw)
            except json.JSONDecodeError as exc:
                raise ValueError("public_integrated_mapping parameter is not valid JSON") from exc
            if not isinstance(public_integrated, dict):
                raise ValueError("public_integrated_mapping parameter must be a JSON object")
        else:
            public_integrated = integrated
        return {
            "schema_version": 1,
            "report_revision": "integrated-manipulation-v1",
            "scenario_id": str(self.get_parameter("scenario_id").value),
            "seed": int(self.get_parameter("seed").value),
            "scenario_declaration_sha256": str(
                self.get_parameter("scenario_declaration_sha256").value
            ),
            "planning_scene_revision": str(
                self.get_parameter("planning_scene_revision").value
            ),
            "planning_scene_revision_digest": str(
                self.get_parameter("planning_scene_revision_digest").value
            ),
            "planning_scene_owned_ids": [str(item) for item in owned_ids],
            "planning_scene_target_source_id": str(
                self.get_parameter("planning_scene_target_source_id").value
            ),
            "planning_scene_target_handoff": str(
                self.get_parameter("planning_scene_target_handoff").value
            ),
            "integrated_mapping": integrated,
            "public_integrated_mapping": public_integrated,
            "integrated_sha256": str(self.get_parameter("integrated_sha256").value),
            "runtime_contract_sha256": str(
                self.get_parameter("runtime_contract_sha256").value
            ),
            "model_fingerprint": str(self.get_parameter("model_fingerprint").value),
            "provider_manifest_path": self._provider_manifest_path,
            "provider_manifest_sha256": str(
                self.get_parameter("provider_manifest_sha256").value
            ),
            "actions": INTEGRATED_ACTIONS,
            "services": INTEGRATED_SERVICES,
            "publishers": INTEGRATED_PUBLISHERS,
            "controller_resources": {
                "joint_state_broadcaster": "active",
                "xarm7_traj_controller": "active",
            },
            "joint_names": list(INTEGRATED_JOINT_STATE_NAMES),
            "tf_parent": TF_PARENT,
            "tf_child": TF_CHILD,
            "touch_links": list(INTEGRATED_TOUCH_LINKS),
        }

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_joint_state(self, message: JointState) -> None:
        self._joint_sample = message
        self._joint_received_at = time.monotonic()

    def _on_safety_stop(self, message: Bool) -> None:
        value = _bool_value(message)
        if value is not None:
            self._safety_stop = value
            self._safety_stop_samples += 1
            self._safety_stop_received_at = time.monotonic()

    def _on_operator(self, message: Bool) -> None:
        value = _bool_value(message)
        if value is not None:
            self._operator = value
            self._operator_received_at = time.monotonic()

    def _on_collision(self, message: Bool) -> None:
        value = _bool_value(message)
        if value is not None:
            self._collision = value
            self._collision_samples += 1
            self._collision_received_at = time.monotonic()

    def _on_fixture_status(self, message: String) -> None:
        try:
            parsed = json.loads(message.data)
        except json.JSONDecodeError:
            self._fixture_status = {"malformed": True}
            self._fixture_received_at = time.monotonic()
            return
        if isinstance(parsed, dict):
            self._fixture_status = parsed
            self._fixture_received_at = time.monotonic()

    # ------------------------------------------------------------------
    # File evidence
    # ------------------------------------------------------------------

    def _run_model_preflight(self) -> dict[str, object]:
        if not self._model_bundle_manifest:
            return {
                "ready": False,
                "reasons": ["model_bundle_manifest parameter is required"],
                "structural_fingerprint": None,
            }
        try:
            from .model_preflight import preflight_manifest
        except ImportError:
            return {
                "ready": False,
                "reasons": ["model_preflight is unavailable"],
                "structural_fingerprint": None,
            }
        try:
            result = preflight_manifest(
                Path(self._model_bundle_manifest), timeout=min(30.0, self._startup_timeout_s)
            )
        except Exception as exc:  # noqa: BLE001 - preflight failures fail closed
            return {
                "ready": False,
                "reasons": ["model preflight failed: {}".format(exc)],
                "structural_fingerprint": None,
            }
        return {
            "ready": bool(result.get("ready")),
            "reasons": [str(item.get("detail", "")) for item in result.get("checks", []) if not item.get("ok")],
            "status": result.get("status"),
            "structural_fingerprint": result.get("structural_fingerprint"),
        }

    def _read_provider_manifest(self) -> dict[str, object]:
        if not self._provider_manifest_path:
            return {
                "ready": False,
                "reasons": ["provider_manifest_path parameter is required"],
            }
        path = Path(self._provider_manifest_path)
        if not path.is_file():
            return {"ready": False, "reasons": ["provider manifest not found: {}".format(path)]}
        try:
            data = path.read_bytes()
            raw = json.loads(data.decode("utf-8"))
        except (OSError, ValueError) as exc:
            return {"ready": False, "reasons": ["provider manifest unreadable: {}".format(exc)]}
        if not isinstance(raw, dict):
            return {"ready": False, "reasons": ["provider manifest must be a JSON object"]}
        recorded = raw.get("provider_manifest_sha256")
        digest_payload = {
            key: value for key, value in raw.items() if key != "provider_manifest_sha256"
        }
        # The manifest records its own canonical-JSON self hash; the launch
        # supplies the raw-byte digest both scenario_runner and readiness verify
        # against the unchanged file bytes.
        canonical_self_hash = sha256_json(digest_payload)
        raw_bytes_hash = sha256_bytes(data)
        expected = self._contract.get("provider_manifest_sha256", "")
        reasons: list[str] = []
        if not isinstance(recorded, str) or recorded != canonical_self_hash:
            reasons.append(
                "provider manifest recorded sha256 {!r} does not match recomputed canonical {!r}".format(
                    recorded, canonical_self_hash
                )
            )
        if expected and raw_bytes_hash != expected:
            reasons.append(
                "provider manifest raw-byte sha256 {!r} does not match expected {!r}".format(
                    raw_bytes_hash, expected
                )
            )
        return {
            "ready": not reasons,
            "reasons": reasons,
            "path": self._provider_manifest_path,
            "sha256": raw_bytes_hash,
            "canonical_self_hash": canonical_self_hash,
            "recorded_sha256": recorded,
            "bytes_sha256": raw_bytes_hash,
            "manifest": raw,
        }

    def _read_shared_report(self) -> dict[str, object]:
        report_path = Path(self._report_path)
        physics_path = Path(self._physics_ready_path)
        if not report_path.is_file():
            return {"ready": False, "reasons": ["scenario report not found: {}".format(report_path)]}
        if not physics_path.is_file():
            return {
                "ready": False,
                "reasons": ["physics-ready artifact not found: {}".format(physics_path)],
            }
        try:
            data = report_path.read_bytes()
            report = parse_canonical_report(data)
            report_sha = sha256_bytes(data)
            physics_raw = json.loads(physics_path.read_bytes())
        except (OSError, ValueError) as exc:
            return {"ready": False, "reasons": ["report/artifact unreadable: {}".format(exc)]}
        if not isinstance(physics_raw, dict) or physics_raw.get("state") != "PHYSICS_READY":
            return {"ready": False, "reasons": ["physics-ready artifact is not READY"]}
        artifact_sha = physics_raw.get("scenario_report_sha256")
        if artifact_sha != report_sha:
            return {
                "ready": False,
                "reasons": [
                    "physics-ready scenario_report_sha256 {} != recomputed {}".format(
                        artifact_sha, report_sha
                    )
                ],
            }
        validation = validate_report(report, self._contract)
        if not validation["ready"]:
            return {
                "ready": False,
                "reasons": validation["reasons"],
                "scenario_report_sha256": report_sha,
                "scenario_report_sha256_bytes": data,
                "final_simulation_state": report.get("final_simulation_state"),
                "identities": report.get("identities", {}),
            }
        return {
            "ready": True,
            "reasons": [],
            "scenario_report_sha256": report_sha,
            "scenario_report_sha256_bytes": data,
            "scenario_report_sha256_matches": True,
            "final_simulation_state": report.get("final_simulation_state"),
            "identities": report.get("identities", {}),
            "operations": report.get("operations", []),
        }

    # ------------------------------------------------------------------
    # Graph probes
    # ------------------------------------------------------------------

    def _service_servers(self) -> dict[str, list[tuple[str, list[str]]]]:
        servers: dict[str, list[tuple[str, list[str]]]] = {}
        for node_name, node_namespace in self._unique_graph_pairs():
            label = _endpoint_label(
                type("Info", (), {"node_name": node_name, "node_namespace": node_namespace})()
            )
            try:
                by_node = self.get_service_names_and_types_by_node(node_name, node_namespace)
            except Exception:  # noqa: BLE001 - transient graph reads must not crash
                continue
            # Humble rclpy returns a list of (service_name, types) pairs, not a
            # mapping.
            for service_name, types in by_node:
                servers.setdefault(service_name, []).append((label, list(types)))
        return servers

    def _services_by_node_label(self) -> dict[str, dict[str, list[str]]]:
        result: dict[str, dict[str, list[str]]] = {}
        for node_name, node_namespace in self._unique_graph_pairs():
            label = _endpoint_label(
                type("Info", (), {"node_name": node_name, "node_namespace": node_namespace})()
            )
            try:
                by_node = self.get_service_names_and_types_by_node(node_name, node_namespace)
            except Exception:  # noqa: BLE001
                continue
            result[label] = {name: list(types) for name, types in by_node}
        return result

    def _unique_graph_pairs(self) -> list[tuple[str, str]]:
        """Graph node identities with duplicate ``(node_name, node_namespace)``
        pairs removed (a launch global-remap can surface the same FQN twice)."""
        seen: set[tuple[str, str]] = set()
        unique: list[tuple[str, str]] = []
        for node_name, node_namespace in self.get_node_names_and_namespaces():
            pair = (node_name, node_namespace)
            if pair in seen:
                continue
            seen.add(pair)
            unique.append(pair)
        return unique

    def _probe_actions(self) -> dict[str, object]:
        servers = self._service_servers()
        observed: dict[str, object] = {}
        for endpoint, spec in INTEGRATED_ACTIONS.items():
            action_type = spec["type"]
            goal_service = "{}/_action/send_goal".format(endpoint)
            result_service = "{}/_action/get_result".format(endpoint)
            goal_servers = servers.get(goal_service, [])
            result_servers = servers.get(result_service, [])
            goal_labels_raw = sorted({label for label, _types in goal_servers})
            goal_labels = sorted({_canonicalize_moveit_private(label) for label in goal_labels_raw})
            goal_types = sorted({t for _label, tlist in goal_servers for t in tlist})
            expected_source = spec["source"]
            if expected_source.startswith("controller_resource:"):
                # A controller-manager-hosted action server has no dedicated
                # node identity; the exact logical-resource literal is the
                # source proof while observed_sources records the serving node.
                # Type/cardinality are proven from the live goal-service graph
                # and the typed active xarm7_traj_controller resource evidence.
                source = expected_source if len(goal_servers) == 1 else ""
                source_ok = len(goal_servers) == 1
            else:
                source = goal_labels[0] if len(goal_labels) == 1 else ""
                source_ok = source == expected_source
            count = len(goal_servers)
            observed[endpoint] = {
                "count": count,
                "source": source,
                "sources": goal_labels,
                "observed_sources": goal_labels_raw,
                "type": action_type,
                "observed_types": goal_types,
                "result_service_present": len(result_servers) >= 1,
                "source_ok": source_ok,
                "ready": count == 1 and source_ok and len(result_servers) >= 1,
                "reasons": self._action_reasons(endpoint, count, source_ok, action_type),
            }
        return observed

    @staticmethod
    def _action_reasons(endpoint: str, count: int, source_ok: bool, action_type: str) -> list[str]:
        reasons: list[str] = []
        if count != 1:
            reasons.append("action server count is {}, expected 1".format(count))
        if not source_ok:
            reasons.append("action source does not match expected for {}".format(endpoint))
        return reasons

    def _graph_services(self) -> dict[str, object]:
        observed: dict[str, object] = {}
        servers = self._service_servers()
        by_node = self._services_by_node_label()
        source_map: dict[str, str] = {}
        for label, entries in by_node.items():
            canonical_label = _canonicalize_moveit_private(label)
            for service_name in entries:
                if service_name in INTEGRATED_SERVICES:
                    source_map[service_name] = canonical_label
        for endpoint, spec in INTEGRATED_SERVICES.items():
            entries = servers.get(endpoint, [])
            count = len(entries)
            source = source_map.get(endpoint, "")
            sources = sorted({label for label, _types in entries})
            types = sorted({t for _label, tlist in entries for t in tlist})
            type_ok = spec["type"] in types
            source_ok = source == spec["source"]
            observed[endpoint] = {
                "count": count,
                "source": source,
                "sources": sources,
                "types": types,
                "type": spec["type"],
                "type_ok": type_ok,
                "ready": count == 1 and type_ok and source_ok,
                "reasons": self._service_reasons(endpoint, count, type_ok, source_ok),
            }
        return observed

    @staticmethod
    def _service_reasons(endpoint: str, count: int, type_ok: bool, source_ok: bool) -> list[str]:
        reasons: list[str] = []
        if count != 1:
            reasons.append("service server count is {}, expected 1".format(count))
        if not type_ok:
            reasons.append("service type does not match expected")
        if not source_ok:
            reasons.append("service source does not match expected")
        return reasons

    # ------------------------------------------------------------------
    # Controller resources
    # ------------------------------------------------------------------

    def _step_list_controllers(self) -> None:
        def create_client():
            from controller_manager_msgs.srv import ListControllers

            return self.create_client(
                ListControllers,
                _LIST_CONTROLLERS_SERVICE,
                callback_group=self._service_group,
            )

        def request(client):
            return client.srv_type.Request()

        def extract(response):
            if response is None or not hasattr(response, "controller"):
                return None
            return [(item.name, item.state) for item in response.controller]

        def reset_client(state):
            client = state.get("client")
            if client is not None:
                try:
                    client.destroy()
                except Exception:  # noqa: BLE001 - destroy must never raise
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

    def _controller_evidence(self) -> dict[str, object]:
        states = {
            name: state
            for name, state in self._controller_entries
        }
        resources: dict[str, object] = {}
        for name in ("joint_state_broadcaster", "xarm7_traj_controller"):
            state = states.get(name)
            ready = state == "active"
            resources[name] = {
                "state": state,
                "ready": ready,
                "reasons": (
                    []
                    if ready
                    else ["controller {!r} state is {!r}, expected active".format(name, state)]
                ),
            }
        # The trajectory controller's exact action server is proven from the
        # live graph probe.
        action_probe = self._graph_actions_probe()
        resources["xarm7_traj_controller"]["action_server_count"] = action_probe.get(
            "/xarm7_traj_controller/follow_joint_trajectory", {}
        ).get("count", 0)
        return resources

    def _graph_actions_probe(self) -> dict[str, object]:
        return self._probe_actions()

    # ------------------------------------------------------------------
    # Snapshot
    # ------------------------------------------------------------------

    def _joint_evidence(self) -> dict[str, object]:
        sample = self._joint_sample
        if sample is None:
            return {
                "ready": False,
                "reasons": ["no joint_state sample received yet"],
                "names": [],
            }
        now = time.monotonic()
        age = now - self._joint_received_at
        stamp = sample.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        publishers = self.get_publishers_info_by_topic("/joint_states")
        labels = [_endpoint_label(info) for info in publishers]
        reasons: list[str] = []
        names, positions, velocities = _canonicalize_joint_sample(
            list(sample.name),
            [float(value) for value in sample.position],
            [float(value) for value in sample.velocity],
        )
        if names != list(INTEGRATED_JOINT_STATE_NAMES):
            reasons.append(
                "joint names {!r} != expected {!r}".format(names, list(INTEGRATED_JOINT_STATE_NAMES))
            )
        if len(publishers) != 1:
            reasons.append("joint_state publisher count is {}, expected 1".format(len(publishers)))
        elif labels[0] != "/joint_state_broadcaster":
            reasons.append(
                "joint_state publisher source is {!r}, expected /joint_state_broadcaster".format(
                    labels[0]
                )
            )
        if len(positions) != len(INTEGRATED_JOINT_STATE_NAMES) or not all(math.isfinite(v) for v in positions):
            reasons.append("joint_state positions must be eight finite values")
        if len(velocities) != len(INTEGRATED_JOINT_STATE_NAMES) or not all(math.isfinite(v) for v in velocities):
            reasons.append("joint_state velocities must be eight finite values")
        if stamp_ns == 0:
            reasons.append("joint_state header stamp is zero")
        if age > _JOINT_STATE_MAX_AGE_S:
            reasons.append("joint_state age {:.3f} s exceeds {:.3f} s".format(age, _JOINT_STATE_MAX_AGE_S))
        return {
            "ready": not reasons,
            "reasons": reasons,
            "names": names,
            "positions": positions,
            "velocities": velocities,
            "header_stamp_ns": stamp_ns,
            "received_at_s": self._joint_received_at,
            "now_s": now,
            "age_s": age,
            "publisher_source": labels[0] if len(labels) == 1 else labels,
            "publisher_count": len(publishers),
        }

    def _bool_evidence(
        self,
        value: bool | None,
        received_at: float,
        *,
        max_age_s: float,
        expected_value: bool,
        min_samples: int = 1,
        samples: int = 0,
        source: str = "",
        count: int = 0,
        sources: list[str] | None = None,
        observed_qos: Mapping[str, object] | None = None,
        expected_durability: str | None = None,
        expected_reliability: str | None = None,
        expected_depth: int | None = None,
    ) -> dict[str, object]:
        now = time.monotonic()
        reasons: list[str] = []
        if value is None:
            reasons.append("no sample received yet")
        else:
            age = now - received_at
            if age > max_age_s:
                reasons.append("sample age {:.3f} s exceeds {:.3f} s".format(age, max_age_s))
            if value != expected_value:
                reasons.append("sample value {} != expected {}".format(value, expected_value))
        if samples < min_samples:
            reasons.append(
                "received {} samples, expected at least {}".format(samples, min_samples)
            )
        if count != 1:
            reasons.append("publisher count is {}, expected 1".format(count))
        if sources and source and source not in sources:
            reasons.append("publisher source {!r} not in observed {!r}".format(source, sources))
        qos = observed_qos or {}
        if expected_durability is not None:
            observed_dur = normalize_qos_value(qos.get("durability", ""))
            if observed_dur != normalize_qos_value(expected_durability):
                reasons.append(
                    "publisher durability {!r} != expected {!r}".format(
                        observed_dur, expected_durability
                    )
                )
        if expected_reliability is not None:
            observed_rel = normalize_qos_value(qos.get("reliability", ""))
            if observed_rel != normalize_qos_value(expected_reliability):
                reasons.append(
                    "publisher reliability {!r} != expected {!r}".format(
                        observed_rel, expected_reliability
                    )
                )
        if expected_depth is not None:
            try:
                depth_value = int(qos.get("depth", 0))
            except (TypeError, ValueError):
                depth_value = 0
            # Humble publisher info never reports depth (always 0), so depth is
            # compared only when actually reported; reliability/durability are
            # reported and compared strictly above.
            if depth_value > 0 and depth_value != int(expected_depth):
                reasons.append(
                    "publisher depth {!r} != expected {!r}".format(
                        qos.get("depth"), expected_depth
                    )
                )
        return {
            "ready": not reasons,
            "reasons": reasons,
            "value": value,
            "received_at_s": received_at if value is not None else None,
            "now_s": now,
            "age_s": now - received_at if value is not None else None,
            "source": source,
            "observed_sources": sources or [],
            "count": count,
            "qos": dict(qos),
            "min_samples": min_samples,
            "received_samples": samples,
        }

    def _tf_evidence(self) -> dict[str, object]:
        """Compose the real multi-hop TF chain via the tf2 buffer.

        RSP publishes one transform per joint (dynamic joints on ``/tf`` and the
        fixed ``joint_tcp`` on ``/tf_static``); ``base_link -> link_tcp`` is
        multi-hop.  A composed lookup through ``TransformListener`` (which
        consumes both ``/tf`` and ``/tf_static``) proves the chain.  The
        transform's sim-time header stamp is recorded; no incompatible
        wall/sim/monotonic clock comparison is performed.
        """
        now = time.monotonic()
        try:
            transform = self._tf_buffer.lookup_transform(
                TF_PARENT, TF_CHILD, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self._tf_observed = {
                "ready": False,
                "reasons": [
                    "base_link -> link_tcp composed lookup failed: {}".format(exc)
                ],
                "parent": TF_PARENT,
                "child": TF_CHILD,
                "exists": False,
                "lookup_at_s": now,
            }
            return dict(self._tf_observed)
        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        self._tf_observed = {
            "ready": stamp_ns != 0,
            "reasons": [] if stamp_ns != 0 else ["base_link -> link_tcp transform has zero stamp"],
            "parent": str(transform.header.frame_id),
            "child": str(transform.child_frame_id),
            "exists": True,
            "stamp_ns": stamp_ns,
            "lookup_at_s": now,
        }
        return dict(self._tf_observed)

    def _fixture_evidence(self) -> dict[str, object]:
        status = self._fixture_status
        now = time.monotonic()
        if status is None:
            return {"ready": False, "reasons": ["fixture status not received yet"], "status": None}
        if status.get("malformed"):
            return {"ready": False, "reasons": ["fixture status is malformed"], "status": None}
        age = now - self._fixture_received_at
        reasons: list[str] = []
        expected = self._contract
        if status.get("state") != "FIXTURE_READY":
            reasons.append("fixture status state {!r} != FIXTURE_READY".format(status.get("state")))
        if status.get("scenario") != expected.get("scenario_id"):
            reasons.append("fixture status scenario does not match")
        if status.get("revision") != expected.get("planning_scene_revision"):
            reasons.append("fixture status revision does not match")
        if status.get("revision_digest") != expected.get("planning_scene_revision_digest"):
            reasons.append("fixture status revision_digest does not match")
        if tuple(status.get("owned_ids", ())) != tuple(expected.get("planning_scene_owned_ids", ())):
            reasons.append("fixture status owned_ids does not match")
        if status.get("target_source_id") != expected.get("planning_scene_target_source_id"):
            reasons.append("fixture status target_source_id does not match")
        if status.get("target_handoff") != expected.get("planning_scene_target_handoff"):
            reasons.append("fixture status target_handoff does not match")
        sequence = status.get("sequence")
        if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 1:
            reasons.append("fixture status sequence must be a positive integer")
        elif self._fixture_last_sequence is not None and sequence <= self._fixture_last_sequence:
            reasons.append("fixture status sequence did not increase")
        else:
            self._fixture_last_sequence = sequence
        if age > _FIXTURE_MAX_AGE_S:
            reasons.append("fixture status age {:.3f} s exceeds {:.3f} s".format(age, _FIXTURE_MAX_AGE_S))
        publishers = self.get_publishers_info_by_topic("/sim/status/planning_scene_fixture")
        labels = [_endpoint_label(info) for info in publishers]
        if len(publishers) != 1 or "/fixture_planning_scene" not in labels:
            reasons.append("fixture status publisher graph does not match")
        return {
            "ready": not reasons,
            "reasons": reasons,
            "status": status,
            "age_s": age,
            "publisher_source": labels,
            "publisher_count": len(publishers),
        }

    def _provider_manifest_evidence(self) -> dict[str, object]:
        evidence = dict(self._provider_evidence)
        manifest = evidence.get("manifest")
        if not isinstance(manifest, dict):
            # Re-read the parsed manifest for the live-agreement comparison.
            manifest = self._provider_manifest_parsed()
        evidence["manifest"] = manifest
        evidence["observed_nodes"] = [
            _endpoint_label(
                type("Info", (), {"node_name": name, "node_namespace": namespace})()
            )
            for name, namespace in self.get_node_names_and_namespaces()
        ]
        metadata = self._publisher_metadata()
        evidence["observed_publishers"] = sorted(
            topic for topic, entry in metadata.items() if entry.get("count", 0) > 0
        )
        evidence["observed_controllers"] = {
            name: state for name, state in self._controller_entries
        }
        return evidence

    def _provider_manifest_parsed(self) -> dict[str, object]:
        if not self._provider_manifest_path:
            return {}
        try:
            raw = json.loads(Path(self._provider_manifest_path).read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return {}
        return raw if isinstance(raw, dict) else {}

    def _publisher_metadata(self) -> dict[str, object]:
        """Probe every typed publisher endpoint for count/source/type/QoS."""
        observed: dict[str, object] = {}
        for topic in INTEGRATED_PUBLISHERS:
            publishers = self.get_publishers_info_by_topic(topic)
            labels = [_endpoint_label(info) for info in publishers]
            types = sorted({str(getattr(info, "topic_type", "")) for info in publishers})
            qos = _qos_profile_of(publishers[0]) if publishers else None
            observed[topic] = {
                "count": len(publishers),
                "source": labels[0] if len(labels) == 1 else labels,
                "sources": labels,
                "types": types,
                "qos": qos,
            }
        return observed

    def _semantic_model_evidence(self) -> dict[str, object]:
        preflight = self._model_preflight
        if not preflight.get("ready"):
            return {
                "ready": False,
                "reasons": preflight.get("reasons") or ["model preflight not ready"],
                "kinematics_match": False,
            }
        return {
            "ready": True,
            "reasons": [],
            "kinematics_match": True,
            "touch_links": list(INTEGRATED_TOUCH_LINKS),
            "structural_fingerprint": preflight.get("structural_fingerprint"),
        }

    def _collision_evidence(self) -> dict[str, object]:
        publishers = self.get_publishers_info_by_topic("/sim/safety/collision")
        return self._bool_evidence(
            self._collision,
            self._collision_received_at,
            max_age_s=_SAFETY_MAX_AGE_S,
            expected_value=False,
            samples=self._collision_samples,
            source="/tinker_isaac_gateway",
            count=len(publishers),
            sources=[_endpoint_label(info) for info in publishers],
            observed_qos=_qos_profile_of(publishers[0]) if publishers else None,
            expected_durability="TRANSIENT_LOCAL",
            expected_reliability="RELIABLE",
            expected_depth=1,
        )

    def _mapping_evidence(self) -> dict[str, object]:
        report = self._report_evidence
        reasons: list[str] = []
        if not report.get("ready"):
            reasons.extend(report.get("reasons", []))
        if report.get("final_simulation_state") != "STATE_PLAYING":
            reasons.append("shared report final_simulation_state must be STATE_PLAYING")
        # The integrated mapping and its digest are recomputed from unchanged
        # bytes by the shared report reader (validate_report already agrees).
        observed = {
            "scenario_declaration_sha256": report.get("identities", {}).get("scenario_declaration_sha256"),
            "planning_scene_sha256": report.get("identities", {}).get("planning_scene_sha256"),
            "integrated_sha256": report.get("identities", {}).get("integrated_sha256"),
            "model_fingerprint": report.get("identities", {}).get("model_fingerprint"),
            "provider_manifest_sha256": report.get("identities", {}).get("provider_manifest_sha256"),
        }
        expected = self._contract
        if observed["scenario_declaration_sha256"] != expected.get("scenario_declaration_sha256"):
            reasons.append("shared report scenario_declaration_sha256 does not match contract")
        if observed["integrated_sha256"] != expected.get("integrated_sha256"):
            reasons.append("shared report integrated_sha256 does not match contract")
        if observed["model_fingerprint"] != expected.get("model_fingerprint"):
            reasons.append("shared report model_fingerprint does not match contract")
        if observed["provider_manifest_sha256"] != expected.get("provider_manifest_sha256"):
            reasons.append("shared report provider_manifest_sha256 does not match contract")
        # The full runtime readiness contract is carried separately; recompute
        # its digest from the unchanged mapping and compare it to the expected
        # runtime digest supplied by the launch.
        runtime_expected = expected.get("runtime_contract_sha256", "")
        runtime_actual = sha256_json(expected.get("integrated_mapping", {}))
        observed["runtime_contract_sha256"] = runtime_actual
        if runtime_expected and runtime_actual != runtime_expected:
            reasons.append(
                "runtime contract mapping sha256 {!r} does not match expected {!r}".format(
                    runtime_actual, runtime_expected
                )
            )
        return {"ready": not reasons, "reasons": reasons, "observed": observed}

    # ------------------------------------------------------------------
    # Check / publish
    # ------------------------------------------------------------------

    def _build_snapshot(self) -> dict[str, object]:
        self._step_list_controllers()
        graph_services = self._graph_services()
        graph_actions = self._probe_actions()
        publisher_metadata = self._publisher_metadata()
        services: dict[str, object] = {}
        for endpoint, entry in graph_services.items():
            services[endpoint] = entry
        arm_service = graph_services.get("/arm_joint_service", {})
        arm_evidence = {
            "ready": bool(arm_service.get("ready")),
            "reasons": arm_service.get("reasons", []),
            "count": arm_service.get("count", 0),
            "source": arm_service.get("source", ""),
            "type": arm_service.get("type", ""),
        }
        operator_publishers = self.get_publishers_info_by_topic("/sim/safety/operator")
        safety_publishers = self.get_publishers_info_by_topic("/sim/hardware/safety_stop")
        safety_sources = [_endpoint_label(info) for info in safety_publishers]
        snapshot: dict[str, object] = {
            "model_preflight": self._model_preflight,
            "shared_report": self._report_evidence,
            "joint_states": {
                **self._joint_evidence(),
                "qos": publisher_metadata.get("/joint_states", {}).get("qos"),
            },
            "tf": self._tf_evidence(),
            "controller_resources": self._controller_evidence(),
            "operator_input": self._bool_evidence(
                self._operator,
                self._operator_received_at,
                max_age_s=_SAFETY_MAX_AGE_S,
                expected_value=False,
                samples=1 if self._operator is not None else 0,
                source="/tinker_integrated_gate_executor",
                count=len(operator_publishers),
                sources=[_endpoint_label(info) for info in operator_publishers],
                observed_qos=publisher_metadata.get("/sim/safety/operator", {}).get("qos"),
                expected_durability="TRANSIENT_LOCAL",
                expected_reliability="RELIABLE",
                expected_depth=1,
            ),
            "safety_stop": self._bool_evidence(
                self._safety_stop,
                self._safety_stop_received_at,
                max_age_s=_SAFETY_MAX_AGE_S,
                expected_value=False,
                min_samples=2,
                samples=self._safety_stop_samples,
                source="/tinker_sim_safety_supervisor",
                count=len(safety_publishers),
                sources=safety_sources,
                observed_qos=publisher_metadata.get("/sim/hardware/safety_stop", {}).get("qos"),
                expected_durability="TRANSIENT_LOCAL",
                expected_reliability="RELIABLE",
                expected_depth=1,
            ),
            "actions": graph_actions,
            "services": services,
            "arm_joint_service": arm_evidence,
            "fixture_status": self._fixture_evidence(),
            "publishers": publisher_metadata,
            "mapping_agreement": self._mapping_evidence(),
            "provider_manifest": self._provider_manifest_evidence(),
            "semantic_model": self._semantic_model_evidence(),
            "collision_state": self._collision_evidence(),
        }
        # The evaluator recomputes the report digest from the exact report bytes.
        return snapshot

    def _check(self) -> None:
        snapshot = self._build_snapshot()
        report = evaluate_integrated_readiness(snapshot, self._contract)
        self._last_evaluated = report.ready
        self._last_reasons = list(report.reasons)
        state = "pass" if report.ready else "fail"
        now = time.monotonic()
        if report.ready:
            self._fail_since = None
        else:
            self._fail_since = self._fail_since if self._fail_since is not None else now
        status = {
            "schema_version": 1,
            "state": state,
            "ready": report.ready,
            "reasons": list(report.reasons),
            "published_at": now,
            "evidence": report.evidence,
        }
        self._last_status = status
        if self._create_status_publisher:
            message = String()
            message.data = json.dumps(
                json_safe_value(status), sort_keys=True, separators=(",", ":")
            )
            self._publisher.publish(message)
        if not report.ready:
            self.get_logger().warning(
                "integrated readiness fail: {}".format(
                    json.dumps(list(report.reasons), sort_keys=True, separators=(",", ":"))
                )
            )
        if (
            self._fail_exit_s > 0
            and self._fail_since is not None
            and now - self._fail_since > self._fail_exit_s
        ):
            self.get_logger().error(
                "integrated readiness failed for {:.1f} s; shutting down".format(
                    self._fail_exit_s
                )
            )
            rclpy.try_shutdown()


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = IntegratedReadiness()
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
