"""Task 3: pure PlanningScene record/transition journal (ROS-free).

This module implements the stable, durable PlanningScene journal consumed by the
later integrated executor/verifier/evidence tasks.  It is deliberately ROS-free:
it imports neither ``rclpy`` nor any ROS message package and starts no nodes.
Task 4 performs actual ROS graph/type/QoS/node discovery and passes a projection
into :func:`validate_graph_evidence`; Task 7 performs the independent
``physics_truth.jsonl``/``evaluator.jsonl`` correlation.  This journal only
records the exact ``(frame_index, timestamp)`` join key and never claims that a
scene attachment proves physical contact.

Key guarantees
--------------
- **Transactional validation.**  Every rejected ``record_diff``, ``snapshot``,
  transition, event, or ``finalize`` leaves ``_records``, ``_last_scene``, the
  JSONL bytes, and the final JSON bytes unchanged.  ``_last_scene`` is updated
  only after a record has been appended successfully.
- **Durable canonical artifacts.**  Each successful append writes exactly one
  compact sorted-key canonical JSON record plus ``\\n``, flushes, and fsyncs
  *before* the in-memory record becomes visible.  ``finalize(..., json_path=...)``
  writes canonical finite JSON through temp-file + file fsync + atomic
  ``os.replace`` + directory fsync with no temp residue; a failed finalize never
  replaces an existing final artifact.
- **No physics fields.**  The journal stores scene events and scene snapshots
  only.  Contact, force, object-pose, evaluator-metric, and physical-verdict
  fields are rejected recursively at input time rather than silently dropped.
  Scene state remains diagnostic consistency evidence, never physical authority.
- **Model contract loaded, not caller-invented.**  The expected attach/TCP link,
  the ordered eight-link touch set, and the handoff identity are loaded verbatim
  from the committed ``integration/ompl-overlay-contract.json`` by
  :func:`load_model_touch_contract`.

The recorder observes exactly these unremapped interfaces (see
:func:`validate_graph_evidence`): ``/planning_scene`` and
``/monitored_planning_scene`` (``moveit_msgs/msg/PlanningScene``, reliable +
transient-local, depth 1), ``/get_planning_scene`` and ``/apply_planning_scene``
(``moveit_msgs/srv/GetPlanningScene`` / ``moveit_msgs/srv/ApplyPlanningScene``,
reliable + volatile), and ``/sim/status/planning_scene_fixture``
(``std_msgs/msg/String`` carrying the exact canonical compact fixture-status JSON
with scalar ``target_handoff="pick_and_place/object_mesh"``, exactly one
publisher ``/fixture_planning_scene``, reliable + transient-local, depth 1).
Payload content never substitutes for graph ownership.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import re
import tempfile
from collections.abc import Mapping, Sequence
from pathlib import Path

SCHEMA_VERSION = 1

# Canonical robot contract identity.  This is the exact SRDF-resolved
# ``xarm_gripper`` group order; never sort or reorder it.
CANONICAL_TOUCH_LINKS: tuple[str, ...] = (
    "xarm_gripper_base_link",
    "left_outer_knuckle",
    "left_finger",
    "left_inner_knuckle",
    "right_inner_knuckle",
    "right_outer_knuckle",
    "right_finger",
    "link_tcp",
)
CANONICAL_LINK_TCP = "link_tcp"
CANONICAL_TARGET_HANDOFF = "pick_and_place/object_mesh"

# Nonzero lowercase 64-hex SHA-256 digest.  Missing, uppercase, malformed, or
# all-zero values never match and fail closed.
DIGEST = re.compile(r"^(?!0{64}$)[0-9a-f]{64}$")

# Required positive scene-event order (preserve exactly; no reordering).
POSITIVE_ORDER: tuple[str, ...] = (
    "fixture-ready",
    "before-pick",
    "scene-attach",
    "lift-complete",
    "transport",
    "before-release",
    "scene-detach",
    "released-settled",
    "teardown",
)

# Physics-truth fields are forbidden anywhere in a scene mapping.  Keys are
# checked recursively so a deeply nested leak is rejected rather than dropped.
PHYSICS_FORBIDDEN_KEYS: frozenset[str] = frozenset({
    "bilateral_contact",
    "contact",
    "contact_detected",
    "contact_pairs",
    "contact_state",
    "contacts",
    "effort",
    "evaluator",
    "evaluator_metric",
    "evaluator_metrics",
    "force",
    "forces",
    "grasp_verified",
    "metric",
    "metrics",
    "object_pose",
    "object_poses",
    "orientation",
    "physical_attachment",
    "physical_bilateral_contact",
    "physical_grasp_verified",
    "physical_verdict",
    "physics",
    "physics_truth",
    "pose",
    "poses",
    "position",
    "quaternion_xyzw",
    "torque",
    "torques",
    "verdict",
    "xyz",
})

# Recorder identity required by validate_graph_evidence.
RECORDER_NODE = "/tinker_integrated_gate_executor"
RECORDER_NAMESPACE = "/"

# Observed topic/service interface sets with exact projected types.
REQUIRED_TOPICS: dict[str, str] = {
    "/planning_scene": "moveit_msgs/msg/PlanningScene",
    "/monitored_planning_scene": "moveit_msgs/msg/PlanningScene",
}
FIXTURE_TOPIC = "/sim/status/planning_scene_fixture"
FIXTURE_TOPIC_TYPE = "std_msgs/msg/String"
FIXTURE_PUBLISHER_NODE = "/fixture_planning_scene"
REQUIRED_SERVICES: dict[str, str] = {
    "/get_planning_scene": "moveit_msgs/srv/GetPlanningScene",
    "/apply_planning_scene": "moveit_msgs/srv/ApplyPlanningScene",
}

# Required QoS.  Topics are reliable + transient-local + depth 1; services are
# reliable + volatile.
TOPIC_QOS: dict[str, object] = {
    "reliability": "RELIABLE",
    "durability": "TRANSIENT_LOCAL",
    "depth": 1,
}
SERVICE_QOS: dict[str, object] = {
    "reliability": "RELIABLE",
    "durability": "VOLATILE",
}

# Exact canonical fixture-status field set (canonical_fixture_status shape).
FIXTURE_STATUS_KEYS: frozenset[str] = frozenset({
    "schema_version",
    "state",
    "scenario",
    "owner",
    "revision",
    "revision_digest",
    "sequence",
    "published_at",
    "owned_ids",
    "target_source_id",
    "target_handoff",
    "fixture_descriptor_sha256",
})

DEFAULT_CONTRACT_PATH = Path(__file__).resolve().parents[1] / "integration" / "ompl-overlay-contract.json"

__all__ = [
    "CANONICAL_LINK_TCP",
    "CANONICAL_TARGET_HANDOFF",
    "CANONICAL_TOUCH_LINKS",
    "DIGEST",
    "FIXTURE_PUBLISHER_NODE",
    "FIXTURE_TOPIC",
    "FIXTURE_TOPIC_TYPE",
    "PHYSICS_FORBIDDEN_KEYS",
    "POSITIVE_ORDER",
    "RECORDER_NODE",
    "RECORDER_NAMESPACE",
    "REQUIRED_SERVICES",
    "REQUIRED_TOPICS",
    "SCHEMA_VERSION",
    "SERVICE_QOS",
    "TOPIC_QOS",
    "PlanningSceneJournal",
    "load_model_touch_contract",
    "validate_graph_evidence",
]


def _canonical_json_bytes(value: object) -> bytes:
    """Compact canonical JSON bytes (sorted keys, minimal separators)."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")


def _validate_digest(value: object, field: str) -> str:
    if not isinstance(value, str) or not DIGEST.fullmatch(value):
        raise ValueError(f"{field} must be a nonzero lowercase 64-hex digest")
    return value


def _non_negative_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return int(value)


def _finite_non_negative(value: object, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite non-negative number")
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be a finite non-negative number")
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number")
    return result


def _unique_strings(values: object, name: str) -> list[str]:
    if not isinstance(values, (list, tuple)):
        raise ValueError(f"{name} must be a sequence of nonempty strings")
    result: list[str] = []
    seen: set[str] = set()
    for value in values:
        if not isinstance(value, str) or not value:
            raise ValueError(f"{name} entries must be nonempty strings")
        if value in seen:
            raise ValueError(f"{name} contains a duplicate id: {value}")
        seen.add(value)
        result.append(value)
    return result


def _reject_physics_keys(value: object, path: str = "$") -> None:
    """Reject any forbidden physics-truth key at any nesting depth."""
    if isinstance(value, Mapping):
        for key, item in value.items():
            if key in PHYSICS_FORBIDDEN_KEYS:
                raise ValueError(f"forbidden physics field at {path}.{key}")
            _reject_physics_keys(item, f"{path}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _reject_physics_keys(item, f"{path}[{index}]")


def load_model_touch_contract(contract_path: str | Path | None = None) -> dict[str, object]:
    """Load the pinned model touch contract from the committed overlay contract.

    By default reads ``integration/ompl-overlay-contract.json`` beside the
    repository root.  Returns exactly:

    - ``link_tcp``: the attach/TCP link (``model_bundle.semantic_contract.tcp_link``);
    - ``touch_links``: the ordered eight touch links
      (``model_bundle.semantic_contract.touch_links``);
    - ``target_handoff``: the handoff identity
      (``fixture_contract.target_handoff``).

    Fails closed on a missing/malformed contract, a non-eight / duplicate /
    permuted touch-link set relative to the canonical robot contract, a missing
    ``link_tcp``, or a wrong handoff.
    """
    path = Path(contract_path) if contract_path is not None else DEFAULT_CONTRACT_PATH
    try:
        raw_text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ValueError(f"model touch contract is unreadable: {path}") from exc
    try:
        raw = json.loads(raw_text)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"model touch contract is malformed JSON: {path}") from exc
    if not isinstance(raw, dict):
        raise ValueError("model touch contract must be a JSON object")
    try:
        semantic = raw["model_bundle"]["semantic_contract"]
        link_tcp = str(semantic["tcp_link"])
        touch_links = tuple(str(value) for value in semantic["touch_links"])
        target_handoff = str(raw["fixture_contract"]["target_handoff"])
    except (KeyError, TypeError) as exc:
        raise ValueError("model touch contract is missing required contract fields") from exc
    if len(touch_links) != 8:
        raise ValueError("model touch contract touch_links must be the complete eight-link SRDF set")
    if len(set(touch_links)) != 8:
        raise ValueError("model touch contract touch_links must be unique")
    if touch_links != CANONICAL_TOUCH_LINKS:
        raise ValueError("model touch contract touch_links must match the canonical eight-link order")
    if link_tcp != CANONICAL_LINK_TCP:
        raise ValueError("model touch contract tcp_link must be link_tcp")
    if target_handoff != CANONICAL_TARGET_HANDOFF:
        raise ValueError("model touch contract target_handoff must be pick_and_place/object_mesh")
    return {
        "link_tcp": link_tcp,
        "touch_links": touch_links,
        "target_handoff": target_handoff,
    }


def _atomic_write_json(value: object, path: Path) -> None:
    """Write *value* canonically through temp-file + fsync + os.replace + dir fsync."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, sort_keys=True, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        try:
            dir_fd = os.open(path.parent, os.O_RDONLY)
        except OSError:
            pass
        else:
            try:
                os.fsync(dir_fd)
            finally:
                os.close(dir_fd)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _qos_matches(qos: object, expected: Mapping[str, object]) -> bool:
    if not isinstance(qos, Mapping):
        return False
    return all(qos.get(key) == value for key, value in expected.items())


def _validate_endpoints(label: str, endpoints: object) -> list[dict[str, str]]:
    """Validate real endpoint/provider metadata (never payload-only claims)."""
    if not isinstance(endpoints, (list, tuple)) or not endpoints:
        raise ValueError(f"{label} must have real endpoint metadata")
    normalized: list[dict[str, str]] = []
    for endpoint in endpoints:
        if isinstance(endpoint, str):
            if not endpoint:
                raise ValueError(f"{label} has an empty endpoint")
            normalized.append({"node": endpoint})
        elif isinstance(endpoint, Mapping):
            node = endpoint.get("node")
            if not isinstance(node, str) or not node:
                raise ValueError(f"{label} has an endpoint without a real node")
            node_namespace = endpoint.get("node_namespace")
            normalized.append(
                {
                    "node": node,
                    "node_namespace": str(node_namespace) if node_namespace is not None else "",
                }
            )
        else:
            raise ValueError(f"{label} has malformed endpoint metadata")
    return normalized


def _validate_fixture_payload(payload: object) -> dict[str, object]:
    """Independently validate the exact canonical compact fixture-status payload.

    The payload is the raw ``std_msgs/msg/String`` data: it must be the canonical
    compact sorted-key JSON encoding of exactly the canonical fixture-status
    field set, with scalar ``target_handoff="pick_and_place/object_mesh"``.
    """
    if not isinstance(payload, str) or not payload:
        raise ValueError("fixture payload must be a nonempty canonical compact JSON string")
    try:
        parsed = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise ValueError("fixture payload must be parseable JSON") from exc
    if not isinstance(parsed, dict):
        raise ValueError("fixture payload must be a JSON object")
    canonical = _canonical_json_bytes(parsed).decode("utf-8")
    if canonical != payload:
        raise ValueError("fixture payload must be the canonical compact fixture-status encoding")
    if set(parsed) != FIXTURE_STATUS_KEYS:
        raise ValueError("fixture payload must be the exact canonical fixture-status field set")
    if parsed.get("schema_version") != 1:
        raise ValueError("fixture payload schema_version must be 1")
    if parsed.get("owner") != "sim_fixture":
        raise ValueError("fixture payload owner must be sim_fixture")
    if parsed.get("target_handoff") != CANONICAL_TARGET_HANDOFF:
        raise ValueError(f"fixture payload target_handoff must be {CANONICAL_TARGET_HANDOFF}")
    for field in ("state", "scenario", "revision", "target_source_id"):
        value = parsed.get(field)
        if not isinstance(value, str) or not value:
            raise ValueError(f"fixture payload {field} must be a nonempty string")
    for field in ("revision_digest", "fixture_descriptor_sha256"):
        _validate_digest(parsed.get(field), f"fixture payload {field}")
    sequence = parsed.get("sequence")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError("fixture payload sequence must be a non-negative integer")
    published_at = parsed.get("published_at")
    if isinstance(published_at, bool):
        raise ValueError("fixture payload published_at must be finite non-negative")
    try:
        published = float(published_at)
    except (TypeError, ValueError):
        raise ValueError("fixture payload published_at must be finite non-negative")
    if not math.isfinite(published) or published < 0.0:
        raise ValueError("fixture payload published_at must be finite non-negative")
    owned_ids = parsed.get("owned_ids")
    if not isinstance(owned_ids, list) or not all(
        isinstance(value, str) and value for value in owned_ids
    ):
        raise ValueError("fixture payload owned_ids must be a list of nonempty strings")
    return dict(parsed)


def _validate_topic_entry(
    name: str, entry: object, expected_type: str, *, fixture: bool = False
) -> dict[str, object]:
    if not isinstance(entry, Mapping):
        raise ValueError(f"topic {name} evidence must be a mapping")
    if entry.get("type") != expected_type:
        raise ValueError(f"topic {name} has wrong type {entry.get('type')!r}; expected {expected_type}")
    for side in ("requested_qos", "offered_qos"):
        if not _qos_matches(entry.get(side), TOPIC_QOS):
            raise ValueError(
                f"topic {name} {side} QoS must be reliable + transient-local + depth 1"
            )
    publishers = _validate_endpoints(f"topic {name} publishers", entry.get("publishers"))
    subscribers = _validate_endpoints(f"topic {name} subscribers", entry.get("subscribers"))
    normalized: dict[str, object] = {
        "type": expected_type,
        "requested_qos": TOPIC_QOS,
        "offered_qos": TOPIC_QOS,
        "publishers": publishers,
        "subscribers": subscribers,
    }
    if fixture:
        if len(publishers) != 1:
            raise ValueError(f"topic {name} must have exactly one publisher")
        if publishers[0]["node"] != FIXTURE_PUBLISHER_NODE:
            raise ValueError(f"topic {name} publisher must be {FIXTURE_PUBLISHER_NODE}")
        normalized["payload"] = _validate_fixture_payload(entry.get("payload"))
    return normalized


def _validate_service_entry(name: str, entry: object, expected_type: str) -> dict[str, object]:
    if not isinstance(entry, Mapping):
        raise ValueError(f"service {name} evidence must be a mapping")
    if entry.get("type") != expected_type:
        raise ValueError(f"service {name} has wrong type {entry.get('type')!r}; expected {expected_type}")
    for side in ("requested_qos", "offered_qos"):
        if not _qos_matches(entry.get(side), SERVICE_QOS):
            raise ValueError(f"service {name} {side} QoS must be reliable + volatile")
    servers = _validate_endpoints(f"service {name} servers", entry.get("servers"))
    clients = _validate_endpoints(f"service {name} clients", entry.get("clients"))
    return {
        "type": expected_type,
        "requested_qos": SERVICE_QOS,
        "offered_qos": SERVICE_QOS,
        "servers": servers,
        "clients": clients,
    }


def validate_graph_evidence(graph: object) -> dict[str, object]:
    """Validate a Task-4-supplied graph projection and return normalized evidence.

    Requires the exact projected interface sets (``/planning_scene``,
    ``/monitored_planning_scene``, ``/sim/status/planning_scene_fixture`` topics
    and ``/get_planning_scene``, ``/apply_planning_scene`` services), the
    recorder identity (``node_name="/tinker_integrated_gate_executor"``,
    ``namespace="/"``, ``remap_table={}``), real endpoint/provider metadata,
    topic QoS reliable + transient-local + depth 1, service QoS reliable +
    volatile, and the exact canonical fixture-status payload with scalar
    ``target_handoff="pick_and_place/object_mesh"`` from exactly one publisher
    ``/fixture_planning_scene``.  The returned normalized graph is the evidence
    retained in the final journal JSON.
    """
    if not isinstance(graph, Mapping):
        raise TypeError("graph evidence must be a mapping")
    node_name = graph.get("node_name")
    if node_name != RECORDER_NODE:
        raise ValueError("graph evidence node_name must be /tinker_integrated_gate_executor")
    namespace = graph.get("namespace")
    if namespace != RECORDER_NAMESPACE:
        raise ValueError("graph evidence namespace must be /")
    remap_table = graph.get("remap_table")
    if not isinstance(remap_table, Mapping) or len(remap_table) != 0:
        raise ValueError("graph evidence remap_table must be empty")
    topics = graph.get("topics")
    if not isinstance(topics, Mapping):
        raise ValueError("graph evidence must include a topics mapping")
    services = graph.get("services")
    if not isinstance(services, Mapping):
        raise ValueError("graph evidence must include a services mapping")

    normalized_topics: dict[str, dict[str, object]] = {}
    for name, expected_type in REQUIRED_TOPICS.items():
        if name not in topics:
            raise ValueError(f"graph evidence missing required topic {name}")
        normalized_topics[name] = _validate_topic_entry(name, topics[name], expected_type)
    if FIXTURE_TOPIC not in topics:
        raise ValueError(f"graph evidence missing required topic {FIXTURE_TOPIC}")
    normalized_topics[FIXTURE_TOPIC] = _validate_topic_entry(
        FIXTURE_TOPIC, topics[FIXTURE_TOPIC], FIXTURE_TOPIC_TYPE, fixture=True
    )

    normalized_services: dict[str, dict[str, object]] = {}
    for name, expected_type in REQUIRED_SERVICES.items():
        if name not in services:
            raise ValueError(f"graph evidence missing required service {name}")
        normalized_services[name] = _validate_service_entry(name, services[name], expected_type)

    return {
        "node_name": RECORDER_NODE,
        "namespace": RECORDER_NAMESPACE,
        "remap_table": {},
        "topics": normalized_topics,
        "services": normalized_services,
    }


class PlanningSceneJournal:
    """Append-only durable journal of PlanningScene records and transitions.

    The journal records scene events and scene snapshots only.  It never stores
    contact, force, object-pose, evaluator metric, or any other physics-truth
    field; scene state remains diagnostic consistency evidence, never physical
    authority.

    Each record carries three distinct identities:

    - journal: ``journal_sequence``;
    - raw/evaluator join: exact ``frame_index``, ``timestamp``;
    - diagnostic scene identity: ``scene_sequence``, ``scene_timestamp``,
      ``scene_revision_digest``.
    """

    def __init__(
        self,
        *,
        fixture_revision: str,
        task_namespace: str,
        target_object_id: str,
        expected_attach_link: str,
        expected_touch_links: Sequence[str],
        required_event_order: Sequence[str] = (),
        forbidden_events: Sequence[str] = (),
        jsonl_path: str | Path | None = None,
    ) -> None:
        if not isinstance(fixture_revision, str) or not fixture_revision:
            raise ValueError("fixture_revision must be a nonempty string")
        if not isinstance(task_namespace, str) or not task_namespace:
            raise ValueError("task_namespace must be a nonempty string")
        if not isinstance(target_object_id, str) or not target_object_id:
            raise ValueError("target_object_id must be a nonempty string")
        if not isinstance(expected_attach_link, str) or not expected_attach_link:
            raise ValueError("expected_attach_link must be a nonempty string")
        self.fixture_revision = fixture_revision
        self.task_namespace = task_namespace
        self.target_object_id = target_object_id
        self.expected_attach_link = expected_attach_link
        self.expected_touch_links = tuple(expected_touch_links)
        if tuple(self.expected_touch_links) != CANONICAL_TOUCH_LINKS:
            raise ValueError(
                "expected touch links must be the exact canonical eight-link SRDF set in order"
            )
        self.required_event_order = tuple(required_event_order)
        self.forbidden_events = frozenset(forbidden_events)
        self.jsonl_path = Path(jsonl_path) if jsonl_path is not None else None
        self._records: list[dict[str, object]] = []
        self._last_scene: dict[str, object] | None = None

    def _normalize(self, scene: object) -> dict[str, object]:
        """Validate and coerce a scene snapshot before any mutation."""
        if not isinstance(scene, Mapping):
            raise TypeError("PlanningScene snapshots must be mappings")
        _reject_physics_keys(scene, "$")
        required = (
            "scene_sequence",
            "scene_timestamp",
            "frame_index",
            "timestamp",
            "owned_ids",
            "attached_ids",
            "attached_links",
            "touch_links",
            "fixture_revision",
            "scene_revision_digest",
            "acm_digest",
            "robot_state_digest",
            "source",
        )
        missing = [field for field in required if field not in scene]
        if missing:
            raise ValueError(f"PlanningScene snapshot missing required fields: {missing}")

        normalized = dict(scene)
        normalized["scene_sequence"] = _non_negative_int(scene["scene_sequence"], "scene_sequence")
        normalized["scene_timestamp"] = _finite_non_negative(scene["scene_timestamp"], "scene_timestamp")
        normalized["frame_index"] = _non_negative_int(scene["frame_index"], "frame_index")
        normalized["timestamp"] = _finite_non_negative(scene["timestamp"], "timestamp")
        normalized["owned_ids"] = _unique_strings(scene["owned_ids"], "owned_ids")
        normalized["attached_ids"] = _unique_strings(scene["attached_ids"], "attached_ids")

        attached_links = scene["attached_links"]
        if not isinstance(attached_links, Mapping):
            raise ValueError("attached_links must be a mapping")
        normalized["attached_links"] = {
            str(key): str(value) for key, value in attached_links.items()
        }
        touch_links = scene["touch_links"]
        if not isinstance(touch_links, Mapping):
            raise ValueError("touch_links must be a mapping")
        normalized["touch_links"] = {
            str(key): [str(value) for value in values]
            for key, values in touch_links.items()
        }

        if set(normalized["owned_ids"]) & set(normalized["attached_ids"]):
            raise ValueError("an object cannot be both world and attached")

        fixture_revision = scene["fixture_revision"]
        if not isinstance(fixture_revision, str) or not fixture_revision:
            raise ValueError("fixture_revision must be a nonempty string")
        if fixture_revision != self.fixture_revision:
            raise ValueError("fixture revision mismatch")

        source = scene["source"]
        if not isinstance(source, str) or not source:
            raise ValueError("source must be a nonempty string")

        for field in ("scene_revision_digest", "acm_digest", "robot_state_digest"):
            _validate_digest(scene.get(field), field)

        for object_id in normalized["attached_ids"]:
            if object_id not in normalized["attached_links"]:
                raise ValueError(f"attached object {object_id} has no attach link")
            if object_id.startswith(self.task_namespace):
                if normalized["attached_links"][object_id] != self.expected_attach_link:
                    raise ValueError(f"attached object {object_id} has wrong attach link")
                if tuple(normalized["touch_links"].get(object_id, ())) != self.expected_touch_links:
                    raise ValueError(f"attached object {object_id} has wrong touch links")
        return normalized

    def _append(
        self,
        event: str,
        scene: Mapping[str, object],
        *,
        frame_index: int,
        timestamp: float,
    ) -> dict[str, object]:
        """Validate, durably append one canonical JSONL record, then store it."""
        if not isinstance(event, str) or not event or event in self.forbidden_events:
            raise ValueError(f"forbidden or empty PlanningScene event: {event}")
        frame_index = _non_negative_int(frame_index, "frame_index")
        timestamp = _finite_non_negative(timestamp, "timestamp")
        if self._records:
            previous = self._records[-1]
            if frame_index <= int(previous["frame_index"]) or timestamp <= float(previous["timestamp"]):
                raise ValueError("journal frame_index and timestamp must be monotonic")
        for field in ("scene_revision_digest", "acm_digest", "robot_state_digest"):
            _validate_digest(scene.get(field), field)
        record = {
            "event": event,
            "journal_sequence": len(self._records) + 1,
            "frame_index": frame_index,
            "timestamp": timestamp,
            "scene_sequence": int(scene["scene_sequence"]),
            "scene_timestamp": float(scene["scene_timestamp"]),
            "scene_revision_digest": str(scene["scene_revision_digest"]),
            "owned_ids": list(scene["owned_ids"]),
            "attached_ids": list(scene["attached_ids"]),
            "attached_links": dict(scene["attached_links"]),
            "touch_links": {key: list(value) for key, value in scene["touch_links"].items()},
            "fixture_revision": str(scene["fixture_revision"]),
            "acm_digest": str(scene["acm_digest"]),
            "robot_state_digest": str(scene["robot_state_digest"]),
            "source": str(scene["source"]),
        }
        if self.jsonl_path is not None:
            with self.jsonl_path.open("a", encoding="utf-8") as stream:
                stream.write(_canonical_json_bytes(record).decode("utf-8") + "\n")
                stream.flush()
                os.fsync(stream.fileno())
        self._records.append(record)
        return dict(record)

    def snapshot(self, event: str, *, frame_index: int, timestamp: float) -> dict[str, object]:
        """Append a new journal/join identity retaining the prior scene identity."""
        if self._last_scene is None:
            raise RuntimeError("cannot snapshot before the first scene diff")
        return self._append(event, self._last_scene, frame_index=frame_index, timestamp=timestamp)

    def record_diff(self, event: str, scene: object) -> dict[str, object]:
        """Validate and append a scene diff, then update ``_last_scene``."""
        normalized = self._normalize(scene)
        if self._last_scene is not None:
            if (
                normalized["scene_sequence"] <= int(self._last_scene["scene_sequence"])
                or normalized["scene_timestamp"] <= float(self._last_scene["scene_timestamp"])
            ):
                raise ValueError("PlanningScene sequence and timestamp must be monotonic")
            if event == "task-cleanup":
                before_ids = set(self._last_scene["owned_ids"]) | set(self._last_scene["attached_ids"])
                after_ids = set(normalized["owned_ids"]) | set(normalized["attached_ids"])
                removed = before_ids - after_ids
                foreign = sorted(
                    object_id for object_id in removed
                    if not object_id.startswith(self.task_namespace)
                )
                if foreign:
                    raise PermissionError(f"task cleanup removed foreign objects: {foreign}")
        record = self._append(
            event,
            normalized,
            frame_index=normalized["frame_index"],
            timestamp=normalized["timestamp"],
        )
        self._last_scene = normalized
        return record

    def assert_transition(self, before: object, after: object, expected: str) -> None:
        """Validate a world -> attached -> world transition strictly.

        Requires strictly increasing diagnostic scene identity and exact
        attach/detach/task-cleanup semantics.  Scene attach remains diagnostic
        only; Task 7 later requires bilateral physical contact strictly before
        the scene-attach record's ``(frame_index, timestamp)`` join key.
        """
        before_scene = self._normalize(before)
        after_scene = self._normalize(after)
        if int(after_scene["scene_sequence"]) <= int(before_scene["scene_sequence"]):
            raise ValueError("transition sequence must increase")
        if float(after_scene["scene_timestamp"]) <= float(before_scene["scene_timestamp"]):
            raise ValueError("transition timestamp must increase")
        before_world = set(before_scene["owned_ids"])
        before_attached = set(before_scene["attached_ids"])
        after_world = set(after_scene["owned_ids"])
        after_attached = set(after_scene["attached_ids"])
        if expected == "scene-attach":
            if self.target_object_id not in after_attached or self.target_object_id in after_world:
                raise ValueError("scene-attach must attach the exact target object")
            if self.target_object_id in before_attached:
                raise ValueError("scene-attach target was already attached")
        elif expected == "scene-detach":
            if (
                self.target_object_id not in before_attached
                or self.target_object_id in after_attached
                or self.target_object_id not in after_world
            ):
                raise ValueError("scene-detach must return the exact target object to the world")
        elif expected == "task-cleanup":
            removed = (before_world | before_attached) - (after_world | after_attached)
            foreign = sorted(
                object_id for object_id in removed
                if not object_id.startswith(self.task_namespace)
            )
            if foreign:
                raise PermissionError(f"task cleanup removed foreign objects: {foreign}")
        else:
            raise ValueError(f"unknown PlanningScene transition: {expected}")

    def finalize(
        self,
        status: str,
        *,
        graph: object = None,
        json_path: str | Path | None = None,
    ) -> dict[str, object]:
        """Validate event order / graph evidence and return the finite final object.

        Returns ``{schema_version, status, authority, events, records, graph}``.
        If *json_path* is given, the canonical finite JSON is written atomically
        through temp-file + file fsync + ``os.replace`` + directory fsync with no
        temp residue.  A failed finalize never replaces an existing final
        artifact.
        """
        if not isinstance(status, str) or not status:
            raise ValueError("finalize status must be a nonempty string")
        events = [str(record["event"]) for record in self._records]
        cursor = -1
        for required in self.required_event_order:
            matches = [index for index, event in enumerate(events) if event == required]
            if len(matches) != 1 or matches[0] <= cursor:
                raise ValueError(f"required event order violated at {required}: {events}")
            cursor = matches[0]
        if self.required_event_order and self.required_event_order[-1] == "teardown":
            if not events or events[-1] != "teardown":
                raise ValueError("teardown must be the final PlanningScene event")
        validated_graph: dict[str, object] = {}
        if graph is not None:
            validated_graph = validate_graph_evidence(graph)
        final = {
            "schema_version": SCHEMA_VERSION,
            "status": status,
            "authority": "physics_truth",
            "events": events,
            "records": list(self._records),
            "graph": validated_graph,
        }
        if json_path is not None:
            _atomic_write_json(final, json_path)
        return final
