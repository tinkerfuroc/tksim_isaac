"""ROS Humble live fixture PlanningScene adapter (Task 5).

The node applies exactly one atomic PlanningScene diff: all desired
``sim_fixture/*`` objects as ADDs plus every stale existing ``sim_fixture/*`` id
as a REMOVE, leaving other namespaces untouched.  It first gates on the staged
``/sim/ready/physics`` service, waits boundedly for the typed
``/apply_planning_scene`` and ``/get_planning_scene`` MoveIt services,
pre-reads the current scene to discover stale fixture ids, sends the one atomic
apply request, reads the scene back, confirms the readback and its own canonical
status, and only then serves ``/sim/ready/fixture`` while publishing a reliable
transient-local 5 Hz compact JSON heartbeat on
``/sim/status/planning_scene_fixture``.

The full SRDF-derived eight-link touch set reaches the downstream hardening
reconciler through the validated model contract; this adapter never owns task
objects and only inserts the declared ``sim_fixture/*`` geometry.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Mapping, Sequence

import rclpy
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .contract_guard import step_service
from .fixture_contract import (
    FIXTURE_STATE_FAILED,
    FIXTURE_STATE_PENDING,
    FIXTURE_STATE_READY,
    FIXTURE_NAMESPACE_PREFIX,
    FixtureContractError,
    build_atomic_revision_diff,
    confirm_fixture_revision,
    parse_required_fixture_owned_ids,
    readback_geometry,
    revision_digest,
    spec_geometry,
)
from .fixture_planning_scene import (
    canonical_fixture_status,
    fixture_descriptor_sha256,
    fixture_owned_ids,
    fixture_to_specs,
    find_project_root,
    load_mesh_asset,
    serialize_status,
)

_STATUS_TOPIC = "/sim/status/planning_scene_fixture"
_READY_SERVICE = "/sim/ready/fixture"
_PHYSICS_READY_SERVICE = "/sim/ready/physics"
_APPLY_SERVICE = "/apply_planning_scene"
_GET_SERVICE = "/get_planning_scene"

_PHASE_PHYSICS = "physics"
_PHASE_DISCOVER = "discover"
_PHASE_APPLY = "apply"
_PHASE_READBACK = "readback"
_PHASE_CONFIRM = "confirm"
_PHASE_READY = "ready"
_PHASE_FAILED = "failed"

_SOLID_PRIMITIVE_TYPE = {"box": 1, "sphere": 2, "cylinder": 3}
_SERVICE_TTL_S = 30.0
_SERVICE_TIMEOUT_S = 5.0


def _pose_from_seven(pose7: Sequence[float]) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = pose7[0], pose7[1], pose7[2]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
        pose7[3],
        pose7[4],
        pose7[5],
        pose7[6],
    )
    return pose


def _spec_to_collision_object(spec, *, mesh_loader=None) -> CollisionObject:
    obj = CollisionObject()
    obj.id = spec.id
    obj.header.frame_id = spec.frame_id
    obj.operation = bytes([spec.operation])
    for primitive in spec.primitives:
        solid = SolidPrimitive()
        solid.type = _SOLID_PRIMITIVE_TYPE[str(primitive["type"])]
        solid.dimensions = [float(value) for value in primitive["dimensions"]]
        obj.primitives.append(solid)
    for pose7 in spec.primitive_poses:
        obj.primitive_poses.append(_pose_from_seven(pose7))
    for mesh in spec.meshes:
        if mesh_loader is None:
            raise ValueError(
                "mesh fixture {!r} cannot be built without a mesh loader".format(spec.id)
            )
        vertices, triangles = mesh_loader(mesh)
        mesh_msg = Mesh()
        for vertex in vertices:
            point = Point()
            point.x, point.y, point.z = (
                float(vertex[0]),
                float(vertex[1]),
                float(vertex[2]),
            )
            mesh_msg.vertices.append(point)
        for triangle in triangles:
            mesh_triangle = MeshTriangle()
            mesh_triangle.vertex_indices = [int(index) for index in triangle]
            mesh_msg.triangles.append(mesh_triangle)
        obj.meshes.append(mesh_msg)
    for pose7 in spec.mesh_poses:
        obj.mesh_poses.append(_pose_from_seven(pose7))
    return obj


class FixturePlanningScene(Node):
    """Apply the one atomic fixture diff and serve readiness once confirmed."""

    def __init__(
        self,
        *,
        node_name: str | None = None,
        context=None,
        parameter_overrides=None,
    ) -> None:
        super().__init__(
            node_name or "tinker_sim_fixture_planning_scene",
            context=context,
            parameter_overrides=parameter_overrides or [],
        )
        try:
            self._initialize()
        except Exception:
            # A partially-initialized rclpy Node must be destroyed before
            # re-raising, otherwise its C++ handle leaks and later GC of the
            # half-constructed object can crash the process.
            try:
                self.destroy_node()
            except Exception:  # noqa: BLE001 - destroy must never mask the cause
                pass
            raise

    def _initialize(self) -> None:
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("heartbeat_period", 0.2)
        self.declare_parameter("start_deadline_s", 60.0)
        self.declare_parameter("required_fixture_owned_ids", "")
        self._scenario_file = str(self.get_parameter("scenario_file").value)
        if not self._scenario_file:
            raise ValueError("scenario_file parameter is required")
        self._load_scenario(self._scenario_file)

        period = float(self.get_parameter("heartbeat_period").value)
        if period <= 0:
            raise ValueError("heartbeat_period must be positive")
        self._heartbeat_period = period
        self._start_deadline_s = float(self.get_parameter("start_deadline_s").value)
        declared_owned = str(self.get_parameter("required_fixture_owned_ids").value)
        required_owned = parse_required_fixture_owned_ids(declared_owned) if declared_owned else self._owned_ids
        if tuple(required_owned) != self._owned_ids:
            raise ValueError(
                "required_fixture_owned_ids does not match the declared fixture owned ids"
            )

        self._sequence = 0
        self._last_status: Mapping[str, object] | None = None
        self._phase = _PHASE_PHYSICS
        self._state = FIXTURE_STATE_PENDING
        self._fail_reason: str | None = None
        self._phase_started_at = time.monotonic()
        self._specs = fixture_to_specs(self._planning_scene)
        self._existing_ids: tuple[str, ...] = ()
        self._diff_plan = None
        self._scene_ids: tuple[str, ...] = ()
        self._scene_objects: tuple = ()
        self._project_root = None
        self._load_mesh = None
        self._service_group = ReentrantCallbackGroup()
        self._clock = self.get_clock()

        self._physics_state: dict[str, object] = {
            "client": None, "future": None, "error": None, "pending": None,
            "succeeded": False, "result": None,
        }
        self._discover_state: dict[str, object] = {
            "client": None, "future": None, "error": None, "pending": None,
            "succeeded": False, "result": None,
        }
        self._apply_state: dict[str, object] = {
            "client": None, "future": None, "error": None, "pending": None,
            "succeeded": False, "result": None,
        }
        self._readback_state: dict[str, object] = {
            "client": None, "future": None, "error": None, "pending": None,
            "succeeded": False, "result": None,
        }

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(String, _STATUS_TOPIC, status_qos)
        self._ready_service = self.create_service(
            Trigger, _READY_SERVICE, self._on_ready
        )
        self.create_timer(period, self._tick)

    # ------------------------------------------------------------------
    # Scenario loading
    # ------------------------------------------------------------------

    def _load_scenario(self, path: str) -> None:
        from tinker_sim_core.scenario import ScenarioDefinition

        scenario_path = Path(path)
        ScenarioDefinition.load(scenario_path)
        raw = json.loads(scenario_path.read_text(encoding="utf-8"))
        planning_scene = raw.get("planning_scene")
        if not isinstance(planning_scene, dict):
            raise ValueError(f"{path}: scenario has no planning_scene object")
        self._planning_scene = planning_scene
        self._scenario_id = str(raw["id"])
        self._revision = str(planning_scene["revision"])
        self._revision_digest = revision_digest(planning_scene)
        self._owned_ids = fixture_owned_ids(planning_scene)
        self._target_source_id = str(planning_scene["target_source_id"])
        self._target_handoff = str(planning_scene["target_handoff"])
        self._descriptor_sha256 = fixture_descriptor_sha256(planning_scene)
        self._project_root = find_project_root(scenario_path)
        self._load_mesh = (
            lambda mesh: load_mesh_asset(mesh, project_root=self._project_root)
        )

    # ------------------------------------------------------------------
    # Heartbeat / status
    # ------------------------------------------------------------------

    def _current_status(self, *, state: str | None = None) -> Mapping[str, object]:
        return canonical_fixture_status(
            scenario=self._scenario_id,
            revision=self._revision,
            revision_digest=self._revision_digest,
            sequence=self._sequence,
            published_at=float(self._clock.now().nanoseconds) / 1e9,
            owned_ids=self._owned_ids,
            target_source_id=self._target_source_id,
            target_handoff=self._target_handoff,
            descriptor_sha256=self._descriptor_sha256,
            state=self._state if state is None else state,
        )

    def _publish_heartbeat(self) -> None:
        self._sequence += 1
        status = self._current_status()
        self._last_status = status
        message = String()
        message.data = serialize_status(status)
        self._publisher.publish(message)

    def _on_ready(self, request: Trigger.Request, response: Trigger.Response):
        del request
        if self._phase == _PHASE_READY and self._state == FIXTURE_STATE_READY:
            response.success = True
        else:
            response.success = False
        status = self._last_status or self._current_status()
        response.message = serialize_status(status)
        return response

    # ------------------------------------------------------------------
    # Fail-closed transitions
    # ------------------------------------------------------------------

    def _fail(self, reason: str) -> None:
        self._phase = _PHASE_FAILED
        self._state = FIXTURE_STATE_FAILED
        self._fail_reason = reason
        self.get_logger().error("fixture planning scene failed: {}".format(reason))

    def _past_start_deadline(self, now_s: float) -> bool:
        return now_s - self._phase_started_at > self._start_deadline_s

    # ------------------------------------------------------------------
    # State machine tick
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        now_s = time.monotonic()
        self._advance(now_s)
        self._publish_heartbeat()

    def _advance(self, now_s: float) -> None:
        if self._phase == _PHASE_PHYSICS:
            self._advance_physics(now_s)
        elif self._phase == _PHASE_DISCOVER:
            self._advance_discover(now_s)
        elif self._phase == _PHASE_APPLY:
            self._advance_apply(now_s)
        elif self._phase == _PHASE_READBACK:
            self._advance_readback(now_s)
        elif self._phase == _PHASE_CONFIRM:
            self._advance_confirm()

    def _get_extract(self, response):
        if response is None:
            return None
        scene = getattr(response, "scene", None)
        if scene is None:
            return None
        world = getattr(scene, "world", None)
        if world is None:
            return None
        collision_objects = getattr(world, "collision_objects", None)
        if collision_objects is None:
            return None
        try:
            return tuple(readback_geometry(obj) for obj in collision_objects)
        except FixtureContractError as exc:
            raise FixtureContractError(
                "malformed readback collision object: {}".format(exc)
            ) from exc

    def _make_get_service(self, state: dict[str, object]):
        def create_client():
            return self.create_client(
                GetPlanningScene,
                _GET_SERVICE,
                callback_group=self._service_group,
            )

        def request(client):
            # Request the world object names AND geometry explicitly; do not rely
            # on the default components=0 (server-dependent) readback behavior.
            request = client.srv_type.Request()
            request.components = PlanningSceneComponents()
            request.components.components = (
                PlanningSceneComponents.WORLD_OBJECT_NAMES
                | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            )
            return request

        def reset_client(inner):
            client = inner.get("client")
            if client is not None:
                try:
                    client.destroy()
                except Exception:  # noqa: BLE001 - destroy must never raise
                    pass
            inner["client"] = None
            inner["future"] = None

        def step(now_s: float) -> None:
            step_service(
                state,
                create_client=create_client,
                request=request,
                extract=self._get_extract,
                reset_client=reset_client,
                now_s=now_s,
                ttl_s=_SERVICE_TTL_S,
                timeout_s=_SERVICE_TIMEOUT_S,
            )

        return step

    # ------------------------------------------------------------------
    # Physics-ready gate
    # ------------------------------------------------------------------

    def _advance_physics(self, now_s: float) -> None:
        if self._past_start_deadline(now_s):
            self._fail("timed out waiting for physics-ready gate")
            return

        def create_client():
            return self.create_client(
                Trigger,
                _PHYSICS_READY_SERVICE,
                callback_group=self._service_group,
            )

        def request(client):
            return client.srv_type.Request()

        def extract(response):
            if response is None:
                return None
            return True if getattr(response, "success", False) else None

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
            self._physics_state,
            create_client=create_client,
            request=request,
            extract=extract,
            reset_client=reset_client,
            now_s=now_s,
            ttl_s=_SERVICE_TTL_S,
            timeout_s=_SERVICE_TIMEOUT_S,
        )
        if self._physics_state.get("succeeded"):
            self._phase = _PHASE_DISCOVER
            self._phase_started_at = now_s
            self.get_logger().info("physics-ready gate passed; discovering existing fixtures")

    # ------------------------------------------------------------------
    # Discover existing sim_fixture ids (pre-apply readback)
    # ------------------------------------------------------------------

    def _advance_discover(self, now_s: float) -> None:
        if self._discover_state.get("error"):
            self._fail(str(self._discover_state["error"]))
            return
        if self._past_start_deadline(now_s):
            self._fail("timed out discovering the existing planning scene")
            return
        step = self._make_get_service(self._discover_state)
        step(now_s)
        if self._discover_state.get("error"):
            self._fail(str(self._discover_state["error"]))
            return
        if self._discover_state.get("succeeded"):
            existing = tuple(
                str(obj["id"])
                for obj in self._discover_state["result"]
                if str(obj["id"]).startswith(FIXTURE_NAMESPACE_PREFIX)
            )
            self._existing_ids = existing
            self._diff_plan = build_atomic_revision_diff(
                desired_objects=self._specs,
                existing_ids=self._existing_ids,
            )
            self._phase = _PHASE_APPLY
            self._phase_started_at = now_s

    # ------------------------------------------------------------------
    # Apply (exactly one atomic diff per successful attempt)
    # ------------------------------------------------------------------

    def _build_apply_request(self, client):
        request = client.srv_type.Request()
        plan = self._diff_plan
        if plan is None:
            raise RuntimeError("fixture diff plan is not built")
        request.scene.is_diff = True
        for spec in plan.operations:
            request.scene.world.collision_objects.append(
                _spec_to_collision_object(
                    spec, mesh_loader=getattr(self, "_load_mesh", None)
                )
            )
        return request

    def _advance_apply(self, now_s: float) -> None:
        if self._apply_state.get("error"):
            self._fail(str(self._apply_state["error"]))
            return
        if self._past_start_deadline(now_s):
            self._fail("timed out applying the fixture planning scene")
            return
        if self._diff_plan is None:
            self._diff_plan = build_atomic_revision_diff(
                desired_objects=self._specs,
                existing_ids=self._existing_ids,
            )

        def create_client():
            return self.create_client(
                ApplyPlanningScene,
                _APPLY_SERVICE,
                callback_group=self._service_group,
            )

        def request(client):
            return self._build_apply_request(client)

        def extract(response):
            if response is None:
                return None
            return True if getattr(response, "success", False) else None

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
            self._apply_state,
            create_client=create_client,
            request=request,
            extract=extract,
            reset_client=reset_client,
            now_s=now_s,
            ttl_s=_SERVICE_TTL_S,
            timeout_s=_SERVICE_TIMEOUT_S,
        )
        if self._apply_state.get("error"):
            self._fail(str(self._apply_state["error"]))
            return
        if self._apply_state.get("succeeded"):
            self._phase = _PHASE_READBACK
            self._phase_started_at = now_s
            self.get_logger().info("fixture planning scene applied atomically")

    # ------------------------------------------------------------------
    # Readback
    # ------------------------------------------------------------------

    def _advance_readback(self, now_s: float) -> None:
        if self._readback_state.get("error"):
            self._fail(str(self._readback_state["error"]))
            return
        if self._past_start_deadline(now_s):
            self._fail("timed out reading back the fixture planning scene")
            return
        step = self._make_get_service(self._readback_state)
        step(now_s)
        if self._readback_state.get("error"):
            self._fail(str(self._readback_state["error"]))
            return
        if self._readback_state.get("succeeded"):
            self._scene_objects = tuple(self._readback_state["result"])
            self._scene_ids = tuple(str(obj["id"]) for obj in self._scene_objects)
            self._phase = _PHASE_CONFIRM

    # ------------------------------------------------------------------
    # Confirmation
    # ------------------------------------------------------------------

    def _advance_confirm(self) -> None:
        # The status the node will publish once ready must itself be a fully
        # self-consistent READY status; confirm it before committing the
        # transition so a malformed final status can never be served.
        status = self._current_status(state=FIXTURE_STATE_READY)
        expected_geometry = {
            spec.id: spec_geometry(spec, resolve_mesh=self._load_mesh)
            for spec in self._specs
        }
        confirmation = confirm_fixture_revision(
            service_result=True,
            scene_ids=self._scene_ids,
            status=status,
            expected_revision=self._revision,
            expected_digest=self._revision_digest,
            expected_owned_ids=self._owned_ids,
            expected_geometry=expected_geometry,
            observed_geometry=self._scene_objects,
        )
        if not confirmation.ready:
            self._fail("fixture readback/status confirmation failed: {}".format(
                "; ".join(confirmation.reasons)
            ))
            return
        self._phase = _PHASE_READY
        self._state = FIXTURE_STATE_READY
        self.get_logger().info(
            "fixture planning scene ready: revision={} owned={}".format(
                self._revision, ",".join(self._owned_ids)
            )
        )


def main(argv: list[str] | None = None) -> None:
    rclpy.init(args=argv)
    node = FixturePlanningScene()
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
