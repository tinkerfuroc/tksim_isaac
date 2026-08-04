"""Task 8 fix round 3 (Humble): live integrated driver provider tests.

Runs under sourced ROS Humble Python 3.10 with the simulator paths prepended
onto the sourced ``PYTHONPATH``:

.. code-block:: bash

    env -u COLCON_CURRENT_PREFIX
    source /opt/ros/humble/setup.bash
    source /home/tinker/tk25_ws/install/setup.bash
    export PYTHONPATH=/home/tinker/tinker-sim/6.0.1:/home/tinker/tinker-sim/6.0.1/validation:/home/tinker/tinker-sim/6.0.1/tests:/home/tinker/tinker-sim/6.0.1/simulation:/home/tinker/tinker-sim/6.0.1/ros2_ws/src/tinker_sim_bridge${PYTHONPATH:+:$PYTHONPATH}
    /usr/bin/python3 -m pytest -q /home/tinker/tinker-sim/6.0.1/tests/ros_humble/test_integrated_gate_executor_driver_providers.py

This module FAILS (never skips) when the Humble ROS runtime is unavailable.
It exercises the REAL driver construction/providers against the real immutable
``IntegratedGateExecutor`` on a controlled ROS graph.  Controlled nodes stand in
for MoveIt/controller/fixture; the driver provider code under test is never
replaced by doubles.
"""
from __future__ import annotations

import copy
import hashlib
import itertools
import json
import math
import os
import struct
import sys
import threading
import time
import uuid as _uuid
from array import array
from pathlib import Path

import pytest


def _require_ros_runtime() -> None:
    missing: list[str] = []
    for name in (
        "rclpy",
        "rcl_interfaces",
        "action_msgs",
        "sensor_msgs",
        "geometry_msgs",
        "moveit_msgs",
        "tf2_ros",
        "tf2_sensor_msgs",
        "controller_manager_msgs",
        "tinker_arm_msgs",
        "std_srvs",
    ):
        try:
            __import__(name)
        except ImportError as exc:
            missing.append(f"{name}: {exc}")
    if missing:
        pytest.fail(
            "Task-8 Humble provider suite requires the ROS Humble Python runtime; "
            "run under sourced /opt/ros/humble with the simulator paths prepended "
            "onto PYTHONPATH (see module docstring). Missing: "
            + "; ".join(missing)
        )


_require_ros_runtime()

import rclpy  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))
sys.path.insert(0, str(ROOT / "validation"))
sys.path.insert(0, str(ROOT / "tests"))

from tinker_sim_bridge.fixture_planning_scene import (  # noqa: E402
    canonical_fixture_status,
    fixture_descriptor_sha256,
    fixture_owned_ids,
    serialize_status,
)
from test_integrated_gate_executor import (  # noqa: E402
    POSITIVE_REPORT_CONTRACT,
    canonical_report_bytes,
    readiness_scenario,
    scenario_report_contract,
)
from validation.integrated_gate_executor import (  # noqa: E402
    FJT_ENDPOINT,
    FJT_STATUS_TOPIC,
    FIXTURE_PUBLISHER_NODE,
    FIXTURE_TOPIC,
    JOINT_STATES_TOPIC,
    OPERATOR_NODE,
    PLANNING_SCENE_TOPIC,
    MONITORED_PLANNING_SCENE_TOPIC,
    REQUIRED_ACTIONS,
    REQUIRED_SERVICES,
    SAFETY_STOP_TOPIC,
    CONTROLLER_MANAGER_NODE,
    IntegratedGateExecutor,
    _REQUIRED_ENDPOINT_SOURCES,
    _validate_observed_graph,
    build_execute_trajectory_goal,
    build_joint_move_group_goal,
    evaluate_executor_readiness,
)
from planning_scene_journal import validate_graph_evidence
import integrated_gate_executor_driver as d  # noqa: E402

Q_OUTBOUND = (0.20, -0.20, 0.15, 0.30, -0.15, 0.20, 0.15)
_JOINTS = [f"joint{i}" for i in range(1, 8)] + ["drive_joint"]
_DOMAIN = 47

# Each test gets a fresh ROS domain so Fast-DDS graph state from prior tests'
# (destroyed but not-yet-flushed) participants never interferes with the current
# test's service discovery / readiness.  Domains cycle within the valid [0, 232].
_domain_next = 101
_ACTIVE_DOMAIN = _DOMAIN


def _next_domain() -> int:
    global _domain_next, _ACTIVE_DOMAIN
    domain = _domain_next
    _domain_next += 1
    if _domain_next > 232:
        _domain_next = 0
    _ACTIVE_DOMAIN = domain
    return domain


def _planned_trajectory():
    """Build a structurally valid non-empty planned ``RobotTrajectory``."""
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    traj = RobotTrajectory()
    traj.joint_trajectory = JointTrajectory()
    point = JointTrajectoryPoint()
    point.positions = list(Q_OUTBOUND)
    traj.joint_trajectory.points.append(point)
    return traj


def _fresh_uuid_hex() -> str:
    return _uuid.uuid4().hex


def _bundle_from_contract(contract, scenario_id, attempt_dir) -> dict[str, object]:
    scenario_mapping = dict(contract["scenario_mapping"])
    scenario_mapping["id"] = scenario_id
    scenario_mapping["seed"] = contract["scenario_mapping"]["seed"]
    identities = dict(contract["identities"])
    identities["scenario_id"] = scenario_id
    return {
        "schema_version": 1,
        "scenario_id": scenario_id,
        "attempt_id": "humble-provider-attempt",
        "attempt_dir": str(attempt_dir),
        "scenario": scenario_mapping,
        "planning_scene": dict(contract["planning_scene"]),
        "planning_scene_declaration": dict(contract["planning_scene_declaration"]),
        "integrated": dict(contract["integrated"]),
        "report_identities": identities,
    }


def _contract_for(scenario_id: str) -> dict[str, object]:
    return copy.deepcopy(scenario_report_contract(scenario_id))


def _write_report(attempt_dir: Path, contract: dict[str, object]) -> bytes:
    report_bytes = canonical_report_bytes(contract)
    (attempt_dir / "scenario-runner.json").write_bytes(report_bytes)
    return report_bytes


def _canonical_fixture_payload(contract: dict[str, object], sequence: int = 2) -> str:
    declaration = contract["planning_scene_declaration"]
    status = canonical_fixture_status(
        scenario=contract["scenario_mapping"]["id"],
        revision=declaration["revision"],
        revision_digest=declaration["revision_digest"],
        sequence=sequence,
        published_at=1.0,
        owned_ids=fixture_owned_ids(declaration),
        target_source_id=declaration["target_source_id"],
        target_handoff=declaration["target_handoff"],
        descriptor_sha256=fixture_descriptor_sha256(declaration),
        state="FIXTURE_READY",
    )
    return serialize_status(status)


def _build_planning_scene_message(contract: dict[str, object]):
    """Build a real moveit_msgs PlanningScene carrying the declared fixtures."""
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import AllowedCollisionMatrix, CollisionObject, PlanningScene, RobotState
    from shape_msgs.msg import SolidPrimitive

    declaration = contract["planning_scene_declaration"]
    frame_id = str(declaration.get("frame_id", "base_link"))
    records = list(declaration.get("objects", []))
    by_id = {str(record["id"]): record for record in records}
    scene = PlanningScene()
    for object_id in fixture_owned_ids(declaration):
        record = by_id.get(str(object_id))
        if record is None:
            continue
        obj = CollisionObject()
        obj.id = str(object_id)
        obj.header.frame_id = frame_id
        primitive = record.get("primitive")
        if primitive is not None:
            sp = SolidPrimitive()
            sp.type = SolidPrimitive.BOX
            sp.dimensions = [float(v) for v in primitive["dimensions"]]
            obj.primitives.append(sp)
        pose = record.get("pose", {})
        xyz = pose.get("xyz", [0.0, 0.0, 0.0])
        quat = pose.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        p = Pose()
        p.position.x = float(xyz[0])
        p.position.y = float(xyz[1])
        p.position.z = float(xyz[2])
        p.orientation.x = float(quat[0])
        p.orientation.y = float(quat[1])
        p.orientation.z = float(quat[2])
        p.orientation.w = float(quat[3])
        obj.primitive_poses.append(p)
        scene.world.collision_objects.append(obj)
    scene.allowed_collision_matrix = AllowedCollisionMatrix()
    scene.robot_state = RobotState()
    return scene


class _ControlledGraph:
    """Controlled ROS graph standing in for MoveIt/controller/fixture.

    Runs its own executor in a daemon thread; publishes the joint/safety/fixture/
    collision/planning-scene/lidar streams, broadcasts static TF, and serves all
    required actions/services plus the pick_and_place parameter services.
    """

    def __init__(
        self,
        *,
        scenario_id: str = "qualification-pick-place-positive",
        domain: int | None = None,
        skip_fold_action: bool = False,
        joint_qos_best_effort: bool = False,
        publish_fixture: bool = True,
    ) -> None:
        self.scenario_id = scenario_id
        self.contract = _contract_for(scenario_id)
        self._domain = _next_domain() if domain is None else int(domain)
        self._sequence = 0
        self._controller_active = True
        self._safety_stop = False
        self._collision = False
        self._fixture_payload = _canonical_fixture_payload(self.contract, 2)
        self._fixture_sequence_lock: int | None = None
        self._skip_fold_action = bool(skip_fold_action)
        self._joint_qos_best_effort = bool(joint_qos_best_effort)
        self._publish_fixture = bool(publish_fixture)
        self._execute_result_builder = None
        self._move_result_builder = None
        self._fjt_emitter = None
        self._parameter_reject = False
        self._parameter_readback_override: float | None = None
        self._nodes: list[Node] = []

    # -- node/publisher helpers ---------------------------------------------

    def _add_node(self, name: str) -> Node:
        node = Node(
            name, namespace="/", cli_args=[], context=self._context, use_global_arguments=False
        )
        self._nodes.append(node)
        return node

    def _make_pub(self, node, msg_type, topic, *, depth=1, transient=False, best_effort=False):
        qos = QoSProfile(
            depth=depth,
            reliability=ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE,
        )
        return node.create_publisher(msg_type, topic, qos)

    def start(self) -> None:
        self._context = rclpy.Context()
        self._context.init(domain_id=int(self._domain))
        from action_msgs.msg import GoalStatusArray
        from controller_manager_msgs.srv import (
            ConfigureController,
            ListControllers,
            LoadController,
            SwitchController,
        )
        from moveit_msgs.msg import PlanningScene
        from moveit_msgs.srv import (
            ApplyPlanningScene,
            GetCartesianPath,
            GetPlanningScene,
            GetStateValidity,
        )
        from rclpy.action import ActionServer as _ActionServer
        from sensor_msgs.msg import JointState, PointCloud2
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger
        from tinker_arm_msgs.srv import ArmJointService

        self._joint_pub_node = self._add_node("controller_manager")
        self._safety_node = self._add_node("tinker_sim_safety_supervisor")
        self._fixture_node = self._add_node("fixture_planning_scene")
        self._move_group_node = self._add_node("move_group")
        self._gripper_node = self._add_node("tinker_sim_gripper_facade")
        self._pick_node = self._add_node("pick_and_place")
        self._gate_node = self._add_node("tinker_sim_physics_ready_gate")
        self._collision_node = self._add_node("sim_collision_source")
        self._tf_node = self._add_node("sim_static_tf")
        self._lidar_node = self._add_node("livox_lidar_source")

        # Continuous streams.
        self._joint_pub = self._make_pub(
            self._joint_pub_node, JointState, JOINT_STATES_TOPIC, depth=10,
            best_effort=self._joint_qos_best_effort,
        )
        self._safety_pub = self._make_pub(self._safety_node, Bool, SAFETY_STOP_TOPIC, transient=True)
        self._fixture_pub = self._make_pub(self._fixture_node, String, FIXTURE_TOPIC, transient=True)
        self._collision_pub = self._make_pub(self._collision_node, Bool, "/sim/safety/collision", transient=True)
        self._scene_pub = self._make_pub(self._move_group_node, PlanningScene, PLANNING_SCENE_TOPIC, depth=100)
        self._scene_pub_monitored = self._make_pub(self._move_group_node, PlanningScene, MONITORED_PLANNING_SCENE_TOPIC, depth=100)
        self._lidar_pub = self._make_pub(self._lidar_node, PointCloud2, "/livox/lidar", best_effort=True)

        # Static TF: base_link -> link_tcp and base_link -> livox360.
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage

        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster(self._tf_node)
        self._static_broadcaster.sendTransform(
            [
                self._tf("base_link", "link_tcp", 0.25, 0.0, 0.3),
                self._tf("base_link", "livox360", 0.12, 0.0, 0.25),
            ]
        )
        # Production robot_state_publisher emits a continuous dynamic /tf
        # stream; mirror that so the observer TF receipt age stays fresh.
        self._tf_dynamic_pub = self._make_pub(self._tf_node, TFMessage, "/tf", depth=100)

        # Action servers (all nine required).
        self._install_action_servers()

        # Required services.
        def _ok_response(_req, _resp):
            return _resp

        def _on_list_controllers(request, response):
            from controller_manager_msgs.msg import ControllerState

            def _state(name):
                state = "active"
                if name == "xarm7_traj_controller" and not self._controller_active:
                    state = "inactive"
                record = ControllerState()
                record.name = name
                record.state = state
                record.type = "joint_trajectory_controller/JointTrajectoryController"
                return record

            response.controller.append(_state("joint_state_broadcaster"))
            response.controller.append(_state("xarm7_traj_controller"))
            return response

        self._joint_pub_node.create_service(
            ListControllers, "/controller_manager/list_controllers", _on_list_controllers
        )

        for service_name, service_type, owner in (
            ("/controller_manager/load_controller", LoadController, self._joint_pub_node),
            ("/controller_manager/configure_controller", ConfigureController, self._joint_pub_node),
            ("/controller_manager/switch_controller", SwitchController, self._joint_pub_node),
            ("/get_planning_scene", GetPlanningScene, self._move_group_node),
            ("/apply_planning_scene", ApplyPlanningScene, self._move_group_node),
            ("/check_state_validity", GetStateValidity, self._move_group_node),
            ("/compute_cartesian_path", GetCartesianPath, self._move_group_node),
            ("/arm_joint_service", ArmJointService, self._pick_node),
            ("/sim/ready/physics", Trigger, self._gate_node),
            ("/sim/ready/fixture", Trigger, self._fixture_node),
        ):
            owner.create_service(service_type, service_name, _ok_response)

        # pick_and_place parameter services: rclpy auto-creates
        # ``/pick_and_place/set_parameters`` and ``/pick_and_place/get_parameters``
        # on every node.  Declare the production parameter and control the set
        # path with an on-set callback (reject / clamp readback) so the REAL
        # auto service is exercised, not a manual double.
        self._pick_node.declare_parameter("post_grasp_lift_m", 0.10)

        def _on_set_parameters(params):
            from rcl_interfaces.msg import SetParametersResult
            from rclpy.parameter import Parameter

            if self._parameter_reject:
                result = SetParametersResult()
                result.successful = False
                result.reason = "rejected by test"
                return result
            if self._parameter_readback_override is not None:
                # Clamp the stored value so a successful set reads back low.
                for i, param in enumerate(params):
                    if param.name == "post_grasp_lift_m":
                        params[i] = Parameter(param.name, value=self._parameter_readback_override)
            result = SetParametersResult()
            result.successful = True
            return result

        self._pick_node.add_on_set_parameters_callback(_on_set_parameters)

        # FJT status emitter (controller_manager).
        self._fjt_status_pub = self._make_pub(self._joint_pub_node, GoalStatusArray, FJT_STATUS_TOPIC, transient=True)

        # Spinner.
        self._executor = MultiThreadedExecutor(context=self._context)
        for node in self._nodes:
            self._executor.add_node(node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

        # Continuous stream publisher: the real simulator publishes the
        # joint/fixture/safety/collision/scene/lidar streams on a timer, so the
        # controlled graph must too — otherwise the executor/observer caches go
        # stale between test-driven spins (freshness gates are sub-second).
        self._stop_publish = False
        self._publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._publish_thread.start()

    def _publish_loop(self) -> None:
        while not self._stop_publish:
            try:
                self.publish_burst()
            except Exception:  # noqa: BLE001 - daemon publisher boundary
                pass
            time.sleep(0.05)

    def _tf(self, parent, child, x, y, z):
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()
        t.header.stamp = self._tf_node.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.w = 1.0
        return t

    # -- action servers ------------------------------------------------------

    def _install_action_servers(self) -> None:
        from control_msgs.action import FollowJointTrajectory, GripperCommand
        from moveit_msgs.action import ExecuteTrajectory, MoveGroup
        from tinker_arm_msgs.action import CartesianMove, Fold, JointMove, Pick, Place

        def _accept(goal_handle):
            from rclpy.action import GoalResponse

            return GoalResponse.ACCEPT

        def _make_simple_execute(result_type):
            def execute(goal_handle):
                result = result_type()
                goal_handle.succeed()
                return result

            return execute

        def _move_execute(goal_handle):
            if self._move_result_builder is not None:
                return self._move_result_builder(goal_handle)
            from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

            # A success MoveGroup result carries a planned trajectory for the
            # split/execute/cancel/safety pre-send path.
            result = MoveGroup.Result()
            result.error_code = MoveItErrorCodes()
            result.error_code.val = MoveItErrorCodes.SUCCESS
            result.planned_trajectory = RobotTrajectory()
            result.planned_trajectory.joint_trajectory = JointTrajectory()
            point = JointTrajectoryPoint()
            point.positions = list(Q_OUTBOUND)
            result.planned_trajectory.joint_trajectory.points.append(point)
            goal_handle.succeed()
            return result

        def _execute_execute(goal_handle):
            if self._execute_result_builder is not None:
                return self._execute_result_builder(goal_handle)
            result = ExecuteTrajectory.Result()
            goal_handle.succeed()
            if self._fjt_emitter is not None:
                self._fjt_emitter(goal_handle.goal_id)
            return result

        specs = [
            (self._move_group_node, MoveGroup, "/move_action", _move_execute),
            (self._move_group_node, ExecuteTrajectory, "/execute_trajectory", _execute_execute),
            (self._joint_pub_node, FollowJointTrajectory, "/xarm7_traj_controller/follow_joint_trajectory", _make_simple_execute(FollowJointTrajectory.Result)),
            (self._gripper_node, GripperCommand, "/xarm_gripper/gripper_action", _make_simple_execute(GripperCommand.Result)),
            (self._pick_node, Pick, "/pickup_action", _make_simple_execute(Pick.Result)),
            (self._pick_node, Place, "/place_action", _make_simple_execute(Place.Result)),
            (self._pick_node, CartesianMove, "/cartesian_move_action", _make_simple_execute(CartesianMove.Result)),
            (self._pick_node, JointMove, "/joint_move_action", _make_simple_execute(JointMove.Result)),
        ]
        if not self._skip_fold_action:
            specs.append((self._pick_node, Fold, "/fold_action", _make_simple_execute(Fold.Result)))
        self._action_servers = []
        for node, action_type, name, execute in specs:
            server = ActionServer(
                node,
                action_type,
                name,
                execute_callback=execute,
                goal_callback=_accept,
            )
            self._action_servers.append(server)

    # -- publish helpers ------------------------------------------------------

    def publish_burst(self) -> None:
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage

        self._sequence += 1
        # Continuous dynamic /tf stream (an empty TFMessage is structurally
        # valid and keeps the observer TF receipt time fresh).
        self._tf_dynamic_pub.publish(TFMessage())

        msg = JointState()
        msg.header.stamp = self._joint_pub_node.get_clock().now().to_msg()
        msg.name = list(_JOINTS)
        msg.position = [0.0] * 8
        msg.velocity = [0.0] * 8
        msg.effort = [0.0] * 8
        self._joint_pub.publish(msg)

        safety = self._safety_pub.msg_type()
        safety.data = self._safety_stop
        self._safety_pub.publish(safety)

        collision = self._collision_pub.msg_type()
        collision.data = self._collision
        self._collision_pub.publish(collision)

        if self._publish_fixture:
            fixture = self._fixture_pub.msg_type()
            seq = self._sequence if self._fixture_sequence_lock is None else self._fixture_sequence_lock
            fixture.data = _canonical_fixture_payload(self.contract, seq)
            self._fixture_pub.publish(fixture)

        scene = _build_planning_scene_message(self.contract)
        self._scene_pub.publish(scene)
        self._scene_pub_monitored.publish(scene)

        self._publish_lidar()

    def _publish_lidar(self) -> None:
        from sensor_msgs.msg import PointCloud2, PointField

        msg = PointCloud2()
        msg.header.stamp = self._lidar_node.get_clock().now().to_msg()
        msg.header.frame_id = "livox360"
        msg.height = 1
        msg.width = 3
        msg.is_bigendian = False
        msg.is_dense = True
        for index, name in enumerate(("x", "y", "z")):
            field = PointField()
            field.name = name
            field.offset = 4 * index
            field.datatype = PointField.FLOAT32
            field.count = 1
            msg.fields.append(field)
        msg.point_step = 12
        msg.row_step = 12 * msg.width
        msg.data = array("B", b"".join(struct.pack("<fff", *p) for p in ((0.5, 0.0, 0.0), (0.6, 0.1, 0.0), (0.7, -0.1, 0.0))))
        self._lidar_pub.publish(msg)

    def emit_fjt(self, goal_uuid_hex: str, status: int) -> None:
        """Publish a real GoalStatusArray FJT status entry (exact action schema)."""
        from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray

        msg = GoalStatusArray()
        status_entry = GoalStatus()
        info = GoalInfo()
        info.goal_id = _uuid.UUID(hex=goal_uuid_hex).bytes
        status_entry.goal_info = info
        status_entry.status = status
        msg.status_list.append(status_entry)
        self._fjt_status_pub.publish(msg)

    def stop(self) -> None:
        self._stop_publish = True
        publish_thread = getattr(self, "_publish_thread", None)
        if publish_thread is not None:
            try:
                publish_thread.join(timeout=2.0)
            except Exception:  # noqa: BLE001
                pass
        if getattr(self, "_executor", None) is not None:
            try:
                self._executor.shutdown()
            except Exception:
                pass
        for node in self._nodes:
            try:
                node.destroy_node()
            except Exception:
                pass
        self._nodes = []
        context = getattr(self, "_context", None)
        if context is not None:
            try:
                context.try_shutdown()
            except Exception:
                pass
            self._context = None


@pytest.fixture(scope="module")
def rclpy_runtime():
    if not rclpy.ok():
        rclpy.init(args=["--ros-args", "-r", f"__ns:=/"])
    yield
    if rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception:
            pass


@pytest.fixture()
def graph(rclpy_runtime):
    g = _ControlledGraph()
    g.start()
    yield g
    g.stop()


def _construct_real_executor(attempt_dir: Path, *, scenario_id: str = "qualification-pick-place-positive", domain_id: int | None = None):
    contract = _contract_for(scenario_id)
    bundle = _bundle_from_contract(contract, scenario_id, attempt_dir)
    _write_report(attempt_dir, contract)
    return d._construct_executor(
        bundle=bundle,
        attempt_dir=attempt_dir,
        config=_test_config(),
        domain_id=_ACTIVE_DOMAIN if domain_id is None else domain_id,
        seed=int(contract["scenario_mapping"]["seed"]),
    )


def _test_config() -> dict[str, object]:
    return {
        "execution_profile": "sim_ompl",
        "thresholds": {
            "joint_state_fresh_s": 0.5,
            "tf_fresh_s": 0.5,
            "fixture_fresh_s": 0.5,
            "operator_fresh_s": 0.5,
            "action_server_wait_s": 0.5,
            "goal_accept_timeout_s": 0.5,
            "plan_result_timeout_s": 0.5,
            "cancel_timeout_s": 0.5,
            "execute_timeout_s": 0.5,
            "scene_acquire_timeout_s": 0.5,
        },
    }


def _drive_spin(executor, *, until=None, timeout_s: float = 5.0, graph=None) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if graph is not None:
            graph.publish_burst()
        try:
            executor._spin_once()
        except Exception:
            pass
        if until is not None and until():
            return
        time.sleep(0.02)


# --------------------------------------------------------------------------- #
# F3.9 — readiness gate with the live observer
# --------------------------------------------------------------------------- #

def test_readiness_requires_the_spinner(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        # Before any spin the executor/observer caches are empty: not ready.
        first = executor._readiness()
        assert first is None or first.get("ready") is False

        # Drive the shared spinner; readiness becomes ready.
        outcome = d._wait_for_readiness(executor, timeout_s=15.0)
        assert outcome.get("ready") is True, outcome.get("reasons")
    finally:
        executor.shutdown()


def test_readiness_negative_mutations_fail_closed(tmp_path):
    def _missing_operator(executor):
        # The driver re-publishes the operator baseline inside the readiness
        # snapshot, so break the publisher to keep the operator stream absent.
        def _disabled(_value):
            raise RuntimeError("operator publisher disabled for mutation")
        executor.publish_operator = _disabled
        observer = executor._driver_observer
        observer.operator_samples = 0
        observer.operator_received_mono = None

    def _stale_fixture(executor):
        observer = executor._driver_observer
        observer.fixture_received_mono = time.monotonic() - 10.0
        executor._fixture_payload = None

    # graph_kwargs configure the graph before start(); setup/post_executor run
    # after start()/construction respectively.
    cases = [
        ("missing-operator", {}, None, _missing_operator),
        ("sequence-1-fixture", {}, lambda g: setattr(g, "_fixture_sequence_lock", 1), None),
        ("stale-fixture", {"publish_fixture": False}, None, _stale_fixture),
        ("inactive-controller", {}, lambda g: setattr(g, "_controller_active", False), None),
        ("asserted-collision", {}, lambda g: setattr(g, "_collision", True), None),
        ("missing-endpoint", {"skip_fold_action": True}, None, None),
        ("wrong-topic-qos", {"joint_qos_best_effort": True}, None, None),
    ]
    for scenario_id in ("qualification-pick-place-positive", "qualification-moveit-execute-joint"):
        for label, graph_kwargs, setup, post_executor in cases:
            g = _ControlledGraph(scenario_id=scenario_id, **graph_kwargs)
            g.start()
            executor = None
            try:
                if setup is not None:
                    setup(g)
                executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
                if post_executor is not None:
                    post_executor(executor)
                _drive_spin(executor, graph=g, timeout_s=2.0)
                outcome = d._wait_for_readiness(executor, timeout_s=2.0)
                assert outcome.get("ready") is False, f"{label} unexpectedly ready"
            finally:
                if executor is not None:
                    executor.shutdown()
                g.stop()


def test_server_is_ready_path_and_no_parameter_client(graph, tmp_path):
    import rclpy.parameter as _param

    assert hasattr(_param, "ParameterClient") is False
    assert not hasattr(d, "_parameter_client")
    executor = _construct_real_executor(Path(tmp_path))
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        assert executor._action_clients["/move_action"].server_is_ready() is True
        assert executor._service_clients["/controller_manager/list_controllers"].service_is_ready() is True
    finally:
        executor.shutdown()


# --------------------------------------------------------------------------- #
# F3.7 — service-based post_grasp_lift_m parameter protocol
# --------------------------------------------------------------------------- #

def test_parameter_set_get_acceptance_at_010(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        client = d._ParameterServiceClient(executor)
        observed = d.set_post_grasp_lift_m(client, value_m=0.10, timeout_s=5.0)
        assert observed["value_m"] == 0.10
        assert observed["requested_value_m"] == 0.10
    finally:
        executor.shutdown()


def test_parameter_low_readback_rejection(graph, tmp_path):
    graph._parameter_readback_override = 0.05
    executor = _construct_real_executor(Path(tmp_path))
    try:
        client = d._ParameterServiceClient(executor)
        with pytest.raises(d.LiftParameterError, match="below required"):
            d.set_post_grasp_lift_m(client, value_m=0.10, timeout_s=5.0)
    finally:
        executor.shutdown()


def test_parameter_set_rejected(graph, tmp_path):
    graph._parameter_reject = True
    executor = _construct_real_executor(Path(tmp_path))
    try:
        client = d._ParameterServiceClient(executor)
        with pytest.raises(d.LiftParameterError, match="rejected"):
            d.set_post_grasp_lift_m(client, value_m=0.10, timeout_s=5.0)
    finally:
        executor.shutdown()


# --------------------------------------------------------------------------- #
# F3.3 — real TF TCP pose + Option A+ livox cloud transform
# --------------------------------------------------------------------------- #

def test_tcp_pose_provider_positive(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        provider = d._current_tcp_pose_provider(executor)
        pose = provider()
        assert pose["frame_id"] == "base_link"
        assert len(pose["xyz"]) == 3 and all(math.isfinite(v) for v in pose["xyz"])
        assert len(pose["quaternion_xyzw"]) == 4
        assert pose["identity"].startswith("tf:base_link->link_tcp:")
        assert isinstance(pose["age_s"], float) and pose["age_s"] >= 0.0
    finally:
        executor.shutdown()


def test_tcp_pose_provider_missing_fails_closed(tmp_path):
    graph = _ControlledGraph()
    graph.start()
    executor = _construct_real_executor(Path(tmp_path))
    try:
        # Destroy the static TF node's transforms: use a fresh observer-less path.
        provider = d._current_tcp_pose_provider(executor)
        with pytest.raises(d.DriverError, match="no base_link -> link_tcp"):
            provider()
    finally:
        executor.shutdown()
        graph.stop()


def test_livox_cloud_provider_transforms_to_base_link(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        provider = d._environment_cloud_provider(executor)
        cloud = provider()
        assert getattr(cloud.header, "frame_id", "") == "base_link"
        assert cloud.width >= 1 and len(cloud.data) >= 1
    finally:
        executor.shutdown()


def test_livox_cloud_empty_and_stale_fail_closed(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        observer = executor._driver_observer
        assert observer is not None
        # Stale: back-date the receipt beyond the 5 s gate.
        observer.cloud_received_mono = time.monotonic() - d.ENV_CLOUD_MAX_AGE_S - 1.0
        with pytest.raises(d.DriverError, match="stale"):
            d._environment_cloud_provider(executor)()
        # Empty: zero-width cloud fails closed.
        observer.cloud_received_mono = time.monotonic()
        from sensor_msgs.msg import PointCloud2

        empty = PointCloud2()
        empty.header.frame_id = "livox360"
        empty.width = 0
        empty.height = 1
        empty.data = b""
        observer.latest_cloud = empty
        with pytest.raises(d.DriverError, match="empty"):
            d._environment_cloud_provider(executor)()
    finally:
        executor.shutdown()


def test_livox_cloud_wrong_frame_fails_closed(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        observer = executor._driver_observer
        from sensor_msgs.msg import PointCloud2, PointField

        msg = PointCloud2()
        msg.header.frame_id = "no_such_frame"
        msg.height = 1
        msg.width = 2
        msg.is_bigendian = False
        msg.is_dense = True
        for index, name in enumerate(("x", "y", "z")):
            field = PointField()
            field.name = name
            field.offset = 4 * index
            field.datatype = PointField.FLOAT32
            field.count = 1
            msg.fields.append(field)
        msg.point_step = 12
        msg.row_step = 24
        msg.data = array("B", b"".join(struct.pack("<fff", 0.5, 0.0, 0.0) for _ in range(2)))
        observer.latest_cloud = msg
        observer.cloud_received_mono = time.monotonic()
        with pytest.raises(d.DriverError):
            d._environment_cloud_provider(executor)()
    finally:
        executor.shutdown()


# --------------------------------------------------------------------------- #
# F3.4 — deterministic digest + FJT evidence
# --------------------------------------------------------------------------- #

def test_deterministic_serialize_and_execute_digest(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        _install_deterministic = d._install_deterministic_serialize
        # The driver already installed the seam in construction; verify it.
        trajectory = build_execute_trajectory_goal(_planned_trajectory()).trajectory
        first = executor.ros["serialize_message"](trajectory)
        second = executor.ros["serialize_message"](trajectory)
        assert bytes(first) == bytes(second)
        digest = hashlib.sha256(bytes(first)).hexdigest()
        # A fresh deterministic serialize of the same object is stable.
        third = executor.ros["serialize_message"](trajectory)
        assert hashlib.sha256(bytes(third)).hexdigest() == digest
    finally:
        executor.shutdown()


def test_fjt_provider_joins_newest_status(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        recorder = executor._execute_recorder
        goal = build_execute_trajectory_goal(_planned_trajectory())
        recorder.send_goal_async(goal)
        digest = recorder.last_trajectory_digest
        assert isinstance(digest, str) and len(digest) == 64
        uuid_hex = _fresh_uuid_hex()
        executor._seed_fjt_status(uuid_hex, 4, seq=executor._fjt_receipt_sequence + 1)
        record = d._fjt_transaction_provider(executor)()
        assert record["endpoint"] == FJT_ENDPOINT
        assert record["goal_uuid"] == uuid_hex
        assert record["trajectory_digest"] == digest
        assert record["status"] == 4
        assert record["source"]
        assert isinstance(record["sequence"], int) and record["sequence"] >= 0
        assert math.isfinite(record["timestamp"])
        # Join validates.
        ok, reason = executor._validate_fjt_evidence(record, expected_trajectory_digest=None)
        assert ok, reason
    finally:
        executor.shutdown()


def test_fjt_digest_uuid_status_mutations_fail(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        recorder = executor._execute_recorder
        goal = build_execute_trajectory_goal(_planned_trajectory())
        recorder.send_goal_async(goal)
        digest = recorder.last_trajectory_digest
        uuid_hex = _fresh_uuid_hex()
        executor._seed_fjt_status(uuid_hex, 4, seq=executor._fjt_receipt_sequence + 1)
        good = d._fjt_transaction_provider(executor)()
        ok, _ = executor._validate_fjt_evidence(good, expected_trajectory_digest=digest)
        assert ok
        # Digest mutation (the recorded digest is the authority).
        bad = dict(good)
        bad["trajectory_digest"] = "f" * 64
        ok, reason = executor._validate_fjt_evidence(bad, expected_trajectory_digest=digest)
        assert not ok and "trajectory digest" in (reason or "")
        # UUID mutation.
        bad = dict(good)
        bad["goal_uuid"] = _fresh_uuid_hex()
        ok, reason = executor._validate_fjt_evidence(bad, expected_trajectory_digest=digest)
        assert not ok and "goal_uuid" in (reason or "")
        # Status mutation.
        bad = dict(good)
        bad["status"] = 1
        ok, reason = executor._validate_fjt_evidence(bad, expected_trajectory_digest=digest)
        assert not ok and "status" in (reason or "")
    finally:
        executor.shutdown()


# --------------------------------------------------------------------------- #
# F3.6 — real journal graph projection
# --------------------------------------------------------------------------- #

def test_journal_graph_projection_positive(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        observed = d._observe_journal_graph(executor)
        # The observed graph passes the executor's own validator.
        _validate_observed_graph(observed)
        projection = d._build_journal_graph_projection(executor)
        assert projection["topics"][FIXTURE_TOPIC]["payload"]
        assert isinstance(validate_graph_evidence(projection), dict)
    finally:
        executor.shutdown()


def test_journal_graph_projection_mutations_fail(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        observed = d._observe_journal_graph(executor)
        # Missing publisher on the planning-scene topic.
        bad = copy.deepcopy(observed)
        bad["topics"][PLANNING_SCENE_TOPIC]["publishers"] = []
        with pytest.raises(ValueError, match="publishers"):
            _validate_observed_graph(bad)
        # Wrong fixture owner.
        bad = copy.deepcopy(observed)
        bad["topics"][FIXTURE_TOPIC]["publishers"][0]["node"] = "/wrong_owner"
        with pytest.raises(ValueError, match="fixture"):
            _validate_observed_graph(bad)
        # Wrong topic QoS.
        bad = copy.deepcopy(observed)
        bad["topics"][PLANNING_SCENE_TOPIC]["offered_qos"] = {
            "reliability": "BEST_EFFORT", "durability": "VOLATILE", "depth": 100,
        }
        with pytest.raises(ValueError, match="QoS"):
            _validate_observed_graph(bad)
        # Wrong service owner (no recorder client).
        bad = copy.deepcopy(observed)
        bad["services"]["/get_planning_scene"]["clients"] = [
            {"node": "/not_the_recorder", "node_namespace": ""}
        ]
        with pytest.raises(ValueError, match="called by"):
            _validate_observed_graph(bad)
    finally:
        executor.shutdown()


# --------------------------------------------------------------------------- #
# F3.5 — native gripper wrapper count + cancel/safety pre-send live handle
# --------------------------------------------------------------------------- #

def test_native_gripper_wrapper_count_and_freshness(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        provider = d._native_gripper_goal_count_provider(executor)
        before = provider()
        assert before["count"] >= 0
        recorder = executor._gripper_recorder
        from control_msgs.action import GripperCommand

        goal = GripperCommand.Goal()
        goal.command.position = 0.0
        recorder.send_goal_async(goal)
        after = provider()
        assert after["count"] == before["count"] + 1
        assert isinstance(after["age_s"], float) and after["age_s"] >= 0.0
    finally:
        executor.shutdown()


def test_cancel_presend_returns_live_execute_goal_handle(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path), scenario_id="qualification-moveit-cancel")
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        presend = d._presend_long_motion(executor, "qualification-moveit-cancel")
        assert isinstance(presend["planning_goal_id"], str) and len(presend["planning_goal_id"]) == 32
        assert isinstance(presend["execute_goal_id"], str) and len(presend["execute_goal_id"]) == 32
        handle = presend["execute_goal_handle"]
        assert getattr(handle, "accepted", False) is True
        # The handle's driver-normalized id equals the recorded execute id.
        assert d._goal_id_hex(handle) == presend["execute_goal_id"]
    finally:
        executor.shutdown()


# --------------------------------------------------------------------------- #
# F3.9 — controlled C / D-execute / D-retreat / E dispatch paths reach the
# immutable executor methods with the real driver provider factory.
# --------------------------------------------------------------------------- #

def _stub_action_clients_for_plan_only(executor, contract):
    """Replace the executor's action clients with controlled send-goal doubles
    that retain the full ActionClient interface (server_is_ready/wait_for_server/
    send_goal_async)."""
    from test_integrated_gate_executor_ros import (
        _FakeFuture,
        _FakeGoalHandle,
        _FakeResultResponse,
        _FakeMoveClient,
    )

    handle = _FakeGoalHandle(accepted=True, result=_success_result(), goal_id=_uuid.uuid4().bytes)
    clients = {
        "/move_action": _FakeMoveClient(server_ready=True, goal_handle=handle, send_ready_at=0.0),
        "/execute_trajectory": executor._action_clients["/execute_trajectory"],
        "/xarm7_traj_controller/follow_joint_trajectory": executor._action_clients["/xarm7_traj_controller/follow_joint_trajectory"],
        "/xarm_gripper/gripper_action": executor._action_clients["/xarm_gripper/gripper_action"],
        "/pickup_action": executor._action_clients["/pickup_action"],
        "/place_action": executor._action_clients["/place_action"],
        "/cartesian_move_action": executor._action_clients["/cartesian_move_action"],
        "/joint_move_action": executor._action_clients["/joint_move_action"],
        "/fold_action": executor._action_clients["/fold_action"],
    }
    executor._action_clients = clients
    return clients


def _success_result():
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import MoveItErrorCodes, RobotTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    result = MoveGroup.Result()
    result.error_code = MoveItErrorCodes()
    result.error_code.val = MoveItErrorCodes.SUCCESS
    result.planned_trajectory = RobotTrajectory()
    result.planned_trajectory.joint_trajectory = JointTrajectory()
    point = JointTrajectoryPoint()
    point.positions = list(Q_OUTBOUND)
    result.planned_trajectory.joint_trajectory.points.append(point)
    return result


def test_controlled_c_path_reaches_run_gate_c_plan_only(tmp_path):
    scenario_id = "qualification-moveit-plan-joint"
    graph = _ControlledGraph(scenario_id=scenario_id)
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        assert d._wait_for_readiness(executor, timeout_s=5.0)["ready"] is True
        kwargs = d._live_runtime_provider_factory(
            executor=executor, scenario_id=scenario_id, bundle=None,
            config=_test_config(), attempt_dir=Path(tmp_path),
        )
        assert kwargs == {}
        _stub_action_clients_for_plan_only(executor, POSITIVE_REPORT_CONTRACT)
        record = executor.run_gate_c_plan_only(scenario_id)
        assert record["status"] in {"diagnostic-pass", "verified-pass", "evidence-invalid"}
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_d_execute_path_reaches_run_execute_sequence(tmp_path):
    scenario_id = "qualification-moveit-execute-joint"
    graph = _ControlledGraph(scenario_id=scenario_id)
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        assert d._wait_for_readiness(executor, timeout_s=5.0)["ready"] is True
        kwargs = d._live_runtime_provider_factory(
            executor=executor, scenario_id=scenario_id, bundle=None,
            config=_test_config(), attempt_dir=Path(tmp_path),
        )
        assert "fjt_transaction_provider" in kwargs
        _stub_action_clients_for_plan_only(executor, POSITIVE_REPORT_CONTRACT)
        record = executor.run_execute_sequence(scenario_id, **kwargs)
        assert isinstance(record, dict) and "status" in record
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_d_retreat_path_reaches_run_cartesian_retreat(tmp_path):
    scenario_id = "qualification-moveit-cartesian-retreat"
    graph = _ControlledGraph(scenario_id=scenario_id)
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        assert d._wait_for_readiness(executor, timeout_s=5.0)["ready"] is True
        kwargs = d._live_runtime_provider_factory(
            executor=executor, scenario_id=scenario_id, bundle=None,
            config=_test_config(), attempt_dir=Path(tmp_path),
        )
        assert "current_tcp_pose_provider" in kwargs
        assert "environment_cloud_provider" in kwargs
        record = executor.run_cartesian_retreat(scenario_id, **kwargs)
        assert isinstance(record, dict) and "status" in record
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_e_path_reaches_run_pick_place_sequence(graph, tmp_path):
    scenario_id = "qualification-pick-place-positive"
    executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        assert d._wait_for_readiness(executor, timeout_s=5.0)["ready"] is True
        kwargs = d._live_runtime_provider_factory(
            executor=executor, scenario_id=scenario_id, bundle=None,
            config=_test_config(), attempt_dir=Path(tmp_path),
        )
        assert "current_tcp_pose_provider" in kwargs
        assert "native_gripper_goal_count_provider" in kwargs
        assert "post_grasp_lift_m_provider" in kwargs
        record = executor.run_pick_place_sequence(scenario_id, **kwargs)
        assert isinstance(record, dict) and "status" in record
    finally:
        executor.shutdown()
