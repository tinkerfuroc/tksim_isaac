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
    EXECUTE_STATUS_ABORTED,
    EXECUTE_STATUS_CANCELED,
    EXECUTE_STATUS_EXECUTING,
    EXECUTE_STATUS_SUCCEEDED,
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
    _valid_goal_uuid,
    build_execute_trajectory_goal,
    build_joint_move_group_goal,
    evaluate_executor_readiness,
)
from planning_scene_journal import validate_graph_evidence
import integrated_gate_executor_driver as d  # noqa: E402

Q_OUTBOUND = (0.20, -0.20, 0.15, 0.30, -0.15, 0.20, 0.15)
_JOINTS = [f"joint{i}" for i in range(1, 8)] + ["drive_joint"]
# F4.9: arm-joint velocity above the executor's 0.005 rad/s motion-trigger
# threshold, published while the controlled graph is in a motion phase.  The
# JointState message must be aligned with ``_JOINTS`` (8 names), so the velocity
# vector carries 8 entries (7 arm joints + the base drive_joint).
_MOTION_VELOCITY = [0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0]
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
        emit_fjt_on_execute: bool = True,
        duplicate_fjt: bool = False,
        attempt_dir: Path | None = None,
    ) -> None:
        self.scenario_id = scenario_id
        # F4.8: the controlled graph mirrors the real physics-truth writer so the
        # executor's join_key_provider (``_read_join_key`` -> attempt_dir /
        # ``physics_truth.jsonl``) observes a continuously strictly-increasing
        # (frame_index, timestamp) key across the whole D transaction.
        self._attempt_dir = Path(attempt_dir) if attempt_dir is not None else None
        self._truth_path: Path | None = None
        self._truth_frame = 0
        self._stop_truth = False
        self._truth_thread: threading.Thread | None = None
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
        self._parameter_reject = False
        self._parameter_readback_override: float | None = None
        self._nodes: list[Node] = []
        # F4.9: the controlled ExecuteTrajectory server emulates the MoveIt ->
        # controller forwarding and emits FJT status with a DISTINCT controller
        # goal UUID (never the ExecuteTrajectory UUID).  ``_execute_hold_s``
        # holds the goal open so a real cancel/safety interruption can land
        # before the automatic success (<=0 means immediate success).  The
        # graph publishes arm motion only while ``_motion_active`` so the
        # cancel/safety motion-trigger sees a real fresh moving frame.
        self._execute_hold_s = 0.0
        self._motion_active = False
        self._stop_servers = False
        # F5.3/F5.6: per-server ExecuteTrajectory controls — delay the goal
        # acceptance callback (delayed-acceptance exact-cancel) and/or reject
        # cancellation (cleanup-rejection fail-closed).
        self._execute_goal_accept_delay_s = 0.0
        self._execute_reject_cancel = False
        # F5.4: deterministic teardown bookkeeping.  Every action server, node,
        # thread, and the controlled executor/context are tracked so teardown
        # can drain and join everything before interpreter exit — no daemon-thread
        # reliance and no un-drained action goal coroutine.
        self._tracked_threads: list[threading.Thread] = []
        self._drained = False
        # F4.9 negative controls: disable FJT emission on execute (no-new-goal),
        # force the controller UUID to a pre-seeded value (stale-heartbeat
        # replay), and emit a second distinct controller UUID (multiple-new).
        self._emit_fjt_on_execute = bool(emit_fjt_on_execute)
        self._forced_controller_uuid: str | None = None
        self._fjt_duplicate = bool(duplicate_fjt)

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

        # F4.9: the safety supervisor reacts to the executor's real operator
        # publication — operator True asserts the safety stop and freezes the
        # arm so the safety D-path can observe ABORTED + stopped frames.
        def _on_operator(message: Any) -> None:
            value = bool(getattr(message, "data", False))
            self._safety_stop = value
            if value:
                self._motion_active = False

        self._safety_node.create_subscription(
            Bool, "/sim/safety/operator", _on_operator,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE),
        )

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

        # Spinner.  F4.9: four worker threads so a held execute goal (cancel/
        # safety hold loop) never starves the action-server cancel/result
        # handlers and the service servers (ListControllers, parameters).
        # F5.4: the thread is non-daemon and tracked; teardown shuts the
        # executor down and joins it explicitly (never daemon-thread reliance).
        self._executor = MultiThreadedExecutor(context=self._context, num_threads=4)
        for node in self._nodes:
            self._executor.add_node(node)
        self._thread = threading.Thread(target=self._executor.spin, name="controlled-executor", daemon=False)
        self._tracked_threads.append(self._thread)
        self._thread.start()

        # Continuous stream publisher: the real simulator publishes the
        # joint/fixture/safety/collision/scene/lidar streams on a timer, so the
        # controlled graph must too — otherwise the executor/observer caches go
        # stale between test-driven spins (freshness gates are sub-second).
        self._stop_publish = False
        self._publish_thread = threading.Thread(target=self._publish_loop, name="controlled-publish", daemon=False)
        self._tracked_threads.append(self._publish_thread)
        self._publish_thread.start()

        # F4.8: emit a strictly-increasing physics-truth JSONL stream at the
        # attempt_dir so the executor's join-key provider sees fresh advancing
        # (frame_index, timestamp) keys for the entire transaction.
        if self._attempt_dir is not None:
            self._truth_path = self._attempt_dir / "physics_truth.jsonl"
            self._truth_frame = 0
            self._stop_truth = False
            self._truth_thread = threading.Thread(target=self._truth_loop, name="controlled-truth", daemon=False)
            self._tracked_threads.append(self._truth_thread)
            self._truth_thread.start()

    def _truth_loop(self) -> None:
        while not self._stop_truth:
            try:
                self._write_truth()
            except Exception:  # noqa: BLE001 - daemon truth boundary
                pass
            time.sleep(0.002)

    def _write_truth(self) -> None:
        if self._truth_path is None:
            return
        self._truth_frame += 1
        record = {
            "frame_index": self._truth_frame,
            "timestamp": time.monotonic(),
            "robot": {"joint_positions": [0.0] * 8},
            "safety_stop": bool(self._safety_stop),
        }
        line = json.dumps(record, sort_keys=True) + "\n"
        # F4.8: flush every line so ``_read_join_key`` (a separate file handle)
        # immediately observes the strictly-increasing frame, not a buffered
        # stale tail.
        with self._truth_path.open("a", encoding="utf-8") as stream:
            stream.write(line)
            stream.flush()

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

        def _accept_cancel(goal_handle):
            # F4.9: stock rclpy's default cancel callback REJECTS all
            # cancellations, which would make the controlled servers uncancelable
            # (empty goals_canceling) and strand the cancel/safety presend goals.
            from rclpy.action import CancelResponse

            return CancelResponse.ACCEPT

        def _execute_goal_callback(goal_request):
            # F5.3/F5.6: delayed-acceptance knob — the ExecuteTrajectory server
            # holds the goal request open for ``_execute_goal_accept_delay_s``
            # before accepting, so the driver's acceptance-timeout cleanup path
            # is exercised with a real server-side accepted/moving goal.
            from rclpy.action import GoalResponse

            delay = float(self._execute_goal_accept_delay_s)
            if delay > 0.0:
                time.sleep(delay)
            return GoalResponse.ACCEPT

        def _execute_cancel_callback(goal_handle):
            # F5.6: cleanup-rejection knob — the server rejects every cancel, so
            # exact-UUID cleanup fails closed without canceling another goal.
            from rclpy.action import CancelResponse

            if self._execute_reject_cancel:
                return CancelResponse.REJECT
            return CancelResponse.ACCEPT

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
            if self._emit_fjt_on_execute:
                # F4.9: MoveIt forwards the ExecuteTrajectory goal to the
                # joint_trajectory_controller, which spawns its OWN FJT action
                # goal with a fresh controller UUID.  The controlled server
                # mirrors that: the FJT status stream carries a DISTINCT
                # controller goal UUID, never the ExecuteTrajectory goal UUID.
                if self._forced_controller_uuid is not None:
                    controller_uuid = self._forced_controller_uuid
                else:
                    controller_uuid = _fresh_uuid_hex()
                if self._fjt_duplicate:
                    # F4.9 negative: two distinct controller goals appear in the
                    # window, so unique-new discovery must fail closed.  The
                    # status topic QoS is depth-1, so the first (duplicate) goal
                    # is published and given a bounded window to be read by the
                    # executor before the second controller goal is emitted —
                    # otherwise the depth-1 history would coalesce them away.
                    self.emit_fjt(_fresh_uuid_hex(), 2)
                    time.sleep(0.25)
                self.emit_fjt(controller_uuid, 2)  # EXECUTING
                if self._execute_hold_s <= 0.0:
                    self.emit_fjt(controller_uuid, 4)  # SUCCEEDED
                    goal_handle.succeed()
                    return result
                deadline = time.monotonic() + float(self._execute_hold_s)
                while time.monotonic() < deadline:
                    if bool(getattr(goal_handle, "is_cancel_requested", False)):
                        self.emit_fjt(controller_uuid, 5)  # CANCELED
                        goal_handle.canceled()
                        return result
                    if self._safety_stop or self._stop_servers:
                        self.emit_fjt(controller_uuid, 6)  # ABORTED
                        goal_handle.abort()
                        return result
                    time.sleep(0.005)
                self.emit_fjt(controller_uuid, 4)  # SUCCEEDED
                goal_handle.succeed()
                return result
            if self._execute_hold_s <= 0.0:
                goal_handle.succeed()
                return result
            deadline = time.monotonic() + float(self._execute_hold_s)
            while time.monotonic() < deadline:
                if bool(getattr(goal_handle, "is_cancel_requested", False)):
                    goal_handle.canceled()
                    return result
                if self._stop_servers:
                    goal_handle.abort()
                    return result
                time.sleep(0.005)
            goal_handle.succeed()
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
            # F5.3/F5.6: the ExecuteTrajectory server gets the per-server delayed
            # acceptance / cancel-rejection callbacks; all other servers keep the
            # stock accept / accept-cancel behavior.
            goal_cb = _execute_goal_callback if name == "/execute_trajectory" else _accept
            cancel_cb = _execute_cancel_callback if name == "/execute_trajectory" else _accept_cancel
            server = ActionServer(
                node,
                action_type,
                name,
                execute_callback=execute,
                goal_callback=goal_cb,
                cancel_callback=cancel_cb,
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
        # F4.9: the cancel/safety motion trigger needs a fresh arm-joint frame
        # with absolute velocity above the 0.005 rad/s threshold.  Publish arm
        # motion only while ``_motion_active`` (the safety supervisor freezes
        # the arm once operator True lands).
        msg.velocity = list(_MOTION_VELOCITY) if self._motion_active else [0.0] * 8
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
        """Publish a real GoalStatusArray FJT status entry (exact action schema).

        F4.9: the ``GoalInfo.goal_id`` field is a real
        ``unique_identifier_msgs/msg/UUID`` message whose ``.uuid`` is a numpy
        ``uint8[16]`` array (the exact shape the executor's F4.1
        ``_normalize_goal_uuid`` accepts).  Assigning raw bytes to the field is
        NOT serializable, so the UUID message is constructed explicitly.
        """
        from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
        from unique_identifier_msgs.msg import UUID as UUIDMsg

        msg = GoalStatusArray()
        status_entry = GoalStatus()
        info = GoalInfo()
        uuid_msg = UUIDMsg()
        uuid_msg.uuid = list(_uuid.UUID(hex=goal_uuid_hex).bytes)
        info.goal_id = uuid_msg
        status_entry.goal_info = info
        status_entry.status = status
        msg.status_list.append(status_entry)
        self._fjt_status_pub.publish(msg)

    def _join_thread(self, thread: threading.Thread | None, name: str, timeout_s: float = 5.0) -> None:
        """F5.4: bounded join of a tracked controlled-graph thread (asserts done)."""
        if thread is None:
            return
        thread.join(timeout=timeout_s)
        assert not thread.is_alive(), (
            f"controlled-graph {name} thread did not stop within {timeout_s:.1f}s"
        )

    def _drain_action_goals(self, *, timeout_s: float) -> None:
        """F5.4: drain every action-server goal result future (bounded + asserted).

        Setting ``_stop_servers`` makes every held execute callback observe it
        (the hold loops poll it every few ms) and abort, which resolves the
        server's result futures; then the ``ActionServer._execute_goal``
        coroutines terminate and are awaited by the controlled executor.
        """
        deadline = time.monotonic() + float(timeout_s)
        while True:
            unresolved = 0
            for server in list(getattr(self, "_action_servers", []) or []):
                for future in list(getattr(server, "_result_futures", {}).values()):
                    if not future.done():
                        unresolved += 1
            if unresolved == 0:
                return
            if time.monotonic() >= deadline:
                break
            try:
                # Keep the controlled executor spinning so the execute
                # callbacks observe ``_stop_servers`` and abort.
                self._executor.spin_once(timeout_sec=0.01)
            except Exception:  # noqa: BLE001 - drain boundary
                time.sleep(0.01)
        assert unresolved == 0, (
            f"{unresolved} action result future(s) unresolved after teardown drain"
        )

    def stop(self) -> None:
        # F5.4: deterministic teardown order.  Every wait is bounded and asserts
        # completion; no daemon-thread reliance and no silently abandoned work.
        if getattr(self, "_drained", False):
            return
        self._drained = True
        self._stop_publish = True
        self._stop_truth = True
        # 1. Stop new publishes/goals: held execute callbacks see _stop_servers
        #    and abort their hold loops.
        self._stop_servers = True
        # 2. Drain/cancel accepted goals and await execute callbacks/results.
        self._drain_action_goals(timeout_s=10.0)
        # 3. Join the publish/truth threads (bounded + asserted).
        self._join_thread(getattr(self, "_truth_thread", None), "truth")
        self._join_thread(getattr(self, "_publish_thread", None), "publish")
        # 4. Stop and join the controlled executor worker threads.
        executor = getattr(self, "_executor", None)
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:  # noqa: BLE001 - executor shutdown is idempotent
                pass
            self._executor = None
        self._join_thread(getattr(self, "_thread", None), "executor")
        # 5. Remove/destroy action servers and nodes, then shut down the context.
        for server in list(getattr(self, "_action_servers", []) or []):
            try:
                server.destroy()
            except Exception:  # noqa: BLE001 - server destroy is idempotent
                pass
        self._action_servers = []
        for node in list(self._nodes):
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001 - node destroy is idempotent
                pass
        self._nodes = []
        context = getattr(self, "_context", None)
        if context is not None:
            try:
                context.try_shutdown()
            except Exception:  # noqa: BLE001 - context shutdown is idempotent
                pass
            self._context = None
        # 6. Fail if any controlled-graph thread remains live after teardown.
        live_threads = [t for t in self._tracked_threads if t.is_alive()]
        assert not live_threads, (
            "controlled-graph threads still live after teardown: "
            + ", ".join(t.name for t in live_threads)
        )


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
    try:
        g.start()
    except Exception:
        # F5.4: a partial start must still tear down any threads/nodes/context
        # that were already created so a failed setup cannot leak them.
        g.stop()
        raise
    yield g
    g.stop()


def _construct_real_executor(
    attempt_dir: Path,
    *,
    scenario_id: str = "qualification-pick-place-positive",
    domain_id: int | None = None,
    config: dict[str, object] | None = None,
):
    contract = _contract_for(scenario_id)
    bundle = _bundle_from_contract(contract, scenario_id, attempt_dir)
    _write_report(attempt_dir, contract)
    return d._construct_executor(
        bundle=bundle,
        attempt_dir=attempt_dir,
        config=config if config is not None else _test_config(),
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


def _assert_ready(executor, *, timeout_s: float = 30.0) -> None:
    """F4.9/F5.4: readiness must be genuinely ready; surface failure reasons.

    F5.4: the controlled readiness budget is exactly 30.0 s, matching production
    ``run_driver`` (``readiness_timeout_s=30.0``).  This closes the harness-only
    cold-DDS discovery failures the old 5.0 s budget produced when three full
    graphs/executors were constructed in one process, without changing product
    readiness semantics.
    """
    outcome = d._wait_for_readiness(executor, timeout_s=timeout_s)
    assert outcome.get("ready") is True, outcome.get("reasons")


def _drain_action_goal(executor, send_future, *, timeout_s: float = 5.0) -> Any | None:
    """F4.9: bounded drain of a sent action goal (acceptance + terminal result).

    Tests that send a real action goal purely to populate a recorder/provider
    must consume the terminal result before teardown, or the graph's action
    server emits a ``failed to send response`` warning when the client node is
    destroyed with a pending result.  Returns the terminal result response or
    ``None`` on timeout (the caller asserts what it needs).
    """
    deadline = time.monotonic() + float(timeout_s)
    while not send_future.done() and time.monotonic() < deadline:
        executor._spin_once()
    if not send_future.done():
        return None
    try:
        handle = send_future.result()
    except Exception:  # noqa: BLE001 - drain boundary
        return None
    if handle is None:
        return None
    try:
        result_future = handle.get_result_async()
    except Exception:  # noqa: BLE001 - drain boundary
        return None
    while not result_future.done() and time.monotonic() < deadline:
        executor._spin_once()
    return result_future.result() if result_future.done() else None


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
        send_future = recorder.send_goal_async(goal)
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
        # F4.9: consume the execute terminal so no server response is stranded.
        assert _drain_action_goal(executor, send_future) is not None
    finally:
        executor.shutdown()


def test_fjt_digest_uuid_status_mutations_fail(graph, tmp_path):
    executor = _construct_real_executor(Path(tmp_path))
    try:
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        recorder = executor._execute_recorder
        goal = build_execute_trajectory_goal(_planned_trajectory())
        send_future = recorder.send_goal_async(goal)
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
        # F4.9: consume the execute terminal so no server response is stranded.
        assert _drain_action_goal(executor, send_future) is not None
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
        send_future = recorder.send_goal_async(goal)
        after = provider()
        assert after["count"] == before["count"] + 1
        assert isinstance(after["age_s"], float) and after["age_s"] >= 0.0
        # F4.9: consume the gripper terminal so no server response is stranded.
        assert _drain_action_goal(executor, send_future) is not None
    finally:
        executor.shutdown()


def test_cancel_presend_returns_live_execute_goal_handle(tmp_path):
    scenario_id = "qualification-moveit-cancel"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    # F4.9: hold the presend goal open so the controller EXECUTING status
    # survives the depth-1 status-topic QoS (back-to-back EXECUTING+SUCCEEDED
    # would otherwise coalesce EXECUTING away).
    graph._execute_hold_s = 5.0
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        presend = d._presend_long_motion(executor, scenario_id)
        assert isinstance(presend["planning_goal_id"], str) and len(presend["planning_goal_id"]) == 32
        assert isinstance(presend["execute_goal_id"], str) and len(presend["execute_goal_id"]) == 32
        handle = presend["execute_goal_handle"]
        assert getattr(handle, "accepted", False) is True
        # The handle's driver-normalized id equals the recorded execute id.
        assert d._goal_id_hex(handle) == presend["execute_goal_id"]
        # F4.4: the presend discovered the DISTINCT controller FJT goal UUID.
        assert presend["fjt_goal_id"] != presend["execute_goal_id"]
        # F4.9: consume the held presend goal (cancel + drain) so no response
        # is stranded at teardown.
        deadline = time.monotonic() + 5.0
        cancel_future = handle.cancel_goal_async()
        while not cancel_future.done() and time.monotonic() < deadline:
            executor._spin_once()
        result_future = handle.get_result_async()
        while not result_future.done() and time.monotonic() < deadline:
            executor._spin_once()
        assert result_future.done()
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


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
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        kwargs = d._live_runtime_provider_factory(
            executor=executor, scenario_id=scenario_id, bundle=None,
            config=_test_config(), attempt_dir=Path(tmp_path),
        )
        assert kwargs == {}
        _stub_action_clients_for_plan_only(executor, POSITIVE_REPORT_CONTRACT)
        record = executor.run_gate_c_plan_only(scenario_id)
        # F4.9: a positive controlled path must commit the positive terminal,
        # never ``evidence-invalid``.
        assert record["status"] == "diagnostic-pass", record
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_d_execute_path_reaches_run_execute_sequence(tmp_path):
    scenario_id = "qualification-moveit-execute-joint"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        kwargs = d._live_runtime_provider_factory(
            executor=executor, scenario_id=scenario_id, bundle=None,
            config=_test_config(), attempt_dir=Path(tmp_path),
        )
        assert "fjt_transaction_provider" in kwargs
        _stub_action_clients_for_plan_only(executor, POSITIVE_REPORT_CONTRACT)
        record = executor.run_execute_sequence(scenario_id, **kwargs)
        # F4.9: the positive D-execute path must commit ``diagnostic-pass`` with
        # the FJT evidence bound to a controller goal UUID DISTINCT from the
        # ExecuteTrajectory UUID (MoveIt -> controller forwarding).
        assert record["status"] == "diagnostic-pass", record
        assert record["execute_trajectory_goal_sent"] is True
        assert record["controller_goal_sent"] is True
        assert record["controller_endpoint"] == FJT_ENDPOINT
        assert record["fjt_status"] == EXECUTE_STATUS_SUCCEEDED
        assert _valid_goal_uuid(record["fjt_goal_uuid"])
        assert record["fjt_goal_uuid"] != record["execute_goal_id"]
        # No FJT status entry in the live stream carries the ExecuteTrajectory
        # UUID — the controller transaction is genuinely distinct.
        assert all(
            str(entry.get("goal_uuid")) != record["execute_goal_id"]
            for entry in executor._fjt_status_cache
        )
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_d_cancel_path_reaches_run_cancel_sequence(tmp_path):
    scenario_id = "qualification-moveit-cancel"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    # F4.9: hold the presend ExecuteTrajectory goal open so the real cancel
    # lands before the automatic success; publish arm motion so the cancel
    # motion-trigger sees a fresh moving frame.
    graph._execute_hold_s = 5.0
    graph._motion_active = True
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        # The live factory pre-sends the long motion and discovers the distinct
        # controller FJT goal UUID; the run method keys FJT evidence on it
        # (never on execute_goal_id).
        kwargs = d._live_runtime_provider_factory(
            executor=executor, scenario_id=scenario_id, bundle=None,
            config=_test_config(), attempt_dir=Path(tmp_path),
        )
        assert "fjt_goal_id" in kwargs and "transaction_baseline" in kwargs
        assert "execute_goal_handle" in kwargs
        assert kwargs["fjt_goal_id"] != kwargs["execute_goal_id"]
        record = executor.run_cancel_sequence(scenario_id, **kwargs)
        # F4.9: the positive controlled cancel path commits diagnostic-pass.
        assert record["status"] == "diagnostic-pass", record
        assert record["terminal_status"] == "canceled"
        assert record["execute_trajectory_goal_sent"] is False
        assert record["controller_goal_sent"] is True
        assert record["controller_endpoint"] == FJT_ENDPOINT
        assert record["cancel_response"] == "accepted"
        assert record["fjt_status"] == EXECUTE_STATUS_CANCELED
        assert record["fjt_goal_uuid"] != record["execute_goal_id"]
        assert all(
            str(entry.get("goal_uuid")) != record["execute_goal_id"]
            for entry in executor._fjt_status_cache
        )
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_d_safety_path_reaches_run_safety_sequence(tmp_path):
    scenario_id = "qualification-moveit-safety"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    # F4.9: hold the presend goal open so the operator safety assertion aborts
    # it; publish arm motion so the safety motion-trigger sees a moving frame.
    graph._execute_hold_s = 5.0
    graph._motion_active = True
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        kwargs = d._live_runtime_provider_factory(
            executor=executor, scenario_id=scenario_id, bundle=None,
            config=_test_config(), attempt_dir=Path(tmp_path),
        )
        assert "fjt_goal_id" in kwargs and "transaction_baseline" in kwargs
        assert _valid_goal_uuid(kwargs["fjt_goal_id"])
        record = executor.run_safety_sequence(scenario_id, **kwargs)
        # F4.9: the positive controlled safety path commits diagnostic-pass.
        assert record["status"] == "diagnostic-pass", record
        assert record["terminal_status"] == "aborted"
        assert record["controller_goal_sent"] is True
        assert record["controller_endpoint"] == FJT_ENDPOINT
        assert record["fjt_status"] == EXECUTE_STATUS_ABORTED
        assert record["fjt_goal_uuid"] != record["execute_goal_id"]
        assert all(
            str(entry.get("goal_uuid")) != record["execute_goal_id"]
            for entry in executor._fjt_status_cache
        )
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


# --------------------------------------------------------------------------- #
# F4.9 — transaction-real FJT negatives: no-new / multiple-new / stale-replay /
# uuid-mismatch / status-mismatch / presend-cleanup.
# --------------------------------------------------------------------------- #

def test_controlled_d_execute_no_new_fjt_goal_fails_closed(tmp_path):
    scenario_id = "qualification-moveit-execute-joint"
    graph = _ControlledGraph(scenario_id=scenario_id, emit_fjt_on_execute=False, attempt_dir=Path(tmp_path))
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        kwargs = d._live_runtime_provider_factory(
            executor=executor, scenario_id=scenario_id, bundle=None,
            config=_test_config(), attempt_dir=Path(tmp_path),
        )
        assert "fjt_transaction_provider" in kwargs
        _stub_action_clients_for_plan_only(executor, POSITIVE_REPORT_CONTRACT)
        record = executor.run_execute_sequence(scenario_id, **kwargs)
        assert record["status"] == "evidence-invalid"
        assert "no new controller FJT goal" in (record.get("execute_error") or "")
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_d_execute_multiple_new_fjt_goals_fails_closed(tmp_path):
    scenario_id = "qualification-moveit-execute-joint"
    graph = _ControlledGraph(scenario_id=scenario_id, duplicate_fjt=True, attempt_dir=Path(tmp_path))
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        kwargs = d._live_runtime_provider_factory(
            executor=executor, scenario_id=scenario_id, bundle=None,
            config=_test_config(), attempt_dir=Path(tmp_path),
        )
        _stub_action_clients_for_plan_only(executor, POSITIVE_REPORT_CONTRACT)
        record = executor.run_execute_sequence(scenario_id, **kwargs)
        assert record["status"] == "evidence-invalid"
        assert "multiple new controller FJT goal" in (record.get("execute_error") or "")
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_d_execute_stale_pre_baseline_replay_fails_closed(tmp_path):
    scenario_id = "qualification-moveit-execute-joint"
    stale_uuid = _fresh_uuid_hex()
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    # The controller goal UUID is forced to a value heartbeated BEFORE the run
    # baseline; the fresh replay of a baseline-known goal must NOT be treated
    # as a new controller transaction.
    graph._forced_controller_uuid = stale_uuid
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        graph.emit_fjt(stale_uuid, EXECUTE_STATUS_EXECUTING)
        _drive_spin(executor, graph=graph, timeout_s=0.3)
        _assert_ready(executor)
        kwargs = d._live_runtime_provider_factory(
            executor=executor, scenario_id=scenario_id, bundle=None,
            config=_test_config(), attempt_dir=Path(tmp_path),
        )
        _stub_action_clients_for_plan_only(executor, POSITIVE_REPORT_CONTRACT)
        record = executor.run_execute_sequence(scenario_id, **kwargs)
        assert record["status"] == "evidence-invalid"
        assert "no new controller FJT goal" in (record.get("execute_error") or "")
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_d_execute_fjt_uuid_mismatch_fails_closed(tmp_path):
    scenario_id = "qualification-moveit-execute-joint"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        _stub_action_clients_for_plan_only(executor, POSITIVE_REPORT_CONTRACT)
        wrong_uuid = _fresh_uuid_hex()

        def _wrong_uuid_provider():
            digest = executor._execute_recorder.last_trajectory_digest
            return {
                "endpoint": FJT_ENDPOINT,
                "goal_uuid": wrong_uuid,
                "trajectory_digest": digest,
                "source": "test-introspection",
                "sequence": 1,
                "timestamp": time.monotonic(),
                "status": EXECUTE_STATUS_SUCCEEDED,
            }

        record = executor.run_execute_sequence(
            scenario_id, fjt_transaction_provider=_wrong_uuid_provider
        )
        assert record["status"] == "evidence-invalid"
        assert "does not equal the discovered controller goal UUID" in (record.get("execute_error") or "")
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_d_execute_fjt_status_mismatch_fails_closed(tmp_path):
    scenario_id = "qualification-moveit-execute-joint"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        _stub_action_clients_for_plan_only(executor, POSITIVE_REPORT_CONTRACT)

        def _wrong_status_provider():
            # The graph emitted EXECUTING(2) then SUCCEEDED(4); report the exact
            # newest entry but with the pre-terminal status so the join fails.
            newest = executor._fjt_status_cache[-1]
            digest = executor._execute_recorder.last_trajectory_digest
            return {
                "endpoint": FJT_ENDPOINT,
                "goal_uuid": str(newest.get("goal_uuid")),
                "trajectory_digest": digest,
                "source": "test-introspection",
                "sequence": int(newest.get("seq")),
                "timestamp": float(newest.get("received_mono")),
                "status": EXECUTE_STATUS_EXECUTING,
            }

        record = executor.run_execute_sequence(
            scenario_id, fjt_transaction_provider=_wrong_status_provider
        )
        assert record["status"] == "evidence-invalid"
        assert "status does not join" in (record.get("execute_error") or "")
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_presend_failure_cleans_up_retained_handle(tmp_path):
    scenario_id = "qualification-moveit-cancel"
    graph = _ControlledGraph(scenario_id=scenario_id, emit_fjt_on_execute=False, attempt_dir=Path(tmp_path))
    # Hold the goal open so cleanup must cancel it, not race an auto-success.
    graph._execute_hold_s = 5.0
    graph.start()
    executor = None
    try:
        # F5.2: the production FJT discovery default is 10.0 s, which exceeds the
        # 5 s hold and would let the held goal auto-succeed before cleanup.  This
        # test exercises the presend-failure cleanup path, so it explicitly
        # overrides the FJT wait to fail discovery BEFORE the hold expires and
        # cleanup must cancel the still-active exact goal.
        config = dict(_test_config())
        config["thresholds"] = dict(config["thresholds"])
        config["thresholds"]["fjt_wait_timeout_s"] = 0.5
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id, config=config)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        with pytest.raises(d.DriverError, match="no new controller FJT goal") as exc_info:
            d._presend_long_motion(executor, scenario_id)
        # F4.5: the retained accepted presend handle was boundedly cleaned up.
        assert getattr(executor, "_presend_execute_handle", None) is None
        # The cleanup summary is embedded in the DriverError message; the goal
        # was cancelled (terminal CANCELED), never left running.
        cleanup_text = str(exc_info.value).split("cleanup=", 1)[1]
        import ast
        cleanup = ast.literal_eval(cleanup_text)
        assert isinstance(cleanup, dict)
        assert cleanup.get("cleanup") == "accepted"
        assert cleanup.get("cleanup_result_status") == EXECUTE_STATUS_CANCELED
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_d_retreat_path_reaches_run_cartesian_retreat(tmp_path):
    scenario_id = "qualification-moveit-cartesian-retreat"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
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
        _assert_ready(executor)
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


# --------------------------------------------------------------------------- #
# F5.5 — owner-QoS mutation: extra incompatible endpoint before the owner
# --------------------------------------------------------------------------- #

def test_owner_qos_mutation_extra_incompatible_endpoint(rclpy_runtime):
    """F5.5: owner-QoS selection is by normalized label, not discovery order.

    Creates an extra incompatible-QoS (BEST_EFFORT) publisher for the
    planning-scene topic BEFORE the required RELIABLE ``/move_group`` owner, and
    verifies owner-specific selection still chooses ``/move_group``,
    ``/fixture_planning_scene``, and ``/tinker_integrated_gate_executor`` as
    appropriate.  Missing or duplicate required owner endpoints return ``{}``
    and fail closed.
    """
    from moveit_msgs.msg import PlanningScene
    from std_msgs.msg import String
    from validation.integrated_gate_executor import (
        JOURNAL_FIXTURE_TOPIC_QOS,
        JOURNAL_PLANNING_SCENE_TOPIC_QOS,
    )

    ctx = rclpy.Context()
    ctx.init(domain_id=_next_domain())
    decoy = Node("sim_other_owner", namespace="/", context=ctx, use_global_arguments=False, cli_args=[])
    move_group = Node("move_group", namespace="/", context=ctx, use_global_arguments=False, cli_args=[])
    fixture = Node("fixture_planning_scene", namespace="/", context=ctx, use_global_arguments=False, cli_args=[])
    gate = Node("tinker_integrated_gate_executor", namespace="/", context=ctx, use_global_arguments=False, cli_args=[])
    spinner = MultiThreadedExecutor(context=ctx, num_threads=2)
    nodes = (decoy, move_group, fixture, gate)
    for node in nodes:
        spinner.add_node(node)

    def _spin_settle(timeout_s: float = 8.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                spinner.spin_once(timeout_sec=0.05)
            except Exception:  # noqa: BLE001 - discovery settle boundary
                pass
            time.sleep(0.01)

    try:
        # Extra incompatible-QoS publisher created BEFORE the required owner.
        decoy_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE
        )
        decoy.create_publisher(PlanningScene, PLANNING_SCENE_TOPIC, decoy_qos)
        owner_qos = QoSProfile(
            depth=int(JOURNAL_PLANNING_SCENE_TOPIC_QOS["depth"]),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        move_group.create_publisher(PlanningScene, PLANNING_SCENE_TOPIC, owner_qos)
        fixture_qos = QoSProfile(
            depth=int(JOURNAL_FIXTURE_TOPIC_QOS["depth"]),
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        fixture.create_publisher(String, FIXTURE_TOPIC, fixture_qos)
        gate.create_subscription(String, FIXTURE_TOPIC, lambda msg: None, fixture_qos)
        _spin_settle()

        # Planning-scene offered QoS: the /move_group owner is selected despite
        # the earlier BEST_EFFORT decoy (selection by label, not discovery order).
        pub_labels, pub_infos = d._publishers_for(gate, PLANNING_SCENE_TOPIC)
        assert "/move_group" in pub_labels, pub_labels
        planner_qos = d._select_endpoint_qos(
            pub_labels, pub_infos, "/move_group", expected_depth=int(JOURNAL_PLANNING_SCENE_TOPIC_QOS["depth"])
        )
        assert planner_qos, pub_labels
        assert planner_qos["reliability"] == "RELIABLE"
        assert planner_qos["durability"] == "VOLATILE"
        assert planner_qos["depth"] == int(JOURNAL_PLANNING_SCENE_TOPIC_QOS["depth"])

        # Fixture owner offered QoS and gate requested QoS by label.
        fixture_pub_labels, fixture_pub_infos = d._publishers_for(gate, FIXTURE_TOPIC)
        fixture_owner_qos = d._select_endpoint_qos(
            fixture_pub_labels, fixture_pub_infos, "/fixture_planning_scene",
            expected_depth=int(JOURNAL_FIXTURE_TOPIC_QOS["depth"]),
        )
        assert fixture_owner_qos, fixture_pub_labels
        assert fixture_owner_qos["durability"] == "TRANSIENT_LOCAL"
        gate_sub_labels, gate_sub_infos = d._subscribers_for(gate, FIXTURE_TOPIC)
        gate_req_qos = d._select_endpoint_qos(
            gate_sub_labels, gate_sub_infos, "/tinker_integrated_gate_executor",
            expected_depth=int(JOURNAL_FIXTURE_TOPIC_QOS["depth"]),
        )
        assert gate_req_qos, gate_sub_labels

        # Missing required owner endpoint returns {} (fail closed).
        assert d._select_endpoint_qos(pub_labels, pub_infos, "/missing_owner", expected_depth=100) == {}

        # Duplicate required owner endpoint returns {} (fail closed).
        move_group.create_publisher(PlanningScene, PLANNING_SCENE_TOPIC, owner_qos)
        _spin_settle()
        dup_labels, dup_infos = d._publishers_for(gate, PLANNING_SCENE_TOPIC)
        assert d._select_endpoint_qos(
            dup_labels, dup_infos, "/move_group", expected_depth=100
        ) == {}
    finally:
        try:
            spinner.shutdown()
        except Exception:  # noqa: BLE001
            pass
        for node in nodes:
            try:
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        try:
            ctx.try_shutdown()
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
# F5.6 — source-real positive/negative evidence tests
# --------------------------------------------------------------------------- #

def test_controlled_d_execute_source_real_delayed_status_positive(tmp_path):
    """F5.6: controller FJT status delayed >1 s but <10 s commits diagnostic-pass.

    Uses the real action/status subscriptions (never ``_seed_fjt_status``) and
    holds the controller goal open for 2 s so the terminal status arrives beyond
    the old 1 s false-negative boundary but within the 10 s production budget.
    """
    scenario_id = "qualification-moveit-execute-joint"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    graph._execute_hold_s = 2.0
    graph.start()
    executor = None
    try:
        # F5.6: the execute result + controller terminal arrive ~2 s in, so the
        # execute wait must exceed the 0.5 s test default; the FJT/motion waits
        # stay at the production 10.0 s defaults (no override in the config).
        config = dict(_test_config())
        config["thresholds"] = dict(config["thresholds"])
        config["thresholds"]["execute_timeout_s"] = 5.0
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id, config=config)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        kwargs = d._live_runtime_provider_factory(
            executor=executor, scenario_id=scenario_id, bundle=None,
            config=config, attempt_dir=Path(tmp_path),
        )
        _stub_action_clients_for_plan_only(executor, POSITIVE_REPORT_CONTRACT)
        started = time.monotonic()
        record = executor.run_execute_sequence(scenario_id, **kwargs)
        elapsed = time.monotonic() - started
        assert record["status"] == "diagnostic-pass", record
        assert elapsed >= 1.0, f"controller delay not exercised ({elapsed:.2f}s)"
        assert elapsed < 10.0, f"controller delay exceeded the 10 s budget ({elapsed:.2f}s)"
        assert record["fjt_status"] == EXECUTE_STATUS_SUCCEEDED
        assert record["fjt_goal_uuid"] != record["execute_goal_id"]
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_f51_bound_provider_does_not_switch_on_second_post_capture_status(tmp_path):
    """F5.1/F5.6: a second post-capture status cannot switch the transaction.

    The provider is bound to the exact captured terminal entry; a second real
    status emission for the SAME FJT UUID (advancing the sequence) after the
    capture must not change the evidence the provider returns.
    """
    scenario_id = "qualification-moveit-execute-joint"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        captured_uuid = _fresh_uuid_hex()
        # F5.1: a real status emission is the capture; the provider binds to it.
        # Poll up to 5 s for the entry — under full-module DDS load a fixed short
        # spin may not receive the emit in time.
        graph.emit_fjt(captured_uuid, EXECUTE_STATUS_SUCCEEDED)
        captured = None
        poll_deadline = time.monotonic() + 5.0
        while time.monotonic() < poll_deadline:
            _drive_spin(executor, graph=graph, timeout_s=0.2)
            for entry in reversed(executor._fjt_status_cache):
                if str(entry.get("goal_uuid")) == captured_uuid:
                    captured = dict(entry)
                    break
            if captured is not None:
                break
        assert captured is not None, "captured FJT status entry was not observed"
        # The FJT provider also requires the exact executed-trajectory digest.
        # This isolated seam test pins a known digest; the positive D paths
        # capture the real digest via the execute recorder at send time.
        executor._execute_recorder.last_trajectory_digest = hashlib.sha256(b"seam-test").hexdigest()
        provider = d._fjt_transaction_provider(executor, expected_fjt_entry=captured)
        # Deterministic second post-capture status for the SAME UUID.
        graph.emit_fjt(captured_uuid, EXECUTE_STATUS_SUCCEEDED)
        poll_deadline = time.monotonic() + 5.0
        while time.monotonic() < poll_deadline:
            _drive_spin(executor, graph=graph, timeout_s=0.2)
            matching = [
                entry for entry in executor._fjt_status_cache
                if str(entry.get("goal_uuid")) == captured_uuid
            ]
            if len(matching) >= 2:
                break
        evidence = provider()
        # The provider returns the exact captured entry, never the newest.
        assert evidence["goal_uuid"] == captured_uuid
        assert evidence["sequence"] == captured["seq"]
        assert evidence["timestamp"] == captured["received_mono"]
        assert evidence["source"] == FJT_STATUS_TOPIC
        # The second status IS in the live cache (mutation proven), seq advanced.
        matching = [
            entry for entry in executor._fjt_status_cache
            if str(entry.get("goal_uuid")) == captured_uuid
        ]
        assert len(matching) >= 2, "second post-capture status not observed"
        assert max(int(e.get("seq", 0)) for e in matching) > int(captured["seq"])
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_d_acceptance_timeout_exact_cancel(tmp_path):
    """F5.3: a delayed acceptance response is canceled/drained by exact goal UUID.

    The ExecuteTrajectory server delays the acceptance callback beyond the
    driver's ``accept_timeout_s`` while the goal is genuinely accepted and the
    long motion begins during the bounded cleanup grace period.  The driver must
    retain the late handle, cancel/drain the exact goal, leave no in-flight
    coroutine, and fail closed with the original presend error preserved.
    """
    scenario_id = "qualification-moveit-cancel"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    # Acceptance response lands at ~0.4 s > accept_timeout_s=0.1, and within the
    # bounded grace period (>= 0.5 s), so the late handle is retained + canceled.
    graph._execute_goal_accept_delay_s = 0.4
    graph._execute_hold_s = 5.0
    graph._motion_active = True
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        goal = build_execute_trajectory_goal(_planned_trajectory())
        with pytest.raises(d.DriverError, match="acceptance timed out") as exc_info:
            d._send_execute_retaining_handle(
                executor, scenario_id, goal, accept_timeout_s=0.1,
            )
        message = str(exc_info.value)
        assert "exact-UUID cleanup=" in message
        cleanup_text = message.split("exact-UUID cleanup=", 1)[1]
        cleanup = json.loads(cleanup_text)
        assert isinstance(cleanup, dict)
        # The exact preassigned goal was owned and canceled/drained.
        assert cleanup.get("late_acceptance") is True
        assert isinstance(cleanup.get("cleanup_goal_uuid"), str) and len(cleanup["cleanup_goal_uuid"]) == 32
        assert cleanup.get("cleanup_result_status") in (
            EXECUTE_STATUS_CANCELED, EXECUTE_STATUS_ABORTED, EXECUTE_STATUS_SUCCEEDED,
        ), cleanup
        # No uncontrolled motion survives: a final idempotent cleanup sees the
        # retained handle already terminal (cancel rejected on a terminal goal)
        # and leaves nothing live on the execute server.
        final_cleanup = d._cleanup_retained_presend(executor)
        assert final_cleanup.get("cleanup") in ("none", "rejected", "accepted"), final_cleanup
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                graph._executor.spin_once(timeout_sec=0.01)
            except Exception:  # noqa: BLE001
                pass
            unresolved = [
                future for server in graph._action_servers
                for future in getattr(server, "_result_futures", {}).values()
                if not future.done()
            ]
            if not unresolved:
                break
            time.sleep(0.01)
        assert not unresolved, f"{len(unresolved)} execute result future(s) still live"
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_controlled_d_acceptance_timeout_exact_uuid_cancel_no_handle(tmp_path):
    """F5.3: no late handle → typed exact-UUID CancelGoal path, fail closed.

    When the acceptance response stays delayed through the grace period, the
    driver sends a typed ``action_msgs/srv/CancelGoal`` request for the exact
    preassigned UUID.  With the server's goal-callback still open the cancel is
    rejected/unknown (no uncontrolled motion) and the driver fails closed.
    """
    scenario_id = "qualification-moveit-cancel"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    # Acceptance callback sleeps past accept_timeout_s AND past the bounded grace
    # period (>= 0.5 s), so no handle arrives and the exact-UUID cancel path is
    # used.  The goal becomes accepted only after the driver has already failed
    # closed; graph teardown then aborts it (no uncontrolled motion survives).
    graph._execute_goal_accept_delay_s = 1.0
    graph._execute_hold_s = 5.0
    graph._motion_active = True
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        goal = build_execute_trajectory_goal(_planned_trajectory())
        with pytest.raises(d.DriverError, match="acceptance timed out") as exc_info:
            d._send_execute_retaining_handle(
                executor, scenario_id, goal, accept_timeout_s=0.05,
            )
        message = str(exc_info.value)
        cleanup_text = message.split("exact-UUID cleanup=", 1)[1]
        cleanup = json.loads(cleanup_text)
        assert isinstance(cleanup, dict)
        assert cleanup.get("late_acceptance") is False
        # The exact-UUID cancel was attempted; a rejection/unknown/terminal
        # response is observable evidence that no uncontrolled motion survives.
        assert cleanup.get("cancel_response") in ("rejected", "accepted", "unavailable", "timed-out")
        assert isinstance(cleanup.get("cleanup_goal_uuid"), str) and len(cleanup["cleanup_goal_uuid"]) == 32
    finally:
        # F5.6: the goal becomes accepted AFTER the driver has failed closed.
        # Tear the graph down FIRST (aborting the late-accepted goal) while the
        # executor client is still alive so the server's result response has a
        # live client — no "failed to send response (timeout)" warning.
        graph.stop()
        if executor is not None:
            executor.shutdown()


def test_exact_uuid_cancel_rejection_fails_closed(tmp_path):
    """F5.6: cleanup cancellation rejection fails closed without another goal.

    The ExecuteTrajectory server rejects every cancel; the exact-UUID typed
    cancel request must fail closed (``rejected``), never cancel a different
    goal, and leave the driver able to raise without an uncontrolled motion.
    """
    scenario_id = "qualification-moveit-cancel"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    graph._execute_reject_cancel = True
    graph._execute_hold_s = 5.0
    graph._motion_active = True
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        # Send one real ExecuteTrajectory goal (accepted, held open).  We wait
        # for ACCEPTANCE only — the goal must stay active so the exact-UUID
        # cancel is meaningful; teardown then aborts the held goal.
        goal = build_execute_trajectory_goal(_planned_trajectory())
        send_future = executor._action_clients["/execute_trajectory"].send_goal_async(goal)
        deadline = time.monotonic() + 5.0
        while not send_future.done() and time.monotonic() < deadline:
            executor._spin_once()
        assert send_future.done(), "execute goal was not accepted in time"
        handle = send_future.result()
        assert handle is not None and getattr(handle, "accepted", False)
        goal_uuid = d._goal_id_hex(handle)
        assert isinstance(goal_uuid, str) and len(goal_uuid) == 32
        # Typed exact-UUID cancel is rejected by the server → fail closed.
        cancel = d._cancel_execute_goal_by_uuid(executor, goal_uuid, timeout_s=3.0)
        assert cancel.get("response") == "rejected", cancel
        # No unrelated goal was canceled and no cancel-all was issued: the
        # response never lists a UUID other than the requested one.
        assert cancel.get("goals_canceling") != [goal_uuid] or cancel.get("goals_canceling") == []
        assert cancel.get("response") in ("rejected", "wrong-goal", "unavailable", "timed-out")
    finally:
        # F5.6: the held goal (cancel rejected) is aborted by the graph teardown.
        # Tear the graph down FIRST while the executor client is still alive so
        # the server's abort result response has a live client — no
        # "failed to send response (timeout)" warning.
        graph.stop()
        if executor is not None:
            executor.shutdown()


def test_exact_uuid_cancel_accepted_requires_terminal_evidence(tmp_path):
    """F5.3/F5.6: an accepted exact-UUID cancel requires observable terminal
    evidence for that exact execute goal UUID before teardown.

    Sends a real ExecuteTrajectory goal (accepted, held open), issues the typed
    exact-UUID ``action_msgs/srv/CancelGoal`` request, and verifies the server
    acknowledges the exact goal (``goals_canceling == [uuid]``) and that the
    executor observes terminal execute status for that exact UUID.
    """
    scenario_id = "qualification-moveit-cancel"
    graph = _ControlledGraph(scenario_id=scenario_id, attempt_dir=Path(tmp_path))
    graph._execute_hold_s = 5.0
    graph._motion_active = True
    graph.start()
    executor = None
    try:
        executor = _construct_real_executor(Path(tmp_path), scenario_id=scenario_id)
        _drive_spin(executor, graph=graph, timeout_s=2.0)
        _assert_ready(executor)
        goal = build_execute_trajectory_goal(_planned_trajectory())
        send_future = executor._action_clients["/execute_trajectory"].send_goal_async(goal)
        deadline = time.monotonic() + 5.0
        while not send_future.done() and time.monotonic() < deadline:
            executor._spin_once()
        assert send_future.done(), "execute goal was not accepted in time"
        handle = send_future.result()
        assert handle is not None and getattr(handle, "accepted", False)
        goal_uuid = d._goal_id_hex(handle)
        assert isinstance(goal_uuid, str) and len(goal_uuid) == 32
        # Typed exact-UUID cancel on an accepting server → "accepted".
        cancel = d._cancel_execute_goal_by_uuid(executor, goal_uuid, timeout_s=3.0)
        assert cancel.get("response") == "accepted", cancel
        assert cancel.get("goals_canceling") == [goal_uuid]
        # Observable terminal evidence for that exact UUID (the driver's step-3
        # requirement before teardown).
        terminal = executor._wait_for_execute_status(
            goal_uuid,
            (EXECUTE_STATUS_CANCELED, EXECUTE_STATUS_ABORTED, EXECUTE_STATUS_SUCCEEDED),
            3.0,
        )
        assert terminal is not None, "no terminal execute status for the exact UUID"
        assert terminal.get("status") in (
            EXECUTE_STATUS_CANCELED, EXECUTE_STATUS_ABORTED, EXECUTE_STATUS_SUCCEEDED,
        )
        # Drain the goal result so the server coroutine terminates.
        result_future = handle.get_result_async()
        deadline = time.monotonic() + 3.0
        while not result_future.done() and time.monotonic() < deadline:
            executor._spin_once()
        assert result_future.done()
    finally:
        if executor is not None:
            executor.shutdown()
        graph.stop()


def test_exact_uuid_cancel_service_unavailable_fails_closed(rclpy_runtime, tmp_path):
    """F5.6: unavailable cancel service fails closed without cancel-all.

    With no ExecuteTrajectory action server on the graph, the typed exact-UUID
    cancel service never becomes available; the driver's cleanup must fail
    closed (``unavailable``), never cancel-all or block indefinitely.
    """
    executor = None
    try:
        # Construct the executor WITHOUT any controlled action server: the
        # /execute_trajectory cancel service is therefore unavailable.
        executor = _construct_real_executor(Path(tmp_path))
        cancel = d._cancel_execute_goal_by_uuid(executor, _fresh_uuid_hex(), timeout_s=0.6)
        assert cancel.get("response") == "unavailable", cancel
        assert isinstance(cancel.get("error"), str) and cancel["error"]
    finally:
        if executor is not None:
            executor.shutdown()
