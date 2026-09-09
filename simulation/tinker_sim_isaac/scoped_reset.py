"""In-place TIME/STATE reset for the native provider's idle qualification path.

The standard provider owns the service. This adapter uses the existing safety
supervisor/controller/mux lifecycle; it never stops the PhysX timeline, deletes
prims, replaces live views, or publishes a second clock.
"""
from __future__ import annotations

import asyncio
import time


class ResetFailure(RuntimeError):
    """A scoped reset could not establish or complete its idle boundary."""


class IdleReset:
    def __init__(self, gateway, timeline, transport, baseline, *, settle=None):
        self.gateway = gateway
        self.timeline = timeline
        self.transport = transport
        self.baseline = baseline
        self.settle = settle or self._settle
        self.busy = False
        self.mutations_inflight = 0

    async def _settle(self):
        # Exceed the task's one-second clock-pause watchdog before rewinding.
        # This also gives consumers a visible invalidation boundary for STATE.
        await asyncio.sleep(1.2)

    async def run(self, scope):
        if self.busy:
            raise ResetFailure("scoped reset is already in progress")
        if self.mutations_inflight:
            raise ResetFailure("a provider mutation is in progress")
        if scope not in (1, 2, 3):
            raise ResetFailure("scoped reset supports only TIME, STATE, or TIME|STATE")
        self.busy = True
        asserted = False
        try:
            await self.transport.idle()
            generation = self.transport.action_generation
            if scope & 2:
                self.baseline.prepare()
            self.transport.assert_stop(True)
            asserted = True
            await self.transport.stopped(generation)
            self.timeline.pause()
            self.gateway._scoped_reset_publishing = True
            await self.settle()
            if scope & 2:
                self.baseline.restore()
            else:
                self.baseline.clear_commands()
            if scope & 1:
                self.gateway.backend._ros_reset_origin_s = self.gateway.backend.simulation_time
            self.gateway.backend.render_frame()
            self.gateway.publish_reset_boundary()
            # The measured reset state reaches the stopped mux before its next
            # generation. Controller activation needs physics ticks to finish.
            await asyncio.sleep(0.1)
            self.transport.assert_stop(False)
            self.timeline.play()
            await self.transport.released()
        except Exception as error:
            if asserted:
                self.transport.assert_stop(True)
                self.timeline.pause()
                raise ResetFailure(
                    f"{error}; simulation paused; reset stop remains asserted"
                ) from error
            raise
        finally:
            self.gateway._scoped_reset_publishing = False
            self.busy = False


def bind_provider(provider_type, reset):
    """Bind before extension startup captures the standard service callbacks."""
    from simulation_interfaces.msg import Result, SimulatorFeatures

    original_reset = provider_type._handle_reset_simulation
    original_features = provider_type._handle_get_simulator_features
    original_spawn = provider_type._spawn_single_entity_core

    async def handle_reset(self, request, response):
        if request.scope not in (1, 2, 3):
            if reset.busy:
                response.result = Result(
                    result=Result.RESULT_OPERATION_FAILED,
                    error_message="scoped reset is already in progress",
                )
                return response
            reset.mutations_inflight += 1
            try:
                return await original_reset(self, request, response)
            finally:
                reset.mutations_inflight -= 1
        try:
            await reset.run(request.scope)
            response.result = Result(result=Result.RESULT_OK, error_message="")
        except Exception as error:
            response.result = Result(
                result=Result.RESULT_OPERATION_FAILED, error_message=str(error)
            )
        return response

    async def features(self, request, response):
        response = await original_features(self, request, response)
        for name in ("SIMULATION_RESET_TIME", "SIMULATION_RESET_STATE"):
            value = getattr(SimulatorFeatures, name, None)
            if value is not None and value not in response.features.features:
                response.features.features.append(value)
        return response

    async def spawn(self, *args, **kwargs):
        if reset.busy:
            return Result.RESULT_OPERATION_FAILED, "scoped reset is in progress", ""
        reset.mutations_inflight += 1
        try:
            result = await original_spawn(self, *args, **kwargs)
            if result[0] == Result.RESULT_OK:
                reset.baseline.register_spawn()
            return result
        finally:
            reset.mutations_inflight -= 1

    provider_type._handle_reset_simulation = handle_reset
    provider_type._handle_get_simulator_features = features
    provider_type._spawn_single_entity_core = spawn
    # A mutating request must not cross the paused state/clock publication
    # boundary. Ordinary requests retain their standard provider behavior.
    for name in (
        "_handle_set_entity_state", "_handle_delete_entity",
        "_handle_set_simulation_state", "_handle_load_world",
        "_handle_unload_world", "_handle_step_simulation",
    ):
        original = getattr(provider_type, name, None)
        if original is None:
            continue

        async def guarded(self, request, response, _original=original):
            if reset.busy:
                response.result = Result(
                    result=Result.RESULT_OPERATION_FAILED,
                    error_message="scoped reset is in progress",
                )
                return response
            reset.mutations_inflight += 1
            try:
                return await _original(self, request, response)
            finally:
                reset.mutations_inflight -= 1
        setattr(provider_type, name, guarded)


class PhysicsBaseline:
    """Initial bound robot state and initial free-body state, retained in place."""

    def __init__(self, backend, *, read_bodies=None):
        self.backend = backend
        self._read_bodies = read_bodies or self._native_bodies
        data = backend._robot.data
        self.root_pose = backend._torch_value(data.root_link_pose_w).clone()
        self.root_velocity = backend._torch_value(data.root_com_vel_w).clone()
        self.joint_position = backend._torch_value(data.joint_pos).clone()
        self.joint_velocity = backend._torch_value(data.joint_vel).clone()
        self.bodies = self._read_bodies()
        self.live_bodies = set(self.bodies)
        self.gripper_effort = getattr(backend, "gripper_effort_limit", None)

    def _native_bodies(self):
        import omni.usd
        from pxr import UsdPhysics
        from isaaclab_physx.physics import PhysxManager

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise ResetFailure("stage unavailable while reading initial state")
        bodies = {}
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path == "/World/Tinker" or path.startswith("/World/Tinker/"):
                continue
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                raise ResetFailure(f"initial state of additional articulation is unsupported: {path}")
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            view = self.backend._park_views.get(path)
            if view is None:
                view = PhysxManager.get_physics_sim_view().create_rigid_body_view(path)
                if view is None or int(view.count) != 1:
                    raise ResetFailure(f"initial rigid-body state unavailable: {path}")
                self.backend._park_views[path] = view
            pose = tuple(float(v) for v in view.get_transforms().numpy()[0])
            kinematic = UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get()
            velocity = None if kinematic else tuple(float(v) for v in view.get_velocities().numpy()[0])
            bodies[path] = (pose, velocity)
        return bodies

    def register_spawn(self):
        # Existing bodies keep their original baseline even if they moved
        # before another entity was spawned.
        for path, state in self._read_bodies().items():
            self.bodies.setdefault(path, state)

    def prepare(self):
        current = self._read_bodies()
        unknown = set(current) - set(self.bodies)
        if unknown:
            raise ResetFailure(f"initial state was not registered for: {sorted(unknown)}")
        self.live_bodies = set(current)

    def restore(self):
        backend = self.backend
        if not backend._safety_stopped:
            raise ResetFailure("backend hold is not active")
        robot = backend._robot
        # Articulation.reset clears actuator delays, external wrenches and
        # controller memory; the direct writers preserve the live PhysX view.
        robot.reset()
        robot.write_root_pose_to_sim_index(root_pose=self.root_pose.clone())
        robot.write_root_velocity_to_sim_index(root_velocity=self.root_velocity.clone())
        robot.write_joint_state_to_sim_index(
            position=self.joint_position.clone(), velocity=self.joint_velocity.clone()
        )
        backend.discard_command_snapshot_staging()
        backend._command_snapshot_id = None
        backend._drive_command_target = None
        backend._position_targets = self.joint_position.clone()
        backend._safety_snapshot = self.joint_position.clone()
        backend._velocity_targets.zero_()
        backend._effort_targets.zero_()
        backend._applied_wheel_velocities = {
            index: 0.0 for index in getattr(backend, "_wheel_indices", ())
        }
        if backend._base_hold_pose is not None:
            backend._base_hold_pose = self.root_pose.clone()
            backend._base_hold_vel = self.root_velocity.clone()
        if self.gripper_effort is not None:
            # The read-only property is Nm; the existing setter accepts
            # commanded fingertip force in N and updates both the actuator
            # model and the live PhysX drive. Invert its configured scale.
            requested_n = (
                self.gripper_effort * backend._gripper_effort_full_scale_n
                / backend._default_gripper_effort_limit
            )
            backend._gripper_effort_limit_written = False
            backend._set_gripper_effort_limit(requested_n)
            if not backend._gripper_effort_limit_written:
                raise ResetFailure("Initial gripper effort limit did not reach PhysX")
        backend._contact_pairs_by_key.clear()
        backend._target_write_gate.force_next()
        robot.set_joint_position_target(backend._position_targets)
        robot.set_joint_velocity_target(backend._velocity_targets)
        robot.set_joint_effort_target(backend._effort_targets)
        robot.write_data_to_sim()
        for path in sorted(self.live_bodies):
            pose, velocity = self.bodies[path]
            if not backend.set_entity_pose_physics(
                path, position=pose[:3], quaternion_xyzw=pose[3:],
                linear_velocity=None if velocity is None else velocity[:3],
                angular_velocity=None if velocity is None else velocity[3:],
            ):
                raise ResetFailure(f"initial state write failed: {path}")

    def clear_commands(self):
        # TIME-only reset keeps current physical state, but no old gripper
        # slew target or partially staged snapshot may survive its stop epoch.
        backend = self.backend
        if not backend._safety_stopped:
            raise ResetFailure("backend hold is not active")
        backend.discard_command_snapshot_staging()
        backend._command_snapshot_id = None
        backend._drive_command_target = None
        backend._robot.reset()
        backend._velocity_targets.zero_()
        backend._effort_targets.zero_()
        backend._target_write_gate.force_next()



class StationaryWindow:
    """Detect a stable commanded posture despite contact-solver velocity noise."""

    REQUIRED_JOINTS = tuple(f"joint{i}" for i in range(1, 8)) + ("drive_joint",)

    def __init__(self):
        from collections import deque
        self.samples = deque()
        self.reason = "collecting a stationary pose window"

    def observe(self, now, names, positions, velocities, root):
        import math

        required = set(self.REQUIRED_JOINTS)
        if not required.issubset(names) or len(positions) != len(names) or len(velocities) != len(names):
            self.reason = "required measured joints are missing"
            self.samples.clear()
            return False
        root_position = tuple(float(v) for v in root["position"])
        quaternion = tuple(float(v) for v in root["quaternion_wxyz"])
        values = (
            list(positions) + list(velocities) + list(root_position) + list(quaternion)
            + list(root["linear_velocity_world"]) + list(root["angular_velocity_world"])
        )
        if not all(math.isfinite(v) for v in values):
            self.reason = "nonfinite measured joint or root state"
            self.samples.clear()
            return False
        norm = math.sqrt(sum(v * v for v in quaternion))
        if norm < 1e-12:
            self.reason = "invalid root quaternion"
            self.samples.clear()
            return False
        quaternion = tuple(v / norm for v in quaternion)
        measured = dict(zip(names, positions))
        monitored = self.REQUIRED_JOINTS + tuple(
            name for name in ("pan_joint", "tilt_joint") if name in measured
        )
        pose = tuple(float(measured[name]) for name in monitored) + root_position
        self.samples.append((float(now), pose, quaternion))
        while len(self.samples) > 1 and self.samples[1][0] <= now - .25:
            self.samples.popleft()
        if now - self.samples[0][0] < .25:
            self.reason = "collecting a 0.25 second stationary pose window"
            return False
        # Passive caster/wheel rotation and mimic-joint velocity noise do not
        # establish arm motion. The root pose still detects actual base travel.
        for index in range(len(pose)):
            span = max(s[1][index] for s in self.samples) - min(s[1][index] for s in self.samples)
            if span > .0005:
                name = monitored[index] if index < len(monitored) else "root position"
                self.reason = f"{name} measured span {span:.6g} exceeds 0.0005"
                return False
        reference = self.samples[0][2]
        for _, _, current in self.samples:
            distance = min(
                math.sqrt(sum((a - b) ** 2 for a, b in zip(reference, current))),
                math.sqrt(sum((a + b) ** 2 for a, b in zip(reference, current))),
            )
            if distance > .0005:
                self.reason = f"root orientation distance {distance:.6g} exceeds 0.0005"
                return False
        self.reason = "stationary"
        return True


class ResetTransport:
    """ROS observation and the provider's one optional supervisor stop input."""

    def __init__(self, gateway, *, timeout_s=12.0):
        import threading
        from action_msgs.msg import GoalStatusArray
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool

        self.gateway = gateway
        self.timeout_s = timeout_s
        self.action_generation = 0
        self.active_actions = {}
        self.effective_stop = None
        self.reset_ready = False
        self._reset_asserted_at = 0.0
        self._reset_ready_at = 0.0
        self.stationary_window = StationaryWindow()
        self._Bool = Bool
        self.node = Node("tinker_scoped_reset")
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.node.create_publisher(Bool, "/sim/safety/reset", qos)
        self.node.create_subscription(
            Bool, "/sim/hardware/safety_stop", self._stop_sample, qos
        )
        self.node.create_subscription(
            Bool, "/sim/safety/reset_ready", self._ready_sample, qos
        )
        self.action_topics = (
            "/xarm7_traj_controller/follow_joint_trajectory/_action/status",
            "/xarm_gripper/gripper_action/_action/status",
        )
        self.active_actions = dict.fromkeys(self.action_topics, False)
        self._subscriptions = [
            self.node.create_subscription(
                GoalStatusArray, topic,
                lambda message, key=topic: self._action_sample(key, message), qos,
            )
            for topic in self.action_topics
        ]
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.assert_stop(False)

    def _stop_sample(self, message):
        self.effective_stop = bool(message.data)

    def _ready_sample(self, message):
        self.reset_ready = bool(message.data)
        self._reset_ready_at = time.monotonic()

    def _action_sample(self, topic, message):
        active = any(item.status in (1, 2, 3) for item in message.status_list)
        if active:
            self.action_generation += 1
        self.active_actions[topic] = active

    def assert_stop(self, active):
        if active:
            self._reset_asserted_at = time.monotonic()
            self.reset_ready = False
        message = self._Bool()
        message.data = bool(active)
        self.publisher.publish(message)

    def _stationary(self):
        names, positions, velocities, _ = self.gateway.backend.joint_state()
        root = self.gateway.backend.root_state()
        return self.stationary_window.observe(
            time.monotonic(), names, positions, velocities, root
        )

    async def _wait(self, predicate, description, *, reject_action=False):
        deadline = time.monotonic() + self.timeout_s
        stable_since = None
        while time.monotonic() < deadline:
            if reject_action and any(self.active_actions.values()):
                raise ResetFailure("active action: scoped reset supports idle qualification only")
            if predicate():
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 0.25:
                    return
            else:
                stable_since = None
            await asyncio.sleep(0.02)
        missing = [topic for topic in self.action_topics if self.node.count_publishers(topic) == 0]
        raise ResetFailure(
            f"{description} timed out; missing action publishers={missing}; "
            f"effective stop={self.effective_stop}; reset ready={self.reset_ready}; "
            f"stationary observation={self.stationary_window.reason}"
        )

    async def idle(self):
        # Monitor from service startup, including retained action status.
        # A missing action server is not evidence that the robot is idle.
        await self._wait(
            lambda: (
                all(self.node.count_publishers(topic) > 0 for topic in self.action_topics)
                and self.effective_stop is False and self._stationary()
            ), "idle action/controller observation", reject_action=True,
        )

    async def stopped(self, generation):
        def stopped():
            if self.action_generation != generation:
                raise ResetFailure("action crossed reset stop boundary")
            return (
                self.effective_stop is True and self.reset_ready
                and self._reset_ready_at >= self._reset_asserted_at
                and self.gateway.backend.safety_stopped and self._stationary()
            )
        await self._wait(stopped, "controller deactivation and backend hold", reject_action=True)

    async def released(self):
        await self._wait(
            lambda: (
                self.effective_stop is False
                and not self.gateway.backend.safety_stopped
            ), "controller reactivation and effective stop release",
        )

    def close(self):
        # Do not publish a clear during teardown: a failed reset keeps its own
        # retained assertion until the operator recovers/restarts this provider.
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()


def install_scoped_reset(gateway):
    """Install the project-owned provider adapter before service registration."""
    existing = getattr(gateway, "_scoped_reset", None)
    if existing is not None:
        return existing
    import omni.timeline
    from isaacsim.ros2.sim_control.impl.simulation_control import SimulationControl

    baseline = PhysicsBaseline(gateway.backend)
    transport = ResetTransport(gateway)
    reset = IdleReset(gateway, omni.timeline.get_timeline_interface(), transport, baseline)
    gateway._scoped_reset = reset
    bind_provider(SimulationControl, reset)
    return reset
