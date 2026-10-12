"""CPU contracts for the provider-owned idle reset transaction."""
import asyncio
from types import SimpleNamespace

import pytest

from tinker_sim_isaac.scoped_reset import IdleReset, ResetFailure, bind_provider, PhysicsBaseline


class Timeline:
    def __init__(self, events):
        self.events = events
        self.playing = True

    def is_playing(self):
        return self.playing

    def pause(self):
        self.events.append("pause")
        self.playing = False

    def play(self):
        self.events.append("play")
        self.playing = True


def test_spawn_registration_does_not_read_stopped_physics_or_replace_old_baseline():
    baseline=object.__new__(PhysicsBaseline)
    baseline.bodies={'/World/old':('original',None)}
    def invalid_physics():
        raise RuntimeError('Simulation view is invalidated while stopped')
    baseline._read_bodies=invalid_physics
    baseline._read_spawn_bodies=lambda:{'/World/old':('moved',None),
                                        '/World/new':('authored',None)}
    baseline.register_spawn()
    assert baseline.bodies=={'/World/old':('original',None),'/World/new':('authored',None)}


class Transport:
    def __init__(self, events):
        self.events = events
        self.active = False
        self.own_stop = False
        self.operator_stop = False
        self.fail_at = None
        self.action_generation = 0

    async def idle(self):
        self.events.append("idle")
        if self.active:
            raise ResetFailure("active action")

    def assert_stop(self, active):
        self.events.append(("reset_stop", active))
        self.own_stop = active

    async def stopped(self, generation):
        self.events.append("stopped")
        if self.fail_at == "stopped":
            raise ResetFailure("controller stop timed out")
        if self.active or self.action_generation != generation:
            raise ResetFailure("action crossed reset boundary")

    async def released(self):
        self.events.append("released")
        if self.operator_stop or self.fail_at == "released":
            raise ResetFailure("effective stop remains active")


class Baseline:
    def __init__(self, events):
        self.events = events
        self.initial = [1, 2]
        self.actual = [3, 4]
        self.fail = False

    def prepare(self):
        self.events.append("prepare")

    def clear_commands(self):
        self.events.append("clear_commands")

    def restore(self):
        self.events.append("restore")
        if self.fail:
            raise RuntimeError("physics write failed")
        self.actual[:] = self.initial


class Gateway:
    def __init__(self, events):
        self.events = events
        self.backend = SimpleNamespace(
            simulation_time=42.0, _ros_reset_origin_s=0.0,
            render_frame=lambda: events.append("render"),
        )
        self._scoped_reset_publishing = False

    def publish_reset_boundary(self):
        self.events.append(("publish", self.backend._ros_reset_origin_s))


def fixture():
    events = []
    gateway = Gateway(events)
    transport = Transport(events)
    baseline = Baseline(events)
    timeline = Timeline(events)

    async def settle():
        events.append("settle")

    reset = IdleReset(gateway, timeline, transport, baseline, settle=settle)
    return reset, events, gateway, transport, baseline, timeline


@pytest.mark.parametrize("scope", [1, 2, 3])
def test_idle_reset_restores_requested_scope_in_place_and_orders_boundaries(scope):
    reset, events, gateway, transport, baseline, timeline = fixture()
    identity = id(baseline.actual)
    asyncio.run(reset.run(scope))
    assert baseline.actual == ([1, 2] if scope & 2 else [3, 4])
    assert id(baseline.actual) == identity
    assert gateway.backend._ros_reset_origin_s == (42.0 if scope & 1 else 0.0)
    assert events.index("stopped") < events.index("pause")
    assert events.index("pause") < events.index("settle")
    if not scope & 2:
        assert events.index("settle") < events.index("clear_commands")
    assert events.index("render") < events.index(("publish", gateway.backend._ros_reset_origin_s))
    assert events.index(("publish", gateway.backend._ros_reset_origin_s)) < events.index(("reset_stop", False))
    assert timeline.playing and not transport.own_stop
    assert not gateway._scoped_reset_publishing


def test_active_motion_is_rejected_before_stop_or_state_change():
    reset, events, gateway, transport, baseline, timeline = fixture()
    transport.active = True
    with pytest.raises(ResetFailure, match="active action"):
        asyncio.run(reset.run(3))
    assert events == ["idle"]
    assert baseline.actual == [3, 4]
    assert gateway.backend._ros_reset_origin_s == 0.0
    assert timeline.playing and not transport.own_stop


@pytest.mark.parametrize("failure", ["stopped", "write", "released"])
def test_failure_keeps_own_stop_and_paused_state_explicit(failure):
    reset, events, gateway, transport, baseline, timeline = fixture()
    transport.fail_at = failure
    baseline.fail = failure == "write"
    with pytest.raises(ResetFailure, match="paused; reset stop remains asserted"):
        asyncio.run(reset.run(3))
    assert transport.own_stop and not timeline.playing
    assert events[-2:] == [("reset_stop", True), "pause"]
    if failure == "stopped":
        assert baseline.actual == [3, 4]
        assert gateway.backend._ros_reset_origin_s == 0.0


def test_release_never_clears_operator_stop():
    reset, events, gateway, transport, baseline, timeline = fixture()
    transport.operator_stop = True
    with pytest.raises(ResetFailure, match="effective stop remains active"):
        asyncio.run(reset.run(3))
    assert transport.operator_stop
    assert transport.own_stop and not timeline.playing


def test_concurrent_reset_is_rejected_without_touching_first_transaction():
    reset, events, *_ = fixture()
    reset.busy = True
    with pytest.raises(ResetFailure, match="already in progress"):
        asyncio.run(reset.run(3))
    assert not events


def test_inflight_provider_mutation_blocks_reset_without_changing_physics():
    reset, events, *_ = fixture()
    reset.mutations_inflight = 1
    with pytest.raises(ResetFailure, match="mutation is in progress"):
        asyncio.run(reset.run(3))
    assert not events


def test_action_arriving_during_stop_barrier_cannot_restore_state():
    reset, events, gateway, transport, baseline, timeline = fixture()
    original = transport.assert_stop
    def crossed(active):
        original(active)
        if active:
            transport.action_generation += 1
    transport.assert_stop = crossed
    with pytest.raises(ResetFailure, match="action crossed reset boundary"):
        asyncio.run(reset.run(3))
    assert baseline.actual == [3, 4]
    assert gateway.backend._ros_reset_origin_s == 0.0


def test_provider_routes_scopes_and_rejects_mutators_during_reset(monkeypatch):
    import sys
    class Result:
        RESULT_OK = 0
        RESULT_OPERATION_FAILED = 1
        def __init__(self, result=0, error_message=""):
            self.result, self.error_message = result, error_message
    features = SimpleNamespace(SIMULATION_RESET_TIME=31, SIMULATION_RESET_STATE=32)
    monkeypatch.setitem(sys.modules, "simulation_interfaces.msg",
                        SimpleNamespace(Result=Result, SimulatorFeatures=features))
    calls = []
    class Provider:
        async def _handle_reset_simulation(self, request, response):
            calls.append("legacy")
            return response
        async def _handle_get_simulator_features(self, request, response):
            return response
        async def _handle_set_entity_state(self, request, response):
            calls.append("mutation")
            return response
        async def _spawn_single_entity_core(self, **kwargs):
            calls.append("spawn")
            return 0, "", "/World/cube"
    reset, events, *_ = fixture()
    reset.baseline.register_spawn = lambda: calls.append("register")
    bind_provider(Provider, reset)
    provider = Provider()
    response = SimpleNamespace(result=Result(), features=SimpleNamespace(features=[]))
    asyncio.run(provider._handle_reset_simulation(SimpleNamespace(scope=3), response))
    assert response.result.result == 0 and "legacy" not in calls
    asyncio.run(provider._handle_reset_simulation(SimpleNamespace(scope=0), response))
    assert calls == ["legacy"]
    asyncio.run(provider._handle_get_simulator_features(None, response))
    assert response.features.features == [31, 32]
    asyncio.run(provider._spawn_single_entity_core())
    assert calls[-2:] == ["spawn", "register"]
    reset.busy = True
    asyncio.run(provider._handle_set_entity_state(None, response))
    assert response.result.result == 1 and "mutation" not in calls
    assert "reset" in response.result.error_message
    assert asyncio.run(provider._spawn_single_entity_core())[0] == 1


def test_physics_baseline_restores_original_robot_and_spawn_state_without_rebinding():
    import numpy as np
    class Tensor(np.ndarray):
        def clone(self):
            return self.copy()
        def zero_(self):
            self[:] = 0
    def tensor(value):
        return np.asarray(value, dtype=float).view(Tensor)
    data = SimpleNamespace(
        root_link_pose_w=tensor([[1, 2, 3, 0, 0, 0, 1]]),
        root_com_vel_w=tensor([[0] * 6]),
        joint_pos=tensor([[0.1, 0.2]]), joint_vel=tensor([[0, 0]]),
    )
    writes = []
    robot = SimpleNamespace(
        data=data, reset=lambda: writes.append("actuators"),
        write_root_pose_to_sim_index=lambda **kw: writes.append(("pose", kw["root_pose"].copy())),
        write_root_velocity_to_sim_index=lambda **kw: writes.append(("velocity", kw["root_velocity"].copy())),
        write_joint_state_to_sim_index=lambda **kw: writes.append(("joints", kw["position"].copy())),
        set_joint_position_target=lambda p: writes.append(("target", p.copy())),
        set_joint_velocity_target=lambda p: None,
        set_joint_effort_target=lambda p: None,
        write_data_to_sim=lambda: writes.append("write"),
    )
    bodies = {"/cube": ((1, 2, 3, 0, 0, 0, 1), (0,) * 6),
              "/table": ((0, 0, 0, 0, 0, 0, 1), None)}
    backend = SimpleNamespace(
        _robot=robot, _torch_value=lambda v: v,
        _position_targets=tensor([[9, 9]]), _velocity_targets=tensor([[9, 9]]),
        _effort_targets=tensor([[9, 9]]), _safety_stopped=True,
        _target_write_gate=SimpleNamespace(force_next=lambda: writes.append("force")),
        _base_hold_pose=data.root_link_pose_w.copy(), _wheel_indices=(0,),
        _contact_pairs_by_key={"old": 1}, _drive_command_target=9,
        _command_snapshot_id=9, _park_views={"/cube": object()},
        discard_command_snapshot_staging=lambda: writes.append("discard"),
        set_entity_pose_physics=lambda path, **kw: writes.append((path, kw)) or True,
    )
    baseline = PhysicsBaseline(backend, read_bodies=lambda: dict(bodies))
    original_view = backend._park_views["/cube"]
    bodies["/new"] = ((4, 5, 6, 0, 0, 0, 1), (0,) * 6)
    baseline.register_spawn()
    bodies["/cube"] = ((9, 9, 9, 0, 0, 0, 1), (1,) * 6)
    data.joint_pos[:] = 4
    baseline.prepare()
    baseline.restore()
    np.testing.assert_allclose(backend._position_targets, [[0.1, 0.2]])
    np.testing.assert_allclose(backend._safety_snapshot, [[0.1, 0.2]])
    assert not backend._velocity_targets.any() and not backend._effort_targets.any()
    assert backend._drive_command_target is None and backend._command_snapshot_id is None
    assert backend._contact_pairs_by_key == {}
    assert backend._park_views["/cube"] is original_view
    restored = dict(item for item in writes if isinstance(item, tuple) and item[0].startswith("/"))
    assert restored["/cube"]["position"] == (1, 2, 3)
    assert restored["/new"]["position"] == (4, 5, 6)
    assert restored["/cube"]["linear_velocity"] == (0,) * 3
    assert restored["/table"]["position"] == (0, 0, 0)
    assert restored["/table"]["linear_velocity"] is None
    assert restored["/table"]["angular_velocity"] is None
    bodies["/unregistered"] = bodies["/cube"]
    with pytest.raises(ResetFailure, match="initial state"):
        baseline.prepare()


@pytest.mark.parametrize("state,reset_source,switching,expected", [
    ("active", True, False, False),
    ("inactive", True, False, True),
    ("unconfigured", True, False, False),
    ("inactive", False, False, False),
    ("inactive", True, True, False),
])
def test_supervisor_acknowledges_only_confirmed_inactive_reset(state, reset_source, switching, expected):
    from tinker_sim_bridge.safety_supervisor import SafetySupervisor
    supervisor = object.__new__(SafetySupervisor)
    supervisor._sources = {"reset": reset_source, "operator": True}
    supervisor._management_ready = True
    supervisor._manage_controllers = True
    supervisor._controller_state = state
    supervisor._switch_inflight = switching
    assert supervisor._reset_is_ready() is expected
    # Provider release cannot clear a distinct operator assertion.
    supervisor._sources["reset"] = False
    assert supervisor._sources["operator"] is True
    assert not supervisor._reset_is_ready()


def test_clock_and_reset_joint_publication_share_one_rewound_epoch():
    from tinker_sim_isaac.backend import IsaacWholeRobotBackend
    from tinker_sim_isaac.ros_gateway import RosStandardGateway
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    backend = object.__new__(IsaacWholeRobotBackend)
    counter = [420]
    backend._sim = SimpleNamespace(get_physics_step_count=lambda: counter[0])
    backend._clock_step_origin = 0
    backend.physics_dt = 0.1
    backend._clock_epoch_s = 1000.0
    assert backend.ros_clock_time == 1042.0
    backend._ros_reset_origin_s = backend.simulation_time
    assert backend.simulation_time == 42.0
    assert backend.ros_clock_time == 1000.0
    backend.joint_state = lambda: (("joint",), [0.1], [0.0], [0.0])
    gateway = object.__new__(RosStandardGateway)
    gateway.backend, gateway._Clock, gateway._JointState = backend, Clock, JointState
    clocks, joints = [], []
    gateway.clock_pub = SimpleNamespace(publish=clocks.append)
    gateway.joint_pub = SimpleNamespace(publish=joints.append)
    gateway.publish_reset_boundary()
    assert clocks[0].clock == joints[0].header.stamp
    assert clocks[0].clock.sec == 1000 and list(joints[0].position) == [0.1]
    counter[0] += 1
    assert backend.simulation_time > 42.0 and backend.ros_clock_time > 1000.0
    gateway._scoped_reset_publishing = True
    gateway._camera_rig = SimpleNamespace(capture=lambda: pytest.fail("old camera frame consumed"))
    gateway.publish_cameras()


def test_ros_transport_uses_only_its_reset_source_and_no_controller_client(monkeypatch):
    import rclpy
    from tinker_sim_isaac.scoped_reset import ResetTransport
    monkeypatch.setenv("ROS_DOMAIN_ID", "89")
    rclpy.init()
    transport = None
    try:
        transport = ResetTransport(SimpleNamespace(backend=None))
        publishers = dict(transport.node.get_publisher_names_and_types_by_node(
            "tinker_scoped_reset", "/"))
        assert "/sim/safety/reset" in publishers
        assert "/sim/hardware/safety_stop" not in publishers
        assert transport.node.get_client_names_and_types_by_node("tinker_scoped_reset", "/") == []
        transport._ready_sample(SimpleNamespace(data=True))
        transport.assert_stop(True)
        assert not transport.reset_ready
        transport._action_sample("arm", SimpleNamespace(status_list=[SimpleNamespace(status=2)]))
        generation = transport.action_generation
        transport._action_sample("arm", SimpleNamespace(status_list=[SimpleNamespace(status=4)]))
        assert not transport.active_actions["arm"] and generation == transport.action_generation
    finally:
        if transport is not None:
            transport.close()
        rclpy.shutdown()


def stationary_fixture():
    names = [f"joint{i}" for i in range(1, 8)] + ["drive_joint", "pan_joint", "tilt_joint", "rear_wheel_joint", "left_finger_joint"]
    root = {"position": (0., 0., 0.), "quaternion_wxyz": (1., 0., 0., 0.),
            "linear_velocity_world": (0., 0., 0.), "angular_velocity_world": (0., 0., 0.)}
    return names, [0.] * len(names), [0.] * len(names), root


def test_stationary_window_accepts_contact_velocity_noise_and_passive_wheel_rotation():
    from tinker_sim_isaac.scoped_reset import StationaryWindow
    window = StationaryWindow()
    names, positions, velocities, root = stationary_fixture()
    velocities[7] = .02244
    velocities[-1] = .041518
    velocities[-2] = .02536
    assert not window.observe(0., names, positions, velocities, root)
    for tick in range(1, 6):
        positions[7] = .00006 * (tick % 2)
        positions[-2] += .001
        stationary = window.observe(tick * .1, names, positions, velocities, root)
    assert stationary


@pytest.mark.parametrize("kind", ["joint", "root", "rotation", "nonfinite"])
def test_stationary_window_rejects_actual_slow_displacement_or_invalid_measurement(kind):
    from tinker_sim_isaac.scoped_reset import StationaryWindow
    window = StationaryWindow()
    names, positions, velocities, root = stationary_fixture()
    assert not window.observe(0., names, positions, velocities, root)
    if kind == "joint":
        positions[0] = .001
    elif kind == "root":
        root["position"] = (.001, 0., 0.)
    elif kind == "rotation":
        root["quaternion_wxyz"] = (.99995, 0., 0., .01)
    else:
        velocities[0] = float("nan")
    assert not window.observe(.3, names, positions, velocities, root)
    assert window.reason != "stationary"


def test_idle_timeout_reports_the_specific_unmet_observer_prerequisite():
    from tinker_sim_isaac.scoped_reset import ResetTransport
    transport = object.__new__(ResetTransport)
    transport.timeout_s = 0.
    transport.action_topics = ["arm", "gripper"]
    transport.node = SimpleNamespace(count_publishers=lambda topic: 1)
    transport.effective_stop = False
    transport.reset_ready = False
    transport.stationary_window = SimpleNamespace(reason="drive_joint measured span exceeds 0.0005")
    with pytest.raises(ResetFailure, match="stationary observation=drive_joint measured span"):
        asyncio.run(transport._wait(lambda: False, "idle observation"))
