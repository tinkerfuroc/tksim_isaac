"""Opt-in wall-time attribution of the lidar's Python path (TINKER_SIM_PROFILE=1)."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.lidar_rig import FrameAccumulator, LidarSpec, RaycastLidar, lidar_pattern  # noqa: E402
from tinker_sim_isaac.ros_gateway import RosStandardGateway  # noqa: E402


def _rig(profile: bool) -> RaycastLidar:
    spec = LidarSpec(
        pointcloud_topic="/livox/lidar", scan_topic="/scan", frame_id="livox360",
        tick_rate_hz=10.0, mount_prim="livox_frame", channels=1, columns=12,
        elevation_min_deg=0.0, elevation_max_deg=0.0, min_range_m=0.2,
        max_range_m=40.0, sweep=True, self_filter=True,
    )
    rig = RaycastLidar(spec)
    rig.profile_enabled = profile
    step = [0]

    def _hits():
        hits = np.zeros((12, 3), dtype=np.float32)
        hits[0] = (1.0, 0.0, 0.0)
        return hits

    def get_data():
        step[0] += 1
        return {"hit_positions": _hits(), "hit_prim_paths": ["/World/Arena/wall"] * 12,
                "physics_step": step[0]}

    def get_sensor_reading():
        step[0] += 1
        return SimpleNamespace(is_valid=True, hit_positions=_hits(),
                               hit_prim_paths=["/World/Arena/wall"] * 12)

    rig.sensor = SimpleNamespace(get_data=get_data, get_sensor_reading=get_sensor_reading)
    rig._step_counter = lambda: step[0]  # read by Task 7's _read(); harmless before it
    _, _, offsets = lidar_pattern(spec)
    rig._accumulator = FrameAccumulator(12, 12, offsets)
    return rig


def test_profile_counts_calls_and_resets():
    rig = _rig(profile=True)
    for _ in range(5):
        rig.accumulate(source="substep")
    snap = rig.profile_snapshot()
    assert snap["calls"] == 5
    assert set(snap) == {"get_data", "self_mask", "fold", "calls"}
    assert all(snap[k] >= 0.0 for k in ("get_data", "self_mask", "fold"))
    assert rig.profile_snapshot()["calls"] == 0  # window reset


def test_profile_off_records_nothing():
    rig = _rig(profile=False)
    rig.accumulate(source="substep")
    assert rig.profile_snapshot()["calls"] == 0


def test_profile_reads_env_flag(monkeypatch):
    monkeypatch.setenv("TINKER_SIM_PROFILE", "1")
    spec = _rig(profile=False).spec
    assert RaycastLidar(spec).profile_enabled is True
    monkeypatch.delenv("TINKER_SIM_PROFILE")
    assert RaycastLidar(spec).profile_enabled is False


def test_gateway_snapshot_passthrough():
    gateway = object.__new__(RosStandardGateway)
    gateway.lidar_rig = _rig(profile=True)
    gateway.lidar_rig.accumulate(source="substep")
    assert gateway.lidar_profile_snapshot()["calls"] == 1
    gateway.lidar_rig = None
    assert gateway.lidar_profile_snapshot() is None


def test_reads_only_positions_and_paths_from_the_raw_reading():
    rig = _rig(profile=False)
    touched = []

    class _Reading:
        is_valid = True

        def __getattr__(self, name):
            touched.append(name)
            if name == "hit_positions":
                hits = np.zeros((12, 3), dtype=np.float32)
                hits[0] = (1.0, 0.0, 0.0)
                return hits
            if name == "hit_prim_paths":
                return ["/World/Arena/wall"] * 12
            raise AssertionError(f"unexpected field read: {name}")

    rig.sensor = SimpleNamespace(get_sensor_reading=lambda: _Reading())
    rig._step_counter = lambda: 7
    rig.accumulate(source="substep")
    assert sorted(set(touched)) == ["hit_positions", "hit_prim_paths"]
    assert rig.diagnostics()["substep_folds"] == 1


def test_default_step_counter_uses_the_sensor_modules_simulation_manager(monkeypatch):
    import types

    sensor_mod = types.ModuleType("fake_raycast_sensor_mod")

    class _Manager:
        @staticmethod
        def get_num_physics_steps():
            return 41

    sensor_mod.SimulationManager = _Manager

    class _Sensor:
        def get_sensor_reading(self):
            hits = np.zeros((12, 3), dtype=np.float32)
            hits[0] = (1.0, 0.0, 0.0)
            return SimpleNamespace(is_valid=True, hit_positions=hits,
                                   hit_prim_paths=["/World/Arena/wall"] * 12)

    _Sensor.__module__ = "fake_raycast_sensor_mod"
    # Isaac Lab rebinds the global name to a class without the method.
    global_mod = types.ModuleType("isaacsim.core.simulation_manager")
    global_mod.SimulationManager = type("PhysxManager", (), {})
    monkeypatch.setitem(sys.modules, "fake_raycast_sensor_mod", sensor_mod)
    monkeypatch.setitem(sys.modules, "isaacsim.core.simulation_manager", global_mod)

    rig = _rig(profile=False)
    rig._step_counter = rig._sensor_module_step_count  # the production default, un-faked
    rig.sensor = _Sensor()
    rig.accumulate(source="substep")
    assert rig.diagnostics()["substep_folds"] == 1
    assert rig._last_physics_step == 41
    assert "read" not in rig._reported_failures
