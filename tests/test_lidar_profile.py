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
