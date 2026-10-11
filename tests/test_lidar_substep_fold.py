"""The lidar must fold every PhysX substep, not once per control step.

At 120 Hz physics / 60 Hz control the gateway used to poll the rig once per
control step: every other sweep slice was cast but never read, a 12-slice
window took 24 physics steps (5 Hz frames), and alternating ~30 deg wedges
were never published (measured live 2026-10-11).
"""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "simulation"))

from tinker_sim_isaac.backend import IsaacWholeRobotBackend  # noqa: E402
from tinker_sim_isaac.lidar_rig import FrameAccumulator, LidarSpec, RaycastLidar, lidar_pattern  # noqa: E402
from tinker_sim_isaac.ros_gateway import RosStandardGateway  # noqa: E402

COLUMNS = 12  # one column fires per physics step: 12 steps = one 10 Hz frame at 120 Hz


def _spec() -> LidarSpec:
    return LidarSpec(
        pointcloud_topic="/livox/lidar", scan_topic="/scan", frame_id="livox360",
        tick_rate_hz=10.0, mount_prim="livox_frame", channels=1, columns=COLUMNS,
        elevation_min_deg=0.0, elevation_max_deg=0.0, min_range_m=0.2,
        max_range_m=40.0, sweep=True, self_filter=False,
    )


class _SweepSensor:
    """Each physics step fires exactly one ray (column = step % 12)."""

    def __init__(self, counter, *, valid_from_step: int = 0):
        self.counter = counter
        self.valid_from_step = valid_from_step

    def get_data(self):
        step = self.counter[0]
        if step < self.valid_from_step:
            return {"hit_positions": np.zeros((0, 3), dtype=np.float32),
                    "hit_prim_paths": [], "physics_step": step}
        hits = np.zeros((COLUMNS, 3), dtype=np.float32)
        hits[(step - 1) % COLUMNS] = (float((step - 1) % COLUMNS) + 1.0, 0.0, 0.0)
        return {"hit_positions": hits, "hit_prim_paths": [""] * COLUMNS, "physics_step": step}

    def get_sensor_reading(self):
        """Raw-struct view of the same reading (used once Task 7 lands)."""
        data = self.get_data()
        return SimpleNamespace(
            is_valid=data["hit_positions"].shape[0] > 0,
            hit_positions=data["hit_positions"],
            hit_prim_paths=data["hit_prim_paths"],
        )


def _rig(counter, **sensor_kw) -> RaycastLidar:
    spec = _spec()
    rig = RaycastLidar(spec)
    rig.sensor = _SweepSensor(counter, **sensor_kw)
    rig._step_counter = lambda: counter[0]  # read by Task 7's _read(); harmless before it
    _, _, offsets = lidar_pattern(spec)
    rig._accumulator = FrameAccumulator(spec.num_rays, COLUMNS, offsets)
    return rig


def _backend(counter, substeps: int) -> IsaacWholeRobotBackend:
    backend = object.__new__(IsaacWholeRobotBackend)
    backend.physics_substeps = substeps
    backend.render = False

    def _step(render):
        counter[0] += 1

    backend._sim = SimpleNamespace(step=_step)
    return backend


def _gateway(backend, rig) -> RosStandardGateway:
    gateway = object.__new__(RosStandardGateway)
    gateway.backend = backend
    gateway.lidar_rig = rig
    gateway._lidar_folds_on_substep = gateway._attach_lidar_to_substeps()
    return gateway


def _run_control_steps(backend, gateway, n: int) -> None:
    for _ in range(n):
        backend._step_simulation()
        gateway._poll_lidar_if_unattached()


def test_hooks_run_after_every_substep_in_order():
    counter = [0]
    backend = _backend(counter, substeps=3)
    seen = []
    backend.add_post_substep_hook(lambda: seen.append(counter[0]))
    backend._step_simulation()
    assert seen == [1, 2, 3]


def test_two_substeps_complete_a_full_frame_in_six_control_steps():
    counter = [0]
    backend, rig = _backend(counter, substeps=2), _rig(counter)
    gateway = _gateway(backend, rig)
    assert gateway._lidar_folds_on_substep is True
    _run_control_steps(backend, gateway, 6)  # 12 physics steps = one 10 Hz frame
    assert rig.frame_ready
    points, _ = rig.take_frame()
    assert sorted(points[:, 0].tolist()) == [float(i) for i in range(1, COLUMNS + 1)]
    assert rig.diagnostics()["substep_folds"] == 12
    assert rig.diagnostics()["poll_folds"] == 0


def test_one_substep_folds_once_and_publish_path_stays_idle():
    counter = [0]
    backend, rig = _backend(counter, substeps=1), _rig(counter)
    gateway = _gateway(backend, rig)
    _run_control_steps(backend, gateway, COLUMNS)
    diag = rig.diagnostics()
    assert diag["substep_folds"] == COLUMNS
    assert diag["poll_folds"] == 0
    assert diag["frames"] == 1


def test_backend_without_hook_falls_back_to_publish_path_poll():
    counter = [0]
    rig = _rig(counter)
    legacy_backend = SimpleNamespace()  # no add_post_substep_hook
    gateway = _gateway(legacy_backend, rig)
    assert gateway._lidar_folds_on_substep is False
    counter[0] = 1
    gateway._poll_lidar_if_unattached()
    assert rig.diagnostics()["poll_folds"] == 1


def test_empty_readings_during_warmup_do_not_raise():
    counter = [0]
    backend, rig = _backend(counter, substeps=2), _rig(counter, valid_from_step=5)
    gateway = _gateway(backend, rig)
    _run_control_steps(backend, gateway, 6)
    diag = rig.diagnostics()
    assert diag["substep_folds"] == 12
    assert diag["frames"] == 1  # window still advances one slot per physics step
