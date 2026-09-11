"""Contract tests for the live PhysX raycast lidar's pure logic.

The rig's Isaac-touching half (prim creation, mount resolution, the physics
callback) needs a running simulator, but the parts that decide whether the
published cloud is CORRECT do not: the declared contract, the ray pattern, and
above all the frame-accumulation window. Those are covered here.

The accumulation window is the subtle one. Isaac Sim 6.0.1's raycast sensor
returns only the rays fired in the current physics step and clears the buffer
each step, so a frame is the union of `window_steps` consecutive readings.
Getting that wrong yields a cloud that looks plausible -- roughly a twelfth of
the points, arriving at the right rate -- while silently throwing away most of
the scan.
"""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.lidar_rig import (  # noqa: E402
    FrameAccumulator,
    self_hit_mask,
    LidarSpec,
    LidarSpecError,
    frame_summary,
    lidar_pattern,
    load_lidar_spec,
    scan_ring_indices,
    window_steps_for,
)

CONTRACT = ROOT / "simulation/sensors/hardware-parity.json"


def _spec(**overrides) -> LidarSpec:
    base = dict(
        pointcloud_topic="/livox/lidar",
        scan_topic="/scan",
        frame_id="livox360",
        tick_rate_hz=10.0,
        mount_prim="livox_frame",
        channels=4,
        columns=8,
        elevation_min_deg=-7.0,
        elevation_max_deg=52.0,
        min_range_m=0.2,
        max_range_m=40.0,
        sweep=True,
        self_filter=True,
    )
    base.update(overrides)
    return LidarSpec(**base)


class TestLoadLidarSpec(unittest.TestCase):
    def test_loads_the_shipped_contract(self) -> None:
        spec = load_lidar_spec(CONTRACT)
        self.assertEqual(spec.pointcloud_topic, "/livox/lidar")
        self.assertEqual(spec.frame_id, "livox360")
        self.assertEqual(spec.mount_prim, "livox_frame")
        self.assertEqual(spec.tick_rate_hz, 10.0)
        self.assertEqual((spec.elevation_min_deg, spec.elevation_max_deg), (-7.0, 52.0))
        self.assertTrue(spec.sweep)

    def test_shipped_contract_is_mid360_scale(self) -> None:
        """198,930 points/s against the Mid-360's 200,000."""
        spec = load_lidar_spec(CONTRACT)
        self.assertEqual(spec.num_rays, 19893)
        self.assertAlmostEqual(spec.points_per_second, 198930.0)
        self.assertLess(abs(spec.points_per_second - 200_000.0) / 200_000.0, 0.01)

    def test_shipped_contract_spacing_is_isotropic(self) -> None:
        spec = load_lidar_spec(CONTRACT)
        vertical = (spec.elevation_max_deg - spec.elevation_min_deg) / (spec.channels - 1)
        horizontal = 360.0 / spec.columns
        self.assertLess(abs(vertical - horizontal), 0.05)

    def _write(self, mutate) -> Path:
        raw = json.loads(CONTRACT.read_text(encoding="utf-8"))
        mutate(raw)
        directory = tempfile.mkdtemp()
        path = Path(directory) / "contract.json"
        path.write_text(json.dumps(raw), encoding="utf-8")
        return path

    def test_rejects_wrong_schema_version(self) -> None:
        path = self._write(lambda raw: raw.__setitem__("schema_version", 1))
        with self.assertRaises(LidarSpecError):
            load_lidar_spec(path)

    def test_rejects_missing_raycast_block(self) -> None:
        path = self._write(lambda raw: raw["lidar"].pop("raycast"))
        with self.assertRaises(LidarSpecError):
            load_lidar_spec(path)

    def test_rejects_non_sensor_data_qos(self) -> None:
        path = self._write(lambda raw: raw["lidar"].__setitem__("qos", "reliable"))
        with self.assertRaises(LidarSpecError):
            load_lidar_spec(path)

    def test_rejects_inverted_elevation_range(self) -> None:
        path = self._write(
            lambda raw: raw["lidar"]["raycast"].__setitem__(
                "elevation_range_deg", [52.0, -7.0]
            )
        )
        with self.assertRaises(LidarSpecError):
            load_lidar_spec(path)

    def test_rejects_min_range_at_or_above_max(self) -> None:
        """The plugin silently DISABLES a sensor with minRange >= maxRange."""
        path = self._write(
            lambda raw: raw["lidar"]["raycast"].__setitem__("min_range_m", 40.0)
        )
        with self.assertRaises(LidarSpecError):
            load_lidar_spec(path)

    def test_rejects_zero_channels(self) -> None:
        path = self._write(lambda raw: raw["lidar"]["raycast"].__setitem__("channels", 0))
        with self.assertRaises(LidarSpecError):
            load_lidar_spec(path)

    def test_rejects_non_boolean_sweep(self) -> None:
        path = self._write(
            lambda raw: raw["lidar"]["raycast"].__setitem__("sweep", "yes")
        )
        with self.assertRaises(LidarSpecError):
            load_lidar_spec(path)


class TestLidarPattern(unittest.TestCase):
    def test_shape_and_row_order(self) -> None:
        spec = _spec(channels=4, columns=8)
        origins, directions, offsets = lidar_pattern(spec)
        self.assertEqual(origins.shape, (32, 3))
        self.assertEqual(directions.shape, (32, 3))
        self.assertEqual(offsets.shape, (32,))
        # Channel-major: the first `columns` rays share one elevation.
        first_channel_z = directions[: spec.columns, 2]
        self.assertTrue(np.allclose(first_channel_z, first_channel_z[0]))
        self.assertFalse(np.isclose(directions[0, 2], directions[spec.columns, 2]))

    def test_origins_are_at_the_sensor(self) -> None:
        _, _, _ = lidar_pattern(_spec())
        origins, _, _ = lidar_pattern(_spec())
        self.assertTrue(np.array_equal(origins, np.zeros_like(origins)))

    def test_directions_are_unit_length(self) -> None:
        _, directions, _ = lidar_pattern(_spec(channels=9, columns=37))
        self.assertTrue(np.allclose(np.linalg.norm(directions, axis=1), 1.0, atol=1e-6))

    def test_elevation_span_matches_the_spec(self) -> None:
        spec = _spec(channels=13, columns=5)
        _, directions, _ = lidar_pattern(spec)
        elevations = np.degrees(np.arcsin(directions[:, 2]))
        self.assertAlmostEqual(float(elevations.min()), spec.elevation_min_deg, places=4)
        self.assertAlmostEqual(float(elevations.max()), spec.elevation_max_deg, places=4)

    def test_azimuth_covers_the_full_circle_without_duplicating(self) -> None:
        spec = _spec(channels=1, columns=8)
        _, directions, _ = lidar_pattern(spec)
        azimuths = np.degrees(np.arctan2(directions[:, 1], directions[:, 0])) % 360.0
        self.assertEqual(len(np.unique(np.round(azimuths, 6))), 8)

    def test_sweep_offsets_span_one_frame_period(self) -> None:
        spec = _spec(channels=3, columns=10, tick_rate_hz=10.0)
        _, _, offsets = lidar_pattern(spec)
        self.assertGreaterEqual(float(offsets.min()), 0.0)
        self.assertLess(float(offsets.max()), spec.frame_period_s)
        # Offsets sweep by azimuth, so each channel repeats the same schedule.
        self.assertTrue(np.allclose(offsets[:10], offsets[10:20]))

    def test_sweep_disabled_gives_an_instantaneous_scan(self) -> None:
        _, _, offsets = lidar_pattern(_spec(sweep=False))
        self.assertTrue(np.array_equal(offsets, np.zeros_like(offsets)))


class TestWindowSteps(unittest.TestCase):
    def test_validated_rates_give_twelve_steps(self) -> None:
        """120 Hz physics, 10 Hz lidar."""
        self.assertEqual(window_steps_for(_spec(tick_rate_hz=10.0), 1.0 / 120.0), 12)

    def test_sixty_hertz_physics_halves_the_window(self) -> None:
        self.assertEqual(window_steps_for(_spec(tick_rate_hz=10.0), 1.0 / 60.0), 6)

    def test_window_is_never_zero(self) -> None:
        self.assertEqual(window_steps_for(_spec(tick_rate_hz=10.0), 1.0), 1)

    def test_rejects_non_positive_dt(self) -> None:
        with self.assertRaises(ValueError):
            window_steps_for(_spec(), 0.0)


class TestFrameAccumulator(unittest.TestCase):
    def _offsets(self, num_rays: int) -> np.ndarray:
        return np.linspace(0.0, 0.09, num_rays)

    def test_frame_completes_only_after_the_full_window(self) -> None:
        accumulator = FrameAccumulator(4, 3, self._offsets(4))
        empty = np.zeros((4, 3), dtype=np.float32)
        self.assertFalse(accumulator.add_reading(empty))
        self.assertFalse(accumulator.add_reading(empty))
        self.assertTrue(accumulator.add_reading(empty))

    def test_union_across_the_window_is_the_whole_frame(self) -> None:
        """Each step returns a different slice; the frame is their union."""
        accumulator = FrameAccumulator(4, 2, self._offsets(4))
        first = np.zeros((4, 3), dtype=np.float32)
        first[0] = (1.0, 0.0, 0.0)
        first[1] = (0.0, 2.0, 0.0)
        second = np.zeros((4, 3), dtype=np.float32)
        second[2] = (0.0, 0.0, 3.0)
        second[3] = (4.0, 0.0, 0.0)

        self.assertFalse(accumulator.add_reading(first))
        self.assertTrue(accumulator.add_reading(second))
        points, times = accumulator.take_frame()
        self.assertEqual(points.shape, (4, 3))
        self.assertEqual(times.shape, (4,))
        self.assertTrue(np.allclose(np.linalg.norm(points, axis=1), [1.0, 2.0, 3.0, 4.0]))

    def test_zero_vectors_are_dropped_as_misses(self) -> None:
        """A miss and an unfired ray both read as the zero vector."""
        accumulator = FrameAccumulator(4, 1, self._offsets(4))
        reading = np.zeros((4, 3), dtype=np.float32)
        reading[2] = (0.0, 0.0, 5.0)
        self.assertTrue(accumulator.add_reading(reading))
        points, times = accumulator.take_frame()
        self.assertEqual(points.shape, (1, 3))
        self.assertTrue(np.allclose(points[0], (0.0, 0.0, 5.0)))
        self.assertAlmostEqual(float(times[0]), float(self._offsets(4)[2]))

    def test_an_all_miss_frame_is_empty_not_an_error(self) -> None:
        accumulator = FrameAccumulator(4, 1, self._offsets(4))
        self.assertTrue(accumulator.add_reading(np.zeros((4, 3), dtype=np.float32)))
        points, times = accumulator.take_frame()
        self.assertEqual(points.shape, (0, 3))
        self.assertEqual(times.shape, (0,))

    def test_take_frame_resets_for_the_next_window(self) -> None:
        accumulator = FrameAccumulator(4, 1, self._offsets(4))
        reading = np.zeros((4, 3), dtype=np.float32)
        reading[0] = (1.0, 0.0, 0.0)
        accumulator.add_reading(reading)
        accumulator.take_frame()
        self.assertEqual(accumulator.steps_in_window, 0)

        # The next frame must not inherit the previous frame's points.
        self.assertTrue(accumulator.add_reading(np.zeros((4, 3), dtype=np.float32)))
        points, _ = accumulator.take_frame()
        self.assertEqual(points.shape, (0, 3))
        self.assertEqual(accumulator.frames_completed, 2)

    def test_a_later_reading_refreshes_a_ray_rather_than_duplicating_it(self) -> None:
        accumulator = FrameAccumulator(2, 2, self._offsets(2))
        first = np.zeros((2, 3), dtype=np.float32)
        first[0] = (1.0, 0.0, 0.0)
        second = np.zeros((2, 3), dtype=np.float32)
        second[0] = (9.0, 0.0, 0.0)
        accumulator.add_reading(first)
        accumulator.add_reading(second)
        points, _ = accumulator.take_frame()
        self.assertEqual(points.shape, (1, 3))
        self.assertTrue(np.allclose(points[0], (9.0, 0.0, 0.0)))

    def test_a_mismatched_reading_raises_rather_than_misaligning(self) -> None:
        """Ray index must line up with the time-offset table, or points lie."""
        accumulator = FrameAccumulator(4, 1, self._offsets(4))
        with self.assertRaises(ValueError):
            accumulator.add_reading(np.zeros((3, 3), dtype=np.float32))

    def test_an_empty_reading_is_tolerated(self) -> None:
        """Before play the sensor returns a zero-length buffer."""
        accumulator = FrameAccumulator(4, 1, self._offsets(4))
        self.assertTrue(accumulator.add_reading(np.zeros((0, 3), dtype=np.float32)))

    def test_construction_rejects_a_mismatched_offset_table(self) -> None:
        with self.assertRaises(ValueError):
            FrameAccumulator(4, 1, self._offsets(3))

    def test_real_scale_window_assembles_one_mid360_frame(self) -> None:
        """12 slices of a 19,893-ray pattern make exactly one frame."""
        spec = load_lidar_spec(CONTRACT)
        _, _, offsets = lidar_pattern(spec)
        window = window_steps_for(spec, 1.0 / 120.0)
        accumulator = FrameAccumulator(spec.num_rays, window, offsets)

        rng = np.random.default_rng(0)
        fired = np.array_split(rng.permutation(spec.num_rays), window)
        complete = False
        for slice_indices in fired:
            reading = np.zeros((spec.num_rays, 3), dtype=np.float32)
            reading[slice_indices] = rng.uniform(1.0, 20.0, (len(slice_indices), 3))
            complete = accumulator.add_reading(reading)
        self.assertTrue(complete)
        points, times = accumulator.take_frame()
        self.assertEqual(points.shape[0], spec.num_rays)
        self.assertLess(float(times.max()), spec.frame_period_s)


class TestHelpers(unittest.TestCase):
    def test_scan_ring_is_the_channel_nearest_horizontal(self) -> None:
        spec = _spec(channels=3, columns=4, elevation_min_deg=-10.0,
                     elevation_max_deg=50.0)
        indices = scan_ring_indices(spec)
        self.assertEqual(len(indices), spec.columns)
        _, directions, _ = lidar_pattern(spec)
        elevations = np.degrees(np.arcsin(directions[indices, 2]))
        self.assertTrue(np.all(np.abs(elevations) <= 20.0))

    def test_frame_summary_reports_the_capture_span(self) -> None:
        points = np.array([[1.0, 0.0, 0.0], [0.0, 3.0, 0.0]], dtype=np.float32)
        times = np.array([0.0, 0.09])
        summary = frame_summary(points, times)
        self.assertEqual(summary["points"], 2)
        self.assertAlmostEqual(summary["range_min"], 1.0)
        self.assertAlmostEqual(summary["range_max"], 3.0)
        self.assertAlmostEqual(summary["span_s"], 0.09)

    def test_frame_summary_handles_an_empty_frame(self) -> None:
        summary = frame_summary(np.zeros((0, 3), dtype=np.float32), np.zeros(0))
        self.assertEqual(summary["points"], 0)


if __name__ == "__main__":
    unittest.main()


class TestSelfHitMask(unittest.TestCase):
    """Returns landing on the robot's own body.

    Measured against the real robot USD, 9,840 of 19,893 points -- 49.5% of a
    frame -- hit the robot itself, because the raycast sees the inflated
    convex-hull COLLISION geometry rather than the visual shape a real Mid-360
    would. Publishing those would put a solid shell of phantom obstacles at
    ~0.2 m around the robot.
    """

    def test_only_paths_under_the_robot_are_self_hits(self) -> None:
        paths = [
            "/World/Tinker/base_link",
            "/World/Arena/wall_3",
            "/World/Tinker/link4/collisions",
            "/World/Ground",
        ]
        mask = self_hit_mask(paths, np.array([0, 1, 2, 3]), "/World/Tinker")
        self.assertEqual(list(mask), [True, False, True, False])

    def test_only_live_rays_are_examined(self) -> None:
        """The full table is ~19,893 entries per physics step.

        Checking every one in Python would cost more than the raycast; only
        the ~3,300 rays that actually returned a point need testing.
        """
        paths = ["/World/Tinker/a", "/World/Arena/b", "/World/Tinker/c"]
        mask = self_hit_mask(paths, np.array([1, 2]), "/World/Tinker")
        self.assertEqual(list(mask), [False, True])

    def test_no_live_rays_yields_an_empty_mask(self) -> None:
        mask = self_hit_mask([], np.array([], dtype=int), "/World/Tinker")
        self.assertEqual(mask.shape, (0,))
        self.assertEqual(mask.dtype, np.dtype(bool))

    def test_a_similarly_named_sibling_is_not_a_self_hit(self) -> None:
        """A prefix match must not swallow /World/TinkerArena or similar."""
        paths = ["/World/TinkerTable/top"]
        mask = self_hit_mask(paths, np.array([0]), "/World/Tinker/")
        self.assertEqual(list(mask), [False])

    def test_spec_defaults_to_filtering(self) -> None:
        self.assertTrue(load_lidar_spec(CONTRACT).self_filter)

    def test_rejects_non_boolean_self_filter(self) -> None:
        raw = json.loads(CONTRACT.read_text(encoding="utf-8"))
        raw["lidar"]["raycast"]["self_filter"] = "yes"
        directory = tempfile.mkdtemp()
        path = Path(directory) / "contract.json"
        path.write_text(json.dumps(raw), encoding="utf-8")
        with self.assertRaises(LidarSpecError):
            load_lidar_spec(path)
