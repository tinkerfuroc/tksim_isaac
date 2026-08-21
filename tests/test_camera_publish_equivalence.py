"""Equivalence tests: optimized camera conversions vs. the original reference.

``camera_rig.rgb8_array``/``depth_to_16uc1_mm`` were rewritten to reuse
scratch buffers instead of allocating fresh temporaries every frame (see
``simulation/tinker_sim_isaac/camera_rig.py``). These tests pin that the
optimized functions produce byte-for-byte identical output, dtype, and shape
to ``_rgb8_array_reference``/``_depth_to_16uc1_mm_reference`` (the original
implementations, kept verbatim under those names) across the resolutions and
edge cases the real camera streams can present, including the exact
``array.array('B', x.tobytes())`` bytes handed to rclpy.
"""

from __future__ import annotations

import array
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.camera_rig import (  # noqa: E402
    _depth_to_16uc1_mm_reference,
    _rgb8_array_reference,
    depth_to_16uc1_mm,
    rgb8_array,
)

#: (width, height) for head_camera and wrist_camera, from hardware-parity.json.
RESOLUTIONS = ((1280, 720), (848, 480))


def _assert_identical(case: unittest.TestCase, reference: np.ndarray, optimized: np.ndarray) -> None:
    case.assertEqual(optimized.dtype, reference.dtype)
    case.assertEqual(optimized.shape, reference.shape)
    case.assertTrue(
        np.array_equal(optimized, reference),
        msg="value mismatch between optimized and reference conversion",
    )
    case.assertEqual(bytes(optimized.tobytes()), bytes(reference.tobytes()))
    # This is the exact idiom ros_gateway.publish_cameras uses to fill
    # Image.data; pin it end-to-end, not just the ndarray bytes.
    case.assertEqual(
        array.array("B", optimized.tobytes()),
        array.array("B", reference.tobytes()),
    )


class RgbEquivalenceTest(unittest.TestCase):
    def _check(self, frame: np.ndarray, height: int, width: int) -> None:
        reference = _rgb8_array_reference(frame, height, width)
        optimized = rgb8_array(frame, height, width)
        _assert_identical(self, reference, optimized)

    def test_uint8_rgba_strided_slice(self) -> None:
        rng = np.random.default_rng(1)
        for width, height in RESOLUTIONS:
            frame = rng.integers(0, 256, size=(height, width, 4), dtype=np.uint8)
            self._check(frame, height, width)

    def test_uint8_rgba_batched(self) -> None:
        rng = np.random.default_rng(2)
        for width, height in RESOLUTIONS:
            frame = rng.integers(0, 256, size=(1, height, width, 4), dtype=np.uint8)
            self._check(frame, height, width)

    def test_uint8_rgb_exact(self) -> None:
        rng = np.random.default_rng(3)
        for width, height in RESOLUTIONS:
            frame = rng.integers(0, 256, size=(height, width, 3), dtype=np.uint8)
            self._check(frame, height, width)

    def test_float32_unit_range(self) -> None:
        rng = np.random.default_rng(4)
        for width, height in RESOLUTIONS:
            frame = rng.random(size=(height, width, 3)).astype(np.float32)
            self._check(frame, height, width)

    def test_float32_0_to_255_range(self) -> None:
        rng = np.random.default_rng(5)
        for width, height in RESOLUTIONS:
            frame = (rng.random(size=(height, width, 3)) * 255.0).astype(np.float32)
            self._check(frame, height, width)

    def test_float_exact_half_boundaries(self) -> None:
        for width, height in RESOLUTIONS:
            frame = np.full((height, width, 4), 127.5, dtype=np.float32)
            frame[0, 0] = (254.5, 0.5, 0.0, 9.0)
            self._check(frame, height, width)

    def test_float_all_zero_uses_unit_scale(self) -> None:
        for width, height in RESOLUTIONS:
            frame = np.zeros((height, width, 3), dtype=np.float32)
            self._check(frame, height, width)

    def test_non_finite_rgb_raises_for_both(self) -> None:
        for width, height in RESOLUTIONS:
            frame = np.zeros((height, width, 3), dtype=np.float32)
            frame[0, 0, 0] = np.nan
            with self.assertRaises(ValueError):
                _rgb8_array_reference(frame, height, width)
            with self.assertRaises(ValueError):
                rgb8_array(frame, height, width)

            frame2 = np.zeros((height, width, 3), dtype=np.float32)
            frame2[-1, -1, -1] = np.inf
            with self.assertRaises(ValueError):
                _rgb8_array_reference(frame2, height, width)
            with self.assertRaises(ValueError):
                rgb8_array(frame2, height, width)

    def test_repeated_calls_reuse_buffers_without_stale_data(self) -> None:
        # Guards against scratch-buffer reuse leaking a previous frame's
        # values into a later frame at the same resolution.
        for width, height in RESOLUTIONS:
            first = np.full((height, width, 3), 200, dtype=np.uint8)
            second = np.zeros((height, width, 3), dtype=np.uint8)
            rgb8_array(first, height, width)
            optimized_second = rgb8_array(second, height, width)
            reference_second = _rgb8_array_reference(second, height, width)
            _assert_identical(self, reference_second, optimized_second)

    def test_alternating_resolutions_do_not_cross_contaminate(self) -> None:
        rng = np.random.default_rng(6)
        (w0, h0), (w1, h1) = RESOLUTIONS
        frame0 = rng.integers(0, 256, size=(h0, w0, 4), dtype=np.uint8)
        frame1 = rng.integers(0, 256, size=(h1, w1, 4), dtype=np.uint8)
        for _ in range(3):
            self._check(frame0, h0, w0)
            self._check(frame1, h1, w1)


class DepthEquivalenceTest(unittest.TestCase):
    def _check(self, frame: np.ndarray) -> None:
        reference = _depth_to_16uc1_mm_reference(frame)
        optimized = depth_to_16uc1_mm(frame)
        _assert_identical(self, reference, optimized)

    def test_random_float32_with_special_values(self) -> None:
        rng = np.random.default_rng(11)
        for width, height in RESOLUTIONS:
            depth = rng.random(size=(height, width)).astype(np.float32) * 10.0
            depth[0, 0] = np.nan
            depth[0, 1] = np.inf
            depth[0, 2] = -np.inf
            depth[0, 3] = -1.0
            depth[0, 4] = 0.0
            depth[0, 5] = 70.0  # > 65.535 m: must clamp to 65535
            depth[0, 6] = 65.535  # exact boundary
            depth[0, 7] = 65.5355  # rounds to 65536 pre-clamp -> clamp
            self._check(depth)

    def test_hw1_channel_squeeze(self) -> None:
        rng = np.random.default_rng(12)
        for width, height in RESOLUTIONS:
            depth = rng.random(size=(height, width, 1)).astype(np.float32) * 5.0
            self._check(depth)

    def test_float64_input(self) -> None:
        rng = np.random.default_rng(13)
        for width, height in RESOLUTIONS:
            depth = rng.random(size=(height, width)).astype(np.float64) * 20.0
            depth[1, 1] = np.nan
            depth[1, 2] = np.inf
            self._check(depth)

    def test_integer_input_upcasts(self) -> None:
        rng = np.random.default_rng(14)
        for width, height in RESOLUTIONS:
            depth = rng.integers(-5, 200, size=(height, width), dtype=np.int32)
            self._check(depth)

    def test_uint16_input(self) -> None:
        rng = np.random.default_rng(15)
        for width, height in RESOLUTIONS:
            depth = rng.integers(0, 1000, size=(height, width), dtype=np.uint16)
            self._check(depth)

    def test_half_even_rounding_boundary(self) -> None:
        for width, height in RESOLUTIONS:
            depth = np.zeros((height, width), dtype=np.float32)
            depth[0, 0] = 1.2345  # -> 1234.5mm -> rounds to even (1234)
            depth[0, 1] = 1.2355  # -> 1235.5mm -> rounds to even (1236)
            self._check(depth)

    def test_repeated_calls_reuse_buffers_without_stale_data(self) -> None:
        for width, height in RESOLUTIONS:
            first = np.full((height, width), 40.0, dtype=np.float32)
            second = np.zeros((height, width), dtype=np.float32)
            second[0, 0] = np.nan
            depth_to_16uc1_mm(first)
            optimized_second = depth_to_16uc1_mm(second)
            reference_second = _depth_to_16uc1_mm_reference(second)
            _assert_identical(self, reference_second, optimized_second)

    def test_alternating_resolutions_do_not_cross_contaminate(self) -> None:
        rng = np.random.default_rng(16)
        (w0, h0), (w1, h1) = RESOLUTIONS
        depth0 = rng.random(size=(h0, w0)).astype(np.float32) * 30.0
        depth1 = rng.random(size=(h1, w1)).astype(np.float32) * 30.0
        for _ in range(3):
            self._check(depth0)
            self._check(depth1)

    def test_rejects_wrong_rank_for_both(self) -> None:
        bad = np.ones(4, dtype=np.float32)
        with self.assertRaises(ValueError):
            _depth_to_16uc1_mm_reference(bad)
        with self.assertRaises(ValueError):
            depth_to_16uc1_mm(bad)


if __name__ == "__main__":
    unittest.main()
