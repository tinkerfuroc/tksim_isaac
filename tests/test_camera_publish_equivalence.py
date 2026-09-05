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

#: warp is installed only under ./.venv (Isaac Sim's Python), not under
#: plain system python; the real-kernel equivalence test below needs it to
#: run the exact CameraRig.capture() kernel (see camera_rig._depth_to_mm_u16_kernel),
#: not a numpy re-implementation, so it must skip cleanly (not fail) when
#: warp isn't importable.
try:
    import warp as wp

    _WARP_AVAILABLE = True
except ImportError:
    wp = None  # type: ignore[assignment]
    _WARP_AVAILABLE = False

#: (width, height) for head_camera and wrist_camera, from hardware-parity.json.
RESOLUTIONS = ((1280, 720), (848, 480))


class _FakePinnedHostArray:
    """Duck-types the ``.numpy()`` surface ``to_numpy()`` relies on.

    ``CameraRig.capture()`` (simulation/tinker_sim_isaac/camera_rig.py) now
    hands ``rgb8_array``/``depth_to_16uc1_mm`` a pinned host ``wp.array``
    instead of the GPU-side array ``to_numpy()`` used to clone itself. A
    real ``wp.array`` already on the CPU makes ``.numpy()`` a zero-copy view
    (see ``warp.array.to``: same-device ``to()`` is a no-op), not a fresh
    clone -- this fake reproduces exactly that surface (a ``.numpy()``
    method returning the backing ndarray, no ``.cpu()`` method) without
    requiring warp/Isaac Sim to be importable, so this equivalence coverage
    runs under plain system Python too.
    """

    def __init__(self, array: np.ndarray) -> None:
        self._array = array

    def numpy(self) -> np.ndarray:
        return self._array


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

    def test_pinned_host_array_input_matches_plain_ndarray(self) -> None:
        # CameraRig.capture()'s pinned-buffer path hands rgb8_array a
        # wp.array-like object (``.numpy()``, no ``.cpu()``) already sized
        # to (height, width, 3) uint8 rather than a bare (H, W, 4) ndarray;
        # both must convert identically.
        rng = np.random.default_rng(7)
        for width, height in RESOLUTIONS:
            frame = rng.integers(0, 256, size=(height, width, 3), dtype=np.uint8)
            reference = _rgb8_array_reference(frame, height, width)
            optimized = rgb8_array(_FakePinnedHostArray(frame), height, width)
            _assert_identical(self, reference, optimized)


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

    def test_uint16_input_is_passthrough_not_reprocessed(self) -> None:
        """A uint16 (H, W) array is now CameraRig.capture()'s depth output
        shape: already converted to 16UC1 mm on the GPU (see
        camera_rig._depth_to_mm_u16_kernel), so depth_to_16uc1_mm detects
        the dtype and returns it unchanged instead of running it back
        through the metres->mm pipeline.

        This is a deliberate divergence from ``_depth_to_16uc1_mm_reference``
        for this one dtype: the reference has no such special case and would
        upcast + multiply by 1000 + clamp like any other non-float input
        (see ``test_integer_input_upcasts``). Do not use ``_check`` here --
        it asserts equality with the reference, which is exactly the
        behavior this test must show does NOT happen any more. See
        ``test_uint16_passthrough_matches_converted_float_frame`` below for
        the passthrough's actual equivalence guarantee.
        """
        rng = np.random.default_rng(15)
        for width, height in RESOLUTIONS:
            depth = rng.integers(0, 1000, size=(height, width), dtype=np.uint16)
            optimized = depth_to_16uc1_mm(depth)
            self.assertEqual(optimized.dtype, np.uint16)
            self.assertEqual(optimized.shape, depth.shape)
            self.assertTrue(np.array_equal(optimized, depth))
            self.assertEqual(bytes(optimized.tobytes()), bytes(depth.tobytes()))
            # Sanity: confirm this genuinely differs from what the
            # reference's generic-numeric-upcast path would have produced,
            # so this test cannot pass "by accident".
            reference = _depth_to_16uc1_mm_reference(depth)
            if np.any(depth != 0):
                self.assertFalse(np.array_equal(optimized, reference))

    def test_uint16_passthrough_matches_converted_float_frame(self) -> None:
        """Simulates CameraRig.capture(): the uint16 array handed to
        depth_to_16uc1_mm is exactly what converting the *original* float
        metres frame would have produced (that is what the GPU kernel
        guarantees, see camera_rig._depth_to_mm_u16_kernel and
        DepthKernelWarpCpuEquivalenceTest below); the passthrough must
        return those same bytes unchanged, i.e. identical to converting the
        source float frame directly.
        """
        rng = np.random.default_rng(21)
        for width, height in RESOLUTIONS:
            depth_metres = rng.random(size=(height, width)).astype(np.float32) * 50.0
            depth_metres[0, 0] = np.nan
            depth_metres[0, 1] = np.inf
            depth_metres[0, 2] = -np.inf
            depth_metres[0, 3] = 0.0
            depth_metres[0, 4] = 70.0
            converted_from_float = depth_to_16uc1_mm(depth_metres)
            passthrough = depth_to_16uc1_mm(converted_from_float)
            self.assertEqual(passthrough.dtype, np.uint16)
            self.assertEqual(passthrough.shape, converted_from_float.shape)
            self.assertTrue(np.array_equal(passthrough, converted_from_float))
            self.assertEqual(
                bytes(passthrough.tobytes()), bytes(converted_from_float.tobytes())
            )

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

    def test_pinned_host_array_input_matches_plain_ndarray(self) -> None:
        # Same pinned-buffer duck type as the RGB case, but for depth: an
        # (H, W) float32 wp.array-like already squeezed to 2D (capture()'s
        # depth pinned buffer has no trailing channel dim to strip).
        rng = np.random.default_rng(17)
        for width, height in RESOLUTIONS:
            depth = rng.random(size=(height, width)).astype(np.float32) * 10.0
            depth[0, 0] = np.nan
            depth[0, 1] = np.inf
            reference = _depth_to_16uc1_mm_reference(depth)
            optimized = depth_to_16uc1_mm(_FakePinnedHostArray(depth))
            _assert_identical(self, reference, optimized)


@unittest.skipUnless(_WARP_AVAILABLE, "warp is not importable under this interpreter")
class DepthKernelWarpCpuEquivalenceTest(unittest.TestCase):
    """Runs the *real* Warp kernel ``CameraRig.capture()`` launches on the
    GPU (``camera_rig._depth_to_mm_u16_kernel()``) on Warp's CPU device
    instead, and checks its output is bit-identical to
    ``_depth_to_16uc1_mm_reference``.

    Deliberately not a numpy re-implementation of the kernel's semantics --
    that would only prove two independent implementations agree with each
    other, not that this exact kernel (the one ``CameraRig`` actually
    launches on CUDA) is correct. Running it on ``device="cpu"`` instead of
    ``"cuda"`` exercises the identical kernel object/logic without needing a
    GPU in this test process; GPU-side byte-identity against real annotator
    frames is covered separately by ``outputs/bench/probe_depth_kernel_final.py``
    (see that script/its JSON for the >=60-real-frame-per-camera result).

    Requires ``warp`` (installed only under ``./.venv``, Isaac Sim's
    Python); skips cleanly under plain system Python.
    """

    @classmethod
    def setUpClass(cls) -> None:
        wp.init()

    def _run_kernel(self, depth: np.ndarray) -> np.ndarray:
        from tinker_sim_isaac.camera_rig import _depth_to_mm_u16_kernel

        flat = np.ascontiguousarray(depth.reshape(-1), dtype=np.float32)
        n = int(flat.shape[0])
        depth_in = wp.array(flat, dtype=wp.float32, device="cpu")
        depth_out = wp.zeros(n, dtype=wp.uint16, device="cpu")
        wp.launch(
            _depth_to_mm_u16_kernel(), dim=n, inputs=[depth_in, depth_out], device="cpu"
        )
        return depth_out.numpy().reshape(depth.shape)

    def _check(self, depth: np.ndarray) -> None:
        expected = _depth_to_16uc1_mm_reference(depth)
        actual = self._run_kernel(depth)
        self.assertEqual(actual.dtype, expected.dtype)
        self.assertEqual(actual.shape, expected.shape)
        self.assertTrue(
            np.array_equal(actual, expected),
            msg="Warp kernel output does not match _depth_to_16uc1_mm_reference",
        )
        self.assertEqual(bytes(actual.tobytes()), bytes(expected.tobytes()))

    def test_edge_value_set(self) -> None:
        # The exact edge set outputs/bench/probe_gpu_depth_rgba.py proved
        # bit-identical on the GPU: NaN, +-Inf, negative, zero, the two
        # tie-to-even boundaries the reference and wp.round disagree on,
        # sub-mm ties, a denormal, and an out-of-range value that must clamp.
        edge_values = np.array(
            [
                float("nan"),
                float("inf"),
                float("-inf"),
                -1.0,
                0.0,
                65.5345,  # *1000 = 65534.5 tie -> even (65534)
                65.5355,  # *1000 = 65535.5 tie -> even (65536) -> clamp 65535
                0.0005,  # *1000 = 0.5 tie -> even (0)
                0.0015,  # *1000 = 1.5 tie -> even (2)
                1e-45,
                70000.0,
                1.234567,
                0.0009999,
            ],
            dtype=np.float32,
        ).reshape(1, -1)
        self._check(edge_values)

    def test_random_frames_both_resolutions(self) -> None:
        rng = np.random.default_rng(101)
        for width, height in RESOLUTIONS:
            depth = (rng.random(size=(height, width)).astype(np.float32) * 80.0) - 5.0
            depth[0, 0] = np.nan
            depth[0, 1] = np.inf
            depth[0, 2] = -np.inf
            depth[0, 3] = 0.0
            depth[0, 4] = 65.535
            depth[0, 5] = 65.5345
            depth[0, 6] = 65.5355
            depth[0, 7] = 70.0
            self._check(depth)


if __name__ == "__main__":
    unittest.main()
