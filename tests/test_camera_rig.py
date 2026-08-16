from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.camera_rig import (
    CAMERA_NAMES,
    CameraStreamSpec,
    load_camera_specs,
)

CONTRACT = ROOT / "simulation/sensors/hardware-parity.json"


class LoadCameraSpecsTest(unittest.TestCase):
    def test_loads_committed_contract(self) -> None:
        specs = load_camera_specs(CONTRACT)
        self.assertEqual(tuple(spec.name for spec in specs), CAMERA_NAMES)
        head, wrist = specs
        self.assertEqual(head.color_topic, "/camera/color/image_raw")
        self.assertEqual(head.depth_topic, "/camera/depth/image_raw")
        self.assertEqual(head.camera_info_topics, ("/camera/color/camera_info",))
        self.assertEqual(head.frame_id, "camera_color_optical_frame")
        self.assertEqual(head.mount_prim, "head_camera_color_optical_frame")
        self.assertEqual((head.width, head.height), (1280, 720))
        self.assertEqual(head.horizontal_fov_deg, 90.0)
        self.assertEqual(head.tick_rate_hz, 15.0)
        self.assertEqual(
            wrist.camera_info_topics,
            (
                "/camera/xarm_camera/color/camera_info",
                "/camera/xarm_camera/aligned_depth_to_color/camera_info",
            ),
        )
        self.assertEqual((wrist.width, wrist.height), (848, 480))

    def _mutated(self, mutate) -> Path:
        raw = json.loads(CONTRACT.read_text(encoding="utf-8"))
        mutate(raw)
        handle = tempfile.NamedTemporaryFile(
            "w", suffix=".json", delete=False, encoding="utf-8"
        )
        json.dump(raw, handle)
        handle.close()
        self.addCleanup(Path(handle.name).unlink)
        return Path(handle.name)

    def test_rejects_wrong_schema_version(self) -> None:
        path = self._mutated(lambda raw: raw.update(schema_version=1))
        with self.assertRaisesRegex(ValueError, "schema_version"):
            load_camera_specs(path)

    def test_rejects_best_effort_qos(self) -> None:
        path = self._mutated(
            lambda raw: raw["camera_qos"].update(reliability="best_effort")
        )
        with self.assertRaisesRegex(ValueError, "reliable"):
            load_camera_specs(path)

    def test_rejects_wrong_depth_encoding(self) -> None:
        path = self._mutated(
            lambda raw: raw["head_camera"].update(depth_encoding="32FC1")
        )
        with self.assertRaisesRegex(ValueError, "16UC1"):
            load_camera_specs(path)

    def test_rejects_missing_camera(self) -> None:
        path = self._mutated(lambda raw: raw.pop("wrist_camera"))
        with self.assertRaisesRegex(ValueError, "wrist_camera"):
            load_camera_specs(path)

    def test_rejects_nonpositive_dimensions(self) -> None:
        path = self._mutated(lambda raw: raw["head_camera"].update(width=0))
        with self.assertRaisesRegex(ValueError, "positive"):
            load_camera_specs(path)


import math

import numpy as np

from tinker_sim_isaac.camera_rig import (
    HORIZONTAL_APERTURE_MM,
    camera_info_fields,
    depth_to_16uc1_mm,
    focal_from_fov,
    rgb8_array,
)


class OpticsTest(unittest.TestCase):
    def test_focal_matches_fov(self) -> None:
        focal = focal_from_fov(1280, 90.0)
        recovered = 2.0 * math.degrees(
            math.atan(HORIZONTAL_APERTURE_MM / (2.0 * focal))
        )
        self.assertAlmostEqual(recovered, 90.0, places=6)

    def test_camera_info_is_consistent_pinhole(self) -> None:
        head = load_camera_specs(CONTRACT)[0]
        fields = camera_info_fields(head)
        fx = head.width / (2.0 * math.tan(math.radians(head.horizontal_fov_deg) / 2))
        self.assertEqual((fields["height"], fields["width"]), (720, 1280))
        self.assertEqual(fields["distortion_model"], "plumb_bob")
        self.assertEqual(fields["d"], [0.0] * 5)
        self.assertAlmostEqual(fields["k"][0], fx, places=6)
        self.assertAlmostEqual(fields["k"][4], fx, places=6)  # square pixels
        self.assertAlmostEqual(fields["k"][2], 640.0)
        self.assertAlmostEqual(fields["k"][5], 360.0)
        self.assertEqual(fields["r"], [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.assertAlmostEqual(fields["p"][0], fx, places=6)
        self.assertEqual(len(fields["p"]), 12)


class DepthConversionTest(unittest.TestCase):
    def test_metres_become_rounded_millimetres(self) -> None:
        depth = np.array([[0.5, 1.2345]], dtype=np.float32)
        result = depth_to_16uc1_mm(depth)
        self.assertEqual(result.dtype, np.uint16)
        self.assertEqual(result.tolist(), [[500, 1234]])  # 1234.5 rounds to even

    def test_invalid_values_become_zero(self) -> None:
        depth = np.array([[np.nan, np.inf, -1.0, 0.0]], dtype=np.float32)
        self.assertEqual(depth_to_16uc1_mm(depth).tolist(), [[0, 0, 0, 0]])

    def test_clamps_to_uint16(self) -> None:
        self.assertEqual(depth_to_16uc1_mm(np.array([[70.0]])).tolist(), [[65535]])

    def test_squeezes_trailing_channel(self) -> None:
        depth = np.ones((2, 3, 1), dtype=np.float32)
        self.assertEqual(depth_to_16uc1_mm(depth).shape, (2, 3))

    def test_rejects_wrong_rank(self) -> None:
        with self.assertRaisesRegex(ValueError, "depth"):
            depth_to_16uc1_mm(np.ones(4, dtype=np.float32))


class Rgb8ArrayTest(unittest.TestCase):
    def test_strips_alpha_and_batch(self) -> None:
        frame = np.zeros((1, 2, 3, 4), dtype=np.uint8)
        result = rgb8_array(frame, 2, 3)
        self.assertEqual(result.shape, (2, 3, 3))
        self.assertTrue(result.flags["C_CONTIGUOUS"])

    def test_scales_unit_floats(self) -> None:
        frame = np.ones((2, 3, 3), dtype=np.float32)
        self.assertEqual(int(rgb8_array(frame, 2, 3).max()), 255)

    def test_rejects_wrong_resolution(self) -> None:
        with self.assertRaisesRegex(ValueError, "shape"):
            rgb8_array(np.zeros((4, 4, 3), dtype=np.uint8), 2, 3)


from tinker_sim_isaac.camera_rig import pack_registered_cloud


class PackRegisteredCloudTest(unittest.TestCase):
    def test_unprojects_with_intrinsics(self) -> None:
        depth = np.array([[2.0, np.nan]], dtype=np.float32)
        data = pack_registered_cloud(depth, fx=100.0, fy=100.0, cx=0.5, cy=0.0)
        self.assertEqual(len(data), 2 * 16)
        cloud = np.frombuffer(data, dtype=np.float32).reshape(2, 4)
        # pixel (u=0, v=0): x = (0 - 0.5)/100 * 2 = -0.01, y = 0, z = 2
        np.testing.assert_allclose(cloud[0, :3], [-0.01, 0.0, 2.0], atol=1e-6)
        self.assertEqual(cloud[0, 3], 0.0)  # 4-byte pad
        self.assertTrue(np.isnan(cloud[1, :3]).all())  # invalid depth -> NaN

    def test_rejects_wrong_rank(self) -> None:
        with self.assertRaisesRegex(ValueError, "depth"):
            pack_registered_cloud(
                np.ones(3, dtype=np.float32), fx=1.0, fy=1.0, cx=0.0, cy=0.0
            )


if __name__ == "__main__":
    unittest.main()
