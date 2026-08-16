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


if __name__ == "__main__":
    unittest.main()
