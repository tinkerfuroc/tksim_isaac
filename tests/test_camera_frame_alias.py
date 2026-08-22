"""Every camera's published frame_id must exist in TF.

The sim stamps head camera images ``camera_color_optical_frame`` -- the name
the real Orbbec driver uses -- while the robot description calls that link
``head_camera_color_optical_frame``. Nothing bridged the two, so the frame
the images advertise did not exist in TF at all.

That is invisible until a consumer asks for a detection in a world frame.
GPSR run16's scans did detect the person (the request log records
``bbox [677, 54, 829, 420], cls_name 'person', conf 1.0``) and then threw the
detection away: with ``target_frame: "map"`` the node got ``n_bboxes: 1`` but
``n_objects: 0``, because it could not transform out of a frame TF had never
heard of. The behaviour tree saw only ``no matches for "person"``.

The bridge already solves exactly this for the lidar, and says so: the URDF's
link is ``livox_frame`` while hardware messages and Nav2 use ``livox360``, so
``gpsr.launch.py`` publishes a static transform between them. The head camera
needs the same bridge. The wrist camera needs none -- its frame_id already
matches its URDF link, which is why only one camera was ever broken.
"""
from __future__ import annotations

import json
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "simulation/sensors/hardware-parity.json"
LAUNCH = ROOT / "ros2_ws/src/tinker_sim_bridge/launch/gpsr.launch.py"


def _cameras():
    raw = json.loads(CONTRACT.read_text(encoding="utf-8"))
    return {
        name: spec
        for name, spec in raw.items()
        if isinstance(spec, dict) and spec.get("mount_prim") and spec.get("frame_id")
    }


class FrameAliasTest(unittest.TestCase):
    def setUp(self):
        self.cameras = _cameras()
        self.launch = LAUNCH.read_text(encoding="utf-8")

    def test_the_contract_still_has_both_cameras(self):
        self.assertEqual(set(self.cameras), {"head_camera", "wrist_camera"})

    def test_wrist_camera_needs_no_alias(self):
        """Its frame_id is its URDF link, so nothing has to bridge it."""
        wrist = self.cameras["wrist_camera"]
        self.assertEqual(wrist["frame_id"], wrist["mount_prim"])

    def test_head_camera_frame_id_differs_from_its_urdf_link(self):
        """Recording the mismatch this test exists for."""
        head = self.cameras["head_camera"]
        self.assertNotEqual(head["frame_id"], head["mount_prim"])

    def test_every_mismatched_camera_is_aliased_in_the_launch(self):
        """A published frame_id that TF does not know is a silent data loss."""
        for name, spec in self.cameras.items():
            frame_id, mount = spec["frame_id"], spec["mount_prim"]
            if frame_id == mount:
                continue
            with self.subTest(camera=name):
                self.assertIn(
                    mount, self.launch, f"{name}: URDF link {mount} not referenced"
                )
                self.assertIn(
                    frame_id, self.launch, f"{name}: frame_id {frame_id} not published"
                )
                pattern = (
                    r'"--frame-id",\s*"' + re.escape(mount) + r'"'
                    r',\s*"--child-frame-id",\s*"' + re.escape(frame_id) + r'"'
                )
                self.assertRegex(
                    self.launch.replace("\n", " "),
                    pattern,
                    f"{name}: no static transform {mount} -> {frame_id}",
                )

    def test_the_alias_is_identity(self):
        """The URDF link already carries the optical rotation.

        head_camera_color_optical_frame is the optical frame, so the driver's
        name for it is the same pose under a different label. A rotation here
        would silently tilt every detection.
        """
        head = self.cameras["head_camera"]
        flat = self.launch.replace("\n", " ")
        start = flat.find(head["mount_prim"] + '", "--child-frame-id"')
        if start == -1:
            start = flat.find('"--frame-id", "' + head["mount_prim"])
        self.assertNotEqual(start, -1, "alias node not found")
        window = flat[max(0, start - 400):start + 200]
        for axis in ("--x", "--y", "--z", "--qx", "--qy", "--qz"):
            self.assertRegex(
                window, re.escape(f'"{axis}", "0"'), f"{axis} must be zero in the alias"
            )
        self.assertRegex(window, re.escape('"--qw", "1"'))


if __name__ == "__main__":
    unittest.main()
