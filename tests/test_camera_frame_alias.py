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

    def test_the_alias_follows_the_rendered_head_camera(self):
        """Identity under hardware parity; the render's correction under the aim override.

        head_camera_color_optical_frame is the optical frame, so without
        TINKER_SIM_HEAD_CAMERA_AIM the driver's name for it is the same pose
        under a different label and a rotation here would tilt every
        detection. With the override the sim RENDERS the head camera rotated
        by the correction (head_camera_aim.apply_head_aim_correction), so the
        alias must carry the same rotation -- 2026-10-01: an identity alias
        under level-forward put the optical axis straight up in TF and every
        map-frame person detection at z = 2-5 m.
        """
        head = self.cameras["head_camera"]
        flat = self.launch.replace("\n", " ")
        start = flat.find('name="head_camera_optical_alias_tf"')
        self.assertNotEqual(start, -1, "alias node not found")
        window = flat[start:start + 600]
        self.assertIn('"--frame-id", "' + head["mount_prim"] + '"', window)
        for axis, src in (("--x", "_HEAD_ALIAS_XYZ[0]"), ("--y", "_HEAD_ALIAS_XYZ[1]"),
                          ("--z", "_HEAD_ALIAS_XYZ[2]"), ("--qx", "_HEAD_ALIAS_Q[0]"),
                          ("--qy", "_HEAD_ALIAS_Q[1]"), ("--qz", "_HEAD_ALIAS_Q[2]"),
                          ("--qw", "_HEAD_ALIAS_Q[3]")):
            self.assertIn(f'"{axis}", repr({src})', window)
        self.assertIn("head_camera_optical_alias(", self.launch)
        self.assertIn('os.environ.get("TINKER_SIM_HEAD_CAMERA_AIM")', self.launch)

        from tinker_sim_isaac.head_camera_aim import head_camera_optical_alias
        self.assertEqual(head_camera_optical_alias(None), ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))

if __name__ == "__main__":
    unittest.main()
