"""The simulated head must publish its two moving transforms.

``robot_state_publisher`` emits a joint's transform only when it sees that
joint in ``/joint_states``. The head pan and tilt joints are not in
ros2_control -- ``pan_tilt_facade`` drives them over a topic -- so
``joint_state_broadcaster``, the single permitted ``/joint_states``
publisher, never mentions them and RSP never publishes
``base_link -> pan_link`` or ``pan_link -> tilt_link``.

Everything below those joints is fixed, so RSP does publish it, on
``/tf_static``: ``camera_link``, ``head_camera_color_optical_frame`` and the
rest all exist -- as an island. TF says so exactly:

    Could not find a connection between 'map' and 'camera_color_optical_frame'
    because they are not part of the same tree. Tf has two or more
    unconnected trees.

That is what threw away GPSR run16's detections. The scan found the person
(``cls_name 'person', conf 1.0``) and returned ``n_objects: 0``, because the
request asked for ``target_frame: "map"`` and no transform reached it.

A second ``/joint_states`` publisher is not an option: ``pick_and_place``
requires exactly one, and adding another produced 156 "controller manager
observation refresh failed" errors. So the facade, which already owns the
head's state, broadcasts these two transforms itself.
"""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/tinker_sim_bridge"))

from tinker_sim_bridge.head_tf import (  # noqa: E402
    PAN_JOINT,
    TILT_JOINT,
    head_transforms,
)


def _quat_to_matrix(q):
    x, y, z, w = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


class JointConstantsTest(unittest.TestCase):
    """Values are the robot description's, not invented here."""

    def test_pan_joint_matches_the_urdf(self):
        self.assertEqual(PAN_JOINT.parent, "base_link")
        self.assertEqual(PAN_JOINT.child, "pan_link")
        self.assertEqual(PAN_JOINT.axis, (0.0, 0.0, -1.0))
        for got, want in zip(PAN_JOINT.xyz, (-0.310913, 0.00283274, 1.35846)):
            self.assertAlmostEqual(got, want, places=9)

    def test_tilt_joint_matches_the_urdf(self):
        self.assertEqual(TILT_JOINT.parent, "pan_link")
        self.assertEqual(TILT_JOINT.child, "tilt_link")
        self.assertEqual(TILT_JOINT.axis, (0.0, 1.0, 0.0))
        self.assertEqual(TILT_JOINT.xyz, (0.0, 0.0, 0.135))
        self.assertEqual(TILT_JOINT.rpy, (0.0, 0.0, 0.0))


class HeadTransformsTest(unittest.TestCase):
    def test_publishes_exactly_the_two_missing_links(self):
        pan, tilt = head_transforms(0.0, 0.0)
        self.assertEqual((pan.parent, pan.child), ("base_link", "pan_link"))
        self.assertEqual((tilt.parent, tilt.child), ("pan_link", "tilt_link"))

    def test_translations_are_the_joint_origins_and_do_not_move(self):
        """A revolute joint rotates; its origin is fixed."""
        for pan_rad, tilt_rad in ((0.0, 0.0), (1.2, -0.4), (-2.5, 0.9)):
            pan, tilt = head_transforms(pan_rad, tilt_rad)
            for got, want in zip(pan.xyz, PAN_JOINT.xyz):
                self.assertAlmostEqual(got, want, places=12)
            self.assertEqual(tilt.xyz, (0.0, 0.0, 0.135))

    def test_quaternions_are_unit(self):
        for pan_rad, tilt_rad in ((0.0, 0.0), (1.5, -0.5), (-3.0, 1.4)):
            for tf in head_transforms(pan_rad, tilt_rad):
                self.assertAlmostEqual(
                    math.fsum(v * v for v in tf.quaternion_xyzw), 1.0, places=12
                )

    def test_tilt_rotates_about_the_joint_axis(self):
        """+tilt turns the child's +X toward -Z, per axis (0, 1, 0)."""
        _, tilt = head_transforms(0.0, math.radians(90.0))
        matrix = _quat_to_matrix(tilt.quaternion_xyzw)
        forward = tuple(matrix[i][0] for i in range(3))
        self.assertAlmostEqual(forward[0], 0.0, places=9)
        self.assertAlmostEqual(forward[2], -1.0, places=9)

    def test_pan_rotates_about_the_negated_z_axis(self):
        """axis is (0, 0, -1), so +pan turns +X toward -Y."""
        pan, _ = head_transforms(math.radians(90.0), 0.0)
        # strip the joint origin's fixed rpy to isolate the joint rotation
        neutral = _quat_to_matrix(head_transforms(0.0, 0.0)[0].quaternion_xyzw)
        rotated = _quat_to_matrix(pan.quaternion_xyzw)
        # neutral^T . rotated is the pure joint rotation
        joint = tuple(
            tuple(sum(neutral[k][i] * rotated[k][j] for k in range(3)) for j in range(3))
            for i in range(3)
        )
        forward = tuple(joint[i][0] for i in range(3))
        self.assertAlmostEqual(forward[0], 0.0, places=9)
        self.assertAlmostEqual(forward[1], -1.0, places=9)

    def test_zero_pose_keeps_the_joint_origin_rpy(self):
        """At 0/0 the transform is the URDF origin, not identity."""
        pan, _ = head_transforms(0.0, 0.0)
        self.assertNotAlmostEqual(pan.quaternion_xyzw[3], 1.0, places=6)

    def test_tilt_at_zero_is_identity_rotation(self):
        _, tilt = head_transforms(0.0, 0.0)
        self.assertAlmostEqual(tilt.quaternion_xyzw[3], 1.0, places=12)


if __name__ == "__main__":
    unittest.main()
