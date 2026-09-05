"""Under ``cam-stand`` the published TF must follow the rendered wrist camera.

The sim renders the wrist camera from xArm's D435 cam-stand bracket while
the artifact URDF still carries the placeholder flange mount
(``tinker_sim_isaac.head_camera_aim``, cam-stand section). A consumer that
deprojects wrist pixels through the URDF's ``xarm_camera_color_optical_frame``
would land ~6 cm off and 90 deg rotated, so every bridge launch publishes
``sim_robot_description``: ``topic_control_description`` plus a rewrite of
``xarm_camera_joint`` keyed on the SAME env value the sim stage reads.
"""
from __future__ import annotations

import math
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_isaac.head_camera_aim import (  # noqa: E402
    CAM_STAND_CAMERA_JOINT,
    CAM_STAND_CORRECTION_WXYZ,
    CAM_STAND_MOUNT_OFFSET_XYZ,
    WRIST_AIM_ENV,
    cam_stand_camera_joint_origin,
    cam_stand_robot_description,
)
from tinker_sim_deploy.runtime import (  # noqa: E402
    sim_robot_description,
    topic_control_description,
)

# The artifact's wrist chain, verbatim from robot.urdf (Intel sensor_d435
# macro layered on link_eef with a placeholder identity origin).
ARTIFACT_WRIST_CHAIN = """
<robot name="tinker_full">
  <link name="world"/>
  <link name="base_link"/>
  <joint name="world_joint" type="fixed">
    <parent link="world"/><child link="base_link"/><origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <link name="link_eef"/>
  <link name="xarm_camera_bottom_screw_frame"/>
  <link name="xarm_camera_link"/>
  <link name="xarm_camera_color_frame"/>
  <link name="xarm_camera_color_optical_frame"/>
  <joint name="xarm_camera_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="link_eef"/>
    <child link="xarm_camera_bottom_screw_frame"/>
  </joint>
  <joint name="xarm_camera_link_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0.010600000000000002 0.0175 0.0125"/>
    <parent link="xarm_camera_bottom_screw_frame"/>
    <child link="xarm_camera_link"/>
  </joint>
  <joint name="xarm_camera_color_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0.015 0"/>
    <parent link="xarm_camera_link"/>
    <child link="xarm_camera_color_frame"/>
  </joint>
  <joint name="xarm_camera_color_optical_joint" type="fixed">
    <origin rpy="-1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0"/>
    <parent link="xarm_camera_color_frame"/>
    <child link="xarm_camera_color_optical_frame"/>
  </joint>
</robot>
"""


def _rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _mm(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _mv(a, v):
    return tuple(sum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def _quat_matrix(q):
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def _fk(urdf: str, parent: str, child: str):
    """(R, t) of *child* in *parent* by walking fixed joints."""
    root = ET.fromstring(urdf)
    joints = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        xyz = tuple(float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split())
        rpy = tuple(float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split())
        joints[joint.find("child").get("link")] = (joint.find("parent").get("link"), xyz, rpy)
    chain = []
    link = child
    while link != parent:
        parent_link, xyz, rpy = joints[link]
        chain.append((xyz, rpy))
        link = parent_link
    rotation = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    translation = (0.0, 0.0, 0.0)
    for xyz, rpy in reversed(chain):
        r = _rpy(*rpy)
        translation = tuple(a + b for a, b in zip(translation, _mv(rotation, xyz)))
        rotation = _mm(rotation, r)
    return rotation, translation


VENDOR_OPTICAL_XYZ = (0.06746, -0.0325, 0.0237)


class RewriteTest(unittest.TestCase):
    def test_rewritten_chain_puts_the_optical_frame_on_the_cam_stand(self):
        rotation, translation = _fk(
            cam_stand_robot_description(ARTIFACT_WRIST_CHAIN),
            "link_eef",
            "xarm_camera_color_optical_frame",
        )
        for got, want in zip(translation, VENDOR_OPTICAL_XYZ):
            self.assertAlmostEqual(got, want, places=9)
        # Optical +Z (view) along the tool axis, -Y (image up) radially out.
        for got, want in zip(_mv(rotation, (0, 0, 1)), (0.0, 0.0, 1.0)):
            self.assertAlmostEqual(got, want, places=9)
        for got, want in zip(_mv(rotation, (0, -1, 0)), (1.0, 0.0, 0.0)):
            self.assertAlmostEqual(got, want, places=9)

    def test_tf_pose_equals_the_render_pose(self):
        """TF (URDF rewrite) and render (preset offset + rotation) are two
        expressions of one pose; they must agree exactly."""
        r_art, t_art = _fk(ARTIFACT_WRIST_CHAIN, "link_eef", "xarm_camera_color_optical_frame")
        render_t = tuple(a + b for a, b in zip(t_art, _mv(r_art, CAM_STAND_MOUNT_OFFSET_XYZ)))
        render_r = _mm(r_art, _quat_matrix(CAM_STAND_CORRECTION_WXYZ))
        tf_r, tf_t = _fk(
            cam_stand_robot_description(ARTIFACT_WRIST_CHAIN),
            "link_eef",
            "xarm_camera_color_optical_frame",
        )
        for got, want in zip(render_t, tf_t):
            self.assertAlmostEqual(got, want, places=9)
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(render_r[i][j], tf_r[i][j], places=9)

    def test_only_the_placeholder_joint_changes(self):
        before = ET.fromstring(ARTIFACT_WRIST_CHAIN)
        after = ET.fromstring(cam_stand_robot_description(ARTIFACT_WRIST_CHAIN))
        for b, a in zip(before.findall("joint"), after.findall("joint")):
            self.assertEqual(b.get("name"), a.get("name"))
            if b.get("name") == CAM_STAND_CAMERA_JOINT:
                continue
            self.assertEqual(
                ET.tostring(b, encoding="unicode"), ET.tostring(a, encoding="unicode")
            )

    def test_joint_origin_is_the_documented_bracket_pose(self):
        xyz, rpy = cam_stand_camera_joint_origin()
        # vendor camera_link (0.06746, -0.0175, 0.0237) pulled back by the
        # Intel bottom-screw offset expressed in the bracket's axes.
        for got, want in zip(xyz, (0.05496, 0.0, 0.0131)):
            self.assertAlmostEqual(got, want, places=9)
        self.assertEqual(rpy, (math.pi, -math.pi / 2, 0.0))

    def test_a_description_without_the_joint_is_refused(self):
        with self.assertRaises(ValueError):
            cam_stand_robot_description('<robot name="x"><link name="link_eef"/></robot>')


class SimRobotDescriptionTest(unittest.TestCase):
    def test_unset_env_is_plain_topic_control(self):
        self.assertEqual(
            sim_robot_description(ARTIFACT_WRIST_CHAIN, environ={}),
            topic_control_description(ARTIFACT_WRIST_CHAIN),
        )

    def test_tool_forward_leaves_tf_alone(self):
        # tool-forward tilts the render in place; the URDF origin is still
        # where that camera sits, so TF stays as-is.
        self.assertEqual(
            sim_robot_description(ARTIFACT_WRIST_CHAIN, environ={WRIST_AIM_ENV: "tool-forward"}),
            topic_control_description(ARTIFACT_WRIST_CHAIN),
        )

    def test_cam_stand_rewrites_the_joint_on_top_of_topic_control(self):
        out = sim_robot_description(ARTIFACT_WRIST_CHAIN, environ={WRIST_AIM_ENV: "cam-stand"})
        self.assertEqual(
            out, cam_stand_robot_description(topic_control_description(ARTIFACT_WRIST_CHAIN))
        )
        _, translation = _fk(out, "link_eef", "xarm_camera_color_optical_frame")
        for got, want in zip(translation, VENDOR_OPTICAL_XYZ):
            self.assertAlmostEqual(got, want, places=9)

    def test_the_gpsr_stack_hands_both_stages_the_same_value(self):
        source = (ROOT / "scripts/gpsr-stack").read_text(encoding="utf-8")
        self.assertIn('WRIST_CAMERA_AIM = "cam-stand"', source)
        self.assertEqual(source.count('"TINKER_SIM_WRIST_CAMERA_AIM": WRIST_CAMERA_AIM'), 2)


if __name__ == "__main__":
    unittest.main()
