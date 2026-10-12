"""base_footprint: a floor-level child of base_link for floor-referenced heights.

The sim base_link rests 0.0775 m above the floor (wheel bottoms at z -0.0775
in base_link), and the 2-D EKF publishes odom -> base_link with z = 0, so a
map-frame z is a height above the base_link plane, not above the floor. The
frame tree that localization and Nav2 run on stays as it is; consumers that
need floor height look up base_footprint instead of carrying a constant.
"""

import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.runtime import (  # noqa: E402
    resolve_current_artifact,
    sim_robot_description,
    with_base_footprint,
)

WHEELED = """
<robot name="r">
  <link name="base_link"/>
  <link name="front_left_wheel"><collision><origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <geometry><cylinder length="0.063" radius="0.0525"/></geometry></collision></link>
  <joint name="front_left_wheel_joint" type="continuous"><parent link="base_link"/>
    <child link="front_left_wheel"/><origin xyz="0 0.125 -0.025" rpy="0 0 0"/></joint>
  <link name="rear_left_swivel"/>
  <joint name="rear_left_swivel_joint" type="continuous"><parent link="base_link"/>
    <child link="rear_left_swivel"/><origin xyz="-0.25 0.125 -0.0475" rpy="0 0 0"/></joint>
  <link name="rear_left_wheel"><collision><origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <geometry><cylinder length="0.025" radius="0.03"/></geometry></collision></link>
  <joint name="rear_left_wheel_joint" type="continuous"><parent link="rear_left_swivel"/>
    <child link="rear_left_wheel"/><origin xyz="-0.025 0 0" rpy="0 0 0"/></joint>
</robot>
"""


def _footprint_joint(description: str) -> ET.Element:
    root = ET.fromstring(description)
    joints = [j for j in root.findall("joint") if j.find("child").get("link") == "base_footprint"]
    links = [link for link in root.findall("link") if link.get("name") == "base_footprint"]
    assert len(joints) == 1 and len(links) == 1, (len(joints), len(links))
    return joints[0]


class WithBaseFootprintTest(unittest.TestCase):
    def test_footprint_sits_at_the_wheel_bottoms(self):
        joint = _footprint_joint(with_base_footprint(WHEELED))
        self.assertEqual(joint.get("type"), "fixed")
        self.assertEqual(joint.find("parent").get("link"), "base_link")
        x, y, z = (float(v) for v in joint.find("origin").get("xyz").split())
        self.assertEqual((x, y), (0.0, 0.0))
        self.assertAlmostEqual(z, -0.0775, places=6)

    def test_wheels_that_disagree_on_the_floor_are_an_error(self):
        tilted = WHEELED.replace('xyz="-0.25 0.125 -0.0475"', 'xyz="-0.25 0.125 -0.025"')
        with self.assertRaisesRegex(ValueError, "wheel bottoms disagree"):
            with_base_footprint(tilted)

    def test_a_description_without_wheels_is_left_alone(self):
        plain = '<robot name="x"><link name="base_link"/></robot>'
        self.assertEqual(with_base_footprint(plain), plain)

    def test_an_existing_base_footprint_is_left_alone_when_already_derived(self):
        once = with_base_footprint(WHEELED)
        self.assertEqual(with_base_footprint(once), once)

    def test_sim_moves_a_hardware_base_footprint_to_the_derived_height(self):
        # The hardware xacro (shared with the sim's tinker_full) carries
        # base_footprint at a placeholder -0.30; in sim the wheels decide.
        hardware = WHEELED.replace("</robot>", (
            '  <link name="base_footprint"/>\n'
            '  <joint name="base_footprint_joint" type="fixed"><parent link="base_link"/>'
            '<child link="base_footprint"/><origin xyz="0 0 -0.30" rpy="0 0 0"/></joint>\n'
            "</robot>"))
        joint = _footprint_joint(with_base_footprint(hardware))
        self.assertAlmostEqual(float(joint.find("origin").get("xyz").split()[2]), -0.0775, places=6)


class SimDescriptionsCarryTheFootprintTest(unittest.TestCase):
    def test_the_current_artifact_gets_base_footprint_at_its_rest_height(self):
        urdf = resolve_current_artifact(ROOT).robot_urdf
        joint = _footprint_joint(sim_robot_description(urdf, environ={}))
        z = float(joint.find("origin").get("xyz").split()[2])
        self.assertAlmostEqual(z, -0.0775, places=4)

    def test_navigation_launch_publishes_it_too(self):
        # navigation.launch.py feeds robot_state_publisher the artifact URDF
        # directly rather than through sim_robot_description.
        source = (ROOT / "ros2_ws/src/tinker_sim_bridge/launch/navigation.launch.py").read_text(
            encoding="utf-8")
        self.assertIn("with_base_footprint(", source)


if __name__ == "__main__":
    unittest.main()
