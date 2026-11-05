import math
import shutil
from pathlib import Path

import pytest

from map_check.poses import (Pose, PasteError, clashes, export_code, export_yaml, key,
                             parse_pasted, read_map_studio_poses)

REAL_POSES = Path("/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/maps/"
                  "robocup_home_2026_measured_5cm.poses.yaml")

IMPORTS = ("from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion\n"
           "from std_msgs.msg import Header\n"
           "import rclpy\n")

SNIPPET = IMPORTS + (
    "POS_LIVINGROOM_DESTINATION = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),\n"
    "                                         pose=Pose(position=Point(x=1.215, y=1.052, z=0.0),\n"
    "                                                   orientation=Quaternion(x=0.0, y=0.0, z=-0.8980208040998114, w=0.43995299226613754)))\n")

# Generated once by Map Studio's own CODE_IMPORTS + '\n' + pose_code(...).
MAP_STUDIO_CODE = (
    "from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion\nfrom std_msgs.msg import Header\n"
    "import rclpy\n\nPOS_LIVINGROOM_DESTINATION = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), "
    "frame_id='map'),\n                                         pose=Pose(position=Point(x=1.215, y=1.052, z=0.0),\n"
    "                                                   orientation=Quaternion(x=0.0, y=0.0, "
    "z=-0.8980208040998114, w=0.43995299226613754)))\n")


def block(name, x, y, z, w, point=True):
    pt = f"Point(x={x}, y={y}, z=0.0)" if point else "NotAPoint"
    return (f"POS_{name} = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),\n"
            f"    pose=Pose(position={pt},\n"
            f"    orientation=Quaternion(x=0.0, y=0.0, z={z}, w={w})))\n")


def test_read_real_file(tmp_path):
    if not REAL_POSES.is_file():
        pytest.skip("real poses file absent")
    shutil.copy(REAL_POSES, tmp_path / REAL_POSES.name)
    poses = {p.name: p for p in read_map_studio_poses(tmp_path, "robocup_home_2026_measured_5cm")}
    p = poses["livingroom_destination"]
    assert (p.x, p.y, p.yaw) == pytest.approx((1.215, 1.052, -2.2305), abs=1e-9)


def test_read_missing_file(tmp_path):
    assert read_map_studio_poses(tmp_path, "nope") == []


def test_parse_map_studio_snippet():
    poses, errors = parse_pasted(SNIPPET)
    assert errors == []
    assert len(poses) == 1
    p = poses[0]
    assert p.name == "POS_LIVINGROOM_DESTINATION"
    assert (p.x, p.y) == (1.215, 1.052)
    assert p.yaw == pytest.approx(-2.2305, abs=1e-4)


def test_parse_variants():
    crlf = SNIPPET.replace("\n", "\r\n")
    poses, errors = parse_pasted(crlf)
    assert errors == [] and len(poses) == 1

    poses, errors = parse_pasted(SNIPPET[len(IMPORTS):])
    assert errors == [] and len(poses) == 1

    one_line = block("A", 1.0, 2.0, 0.0, 1.0) .replace("\n", " ") + block("B", 3.0, 4.0, 0.0, 1.0).replace("\n", " ")
    poses, errors = parse_pasted(one_line)
    assert errors == [] and [p.name for p in poses] == ["POS_A", "POS_B"]

    poses, errors = parse_pasted(block("C", 0, 0, 0.2, 0.2))
    assert poses[0].yaw == pytest.approx(math.pi / 2)


def test_parse_partial_failure():
    text = block("A", 1, 1, 0.0, 1.0) + block("B", 1, 1, 0.0, 1.0, point=False) + block("C", 2, 2, 0.0, 1.0)
    poses, errors = parse_pasted(text)
    assert [p.name for p in poses] == ["POS_A", "POS_C"]
    assert len(errors) == 1
    assert errors[0].index == 2
    assert errors[0].text.startswith("POS_B")
    assert len(errors[0].text) <= 60


def test_parse_bad_name():
    poses, errors = parse_pasted(block("A", 1, 1, 0.0, 1.0).replace("POS_A", "POS_" + "A" * 70))
    assert poses == [] and len(errors) == 1 and errors[0].index == 1


def test_export_code_matches_map_studio():
    out = export_code([Pose("livingroom_destination", 1.215, 1.052, -2.2305)])
    assert out == MAP_STUDIO_CODE


def test_export_yaml_roundtrip(tmp_path):
    poses = [Pose("a_b", 1.23456, -2.5, 3.14159), Pose("c", 0.0, 4.0, -1.0)]
    text = export_yaml("m1", poses)
    assert text.startswith('map: "m1"\nframe_id: map\nposes:\n  "a_b": {x: 1.2346, y: -2.5000, yaw: 3.1415}\n')
    (tmp_path / "m1.poses.yaml").write_text(text)
    back = read_map_studio_poses(tmp_path, "m1")
    assert [p.name for p in back] == ["a_b", "c"]
    for a, b in zip(poses, back):
        assert (b.x, b.y, b.yaw) == pytest.approx((a.x, a.y, a.yaw), abs=1e-4)


def test_export_yaml_keeps_map_studio_names_verbatim(tmp_path):
    poses = [Pose("TestSpawn", 1.0, 2.0, 0.0), Pose("kitchen_A2", 0.5, 0.5, 0.0)]
    text = export_yaml("m1", poses)
    assert '  "TestSpawn": {x: 1.0000' in text and '  "kitchen_A2": {x: 0.5000' in text
    (tmp_path / "m1.poses.yaml").write_text(text)
    assert [p.name for p in read_map_studio_poses(tmp_path, "m1")] == ["TestSpawn", "kitchen_A2"]


def test_export_yaml_converts_pasted_code_names():
    text = export_yaml("m1", [Pose("POS_DININGROOM_TABLE_2_0", 12.8, 5.45, 0.0),
                              Pose("TestSpawn", 0.0, 0.0, 0.0)])
    assert '  "diningroom_table_2_0": {x: 12.8000' in text
    assert '  "TestSpawn": {x: 0.0000' in text
    assert "POS_" not in text


def test_export_code_unchanged_for_mixed_case_names():
    assert "POS_TESTSPAWN = PoseStamped(" in export_code([Pose("TestSpawn", 0.0, 0.0, 0.0)])


def test_clash_distance():
    saved = [Pose("diningroom_table_2_0", 12.835, 5.45, 0.0)]
    pasted = [Pose("POS_DININGROOM_TABLE_2_0", 12.95, 5.45, 0.0), Pose("POS_OTHER", 0, 0, 0)]
    out = clashes(saved, pasted)
    assert out == {"POS_DININGROOM_TABLE_2_0": pytest.approx(0.115)}


def test_key():
    assert key("POS_Foo") == "foo" and key("foo") == "foo"
