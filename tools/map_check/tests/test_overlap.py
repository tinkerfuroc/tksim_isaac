import math
import time
from pathlib import Path

import numpy as np
import pytest

from map_check import robot
from map_check.arena_build import Box, build_boxes, default_settings
from map_check.config import Settings
from map_check.occupancy import closed_outlines, load_map_dir
from map_check.overlap import Flag, RobotModel, arena_bounds, check_pose, part_name
from map_check.poses import Pose

MAP = Path("/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/maps/"
           "robocup_home_2026_measured_5cm.yaml")
ARM_NAV = [-86.0, -52.0, 18.0, 2.0, 34.0, -73.0, -9.0]  # arm_pos_navigating
ARM_ZERO = [0.0] * 7  # gripper reaches past the base front


@pytest.fixture
def model(real_root, tmp_path_factory):
    pointer = real_root / "artifacts" / "robot" / "tinker2" / "current.json"
    if not pointer.is_file():
        pytest.skip(f"robot artifact absent: {pointer}")
    files = robot.resolve_robot(pointer)
    # link_points caches by the USD's sha256, so one session dir serves every test.
    return RobotModel(files.urdf, files.usd, tmp_path_factory.getbasetemp() / "points")


def wall(x0, x1, y0, y1, h=0.6):
    return Box("wall", None, x0, x1, y0, y1, 0.0, h)


def framed(*boxes):
    """Boxes plus four walls 1.5 m out, so the origin is inside the arena."""
    return list(boxes) + [wall(-1.6, -1.5, -1.6, 1.6), wall(1.5, 1.6, -1.6, 1.6),
                          wall(-1.6, 1.6, -1.6, -1.5), wall(-1.6, 1.6, 1.5, 1.6)]


def world_points(model, arm):
    """Per-link points of the robot at the origin, yaw 0."""
    fk = model.fk(arm, 0.0, 0.0)
    return {k: p @ fk[k][:3, :3].T + fk[k][:3, 3] + [0.0, 0.0, model.z0]
            for k, p in model.points.items()}


def check(model, x, y, yaw, arm, boxes):
    return model.flags(Pose("p", x, y, yaw), arm, 0.0, 0.0, boxes, arena_bounds(boxes))


def assert_well_formed(flags):
    parts = [f.part for f in flags]
    assert len(parts) == len(set(parts))
    assert [f.distance_m for f in flags] == sorted(f.distance_m for f in flags)


def test_part_name():
    assert part_name("base_link") == "base"
    assert part_name("front_left_wheel") == "base"
    assert part_name("rear_right_swivel") == "base"
    assert part_name("ballast") == "base"
    assert part_name("livox_frame") == "base"
    assert part_name("link3") == "arm (link3)"
    assert part_name("link_base") == "arm (link_base)"
    assert part_name("xarm_gripper_base_link") == "gripper"
    assert part_name("left_finger") == "gripper"
    assert part_name("right_inner_knuckle") == "gripper"
    assert part_name("xarm_camera_link") == "gripper"
    assert part_name("pan_link") == "head"
    assert part_name("camera_link") == "head"
    assert part_name("head_camera_base_link") == "head"
    assert part_name("mystery") == "mystery"


def test_part_name_covers_real_links(model):
    for link in model.points:
        name = part_name(link)
        assert name in ("base", "gripper", "head") or name.startswith("arm ("), link


def test_arena_bounds():
    assert arena_bounds([wall(0, 1, 2, 3), wall(-1, 0.5, 2.5, 4)]) == (-1, 1, 2, 4)


def test_clear_pose(model):
    pts = np.concatenate(list(world_points(model, ARM_NAV).values()))
    assert np.abs(pts[:, :2]).max() < 0.5  # every frame wall is at least 1 m away
    assert check(model, 0.0, 0.0, 0.0, ARM_NAV, framed()) == []


def test_base_in_wall(model):
    flags = check(model, 0.0, 0.0, 0.0, ARM_NAV, framed(wall(0.1, 0.3, -0.3, 0.3)))
    assert_well_formed(flags)
    base = [f for f in flags if f.part == "base"]
    assert len(base) == 1
    assert base[0].kind == "overlap" and base[0].target == "wall" and base[0].distance_m < 0
    assert base[0].text.startswith("base overlaps wall by ")


def test_near_margin(model):
    pts = world_points(model, ARM_NAV)
    front = max(p[:, 0].max() for k, p in pts.items() if part_name(k) == "base")
    others = max(p[:, 0].max() for k, p in pts.items() if part_name(k) != "base")
    assert others < front  # the base is the robot's front at this arm pose
    flags = check(model, 0.0, 0.0, 0.0, ARM_NAV, framed(wall(front + 0.01, front + 0.2, -0.3, 0.3)))
    assert len(flags) == 1
    f = flags[0]
    assert (f.kind, f.part, f.target) == ("near", "base", "wall")
    assert f.distance_m == pytest.approx(0.01, abs=1e-6)
    assert f.text == "base 1 cm from wall (under 2 cm)"


def test_arm_over_low_shape(model):
    pts = world_points(model, ARM_ZERO)
    front = max(p[:, 0].max() for k, p in pts.items() if part_name(k) == "base")
    x0, x1 = front + 0.03, front + 0.5
    over = np.concatenate([p[p[:, 0] >= x0] for k, p in pts.items() if part_name(k) != "base"])
    assert len(over) > 0 and over[:, 2].min() > 0.32  # arm reaches over the shape, above 0.32 m
    low = Box("shape", 7, x0, x1, -0.3, 0.3, 0.0, 0.30)
    assert check(model, 0.0, 0.0, 0.0, ARM_ZERO, framed(low)) == []
    # The same footprint at full height catches the arm.
    tall = Box("shape", 7, x0, x1, -0.3, 0.3, 0.0, 0.60)
    flags = check(model, 0.0, 0.0, 0.0, ARM_ZERO, framed(tall))
    assert_well_formed(flags)
    assert flags and all(f.target == "shape 7" and f.part != "base" for f in flags)
    assert flags[0].kind == "overlap"


def test_outside(model):
    flags = check(model, 100.0, 100.0, 0.0, ARM_NAV, framed())
    assert flags == [Flag("outside", "base", "arena", flags[0].distance_m, "outside the arena")]
    assert flags[0].distance_m > 0


def test_on_wall_cell(model):
    flags = check(model, 1.55, 0.0, 0.3, ARM_NAV, framed())
    assert_well_formed(flags)
    assert any(f.kind == "overlap" and f.target == "wall" for f in flags)
    assert all(f.kind != "outside" for f in flags)


def test_check_pose_text_rounding():
    """One point, one box: N is the rounded whole cm, at least 1."""
    pts = {"link3": np.array([[0.0, 0.0, 0.0]])}
    fk = {"link3": np.eye(4)}
    bounds = (-5.0, 5.0, -5.0, 5.0)
    pose = Pose("p", 0.0, 0.0, 0.0)

    def one(box):
        return check_pose(pose, [box, wall(-5, -4.9, -5, 5)], bounds, pts, fk, 0.3)

    [f] = one(Box("shape", 3, -0.1, 0.1, -0.1, 0.1, 0.0, 0.33))  # 3 cm deep
    assert (f.kind, f.part, f.target) == ("overlap", "arm (link3)", "shape 3")
    assert f.distance_m == pytest.approx(-0.03)
    assert f.text == "arm (link3) overlaps shape 3 by 3 cm"
    [f] = one(Box("shape", 3, -0.1, 0.1, -0.1, 0.1, 0.0, 0.302))  # 2 mm deep
    assert f.text == "arm (link3) overlaps shape 3 by 1 cm"
    [f] = one(wall(0.0, 0.1, -0.1, 0.1, 0.3))  # touching
    assert f.kind == "near" and f.text == "arm (link3) 1 cm from wall (under 2 cm)"
    assert one(wall(0.02, 0.1, -0.1, 0.1)) == []  # exactly the margin is clear


@pytest.fixture
def real_map():
    if not MAP.is_file():
        pytest.skip("measured map absent")
    m = load_map_dir(MAP)
    labels, outlines = closed_outlines(m)
    return build_boxes(m, labels, default_settings(m, outlines))


@pytest.fixture
def arm_nav():
    path = Settings().arm_poses_json
    if not path.is_file():
        pytest.skip(f"arm poses file absent: {path}")
    return robot.arm_poses(path)["arm_pos_navigating"]


def test_real_dining_pose(model, real_map, arm_nav):
    bounds = arena_bounds(real_map)
    pose = Pose("diningroom_table_2_0", 12.835, 5.45, -math.pi / 2)
    flags = model.flags(pose, arm_nav, 0.0, 0.0, real_map, bounds)
    assert [f for f in flags if f.kind == "overlap"] == []
    east = Pose("diningroom_table_2_0", 13.30, 5.45, -math.pi / 2)
    flags = model.flags(east, arm_nav, 0.0, 0.0, real_map, bounds)
    assert any(f.part == "base" and f.kind in ("near", "overlap") for f in flags)


def test_real_check_pose_speed(model, real_map, arm_nav):
    fk = model.fk(arm_nav, 0.0, 0.0)
    pts, z0 = model.points, model.z0
    bounds = arena_bounds(real_map)
    pose = Pose("diningroom_table_2_0", 13.30, 5.45, -math.pi / 2)
    t = time.perf_counter()
    check_pose(pose, real_map, bounds, pts, fk, z0)
    assert time.perf_counter() - t < 1.0
