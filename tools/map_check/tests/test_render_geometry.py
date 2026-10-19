import importlib.util
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from map_check import robot
from map_check.arena_build import build_boxes, default_settings
from map_check.occupancy import closed_outlines, load_map_dir
from map_check.overlap import arena_bounds
from map_check.poses import Pose, read_map_studio_poses
from map_check.render import cameras, jobfile
from map_check.render.cameras import (CameraSpec, behind_left, cam_to_world, head_cam, overview_top,
                                      project, top_down)
from map_check.render.labels import place_labels

MAPS_DIR = Path("/home/tinker/tk25_ws/src/tk26_navigation/src/navigation_bringup/maps")
MAP_NAME = "robocup_home_2026_measured_5cm"
LABEL_Z = 1.65  # label anchors sit above the head, as in prototypes/render_rooms.py


@pytest.fixture
def robot_files(real_root):
    pointer = real_root / "artifacts" / "robot" / "tinker2" / "current.json"
    if not pointer.is_file():
        pytest.skip(f"robot artifact absent: {pointer}")
    return robot.resolve_robot(pointer)


def test_behind_left_geometry():
    cam = behind_left(Pose("p", 0.0, 0.0, 0.0))
    ex, ey, ez = cam.eye
    assert math.hypot(ex, ey) == pytest.approx(2.2)
    assert math.atan2(ey, ex) == pytest.approx(math.pi - 0.7)
    assert ez == pytest.approx(2.0)
    assert cam.target == pytest.approx((0.5, 0.0, 0.5))
    assert cam.up == (0.0, 0.0, 1.0)
    assert (cam.focal_mm, cam.width, cam.height) == (20.0, 1280, 800)
    assert cam.world_from_cam is None
    # Yaw turns the whole rig around the pose.
    cam = behind_left(Pose("p", 1.0, 2.0, math.pi / 2))
    a = math.pi / 2 + math.pi - 0.7
    assert cam.eye == pytest.approx((1.0 + 2.2 * math.cos(a), 2.0 + 2.2 * math.sin(a), 2.0))
    assert cam.target == pytest.approx((1.0, 2.5, 0.5))


def test_top_down_geometry():
    cam = top_down(Pose("p", 3.0, -1.0, math.pi / 2))
    assert cam.eye == pytest.approx((3.0, -1.0, 4.0))
    assert cam.target == pytest.approx((3.0, -1.0, 0.0))
    assert cam.up == pytest.approx((0.0, 1.0, 0.0))
    assert (cam.focal_mm, cam.width, cam.height) == (24.0, 1280, 800)
    # The robot's heading is image-up and its right side image-right.
    u, v = project(cam, (3.0, 0.0, 0.0))
    assert u == pytest.approx(640) and v < 400
    u, v = project(cam, (4.0, -1.0, 0.0))
    assert u > 640 and v == pytest.approx(400)


def test_overview_top_geometry():
    cam = overview_top((0.0, 14.0, -1.0, 9.0))
    assert (cam.focal_mm, cam.width, cam.height) == (35.0, 2400, 1700)
    h = 1.08 * max(7.0 / (18 / 35), 5.0 / (18 * 1700 / 2400 / 35)) + 1.7
    assert cam.eye == pytest.approx((7.0, 4.0, h))
    assert cam.target == pytest.approx((7.0, 4.0, 0.0))
    # +x is image-right, +y image-up, and the whole arena is in the frame.
    corners = [project(cam, (x, y, 0.0)) for x in (0.0, 14.0) for y in (-1.0, 9.0)]
    assert corners[0][0] < 1200 < corners[2][0]
    assert corners[1][1] < 850 < corners[0][1]
    assert all(0 < u < 2400 and 0 < v < 1700 for u, v in corners)


def test_project_center():
    for cam in (behind_left(Pose("p", 2.0, 3.0, 0.4)), top_down(Pose("p", 2.0, 3.0, 0.4)),
                overview_top((-3.0, 5.0, 0.0, 2.0))):
        assert project(cam, cam.target) == pytest.approx((cam.width / 2, cam.height / 2))


def test_look_at_frame():
    m = cam_to_world(CameraSpec("c", (0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (0.0, 0.0, 1.0), 20.0, 100, 50))
    np.testing.assert_allclose(m[:3, :3].T @ m[:3, :3], np.eye(3), atol=1e-12)
    np.testing.assert_allclose(-m[:3, 2], [1, 0, 0], atol=1e-12)  # looks down -Z
    np.testing.assert_allclose(m[:3, 1], [0, 0, 1], atol=1e-12)  # +Y up
    np.testing.assert_allclose(m[:3, 3], [0, 0, 1], atol=1e-12)


def _head_axes(urdf, pan, tilt, pose=Pose("p", 0.0, 0.0, 0.0), z0=0.0):
    fk = robot.link_transforms(urdf, robot.joint_values([0.0] * 7, pan, tilt))
    cam = head_cam(pose, fk, z0)
    m = cam.world_from_cam
    return cam, fk, -m[:3, 2], m[:3, 1]  # forward (-Z), up (+Y)


def _elev_az(fwd):
    return math.degrees(math.asin(fwd[2])), math.degrees(math.atan2(fwd[1], fwd[0]))


def test_head_aim_constants_match_sim():
    path = Path(__file__).resolve().parents[3] / "simulation" / "tinker_sim_isaac" / "head_camera_aim.py"
    spec = importlib.util.spec_from_file_location("_map_check_head_camera_aim", path)
    sim = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sim)
    assert cameras.LEVEL_FORWARD_CORRECTION_WXYZ == sim.LEVEL_FORWARD_CORRECTION_WXYZ
    assert cameras.LEVEL_FORWARD_VIEW_OFFSET_M == sim.LEVEL_FORWARD_VIEW_OFFSET_M


def test_head_cam_looks_forward(robot_files):
    """Level-forward aim, as gpsr-stack renders it (head_camera_aim.py's table)."""
    urdf = robot_files.urdf
    cam, fk, fwd, up = _head_axes(urdf, 0.0, 0.0)
    assert (cam.width, cam.height) == (1280, 720)
    assert cam.focal_mm == pytest.approx(18.0)  # 36 mm aperture, 90 deg HFOV
    elev, az = _elev_az(fwd)
    assert elev == pytest.approx(0.0, abs=1.0) and az == pytest.approx(0.0, abs=1.0)
    assert fwd[0] > 0
    assert up[2] > 0.99  # level horizon, image-up is world-up
    elev, _ = _elev_az(_head_axes(urdf, 0.0, 30.0)[2])
    assert elev == pytest.approx(-30.0, abs=1.0)
    elev, az = _elev_az(_head_axes(urdf, 45.0, 0.0)[2])
    assert elev == pytest.approx(0.0, abs=1.0) and az == pytest.approx(-45.0, abs=1.0)
    # The render origin sits 3 cm along the view axis from the optical frame.
    np.testing.assert_allclose(np.asarray(cam.eye), fk["head_camera_color_optical_frame"][:3, 3] + 0.03 * fwd,
                               atol=1e-12)
    # Placed at the pose: base_link at (x, y, z0) turned by yaw.
    pose = Pose("p", 2.0, -1.0, 0.8)
    cam, fk, fwd, _ = _head_axes(urdf, 30.0, 10.0, pose, z0=0.0775)
    eye = np.asarray(cam.eye)
    c, s = math.cos(0.8), math.sin(0.8)
    local = fk["head_camera_color_optical_frame"][:3, 3]
    origin = [2.0 + c * local[0] - s * local[1], -1.0 + s * local[0] + c * local[1], 0.0775 + local[2]]
    np.testing.assert_allclose(eye, np.asarray(origin) + 0.03 * fwd, atol=1e-12)
    # The point one metre along the view axis is the image centre.
    assert project(cam, eye + fwd) == pytest.approx((640.0, 360.0))
    assert cam.target == pytest.approx(tuple(eye + fwd))


def test_tilt_down_sign(robot_files):
    """Ruling 2: TILT_DOWN_SIGN is +1 when raising tilt_joint turns the view toward -z."""
    _, _, f0, _ = _head_axes(robot_files.urdf, 0.0, 0.0)
    _, _, f30, _ = _head_axes(robot_files.urdf, 0.0, 30.0)
    dz = f30[2] - f0[2]
    assert abs(dz) > 0.05
    assert cameras.TILT_DOWN_SIGN == (1 if dz < 0 else -1)
    # Same answer in image terms, at either pan: +tilt moves the view toward the
    # image's bottom edge when TILT_DOWN_SIGN is +1.
    for pan in (0.0, 180.0):
        _, _, a, up = _head_axes(robot_files.urdf, pan, 0.0)
        _, _, b, _ = _head_axes(robot_files.urdf, pan, 5.0)
        assert np.sign(np.dot(b - a, up)) == -cameras.TILT_DOWN_SIGN


def _boxes_disjoint(a, b):
    return a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1]


def _check_layout(placed, anchors, w, h):
    assert set(placed) == set(anchors)
    items = list(placed.items())
    for i, (n, (box, anchor)) in enumerate(items):
        assert anchor == anchors[n]
        assert box[0] >= 4 and box[1] >= 48 and box[2] <= w - 4 and box[3] <= h - 4, n
        for m, (other, _) in items[i + 1:]:
            assert _boxes_disjoint(box, other), (n, m)
        for u, v in anchors.values():  # never on top of a robot
            assert _boxes_disjoint(box, (u - 18, v - 18, u + 18, v + 18)), n


def test_labels_facing_first():
    # A narrow label fits 40 px along the facing direction, clear of the robot square.
    placed = place_labels({"a": (500.0, 400.0)}, {"a": (20, 10)}, {"a": 0.0}, 1000, 800)
    box, anchor = placed["a"]
    assert anchor == (500.0, 400.0)
    assert ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2) == pytest.approx((540.0, 400.0))
    assert (box[2] - box[0], box[3] - box[1]) == pytest.approx((32.0, 18.0))  # 6 / 4 px padding
    # Facing up the image (v decreasing), and the header keeps it below 48 px.
    box, _ = place_labels({"a": (500.0, 400.0)}, {"a": (20, 10)}, {"a": -math.pi / 2}, 1000, 800)["a"]
    assert ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2) == pytest.approx((500.0, 360.0))
    box, _ = place_labels({"a": (500.0, 60.0)}, {"a": (20, 10)}, {"a": -math.pi / 2}, 1000, 800)["a"]
    assert box[1] >= 48


def test_labels_crowded_synthetic():
    w, h = 1280, 800
    anchors = {f"p{i}": (600.0 + 30 * (i % 4), 380.0 + 30 * (i // 4)) for i in range(12)}
    sizes = {n: (11 * 18, 24) for n in anchors}
    facing = {n: i * 0.5 for i, n in enumerate(anchors)}
    _check_layout(place_labels(anchors, sizes, facing, w, h), anchors, w, h)


def test_labels_no_overlap():
    if not (MAPS_DIR / f"{MAP_NAME}.yaml").is_file():
        pytest.skip("measured map absent")
    poses = read_map_studio_poses(MAPS_DIR, MAP_NAME)
    assert len(poses) == 26
    m = load_map_dir(MAPS_DIR / f"{MAP_NAME}.yaml")
    labels, outlines = closed_outlines(m)
    cam = overview_top(arena_bounds(build_boxes(m, labels, default_settings(m, outlines))))
    anchors, facing = {}, {}
    for p in poses:
        u, v = anchors[p.name] = project(cam, (p.x, p.y, LABEL_Z))
        fu, fv = project(cam, (p.x + math.cos(p.yaw), p.y + math.sin(p.yaw), LABEL_Z))
        facing[p.name] = math.atan2(fv - v, fu - u)
    sizes = {n: (11 * len(n), 24) for n in anchors}  # stand-in for PIL text sizes
    _check_layout(place_labels(anchors, sizes, facing, cam.width, cam.height), anchors,
                  cam.width, cam.height)


def _job():
    return jobfile.JobFile(
        job_id="j1", map=MAP_NAME, arena_usd="/a/arena.usda", robot_usd="/r/robot.usd",
        robot_urdf="/r/robot.urdf", z0=0.0775, gpu=1, out_dir="/tmp/out",
        arm_deg=[-86.0, -52.0, 18.0, 2.0, 34.0, -73.0, -9.0],
        poses=[jobfile.JobPose(key="kitchen_table_0_0", name="kitchen_table_0_0", x=12.35, y=2.646,
                               yaw=0.0, views=["behind", "head"], pan_deg=180.0, tilt_deg=-30.0),
               jobfile.JobPose(key="a", name="POS_A", x=0.0, y=0.0, yaw=-1.5, views=["top"])],
        overview=True)


def test_jobfile_roundtrip(tmp_path):
    job = _job()
    assert jobfile.JobFile.from_dict(json.loads(json.dumps(job.to_dict()))) == job
    path = tmp_path / "job.json"
    job.write(path)
    assert jobfile.JobFile.read(path) == job
    d = json.loads(path.read_text())
    assert d["poses"][0] == {"key": "kitchen_table_0_0", "name": "kitchen_table_0_0", "x": 12.35,
                             "y": 2.646, "yaw": 0.0, "views": ["behind", "head"], "pan_deg": 180.0,
                             "tilt_deg": -30.0}
    assert d["gpu"] == 1 and d["overview"] is True
    bad = job.to_dict()
    bad["poses"][1]["views"] = ["side"]
    with pytest.raises(ValueError, match="side"):
        jobfile.JobFile.from_dict(bad)
    bad = job.to_dict()
    del bad["z0"]
    with pytest.raises(ValueError, match="z0"):
        jobfile.JobFile.from_dict(bad)


def test_progress_events():
    lines = [jobfile.view_event("a", "behind", "/o/a_behind.png"), jobfile.done_event(),
             jobfile.error_event("GPU lost")]
    assert [json.loads(s) for s in lines] == [
        {"event": "view", "key": "a", "view": "behind", "file": "/o/a_behind.png"},
        {"event": "done"}, {"event": "error", "message": "GPU lost"}]
    assert all("\n" not in s for s in lines)
    assert [jobfile.parse_event(s) for s in lines] == [json.loads(s) for s in lines]
    for noise in ("[Info] [omni.kit] started", "{not json", '{"event": "other"}', "[1, 2]", ""):
        assert jobfile.parse_event(noise) is None


def test_render_modules_import_numpy_only():
    code = ("import sys\n"
            "for m in ('yaml', 'scipy', 'pxr', 'fastapi', 'PIL'): sys.modules[m] = None\n"
            "import map_check.render.cameras, map_check.render.labels, map_check.render.jobfile\n")
    subprocess.run([sys.executable, "-c", code], check=True, cwd=Path(__file__).resolve().parents[1])
