import json
import math
from pathlib import Path

import numpy as np
import pytest

from map_check import robot
from map_check.config import Settings

BASE_LINKS = ("base_link", "front_left_wheel", "rear_right_wheel", "link_base", "link1")


@pytest.fixture
def robot_files(real_root):
    pointer = real_root / "artifacts" / "robot" / "tinker2" / "current.json"
    if not pointer.is_file():
        pytest.skip(f"robot artifact absent: {pointer}")
    return robot.resolve_robot(pointer)


def _authored_link_transforms(usd: Path) -> dict:
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.Open(str(usd))
    root = stage.GetDefaultPrim()
    cache = UsdGeom.XformCache()
    root_inv = cache.GetLocalToWorldTransform(root).GetInverse()
    out = {}
    for prim in root.GetChildren():
        if prim.IsA(UsdGeom.Xform):
            m = cache.GetLocalToWorldTransform(prim) * root_inv
            out[prim.GetName()] = np.array(m).T
    return out


def test_resolve_robot(tmp_path):
    pointer = tmp_path / "artifacts" / "robot" / "tinker2" / "current.json"
    art = tmp_path / "artifacts" / "robot" / "tinker2" / "abc"
    art.mkdir(parents=True)
    (art / "robot.urdf").write_text("<robot/>")
    (art / "robot.usd").write_text("")
    pointer.write_text(json.dumps({"artifact_dir": "artifacts/robot/tinker2/abc"}))
    files = robot.resolve_robot(pointer)
    assert files == robot.RobotFiles(dir=art, urdf=art / "robot.urdf", usd=art / "robot.usd")
    (art / "robot.usd").unlink()
    with pytest.raises(FileNotFoundError):
        robot.resolve_robot(pointer)


def test_arm_poses_source():
    path = Settings().arm_poses_json
    if not path.is_file():
        pytest.skip(f"arm poses file absent: {path}")
    poses = robot.arm_poses(path)
    assert poses["arm_pos_navigating"] == [-86.0, -52.0, 18.0, 2.0, 34.0, -73.0, -9.0]
    assert "arm_pos_orbbec_look" in poses
    assert all(k.startswith("arm_pos_") for k in poses)


def test_arm_poses_order_and_validation(tmp_path):
    path = tmp_path / "constants.json"
    path.write_text('{"arm_pos_b": [1,2,3,4,5,6,7], "other": 1, "arm_pos_a": [0,0,0,0,0,0,0.5]}')
    poses = robot.arm_poses(path)
    assert list(poses) == ["arm_pos_b", "arm_pos_a"]
    assert poses["arm_pos_a"][6] == 0.5
    path.write_text('{"arm_pos_bad": [1, 2]}')
    with pytest.raises(ValueError, match="arm_pos_bad"):
        robot.arm_poses(path)


def test_joint_values():
    q = robot.joint_values([90, 0, 0, 0, 0, 0, -45], 180, -30)
    assert set(q) == {f"joint{i}" for i in range(1, 8)} | {"pan_joint", "tilt_joint"}
    assert q["joint1"] == pytest.approx(math.pi / 2)
    assert q["joint7"] == pytest.approx(-math.pi / 4)
    assert q["pan_joint"] == pytest.approx(math.pi)
    assert q["tilt_joint"] == pytest.approx(-math.pi / 6)
    with pytest.raises(ValueError):
        robot.joint_values([0] * 6, 0, 0)


def test_fk_matches_authored_usd(robot_files):
    fk = robot.link_transforms(robot_files.urdf, {})
    authored = _authored_link_transforms(robot_files.usd)
    common = set(fk) & set(authored)
    assert {"base_link", "link7", "head_camera_color_optical_frame"} <= common
    for name in common:
        assert np.abs(fk[name][:3, 3] - authored[name][:3, 3]).max() < 1e-5, name
        assert np.abs(fk[name][:3, :3] - authored[name][:3, :3]).max() < 1e-5, name


def test_pan_moves_head_camera(robot_files):
    cam = "head_camera_color_optical_frame"
    t0 = robot.link_transforms(robot_files.urdf, robot.joint_values([0] * 7, 0, 0))
    t90 = robot.link_transforms(robot_files.urdf, robot.joint_values([0] * 7, 90, 0))
    pan = t0["pan_link"]
    pivot = pan[:3, 3]
    axis = pan[:3, :3] @ np.array([0.0, 0.0, -1.0])  # pan_joint axis in the URDF
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    rot90 = np.eye(3) + k + k @ k  # Rodrigues, sin=1 cos=0
    p0, p90 = t0[cam][:3, 3], t90[cam][:3, 3]
    assert np.linalg.norm(p90 - p0) > 0.01
    np.testing.assert_allclose(p90 - pivot, rot90 @ (p0 - pivot), atol=1e-9)
    for name in BASE_LINKS:
        np.testing.assert_allclose(t90[name], t0[name], atol=1e-12)


def test_link_points_cover_base(robot_files, tmp_path):
    pts = robot.link_points(robot_files.usd, tmp_path)
    base = pts["base_link"]
    assert base.ndim == 2 and base.shape[1] == 3
    assert base[:, 0].min() == pytest.approx(-0.38, abs=0.01)
    assert base[:, 1].min() == pytest.approx(-0.205, abs=0.01)
    assert base[:, 1].max() == pytest.approx(0.205, abs=0.01)
    assert {"link3", "front_left_wheel", "ballast"} <= set(pts)
    # The 0.2135 front edge of the 2026-10-16 footprint (whole-robot bounds) is
    # the arm at zero joints; base_link's own mesh ends near x = 0.13.
    fk = robot.link_transforms(robot_files.urdf, {})
    xs = np.concatenate([p @ fk[k][:3, :3].T[:, 0] + fk[k][0, 3] for k, p in pts.items()])
    assert xs.min() == pytest.approx(-0.38, abs=0.01)
    assert xs.max() == pytest.approx(0.2135, abs=0.01)
    # Second call is served from the sha256-keyed cache.
    assert len(list(tmp_path.glob("*.npz"))) == 1
    again = robot.link_points(robot_files.usd, tmp_path)
    np.testing.assert_array_equal(again["base_link"], base)


def test_link_points_spacing(tmp_path):
    from pxr import Usd, UsdGeom
    usd = tmp_path / "r.usda"
    stage = Usd.Stage.CreateNew(str(usd))
    root = UsdGeom.Xform.Define(stage, "/r")
    stage.SetDefaultPrim(root.GetPrim())
    link = UsdGeom.Xform.Define(stage, "/r/plate")
    link.AddTranslateOp().Set((5.0, 0.0, 0.0))
    mesh = UsdGeom.Mesh.Define(stage, "/r/plate/visuals/mesh")
    mesh.AddTranslateOp().Set((1.0, 2.0, 0.0))
    mesh.CreatePointsAttr([(0, 0, 0), (0.3, 0, 0), (0.3, 0.1, 0), (0, 0.1, 0)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    guide = UsdGeom.Cube.Define(stage, "/r/plate/collisions/box")
    guide.CreatePurposeAttr("guide")
    guide.AddTranslateOp().Set((0.0, 0.0, 10.0))
    stage.Save()
    pts = robot.link_points(usd, tmp_path / "cache")["plate"]
    # Mesh transform applied, link transform removed, guide geometry ignored.
    np.testing.assert_allclose(pts.min(0), [1.0, 2.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(pts.max(0), [1.3, 2.1, 0.0], atol=1e-9)
    g = np.stack(np.meshgrid(np.linspace(1.0, 1.3, 61), np.linspace(2.0, 2.1, 21)), -1).reshape(-1, 2)
    d = np.linalg.norm(g[:, None, :] - pts[None, :, :2], axis=2).min(1)
    assert d.max() <= 0.02


def test_grounded_z(robot_files, real_root):
    assert robot.grounded_z(robot_files.urdf, real_root) == pytest.approx(0.0775, abs=1e-4)


def test_module_imports_without_pxr():
    import subprocess
    import sys
    # A None entry makes any `import pxr` raise ImportError.
    code = "import sys; sys.modules['pxr'] = None; import map_check.robot"
    subprocess.run([sys.executable, "-c", code], check=True,
                   cwd=Path(__file__).resolve().parents[1])
