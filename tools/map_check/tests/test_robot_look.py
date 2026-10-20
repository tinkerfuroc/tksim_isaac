import hashlib
from pathlib import Path

import numpy as np
import pytest

from map_check import robot
from map_check.render import robot_look

pxr = pytest.importorskip("pxr")
from pxr import Usd, UsdGeom, UsdShade  # noqa: E402

WHEELS = ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel")
XARM_WHITE = ("link_base", "link1", "link2", "link3", "link4", "link5", "link6",
              "xarm_gripper_base_link")


@pytest.fixture
def robot_usd(real_root) -> Path:
    pointer = real_root / "artifacts" / "robot" / "tinker2" / "current.json"
    if not pointer.is_file():
        pytest.skip(f"robot artifact absent: {pointer}")
    return robot.resolve_robot(pointer).usd


def _stage_with(usd: Path):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    tinker = stage.DefinePrim("/World/Tinker", "Xform")
    tinker.GetReferences().AddReference(str(usd))
    mats = robot_look.define_materials(stage, "/World/Looks/Robot")
    return stage, tinker, mats


def _visual_gprims(tinker, link):
    visuals = tinker.GetChild(link).GetChild("visuals")
    return [p for p in Usd.PrimRange(visuals, Usd.TraverseInstanceProxies())
            if p.IsA(UsdGeom.Gprim) and UsdGeom.Imageable(p).ComputePurpose() != "guide"]


def _bound(prim):
    mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    return mat.GetPath() if mat else None


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def test_colours_follow_the_real_robot(robot_usd):
    stage, tinker, mats = _stage_with(robot_usd)
    missing = robot_look.apply(stage, tinker, mats)
    assert missing == []
    look = {name: m.GetPath() for name, m in mats.items()}
    for link in WHEELS + ("ballast",):
        assert {_bound(p) for p in _visual_gprims(tinker, link)} == {look["black"]}, link
    assert {_bound(p) for p in _visual_gprims(tinker, "livox_frame")} == {look["lidar"]}
    for link in XARM_WHITE:
        assert {_bound(p) for p in _visual_gprims(tinker, link)} == {look["white"]}, link


def test_untouched_parts_keep_the_artifact_materials(robot_usd):
    stage, tinker, mats = _stage_with(robot_usd)
    before = {link: {_bound(p) for p in _visual_gprims(tinker, link)}
              for link in ("left_finger", "link7", "head_camera_base_link", "xarm_camera_link")}
    robot_look.apply(stage, tinker, mats)
    after = {link: {_bound(p) for p in _visual_gprims(tinker, link)} for link in before}
    assert after == before
    assert all(str(m).startswith("/World/Tinker/Looks/") for ms in after.values() for m in ms)


def test_base_box_black_beams_grey(robot_usd):
    stage, tinker, mats = _stage_with(robot_usd)
    robot_look.apply(stage, tinker, mats)
    (mesh,) = _visual_gprims(tinker, robot_look.BASE_LINK)
    subsets = {s.GetPrim().GetName(): s for s in UsdShade.MaterialBindingAPI(mesh).GetMaterialBindSubsets()}
    assert set(subsets) == {"base", "beams"}
    assert _bound(subsets["base"].GetPrim()) == mats["black"].GetPath()
    assert _bound(subsets["beams"].GetPrim()) == mats["grey"].GetPath()

    m = UsdGeom.Mesh(mesh)
    n = len(m.GetFaceVertexCountsAttr().Get())
    base = np.array(subsets["base"].GetIndicesAttr().Get())
    beams = np.array(subsets["beams"].GetIndicesAttr().Get())
    assert sorted(np.r_[base, beams].tolist()) == list(range(n))  # a partition of the faces
    top = robot_look.face_top_z(m, tinker.GetChild(robot_look.BASE_LINK))
    assert top[base].max() <= robot_look.BASE_TOP_Z
    assert top[beams].min() > robot_look.BASE_TOP_Z
    assert top[beams].max() > 1.4  # the beams reach the head mount
    assert 20 <= len(base) < n // 2  # the 0.40 x 0.40 x 0.18 m box, not most of the mesh


def test_shared_artifact_is_not_modified(robot_usd, tmp_path):
    sha = _sha(robot_usd)
    stage, tinker, mats = _stage_with(robot_usd)
    robot_look.apply(stage, tinker, mats)
    layer = pxr.Sdf.Layer.Find(str(robot_usd))
    assert layer is None or not layer.dirty
    stage.GetRootLayer().Export(str(tmp_path / "scene.usda"))
    assert _sha(robot_usd) == sha


def test_missing_links_are_reported_not_fatal(tmp_path):
    usd = tmp_path / "tiny.usda"
    src = Usd.Stage.CreateNew(str(usd))
    root = UsdGeom.Xform.Define(src, "/tiny").GetPrim()
    src.SetDefaultPrim(root)
    UsdGeom.Cube.Define(src, "/tiny/front_left_wheel/visuals/mesh_0/cube")
    src.Save()
    stage, tinker, mats = _stage_with(usd)
    missing = robot_look.apply(stage, tinker, mats)
    assert "front_left_wheel" not in missing
    assert robot_look.BASE_LINK in missing and "livox_frame" in missing
    (cube,) = _visual_gprims(tinker, "front_left_wheel")
    assert _bound(cube) == mats["black"].GetPath()


def test_module_imports_without_pxr():
    import subprocess
    import sys
    code = ("import sys; sys.modules['pxr'] = None; "
            "import map_check.render.robot_look as m; print(sorted(m.COLOURS))")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
    assert "black" in out.stdout
