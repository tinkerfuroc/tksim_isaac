"""Headless RTX renders of Tinker (kinematically posed, no physics) at named map poses."""
import argparse
import math
import sys
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--arena", type=Path, required=True)
ap.add_argument("--robot", type=Path, required=True, help="robot artifact dir (robot.usd + robot.urdf)")
ap.add_argument("--poses", type=Path, required=True)
ap.add_argument("--arm-deg", default="-87.4,-58.8,-2.4,8.0,12.3,-68.5,-8.2")
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--gpu", type=int, default=1)
ap.add_argument("--size", default="1280x800")
ap.add_argument("--only", default=None, help="regex; render only matching pose names")
ap.add_argument("--prefix", default="", help="filename prefix for the overview images")
ap.add_argument("--views", choices=("all", "closeups", "overview"), default="all")
ap.add_argument("--closeup-only", default=None, help="regex; close-ups only for these poses (all matching --only are placed)")
args = ap.parse_args()
W, H = (int(v) for v in args.size.split("x"))

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path.cwd() / "simulation"))
import yaml
from tinker_fk import link_transforms
from tinker_sim_isaac.base_spawn import grounded_base_height

poses = yaml.safe_load(args.poses.read_text())["poses"]
if args.only:
    import re
    poses = {k: v for k, v in poses.items() if re.fullmatch(args.only, k)}
arm = {f"joint{i + 1}": math.radians(float(v)) for i, v in enumerate(args.arm_deg.split(","))}
fk = link_transforms(args.robot / "robot.urdf", arm)
z0 = grounded_base_height(args.robot / "robot.urdf")

from isaacsim import SimulationApp

app = SimulationApp({
    "headless": True, "renderer": "RaytracedLighting", "width": W, "height": H, "fast_shutdown": True,
    "extra_args": ["--/renderer/multiGpu/enabled=false", f"--/renderer/activeGpu={args.gpu}"],
})

import numpy as np
import omni.replicator.core as rep
import omni.usd
from PIL import Image, ImageDraw, ImageFont
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, "/World")
arena = stage.DefinePrim("/World/Arena", "Xform")
arena.GetReferences().AddReference(str(args.arena.resolve()))
bbox = UsdGeom.BBoxCache(0, ["default"]).ComputeWorldBound(arena).ComputeAlignedRange()
lo, hi = bbox.GetMin(), bbox.GetMax()
cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
sx, sy = hi[0] - lo[0], hi[1] - lo[1]


def material(path, rgb, rough):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


floor = UsdGeom.Cube.Define(stage, "/World/Floor")
floor.CreateSizeAttr(1.0)
floor.AddTranslateOp().Set(Gf.Vec3d(cx, cy, -0.01))
floor.AddScaleOp().Set(Gf.Vec3d(sx + 4, sy + 4, 0.02))
UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(material("/World/Looks/Floor", (0.62, 0.58, 0.52), 0.8))
UsdShade.MaterialBindingAPI.Apply(arena).Bind(material("/World/Looks/Walls", (0.30, 0.45, 0.62), 0.5))
UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(350)
sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(1100)
sun.CreateAngleAttr(1.0)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35, 20, 30))

for name, p in poses.items():
    root = UsdGeom.Xform.Define(stage, f"/World/Robots/{name}")
    root.AddTranslateOp().Set(Gf.Vec3d(p["x"], p["y"], z0))
    root.AddRotateZOp().Set(math.degrees(p["yaw"]))
    robot = stage.DefinePrim(f"/World/Robots/{name}/Tinker", "Xform")
    robot.GetReferences().AddReference(str((args.robot / "robot.usd").resolve()))
    for child in robot.GetChildren():
        T = fk.get(child.GetName())
        if T is not None:
            m = Gf.Matrix4d(*T.T.flatten().tolist())  # column-major numpy -> USD row-vector convention
            xf = UsdGeom.Xformable(child)
            ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
            ops["xformOp:translate"].Set(Gf.Vec3d(m.ExtractTranslation()))
            q = m.ExtractRotationQuat()
            orient = ops["xformOp:orient"]
            orient.Set(Gf.Quatd(q) if orient.GetPrecision() == UsdGeom.XformOp.PrecisionDouble else Gf.Quatf(q))
    # kinematic pose only: no simulation of this copy
    for prim in Usd.PrimRange(robot):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    joints = robot.GetChild("joints")
    if joints:
        joints.SetActive(False)


# ---- solo close-ups: one camera, one robot visible at a time ----
sys.path.insert(0, str(args.arena.parent.parent))
from build_arena import fill_enclosed, occupied_mask

mask, meta = occupied_mask(args.arena.parent / "map.yaml")
solid, _ = fill_enclosed(mask, meta["resolution"], 4.0)
ys, xs = np.nonzero(solid)
res, (ox, oy) = meta["resolution"], meta["origin"][:2]
wx, wy = ox + (xs + 0.5) * res, oy + (solid.shape[0] - ys - 0.5) * res
X0, X1, HY = -0.38, 0.2135, 0.205  # base footprint, from robot.usd bounds


def overlap_m(p):
    """Depth (m) by which the base rectangle overlaps solid cells; 0 if clear."""
    c, s = math.cos(p["yaw"]), math.sin(p["yaw"])
    dx, dy = wx - p["x"], wy - p["y"]
    rx, ry = c * dx + s * dy, -s * dx + c * dy
    h = res / 2
    inside = (rx + h > X0) & (rx - h < X1) & (np.abs(ry) - h < HY)
    if not inside.any():
        return 0.0
    depth = np.minimum.reduce([rx[inside] + h - X0, X1 - (rx[inside] - h), HY - (np.abs(ry[inside]) - h)])
    return float(depth.max())


cam = UsdGeom.Camera.Define(stage, "/World/Cams/solo")
cam.CreateFocalLengthAttr(20)
cam.CreateHorizontalApertureAttr(36.0)
cam.CreateVerticalApertureAttr(36.0 * H / W)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 500))
cam_op = UsdGeom.Xformable(cam).AddTransformOp()
rp = rep.create.render_product(cam.GetPath().pathString, (W, H))
annot = rep.AnnotatorRegistry.get_annotator("rgb")
annot.attach([rp])
for _ in range(60):
    app.update()

args.out.mkdir(parents=True, exist_ok=True)
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)
except OSError:
    font = ImageFont.load_default()
ok = 0
for name, p in poses.items():
    # visibility toggles render one pose late; moving the other robots away does not
    for other, q in poses.items():
        op = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/Robots/{other}")).GetOrderedXformOps()[0]
        op.Set(Gf.Vec3d(q["x"], q["y"], z0 if other == name else -100.0))
    a = p["yaw"] + math.pi - 0.7  # behind-left: the robot and what it faces
    eye = Gf.Vec3d(p["x"] + 2.2 * math.cos(a), p["y"] + 2.2 * math.sin(a), 2.0)
    target = Gf.Vec3d(p["x"] + 0.5 * math.cos(p["yaw"]), p["y"] + 0.5 * math.sin(p["yaw"]), 0.5)
    cam_op.Set(Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse())
    for _ in range(4):
        app.update()
    # the annotator returns the previous step's frame; render twice, keep the second
    rep.orchestrator.step(rt_subframes=16)
    rep.orchestrator.step(rt_subframes=16)
    data = np.asarray(annot.get_data())
    if data.size == 0:
        print(f"{name}: EMPTY", flush=True)
        continue
    img = Image.fromarray(data[..., :3].astype(np.uint8))
    draw = ImageDraw.Draw(img)
    ov = overlap_m(p)
    label = f"{name}   x {p['x']:.3f}  y {p['y']:.3f}  yaw {math.degrees(p['yaw']):.1f}°"
    bar = (150, 30, 30) if ov >= 0.01 else (20, 20, 24)
    draw.rectangle((0, 0, W, 40), fill=bar)
    draw.text((14, 8), label + (f"   base overlaps furniture by {ov * 100:.1f} cm" if ov >= 0.01 else ""),
              fill=(240, 240, 240), font=font)
    img.save(args.out / f"{name}.png")
    print(f"{name}: saved overlap={ov:.3f} mean={data[..., :3].mean():.1f}", flush=True)
    ok += 1
app.close()
sys.exit(0 if ok == len(poses) else 1)
