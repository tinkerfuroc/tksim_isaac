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
ap.add_argument("--room", required=True)
ap.add_argument("--region", required=True, help="x0,x1,y0,y1")
ap.add_argument("--flag", default="", help="regex of names to label amber")
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


# ---- per-room labelled views ----
import re as _re
rx0, rx1, ry0, ry1 = (float(v) for v in args.region.split(","))
rcx, rcy, rw, rh = (rx0 + rx1) / 2, (ry0 + ry1) / 2, rx1 - rx0, ry1 - ry0
APX = 36.0
specs = {}
f_top = 35.0
h_top = 1.08 * max((rw / 2) / (APX / 2 / f_top), (rh / 2) / (APX * H / W / 2 / f_top)) + 1.7
specs["top"] = ((rcx, rcy - 0.001, h_top), (rcx, rcy, 0.0), f_top, (0, 1, 0))
f_ob = 24.0
dist = 0.95 * max(rw, rh * W / H) / (APX / f_ob) + 2.0
specs["oblique"] = ((rcx - 0.35 * dist, rcy - 0.75 * dist, 0.75 * dist), (rcx, rcy, 0.3), f_ob, (0, 0, 1))
annots, views = {}, {}
for key, (eye, target, focal, up) in specs.items():
    cam = UsdGeom.Camera.Define(stage, f"/World/Cams/{key}")
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(APX)
    cam.CreateVerticalApertureAttr(APX * H / W)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 500))
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
    UsdGeom.Xformable(cam).AddTransformOp().Set(view.GetInverse())
    rp = rep.create.render_product(cam.GetPath().pathString, (W, H))
    a = rep.AnnotatorRegistry.get_annotator("rgb")
    a.attach([rp])
    annots[key], views[key] = a, (view, focal)
for _ in range(80):
    app.update()
rep.orchestrator.step(rt_subframes=16)
rep.orchestrator.step(rt_subframes=16)


def project(view, focal, x, y, z):
    pc = view.Transform(Gf.Vec3d(x, y, z))  # camera looks down -Z
    fpx = focal / APX * W
    return W / 2 + fpx * pc[0] / -pc[2], H / 2 - fpx * pc[1] / -pc[2]


try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 19)
    title_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
except OSError:
    font = title_font = ImageFont.load_default()
args.out.mkdir(parents=True, exist_ok=True)
ok = 0
for key, a in annots.items():
    data = np.asarray(a.get_data())
    if data.size == 0:
        print(f"{key}: EMPTY", flush=True)
        continue
    img = Image.fromarray(data[..., :3].astype(np.uint8))
    draw = ImageDraw.Draw(img)
    view, focal = views[key]
    taken = [(0, 0, W, 44)]
    anchors = {n: project(view, focal, p["x"], p["y"], 1.65) for n, p in poses.items()}
    for n, (u, v) in anchors.items():  # keep labels off the robots themselves
        taken.append((u - 18, v - 18, u + 18, v + 18))

    def free(box):
        return all(box[2] < t[0] or box[0] > t[2] or box[3] < t[1] or box[1] > t[3] for t in taken) and \
            box[0] >= 4 and box[1] >= 48 and box[2] <= W - 4 and box[3] <= H - 4

    for n, p in sorted(poses.items(), key=lambda kv: anchors[kv[0]][1]):
        u, v = anchors[n]
        tw, th = draw.textbbox((0, 0), n, font=font)[2:]
        fx, fy = math.cos(p["yaw"]), math.sin(p["yaw"])
        fu, fv = project(view, focal, p["x"] + fx, p["y"] + fy, 1.65)
        base = math.atan2(fv - v, fu - u)  # try the robot's facing direction first
        box = None
        for r in (40, 70, 100, 135, 175, 220):
            for k in range(16):
                ang = base + (k // 2 + 1) * (math.pi / 8) * (1 if k % 2 else -1) if k else base
                cxl, cyl = u + r * math.cos(ang), v + r * math.sin(ang)
                cand = (cxl - tw / 2 - 6, cyl - th / 2 - 4, cxl + tw / 2 + 6, cyl + th / 2 + 4)
                if free(cand):
                    box = cand
                    break
            if box:
                break
        if box is None:
            box = (u + 20, v - th - 8, u + 20 + tw + 12, v)
        taken.append(box)
        flagged = bool(args.flag) and _re.fullmatch(args.flag, n)
        edge = (230, 140, 0) if flagged else (40, 40, 48)
        bx, by = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        draw.line((u, v, bx, by), fill=edge, width=2)
        draw.ellipse((u - 4, v - 4, u + 4, v + 4), fill=edge)
        draw.rounded_rectangle(box, radius=6, fill=(255, 244, 214) if flagged else (255, 255, 255), outline=edge, width=2)
        draw.text((box[0] + 6, box[1] + 4), n, fill=(30, 30, 34), font=font)
    draw.rectangle((0, 0, W, 44), fill=(20, 20, 24))
    draw.text((14, 9), f"{args.room} · {len(poses)} poses · {'top-down' if key == 'top' else 'from the south-west'}",
              fill=(240, 240, 240), font=title_font)
    img.save(args.out / f"{args.room}_{key}.png")
    print(f"{key}: saved mean={data[..., :3].mean():.1f}", flush=True)
    ok += 1
app.close()
sys.exit(0 if ok == len(annots) else 1)
