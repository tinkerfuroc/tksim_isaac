"""Render one Map Check job in Isaac Sim: python -m map_check.render.isaac_render <job.json>.

Runs under the repo's Isaac Python (map_check.jobs starts it with PYTHONPATH set
to tools/map_check). The scene follows prototypes/render_rooms.py: the arena on
a floor slab, dome and sun lights, and one kinematically posed Tinker copy per
pose (URDF FK with that pose's arm, pan and tilt; physics off, joints prim
inactive). The overview renders first in its own step, labelled with
place_labels. Close-ups follow one pose at a time, with every other robot moved
to z = -100, through at most one render product per close-up view.

Stdout carries the progress events from jobfile; anything else Isaac prints
there is ignored by the runner. Isaac is imported inside main(), so importing
this module needs only numpy.
"""
from __future__ import annotations

import json
import math
import os
import re
import sys
import traceback
from pathlib import Path

import numpy as np

from map_check import robot
from map_check.render import cameras
from map_check.render.jobfile import OVERVIEW, JobFile, done_event, error_event, view_event
from map_check.render.labels import place_labels
from map_check.render.watchdog import guard

APP_W, APP_H = 1280, 800
WARMUP_UPDATES = 80  # app updates after the scene is built, before the first step
SETTLE_UPDATES = 4  # app updates after moving robots or cameras
RT_SUBFRAMES = 16
HIDDEN_Z = -100.0  # robots not in a close-up move here (visibility toggles lag a frame)
LABEL_Z = 1.65  # m, height of the overview label anchors (top of the robot)
FLOOR_RGB, WALL_RGB = (0.62, 0.58, 0.52), (0.30, 0.45, 0.62)
FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
HEADER_PX = 52  # overview title bar
CLOSEUP_HEADER_PX = 40
CAPTION = {"behind": "behind-left", "top": "top-down", "head": "head cam"}


def emit(line: str) -> None:
    print(line, flush=True)


def display_name(pose) -> str:
    """The pose name as the teammate sees it; pasted rows can share a Map Studio name."""
    return f"{pose.name} (pasted)" if pose.key.startswith("pasted:") else pose.name


def overview_labels(spec: cameras.CameraSpec, poses, flagged, text_size) -> list[dict]:
    """One label per row, keyed by row key: {key, text, box, anchor, flagged}.

    text_size(text) -> (w, h) in pixels. Needs numpy only, so it is unit-tested.
    """
    anchors, sizes, facing, texts = {}, {}, {}, {}
    for p in poses:
        u, v = cameras.project(spec, (p.x, p.y, LABEL_Z))
        fu, fv = cameras.project(spec, (p.x + math.cos(p.yaw), p.y + math.sin(p.yaw), LABEL_Z))
        anchors[p.key], facing[p.key] = (u, v), math.atan2(fv - v, fu - u)
        texts[p.key] = display_name(p)
        sizes[p.key] = tuple(text_size(texts[p.key]))
    placed = place_labels(anchors, sizes, facing, spec.width, spec.height, header_px=HEADER_PX)
    return [{"key": p.key, "text": texts[p.key], "box": placed[p.key][0],
             "anchor": placed[p.key][1], "flagged": p.key in flagged} for p in poses]


def _file_stem(i: int, name: str) -> str:
    return f"{i:02d}_{re.sub(r'[^A-Za-z0-9_]', '_', name)}"


def _usd_matrix(m: np.ndarray):
    """Gf.Matrix4d of a column-vector 4x4 (USD multiplies row vectors, so transpose)."""
    from pxr import Gf

    return Gf.Matrix4d(*np.asarray(m, float).T.flatten().tolist())


def _bounds(job: JobFile, extra: dict, arena_range) -> tuple[float, float, float, float]:
    if extra.get("bounds"):
        return tuple(float(v) for v in extra["bounds"])
    lo, hi = arena_range.GetMin(), arena_range.GetMax()
    return (lo[0], hi[0], lo[1], hi[1])


class Renderer:
    def __init__(self, app, job: JobFile, extra: dict):
        import omni.replicator.core as rep
        import omni.usd
        from PIL import ImageFont

        self.app, self.job, self.rep = app, job, rep
        self.flagged = set(extra.get("flagged", []))
        self.out = Path(job.out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        omni.usd.get_context().new_stage()
        self.stage = omni.usd.get_context().get_stage()
        self._fk_cache: dict[tuple[float, float], dict] = {}
        self.fonts = {}
        for key, size in (("label", 24), ("title", 30), ("closeup", 22)):
            try:
                self.fonts[key] = ImageFont.truetype(FONT_PATH, size)
            except OSError:
                self.fonts[key] = ImageFont.load_default()
        self.bounds = self._build_scene(extra)
        self._warm = False

    # -- scene --
    def fk(self, pose):
        k = (pose.pan_deg, pose.tilt_deg)
        if k not in self._fk_cache:
            q = robot.joint_values(self.job.arm_deg, pose.pan_deg, pose.tilt_deg)
            self._fk_cache[k] = robot.link_transforms(Path(self.job.robot_urdf), q)
        return self._fk_cache[k]

    def _material(self, path, rgb, rough):
        from pxr import Gf, Sdf, UsdShade

        mat = UsdShade.Material.Define(self.stage, path)
        sh = UsdShade.Shader.Define(self.stage, path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat

    def _build_scene(self, extra):
        from pxr import Gf, UsdGeom, UsdLux, UsdShade

        stage = self.stage
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")
        arena = stage.DefinePrim("/World/Arena", "Xform")
        arena.GetReferences().AddReference(str(Path(self.job.arena_usd).resolve()))
        rng = UsdGeom.BBoxCache(0, ["default"]).ComputeWorldBound(arena).ComputeAlignedRange()
        lo, hi = rng.GetMin(), rng.GetMax()
        floor = UsdGeom.Cube.Define(stage, "/World/Floor")
        floor.CreateSizeAttr(1.0)
        floor.AddTranslateOp().Set(Gf.Vec3d((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, -0.01))
        floor.AddScaleOp().Set(Gf.Vec3d(hi[0] - lo[0] + 4, hi[1] - lo[1] + 4, 0.02))
        UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(
            self._material("/World/Looks/Floor", FLOOR_RGB, 0.8))
        UsdShade.MaterialBindingAPI.Apply(arena).Bind(
            self._material("/World/Looks/Walls", WALL_RGB, 0.5))
        UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(350)
        sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
        sun.CreateIntensityAttr(1100)
        sun.CreateAngleAttr(1.0)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35, 20, 30))
        self.root_ops = [self._add_robot(i, p) for i, p in enumerate(self.job.poses)]
        return _bounds(self.job, extra, rng)

    def _add_robot(self, i, pose):
        """A posed, physics-free Tinker copy; returns its root translate op."""
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        root = UsdGeom.Xform.Define(self.stage, f"/World/Robots/r{i:03d}")
        translate = root.AddTranslateOp()
        translate.Set(Gf.Vec3d(pose.x, pose.y, self.job.z0))
        root.AddRotateZOp().Set(math.degrees(pose.yaw))
        tinker = self.stage.DefinePrim(f"/World/Robots/r{i:03d}/Tinker", "Xform")
        tinker.GetReferences().AddReference(str(Path(self.job.robot_usd).resolve()))
        fk = self.fk(pose)
        for child in tinker.GetChildren():
            t = fk.get(child.GetName())
            if t is None:
                continue
            m = _usd_matrix(t)
            ops = {op.GetOpName(): op for op in UsdGeom.Xformable(child).GetOrderedXformOps()}
            ops["xformOp:translate"].Set(Gf.Vec3d(m.ExtractTranslation()))
            q = m.ExtractRotationQuat()
            orient = ops["xformOp:orient"]
            orient.Set(Gf.Quatd(q) if orient.GetPrecision() == UsdGeom.XformOp.PrecisionDouble
                       else Gf.Quatf(q))
        for prim in Usd.PrimRange(tinker):  # kinematic pose only: nothing simulates the copy
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        joints = tinker.GetChild("joints")
        if joints:
            joints.SetActive(False)
        return translate

    def show_only(self, keep: int | None) -> None:
        """Every robot at its pose (keep None) or only robot `keep`; the rest at HIDDEN_Z."""
        from pxr import Gf

        for i, (op, p) in enumerate(zip(self.root_ops, self.job.poses)):
            z = self.job.z0 if keep is None or i == keep else HIDDEN_Z
            op.Set(Gf.Vec3d(p.x, p.y, z))

    # -- cameras and steps --
    def camera(self, name: str, spec: cameras.CameraSpec):
        from pxr import Gf, UsdGeom

        cam = UsdGeom.Camera.Define(self.stage, f"/World/Cams/{name}")
        cam.CreateHorizontalApertureAttr(cameras.APERTURE_MM)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 500))
        op = UsdGeom.Xformable(cam).AddTransformOp()
        self.aim(cam, op, spec)
        return cam, op

    @staticmethod
    def aim(cam, op, spec: cameras.CameraSpec) -> None:
        cam.GetFocalLengthAttr().Set(spec.focal_mm)
        cam.GetVerticalApertureAttr().Set(cameras.APERTURE_MM * spec.height / spec.width)
        op.Set(_usd_matrix(cameras.cam_to_world(spec)))

    def attach(self, cam_path: str, size: tuple[int, int]):
        rp = self.rep.create.render_product(cam_path, size)
        annot = self.rep.AnnotatorRegistry.get_annotator("rgb")
        annot.attach([rp])
        self._warm = False  # new render products need the long warmup
        return rp, annot

    def detach(self, rp, annot) -> None:
        annot.detach()
        destroy = getattr(rp, "destroy", None)
        if destroy:
            destroy()

    def step(self, annots: list) -> list[np.ndarray]:
        """Render once all products; the annotators lag a step, so step twice."""
        for _ in range(SETTLE_UPDATES if self._warm else WARMUP_UPDATES):
            self.app.update()
        self._warm = True
        self.rep.orchestrator.step(rt_subframes=RT_SUBFRAMES)
        self.rep.orchestrator.step(rt_subframes=RT_SUBFRAMES)
        out = []
        for a in annots:
            data = np.asarray(a.get_data())
            if data.size == 0:  # one more step before giving up on this frame
                self.rep.orchestrator.step(rt_subframes=RT_SUBFRAMES)
                data = np.asarray(a.get_data())
            if data.size == 0:
                raise RuntimeError("the renderer returned an empty frame")
            out.append(data[..., :3].astype(np.uint8))
        return out

    # -- views --
    def overview(self) -> None:
        from PIL import Image, ImageDraw

        spec = cameras.overview_top(self.bounds)
        cam, _ = self.camera("overview", spec)
        self.show_only(None)
        rp, annot = self.attach(cam.GetPath().pathString, (spec.width, spec.height))
        (rgb,) = self.step([annot])
        self.detach(rp, annot)

        img = Image.fromarray(rgb)
        draw = ImageDraw.Draw(img)
        font = self.fonts["label"]
        labels = overview_labels(spec, self.job.poses, self.flagged,
                                 lambda t: draw.textbbox((0, 0), t, font=font)[2:])
        for lab in labels:
            box, (u, v), bad = lab["box"], lab["anchor"], lab["flagged"]
            edge = (200, 40, 40) if bad else (40, 40, 48)
            draw.line((u, v, (box[0] + box[2]) / 2, (box[1] + box[3]) / 2), fill=edge, width=2)
            draw.ellipse((u - 5, v - 5, u + 5, v + 5), fill=edge)
            draw.rounded_rectangle(box, radius=6, fill=(255, 226, 226) if bad else (255, 255, 255),
                                   outline=edge, width=3 if bad else 2)
            draw.text((box[0] + 6, box[1] + 4), lab["text"], fill=(30, 30, 34), font=font)
        draw.rectangle((0, 0, spec.width, HEADER_PX), fill=(20, 20, 24))
        title = f"{self.job.map} · {len(labels)} poses · top-down"
        n_flagged = sum(lab["flagged"] for lab in labels)
        if n_flagged:
            title += f" · {n_flagged} flagged"
        draw.text((16, 10), title, fill=(240, 240, 240), font=self.fonts["title"])
        img.save(self.out / "overview.png")
        emit(view_event(OVERVIEW, OVERVIEW, "overview.png"))

    def closeups(self) -> None:
        """One step per pose with only that robot shown: at most 3 render products per step."""
        used = [v for v in ("behind", "top", "head") if any(v in p.views for p in self.job.poses)]
        if not used:
            return
        first = self.job.poses[0]
        specs = {"behind": cameras.behind_left(first), "top": cameras.top_down(first),
                 "head": cameras.head_cam(first, self.fk(first), self.job.z0)}
        rig = {}
        for v in used:
            cam, op = self.camera(v, specs[v])
            rp, annot = self.attach(cam.GetPath().pathString, (specs[v].width, specs[v].height))
            rig[v] = (cam, op, rp, annot)
        try:
            for i, p in enumerate(self.job.poses):
                if not p.views:
                    continue
                self.show_only(i)
                for v in used:
                    cam, op, _, _ = rig[v]
                    self.aim(cam, op, self._spec(v, p))
                frames = dict(zip(used, self.step([rig[v][3] for v in used])))
                for v in p.views:
                    name = f"{_file_stem(i, p.name)}_{v}.png"
                    self._save_closeup(frames[v], p, v, name)
                    emit(view_event(p.key, v, name))
        finally:
            for v in used:
                self.detach(rig[v][2], rig[v][3])

    def _spec(self, view: str, p) -> cameras.CameraSpec:
        if view == "behind":
            return cameras.behind_left(p)
        if view == "top":
            return cameras.top_down(p)
        return cameras.head_cam(p, self.fk(p), self.job.z0)

    def _save_closeup(self, rgb: np.ndarray, p, view: str, name: str) -> None:
        from PIL import Image, ImageDraw

        img = Image.fromarray(rgb)
        draw = ImageDraw.Draw(img)
        bad = p.key in self.flagged
        draw.rectangle((0, 0, img.width, CLOSEUP_HEADER_PX), fill=(150, 30, 30) if bad else (20, 20, 24))
        text = f"{display_name(p)} · {CAPTION[view]}"
        if view == "head":
            text += f" · pan {p.pan_deg:g}° tilt {p.tilt_deg:g}°"
        draw.text((14, 8), text, fill=(240, 240, 240), font=self.fonts["closeup"])
        img.save(self.out / name)


def main(argv: list[str] | None = None) -> int:
    start_ppid = os.getppid()  # first thing: the server that started us
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        emit(error_event("usage: python -m map_check.render.isaac_render <job.json>"))
        return 2
    try:
        job = JobFile.read(Path(argv[0]))
        extra = json.loads(Path(argv[0]).read_text())
    except ValueError as e:
        emit(error_event(str(e)))
        return 2
    # Before Isaac loads: exit at the job's deadline or when the server is gone, so a hung
    # Isaac can't hold the GPU after its server died without stopping it.
    guard(job.deadline, start_ppid, report=lambda why: emit(error_event(f"{why}; render stopped")))

    from isaacsim import SimulationApp

    app = SimulationApp({
        "headless": True, "renderer": "RaytracedLighting", "width": APP_W, "height": APP_H,
        "fast_shutdown": True,
        # The server sets CUDA_VISIBLE_DEVICES to job.gpu (jobs.job_env), so the chosen GPU
        # is device 0 here, as in scripts/gpsr-stack; physics stays off the GPU.
        "extra_args": ["--/renderer/multiGpu/enabled=false", "--/renderer/activeGpu=0",
                       "--/physics/cudaDevice=-1"],
    })
    code = 0
    try:
        r = Renderer(app, job, extra)
        if job.overview:
            r.overview()
        r.closeups()
        emit(done_event())
    except Exception as e:  # noqa: BLE001 - reported to the runner as the job's message
        traceback.print_exc()
        emit(error_event(f"{type(e).__name__}: {e}"))
        code = 1
    finally:
        app.close()
    return code


if __name__ == "__main__":
    sys.exit(main())
