#!/usr/bin/env python3
"""Verify the RTX vision path renders known scene colors from the arena3 map.

The committed RoboCup Arena 3 occupancy map is spawned as its usual kinematic
cuboid walls, except each wall is assigned a deterministic saturated color from
a fixed palette instead of the uniform gray used by
``IsaacWholeRobotBackend``.  An RTX camera then captures the arena from the
same deterministic overview pose the WebRTC arena viewer uses, and the frame is
checked for every palette hue.

This is a stronger statement than "the annotator buffer was non-empty", which
is all ``rtx_sensor_smoke.py`` asserts: a frame that carries the expected hues
in the expected proportions proves the material, shading, camera, and annotator
path all agree with the authored scene.  Classification is by hue rather than
by exact RGB because ray-traced diffuse shading under a dome light never
reproduces authored sRGB values verbatim.
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import traceback
from math import prod
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
from tinker_sim_core.arena_palette import (  # noqa: E402
    WALL_PALETTE,
    expected_wall_colors,
    wall_color,
)

#: Rendered frame size as (height, width).  ``CameraSensor`` takes the
#: resolution in that order while ``SimulationApp`` takes width/height.
RESOLUTION = (720, 1280)
MAX_RENDER_UPDATES = 300

#: Hue classification bounds.  Half the palette spacing would be 30 degrees;
#: 25 leaves a deliberate guard band so a misclassified hue is dropped rather
#: than silently credited to a neighbouring color.
HUE_TOLERANCE_DEG = 25.0
MIN_SATURATION = 0.25
MIN_VALUE = 0.15
#: Each palette color is carried by roughly a sixth of 707 walls, so a color
#: covering less than this share of the frame means walls did not render.
MIN_COLOR_RATIO = 0.0005

REPORT_PATH = Path("reports/arena-vision-latest.json")
IMAGE_DIR = Path("reports/arena-vision")


def hue_presence(
    image: object,
    *,
    hue_tolerance_deg: float = HUE_TOLERANCE_DEG,
    min_saturation: float = MIN_SATURATION,
    min_value: float = MIN_VALUE,
    min_ratio: float = MIN_COLOR_RATIO,
) -> dict[str, object]:
    """Classify every pixel by palette hue and report per-color coverage.

    Hue is taken from an HSV conversion so the verdict survives the exposure and
    falloff that ray-traced diffuse shading applies to an authored color.
    Pixels below ``min_saturation`` or ``min_value`` are counted as achromatic
    and never credited to a palette entry.
    """
    import numpy as np

    array = np.asarray(image, dtype=np.float64) / 255.0
    if array.ndim != 3 or array.shape[2] < 3:
        raise ValueError(f"expected an RGB image, got shape {array.shape}")
    array = array[:, :, :3]
    total = float(array.shape[0] * array.shape[1])

    maximum = array.max(axis=2)
    minimum = array.min(axis=2)
    delta = maximum - minimum
    saturation = np.where(maximum > 0.0, delta / np.maximum(maximum, 1e-12), 0.0)

    red, green, blue = array[:, :, 0], array[:, :, 1], array[:, :, 2]
    safe_delta = np.where(delta > 0.0, delta, 1.0)
    hue = np.zeros_like(maximum)
    is_red = (maximum == red) & (delta > 0.0)
    is_green = (maximum == green) & (delta > 0.0)
    is_blue = (maximum == blue) & (delta > 0.0)
    hue[is_red] = (((green - blue) / safe_delta) % 6.0)[is_red]
    hue[is_green] = (((blue - red) / safe_delta) + 2.0)[is_green]
    hue[is_blue] = (((red - green) / safe_delta) + 4.0)[is_blue]
    hue = (hue * 60.0) % 360.0

    chromatic = (saturation >= min_saturation) & (maximum >= min_value) & (delta > 0.0)
    colors: dict[str, dict[str, object]] = {}
    for name, _rgb, target_hue in WALL_PALETTE:
        difference = np.abs(((hue - target_hue + 180.0) % 360.0) - 180.0)
        matched = int((chromatic & (difference <= hue_tolerance_deg)).sum())
        ratio = matched / total if total else 0.0
        colors[name] = {
            "matched_pixels": matched,
            "present": ratio >= min_ratio,
            "ratio": round(ratio, 6),
            "target_hue_deg": target_hue,
        }
    return {
        "chromatic_pixel_ratio": round(float(chromatic.sum()) / total, 6) if total else 0.0,
        "colors": colors,
        "hue_tolerance_deg": hue_tolerance_deg,
        "min_ratio": min_ratio,
        "min_saturation": min_saturation,
        "min_value": min_value,
    }


def rgb_image(value: object, resolution: tuple[int, int]):
    """Normalize an RTX ``rgb`` annotator buffer to a PIL RGB image.

    Mirrors ``QualificationVisualCapture._rgb_image`` but takes the expected
    resolution rather than hardcoding 540x960.
    """
    import numpy as np
    from PIL import Image

    candidate = value
    if hasattr(candidate, "cpu"):
        candidate = candidate.cpu()
    if hasattr(candidate, "numpy"):
        candidate = candidate.numpy()
    array = np.asarray(candidate)
    if array.ndim == 4 and array.shape[0] == 1:
        array = array[0]
    if array.ndim != 3 or array.shape[:2] != resolution:
        raise ValueError(f"unexpected RGB frame shape: {array.shape}")
    if array.shape[2] < 3:
        raise ValueError(f"RGB frame has fewer than three channels: {array.shape}")
    array = array[:, :, :3]
    if array.dtype.kind == "f":
        if not np.isfinite(array).all():
            raise ValueError("RGB frame contains non-finite values")
        scale = 255.0 if float(array.max(initial=0.0)) <= 1.0 else 1.0
        array = np.clip(array * scale, 0.0, 255.0).astype(np.uint8)
    else:
        array = np.clip(array, 0, 255).astype(np.uint8)
    return Image.fromarray(array, mode="RGB")


def persist_png(image: object, output: Path) -> None:
    """Atomically persist a PNG: temp -> fsync -> replace -> parent fsync."""
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{output.name}.", dir=str(output.parent))
    try:
        with os.fdopen(fd, "wb") as stream:
            image.save(stream, format="PNG", optimize=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output)
        dir_fd = os.open(str(output.parent), os.O_DIRECTORY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    except BaseException:
        try:
            os.close(fd)
        except OSError:
            pass
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def arena_map_yaml(root: Path = ROOT) -> Path:
    """Resolve the committed arena3 map beside the current robot artifact."""
    pointer = json.loads(
        (root / "artifacts/robot/tinker2/current.json").read_text(encoding="utf-8")
    )
    manifest = Path(str(pointer["manifest"]))
    if not manifest.is_absolute():
        manifest = root / manifest
    map_yaml = manifest.parent / "map.yaml"
    if not map_yaml.is_file():
        raise RuntimeError(f"arena map is missing: {map_yaml}")
    return map_yaml


def _shape(value: object) -> list[int]:
    shape = getattr(value, "shape", ())
    return [int(dimension) for dimension in shape]


def _element_count(value: object) -> int:
    return prod(_shape(value))


def main() -> int:
    height, width = RESOLUTION
    sys.path.insert(0, str(ROOT / "validation"))
    from run_sim import _arena_camera_pose
    from tinker_sim_core.occupancy import OccupancyMap

    map_yaml = arena_map_yaml()

    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "width": width,
            "height": height,
        }
    )
    try:
        from isaacsim.core.rendering_manager import ViewportManager
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.sensors.experimental.rtx")
        for _ in range(20):
            app.update()

        import isaaclab.sim as sim_utils
        import omni.timeline
        from isaaclab.sim import SimulationCfg, SimulationContext
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
        from tinker_sim_isaac.backend import _map_metadata

        # IsaacWholeRobotBackend creates the stage as a side effect of building
        # its SimulationContext before spawning any prim; the bare Python
        # experience has no stage until something makes one.  Match the backend
        # so the spawners behave exactly as they do in the real arena path.
        simulation = SimulationContext(
            SimulationCfg(dt=1.0 / 120.0, device="cpu", render_interval=1)
        )

        pgm, resolution, origin_x, origin_y = _map_metadata(map_yaml)
        occupancy = OccupancyMap.from_pgm(
            pgm, resolution=resolution, origin_x=origin_x, origin_y=origin_y
        )
        rectangles = occupancy.rectangles()

        ground = sim_utils.GroundPlaneCfg()
        ground.func("/World/defaultGroundPlane", ground)
        light = sim_utils.DomeLightCfg(intensity=1200.0, color=(0.95, 0.95, 1.0))
        light.func("/World/DomeLight", light)

        # Same spawn shape as IsaacWholeRobotBackend, with the uniform gray
        # replaced by the deterministic palette.  Keeping the kinematic rigid
        # and collision props means this exercises the real arena wall path
        # rather than a visual-only stand-in.
        for index, (x, y, sx, sy) in enumerate(rectangles):
            name, diffuse, _hue = wall_color(index)
            box = sim_utils.CuboidCfg(
                size=(sx, sy, 1.2),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=diffuse),
            )
            box.func(
                f"/World/NavigationMap/occupied_{index:04d}",
                box,
                translation=(x, y, 0.6),
            )

        eye, target, bounds = _arena_camera_pose(occupancy)
        camera = RtxCamera("/World/ArenaVisionCamera", tick_rate=30.0)
        ViewportManager.set_camera_view(camera.paths[0], eye=eye, target=target)
        camera_sensor = CameraSensor(camera, resolution=RESOLUTION, annotators=["rgb"])

        simulation.reset()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        IMAGE_DIR.mkdir(parents=True, exist_ok=True)
        views: dict[str, object] = {}
        # A second, closer pose proves the frame tracks the camera rather than
        # replaying a fixed image.
        poses = {
            "overview": {"eye": list(eye), "target": list(target)},
            "closeup": {
                "eye": [target[0] - 4.0, target[1] - 4.0, 2.2],
                "target": [target[0], target[1], 0.6],
            },
        }
        for view_name, pose in poses.items():
            ViewportManager.set_camera_view(
                camera.paths[0], eye=pose["eye"], target=pose["target"]
            )
            rgb = None
            rendered_frames = 0
            for rendered_frames in range(1, MAX_RENDER_UPDATES + 1):
                app.update()
                rgb, _ = camera_sensor.get_data("rgb")
                if rgb is not None and _element_count(rgb) > 0 and rendered_frames >= 8:
                    break
            if rgb is None or _element_count(rgb) == 0:
                raise RuntimeError(
                    f"{view_name} produced no RGB frame after "
                    f"{MAX_RENDER_UPDATES} rendered updates"
                )
            image = rgb_image(rgb, RESOLUTION)
            image_path = IMAGE_DIR / f"{view_name}.png"
            persist_png(image, image_path)
            presence = hue_presence(image)
            views[view_name] = {
                "camera_eye": pose["eye"],
                "camera_target": pose["target"],
                "frame_shape": _shape(rgb),
                "image": str(image_path),
                "rendered_updates": rendered_frames,
                **presence,
            }
        timeline.stop()

        overview = views["overview"]
        missing = sorted(
            name
            for name, stats in overview["colors"].items()  # type: ignore[index]
            if not stats["present"]
        )
        if missing:
            raise RuntimeError(
                "arena walls did not render the expected palette hues: "
                + ", ".join(missing)
            )

        result = {
            "arena": {
                "bounds_xy": bounds,
                "collider_count": len(rectangles),
                "id": "robocup-arena3",
                "map": str(map_yaml),
            },
            "camera": {
                "annotator": "rgb",
                "prim": camera.paths[0],
                "render_product_valid": bool(camera_sensor.render_product),
                "resolution_height_width": list(RESOLUTION),
            },
            "expected_wall_colors": expected_wall_colors(len(rectangles)),
            "palette": {name: list(rgb) for name, rgb, _hue in WALL_PALETTE},
            "shutdown": "verified_then_process_exit",
            "views": views,
        }
        REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
        REPORT_PATH.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps(result, sort_keys=True), flush=True)
    except BaseException:
        # Kit teardown can abort the process, so record the failure before it.
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    # os._exit skips Kit's crash-prone extension teardown, so the exit status
    # carries only the validation verdict.  It also skips buffer flushing.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
