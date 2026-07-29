#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import sys
import traceback
from math import prod
from pathlib import Path


def _shape(value: object) -> list[int]:
    shape = getattr(value, "shape", ())
    return [int(dimension) for dimension in shape]


def _element_count(value: object) -> int:
    return prod(_shape(value))


def main() -> int:
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "width": 320,
            "height": 240,
        }
    )
    try:
        from isaacsim.core.experimental.objects import Cube, GroundPlane, SphereLight
        from isaacsim.core.rendering_manager import ViewportManager
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.sensors.experimental.rtx")
        for _ in range(20):
            app.update()

        from isaacsim.sensors.experimental.rtx import (
            CameraSensor,
            Lidar,
            LidarSensor,
            RtxCamera,
        )
        import omni.timeline

        GroundPlane("/World/GroundPlane")
        Cube(
            "/World/Target",
            sizes=1.0,
            positions=[0.0, 0.0, 0.5],
            colors=[0.0, 1.0, 0.0],
        )
        light = SphereLight("/World/KeyLight", positions=[2.0, -2.0, 4.0])
        light.set_intensities(intensities=100000)

        camera = RtxCamera("/World/Camera", tick_rate=30.0)
        ViewportManager.set_camera_view(
            camera.paths[0],
            eye=[3.0, 2.0, 1.5],
            target=[0.0, 0.0, 0.5],
        )
        camera_sensor = CameraSensor(
            camera,
            resolution=(240, 320),
            annotators=["rgb"],
        )

        lidar = Lidar(
            "/World/Lidar",
            tick_rate=10.0,
            aux_output_level="BASIC",
            positions=[0.0, 0.0, 1.0],
        )
        lidar_sensor = LidarSensor(
            lidar,
            annotators=["generic-model-output"],
        )

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        rgb = None
        lidar_data = None
        rendered_frames = 0
        for rendered_frames in range(1, 301):
            app.update()
            rgb, _ = camera_sensor.get_data("rgb")
            lidar_data, _ = lidar_sensor.get_data("generic-model-output")
            if (
                rgb is not None
                and _element_count(rgb) > 0
                and lidar_data is not None
                and _element_count(lidar_data) > 0
            ):
                break
        timeline.stop()

        if rgb is None:
            raise RuntimeError("RTX camera produced no RGB frame after 300 rendered updates")
        if lidar_data is None or _element_count(lidar_data) == 0:
            raise RuntimeError("RTX LiDAR produced no GenericModelOutput after 300 rendered updates")

        result = {
            "camera": {
                "annotator": "rgb",
                "frame_shape": _shape(rgb),
                "prim": camera.paths[0],
                "render_product_valid": bool(camera_sensor.render_product),
            },
            "lidar": {
                "annotator": "generic-model-output",
                "buffer_shape": _shape(lidar_data),
                "prim": lidar.paths[0],
                "render_product_valid": bool(lidar_sensor.render_product),
            },
            "rendered_updates": rendered_frames,
            "shutdown": "verified_then_process_exit",
        }
        result_path = Path("reports/rtx-sensors-latest.json")
        result_path.parent.mkdir(parents=True, exist_ok=True)
        result_path.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(result, sort_keys=True), flush=True)
    except BaseException:
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    raise SystemExit(main())
