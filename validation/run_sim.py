#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import signal
import sys
import time
import traceback
from pathlib import Path


def _content_addressed_tinker_usd(root: Path, requested: Path | None) -> Path:
    pointer = root / "artifacts/robot/tinker2/current.json"
    if requested is None:
        current = json.loads(pointer.read_text(encoding="utf-8"))
        manifest = Path(str(current["manifest"]))
        if not manifest.is_absolute():
            manifest = root / manifest
        requested = manifest.parent / "robot.usd"
    artifact = requested.resolve()
    manifest = artifact.parent / "manifest.json"
    if (
        artifact.name != "robot.usd"
        or artifact.parent.parent.name != "tinker2"
        or len(artifact.parent.name) != 16
        or not all(character in "0123456789abcdef" for character in artifact.parent.name)
        or not artifact.is_file()
        or not manifest.is_file()
    ):
        raise RuntimeError(
            "manipulation-core requires the content-addressed artifacts/robot/tinker2/<hash>/robot.usd"
        )
    return artifact


def _expected_scenario_objects(root: Path, scenario_name: str) -> dict[str, dict[str, object]]:
    if scenario_name == "empty":
        return {}
    sys.path.insert(0, str(root / "simulation"))
    from tinker_sim_core.scenario import load_named_scenario

    scenario = load_named_scenario(root, scenario_name)
    expected: dict[str, dict[str, object]] = {}
    for record in scenario.objects:
        pose = record.get("pose", {})
        xyz = pose.get("xyz", [0.0, 0.0, 0.0])
        quaternion = pose.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        twist = record.get("twist", {})
        expected[str(record["id"])] = {
            "class_name": str(record.get("class_name", "")),
            "prim_path": f"/World/Scenario/{record['id']}",
            "pose": {
                "position": [float(value) for value in xyz],
                "quaternion_xyzw": [float(value) for value in quaternion],
            },
            "twist": {
                "linear": [float(value) for value in twist.get("linear", [0.0, 0.0, 0.0])],
                "angular": [float(value) for value in twist.get("angular", [0.0, 0.0, 0.0])],
            },
        }
    return expected


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sensor-profile",
        choices=("physics-only", "sensor-rich", "navigation-parity", "manipulation-core"),
        required=True,
    )
    parser.add_argument("--profile", choices=("parity", "oracle"), required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--artifact", type=Path)
    parser.add_argument("--map", dest="map_yaml", type=Path)
    parser.add_argument("--ros", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0, help="simulation seconds; 0 runs until signalled")
    parser.add_argument("--qualification", action="store_true")
    args, kit_args = parser.parse_known_args()

    from isaacsim import SimulationApp

    application_config = {
        "headless": args.headless,
        "disable_viewport_updates": args.sensor_profile == "physics-only",
        # Isaac's full Python experience may fault after all extensions have
        # individually torn down.  Fast shutdown is the supported server
        # path and keeps launch exit status meaningful for automation.
        "fast_shutdown": True,
        "extra_args": [
            "--/physics/useGpu=false",
            "--/physics/cudaDevice=-1",
            *kit_args,
        ],
    }
    if args.sensor_profile == "manipulation-core" and args.qualification:
        application_config.update(
            {
                "renderer": "RaytracedLighting",
                "width": 960,
                "height": 540,
            }
        )
    app = SimulationApp(
        {
            **application_config,
        }
    )
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    gateway = None
    visual_capture = None
    failed = False
    try:
        if args.ros:
            from isaacsim.core.utils.extensions import enable_extension

            enable_extension("isaacsim.ros2.sim_control")
            app.update()
        if args.sensor_profile == "navigation-parity":
            root = Path(__file__).resolve().parents[1]
            sys.path.insert(0, str(root / "simulation"))
            from tinker_sim_core.calibration import BaseCalibration
            calibration = BaseCalibration.load(root / "simulation/calibration/tinker2-missing.json")
            if args.qualification and calibration.qualification_error():
                raise RuntimeError(calibration.qualification_error())
            if args.artifact is None:
                current = json.loads((root / "artifacts/robot/tinker2/current.json").read_text())
                manifest_path = Path(current["manifest"])
                if not manifest_path.is_absolute():
                    manifest_path = root / manifest_path
                args.artifact = manifest_path.parent / "robot.usd"
            if args.map_yaml is None:
                args.map_yaml = args.artifact.parent / "map.yaml"
            from tinker_sim_isaac.backend import IsaacNavigationBackend
            backend = IsaacNavigationBackend(
                usd_path=args.artifact, map_yaml=args.map_yaml, seed=args.seed,
                render=not args.headless, enable_contacts=False,
            )
            if args.ros:
                from tinker_sim_isaac.ros_gateway import RosStandardGateway

                gateway = RosStandardGateway(backend, development_lidar=True)
            print(
                json.dumps(
                    {
                        "artifact": str(args.artifact),
                        "map": str(args.map_yaml),
                        "calibration": calibration.status.value,
                        "ros": args.ros,
                        "physics_device": backend.physics_device,
                        "timeline_end_time": backend.timeline_end_time,
                        "simulation_control": (
                            "isaacsim.ros2.sim_control" if args.ros else "disabled"
                        ),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            next_step_wall = time.monotonic()
            while (
                running
                and app.is_running()
                and (args.duration <= 0.0 or backend.simulation_time < args.duration)
            ):
                if gateway is not None:
                    gateway.spin_once()
                import omni.timeline

                if omni.timeline.get_timeline_interface().is_playing():
                    backend.step()
                    if gateway is not None:
                        if not running:
                            break
                        try:
                            gateway.publish()
                        except BaseException:
                            if running:
                                raise
                            break
                        if not running:
                            break
                        # SimulationContext performs a direct headless PhysX
                        # step.  NVIDIA's simulation-control callbacks run on
                        # Kit's asyncio loop, so pump one Kit update per ROS
                        # frame to execute standard service/action handlers.
                        app.update()
                    # DDS consumers and the wall-clock command watchdog are
                    # hardware-parity processes.  Keep ROS-integrated physics
                    # at real time so their queues and TF caches remain valid.
                    next_step_wall += backend.dt
                    remaining = next_step_wall - time.monotonic()
                    if remaining > 0.0:
                        time.sleep(remaining)
                    elif remaining < -1.0:
                        next_step_wall = time.monotonic()
                else:
                    # Simulation-control services run on a separate executor,
                    # while Kit still needs updates to apply stage/timeline work.
                    app.update()
                    time.sleep(0.001)
        elif args.sensor_profile == "manipulation-core":
            root = Path(__file__).resolve().parents[1]
            profile = json.loads(
                (root / "simulation/profiles/manipulation-core.json").read_text(encoding="utf-8")
            )
            if profile.get("physics_device") != "cpu" or profile.get("render") is not False:
                raise RuntimeError("manipulation-core profile must use CPU PhysX with render=false")
            if not profile.get("contacts"):
                raise RuntimeError("manipulation-core profile must enable contacts")
            artifact = _content_addressed_tinker_usd(root, args.artifact)
            expected_objects = _expected_scenario_objects(root, args.scenario)
            sys.path.insert(0, str(root / "simulation"))
            from tinker_sim_isaac.backend import IsaacWholeRobotBackend

            backend = IsaacWholeRobotBackend(
                usd_path=artifact,
                map_yaml=None,
                seed=args.seed,
                render=False,
                enable_contacts=True,
                add_ground_plane=True,
                expected_objects=expected_objects,
                scenario=args.scenario,
                task=args.scenario,
            )
            if backend.physics_device != "cpu":
                raise RuntimeError("manipulation-core selected a non-CPU physics device")
            if args.ros:
                from tinker_sim_isaac.ros_gateway import RosStandardGateway

                gateway = RosStandardGateway(backend)
            if args.qualification:
                from tinker_sim_isaac.qualification_visual_capture import (
                    QualificationVisualCapture,
                )

                visual_capture = QualificationVisualCapture.from_environment(
                    app=app,
                    backend=backend,
                    event_pump=gateway.spin_once if gateway is not None else None,
                )
            print(
                json.dumps(
                    {
                        "artifact": str(artifact),
                        "contacts": True,
                        "expected_objects": sorted(expected_objects),
                        "physics_device": backend.physics_device,
                        "profile": "manipulation-core",
                        "render": False,
                        "ros": args.ros,
                        "scenario": args.scenario,
                        "seed": args.seed,
                        "simulation_control": (
                            "isaacsim.ros2.sim_control" if args.ros else "disabled"
                        ),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            next_step_wall = time.monotonic()
            while (
                running
                and app.is_running()
                and (args.duration <= 0.0 or backend.simulation_time < args.duration)
            ):
                if gateway is not None:
                    gateway.spin_once()
                import omni.timeline

                if omni.timeline.get_timeline_interface().is_playing():
                    backend.step()
                    if gateway is not None:
                        if not running:
                            break
                        try:
                            gateway.publish()
                            if visual_capture is not None:
                                visual_capture.poll()
                        except BaseException:
                            if running:
                                raise
                            break
                        if not running:
                            break
                        app.update()
                    next_step_wall += backend.dt
                    remaining = next_step_wall - time.monotonic()
                    if remaining > 0.0:
                        time.sleep(remaining)
                    elif remaining < -1.0:
                        next_step_wall = time.monotonic()
                else:
                    app.update()
                    time.sleep(0.001)
        elif args.sensor_profile == "physics-only":
            from isaacsim.core.api import World

            world = World(stage_units_in_meters=1.0, backend="torch", device="cpu")
            world.scene.add_default_ground_plane()
            world.reset()
            print(
                json.dumps(
                    {
                        "profile": args.profile,
                        "scenario": args.scenario,
                        "seed": args.seed,
                        "sensor_profile": args.sensor_profile,
                        "physics_device": "cpu",
                        "simulation_control": (
                            "isaacsim.ros2.sim_control" if args.ros else "disabled"
                        ),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            while running and app.is_running():
                world.step(render=args.sensor_profile != "physics-only")
        else:
            raise RuntimeError(f"unsupported Isaac sensor profile: {args.sensor_profile}")
    except BaseException:
        # Fast Kit shutdown can terminate the interpreter before Python emits
        # an unhandled exception, so record it before extension teardown.
        failed = True
        traceback.print_exc()
    finally:
        if visual_capture is not None:
            visual_capture.close()
        if gateway is not None:
            gateway.close()
        app.close(wait_for_replicator=False, exit_code=1 if failed else 0)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
