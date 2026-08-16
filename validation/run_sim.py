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
        # 16 = legacy truncated identity; 64 = full sha256 written by current artifact-export
        or len(artifact.parent.name) not in (16, 64)
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


# --------------------------------------------------------------------------- #
# Task-8 fix round 3 (Option A+): qualification-only occupancy for the
# development LiDAR.  For ``manipulation-core`` runs with ``--qualification``,
# a pure 2-D ``OccupancyMap`` is built from the committed scenario
# ``planning_scene.objects`` box footprints in world coordinates.  The grid is
# a fixed deterministic resolution with a generous half-extent (well beyond the
# 40 m development-lidar range) so a 40 m ray never reaches the map boundary:
# out-of-bounds is never a fake obstacle.  Ordinary ``manipulation-core``
# without ``--qualification`` keeps ``backend.occupancy is None`` and
# ``development_lidar=False``; navigation-parity is unchanged.
# --------------------------------------------------------------------------- #

#: Development-lidar raycast maximum in the simulator gateway (meters).
_LIDAR_MAX_RANGE_M = 40.0
#: Grid resolution used for the qualification occupancy map.
_OCCUPANCY_RESOLUTION_M = 0.05
#: Half-extent of the square world-frame occupancy grid.  With the robot at the
#: canonical origin and the lidar 0.12 m ahead of base, a 40 m ray reaches at
#: most ~40.13 m from the origin, so a 60 m half-extent guarantees the map
#: boundary is never reached within the lidar range.
_OCCUPANCY_HALF_EXTENT_M = 60.0


def build_occupancy_from_planning_scene(
    objects: list[dict[str, object]],
    *,
    resolution: float = _OCCUPANCY_RESOLUTION_M,
    half_extent: float = _OCCUPANCY_HALF_EXTENT_M,
) -> object:
    """Build a pure deterministic ``OccupancyMap`` from scenario box footprints.

    Every PlanningScene object must be a box (``primitive.type == "box"`` with
    three finite positive ``dimensions``, a finite pose, and a normalized
    yaw-only quaternion — negligible roll/pitch, sign-equivalent quaternions
    accepted).  F4.6: the XY footprint of each box is rasterized by
    inverse-rotating each candidate cell center into box-local coordinates;
    oriented (e.g. the canonical 45 degree z-rotated target) boxes are
    supported deterministically.  Non-box primitives, malformed dimensions/
    poses/quaternions, and roll/pitch rotations are rejected rather than
    silently inventing occupancy.  The map covers ``[-half_extent, half_extent]^2``
    in world coordinates so the full 40 m lidar range stays inside bounds.
    """
    import math

    from tinker_sim_core.occupancy import OccupancyMap

    if isinstance(resolution, bool) or not isinstance(resolution, (int, float)) or not math.isfinite(float(resolution)) or float(resolution) <= 0.0:
        raise ValueError("occupancy resolution must be finite and positive")
    if isinstance(half_extent, bool) or not isinstance(half_extent, (int, float)) or not math.isfinite(float(half_extent)) or float(half_extent) <= 0.0:
        raise ValueError("occupancy half_extent must be finite and positive")
    resolution = float(resolution)
    half_extent = float(half_extent)
    if half_extent <= _LIDAR_MAX_RANGE_M + 2.0:
        raise ValueError("occupancy half_extent must exceed the 40 m lidar range")
    width = int(round(2.0 * half_extent / resolution))
    height = int(round(2.0 * half_extent / resolution))
    origin_x = -half_extent
    origin_y = -half_extent
    occupied: list[list[bool]] = [[False] * width for _ in range(height)]

    def _finite_vector(values: object, *, length: int, name: str) -> list[float]:
        if not isinstance(values, (list, tuple)) or len(values) != length:
            raise ValueError(f"{name} must be a sequence of {length} finite values")
        converted = []
        for value in values:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be finite")
            number = float(value)
            if not math.isfinite(number):
                raise ValueError(f"{name} must be finite")
            converted.append(number)
        return converted

    for record in objects:
        if not isinstance(record, dict) or not isinstance(record.get("id"), str):
            raise ValueError("planning_scene object must carry an id string")
        primitive = record.get("primitive")
        if not isinstance(primitive, dict) or primitive.get("type") != "box":
            raise ValueError(f"unsupported qualification fixture primitive for {record.get('id')!r}")
        dimensions = _finite_vector(primitive.get("dimensions"), length=3, name="box dimensions")
        if any(value <= 0.0 for value in dimensions):
            raise ValueError(f"box dimensions must be positive for {record.get('id')!r}")
        pose = record.get("pose")
        if not isinstance(pose, dict):
            raise ValueError(f"box fixture {record.get('id')!r} has no pose")
        xyz = _finite_vector(pose.get("xyz"), length=3, name="pose xyz")
        quaternion = _finite_vector(pose.get("quaternion_xyzw"), length=4, name="pose quaternion")
        qx, qy, qz, qw = quaternion
        # F4.6: normalize the quaternion (reject a non-unit quaternion outside
        # tolerance), canonicalize sign-equivalent rotations, and require
        # yaw-only (negligible roll/pitch).
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(norm) or abs(norm - 1.0) > 1.0e-3:
            raise ValueError(f"box quaternion must be normalized for {record.get('id')!r}")
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm
        if qw < 0.0:
            qx, qy, qz, qw = -qx, -qy, -qz, -qw
        if abs(qx) > 1.0e-6 or abs(qy) > 1.0e-6:
            raise ValueError(
                f"only yaw-only box rotations are supported for occupancy ({record.get('id')!r})"
            )
        cos_theta = 1.0 - 2.0 * qz * qz
        sin_theta = 2.0 * qw * qz
        cx, cy = xyz[0], xyz[1]
        half_x = dimensions[0] / 2.0
        half_y = dimensions[1] / 2.0
        # Conservative world-frame AABB of the oriented rectangle (yaw about z).
        corners = (
            (half_x, half_y),
            (half_x, -half_y),
            (-half_x, -half_y),
            (-half_x, half_y),
        )
        world_corners = [
            (
                cx + (px * cos_theta - py * sin_theta),
                cy + (px * sin_theta + py * cos_theta),
            )
            for (px, py) in corners
        ]
        min_wx = min(point[0] for point in world_corners)
        max_wx = max(point[0] for point in world_corners)
        min_wy = min(point[1] for point in world_corners)
        max_wy = max(point[1] for point in world_corners)
        raw_min_gx = int((min_wx - origin_x) // resolution)
        raw_max_gx = int((max_wx - origin_x) // resolution)
        raw_min_gy = int((min_wy - origin_y) // resolution)
        raw_max_gy = int((max_wy - origin_y) // resolution)
        if raw_max_gx < 0 or raw_min_gx >= width or raw_max_gy < 0 or raw_min_gy >= height:
            continue
        gx0 = max(raw_min_gx, 0)
        gx1 = min(raw_max_gx, width - 1)
        gy0 = max(raw_min_gy, 0)
        gy1 = min(raw_max_gy, height - 1)
        # Mark each candidate cell whose center inverse-rotates inside the box
        # half extents (small deterministic numeric tolerance).
        fit_tol = 1.0e-6
        for gy in range(gy0, gy1 + 1):
            row = occupied[gy]
            for gx in range(gx0, gx1 + 1):
                world_x = origin_x + (gx + 0.5) * resolution
                world_y = origin_y + (gy + 0.5) * resolution
                dx = world_x - cx
                dy = world_y - cy
                local_x = dx * cos_theta + dy * sin_theta
                local_y = -dx * sin_theta + dy * cos_theta
                if abs(local_x) <= half_x + fit_tol and abs(local_y) <= half_y + fit_tol:
                    row[gx] = True
    return OccupancyMap(
        width,
        height,
        resolution,
        origin_x,
        origin_y,
        tuple(tuple(row) for row in occupied),
    )


def qualification_occupancy(root: Path, scenario_name: str) -> object | None:
    """Return the qualification-only ``OccupancyMap`` for the committed scenario.

    ``None`` for ``"empty"`` and for scenarios without PlanningScene box
    geometry; a deterministic map from the scenario ``planning_scene.objects``
    otherwise.  Malformed/unsupported fixture geometry raises (never silently
    invents occupancy).
    """
    if scenario_name == "empty":
        return None
    sys.path.insert(0, str(root / "simulation"))
    from tinker_sim_core.scenario import load_named_scenario

    scenario = load_named_scenario(root, scenario_name)
    planning_scene = scenario.planning_scene
    objects = planning_scene.get("objects", ()) if isinstance(planning_scene, dict) else ()
    if not objects:
        return None
    return build_occupancy_from_planning_scene(list(objects))


def gateway_lidar_enabled(sensor_profile: str, qualification: bool) -> bool:
    """Resolve the development-lidar gateway flag per sensor profile.

    navigation-parity always enables development LiDAR (unchanged).  Only
    ``manipulation-core`` qualification runs enable it; ordinary
    ``manipulation-core`` and other profiles keep it disabled.
    """
    if sensor_profile in ("navigation-parity", "sensor-rich"):
        return True
    if sensor_profile == "manipulation-core":
        return bool(qualification)
    return False


def sensor_rich_implies_ros(sensor_profile: str, ros: bool) -> bool:
    """sensor-rich exists to serve hardware-parity topics; it forces --ros."""
    return sensor_profile == "sensor-rich" and not ros


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
    parser.add_argument("--camera-pointcloud", action="store_true")
    parser.add_argument("--arena-colors", action="store_true")
    args, kit_args = parser.parse_known_args()

    if args.camera_pointcloud and args.sensor_profile != "sensor-rich":
        parser.error("--camera-pointcloud is supported only with sensor-rich")
    if args.arena_colors and args.sensor_profile != "sensor-rich":
        parser.error("--arena-colors is supported only with sensor-rich")
    if sensor_rich_implies_ros(args.sensor_profile, args.ros):
        print("sensor-rich implies --ros; enabling the ROS gateway", flush=True)
        args.ros = True

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

                gateway = RosStandardGateway(
                    backend,
                    development_lidar=gateway_lidar_enabled(args.sensor_profile, args.qualification),
                )
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
            next_collision_heartbeat = time.monotonic()
            collision_heartbeat_period_s = 0.1
            while (
                running
                and app.is_running()
                and (args.duration <= 0.0 or backend.simulation_time < args.duration)
            ):
                if gateway is not None:
                    gateway.spin_once()
                import omni.timeline

                if (
                    gateway is not None
                    and time.monotonic() - next_collision_heartbeat
                    >= collision_heartbeat_period_s
                ):
                    # The collision source must stay fresh while paused (world
                    # load/spawn), otherwise the supervisor trips a spurious
                    # stop + controller deactivate on the first cold start.
                    gateway.publish_safety_heartbeat()
                    next_collision_heartbeat = time.monotonic()

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
        elif args.sensor_profile == "sensor-rich":
            root = Path(__file__).resolve().parents[1]
            sys.path.insert(0, str(root / "simulation"))
            if args.artifact is None:
                current = json.loads(
                    (root / "artifacts/robot/tinker2/current.json").read_text()
                )
                manifest_path = Path(current["manifest"])
                if not manifest_path.is_absolute():
                    manifest_path = root / manifest_path
                args.artifact = manifest_path.parent / "robot.usd"
            if args.map_yaml is None:
                args.map_yaml = args.artifact.parent / "map.yaml"
            expected_objects = _expected_scenario_objects(root, args.scenario)
            wall_color_fn = None
            if args.arena_colors:
                from tinker_sim_core.arena_palette import wall_color

                wall_color_fn = lambda index: wall_color(index)[1]  # noqa: E731
            from tinker_sim_isaac.backend import IsaacWholeRobotBackend

            backend = IsaacWholeRobotBackend(
                usd_path=args.artifact,
                map_yaml=args.map_yaml,
                seed=args.seed,
                render=False,
                enable_contacts=False,
                add_ground_plane=True,
                expected_objects=expected_objects,
                scenario=args.scenario,
                task=args.scenario,
                wall_color_fn=wall_color_fn,
            )
            from tinker_sim_isaac.camera_rig import CameraRig, load_camera_specs

            camera_specs = load_camera_specs(
                root / "simulation/sensors/hardware-parity.json"
            )
            camera_rig = CameraRig(camera_specs)
            camera_rig.initialize(app)
            from tinker_sim_isaac.ros_gateway import RosStandardGateway

            gateway = RosStandardGateway(
                backend,
                development_lidar=gateway_lidar_enabled(
                    args.sensor_profile, args.qualification
                ),
                camera_rig=camera_rig,
                camera_pointcloud=args.camera_pointcloud,
            )
            camera_hz = min(spec.tick_rate_hz for spec in camera_specs)
            camera_stride = _streaming_update_stride(backend.dt, update_hz=camera_hz)
            print(
                json.dumps(
                    {
                        "artifact": str(args.artifact),
                        "map": str(args.map_yaml),
                        "arena_colors": args.arena_colors,
                        "cameras": {
                            spec.name: {
                                "color_topic": spec.color_topic,
                                "depth_topic": spec.depth_topic,
                                "camera_info_topics": list(spec.camera_info_topics),
                                "frame_id": spec.frame_id,
                                "resolution": [spec.width, spec.height],
                                "target_hz": camera_hz,
                            }
                            for spec in camera_specs
                        },
                        "camera_pointcloud": args.camera_pointcloud,
                        "physics_device": backend.physics_device,
                        "profile": "sensor-rich",
                        "ros": args.ros,
                        "scenario": args.scenario,
                        "seed": args.seed,
                        "simulation_control": "isaacsim.ros2.sim_control",
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            import omni.timeline

            # No external scenario runner drives this profile by default;
            # start the timeline so cameras and physics run immediately.
            # isaacsim.ros2.sim_control can still pause/resume it.
            omni.timeline.get_timeline_interface().play()
            next_step_wall = time.monotonic()
            next_collision_heartbeat = time.monotonic()
            collision_heartbeat_period_s = 0.1
            camera_frame_index = 0
            while (
                running
                and app.is_running()
                and (args.duration <= 0.0 or backend.simulation_time < args.duration)
            ):
                gateway.spin_once()
                if (
                    time.monotonic() - next_collision_heartbeat
                    >= collision_heartbeat_period_s
                ):
                    gateway.publish_safety_heartbeat()
                    next_collision_heartbeat = time.monotonic()
                if omni.timeline.get_timeline_interface().is_playing():
                    backend.step()
                    if not running:
                        break
                    try:
                        gateway.publish()
                        camera_frame_index += 1
                        if camera_frame_index % camera_stride == 0:
                            gateway.publish_cameras()
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
            # Task-8 fix round 3 (Option A+): qualification-only occupancy for
            # the development LiDAR.  Ordinary manipulation-core keeps
            # ``backend.occupancy is None`` and ``development_lidar=False``.
            if args.qualification:
                backend.occupancy = qualification_occupancy(root, args.scenario)
            if args.ros:
                from tinker_sim_isaac.ros_gateway import RosStandardGateway

                gateway = RosStandardGateway(
                    backend,
                    development_lidar=gateway_lidar_enabled(args.sensor_profile, args.qualification),
                )
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
            next_collision_heartbeat = time.monotonic()
            collision_heartbeat_period_s = 0.1
            while (
                running
                and app.is_running()
                and (args.duration <= 0.0 or backend.simulation_time < args.duration)
            ):
                if gateway is not None:
                    gateway.spin_once()
                import omni.timeline

                if (
                    gateway is not None
                    and time.monotonic() - next_collision_heartbeat
                    >= collision_heartbeat_period_s
                ):
                    # The collision source must stay fresh while paused (world
                    # load/spawn), otherwise the supervisor trips a spurious
                    # stop + controller deactivate on the first cold start.
                    gateway.publish_safety_heartbeat()
                    next_collision_heartbeat = time.monotonic()

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
            while (
                running
                and app.is_running()
                and (args.duration <= 0.0 or world.current_time < args.duration)
            ):
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
