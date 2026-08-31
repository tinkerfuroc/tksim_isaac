#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
import traceback
from pathlib import Path


STREAM_SIGNAL_PORT = 49100
STREAM_MEDIA_PORT = 47998
STREAM_RESOLUTION = (1280, 720)
STREAM_UPDATE_HZ = 10.0
STREAM_READY_FILE = Path("/tmp/tinker-sim-arena-streaming.ready.json")


def _streaming_application_config(kit_args: list[str]) -> dict[str, object]:
    width, height = STREAM_RESOLUTION
    return {
        "headless": True,
        "hide_ui": False,
        "renderer": "RaytracedLighting",
        # WebRTC's primary app stream captures the application surface, while
        # Isaac's renderer and window have independent size settings. Start
        # them at the same size, then let the primary client resize the app
        # surface through Isaac's supported dynamic-resize path.
        "width": width,
        "height": height,
        "window_width": width,
        "window_height": height,
        "multi_gpu": False,
        "max_gpu_count": 1,
        "physics_gpu": -1,
        "extra_args": [
            "--/physics/useGpu=false",
            "--/physics/cudaDevice=-1",
            "--/exts/omni.kit.livestream.app/primaryStream/allowDynamicResize=true",
            "--/exts/omni.kit.livestream.app/primaryStream/publicIp=127.0.0.1",
            f"--/exts/omni.kit.livestream.app/primaryStream/signalPort={STREAM_SIGNAL_PORT}",
            f"--/exts/omni.kit.livestream.app/primaryStream/streamPort={STREAM_MEDIA_PORT}",
            *kit_args,
        ],
    }


def _streaming_experience(isaacsim_package: Path) -> Path:
    experience = isaacsim_package.resolve().parent / "apps" / "isaacsim.exp.full.streaming.kit"
    if not experience.is_file():
        raise RuntimeError(f"Isaac streaming experience is missing: {experience}")
    return experience


def _pump_streaming_app_update(app: object, settings: object) -> None:
    """Pump one Kit frame so livestream input reaches the headless UI.

    Isaac Lab owns the explicit physics step in this process.  Kit's livestream
    extension, however, consumes remote mouse/keyboard events during PreUpdate.
    Temporarily disable Kit-owned simulation stepping around ``app.update()`` so
    input and UI events are processed without advancing PhysX a second time.
    This mirrors Isaac Lab's KitVisualizer update boundary.
    """
    play_simulations = "/app/player/playSimulations"
    settings.set_bool(play_simulations, False)
    try:
        app.update()
    finally:
        settings.set_bool(play_simulations, True)


def _emit_step_profile(
    prof: dict,
    backend_breakdown: dict | None = None,
    publish_breakdown: dict | None = None,
    camera_breakdown: dict | None = None,
    spin_breakdown: dict | None = None,
    sim_time: float | None = None,
) -> None:
    """Print where sensor-rich wall time actually goes, then reset the window.

    Opt-in via TINKER_SIM_PROFILE=1. The sensor-rich loop is latency-bound
    rather than compute-bound (the GPU idles at a few percent while stepping
    collapses to single-digit Hz), so the only way to target an optimisation
    is to attribute wall time across the four things the loop does per camera
    cycle: the PhysX step, the lightweight ROS publish, the Kit app update
    that renders both RTX products, and the camera capture/convert/publish.
    """
    import json
    import sys

    cycles = max(1, prof["cycles"])
    steps = max(1, prof["physics_n"])
    total = prof["physics"] + prof["publish"] + prof["kit_pump"] + prof["cameras"]
    # Wall time per cycle not covered by the four buckets: gateway.spin_once,
    # the heartbeat, pacing sleeps and anything the GIL gave away to the
    # gateway's executor thread. This is where a live stack's cost hides.
    spin = prof.get("spin", 0.0)
    wall = prof.get("wall", 0.0)
    other = max(0.0, wall - total - spin)
    import time as _time

    payload = {
        "step_profile": {
            "wall_time": round(_time.time(), 3),
            "sim_time": None if sim_time is None else round(float(sim_time), 3),
            "cycles": prof["cycles"],
            "physics_steps": prof["physics_n"],
            "ms_per_physics_step": round(1000.0 * prof["physics"] / steps, 3),
            "ms_per_cycle": {
                "physics": round(1000.0 * prof["physics"] / cycles, 2),
                "publish": round(1000.0 * prof["publish"] / cycles, 2),
                "kit_pump": round(1000.0 * prof["kit_pump"] / cycles, 2),
                "cameras": round(1000.0 * prof["cameras"] / cycles, 2),
                "total": round(1000.0 * total / cycles, 2),
                "spin": round(1000.0 * spin / cycles, 2),
                "unaccounted": round(1000.0 * other / cycles, 2),
                "wall": round(1000.0 * wall / cycles, 2),
            },
            "share_pct": {
                key: round(100.0 * prof[key] / total, 1) if total > 0 else 0.0
                for key in ("physics", "publish", "kit_pump", "cameras")
            },
        }
    }
    if backend_breakdown is not None:
        payload["step_profile"]["physics_breakdown_ms"] = backend_breakdown
    if publish_breakdown is not None:
        payload["step_profile"]["publish_breakdown_ms"] = publish_breakdown
    if camera_breakdown is not None:
        payload["step_profile"]["camera_breakdown_ms"] = camera_breakdown
    if spin_breakdown is not None:
        payload["step_profile"]["spin_breakdown"] = spin_breakdown
    print(json.dumps(payload, sort_keys=True), flush=True)
    sys.stdout.flush()
    for key in ("physics", "publish", "kit_pump", "cameras", "spin", "wall"):
        prof[key] = 0.0
    prof["cycles"] = 0
    prof["physics_n"] = 0


def _streaming_update_stride(
    physics_dt: float,
    update_hz: float = STREAM_UPDATE_HZ,
) -> int:
    """Return the number of physics frames between streamed Kit updates.

    Rendering the full application surface on every 120 Hz CPU-PhysX frame
    couples simulation time to the much slower ray-traced stream. Keep physics
    authoritative and service UI/input/video at its own bounded cadence.
    """
    if physics_dt <= 0.0 or update_hz <= 0.0:
        raise ValueError("physics_dt and streaming update_hz must be positive")
    return max(1, round(1.0 / (physics_dt * update_hz)))


def _resolve_camera_hz(parity_hz: float, override: str | None) -> float:
    """Resolve the camera cadence, honouring an explicit opt-in override.

    `simulation/sensors/hardware-parity.json` states what the real cameras do
    and stays authoritative: with no override its value is returned unchanged.
    `TINKER_SIM_CAMERA_HZ` lets a simulation run publish at a lower rate the
    hardware also sustains, which halves both the Kit render pump and the image
    payload per simulated second -- the two dominant costs once real
    subscribers are attached. Raising the rate above the hardware's is refused;
    that would be a parity violation, not an optimisation.
    """
    if override is None or not str(override).strip():
        return float(parity_hz)
    try:
        value = float(str(override).strip())
    except (TypeError, ValueError):
        raise ValueError(
            f"TINKER_SIM_CAMERA_HZ must be a number, got {override!r}"
        ) from None
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(
            f"TINKER_SIM_CAMERA_HZ must be finite and positive, got {override!r}"
        )
    if value > float(parity_hz):
        raise ValueError(
            f"TINKER_SIM_CAMERA_HZ={value} exceeds the hardware rate "
            f"{parity_hz}; the simulation must not out-run the real camera"
        )
    return value


class _StreamingSessionLifecycle:
    """Track the primary client's real connection lifecycle.

    The WebRTC extension can emit disconnect notifications for abandoned
    connection attempts. Only a disconnect after its matching first successful
    connection ends this single-viewer application session.
    """

    def __init__(self) -> None:
        self.connected = False
        self.ended = False
        self.ready = False

    def mark_ready(self) -> None:
        self.ready = True

    def on_connected(self, _event: object) -> None:
        self.connected = True

    def on_disconnected(self, _event: object) -> None:
        if self.connected and self.ready:
            self.ended = True
        self.connected = False


def _write_stream_ready_file(path: Path = STREAM_READY_FILE) -> None:
    payload = {
        "media_port_udp": STREAM_MEDIA_PORT,
        "pid": os.getpid(),
        "signal_port_tcp": STREAM_SIGNAL_PORT,
        "state": "ready",
    }
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(payload, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def _clear_stream_ready_file(path: Path = STREAM_READY_FILE) -> None:
    path.unlink(missing_ok=True)


def _arena_camera_pose(occupancy: object) -> tuple[list[float], list[float], list[float]]:
    from tinker_sim_isaac.arena_camera import arena_camera_pose

    return arena_camera_pose(occupancy)


#: Opt-in gate: truthy disables the wrist camera (its RTX render product is
#: never created). NOT used by gpsr-stack battery runs any more: the theory
#: this flag was introduced for -- that capping concurrent RTX render
#: products at 2 (of head/wrist/arena) avoids a CUDA illegal-memory-access
#: (error 700) under GPU contention -- was disproved live (see
#: .superpowers/sdd/2026-08-25-gpsr-recorded-sim-battery/task-9-report.md,
#: attempt 8: error 700 recurred with only 2 products active). The battery's
#: actual fix was parking the arena camera outright (commit 1e730d4); the
#: sim now runs its stock head+wrist cameras in both mock and live mode.
#: This flag is retained as a manual operator escape hatch (e.g. isolating a
#: wrist-camera-specific repro) -- but it is incompatible with
#: ``scripts/gpsr-stack``: that script's "sim" gate now requires the wrist
#: camera's census stack unconditionally (see
#: tools/gpsr_interface_census.py's "sim cameras wrist" topics and
#: ``_gate_census_stacks`` in scripts/gpsr-stack), so setting this env var
#: for a ``gpsr-stack up`` run hangs that gate for the full 180s timeout and
#: then tears the whole stack down.
DISABLE_WRIST_CAMERA_ENV = "TINKER_SIM_DISABLE_WRIST_CAMERA"
_TRUTHY = {"1", "true", "yes"}


def _without_wrist_camera(specs: tuple, env: dict) -> tuple:
    """Drop the ``wrist_camera`` spec from ``specs`` when
    ``TINKER_SIM_DISABLE_WRIST_CAMERA`` is a truthy literal ("1", "true",
    "yes", case-insensitive); otherwise ``specs`` is returned unchanged.

    Pure. Must run before ``_with_arena_camera`` (so a disabled wrist
    camera never contributes to ``robot_min_hz``, and the arena camera's
    append still lands after it) and before any RTX render products are
    created.
    """
    raw = env.get(DISABLE_WRIST_CAMERA_ENV)
    if raw is None or raw.strip().lower() not in _TRUTHY:
        return specs
    return tuple(spec for spec in specs if spec.name != "wrist_camera")


def _with_arena_camera(
    specs: tuple, occupancy: object, env: dict
) -> tuple[tuple, float]:
    """Opt-in append of the arena observer camera to the robot's ``specs``.

    ``robot_min_hz`` is always ``min(tick_rate_hz)`` over the *original*
    ``specs`` only -- the arena camera (a slow, fixed overview stream) must
    never lower the robot cameras' resolved cadence, whether or not it is
    itself enabled via ``TINKER_SIM_ARENA_CAMERA``.
    """
    robot_min_hz = min(spec.tick_rate_hz for spec in specs)
    from tinker_sim_isaac.arena_camera import (
        arena_camera_spec, resolve_arena_camera, resolve_arena_camera_size,
    )

    hz = resolve_arena_camera(env)
    if hz is None:
        return specs, robot_min_hz
    size = resolve_arena_camera_size(env)
    return specs + (arena_camera_spec(occupancy, hz=hz, size=size),), robot_min_hz


def _arena_camera_enabled(camera_specs: tuple) -> bool:
    """True when ``_with_arena_camera`` appended the arena camera's spec.

    ``camera_specs`` is always non-empty (``load_camera_specs`` guarantees at
    least head+wrist); ``_with_arena_camera`` only ever appends, so the
    arena camera -- when present -- is always last.
    """
    return camera_specs[-1].name == "arena_camera"


#: A/B aid for the RTF work. Unset: the pin follows the arena camera's
#: presence, scoped to just its render product (see _stable_aa_cameras) --
#: on with the arena camera, off without it. "1" forces the pin on even
#: without the arena camera, which (via _stable_aa_cameras) falls back to
#: the historical *global* pin across every render product, isolating its
#: per-frame cost on the parity cameras for A/B comparison. "0" forces the
#: pin off with the arena camera on (only for the crash-recipe re-check;
#: see docs/superpowers/specs/2026-08-29-arena-camera-rtf-design.md).
STABLE_AA_ENV = "TINKER_SIM_STABLE_AA"

#: The DLAA pin, when requested, is scoped to just the arena camera's render
#: product -- see the docstring above camera_rig.initialize's call site.
STABLE_AA_CAMERAS = frozenset({"arena_camera"})


def _stable_aa_requested(arena_enabled: bool, env: dict) -> bool:
    """DLAA pin decision: the arena camera's presence unless the env forces it."""
    raw = env.get(STABLE_AA_ENV)
    if raw is None:
        return arena_enabled
    return raw.strip().lower() in _TRUTHY


def _stable_aa_cameras(arena_enabled: bool) -> frozenset[str] | None:
    """Per-product scope for the DLAA pin: the arena product when it exists,
    else ``None`` so a forced pin (``TINKER_SIM_STABLE_AA=1`` without the arena)
    falls back to the historical global pin -- what the spike's variant D measures."""
    return STABLE_AA_CAMERAS if arena_enabled else None


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


def resolve_arena_artifact(root: Path, arena_id: str) -> Path:
    """Resolve ``--arena <arena_id>`` to its content-addressed artifact dir.

    Mirrors the robot-artifact pointer consumption above (``current.json`` ->
    manifest -> payload directory), fixed to the arena asset kind. Fails
    closed with ``FileNotFoundError``/``ValueError`` on a missing pointer
    file, a missing or invalid ``manifest`` key, a missing manifest file, or
    a missing ``arena.usd``/``map.yaml`` payload in the resolved directory.
    """
    pointer_path = root / "artifacts/arena" / arena_id / "current.json"
    if not pointer_path.is_file():
        raise FileNotFoundError(f"missing arena artifact pointer: {pointer_path}")
    current = json.loads(pointer_path.read_text(encoding="utf-8"))
    manifest_value = current.get("manifest")
    if not isinstance(manifest_value, str) or not manifest_value:
        raise ValueError(f"arena artifact pointer missing 'manifest': {pointer_path}")
    manifest_path = Path(manifest_value)
    if not manifest_path.is_absolute():
        manifest_path = root / manifest_path
    if not manifest_path.is_file():
        raise FileNotFoundError(f"missing arena artifact manifest: {manifest_path}")
    artifact_dir = manifest_path.parent
    if not (artifact_dir / "arena.usd").is_file():
        raise FileNotFoundError(f"arena artifact missing arena.usd: {artifact_dir}")
    if not (artifact_dir / "map.yaml").is_file():
        raise FileNotFoundError(f"arena artifact missing map.yaml: {artifact_dir}")
    return artifact_dir


def _expected_scenario_objects(
    root: Path, scenario_name: str, arena_id: str | None = None
) -> dict[str, dict[str, object]]:
    if scenario_name == "empty":
        return {}
    sys.path.insert(0, str(root / "simulation"))
    from tinker_sim_core.scenario import load_named_scenario, validate_world_selection

    scenario = load_named_scenario(root, scenario_name)
    for warning in validate_world_selection(scenario, arena_id):
        print(json.dumps({"world_selection_warning": warning}), flush=True)
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


def arena_flag_supported(sensor_profile: str) -> bool:
    """``--arena`` requires a profile that loads the robot backend.

    ``physics-only`` builds a bare ``isaacsim.core.api.World`` with no
    ``IsaacWholeRobotBackend``/``IsaacNavigationBackend`` to reference an
    arena artifact into -- the physics-only branch never even reads
    ``args.arena``, so passing it there was silently ignored rather than
    rejected (Final review Finding 4).
    """
    return sensor_profile != "physics-only"


def parse_spawn_xy(text: str) -> tuple[float, float]:
    """Parse ``--spawn-xy X,Y`` strictly.

    The default arena spawn (0, 0) sits inside shelf_02's footprint in the
    rcw2026 arena; navigation runs pass an explicit free-space spawn instead.
    """
    parts = text.split(",")
    if len(parts) != 2:
        raise ValueError("--spawn-xy must be two comma-separated numbers: X,Y")
    try:
        x, y = float(parts[0]), float(parts[1])
    except ValueError:
        raise ValueError("--spawn-xy values must be numbers")
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("--spawn-xy values must be finite")
    return x, y


SPAWN_CLEARANCE_M = 0.35  # robot inscribed radius 0.25 m + margin


def validate_arena_spawn(arena_dir: Path, spawn_xy: tuple[float, float]) -> None:
    """Fail closed when the robot spawn lacks clearance on the arena map.

    The rcw2026 default spawn (0, 0) sits inside shelf_02's rasterized
    footprint; a spawn inside furniture corrupts odometry, lidar, and AMCL.
    The error names the nearest free cell so the user can retry.

    Every caller reaches this from one of the three sensor-profile branch
    preambles in ``main()`` (``navigation-parity``, ``sensor-rich``,
    ``manipulation-core``), each of which already puts ``root/simulation`` on
    ``sys.path`` before calling this function, so the imports below resolve
    without any path surgery here.
    """
    from tinker_sim_core.occupancy import OccupancyMap
    from tinker_sim_isaac.backend import _map_metadata

    pgm, resolution, origin_x, origin_y = _map_metadata(arena_dir / "map.yaml")
    occupancy = OccupancyMap.from_pgm(
        pgm, resolution=resolution, origin_x=origin_x, origin_y=origin_y
    )
    x, y = spawn_xy
    if occupancy.free_with_clearance(x, y, SPAWN_CLEARANCE_M):
        return
    suggestion = occupancy.nearest_free_world(x, y, SPAWN_CLEARANCE_M)
    if suggestion is None:
        raise RuntimeError(
            f"arena spawn ({x}, {y}) is obstructed and no free cell was found "
            f"within 5 m on the derived map"
        )
    raise RuntimeError(
        f"arena spawn ({x}, {y}) lacks {SPAWN_CLEARANCE_M} m clearance on the "
        f"derived map; try --spawn-xy={suggestion[0]},{suggestion[1]}"
    )


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
    parser.add_argument("--arena")
    parser.add_argument("--spawn-xy")
    parser.add_argument("--ros", action="store_true")
    parser.add_argument("--duration", type=float, default=0.0, help="simulation seconds; 0 runs until signalled")
    parser.add_argument("--qualification", action="store_true")
    parser.add_argument("--livestream", action="store_true")
    parser.add_argument("--camera-pointcloud", action="store_true")
    parser.add_argument("--arena-colors", action="store_true")
    args, kit_args = parser.parse_known_args()

    if args.livestream and args.sensor_profile != "navigation-parity":
        parser.error("--livestream is supported only with navigation-parity")
    if args.livestream and not args.headless:
        parser.error("--livestream requires --headless")
    if args.camera_pointcloud and args.sensor_profile != "sensor-rich":
        parser.error("--camera-pointcloud is supported only with sensor-rich")
    if args.arena_colors and args.sensor_profile != "sensor-rich":
        parser.error("--arena-colors is supported only with sensor-rich")
    if args.arena and not arena_flag_supported(args.sensor_profile):
        parser.error("--arena requires a profile that loads the robot backend")
    if args.arena and args.map_yaml is not None:
        parser.error("--arena and --map are mutually exclusive")
    if args.arena and args.arena_colors:
        parser.error("--arena-colors applies only to occupancy cuboid walls")
    if args.spawn_xy is not None and not arena_flag_supported(args.sensor_profile):
        parser.error("--spawn-xy requires a profile that loads the robot backend")
    spawn_xy = (0.0, 0.0)
    if args.spawn_xy is not None:
        try:
            spawn_xy = parse_spawn_xy(args.spawn_xy)
        except ValueError as error:
            parser.error(str(error))
    if sensor_rich_implies_ros(args.sensor_profile, args.ros):
        print("sensor-rich implies --ros; enabling the ROS gateway", flush=True)
        args.ros = True

    stream_ready_file = Path(
        os.environ.get("TINKER_SIM_STREAM_READY_FILE", str(STREAM_READY_FILE))
    )
    if args.livestream:
        _clear_stream_ready_file(stream_ready_file)

    import isaacsim
    from isaacsim import SimulationApp

    application_config = {
        "headless": args.headless,
        # sensor-rich renders through its RTX camera products only; the app
        # surface would be a third ray-traced render nobody consumes.
        "disable_viewport_updates": args.sensor_profile in ("physics-only", "sensor-rich"),
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
    switch_interval = os.environ.get("TINKER_SIM_GIL_SWITCH_INTERVAL_MS")
    if switch_interval:
        # The ROS gateway's executor thread competes for the GIL with the
        # simulation loop.  Every GIL release in the loop (PhysX, Kit, Warp,
        # rcl publish) can then stall for a full switch interval (5 ms by
        # default) while a command backlog is being drained.
        import sys as _sys
        _sys.setswitchinterval(max(0.05, float(switch_interval)) / 1000.0)
        print(json.dumps({"gil_switch_interval_ms": _sys.getswitchinterval() * 1000.0}), flush=True)
    cpu_threads = os.environ.get("TINKER_SIM_CPU_THREADS")
    if cpu_threads:
        # Kit defaults to min(cpu_count, 32) worker threads.  Under a live ROS
        # stack that leaves no cores for the stack, so allow an explicit cap.
        application_config["limit_cpu_threads"] = max(1, int(cpu_threads))
        print(json.dumps({"kit_cpu_threads": application_config["limit_cpu_threads"]}), flush=True)
    experience = ""
    if args.livestream:
        experience = str(_streaming_experience(Path(isaacsim.__file__)))
        application_config.update(_streaming_application_config(kit_args))
    if args.sensor_profile == "manipulation-core" and args.qualification:
        application_config.update(
            {
                "renderer": "RaytracedLighting",
                "width": 960,
                "height": 540,
            }
        )
    app = SimulationApp({**application_config}, experience=experience)
    streaming_settings = None
    if args.livestream:
        import carb.settings

        streaming_settings = carb.settings.get_settings()
    running = True
    streaming_lifecycle = None
    streaming_event_subscriptions: list[object] = []
    if args.livestream:
        import carb.eventdispatcher

        streaming_lifecycle = _StreamingSessionLifecycle()
        dispatcher = carb.eventdispatcher.get_eventdispatcher()
        streaming_event_subscriptions = [
            dispatcher.observe_event(
                observer_name="tinker-sim.streaming-client-connected",
                event_name="omni.kit.livestream.client_connected:immediate",
                on_event=streaming_lifecycle.on_connected,
            ),
            dispatcher.observe_event(
                observer_name="tinker-sim.streaming-client-disconnected",
                event_name="omni.kit.livestream.client_disconnected:immediate",
                on_event=streaming_lifecycle.on_disconnected,
            ),
        ]

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
            arena_dir = None
            if args.arena:
                arena_dir = resolve_arena_artifact(root, args.arena)
            elif args.map_yaml is None:
                args.map_yaml = args.artifact.parent / "map.yaml"
            if arena_dir is not None:
                validate_arena_spawn(arena_dir, spawn_xy)
            expected_objects = _expected_scenario_objects(root, args.scenario, args.arena)
            from tinker_sim_isaac.backend import IsaacNavigationBackend
            backend = IsaacNavigationBackend(
                usd_path=args.artifact, map_yaml=args.map_yaml, seed=args.seed,
                render=args.livestream or not args.headless, enable_contacts=False,
                arena_artifact=arena_dir, spawn_xy=spawn_xy,
                expected_objects=expected_objects, scenario=args.scenario,
                task=args.scenario,
            )
            arena_camera_eye = None
            arena_camera_target = None
            arena_bounds = None
            arena_collider_count = 0
            viewport_ready = None
            if backend.occupancy is not None:
                arena_collider_count = len(backend.occupancy.rectangles())
            if args.livestream:
                if backend.occupancy is None:
                    raise RuntimeError("arena streaming requires a loaded occupancy map")
                from isaacsim.core.rendering_manager import ViewportManager

                arena_camera_eye, arena_camera_target, arena_bounds = _arena_camera_pose(
                    backend.occupancy
                )
                camera = ViewportManager.get_camera()
                ViewportManager.set_camera_view(
                    camera,
                    eye=arena_camera_eye,
                    target=arena_camera_target,
                )
                viewport_ready, _ = ViewportManager.wait_for_viewport(max_frames=120)
                if not viewport_ready:
                    raise RuntimeError("arena streaming viewport did not become ready")
                for _ in range(4):
                    backend.render_frame()
            if args.ros:
                from tinker_sim_isaac.ros_gateway import RosStandardGateway

                gateway = RosStandardGateway(
                    backend,
                    development_lidar=gateway_lidar_enabled(args.sensor_profile, args.qualification),
                )
            stream_update_stride = 1
            stream_physics_frames = 0
            if args.livestream:
                # The guarded Kit update below renders the app stream. Avoid a
                # second full render inside every SimulationContext step.
                backend.render = False
                stream_update_stride = _streaming_update_stride(backend.dt)
            print(
                json.dumps(
                    {
                        "arena": args.arena,
                        "artifact": str(args.artifact),
                        "map": str(args.map_yaml),
                        "arena": {
                            "bounds_xy": arena_bounds,
                            "camera_eye": arena_camera_eye,
                            "camera_target": arena_camera_target,
                            "collider_count": arena_collider_count,
                            "id": "robocup-arena3",
                        },
                        "calibration": calibration.status.value,
                        "livestream": {
                            "dynamic_resize": args.livestream,
                            "enabled": args.livestream,
                            "media_port_udp": STREAM_MEDIA_PORT if args.livestream else None,
                            "quit_on_session_end": True if args.livestream else None,
                            "resolution": list(STREAM_RESOLUTION) if args.livestream else None,
                            "signal_port_tcp": STREAM_SIGNAL_PORT if args.livestream else None,
                            "update_hz": STREAM_UPDATE_HZ if args.livestream else None,
                            "viewport_ready": viewport_ready,
                        },
                        "ros": args.ros,
                        "physics_device": backend.physics_device,
                        "chassis_ballast_mass_kg": backend.chassis_ballast_mass_kg,
                        "timeline_end_time": backend.timeline_end_time,
                        "simulation_control": (
                            "isaacsim.ros2.sim_control" if args.ros else "disabled"
                        ),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            if args.livestream:
                if streaming_lifecycle is None:
                    raise RuntimeError("livestream session lifecycle was not initialized")
                streaming_lifecycle.mark_ready()
                _write_stream_ready_file(stream_ready_file)
            next_step_wall = time.monotonic()
            next_collision_heartbeat = time.monotonic()
            collision_heartbeat_period_s = 0.1
            while (
                running
                and app.is_running()
                and not (streaming_lifecycle is not None and streaming_lifecycle.ended)
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
                    if args.livestream:
                        stream_physics_frames += 1
                        if stream_physics_frames % stream_update_stride == 0:
                            # Service remote input and stream video without
                            # letting Kit issue a second physics step.
                            _pump_streaming_app_update(app, streaming_settings)
                    elif gateway is not None:
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
            arena_dir = None
            if args.arena:
                arena_dir = resolve_arena_artifact(root, args.arena)
            elif args.map_yaml is None:
                args.map_yaml = args.artifact.parent / "map.yaml"
            if arena_dir is not None:
                validate_arena_spawn(arena_dir, spawn_xy)
            expected_objects = _expected_scenario_objects(root, args.scenario, args.arena)
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
                arena_artifact=arena_dir,
                spawn_xy=spawn_xy,
            )
            from tinker_sim_isaac.camera_rig import CameraRig, load_camera_specs

            camera_specs = load_camera_specs(
                root / "simulation/sensors/hardware-parity.json"
            )
            # Opt-in, sim-only correction for the head camera's aim. The robot
            # description points it 14-48 deg above the horizon everywhere in
            # the reachable pan/tilt range, so the head camera sees only wall
            # and sky; this lets GPSR be exercised until that is fixed in the
            # description. Deliberately a workaround, and deliberately off
            # unless asked for -- see tinker_sim_isaac.head_camera_aim.
            from tinker_sim_isaac.head_camera_aim import (
                HEAD_AIM_ENV,
                apply_head_aim_correction,
                resolve_head_aim_correction,
            )

            head_aim = resolve_head_aim_correction(os.environ.get(HEAD_AIM_ENV))
            if head_aim is not None:
                camera_specs = apply_head_aim_correction(camera_specs, head_aim)
                print(
                    f"[sim] {HEAD_AIM_ENV} active: head camera aim corrected in "
                    "simulation only -- hardware parity is deliberately broken",
                    flush=True,
                )
            filtered_camera_specs = _without_wrist_camera(camera_specs, os.environ)
            if len(filtered_camera_specs) != len(camera_specs):
                print("[sim] wrist camera disabled", flush=True)
            camera_specs = filtered_camera_specs
            camera_specs, robot_min_camera_hz = _with_arena_camera(
                camera_specs, backend.occupancy, os.environ
            )
            arena_camera_enabled = _arena_camera_enabled(camera_specs)
            if arena_camera_enabled:
                spec = camera_specs[-1]
                print(
                    f"[sim] arena camera enabled at {spec.tick_rate_hz:g} Hz, "
                    f"{spec.width}x{spec.height} -> /sim/arena_camera/image_raw",
                    flush=True,
                )
            camera_rig = CameraRig(camera_specs)
            # stable_aa=True only once the arena camera is in play: with it,
            # 3+ concurrent RTX camera render products are alive, which has
            # raced DLSS's default live-resize path into a CUDA illegal
            # memory access ~15s later (see CameraRig.initialize's
            # docstring). Runs with just the two hardware-parity cameras
            # keep the previously-verified-stable default AA behaviour.
            # TINKER_SIM_STABLE_AA can force this decision either way for
            # the RTF spike -- see _stable_aa_requested. Scope (which render
            # products actually get the pin) follows the arena camera's own
            # presence via _stable_aa_cameras, independent of why stable_aa
            # ended up True: unset or "1" with the arena camera on scopes
            # the pin to just the arena product; "1" with the arena camera
            # off has no arena product to scope to, so it falls back to the
            # historical global pin (what the spike's variant D measures).
            # The arena-scoped pin is enough on its own: the actual CUDA-700
            # race was root-caused to stale sub-rate annotator reads and
            # fixed independently in a9fa951, so head/wrist can keep the
            # default DLSS op -- resize included, see CameraRig.initialize's
            # docstring -- without reopening the race; Task 3 / Phase 0
            # measured the global pin taxing those parity renders by ~50 ms
            # per Kit pump for no benefit, since neither was ever the
            # sub-rate camera the race depended on. The pin stays on the
            # arena product as defence in depth for the one product added
            # after the original crash.
            stable_aa = _stable_aa_requested(arena_camera_enabled, os.environ)
            if stable_aa != arena_camera_enabled:
                print(f"[sim] stable_aa forced to {stable_aa} by {STABLE_AA_ENV}", flush=True)
            camera_rig.initialize(
                app,
                stable_aa=stable_aa,
                stable_aa_cameras=_stable_aa_cameras(arena_camera_enabled),
            )
            from tinker_sim_isaac.ros_gateway import RosStandardGateway

            gateway = RosStandardGateway(
                backend,
                development_lidar=gateway_lidar_enabled(
                    args.sensor_profile, args.qualification
                ),
                camera_rig=camera_rig,
                camera_pointcloud=args.camera_pointcloud,
            )
            camera_hz = _resolve_camera_hz(
                robot_min_camera_hz,
                os.environ.get("TINKER_SIM_CAMERA_HZ"),
            )
            camera_stride = _streaming_update_stride(backend.dt, update_hz=camera_hz)
            import carb.settings

            kit_settings = carb.settings.get_settings()
            print(
                json.dumps(
                    {
                        "arena": args.arena,
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
            # Opt-in wall-time attribution for the sensor-rich loop; see
            # _emit_step_profile. Off by default so production runs are
            # unaffected.
            _profile = os.environ.get("TINKER_SIM_PROFILE", "") == "1"
            _profile_every = max(1, int(os.environ.get("TINKER_SIM_PROFILE_EVERY", "10")))
            _prof = {
                "physics": 0.0,
                "publish": 0.0,
                "kit_pump": 0.0,
                "cameras": 0.0,
                "spin": 0.0,
                "wall": 0.0,
                "_wall_mark": time.monotonic(),
                "cycles": 0,
                "physics_n": 0,
            }
            if _profile:
                print(
                    f"step profiling on: reporting every {_profile_every} camera "
                    f"cycles (stride={camera_stride}, camera_hz={camera_hz})",
                    flush=True,
                )
            while (
                running
                and app.is_running()
                and (args.duration <= 0.0 or backend.simulation_time < args.duration)
            ):
                _tl = time.monotonic() if _profile else 0.0
                gateway.spin_once()
                if (
                    time.monotonic() - next_collision_heartbeat
                    >= collision_heartbeat_period_s
                ):
                    gateway.publish_safety_heartbeat()
                    next_collision_heartbeat = time.monotonic()
                if _profile:
                    _prof["spin"] += time.monotonic() - _tl
                if omni.timeline.get_timeline_interface().is_playing():
                    _t0 = time.monotonic() if _profile else 0.0
                    backend.step()
                    if _profile:
                        _prof["physics"] += time.monotonic() - _t0
                        _prof["physics_n"] += 1
                    if not running:
                        break
                    try:
                        _t0 = time.monotonic() if _profile else 0.0
                        gateway.publish()
                        if _profile:
                            _prof["publish"] += time.monotonic() - _t0
                        camera_frame_index += 1
                        if camera_frame_index % camera_stride == 0:
                            # Rendering both RTX camera products on every
                            # 120 Hz physics frame collapses the step rate to
                            # ~2 Hz on this class of GPU.  Pump Kit (render,
                            # sim_control callbacks, UI events) only at the
                            # camera cadence, guarded so Kit cannot issue a
                            # second physics step.
                            _t0 = time.monotonic() if _profile else 0.0
                            _pump_streaming_app_update(app, kit_settings)
                            _t1 = time.monotonic() if _profile else 0.0
                            gateway.publish_cameras()
                            if _profile:
                                _t2 = time.monotonic()
                                _prof["kit_pump"] += _t1 - _t0
                                _prof["cameras"] += _t2 - _t1
                                _prof["wall"] += _t2 - _prof["_wall_mark"]
                                _prof["_wall_mark"] = _t2
                                _prof["cycles"] += 1
                                if _prof["cycles"] % _profile_every == 0:
                                    _emit_step_profile(
                                        _prof,
                                        backend_breakdown=(
                                            backend.step_profile_snapshot()
                                            if hasattr(backend, "step_profile_snapshot")
                                            else None
                                        ),
                                        publish_breakdown=(
                                            gateway.publish_profile_snapshot()
                                            if hasattr(gateway, "publish_profile_snapshot")
                                            else None
                                        ),
                                        camera_breakdown=(
                                            gateway.camera_profile_snapshot()
                                            if hasattr(gateway, "camera_profile_snapshot")
                                            else None
                                        ),
                                        spin_breakdown=(
                                            gateway.spin_profile_snapshot()
                                            if hasattr(gateway, "spin_profile_snapshot")
                                            else None
                                        ),
                                        sim_time=getattr(backend, "simulation_time", None),
                                    )
                    except BaseException:
                        if running:
                            raise
                        break
                    if not running:
                        break
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
            expected_objects = _expected_scenario_objects(root, args.scenario, args.arena)
            sys.path.insert(0, str(root / "simulation"))
            arena_dir = None
            if args.arena:
                arena_dir = resolve_arena_artifact(root, args.arena)
            if arena_dir is not None:
                validate_arena_spawn(arena_dir, spawn_xy)
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
                arena_artifact=arena_dir,
                spawn_xy=spawn_xy,
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
        if args.livestream:
            _clear_stream_ready_file(stream_ready_file)
        for subscription in streaming_event_subscriptions:
            subscription.reset()
        if visual_capture is not None:
            visual_capture.close()
        if gateway is not None:
            gateway.close()
        app.close(wait_for_replicator=False, exit_code=1 if failed else 0)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
