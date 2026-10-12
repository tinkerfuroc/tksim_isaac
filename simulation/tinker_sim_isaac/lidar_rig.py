"""Live PhysX raycast lidar for the simulator's ``/livox/lidar`` topic.

Replaces ``ros_gateway._development_point_cloud``, which raycast the arena
occupancy PGM: 181 rays, forward half only, every point at ``z = 0``. That
read the *map*, so it could not see a spawned object or the person capsule --
it only re-published what Nav2 already holds as ``static_layer``.

This rig casts against the live PhysX scene through
``isaacsim.sensors.experimental.physics``, so anything carrying a collider is
hit with no registration, including bodies spawned after ``play()``. The
extension declares no RTX dependency, so it runs in every profile including
``navigation-parity`` (``render=false``, CPU physics) -- the profile that
actually needs a lidar.

Three properties of the Isaac Sim 6.0.1 sensor drive this module's shape, all
established by measurement (see ``isaac-raycast-sensor-api-gotchas``):

* ``depths`` IS BROKEN. It replicates ray 0's value across every ray. Six
  axis-aligned rays into known geometry returned ``[3.5] * 6`` while the same
  reading's ``hit_positions`` were all correct. We read ``hit_positions``
  only, and derive range as its norm.
* ``ray_time_offsets`` is a firing SCHEDULE, not merely the pose extrapolation
  the schema documents. Spreading a frame's offsets across ``1 / tick_rate``
  makes the plugin fire each ray at its own instant instead of casting the
  whole pattern every physics step: 4.47 vs 27.89 ms/step at Mid-360 scale in
  an empty room. Without the sweep, full scale is unaffordable.

  THE PER-RAY COSTS THAT USED TO BE QUOTED HERE WERE VOID and have been
  removed. They were taken before the scene-query fix, i.e. with the query
  manager absent and EVERY RAY MISSING, so they timed rays that never resolved
  a hit and never touched a collision shape. They understated the real cost by
  roughly 7x: the figure given for the full pattern was ~0.75 s of compute per
  simulated second, and the true marginal cost measured after the fix is ~2.6 s
  per simulated second at 120 Hz. Treat any raycast sizing number dated before
  2026-09-09 the same way. See :func:`resolve_lidar_scale` for the measured
  RTF table that replaces them.
* Because rays fire on their own step, the reading buffer holds ONLY that
  step's rays and is CLEARED each step. A frame must therefore be ACCUMULATED
  across the sweep window (12 physics steps at 120 Hz / 10 Hz). The union over
  one window recovers the full pattern exactly.

A miss is a ZERO VECTOR in ``hit_positions`` -- not the ray endpoint, not
``max_range``. That is also how an unfired ray reads, which is harmless: both
contribute no point.

Fidelity ceiling: a PhysX raycast hits COLLISION geometry, so arena furniture
scans as ``arena_convert``'s deliberately over-approximated boxes rather than
the visual mesh. Conservative in the safe direction for navigation. Lifting it
means the RTX ``OmniLidar``, which needs a render pass and is therefore
sensor-rich only.
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np

#: Livox Mid-360 vertical field of view (degrees). Asymmetric by design: the
#: unit is meant to sit low and look up and out.
MID360_ELEVATION_MIN_DEG = -7.0
MID360_ELEVATION_MAX_DEG = 52.0
#: Mid-360 datasheet range at 10% reflectivity. Kept honest rather than
#: trimmed to what pointcloud_to_laserscan consumes (8 m): measurement showed
#: max_range is not a cost lever (8 m vs 40 m differed by under 4%).
MID360_MAX_RANGE_M = 40.0
#: Matches the hardware pointcloud_to_laserscan ``range_min``. NOTE this
#: offsets each ray's START (origin + direction * min_range); it is not a
#: rejection filter. It doubles as the first line of defence against the
#: sensor hitting the robot's own chassis.
MID360_MIN_RANGE_M = 0.2

#: 57 x 349 = 19,893 rays; at 10 Hz that is 198,930 points/s against the
#: Mid-360's 200,000, with ~1.03 deg spacing in both axes.
MID360_CHANNELS = 57
MID360_COLUMNS = 349

_ZERO_HIT_EPSILON_M = 1e-6

#: Post-physics-step callback order for the frame accumulator. The sensor
#: extension fills its reading buffer at the default order 0; we must read
#: strictly after that.
_ACCUMULATE_CALLBACK_ORDER = 100


class LidarSpecError(ValueError):
    """Malformed lidar contract. Raised, never guessed around."""


@dataclass(frozen=True)
class LidarSpec:
    """The declared lidar contract, loaded from the hardware-parity file."""

    pointcloud_topic: str
    scan_topic: str
    frame_id: str
    tick_rate_hz: float
    #: Name of the URDF link the sensor rides on, searched for under the robot
    #: prim exactly as ``CameraRig`` resolves ``mount_prim``.
    mount_prim: str
    channels: int
    columns: int
    elevation_min_deg: float
    elevation_max_deg: float
    min_range_m: float
    max_range_m: float
    #: Spread ray firing across one frame. Effectively mandatory at Mid-360
    #: scale -- see the module docstring. Off casts the whole pattern every
    #: physics step, which is ~9x the cost and models an instantaneous scan.
    sweep: bool = True
    #: Drop returns that land on the robot's own body.
    #:
    #: Measured against the real robot USD: 9,840 of 19,893 points -- 49.5% of
    #: the frame -- hit the robot itself. That is far more than a real Mid-360
    #: loses to its own chassis, because the sensor casts against COLLISION
    #: geometry and `arena_convert`/the URDF import wrap the arm and body in
    #: deliberately over-approximated convex hulls. Filtering therefore moves
    #: the sim toward hardware behaviour rather than away from it, and it is
    #: effectively free: enabling `report_hit_prim_paths` measured 31.95 vs
    #: 31.91 ms/step, inside noise.
    self_filter: bool = True

    @property
    def num_rays(self) -> int:
        return self.channels * self.columns

    @property
    def frame_period_s(self) -> float:
        return 1.0 / self.tick_rate_hz

    @property
    def points_per_second(self) -> float:
        return self.num_rays * self.tick_rate_hz


def _mapping(raw: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = raw.get(key)
    if not isinstance(value, Mapping):
        raise LidarSpecError(f"lidar contract is missing the {key!r} object")
    return value


def _string(raw: Mapping[str, Any], key: str, where: str) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value:
        raise LidarSpecError(f"{where}.{key} must be a non-empty string")
    return value


def _positive(raw: Mapping[str, Any], key: str, where: str) -> float:
    value = raw.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
        raise LidarSpecError(f"{where}.{key} must be a positive number")
    return float(value)


def _positive_int(raw: Mapping[str, Any], key: str, where: str) -> int:
    value = raw.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise LidarSpecError(f"{where}.{key} must be a positive integer")
    return value


#: Below this the vertical fan is too sparse to put more than a stripe or two
#: on a person-sized obstacle at room range.
MINIMUM_LIDAR_CHANNELS = 8
#: Below this the azimuth spacing gets wide enough that a person can fall
#: BETWEEN two beams -- see test_the_floors_still_see_a_dynamic_obstacle.
MINIMUM_LIDAR_COLUMNS = 90


#: What an unset ``TINKER_SIM_LIDAR_CHANNELS`` resolves to now that the live
#: sensor is the DEFAULT source rather than an opt-in.
#:
#: This is not a guess and not an extrapolation: 44 x 349 is the exact
#: configuration that passed the live Nav2 battery on 2026-09-10 -- RTF 0.521
#: with the whole tinker_sim_bridge stack attached, AMCL converged, and a
#: completed ``navigate_to_pose``. The contract's own 57 channels was never
#: measured with the stack attached; bare-sim it ran 0.514, which leaves no
#: room once Nav2 is also on the box. Defaulting to the validated number keeps
#: an out-of-the-box navigation run above the 0.5 RTF floor the project holds
#: itself to, instead of just below it.
#:
#: Elevation is the right axis to spend, because it is the one navigation does
#: not read. ``pointcloud_to_laserscan`` flattens the cloud to 2-D, and its
#: 0..2.0 m height band at the 8 m ``range_max`` only ever uses elevations of
#: roughly 0..14 deg -- a small slice of the contract's -7..+52 deg fan. Cutting
#: COLUMNS instead would be the wrong trade: at 349 the azimuth step is already
#: 1.03 deg, coarser than the scan's own 0.008 rad (0.458 deg) increment, so
#: every column removed is directly visible in ``/scan``.
#:
#: Raise it to the contract with ``TINKER_SIM_LIDAR_CHANNELS=57`` for a parity
#: run that wants the full fan and can pay for it.
DEFAULT_LIDAR_CHANNELS = 44
#: Azimuth is what navigation actually consumes, so the default is the
#: contract and the override exists only to go lower deliberately.
DEFAULT_LIDAR_COLUMNS = 349


def _resolve_axis(
    contract: int, override: str | None, name: str, floor: int, default: int
) -> int:
    """One axis of :func:`resolve_lidar_scale`.

    ``default`` is what an unset override resolves to. It is NOT necessarily
    the contract: the sensor is on by default now, and the default has to be
    affordable at the validated 120 Hz physics rate. The contract remains the
    ceiling an explicit override may not exceed.
    """
    if override is None or not str(override).strip():
        return min(int(default), int(contract))
    text = str(override).strip()
    try:
        value = int(text)
    except (TypeError, ValueError):
        raise ValueError(
            f"{name} must be a whole number, got {override!r}"
        ) from None
    if value <= 0:
        raise ValueError(f"{name} must be positive, got {override!r}")
    if value > int(contract):
        raise ValueError(
            f"{name}={value} exceeds the hardware contract's {contract}; the "
            "contract is what the Mid-360 actually does, and a run claiming "
            "more would invalidate parity results measured against it"
        )
    if value < floor:
        raise ValueError(
            f"{name}={value} is below the {floor} floor; past it the scan can "
            "no longer be relied on to see a person-sized dynamic obstacle"
        )
    return value


def resolve_lidar_scale(
    channels: int,
    columns: int,
    channels_override: str | None,
    columns_override: str | None,
) -> tuple[int, int]:
    """Resolve the ray grid, honouring explicit opt-in overrides.

    Every ray is a PhysX scene query paid ``physics_hz`` times per simulated
    second, so ray count is the dominant term in the sensor's wall-clock cost
    -- and the only effective one. ``max_range`` and ``min_range`` were both
    measured worth under 4%, because the expense is TESTING each ray against
    the robot's ~200 convex collision hulls rather than hitting anything.

    Because the cost is paid per PHYSICS step, the physics rate multiplies it.
    Measured live in navigation-parity, arena rcw2026 (RTF, higher is better):

        rate        occupancy   44x349 (default)   full 57x349
        120 / 120     0.248 / 0.246      0.164            0.151
         60 /  30           0.927          --             0.483
         40 /  40             --           0.578          0.514

    (The two occupancy figures at 120 Hz are independent measurements taken
    an hour apart -- 0.248 as a bare baseline, 0.246 via ``--map-lidar`` --
    which is the reproducibility these numbers carry.)

    Read the top row before sizing anything: at the validated 120 Hz default
    the simulator is already at RTF 0.248 with the CHEAP occupancy lidar and no
    raycast sensor at all. No ray budget recovers that -- the rate does. Runs
    that need RTF >= 0.5 lower the rate (TINKER_SIM_PHYSICS_HZ /
    TINKER_SIM_CONTROL_HZ), and at 40/40 even the full contract fan clears it.

    Unset, :data:`DEFAULT_LIDAR_CHANNELS` x :data:`DEFAULT_LIDAR_COLUMNS` is
    returned -- the live-battery-validated 44 x 349, not the contract, now that
    the sensor is on by default. Lowering further is a deliberate
    fidelity-for-speed trade; raising above the CONTRACT is refused, because
    that file is the hardware's own specification. Raising from the default up
    to the contract is allowed and is how a parity run asks for the full fan.
    """
    return (
        _resolve_axis(
            channels, channels_override, "TINKER_SIM_LIDAR_CHANNELS",
            MINIMUM_LIDAR_CHANNELS, DEFAULT_LIDAR_CHANNELS,
        ),
        _resolve_axis(
            columns, columns_override, "TINKER_SIM_LIDAR_COLUMNS",
            MINIMUM_LIDAR_COLUMNS, DEFAULT_LIDAR_COLUMNS,
        ),
    )


def load_lidar_spec(path: Path | str) -> LidarSpec:
    """Load the lidar contract; malformed declarations raise, never guess.

    Mirrors ``camera_rig.load_camera_specs``: the sensor contract file is the
    single source of truth, and a missing or wrong field is a hard error so a
    silently degraded sensor can never reach a parity run.
    """
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if raw.get("schema_version") != 2:
        raise LidarSpecError("hardware-parity sensor contract must be schema_version 2")

    lidar = _mapping(raw, "lidar")
    if lidar.get("qos") != "sensor_data":
        raise LidarSpecError("lidar.qos must be sensor_data")

    geometry = _mapping(lidar, "raycast")
    elevation = geometry.get("elevation_range_deg")
    if (
        not isinstance(elevation, list)
        or len(elevation) != 2
        or any(not isinstance(v, (int, float)) or isinstance(v, bool) for v in elevation)
    ):
        raise LidarSpecError("lidar.raycast.elevation_range_deg must be [min, max]")
    elevation_min, elevation_max = float(elevation[0]), float(elevation[1])
    if elevation_min >= elevation_max:
        raise LidarSpecError("lidar.raycast.elevation_range_deg must be increasing")

    min_range = _positive(geometry, "min_range_m", "lidar.raycast")
    max_range = _positive(geometry, "max_range_m", "lidar.raycast")
    if min_range >= max_range:
        # The plugin disables a sensor whose minRange >= maxRange, silently.
        raise LidarSpecError("lidar.raycast.min_range_m must be below max_range_m")

    sweep = geometry.get("sweep", True)
    if not isinstance(sweep, bool):
        raise LidarSpecError("lidar.raycast.sweep must be a boolean")

    self_filter = geometry.get("self_filter", True)
    if not isinstance(self_filter, bool):
        raise LidarSpecError("lidar.raycast.self_filter must be a boolean")

    # Opt-in fidelity-for-RTF trade; unset, the contract is used as written.
    channels, columns = resolve_lidar_scale(
        _positive_int(geometry, "channels", "lidar.raycast"),
        _positive_int(geometry, "columns", "lidar.raycast"),
        os.environ.get("TINKER_SIM_LIDAR_CHANNELS"),
        os.environ.get("TINKER_SIM_LIDAR_COLUMNS"),
    )

    return LidarSpec(
        pointcloud_topic=_string(lidar, "pointcloud_topic", "lidar"),
        scan_topic=_string(lidar, "scan_topic", "lidar"),
        frame_id=_string(lidar, "frame_id", "lidar"),
        tick_rate_hz=_positive(lidar, "tick_rate_hz", "lidar"),
        mount_prim=_string(geometry, "mount_prim", "lidar.raycast"),
        channels=channels,
        columns=columns,
        elevation_min_deg=elevation_min,
        elevation_max_deg=elevation_max,
        min_range_m=min_range,
        max_range_m=max_range,
        sweep=sweep,
        self_filter=self_filter,
    )


def self_hit_mask(hit_prim_paths: Any, live_indices: np.ndarray, prefix: str) -> np.ndarray:
    """Which of ``live_indices`` landed on the robot (paths under *prefix*).

    Only the rays that actually returned a point this step are examined. The
    full table is ~19,893 entries and comparing every one of them in Python on
    every physics step would cost more than the raycast itself; the live subset
    is ~3,300, which is an order of magnitude cheaper for the same answer.
    """
    if len(live_indices) == 0:
        return np.zeros(0, dtype=bool)
    paths = hit_prim_paths
    return np.fromiter(
        (str(paths[index]).startswith(prefix) for index in live_indices),
        dtype=bool,
        count=len(live_indices),
    )


def lidar_pattern(
    spec: LidarSpec,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """``(origins, directions, time_offsets)`` for *spec*, in the sensor frame.

    Row order is channel-major: ray ``c * columns + a`` is elevation ``c`` at
    azimuth ``a``. Time offsets sweep by AZIMUTH so that one frame is a
    rotation through 360 degrees, which is both what the plugin schedules on
    and what gives a moving robot the correct within-frame skew.

    The pattern is a fixed isotropic grid, not the Mid-360's non-repetitive
    rosette: ray attributes are read once at simulation start, so directions
    cannot advance per frame. A stationary robot therefore rescans identical
    directions instead of progressively filling coverage. That is a real
    fidelity gap, and it is invisible to Nav2, which collapses the cloud to a
    2-D scan anyway.
    """
    elevations = np.radians(
        np.linspace(spec.elevation_min_deg, spec.elevation_max_deg, spec.channels)
    )
    azimuths = np.radians(np.linspace(0.0, 360.0, spec.columns, endpoint=False))
    elevation_grid, azimuth_grid = np.meshgrid(elevations, azimuths, indexing="ij")

    directions = np.stack(
        [
            np.cos(elevation_grid) * np.cos(azimuth_grid),
            np.cos(elevation_grid) * np.sin(azimuth_grid),
            np.sin(elevation_grid),
        ],
        axis=-1,
    ).reshape(-1, 3).astype(np.float32)

    # Every ray leaves the sensor origin; min_range offsets the start for us.
    origins = np.zeros_like(directions)

    column_index = np.tile(np.arange(spec.columns), spec.channels).astype(np.float64)
    if spec.sweep:
        offsets = (column_index / spec.columns) * spec.frame_period_s
    else:
        offsets = np.zeros_like(column_index)
    return origins, directions, offsets.astype(np.float32)


class FrameAccumulator:
    """Assembles one lidar frame from the per-step slices the plugin returns.

    With sweep offsets the sensor's reading buffer holds only the rays fired
    in the current physics step and is cleared each step, so a frame is the
    union of ``window_steps`` consecutive readings. Pure numpy and free of any
    Isaac import, so the windowing contract is unit-testable.
    """

    def __init__(self, num_rays: int, window_steps: int, time_offsets: np.ndarray) -> None:
        if num_rays <= 0:
            raise ValueError("num_rays must be positive")
        if window_steps <= 0:
            raise ValueError("window_steps must be positive")
        if len(time_offsets) != num_rays:
            raise ValueError("time_offsets must have one entry per ray")
        self.num_rays = num_rays
        self.window_steps = window_steps
        self._time_offsets = np.asarray(time_offsets, dtype=np.float64)
        self._points = np.zeros((num_rays, 3), dtype=np.float32)
        self._filled = np.zeros(num_rays, dtype=bool)
        self._steps = 0
        self._frames = 0

    @property
    def steps_in_window(self) -> int:
        return self._steps

    @property
    def frames_completed(self) -> int:
        return self._frames

    def add_reading(self, hit_positions: Any) -> bool:
        """Fold one physics step's reading in. True when the frame is complete.

        A zero vector means "no point": either the ray missed, or it did not
        fire on this step. Both contribute nothing, so one test covers both.
        """
        hits = np.asarray(hit_positions, dtype=np.float32).reshape(-1, 3)
        if hits.shape[0] == self.num_rays:
            live = np.linalg.norm(hits, axis=1) > _ZERO_HIT_EPSILON_M
            if live.any():
                self._points[live] = hits[live]
                self._filled |= live
        elif hits.shape[0] != 0:
            # A short buffer means the sensor is mid-initialisation or the
            # pattern was re-authored underneath us; drop it rather than
            # misalign ray indices against time offsets.
            raise ValueError(
                f"reading has {hits.shape[0]} rays, expected {self.num_rays}"
            )
        self._steps += 1
        return self._steps >= self.window_steps

    def take_frame(self) -> tuple[np.ndarray, np.ndarray]:
        """``(points, firing_times)`` for the completed frame, then reset.

        ``points`` is ``(M, 3)`` float32 in the sensor frame; ``firing_times``
        is ``(M,)`` seconds relative to the frame start, carrying each point's
        own capture instant. Real Livox hardware ships a per-point timestamp
        for exactly this reason: within one frame the sensor has moved, so the
        cloud is skewed and consumers de-skew with the per-point time.
        """
        points = self._points[self._filled].copy()
        times = self._time_offsets[self._filled].copy()
        self._points[self._filled] = 0.0
        self._filled[:] = False
        self._steps = 0
        self._frames += 1
        return points, times


def window_steps_for(spec: LidarSpec, physics_dt: float) -> int:
    """Physics steps per lidar frame; at least one.

    12 at the validated 120 Hz physics rate and the declared 10 Hz lidar rate.
    """
    if physics_dt <= 0.0:
        raise ValueError("physics_dt must be positive")
    return max(1, int(round(spec.frame_period_s / physics_dt)))


class RaycastLidar:
    """Owns the sensor prim and turns per-step readings into whole frames.

    Constructed after the backend, like ``CameraRig``: the prim can only be
    created once ``backend.__init__`` has run ``sim.reset()``, which is why
    both rigs are built by ``run_sim`` rather than inside the backend.
    """

    def __init__(self, spec: LidarSpec, *, robot_prim_path: str = "/World/Tinker") -> None:
        self.spec = spec
        self.robot_prim_path = robot_prim_path
        self.sensor: Any = None
        self.sensor_path: str | None = None
        #: The authoring object from `author_prim`, consumed by `bind`.
        self._authoring: Any = None
        self._offsets: np.ndarray | None = None
        #: Static offset from the rigid body the sensor is parented to.
        self.mount_translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._accumulator: FrameAccumulator | None = None
        self._callback_uid: int | None = None
        self._pending: tuple[np.ndarray, np.ndarray] | None = None
        self._frame_ready = False
        self._reported_failures: set[str] = set()
        #: Physics step of the last reading folded in. The accumulator is
        #: driven from two places (see ``accumulate``), so folding is keyed on
        #: the sensor's own step counter rather than on call count: being
        #: called twice for one physics step must not consume two window slots.
        self._last_physics_step: int | None = None
        self._callback_folds = 0
        self._poll_folds = 0
        self._last_reading_len = -1
        self._last_points = -1
        self._last_raw_hits = -1
        self._last_self_hits = -1
        self._last_path_table_len = -1
        #: First hit prim path seen, kept once so the status heartbeat can show
        #: WHAT the sensor is actually hitting without streaming every path.
        self._last_sample_path = ""

    # -- lifecycle ------------------------------------------------------

    def initialize(self, app: Any) -> None:
        """Create the sensor prim and attach the runtime reader.

        Authoring order was investigated and ruled out: creating the prim
        BEFORE ``sim.reset()`` (via a backend pre-reset hook) produced results
        identical to creating it after -- same ray geometry, same zero hits --
        so the simpler post-reset construction is kept.
        """
        import omni.usd
        from isaacsim.sensors.experimental.physics import Raycast

        stage = omni.usd.get_context().get_stage()
        parent_path, translation = self._resolve_mount(stage)
        self.mount_translation = translation

        origins, directions, offsets = lidar_pattern(self.spec)
        self.sensor_path = f"{parent_path}/livox_raycast"

        create_kwargs: dict[str, Any] = {
            "min_range": self.spec.min_range_m,
            "max_range": self.spec.max_range_m,
            "ray_origins": origins,
            "ray_directions": directions,
            # Hits come back already in the sensor's own frame, so no
            # quaternion maths is needed to publish in `livox360`.
            "output_frame": "SENSOR",
            # Needed only to identify the robot's own body. The schema warns
            # of a per-ray cost for resolving prim paths from physics shapes,
            # but measured at this scale it is inside noise (31.95 vs 31.91
            # ms/step), and it removes ~half the frame.
            "report_hit_prim_paths": bool(self.spec.self_filter),
            "translations": np.array([translation], dtype=np.float32),
        }
        if self.spec.sweep:
            create_kwargs["ray_time_offsets"] = offsets

        from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager
        from isaacsim.sensors.experimental.physics import RaycastSensor

        self.sensor = RaycastSensor(Raycast.create(self.sensor_path, **create_kwargs))
        app.update()

        physics_dt = float(SimulationManager.get_physics_dt())
        self._accumulator = FrameAccumulator(
            self.spec.num_rays, window_steps_for(self.spec, physics_dt), offsets
        )

        # Accumulate on the PHYSICS step, not the control step: with
        # TINKER_SIM_CONTROL_HZ set below physics_hz one control step covers
        # several physics substeps, and reading only once per control step
        # would silently drop most of the sweep.
        #
        # `order` matters. The sensor extension fills its reading from its own
        # PHYSICS_POST_STEP callback registered at the default order 0, so we
        # must run after it or we would read the previous step's slice; a
        # positive order guarantees that rather than relying on registration
        # sequence.
        #
        # Measured caveat: in the production loop this callback is never
        # dispatched at all (`callback_folds` stayed 0 across a full run while
        # `poll_folds` reached 1261), because physics is stepped through
        # IsaacLab's SimulationContext rather than SimulationManager. The
        # gateway's publish-path poll is what actually drives the accumulator;
        # this subscription is kept for loops that DO step through
        # SimulationManager, and double-driving is safe because folding is
        # deduplicated by physics step.
        self._callback_uid = SimulationManager.register_callback(
            self._on_physics_step,
            event=SimulationEvent.PHYSICS_POST_STEP,
            order=_ACCUMULATE_CALLBACK_ORDER,
        )

    def shutdown(self) -> None:
        """Drop the physics subscription and release the sensor."""
        if self._callback_uid is not None:
            try:
                from isaacsim.core.simulation_manager import SimulationManager

                SimulationManager.deregister_callback(self._callback_uid)
            except Exception:  # noqa: BLE001 - teardown must never raise
                pass
            self._callback_uid = None
        if self.sensor is not None:
            try:
                self.sensor.reset()
            except Exception:  # noqa: BLE001 - teardown must never raise
                pass
            self.sensor = None

    # -- per-step -------------------------------------------------------

    def _on_physics_step(self, step_dt: float, context: Any = None) -> None:
        """Physics post-step handler.

        The signature must match what SimulationManager's dispatcher calls
        with -- ``(step_dt, context)``, exactly as the sensor extension's own
        ``_SensorStepManager._on_physics_step`` declares it. A narrower
        signature raises TypeError inside the message bus, which swallows it:
        the callback simply never appears to run, and every frame comes back
        empty with nothing logged.

        This is a best-effort driver, NOT the only one. It fires when physics
        is stepped through ``SimulationManager``, but the production loop steps
        through IsaacLab's ``SimulationContext``, where the event is not
        guaranteed to be dispatched -- which is exactly how the first live nav
        battery produced a sensor that cast rays (GPU busy, no errors logged)
        while ``/livox/lidar`` stayed silent for the entire run. The gateway
        therefore also polls ``accumulate()`` on its publish path, and folding
        is deduplicated by physics step so both drivers together stay correct.
        """
        self.accumulate(source="callback")

    def accumulate(self, *, source: str = "poll") -> None:
        """Fold the current reading into the frame under construction.

        A read failure must not kill the simulation loop, but it must not be
        invisible either -- a silently swallowed exception here produces a
        lidar that publishes nothing at all while looking healthy. So the
        first failure of each kind is reported once, then suppressed.
        """
        if self.sensor is None or self._accumulator is None:
            return
        try:
            data = self.sensor.get_data()
            step = data.get("physics_step")
            if step is not None and step == self._last_physics_step:
                # This physics step was already folded by the other driver.
                return
            self._last_physics_step = step
            hits = data["hit_positions"]
            if self.spec.self_filter:
                hits = self._drop_self_hits(hits, data.get("hit_prim_paths"))
        except Exception as exc:  # noqa: BLE001
            self._report_once("read", exc)
            return
        if source == "callback":
            self._callback_folds += 1
        else:
            self._poll_folds += 1
        try:
            self._last_reading_len = int(np.asarray(hits).reshape(-1, 3).shape[0])
        except Exception:  # noqa: BLE001 - diagnostics must never raise
            self._last_reading_len = -1
        try:
            complete = self._accumulator.add_reading(hits)
        except ValueError as exc:
            self._report_once("accumulate", exc)
            return
        if complete:
            self._pending = self._accumulator.take_frame()
            self._last_points = int(self._pending[0].shape[0])
            self._frame_ready = True

    def _drop_self_hits(self, hit_positions: Any, hit_prim_paths: Any) -> np.ndarray:
        """Zero out returns that landed on the robot.

        Zeroing rather than removing keeps ray index aligned with the
        time-offset table, and a zero vector already means "no point" to the
        accumulator -- the same representation a miss uses.
        """
        hits = np.array(hit_positions, dtype=np.float32, copy=True).reshape(-1, 3)
        # Counted BEFORE filtering, so "every ray missed" is distinguishable
        # from "the self-filter ate the whole frame". Both end as zero points.
        self._last_raw_hits = int(
            (np.linalg.norm(hits, axis=1) > _ZERO_HIT_EPSILON_M).sum()
        )
        if hit_prim_paths is None or len(hit_prim_paths) != hits.shape[0]:
            self._last_path_table_len = -1 if hit_prim_paths is None else len(hit_prim_paths)
            # No usable path table (sensor still initialising, or the option
            # was off): publish the frame unfiltered rather than dropping it.
            return hits
        self._last_path_table_len = len(hit_prim_paths)
        live_indices = np.flatnonzero(np.linalg.norm(hits, axis=1) > _ZERO_HIT_EPSILON_M)
        mask = self_hit_mask(hit_prim_paths, live_indices, self.robot_prim_path)
        self._last_self_hits = int(mask.sum())
        if len(live_indices) and self._last_sample_path == "":
            self._last_sample_path = str(hit_prim_paths[live_indices[0]])
        if mask.any():
            hits[live_indices[mask]] = 0.0
        return hits

    def _report_once(self, kind: str, exc: BaseException) -> None:
        if kind in self._reported_failures:
            return
        self._reported_failures.add(kind)
        print(
            f"[lidar_rig] {kind} failure (reported once): {exc!r}",
            flush=True,
        )

    @property
    def frame_ready(self) -> bool:
        return self._frame_ready

    def diagnostics(self) -> dict[str, Any]:
        """Counters for the status heartbeat.

        ``callback_folds`` vs ``poll_folds`` says WHICH driver is actually
        feeding the accumulator in a given profile; ``frames`` staying at 0
        while folds climb means the window never closes. None of this was
        observable during the first live nav battery, which is why a silent
        ``/livox/lidar`` cost a whole run to localise.
        """
        accumulator = self._accumulator
        report = {
            "frames": accumulator.frames_completed if accumulator else 0,
            "steps_in_window": accumulator.steps_in_window if accumulator else 0,
            "window_steps": accumulator.window_steps if accumulator else 0,
            "callback_folds": self._callback_folds,
            "poll_folds": self._poll_folds,
            # Length of the last reading and the sensor's own validity flag.
            # A sensor that constructs cleanly but is never EVALUATED returns a
            # zero-length reading forever; the accumulator tolerates that (an
            # empty buffer is indistinguishable from an all-miss frame), so
            # windows still close on time and publish empty clouds. Without
            # these two fields that looks identical to a working sensor
            # scanning an empty room.
            "last_reading_len": self._last_reading_len,
            "last_points": self._last_points,
            "raw_hits": self._last_raw_hits,
            "self_hits": self._last_self_hits,
            "path_table_len": self._last_path_table_len,
            "sample_path": self._last_sample_path,
        }
        if self.sensor is not None:
            try:
                reading = self.sensor.get_sensor_reading()
                report["is_valid"] = bool(reading.is_valid)
                # WHERE the rays actually start in world coordinates. A sensor
                # whose rays all miss is either somewhere with no geometry or
                # buried inside it, and nothing else in the reading can tell
                # those apart.
                origins = np.asarray(reading.ray_origins_world).reshape(-1, 3)
                ends = np.asarray(reading.ray_end_points_world).reshape(-1, 3)
                if origins.shape[0]:
                    report["ray_origin_world"] = [
                        round(float(v), 4) for v in origins[0]
                    ]
                if ends.shape[0]:
                    report["ray_end_world"] = [round(float(v), 4) for v in ends[0]]
            except Exception as exc:  # noqa: BLE001 - diagnostics must never raise
                report["is_valid"] = None
                report["reading_error"] = repr(exc)[:120]
        return report

    def take_frame(self) -> tuple[np.ndarray, np.ndarray] | None:
        """The most recent complete frame, or ``None`` if one is not ready."""
        if not self._frame_ready or self._pending is None:
            return None
        frame = self._pending
        self._pending = None
        self._frame_ready = False
        return frame

    # -- mounting -------------------------------------------------------

    def _resolve_mount(self, stage: Any) -> tuple[str, tuple[float, float, float]]:
        """``(rigid_body_path, translation)`` for the sensor prim.

        The sensor's world transform is derived from the nearest rigid body's
        pose, so it must hang off a prim PhysX actually writes a pose for. The
        URDF importer welds fixed-joint links like ``livox_frame`` onto their
        parent, leaving them as plain child Xforms -- the same trap
        ``camera_rig`` documents for optical frames, and the reason the
        tk26_sim reference mounts its raycaster on ``base_link`` rather than
        the articulation root. So: find the declared mount link, walk up to
        the nearest ``RigidBodyAPI`` ancestor, and fold the residual offset
        between them into the sensor's own translation.
        """
        from pxr import UsdGeom, UsdPhysics

        robot = stage.GetPrimAtPath(self.robot_prim_path)
        if not robot or not robot.IsValid():
            raise RuntimeError(f"lidar mount: {self.robot_prim_path} is not on the stage")

        from pxr import Usd

        matches = [
            prim
            for prim in Usd.PrimRange(robot)
            if prim.GetName() == self.spec.mount_prim
        ]
        if len(matches) != 1:
            raise RuntimeError(
                f"lidar mount: expected exactly one {self.spec.mount_prim!r} under "
                f"{self.robot_prim_path}, found {len(matches)}"
            )
        mount = matches[0]

        body = mount
        while body is not None and body.IsValid():
            if body.HasAPI(UsdPhysics.RigidBodyAPI):
                break
            body = body.GetParent()
        if body is None or not body.IsValid():
            raise RuntimeError(
                f"lidar mount: no RigidBodyAPI ancestor above {self.spec.mount_prim!r}; "
                "the sensor would not track the robot"
            )

        cache = UsdGeom.XformCache()
        body_world = cache.GetLocalToWorldTransform(body)
        mount_world = cache.GetLocalToWorldTransform(mount)
        offset = (mount_world * body_world.GetInverse()).ExtractTranslation()
        return str(body.GetPath()), (
            float(offset[0]),
            float(offset[1]),
            float(offset[2]),
        )


def scan_ring_indices(spec: LidarSpec) -> np.ndarray:
    """Ray indices of the channel nearest horizontal.

    Only needed if the sim ever publishes ``/scan`` directly; today
    ``pointcloud_to_laserscan`` derives it from the cloud with the hardware's
    own parameters, which is the parity path.
    """
    elevations = np.linspace(
        spec.elevation_min_deg, spec.elevation_max_deg, spec.channels
    )
    channel = int(np.argmin(np.abs(elevations)))
    start = channel * spec.columns
    return np.arange(start, start + spec.columns)


def frame_summary(points: np.ndarray, times: np.ndarray) -> dict[str, float | int]:
    """Small diagnostic digest for status heartbeats and tests."""
    count = int(points.shape[0])
    if count == 0:
        return {"points": 0, "range_min": 0.0, "range_max": 0.0, "span_s": 0.0}
    ranges = np.linalg.norm(points, axis=1)
    return {
        "points": count,
        "range_min": float(ranges.min()),
        "range_max": float(ranges.max()),
        "span_s": float(times.max() - times.min()) if count > 1 else 0.0,
    }
