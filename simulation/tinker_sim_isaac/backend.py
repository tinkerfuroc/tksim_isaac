from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping

from tinker_sim_isaac.physics_rate import (
    physics_substeps,
    resolve_control_hz,
    resolve_physics_hz,
    resolve_solver_iterations,
)
from tinker_sim_isaac.target_write_gate import TargetWriteGate
from tinker_sim_core.command_mux import JointCommand, decode_snapshot_packet
from tinker_sim_core.occupancy import OccupancyMap


CHASSIS_BALLAST_SOURCE_MASS_KG = 20.0
# 10.0 -> 30.0 (sim_cumotion campaign): at 30 kg total the base tipped over
# (75 deg pitch, kN gripper<->ground contacts) under full-speed or full-reach
# arm motion — the real chassis is heavier, so 30 kg under-modeled the
# hardware's static stability. 50 kg total holds the arm's full 0.45 m
# forward reach upright (hardware-parity direction chosen by the operator).
CHASSIS_BALLAST_ADDED_MASS_KG = 30.0
CHASSIS_BALLAST_TARGET_MASS_KG = (
    CHASSIS_BALLAST_SOURCE_MASS_KG + CHASSIS_BALLAST_ADDED_MASS_KG
)
CHASSIS_BALLAST_SOURCE_DIAGONAL_INERTIA = (0.22, 0.30, 0.22)
CHASSIS_BALLAST_TARGET_DIAGONAL_INERTIA = tuple(
    value * CHASSIS_BALLAST_TARGET_MASS_KG / CHASSIS_BALLAST_SOURCE_MASS_KG
    for value in CHASSIS_BALLAST_SOURCE_DIAGONAL_INERTIA
)


def chassis_ballast_target_properties(
    current_mass_kg: float,
) -> tuple[float, tuple[float, float, float]]:
    """Return the idempotent low-mounted ballast override for Tinker 2.

    The source USD carries the original 20 kg batteries/electronics ballast.
    Accept either that source value or an artifact that already contains this
    30 kg addition, but reject an unknown mass instead of silently replacing a
    newer physical model.
    """
    mass = float(current_mass_kg)
    if not math.isfinite(mass) or not (
        math.isclose(mass, CHASSIS_BALLAST_SOURCE_MASS_KG, abs_tol=1.0e-4)
        or math.isclose(mass, CHASSIS_BALLAST_TARGET_MASS_KG, abs_tol=1.0e-4)
    ):
        raise ValueError(
            "Tinker chassis ballast must be the 20 kg source mass or the "
            "50 kg augmented mass"
        )
    return CHASSIS_BALLAST_TARGET_MASS_KG, CHASSIS_BALLAST_TARGET_DIAGONAL_INERTIA


# Wheel radius 0.0525 m => 60 rad/s^2 ~= 3.1 m/s^2 linear. Deliberately above
# Nav2's acc_lim (~2.5 m/s^2) so planner-shaped profiles pass through; this is
# the floor-level bound for non-planner commanders and stale-target transients.
WHEEL_VELOCITY_SLEW_RAD_S2 = 60.0
WHEEL_JOINT_NAMES = frozenset(
    {
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    }
)
#: Isaac Lab ``joint_names_expr`` for the velocity-driven wheel group: the two
#: front drive wheels only.
WHEEL_ACTUATOR_JOINT_PATTERNS = ("front_.*_wheel_joint",)
#: The rear casters (swivel + 0.03 m wheel) are passive on the real base, and
#: must be passive here too.  The URDF importer bakes a stiffness-625,
#: unlimited-force position drive (target 0) onto every continuous joint, so
#: an unconfigured swivel is rigidly held straight; and a caster wheel driven
#: at the front wheels' angular velocity (base_facade commands all four) is
#: wrong by the radius ratio, so its damping-200 drive saturates as a brake:
#: forward driving ran the fronts at 3.1 rad/s against a 3.8 target, and an
#: in-place turn reached ~20% of the commanded yaw rate with the fronts
#: stalled and chattering (wheel odometry garbage -> Nav2 stalls at the
#: goal).  A zero-gain group overrides the baked drives and frees them.
CASTER_JOINT_PATTERNS = ("rear_.*_swivel_joint", "rear_.*_wheel_joint")
#: Wheel links whose authored cylinder collider is replaced by a sphere of the
#: same radius at spawn (see ``_apply_wheel_sphere_colliders``).  The exact
#: cylinder's line contact across the 63 mm tread cannot roll on a 0.125 m
#: turn radius (inner and outer edges need different speeds), so the contact
#: patch locks and an in-place turn reached 0% of the command below
#: 0.2 rad/s and ~55% at 0.5 rad/s -- independent of drive effort, solver
#: type, iteration count and sleep settings.  A single contact point rolls:
#: measured truth yaw rate 0.098/0.197/0.298/0.497/0.795 for commands of
#: 0.1/0.2/0.3/0.5/0.8 rad/s.  The sphere touches the floor where the
#: cylinder did, so base height and odometry ``wheel_radius_m`` are unchanged.
WHEEL_COLLIDER_LINKS = (
    "front_left_wheel",
    "front_right_wheel",
    "rear_left_wheel",
    "rear_right_wheel",
)
WHEEL_COLLIDER_MODES = ("sphere", "cylinder")


def resolve_wheel_collider_mode(value: str | None) -> str:
    """Parse ``TINKER_SIM_WHEEL_COLLIDER``: ``sphere`` (default) or the
    authored ``cylinder`` for A/B runs."""
    mode = (value or "").strip().lower()
    if not mode:
        return "sphere"
    if mode not in WHEEL_COLLIDER_MODES:
        raise ValueError(
            "TINKER_SIM_WHEEL_COLLIDER must be one of "
            + ", ".join(WHEEL_COLLIDER_MODES)
            + f", got {value!r}"
        )
    return mode


def wheel_sphere_radius(cylinder_radius: object) -> float:
    """Sphere radius for a wheel: exactly the authored cylinder radius."""
    if isinstance(cylinder_radius, bool) or not isinstance(cylinder_radius, (int, float)):
        raise TypeError("wheel cylinder radius must be a number")
    radius = float(cylinder_radius)
    if not math.isfinite(radius) or radius <= 0.0:
        raise ValueError(f"wheel cylinder radius must be positive and finite, got {radius!r}")
    return radius




#: Mass authored at spawn onto robot links whose URDF declares no ``<inertial>``.
#: The URDF -> USD importer leaves ``physics:mass`` unauthored on those links
#: and PhysX then assigns its 1.0 kg default to every one of them.  This robot
#: description uses 22 such links as pure frames (link_eef, link_tcp, the
#: eleven wrist/head camera frame links, pan/tilt stubs), so the default hangs
#: ~11 kg of phantom mass off the wrist and ~10 kg off the head.  Measured
#: 2026-08-31 (in-process manipulation-core probe): the phantom wrist mass
#: alone loads joint4 with 50.0 Nm of static gravity torque at the
#: orchestrator's tuck posture -- exactly its effort cap -- so the elbow
#: stalls up to 0.04 rad short of its target and every tuck trajectory aborts
#: on the controller's 0.01 rad goal tolerance.  Real downstream mass at the
#: elbow is ~3.6 kg (<= 15 Nm).  1 g per frame link keeps the articulation
#: valid for PhysX while contributing nothing measurable.
STUB_LINK_MASS_KG = 0.001


def massless_stub_links(urdf_bytes: bytes) -> tuple[str, ...]:
    """Links the URDF declares with neither ``<inertial>`` nor ``<collision>``.

    These are attachment frames, not physical bodies: the description gives
    them no mass, no inertia, and no geometry, and the real robot treats them
    as coordinate frames only.  Excludes ``world`` (never a rigid body).
    Fails closed on malformed XML -- a broken robot description must not
    silently disable the mass correction.
    """
    import xml.etree.ElementTree as ElementTree

    root = ElementTree.fromstring(urdf_bytes)
    names = []
    for link in root.findall("link"):
        name = link.get("name")
        if not name or name == "world":
            continue
        if link.find("inertial") is None and link.find("collision") is None:
            names.append(name)
    return tuple(names)


def slew_velocity_target(current: float, target: float, max_delta: float) -> float:
    """Move a velocity target toward ``target`` by at most ``max_delta``.

    Wheel targets were previously applied verbatim (README: upstream owns
    acceleration limits — nothing upstream did). This bounds every wheel
    transient, including stale-held-target windows, without overshooting.
    """
    values = (current, target, max_delta)
    if any(isinstance(v, bool) or not math.isfinite(float(v)) for v in values):
        raise ValueError("slew inputs must be finite numbers")
    if max_delta < 0.0:
        raise ValueError("max_delta must be non-negative")
    delta = target - current
    if abs(delta) <= max_delta:
        return float(target)
    return float(current + math.copysign(max_delta, delta))


#: Uniform wall material used when no palette override is supplied.
DEFAULT_WALL_COLOR = (0.35, 0.38, 0.42)


def _map_metadata(path: Path) -> tuple[Path, float, float, float]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    origin = json.loads(values["origin"])
    return (
        path.parent / values["image"],
        float(values["resolution"]),
        float(origin[0]),
        float(origin[1]),
    )


def resolve_arena_inputs(
    arena_artifact: Path | None, map_yaml: Path | None
) -> tuple[Path | None, Path | None]:
    """Resolve the effective ``(arena_usd, map_yaml)`` pair for the backend.

    ``arena_artifact`` and an explicit ``map_yaml`` are mutually exclusive: an
    arena artifact directory carries its own colocated ``arena.usd`` and
    ``map.yaml``. When no arena artifact is supplied, ``map_yaml`` passes
    through unchanged.
    """
    if arena_artifact is None:
        return None, map_yaml
    if map_yaml is not None:
        raise ValueError(
            "arena_artifact and map_yaml are mutually exclusive: an arena "
            "artifact supplies its own colocated map.yaml"
        )
    arena_usd = arena_artifact / "arena.usd"
    effective_map_yaml = arena_artifact / "map.yaml"
    if not arena_usd.is_file():
        raise FileNotFoundError(f"arena artifact missing arena.usd: {arena_usd}")
    if not effective_map_yaml.is_file():
        raise FileNotFoundError(
            f"arena artifact missing map.yaml: {effective_map_yaml}"
        )
    return arena_usd, effective_map_yaml


def validate_spawn_xy(spawn_xy: object) -> tuple[float, float]:
    """Validate an ``(x, y)`` robot spawn override.

    The default arena spawn of world (0, 0) lands inside shelf_02's physical
    and rasterized footprint in the rcw2026 arena, so navigation work needs an
    explicit free-space spawn. Fails closed on malformed or non-finite input.
    """
    try:
        x_raw, y_raw = spawn_xy  # type: ignore[misc]
    except (TypeError, ValueError):
        raise ValueError("spawn_xy must be a two-element (x, y) pair")
    if isinstance(x_raw, bool) or isinstance(y_raw, bool):
        raise ValueError("spawn_xy values must be numbers")
    try:
        x, y = float(x_raw), float(y_raw)
    except (TypeError, ValueError):
        raise ValueError("spawn_xy values must be numbers")
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("spawn_xy values must be finite")
    return x, y


def resolve_spawn_yaw(value: str | None) -> float:
    """Parse ``TINKER_SIM_SPAWN_YAW`` (radians, world frame; default 0).

    Fails closed on malformed or non-finite input, mirroring
    ``validate_spawn_xy``.
    """
    if value is None or not value.strip():
        return 0.0
    try:
        yaw = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"TINKER_SIM_SPAWN_YAW must be a number, got {value!r}")
    if not math.isfinite(yaw):
        raise ValueError("TINKER_SIM_SPAWN_YAW must be finite")
    return yaw


def _fused_apply_actuator_model(self) -> None:
    """Replacement for ``Articulation._apply_actuator_model`` that launches once.

    Isaac Lab's stock implementation runs, for every actuator group, the
    group's ``compute()`` and then two Warp launches (``update_targets`` and
    ``update_actuator_state_model``) that scatter the group's slice into the
    articulation-wide staging buffers. With this robot's five implicit groups
    that is ten launches per target push; measured 2026-08-21 on CPU PhysX
    the launches are ~3.2 ms of a ~3.6 ms push, the five ``compute()`` calls
    ~0.3 ms.

    This version keeps the per-group ``compute()`` exactly as stock -- every
    group's own stiffness/damping/effort-limit tensors and its
    ``computed_effort``/``applied_effort`` buffers are used and updated the
    same way, so telemetry and the runtime gripper effort-limit sync are
    unchanged -- and then issues each kernel once over the concatenation of
    all groups' joint indices. The staging and data buffers receive the same
    values at the same joints; no joint belongs to two groups, so ordering
    cannot matter. Bound per instance (Isaac Lab's factory classes refuse
    subclassing outside the package); any group with a gear ratio falls back
    to the stock loop.
    """
    import torch
    import warp as wp
    from isaaclab_physx.assets.articulation import kernels as articulation_kernels
    from isaaclab.utils.types import ArticulationActions

    actuators = list(self.actuators.values())
    if not actuators or any(hasattr(actuator, "gear_ratio") for actuator in actuators):
        return type(self)._apply_actuator_model(self)
    fused = getattr(self, "_tinker_fused_indices", None)
    if fused is None:
        pieces = []
        for actuator in actuators:
            joint_indices = actuator.joint_indices
            if joint_indices == slice(None) or joint_indices is None:
                pieces.append(torch.arange(self.num_joints, dtype=torch.int32, device=self.device))
            else:
                pieces.append(torch.as_tensor(joint_indices, device=self.device).to(torch.int32))
        fused_torch = torch.cat(pieces)
        fused = (
            fused_torch.to(torch.long),
            wp.array(fused_torch.cpu().numpy(), dtype=wp.int32, device=self.device),
            [piece.to(torch.long) for piece in pieces],
        )
        self._tinker_fused_indices = fused
    fused_long, fused_wp, group_long = fused

    joint_pos_target = self._data.joint_pos_target.torch
    joint_vel_target = self._data.joint_vel_target.torch
    joint_effort_target = self._data.joint_effort_target.torch
    joint_pos = self._data.joint_pos.torch
    joint_vel = self._data.joint_vel.torch
    computed = []
    applied = []
    vel_limits = []
    for actuator, idx in zip(actuators, group_long):
        control_action = ArticulationActions(
            joint_positions=joint_pos_target[:, idx],
            joint_velocities=joint_vel_target[:, idx],
            joint_efforts=joint_effort_target[:, idx],
            joint_indices=actuator.joint_indices,
        )
        actuator.compute(control_action, joint_pos=joint_pos[:, idx], joint_vel=joint_vel[:, idx])
        computed.append(actuator.computed_effort)
        applied.append(actuator.applied_effort)
        vel_limits.append(actuator.velocity_limit)
    wp.launch(
        articulation_kernels.update_targets,
        dim=(self.num_instances, fused_long.shape[0]),
        inputs=[
            joint_pos_target[:, fused_long].contiguous(),
            joint_vel_target[:, fused_long].contiguous(),
            joint_effort_target[:, fused_long].contiguous(),
            fused_wp,
        ],
        outputs=[self._joint_pos_target_sim, self._joint_vel_target_sim, self._joint_effort_target_sim],
        device=self.device,
    )
    wp.launch(
        articulation_kernels.update_actuator_state_model,
        dim=(self.num_instances, fused_long.shape[0]),
        inputs=[
            torch.cat(computed, dim=1).contiguous(),
            torch.cat(applied, dim=1).contiguous(),
            None,
            torch.cat(vel_limits, dim=1).contiguous(),
            fused_wp,
        ],
        outputs=[
            self._data.computed_torque,
            self._data.applied_torque,
            self._data.gear_ratio,
            self._data.soft_joint_vel_limits,
        ],
        device=self.device,
    )


def bind_fused_actuator_model(robot: Any) -> None:
    """Bind `_fused_apply_actuator_model` on this articulation instance."""
    import types

    robot._apply_actuator_model = types.MethodType(_fused_apply_actuator_model, robot)


class IsaacWholeRobotBackend:
    """CPU-PhysX articulation controlled only by standard JointState commands."""

    TRUTH_TOKEN = object()
    PHYSICS_TRUTH_SCHEMA_VERSION = 2
    DEFAULT_GRIPPER_EFFORT_LIMIT = 80.0
    # The stopped arm uses one explicit actuator path.  These fixed gains are
    # intentionally sized for a five-physics-frame (5 / 120 s) stop: the
    # velocity term removes motion immediately, while the position term keeps
    # the measured stop position latched without relying on an implicit drive.
    SAFETY_STOP_POSITION_GAIN = 600.0
    SAFETY_STOP_VELOCITY_GAIN = 80.0
    # The hold is PhysX's own joint drive at the latched pose: stiffness and
    # damping are the hold PD gains, evaluated by the solver at every substep
    # rather than by Python at the control rate (which, with zero drive
    # damping, limit-cycled: measured 2026-08-21 joint1 pinned at -100 Nm).
    SAFETY_HOLD_STIFFNESS = SAFETY_STOP_POSITION_GAIN
    SAFETY_HOLD_DAMPING = SAFETY_STOP_VELOCITY_GAIN
    # Control steps between gravity feed-forward refreshes while held. The
    # held pose is static, so the compensation changes only with slow sag.
    SAFETY_HOLD_GRAVITY_REFRESH_STEPS = 30
    SAFETY_HOLD_EFFORT_LIMIT = 100.0
    # joint[1-2] tier values only; used as a degenerate fallback when live
    # per-joint gain reads fail (see _read_joint_gain_values).
    NOMINAL_ARM_STIFFNESS = 20_000.0
    NOMINAL_ARM_DAMPING = 1_500.0
    CONTACT_FORCE_THRESHOLD = 1.0
    ARM_CONTACT_BODIES = tuple(f"link{index}" for index in range(1, 8))
    GRASP_CONTACT_BODIES = ("left_finger", "right_finger", "link_tcp")

    def __init__(
        self,
        *,
        usd_path: Path,
        map_yaml: Path | None,
        seed: int,
        physics_hz: float = 120.0,
        render: bool = False,
        spawn_z: float = 0.20,
        enable_contacts: bool = False,
        add_ground_plane: bool = True,
        expected_objects: Mapping[str, Mapping[str, object]] | None = None,
        scenario: str = "",
        task: str = "",
        wall_color_fn: Callable[[int], tuple[float, float, float]] | None = None,
        arena_artifact: Path | None = None,
        spawn_xy: tuple[float, float] = (0.0, 0.0),
    ) -> None:
        spawn_x, spawn_y = validate_spawn_xy(spawn_xy)
        arena_usd, map_yaml = resolve_arena_inputs(arena_artifact, map_yaml)
        if arena_artifact is not None and wall_color_fn is not None:
            raise ValueError(
                "arena_artifact and wall_color_fn are mutually exclusive: "
                "wall coloring only applies to procedurally spawned cuboids"
            )

        import torch
        import omni.timeline
        import isaaclab.sim as sim_utils
        from isaaclab.actuators import ImplicitActuatorCfg
        from isaaclab.assets import Articulation, ArticulationCfg
        from isaaclab.sim import SimulationCfg, SimulationContext
        from omni.physx import get_physx_simulation_interface
        from omni.physx.bindings._physx import ContactEventType
        from pxr import PhysicsSchemaTools

        self._torch = torch
        # Opt-in override; unset, the validated 120 Hz above is used.
        # See simulation/tinker_sim_isaac/physics_rate.py for why the
        # override may only lower the rate.
        import os as _os

        physics_hz = resolve_physics_hz(
            physics_hz, _os.environ.get("TINKER_SIM_PHYSICS_HZ")
        )
        self.physics_hz = physics_hz
        # The control step (target writes, wheel slew, Articulation.update,
        # gateway strides, /clock) may be a whole multiple of the PhysX solver
        # step: each control step then runs `physics_substeps` Isaac Lab
        # physics steps of the validated 1/physics_hz, so contact fidelity is
        # unchanged while every per-step wrapper cost is paid control_hz times
        # a second. (omni.physx's IPhysxSimulation.simulate(elapsed) does NOT
        # substep -- measured 2026-08-21: one 1/60 s solver step for a 1/60 s
        # call even with timeStepsPerSecond=120 -- so the substeps are
        # explicit.) Opt-in via TINKER_SIM_CONTROL_HZ; unset, control_hz ==
        # physics_hz and the loop is exactly what it was.
        control_hz = resolve_control_hz(
            physics_hz, _os.environ.get("TINKER_SIM_CONTROL_HZ")
        )
        self.control_hz = control_hz
        self.physics_substeps = physics_substeps(physics_hz, control_hz)
        self.dt = 1.0 / control_hz
        self.physics_dt = 1.0 / physics_hz
        self.render = render
        # Opt-in per-step wall-time attribution (TINKER_SIM_PROFILE=1). Splits
        # the PhysX solve from the Isaac Lab tensor/Python work around it.
        import os as _os
        import time as _time

        # Physics steps between attempts to resolve not-yet-found scenario
        # objects (each miss costs a full USD stage traversal). 60 steps is
        # 0.5 s at the default 120 Hz. Set to 1 to restore per-step retry.
        self._object_discovery_interval = max(
            1, int(_os.environ.get("TINKER_SIM_OBJECT_DISCOVERY_INTERVAL", "60"))
        )
        self._object_discovery_step = 0
        self._target_write_gate = TargetWriteGate(
            always_write=_os.environ.get("TINKER_SIM_ALWAYS_WRITE_TARGETS", "")
            == "1"
        )
        self.step_profile = {
            "enabled": _os.environ.get("TINKER_SIM_PROFILE", "") == "1",
            "_clock": _time.monotonic,
            "_mark": _time.monotonic(),
            "targets": 0.0,
            "write_data": 0.0,
            "physx": 0.0,
            "robot_update": 0.0,
            "object_views": 0.0,
            "target_writes": 0,
            "n": 0,
        }
        self.seed = int(seed)
        self.physics_device = "cpu"
        self.contacts_enabled = bool(enable_contacts)
        # Discovery is not proof of an effective safety-clear state.
        self._safety_stopped = True
        self._safety_snapshot: Any | None = None
        self._safety_joint_ids: tuple[int, ...] = ()
        self._safety_nominal_stiffness: tuple[float, ...] = ()
        self._safety_nominal_damping: tuple[float, ...] = ()
        self._safety_nominal_effort_limits: tuple[float, ...] = ()
        self._safety_gains_applied = False
        self._safety_hold_steps = 0
        self._safety_hold_effort: Any | None = None
        self._command_snapshot_id: int | None = None
        self._pending_snapshot_id: int | None = None
        self._pending_snapshot_count = 0
        self._pending_snapshot_index = 0
        self._pending_snapshot_commands: list[JointCommand] = []
        self._default_gripper_effort_limit = self.DEFAULT_GRIPPER_EFFORT_LIMIT
        self._gripper_effort_limit = self._default_gripper_effort_limit
        # The first commanded limit always reaches PhysX; later identical
        # requests are no-ops (see _set_gripper_effort_limit).
        self._gripper_effort_limit_written = False
        self.gripper_effort_limit_writes = 0
        self._expected_objects = {
            str(name): dict(value) for name, value in (expected_objects or {}).items()
        }
        self.scenario = str(scenario)
        self.task = str(task or scenario)
        self._object_views: dict[str, Any] = {}
        # Diagnostic pose tracker for arbitrary rigid-body prims, opt-in via
        # TINKER_SIM_TRACK_OBJECTS=/World/Scenario/a,/World/Scenario/b.
        # Prints one JSON line per tracked prim every ~0.25 s of control
        # time through the backend's own (working) view read path -- the
        # standard channels cannot answer "where did a spawned object go":
        # /get_entity_state reads the stale USD layer for spawned bodies and
        # physics-truth only covers scenario-declared objects. Built for the
        # 2026-08-31 vanishing-spawn investigation; costs nothing unset.
        self._tracked_object_paths = tuple(
            item.strip()
            for item in _os.environ.get("TINKER_SIM_TRACK_OBJECTS", "").split(",")
            if item.strip()
        )
        self._tracked_object_views: dict[str, Any] = {}
        self._tracked_object_step = 0
        # Self-healing watchdog for the mid-play spawn attach race: about 1
        # in 3 /spawn_entity spawns onto a playing timeline never enters
        # PhysX (prim created and acked, RigidBodyAPI authored, but no
        # rigid body ever materialises -- a ghost no gripper or teleport
        # can touch; observed 2026-08-31, per-spawn nondeterministic, same
        # boot attaches the next spawn fine). The race lives in
        # omni.physx's incremental parse of stage notices, which this repo
        # cannot patch, so the backend watches /World/Scenario children
        # and, when a rigid-body prim has no PhysX body after a discovery
        # interval, re-triggers the parse with the standard active-toggle
        # nudge. Attached bodies are never touched; a ghost cannot be made
        # worse. TINKER_SIM_HEAL_DETACHED_SPAWNS=0 disables.
        self._heal_detached_spawns = (
            _os.environ.get("TINKER_SIM_HEAL_DETACHED_SPAWNS", "1") == "1"
        )
        #: path -> {"first_step", "attempts", "healed"} for scenario children.
        self._spawn_attach_watch: dict[str, dict[str, int]] = {}
        self._spawn_attach_step = 0
        self._contact_pairs_by_key: dict[tuple[int, int, int, int], dict[str, object]] = {}
        self._contact_path_decoder = lambda path_id: str(
            PhysicsSchemaTools.intToSdfPath(path_id)
        )
        self._contact_event_found = ContactEventType.CONTACT_FOUND
        self._contact_event_lost = ContactEventType.CONTACT_LOST
        self._contact_event_persist = ContactEventType.CONTACT_PERSIST
        self._contact_report_subscription: Any | None = None
        # use_fabric: SPAWN_YAW authors a non-identity orient on the robot
        # root, which leaks through omni.physx.fabric's C++ ingestion of
        # newly-spawned rigid bodies -- sibling /World/Scenario spawns land
        # rotated by -robot_yaw about the robot origin in PHYSICS, while
        # USD/get_entity_state read back the commanded pose correctly (the
        # spawn's XformPrim usd-path composition is provably correct; the leak
        # is below Python, in fabric's world-transform resolution against the
        # rotated root while /physics/updateToUsd is off). Disabling fabric
        # makes USD authoritative for spawned-body ingestion and removes the
        # leak; the cost is per-step PhysX->USD transform write-back --
        # rendering, cameras/render products and the tensor pipeline are gated
        # separately and unaffected. Default on (keeps the hard-won RTF for
        # nav/arena runs); auto-off only when SPAWN_YAW is set (the sole
        # trigger, itself opt-in); TINKER_SIM_USE_FABRIC=0/1 forces either way
        # (e.g. to A/B the RTF cost).
        _spawn_yaw_set = (
            abs(resolve_spawn_yaw(_os.environ.get("TINKER_SIM_SPAWN_YAW"))) > 1.0e-9
        )
        _use_fabric = not _spawn_yaw_set
        _use_fabric_env = _os.environ.get("TINKER_SIM_USE_FABRIC")
        if _use_fabric_env == "1":
            _use_fabric = True
        elif _use_fabric_env == "0":
            _use_fabric = False
        print(
            json.dumps(
                {"use_fabric": _use_fabric, "spawn_yaw_set": _spawn_yaw_set},
                sort_keys=True,
            ),
            flush=True,
        )
        self._sim = SimulationContext(
            SimulationCfg(
                dt=self.physics_dt,
                device=self.physics_device,
                render_interval=1,
                use_fabric=_use_fabric,
            )
        )
        self._timeline = omni.timeline.get_timeline_interface()
        if str(self._sim.device) != self.physics_device:
            raise RuntimeError(
                f"behavior validation requires CPU physics; Isaac selected {self._sim.device}"
            )
        if self.physics_substeps > 1:
            print(
                json.dumps(
                    {
                        "physics_hz": self.physics_hz,
                        "control_hz": self.control_hz,
                        "physx_substeps": self.physics_substeps,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )

        if add_ground_plane:
            ground = sim_utils.GroundPlaneCfg()
            ground.func("/World/defaultGroundPlane", ground)
        light = sim_utils.DomeLightCfg(intensity=1200.0, color=(0.95, 0.95, 1.0))
        light.func("/World/DomeLight", light)

        self.occupancy: OccupancyMap | None = None
        if map_yaml is not None:
            pgm, resolution, origin_x, origin_y = _map_metadata(map_yaml)
            self.occupancy = OccupancyMap.from_pgm(
                pgm, resolution=resolution, origin_x=origin_x, origin_y=origin_y
            )
            if arena_usd is not None:
                from isaacsim.core.utils.stage import add_reference_to_stage

                add_reference_to_stage(str(arena_usd.resolve()), "/World/Arena")
            else:
                for index, (x, y, sx, sy) in enumerate(self.occupancy.rectangles()):
                    color = (
                        DEFAULT_WALL_COLOR
                        if wall_color_fn is None
                        else tuple(wall_color_fn(index))
                    )
                    box = sim_utils.CuboidCfg(
                        size=(sx, sy, 1.2),
                        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                        collision_props=sim_utils.CollisionPropertiesCfg(),
                        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color),
                    )
                    box.func(
                        f"/World/NavigationMap/occupied_{index:04d}",
                        box,
                        translation=(x, y, 0.6),
                    )

        # Opt-in articulation solver iteration override; unset, the USD's
        # authored counts (32 position / 1 velocity for tinker2) apply.
        solver_position = resolve_solver_iterations(
            "position", _os.environ.get("TINKER_SIM_SOLVER_POSITION_ITERATIONS")
        )
        solver_velocity = resolve_solver_iterations(
            "velocity", _os.environ.get("TINKER_SIM_SOLVER_VELOCITY_ITERATIONS")
        )
        self.solver_iterations = {
            "position": solver_position,
            "velocity": solver_velocity,
        }
        # Opt-in kinematic base hold (TINKER_SIM_FIX_BASE=1): the free base
        # yaws 5-17 deg per pick under arm reaction torques, drifting every
        # base-frame goal cm off its world target. Held by latching the
        # settled root pose and re-writing it each step (see _apply_base_hold)
        # -- NOT fix_root_link, which makes the arm joints stick-slip and
        # welds at the airborne spawn pose. Off by default: navigation
        # profiles need a rolling base.
        self.base_fixed = _os.environ.get("TINKER_SIM_FIX_BASE") == "1"
        self._base_hold_pose = None  # latched [1,7] pos+quat(xyzw) tensor
        self._base_hold_vel = None   # zeros [1,6]
        # Let the free base drop-settle onto its wheels before latching.
        self._base_hold_after_sim_s = 2.0
        # A mid-play /spawn_entity (or delete) rebuilds the shared physics
        # view; a root write issued across that rebuild aliases onto the
        # freshly inserted body, teleporting the spawned object to the robot's
        # latched pose. Watch the Scenario child set and pause hold writes for
        # a short resettle window on any change, so writes only resume once
        # they target the true robot root again.
        self._base_hold_scene_sig: int | None = None
        self._base_hold_skip_until_sim_s = 0.0
        self._base_hold_resettle_s = 0.15
        try:
            _resettle = _os.environ.get("TINKER_SIM_FIX_BASE_RESETTLE_S")
            if _resettle is not None and _resettle.strip():
                self._base_hold_resettle_s = max(0.0, float(_resettle))
        except (TypeError, ValueError):
            pass
        # Diagnostic (TINKER_SIM_FIX_BASE_DRYRUN=1): run the whole hold -- settle,
        # latch, scene-change detection, logging -- but SKIP the two
        # set_root_transforms writes. If a spawn still lands at the robot pose
        # with writes suppressed, the hold's root writes are NOT the cause
        # (isaaclab source shows they are scoped to /World/Tinker and cannot
        # address a loose body); if the spawn lands correctly, they are.
        self._base_hold_dryrun = _os.environ.get("TINKER_SIM_FIX_BASE_DRYRUN") == "1"
        articulation_props = None
        if solver_position is not None or solver_velocity is not None:
            articulation_props = sim_utils.ArticulationRootPropertiesCfg(
                solver_position_iteration_count=solver_position,
                solver_velocity_iteration_count=solver_velocity,
            )
            print(
                json.dumps({"solver_iterations": self.solver_iterations}, sort_keys=True),
                flush=True,
            )
        robot_cfg = ArticulationCfg(
            prim_path="/World/Tinker",
            spawn=sim_utils.UsdFileCfg(
                usd_path=str(usd_path.resolve()),
                activate_contact_sensors=enable_contacts,
                joint_drive_props=sim_utils.JointDriveBaseCfg(drive_type="force"),
                articulation_props=articulation_props,
            ),
            # TINKER_SIM_SPAWN_YAW (radians, world frame): spawn heading. The
            # spawner drops this rot for a USD-referenced articulation (it is
            # re-authored on the root xformOp:orient just after construction),
            # but it is set here too so the intent reads with the pose. A
            # fixed-root/held base can never be re-aimed after spawn and a
            # post-bind /set_entity_state root write is a physics no-op, so the
            # heading must be right from the start.
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(spawn_x, spawn_y, spawn_z),
                rot=(
                    math.cos(resolve_spawn_yaw(
                        _os.environ.get("TINKER_SIM_SPAWN_YAW")) / 2.0),
                    0.0,
                    0.0,
                    math.sin(resolve_spawn_yaw(
                        _os.environ.get("TINKER_SIM_SPAWN_YAW")) / 2.0),
                ),
            ),
            actuators={
                "arm": ImplicitActuatorCfg(
                    joint_names_expr=["joint[1-7]"],
                    # stiffness ~200x effort cap; uniform 20000 drove all joints to
                    # 26-76% effort-cap dwell in recorded execute-joint runs (bang-bang saturation).
                    stiffness={
                        "joint[1-2]": 20000.0,
                        "joint[3]": 6000.0,
                        "joint[4]": 7000.0,
                        "joint[5]": 6000.0,
                        "joint[6]": 12000.0,
                        "joint[7]": 4000.0,
                    },
                    damping={
                        "joint[1-2]": 1500.0,
                        "joint[3]": 450.0,
                        "joint[4]": 600.0,
                        "joint[5]": 450.0,
                        "joint[6]": 800.0,
                        "joint[7]": 300.0,
                    },
                    effort_limit_sim={
                        "joint[1-2]": 100.0,
                        "joint[3]": 30.0,
                        "joint[4]": 50.0,
                        "joint[5]": 30.0,
                        "joint[6-7]": 20.0,
                    },
                ),
                "head": ImplicitActuatorCfg(
                    joint_names_expr=["pan_joint", "tilt_joint"],
                    stiffness=500.0,
                    damping=50.0,
                    # The URDF's effort="1.0" is a hand-authored placeholder
                    # (massless stub links, no ros2_control entry); 1 Nm cannot
                    # move the head against default-assigned inertias.
                    effort_limit_sim=10.0,
                ),
                "gripper": ImplicitActuatorCfg(
                    joint_names_expr=["drive_joint"],
                    stiffness=200.0,
                    damping=20.0,
                ),
                "gripper_mimic": ImplicitActuatorCfg(
                    joint_names_expr=[".*finger.*", ".*knuckle.*"],
                    stiffness=0.0,
                    damping=0.0,
                ),
                "casters": ImplicitActuatorCfg(
                    joint_names_expr=["rear_.*_swivel_joint", "rear_.*_wheel_joint"],
                    stiffness=0.0,
                    damping=0.0,
                ),
                "wheels": ImplicitActuatorCfg(
                    joint_names_expr=["front_.*_wheel_joint"],
                    stiffness=0.0,
                    damping=200.0,
                    velocity_limit_sim=30.0,
                    effort_limit_sim=80.0,
                ),
            },
        )
        self._robot = Articulation(robot_cfg)
        # Author the spawn yaw directly on the robot root prim: the spawner
        # honors InitialStateCfg.pos (xformOp:translate) but drops the
        # orientation for this USD-referenced articulation (observed: rot
        # (0.707, 0, 0, 0.707) requested, layer yaw 0 after spawn), and
        # post-bind /set_entity_state root writes are physics no-ops.
        spawn_yaw = resolve_spawn_yaw(_os.environ.get("TINKER_SIM_SPAWN_YAW"))
        if abs(spawn_yaw) > 1.0e-9:
            from pxr import Gf, UsdGeom
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            robot_prim = stage.GetPrimAtPath("/World/Tinker")
            if not robot_prim.IsValid():
                raise RuntimeError("TINKER_SIM_SPAWN_YAW: /World/Tinker not found")
            xformable = UsdGeom.Xformable(robot_prim)
            half = spawn_yaw / 2.0
            quat = Gf.Quatd(math.cos(half), Gf.Vec3d(0.0, 0.0, math.sin(half)))
            orient_op = None
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
                    orient_op = op
                    break
            if orient_op is None:
                orient_op = xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
            if orient_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
                orient_op.Set(Gf.Quatf(quat))
            else:
                orient_op.Set(quat)
            print(json.dumps({"spawn_yaw_rad": spawn_yaw}, sort_keys=True), flush=True)
        # Ten Warp launches per target push -> two; see _fused_apply_actuator_model.
        if _os.environ.get("TINKER_SIM_STOCK_ACTUATOR_MODEL", "") != "1":
            bind_fused_actuator_model(self._robot)
        self.chassis_ballast_mass_kg = self._apply_chassis_ballast_mass()
        self._apply_two_tone_paint()
        self._robot_view_identity: int | None = None
        self._clock_step_origin = 0
        # Elapsed steps observed at the last simulation_time read; anchors the
        # monotonic-clock continuation across STOP -> PLAY resets (see
        # _refresh_robot_handles).
        self._clock_elapsed_steps = 0
        import omni.kit.app

        # Flush USD/Fabric stage notices before SimulationContext creates the
        # articulation tensor view.  Without this update, sim_control applies
        # the deferred contact-prim changes on the first physics frame.
        omni.kit.app.get_app().update()
        self.wheel_collider_mode = resolve_wheel_collider_mode(
            _os.environ.get("TINKER_SIM_WHEEL_COLLIDER")
        )
        if self.wheel_collider_mode == "sphere":
            self._apply_wheel_sphere_colliders()
            omni.kit.app.get_app().update()
        print(json.dumps({"wheel_collider": self.wheel_collider_mode}, sort_keys=True), flush=True)
        self.stub_links_corrected = self._apply_stub_link_masses(usd_path)
        print(
            json.dumps(
                {
                    "stub_link_mass_kg": STUB_LINK_MASS_KG,
                    "stub_links_corrected": list(self.stub_links_corrected),
                },
                sort_keys=True,
            ),
            flush=True,
        )
        omni.kit.app.get_app().update()
        self._sim.reset()
        # UsdFileCfg imports the robot stage metadata, including its short
        # playback range.  Override it only after every USD has been authored.
        # Reaching that range stops PhysX and invalidates tensor views.
        self._timeline.set_looping(False)
        self._timeline.set_end_time(365.0 * 24.0 * 60.0 * 60.0)
        if not self._refresh_robot_handles():
            raise RuntimeError("Tinker articulation did not initialize after simulation reset")
        self._robot.update(self.dt)
        self._safety_snapshot = self._robot.data.joint_pos.clone()
        if enable_contacts:
            self._contact_report_subscription = (
                get_physx_simulation_interface().subscribe_contact_report_events(
                    self._on_contact_report_event
                )
            )
            if self._contact_report_subscription is None:
                raise RuntimeError("manipulation-core requires PhysX contact reporting")

    def _step_simulation(self) -> None:
        """One control step: ``physics_substeps`` solver steps of ``physics_dt``.

        Each Isaac Lab ``step()`` is one ``IPhysxSimulation.simulate(dt)`` +
        ``fetch_results()`` at the validated solver dt; Kit is rendered (when
        enabled) only after the last substep.
        """
        last = self.physics_substeps - 1
        for index in range(self.physics_substeps):
            self._sim.step(render=self.render and index == last)

    def _apply_two_tone_paint(self) -> None:
        """Two-tone the robot so the arm reads apart from the chassis on camera.

        Arm links stay near-white with a hint of grey; everything else on the
        chassis goes dark grey.  Bindings target each link's ``visuals`` scope
        with strongerThanDescendants so they override the asset's per-mesh
        materials; collision meshes are left alone (never rendered).  Sim-only
        visual aid; opt out with TINKER_SIM_TWO_TONE=0.
        """
        import os as _os

        if _os.environ.get("TINKER_SIM_TWO_TONE", "1") == "0":
            return
        import omni.usd
        from pxr import Gf, Sdf, UsdShade

        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath("/World/Tinker")
        if not root.IsValid():
            return

        def _material(name: str, rgb, roughness: float, metallic: float):
            path = Sdf.Path(f"/World/Tinker/Looks/two_tone_{name}")
            material = UsdShade.Material.Define(stage, path)
            shader = UsdShade.Shader.Define(stage, path.AppendChild("shader"))
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            return material

        # Deterministic scene lighting.  The arena has appeared fully lit only
        # after a destructive /reset_simulation STOP->PLAY; a plain bring-up
        # (or the teleport-based per-run reset) can leave the stage so dark
        # that the head camera sees only the emissive floor grid — which
        # starves every vision detector.  A dome light guarantees a lit scene
        # regardless of reset history.
        from pxr import UsdLux

        dome_path = "/World/tinker_sim_dome_light"
        if not stage.GetPrimAtPath(dome_path).IsValid():
            dome = UsdLux.DomeLight.Define(stage, dome_path)
            dome.CreateIntensityAttr(1000.0)
            dome.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
            print("[tinker-sim] dome light added for deterministic scene lighting")

        arm = _material("arm_white", (0.92, 0.92, 0.94), 0.55, 0.0)
        body = _material("body_grey", (0.25, 0.25, 0.27), 0.6, 0.25)
        # Arm chain: link_base/link1-7/link_eef/link_tcp, gripper base, and the
        # knuckle/finger parts (wheels are front_/rear_-prefixed, so the bare
        # left_/right_ prefixes only catch gripper parts).
        arm_prefixes = ("link", "xarm_gripper", "left_", "right_")
        painted = 0
        for child in root.GetChildren():
            name = child.GetName()
            if name in ("Looks", "joints"):
                continue
            visuals = child.GetChild("visuals")
            target_prim = visuals if visuals.IsValid() else child
            material = arm if name.startswith(arm_prefixes) else body
            UsdShade.MaterialBindingAPI.Apply(target_prim).Bind(
                material, UsdShade.Tokens.strongerThanDescendants
            )
            painted += 1
        print(f"[tinker-sim] two-tone paint applied to {painted} links "
              "(arm near-white, chassis dark grey)")

    def _apply_chassis_ballast_mass(self) -> float:
        """Add 10 kg to the existing low-mounted chassis ballast before reset."""
        import omni.usd
        from pxr import Gf, UsdPhysics

        prim_path = "/World/Tinker/ballast"
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid() or not prim.HasAPI(UsdPhysics.MassAPI):
            raise RuntimeError(f"Tinker chassis ballast rigid body is missing: {prim_path}")
        mass_api = UsdPhysics.MassAPI(prim)
        current_mass = mass_api.GetMassAttr().Get()
        if current_mass is None:
            raise RuntimeError(f"Tinker chassis ballast has no authored mass: {prim_path}")
        try:
            target_mass, target_inertia = chassis_ballast_target_properties(current_mass)
        except (TypeError, ValueError) as error:
            raise RuntimeError(f"unsupported Tinker chassis ballast at {prim_path}: {error}") from error
        mass_api.GetMassAttr().Set(target_mass)
        mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*target_inertia))
        return target_mass

    def _apply_stub_link_masses(self, usd_path: Path) -> tuple[str, ...]:
        """Author ``STUB_LINK_MASS_KG`` on the URDF's massless frame links before reset.

        Runtime override of the spawned stage, like the chassis ballast and
        the wheel sphere colliders: the artifact USD is untouched.  The link
        set comes from the artifact's own colocated ``robot.urdf`` (links
        with neither ``<inertial>`` nor ``<collision>``), never from a
        hard-coded name list, so a description change reflows automatically.
        Only unauthored masses are corrected -- an authored ``physics:mass``
        is the description speaking and stays authoritative.
        """
        import omni.usd
        from pxr import Usd, UsdPhysics

        urdf_path = usd_path.parent / "robot.urdf"
        if not urdf_path.is_file():
            raise RuntimeError(
                f"stub-link mass correction requires the colocated robot.urdf: {urdf_path}"
            )
        stubs = massless_stub_links(urdf_path.read_bytes())
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath("/World/Tinker")
        if not root.IsValid():
            raise RuntimeError("stub-link mass correction requires the spawned /World/Tinker robot")
        by_name = {
            prim.GetName(): prim
            for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
            if prim.GetName() in set(stubs)
        }
        corrected = []
        for name in stubs:
            prim = by_name.get(name)
            if prim is None or not prim.HasAPI(UsdPhysics.MassAPI):
                # The importer may merge a fixed frame into its parent; a
                # frame that is not a rigid body carries no phantom mass.
                continue
            mass_attr = UsdPhysics.MassAPI(prim).GetMassAttr()
            # An unauthored physics:mass still reads back as the schema's 0.0
            # fallback, so authorship -- not value -- is the discriminator.
            if mass_attr and mass_attr.HasAuthoredValue():
                continue
            UsdPhysics.MassAPI(prim).CreateMassAttr(STUB_LINK_MASS_KG)
            corrected.append(name)
        return tuple(corrected)

    def _apply_wheel_sphere_colliders(self) -> None:
        """Replace each wheel's authored cylinder collider with a sphere before reset.

        Runtime override of the spawned stage, like the chassis ballast: the
        artifact USD is untouched.  The cylinder under ``<wheel>/collisions/
        mesh_0`` is deactivated and ``<wheel>/collisions/sphere`` is authored
        with the cylinder's own radius (``wheel_sphere_radius``).  Fails
        closed on a missing wheel or collider so a renamed artifact cannot
        silently keep the locking line contact.
        """
        import omni.usd
        from pxr import Sdf, Usd, UsdGeom, UsdPhysics

        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath("/World/Tinker")
        if not root.IsValid():
            raise RuntimeError("wheel collider override requires the spawned /World/Tinker robot")
        for link in WHEEL_COLLIDER_LINKS:
            wheel = next(
                (
                    prim
                    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
                    if prim.GetName() == link
                ),
                None,
            )
            if wheel is None:
                raise RuntimeError(f"wheel link missing under /World/Tinker: {link}")
            wheel_path = wheel.GetPath()
            # The importer marks the collision subtree instanceable; editing
            # an instance proxy is an error, so de-instance this wheel only.
            ancestor = wheel
            while ancestor and ancestor.GetPath() != Sdf.Path("/"):
                if ancestor.IsInstance():
                    ancestor.SetInstanceable(False)
                ancestor = ancestor.GetParent()
            for descendant in list(Usd.PrimRange(stage.GetPrimAtPath(wheel_path))):
                if descendant.IsInstance():
                    descendant.SetInstanceable(False)
            collisions = stage.GetPrimAtPath(wheel_path.AppendChild("collisions"))
            cylinder = stage.GetPrimAtPath(collisions.GetPath().AppendPath("mesh_0/cylinder"))
            if not collisions.IsValid() or not cylinder.IsValid() or not cylinder.IsA(UsdGeom.Cylinder):
                raise RuntimeError(f"wheel {link} has no authored cylinder collider under {wheel_path}")
            radius = wheel_sphere_radius(UsdGeom.Cylinder(cylinder).GetRadiusAttr().Get())
            stage.GetPrimAtPath(collisions.GetPath().AppendChild("mesh_0")).SetActive(False)
            sphere = UsdGeom.Sphere.Define(stage, collisions.GetPath().AppendChild("sphere"))
            sphere.CreateRadiusAttr(radius)
            sphere.CreatePurposeAttr(UsdGeom.Tokens.guide)
            UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())

    def _refresh_robot_handles(self) -> bool:
        """Refresh tensors after a standard stop/reset/play lifecycle."""
        if not self._robot.is_initialized or self._robot.root_view is None:
            return False
        identity = id(self._robot.root_view)
        if identity == self._robot_view_identity:
            return True
        # A freshly resolved PhysX view holds no drive targets yet, so the next
        # step must push them even if the buffers are unchanged.
        self._target_write_gate.force_next()
        if self._robot_view_identity is not None:
            # Standard ResetSimulation performs STOP -> PLAY.  Isaac Lab
            # recreates the articulation view on PHYSICS_READY.  Keep ROS
            # simulation time MONOTONIC across that boundary: a backward
            # /clock jump wedges TF caches, Nav2 message filters, and
            # ros2_control's controller-switch machinery in every long-lived
            # stack node (observed 2026-08-27: the second in-stack reset
            # re-zeroed /clock, Nav2 dropped all sensor data as "from the
            # past", the bridge's safety controller switch timed out, and the
            # safety supervisor tore the whole bridge launch down).  The step
            # counter may or may not survive STOP -> PLAY, so anchor the new
            # origin to the elapsed count observed before the boundary
            # instead of re-zeroing.
            count_now = self._sim.get_physics_step_count()
            self._clock_step_origin = count_now - self._clock_elapsed_steps
            self._object_views.clear()
            self._contact_pairs_by_key.clear()
        self.joint_names = tuple(self._robot.data.joint_names)
        self._joint_index = {name: index for index, name in enumerate(self.joint_names)}
        self._wheel_indices = tuple(
            self._joint_index[name]
            for name in sorted(WHEEL_JOINT_NAMES)
            if name in self._joint_index
        )
        self._applied_wheel_velocities = {index: 0.0 for index in self._wheel_indices}
        self._safety_joint_ids = tuple(
            self._joint_index[name]
            for name in (f"joint{index}" for index in range(1, 8))
            if name in self._joint_index
        )
        self._safety_nominal_stiffness = self._read_joint_gain_values(
            "joint_stiffness",
            self.NOMINAL_ARM_STIFFNESS,
        )
        self._safety_nominal_damping = self._read_joint_gain_values(
            "joint_damping",
            self.NOMINAL_ARM_DAMPING,
        )
        self._safety_nominal_effort_limits = self._read_joint_gain_values(
            "joint_effort_limits",
            self.SAFETY_HOLD_EFFORT_LIMIT,
        )
        self._safety_gains_applied = False
        self._velocity_targets = self._torch.zeros_like(self._robot.data.joint_vel)
        self._position_targets = self._robot.data.joint_pos.clone()
        self._effort_targets = self._torch.zeros_like(self._robot.data.joint_vel)
        # Cached once here (only rebuilt when the articulation view changes,
        # not per physics step) so the vectorised wheel slew in step() never
        # pays for a fresh tensor allocation on the hot path.
        self._wheel_index_tensor = self._torch.tensor(
            self._wheel_indices, dtype=self._torch.long, device=self._velocity_targets.device
        )
        if self._safety_stopped:
            self._safety_snapshot = self._position_targets.clone()
        try:
            drive_index = self._joint_index.get("drive_joint")
            limits = self._robot.data.joint_effort_limits
            if drive_index is not None and limits is not None:
                configured = float(limits[0, drive_index].detach().cpu().item())
                if math.isfinite(configured) and configured > 0.0:
                    self._default_gripper_effort_limit = configured
                    self._gripper_effort_limit = configured
        except (AttributeError, IndexError, TypeError, ValueError):
            # Some Isaac Lab versions expose actuator limits only after the first
            # physics update. Keep the configured actuator fallback in that case.
            pass
        self._robot_view_identity = identity
        return True

    @property
    def simulation_time(self) -> float:
        # SimulationContext performs explicit CPU PhysX steps.  With NVIDIA's
        # simulation-control extension loaded, Kit's presentation timeline may
        # remain at its first frame even while physics advances.  Isaac Lab's
        # public monotonic counter is therefore the authoritative /clock.
        # Isaac Lab counts solver steps; with substepping several make up one
        # control step, so the clock advances by physics_dt per counted step.
        steps = max(0, self._sim.get_physics_step_count() - self._clock_step_origin)
        # Remember the elapsed count so a STOP -> PLAY reset can continue the
        # clock from here instead of jumping backwards (_refresh_robot_handles).
        self._clock_elapsed_steps = steps
        return float(steps) * self.physics_dt

    @property
    def physics_frame_index(self) -> int:
        simulation = getattr(self, "_sim", None)
        if simulation is None:
            return 0
        steps = simulation.get_physics_step_count() - getattr(
            self, "_clock_step_origin", 0
        )
        # One frame per control step (the cadence truth is published on), so
        # frame indices stay contiguous under substepping.
        return max(0, int(steps)) // max(1, getattr(self, "physics_substeps", 1))

    @property
    def timeline_end_time(self) -> float:
        return float(self._timeline.get_end_time())

    @property
    def safety_stopped(self) -> bool:
        return self._safety_stopped

    @property
    def gripper_effort_limit(self) -> float:
        return float(self._gripper_effort_limit)

    def set_safety_stop(self, active: bool) -> None:
        """Latch a physical hold target and invalidate all pre-stop commands."""
        if bool(active) == self._safety_stopped:
            # A repeated identical sample must return before it clears the
            # acceleration-limited wheel state (see tests/test_base_velocity_slew.py).
            return
        active = bool(active)
        # A real stop transition must reach PhysX even if the target buffers
        # happen to compare equal to what was last written. A repeated
        # identical sample returned above and is deliberately not a transition.
        self._target_write_gate.force_next()
        if active:
            self._pending_snapshot_id = None
            self._pending_snapshot_commands.clear()
            self._command_snapshot_id = None
            if not self._safety_stopped or self._safety_snapshot is None:
                self._safety_snapshot = self._robot.data.joint_pos.clone()
            self._safety_stopped = True
            self._position_targets = self._safety_snapshot.clone()
            self._velocity_targets.zero_()
            self._effort_targets.zero_()
            self._applied_wheel_velocities = {
                index: 0.0 for index in getattr(self, "_wheel_indices", ())
            }
            return
        self._restore_safety_actuator_gains()
        self._safety_stopped = False
        # Clearing a stop creates a fresh hold target. It must not restore the
        # command buffers that were active before the stop.
        self._position_targets = self._robot.data.joint_pos.clone()
        self._velocity_targets.zero_()
        self._effort_targets.zero_()
        self._safety_snapshot = None

    def _read_joint_gain_values(self, attribute: str, fallback: float) -> tuple[float, ...]:
        """Read the configured arm gains, retaining a deterministic fallback for test doubles."""
        values = getattr(getattr(self._robot, "data", None), attribute, None)
        if values is None or not self._safety_joint_ids:
            return tuple(float(fallback) for _ in self._safety_joint_ids)
        try:
            tensor = self._torch_value(values)
            row = tensor[0] if getattr(tensor, "ndim", 0) > 1 else tensor
            rendered = row.detach().cpu().tolist()
            return tuple(float(rendered[index]) for index in self._safety_joint_ids)
        except (AttributeError, IndexError, TypeError, ValueError):
            return tuple(float(fallback) for _ in self._safety_joint_ids)

    def _write_safety_joint_gain(
        self,
        method_name: str,
        keyword: str,
        values: float | tuple[float, ...],
    ) -> None:
        if not self._safety_joint_ids:
            raise RuntimeError("Tinker articulation lacks the joint1-joint7 safety hold group")
        writer = getattr(self._robot, method_name, None)
        if writer is None:
            raise RuntimeError(f"Isaac articulation API {method_name} is unavailable")
        value: object = values
        if isinstance(values, tuple):
            dtype = getattr(getattr(self._robot, "data", None), "joint_pos", None)
            dtype = getattr(dtype, "dtype", None)
            if not isinstance(dtype, self._torch.dtype):
                dtype = self._torch.float32
            device = getattr(self._robot, "device", "cpu")
            value = self._torch.tensor([list(values)], dtype=dtype, device=device)
        writer(
            **{
                keyword: value,
                "joint_ids": list(self._safety_joint_ids),
                "env_ids": [0],
            }
        )
        self._sync_actuator_model_parameter(
            {"stiffness": "stiffness", "damping": "damping", "limits": "effort_limit"}[
                keyword
            ],
            values,
        )

    def _sync_actuator_model_parameter(
        self, attribute: str, values: float | tuple[float, ...]
    ) -> None:
        """Mirror a PhysX gain/limit write into the owning actuator models.

        Isaac Lab issue #128: the ``write_joint_*_to_sim_index`` writers update
        only the simulator buffers. Each ImplicitActuator keeps its own
        stiffness/damping/effort-limit tensors, which (a) are re-applied on
        reset/reinit and (b) are what ``applied_torque`` telemetry -- the
        published joint effort -- is computed from. Without this mirror the
        hold reported ``20000 * error`` (the nominal arm stiffness) while
        PhysX was actually applying ``600 * error``.
        """
        ids = tuple(self._safety_joint_ids)
        if isinstance(values, tuple):
            per_joint = {joint_id: float(v) for joint_id, v in zip(ids, values)}
        else:
            per_joint = {joint_id: float(values) for joint_id in ids}
        for actuator in getattr(self._robot, "actuators", {}).values():
            names = getattr(actuator, "joint_names", None)
            tensor = getattr(actuator, attribute, None)
            if names is None or not isinstance(tensor, self._torch.Tensor):
                continue
            for local_index, name in enumerate(names):
                joint_id = self._joint_index.get(name)
                if joint_id in per_joint:
                    tensor[:, local_index] = per_joint[joint_id]

    def _write_safety_effort_limit(self, values: float | tuple[float, ...]) -> None:
        self._write_safety_joint_gain(
            "write_joint_effort_limit_to_sim_index",
            "limits",
            values,
        )

    def _safety_effort_limits(self) -> tuple[float, ...]:
        """Return the finite emergency effort ceiling for every arm joint."""
        if not self._safety_joint_ids:
            raise RuntimeError("Tinker articulation lacks the joint1-joint7 safety hold group")
        ceiling = float(self.SAFETY_HOLD_EFFORT_LIMIT)
        if not math.isfinite(ceiling) or ceiling < 0.0:
            raise RuntimeError("safety stop has an invalid finite effort ceiling")
        return tuple(ceiling for _ in self._safety_joint_ids)

    def _safety_gravity_efforts(self) -> Any:
        """Read gravity compensation for arm joints, including base-DoF offset."""
        if not self._safety_joint_ids:
            raise RuntimeError("Tinker articulation lacks the joint1-joint7 safety hold group")
        try:
            gravity_proxy = getattr(self._robot.data, "gravity_compensation_forces")
            gravity = self._torch_value(gravity_proxy)
            base_dofs = getattr(self._robot, "num_base_dofs")
            if isinstance(base_dofs, bool) or not isinstance(base_dofs, int):
                raise ValueError(f"invalid num_base_dofs: {base_dofs!r}")
            if base_dofs < 0:
                raise ValueError(f"invalid num_base_dofs: {base_dofs!r}")
            expected_shape = (
                int(self._position_targets.shape[0]),
                base_dofs + len(self.joint_names),
            )
            actual_shape = tuple(int(item) for item in gravity.shape)
            if actual_shape != expected_shape:
                raise ValueError(
                    "gravity compensation shape "
                    f"{actual_shape} does not match {expected_shape}"
                )
            gravity_ids = [base_dofs + joint_id for joint_id in self._safety_joint_ids]
            if any(joint_id >= actual_shape[1] for joint_id in gravity_ids):
                raise ValueError("gravity compensation is missing an arm joint column")
            arm_gravity = gravity[:, gravity_ids]
            arm_gravity = arm_gravity.to(
                dtype=self._position_targets.dtype,
                device=self._position_targets.device,
            )
            if not bool(self._torch.isfinite(arm_gravity).all().item()):
                raise ValueError("gravity compensation contains non-finite values")
            return arm_gravity
        except Exception as error:
            raise RuntimeError(
                "safety stop requires finite gravity compensation for every arm joint"
            ) from error

    def _compute_safety_efforts(self) -> Any:
        """Gravity feed-forward for the arm hold, bounded by the hold ceiling.

        The position/velocity correction is PhysX's drive (stiffness
        ``SAFETY_HOLD_STIFFNESS``, damping ``SAFETY_HOLD_DAMPING`` at the
        latched pose, effort limit ``SAFETY_HOLD_EFFORT_LIMIT``); only the
        gravity term is fed forward as an explicit effort.
        """
        if self._safety_snapshot is None:
            raise RuntimeError("safety stop has no latched measured position")
        joint_ids = list(self._safety_joint_ids)
        raw_effort = self._safety_gravity_efforts()
        limits = self._torch.tensor(
            [self._safety_effort_limits()],
            dtype=raw_effort.dtype,
            device=raw_effort.device,
        )
        if not bool(self._torch.isfinite(limits).all().item()):
            raise RuntimeError("safety stop has non-finite nominal arm effort limits")
        effort = self._torch.minimum(
            self._torch.maximum(raw_effort, -limits),
            limits,
        )
        if not bool(self._torch.isfinite(effort).all().item()):
            raise RuntimeError("safety stop produced non-finite arm effort")
        result = self._torch.zeros_like(self._effort_targets)
        result[:, joint_ids] = effort
        return result

    def _apply_safety_actuator_hold(self) -> None:
        """Hold the arm with PhysX's drive at the latched pose.

        On hold entry (and again after the articulation view was re-resolved,
        which clears ``_safety_gains_applied``) the arm drive is configured
        once: effort ceiling ``SAFETY_HOLD_EFFORT_LIMIT``, stiffness
        ``SAFETY_HOLD_STIFFNESS`` and damping ``SAFETY_HOLD_DAMPING``. With the
        position target latched and the velocity target zero, PhysX then
        applies ``clip(k_p (q_hold - q) - k_d qdot + g, +-ceiling)`` at every
        solver substep -- the same law the former Python hold evaluated at
        the control rate, minus its limit cycle. The gravity feed-forward
        ``g`` is written on entry and refreshed every
        ``SAFETY_HOLD_GRAVITY_REFRESH_STEPS`` control steps; between refreshes
        nothing is pushed (the write gate sees unchanged targets).
        """
        if not self._safety_gains_applied:
            self._write_safety_effort_limit(self._safety_effort_limits())
            self._write_safety_joint_gain(
                "write_joint_stiffness_to_sim_index",
                "stiffness",
                float(self.SAFETY_HOLD_STIFFNESS),
            )
            self._write_safety_joint_gain(
                "write_joint_damping_to_sim_index",
                "damping",
                float(self.SAFETY_HOLD_DAMPING),
            )
            self._safety_gains_applied = True
            self._safety_hold_steps = 0
        if self._safety_hold_steps % self.SAFETY_HOLD_GRAVITY_REFRESH_STEPS == 0:
            self._safety_hold_effort = self._compute_safety_efforts()
        # step() zeroes the effort buffer before the hold runs; re-assert the
        # cached feed-forward so the gate sees an unchanged target between
        # refreshes and nothing is pushed.
        self._effort_targets.copy_(self._safety_hold_effort)
        self._safety_hold_steps += 1

    def _restore_safety_actuator_gains(self) -> None:
        if not getattr(self, "_safety_gains_applied", False):
            return
        self._write_safety_effort_limit(
            self._safety_nominal_effort_limits
            or tuple(self.SAFETY_HOLD_EFFORT_LIMIT for _ in self._safety_joint_ids)
        )
        self._write_safety_joint_gain(
            "write_joint_stiffness_to_sim_index",
            "stiffness",
            self._safety_nominal_stiffness
            or tuple(self.NOMINAL_ARM_STIFFNESS for _ in self._safety_joint_ids),
        )
        self._write_safety_joint_gain(
            "write_joint_damping_to_sim_index",
            "damping",
            self._safety_nominal_damping
            or tuple(self.NOMINAL_ARM_DAMPING for _ in self._safety_joint_ids),
        )
        self._safety_gains_applied = False

    def _set_gripper_effort_limit(self, requested: float) -> None:
        if not math.isfinite(requested) or requested < 0.0:
            raise ValueError("drive_joint effort limit must be finite and non-negative")
        limit = (
            self._default_gripper_effort_limit
            if requested == 0.0
            else min(requested, self._default_gripper_effort_limit)
        )
        index = self._joint_index.get("drive_joint")
        if index is None:
            raise RuntimeError("Tinker articulation lacks drive_joint")
        if limit == self._gripper_effort_limit and getattr(
            self, "_gripper_effort_limit_written", False
        ):
            # The bridge re-sends the gripper packet on every 150 Hz tick.  The
            # PhysX effort-limit write plus the actuator-model mirror cost on
            # the order of a physics step each, so only repeat them when the
            # ceiling actually changes.  Safety hold only rewrites joint1-7.
            return
        self.gripper_effort_limit_writes = getattr(self, "gripper_effort_limit_writes", 0) + 1
        dtype = getattr(self._robot.data.joint_pos, "dtype", None)
        if not isinstance(dtype, self._torch.dtype):
            dtype = self._torch.float32
        limits = self._torch.tensor([[limit]], dtype=dtype, device=self._robot.device)
        writer = getattr(self._robot, "write_joint_effort_limit_to_sim_index", None)
        if writer is None:
            raise RuntimeError("Isaac Lab runtime effort-limit API is unavailable")
        writer(limits=limits, joint_ids=[index], env_ids=[0])
        # Isaac Lab issue #128: the PhysX/shared effort-limit writer updates only
        # the simulator buffer; the owning ImplicitActuator keeps its own
        # effort_limit tensor that is re-applied on reset/reinit.  Keep that
        # model entry in sync using the actuator's own joint-name order, updating
        # every environment at the local drive_joint index only.
        for actuator in getattr(self._robot, "actuators", {}).values():
            names = getattr(actuator, "joint_names", None)
            if names is None:
                continue
            try:
                local_index = list(names).index("drive_joint")
            except (TypeError, ValueError):
                continue
            model_limits = getattr(actuator, "effort_limit", None)
            if model_limits is None or not isinstance(model_limits, self._torch.Tensor):
                continue
            model_limits[:, local_index] = limit
        self._gripper_effort_limit = limit
        self._gripper_effort_limit_written = True

    def command_joints(self, command: JointCommand) -> bool:
        if self._safety_stopped:
            return False
        try:
            self._validate_backend_command(command)
        except Exception:
            if self._pending_snapshot_id is not None:
                self._pending_snapshot_id = None
                self._pending_snapshot_commands.clear()
            raise
        if self._pending_snapshot_id is not None:
            self._pending_snapshot_commands.append(command)
            if self._pending_snapshot_index < self._pending_snapshot_count:
                return True
            pending = tuple(self._pending_snapshot_commands)
            try:
                for staged in pending:
                    self._validate_backend_command(staged)
            except Exception:
                self._pending_snapshot_id = None
                self._pending_snapshot_commands.clear()
                raise
            self._velocity_targets.zero_()
            self._effort_targets.zero_()
            for staged in pending:
                self._apply_joint_command(staged)
            self._command_snapshot_id = self._pending_snapshot_id
            self._pending_snapshot_id = None
            self._pending_snapshot_commands.clear()
            return True
        self._apply_joint_command(command)
        return True

    def _validate_backend_command(self, command: JointCommand) -> None:
        command.validate()
        unknown = set(command.names) - set(self._joint_index)
        if unknown:
            raise ValueError(f"Isaac articulation lacks commanded joints: {sorted(unknown)}")
        if command.efforts and "drive_joint" in command.names:
            effort = command.efforts[command.names.index("drive_joint")]
            if effort < 0.0:
                raise ValueError("drive_joint effort limit must be non-negative")

    def _apply_joint_command(self, command: JointCommand) -> None:
        # Gather in Python, then write each target tensor once.  Every torch
        # element write releases the GIL; under a live bridge the gateway's
        # executor thread takes it each time, and a 7-joint packet applied
        # element-wise was measured at ~0.9 ms (vs ~0.03 ms uncontended).
        # Joint names within a packet are unique (JointCommand.validate), so
        # the gathered index lists never repeat an index.
        position_index: list[int] = []
        position_values: list[float] = []
        velocity_index: list[int] = []
        velocity_values: list[float] = []
        for offset, name in enumerate(command.names):
            index = self._joint_index[name]
            if command.positions and math.isfinite(command.positions[offset]):
                position_index.append(index)
                position_values.append(command.positions[offset])
                if not command.velocities:
                    # A position-only packet takes ownership of this joint's
                    # control mode and must retire an older velocity target.
                    velocity_index.append(index)
                    velocity_values.append(0.0)
            if command.velocities and math.isfinite(command.velocities[offset]):
                velocity_index.append(index)
                velocity_values.append(command.velocities[offset])
            if command.efforts and name == "drive_joint":
                self._set_gripper_effort_limit(command.efforts[offset])
        if position_index:
            self._position_targets[0, position_index] = self._torch.tensor(
                position_values,
                dtype=self._position_targets.dtype,
                device=self._position_targets.device,
            )
        if velocity_index:
            self._velocity_targets[0, velocity_index] = self._torch.tensor(
                velocity_values,
                dtype=self._velocity_targets.dtype,
                device=self._velocity_targets.device,
            )

    def discard_command_snapshot_staging(self) -> None:
        """Drop a partially staged snapshot without touching physical state.

        `set_safety_stop(True)` also clears staging, but it early-returns when
        the stop is already active so that a repeated identical sample cannot
        wipe the acceleration-limited wheel state (see
        tests/test_base_velocity_slew.py). The gateway needs a reset that runs
        unconditionally -- it replays the same packet list twice around its
        preflight pass, and strict packet ordering refuses the second pass
        otherwise. This is bookkeeping only: command buffers, the applied
        wheel velocities, and the last completed snapshot ID (anti-replay) are
        all left alone.
        """
        self._pending_snapshot_id = None
        self._pending_snapshot_count = 0
        self._pending_snapshot_index = 0
        self._pending_snapshot_commands = []

    def begin_command_snapshot(self, snapshot_id: int) -> None:
        """Start a complete mux snapshot, staging multi-packet snapshots."""
        if (
            isinstance(snapshot_id, bool)
            or not isinstance(snapshot_id, int)
            or snapshot_id < 0
        ):
            raise ValueError("command snapshot ID must be a non-negative integer")
        logical_id, packet_count, packet_index = decode_snapshot_packet(snapshot_id)
        current = getattr(self, "_command_snapshot_id", None)
        if current is not None and logical_id < current:
            raise ValueError(
                f"command snapshot {logical_id} is older than {current}"
            )
        if packet_count == 1:
            if self._pending_snapshot_id is not None:
                self._pending_snapshot_id = None
                self._pending_snapshot_commands.clear()
            if current == logical_id:
                return
            self._velocity_targets.zero_()
            self._effort_targets.zero_()
            self._command_snapshot_id = logical_id
            return
        if self._pending_snapshot_id is None:
            if current == logical_id:
                raise ValueError("command snapshot is already complete")
            self._pending_snapshot_id = logical_id
            self._pending_snapshot_count = packet_count
            self._pending_snapshot_index = 0
            self._pending_snapshot_commands = []
        elif self._pending_snapshot_id != logical_id:
            self._pending_snapshot_id = logical_id
            self._pending_snapshot_count = packet_count
            self._pending_snapshot_index = 0
            self._pending_snapshot_commands = []
        elif packet_count != self._pending_snapshot_count:
            raise ValueError("command snapshot packet count changed mid-snapshot")
        expected = self._pending_snapshot_index + 1
        if packet_index != expected:
            raise ValueError(
                f"expected command snapshot packet {expected}, got {packet_index}"
            )
        self._pending_snapshot_index = packet_index

    def _slew_wheel_targets(self) -> None:
        """Vectorised replacement for the former per-wheel scalar slew loop.

        Applies ``applied_next = applied + clip(target - applied, -max_delta,
        +max_delta)`` to every wheel joint in one torch expression instead of
        pulling each wheel's target out to a Python float and writing a
        scalar back (the old loop paid that tensor<->float round trip four
        times per step). This is exactly `slew_velocity_target` per wheel --
        see tests/test_wheel_slew_vectorisation.py.
        """
        wheel_index = self._wheel_index_tensor
        if wheel_index.numel() == 0:
            return
        max_delta = WHEEL_VELOCITY_SLEW_RAD_S2 * self.dt
        commanded = self._velocity_targets[0, wheel_index]
        if not bool(self._torch.isfinite(commanded).all()):
            raise ValueError("wheel velocity targets must be finite")
        applied = self._torch.tensor(
            [self._applied_wheel_velocities[index] for index in self._wheel_indices],
            dtype=commanded.dtype,
            device=commanded.device,
        )
        updated = applied + self._torch.clamp(commanded - applied, -max_delta, max_delta)
        self._velocity_targets[0, wheel_index] = updated
        for index, value in zip(self._wheel_indices, updated.tolist()):
            self._applied_wheel_velocities[index] = value

    def _apply_base_hold(self) -> None:
        """Kinematic braked-base hold (TINKER_SIM_FIX_BASE=1).

        Latch the settled root pose once, then re-write it with zero twist
        every step: to the arm this is a braked base, while the articulation
        stays floating-base so the solver problem (and the mm-accurate joint
        tracking it gives) is unchanged. fix_root_link is NOT equivalent --
        converting to a fixed-base articulation made the arm joints
        stick-slip (recover-8 probes 0/6).
        """
        data = self._robot.data
        if self._base_hold_pose is None:
            if self.simulation_time < self._base_hold_after_sim_s:
                return
            self._base_hold_pose = self._torch.cat(
                [data.root_pos_w[:1].detach().clone(),
                 data.root_quat_w[:1].detach().clone()],
                dim=-1,
            )
            self._base_hold_vel = self._torch.zeros(
                (1, 6),
                dtype=self._base_hold_pose.dtype,
                device=self._base_hold_pose.device,
            )
            self._base_hold_scene_sig = self._scenario_child_signature()
            print(
                json.dumps(
                    {"base_hold_latched": [round(float(v), 4)
                                           for v in self._base_hold_pose[0]]},
                    sort_keys=True,
                ),
                flush=True,
            )
        # A spawn/delete rebuilds the shared physics view; a root write across
        # that rebuild aliases onto the freshly inserted body (Task #13). On
        # any Scenario child-set change, pause writes for a resettle window so
        # the write never lands on the wrong body -- the base free-drifts for
        # that ~0.15 s, negligible next to the 5-17 deg/pick it fixes.
        sig = self._scenario_child_signature()
        if sig != self._base_hold_scene_sig:
            self._base_hold_scene_sig = sig
            self._base_hold_skip_until_sim_s = (
                self.simulation_time + self._base_hold_resettle_s
            )
            # Log the robot's own read-back root pose at the change: if it now
            # reads far from the latched pose, the articulation view has gone
            # stale (loose-body insertion fires no invalidation callback), which
            # is the only failure mode the isaaclab write path admits.
            print(
                json.dumps(
                    {
                        "base_hold_scene_change": sig,
                        "robot_readback_xyz": [
                            round(float(v), 4)
                            for v in data.root_pos_w[0][:3].detach().cpu()
                        ],
                        "latched_xyz": [
                            round(float(v), 4) for v in self._base_hold_pose[0][:3]
                        ],
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            return
        if self.simulation_time < self._base_hold_skip_until_sim_s:
            return
        if self._base_hold_dryrun:
            return
        self._robot.write_root_pose_to_sim_index(root_pose=self._base_hold_pose)
        self._robot.write_root_velocity_to_sim_index(root_velocity=self._base_hold_vel)

    def _scenario_child_signature(self) -> int:
        """Count /World/Scenario children -- a per-step spawn/delete signal for
        the base hold. Cheap (one stage child listing) and only called while
        TINKER_SIM_FIX_BASE is active."""
        try:
            import omni.usd
            stage = omni.usd.get_context().get_stage()
            scenario = stage.GetPrimAtPath("/World/Scenario")
            if not scenario.IsValid():
                return 0
            return len(scenario.GetChildren())
        except (AttributeError, ImportError, RuntimeError, TypeError):
            return self._base_hold_scene_sig or 0

    def step(self) -> None:
        if self.step_profile["enabled"]:
            self.step_profile["_mark"] = self.step_profile["_clock"]()
        if not self._refresh_robot_handles():
            # PHYSICS_READY is dispatched on the first update after a standard
            # STOP -> PLAY transition.  Advance Kit once without touching an
            # invalid tensor view, then refresh the articulation buffers.
            self._sim.step(render=self.render)
            if self._refresh_robot_handles():
                self._robot.update(self.dt)
            return
        if self._safety_stopped:
            if self._safety_snapshot is None:
                self._safety_snapshot = self._robot.data.joint_pos.clone()
            # Reassert the latched target and retire any buffer mutation that
            # could have arrived after the command epoch was stopped.  These
            # are ordinary articulation targets, not a state write.
            self._position_targets.copy_(self._safety_snapshot)
            self._velocity_targets.zero_()
            self._effort_targets.zero_()
            self._apply_safety_actuator_hold()
        else:
            self._slew_wheel_targets()
        if getattr(self, "base_fixed", False):
            self._apply_base_hold()
        # Physics runs at 120 Hz while commands arrive far slower, so most
        # steps would re-send byte-identical targets. PhysX drive targets
        # persist until changed and this backend uses implicit (stateless)
        # actuators with no external wrenches, so skipping an unchanged write
        # is semantically identical -- and it was 6.8 ms of a 12.2 ms step.
        _write_targets = self._target_write_gate.should_write(
            (self._position_targets, self._velocity_targets, self._effort_targets)
        )
        effort_writer = getattr(self._robot, "set_joint_effort_target", None)
        if effort_writer is None and self._safety_stopped:
            raise RuntimeError(
                "safety stop requires Isaac Lab set_joint_effort_target"
            )
        if _write_targets:
            if self.step_profile["enabled"]:
                self._profile_changed_targets()
            self._robot.set_joint_position_target(self._position_targets)
            self._robot.set_joint_velocity_target(self._velocity_targets)
        _sp = self.step_profile
        _t = _sp["_clock"] if _sp["enabled"] else None
        if _t is not None:
            _sp["targets"] += _t() - _sp["_mark"]
            _sp["_mark"] = _t()
        if _write_targets:
            if effort_writer is not None:
                effort_writer(self._effort_targets)
            try:
                self._robot.write_data_to_sim()
            except Exception as error:
                if self._maybe_recover_simulation_view(error):
                    return
                raise RuntimeError(
                    "articulation tensor view failed "
                    f"(time={self.simulation_time:.6f}, end={self.timeline_end_time:.6f}, "
                    f"playing={self._timeline.is_playing()}, "
                    f"initialized={self._robot.is_initialized})"
                ) from error
            self._target_write_gate.note_written(
                (
                    self._position_targets.clone(),
                    self._velocity_targets.clone(),
                    self._effort_targets.clone(),
                )
            )
            self.step_profile["target_writes"] += 1
        if _t is not None:
            _sp["write_data"] += _t() - _sp["_mark"]
            _sp["_mark"] = _t()
        self._step_simulation()
        if _t is not None:
            _sp["physx"] += _t() - _sp["_mark"]
            _sp["_mark"] = _t()
        try:
            self._robot.update(self.dt)
        except Exception as error:
            if not self._maybe_recover_simulation_view(error):
                raise
            return
        if _t is not None:
            _sp["robot_update"] += _t() - _sp["_mark"]
            _sp["_mark"] = _t()
        self._refresh_object_views()
        if _t is not None:
            _sp["object_views"] += _t() - _sp["_mark"]
            _sp["n"] += 1
        # getattr defaults: test doubles construct via object.__new__ and
        # call step() without running __init__.
        if getattr(self, "_tracked_object_paths", ()):
            self._log_tracked_objects()
        if getattr(self, "_heal_detached_spawns", False):
            self._heal_detached_scenario_bodies()

    def _maybe_recover_simulation_view(self, error: Exception) -> bool:
        """Rebuild the tensor simulation view after a topology invalidation.

        Deleting a prim any live tensor view covers invalidates the SHARED
        SimulationView ("prim ... was deleted while being used by a tensor
        view class"); the physics SCENE itself keeps simulating -- only the
        view layer dies -- but every ``Articulation.update`` read then
        raises ("Failed to get DOF velocities from backend") and, without
        this, the boot is unrecoverable (observed 2026-08-31 twice: the
        healer's probe views, and the referee's cached write view, each
        made a routine post-run clear fatal). The views have no release
        API and even a dropped, garbage-collected Python view leaves the
        backend registration live (measured), so avoidance alone cannot
        protect against every component.

        Recovery mirrors the manager's own STOP -> PLAY view lifecycle
        minus the timeline (and minus the warmup ``force_load``, which
        would re-parse the stage and snap every body back to its authored
        pose): invalidate + clear the views, recreate them against the
        still-live scene, and let the re-dispatched PHYSICS_READY rebind
        the articulation -- ``_refresh_robot_handles`` already handles the
        new root-view identity on the next step. Budgeted to 5 recoveries
        per boot so a genuinely broken scene still fails loudly.
        """
        message = str(error)
        if not any(
            token in message
            for token in ("Failed to get", "Failed to set", "invalidated", "Simulation view")
        ):
            return False
        budget_used = getattr(self, "_view_recoveries", 0)
        if budget_used >= 5:
            return False
        self._view_recoveries = budget_used + 1
        try:
            from isaaclab_physx.physics import PhysxManager

            # De-initialize the articulation first: its PHYSICS_READY
            # handler early-returns while it believes itself initialized
            # (measured: without this, view recreation "succeeds" but every
            # read keeps failing against the stale articulation view).
            # This is exactly what the timeline-stop callback does.
            self._robot._invalidate_initialize_callback(None)
            # Replicate ONLY the view-creation lines of PhysxManager.
            # _warmup_and_create_views: the method itself is unusable here
            # (its first guard early-returns without _warmup_needed, and
            # with it, its warmup force_load_physics_from_usd re-parses the
            # stage and snaps every body back to its authored pose).
            import omni.physics.tensors as _tensors
            from isaaclab.sim.utils.stage import get_current_stage_id

            stage_id = get_current_stage_id()
            PhysxManager._warmup_needed = False
            PhysxManager._invalidate_views()
            fresh_view = _tensors.create_simulation_view("warp", stage_id=stage_id)
            fresh_view_warp = _tensors.create_simulation_view("warp", stage_id=stage_id)
            fresh_view.set_subspace_roots("/")
            fresh_view_warp.set_subspace_roots("/")
            PhysxManager._view = fresh_view
            PhysxManager._view_warp = fresh_view_warp
            PhysxManager._physx.update_simulation(PhysxManager.get_physics_dt(), 0.0)
            PhysxManager._view_created = True
            try:
                PhysxManager._scene_data_backend.simulation_view = PhysxManager._view
            except Exception:
                pass
            # The physics step counter resets with the new views; re-anchor
            # the monotonic clock immediately -- the gateway may publish
            # /clock before the next _refresh_robot_handles, and a single
            # backward /clock sample wedges TF caches and Nav2 stack-wide
            # (the documented 2026-08-27 trap).
            count_now = self._sim.get_physics_step_count()
            self._clock_step_origin = count_now - self._clock_elapsed_steps
            # The articulation's _initialize_impl sources its view from
            # isaacsim's SimulationManager (a separate holder from
            # PhysxManager); rebuild those views too, replicating only the
            # view-creation lines of SimulationManager.initialize_physics --
            # never its warmup, whose force_load_physics_from_usd would
            # re-parse the stage and snap every body to its authored pose.
            import omni.physics.tensors as _tensors
            import omni.usd as _omni_usd
            from isaacsim.core.simulation_manager import SimulationManager

            for attr in ("_physics_sim_view", "_physics_sim_view__warp"):
                stale = getattr(SimulationManager, attr, None)
                if stale is not None:
                    try:
                        stale.invalidate()
                    except Exception:
                        pass
                    setattr(SimulationManager, attr, None)
            stage_id = _omni_usd.get_context().get_stage_id()
            engine = getattr(SimulationManager, "_engine", None)

            def _create_view(frontend: str):
                try:
                    view = _tensors.create_simulation_view(
                        frontend, stage_id=stage_id, backend=engine
                    )
                except TypeError:
                    view = _tensors.create_simulation_view(frontend, stage_id=stage_id)
                view.set_subspace_roots("/")
                return view

            SimulationManager._physics_sim_view__warp = _create_view("warp")
            frontend = None
            try:
                frontend = SimulationManager.get_backend()
            except Exception:
                pass
            SimulationManager._physics_sim_view = (
                _create_view(frontend)
                if frontend and frontend != "warp"
                else SimulationManager._physics_sim_view__warp
            )
            SimulationManager._simulation_view_created = True
            if not self._robot.is_initialized:
                # The event-bus init path invokes assets through
                # safe_callback_invoke, which STORES exceptions instead of
                # raising; re-run the initialization directly so a failure
                # lands in this try block and is reported.
                self._robot._initialize_callback(None)
            if not self._robot.is_initialized:
                raise RuntimeError("articulation did not re-initialize after view recreation")
        except Exception as recovery_error:
            print(
                json.dumps(
                    {
                        "simulation_view_recovery": "failed",
                        "error": str(recovery_error)[:300],
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            return False
        # Deliver any deferred PHYSICS_READY work and rebind the articulation
        # before the next target write; a write against the old view raises
        # "articulation tensor view failed".
        try:
            import omni.kit.app

            for _ in range(2):
                omni.kit.app.get_app().update()
            self._refresh_robot_handles()
        except Exception:
            pass
        self._target_write_gate.force_next()
        print(
            json.dumps(
                {
                    "simulation_view_recovery": "ok",
                    "attempt": self._view_recoveries,
                    "t": round(self.simulation_time, 3),
                    "trigger": message[:160],
                },
                sort_keys=True,
            ),
            flush=True,
        )
        return True

    def _heal_detached_scenario_bodies(self) -> None:
        """Re-parse /World/Scenario rigid-body prims PhysX failed to attach.

        Runs at the object-discovery cadence. For each Scenario child prim
        carrying ``UsdPhysics.RigidBodyAPI``: one attachment probe per
        interval; if the body is absent after a full interval since the
        prim appeared, toggle the prim inactive/active (the standard
        omni.physx re-parse trigger) and re-probe next interval. Two nudges
        maximum, then one ``spawn_attach_failed`` JSON line and a permanent
        mark. Attached prims are marked healed on the first successful
        probe and never probed again, so steady-state cost is one stage
        child listing per interval.

        The probe is ``IPhysx.get_rigidbody_transformation`` -- deliberately
        NOT a tensor view: the physics.tensors views have no release API,
        and deleting a prim that any live view covers invalidates the
        SHARED SimulationView, killing the articulation for the rest of the
        boot (observed 2026-08-31: the first battery boot that probed
        spawned objects with views died at the first multi-entity clear --
        "prim ... was deleted while being used by a tensor view class").
        Spawned objects are exactly the prims that later get deleted, so
        nothing that watches them may ever hold a view on them.
        """
        self._spawn_attach_step += 1
        interval = self._object_discovery_interval
        if interval > 1 and self._spawn_attach_step % interval:
            return
        try:
            import omni.usd
            from omni.physx import get_physx_interface
            from pxr import UsdPhysics

            stage = omni.usd.get_context().get_stage()
            scenario = stage.GetPrimAtPath("/World/Scenario")
            if not scenario.IsValid():
                return
            physx = get_physx_interface()
        except (AttributeError, ImportError, RuntimeError, TypeError):
            return
        for child in scenario.GetChildren():
            path = child.GetPath().pathString
            state = self._spawn_attach_watch.get(path)
            if state is None:
                state = {"first_step": self._spawn_attach_step, "attempts": 0, "healed": 0}
                self._spawn_attach_watch[path] = state
                continue  # give a fresh spawn one full interval to attach
            if state["healed"] or state["attempts"] >= 3:
                continue
            if not child.HasAPI(UsdPhysics.RigidBodyAPI):
                state["healed"] = 1  # not a rigid body (actor, prop); ignore
                continue
            attached = False
            try:
                result = physx.get_rigidbody_transformation(path)
                attached = bool(result.get("ret_val"))
            except (AttributeError, RuntimeError, TypeError):
                attached = False
            if attached:
                if state["attempts"]:
                    print(
                        json.dumps(
                            {
                                "spawn_attach_healed": path,
                                "nudges": state["attempts"],
                                "t": round(self.simulation_time, 3),
                            },
                            sort_keys=True,
                        ),
                        flush=True,
                    )
                state["healed"] = 1
                continue
            state["attempts"] += 1
            if state["attempts"] >= 3:
                print(
                    json.dumps(
                        {
                            "spawn_attach_failed": path,
                            "t": round(self.simulation_time, 3),
                            "hint": (
                                "PhysX never attached a rigid body for this "
                                "spawned prim and two active-toggle re-parse "
                                "nudges did not help"
                            ),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                continue
            try:
                child.SetActive(False)
                child.SetActive(True)
            except Exception as error:
                print(
                    json.dumps(
                        {"spawn_attach_nudge_error": path, "error": str(error)},
                        sort_keys=True,
                    ),
                    flush=True,
                )

    def _log_tracked_objects(self) -> None:
        """Print tracked rigid-body world poses (TINKER_SIM_TRACK_OBJECTS).

        Reads through ``IPhysx.get_rigidbody_transformation`` -- view-free
        on purpose: tracked objects are typically spawned objects that
        later get deleted, and a tensor view held (or ever created -- the
        views have no release API) on a deleted prim invalidates the shared
        SimulationView and kills the boot. See
        ``_heal_detached_scenario_bodies``.
        """
        self._tracked_object_step += 1
        interval = max(1, int(0.25 * self.control_hz))
        if self._tracked_object_step % interval:
            return
        try:
            from omni.physx import get_physx_interface

            physx = get_physx_interface()
        except (AttributeError, ImportError, RuntimeError):
            return
        for path in self._tracked_object_paths:
            try:
                result = physx.get_rigidbody_transformation(path)
            except (AttributeError, RuntimeError, TypeError) as error:
                print(
                    json.dumps(
                        {
                            "tracked_object": path,
                            "t": round(self.simulation_time, 3),
                            "state": f"pose query failed: {error}",
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                continue
            if not result.get("ret_val"):
                print(
                    json.dumps(
                        {
                            "tracked_object": path,
                            "t": round(self.simulation_time, 3),
                            "state": "no rigid body",
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                continue
            position = result.get("position")
            print(
                json.dumps(
                    {
                        "tracked_object": path,
                        "t": round(self.simulation_time, 3),
                        "xyz": [round(float(v), 4) for v in tuple(position)[:3]],
                    },
                    sort_keys=True,
                ),
                flush=True,
            )

    def _profile_changed_targets(self) -> None:
        """Profile-only: count which joints' targets differ from the last push."""
        last = getattr(self, "_profile_last_pushed", None)
        current = (
            self._position_targets.clone(),
            self._velocity_targets.clone(),
            self._effort_targets.clone(),
        )
        self._profile_last_pushed = current
        if last is None:
            return
        hist = self.step_profile.setdefault("changed_targets", {})
        names = {index: name for name, index in self._joint_index.items()}
        for label, before, after in zip(("pos", "vel", "eff"), last, current):
            diff = (before != after)[0].nonzero().flatten().tolist()
            for index in diff:
                key = f"{label}:{names.get(index, index)}"
                hist[key] = hist.get(key, 0) + 1

    def step_profile_snapshot(self) -> dict:
        """Return per-step wall-time attribution (ms) and reset the window.

        Only meaningful when TINKER_SIM_PROFILE=1. Separates the PhysX solve
        from the Isaac Lab / Python work wrapped around it, which is the
        distinction that decides whether a physics backend change (e.g. GPU
        PhysX) could help at all.
        """
        sp = self.step_profile
        n = max(1, sp["n"])
        out = {
            key: round(1000.0 * sp[key] / n, 3)
            for key in ("targets", "write_data", "physx", "robot_update", "object_views")
        }
        out["steps"] = sp["n"]
        out["total_ms"] = round(sum(v for k, v in out.items() if k not in ("steps",)), 3)
        # How many of those steps actually pushed targets to PhysX; the rest
        # were byte-identical re-sends the write gate skipped.
        out["target_writes"] = sp["target_writes"]
        if sp.get("changed_targets"):
            out["changed_targets"] = dict(
                sorted(sp["changed_targets"].items(), key=lambda kv: -kv[1])[:8]
            )
            sp["changed_targets"] = {}
        # PhysX solver steps per control step (1 unless TINKER_SIM_CONTROL_HZ
        # lowered the control rate below physics_hz).
        out["physx_substeps"] = self.physics_substeps
        for key in ("targets", "write_data", "physx", "robot_update", "object_views"):
            sp[key] = 0.0
        sp["target_writes"] = 0
        sp["n"] = 0
        return out

    def render_frame(self) -> None:
        """Render current transforms without issuing another physics step."""
        self._sim.forward()
        self._sim.render()

    def _torch_value(self, value: Any) -> Any:
        """Normalize Isaac proxy, Torch, and NumPy-like values to a tensor."""
        value = getattr(value, "torch", value)
        if not hasattr(value, "detach"):
            tolist = getattr(value, "tolist", None)
            if callable(tolist):
                value = tolist()
            value = self._torch.as_tensor(value)
        return value

    def _refresh_object_views(self) -> None:
        # These are TENSOR VIEWS, held for the truth stream's pose+twist
        # reads. The views have no release API, and deleting a prim any
        # live view covers invalidates the shared SimulationView (whole
        # boot dies -- see _heal_detached_scenario_bodies). That is safe
        # here ONLY because expected (scenario-declared) objects are never
        # deleted on a playing timeline in any current flow; anything that
        # wants to delete one mid-play must first go through a stop
        # boundary. Watchers of DELETABLE spawns must use the view-free
        # IPhysx.get_rigidbody_transformation instead.
        if not self._expected_objects:
            return
        # Every expected object that has already resolved is skipped below, so
        # the only work left is DISCOVERY of the ones that have not -- and the
        # fallback for those is a full stage.Traverse(). Running that on every
        # physics step costs more than the PhysX solve itself: measured
        # 2026-08-20 at 10.4 ms of a 23.7 ms step (44%) with four unresolved
        # scenario objects, against 5.7-6.1 ms for PhysX. Discovery is a
        # startup concern, so retry a few times a second instead. Objects that
        # spawn late are still picked up; only the polling rate changes.
        if all(name in self._object_views for name in self._expected_objects):
            return
        interval = self._object_discovery_interval
        if interval > 1 and self._object_discovery_step % interval != 0:
            self._object_discovery_step += 1
            return
        self._object_discovery_step += 1
        try:
            import omni.usd
            from isaaclab_physx.physics import PhysxManager
            from pxr import UsdPhysics

            stage = omni.usd.get_context().get_stage()
            physics_view = PhysxManager.get_physics_sim_view()
        except (AttributeError, RuntimeError, TypeError):
            # Standard spawn may not have happened yet; retry on the next step.
            return
        for name, descriptor in self._expected_objects.items():
            if name in self._object_views:
                continue
            # A prim that resolved but exposed no rigid body will not grow one
            # until it is respawned (physics APIs ship inside the asset), so
            # back off hard instead of paying a PhysX pattern miss -- three
            # logged errors per attempt, measured 16,946 error lines per
            # session -- twice a second forever.  The slow retry still picks
            # up a delete + respawn with a fixed asset.
            failed_at = descriptor.get("_rigid_body_missing_step")
            if (
                isinstance(failed_at, int)
                and self._object_discovery_step - failed_at < 20 * interval
            ):
                continue
            # One object's failure must never hide its siblings: everything
            # per-object stays inside this per-object try.
            try:
                candidates = [
                    str(descriptor.get("prim_path", "")),
                    f"/World/{name}",
                    f"/World/Scenario/{name}",
                ]
                prim_path = next(
                    (
                        candidate
                        for candidate in candidates
                        if candidate and stage.GetPrimAtPath(candidate).IsValid()
                    ),
                    None,
                )
                if prim_path is None:
                    for prim in stage.Traverse():
                        if prim.GetName() == name and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                            prim_path = prim.GetPath().pathString
                            break
                if prim_path is None:
                    continue
            except (AttributeError, RuntimeError, TypeError):
                continue
            try:
                view = physics_view.create_rigid_body_view(prim_path)
                count = int(view.count) if view is not None else 0
            except (AttributeError, RuntimeError, TypeError):
                # The prim exists but PhysX matched no rigid body under it
                # (this build raises instead of returning an empty view).
                view = None
                count = 0
            if count > 0:
                self._object_views[name] = view
                descriptor.pop("_rigid_body_missing_step", None)
                descriptor["actual_prim_path"] = prim_path
                continue
            descriptor["_rigid_body_missing_step"] = self._object_discovery_step
            if not descriptor.get("_rigid_body_missing_logged"):
                descriptor["_rigid_body_missing_logged"] = True
                print(
                    json.dumps(
                        {
                            "object_discovery": "rigid_body_missing",
                            "object": name,
                            "prim_path": prim_path,
                            "hint": (
                                "prim exists but PhysX sees no rigid body; "
                                "the spawned asset lacks UsdPhysics.RigidBodyAPI"
                            ),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )

    def _actual_object_states(self) -> list[dict[str, object]]:
        states: list[dict[str, object]] = []
        for name, view in self._object_views.items():
            transforms = self._torch_value(view.get_transforms())
            velocities = self._torch_value(view.get_velocities())
            pose = transforms[0].detach().cpu().tolist()
            twist = velocities[0].detach().cpu().tolist()
            descriptor = self._expected_objects.get(name, {})
            states.append(
                {
                    "id": name,
                    "class_name": str(descriptor.get("class_name", "")),
                    "prim_path": str(descriptor.get("actual_prim_path", "")),
                    "pose": {
                        "xyz": [float(value) for value in pose[:3]],
                        "quaternion_xyzw": [float(value) for value in pose[3:7]],
                    },
                    "twist": {
                        "linear": [float(value) for value in twist[:3]],
                        "angular": [float(value) for value in twist[3:6]],
                    },
                }
            )
        return states

    def _robot_truth_state(self) -> dict[str, object]:
        data = self._robot.data
        root_position = self._torch_value(data.root_pos_w)[0].detach().cpu().tolist()
        root_quaternion = self._torch_value(data.root_quat_w)[0].detach().cpu().tolist()
        base_pose = {
            "xyz": [float(value) for value in root_position],
            "quaternion_xyzw": [float(value) for value in root_quaternion],
        }
        tcp_pose = base_pose
        body_names = tuple(getattr(data, "body_names", ()))
        if "link_tcp" in body_names:
            tcp_index = body_names.index("link_tcp")
            body_positions = self._torch_value(data.body_pos_w)[0, tcp_index]
            body_quaternion = self._torch_value(data.body_quat_w)[0, tcp_index]
            tcp_pose = {
                "xyz": [float(value) for value in body_positions.detach().cpu().tolist()],
                "quaternion_xyzw": [
                    float(value) for value in body_quaternion.detach().cpu().tolist()
                ],
            }
        names, positions, velocities, efforts = self.joint_state()
        root_linear = self._torch_value(data.root_lin_vel_w)[0].detach().cpu().tolist()
        root_angular = self._torch_value(data.root_ang_vel_w)[0].detach().cpu().tolist()
        return {
            "base_pose": base_pose,
            "tcp_pose": tcp_pose,
            "base_twist": {
                "linear": [float(value) for value in root_linear],
                "angular": [float(value) for value in root_angular],
            },
            "joint_names": list(names),
            "joint_positions": positions,
            "joint_velocities": velocities,
            "joint_efforts": efforts,
            "safety_stop": self.safety_stopped,
        }

    def joint_state(self) -> tuple[tuple[str, ...], list[float], list[float], list[float]]:
        data = self._robot.data
        return (
            self.joint_names,
            self._torch_value(data.joint_pos)[0].detach().cpu().tolist(),
            self._torch_value(data.joint_vel)[0].detach().cpu().tolist(),
            self._torch_value(data.applied_torque)[0].detach().cpu().tolist(),
        )

    def command_target_state(self) -> dict[str, object]:
        """Return the targets actually held by the PhysX articulation."""
        return {
            "joint_names": list(self.joint_names),
            "joint_positions": [
                float(value)
                for value in self._torch_value(self._position_targets)[0]
                .detach()
                .cpu()
                .tolist()
            ],
            "joint_velocities": [
                float(value)
                for value in self._torch_value(self._velocity_targets)[0]
                .detach()
                .cpu()
                .tolist()
            ],
            "joint_efforts": [
                float(value)
                for value in self._torch_value(self._effort_targets)[0]
                .detach()
                .cpu()
                .tolist()
            ],
            "snapshot_id": getattr(self, "_command_snapshot_id", None),
            "gripper_effort_limit": self.gripper_effort_limit,
        }

    def root_state(self) -> dict[str, tuple[float, ...]]:
        data = self._robot.data
        quaternion_xyzw = tuple(
            float(value)
            for value in self._torch_value(data.root_quat_w)[0].detach().cpu()
        )
        return {
            "position": tuple(
                float(value) for value in self._torch_value(data.root_pos_w)[0].detach().cpu()
            ),
            "quaternion_wxyz": (
                quaternion_xyzw[3],
                quaternion_xyzw[0],
                quaternion_xyzw[1],
                quaternion_xyzw[2],
            ),
            "linear_velocity_world": tuple(
                float(value)
                for value in self._torch_value(data.root_lin_vel_w)[0].detach().cpu()
            ),
            "angular_velocity_world": tuple(
                float(value)
                for value in self._torch_value(data.root_ang_vel_w)[0].detach().cpu()
            ),
        }

    def contact_state(self) -> dict[str, dict[str, float | bool]]:
        state: dict[str, dict[str, float | bool]] = {
            name: {"in_contact": False, "force": 0.0}
            for name in self.ARM_CONTACT_BODIES + self.GRASP_CONTACT_BODIES
        }
        for pair in self.contact_pairs():
            force = float(pair["normal_force"])
            for body in (str(pair["body_a"]), str(pair["body_b"])):
                prefix = "/World/Tinker/"
                if not body.startswith(prefix):
                    continue
                name = body.removeprefix(prefix)
                if name not in state:
                    continue
                state[name]["force"] = float(state[name]["force"]) + force
                state[name]["in_contact"] = True
        return state

    def contact_pairs(self) -> list[dict[str, object]]:
        """Return active PhysX reports with both rigid-body identities."""
        return [dict(pair) for pair in self._contact_pairs_by_key.values()]

    @staticmethod
    def _contact_vector(value: object) -> list[float]:
        return [float(value[index]) for index in range(3)]  # type: ignore[index]

    def _on_contact_report_event(
        self, contact_headers: Iterable[object], contact_data: object
    ) -> None:
        monitored = {
            f"/World/Tinker/{name}"
            for name in self.ARM_CONTACT_BODIES + self.GRASP_CONTACT_BODIES
        }
        for header in contact_headers:
            actor_ids = (int(header.actor0), int(header.actor1))  # type: ignore[attr-defined]
            collider_ids = (
                int(header.collider0),  # type: ignore[attr-defined]
                int(header.collider1),  # type: ignore[attr-defined]
            )
            key = actor_ids + collider_ids
            event_type = header.type  # type: ignore[attr-defined]
            if event_type == self._contact_event_lost:
                self._contact_pairs_by_key.pop(key, None)
                continue
            if event_type not in {
                self._contact_event_found,
                self._contact_event_persist,
            }:
                continue
            actors = tuple(self._contact_path_decoder(path_id) for path_id in actor_ids)
            if not monitored.intersection(actors):
                continue
            offset = int(header.contact_data_offset)  # type: ignore[attr-defined]
            count = int(header.num_contact_data)  # type: ignore[attr-defined]
            samples = [contact_data[index] for index in range(offset, offset + count)]  # type: ignore[index]
            if not samples:
                self._contact_pairs_by_key.pop(key, None)
                continue
            normal_impulses: list[tuple[float, list[float], int]] = []
            for sample_index, sample in enumerate(samples):
                impulse = self._contact_vector(sample.impulse)
                reported_normal = self._contact_vector(sample.normal)
                normal_length = math.sqrt(sum(value * value for value in reported_normal))
                if not math.isfinite(normal_length) or normal_length <= 0.0:
                    continue
                normal = [value / normal_length for value in reported_normal]
                projected_impulse = sum(
                    impulse_value * normal_value
                    for impulse_value, normal_value in zip(impulse, normal)
                )
                if not math.isfinite(projected_impulse):
                    continue
                normal_impulses.append((abs(projected_impulse), normal, sample_index))

            normal_force = sum(value[0] for value in normal_impulses) / self.dt
            if normal_force <= self.CONTACT_FORCE_THRESHOLD:
                self._contact_pairs_by_key.pop(key, None)
                continue
            points = [self._contact_vector(sample.position) for sample in samples]
            point = [sum(values) / len(values) for values in zip(*points)]
            weighted_normal = [
                sum(weight * normal[axis] for weight, normal, _ in normal_impulses)
                for axis in range(3)
            ]
            weighted_normal_length = math.sqrt(
                sum(value * value for value in weighted_normal)
            )
            if weighted_normal_length > 0.0 and math.isfinite(weighted_normal_length):
                normal = [value / weighted_normal_length for value in weighted_normal]
            else:
                # Equal opposing normals can cancel in the weighted average.  The
                # largest contribution, with source order as the tie-breaker, is
                # deterministic and preserves an actual PhysX-reported normal.
                normal = max(normal_impulses, key=lambda item: (item[0], -item[2]))[1]
            self._contact_pairs_by_key[key] = {
                "body_a": actors[0],
                "body_b": actors[1],
                "normal_force": normal_force,
                "point": point,
                "normal": normal,
            }

    @classmethod
    def is_arm_scenario_collision(
        cls, pairs: Iterable[Mapping[str, object]]
    ) -> bool:
        """Classify identified contacts without reading or mutating backend state."""
        arm_paths = {f"/World/Tinker/{name}" for name in cls.ARM_CONTACT_BODIES}
        scenario_prefix = "/World/Scenario/"
        for pair in pairs:
            body_a = str(pair.get("body_a", ""))
            body_b = str(pair.get("body_b", ""))
            if (
                body_a in arm_paths
                and body_b.startswith(scenario_prefix)
            ) or (
                body_b in arm_paths
                and body_a.startswith(scenario_prefix)
            ):
                return True
        return False

    def arm_scenario_collision(self) -> bool:
        return self.is_arm_scenario_collision(self.contact_pairs())

    def parity_state(self) -> Mapping[str, object]:
        names, positions, velocities, efforts = self.joint_state()
        return {
            "joint_names": names,
            "joint_positions": positions,
            "joint_velocities": velocities,
            "joint_efforts": efforts,
        }

    def truth_state(self, evaluator_token: object) -> Mapping[str, object]:
        if evaluator_token is not self.TRUTH_TOKEN:
            raise PermissionError("truth state is evaluator-only")
        actual_objects = self._actual_object_states()
        return {
            "schema_version": self.PHYSICS_TRUTH_SCHEMA_VERSION,
            "frame_index": self.physics_frame_index,
            "timestamp": self.simulation_time,
            "scenario": self.scenario,
            "task": self.task,
            "robot": self._robot_truth_state(),
            "command_targets": self.command_target_state(),
            "physics_device": self.physics_device,
            "chassis_ballast_mass_kg": self.chassis_ballast_mass_kg,
            "seed": self.seed,
            "contacts": self.contact_pairs(),
            "contact_state": self.contact_state(),
            "contact_pairs": self.contact_pairs(),
            "expected_objects": self._expected_objects,
            "objects": actual_objects,
            "object": (
                actual_objects[0] if actual_objects else None
            ),
            "safety_stop": self.safety_stopped,
            "actuator_limits": {"drive_joint": self.gripper_effort_limit},
        }

    def physics_truth_frame(self, evaluator_token: object) -> Mapping[str, object]:
        if evaluator_token is not self.TRUTH_TOKEN:
            raise PermissionError("physics truth is evaluator-only")
        return self.truth_state(evaluator_token)


IsaacNavigationBackend = IsaacWholeRobotBackend
