#!/usr/bin/env python
"""gripper_close_probe.py -- headless, in-process gripper close-phase probe.

No ROS, no cameras, CPU PhysX, one Isaac boot (~20 s) and many trials. This is
the harness that found the Task #19 root cause (developer log 2026-09-02):
it drives IsaacWholeRobotBackend directly, so a close-phase question costs
seconds instead of a full-stack cuMotion pick.

Phase A  free closes (no object), sweeping (slew, follower damping, follower k):
         per-step follower torque, target-pad lag and pad speed -- the press a
         pad carries INTO first contact (k*lag == d*v in steady motion).
Phase B  closes on a bench object (bottle side-grasp or top-down knife pinch)
         standing on a footprint-sized static pedestal at a built-in bench
         grasp pose: first-contact / peak / hold pad force, object displacement
         and tilt, and (--lift) whether the object rises with the TCP.

Run via scripts/gripper-close-probe (ROS-clean env, PROBE_GPU selects the
render card; physics is CPU). Every result line is JSON on stdout AND appended
to --out; per-step rows go to <out>.rows.jsonl. Examples:

  scripts/gripper-close-probe --out /tmp/a.jsonl --phase A
  scripts/gripper-close-probe --out /tmp/b.jsonl --phase B --lift
  scripts/gripper-close-probe --out /tmp/k.jsonl --phase B --lift \
      --pose topdown --object knife --tcp-above-top 0.012

--mirror-mode target|measured|measured_ff monkeypatches the backend's mimic
mirror for A/B (the shipped backend implements measured_ff).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True)
parser.add_argument("--phase", default="AB", help="A, B or AB")
parser.add_argument(
    "--configs",
    default="1.5:55:1500,0.75:55:1500,0.3:55:1500,1.5:20:1500,1.5:5:1500,1.5:55:500",
    help="comma list of slew:damping:stiffness for the mimic followers",
)
parser.add_argument("--grasp-config", default="", help="JSON with close_events[].arm_joints (optional; --pose supplies the built-in bench poses)")
parser.add_argument("--pose", default="side", choices=("side", "topdown"), help="built-in arm pose: side = bench bottle side-grasp (TCP 0.5375 ahead, 0.7446 up, tool z=+x); topdown = knife pinch (tool z=-z), fingertips --tcp-above-top above the pedestal")
parser.add_argument("--arm-joints", default="", help="d1,d2,d3,d4,d5,d6,d7 (DEGREES, xArm7 joint1..joint7 order): instead of --pose/--tcp-xz IK, hold the arm at exactly this configuration and run the normal phase-B close from it (--no-object for a free close). Overrides --pose and --tcp-xz; incompatible with --descend-from (there is no separate staged pose to descend from -- the arm goes straight to the given configuration and settles there); --closing-axis is ignored in this mode (the jaw's closing direction is whatever this configuration puts it at, not a --closing-axis assumption -- it is measured from the finger link positions and printed in the arm_joints event)")
parser.add_argument("--grasp-index", type=int, default=0)
parser.add_argument("--close-target", type=float, default=0.85)
parser.add_argument("--record-s", type=float, default=3.0)
parser.add_argument("--settle-s", type=float, default=1.5)
parser.add_argument(
    "--bottle-usda",
    default="",
)
parser.add_argument("--bottle-offset", default="", help="x,y,z of bottle BASE relative to pad midpoint (phase B fallback)")
parser.add_argument("--lift", action="store_true", help="phase B: after the close, raise the arm and report whether the bottle follows")
parser.add_argument("--mirror-mode", default="target",
                    choices=("target", "measured", "measured_ff", "central"),
                    help="target = stock mirror (followers track drive TARGET); measured = followers track the drive joint's MEASURED angle (single-DOF jaw); measured_ff = measured + one-step velocity feed-forward (q + qdot*dt); central = virtual central command: all six gripper joints track the applied drive target c (symmetric gains + stall-gated lead clamp)")
parser.add_argument("--max-lead", type=float, default=None, help="override backend._gripper_max_lead (0 disables the stall-gated lead clamp)")
parser.add_argument("--stall-speed", type=float, default=None, help="override backend._gripper_stall_speed")
parser.add_argument("--drive-effort-limit", type=float, default=None, help="raise the drive_joint effort ceiling (Nm) to sweep the clamp force; URDF default 50. The bench close is capped at this ceiling, so this is the only way to press past 50 Nm")
parser.add_argument("--arm-stream-hz", type=float, default=0.0, help="inject synthetic JTC-style arm HOLD packets (names joint1..7, positions = the CURRENT measured arm joint angles, velocities = zeros, no gripper joints) into backend.command_joints() at this many packets per second of SIM time, immediately before each backend.step() call during phase B's close (and its --descend-from descent, if any). Mirrors the live stack's ordering: validation/run_sim.py's main loop calls gateway.spin_once() (which applies queued /isaac_joint_commands via backend.command_joints()) immediately before backend.step() every tick; this probe calls backend.step() directly with no ROS spin inside it, so injecting right before step() is the equivalent point. 0 = off (default; no extra joint traffic, rows unchanged apart from the new PhysX/Lab/Python target readback fields)")
parser.add_argument("--follower-effort-limit", type=float, default=None, help="cap the effort ceiling (Nm) of the five gripper mimic/follower joints; backend default 180 (ImplicitActuatorCfg effort_limit_sim for gripper_mimic). #20: the follower cap (180) out-pushing the drive cap (50/--drive-effort-limit) is a suspect for the post-clamp ratchet, so this lets a trial pin the followers at or below the drive ceiling")
parser.add_argument("--object", default="bottle", choices=("bottle", "knife", "plate"))
parser.add_argument("--object-usda", default="")
parser.add_argument("--tcp-above-top", type=float, default=None, help="pedestal top = tcp_z - this (bottle 0.095 CoM-height side grasp; knife 0.02 top-down)")
parser.add_argument("--object-yaw-axis", default="x", choices=("x", "y"), help="which tool axis the object's long axis is aligned to (knife)")
parser.add_argument("--object-yaw-deg", type=float, default=None, help="absolute world yaw of the object (overrides --object-yaw-axis)")
parser.add_argument("--object-offset", default="0,0", help="dx,dy (base/world frame) of the object ORIGIN from the TCP xy; e.g. plate near-rim pinch = 0.10,0")
parser.add_argument("--pedestal", type=float, default=0.10, help="static pedestal side length (m); must cover the object footprint")
parser.add_argument("--no-object", action="store_true",
                     help="phase B: skip spawning the object entirely -- stage/descend/close with nothing "
                          "between the pads (a free close from a staged pose, since --descend-from only runs "
                          "in the phase-B loop). The pedestal is still authored (its placement doesn't depend "
                          "on the object mesh existing). Object-dependent events (object, bottle_placed, "
                          "the after_spawn sanity check) are skipped and object-dependent metrics (bottle_pre, "
                          "phaseB's lift bottle_dz/tilt/slide) come back null instead of describing an object "
                          "that was never there.")
# Top-down family with a real descent (the bench's pick: pregrasp above, then
# down onto the object, then close). --tcp-xz solves the planar IK for the
# grasp TCP; --descend-from stages the arm that much higher, spawns the object
# there, and each trial descends before closing.
parser.add_argument("--tcp-xz", default="", help="grasp TCP x,z in base_link (top-down planar IK; y = 0, tool z = -z); overrides --pose")
parser.add_argument("--closing-axis", default="y", choices=("x", "y"), help="base axis the jaw closes along in the top-down family (bench yaw=pi/2 -> x)")
parser.add_argument("--descend-from", type=float, default=0.0, help="stage the arm this much above the grasp TCP and descend onto the object before every close (0 = spawn at the grasp pose)")
parser.add_argument("--descend-s", type=float, default=2.0, help="descent duration (joint-space interpolation)")
parser.add_argument("--lift-dz", type=float, default=0.10, help="--lift height when the IK family is in use")
parser.add_argument("--lift-s", type=float, default=0.0, help="--lift: interpolate the arm from the grasp pose to the raised pose over this many seconds (0 = sudden step command, the original behavior). A gentle lift avoids yanking a marginal grasp out of the jaw.")
parser.add_argument("--render-dir", default="", help="if set, capture RGB frames of the close (replicator camera looking at the TCP) into this dir")
parser.add_argument("--render-cam", default="0.45,-0.5,0.35", help="camera position offset (dx,dy,dz world) from the TCP for --render-dir")
parser.add_argument("--render-every", type=float, default=0.08, help="capture a frame each time the drive advances this many rad")
parser.add_argument("--render-focal", type=float, default=40.0, help="camera focal length mm (higher = zoomed in on the jaw)")
parser.add_argument("--video", action="store_true", help="also record a dense per-step RTX video of the whole grasp (descend+close+lift) and encode <render-dir>/grasp.mp4 (needs --render-dir + ffmpeg)")
parser.add_argument("--video-stride", type=int, default=3, help="--video: capture a frame every N sim steps (lower = smoother, slower)")
parser.add_argument("--video-fps", type=int, default=30, help="--video: output mp4 frame rate")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp(
    {
        "headless": True,
        "fast_shutdown": True,
        "disable_viewport_updates": True,
        "extra_args": ["--/physics/useGpu=false", "--/physics/cudaDevice=-1"],
    }
)

import numpy as np  # noqa: E402
import torch  # noqa: E402

ROOT = Path.cwd()
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(ROOT))
from tinker_sim_core.command_mux import JointCommand  # noqa: E402
from tinker_sim_isaac.backend import IsaacWholeRobotBackend  # noqa: E402
from validation.arm_joints_parse import parse_arm_joints_deg  # noqa: E402
from validation.arm_stream_packet import build_arm_hold_packet  # noqa: E402

OUT = Path(args.out)
OUT.parent.mkdir(parents=True, exist_ok=True)
ROWS = OUT.with_suffix(OUT.suffix + ".rows.jsonl")
_out = OUT.open("a")
_rows = ROWS.open("a")


def emit(**payload: object) -> None:
    line = json.dumps(payload, sort_keys=True, default=float)
    print(line, flush=True)
    _out.write(line + "\n")
    _out.flush()


def row(**payload: object) -> None:
    _rows.write(json.dumps(payload, sort_keys=True, default=float) + "\n")


# --------------------------------------------------------------------------- boot
current = json.loads((ROOT / "artifacts/robot/tinker2/current.json").read_text())
manifest = Path(current["manifest"])
if not manifest.is_absolute():
    manifest = ROOT / manifest
t0 = time.time()
backend = IsaacWholeRobotBackend(
    usd_path=manifest.parent / "robot.usd",
    map_yaml=None,
    seed=0,
    render=False,
    # Spawn at ground height: the 0.20 m default drop tumbles the robot on the
    # bare plane (it spun 139 deg on every boot and landed on its side once).
    spawn_z=float(os.environ.get("PROBE_SPAWN_Z", "0.09")),
    enable_contacts=True,
    add_ground_plane=True,
    expected_objects=None,
    scenario="",
    task="",
)
if os.environ.get("PROBE_RELEASE_SAFETY_AT_BOOT", "0") == "1":
    backend.set_safety_stop(False)
# Spawn-time root orientation (identity in the data's own quaternion
# convention): the base hold is re-latched to this, upright at the origin,
# after the settle. The bare-ground boot launches the robot (root +12 cm in
# the first step, gripper joints at 70 rad/s for ~0.3 s, base lands 2 m away
# and sometimes on its side); the arena stack does not show this.
# The raw data.root_quat_w / body_quat_w tensors are (x, y, z, w) on this
# backend (root_state() converts to a labelled wxyz for its output); the hold
# pose is raw, so its identity is (0, 0, 0, 1). The pre-step buffer at
# construction is NOT the spawn orientation, so use the explicit identity.
_root_quat0 = torch.tensor([[0.0, 0.0, 0.0, 1.0]], dtype=torch.float32)
_REST_Z = float(os.environ.get("PROBE_REST_Z", "0.0775"))
# The kinematic base hold latches the root pose at sim t=2 s by default, which
# on the bare ground plane caught the robot mid-tumble after the 0.2 m spawn
# drop (probe1/2: root z 0.218, 40 deg tilt). Latch late, after a real settle.
PRESETTLE_S = float(os.environ.get("PROBE_PRESETTLE_S", "8.0"))
if getattr(backend, "base_fixed", False):
    backend._base_hold_after_sim_s = PRESETTLE_S
for _ in range(int((PRESETTLE_S + 1.5) / backend.dt)):
    backend.step()
if getattr(backend, "base_fixed", False) and getattr(backend, "_base_hold_pose", None) is not None:
    # Re-latch the hold upright at the origin (the write path is the same
    # write_root_pose_to_sim_index the hold uses every step).
    _pos0 = torch.tensor([[0.0, 0.0, _REST_Z]], dtype=_root_quat0.dtype, device=_root_quat0.device)
    backend._base_hold_pose = torch.cat([_pos0, _root_quat0.to(_pos0.device)], dim=-1)
    for _ in range(int(1.5 / backend.dt)):
        backend.step()
# Release the safety stop only now, the way the live stack does (the bridge
# releases it after the base has settled).
backend.set_safety_stop(False)
for _ in range(int(1.0 / backend.dt)):
    backend.step()
root0 = backend.root_state()
_qw, _qx, _qy, _qz = root0["quaternion_wxyz"]
_tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (_qx * _qx + _qy * _qy)))))
print(json.dumps({"event": "root_settled", "root": root0, "tilt_deg": _tilt}, default=float), flush=True)
if _tilt > 3.0 or not (0.03 < root0["position"][2] < 0.15):
    print(json.dumps({"event": "abort", "reason": f"base not settled upright: z={root0['position'][2]:.3f} tilt={_tilt:.1f} deg"}), flush=True)
    app.close()
    sys.exit(3)

if args.mirror_mode in ("measured", "measured_ff"):
    # Single-DOF jaw: every follower targets the drive joint's MEASURED angle,
    # so a knuckle blocked by the object stops the whole linkage (no finger
    # curl, no independent right-side motor). measured_ff adds a one-step
    # velocity feed-forward so the followers' one-step lag does not drag the
    # drive in steady motion (5 x k x dt ~ 62 N.m.s/rad at k=1500).
    _ff = args.mirror_mode == "measured_ff"

    def _mirror_measured() -> None:
        di = getattr(backend, "_drive_joint_index", None)
        ids = getattr(backend, "_gripper_mimic_indices", ())
        if di is None or not ids:
            return
        data = backend._robot.data
        q = float(backend._torch_value(data.joint_pos)[0, di])
        if _ff:
            q += float(backend._torch_value(data.joint_vel)[0, di]) * backend.dt
        for i in ids:
            backend._position_targets[0, i] = q

    backend._mirror_gripper_mimic_targets = _mirror_measured
    print(json.dumps({"event": "mirror_mode", "mode": args.mirror_mode}), flush=True)
if args.max_lead is not None:
    backend._gripper_max_lead = float(args.max_lead)
if args.stall_speed is not None:
    backend._gripper_stall_speed = float(args.stall_speed)

ARM = tuple(f"joint{i}" for i in range(1, 8))
GRIP = (
    "drive_joint",
    "left_finger_joint",
    "left_inner_knuckle_joint",
    "right_outer_knuckle_joint",
    "right_finger_joint",
    "right_inner_knuckle_joint",
)
FOLLOWERS = GRIP[1:]
PADS = ("left_finger_joint", "right_finger_joint")
names, _, _, _ = backend.joint_state()
JIDX = {n: i for i, n in enumerate(names)}
mimic_ids = list(backend._gripper_mimic_indices)
drive_id = backend._drive_joint_index
GRIP_IDS = [JIDX[n] for n in GRIP]
DT = backend.dt


class ArmStreamInjector:
    """--arm-stream-hz: inject synthetic JTC-style arm HOLD packets into
    backend.command_joints() -- the same backend entry point the live
    gateway's spin_once() calls for a queued /isaac_joint_commands message
    (RosStandardGateway._joint_command builds a JointCommand from a
    JointState via command_from_sequences and hands it straight to
    backend.command_joints; see ros_gateway.py) -- at ``hz`` packets/sec of
    SIM time. ``.tick()`` is called once per backend.step() call from the
    phase-B close/descent loops, immediately before that step() -- mirroring
    validation/run_sim.py's main loop, where gateway.spin_once() runs
    directly before backend.step() every tick.

    Fractional accumulation: per_tick = hz * dt is usually not an integer
    (e.g. hz=200 at dt=1/120 -> 1.667/tick); the fractional remainder
    carries across ticks so the long-run packet rate matches ``hz`` exactly
    rather than rounding down every tick.
    """

    def __init__(self, hz: float, dt: float) -> None:
        self.hz = float(hz)
        self.per_tick = self.hz * dt
        self._acc = 0.0
        self.sent = 0

    @property
    def enabled(self) -> bool:
        return self.hz > 0.0

    def tick(self) -> None:
        if not self.enabled:
            return
        self._acc += self.per_tick
        n = int(self._acc)
        self._acc -= n
        for _ in range(n):
            _, pos_now, _, _ = backend.joint_state()
            positions = [float(pos_now[JIDX[j]]) for j in ARM]
            names, hold_positions, velocities = build_arm_hold_packet(positions)
            backend.command_joints(
                JointCommand(names=names, positions=hold_positions, velocities=velocities)
            )
        self.sent += n


ARM_STREAM = ArmStreamInjector(args.arm_stream_hz, DT)
if ARM_STREAM.enabled:
    emit(
        event="arm_stream",
        hz=ARM_STREAM.hz,
        packets_per_tick=ARM_STREAM.per_tick,
        names=list(ARM),
    )


def gains_snapshot() -> dict[str, object]:
    data = backend._robot.data
    out: dict[str, object] = {}
    for key in ("joint_stiffness", "joint_damping", "joint_effort_limits", "joint_velocity_limits"):
        val = getattr(data, key, None)
        if val is None:
            continue
        arr = backend._torch_value(val)[0].detach().cpu().tolist()
        out[key] = {n: float(arr[JIDX[n]]) for n in GRIP if n in JIDX}
    return out


emit(
    event="boot",
    boot_s=round(time.time() - t0, 1),
    dt=DT,
    control_hz=backend.control_hz,
    physics_substeps=backend.physics_substeps,
    joint_names=list(names),
    mimic_ids=mimic_ids,
    drive_id=drive_id,
    slew=backend._gripper_close_slew,
    max_lead=backend._gripper_max_lead,
    stall_speed=backend._gripper_stall_speed,
    halt_force=backend._gripper_contact_halt_force,
    compliant_env=os.environ.get("TINKER_SIM_GRIPPER_COMPLIANT_STIFFNESS"),
    gains=gains_snapshot(),
)


def _as_numpy(value: object) -> np.ndarray:
    """Warp arrays (the tensor-API return type on this build, per the Task
    #12 precedent -- RigidBodyView reads/writes are Warp, not torch) and
    torch tensors both expose .numpy(); plain numpy/lists pass through."""
    return np.asarray(value.numpy() if hasattr(value, "numpy") else value)


def emit_link_masses() -> None:
    """Per-body mass (and inertia tensor, if the view exposes it) straight off
    the runtime PhysX articulation view -- root_view.get_masses(), shape
    (1, n_bodies) -- indexed by backend._robot.body_names (Isaac Lab's
    Articulation.body_names is root_view.shared_metatype.link_names, so the
    ordering already lines up 1:1 with the tensor columns; no JIDX-style
    name->index remap needed). This is what PhysX is actually integrating for
    every articulation body (arm links, gripper base, the six gripper links,
    the camera link) -- not a USD-authored guess -- so a missing/zeroed
    catalog mass (the #17 YCB gap) or an unexpectedly heavy link shows up in
    every probe run without a dedicated flag. No dynamics estimate here
    (nothing fancy): just the masses, for a human to eyeball against gravity
    torque by hand.
    """
    try:
        root_view = getattr(backend._robot, "root_view", None) or getattr(backend._robot, "root_physx_view", None)
        if root_view is None:
            raise RuntimeError("backend._robot exposes neither root_view nor root_physx_view")
        bnames = list(backend._robot.body_names)
        masses_arr = _as_numpy(root_view.get_masses())
        payload: dict[str, object] = {"masses": {name: float(masses_arr[0, i]) for i, name in enumerate(bnames)}}
        get_inertias = getattr(root_view, "get_inertias", None)
        if get_inertias is not None:
            inertias_arr = _as_numpy(get_inertias())
            payload["inertias"] = {name: [float(v) for v in inertias_arr[0, i]] for i, name in enumerate(bnames)}
        emit(event="link_masses", **payload)
    except Exception as error:  # pragma: no cover - defensive, PhysX API surface
        emit(event="link_masses", error=str(error)[:200])


def emit_dof_limits(tag: str | None = None) -> None:
    """Per-joint effort/velocity ceiling straight off the runtime PhysX
    articulation view -- root_view.get_dof_max_forces() / get_dof_max_velocities(),
    shape (1, n_dofs) -- indexed by backend._robot.joint_names (same
    shared_metatype ordering guarantee as body_names above). Emitted once at
    boot (to compare against the config-authored joint_effort_limits) and
    again right after the phase-B close command is issued, because the
    drive/follower caps can be rewritten per-command by the backend's effort
    mapping on newer trees (on this probe tree the direct-PhysX-write flags,
    apply_follower_effort_limit / apply_drive_effort_limit, set them).
    """
    try:
        root_view = getattr(backend._robot, "root_view", None) or getattr(backend._robot, "root_physx_view", None)
        if root_view is None:
            raise RuntimeError("backend._robot exposes neither root_view nor root_physx_view")
        jnames = list(backend._robot.joint_names)
        forces_arr = _as_numpy(root_view.get_dof_max_forces())
        payload: dict[str, object] = {"max_force": {name: float(forces_arr[0, i]) for i, name in enumerate(jnames)}}
        get_dof_max_velocities = getattr(root_view, "get_dof_max_velocities", None)
        if get_dof_max_velocities is not None:
            vel_arr = _as_numpy(get_dof_max_velocities())
            payload["max_velocity"] = {name: float(vel_arr[0, i]) for i, name in enumerate(jnames)}
        if tag is not None:
            payload["tag"] = tag
        emit(event="dof_limits", **payload)
    except Exception as error:  # pragma: no cover - defensive, PhysX API surface
        emit(event="dof_limits", tag=tag, error=str(error)[:200])


emit_link_masses()
emit_dof_limits(tag="boot")


# ------------------------------------------------------------------ helpers
def read_gripper() -> dict[str, tuple[float, float, float]]:
    n, pos, vel, tau = backend.joint_state()
    return {name: (float(pos[JIDX[name]]), float(vel[JIDX[name]]), float(tau[JIDX[name]])) for name in GRIP}


def applied_target() -> float:
    return float(backend._position_targets[0, drive_id])


def pad_forces() -> tuple[float, float]:
    st = backend.contact_state()
    return float(st["left_finger"]["force"]), float(st["right_finger"]["force"])


def body_pose(name: str) -> tuple[list[float], list[float]]:
    data = backend._robot.data
    bnames = tuple(data.body_names)
    i = bnames.index(name)
    p = backend._torch_value(data.body_pos_w)[0, i].detach().cpu().tolist()
    q = backend._torch_value(data.body_quat_w)[0, i].detach().cpu().tolist()  # wxyz
    return [float(v) for v in p], [float(v) for v in q]


def command_arm(positions: dict[str, float]) -> None:
    backend.command_joints(JointCommand(names=ARM, positions=tuple(float(positions[j]) for j in ARM)))


def command_gripper(target: float) -> None:
    backend.command_joints(JointCommand(names=("drive_joint",), positions=(float(target),)))


def wait_arm(positions: dict[str, float], timeout_s: float = 10.0, tol: float = 0.01) -> float:
    steps = int(timeout_s / DT)
    for k in range(steps):
        backend.step()
        n, pos, vel, _ = backend.joint_state()
        err = max(abs(float(pos[JIDX[j]]) - positions[j]) for j in ARM)
        spd = max(abs(float(vel[JIDX[j]])) for j in ARM)
        if err < tol and spd < 0.02:
            return k * DT
    return -1.0


def wait_gripper_open(timeout_s: float = 3.0) -> None:
    command_gripper(0.0)
    steps = int(timeout_s / DT)
    for _ in range(steps):
        backend.step()
        g = read_gripper()
        if all(abs(g[p][0]) < 0.02 and abs(g[p][1]) < 0.02 for p in PADS) and abs(g["drive_joint"][0]) < 0.02:
            break
    for _ in range(int(0.5 / DT)):
        backend.step()


def _write_gain(keyword: str, method: str, value: float, ids: list[int]) -> None:
    writer = getattr(backend._robot, method)
    tensor = torch.tensor([[float(value)] * len(ids)], dtype=torch.float32, device=backend._robot.device)
    writer(**{keyword: tensor, "joint_ids": ids, "env_ids": [0]})


def set_follower_gains(damping: float | None, stiffness: float | None,
                       drive_stiffness: float | None = None, drive_damping: float | None = None) -> dict[str, object]:
    if damping is not None:
        _write_gain("damping", "write_joint_damping_to_sim_index", damping, mimic_ids)
    if stiffness is not None:
        _write_gain("stiffness", "write_joint_stiffness_to_sim_index", stiffness, mimic_ids)
    # optional: the drive joint (left outer knuckle) -- the jaw is asymmetric by
    # default (drive k=200/d=20/cap 80 vs followers 1500/55/180)
    if drive_stiffness is not None:
        _write_gain("stiffness", "write_joint_stiffness_to_sim_index", drive_stiffness, [drive_id])
    if drive_damping is not None:
        _write_gain("damping", "write_joint_damping_to_sim_index", drive_damping, [drive_id])
    return gains_snapshot()


def _read_effective_effort_limits(ids: list[int]) -> list[float]:
    """Read back the effective effort limit for `ids` the same way backend
    does (gains_snapshot/_read_joint_gain_values): data.joint_effort_limits,
    env 0. This is the buffer write_joint_effort_limit_to_sim_index populates
    directly, so it reflects the PhysX-side write immediately; reading it
    again at the top of each phase-B config is what makes a later clobber
    (e.g. an actuator re-init restoring the ImplicitActuatorCfg default)
    visible in the log instead of silently reverting.
    """
    data = backend._robot.data
    limits = getattr(data, "joint_effort_limits", None)
    if limits is None:
        return [float("nan")] * len(ids)
    arr = backend._torch_value(limits)[0].detach().cpu().tolist()
    return [float(arr[i]) for i in ids]


def _read_physx_max_forces(ids: list[int]) -> list[float]:
    """Read the follower DOFs' max-force straight off the PhysX tensor view
    (root_view.get_dof_max_forces / root_physx_view, shape (num_instances,
    num_joints), env 0) -- the actual buffer write_joint_effort_limit_to_sim_
    index's set_dof_max_forces call targets (isaaclab_physx articulation.py
    :1613-1679). data.joint_effort_limits (the other readback in this file)
    is the Isaac Lab-side mirror of that write and can only prove the write
    was *issued*; this proves what PhysX itself is holding, independent of
    any Isaac Lab actuator-model caching (#20 cap5-analysis: bit-identical
    physics from cap 5 through cap 180 means the two readbacks may diverge
    even though both currently agree post-write).
    """
    root_view = getattr(backend._robot, "root_view", None) or getattr(backend._robot, "root_physx_view", None)
    getter = getattr(root_view, "get_dof_max_forces", None) if root_view is not None else None
    if getter is None:
        return [float("nan")] * len(ids)
    try:
        forces = getter()
        arr = forces.numpy() if hasattr(forces, "numpy") else forces
        arr = np.asarray(arr)
        return [float(arr[0, i]) for i in ids]
    except Exception as error:  # pragma: no cover - defensive, PhysX API surface
        print(json.dumps({"physx_max_force_read_error": str(error)[:160]}), flush=True)
        return [float("nan")] * len(ids)


def _read_physx_joint_forces(ids: list[int]) -> list[float]:
    """Read the PhysX-measured joint force/torque actually delivered by the
    solver for the given DOF indices -- distinct from ``joint_state()``'s
    ``tau`` (Isaac Lab's ``data.applied_torque``, the actuator MODEL's
    post-clip command that gets set INTO the sim, per
    isaaclab/assets/articulation/base_articulation_data.py:198-205 -- a
    Python-side estimate of what was asked for, not what PhysX produced).

    Uses ``root_view.get_dof_projected_joint_forces()``
    (omni.physics.tensors ArticulationView): "projects the link's incoming
    joint force[s] in the motion direction", i.e. the actual constraint-
    solver output along each joint's motion axis -- the solver-measured
    force/torque, not a command.
    """
    root_view = getattr(backend._robot, "root_view", None) or getattr(backend._robot, "root_physx_view", None)
    getter = getattr(root_view, "get_dof_projected_joint_forces", None) if root_view is not None else None
    if getter is None:
        return [float("nan")] * len(ids)
    try:
        forces = getter()
        arr = forces.numpy() if hasattr(forces, "numpy") else forces
        arr = np.asarray(arr)
        return [float(arr[0, i]) for i in ids]
    except Exception as error:  # pragma: no cover - defensive, PhysX API surface
        print(json.dumps({"physx_joint_force_read_error": str(error)[:160]}), flush=True)
        return [float("nan")] * len(ids)


def _read_physx_position_targets(ids: list[int]) -> list[float]:
    """Read the PhysX-side per-DOF position target straight off the runtime
    tensor view (root_view.get_dof_position_targets(), shape (num_instances,
    num_dofs), env 0) -- what the implicit drive is actually tracking, one
    layer below both Isaac Lab's data.joint_pos_target (the pre-write
    Isaac Lab buffer -- see _read_lab_position_targets) and this backend's
    own _position_targets (the Python-side JointCommand mirror
    _apply_joint_command writes into). #33's physx_target/lab_target/
    py_target row fields let a row show whether an --arm-stream-hz packet
    actually displaced the drive at every layer, or stalled at one of them.
    """
    root_view = getattr(backend._robot, "root_view", None) or getattr(backend._robot, "root_physx_view", None)
    getter = getattr(root_view, "get_dof_position_targets", None) if root_view is not None else None
    if getter is None:
        return [float("nan")] * len(ids)
    try:
        arr = _as_numpy(getter())
        return [float(arr[0, i]) for i in ids]
    except Exception as error:  # pragma: no cover - defensive, PhysX API surface
        print(json.dumps({"physx_position_target_read_error": str(error)[:160]}), flush=True)
        return [float("nan")] * len(ids)


def _read_lab_position_targets(ids: list[int]) -> list[float | None]:
    """Read Isaac Lab's data.joint_pos_target -- the buffer the fused
    actuator step (_apply_actuator_model, see the docstring near line 570)
    reads every step to compute the actuator model's control action -- for
    the given DOF indices; None per index if the build's Articulation.data
    does not expose the attribute (older Isaac Lab trees), rather than
    raising.
    """
    data = backend._robot.data
    val = getattr(data, "joint_pos_target", None)
    if val is None:
        return [None] * len(ids)
    try:
        arr = backend._torch_value(val)[0].detach().cpu().tolist()
        return [float(arr[i]) for i in ids]
    except Exception as error:  # pragma: no cover - defensive, Isaac Lab API surface
        print(json.dumps({"lab_position_target_read_error": str(error)[:160]}), flush=True)
        return [None] * len(ids)


def _round_or_none(value: float | None, ndigits: int = 4) -> float | None:
    return None if value is None else round(value, ndigits)


def _read_physx_dof_param(getter_name: str, ids: list[int]) -> list[float]:
    """Generic PhysX tensor-view per-DOF gain/limit readback -- root_view's
    get_dof_stiffnesses / get_dof_dampings / get_dof_max_forces /
    get_dof_max_velocities, shape (num_instances, num_dofs), env 0 -- for
    the given DOF indices. Same soft-fail shape as the other PhysX readback
    helpers in this file (nan per index, plus a one-line stderr-style event,
    if the view or the named getter is missing on this build): a diagnostic
    readback must never be the reason a --arm-stream-hz trial aborts.
    """
    root_view = getattr(backend._robot, "root_view", None) or getattr(backend._robot, "root_physx_view", None)
    getter = getattr(root_view, getter_name, None) if root_view is not None else None
    if getter is None:
        return [float("nan")] * len(ids)
    try:
        arr = _as_numpy(getter())
        return [float(arr[0, i]) for i in ids]
    except Exception as error:  # pragma: no cover - defensive, PhysX API surface
        print(json.dumps({f"{getter_name}_read_error": str(error)[:160]}), flush=True)
        return [float("nan")] * len(ids)


def _read_lab_actuator_gains(names: list[str]) -> tuple[dict[str, float | None], dict[str, float | None]]:
    """Isaac Lab's own view of stiffness/damping for the given joint names --
    ActuatorBase.stiffness/.damping tensors (isaaclab/actuators/actuator_base.py),
    indexed by the owning actuator's LOCAL joint index -- same lookup
    _patch_actuator_effort_limit_cache uses for effort_limit/effort_limit_sim.
    None per name if no actuator owns it or the tensor read fails, rather
    than raising: this is a readback for a human to eyeball against the
    PhysX-side get_dof_stiffnesses/get_dof_dampings values (#33: whether
    PhysX's drive parameters drift from Python's during a close under
    --arm-stream-hz), not something the probe's control path depends on.
    """
    k_out: dict[str, float | None] = {n: None for n in names}
    d_out: dict[str, float | None] = {n: None for n in names}
    wanted = set(names)
    for actuator in getattr(backend._robot, "actuators", {}).values():
        joint_names = getattr(actuator, "joint_names", None)
        if joint_names is None:
            continue
        for local_index, name in enumerate(joint_names):
            if name not in wanted:
                continue
            for attr, out in (("stiffness", k_out), ("damping", d_out)):
                tensor = getattr(actuator, attr, None)
                if isinstance(tensor, torch.Tensor):
                    try:
                        out[name] = float(tensor[0, local_index])
                    except (IndexError, ValueError):
                        pass
    return k_out, d_out


def _write_physx_max_forces_direct(ids: list[int], limit: float) -> None:
    """Re-assert the follower cap straight on the PhysX tensor view, bypassing
    the Isaac Lab wrapper (write_joint_effort_limit_to_sim_index only ever
    calls this same root_view.set_dof_max_forces once, immediately and on the
    full joint set), so this is a redundant, same-call re-assertion, not a
    different code path.

    set_dof_max_forces takes the FULL (num_instances, num_joints) row for the
    selected env indices, not a sparse per-joint column, so this reads the
    current full row back (already updated for `ids` by the wrapper call that
    precedes this one), patches it, and pushes the whole row -- and, per the
    Task #12 precedent (RigidBodyView needs WARP arrays, not torch, for
    tensor-API writes to actually land), uses warp arrays for both the
    payload and the indices rather than torch tensors.
    """
    root_view = getattr(backend._robot, "root_view", None) or getattr(backend._robot, "root_physx_view", None)
    setter = getattr(root_view, "set_dof_max_forces", None) if root_view is not None else None
    if setter is None:
        return
    try:
        import warp as wp

        data = backend._robot.data
        full = backend._torch_value(data.joint_effort_limits).clone()
        full[0, ids] = float(limit)
        full_cpu = full.detach().to(device="cpu", dtype=torch.float32).contiguous()
        forces_wp = wp.from_torch(full_cpu, dtype=wp.float32)
        indices_wp = wp.array([0], dtype=wp.int32, device="cpu")
        setter(forces_wp, indices=indices_wp)
    except Exception as error:  # pragma: no cover - defensive, PhysX API surface
        print(json.dumps({"physx_max_force_write_error": str(error)[:160]}), flush=True)


def _author_usd_max_force(ids: list[int], limit: float, event: str = "follower_usd_drive_authored") -> None:
    """Author physics:maxForce on each targeted joint's UsdPhysics.DriveAPI
    directly on the stage, in addition to the runtime tensor-API write, so a
    stage re-parse (a reset that rebuilds the actuator from the USD prim)
    still carries the cap. Best-effort: the follower joints are mimic joints
    the importer dropped drives for, so a DriveAPI may not already be applied
    -- Apply() creates it. Per-joint failures are logged, not fatal.
    """
    try:
        import omni.usd
        from pxr import Usd, UsdPhysics
    except ImportError:
        return
    id_to_name = {index: name for name, index in JIDX.items()}
    target_names = {id_to_name[i] for i in ids if i in id_to_name}
    if not target_names:
        return
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return
    robot_prim_path = str(getattr(getattr(backend._robot, "cfg", None), "prim_path", "") or "/World/Tinker")
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        return
    authored, errors = [], []
    for prim in Usd.PrimRange(robot_prim):
        name = prim.GetName()
        if name not in target_names:
            continue
        for instance in ("angular", "linear"):
            try:
                drive = UsdPhysics.DriveAPI.Apply(prim, instance)
                drive.CreateMaxForceAttr(float(limit))
                authored.append(f"{name}:{instance}")
            except Exception as error:  # pragma: no cover - defensive, schema surface
                errors.append(f"{name}:{instance}:{str(error)[:80]}")
    print(json.dumps({
        "event": event,
        "requested": limit,
        "authored": authored,
        "errors": errors,
    }), flush=True)


def _patch_actuator_effort_limit_cache(ids: list[int], limit: float) -> None:
    """Mirror the PhysX effort-limit write into the owning ImplicitActuator's
    cached tensors -- the same workaround backend._set_gripper_effort_limit
    applies for drive_joint (Isaac Lab issue #128: write_joint_effort_limit_to_
    sim_index only updates the simulator buffer, not the actuator model).
    Patches both `effort_limit` (read every step by _clip_effort) and
    `effort_limit_sim` (read at actuator (re)construction) since ActuatorBase
    keeps them as separate tensors.
    """
    id_to_name = {index: name for name, index in JIDX.items()}
    target_names = {id_to_name[i] for i in ids if i in id_to_name}
    if not target_names:
        return
    for actuator in getattr(backend._robot, "actuators", {}).values():
        names = getattr(actuator, "joint_names", None)
        if names is None:
            continue
        for local_index, name in enumerate(names):
            if name not in target_names:
                continue
            for attr in ("effort_limit", "effort_limit_sim"):
                tensor = getattr(actuator, attr, None)
                if isinstance(tensor, torch.Tensor):
                    tensor[:, local_index] = limit


def apply_follower_effort_limit(emit_event: bool = False, event: str = "follower_effort_limit_set") -> None:
    """Cap the five follower joints' effort ceiling at --follower-effort-limit.

    Reuses the Isaac Lab writer already used for the drive joint (backend
    _set_gripper_effort_limit / _write_safety_effort_limit): PhysX
    write_joint_effort_limit_to_sim_index(limits=..., joint_ids=..., env_ids=...).
    Also patches the owning ImplicitActuator's cached effort_limit/
    effort_limit_sim tensors in place, re-asserts the cap directly on the
    PhysX tensor view (_write_physx_max_forces_direct) and authors it onto
    each follower joint's USD DriveAPI (_author_usd_max_force) so a stage
    re-parse would still carry it, and reads the effective limit back both
    from Isaac Lab's data.joint_effort_limits and straight from PhysX
    (get_dof_max_forces) for the event -- #20 evidence (bit-identical hold
    physics at cap 5/50/100 vs cap-180) shows the write reaching the
    tensor-API buffer is NOT sufficient proof it reaches the solver's actual
    joint-drive force limit for these mimic joints; the physx_max_force
    readback is what would catch that gap. A no-op unless
    --follower-effort-limit was passed.
    """
    if args.follower_effort_limit is None:
        return
    limit = float(args.follower_effort_limit)
    _write_gain("limits", "write_joint_effort_limit_to_sim_index", limit, mimic_ids)
    _write_physx_max_forces_direct(mimic_ids, limit)
    _patch_actuator_effort_limit_cache(mimic_ids, limit)
    _author_usd_max_force(mimic_ids, limit)
    if emit_event:
        print(json.dumps({
            "event": event,
            "requested": limit,
            "indices": mimic_ids,
            "effective": _read_effective_effort_limits(mimic_ids),
            "physx_max_force": _read_physx_max_forces(mimic_ids),
        }), flush=True)


def parse_configs(text: str) -> list[dict[str, float]]:
    """slew:damping:stiffness[:drive_stiffness:drive_damping]"""
    configs = []
    for item in text.split(","):
        item = item.strip()
        if not item:
            continue
        parts = [float(v) for v in item.split(":")]
        cfg = {"slew": parts[0], "damping": parts[1], "stiffness": parts[2]}
        if len(parts) >= 5:
            cfg["drive_stiffness"], cfg["drive_damping"] = parts[3], parts[4]
        configs.append(cfg)
    return configs


CONFIGS = parse_configs(args.configs)

if args.drive_effort_limit is not None:
    # Raise the drive_joint effort ceiling so the close can press past the URDF
    # 50 Nm cap (reuses the backend's own effort-limit writer + actuator-model
    # sync). This is the #20 sweep: quantify the clamp-normal force each object
    # geometry reaches as the drive ceiling rises.
    #
    # #33 caveat on OLDER runs of this flag: until the #33 fix, the backend's
    # effort-limit path also authored physics:maxForce onto drive_joint's USD
    # DriveAPI, and omni.physx's USD change listener answered that by
    # re-creating the drive from the stage -- replacing the runtime gains
    # (200/20) with the asset's authored drive, 35809.86 / 0.0 in PhysX radian
    # units (robot.usd authors stiffness 625.0 in USD degree units, damping
    # 0.0). This block runs BEFORE any later gain write, so which pre-fix runs
    # are affected depends on what the run does next:
    #   - default --mirror-mode target with 3-part configs: nothing rewrites
    #     drive_joint's gains afterwards, so the whole close ran a ~180x-stiff,
    #     zero-damping drive_joint. Those numbers are not comparable with runs
    #     that leave the flag off.
    #   - --mirror-mode central (below), or a 5-part config, whose run_close
    #     calls set_follower_gains(..., drive_stiffness, drive_damping): those
    #     rewrite drive_joint's k/d after the reversion. Whether that write
    #     actually binds in PhysX was never measured, so treat those runs as
    #     unknown rather than clean.
    # Do NOT author USD drives from this probe either -- write the PhysX tensor
    # view directly.
    backend._default_gripper_effort_limit = float(args.drive_effort_limit)
    backend._gripper_effort_limit_written = False
    backend._set_gripper_effort_limit(float(args.drive_effort_limit))
    print(json.dumps({
        "event": "drive_effort_limit_set",
        "requested": args.drive_effort_limit,
        "gripper_effort_limit": backend.gripper_effort_limit,
    }), flush=True)

if args.follower_effort_limit is not None:
    # #20: cap the follower (mimic) joints' effort ceiling -- backend default
    # 180 Nm, vs the drive_joint's 50 (--drive-effort-limit). Same writer as
    # the drive joint (write_joint_effort_limit_to_sim_index), applied across
    # the five follower indices instead of just drive_joint.
    apply_follower_effort_limit(emit_event=True)

if args.mirror_mode == "central":
    set_follower_gains(55.0, 1500.0, drive_stiffness=1500.0, drive_damping=55.0)
    if args.max_lead is None:
        backend._gripper_max_lead = 0.015
    def _mirror_central() -> None:
        di = getattr(backend, "_drive_joint_index", None)
        ids = getattr(backend, "_gripper_mimic_indices", ())
        if di is None or not ids:
            return
        c = float(backend._position_targets[0, di])   # applied central command
        for i in ids:
            backend._position_targets[0, i] = c
    backend._mirror_gripper_mimic_targets = _mirror_central
    print(json.dumps({"event": "mirror_mode", "mode": "central",
                      "max_lead": backend._gripper_max_lead}), flush=True)

# ----------------------------------------------------------- arm staging pose
# Built-in arm poses, solved offline from the artifact URDF (planar elbow
# family, base_link frame) and verified against the sim's link_tcp:
#   side    = the grasp bench's recorded bottle side-grasp: TCP 0.5375 m ahead,
#             0.7446 m up, tool z = +x (horizontal approach), pads at ±y.
#   topdown = knife/plate pinch: TCP 0.5375 ahead, 0.668 up, tool z = -z.
BUILTIN_POSES = {
    "side": {"joint1": 0.0, "joint2": 0.5957, "joint3": 0.0, "joint4": 0.6539, "joint5": 0.0, "joint6": -1.5084, "joint7": math.pi},
    "topdown": {"joint1": 0.0, "joint2": 0.3892, "joint3": 0.0, "joint4": 1.4034, "joint5": 0.0, "joint6": 1.0141, "joint7": 0.0},
}
grasp = None
if args.grasp_config:
    cfg = json.loads(Path(args.grasp_config).read_text())
    events = cfg.get("close_events", cfg if isinstance(cfg, list) else [])
    if events:
        grasp = events[min(args.grasp_index, len(events) - 1)]


# --------------------------------------------------- planar top-down IK (URDF)
# Planar elbow family: joints 1/3/5 = 0, joint7 fixed (0 -> jaw closes along
# base y, +-pi/2 -> along base x); joints 2/4/6 solve TCP (x, z) + tool z = -z.
# FK is the artifact URDF (same file the sim's USD was built from; verified
# against the sim's link_tcp to < 2 mm for the built-in poses).
def _urdf_fk_factory():
    import xml.etree.ElementTree as ET

    urdf = manifest.parent / "robot.urdf"
    root_el = ET.parse(urdf).getroot()
    joints: dict[str, dict] = {}
    child_of: dict[str, str] = {}
    for j in root_el.findall("joint"):
        if j.find("parent") is None or j.find("child") is None:
            continue
        o = j.find("origin")
        xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
        r, p, y = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        a = j.find("axis")
        axis = np.array([float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()])
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        R0 = (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
              @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
        joints[j.get("name")] = {"type": j.get("type"), "parent": j.find("parent").get("link"), "R0": R0, "p0": xyz, "axis": axis / np.linalg.norm(axis)}
        child_of[j.find("child").get("link")] = j.get("name")

    def chain(link: str) -> list[str]:
        out = []
        while link in child_of:
            out.append(child_of[link])
            link = joints[child_of[link]]["parent"]
        return list(reversed(out))

    tcp_chain = chain("link_tcp")

    def fk(q: dict[str, float]) -> tuple[np.ndarray, np.ndarray]:
        R = np.eye(3)
        p = np.zeros(3)
        for jn in tcp_chain:
            jd = joints[jn]
            p = p + R @ jd["p0"]
            R = R @ jd["R0"]
            if jd["type"] in ("revolute", "continuous"):
                a = jd["axis"]
                K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                th = q.get(jn, 0.0)
                R = R @ (np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K)
        return R, p

    return fk


def planar_topdown_ik(x: float, z: float, joint7: float, seed: dict[str, float] | None = None) -> dict[str, float]:
    """Joints for TCP (x, 0, z) in base_link with tool z pointing straight down."""
    fk = planar_topdown_ik._fk
    q = dict(BUILTIN_POSES["topdown"] if seed is None else seed)
    q["joint7"] = joint7
    free = ("joint2", "joint4", "joint6")

    def err(qd: dict[str, float]) -> np.ndarray:
        R, p = fk(qd)
        tz = R[:, 2]
        pitch = math.atan2(tz[0], -tz[2])  # 0 when tool z == -base z (planar family: tz[1] == 0)
        return np.array([p[0] - x, p[2] - z, 0.3 * pitch])

    for _ in range(200):
        e = err(q)
        if np.linalg.norm(e) < 1e-7:
            break
        J = np.zeros((3, 3))
        for c, jn in enumerate(free):
            q2 = dict(q)
            q2[jn] += 1e-6
            J[:, c] = (err(q2) - e) / 1e-6
        try:
            step = -np.linalg.solve(J.T @ J + 1e-9 * np.eye(3), J.T @ e)
        except np.linalg.LinAlgError:
            break
        for c, jn in enumerate(free):
            q[jn] = float(q[jn] + step[c])
    e = err(q)
    if abs(e[0]) > 1e-3 or abs(e[1]) > 1e-3 or abs(e[2]) / 0.3 > 1e-2:
        raise SystemExit(f"planar IK failed for x={x} z={z}: residual {e}")
    return {j: float(q[j]) for j in ARM}


ik_family = bool(args.tcp_xz)
if args.arm_joints:
    # Diagnostic override: hold exactly this configuration instead of an IK
    # or built-in pose. --tcp-xz's closing-axis assumption is meaningless
    # here (the jaw's closing direction falls out of whatever configuration
    # this is, and is measured -- not assumed -- in the arm_joints event
    # below), so the two are mutually exclusive rather than silently picking
    # one.
    if ik_family:
        raise SystemExit("--arm-joints is incompatible with --tcp-xz; pick one arm-pose source")
    try:
        arm_pose = parse_arm_joints_deg(args.arm_joints)
    except ValueError as error:
        raise SystemExit(str(error)) from None
    arm_source = f"arm_joints_deg:{args.arm_joints}"
elif ik_family:
    planar_topdown_ik._fk = _urdf_fk_factory()  # type: ignore[attr-defined]
    _gx, _gz = (float(v) for v in args.tcp_xz.split(","))
    _j7 = 0.0 if args.closing_axis == "y" else math.pi / 2
    arm_pose = planar_topdown_ik(_gx, _gz, _j7)
    arm_source = f"ik:tcp=({_gx},{_gz}),closing={args.closing_axis}"
else:
    arm_pose = dict(BUILTIN_POSES[args.pose])
    arm_source = f"builtin:{args.pose}"
if grasp and "arm_joints" in grasp:
    arm_pose = {j: float(grasp["arm_joints"][j]) for j in ARM}
    arm_source = "grasp_config"
emit(event="arm_pose", source=arm_source, pose=arm_pose)
wait_gripper_open()
command_arm(arm_pose)
settled = wait_arm(arm_pose)
tcp_p, tcp_q = body_pose("link_tcp")
lf_p, _ = body_pose("left_finger")
rf_p, _ = body_pose("right_finger")
pad_mid = [(a + b) / 2 for a, b in zip(lf_p, rf_p)]
pad_axis = [a - b for a, b in zip(lf_p, rf_p)]
emit(event="staged", settle_s=settled, tcp=tcp_p, tcp_quat_wxyz=tcp_q, left_finger=lf_p, right_finger=rf_p, pad_mid=pad_mid, pad_axis=pad_axis, gap=math.dist(lf_p, rf_p),
     root=backend.root_state() if hasattr(backend, "root_state") else None)

if args.arm_joints:
    # --arm-joints diagnostic: since --closing-axis is not in play here, the
    # jaw's closing direction is measured (not assumed) straight from the
    # settled finger link positions -- the same pad_axis the staged event
    # above reports -- and reported alongside its gravity component. The
    # "reach" axis (tcp -> pad_mid) is the direction the fingers extend from
    # the wrist toward the pinch point in this configuration; its gravity
    # component says whether the reach is level, angled up, or angled down.
    def _unit(v: list[float]) -> list[float]:
        n = math.sqrt(sum(c * c for c in v))
        return [c / n for c in v] if n > 1e-9 else [0.0, 0.0, 0.0]

    def _gravity_component(v_unit: list[float]) -> float:
        # positive = axis points downward, i.e. aligned with gravity (world -z)
        return -v_unit[2]

    _n_arm, _pos_arm, _, _ = backend.joint_state()
    measured_deg = [math.degrees(float(_pos_arm[JIDX[j]])) for j in ARM]
    target_deg = [math.degrees(arm_pose[j]) for j in ARM]
    closing_axis_world = _unit(pad_axis)
    reach_axis_world = _unit([a - b for a, b in zip(pad_mid, tcp_p)])
    emit(
        event="arm_joints",
        target_deg=target_deg,
        measured_deg=measured_deg,
        tcp=tcp_p,
        tcp_quat_wxyz=tcp_q,
        closing_axis_world=closing_axis_world,
        closing_axis_gravity_component=_gravity_component(closing_axis_world),
        finger_reach_axis_world=reach_axis_world,
        finger_reach_axis_gravity_component=_gravity_component(reach_axis_world),
    )

# --------------------------------------------------------- optional RGB capture
# A replicator observer camera looking at the TCP; app.update() renders it even
# though the backend steps with render=False. get_data() -> HxWx4 uint8.
_render = {"on": bool(args.render_dir)}
if _render["on"]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.image as _mpimg
    import omni.replicator.core as rep

    import omni.usd
    from pxr import Gf, UsdGeom

    _cam_off = [float(v) for v in args.render_cam.split(",")]
    # aim slightly below the pad origins, toward the fingertips/object
    _look = (pad_mid[0], pad_mid[1], pad_mid[2] - 0.045)
    _cam_pos = (pad_mid[0] + _cam_off[0], pad_mid[1] + _cam_off[1], pad_mid[2] + _cam_off[2])
    # Build the camera prim directly and set its look-at transform with USD math
    # (rep.create.camera's look_at did not orient the camera on this stack).
    _stage = omni.usd.get_context().get_stage()
    _cam_path = "/World/ProbeCam"
    _ucam = UsdGeom.Camera.Define(_stage, _cam_path)
    _ucam.GetFocalLengthAttr().Set(float(args.render_focal))
    _ucam.GetHorizontalApertureAttr().Set(20.955)
    _ucam.GetVerticalApertureAttr().Set(20.955 * 540.0 / 960.0)
    _ucam.GetClippingRangeAttr().Set(Gf.Vec2f(0.02, 100.0))
    # SetLookAt gives world->view (view looks down -Z); camera local-to-world is
    # its inverse.
    _view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*_cam_pos), Gf.Vec3d(*_look), Gf.Vec3d(0, 0, 1))
    _xf = UsdGeom.Xformable(_ucam.GetPrim())
    _xf.ClearXformOpOrder()
    _xf.AddTransformOp().Set(_view.GetInverse())
    _rp = rep.create.render_product(_cam_path, (960, 540))
    _rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
    _rgb_annot.attach([_rp])
    # lights: a dome for fill + a distant key so the object is not a silhouette
    try:
        rep.create.light(light_type="dome", intensity=1200)
        rep.create.light(light_type="distant", intensity=2500,
                         rotation=(-45, 20, 0))
    except Exception as _le:
        print(json.dumps({"event": "render_light_error", "err": str(_le)[:120]}), flush=True)
    for _ in range(8):
        app.update()
    Path(args.render_dir).mkdir(parents=True, exist_ok=True)
    _render.update(next_drive=-1.0, idx=0)

    def capture_frame(tag: str, drive: float, force: str = "") -> None:
        if not _render["on"]:
            return
        for _ in range(2):
            app.update()
        arr = _rgb_annot.get_data()
        try:
            import numpy as _np
            arr = _np.asarray(arr)
            if arr.ndim == 3 and arr.shape[-1] >= 3:
                img = arr[:, :, :3].astype("uint8")
                name = f"{_render['idx']:02d}_{tag}_d{drive:.2f}{('_' + force) if force else ''}.png"
                _mpimg.imsave(str(Path(args.render_dir) / name), img)
                _render["idx"] += 1
        except Exception as _e:
            print(json.dumps({"event": "render_error", "err": str(_e)[:160]}), flush=True)

    def maybe_capture(tag: str, drive: float, force: str = "") -> None:
        if _render["on"] and drive >= _render["next_drive"]:
            capture_frame(tag, drive, force)
            _render["next_drive"] = drive + args.render_every

    # Dense per-step video capture: unlike maybe_capture (drive-gated, so it
    # never samples the open-gripper descent or the constant-drive lift), this
    # fires on a fixed step stride so the mp4 shows the whole grasp arc.
    import numpy as _np_v
    _video = {"on": bool(args.video), "stride": max(1, args.video_stride), "n": 0, "vidx": 0}
    _video_dir = Path(args.render_dir) / "video_frames"
    if _video["on"]:
        _video_dir.mkdir(parents=True, exist_ok=True)

    def video_tick() -> None:
        if not _video["on"]:
            return
        _video["n"] += 1
        if _video["n"] % _video["stride"] != 0:
            return
        app.update()
        try:
            arr = _np_v.asarray(_rgb_annot.get_data())
            if arr.ndim == 3 and arr.shape[-1] >= 3:
                _mpimg.imsave(str(_video_dir / f"frame_{_video['vidx']:06d}.png"),
                              arr[:, :, :3].astype("uint8"))
                _video["vidx"] += 1
        except Exception as _ve:
            print(json.dumps({"event": "video_error", "err": str(_ve)[:160]}), flush=True)
else:
    def capture_frame(tag: str, drive: float, force: str = "") -> None:
        pass

    def maybe_capture(tag: str, drive: float, force: str = "") -> None:
        pass

    def video_tick() -> None:
        pass

# Descent staging: park the arm above the grasp before the object appears.
stage_pose = arm_pose
if args.descend_from > 0.0:
    if args.arm_joints:
        # --arm-joints holds one exact configuration; there is no separate
        # staged pose above it to descend from (the object, if any, is
        # spawned directly at that configuration's pinch point).
        raise SystemExit("--descend-from is not supported with --arm-joints (the arm goes straight to the given configuration)")
    if not ik_family:
        raise SystemExit("--descend-from needs --tcp-xz (planar IK family)")
    stage_pose = planar_topdown_ik(_gx, _gz + args.descend_from, _j7, seed=arm_pose)
    command_arm(stage_pose)
    st = wait_arm(stage_pose)
    stage_tcp, _ = body_pose("link_tcp")
    emit(event="staged_high", settle_s=st, tcp=stage_tcp, dz=stage_tcp[2] - tcp_p[2], pose=stage_pose)


def descend(duration_s: float) -> dict[str, object]:
    """Interpolate stage_pose -> arm_pose in joint space, then hold 1 s; report the TCP z reached."""
    n = max(1, int(duration_s / DT))
    for k in range(1, n + 1):
        a = k / n
        command_arm({j: stage_pose[j] + a * (arm_pose[j] - stage_pose[j]) for j in ARM})
        ARM_STREAM.tick()
        backend.step()
        video_tick()
    for _ in range(int(1.0 / DT)):
        ARM_STREAM.tick()
        backend.step()
        video_tick()
    tcp_now, _ = body_pose("link_tcp")
    lf_now, _ = body_pose("left_finger")
    rf_now, _ = body_pose("right_finger")
    _, pos, _, _ = backend.joint_state()
    joint_err = {j: round(float(pos[JIDX[j]]) - arm_pose[j], 4) for j in ARM}
    pairs = [(str(p["body_a"]).split("/")[-1], str(p["body_b"]).split("/")[-1], round(float(p["normal_force"]), 1)) for p in backend.contact_pairs()]
    return {"tcp": tcp_now, "tcp_dz_vs_commanded": tcp_now[2] - tcp_p[2], "left_finger": lf_now, "right_finger": rf_now,
            "joint_err": joint_err, "contact_pairs": pairs[:12]}


# ----------------------------------------------------------------- Phase A
def run_close(tag: str, cfg: dict[str, float], bottle_reader=None) -> dict[str, object]:
    backend._gripper_close_slew = cfg["slew"]
    gains = set_follower_gains(cfg["damping"], cfg["stiffness"], cfg.get("drive_stiffness"), cfg.get("drive_damping"))
    if _render["on"]:
        _render["next_drive"] = -1.0  # capture from the first step of this close
    command_gripper(args.close_target)
    if tag == "B":
        # The drive/follower effort caps can be rewritten per command on some
        # trees (see emit_dof_limits' docstring); re-read them right after
        # this phase-B close command lands so a per-command clobber shows up
        # here rather than only in the boot-time snapshot.
        emit_dof_limits(tag="phaseB_after_close_command")
    steps = int(args.record_s / DT)
    peak_force = 0.0
    peak_force_t = None
    first_contact_t = None
    first_contact_force = None
    contact_next_force = None
    max_tau_motion = 0.0
    max_lag_motion = 0.0
    lags_motion: list[float] = []
    speeds_motion: list[float] = []
    taus_motion: list[float] = []
    t_reach = None
    stall_t = None
    stall_run = 0
    last: dict[str, object] = {}
    bottle_rows: list[dict[str, float]] = []
    for k in range(steps):
        if tag == "B":
            ARM_STREAM.tick()
        backend.step()
        t = k * DT
        g = read_gripper()
        target = applied_target()
        pad_pos = min(g[p][0] for p in PADS)
        pad_speed = max(abs(g[p][1]) for p in PADS)
        taus = {n: g[n][2] for n in FOLLOWERS}
        max_tau = max(abs(v) for v in taus.values())
        lag = target - pad_pos
        lf, rf = pad_forces()
        force = lf + rf
        moving = pad_speed > 0.1
        if moving:
            max_tau_motion = max(max_tau_motion, max_tau)
            max_lag_motion = max(max_lag_motion, lag)
            lags_motion.append(lag)
            speeds_motion.append(pad_speed)
            taus_motion.append(max_tau)
        if force > peak_force:
            peak_force, peak_force_t = force, t
        if first_contact_t is None and force > 0.0:
            first_contact_t, first_contact_force = t, force
        elif first_contact_t is not None and contact_next_force is None:
            contact_next_force = force
        if t_reach is None and pad_pos >= args.close_target - 0.01:
            t_reach = t
        if t > 0.2 and pad_speed <= backend._gripper_stall_speed:
            stall_run += 1
            if stall_run >= int(0.25 / DT) and stall_t is None:
                stall_t = t
        else:
            stall_run = 0
        physx_taus = _read_physx_joint_forces(GRIP_IDS)
        # #33 --arm-stream-hz readback: the drive target at every layer a
        # command can stall at (PhysX's own drive, Isaac Lab's actuator-model
        # input buffer, this backend's Python-side JointCommand mirror), plus
        # the PhysX-side and Isaac-Lab-side gains (k/d/max_force/max_vel) --
        # same set for left_finger_joint (a follower) so a row can show
        # whether an injected hold packet (which only ever names arm joints)
        # still perturbs the gripper's own targets/gains indirectly, or
        # leaves them untouched, and whether PhysX's drive parameters drift
        # from Python's during a close.
        _lf_id = JIDX["left_finger_joint"]
        _grip_target_ids = [drive_id, _lf_id]
        _physx_targets = _read_physx_position_targets(_grip_target_ids)
        _lab_targets = _read_lab_position_targets(_grip_target_ids)
        _physx_k = _read_physx_dof_param("get_dof_stiffnesses", _grip_target_ids)
        _physx_d = _read_physx_dof_param("get_dof_dampings", _grip_target_ids)
        _physx_max_force = _read_physx_dof_param("get_dof_max_forces", _grip_target_ids)
        _physx_max_vel = _read_physx_dof_param("get_dof_max_velocities", _grip_target_ids)
        _lab_k, _lab_d = _read_lab_actuator_gains(["drive_joint", "left_finger_joint"])
        r = {
            "tag": tag, "k": k, "t": round(t, 4), "target": target, "drive": g["drive_joint"][0],
            "pad_pos": pad_pos, "pad_speed": pad_speed, "lag": lag, "lf": lf, "rf": rf,
            "tau_drive": round(g["drive_joint"][2], 3),
            "tau": {n: round(v, 3) for n, v in taus.items()},
            # PhysX solver-measured joint force/torque (get_dof_projected_joint_forces),
            # alongside the pre-existing tau/tau_drive (Isaac Lab's applied_torque,
            # the actuator model's commanded value) -- see _read_physx_joint_forces.
            "physx_tau": {n: round(v, 3) for n, v in zip(GRIP, physx_taus)},
            "physx_target": round(_physx_targets[0], 4),
            "py_target": round(float(backend._position_targets[0, drive_id]), 4),
            "lab_target": _round_or_none(_lab_targets[0]),
            "physx_k": round(_physx_k[0], 3),
            "physx_d": round(_physx_d[0], 3),
            "physx_max_force": round(_physx_max_force[0], 3),
            "physx_max_vel": round(_physx_max_vel[0], 3),
            "lab_k": _round_or_none(_lab_k["drive_joint"], 3),
            "lab_d": _round_or_none(_lab_d["drive_joint"], 3),
            "physx_target_left_finger": round(_physx_targets[1], 4),
            "py_target_left_finger": round(float(backend._position_targets[0, _lf_id]), 4),
            "lab_target_left_finger": _round_or_none(_lab_targets[1]),
            "physx_k_left_finger": round(_physx_k[1], 3),
            "physx_d_left_finger": round(_physx_d[1], 3),
            "physx_max_force_left_finger": round(_physx_max_force[1], 3),
            "physx_max_vel_left_finger": round(_physx_max_vel[1], 3),
            "lab_k_left_finger": _round_or_none(_lab_k["left_finger_joint"], 3),
            "lab_d_left_finger": _round_or_none(_lab_d["left_finger_joint"], 3),
            "pos": {n: round(g[n][0], 4) for n in FOLLOWERS},
        }
        if bottle_reader is not None:
            b = bottle_reader()
            r["bottle"] = b
            bottle_rows.append(b)
            # pad geometry per step: where the finger bodies are (world) and
            # whether a pad touches the pedestal (desk) rather than the object
            lfp, _ = body_pose("left_finger")
            rfp, _ = body_pose("right_finger")
            r["lf_pos"] = [round(v, 4) for v in lfp]
            r["rf_pos"] = [round(v, 4) for v in rfp]
            if k % 6 == 0:
                r["pairs"] = [(str(p["body_a"]).split("/")[-1], str(p["body_b"]).split("/")[-1], round(float(p["normal_force"]), 1)) for p in backend.contact_pairs()][:8]
        row(**r)
        last = r
        maybe_capture(tag, g["drive_joint"][0], force=f"L{lf:.0f}R{rf:.0f}")
        video_tick()
    tail = int(0.5 / DT)
    metrics: dict[str, object] = {
        "tag": tag, "config": cfg, "gains_after_write": {k: gains.get(k) for k in ("joint_damping", "joint_stiffness")},
        "t_reach": t_reach, "stall_t": stall_t,
        "peak_pad_force": peak_force, "peak_pad_force_t": peak_force_t,
        "first_contact_t": first_contact_t, "first_contact_force": first_contact_force, "contact_next_force": contact_next_force,
        "max_tau_motion": max_tau_motion, "max_lag_motion": max_lag_motion,
        "median_lag_motion": float(np.median(lags_motion)) if lags_motion else None,
        "median_speed_motion": float(np.median(speeds_motion)) if speeds_motion else None,
        "median_tau_motion": float(np.median(taus_motion)) if taus_motion else None,
        "final": {k: last.get(k) for k in ("target", "drive", "pad_pos", "pad_speed", "lag", "lf", "rf", "tau_drive", "tau", "physx_tau", "pos")},
    }
    if bottle_rows:
        z0 = bottle_rows[0]["z"]
        xy0 = (bottle_rows[0]["x"], bottle_rows[0]["y"])
        disp = [math.hypot(b["x"] - xy0[0], b["y"] - xy0[1]) for b in bottle_rows]
        tilt = [b["tilt_deg"] for b in bottle_rows]
        end = bottle_rows[-1]
        metrics["bottle"] = {
            "max_xy_disp": max(disp), "end_xy_disp": disp[-1], "max_tilt_deg": max(tilt), "end_tilt_deg": end["tilt_deg"],
            "end_dz": end["z"] - z0, "end_dist_to_pad_mid_xy": math.hypot(end["x"] - pad_mid[0], end["y"] - pad_mid[1]),
            "hold_force_tail_mean": float(np.mean([0.0] * 0 + [r_["lf"] + r_["rf"] for r_ in [last]])),
        }
    return metrics


if "A" in args.phase:
    for cfg in CONFIGS:
        wait_gripper_open()
        for _ in range(int(args.settle_s / DT)):
            backend.step()
        m = run_close("A", cfg)
        emit(event="phaseA", **m)
    # restore defaults (followers and drive)
    set_follower_gains(55.0, 1500.0, 200.0, 20.0)
    backend._gripper_close_slew = 1.5
    wait_gripper_open()

# ----------------------------------------------------------------- Phase B
if "B" in args.phase:
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    # bottle base position
    if grasp and "bottle_rel_base" in grasp:
        rel = grasp["bottle_rel_base"]
        bottle_base = [float(rel["x"]), float(rel["y"]), float(rel["z"])]
        source = "grasp_config"
    elif args.bottle_offset:
        off = [float(v) for v in args.bottle_offset.split(",")]
        bottle_base = [pad_mid[0] + off[0], pad_mid[1] + off[1], pad_mid[2] + off[2]]
        source = "offset"
    else:
        # bench convention (mimic_driver / grasp_benchmark _side_grasp): the
        # object axis sits at the TCP xy; TCP at the bottle mid-band (CoM
        # height, 0.095) for the side grasp, or 0.02 above the desk for the
        # top-down knife pinch.
        above = args.tcp_above_top if args.tcp_above_top is not None else (0.095 if args.object == "bottle" else 0.02)
        odx, ody = (float(v) for v in args.object_offset.split(","))
        bottle_base = [tcp_p[0] + odx, tcp_p[1] + ody, tcp_p[2] - above]
        source = f"tcp_minus_{above}+offset({odx},{ody})"
    support_top = bottle_base[2]
    object_usda = args.object_usda or args.bottle_usda or str(
        ROOT / "simulation/assets/primitives" / f"bench-{args.object}.usda"
    )
    # object yaw: align its long axis (asset +x) with the tool's x or y axis in world.
    # body_quat_w comes through as XYZW here (the backend's root_state reorders
    # the same way); decoding it as wxyz put the knife 139 deg off (probe5-8).
    qx, qy, qz, w = tcp_q
    tool_x = [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy + w * qz), 2 * (qx * qz - w * qy)]
    tool_y = [2 * (qx * qy - w * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz + w * qx)]
    axis_v = tool_x if args.object_yaw_axis == "x" else tool_y
    object_yaw_deg = math.degrees(math.atan2(axis_v[1], axis_v[0]))
    if args.object_yaw_deg is not None:
        object_yaw_deg = float(args.object_yaw_deg)
    # static support block with desk-like friction
    mat = UsdShade.Material.Define(stage, "/World/Probe/SupportMat")
    mapi = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    mapi.CreateStaticFrictionAttr(1.0)
    mapi.CreateDynamicFrictionAttr(1.0)
    mapi.CreateRestitutionAttr(0.0)
    # Narrow pedestal (bottle footprint only): a wide block intersected the
    # gripper hulls on spawn and exploded the articulation (probe1). With
    # --descend-from the arm is parked high at spawn, so --pedestal can be wide.
    PED = float(args.pedestal)
    if support_top > 0.01:
        cube = UsdGeom.Cube.Define(stage, "/World/Probe/Support")
        cube.GetSizeAttr().Set(1.0)
        # dark pedestal so light objects (knife/plate) contrast in renders
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(0.20, 0.22, 0.26)])
        xf = UsdGeom.Xformable(cube.GetPrim())
        # centre the pedestal under the TCP + object (covers both footprints)
        ped_cx = (tcp_p[0] + bottle_base[0]) / 2.0 if args.descend_from > 0.0 else bottle_base[0]
        ped_cy = (tcp_p[1] + bottle_base[1]) / 2.0 if args.descend_from > 0.0 else bottle_base[1]
        xf.AddTranslateOp().Set(Gf.Vec3d(ped_cx, ped_cy, support_top / 2.0))
        xf.AddScaleOp().Set(Gf.Vec3f(PED, PED, float(support_top)))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat, materialPurpose="physics")
    # finger hull clearance report: lowest finger body z vs pedestal top
    lf_now, _ = body_pose("left_finger")
    rf_now, _ = body_pose("right_finger")
    emit(event="support", pedestal=PED, top=support_top, finger_z=[lf_now[2], rf_now[2]], clearance=min(lf_now[2], rf_now[2]) - support_top,
         root=backend.root_state() if hasattr(backend, "root_state") else None)
    def articulation_sane(tag: str) -> bool:
        g = read_gripper()
        pairs = backend.contact_pairs()
        odd = [(str(p["body_a"]).split("/")[-1], str(p["body_b"]).split("/")[-1], round(float(p["normal_force"]), 1)) for p in pairs]
        drive = g["drive_joint"][0]
        vmax = max(abs(g[n][1]) for n in GRIP)
        ok = -0.1 <= drive <= 0.95 and vmax < 0.5 and not any("Support" in a or "Support" in b for a, b, _ in odd)
        emit(event="sanity", tag=tag, ok=ok, drive=drive, max_gripper_speed=vmax, contact_pairs=odd[:12])
        return ok

    if args.no_object:
        # Free close: nothing spawned between the pads. bottle_state() and
        # reset_bottle() become no-ops (null pose / no-op reset) so the
        # shared phase-B loop below (descend/run_close/lift) can keep calling
        # them unconditionally instead of branching at every call site; every
        # object-dependent event (object, bottle_placed, the after_spawn
        # sanity check) is skipped and object-dependent metrics come back
        # null instead of describing an object that was never there.
        NULL_OBJECT_STATE = {"x": None, "y": None, "z": None, "tilt_deg": None, "zx": None, "zy": None}

        def bottle_state() -> dict[str, float]:
            return dict(NULL_OBJECT_STATE)

        def reset_bottle() -> None:
            return None

        emit(event="object", kind=None, skipped=True, reason="--no-object")
    else:
        bottle_path = "/World/Probe/Bottle"
        bprim = stage.DefinePrim(bottle_path, "Xform")
        bprim.GetReferences().AddReference(object_usda)
        # Spawn 2 cm above the support: a body that never attached to PhysX stays
        # exactly at the authored pose, a live one drops onto the support.
        DROP = 0.02
        _bxf = UsdGeom.Xformable(bprim)
        _bxf.AddTranslateOp().Set(Gf.Vec3d(bottle_base[0], bottle_base[1], bottle_base[2] + DROP))
        if args.object != "bottle":
            _bxf.AddRotateZOp().Set(float(object_yaw_deg))
        emit(event="object", kind=args.object, usda=object_usda, yaw_deg=object_yaw_deg, tool_x=tool_x, tool_y=tool_y)

    for _ in range(5):
        app.update()
    for _ in range(int(1.5 / DT)):
        backend.step()

    if not args.no_object:
        from isaaclab_physx.physics import PhysxManager

        view = PhysxManager.get_physics_sim_view().create_rigid_body_view(bottle_path)

        def bottle_state() -> dict[str, float]:
            tf = view.get_transforms()
            arr = tf.numpy() if hasattr(tf, "numpy") else tf
            arr = np.asarray(arr).reshape(-1, 7)[0]
            x, y, z, qx, qy, qz, qw = (float(v) for v in arr)
            # body z axis in world = R * (0,0,1)
            zx = 2 * (qx * qz + qw * qy)
            zy = 2 * (qy * qz - qw * qx)
            zz = 1 - 2 * (qx * qx + qy * qy)
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, zz))))
            return {"x": x, "y": y, "z": z, "tilt_deg": tilt, "zx": zx, "zy": zy}

        initial = view.get_transforms()
        init_arr = np.array(initial.numpy() if hasattr(initial, "numpy") else initial).reshape(-1, 7).copy()
        b0 = bottle_state()
        dropped = (bottle_base[2] + DROP) - b0["z"]
        emit(event="bottle_placed", source=source, bottle_base=bottle_base, support_top=support_top, settled=b0,
             dropped_m=dropped, attached=bool(dropped > 0.005),
             dist_to_pad_mid_xy=math.hypot(b0["x"] - pad_mid[0], b0["y"] - pad_mid[1]), pad_mid=pad_mid)

        if dropped < 0.005 or b0["z"] < bottle_base[2] - 0.05 or not articulation_sane("after_spawn"):
            emit(event="abort", reason="bottle not resting on support, or articulation disturbed by the spawn")
            _out.close()
            _rows.close()
            app.close()
            sys.exit(2)

        def reset_bottle() -> None:
            tf = view.get_transforms()
            arr = tf.numpy() if hasattr(tf, "numpy") else tf
            np.asarray(arr).reshape(-1, 7)[0, :] = init_arr[0, :]
            vel = view.get_velocities()
            varr = vel.numpy() if hasattr(vel, "numpy") else vel
            np.asarray(varr).reshape(-1, 6)[0, :] = 0.0
            if hasattr(tf, "numpy"):
                import warp as wp

                indices = wp.array([0], dtype=wp.uint32, device=str(tf.device))
            else:
                indices = torch.arange(1, device=tf.device, dtype=torch.int32)
            view.set_transforms(tf, indices)
            view.set_velocities(vel, indices)

    for cfg in CONFIGS:
        wait_gripper_open()
        reset_bottle()
        for _ in range(int(args.settle_s / DT)):
            backend.step()
        if not articulation_sane(f"pre_trial {cfg}"):
            emit(event="abort", reason="articulation not sane before trial; stopping the sweep")
            break
        pre = bottle_state()
        if args.descend_from > 0.0:
            d = descend(args.descend_s)
            d["object_after_descent"] = bottle_state()
            emit(event="descent", **d)
        m = run_close("B", cfg, bottle_reader=None if args.no_object else bottle_state)
        m["bottle_pre"] = pre
        if args.lift:
            if ik_family:
                lifted = planar_topdown_ik(_gx, _gz + args.lift_dz, _j7, seed=arm_pose)
            else:
                lifted = dict(arm_pose)
                lifted["joint2"] = arm_pose["joint2"] - 0.15  # shoulder up ~ raises the TCP
            # Object pose at lift start, to measure lift-induced SLIDE (does the
            # lift shove/drag the object across the surface?) separately from the
            # close-phase displacement.
            obj_ls = bottle_state()
            _ls_xy = (obj_ls["x"], obj_ls["y"])
            _slide = {"max": 0.0, "arm_v": 0.0}

            def _track_lift() -> None:
                if not args.no_object:
                    b = bottle_state()
                    _slide["max"] = max(_slide["max"], math.hypot(b["x"] - _ls_xy[0], b["y"] - _ls_xy[1]))
                _, _, v, _ = backend.joint_state()
                _slide["arm_v"] = max(_slide["arm_v"], max(abs(float(v[JIDX[j]])) for j in ARM))

            if args.lift_s > 0.0:
                # Gentle lift: interpolate the arm from the grasp pose to the
                # raised pose (mirrors descend()), then hold. Avoids the
                # step-command yank that can shear a marginal grasp loose.
                n_lift = max(1, int(args.lift_s / DT))
                for k_lift in range(1, n_lift + 1):
                    a_lift = k_lift / n_lift
                    command_arm({j: arm_pose[j] + a_lift * (lifted[j] - arm_pose[j]) for j in ARM})
                    backend.step()
                    _track_lift()
                    video_tick()
                for _ in range(int(1.5 / DT)):  # settle at the top
                    backend.step()
                    _track_lift()
                    video_tick()
            else:
                command_arm(lifted)
                for _ in range(int(2.0 / DT)):
                    backend.step()
                    _track_lift()
                    video_tick()
            tcp_after, _ = body_pose("link_tcp")
            bl = bottle_state()
            lf, rf = pad_forces()
            m["lift"] = {"tcp_dz": tcp_after[2] - tcp_p[2],
                         "bottle_dz": None if args.no_object else bl["z"] - pre["z"],
                         "bottle_tilt": bl["tilt_deg"], "hold_force": lf + rf,
                         "slide_xy_max": _slide["max"],
                         "slide_xy_end": None if args.no_object else math.hypot(bl["x"] - _ls_xy[0], bl["y"] - _ls_xy[1]),
                         "peak_arm_joint_speed": _slide["arm_v"],
                         "obj_at_lift_start": obj_ls,
                         "tcp_after": tcp_after, "object_after": bl}
            capture_frame(
                "LIFT", 0.0,
                force="dz?_tilt?" if args.no_object else f"dz{bl['z']-pre['z']:+.02f}_tilt{bl['tilt_deg']:.0f}",
            )
            # release BEFORE returning: a closed jaw descending onto an object
            # left on the pedestal jams the drive past its limit (probe6)
            wait_gripper_open()
        else:
            wait_gripper_open()
        command_arm(stage_pose)
        wait_arm(stage_pose, timeout_s=5.0)
        emit(event="phaseB", **m)

if _render["on"] and _video["on"]:
    import subprocess
    n_frames = _video["vidx"]
    mp4 = str(Path(args.render_dir) / "grasp.mp4")
    if n_frames > 0:
        cmd = [
            "ffmpeg", "-y", "-loglevel", "error",
            "-framerate", str(args.video_fps),
            "-i", str(_video_dir / "frame_%06d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", mp4,
        ]
        try:
            subprocess.run(cmd, check=True)
            emit(event="video", frames=n_frames, fps=args.video_fps, mp4=mp4)
        except Exception as _ee:
            emit(event="video_encode_error", frames=n_frames, err=str(_ee)[:200],
                 hint="frames are in " + str(_video_dir))
    else:
        emit(event="video_empty", hint="no frames captured; check --video-stride")

emit(event="done")
_out.close()
_rows.close()
app.close()
