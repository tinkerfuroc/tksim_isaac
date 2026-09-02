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
parser.add_argument("--mirror-mode", default="target", choices=("target", "measured", "measured_ff"),
                    help="target = stock mirror (followers track drive TARGET); measured = followers track the drive joint's MEASURED angle (single-DOF jaw); measured_ff = measured + one-step velocity feed-forward (q + qdot*dt)")
parser.add_argument("--max-lead", type=float, default=None, help="override backend._gripper_max_lead (0 disables the stall-gated lead clamp)")
parser.add_argument("--stall-speed", type=float, default=None, help="override backend._gripper_stall_speed")
parser.add_argument("--object", default="bottle", choices=("bottle", "knife"))
parser.add_argument("--object-usda", default="")
parser.add_argument("--tcp-above-top", type=float, default=None, help="pedestal top = tcp_z - this (bottle 0.095 CoM-height side grasp; knife 0.02 top-down)")
parser.add_argument("--object-yaw-axis", default="x", choices=("x", "y"), help="which tool axis the object's long axis is aligned to (knife)")
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
from tinker_sim_core.command_mux import JointCommand  # noqa: E402
from tinker_sim_isaac.backend import IsaacWholeRobotBackend  # noqa: E402

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
DT = backend.dt


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
arm_pose = dict(BUILTIN_POSES[args.pose])
if grasp and "arm_joints" in grasp:
    arm_pose = {j: float(grasp["arm_joints"][j]) for j in ARM}
emit(event="arm_pose", source="grasp_config" if grasp else f"builtin:{args.pose}", pose=arm_pose)
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


# ----------------------------------------------------------------- Phase A
def run_close(tag: str, cfg: dict[str, float], bottle_reader=None) -> dict[str, object]:
    backend._gripper_close_slew = cfg["slew"]
    gains = set_follower_gains(cfg["damping"], cfg["stiffness"], cfg.get("drive_stiffness"), cfg.get("drive_damping"))
    command_gripper(args.close_target)
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
        r = {
            "tag": tag, "k": k, "t": round(t, 4), "target": target, "drive": g["drive_joint"][0],
            "pad_pos": pad_pos, "pad_speed": pad_speed, "lag": lag, "lf": lf, "rf": rf,
            "tau_drive": round(g["drive_joint"][2], 3),
            "tau": {n: round(v, 3) for n, v in taus.items()},
            "pos": {n: round(g[n][0], 4) for n in FOLLOWERS},
        }
        if bottle_reader is not None:
            b = bottle_reader()
            r["bottle"] = b
            bottle_rows.append(b)
        row(**r)
        last = r
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
        "final": {k: last.get(k) for k in ("target", "drive", "pad_pos", "pad_speed", "lag", "lf", "rf", "tau_drive", "tau", "pos")},
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
        bottle_base = [tcp_p[0], tcp_p[1], tcp_p[2] - above]
        source = f"tcp_minus_{above}"
    support_top = bottle_base[2]
    object_usda = args.object_usda or args.bottle_usda or str(
        ROOT / "simulation/assets/primitives" / ("bench-bottle.usda" if args.object == "bottle" else "bench-knife.usda")
    )
    # object yaw: align its long axis (asset +x) with the tool's x or y axis in world.
    # body_quat_w comes through as XYZW here (the backend's root_state reorders
    # the same way); decoding it as wxyz put the knife 139 deg off (probe5-8).
    qx, qy, qz, w = tcp_q
    tool_x = [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy + w * qz), 2 * (qx * qz - w * qy)]
    tool_y = [2 * (qx * qy - w * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz + w * qx)]
    axis_v = tool_x if args.object_yaw_axis == "x" else tool_y
    object_yaw_deg = math.degrees(math.atan2(axis_v[1], axis_v[0]))
    # static support block with desk-like friction
    mat = UsdShade.Material.Define(stage, "/World/Probe/SupportMat")
    mapi = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    mapi.CreateStaticFrictionAttr(1.0)
    mapi.CreateDynamicFrictionAttr(1.0)
    mapi.CreateRestitutionAttr(0.0)
    # Narrow pedestal (bottle footprint only): a wide block intersected the
    # gripper hulls on spawn and exploded the articulation (probe1).
    PED = 0.10
    if support_top > 0.01:
        cube = UsdGeom.Cube.Define(stage, "/World/Probe/Support")
        cube.GetSizeAttr().Set(1.0)
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(bottle_base[0], bottle_base[1], support_top / 2.0))
        xf.AddScaleOp().Set(Gf.Vec3f(PED, PED, float(support_top)))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat, materialPurpose="physics")
    # finger hull clearance report: lowest finger body z vs pedestal top
    lf_now, _ = body_pose("left_finger")
    rf_now, _ = body_pose("right_finger")
    emit(event="support", pedestal=PED, top=support_top, finger_z=[lf_now[2], rf_now[2]], clearance=min(lf_now[2], rf_now[2]) - support_top,
         root=backend.root_state() if hasattr(backend, "root_state") else None)
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
    def articulation_sane(tag: str) -> bool:
        g = read_gripper()
        pairs = backend.contact_pairs()
        odd = [(str(p["body_a"]).split("/")[-1], str(p["body_b"]).split("/")[-1], round(float(p["normal_force"]), 1)) for p in pairs]
        drive = g["drive_joint"][0]
        vmax = max(abs(g[n][1]) for n in GRIP)
        ok = -0.1 <= drive <= 0.95 and vmax < 0.5 and not any("Support" in a or "Support" in b for a, b, _ in odd)
        emit(event="sanity", tag=tag, ok=ok, drive=drive, max_gripper_speed=vmax, contact_pairs=odd[:12])
        return ok

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
        m = run_close("B", cfg, bottle_reader=bottle_state)
        m["bottle_pre"] = pre
        if args.lift:
            lifted = dict(arm_pose)
            lifted["joint2"] = arm_pose["joint2"] - 0.15  # shoulder up ~ raises the TCP
            command_arm(lifted)
            for _ in range(int(2.0 / DT)):
                backend.step()
            tcp_after, _ = body_pose("link_tcp")
            bl = bottle_state()
            lf, rf = pad_forces()
            m["lift"] = {"tcp_dz": tcp_after[2] - tcp_p[2], "bottle_dz": bl["z"] - pre["z"], "bottle_tilt": bl["tilt_deg"], "hold_force": lf + rf,
                         "tcp_after": tcp_after, "object_after": bl}
            # release BEFORE returning: a closed jaw descending onto an object
            # left on the pedestal jams the drive past its limit (probe6)
            wait_gripper_open()
            command_arm(arm_pose)
            wait_arm(arm_pose, timeout_s=5.0)
        emit(event="phaseB", **m)

emit(event="done")
_out.close()
_rows.close()
app.close()
