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
import collections
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
parser.add_argument("--mirror-mode", default="target",
                    choices=("target", "measured", "measured_ff", "central"),
                    help="target = stock mirror (followers track drive TARGET); measured = followers track the drive joint's MEASURED angle (single-DOF jaw); measured_ff = measured + one-step velocity feed-forward (q + qdot*dt); central = virtual central command: all six gripper joints track the applied drive target c (symmetric gains + stall-gated lead clamp)")
parser.add_argument("--max-lead", type=float, default=None, help="override backend._gripper_max_lead (0 disables the stall-gated lead clamp)")
parser.add_argument("--stall-speed", type=float, default=None, help="override backend._gripper_stall_speed")
parser.add_argument("--drive-effort-limit", type=float, default=None, help="raise the drive_joint effort ceiling (Nm) to sweep the clamp force; URDF default 50. The bench close is capped at this ceiling, so this is the only way to press past 50 Nm")
parser.add_argument("--follower-effort-limit", type=float, default=None, help="cap the effort ceiling (Nm) of the five gripper mimic/follower joints; backend default 180 (ImplicitActuatorCfg effort_limit_sim for gripper_mimic). #20: the follower cap (180) out-pushing the drive cap (50/--drive-effort-limit) is a suspect for the post-clamp ratchet, so this lets a trial pin the followers at or below the drive ceiling")
parser.add_argument("--trace-contacts", action="store_true", help="record EVERY contact pair touching the probe object's body (knuckles, palm, table/pedestal -- not just the pads the backend normally monitors), with point/normal, via backend.py's TINKER_SIM_CONTACT_TRACE_BODIES; written into every row under 'trace'. Default off, no cost.")
parser.add_argument("--object", default="bottle", choices=("bottle", "knife", "plate"))
parser.add_argument("--object-usda", default="")
parser.add_argument("--object-friction", type=float, default=None,
                     help="#20 torsional-friction experiment: author/override the spawned "
                          "object's bound physics material static AND dynamic friction to this "
                          "value (binds a new material if the collider has none). Applied right "
                          "after the object reference is added, before the physics parse; a "
                          "post-play readback ('object_friction_check') confirms what PhysX "
                          "actually resolved. Default off (object keeps its authored USDA "
                          "friction).")
parser.add_argument("--pad-friction", type=float, default=None,
                     help="#20 torsional-friction experiment: override the gripper pad material's "
                          "static AND dynamic friction (backend default 1.0/1.0) via "
                          "TINKER_SIM_GRIPPER_PAD_FRICTION, set before the backend boots (the "
                          "material is authored once in _apply_gripper_friction_material() during "
                          "construction). A post-boot readback ('pad_friction_check') confirms the "
                          "authored value. Default off (pads keep 1.0/1.0).")
parser.add_argument("--object-torsional-radius", type=float, default=None,
                     help="#20 torsional-friction experiment: author PhysxSchema.PhysxCollisionAPI's "
                          "torsionalPatchRadius on the spawned object's collision prim(s) (same prim "
                          "discovery as --object-friction). The pads carry 0.01 (backend "
                          "_apply_gripper_friction_material); the object collider carries 0 (no "
                          "PhysxCollisionAPI) unless this is set -- the suspected reason the pinched "
                          "object pivots about the contact normal with no torsional resistance "
                          "(task20-decay-probe-findings.md, 'Impulse-vector hold'). Applied right "
                          "after the object reference is added, before the physics parse; a "
                          "post-play readback ('object_torsional_check') confirms what PhysX actually "
                          "resolved via a USD attribute Get(). Default off (object keeps "
                          "torsionalPatchRadius 0 / no PhysxCollisionAPI).")
parser.add_argument("--object-min-torsional-radius", type=float, default=None,
                     help="minTorsionalPatchRadius to pair with --object-torsional-radius (default "
                          "= --object-torsional-radius / 2, matching a rubber-contact-patch profile; "
                          "the pads use 0.01/0.01 i.e. min==max). Ignored unless "
                          "--object-torsional-radius is also set.")
parser.add_argument("--freeze-at-stall", action="store_true",
                     help="#20 H-CMD discriminator: during phase B, once contact exists (lf+rf > 1 N) "
                          "AND the measured drive-joint speed stays below 0.02 rad/s for 0.3 s, freeze "
                          "the close -- set the drive target and all five follower targets to their "
                          "MEASURED angles + 0.005 rad and monkeypatch backend._ramp_drive_target / "
                          "backend._mirror_gripper_mimic_targets to no-ops for the rest of the hold, so "
                          "nothing keeps advancing the command past the stall. Tests whether the pad "
                          "creep documented in task20-decay-probe-findings.md is the command's continued "
                          "advance (H-CMD) rather than a friction-anchor artifact (H-ANCHOR). Emits "
                          "'freeze_at_stall' with the frozen angles and time (or triggered=False if the "
                          "stall condition never holds for 0.3 s). Default off (byte-identical close).")
parser.add_argument("--freeze-trigger", default="stall", choices=("stall", "contact", "progress"),
                     help="Only meaningful with --freeze-at-stall. 'stall' (default) = the original "
                          "H-CMD trigger: contact (lf+rf > 1 N) AND measured drive-joint speed < "
                          "0.02 rad/s sustained for 0.3 s. This never armed in practice -- the drive "
                          "creeps continuously (~0.03 rad/s) toward its unreachable target and never "
                          "reads as stalled (see h3-result.md). 'contact' = fires on the first "
                          "genuine clamp: lf+rf > 5 N sustained for 0.3 s, independent of drive "
                          "speed. This fires too early in practice -- at first light contact "
                          "(~0.5 s, fingers still moving at ~0.24 rad/s) -- and the clamp force then "
                          "decays to ~0 (freeze2-result.md). 'progress' = the first genuine clamp "
                          "PLATEAU: fires when (a) lf+rf > 5 N has been sustained for the last 0.3 s "
                          "of sim time AND (b) the MEASURED drive-joint angle has advanced less than "
                          "--freeze-progress-eps rad over that same trailing 0.3 s window (a progress "
                          "stall, deliberately NOT the instantaneous drive speed used by 'stall', "
                          "which never reads as stalled while the drive creeps at a near-constant "
                          "rate -- see task20-decay-probe-findings.md). If the drive keeps creeping "
                          "faster than --freeze-progress-eps per 0.3 s for the whole hold, this "
                          "trigger never fires (triggered=False), same as 'stall' not arming. Same "
                          "freeze action as the other triggers either way (see --freeze-lead).")
parser.add_argument("--freeze-progress-eps", type=float, default=0.005,
                     help="Only meaningful with --freeze-at-stall --freeze-trigger progress. Max "
                          "measured drive-joint advance (rad) allowed over the trailing 0.3 s window "
                          "for that window to count as a progress stall (clamp plateau). Default "
                          "0.005 rad / 0.3 s (~0.017 rad/s); the observed creep rate is ~0.03 rad/s "
                          "(~0.009 rad / 0.3 s), so the default may never fire -- pass a looser value "
                          "such as 0.012 to see the effect of relaxing the plateau threshold.")
parser.add_argument("--freeze-lead", type=float, default=0.005,
                     help="Only meaningful with --freeze-at-stall. Amount (rad) added to each "
                          "gripper joint's measured angle to form its frozen target -- 'measured + "
                          "lead' keeps a small closing bias so the freeze doesn't immediately release "
                          "contact. 0.0 = pure hold at the measured angles (no further press). "
                          "Default 0.005.")
parser.add_argument("--friction-type", default=None, choices=("patch", "one_directional", "two_directional"),
                     help="#20 H-ANCHOR discriminator: author physxScene:frictionType on the PhysicsScene "
                          "prim before the physics parse/play (via TINKER_SIM_PHYSICS_FRICTION_TYPE, read "
                          "in IsaacWholeRobotBackend.__init__ before self._sim.reset() -- see "
                          "_apply_physics_scene_friction_type). patch = PhysX default (correlated contact-"
                          "patch friction anchors, reset every step the contact facet changes -- the "
                          "suspected zero-friction artifact on a curved convex-hull pinch); one_directional "
                          "= PhysX's deprecated single-axis model; two_directional = per-contact-point "
                          "friction with no shared patch anchor. Emits 'scene_friction_type_set' before "
                          "boot and a post-play 'scene_friction_type_check' USD-attribute readback. "
                          "Default off (scene keeps PhysX's own default, 'patch').")
parser.add_argument("--object-approximation", default=None,
                     choices=("convexHull", "convexDecomposition", "sdf", "none"),
                     help="#20 H-ANCHOR discriminator: author UsdPhysics.MeshCollisionAPI's "
                          "physics:approximation on the spawned object's MESH collision prim(s) (same "
                          "prim discovery as --object-friction), before the physics parse. Different "
                          "approximations change how PhysX generates the contact manifold, which bears "
                          "on the same patch-friction-anchor hypothesis as --friction-type. "
                          "MeshCollisionAPI only affects UsdGeom.Mesh colliders -- the built-in bench "
                          "objects (bottle: Cylinder, knife: Cube, plate: Cylinder) are ANALYTIC "
                          "primitives, not meshes, so PhysX ignores this attribute for them; a collider "
                          "that is not a UsdGeom.Mesh is reported (not authored) via "
                          "'object_approximation_check' with is_mesh=false, and the flag is otherwise a "
                          "no-op on the bottle/knife/plate bench objects -- pass --object-usda pointing "
                          "at a mesh asset to actually exercise it. For 'sdf' also sets "
                          "physxSDFMeshCollision:sdfResolution=256 on mesh colliders via "
                          "PhysxSDFMeshCollisionAPI. Emits 'object_approximation_set' before play and "
                          "'object_approximation_check' (USD readback) after. Default off (object keeps "
                          "its authored/default approximation).")
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
if args.trace_contacts:
    # The probe object always lands at /World/Probe/Bottle regardless of
    # --object (bottle/knife/plate) -- see the bprim = stage.DefinePrim(...)
    # below -- so the traced body name is always "Bottle". Must be set before
    # the backend (and its contact-report subscription) is constructed.
    os.environ["TINKER_SIM_CONTACT_TRACE_BODIES"] = "Bottle"
if args.pad_friction is not None:
    # Must be set before backend construction: _apply_gripper_friction_material()
    # fires once from IsaacWholeRobotBackend.__init__(), before self._sim.reset().
    os.environ["TINKER_SIM_GRIPPER_PAD_FRICTION"] = str(float(args.pad_friction))
    emit(event="pad_friction_set", static=float(args.pad_friction), dynamic=float(args.pad_friction))
if args.friction_type is not None:
    # Must be set before backend construction: _apply_physics_scene_friction_type()
    # fires once from IsaacWholeRobotBackend.__init__(), before self._sim.reset()
    # (PhysxSceneAPI's frictionType is uniform / parse-time-only).
    os.environ["TINKER_SIM_PHYSICS_FRICTION_TYPE"] = args.friction_type
    emit(event="scene_friction_type_set", requested=args.friction_type)
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
if args.pad_friction is not None:
    import omni.usd as _omni_usd_pf
    from pxr import UsdPhysics as _UsdPhysics_pf

    _pf_stage = _omni_usd_pf.get_context().get_stage()
    _pf_mat_prim = _pf_stage.GetPrimAtPath("/World/Tinker/PhysicsMaterials/gripper_friction")
    _pf_static = _pf_dynamic = None
    if _pf_mat_prim.IsValid():
        _pf_mapi = _UsdPhysics_pf.MaterialAPI(_pf_mat_prim)
        _pf_static = _pf_mapi.GetStaticFrictionAttr().Get()
        _pf_dynamic = _pf_mapi.GetDynamicFrictionAttr().Get()
    emit(event="pad_friction_check", static=_pf_static, dynamic=_pf_dynamic,
         bound=int(getattr(backend, "gripper_friction_bound", 0)),
         prim=str(_pf_mat_prim.GetPath()) if _pf_mat_prim.IsValid() else None)
if args.friction_type is not None:
    # Post-play readback: self._sim.reset() already ran inside the backend
    # constructor above, so this confirms what PhysX actually parsed off the
    # PhysicsScene prim (a USD attribute Get(), same readback_source caveat
    # as the object-torsional check -- there is no live PhysX-side readback
    # of frictionType exposed through the tensor API).
    import omni.usd as _ft_omni_usd
    from pxr import PhysxSchema as _ft_PhysxSchema

    _ft_scene_path = getattr(backend, "physics_scene_prim_path", None) or "/physicsScene"
    _ft_stage = _ft_omni_usd.get_context().get_stage()
    _ft_scene_prim = _ft_stage.GetPrimAtPath(_ft_scene_path)
    _ft_readback = None
    _ft_has_api = False
    if _ft_scene_prim.IsValid():
        _ft_has_api = _ft_scene_prim.HasAPI(_ft_PhysxSchema.PhysxSceneAPI)
        if _ft_has_api:
            _ft_readback = _ft_PhysxSchema.PhysxSceneAPI(_ft_scene_prim).GetFrictionTypeAttr().Get()
    emit(event="scene_friction_type_check", requested=args.friction_type,
         resolved=getattr(backend, "physics_scene_friction_type", None),
         readback=str(_ft_readback) if _ft_readback is not None else None,
         has_physx_scene_api=_ft_has_api, prim=_ft_scene_path,
         readback_source="usd_attribute")
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
    (omni.physics.tensors ArticulationView, see api.py:1991-2020 in this
    Isaac Lab/PhysX build): "projects the link's incoming joint force[s] in
    the motion direction", i.e. the actual constraint-solver output along
    each joint's motion axis -- the solver-measured force/torque, not a
    command. ``get_dof_actuation_forces`` (api.py:1963) was the other
    available getter but reads back the actuation-model's force input to the
    solver (the same class of quantity as ``applied_torque``, just at the
    tensor-API layer) rather than what the solver's joint constraint actually
    carried, so it doesn't answer #20's question. ``get_measured_joint_forces``
    does not exist on this build (grepped omni.physics.tensors/api.py and the
    isaaclab_physx articulation sources -- absent).
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


def _write_physx_max_forces_direct(ids: list[int], limit: float) -> None:
    """Re-assert the follower cap straight on the PhysX tensor view, bypassing
    the Isaac Lab wrapper (write_joint_effort_limit_to_sim_index only ever
    calls this same root_view.set_dof_max_forces once, immediately and on the
    full joint set -- see articulation.py:1679 -- so this is a redundant,
    same-call re-assertion, not a different code path; it exists so that if a
    future change interposes something between the wrapper and the view (or
    changes the wrapper to defer), the direct write here still lands).

    set_dof_max_forces takes the FULL (num_instances, num_joints) row for the
    selected env indices, not a sparse per-joint column (the vendored writer
    clones the whole data._joint_effort_limits buffer, not just the changed
    columns -- see articulation.py:1679); passing a (1, len(ids))-shaped
    tensor here would either shape-mismatch or, worse, silently overwrite
    every OTHER joint's effort limit (arm joints, drive_joint) with `limit`.
    So this reads the current full row back (already updated for `ids` by the
    wrapper call that precedes this one), patches it, and pushes the whole
    row -- and, per the Task #12 precedent (RigidBodyView needs WARP arrays,
    not torch, for tensor-API writes to actually land), uses warp arrays for
    both the payload and the indices rather than torch tensors.
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
    directly on the stage, in addition to the runtime tensor-API write.

    Generic over `ids` -- used for both the five follower/mimic joints
    (default `event`) and, for #20's drive-cap parity, the single drive_joint
    (pass `event="drive_usd_drive_authored"`); the prim-name resolution and
    DriveAPI application are identical either way.

    The runtime write (write_joint_effort_limit_to_sim_index) only touches
    the live PhysX simulation buffers; it does not update the USD prim. If
    anything ever re-parses the stage (a stage reload, or a reset that goes
    through _initialize_impl/_process_actuators_cfg again -- see articulation
    .py:3811/4127), the actuator would be rebuilt from whatever the USD prim
    says, not from this run's runtime override. Authoring the drive attribute
    now means a re-parse still carries the cap. Best-effort: the follower
    joints are mimic joints the importer dropped drives for (see the
    "gripper_mimic" comment in backend.py), so a DriveAPI may not already be
    applied -- Apply() creates it (drive_joint already has one; Apply() on an
    existing DriveAPI is a no-op re-bind). Per-joint failures are logged, not
    fatal.
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

    Without this, ImplicitActuator.compute()'s _clip_effort keeps clipping
    against the stale ImplicitActuatorCfg default (180 for gripper_mimic),
    so the per-step applied-torque telemetry the #20 analysis reads never
    shows the requested cap, and the cached value is what gets re-applied to
    the sim buffer if the actuator model is ever reinitialised. Patches both
    `effort_limit` (read every step by _clip_effort) and `effort_limit_sim`
    (read at actuator (re)construction, see Articulation._create_lab_actuator)
    since ActuatorBase keeps them as separate tensors.
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
    effort_limit_sim tensors in place (see _patch_actuator_effort_limit_cache),
    re-asserts the cap directly on the PhysX tensor view
    (_write_physx_max_forces_direct) and authors it onto each follower
    joint's USD DriveAPI (_author_usd_max_force) so a stage re-parse
    would still carry it, and reads the effective limit back both from Isaac
    Lab's data.joint_effort_limits and straight from PhysX
    (get_dof_max_forces) for the event -- #20 evidence (bit-identical 15 s
    hold physics at cap 5/50/100 vs cap-180, cap5-analysis.md) shows the
    write reaching the tensor-API buffer is NOT sufficient proof it reaches
    the solver's actual joint-drive force limit for these mimic joints; the
    physx_max_force readback is what would catch that gap. A no-op unless
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


def apply_drive_effort_limit(emit_event: bool = False, event: str = "drive_effort_limit_set") -> None:
    """Cap drive_joint's effort ceiling at --drive-effort-limit through the
    SAME direct PhysX path as apply_follower_effort_limit above (#20 round 2).

    The pre-existing path (backend._set_gripper_effort_limit) only issues the
    Isaac Lab tensor-API write (write_joint_effort_limit_to_sim_index) and
    patches the owning ImplicitActuator's `effort_limit` cache -- the SAME
    writer that #20's cap5-analysis proved was a PhysX no-op for the follower
    joints (bit-identical 15 s hold physics from cap 5 through cap 180). There
    is no a-priori reason drive_joint's actuator would be exempt from that
    gap, so this re-asserts the cap straight on the PhysX tensor view
    (_write_physx_max_forces_direct), patches BOTH `effort_limit` and
    `effort_limit_sim` on the actuator model (_patch_actuator_effort_limit_
    cache -- _set_gripper_effort_limit only touches `effort_limit`), and
    authors physics:maxForce onto drive_joint's own USD DriveAPI
    (_author_usd_max_force) so a stage re-parse still carries it. Reads the
    effective limit back both from Isaac Lab's data.joint_effort_limits and
    straight from PhysX (get_dof_max_forces) for the event, exactly like
    apply_follower_effort_limit. A no-op unless --drive-effort-limit was
    passed.
    """
    if args.drive_effort_limit is None:
        return
    limit = float(args.drive_effort_limit)
    ids = [drive_id]
    backend._default_gripper_effort_limit = limit
    backend._gripper_effort_limit_written = False
    backend._set_gripper_effort_limit(limit)
    _write_physx_max_forces_direct(ids, limit)
    _patch_actuator_effort_limit_cache(ids, limit)
    _author_usd_max_force(ids, limit, event="drive_usd_drive_authored")
    if emit_event:
        print(json.dumps({
            "event": event,
            "requested": limit,
            "indices": ids,
            "gripper_effort_limit": backend.gripper_effort_limit,
            "effective": _read_effective_effort_limits(ids),
            "physx_max_force": _read_physx_max_forces(ids),
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
    # Raise (or, for the #20 round-2 sweep, lower) the drive_joint effort
    # ceiling through the same direct PhysX max-force write + USD DriveAPI
    # authoring + readback path as the follower cap (apply_drive_effort_limit)
    # -- this is the #20 sweep: quantify the clamp-normal force / creep
    # behaviour each object geometry reaches as the drive ceiling changes.
    apply_drive_effort_limit(emit_event=True)

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
if ik_family:
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
    if not ik_family:
        raise SystemExit("--descend-from needs --tcp-xz (planar IK family)")
    stage_pose = planar_topdown_ik(_gx, _gz + args.descend_from, _j7, seed=arm_pose)
    command_arm(stage_pose)
    st = wait_arm(stage_pose)
    stage_tcp, _ = body_pose("link_tcp")
    emit(event="staged_high", settle_s=st, tcp=stage_tcp, dz=stage_tcp[2] - tcp_p[2], pose=stage_pose)


def trace_pairs() -> list[dict[str, object]]:
    """--trace-contacts: every contact pair touching the probe object's body
    (knuckles, palm, table/pedestal -- not just the pads the backend normally
    monitors), with point/normal. Empty (and free) unless --trace-contacts.

    Also carries the per-point impulse vectors, separations, and normal/
    tangential impulse decomposition (fn/ft, N.s) plus the physics dt used,
    straight from backend.contact_trace_pairs() -- Task #20's "what does the
    solver actually apply at this contact" instrumentation.
    """
    if not args.trace_contacts or not hasattr(backend, "contact_trace_pairs"):
        return []
    out = []
    for p in backend.contact_trace_pairs():
        out.append({
            "body_a": str(p["body_a"]).split("/")[-1],
            "body_b": str(p["body_b"]).split("/")[-1],
            "force": round(float(p["normal_force"]), 3),
            "n_points": int(p.get("point_count", 0)),
            "points": [[round(float(v), 5) for v in pt] for pt in p.get("points", [])],
            "normals": [[round(float(v), 5) for v in n] for n in p.get("normals", [])],
            "impulses": [[round(float(v), 6) for v in imp] for imp in p.get("impulses", [])],
            "separations": [round(float(v), 6) for v in p.get("separations", [])],
            "fn": [round(float(v), 6) for v in p.get("fn", [])],
            "ft": [round(float(v), 6) for v in p.get("ft", [])],
            "dt": float(p["dt"]) if p.get("dt") is not None else None,
        })
    return out


def descend(duration_s: float) -> dict[str, object]:
    """Interpolate stage_pose -> arm_pose in joint space, then hold 1 s; report the TCP z reached."""
    n = max(1, int(duration_s / DT))
    for k in range(1, n + 1):
        a = k / n
        command_arm({j: stage_pose[j] + a * (arm_pose[j] - stage_pose[j]) for j in ARM})
        backend.step()
        video_tick()
    for _ in range(int(1.0 / DT)):
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
    # Re-pin (and re-emit a readback) before the per-config gains write, so a
    # phase-B config that clobbers the follower/drive cap -- or an actuator
    # re-init that restores the ImplicitActuatorCfg default -- shows up in the
    # log instead of silently reverting (see apply_follower_effort_limit /
    # apply_drive_effort_limit).
    apply_follower_effort_limit(emit_event=True, event="follower_effort_limit_check")
    apply_drive_effort_limit(emit_event=True, event="drive_effort_limit_check")
    gains = set_follower_gains(cfg["damping"], cfg["stiffness"], cfg.get("drive_stiffness"), cfg.get("drive_damping"))
    if _render["on"]:
        _render["next_drive"] = -1.0  # capture from the first step of this close
    if args.freeze_at_stall:
        # Start each close un-frozen: restore whichever ramp/mirror functions
        # were active before any earlier freeze in this run (stock, or a
        # --mirror-mode/central monkeypatch) so a freeze from a prior config
        # in a --configs sweep cannot leak into this one.
        backend._ramp_drive_target = _ORIG_RAMP_DRIVE_TARGET
        backend._mirror_gripper_mimic_targets = _ORIG_MIRROR_GRIPPER_MIMIC_TARGETS
    freeze_run = 0
    frozen = False
    freeze_t = None
    freeze_targets: dict[str, float] | None = None
    # 'progress' trigger state: a trailing 0.3 s window of (contact holding,
    # measured drive angle) so the plateau check can look back at "the last
    # 0.3 s of sim time" rather than requiring one more 0.3 s ON TOP OF an
    # already-windowed condition.
    progress_window = max(1, int(round(0.3 / DT)))
    progress_contact_run = 0
    progress_drive_hist: collections.deque[float] = collections.deque(maxlen=progress_window)
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
        drive_speed_now = abs(g["drive_joint"][1])
        if args.freeze_at_stall and not frozen:
            # H-CMD discriminator (task20-decay-probe-findings.md): once the
            # trigger condition holds, freeze every gripper target at its own
            # measured angle (+ lead) and kill the ramp/mirror so nothing can
            # push the jaw further round the object's curvature for the rest
            # of the hold.
            #
            # 'stall' (original): contact exists AND the drive has stopped
            # MOVING (drive speed < 0.02 rad/s) -- the drive is what
            # _ramp_drive_target keeps advancing. Per h3-result.md this never
            # armed: the drive creeps continuously (~0.03 rad/s) toward its
            # unreachable target and never reads as stalled.
            #
            # 'contact' (#20 fix): fires on the first genuine clamp, lf+rf
            # > 5 N sustained for 0.3 s, independent of drive speed. Per
            # freeze2-result.md this fires too early (first light contact,
            # fingers still moving) and the held clamp force decays to ~0.
            #
            # 'progress' (#20 second fix): the drive's own instantaneous
            # speed never reads as stalled (it creeps at a near-constant
            # ~0.03 rad/s the whole hold -- see h3-result.md), so instead of
            # 'stall' this looks for the first genuine clamp PLATEAU: contact
            # sustained 0.3 s (same as 'contact') AND the measured drive
            # angle has advanced less than --freeze-progress-eps rad over
            # that trailing 0.3 s window. progress_drive_hist is a
            # maxlen=progress_window deque of the measured drive angle
            # appended every step below, so once it is full its two ends are
            # exactly 0.3 s apart in sim time.
            progress_drive_hist.append(g["drive_joint"][0])
            if force > 5.0:
                progress_contact_run += 1
            else:
                progress_contact_run = 0
            progress_contact_ok = progress_contact_run >= progress_window
            progress_plateau_ok = (
                len(progress_drive_hist) == progress_window
                and abs(progress_drive_hist[-1] - progress_drive_hist[0]) < args.freeze_progress_eps
            )
            if args.freeze_trigger == "contact":
                trigger_hold = force > 5.0
            elif args.freeze_trigger == "progress":
                # Both conditions are already evaluated over their own
                # trailing 0.3 s window, so this is a one-shot fire (no
                # additional freeze_run sustain needed) -- see progress_run
                # below.
                trigger_hold = progress_contact_ok and progress_plateau_ok
            else:
                trigger_hold = force > 1.0 and drive_speed_now < 0.02
            if trigger_hold:
                freeze_run += 1
            else:
                freeze_run = 0
            freeze_run_required = 1 if args.freeze_trigger == "progress" else max(1, int(round(0.3 / DT)))
            if freeze_run >= freeze_run_required:
                frozen = True
                freeze_t = t
                freeze_targets = {}
                for name, idx in zip(GRIP, GRIP_IDS):
                    measured = g[name][0]
                    frozen_value = measured + args.freeze_lead
                    freeze_targets[name] = frozen_value
                    backend._position_targets[0, idx] = frozen_value
                backend._ramp_drive_target = lambda: None
                backend._mirror_gripper_mimic_targets = lambda: None
                emit(
                    event="freeze_at_stall", tag=tag, triggered=True, t=round(freeze_t, 4),
                    trigger=args.freeze_trigger, drive_angle_at_trigger=round(g["drive_joint"][0], 5),
                    drive_speed_at_trigger=drive_speed_now,
                    frozen_targets={n: round(v, 5) for n, v in freeze_targets.items()},
                    lf=lf, rf=rf, drive_speed=drive_speed_now,
                    progress_window_advance=(
                        round(progress_drive_hist[-1] - progress_drive_hist[0], 6)
                        if args.freeze_trigger == "progress" and len(progress_drive_hist) == progress_window
                        else None
                    ),
                )
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
        r = {
            "tag": tag, "k": k, "t": round(t, 4), "target": target, "drive": g["drive_joint"][0],
            "pad_pos": pad_pos, "pad_speed": pad_speed, "lag": lag, "lf": lf, "rf": rf,
            "tau_drive": round(g["drive_joint"][2], 3),
            "tau": {n: round(v, 3) for n, v in taus.items()},
            # PhysX solver-measured joint force/torque (get_dof_projected_joint_forces),
            # alongside the pre-existing tau/tau_drive (Isaac Lab's applied_torque,
            # the actuator model's commanded value) -- see _read_physx_joint_forces.
            "physx_tau": {n: round(v, 3) for n, v in zip(GRIP, physx_taus)},
            "pos": {n: round(g[n][0], 4) for n in FOLLOWERS},
            "frozen": frozen,
        }
        if args.trace_contacts:
            r["trace"] = trace_pairs()
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
    if args.freeze_at_stall and not frozen:
        emit(event="freeze_at_stall", tag=tag, triggered=False, trigger=args.freeze_trigger,
             drive_speed_at_trigger=None)
    tail = int(0.5 / DT)
    metrics: dict[str, object] = {
        "tag": tag, "config": cfg, "gains_after_write": {k: gains.get(k) for k in ("joint_damping", "joint_stiffness", "joint_effort_limits")},
        "t_reach": t_reach, "stall_t": stall_t,
        "peak_pad_force": peak_force, "peak_pad_force_t": peak_force_t,
        "first_contact_t": first_contact_t, "first_contact_force": first_contact_force, "contact_next_force": contact_next_force,
        "max_tau_motion": max_tau_motion, "max_lag_motion": max_lag_motion,
        "median_lag_motion": float(np.median(lags_motion)) if lags_motion else None,
        "median_speed_motion": float(np.median(speeds_motion)) if speeds_motion else None,
        "median_tau_motion": float(np.median(taus_motion)) if taus_motion else None,
        "final": {k: last.get(k) for k in ("target", "drive", "pad_pos", "pad_speed", "lag", "lf", "rf", "tau_drive", "tau", "physx_tau", "pos")},
    }
    if args.freeze_at_stall:
        metrics["freeze_at_stall"] = {
            "triggered": frozen, "t": freeze_t, "trigger": args.freeze_trigger,
            "targets": {n: round(v, 5) for n, v in freeze_targets.items()} if freeze_targets else None,
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


# Captured once, after every --mirror-mode/central monkeypatch above has had
# its chance to run, so --freeze-at-stall restores the RIGHT "unfrozen"
# baseline at the top of each run_close() call (stock ramp/mirror, or
# whichever mode this invocation selected) rather than always the class
# default.
_ORIG_RAMP_DRIVE_TARGET = backend._ramp_drive_target
_ORIG_MIRROR_GRIPPER_MIMIC_TARGETS = backend._mirror_gripper_mimic_targets

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
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()

    def _collision_prims(root_prim) -> list:
        """All descendant prims (root included) carrying UsdPhysics.CollisionAPI."""
        return [p for p in Usd.PrimRange(root_prim) if p.HasAPI(UsdPhysics.CollisionAPI)]

    def _direct_binding_target(prim) -> str | None:
        """Path of the material this prim itself (not an ancestor) binds, or None.

        PhysX resolves the physics material from the binding on the collision
        prim or its NEAREST bound ancestor -- distinguishing the two matters
        for #20 (is our override landing on the collider itself?).
        """
        rel = UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel(materialPurpose="physics")
        targets = rel.GetTargets()
        return str(targets[0]) if targets else None

    def _author_object_friction(root_prim, value: float, tag: str) -> None:
        for cprim in _collision_prims(root_prim):
            binding_api = UsdShade.MaterialBindingAPI.Apply(cprim)
            bound_mat, _ = binding_api.ComputeBoundMaterial(materialPurpose="physics")
            if bound_mat and bound_mat.GetPrim().IsValid():
                mat_prim = bound_mat.GetPrim()
                mapi = UsdPhysics.MaterialAPI.Apply(mat_prim)
            else:
                new_mat = UsdShade.Material.Define(
                    stage, cprim.GetPath().AppendChild("FrictionOverrideMat")
                )
                mapi = UsdPhysics.MaterialAPI.Apply(new_mat.GetPrim())
                binding_api.Bind(new_mat, materialPurpose="physics")
                mat_prim = new_mat.GetPrim()
            mapi.CreateStaticFrictionAttr(value)
            mapi.CreateDynamicFrictionAttr(value)
            emit(
                event=f"object_friction_{tag}",
                static=value,
                dynamic=value,
                prim=str(cprim.GetPath()),
                bound_material=str(mat_prim.GetPath()),
                direct_binding=_direct_binding_target(cprim) == str(mat_prim.GetPath()),
            )

    def _check_object_friction(root_prim, tag: str) -> None:
        for cprim in _collision_prims(root_prim):
            bound_mat, _ = UsdShade.MaterialBindingAPI(cprim).ComputeBoundMaterial(materialPurpose="physics")
            static_v = dynamic_v = None
            mat_path = None
            if bound_mat and bound_mat.GetPrim().IsValid():
                mat_path = str(bound_mat.GetPrim().GetPath())
                mapi = UsdPhysics.MaterialAPI(bound_mat.GetPrim())
                static_v = mapi.GetStaticFrictionAttr().Get()
                dynamic_v = mapi.GetDynamicFrictionAttr().Get()
            direct_target = _direct_binding_target(cprim)
            emit(
                event=f"object_friction_{tag}",
                static=static_v,
                dynamic=dynamic_v,
                prim=str(cprim.GetPath()),
                bound_material=mat_path,
                direct_binding=(direct_target == mat_path) if mat_path else None,
            )

    def _author_object_torsional(root_prim, radius: float, min_radius: float, tag: str) -> None:
        try:
            from pxr import PhysxSchema
        except ImportError as error:
            emit(event=f"object_torsional_{tag}", error=f"PhysxSchema import failed: {error}")
            return
        for cprim in _collision_prims(root_prim):
            collision = PhysxSchema.PhysxCollisionAPI.Apply(cprim)
            collision.CreateTorsionalPatchRadiusAttr(radius)
            collision.CreateMinTorsionalPatchRadiusAttr(min_radius)
            emit(
                event=f"object_torsional_{tag}",
                torsional_patch_radius=radius,
                min_torsional_patch_radius=min_radius,
                prim=str(cprim.GetPath()),
            )

    def _check_object_torsional(root_prim, tag: str) -> None:
        try:
            from pxr import PhysxSchema
        except ImportError as error:
            emit(event=f"object_torsional_{tag}", error=f"PhysxSchema import failed: {error}")
            return
        for cprim in _collision_prims(root_prim):
            has_api = cprim.HasAPI(PhysxSchema.PhysxCollisionAPI)
            radius_v = min_radius_v = None
            if has_api:
                collision = PhysxSchema.PhysxCollisionAPI(cprim)
                radius_v = collision.GetTorsionalPatchRadiusAttr().Get()
                min_radius_v = collision.GetMinTorsionalPatchRadiusAttr().Get()
            emit(
                event=f"object_torsional_{tag}",
                has_physx_collision_api=has_api,
                torsional_patch_radius=radius_v,
                min_torsional_patch_radius=min_radius_v,
                prim=str(cprim.GetPath()),
                # This is a USD-attribute readback (Get() on the authored
                # PhysxCollisionAPI attrs), which is authoritative for what
                # PhysX will parse on the next reset/attach. There is no
                # shape-level tensor/physx-API readback of torsional radius
                # exposed through isaaclab_physx's RigidBodyView/tensor API
                # (unlike e.g. the joint-force API used elsewhere in this
                # probe) -- if one existed it would be the harder proof; USD
                # readback only confirms what was AUTHORED, not what a live
                # PxShape resolved internally.
                readback_source="usd_attribute",
            )

    def _author_object_approximation(root_prim, approximation: str, tag: str) -> None:
        for cprim in _collision_prims(root_prim):
            is_mesh = cprim.IsA(UsdGeom.Mesh)
            if not is_mesh:
                # UsdPhysics.MeshCollisionAPI's physics:approximation only
                # governs how PhysX cooks a UsdGeom.Mesh collider into a
                # contact shape; the built-in bench objects (bottle/plate:
                # UsdGeom.Cylinder, knife: UsdGeom.Cube) are ANALYTIC
                # primitives, so PhysX synthesizes their collision shape
                # directly and this attribute has no effect. Report and skip
                # rather than author a schema PhysX will ignore.
                emit(
                    event=f"object_approximation_{tag}",
                    is_mesh=False,
                    geom_type=cprim.GetTypeName(),
                    prim=str(cprim.GetPath()),
                    skipped=True,
                    reason="collider is not a UsdGeom.Mesh; physics:approximation has no effect on an analytic collider",
                )
                continue
            mesh_api = UsdPhysics.MeshCollisionAPI.Apply(cprim)
            mesh_api.CreateApproximationAttr(approximation)
            sdf_resolution = None
            if approximation == "sdf":
                try:
                    from pxr import PhysxSchema

                    sdf_api = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(cprim)
                    sdf_api.CreateSdfResolutionAttr(256)
                    sdf_resolution = 256
                except ImportError as error:
                    emit(event=f"object_approximation_{tag}", error=f"PhysxSchema import failed: {error}")
            emit(
                event=f"object_approximation_{tag}",
                is_mesh=True,
                geom_type=cprim.GetTypeName(),
                approximation=approximation,
                sdf_resolution=sdf_resolution,
                prim=str(cprim.GetPath()),
                skipped=False,
            )

    def _check_object_approximation(root_prim, tag: str) -> None:
        for cprim in _collision_prims(root_prim):
            is_mesh = cprim.IsA(UsdGeom.Mesh)
            approx_v = None
            sdf_resolution_v = None
            has_mesh_api = cprim.HasAPI(UsdPhysics.MeshCollisionAPI)
            if has_mesh_api:
                approx_v = UsdPhysics.MeshCollisionAPI(cprim).GetApproximationAttr().Get()
            if is_mesh:
                try:
                    from pxr import PhysxSchema

                    if cprim.HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                        sdf_resolution_v = PhysxSchema.PhysxSDFMeshCollisionAPI(cprim).GetSdfResolutionAttr().Get()
                except ImportError:
                    pass
            emit(
                event=f"object_approximation_{tag}",
                is_mesh=is_mesh,
                geom_type=cprim.GetTypeName(),
                has_mesh_collision_api=has_mesh_api,
                approximation=approx_v,
                sdf_resolution=sdf_resolution_v,
                prim=str(cprim.GetPath()),
                # USD-attribute readback, same caveat as _check_object_torsional:
                # authoritative for what was AUTHORED, not a live PhysX-side
                # resolution (no tensor/physx-API readback of approximation
                # is exposed through isaaclab_physx's RigidBodyView).
                readback_source="usd_attribute",
            )

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
        if args.object_friction is not None:
            # Author before the physics parse (the app.update() loop below triggers
            # it) so PhysX picks up the override on first parse rather than a
            # runtime material swap.
            _author_object_friction(bprim, float(args.object_friction), "set")
        if args.object_torsional_radius is not None:
            _radius = float(args.object_torsional_radius)
            _min_radius = (
                float(args.object_min_torsional_radius)
                if args.object_min_torsional_radius is not None
                else _radius / 2.0
            )
            # Author before the physics parse, same rationale as --object-friction
            # above (PhysxCollisionAPI is a shape-level schema, not a material --
            # PxShape::setTorsionalPatchRadius refuses while simulation is running,
            # per the plugin's own error strings, so this must land before reset).
            _author_object_torsional(bprim, _radius, _min_radius, "set")
        if args.object_approximation is not None:
            # Author before the physics parse, same rationale as --object-friction
            # above (MeshCollisionAPI is a shape-level schema PhysX reads at parse
            # time).
            _author_object_approximation(bprim, args.object_approximation, "set")
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
        if args.object_friction is not None:
            # Post-play readback: confirm what PhysX actually resolved (collider's
            # own binding vs. an inherited ancestor binding).
            _check_object_friction(bprim, "check")
        if args.object_torsional_radius is not None:
            # Post-play readback: confirm the authored attrs survived the physics
            # parse/reset (USD attribute Get(); see _check_object_torsional's note
            # on why this isn't a live PhysX-side readback).
            _check_object_torsional(bprim, "check")
        if args.object_approximation is not None:
            # Post-play readback: confirm the authored attrs survived the physics
            # parse/reset, or that a non-mesh collider was correctly reported and
            # skipped.
            _check_object_approximation(bprim, "check")
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

    if not args.no_object:
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
