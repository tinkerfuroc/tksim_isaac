# Grip-Force Stiffness Parity — Spec (Task #20)

> Status: DRAFT for user review. Scope chosen by the user 2026-09-04:
> **stiffness parity tune** (no runtime force-servo loop). This spec designs the
> tune and, crucially, names the one acceptance measurement that sets its
> numbers — it does **not** pre-pick a stiffness value, because the wide-object
> force-vs-stiffness data has not been measured yet.

## Problem

The gripper's steady clamp force is geometry-dependent: a bottle holds at ~20 N
but wide YCB objects report only 4–9 N (Task #20). Retention suffers on the
low-force objects.

## Root cause (evidence-complete, 2026-09-04)

Steady pad normal force is set by the **follower finger PD compliant-contact
equilibrium** — specifically by follower **stiffness** — not by drive torque and
not by press depth. Measured on a bottle side grasp (`validation/gripper_close_probe.py`,
`--mirror-mode central`, phase-B 6-config sweep × `--max-lead` sweep; full table
in the `task20-grip-force-lever` memory and `$JOB/tmp/task20-lever-findings.md`):

| Lever | Change | Effect on steady pad force |
|---|---|---|
| Follower **stiffness** | 1500 → 500 | **24 N → 13 N** (both retained) — the force lever |
| **Lead** / press depth | 0.020 → 0.040 | **none** (bit-identical); 0.005 can lock the grip open |
| Follower **damping** | 55 → 20 → 5 | peak 33→50→64 N, held → dropped → object flips 159° (the close-punch) |
| Drive torque (prior sweep) | tau 50→113 | **none** (follower torques + pad forces bit-identical) |

Two corollaries that shape the design:
- **Contact is intrinsically asymmetric**: even on the bottle the drive-side pad
  reads ~4 N against ~20 N on the free pad (object rolls to the free pad). The
  weak-pad number sits in the reported "4–9 N" band — so the low-force symptom is
  partly the weak side of an asymmetric grasp, which higher follower stiffness
  lifts on both pads.
- **Fixed stiffness gives geometry-dependent force** (force ≈ stiffness ×
  effective contact overtravel, mediated by the knuckle→pad lever). A single
  stiffness that lands a narrow object at 24 N will land a wide object lower. The
  tune must therefore be validated across the object-width range, and may need to
  schedule stiffness by commanded aperture if one flat value cannot cover the band.

## Requirements

- **R1 — Force band.** Steady clamp force lands in a target band (proposed
  **20–25 N**, toward the 30 N hardware spec) across the benchmark object set
  (narrow: bottle/knife; wide: the YCB objects that currently read 4–9 N).
- **R2 — No punch, no regression.** Follower damping stays high (~55) so the
  close transient stays bounded (peak ≲ ~35 N, object tilt small); bottle and
  knife retain exactly as at commit 4356f03 (bottle 7/7-class, knife top-down 3/3).
- **R3 — Default-off, flag-gated.** New stiffness (or stiffness schedule) is gated
  by an env flag (proposed `TINKER_SIM_GRIPPER_FORCE_PARITY`, default `0`).
  Flag-off is byte-for-byte the current live model (followers k=1500 / d=55), so
  the live GPSR stack and existing campaigns are unaffected until validated.
- **R4 — Probe-gated before live.** The tune is validated in the headless probe
  (bottle + knife + wide-YCB, force + retention) before any live-stack use.
- **R5 — No URDF / robot.usd edit.** Runtime gain writes only
  (`write_joint_stiffness_to_sim_index`), the mechanism already used for the
  follower gains. `drive_joint` stays the commanded/published joint, range 0..0.85.
- **R6 — Lead guard retained.** Keep the stall-gated lead clamp as an anti-overrun
  guard at a small non-zero value (≥0.015); it is explicitly **not** a force knob.

## Design

Set the **follower finger stiffness** (`left_finger_joint`, `right_finger_joint`,
and the inner/outer knuckle followers) to the value that lands steady force in the
R1 band, holding **damping at 55** (R2). Two candidate shapes, decided by the
acceptance data:

1. **Flat stiffness** (preferred if it fits): one `k_follow` value for all grasps.
   Simplest; the tune is a single constant behind the flag.
2. **Aperture-scheduled stiffness** (fallback if a flat value can't cover the
   band): `k_follow = f(commanded aperture)` — higher stiffness at wider aperture
   (less overtravel) to hold force roughly constant across object width. Still a
   static schedule chosen at grasp time from the commanded `drive_joint` target;
   **no runtime force feedback** (that closed-loop servo is the deferred follow-up,
   out of scope here).

Damping and the drive-side gains are unchanged from the live model. This is a
gain-parameter change only; no coupling change, no new control path.

## The acceptance measurement (sets the numbers — run before finalizing)

Headless probe on GPU 1 (coordinate with the grasp-bench session), for
bottle (side), knife (top-down cross-width), and the wide YCB object(s) that
currently under-force:

- Sweep follower stiffness (e.g. 1000 / 1500 / 2500 / 4000) at damping 55,
  measuring steady pad force (both pads) and post-lift retention per object.
- Choose `k_follow` (flat) that lands **all** objects in 20–25 N with retention.
  If none does, record the per-aperture force curve and define the schedule (shape
  2) that does. Record the chosen value/schedule and the per-object force+hold in
  `$JOB/tmp/task20-parity-results.md`.

This measurement is a precondition for filling in the concrete numbers; the spec
deliberately leaves `k_follow` symbolic until it runs.

## Honest limitation

A position-controlled jaw cannot make pad force perfectly object-independent —
force follows stiffness × overtravel. The tune narrows the spread into a usable
band; if the wide-YCB spread proves too large for even an aperture schedule to fit
20–25 N without punching narrow objects, that is the signal to escalate to the
deferred closed-loop force servo (documented follow-up, not this task).

## Non-goals

- Runtime closed-loop force-target servo (deferred follow-up).
- Central-actuator coupling / the plate rim grasp — that fixes the freeze/asymmetry
  and is an off-CoM lever-arm problem, tracked separately
  (`.claude/plans/toasty-drifting-bumblebee.md`), not force parity.
- Bridge-facade heal on the main checkout (Task #10, separate).
- Any URDF / USD edit.

## Verification (acceptance)

- Probe, GPU 1: bottle + knife + wide-YCB with the flag on → all in the R1 band,
  retained, no punch (R1, R2, R4).
- `pytest tests/test_manipulation_runtime.py -k gripper` (ROS env sourced).
- Flag-off run reproduces the 4356f03 bottle/knife numbers (R3).
