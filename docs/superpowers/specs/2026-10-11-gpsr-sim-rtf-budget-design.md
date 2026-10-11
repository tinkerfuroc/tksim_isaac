# GPSR sim RTF budget: ≥ 0.5 with the 3D raycast lidar (design)

Date: 2026-10-11. Status: approved in conversation; awaiting written-spec review.

## 1. Problem

A full GPSR stack (sensor-rich sim, Stage 2 bridge, Nav2, tk26_vision) ran at
**RTF 0.28** on 2026-10-11 (domain 211, `gpsr_stack_logs/20261011T081535`,
measured as sim-seconds per wall-second of `/clock` over 3 × 5 s windows:
0.273 / 0.281 / 0.276). The runbook's expected figure for the same stack
*without* the raycast lidar is ~0.68 while the base drives.

Cause, in order of size:

1. **The full 3D raycast lidar is now the gpsr-stack default.** An uncommitted
   change in the shared `dev` checkout (`scripts/gpsr-stack`:
   `raycast_lidar: bool = True`, `--planar-lidar` opt-out) passes
   `--raycast-lidar` with the contract's full 57 ch × 349 col = 19,893 rays at
   120 Hz physics / 60 Hz control. `docs/developer-log.md` (2026-09-10) measured
   that pattern at ~0.75–0.99 s of compute per simulated second; the only
   full-stack raycast run that reached 0.52 used 40/40 Hz and 44 channels.
2. **Host contention.** tk26_vision inference shares GPU 0 with the sim;
   loadavg ~22/32; IO pressure `some` ~50 % from unrelated recursive greps.

At RTF 0.3 the decision layer's 120 s wall-clock `GotoAction` budget cancels
moving Nav2 goals (bug report 2026-10-11; being fixed separately in
tk25_decision by the gpsr-bug-a-bug-b session — out of scope here).

### 1.1 A correctness bug found on the way: half the lidar is never published

`RaycastLidar` accumulates a frame from per-physics-step slices (sweep
schedule, 12 slices of ~30° at 120 Hz / 10 Hz). The physics-step callback is
never dispatched in the production loop (documented in `lidar_rig.py`), so the
**only** driver is `ros_gateway`'s poll, called once per **control** step. At
120/60 there are 2 physics substeps per control step, so:

- every other slice is cast (and paid for) but never read;
- `FrameAccumulator` counts polls, so a "12-step" frame spans 24 physics steps:
  `/livox/lidar` publishes at **5 Hz sim time, not 10**;
- the same even slices are read in both rotations, so alternating ~30° azimuth
  wedges are **permanently empty**.

Verified live 2026-10-11 on domain 211: frame period 0.200 s sim time; 15°
histogram bins alternate between 0 and populated frame to frame
(e.g. `[568, 699, 0, 228, 798, 684, 0, …]`).

## 2. Goal and success criteria

- **G1.** A GPSR battery on the default preset holds **RTF ≥ 0.5** (30 s
  window, base driving, full stack attached, vision on the sim's GPU as today —
  the worst case) on this
  host under its normal background load. Design target **≥ 0.55** for
  headroom.
- **G2.** `/livox/lidar` publishes at the declared 10 Hz (sim time) with every
  azimuth slice present, at any physics/control ratio.
- **G3.** Physics stays at the validated 120 Hz. Contact/grasp behaviour is
  unchanged.
- **G4.** Every run records its RTF over time; a run that drops below 0.5
  says so in its logs; a battery trial can get its mean/min RTF from one call.
  Nothing is killed on low RTF.
- **G5.** Full hardware-parity lidar remains one flag away.

Non-goals: changing physics rates; an RTX lidar; changing the lidar contract
file; editing tk25_decision (the battery runner or `GotoAction`); runtime
preset switching.

## 3. Design

Four units, each independently testable. Unit A is a prerequisite for
measuring B honestly (with A unfixed, the cloud already carries half the rays).

### Unit A — fold every physics substep (fixes §1.1)

- `backend._step_simulation` gains a post-substep hook list
  (`self._post_substep_hooks: list[Callable[[], None]]`), called after each
  `self._sim.step(...)`. The gateway registers `rig.accumulate` there when a
  lidar rig exists, instead of polling once per control step.
- The existing per-physics-step de-duplication in `RaycastLidar.accumulate`
  stays, so the (dormant) physics callback and the hook never double-fold.
- The gateway keeps one `accumulate()` call on its publish path only as a
  fallback when no hook is registered (backend without the hook attribute,
  e.g. test doubles).

Acceptance:
- Unit test with a fake backend/rig: `physics_substeps = 2, 3, 1` → exactly
  `substeps` folds per control step; a 12-slice window completes after 12
  physics steps.
- Live: `/livox/lidar` period 0.100 ± 0.005 s sim time; no persistently empty
  30° wedge across 10 frames with the robot stationary in the arena
  (wedges blocked by the robot's own body are identified once and excluded).

Cost note: this doubles `get_data()` + self-filter calls per control step at
120/60. Unit C measures that cost; the ray cost itself is unchanged (the plugin
already casts every slice).

### Unit B — lidar presets in gpsr-stack

`scripts/gpsr-stack up --lidar {battery,parity,planar}`, default **battery**.
Supersedes the uncommitted `--planar-lidar` flag (coordinate with that change's
owner before landing; this unit absorbs its `raycast_lidar` plumbing and
`--raycast-lidar` passthrough).

| preset | sim flags / env | intent |
|---|---|---|
| `battery` | `--raycast-lidar`, `TINKER_SIM_LIDAR_CHANNELS=<N>` (columns stay at the contract's 349) | default for batteries |
| `parity` | `--raycast-lidar`, contract scale | hardware-fidelity runs; ~0.3 RTF accepted |
| `planar` | no `--raycast-lidar` (occupancy-map lidar) | fastest; no height data, no dynamic obstacles |

- `<N>` is one named constant in gpsr-stack (`BATTERY_LIDAR_CHANNELS`), set by
  the Unit E measurement. Starting value **24** (budget: 1/0.55 − 1/0.68 ≈
  0.35 s/sim-s available; the rig's measured marginal cost is ~4.3e-5 s per
  ray per simulated second (32×360 → 0.50, 16×360 → 0.25), so ≈ 8,100 rays ≈
  23 channels at 349 columns).
- Columns are kept because `pointcloud_to_laserscan` flattens the cloud to
  Nav2's 2-D `/scan`; azimuth resolution is what Nav2 consumes, vertical
  channels mostly add redundant points (developer-log 2026-09-10).
- The resolved preset and lidar scale are printed in the `up` banner and in
  the sim log line `[run_sim] live raycast lidar: …` (already present).
- `TINKER_SIM_LIDAR_CHANNELS` exported by the caller wins over the preset,
  so ad-hoc A/Bs need no code change.

Acceptance: dry-run tests (`tests/test_gpsr_stack_dryrun.py`) assert each
preset's sim argv/env; default is `battery`; `--planar-lidar` is gone (or kept
as a deprecated alias for `--lidar planar` if its owner prefers).

### Unit C — lidar Python cost: measure, then fix the dominant term

The developer log attributes ~0.25 s/sim-s to the self-filter, but
`self_hit_mask` already iterates only live rays (~3,300). The cost may instead
be the plugin resolving prim paths, or `get_data()` materialising a
19,893-entry Python list per step. So:

1. Add lidar sub-laps to the existing `TINKER_SIM_PROFILE=1` breakdown:
   `lidar_get_data`, `lidar_self_mask`, `lidar_fold`, plus the plugin cost
   inferred from the A/B of `report_hit_prim_paths` on/off.
2. Optimise **only** the term that measures ≥ 0.05 s/sim-s at the battery
   preset. Candidate fixes, chosen by the measurement:
   - mask dominates → vectorise (`np.char.startswith` over the live subset,
     with a bit-identical unit test against the current implementation);
   - list materialisation dominates → read only live indices from the reading
     if the binding allows it, else document as an upstream limit.
   - plugin path resolution dominates → no in-repo fix; record it.

Acceptance: profile numbers recorded in the developer log; any optimisation
is bit-identical on a recorded reading fixture.

### Unit D — RTF monitor

New `tools/rtf_monitor.py` (rclpy; subscribes `/clock` with sensor-data QoS;
no other topics):

- Every 10 s wall, appends one line to `<run_dir>/rtf.jsonl`:
  `{"wall": <unix s>, "sim": <clock s>, "rtf_10s": x, "rtf_30s": y}`.
- When `rtf_30s < 0.5` it prints `WARN rtf_30s=0.xx < 0.50` (once per
  crossing, plus a reminder every 60 s while below). Never exits on low RTF.
- `--summarise <rtf.jsonl> --from <wall> --to <wall>` prints
  `{"mean": …, "min_30s": …, "samples": n}` for a trial window — the call a
  battery runner makes to stamp per-trial RTF. Wiring it into tk25_decision's
  runner is left to that repo's owners.
- Started by `gpsr-stack up` as its own stage after the sim gate, killed by
  `gpsr-stack down` with the rest of the group. Hard-coded threshold constant
  `RTF_WARN_THRESHOLD = 0.5`.

Acceptance: unit tests on the windowing/summarise maths with a synthetic
`(wall, sim)` series (pure functions, no rclpy); dry-run test asserts the
stage exists and is torn down.

### Unit E — validation and documentation

Live, following the shared-host rules (check GPU owners and ROS domains via
`/proc/*/environ`, `timeout -k` on every step, hard deadline):

1. **A/B matrix**, full stack attached, base driving a fixed waypoint loop,
   RTF from `rtf_monitor` (≥ 60 s each): `planar`, raycast 16 / 24 / 32 ch,
   `parity`. Both with Unit A applied. Pick `BATTERY_LIDAR_CHANNELS` as the
   largest count giving RTF ≥ 0.55.
2. **Contention probes** (one run each at the chosen preset):
   `TINKER_SIM_CPU_THREADS=16`; vision on the other GPU
   (`GPSR_VISION_GPU`). A probe becomes a gpsr-stack default only if it gains
   ≥ 0.03 RTF and doesn't take a GPU another session owns by default. Vision
   GPU stays opt-in unless that holds.
3. **Fidelity check** at the chosen preset: the dev-log dynamic-obstacle probe
   (person 1.5 m ahead: point count rises, returns to baseline on delete) and
   one doorway traversal.
4. **One bench battery** on the default preset: RTF ≥ 0.5 throughout per
   `rtf.jsonl`.

Docs: runbook "Expected RTF" table gains the preset rows and the new defaults;
developer-log entry with the matrix, profile, and the §1.1 bug.

## 4. Risks

- **24 channels may be too few for a height check** the navigation access
  logic relies on (table edges, low obstacles). Mitigation: Unit E fidelity
  check; `parity` is one flag away; channel count is a single constant.
- **Unit A raises Python cost per control step** at 120/60. If that erases
  the gain, the fallback is to fold only the substep's own slice (its
  precomputed ray-index range from the sweep schedule, ~1/12 of the rays)
  instead of norming and masking all 19,893 rows — measured in Unit C.
- **Host load varies** (other sessions' stacks, greps). G1 is defined under
  normal background load; `rtf.jsonl` makes a bad-host run identifiable
  rather than silently mis-scored.
- **Shared checkout:** Unit B overlaps an uncommitted change owned by another
  session. Land only after agreeing ownership.

## 5. Testing summary

| unit | offline tests | live check |
|---|---|---|
| A | substep fold count; window completion | 10 Hz, no missing wedges |
| B | dry-run argv/env per preset | banner + sim log line |
| C | bit-identical mask fixture (if changed) | profile sub-laps |
| D | windowing/summarise maths; dry-run stage | WARN appears when forced low |
| E | — | A/B matrix, fidelity probe, bench battery |
