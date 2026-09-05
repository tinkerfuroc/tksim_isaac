# Arena camera RTF — design

Date: 2026-08-29. Branch: `worktree-rtf-arena-camera`, off `task50-stage-a-repair`
at b8f5033 (the arena camera fast-forwarded in from
`worktree-gpsr-command-variety-spec` the same day).

## Problem

With the arena observer camera on (`TINKER_SIM_ARENA_CAMERA=1`,
`TINKER_SIM_ARENA_CAMERA_HZ=2`, sensor-rich head profile, control 60, sim on
GPU 1) the bench stack runs at **RTF 0.21–0.24**. The same stack with the
bridge attached and no arena camera runs at 0.81 idle / 0.68 driving / 0.63
arm (see `docs/developer-log.md`, 2026-08-21). Measured 2026-08-29 by the
`gpsr command testing robustness` session:

- `/clock` 479→484 s over 20.3 s wall (0.24); 189 s sim over ~900 s wall
  averaged across the first 15 min (0.21). Steady state, not spikes: two
  samples 15 min apart agree.
- GPU 1 sits at 98 % utilisation, 4.9 GB, the whole time the sim runs with
  the arena camera.
- `/clock` is not dropped (≈11 Hz publish, monotonic) and Nav2 stays healthy,
  but every navigation leg takes ~4× wall time; a 600 s run cap timed out a
  battery mid-plan with zero failures, 1500 s is what works.
- Lowering the arena rate from the 4 Hz default to 2 Hz (commit f59582c) did
  not recover much: ~0.2 with arena recording vs 0.4 head-only.

Measurement method (reuse it for every number in this work):
`timeout 20 ros2 topic echo --no-daemon /clock --field clock`, first/last
`sec` against `date +%s.%N`. Use `--no-daemon`; the ros2 daemon dies
intermittently on this box.

## What the code says

- Each camera is `RtxCamera(path, tick_rate=spec.tick_rate_hz)` wrapped in a
  `CameraSensor` (`simulation/tinker_sim_isaac/camera_rig.py:585-633`), so
  Kit does hold a per-product cadence. The pointer-change evidence behind
  a9fa951 (arena buffer fresh on 1 of 3 capture cycles at 4 Hz vs 12 Hz) says
  the arena *render* really is gated to its own rate.
- The Kit pump (`validation/run_sim.py:1169-1179`,
  `_pump_streaming_app_update`) runs `app.update()` once per camera stride
  (12 Hz) on the simulation thread with `playSimulations` off; physics is
  serialised behind it. Every alive render product participates in every
  pump, whether or not it re-renders.
- Turning the arena camera on also flips `stable_aa=True`
  (`run_sim.py:1059`), which sets `/rtx/post/aa/op` to DLAA for **all**
  render products (`camera_rig.py:534-537`) — head and wrist included. This
  was the CUDA-700 workaround for 3+ concurrent products, and it is a
  per-frame cost paid on the 12 Hz head render.
- The arena spec is 960×540, 70° HFOV, colour only (`arena_camera.py:194`);
  the rate override may only lower the 4 Hz default.
- The 2026-08-21 pump probes established that pump cost scales with pixel
  count (~19 ms head, ~7 ms wrist per frame at parity resolution) and is
  not render-mode, GI, async-rendering or readback bound.

So the ~0.6 RTF is one of, or a mix of: (a) the DLAA tax on every product,
(b) a per-product fixed cost in Kit's frame pipeline for a tick-gated but
alive product, (c) the arena product's own 960×540 render on its stride.
Nothing measured so far separates them. The design therefore starts with a
measurement, and the later phases are ordered by what it shows.

## Goal and success criteria

Arena-on RTF within ~0.1 of arena-off RTF for the same stack (target ≥0.6
driving under the bench stack), with the arena view still usable for
contact sheets / evidence (a person and a 10 cm object recognisable in a
bird's-eye frame), no change to hardware-parity cameras' resolution or
rate, and no return of the CUDA-700 crash (crash recipe from
`.superpowers/arena-cam-debug/findings-phase1.md`: sim + bridge, fault at
T+38–46 s pre-fix).

Non-goals: PhysX solver cost, the head camera's parity rate, CPU contention
from other GPUs' training jobs, and the ~15-min `simulation_interfaces` /
`controller_manager` wedge the bench session saw once (logged as a possible
render-starvation symptom; not chased here).

## Phase 0 — measurement spike (throwaway)

Sim alone on GPU 1, no bridge, sensor-rich profile, `TINKER_SIM_PROFILE=1`,
`TINKER_SIM_CONTROL_HZ=60`, camera 12 Hz, ≥60 s steady state per variant,
read `kit_pump` and `cameras` ms/cycle plus `wall` from the `step_profile`
lines (attribute by `wall_time`, never `/clock`):

| variant | isolates |
|---|---|
| A. arena off | baseline |
| B. arena on, 2 Hz (bench config) | total arena cost |
| C. B + `TINKER_SIM_CAPTURE_SKIP=arena_camera` | render vs capture/convert/publish |
| D. arena off, `stable_aa` forced on | the DLAA tax alone |
| E. arena on, 2 Hz, spec resolution 640×360 | pixel-count share of the arena render |
| F. arena on, 0.5 Hz | whether cost tracks the arena stride at all (fixed per-product cost if not) |

D needs a one-line env hook (`TINKER_SIM_STABLE_AA=1`) on `run_sim.py:1059`;
E needs the arena width/height read from env for the spike only. Both are
labelled throwaway unless Phase 2 keeps them. Output: a table in
`docs/developer-log.md` and the decision below.

Decision rule:
- If D alone costs most of B−A → Phase 1a (scope DLAA).
- If F ≈ B (cost does not track the stride) → Phase 1b (detach the product
  between ticks).
- If E recovers most of B−A → Phase 2 resolution is the main lever.
- If C ≈ A → the cost is on the consume/publish side, not the render, and
  Phase 1 is replaced by fixing `publish_cameras` for the arena stream.

## Phase 1 — structural fix (one of, chosen by Phase 0)

**1a. Scope the AA workaround.** Keep DLAA only where the CUDA-700 reproduction
needs it. Try, in order: (i) DLAA on the arena render product only
(per-render-product `/rtx/post/aa/op` via the product's settings path);
(ii) if AA is global-only, re-run the crash recipe with the a9fa951 fresh-
buffer gate and DLAA off — the stale-read race was the primary cause and
may have made DLAA redundant. Acceptance: crash recipe clean 3/3 over
≥5 min with sim + bridge, and head-camera frames byte-identical class of
output (no AA change visible to the detector: rerun
`tests/test_camera_rig*.py` plus a detector smoke on a recorded frame).

**1b. Render the arena product only on its own stride.** Rather than a
tick-gated but always-alive product, hold the arena `CameraSensor`
detached (or its render product disabled via the hydra texture / render
product `enabled` attribute) and enable it only for the pump that falls on
the arena stride, disabling it again after `capture()` consumes the buffer.
This keeps Kit's per-product pipeline out of the other 5-of-6 pumps.
Risks: product enable/disable churn can itself reallocate the annotator
pool — exactly the CUDA-700 trigger — so this must run the crash recipe
and keep the pointer-gated consume in `capture()`. If Isaac's experimental
`RtxCamera` cannot disable a product without destroying it, fall back to
destroying/recreating the sensor on the arena stride and measure the cost.

## Phase 2 — cheap wins (independent of Phase 1, ordered by the bench
session's suggestion)

1. Arena resolution 960×540 → 640×360 (`arena_camera.py:194-195`; 10 cm
   objects do not resolve at 960 px from the arena mount anyway). Update
   `tests/test_arena_camera.py` expectations and the contact-sheet layout
   if it assumes the width.
2. Default `ARENA_CAMERA_DEFAULT_HZ` 4 → 2 (`arena_camera.py:34`) so the
   bench's explicit override becomes the default; keep the "override may
   only lower" rule.
3. Head camera rate gated on subscribers: publish/render the head stream at
   the parity 12 Hz only while `/camera/...` has a subscriber, else drop to
   the arena stride. Out of scope for this branch unless Phase 0 shows the
   head render dominates; note it as follow-up.

## Phase 3 — bench policy and verification

- `scripts/gpsr-stack`: `TINKER_SIM_ARENA_CAMERA=0` for pass/fail batteries,
  arena on only for evidence runs (a `--evidence` flag or env passthrough).
- Verify with the bench session's probe on the real stack: arena-on RTF
  before/after, and confirm the battery run cap can return from 1500 s
  toward 600 s. Record the numbers in `docs/developer-log.md` (fix
  narratives go there, not the runbook); the runbook gets only the final
  knob values.
- Regression guard: a `tests/test_run_sim_arena_wiring.py` case pinning
  whatever Phase 1 chose (e.g. DLAA not set when the arena is off, or the
  arena product enabled only on its stride).

## Files

`simulation/tinker_sim_isaac/camera_rig.py`, `arena_camera.py`,
`validation/run_sim.py`, `scripts/gpsr-stack`, `tests/test_arena_camera.py`,
`tests/test_run_sim_arena_wiring.py`, `docs/developer-log.md`,
`docs/gpsr-sim-runbook.md` (knob values only).

## Constraints carried over

- Arena-enabled sims need ≥5 GB free on their card; vision shares the sim's
  GPU, cuMotion the other.
- A CUDA-poisoned sim needs SIGKILL; Nav2 never does (SHM locks) — nav down
  before sim.
- Hardware-parity cameras are not to be changed in rate or resolution.
- Subagents: Sonnet for the spike runs and rebuilds; never Opus.
