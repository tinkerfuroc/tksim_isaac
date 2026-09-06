# Developer log

Dated engineering notes: what was measured, what was ruled out, why a fix
took the shape it did. Operational instructions live in
`docs/gpsr-sim-runbook.md`; this file is the history behind them.

## 2026-09-06 — Task #20: gripper stall-freeze with force-floor press (post-clamp creep / retention)

**Mechanism.** #20's grasp-force investigation (`task20-decay-probe-findings.md`,
`task20-lever-findings.md`) chased followers' stiffness/damping, drive-effort
caps, contact-friction material, PhysX friction-anchor type (patch vs
two-directional), and convex-hull-vs-SDF contact generation as candidate
causes of a slow post-clamp creep that rolls or squeezes an object out of
the jaw before lift. Every one of those levers was falsified with an H3
probe chain (`$TMP/h3-result.md`): friction type and contact-generation
changes left the trajectory identical or worse. The actual mechanism is the
close COMMAND: `_ramp_drive_target`'s bounded-lead clamp stops the drive
from racing to its unreachable fully-closed target only quasi-statically --
while the pads are still slowly creeping around a curved object's contact
patch, the unreachable target keeps dragging the whole six-joint mimic
linkage (drive_joint + the five `_gripper_mimic_indices` followers) forward,
walking the pinch circumferentially off the object over several seconds
until the grasp slips on lift. A contact-triggered freeze (`freeze2-
result.md`: all six targets frozen the instant pad force exceeds 5N) fully
arrested the creep (drive flat, tilt <1 deg for 15s, world separation
pinned) -- confirming the command-advance hypothesis (H-CMD) -- but fired
too early, before real squeeze built, and clamp force decayed to ~0N by 2s:
retained tilt/geometry, but the bottle was left on the table on lift.

**Freeze evidence.** A plateau-gated retry (`freeze3-result.md`: freeze on
sustained contact force AND negligible drive-angle progress over a trailing
0.3s window, not first light contact) fired later, in real contact, and all
three parameter legs tried (progress eps 0.005/0.012, lead 0.02/0.04) held
flat and were RETAINED ON LIFT (tilt 0.5-6 deg, dz 0.3-3.8mm) -- the first
retained lift of the whole investigation. But a one-shot static lead offset
relaxed to a near-zero (0-3N) sustained force in every leg: the k=1500
position-PD followers converge to within 1e-4 rad of the frozen target
(the compliant multi-link mimic linkage absorbs essentially all of the
commanded lead), so retention there was geometric (the creep stopped, so
the object simply wasn't pushed off), not force-held. freeze3-result.md #4's
proposed fix -- a bounded closed-loop nudge that keeps advancing the frozen
target in small steps while force stays below a floor, instead of one
static offset -- is what's implemented below.

**Parity model.** The real xArm gripper's motor pushes toward its
commanded target until its own current/force limit stalls it, then HOLDS --
it does not keep commanding motion past that limit the way an open-loop sim
ramp does. `simulation/tinker_sim_isaac/backend.py` now implements that
hold-at-stall behaviour as a small state machine wrapping the drive
ramp/mimic path (new `_gripper_stall_freeze_gate`, called from `step()` in
place of the previous unconditional `_ramp_drive_target()` +
`_mirror_gripper_mimic_targets()` pair; both env-config and the state fields
are initialized once in `__init__` next to the existing gripper-close-shaping
block):

- **CLOSING** (default): unchanged -- `_ramp_drive_target` slews/clamps the
  drive target and `_mirror_gripper_mimic_targets` mirrors the MEASURED
  drive angle into the five followers (#19 semantics, untouched). A new
  `_gripper_stall_freeze_check_plateau` runs alongside it each tick,
  appending the measured drive angle to a trailing
  `TINKER_SIM_GRIPPER_STALL_DWELL_S`-second deque and counting consecutive
  ticks where the pad-force sum (`_gripper_grip_force()` -- the same
  `contact_state()` left_finger + right_finger quantity the facade reads on
  `/sim/parity/finger_contact`) exceeds `TINKER_SIM_GRIPPER_STALL_CONTACT_N`.
- **PLATEAU -> PRESS**: once that window is full, the force has been
  sustained for its entire length, the measured drive angle advanced less
  than `TINKER_SIM_GRIPPER_STALL_EPS` rad end-to-end across it, AND the
  commanded target is still ahead of the measured angle (the ramp is still
  trying to close further but the object is what's stopping it), the six
  targets (drive_joint + the five mimic followers) freeze at their OWN
  measured angles (not the stale far-away command) and the state becomes
  PRESS. A `"plateau"` transition is logged, immediately followed by a
  `"press"` transition at the same tick.
- **PRESS**: `_gripper_stall_freeze_press_tick` advances all six frozen
  targets together by `TINKER_SIM_GRIPPER_PRESS_STEP` rad per control tick
  while the pad-force sum stays below `TINKER_SIM_GRIPPER_HOLD_FORCE_N`, up
  to `TINKER_SIM_GRIPPER_PRESS_CAP` rad of total extra travel -- the bounded
  closed-loop nudge freeze3-result.md #4 called for, in place of a one-shot
  static lead.
- **HOLD**: once the force floor or the travel cap is reached, targets
  freeze for good (a `"hold"` transition is logged); nothing in the ramp or
  mirror writes them again. `command_target_state()` reads `_position_targets`
  directly, so the facade's target echo is the frozen/pressed value for free
  -- no separate echo path was needed.
- **RELEASE**: any new `drive_joint` position command while PRESS/HOLD is
  active -- an opening command, or another close to a different target --
  exits back to CLOSING immediately (no dwell), logs a `"release"`
  transition, and clears the plateau/press bookkeeping so the next close
  starts clean. Repeating the identical held command does not release.

Each transition logs one JSON line:
`{"event": "gripper_stall_freeze", "state": "plateau|press|hold|release",
"t": <sim_time>, "drive": ..., "pad_force_n": ..., "travel": ...}`.

**Env knobs** (all optional, defaults match the accepted freeze3-result.md
config): `TINKER_SIM_GRIPPER_STALL_FREEZE` (default `"1"`; `"0"` reproduces
pre-#20 behaviour exactly -- `_gripper_stall_freeze_gate`'s first check calls
`_ramp_drive_target()` + `_mirror_gripper_mimic_targets()` unconditionally
and returns before the state machine is ever reached), `TINKER_SIM_GRIPPER_
STALL_CONTACT_N` (5.0 N), `TINKER_SIM_GRIPPER_STALL_DWELL_S` (0.3 s sim),
`TINKER_SIM_GRIPPER_STALL_EPS` (0.012 rad), `TINKER_SIM_GRIPPER_PRESS_STEP`
(0.002 rad/tick), `TINKER_SIM_GRIPPER_HOLD_FORCE_N` (25.0 N),
`TINKER_SIM_GRIPPER_PRESS_CAP` (0.06 rad total).

**Tests** (`tests/test_manipulation_runtime.py`, backend-double pattern, new
`_gsf_backend()` helper building a 6-joint drive+mimic double): six new
cases -- flag off calls ramp+mirror only, no state machine, no plateau
check; plateau requires dwell+eps+contact together (neither alone fires
it); PRESS advances the step until the force mock reaches the hold floor,
then holds; the press cap stops travel even when force never rises; an
opening command releases HOLD back to CLOSING (and an identical repeated
command does not); and the drive-target echo during HOLD reflects the
frozen/pressed value, not the original close command. All six fail against
main (`2c1b51d`) with, e.g.:
```
AttributeError: 'IsaacWholeRobotBackend' object has no attribute '_gripper_stall_freeze_gate'. Did you mean: '_gripper_stall_freeze_enabled'?
```
(confirmed by temporarily reverting only `backend.py` and rerunning `-k
gripper_stall_freeze`, then restoring). Full suite:
`tests/test_manipulation_runtime.py` 115 passed, 3 subtests passed, 0
failed (109 passed pre-existing + 6 new).

**Acceptance plan** (staged, not run from this worktree -- no GPU/sim
launch here): `$TMP/stallfreeze_chain.sh` re-runs main's
`validation/gripper_close_probe.py` bottle side-pinch close
(`--pose side --object bottle --tcp-above-top 0.095 --record-s 15 --phase B
--lift --mirror-mode target`, control cfg damping/stiffness/drive-stiffness
1.5:55:1500) with the flag on vs off, plus a third leg with the flag on for
a YCB sugar_box, to confirm on the actual probe pipeline (not just the unit
tests) that PRESS restores real hold force (target 15-30N sustained, per
freeze3-result.md's "next change") while keeping tilt <5 deg and the object
retained through lift -- the two criteria the freeze3.md legs split between
(geometric retention without sustained force). `$TMP/stallfreeze_launch.sh`
wraps that chain with `SF_EXIT=`/`SF_CHAIN_DONE` markers for a follow-up GPU
session to run.

## 2026-09-06 — Task #20 round 2: PRESS is peak-hold, not press-to-force-floor (post-round-1 acceptance)

**Round-1 result** (`$TMP/stallfreeze-result.md`, `task20-decay-probe-findings.md`
"acceptance round 1"): the bottle leg's PRESS ran unconditionally to the
fixed `TINKER_SIM_GRIPPER_PRESS_CAP` (0.06 rad) in 0.26s, and the pad-force
sum COLLAPSED from a 19.7N plateau to 4.6N -- retained on lift (dz +4mm, no
fall) but tilt 7-8 deg and hold force only 1.7-5N, both outside the accepted
band. The finding: pressing PAST the plateau's own peak is exactly the
motion that slides the pads around the object -- the maximum achievable
static pinch is at the plateau itself (while the PD error is still loaded);
advancing further only helps if force is still genuinely climbing, and any
further travel once force starts falling is pure overshoot.

**Fix.** `_gripper_stall_freeze_press_tick` (backend.py) no longer presses
to a fixed force floor or fixed cap unconditionally. It is now PEAK-HOLD:
each control tick, the six frozen targets advance by
`TINKER_SIM_GRIPPER_PRESS_STEP` only while the pad-force sum -- smoothed
over the trailing two ticks, to filter single-tick contact noise -- is
still rising (flat counts as rising, so a force that never moves still
walks the travel cap instead of stalling forever) versus the previous
tick's smoothed value. The moment two CONSECUTIVE ticks show the smoothed
force strictly falling, or the smoothed force reaches
`TINKER_SIM_GRIPPER_HOLD_FORCE_N`, PRESS stops, backs the six targets off
by one `PRESS_STEP` (restoring exactly the previous tick's targets, since a
non-rising tick never advances), and transitions to HOLD -- landing one
increment short of whatever pushed the force past its own peak, instead of
continuing to drive through it. `TINKER_SIM_GRIPPER_PRESS_CAP`'s default
dropped from 0.06 to 0.02 rad: it is now a hard safety bound only (entered
directly, no backoff, since hitting it while still rising is not an
overshoot signal), not the routine stopping point PRESS used to reach on
every close. Both exits log the transition with the current smoothed force
(`pad_force_n`), the peak smoothed force this PRESS ever reached
(`peak_force_n`), the final `travel`, and a `reason` (`"target"`,
`"decrease"`, or `"cap"`).

**Tests** (`tests/test_manipulation_runtime.py`): the old
`test_gripper_stall_freeze_press_advances_to_hold_force` (press-to-force-
floor) is replaced by
`test_gripper_stall_freeze_press_peak_hold_backs_off_on_decrease`, which
feeds a mocked pad-force series that rises then falls (raw 5,10,15,20,18
smooths to a still-rising 5/7.5/12.5/17.5/19.0, then 14,10 smooths to a
falling 16.0/12.0) and asserts PRESS stops at travel 0.008 -- one
`PRESS_STEP` back from the 0.010 peak -- not the old fixed-cap value. This
test FAILS against 1b502fb (pre-round-2, confirmed by temporarily
reverting only `backend.py` and rerunning `-k
test_gripper_stall_freeze_press_peak_hold_backs_off_on_decrease` before
restoring the fix):
```
AssertionError: 0.06 != 0.008 within 6 places (0.052 difference)
```
(1b502fb's press-to-force-floor logic ran unconditionally to the fixed cap
since the mocked force never reaches the 25N hold floor.) A new
`test_gripper_stall_freeze_press_backs_off_at_hold_force_target` covers the
other backoff path (reaching `HOLD_FORCE_N` while still rising also backs
off one step, not just a sustained decrease). `test_gripper_stall_freeze_
press_cap_stops_travel` (kept, unchanged assertions) still passes: a force
pinned constant is "still rising" (non-strict) every tick, so it walks the
cap exactly as before. Full suite: `tests/test_manipulation_runtime.py` 118
passed, 3 subtests passed, 0 failed (via the pytest-suite-ros-env-
incantation memory: no `set -u`, a lark-only symlink dir + `/opt/ros/humble/
local/lib/python3.10/dist-packages` on `PYTHONPATH`,
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`).

**Staged acceptance leg (c) fix.** Round 1's YCB sugar_box leg never
exercised the fix: it spawned already tipped 94 deg
(`$TMP/stallfreeze_box_full.log`), aborting before any close. Root-caused
offline (no GPU) with a standalone `pxr` bbox read of the referenced
sugar_box asset (`Usd.Stage.Open` + `UsdGeom.BBoxCache`, no Isaac/renderer
needed): local/world AABB (identity xform) x:[-0.0322,0.0173]
(0.0495m), y:[-0.0638,0.0304] (0.0942m -- the "0.094m face" across the
probe's Y closing axis), z:[0.00003,0.1760] (origin at the geometric
bottom, off-center in x/y). Round 1's `--object-offset 0.0075,0.0167`
already compensated that off-center origin correctly (matches the measured
centroid offset to <0.1mm) -- not the bug. Round 1 also kept `--object
bottle` even though `--object-usda` pointed at the sugar_box, so the
probe's `if args.object != "bottle": _bxf.AddRotateZOp(...)` guard silently
skipped the (unused) auto-computed 179 deg yaw; checked whether that
mattered and it did not (identity already puts local Y, 0.094m, onto world
Y, the confirmed closing axis) -- but round 2 pins it down explicitly
(`--object plate` to activate the yaw-apply branch + `--object-yaw-deg 0`)
instead of relying on an accidental skip. The actual likely cause: the
probe centers its square support pedestal on the object's raw ORIGIN, not
its footprint centroid, so covering the asymmetric Y reach (0.0638m one
side) with `--pedestal 0.14` (half-width 0.07) also reaches 0.07m in X --
putting the pedestal's near edge just 1.4mm from the finger pivot x
logged in `staged` (0.4733m), plausibly grazing the open gripper's
finger/knuckle hardware at spawn and matching the 8.3N of contact force
`stallfreeze_box_full.log` shows while drive was still ~0 (fully open, no
close yet). Round 2 right-sizes `--pedestal` to 0.13 (half-width 0.065,
just covering the 0.0638m reach) for ~6.4mm of pivot clearance instead of
1.4mm; `--object-offset`/`--tcp-above-top` are unchanged (already verified
correct). This is a reasoned, offline best-effort fix, not a live-verified
one -- the real verdict is the next GPU run's leg (c) log.

`$TMP/stallfreeze_chain.sh` is updated in place: `REPO_ROOT` now points at
this worktree, leg (c)'s args are the placement fix above, its outputs
renamed to the `stallfreeze2_*` family, and its final markers renamed
`SF2_EXIT_A/B/C=`, `SF2_EXIT=`, `SF2_CHAIN_DONE` (from `SF_EXIT_A/B/C=`,
`SF_EXIT=`, `SF_CHAIN_DONE`) so a stale round-1 log can't be mistaken for a
round-2 result. A new `$TMP/stallfreeze2_launch.sh` (parallel to round 1's
`stallfreeze_launch.sh`) wraps it into `$TMP/stallfreeze2.log` with those
SF2 markers for a follow-up GPU session to launch and poll.

## 2026-09-06 — Task #20 round 3: default is the plain freeze at measured + lead, press off (post-round-2 acceptance)

**Round-2 result** (`$TMP/stallfreeze2-result.md`): the peak-hold press
found a real plateau (19.68N, matching round 1's own cap-hit magnitude) and
transitioned on `reason=decrease` -- but the 2-tick smoothed-decrease
detector fired on the natural impact ring-down 1-2 press ticks after first
contact, before any real press travel had accrued, and the resulting
back-off left the pads separated: sustained hold force collapsed to a
0-1.3N noise-floor band for the whole 15s hold, and the bottle was NOT
retained on lift (left behind on the table, tilt stayed <2 deg though). The
sugar_box leg remained unscoreable (still spawned tipped 93.9 deg with
main's probe placement args).

**Decision.** `$TMP/task20-decay-probe-findings.md`'s `freeze3-result.md`
chain (three plain, no-press freeze legs: progress trigger, eps 0.005/0.012,
lead 0.02/0.04, all six targets frozen at measured + lead with no advance
and no back-off) is the validated retention configuration: all three legs
were RETAINED ON LIFT (dz 0.3-3.8mm, tilt 0.5-6 deg), the best of them
(`eps 0.012, lead 0.02` -- "P02e") the earliest and cleanest trigger with the
flattest, least-noisy residual force. Both round 1 (press to a fixed cap:
19.7N plateau collapses to 4.6N) and round 2 (peak-hold press: collapses to
0-1.3N) tried to recover more sustained force by advancing further than the
plateau and made things worse -- **any** advance past the plateau slides the
pads circumferentially around the object (the still-open contact-physics
question from `task20-decay-probe-findings.md`'s torsional-friction /
contact-trace / jaw-closure-geometry chain: the pads travel around a curved
object's contact patch under load regardless of friction coefficient,
follower stiffness/cap/lag, drive cap, torsional patch radius, or contact
generation method). Sustained hold force in the 15-30N band stays an OPEN
item, bounded by that contact physics, not by this state machine; retention
through lift is what the bench actually needs and what freeze3 achieves.

**Fix.** `TINKER_SIM_GRIPPER_PRESS_CAP`'s default drops from 0.02 (round 2)
to **0.0**. `_gripper_stall_freeze_check_plateau` now branches on it: when
`<= 0.0` (the default), PLATEAU transitions straight to HOLD via a new
`_gripper_stall_freeze_enter_hold_with_lead` -- all six targets (drive_joint
+ the five mimic followers) are frozen at their OWN measured angle plus a
new knob, `TINKER_SIM_GRIPPER_FREEZE_LEAD` (default 0.02 rad), in one shot,
no PRESS ticks at all. A single `"hold"` transition is logged (no separate
`"press"` event, since PRESS never runs) carrying the lead-derived drive
target, `reason="lead"`, and the pad force sampled at the freeze instant.
The PRESS/peak-hold machinery from round 2 (`_gripper_stall_freeze_press_tick`
/ `_gripper_stall_freeze_press_peak_hold`) is unchanged and stays reachable
by setting `TINKER_SIM_GRIPPER_PRESS_CAP` back above 0.0, for anyone who
wants to re-attempt closed-loop pressing toward `TINKER_SIM_GRIPPER_
HOLD_FORCE_N` later.

**`travel` logging fix.** Every `gripper_stall_freeze` transition's `travel`
field is now computed by a new `_gripper_stall_freeze_current_travel()` --
read directly from `_position_targets` against `_gsf_baseline` -- instead of
echoing the separately incremented/decremented `_gsf_travel` counter, which
could (and, in the round-2 live acceptance run, did) report `0.0` on every
single transition regardless of state (`stallfreeze2-result.md`'s side
note). `"hold"` (both the new lead path and round 2's peak-hold exit) and
`"release"` now use this helper; `"plateau"` and PRESS's own `"press"` entry
keep an explicit `travel=0.0` (the baseline is defined as zero travel at
that instant, before any lead or press advance is applied) and `"reset"`
keeps `travel=0.0` too (baseline is already cleared by the time it logs).

**Env knobs** (all optional; defaults now match freeze3's P02e config):
`TINKER_SIM_GRIPPER_STALL_FREEZE` (default `"1"`), `TINKER_SIM_GRIPPER_
STALL_CONTACT_N` (5.0 N), `TINKER_SIM_GRIPPER_STALL_DWELL_S` (0.3 s sim),
`TINKER_SIM_GRIPPER_STALL_EPS` (0.012 rad), `TINKER_SIM_GRIPPER_FREEZE_LEAD`
(**new**, 0.02 rad -- the one-shot lead applied at PLATEAU when PRESS is
off), `TINKER_SIM_GRIPPER_PRESS_CAP` (**default now 0.0** -- PRESS off;
round 1/2's default was 0.06/0.02), `TINKER_SIM_GRIPPER_PRESS_STEP` (0.002
rad/tick, only used when PRESS_CAP > 0), `TINKER_SIM_GRIPPER_HOLD_FORCE_N`
(25.0 N, only used when PRESS_CAP > 0).

**Tests** (`tests/test_manipulation_runtime.py`): `_gsf_backend()` now takes
`press_cap` (default 0.0, matching the new backend default) and
`freeze_lead` (default 0.02) parameters. The three PRESS-machinery tests
(`test_gripper_stall_freeze_press_peak_hold_backs_off_on_decrease`,
`test_gripper_stall_freeze_press_backs_off_at_hold_force_target`,
`test_gripper_stall_freeze_press_cap_stops_travel`) now explicitly pass
`press_cap=0.06` to keep exercising round 2's PRESS/peak-hold behaviour
unchanged; `test_gripper_stall_freeze_plateau_needs_dwell_eps_and_contact`'s
plateau-into-PRESS case does the same. Two new tests cover the default
path: `test_gripper_stall_freeze_default_plateau_holds_with_lead` asserts
PLATEAU with the default config lands directly in `"hold"` (no
`_gripper_stall_freeze_press_tick` call, all six targets at measured +
0.02) -- this FAILS against 0ce3dc8 (round 2's default `press_cap=0.02`):
```
AssertionError: 'press' != 'hold'
```
(confirmed by temporarily stashing only `backend.py`, rerunning `-k
test_gripper_stall_freeze_default_plateau_holds_with_lead`, then restoring
the fix). `test_gripper_stall_freeze_default_hold_logs_lead_as_travel`
captures stdout and asserts the logged `"plateau"` event's `travel` is
`0.0` and the `"hold"` event's `travel` is exactly the lead (`0.02`) with
`reason="lead"` and the frozen `pad_force_n`. A third new test,
`test_gripper_stall_freeze_release_logs_actual_travel`, asserts RELEASE's
logged `travel` reflects the real applied-target-minus-baseline gap
(0.01) rather than a stale/zeroed counter. Full suite:
`tests/test_manipulation_runtime.py` 121 passed, 3 subtests passed, 0
failed (118 pre-existing + 3 new; run via the pytest-suite-ros-env-
incantation memory: no `set -u`, a lark-only symlink dir +
`/opt/ros/humble/local/lib/python3.10/dist-packages` on `PYTHONPATH`,
`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`).

**Staged acceptance.** `$TMP/stallfreeze3_chain.sh` re-runs main's
`validation/gripper_close_probe.py` bottle side-pinch close with the flag on
vs off (same args as rounds 1/2's legs (a)/(b)) -- two legs only, the YCB
sugar_box leg is dropped: main's probe placement for it has been broken
across all three rounds (spawns tipped 93.9-94 deg before any close), and
box retention is validated by the bench's own soup + sugar box round
instead (`task20-decay-probe-findings.md`'s "Live datapoint" entries).
`$TMP/stallfreeze3_launch.sh` wraps the chain with `SF3_EXIT=`/
`SF3_CHAIN_DONE` markers in `$TMP/stallfreeze3.log` for a follow-up GPU
session to run; `REPO_ROOT` defaults to this worktree.

## 2026-09-06 — Task #27: gripper facade false stall at low RTF (dwell ran on the wall clock)

**Symptom.** Bench round `agv`: a close goal reported
`Gripper close ok: position=0.013920 effort=9.992524 stalled=1
reached_goal=0` only ~0.45s wall after the goal started -- 4% of the drive's
stroke, no contact -- and `pick_and_place` immediately attached the object
and lifted; the arm lifted air. Bench RTF at the time was ~0.27.

**Root cause.** `gripper_facade.py`'s `_execute` runs with
`use_sim_time=True` and already uses `self.get_clock().now()` correctly for
`simulation_timeout`, but the no-progress stall dwell
(`stall_dwell_s`, default 0.3s) was timed with the free function
`time.monotonic()` -- WALL seconds -- via `last_progress_at`/`now_monotonic`
(and, since #23, `contact_stalled` inherits the same dwell). At RTF 0.27,
0.3s of WALL time is only ~0.08s of SIM time, well under the ~0.2s of SIM
actuation latency the drive needs before it even starts moving, so the
dwell fired before the drive had any chance to make progress -- confirmed
against the bench trace's timestamps and the truth-drive samples in
`/home/tinker/.claude/jobs/01ca17b4/tmp/task27-facade-false-stall-findings.md`.
`start_wall` and the 30s wall watchdog are correctly wall-clock (they bound
a mux/keepalive contract measured in wall seconds) and were left alone.

**Fix.** `last_progress_at` and the per-iteration `now_sim` (formerly
`now_monotonic`) now read `self.get_clock().now().nanoseconds * 1e-9`, the
same sim clock `simulation_timeout` already used; `contact_since` (and
therefore `contact_stalled`) now derives from the same `now_sim`, so both
stall paths share one clock. No parameter values changed.

**Observability.** The facade had zero log lines. Added
`_log_execute_outcome`, called once at every terminal `_execute` exit
(`aborted_safety_stop`, `canceled`, `reached_goal`/`stalled`, `timeout_sim`/
`timeout_wall`) with position, effort, stalled, reached_goal, and elapsed
sim/wall seconds -- what previously took cross-referencing the client's own
log line against the truth evaluator is now a single grep on
`tinker_sim_gripper_facade`.

**Test.** `tests/test_gripper_executor_humble.py::
test_position_stall_dwell_uses_sim_clock_not_wall_clock_at_low_rtf` drives a
synthetic `/clock` feed at RTF ~0.27 alongside `/isaac_joint_states` samples
that stay parked for 0.2s SIM, ramp for 0.15s SIM, then genuinely plateau.
On main it failed with:
`AssertionError: goal finished before the drive could have genuinely
stalled in SIM time -- the stall dwell is still running on WALL time`
(`assert not True`, at the 1.4s-wall checkpoint where sim time has only
reached ~0.38s). Getting this test running for real also surfaced an
unrelated test-harness trap worth recording: the file's existing
`_run_goal_with_timer` helper drives `_execute()` from a
`node.create_timer()` callback with no explicit `callback_group`, which
defaults to the node's default `MutuallyExclusiveCallbackGroup` -- the same
group rclpy's internal `TimeSource` uses for its own `/clock` subscription.
`_execute()`'s while-loop then monopolizes that group for the whole goal,
silently freezing `self.get_clock().now()` until the goal ends (reproduced
in isolation before diagnosing it). The real `ActionServer` doesn't have
this problem -- `execute_callback` runs in its own explicit
`ReentrantCallbackGroup` -- so the new test drives `_execute()` from a plain
background thread instead, matching that disjointness. Full targeted run:
`tests/test_gripper_executor_humble.py` 12 passed;
`tests/test_manipulation_runtime.py -k "facade or gripper"` 11 passed; full
`test_manipulation_runtime.py` 102 passed, 3 subtests passed, 0 failed.

**Remaining margin (follow-up, not fixed here).** The dwell (0.3s SIM) and
the observed actuation latency (~0.2s SIM) are close enough that a slower
actuation response, or a lower stall_epsilon combined with sensor noise,
could still trip a false-ish stall inside that 0.1s SIM margin. Not
reproduced live; flagged for whoever next tunes `stall_dwell_s` or
actuation timing.
## 2026-09-06 — Task #28: contact-report enablement was invisible in the sim log

**Symptom.** A whole bench round (`agv`) came back with `/sim/truth/contacts`
publishing nothing, `/sim/parity/finger_contact` flat zero on every axis for
its entire ~6100-sample force trace, and every physics-truth frame's
`contacts` field an empty list — a structurally-silent contact pipeline with
no error, no exception, and nothing distinguishing it in the sim's stdout
from a genuinely contact-free round.

**Diagnosis.** Not a regression in the round's own code path. PhysX contact
reporting is decided exactly once, at `IsaacWholeRobotBackend.__init__`
(`simulation/tinker_sim_isaac/backend.py`), from the `enable_contacts`
constructor argument — which the sensor-rich launch path derives from
`TINKER_SIM_SENSOR_RICH_CONTACTS` (`validation/run_sim.py`). There is no
later re-check: if that boot's environment lacked the flag, the
`subscribe_contact_report_events` call is skipped entirely and
`_on_contact_report_event` (the sole writer of the backend's contact-pairs
dict, which both `/sim/parity/finger_contact` and physics-truth's `contacts`
list read from) never fires for the rest of that process's life — no matter
what changes afterward in the launched task stack. Round `agv`'s bringup
(`restart_c_by_pid.sh`) explicitly restarts only the task-stack terminal
("Isaac + control plane stay up"), so whatever `TINKER_SIM_SENSOR_RICH_CONTACTS`
value the *already-running* Isaac Sim process booted with is the one that
silently governs contacts for every subsequent round until Isaac itself is
restarted — and nothing in the log said which value that was.

**Fix (observability only, no behavior change).** Two new one-line JSON
diagnostics in `backend.py`, alongside the existing `wheel_collider` /
`solver_iterations` boot lines:
- At `__init__`, right where `enable_contacts` is consumed:
  `{"event": "contact_report", "enabled": <bool>, "source":
  "TINKER_SIM_SENSOR_RICH_CONTACTS" | "constructor:enable_contacts",
  "monitored_bodies": <count>}` — printed unconditionally, so a stale
  contacts-off boot is visible in sim stdout without an `/proc/<pid>/environ`
  read or a live force probe. `source` names the env gate when it was set,
  falling back to naming the constructor argument for callers
  (`manipulation-core`, probes) that pass `enable_contacts` explicitly.
- In `_on_contact_report_event`, the first time a pair actually gets
  recorded (not merely reported and filtered out): `{"event":
  "contact_report_first_event", "simulation_time": <t>, "pair": [body_a,
  body_b]}`, gated by a one-shot bool so it never repeats per-contact.

**Operational rule.** For any round that depends on contacts, restart Isaac
Sim itself — a task-stack-only restart keeps the prior process's
`enable_contacts` decision no matter what the next launch wrapper exports.
Before trusting a per-round env override, check the *running* process's own
environment (`cat /proc/<pid>/environ | tr '\0' '\n' | grep
TINKER_SIM_SENSOR_RICH_CONTACTS`), or now, simply read the `contact_report`
boot line the backend already prints.

Non-GPU coverage added (`tests/test_manipulation_runtime.py`,
`test_contact_report_first_event_logged_exactly_once`): two recorded contact
events (found, then persist) against a backend test double produce exactly
one `contact_report_first_event` line, carrying the expected body-pair and a
`simulation_time` field. Full suite: 103 passed, 3 subtests passed, 0
failed.

## 2026-09-06 — Task #25: bound `controller_reconciler`'s post-success teardown so it cannot wedge the launch chain

**Symptom.** In a live GPSR run (`gpsr_stack_logs/20260905T230818`), the
`controller_reconciler` instance spawning `joint_state_broadcaster` logged
`controller joint_state_broadcaster is active` at t=148.65s and then never
logged anything else, never exited, and never wrote `process has finished
cleanly` or `process has died`, for the remaining ~26 minutes of the run
(final teardown at t~1788671093, ~114 min of Isaac Sim's own internal
clock). Because `whole_robot.launch.py` / `gpsr.launch.py` chain
`xarm7_traj_controller`'s reconciler off this process's `OnProcessExit`,
that controller was never loaded and every arm goal was rejected for the
whole run.

**Root cause (not proven; best-supported hypothesis).** Full analysis in
`/home/tinker/.claude/jobs/01ca17b4/tmp/task25-reconciler-hang-findings.md`.
Key evidence:
- Everything in `controller_reconciler.py` up to and including the logged
  "is active" line is provably bounded by `time.monotonic()` deadlines
  (`RosControllerManagerApi._call`, `set_remote_parameter`), independent of
  `/clock` — the log line proves that code already finished. Task #21's
  `/clock` re-zero mechanism is unrelated: this node has no `use_sim_time`
  parameter and no wait in it is gated on sim time.
- The launch arguments for this exact `Node(...)` (controllers, timeouts,
  `--ready-node` absence) are byte-identical across `whole_robot.launch.py`,
  `manipulation.launch.py`, and `gpsr.launch.py` — the healthy bench bridge
  and the failing GPSR composite run the identical code path with identical
  arguments, so whatever differs is external to this file.
- This was the *only* process in the entire 578-line bridge log that never
  responded to SIGINT at teardown — every sibling node (safety_supervisor,
  contract_guard, xarm_facade, base_facade, pan_tilt_facade, etc.) printed a
  Python traceback (`ExternalShutdownException` / `RCLError`) proving its
  interpreter resumed and ran signal-handling code; this pid has exactly one
  log line in the whole file and never appears again, including through the
  full SIGINT teardown cascade. That pattern — silent, SIGINT-immune,
  indefinite — is characteristic of a process blocked inside a
  non-Python-interruptible C call, not a logical/Python-level hang.
- The only unbounded, opaque segment left after the logged success is the
  `finally` block: `node.destroy_node()` / `rclpy.shutdown()`
  (`controller_reconciler.py:326-327` before this fix), which descends into
  rclpy's C extension (`rcl_node_fini` / `rcl_shutdown` / rmw/Fast DDS
  participant teardown). The leading hypothesis is that this hangs inside
  Fast DDS participant-deletion under the GPSR composite's much larger,
  near-simultaneous DDS discovery churn (~29 bridge nodes + Nav2 + vision +
  manipulation all standing up participants within a few seconds) — a known
  class of ROS 2 Humble / Fast DDS issue. **This is not proven**: no live
  process was inspected (the investigation was read-only, no sim/stack
  runs). The confirmation step for the next wedge is `py-spy dump --pid
  <pid>` against the stuck `controller_reconciler` process — if the frame is
  in `destroy_node`/`shutdown` inside `rmw_fastrtps_cpp`, that confirms it;
  if it's still inside `reconcile_controller`/`_call`, that falsifies this
  hypothesis and points to the `time.monotonic()` deadlines somehow not
  firing instead.

**Fix (mitigation, not a native-layer fix): bound the teardown.**
`ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/controller_reconciler.py`
now runs `node.destroy_node()` + `rclpy.shutdown()` (in that order, same as
before) inside a daemon thread and joins it with a timeout —
`bounded_teardown()`, new helper, wired through `_run()`'s `finally` block
for both the success (rc=0) and failure (rc=1) paths, since both already
shared this one teardown call. Default timeout 5.0s, configurable via
`--teardown-timeout-s` / `TINKER_RECONCILER_TEARDOWN_TIMEOUT_S` env var
(`_teardown_timeout_default()` reads the env var, falling back to the
default on unset/blank/invalid values so a bad env var cannot break
startup). If teardown completes inside the bound, behavior and exit code
are exactly as before. If it does not:
- logs one line, `controller_reconciler: teardown did not complete within
  Ns after {success|failure}; forcing exit (rc={0|1})`,
- flushes stdout/stderr,
- calls `os._exit(rc)` (preserving whichever exit code the run already
  earned) instead of waiting further.

Two more log lines bracket every teardown attempt (`controller_reconciler:
starting teardown (bound Ns)` before, `controller_reconciler: teardown
completed` after a normal finish) so the next wedge — if it recurs — is
visible from the log alone, without needing `py-spy` just to know where the
process is stuck.

Also added a one-line readiness marker (`{label} succeeded; starting next
stage`, logged at `info`) in `_process_exit_actions()` in
`whole_robot.launch.py`, `manipulation.launch.py`, and `gpsr.launch.py`,
emitted right before a successful `OnProcessExit` hands off to the next
chained stage (e.g. right before the `xarm7_traj_controller` reconciler is
launched). No restructuring of the chain — this only adds observability.

**Tests.** `tests/test_controller_reconciler.py` gained unit tests for
`bounded_teardown()` with a fake teardown callable: prompt completion
returns normally with no forced exit; a teardown that blocks forever
(`threading.Event().wait()`, never set) hits the timeout, calls a
monkeypatched `exit_fn` instead of really exiting (recording the code), and
emits the expected log line — checked for both the success (rc=0) and
failure (rc=1) exit-code cases. Also covered `_teardown_timeout_default()`
(env var present/invalid/unset). Existing tests in the file were unaffected
except one that asserts the full parsed-args dict, updated to include the
new `teardown_timeout_s` field.

**Not fixed by this change:** the underlying native hang, if hypothesis 1
is right, remains unconfirmed and unaddressed — this only stops it from
wedging the launch chain. The user's explicit choice was to land the bound
now rather than block on live-repro instrumentation first. `py-spy dump` on
the next live wedge (see above) remains the confirmation step for the root
cause.

## 2026-09-05 — /clock re-zero on a full sim-process restart: anchored to a boot epoch (task #21)

**Root cause (findings: `task21-clock-rezero-findings.md`, `.claude/jobs/01ca17b4/tmp/`).**
`767fb89` (2026-08-27) made `/clock` monotonic across an *in-process*
`ResetSimulation` STOP -> PLAY (`backend.py` `_refresh_robot_handles`,
`simulation_time` re-anchors `_clock_step_origin` to the elapsed step count
observed before the boundary instead of re-zeroing). That fix does not, and
was never meant to, cover a full Isaac Sim *process* restart: a fresh
`IsaacWholeRobotBackend` re-initializes `_clock_step_origin = 0` /
`_clock_elapsed_steps = 0` (backend.py, `__init__`), and `ros_gateway.py`
stands up a brand-new rclpy node/DDS participant, so the new process's first
`/clock` samples are near `0.0` again -- a real backward jump relative to
the prior process's last published sample. Long-lived ROS consumers that
keep running across the sim restart see it: Python `tf2_ros.Buffer` (used
by, e.g., AnyGrasp) has no clock-jump handling at all (unlike the C++
`tf2_ros::Buffer`, which registers `onTimeJump`), so a backward `/clock`
sample is either cached as stale "old data" and dropped, or produces
`ExtrapolationException`, until ~10 s of new sim time re-elapses past the
old cached entries (`tf2::BufferCore`'s 10 s default cache window). This is
the same class of wedge `767fb89` fixed for the in-process case, one level
up.

**Fix (option (a) from the findings, chosen over publishing a boot-id for
consumers to clear their own buffers on): anchor the *published* clock to a
boot epoch**, so a fresh process's `/clock` never appears to precede a
prior process's last sample -- matching hardware parity (real ROS time on
hardware is wall-clock and never goes backward).

- `simulation/tinker_sim_isaac/backend.py`: new `resolve_clock_epoch(value)`
  parses `TINKER_SIM_CLOCK_EPOCH`: unset/`"wall"` (new default) ->
  `time.time()` captured once in `__init__`; `"0"` -> the legacy zero-based
  clock; any other numeric string -> a pinned epoch (seconds), for a
  harness that wants a reproducible absolute clock. The resolved value is
  stored once as `self._clock_epoch_s`. A new `ros_clock_time` property
  returns `self.simulation_time + self._clock_epoch_s` -- this is the value
  to publish/stamp with. **`simulation_time` itself is unchanged**: it stays
  the small, process-relative elapsed-steps value that internal consumers
  already depend on starting near zero (run-duration gating in
  `validation/run_sim.py`'s `args.duration <= 0.0 or backend.simulation_time
  < args.duration` loop guards, the base-hold timers, truth-record `t`
  fields) -- only the externally published clock needed to change. The
  767fb89 in-process STOP -> PLAY re-anchoring is untouched and composes
  with the epoch (it still re-anchors `_clock_step_origin`, which
  `ros_clock_time` builds on through `simulation_time`).
- `simulation/tinker_sim_isaac/ros_gateway.py`'s `_stamp()` (used for
  `/clock` and every outgoing ROS message header stamp, including camera
  frames) now reads `backend.ros_clock_time` (falling back to
  `backend.simulation_time` for a backend double that predates the
  property), so every ROS-visible timestamp this gateway produces is
  consistently epoch-anchored, not just `/clock` itself.
- `ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/contract_guard.py`'s
  `evaluate_clock_domain` readiness gate treated `clock_now_ns <= 0` as "sim
  clock hasn't advanced past zero" -- correct under the old zero-based
  clock (rclpy's own `TimeSource` convention: "Zero time is a special value
  that means time is uninitialized"), but under the new wall-clock-anchored
  default a real running sim's first sample is already a large nonzero
  epoch value, so the gate no longer needs (or should assume) a genuine
  zero reading ever occurs in normal operation. `clock_now_ns` is now typed
  `int | None`: `None` explicitly means "no clock sample received yet" (a
  caller that tracks this itself, distinct from a raw `Clock.now()` read);
  the literal `0` case is kept and still treated as not-ready for backward
  compatibility with any caller that can only observe the numeric value.
  `joint_state_probe.py`'s call site needed no change -- it passes a raw
  `Clock.now().nanoseconds` read, which already reads exactly `0` before
  any `/clock` message has ever arrived (rclpy's own uninitialized-clock
  convention) and a large epoch value once the new anchored clock is live.

**Grepped for and ruled safe (no change needed), per the findings' "verify
this" list:** `validation/gripper_close_probe.py --record-s` (a duration
parameter, `steps = args.record_s / DT`, never touches `simulation_time`);
`validation/run_sim.py`'s profile-emission `sim_time=getattr(backend,
"simulation_time", None)` (pure telemetry; unaffected since
`simulation_time` didn't change); the 2026-08-21 profiling-attribution note
in this log about `/clock` vs `wall_time` drifting by the bridge attach
time (predates `767fb89`, describes historical pre-fix behavior, not a live
assumption). `_sim_receipt_time`/`_sim_age_stale` in `ros_gateway.py` (used
for same-process command-liveness deltas) were left reading the raw
`simulation_time`, not `ros_clock_time`: a constant epoch offset cancels
out of a same-process delta, so this is deliberately unchanged rather than
switched for consistency's sake.

**Known limitation, not fixed here:** the wall-clock default only
guarantees the new process's first sample is >= the old process's last one
when real wall-clock time elapses across the restart at least as fast as
sim time did within the old process (i.e. the sim was not running many
times faster than real time right up to the moment of the restart, and the
restart itself takes nonzero wall time) -- true in practice for an Isaac
Sim boot (tens of seconds) but not a mathematically airtight guarantee for
an arbitrarily fast headless sim killed and relaunched in well under a
second. A stronger guarantee would need cross-process persistence (a state
file) rather than a wall-clock anchor; deferred as the findings' option (a)
scoped it as wall-clock-only, matching hardware parity as the acceptance
bar rather than absolute mathematical monotonicity.

Tests: `tests/test_manipulation_runtime.py` (`resolve_clock_epoch`
defaults/legacy-zero/numeric/rejects-garbage; `ros_clock_time` adds the
epoch without perturbing `simulation_time`; two backends constructed
back-to-back with `time.time` monkeypatched to advance publish
non-decreasing clocks; the 767fb89 in-process reset still holds with an
epoch anchored) and `tests/test_integrated_joint_state_contract.py`
(`evaluate_clock_domain` treats `None` as not-ready and a large epoch value
as ready, alongside the pre-existing zero/missing-publisher cases).

## 2026-09-04 — Wrist camera blackout band: the camera was rendered from inside the gripper housing

**Symptom (grasp bench, sensor-rich profile, `TINKER_SIM_WRIST_CAMERA_AIM=tool-forward`).**
Every wrist frame carried a curved black band along the TOP edge, growing
toward the right: first non-black row 0 at x<=212, 40 at x=318, 56 at
x=424, 68 at x=530, 76 at x=636, 80 at x=742, 81 at x=847 — 9.4% of the
848x480 image, present in color and aligned depth alike, fixed relative to
the camera in every arm pose (close scan, wide scans, approach, carry).
Pixels were unlit (mean grey 1.1, half exactly 0), and the aligned depth
inside the band read a NEAR-RANGE 50–60 mm, not 0; deprojected through the
URDF optical frame it landed on `xarm_gripper_base_link` at z ≈ 0.07.
AnyGrasp's depth mask dropped those pixels (row 5: 241/848 valid). A
brighter patch on the right edge came with it. Task-list item #18.

**Reproduction without a GPU.** An offline USD frustum model (pure `pxr`,
no Kit: project every `/visuals/` mesh of `robot.usd` through the rig's
camera — 848x480, 69.4° HFOV, the rig's 0.05 m near clip — placed exactly
as `CameraRig.initialize` places it: mount prim xform · orient(correction ·
`mount_rotation_wxyz`)) reproduces the live band to within a row:

| | live (bench) | offline model |
|---|---|---|
| occluded fraction | 9.4% | 9.6% |
| first clear row @ x=318/424/530/636/742/847 | 40/56/68/76/80/81 | 40/57/69/77/81/82 |
| depth in band | 50–60 mm | 50–66 mm |
| occluder | (deprojects to gripper base) | `xarm_gripper_base_link/visuals/base_link/mesh` — the ONLY hit |

With no aim correction (parity) the model shows zero self-occlusion, so the
band is a product of the `tool-forward` tilt — but the tilt was only the
proximate cause.

**Root cause: the description mounts the wrist camera at a placeholder.**
`tk26_sim/src/isaac_bringup/urdf/tinker_full.urdf.xacro` (the sim
description the robot artifact is built from) layers Intel's
`sensor_d435` macro onto `link_eef` with `<origin xyz="0 0 0" rpy="0 0 0"/>`.
The Intel macro then adds its own bottom-screw → camera_link offset
(0.0106, 0.0175, 0.0125) and the colour frame's +0.015 in Y, so the colour
optical frame sits at link_eef + (0.0106, 0.0325, 0.0125): 12.5 mm above
the flange, 33 mm off the tool axis — i.e. on the surface of the gripper
base housing — looking along +X_eef, 90° off the tool. (That 90° is the
defect `tool-forward` was written for; see 2026-08-31.) The real robot
(`tinker_real.urdf`, xArm's `realsense_d435i.urdf.xacro`, "vendor
factory-nominal" extrinsics with the per-robot hand-eye override hook)
mounts `xarm_camera_link` on the D435 cam-stand bracket at link_eef + xyz
(0.06746, −0.0175, 0.0237) rpy (π, −π/2, 0): 67 mm out along +X_eef, 24 mm
up, looking straight down the tool axis with image-up radially outward.
Tilting the placeholder camera 60° toward the tool IN PLACE swings the
housing's far wall into the top of the frustum, just past the 0.05 m near
clip — hence "black, 50–60 mm". The bright right-edge patch is the lit
exterior of the same housing / left finger.

**Ruled out / not the cause.** Rendering settings (the head camera under
the same profile has no band); the annotator/CUDA-700 class of stale-read
faults (the band is geometrically stable and depth-consistent); the
`mount_rotation_wxyz` fix e2996f8 (parity pose shows no self-occlusion);
lighting (unlit because it is the inside of a closed mesh).

**What would also have "worked", and why not.** In the model, dollying the
tool-forward camera forward by as little as 10–20 mm along its view axis
already clears the housing (it was only 0–16 mm past the near plane), and a
50 mm shift up the optical −y does too. Both keep the render origin inside
the gripper assembly and keep the 60° in-place tilt that was itself a
sweep-picked compromise ("30° shy of the tool because a tool-aligned view
stares into the co-axial hand" — true only because the camera was
co-located with the hand).

**Fix: render from where the real camera is (`cam-stand`).** A second wrist
preset, `TINKER_SIM_WRIST_CAMERA_AIM=cam-stand`, places the render camera
at exactly the vendor cam-stand pose: `CAM_STAND_MOUNT_OFFSET_XYZ` =
(0.065, −0.0112, 0.05686) is the bracket translation and
`CAM_STAND_CORRECTION_WXYZ` = (0, 0, −√½, √½) the rotation, both in the
artifact's (placeholder) optical frame — `inv(T_artifact_optical) ·
T_vendor_optical`, which `tests/test_wrist_camera_aim.py` re-derives from
both URDF chains so the constants cannot drift from the geometry they
claim. Rendered from there the model shows ZERO housing pixels; only the
fingertips at the bottom edge (~750 px, 0.2%) — what a wrist camera sees.
Authoring the exact op sequence the rig produces onto `robot.usd`
(`AddTranslateOp(op0)`, `AddOrientOp`) lands the origin at link_eef +
(0.06746, −0.0325, 0.0237), view +Z_eef, image-up +X_eef, to 4e-8.

Mechanics: `CameraStreamSpec.mount_frame_offset_xyz` (new; default zero)
is a translation in the MOUNT prim's frame, authored as a translate op
listed BEFORE the orient op — USD applies the last-listed op to the
geometry first, so a translate before orient stays in the mount's axes
while the existing `view_axis_forward_offset_m` dolly (after orient) stays
in the camera's. Listing the bracket after the orient would rotate it with
the aim and put the camera straight back on the housing. `camera_xform_ops`
returns the op list as data so the order is unit-tested without Kit
(`tests/test_camera_rig.py`). `tool-forward` is kept as the A/B baseline;
`scripts/gpsr-stack` now hands the sim stage `cam-stand`.

**TF has to move with the pixels.** The artifact URDF still carries the
placeholder joint, so `robot_state_publisher` would put
`xarm_camera_color_optical_frame` 6 cm and 90° away from where the pixels
were rendered — every consumer deprojecting through TF (AnyGrasp's desk
plane fit, detections in base/map) would be wrong by construction, and the
bench had been compensating with a private `_aimed` alias frame.
`tinker_sim_deploy.runtime.sim_robot_description` = `topic_control_description`
plus a rewrite of `xarm_camera_joint`'s origin to (0.05496, 0, 0.0131) rpy
(π, −π/2, 0) — `T_vendor_camera_link · inv(T_intel_bottom_screw)`, which leaves
the Intel chain above it untouched and lands the colour optical frame on the
vendor pose. It is keyed on the SAME env value the sim stage reads
(`TINKER_SIM_WRIST_CAMERA_AIM=cam-stand`; any other value is a no-op), all
four bridge launches (gpsr, manipulation, whole_robot,
integrated_ompl_manipulation) call it, and `gpsr-stack` exports the value to
the bridge stage too. `tests/test_wrist_cam_stand_description.py` checks the
rewritten chain's FK equals the render pose exactly. A custom stack must set
the variable for BOTH the sim and the bridge or TF and pixels disagree.

**Live result (GPU0 relaunch of the bench recipe, same measurement tool on
both).** Relaunched the shared GPU0 sim from this branch with the bench's exact recipe (`launch_isaac_bench.sh`: sensor-rich, rcw2026, seed 7, spawn (−2.99, 3.80), `TINKER_SIM_CAMERA_HZ=4`) and only the wrist preset changed. Same subscriber-side metric on both (first non-black row per column, top-edge-connected black region, aligned-depth stats):

| | tool-forward (before) | cam-stand (after) |
|---|---|---|
| top-edge black band | 6.7% of the frame, 7/7 frames | 0 px, 8/8 frames |
| first non-black row @ x=318/636/742/847 | 39/76/80/81 | 0/0/0/0 |
| aligned depth inside the band | 50–64 mm | (no band) |
| camera_info frame / fx | `xarm_camera_color_optical_frame` / 612.3 | unchanged |

The after-capture is at the boot pose, where the tool-aligned camera looks at the robot's own chassis and lidar dome at close range (median depth 75 mm) — the home-pose view, not the housing; the housing band is pose-independent and gone. At the bench's four scan joint poses the URDF-FK-reposed frustum model gives tool-forward 9.6% (housing) vs cam-stand 0.1% (356 px of fingertip at 110–138 mm along the bottom edge) in every pose. Frames: `wrist_baseline_toolforward_color.png` / `wrist_camstand_color.png` in the job tmp dir; the grasp bench session is re-measuring at its scan poses against the plain optical frame.

**Second round (same day): the first cam-stand authoring froze the wrist render.**
The grasp bench's first live check of `cam-stand` at its wide scan pose
showed the robot's own chassis and lidar dome filling the wrist frame with
5–8 cm depths, which read as "the camera looks back along the tool". It
did not: that frame was pixel-for-pixel my boot-pose capture from 40
minutes earlier (mean difference 6.6 grey levels, 4% of pixels over 20 —
AA dither), while the spectator camera showed the arm at the scan pose;
the bench then moved to the close-scan pose and the wrist frame again
changed by dither only. The RTX render product had stopped tracking the
arm the moment `cam-stand` was on: every frame was the boot frame.

The one thing the first authoring changed on the prim was a SECOND,
suffix-named translate op — `xformOp:translate:op0` listed before
`xformOp:orient` (a distinct suffix is required to author two translates,
and the head's dolly already used the plain name after orient). Every
camera that tracks fine (head with its dolly, wrist under `tool-forward`)
carries only standard-named ops. The pose math was never wrong: the USD
`XformCache` result for the authored ops matched the vendor chain to 4e-8
both times — the static stage composes the suffixed op correctly, the
renderer's live hierarchy evidently does not. Suffix vs. non-standard
order was not separated (each test costs the shared sim a restart); the
fix removes both. `camera_xform_ops` now folds every offset into ONE
standard `xformOp:translate` listed before `xformOp:orient` (plain TRS
order): the mount-frame bracket offset as is, the view-axis dolly rotated
by the mount rotation into the mount frame first (`R · (0, 0, −d)`), which
is exactly what "translate listed after orient" composed to. Head
level-forward pose under the fold vs. the old `[orient, translate]`: 0.0
difference in USD. Rule for the rig from here on: standard op names only,
at most one translate, orient last.

Live after the second relaunch: boot-pose capture, 8/8 frames, top-edge band 0 px, 0.0% black pixels anywhere, depth 100% valid with median 0.568 m; the frame differs from the frozen one by a mean of 120 grey levels (70% of pixels over 20) and shows the desk ahead with the two fingertip pads just entering the bottom edge, the framing the frustum model predicted. Grasp bench verification at its two scan poses (dccfdff): the frame follows the arm (wide-scan vs. frozen frame: mean difference 121.6 grey levels, 70% of pixels over 20); first non-black row 0 at all nine sampled columns at both poses, 0.0% dark pixels, nothing dark along the bottom edge either; depth 100% valid, median 0.861 m (wide) / 0.571 m (close). Desk-plane fit deprojected through the plain vendor optical pose, no in-plane flip: wide pose 48% of desk-footprint points in a plane at z 0.7328 m, tilt 0.12°, residual 1.2 mm; close pose 72% at z 0.7334 m, tilt 0.08°, residual 0.8 mm (desk top is 0.734 m). Flipped variants fail (3–5% in a 40° tilted plane), so image rows/cols match the camera_info convention. Operator trap recorded by the bench: a stale +60° alias publisher from an older recovery script re-latched over the new alias and put the first fit 13 cm low — kill old static_transform_publishers before re-latching.

**Still open.** The durable fix is the description: give the `sensor_d435`
instantiation in `tinker_full.urdf.xacro` the cam-stand origin (or restore
xArm's `add_realsense_d435i` mount) and republish the robot artifact, after
which both `cam-stand` and the TF rewrite become no-ops and hardware parity
is restored rather than broken. Until then the preset is opt-in and
sim-only like the head's `level-forward`. The head camera has the same TF
gap (its `level-forward` correction is not in TF; the alias is identity) —
not addressed here.

## 2026-09-02 — bench retention follow-ons: knife and plate are grasp-geometry, not sim bugs

The mimic fix (below) turned the bench's first physically retained grasps
(bottle held 2/3 at pure defaults, the campaign's first). The two remaining
0/3 objects — knife and plate — were run down in the headless probe with the
fixed backend and contacts ON; neither is a gripper-physics fault.

**Aperture-vs-drive map (URDF FK, verified against the sim).** Pad-face gap by
drive angle: 0.00→89 mm, 0.22→69 mm (bottle), 0.43→99 mm finger-origin sep,
0.54→36 mm, 0.61→30 mm, 0.85→~6 mm. And the fingertips travel ~13 mm along the
tool axis over a close (parallelogram arc): 3 mm short of the TCP plane at open,
~9 mm PAST it near full close. So every top-down grasp height must add the arc:
the commanded TCP sits ~9 mm above where the fingertips actually end.

**Plate — infeasible geometry on the solid-disc asset (probe18).** The bench
`bench-plate.usda` was a solid cylinder r=0.10 h=0.025 lying flush on the desk.
A top-down radial rim pinch cannot grip it: with the jaw open 140 mm and closing
along the radius, the inner pad lands 74 mm IN from the near rim, flat on the
solid top face, and the arm's descent jams it there at ~120 N before the close
starts; the outer pad closes through air and stops 32 mm short of the rim (7 N
graze). The single-DOF linkage halts on the jammed knuckle at drive 0.36 and
nothing is pinched (lift retains 0 N). The bench read 0.54/zero-force because
`/sim/parity/finger_contact` is structurally silent in the sensor-rich profile,
so the 120 N jam was invisible; those descent jams are also the source of the
>120 N force-trace spikes (fingertip-on-rigid-surface strikes, not close
punches — the close itself never exceeds ~20 N with the mirror fix). A side/edge
pinch is equally blocked because the flush disc has no clearance beneath the
rim. A real deep-plate asset (foot ring raising a shallow bowl with a raised
rim, so a top-down rim pinch has clearance beneath) was authored and probed —
and it revealed a deeper truth: **the plate is not the asset, it's the gripper
model.** Across four rim geometries (6/16/25 mm walls, both jaw-centre biases)
and with compliant contact enabled, a top-down rim pinch loads only the
drive-side pad (~10 N) and the follower pad never engages (~1 N), so it never
holds. The reason is the single-DOF mirror doing exactly what the mechanism
says: the drive joint is the LEFT outer knuckle and the five followers track its
MEASURED angle through a rigid coupling, so the instant the left pad jams on the
rim's outer face the whole jaw freezes — before the right pad has closed the
last few mm onto the inner face. Small objects trapped symmetrically between the
pads (bottle 69 mm, knife 30 mm) load both sides and hold; a large object
gripped at a local off-centre rim loads one side. This is faithful to a rigid
single-motor gripper, not a bug (the OLD target mirror would load both pads here
precisely because it drove the followers independently — the same
independence that curled the pads and punched the bottle). Consequences: the
plate needs either a grasp that traps it symmetrically (hard for a 200 mm disc),
or the drive modeled as a CENTRAL actuator so a blocked finger doesn't halt its
partner (a URDF/backend change, a follow-up), or removal from the goalset. The
deep-plate asset and the four probes live in the session's evidence; the shipped
`bench-plate.usda` is unchanged pending that decision.

**Knife — graspable across the width; the failures were candidate + facade
(probe19).** A correct top-down centre pinch with the knife's 30 mm width across
the closing axis holds it 3/3: descent clean (pads straddle the width, no jam),
close stalls at drive 0.61 (=30 mm), pads bite the two sides (18 N / 8 N), lift
raises the knife +94 mm with the TCP at ~15 N hold. The bench's failures: (a)
candidate idx 0 sits 60 mm off-centre and closes on air — drive runs 0→0.825 at
full speed then sits dead flat, the exact signature in the bench mimic trace;
(b) the settled knife yaw is not guaranteed to land the 30 mm width across the
closing axis (a spawn-rotation quirk the benchmark owns); (c) the running bridge
facade on the main checkout (task50-stage-a-repair @ d8cc0ff) predates the
contact-free position-stall path (`stall_dwell`, added on task-sim-bugfixes at
a46d108), so an air-close never latches `stalled=true` and instead times out
three times as "native gripper: execution failed" (~111 s = 3 × the 5 s sim
timeout at sub-1.0 RTF). Benchmark-side geometry + a facade heal on the main
checkout; not the sim's gripper.

**Community cross-check.** Isaac Sim's own closed-loop tutorial (Robotiq 2F-85)
breaks the parallelogram loop and drives followers via the PhysX Mimic Joint API
referencing the drive joint's state — never by copying a commanded target — and
ros2_control's gripper action controller aborts on stall by default
(`allow_stalling: false`), which is the generic shape of the facade abort. Both
corroborate the fixes here.

## 2026-09-02 — close-phase punch root cause: the mimic mirror copied the TARGET, not the drive's angle

Closes Task #19 at the source. Five reactive/solver-level cycles (below) treated
the first-contact spike as a control or contact problem; it is a kinematics
problem. The xArm gripper (UFACTORY manual V1.11.0: one actuator, 84 mm stroke,
30 N max clamping force) is a single-DOF mechanism — one motor on the left
outer knuckle, the right knuckle gear-coupled, each finger on a parallelogram
that keeps the pad parallel. The URDF says exactly that: all five follower
joints carry `<mimic joint="drive_joint" multiplier="1" offset="0"/>` (finger
joints on `-x` axes = counter-rotation = parallel pads; the importer baked
those axes as 180° frame flips, so a uniform +1 mirror is kinematically right).
URDF mimic semantics are `q_follower = q_drive` — the driving joint's ACTUAL
angle. `_mirror_gripper_mimic_targets` copied drive_joint's commanded TARGET.
Identical in free motion; wrong the instant the object blocks the drive
knuckle, when the followers keep chasing the far target as five independent
k=1500 motors.

Measured in-process (headless probe, CPU PhysX, bench bottle centred at the
bench's recorded grasp pose, stock defaults): the k=200 drive side lags the
k=1500 followers 0.06 rad even in free motion, so the right pad always arrives
first; after the drive stalls at 0.43 rad the right outer knuckle runs on to
0.65 and shoves the bottle into the weak left pad (right 25 N vs left 5 N,
bottle tilted 12°, held only by hooking); and the finger joints run to 0.845
regardless — the pads CURL 0.2–0.4 rad about the finger axis. That curl is the
bench's "pads close through to 0.728, 18 mm past the knife"; on a desk-lying
knife it drives the fingertips into desk/knife, the 200+ N spikes.

Two earlier readings are corrected by the same data. (1) The d·v "preload"
hypothesis is real but not the ejector: free closes give k·lag = d·v within 3%
in every config (defaults: 73 N·m per follower entering contact; slew 0.75 →
37; d=20 → 22; k=500 → 60, i.e. lowering k only grows the lag), yet on a
centred bottle the stock jaw peaks at only 33 N and LOWER damping made
retention worse (d=10: 67 N, dropped). isaaclab's `applied_torque` reports
the net (spring − damper ≈ 0 in motion), which is why the preload was never
visible. (2) The dev-log's "drive joint is unloaded" premise is false:
`drive_joint` → `left_outer_knuckle` → `left_finger` carries the left pad.

Fix: followers target `q_drive + q̇_drive·dt` (measured angle plus one control
step of feed-forward so their one-step lag does not drag the drive; at stall
q̇ ≈ 0 so they hold the drive's angle exactly). A blocked knuckle now stops the
whole linkage, the pinch is symmetric, and drive_joint's actuator — its effort
limit, i.e. the facade's `max_effort` (50 → the USD maxForce; ≈20 N total pad
force, vs the 30 N spec) — is the true grip bound. The stall-gated lead clamp
(06cc2e1) is retired to default-off: with mimic-correct followers it has
nothing to bound, and its gate self-locks the close into a 0.1 rad/s crawl
(pads trail the drive by one step, so pad speed sits on the gate; a 30 mm knife
was not reached in 3 s). `TINKER_SIM_GRIPPER_MAX_LEAD_RAD` keeps it available.

Validation (same probe, bench objects, bench grasp poses, lift = joint2
−0.15 rad, retention = object rises with the TCP): bottle 7/7 retained
(peak = hold ≈ 20 N, tilt ≤ 6°, contact at 0.18 s) across slew 1.5/0.5,
follower k 1500/500, clamp on/off; knife (top-down, fingertips 15 mm above the
desk) 3/3 retained (peak 20 N, hold 19 N, no displacement) vs 0/5 with the
stock mirror (46–49 N peak, drive 0.42 while pads curl to 0.845, apparent
30 N "hold" that vanishes on lift — the bench's signature). Unit test:
`test_gripper_mimic_followers_track_measured_drive_angle` (red on the old
mirror, green now).

Probe lessons worth keeping: a static support column wider than the bottle
footprint spawned INTO the gripper hulls and exploded the articulation (drive
−32 rad) before any close — use a footprint-sized pedestal and gate every
trial on "no contact pairs, followers quiet, drive in range"; the kinematic
base hold latches at sim t=2 s, which on the bare ground plane caught the
0.2 m spawn drop mid-tumble (root z 0.218, 40° tilt) — latch after ~8 s;
`body_quat_w` comes through as XYZW here (the backend's `root_state` reorders
the same way) — decoding it as wxyz put the knife 139° off; the finger pad
runs 0–61 mm from the finger joint along the tool axis with the TCP plane at
64 mm, so a top-down pinch of a 25 mm object needs the fingertips within
~10 mm of the desk. One more, unexplained and worth its own round: on the
bare ground plane (no arena) the probe's boot LAUNCHES the robot — root +12 cm
in the first physics step, head tilt and finger joints at 30–70 rad/s against
their effort caps for ~0.3 s, base airborne to z=1.7 m and down 2 m away,
sometimes on its side — with the safety stop held or released, at spawn_z
0.20 or 0.09. The arena stack never shows it (the bench's base stays at its
spawn xy). The probe works around it (settle 8 s, re-latch the base hold
upright at the origin, then release the safety stop); the cause (a spawn-time
state violation — epsilon-mass frame links? the zero-mass link_tcp with an
undefined centre of mass? ground-plane placement?) is open. Follow-ups: the
exact model is a PhysX loop-closure joint
between inner knuckle and finger (NVIDIA: no native closed loops; the mimic
API is reported broken for parallel grippers on Isaac 5.1), which would make
the parallelogram passive; and sizing the drive effort limit to the 30 N spec.

## 2026-09-01 — close-phase punch, and why the first (open-loop) ramp made it worse

With the mimic coupling stiffened to k=1500 (the fingers finally grip), the
grasp-bench round found a new retention failure: every object — light ones
worst — was ejected from the jaw at *first pad contact*. The finger-contact
force trace showed a spike of 12–190 N on the first sample, before the grip
settled: the k=1500 followers applied the full commanded close target in one
step, so on first touch the position error (and thus `k*error` press) was at
its maximum. A punch, not a squeeze.

First attempt (`21d744c`): slew the applied drive target toward the command at
`_gripper_close_slew` rad/s instead of jumping to it, so the press builds
gradually. It made things worse on two axes, and the reason is the same on
both. An open-loop slew bounds `dF/dt` but never *stops* — past first contact
the target keeps advancing to the fully-closed command. (1) The follower press
therefore still climbs to the effort caps; the measured peak rose to 248 N,
worse than the unramped 190. (2) More subtly, it broke a previously-green path:
all three knife grasps began aborting as "native gripper: execution failed".
The gripper facade keys success on the *measured* joint position vs the fixed
goal (see the entry below): a close that stalls short of the goal is a success
only if it *stalls* — either fresh contact force, or measured position no
longer improving for `stall_dwell_s`. The open-loop ramp keeps the target
creeping, so the followers keep deepening and the measured position keeps
inching down by more than `stall_epsilon` every dwell window — the position-
stall detector never latches, contact-stall alone can't carry the thin knife,
and the close runs out its 5 s `simulation_timeout_s` and aborts. The ramp
defeated the very stall detector the entry below had just added.

Fix (`8cde40f`): make the ramp closed-loop — FREEZE the applied target once
finger-pad grip force reaches `_gripper_contact_halt_force`
(`TINKER_SIM_GRIPPER_CONTACT_HALT_N`, default 15 N). Freezing caps the press
near the halt force instead of the effort caps, and — because the target stops
moving — the measured position flatlines, so the facade's stall (contact and
position both) latches and the grasp reports success. The halt reads the same
quantity the facade sees on `/sim/parity/finger_contact` (the sum of the two
finger-pad normal forces, via `contact_state`), so the halt point and the
facade's contact threshold agree by construction — when the sim stops pressing,
the facade sees exactly that force. Only the closing stroke is force-bounded;
an opening command always slews freely so release stays prompt. `slew <= 0` and
`halt <= 0` each disable their own stage.

The 15 N default was sized off the force trace: good grips form in a 13–41 N
band before the runaway climbs 41 → 100 → 249, and the two grasps that *did*
hold settled at 31/36 N.

Cycle-2 (knife-only) showed the force halt was the wrong instrument, for a
reason that matters: a force threshold on the **contact-report** signal is
unreliable for exactly the object that needs it. Knife completion went 0/3 →
1/3 (the freeze-lets-the-facade-latch direction is right), but the one that
completed **froze at drive 0.148** — a single transient brush during approach
crossed 15 N and latched the halt ~73 mm open on a 30 mm knife, no real pinch —
while a 212 N peak persisted on the aborting closes. Both are the same defect:
a thin object reports contact **sparsely and spikily** (the knife baseline is 3
blips over an entire close), so a force latch both fires on a lone transient and
misses a fast spike that ejects the object within the one-step lag between
setting the target and reading the next contact report.

The fix (bounded lead) drops force sensing from the stopping decision entirely.
Instead of freezing on contact force, cap how far the applied target may lead
the **measured** pad position: `applied = min(slewed, pad_measured + max_lead)`
while closing, where `pad_measured` is the least-closed of the two finger
joints. Follower press is then `k * (target − pad_measured) <= k * max_lead` by
construction — a hard force bound with no runaway — and the moment the pads
stall on the object the target clamps at `pad + max_lead` and stops, so the
unloaded `drive_joint` the facade watches flatlines and its position-stall
latches. It reads measured position, not contact reports, so it is robust for a
thin object; and being a continuous clamp rather than a one-shot trigger, a
transient brush cannot latch it (the brush doesn't stall the pads). The drive
joint is unloaded — the pads carry the object, not the drive — so the drive's
own position can't see the stall; that is why the clamp reads the pad joints
specifically. `max_lead` (`TINKER_SIM_GRIPPER_MAX_LEAD_RAD`, default 0.015 rad)
is the grip-force knob: `k * lead ≈ 1500 * 0.015 ≈ 22 N`. The old force cap
(`TINKER_SIM_GRIPPER_CONTACT_HALT_N`) is kept but defaults **off**, since it was
the source of the transient latch.

Cycle-3 showed the naïve always-on clamp has a fatal flaw of its own: it went
0/3, every knife close **timed out with the drive back at 0.0 — jaw fully
open**. The clamp ratchets the target *backward*. The `lead ≈ press/k`
relationship only holds *quasi-statically*; while the pads are moving,
`target − pad` is the **dynamic tracking lag** (the peer measured 0.02–0.03 rad
in motion, vs ±0.004 rad settled), which is *larger* than the 0.015 lead. So
during the approach `pad + lead = target − 0.025 + 0.015 = target − 0.01`, and
`min(slew, pad+lead)` drives the target down 0.01 every step until the jaw sits
fully open and the facade times out. The unit test had masked this by faking
zero-lag pads — an unphysical `pad == target`.

The fix gates the clamp on stall: apply it only once the pads have nearly
stopped (`max pad speed ≤ _gripper_stall_speed`, default 0.1 rad/s;
`TINKER_SIM_GRIPPER_STALL_SPEED`). Free-close pad speed is ~the slew rate (1.5),
so the gate cleanly separates moving from stalled. While the pads move the clamp
is off and the target slews freely (no backward ratchet); the instant they
stall on the object, speed drops through the gate, the dynamic lag has decayed
to the settled ±0.004, and the clamp bounds the now-quasi-static press at
`k * lead`. `max(current, …)` additionally keeps the close monotonic so the
clamp can never retreat the target even at the moment the gate flips. `slew <= 0`
disables the ramp; `lead <= 0` disables the clamp. The `PhysxMaterialAPI`
compliant-contact spring (`TINKER_SIM_GRIPPER_COMPLIANT_STIFFNESS`) remains as a
softer-first-touch escalation, off by default. Not yet live-validated past
cycle-3 — reasoned safe (startup, slow legitimate motion, one-step velocity lag,
and missing-signal all degrade to bounded press or plain slew, none deadlock);
needs a live trace to confirm no >20 N peak, knife grasps complete without
abort, and the grip holds through the lift.

Cycle-4 (stall-gated clamp) is the decisive dataset, and it closes the reactive
approach entirely: 1/3 completed (drive 0.728, 18 mm *past* the knife), 2/3
timed out, and the force bound broke — three single-sample spikes of 118/228/
221 N against the ~22 N `k*lead` ceiling. The spikes land **during pad motion**,
i.e. while the stall gate is off by design, at first contact. At k=1500 and
1.5 rad/s the finger carries enough momentum that first contact with an 80 g
object is resolved impulsively **inside a single physics step** — the object is
ejected/displaced before the pads can slow, so no stall ever forms (the pads
close through to 0.728, or the facade times out on the disturbed geometry).

The conclusion across all four cycles: **any reactive stop scheme is
structurally one step too late for a light object.** The ejection happens inside
the very step the scheme is waiting to observe — a force latch (cycle-2) and a
velocity/stall gate (cycle-4) both see it only after the fact. The dominant term
is not the PD spring `k*error` but the rigid-contact **collision impulse**: the
solver resolving the moving finger's momentum into the light object in one step.

That moves the fix out of the control loop and into the solver step itself.
The mapped lever ladder, in order:
  1. **Compliant contact on the pads** — the primary. `PhysxMaterialAPI`
     compliant-contact spring (`TINKER_SIM_GRIPPER_COMPLIANT_STIFFNESS`, with
     `_DAMPING`; already wired in `_apply_gripper_friction_material`, off by
     default). It softens the *contact constraint* so the collision impulse is
     spread over several steps instead of one — the only place a one-step event
     can be tamed. It also buys the stall-gated clamp the steps it needs to
     latch, so the two are complementary (compliance softens the impact, the
     clamp bounds the steady hold). Acceleration-spring is on, so the stiffness
     is mass-normalized; a first-cut in the 1e5–1e6 range is the place to start
     an A/B, tuning down until the >20 N spike is gone and up until the grip
     still holds.
  2. **Soft-close k** — drop the mimic follower stiffness during the closing
     phase (k is the impulse multiplier) and restore k=1500 only after settled
     contact. k=200 alone fails the *hold* (e320d5b: the object extrudes at
     0.13–0.17 rad lag), so it must be phase-switched, not lowered outright. The
     runtime write path exists (`write_joint_stiffness_to_sim_index`, as the
     safety hold uses); the restore trigger is the same stall gate (not time-
     critical for the hold). Not yet implemented — it is runtime gain-switching
     against the actuator model and the mirror, so it wants a GPU round to
     develop, not a blind commit.
  3. **Slower slew through the contact band** — reduces the finger momentum at
     impact (`TINKER_SIM_GRIPPER_CLOSE_SLEW`). Bounded by the facade's 5 s
     timeout (a global 0.15 rad/s close would overrun it), so it is an adjunct
     to (1)/(2), not a standalone fix.

Reactive tuning is done: `21d744c/8cde40f/1f6f124/06cc2e1` are all on the branch
as the instrumented record, `06cc2e1` (stall-gated clamp) is the head and the
right *hold*-phase bound, but the *impact* must be solved at the solver level.

**Cycle-5 (compliant contact, VALIDATED lever).** The A/B ran at
`COMPLIANT_STIFFNESS=3e5` / `_DAMPING=1e3` (compliance confirmed engaged, no
`gripper_compliant_error` in the boot log). Peak first-contact force
**228.4 → 156.6 N (−31%)**, and — the important part — the trace *character*
changed: sustained mid-range samples (40/60/59/45/90 N) replaced cycle-4's
isolated 118–228 N extremes, i.e. the collision impulse is genuinely spreading
over steps rather than resolving in one. Lever 1 is the right one and the
response is monotonic — a two-point curve now exists (rigid 228 N
`force-trace-gate.txt` → 3e5 gives 157 N `force-trace-compliant.txt`, both in
`results/2026-09-01-retention-campaign/part3-evidence/`). Completion held at
1/3, held 0/3: 3e5 alone doesn't yet clear retention. Two follow-ons, both env
knobs (no code change — the levers are all exposed):
  - Go **softer**: `COMPLIANT_STIFFNESS=1e5` next (expect roughly proportional
    softening toward the <20 N target); add lower `CLOSE_SLEW` through the
    contact band if compliance alone stalls above 20 N.
  - **Retune the stall gate for compliant contact.** The one completing close
    overshot to drive 0.458 (~10 mm past the knife) because the hold-phase clamp
    latched late: under compliance the pads decelerate *gradually* rather than
    stalling sharply, so the rigid-sized `_gripper_stall_speed = 0.1 rad/s`
    catches the stall too late. Raise `TINKER_SIM_GRIPPER_STALL_SPEED` so the
    clamp latches earlier on the gentler deceleration (watch it does not latch
    during the free-close velocity ripple). This is the coupling between the two
    fixes: compliance changes the deceleration profile the hold-phase gate keys
    on.

Next window: `COMPLIANT_STIFFNESS=1e5` with the two-point curve as the guide,
then raise `STALL_SPEED` to kill the hold overshoot; reassess soft-close
(lever 2) only if compliance + slew can't reach <20 N with a holding grip.

## 2026-09-01 — contact-free gripper stall (grasps aborting in the sensor-rich profile)

The grasp benchmark's real closes were all aborting as "native gripper:
execution failed" — every close that physically stalled on an object timed
out instead of succeeding (run went 0/3 on bottles). Root cause is a
profile-parity gap, not a grasp-planning fault. The gripper facade
(`ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/gripper_facade.py`)
recognizes a successful grasp two ways: the finger reaches its commanded
position (`reached_goal`), or it stalls short of it against an object
(`stalled`). The stall test required a *fresh contact force* >=
`contact_force_n` on `/sim/parity/finger_contact`. But the sensor-rich
profile — the one that actually runs GPSR grasps — builds its backend with
`enable_contacts=False` (`validation/run_sim.py`), so that parity topic is
structurally silent there. `manipulation-core` *enforces* contacts
(`run_sim` raises "must enable contacts"); sensor-rich disables them, most
likely for RTF at camera cadence. So a real grasp in sensor-rich stalls the
fingers on the object (correct physics), produces no contact telemetry, and
the facade can only time out.

Why not just turn contacts on in sensor-rich: `activate_contact_sensors`
(`backend.py`) is an all-or-nothing flag on the whole `/World/Tinker` spawn
and the contact-report subscription is robot-global — there is no
finger-only contact path today, so it would mean paying robot-wide contact
reporting in the camera-bound loop. And it is unnecessary: the *only*
real-time consumer of `finger_contact` is the facade's stall test. The
qualification gate verifiers (`validation/integrated_gate_verifier.py`,
`manipulation_gate_verifier.py`) also read contact force, but they run under
`manipulation-core`, where contacts are on — so they are untouched.

Fix: give the facade a contact-free stall path, which is what a real gripper
driver does anyway — detect that the finger has stopped advancing toward its
target while still short of it. A close whose best distance-to-target has
not improved by `stall_epsilon` for `stall_dwell_s` (default 0.3 s) while
still outside `position_tolerance` is declared `stalled`. It is additive to
the contact path (`contact-stall OR position-stall`), so `manipulation-core`
keeps its contact semantics and the two coincide on a real grasp there; the
position path is what carries sensor-rich. Set `stall_dwell_s <= 0` to
disable it. Enabled by default because a profile that cannot complete a real
grasp is not a defensible default — this supersedes the interim
`TINKER_SIM_SENSOR_RICH_CONTACTS` env-gate the grasp-bench session used to
unblock. No launch change: the GPSR/manipulation launches pass no override,
so the default applies once the bridge is rebuilt. A free close still exits
via `reached_goal` (the finger reaches target before any dwell elapses); an
already-touching close reports stalled after one dwell. Verified with the
`test_gripper_executor_humble.py` suite under the Humble overlay (8 passed,
3 consecutive runs), including a new parked-finger-stalls-without-contact
test; the three tests that deliberately park the finger for an orthogonal
concern (cancel, safety, stale-contact) now disable the path explicitly.
`manipulation-core` qualification should get one confirmatory run since its
facade result now has a second success route (gate verdicts are unchanged —
the verifiers read the raw contact topic, not the facade result).

## 2026-08-31 — joint4 tuck stall, physics-less YCB objects, wall-clock safety deadlines

Root-cause round for the sim bugs blocking the GPSR battery and the grasp
benchmark (branch `task-sim-bugfixes`, commits 8e4caa4, 53bc502, 9b5fe4a,
e5d4312). All measurements from in-process manipulation-core probes (CPU
PhysX, contacts on, no bridge; probe scripts under the session job dir).

**joint4 blocked near tuck (every tuck trajectory aborting).** The elbow sat
pinned at its 50 Nm `effort_limit_sim` with zero velocity, up to 0.04 rad
short of the orchestrator's tuck target. Ruled out in order, one variable
per boot: link contact (contact reporting enabled: zero pairs at the
stall), PhysX joint friction (the USD authors `physxJoint:jointFriction=1.0`
on all arm joints and Isaac Lab's own `data.joint_friction_coeff` reads 0.0
— its new-style friction-properties write path never reaches this PhysX
build's live coefficient — but zeroing the live coefficient via
`set_dof_friction_coefficients` left the stall bit-identical), and the fused
actuator model (`TINKER_SIM_STOCK_ACTUATOR_MODEL=1` bit-identical). The
tell: `get_dof_projected_joint_forces` at the stall reads 50.0 Nm on joint4
— a genuine static load at the cap. `data.default_mass` shows why: the
URDF->USD importer applies `MassAPI` with inertia but no authored
`physics:mass` to every link the URDF declares without `<inertial>`, and
PhysX then defaults each to 1.0 kg. tinker2 uses 21 such links as pure
frames — ~11 kg hanging off the wrist (link_eef, link_tcp, nine
xarm-camera frames), ~10 kg on the head. Real elbow-downstream mass is
~3.6 kg (<= 15 Nm), matching the earlier estimate that had "ruled out"
gravity from the *authored* masses. Fix: `_apply_stub_link_masses` authors
1 g at spawn on exactly the links the artifact's colocated `robot.urdf`
declares inertial-less and collision-less (data-driven, no name list),
beside the ballast/wheel corrections. Verified: joint4 tuck error 0.039 ->
0.0019 rad (= real ~13 Nm gravity / 7000 stiffness), stable across cycles.

Two side findings. `/joint_states` effort is **stale telemetry**:
`applied_torque` refreshes only when the target-write gate writes, so the
published effort freezes at the last mid-transient value (a saturated 50.0,
or ~1e-16 after a hold) — judge tracking by position. And the "arm ignores
all trajectories for 240 s after ~3 aborts" degraded state was never a
drive fault: see the deadline item below.

**Every YCB object was static scenery.** `ycb_import` published each
`object.usd` with colliders but no `RigidBodyAPI` on the default prim
(violating the repo's own spawnable-asset contract,
`simulation/assets/primitives/task-object.usda`), and `spawn_entity` adds
no physics APIs. So no scenario object ever resolved a rigid-body view: no
ground-truth pose in physics-truth frames, ungraspable. Only `soup` errored
(~17k physx pattern-miss lines/session) because `create_rigid_body_view`
raises after logging and the discovery loop's broad `except` aborted the
whole pass — soup was merely first in dict order; mug/banana/bowl failed
silently behind it. The old developer-log claim that these messages were
"benign shutdown noise" was wrong (first hit is ~40 s into the run, when
scenario_runner spawns). Fixes: `author_object_rigid_body` in the importer
(mass stays density-derived from the collision hulls — the soup can
computes to ~0.35 kg vs 0.349 kg published); `ycb_import --repair-physics
[--root]` republishes the current artifact Kit-free (deterministic
identity 5117994887a1...) and repoints `asset-manifest.json`; scenarios
reference the repaired identity; the discovery loop is per-object, logs one
`rigid_body_missing` diagnosis, and backs off 20x. Verified in-process: a
repaired soup resolves, reports a truth pose, and settles under gravity; a
deliberately broken sibling logs once and blocks nothing. Behavior change
flagged to the battery/bench sessions: YCB objects now settle and can be
knocked over.

**Wall-clock liveness deadlines re-latched the limp hold.** The gateway's
safety-heartbeat (1.0 s) and command-stream (0.5 s) deadlines were pure
wall clock while the publishers live in separate processes: an RTX render
stride stalls the stepping loop for multiple wall seconds with healthy
samples queued in DDS, and the gateway then re-latched the limp safety
hold / invalidated the command stream on every stride (grasp-bench report;
the GPSR battery ran the 1.0/0.5 s defaults, making this the prime suspect
for its post-abort dead-arm state). Deadlines now require staleness in
BOTH wall and simulation time (`9b5fe4a`): sim time freezes exactly when
the loop stalls, so the loop cannot punish itself; a dead publisher still
trips within one simulated timeout while stepping; wall age still gates
faster-than-realtime runs. The 59b9d7e env overrides remain as escape
hatch.

**Operator traps (grasp-bench reports #4/#5, e5d4312).** World mode
`current` with no `--arena` plus declared spawns now prints a
`world_selection_warning` (a full benchmark run was lost to a silently
bare ground plane), and `pick-deliver-place` moved to the validated free
corridor — its old robot (0,0) and object (0.65, 0, 0.8) poses both sit
inside shelf_02's rasterized footprint in the rcw2026 arena map.

**Head DEPTH "freeze": not reproducible on the current build.** A live
discriminator probe (in-process rig, `level-forward` aim + 3 cm dolly
active, Kit pumped per capture, five pan/tilt poses through pan 180 deg)
shows head depth tracking every pose change — valid fraction 0.47 (level,
empty world) -> 0.97 (tilt 30 down) -> 0.00 (tilt 30 up: sky, all samples
out of range -> all-zero by the 16UC1 contract) -> 0.74 -> 0.90 — with
fresh buffers whenever the scene changes. No housing occlusion from the
dolly at any probed pose, no stale annotator. Two benign behaviors imitate
a freeze: an out-of-range aim yields constant all-zero frames while color
keeps rendering, and a static scene yields byte-identical depth while RGB
still dithers (AA sampling), so "depth unchanged, color changing" is the
*expected* static-scene signature. The 2026-08-27 field report also
predates the pan/tilt `/joint_states` fix (4e9c694): with pan/tilt
commands being rejected, the head physically never left its boot pose, so
its depth naturally never changed while the wrist camera (riding the
moving arm) looked alive. If a freeze recurs post-4e9c694, capture
`TINKER_SIM_CAMERA_DEBUG=1` (annotator buffer-pointer churn) plus a
per-frame depth valid-fraction before filing.

Probe hygiene note for future camera probes: RTX render products tick on
Kit `app.update()` (run_sim's `_pump_streaming_app_update`), NOT on
`SimulationContext.render()` — a probe that skips the Kit pump sees every
camera frozen at the boot frame, color included.

**Spawned objects pass through the gripper (grasp-bench run 5, and the
likely root of live-manip's 100% referee fallback).** Definitive
in-process discriminator (overlap probe, cube spawned intersecting
left_finger, contact pairs accumulated every physics step): on a timeline
that has NEVER been stopped, a mid-play `/spawn_entity` body pairs fully
with the articulation -- 261 N depenetration contact on the finger. After
any timeline STOP -> PLAY cycle, bodies spawned during or after the cycle
pair only with static geometry and free-fall straight through the fingers
with zero contact events. The stack always enters that poisoned regime at
boot: scenario_runner runs `reset_spawned` (itself a stop->play, per the
documented ResetSimulation scope bug) plus the SPAWN_READY state-0 stop
before spawning, so every later bench/command spawn is ungraspable. Fix
(`b37b67a`): `standard_operations(spawn_while_playing=True)` spawns
everything onto the still-playing first-run timeline (no reset_spawned, no
state-0; the final state-1 op stays as a no-op play carrying the
PHYSICS_READY payload); plumbed as `scenario_runner --spawn-while-playing`
/ `TINKER_SIM_SPAWN_WHILE_PLAYING=1`. Deliberately opt-in until the A/B
baselines that assume the old boot sequence are re-cut; end-to-end
validation is the battery's s2026-000 live-manip rerun. Caveats: earlier
probe iterations that judged collision by "object reached the floor" were
worthless (a cube bounces off the narrow gripper to the floor anyway, fast
falls tunnel, and 0.5 s contact sampling misses pairs removed on
CONTACT_LOST) -- accumulate per-step contact maxima and use overlap
spawns. Same defect family, still open: `/get_entity_state` returns the
frozen spawn pose for such bodies (sim_control's RigidPrim binds against
SimulationManager's cached warp sim view, which never re-binds; observed
stale even on a fresh timeline in-process). Consumers should read gateway
physics-truth instead; `force_load_physics_from_usd` as a repair is
destructive (invalidates every live tensor view) -- do not use it mid-run.

**Wrist camera aim: same description defect class as the head, exactly
90 deg (`e821e79`).** Found by the first live s2026-000 run with the
collision fix armed: the wrist frame at the table-scan pose renders the
ceiling. The artifact's robot.urdf FK proves it: the camera optical axis
sits 90 deg from the tool approach axis at every configuration (joint
zeros: gripper -90 deg, camera level; scan pose: TCP -48 deg, camera
+42 deg up). The wrist camera stub frames are among the same hand-authored
inertial-less links as the phantom-mass defect; the real robot survives
because hand-eye calibration, not URDF TF, supplies its grasp extrinsics.
Sim correction mirrors the head one: `TINKER_SIM_WRIST_CAMERA_AIM=
tool-forward` (+90 deg about the optical frame's own +X = render axis onto
the TCP forward), opt-in, parity-breaking, set by gpsr-stack. Watchpoint
for consumers: if a pipeline derives wrist extrinsics from URDF TF instead
of calibration, corrected images now disagree with that TF by 90 deg --
good detections at wrong map positions is the signature.

**MDL-bound spawns render as nothing (`f8e764e`).** The vanished
cmd_spam_0 — tracked physically rock-stable at its desk pose for 57 s
(`TINKER_SIM_TRACK_OBJECTS`, `bad2693`) while the correctly-aimed wrist
camera saw an empty desk — was a MATERIALS defect: a prim spawned onto a
playing stage renders as NOTHING when its material is MDL (any MDL;
textured, untextured, opaque, or the converter's own authored-transparent
`OmniPBR_Opacity` with `opacity_constant=0.0`), while the identical mesh
with no material or a `UsdPreviewSurface` network renders correctly,
textures included. Ruled out along the way, one boot each: the spawn pose
(settles to 0.1 mm), slot overlap (single-item manifest; though two
overlapping spawns DO skitter violently across a desk — a real hazard for
placement planning), the sim_control service pose write, delete/respawn,
and file format (usda vs crate identical). Boot-parse objects rendered
under the old stop-bracket boot, masking this for furniture and
pre-battery scenario objects. Fix: `author_preview_surface_material`
rewrites MDL materials in place to UsdPreviewSurface networks (Material
prim path kept, bindings stay valid); `--repair-physics` is now a
physics+materials repair; round-2 artifact identity `4b635c93c704...`.
Verified live: mid-play-spawned spam and soup cans render fully textured
in the wrist view. Probe lessons: compute camera-frame placement from the
aim geometry before trusting an "invisible" verdict (two boots went to
out-of-frame layouts), and `force_load_physics_from_usd` mid-run destroys
every live tensor view.

**Viewed-prim deletion kills the boot; the backend now recovers
(`90ad061`, `a078da7`, `2c6f587`).** Two whole-boot kills in one evening,
same class: deleting a prim that ANY live tensor view covers invalidates
the SHARED SimulationView ("prim ... was deleted while being used by a
tensor view class"), after which every articulation read/write raises and
/spawn_entity is dead. First kill: the spawn-attach healer's per-spawn
probe views (`a078da7`'s original form) at a routine multi-entity clear.
Second: the referee hand_object's cached write view at its post-run
clear. Measured facts that shape the fix: the physics.tensors views have
NO release API, and even a dropped, garbage-collected Python view leaves
the backend registration alive -- so create-use-drop is NOT safe, and
avoidance alone cannot protect against every component. Layered fix:
(1) everything that watches DELETABLE spawns is view-free via
``IPhysx.get_rigidbody_transformation`` (healer, tracker); discovery keeps
views for scenario-boot objects under the documented invariant that those
are never deleted mid-play; (2) the backend self-recovers from the
invalidation: de-initialize the articulation (its PHYSICS_READY handler
early-returns while it believes itself initialized), rebuild the
PhysxManager AND isaacsim SimulationManager views replicating only their
creation lines (their own warmup paths force_load the stage and snap
every body to its authored pose -- unusable), re-initialize directly (the
event-bus path stores callback exceptions silently), re-anchor the
monotonic /clock (the physics step counter resets with the views), force
the next target write; budget 5 per boot. Verified live: two consecutive
held-view deletes each recovered in one attempt with the arm holding its
commanded pose and a mid-fall can landing DURING recovery. Also fixed
along the way: the spawn-attach healer itself (about 1 in 3 mid-play
spawns never enters PhysX -- per-spawn nondeterministic omni.physx parse
race; active-toggle re-parse nudge, TINKER_SIM_HEAL_DETACHED_SPAWNS).

**Still open.** Head + wrist camera aim: the description-level defects
need measurement on the physical robot; the env-gated sim corrections
remain the sanctioned workarounds. `/get_entity_state` staleness above.

## 2026-08-29 — Arena camera RTF: Phase 0 measurement

Throwaway measurement (`scripts/arena-rtf-spike`, sim-only, GPU 1, no
bridge): six 120 s-sim variants with `TINKER_SIM_PROFILE=1`, profiled every
10 camera cycles, to attribute the arena observer camera's RTF hit (~0.8 ->
~0.24 seen live) to render cost, DLAA, resolution, or the ROS publish path.

Preflight (GPU 1 reserved by prior agreement): worktree had no other
`run_sim.py` on GPU 1 and ≥5 GB free —

```
nvidia-smi --query-gpu=index,memory.used,memory.total,utilization.gpu --format=csv
1, 2498 MiB, 11264 MiB, 21 %
```

```
$ cat /proc/loadavg
8.72 10.60 11.89 3/3232 2861952
```

Note: this worktree was missing the `.cache`, `.deps`, `.venv` symlinks that
sibling `.claude/worktrees/*` checkouts carry to the shared Isaac/uv caches
(it had a partial real `.cache/uv` and an empty `.venv` instead, both
artifacts of the harness's first, failed attempt). All six variants failed
in seconds with `Isaac Sim internal Humble libraries were not found in the
locked environment` until those three symlinks were recreated to point at
the main checkout's `.cache`, `.deps`, `.venv` (same targets the sibling
worktrees use); the measurement below is from the re-run after that fix.

Six logs, six clean completions — no crash signatures (`error 700`,
`illegal memory access`, `cudaErrorMemoryAllocation`, `Traceback`) in any
log, no non-zero variant exits, and every variant well past the ≥20-window
sanity floor after the 30 s warm-up (119-128 windows each):

| variant | windows | kit_pump ms | cameras ms | physics ms | wall ms | RTF |
|---|---|---|---|---|---|---|
| A_arena_off | 119 | 41.0 | 16.1 | 50.0 | 123.4 | 0.68 |
| B_arena_2hz | 128 | 103.1 | 18.3 | 51.8 | 190.3 | 0.44 |
| C_capture_skip | 127 | 104.0 | 16.2 | 53.5 | 190.4 | 0.44 |
| D_dlaa_only | 127 | 90.9 | 15.4 | 50.9 | 175.3 | 0.48 |
| E_arena_640 | 127 | 98.3 | 16.1 | 50.9 | 183.2 | 0.46 |
| F_arena_0p5hz | 127 | 90.6 | 17.5 | 51.8 | 178.1 | 0.47 |

Per-variant notes:

- **A_arena_off** — baseline, arena camera fully disabled. RTF 0.68, the
  best of the six as expected.
- **B_arena_2hz** — arena camera on at 2 Hz, default resolution. RTF drops
  to 0.44; `kit_pump` more than doubles over A (+62.1 ms).
- **C_capture_skip** — same as B but `TINKER_SIM_CAPTURE_SKIP=arena_camera`
  (render still runs, ROS publish of the frame is skipped). `kit_pump`
  104.0 ms, essentially identical to B (104.0 vs 103.1) and nowhere near A
  (41.0) — the publish/bridge path is not where the cost lives.
- **D_dlaa_only** — arena camera *off*, only `TINKER_SIM_STABLE_AA=1`
  forced. `kit_pump` 90.9 ms — most of B's hit reproduced with the camera
  disabled entirely.
- **E_arena_640** — arena camera on at 2 Hz, resolution cut to 640x360.
  `kit_pump` 98.3 ms, only 4.8 ms better than B — resolution is not the
  lever.
- **F_arena_0p5hz** — arena camera on at 0.5 Hz (quarter the capture rate
  of B). `kit_pump` 90.6 ms, close to B (within 12%) — capture rate is not
  the lever either, and it lands right next to D.

Decision rule (all deltas in `kit_pump` ms, skip-first-30s means):

- `B - A` = 103.1 - 41.0 = **62.1 ms**
- `D - A` = 90.9 - 41.0 = **49.9 ms** = **80.3% of (B - A)** — clears the
  ≥60% bar for "DLAA is the tax" outright.
- `E` recovery vs `B`: (B-A) - (E-A) = 62.1 - 57.3 = 4.8 ms, i.e. **7.7%**
  of the gap recovered by dropping resolution — far short of the ≥60% bar
  for "resolution is the lever."
- `F` vs `B`: |90.6 - 103.1| / 103.1 = **12.1%**, within the 15% "F ≈ B"
  band, but `D` is not small (it is 80.3% of the gap), so the "fixed
  per-product cost" reading does not hold either.
- `C` vs `A`: 104.0 vs 41.0 — not close; `C` sits with `B` (104.0 vs 103.1,
  0.9% apart) instead, so the cost is not in `publish_cameras`.

`D - A` clearing 60% of `B - A` fires first and on its own: turning on
`TINKER_SIM_STABLE_AA` alone, with the arena camera left off, reproduces
80% of the arena camera's RTF hit. Neither resolution (E) nor capture rate
(F) meaningfully recovers it, and skipping the publish step (C) recovers
none of it. DLAA is the tax.

Decision: Task 4a

### Phase 1a — scoping the DLAA pin

Task 4a: stop the global `/rtx/post/aa/op` DLAA pin from taxing the
hardware-parity (head/wrist) cameras' render cost while keeping the arena
camera's CUDA-700 workaround intact. Two sub-approaches, tried in order,
per-render-product override first.

**Step 1 — probe (04:11:50 EDT, sensor-rich, `TINKER_SIM_ARENA_CAMERA=1
TINKER_SIM_CAMERA_DEBUG=1`, GPU 1, 20 s sim duration, clean exit).** A
temporary print in `CameraRig.initialize` after each `CameraSensor` is
created, listing every `aa`/`dlss`-named attribute already authored on that
camera's `UsdRender.Product` prim:

```
[probe] head_camera /Render/OmniverseKit/HydraTextures/camera_sensor_7993988978446 ['omni:rtx:dlss:frameGeneration', 'omni:rtx:post:aa:autoExposureMode', 'omni:rtx:post:aa:exposure', 'omni:rtx:post:aa:exposureMultiplier', 'omni:rtx:post:aa:limitedOps', 'omni:rtx:post:aa:op', 'omni:rtx:post:aa:sharpness', 'omni:rtx:post:dlss:execMode', 'omni:rtx:post:dlss:manualScaling', 'omni:rtx:post:fxaa:quality:edgeThreshold', 'omni:rtx:post:fxaa:quality:edgeThresholdMin', 'omni:rtx:post:fxaa:quality:subPixel', 'omni:rtx:post:taa:alpha', 'omni:rtx:post:taa:colorBoxSigma', 'omni:rtx:post:taa:samples', 'omni:rtx:pt:dlss:enabled', 'omni:rtx:scene:GPUProceduralAABB:enabled']
[probe] wrist_camera /Render/OmniverseKit/HydraTextures/camera_sensor_7993988978335 ['omni:rtx:dlss:frameGeneration', 'omni:rtx:post:aa:autoExposureMode', 'omni:rtx:post:aa:exposure', 'omni:rtx:post:aa:exposureMultiplier', 'omni:rtx:post:aa:limitedOps', 'omni:rtx:post:aa:op', 'omni:rtx:post:aa:sharpness', 'omni:rtx:post:dlss:execMode', 'omni:rtx:post:dlss:manualScaling', 'omni:rtx:post:fxaa:quality:edgeThreshold', 'omni:rtx:post:fxaa:quality:edgeThresholdMin', 'omni:rtx:post:fxaa:quality:subPixel', 'omni:rtx:post:taa:alpha', 'omni:rtx:post:taa:colorBoxSigma', 'omni:rtx:post:taa:samples', 'omni:rtx:pt:dlss:enabled', 'omni:rtx:scene:GPUProceduralAABB:enabled']
[probe] arena_camera /Render/OmniverseKit/HydraTextures/camera_sensor_7993988977774 ['omni:rtx:dlss:frameGeneration', 'omni:rtx:post:aa:autoExposureMode', 'omni:rtx:post:aa:exposure', 'omni:rtx:post:aa:exposureMultiplier', 'omni:rtx:post:aa:limitedOps', 'omni:rtx:post:aa:op', 'omni:rtx:post:aa:sharpness', 'omni:rtx:post:dlss:execMode', 'omni:rtx:post:dlss:manualScaling', 'omni:rtx:post:fxaa:quality:edgeThreshold', 'omni:rtx:post:fxaa:quality:edgeThresholdMin', 'omni:rtx:post:fxaa:quality:subPixel', 'omni:rtx:post:taa:alpha', 'omni:rtx:post:taa:colorBoxSigma', 'omni:rtx:post:taa:samples', 'omni:rtx:pt:dlss:enabled', 'omni:rtx:scene:GPUProceduralAABB:enabled']
```

`omni:rtx:post:aa:op` is authored on every render product individually →
approach (i), per-render-product scoping, is possible. Probe removed
before commit.

**Approach chosen: (i), per-render-product override.** Added
`CameraRig.initialize(..., stable_aa_cameras: frozenset[str] | None = None)`:
when given, the global `carb.settings` call is skipped and
`omni:rtx:post:aa:op` is instead set directly on the named cameras' own
render-product prims (`run_sim.py` passes `frozenset({"arena_camera"})`).
`stable_aa_cameras=None` keeps the old global-setting path byte-for-byte,
so any caller that doesn't pass it (there are none left after this change)
sees no behaviour change.

**Implementation bug found and fixed along the way.** The first attempt
authored the attribute with the same int the global carb setting takes
(`AA_OP_DLAA = 4`) via `GetAttribute("omni:rtx:post:aa:op").Set(4)`. This
surfaced immediately, not as a CUDA-700 recurrence but as a Python
exception during the first crash-recipe attempt (`up start` 04:15:30 EDT,
`gpsr_stack_logs/20260829T041530/01-sim.log`):

```
pxr.Tf.ErrorException:
	Error in 'pxrInternal_v0_25_11__pxrReserved__::UsdStage::_SetValueImpl' at line 7058 in file /builds/omniverse/usd-ci/conan/src/0.25.11.kit.2/pxr/usd/usd/stage.cpp : 'Type mismatch for </Render/OmniverseKit/HydraTextures/camera_sensor_8499610349849.omni:rtx:post:aa:op>: expected 'TfToken', got 'int''
```

Unlike the global carb setting, the per-render-product `omni:rtx:post:aa:op`
USD attribute is token-typed, with a *five-token* enum
(`["none", "taa", "fxaa", "dlss", "rtxaa"]`, from
`omni.usd.schema.render_settings.rtx`'s `generatedSchema.usda`) that
doesn't literally spell "dlaa". That extension's own test
(`test_render_settings.py::test_carb_command_line`) asserts
`settings.get("/rtx/post/aa/op") == 4` maps to token `"rtxaa"` — confirmed
as the DLAA-equivalent token, added as `AA_OP_DLAA_TOKEN = "rtxaa"` in
`camera_rig.py` and used for the per-product `.Set()` call instead of the
int constant. A follow-up 20 s sanity run (04:26:11 EDT, same recipe as
the Step 1 probe) completed cleanly with the fix in place, before starting
the real crash recipe.

**Step 5 — crash recipe, 3×5 min, `./scripts/gpsr-stack up --scenario
gpsr-rcw2026-bench --sim-gpu 1` (arena camera on by default in this
recipe), teardown via `./scripts/gpsr-stack down` between runs, GPU 1
confirmed free before each:**

| run | up start (EDT) | bring-up | sim watch window | teardown | result |
|---|---|---|---|---|---|
| 1 | 04:28:29 | 72 s (rc=0) | 300 s total, clean | 04:33:28 → 04:33:39 | no `error 700` / illegal memory access / Traceback in `gpsr_stack_logs/20260829T042828/01-sim.log` |
| 2 | 04:34:04 | 72 s (rc=0) | 300 s total, clean | 04:39:04 → 04:39:17 | no `error 700` / illegal memory access / Traceback in `gpsr_stack_logs/20260829T043404/01-sim.log` |
| 3 | 04:39:36 | 86 s (rc=0) | 300 s total, clean | 04:44:36 → 04:44:47 | no `error 700` / illegal memory access / Traceback in `gpsr_stack_logs/20260829T043936/01-sim.log` |

Clean 3/3. All three sim logs show `[sim] arena camera enabled at 2 Hz,
960x540 -> /sim/arena_camera/image_raw` and run the full 300 s watch
window before teardown; the only errors present are unrelated benign
shutdown-time warnings (`Pattern '/World/Scenario/soup' did not match any
rigid bodies`, a physx-tensors cleanup message, present in all three runs
of Task 3's Phase 0 sweep too).

**Re-measure (sim-only, GPU 1, `TINKER_SIM_PROFILE=1`, variants A and B of
`scripts/arena-rtf-spike`, run by hand — `outputs/rtf-remeasure-4a/run_ab.sh`,
not committed):**

| variant | windows | kit_pump ms | cameras ms | physics ms | wall ms | RTF |
|---|---|---|---|---|---|---|
| A_arena_off | 118 | 37.1 | 13.6 | 49.0 | 115.9 | 0.72 |
| B_arena_2hz | 121 | 44.1 | 15.7 | 50.5 | 127.1 | 0.66 |

`B - A` = 44.1 - 37.1 = **7.0 ms**, down from Task 3's 62.1 ms — a shrink of
55.1 ms, exceeding the 49.9 ms (`D - A`) predicted shrink from Task 3's
decision rule (some of the gap is run-to-run variance: `A` itself came in
4 ms lower than Task 3's 41.0 ms baseline). RTF for the arena-camera-on
variant recovers from 0.44 (Task 3, global pin) to 0.66 (scoped pin),
close to the arena-camera-off baseline's 0.72. The scoped pin removes
essentially all of the DLAA tax from the parity cameras while the arena
camera itself still gets DLAA (crash workaround intact, confirmed by the
3/3 clean crash recipe above).

### Phase 2 — arena defaults

Task 5: lower the arena camera's defaults from 4 Hz / 960x540 to
`ARENA_CAMERA_DEFAULT_HZ = 2.0` and `ARENA_CAMERA_DEFAULT_SIZE = (640, 360)`
(`simulation/tinker_sim_isaac/arena_camera.py`). Justification is Phase 1a's
re-measure, not Task 3's `E`/`F` variants (those were Phase 0 measurements
taken under the *old* global DLAA pin, before the scoped-pin fix landed —
`F_arena_0p5hz` alone moved `kit_pump` by 12.5 ms, RTF 0.44 → 0.47, so they
are not evidence of a small effect post-fix). The real justification: with
the DLAA fix in place, Phase 1a's re-measure shows the arena camera's
*entire* residual cost is ~7 ms of `kit_pump` (post-fix `B - A`: 44.1 vs
37.1 ms), so its capture rate and resolution individually cannot move RTF
by more than that either way — lowering the defaults costs nothing
measurable to give up, not a real tradeoff. 640x360 was chosen because a
bird's-eye frame at that size still shows a recognisable person and table,
which is all a bird's-eye evidence frame needs. `tools/contact_sheet.py`
was checked (`grep -n "960|540|arena"`)
and left untouched: its `960` is the judge sheet's own 3-tile layout width
(`TILE_W = 320`), not a hard-coded arena frame size — each tile is resized
from the loaded frame's actual `width`/`height` (`contact_sheet.py:313`),
so it already adapts to any arena frame size. Step 3 (live visual check of
a 640x360 frame with a person and table recognisable, `TINKER_SIM_ARENA_CAMERA=1`
on GPU 1) is deferred to Task 7's stack run — GPU 1 was in use by another
session's benchmark stack at the time of this task.

### Phase 3 — bench policy

Task 6: `gpsr-stack up` now takes `--evidence` to opt into the arena camera; pass/fail batteries run arena-off by default.

### Phase 4 — verification on the bench stack

Task 7: verified the fix on the real hybrid stack (bridge + Nav2 + vision
attached, `--manipulation mock`), not just sim-only. `TINKER_WS`/GPU 1,
`gpsr-rcw2026-bench`, `scripts/gpsr-stack up/down`.

**Regression guard (Step 1).** Task 4 landed approach (i) (per-render-product
override), so the guard is a module constant:
`STABLE_AA_CAMERAS = frozenset({"arena_camera"})` in `validation/run_sim.py`,
used at the `camera_rig.initialize(...)` call site instead of the inline
literal, with `test_stable_aa_cameras_is_arena_only`
(`tests/test_run_sim_arena_wiring.py`) pinning it. All four camera test
files: 51 passed.

**Before/after, bridge-attached, idle (simulated-sec / wall-sec, 3 samples
each, ~1-2 min apart via `/clock`):**

| configuration | sample 1 | sample 2 | sample 3 | mean RTF | GPU 1 util (nvidia-smi) |
|---|---|---|---|---|---|
| before (bench session, earlier 2026-08-29) | 0.21 | — | 0.24 | 0.21-0.24 | 98% |
| after, arena camera on (`--evidence`) | 0.544 | 0.592 | 0.594 | **0.577** | 45-53% (~0.65 GB more VRAM than off) |
| after, arena camera off (control) | 0.643 | 0.692 | 0.594 | **0.643** | 48-53% |

Raw samples (arena on, `up --scenario gpsr-rcw2026-bench --sim-gpu 1
--evidence`, logs `gpsr_stack_logs/20260829T052441`): 233->244 sec / 20.225 s
wall = 0.544; 297->309 sec / 20.266 s = 0.592; 320->332 sec / 20.220 s =
0.594. Arena off (control, no `--evidence`, logs
`gpsr_stack_logs/20260829T053421`): 25->38 sec / 20.213 s = 0.643; 47->61
sec / 20.241 s = 0.692; 69->81 sec / 20.218 s = 0.594. (Method note: `grep
sec` also matches `nanosec:` lines, since `nanosec` contains `sec` as a
substring — anchored to `grep '^sec:'` before `sed -n '1p;$p'` so the "last"
line is actually the last `sec:` value, not the last message's `nanosec:`.)

Both configurations (4a arena-on-with-evidence and the arena-off control)
were taken; the brief's 4b variant (a lower-cost mitigation short of the
scoped-pin fix) was not needed — the scoped DLAA pin from Task 4a's Phase 1a
already recovers RTF from 0.21-0.24 to 0.58 with `--evidence` on the real
stack, well clear of the ≥0.5 "GPSR runnable" floor, so there was nothing
left for a second, weaker mitigation to buy.

GPU 1 utilisation stayed in the 45-53% band across every sample in both
configurations — nowhere near the 98% seen before the fix (arena-on no
longer saturates the card; the remaining draw is the vision stack sharing
GPU 1, not the arena render).

**Crash recipe.** No re-run needed for this task — Phase 1a's Step 5 already
recorded 3/3 clean 5-minute bring-up/watch/teardown cycles with the arena
camera on and the scoped DLAA pin in place (zero `error 700` / illegal
memory access / `Traceback` in any of the three `01-sim.log`s). This task's
own two stack-up logs (`20260829T052441`, arena-on-with-evidence;
`20260829T053421`, arena-off) add two more clean runs with no CUDA-700
signature, for a combined 5/5 across both tasks.

**Policy.** `--evidence` is the one-line policy: contact-sheet/evidence runs
opt into the arena camera (and pay the RTF cost above); pass/fail batteries
run arena-off by default and are unaffected.

**Visual check (deferred from Phase 2/Task 5).** With arena-on up:
`/sim/arena_camera/image_raw` reports `width=640`, `height=360`,
`encoding=rgb8` (matches Phase 2's 640x360 default). `image_view` is not
installed on this box, so a frame was saved via a small `rclpy` +
`sensor_msgs/Image` + PIL script (`cv_bridge` is broken in this env — built
against numpy 1.x, current numpy is 2.x) to `outputs/arena-640-check.jpg`
(gitignored, not committed). Visual inspection: a bird's-eye view of the
arena floor plan is legible, with a round dining table and two chairs
clearly recognisable, and two humanoid figures clearly recognisable — one
standing near the table, one standing near a doorway at the bottom of the
frame. Confirms Phase 2's 640x360 choice.

**Follow-up, not done.** The spec's Phase 2 item 3 (gating the head camera's
12 Hz publish on having a live subscriber) was not pursued here. Reason:
Phase 0's row `A_arena_off` — the baseline with the arena camera fully
disabled — already shows `kit_pump` (41.0 ms) is the largest per-cycle
bucket *after* physics (50.0 ms) and well ahead of `cameras` (16.1 ms);
subscriber-gating only touches the `cameras` bucket, which is not where the
arena-off budget is going, so the payback looked small relative to the
change's risk (a codepath that silently stops publishing when nothing is
subscribed) and it was left for a future task with its own measurement.

## 2026-08-26 — GPSR recorded sim battery bring-up

`scripts/gpsr-stack up/down/status` automates Stages 1-5 of
`docs/gpsr-sim-runbook.md` (Stage 6, the GPSR orchestrator, still lives in
the `tk25_ws` decision repo and is started separately). Getting a full
hybrid (`--manipulation mock`) bring-up to all-gates-green took nine
attempts across two repos; this entry is the fix narrative behind those
attempts (full blow-by-blow, if ever needed, was kept in this task's
now-deleted scratch notes — the summary below is complete on its own).

**Fix 1 — vision-stage headless crash: `waving_person_server`'s debug
window.** `vision_bringup.launch.py` starts `waving_person_server`
(`tk_vision_specialized`) with no parameter override, so it kept its
`show_window=True` default; on a headless GPU box with no `xcb` display and
no `offscreen` Qt plugin bundled in that venv, the node SIGABRTs at startup
and takes the whole vision stage down with it. `detect_waving.launch.py` (a
different, standalone launch entry point in the same `tk26_vision` package)
already forced `show_window:=False` for exactly this reason — the gap was
`vision_bringup.launch.py` not doing the same. Fixed upstream in
`tk26_vision` (commit `20b1a20`, "show_window off"); this repo's own
mitigation is `scripts/gpsr-stack`'s vision-stage env,
`QT_QPA_PLATFORM=offscreen`, which is a general belt-and-suspenders guard
against any node in that stage defaulting to an on-screen Qt window, not a
substitute for the upstream fix.

**Fix 2 — vision gate never satisfied despite every node self-reporting
ready: Fast DDS interface whitelist.** With the show_window crash fixed, all
6 vision-0 nodes logged successful startup (services, actions, and topics
all created, verified by reading node source directly against
`tools/gpsr_interface_census.py`'s expected names/types — exact matches),
yet the census's own `rclpy` discovery never saw them, across ~36
independent polls over a full 180 s gate timeout. Root cause: the vision
stage's Fast DDS profile's interface whitelist did not include this host's
current wired IP, so default-transport participants (the census subprocess,
the orchestrator) could not discover vision's participants at all — not a
name/type/namespace mismatch, a transport-level one. Fixed by adding the
host's wired IP to that whitelist; verified live (vision services became
discoverable to a fresh default-transport participant immediately).

**Fix 3 — `vision_bringup` needed a rebuild.** Between two attempts, the
`vision_bringup` package the earlier attempts launched against was stale in
`tk25_ws/install` (missing a fix already merged upstream); rebuilding it
into `tk25_ws/install` picked up the current source. Routine but worth
recording: a hybrid bring-up against a stale `tk25_ws/install` can silently
run old vision code.

**Fix 4 — CUDA error 700 (illegal memory access) during camera capture:
DLSS resize race, arena camera parked.** Once the vision gate cleared, sim
startup hit a Warp `wp_cuda_stream_synchronize` CUDA error 700 roughly 15-40 s
in, reproduced 3/3 with the world-fixed arena observer camera enabled
alongside the two hardware-parity cameras (head+wrist), 0/2 with only the
two hardware-parity cameras. Root cause (see
`simulation/tinker_sim_isaac/camera_rig.py`'s `CameraRig.initialize`
docstring for the full mechanism): DLSS's default anti-aliasing op
auto-picks an internal render resolution below the render product's
declared output resolution when that pick falls under DLSS's ~300 px
minimum input size (the 848x480 wrist camera's default-picked internal size
is 424x240, both under 300), then live-resizes up — and with 3+ concurrent
RTX render products alive, that resize raced this rig's Warp device-to-host
copy/synchronize. Fix: pin DLSS to its native-resolution `DLAA` op
(`stable_aa=True`, `AA_OP_DLAA`) whenever the arena camera pushes the
render-product count to 3+, scoped so hardware-parity-only runs (2 render
products) keep their previously-verified default AA path (commit `1ec9ade`).
**This fix was necessary but not sufficient**: error 700 recurred on the
*wrist* camera under the same `sensor-rich` (3-camera) profile even with the
DLAA pin in place, and a follow-up retest with only 2 render products
(head+wrist, arena off) still hit error 700 once — disproving the working
theory that render-product *count* (specifically, "3 is unstable, 2 is
stable") was the load-bearing variable
(`.superpowers/sdd/2026-08-25-gpsr-recorded-sim-battery/task-9-report.md`,
attempt 8). The controller ruling that actually stuck: park the arena
observer camera outright as a known issue (it is sim-only tooling, not
required for GPSR parity) and record head-camera only for the battery
(commit `1e730d4`). Zero error-700 hits across every subsequent attempt.
`TINKER_SIM_ARENA_CAMERA=1` still exists to re-enable it once someone picks
the CUDA-700 investigation back up; `TINKER_SIM_DISABLE_WRIST_CAMERA=1` (the
now-disproven 2-product mitigation) is retained only as a manual operator
escape hatch and is incompatible with `scripts/gpsr-stack`'s census gate
(see that flag's comment in `validation/run_sim.py`).

**Fix 5 — live-manipulation launch needed the model-bundle manifest wired
in.** `mobile_bringup manipulation_planning_task_only.launch.py` takes a
`model_bundle_manifest:=` argument `scripts/gpsr-stack` was not passing at
all; `resolve_model_bundle_manifest()` (`scripts/gpsr-stack:89-140`) now
resolves the canonical bundle produced per
`ros2_ws/src/tinker_sim_bridge/README.md` at
`outputs/ompl-overlay/model-bundle/model-bundle.json` and raises with that
README's exact generation recipe if it hasn't been produced yet — deliberately
not falling back to the robot-artifact `manifest.json` under `artifacts/`,
which does not satisfy `mobile_bringup`'s model-bundle schema and would
otherwise fail later, opaquely, inside the launch itself.

With all five fixes in place: hybrid (`--manipulation mock`) bring-up
reaches all 4 gates green with zero error-700 hits, and step 5's tier2
smoke corpus ran 2/2 PASS. Live-manipulation (`--manipulation live`)
bring-up reaches the manipulation stage (previously never reached) and is
blocked only on environment-local pieces out of this repo's scope (a
missing `anygrasp` checkpoint, `tk25_ws`-side Python dependencies for the
orchestrator) — not on anything `scripts/gpsr-stack` itself does.

## 2026-08-28 — Arena reskin: wood floor + tinted furniture (rcw2026)

The rendered arena read as stark-white: Isaac's default gray ground grid
under everything, and GLB furniture that renders bland/white because the
converted models' own material bindings do not resolve at render time (the
artifact ships `textures/<id>/*.png`, but they don't bind through). Fix is
in `tools/tinker_sim_deploy/arena_convert.py`, all at compose time — no GLB
re-conversion, no image textures (solid PBR keeps render cost ≈ current):

- **Floor**: `compose_arena` now authors `/World/Arena/Floor`, a thin
  (`2 cm`) visual cube covering the wall-footprint AABB (+10 cm margin,
  `floor_slab`), oak-tinted (`FLOOR_COLOR`). No collider — physics still
  rides the backend's `GroundPlaneCfg` plane at z=0; the slab's top face
  sits `2 mm` above it so it wins the depth test against the default grid.
- **Furniture**: a solid PBR from `FURNITURE_COLORS` (keyed by model_id,
  warm-neutral fallback) bound on each furniture wrapper at
  `strongerThanDescendants`, overriding the GLB subtree's bindings. Woods
  brown, fabrics muted, appliances off-white steel (intentional, not the
  untextured default), plants green, TV black.
- **Walls**: unchanged (gray `0.6`). `_bind_gray_material` now delegates to
  the shared `_bind_pbr_material`.

Pure geometry/color logic (`floor_slab`, `furniture_material`) is unit-tested
in `tests/test_arena_convert.py` under plain Python; the pxr authoring is
live-only as usual.

**Operator step — regenerate + re-pin (Isaac box, needs Kit + pinned SOBITS
checkout; not runnable in dev/CI):**

1. `python tools/arena_import.py --config config/arena-import.json`
   (add `--checkout <existing pinned checkout>` to skip the clone). This
   publishes a new content-addressed arena artifact and re-points
   `artifacts/arena/rcw2026/current.json`. Then register the new
   `arena.usd` path + sha256 under `generated_arena_usds` in
   `artifacts/asset-manifest.json` (the tool prints this reminder).
2. Eyeball the render: wood floor present, furniture tinted, walls unchanged.
3. Run the vision/detection smoke to confirm detection parity and that
   per-frame render time stayed close to the pre-change baseline (solid PBR
   should not move it). If a furniture tint still reads white, the override
   binding didn't win — check the `strongerThanDescendants` strength.

Design: `docs/superpowers/specs/2026-08-28-arena-wood-floor-furniture-tint-design.md`.

## 2026-08-22 — GPSR `goto_command_point` stall: two root causes in the sim, one residual

Starting point: `reports/gpsr-sim-2026-08-20/NAV-HANDOFF.md` — Nav2 never
left the first goal, `controller_server` aborted with `Failed to make
progress` every ~13 s (57×), recoveries ran 8×, `/amcl_pose` wandered in and
out of the 0.1 m tolerance. The handoff suspected `min_theta_velocity_threshold`
and the progress checker. Neither was the cause (`min_theta_velocity_threshold`
filters *odometry*, not commands). Everything below was measured on the same
stack (`sensor-rich`, `gpsr-rcw2026`, `--arena rcw2026 --spawn-xy=-2,-2`,
`navigation.launch.py`, domain 71) with `/sim/internal/physics_truth` as ground
truth; probe scripts lived in the job's tmp dir, numbers are in the text.

**Reproduced first.** Nav-only stack, goal = the scenario spawn (−2, −2, yaw 0):
same 13 s abort cadence, same recoveries. Truth vs estimate showed the
*physical* robot turning at −0.1…−0.2 rad/s for a −0.6 command while wheel
odometry reported +0.2…+1.3 rad/s, the estimate "moving" 0.5 m in 6 s, and
the robot physically 1.2 m off the goal while AMCL (cov 0.012) put it 0.1 m
away. Open-loop `/cmd_vel` without Nav2 isolated it: forward 0.2 m/s was fine
(truth 0.17, fronts 3.1 vs 3.8 rad/s target); **rotate ±0.5 rad/s gave truth
0.11 / −0.09 rad/s**, front wheels stalled and chattering (−0.24±0.66 vs
−1.19 target), odom yaw rate garbage.

**Root cause 1 — the rear casters were driven and held (fixed).** The URDF's
rear wheels (r = 0.03 m) sit on free swivel joints; `base_facade` commands
all four wheel joints with the front wheels' angular velocity and the
backend's `wheels` actuator group matched `rear_.*`, so (a) the caster wheels
got a damping-200 velocity drive at a target wrong by the radius ratio —
forward they were braked (the front drive saturated, 3.1 vs 3.8 rad/s), in a
turn they were skids — and (b) the caster *swivels* were caught by the same
group (damping 200 toward zero). Narrowing the group to the wheels only made
it worse: the URDF importer bakes a stiffness-625, unlimited-force position
drive (target 0) onto every continuous joint, so an unconfigured swivel is
rigidly held straight (swivel position stayed within ±0.0006 rad through an
8 s turn). Fix: drive only `front_.*_wheel_joint`; `rear_.*_swivel_joint`
and `rear_.*_wheel_joint` form an explicit zero-gain `casters` group
(`CASTER_JOINT_PATTERNS`). Result: forward 0.20 m/s at 3.76/3.80 rad/s (no
saturation), casters free-roll at 6.6 rad/s (= 0.2/0.03), **rotate ±0.5 →
0.28 / −0.27 rad/s and odom yaw rate now tracks truth** (0.29 / −0.33).
`tests/test_wheel_actuator_patterns.py`.

**Root cause 2 — AMCL was given the wrong map (fixed).** The `sensor-rich`
lidar is not a rendered sensor: `ros_gateway.py` raycasts 181 rays over ±90°
against the simulator's occupancy grid from the truth pose, i.e. against
`artifacts/arena/rcw2026/<current>/map.yaml`. `navigation.launch.py` and
`gpsr.launch.py` default `map_yaml` to the *robot artifact's* colocated
`map.yaml`, which the manifest traces to `0701_robocup_arena3` — the hardware
arena. The two maps share **zero** occupied cells in world coordinates (even
under a ±2 m shift search). The 2026-08-18 AMCL study passed the arena map
explicitly, which is why it found AMCL healthy. With the arena map and
passive casters, AMCL tracks truth: 0.03 m at rest, 0.13–0.26 m through
in-place rotation, 0.09 m after 1.5 m of driving, yaw within 0.10 rad. Fix:
`gpsr.launch.py` resolves the map from the scenario's `world.arena`;
`navigation.launch.py` takes `arena:=`; explicit `map_yaml:=` still wins
(`runtime.resolve_arena_map_yaml`, `scenario_arena_id`;
`tests/test_arena_map_resolution.py`, `test_navigation_launch_map.py`).

**Root cause 3 — the wheel colliders' line contact locked the turn (fixed).**
With the first two fixes the robot physically arrived within 0.16–0.21 m of
the goal and the estimate within 0.1 m, but the final yaw trim never
completed: DWB commanded ~0.09 rad/s and the base did not move, so
`SimpleProgressChecker` (XY only, 0.5 m in 10 s) aborted, the spin recovery
added ±1.57 rad, and it repeated. Measured truth yaw rate vs command:
**0.1 → 0.00, 0.2 → 0.07, 0.3 → 0.13, 0.5 → 0.27, 0.8 → 0.50**, wheel joints
reading exactly 0.0 under small commands with the drive at its cap; in a
turn the wheels lagged their targets by a near-constant 0.2–0.5 rad/s, a
dry-friction-like resistance of 40–80 N·m per wheel. Ruled out one variable
at a time: drive effort cap (10 vs 80 N·m: identical), articulation velocity
solver iterations (8 vs 1: identical), PGS instead of TGS (chatter gone,
0.5 → 0.36, deadband unchanged), articulation `sleep_threshold=0` (wheels
then read −0.03 rad/s — awake, still stuck), and the PhysX-side drive gains
(read back through the tensor view: stiffness 0, damping 200, max force 80,
force-type — the drive is what the config says). The casters do align
(swivels at −120°/−68°, the tangential trailing angles). What did move the
needle was the wheel *collider*: the importer's `Cylinder` prims are exact
custom-geometry cylinders (`/physics/collisionApproximateCylinders` is
already false; forcing convex hulls made it worse, 0.3 → 0.002), and a
cylinder's line contact across the 63 mm tread cannot roll on a 0.125 m
turn radius — the inner and outer edges need different speeds, so the patch
locks. Replacing the cylinder with a **sphere of the same radius** on the
drive wheels gave 0.2 → 0.20, 0.3 → 0.29, 0.5 → 0.49 (0.1 still dead: the
caster wheels had the same patch); on all four wheels, on the stock TGS
solver with default sleep settings, **0.1 → 0.099, 0.2 → 0.197, 0.3 →
0.298, 0.5 → 0.498, 0.8 → 0.795** with steady wheel speeds, forward driving
unchanged (0.20 m/s at 3.77 rad/s), `/clock` still ~48 Hz. Fix:
`_apply_wheel_sphere_colliders` (runtime override at spawn like the chassis
ballast — artifact untouched; deactivates `<wheel>/collisions/mesh_0`,
authors `<wheel>/collisions/sphere` with the cylinder's own radius, fails
closed on a missing wheel or collider); `TINKER_SIM_WHEEL_COLLIDER=cylinder`
restores the authored collider for A/B; `tests/test_wheel_colliders.py`.

**End to end.** Nav-only stack (`navigation.launch.py map_yaml:=<rcw2026
arena map>`), AMCL seeded at truth, robot 1.0 m from the goal and facing
away: `Reached the goal!` / `Goal succeeded` in 13 s with the stock
tk26_navigation parameters, truth 0.11 m / 0.12 rad from (−2, −2, 0),
estimate within 0.07 m of truth throughout, zero `Failed to make progress`.

**Nav2-side observations for the navigation owners** (tk26_navigation, not
this repo, no change needed now): `tracking_goal_checker` (`yaw_goal_tolerance:
3.14`) exists in `nav2_dwb_params.yaml` but is commented out of
`goal_checker_plugins`; the XY-only `SimpleProgressChecker` converts any
station-keeping yaw trim slower than 10 s into a recovery, which is what
amplified the base defect above into a 13-minute stall.

**Also learned.** A stack launched with `&` from a non-interactive shell
inherits SIGINT=ignored, so `ros2 launch` never sees a later SIGINT — reset
the disposition in a wrapper before exec. Tear the Nav2 launch down *before*
the simulator: its nodes run on sim time and hang in shutdown on a frozen
clock. The robot artifact's `map.yaml` is the hardware map by design; the
`(−2, −2)` scenario spawn cell is "unknown", not "free", in both maps.

## 2026-08-21 — Bridge attached while driving and while the arm moves

Exercise: `/cmd_vel` 0.15 m/s + 0.25 rad/s at 15 Hz for 45 s (Nav2's
controller rate, no localisation needed), then `JointTrajectory` goals to
`xarm7_traj_controller` alternating two poses every 5 s for 45 s; motion
verified from `/isaac_joint_states`. Bench scripts in the main checkout's gitignored
`outputs/bench/` (`run_with_bridge_exercise.sh`, `exercise.py`,
`record2.py`, `show_phases_wall.py`).

| bridge attached, control 60, cameras 12 Hz | old intake | main-thread intake + 60 Hz cap |
|---|---|---|
| idle | 0.71 | **0.81** |
| base driving | 0.59 | **0.68** |
| arm trajectories | 0.50 | **0.63** |

What the remaining cost is: driving pushes no new targets (the wheel
velocity targets are constant) — it is PhysX contact/rolling work, +15 ms
per camera cycle; a 45 s circle in the rcw2026 arena ends with the robot
against props, and that contact cost persists afterwards (physics 54 vs 41
ms/cycle), which is the scenario, not the bridge. Arm trajectories push
targets on ~70% of control steps (JTC rewrites the command each cycle) and
the moving drives cost PhysX ~3 ms/step more.

Three changes came out of this:

- **Main-thread intake.** The gateway's private executor thread is gone.
  `spin_once()` now takes messages straight from the two DDS readers
  (`_take_pending`, rclpy `handle.take_message`, up to 512 per reader per
  step). The thread could only take one message per wait-set pass and each
  pass had to win the GIL back from the simulation loop; removing it took
  `publish` from 24 back to 15 ms/cycle and idle RTF from 0.71 to 0.81.
  `TINKER_SIM_GATEWAY_EXECUTOR=1` restores the thread for comparison.
- **60 Hz cap on changed snapshots** in `command_gateway`
  (`MIN_PUBLISH_PERIOD_S`): ros2_control rewrites the arm command every
  150 Hz cycle during a trajectory; the simulator applies targets 60 times
  a second, so anything faster was pure overhead. Safety-stop snapshots and
  epoch bumps are never delayed.
- **Profile lines carry `wall_time` and `sim_time`.** Attribute phases by
  wall time. The simulator re-zeroes `/clock` when the bridge's
  `ResetSimulation` (STOP -> PLAY) lands, by design (Isaac Lab recreates the
  articulation view on PHYSICS_READY and that boundary is the new zero), so
  `/clock` and a step counter started at process launch differ by the
  attach time (~34 s here). Six runs of this investigation chased a
  "stale trajectory replayed 30 s late" that was entirely this offset: the
  snapshot ids received by the simulator matched the bridge's live
  publish counter once compared on wall time, and every message's DDS
  source timestamp was < 70 ms old. Ruled out on the way, each by
  measurement: DDS shared-memory transport (UDP-only identical), reliable
  repair (BEST_EFFORT identical), Kit worker-thread starvation (16 threads
  identical), a second publisher (one `command_gateway` process, one
  writer), a gateway replay path (none exists).

Side finding: `TINKER_SIM_CPU_THREADS=16` (Kit's default is 32 on this
32-thread host) gave idle 0.81 -> 0.82-0.84 and arm 0.63 -> 0.66 with
physics 44 -> 40 ms/cycle; the TBB workers otherwise spin at ~30% each.
Worth recommending for live-stack runs; not made the default.

## 2026-08-21 — ROS bridge attached: RTF 0.77 -> 0.23 -> 0.71


Measured on 2026-08-21 with `sensor-rich`, `TINKER_SIM_CONTROL_HZ=60`,
`TINKER_SIM_CAMERA_HZ=12`, the Stage 2 bridge attached ~50 s in and the
stack idle (no goals): RTF 0.77 standalone -> **0.23** with the bridge up ->
0.75 again once it was killed. The loss was entirely inside the simulator's
Python loop, not the host: scheduler wait stayed at 0.1%, GPU 0 idle, and
the sim process's own CPU share *fell* from 80% to 47% while its voluntary
context switches quadrupled. The profile attributed it:

| per camera cycle (5 control steps) | alone | bridge attached |
|---|---|---|
| `spin` (`gateway.spin_once`, inbound commands) | 0.2 ms | 100 ms |
| `publish` | 12 ms | 92 ms |
| physics / Kit pump / cameras | 47 / 31 / 14 ms | 78 / 52 / 34 ms |

Root cause: `command_gateway` re-sent **every** mux packet (base,
ros2_control, gripper, pan_tilt) as a fresh full snapshot on its 150 Hz
tick even when nothing changed, ~300-600 `JointState`/s into the simulator.
Each packet cost the simulator ~0.9 ms in `command_joints` (torch element
writes that each release the GIL to the busy executor thread), and the
executor thread's deserialisation work taxed every other GIL release in the
loop -- `imu` publish, which has no subscriber at all, went from 0.6 to
5.5 ms per call. Things that were **not** the cause (each measured): CPU
oversubscription, GPU sharing with vision (the bridge launches no vision
and tinker2-net's GPSR set has two BEST_EFFORT camera subscribers), Fast DDS
synchronous publish mode (`RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS`
changed nothing), and the gripper effort-limit write (never exercised).

Fixes, all result-neutral for the simulator's targets:

- **Bridge keepalive.** `command_gateway` still evaluates deadlines and
  composes the snapshot at 150 Hz but publishes it only when it differs
  from the last one sent or every 50 ms (`CommandGateway.KEEPALIVE_PERIOD_S`,
  10x inside the simulator's 0.5 s command-stream watchdog). Snapshots stay
  complete (the simulator zeroes velocity targets per snapshot, so partial
  snapshots are not an option). Idle inbound packets: 2141 -> 88 per
  100-step window. **Bridge-attached RTF 0.23 -> 0.71** (0.77 standalone in
  the same run). While the stack is driving, packets rise with the base
  facade's 50 Hz command rate; expect a partial regression that was not yet
  measured.
- **Batched target apply.** `backend._apply_joint_command` gathers a packet
  in Python and writes each target tensor once instead of per element
  (fewer GIL release points per packet; identical resulting targets).
- **Gripper effort-limit dedup.** An unchanged ceiling no longer reaches
  PhysX (harmless, but it was a plausible suspect and is now cheap to rule
  out via the `gripper_limit_writes` counter).
- **Profile.** `TINKER_SIM_PROFILE=1` now also reports `spin`, `unaccounted`
  and `wall` per cycle and a `spin_breakdown` (events, commands,
  `command_joints_ms`, `gripper_limit_writes`) -- if `spin` climbs again,
  count commands first.
- **Opt-in knobs** (default behaviour unchanged):
  `TINKER_SIM_GIL_SWITCH_INTERVAL_MS` (0.5 recovered 0.23 -> 0.29 with the
  *old* bridge; residual value with the keepalive is small) and
  `TINKER_SIM_CPU_THREADS` (caps Kit's worker pool, default min(cores, 32);
  not needed on this host -- scheduler wait was never the problem).

Why not a C++ bridge: the cost was packet *count* times simulator-side
Python per packet; a C++ sender of the same stream reproduces RTF 0.23
exactly. C++ would only matter on the simulator side (rclpy's GIL), which
is a large port of the safety-critical session/epoch state machine and is
not justified by the residual 0.06.

Follow-up runs on the same day (all bridge attached, idle, control 60,
cameras 12 Hz): keepalive alone 0.71; keepalive + `TINKER_SIM_GIL_SWITCH_INTERVAL_MS=0.5`
0.70 (no further gain, so the knob is not recommended); old bridge +
switch interval 0.5 alone 0.29. Residual gap to standalone (0.77-0.79) is
`publish` 12 -> 24 ms per cycle and `spin` 2 ms: the ~88 packets and ~64
safety heartbeats per 100-step window still cost a GIL hand-off each.
Unmeasured: the stack actively driving (base facade then emits changing
50 Hz commands, so packet count and the cost rise again).

## 2026-08-21 — Per-step costs removed (result-neutral)


- **Safety hold.** While stopped, the backend used to disable the arm's
  PhysX drive and push a gravity-compensated PD effort from Python every
  control step; at the control rate that PD limit-cycled (joint1 pinned at
  -100 Nm, arm never at rest) and cost ~8 ms per step. The hold is now
  PhysX's own drive at the latched pose (stiffness 600, damping 80, 100 Nm
  ceiling) with only the gravity term fed forward and refreshed every 30
  control steps. Stopped step 13.3 -> 4.5 ms; the arm sits within 0.005 rad
  at ~zero velocity, and the published joint efforts during a hold are now
  computed with the hold gains (they used to report the nominal 20 000
  stiffness).
- **Target pushes.** Isaac Lab applies actuator groups with two Warp launches
  per group; tinker2 has five, so every target push (arm trajectories, the
  wheel slew ramp, hold refreshes) paid ten launches, ~3.2 ms of a ~3.6 ms
  push. The backend now binds a fused `_apply_actuator_model` that launches
  each kernel once over all groups -- bit-identical staging and telemetry
  buffers, push 4.3-6.4 -> 1.8 ms. `TINKER_SIM_STOCK_ACTUATOR_MODEL=1`
  restores Isaac Lab's loop.
- **Camera conversions.** Frames are copied into pinned host buffers with one
  stream sync per cycle, RGB conversion writes into reused scratch buffers,
  and depth metres->16UC1 mm is a Warp kernel on the GPU (`wp.rint`, i.e.
  banker's rounding like `np.rint`); all byte-identical to the reference
  implementations kept in `camera_rig.py`, proven by
  `tests/test_camera_publish_equivalence.py` on synthetic edge cases and 120
  real frames. Camera stage 23 -> 13 ms per cycle. Note: with the optional
  `--camera-pointcloud` flag (off in this runbook) the cloud is now built
  from the millimetre depth, i.e. quantised to 1 mm.
  With `TINKER_SIM_PROFILE=1` the profile line now also carries a
  `camera_breakdown_ms` (capture / rgb_convert / depth_convert / image_fill /
  image_publish / info).

That breakdown is how the development-lidar ray-cast was found to cost
~35 ms per lidar frame (~350 ms per simulated second); it is now vectorised
and bit-identical (`OccupancyMap.raycast_many`), at ~2–5 ms per frame;
that alone took the default sensor-rich run from RTF 0.35 to 0.40.

## 2026-08-21 — Physics/control cadence, Kit pump and solver probes

Text moved from the runbook when it was reduced to current operation only.

### Camera cadence under a live stack

Export `TINKER_SIM_CAMERA_HZ=15` before Stage 1 for any run with the full
GPSR stack attached.

The simulator holds RTF ~0.30 on its own, but drops to ~0.06 once real
subscribers attach: Kit is pumped (both RTX cameras rendered) once per camera
stride, and the image payloads are serialised on the same beat, all inside the
step loop. At that speed Nav2 cannot service its lifecycle bonds and tears its
own stack down — `Switch controller timed out after 2.000000 seconds`, then
every navigation node deactivates, leaving no `map -> odom` and an unusable TF
tree. Halving the cadence halves both costs.

`simulation/sensors/hardware-parity.json` stays authoritative and unedited:
unset, its 30 Hz is used. The override may only *lower* the rate — publishing
faster than the real camera would be a parity violation, and is refused.

### Control cadence under a live stack (`TINKER_SIM_CONTROL_HZ`)

Export `TINKER_SIM_CONTROL_HZ=60` (with `TINKER_SIM_CAMERA_HZ=12`; 15 if the
vision stack needs it) before Stage 1 for live-stack runs. Do **not** use
`TINKER_SIM_PHYSICS_HZ=60` for that purpose any more. 12 Hz is exact at
control 120 and 60 (strides 10 and 5); at control 30 it rounds to 15 Hz.
Measured 2026-08-21 at control 60: cameras 15 Hz RTF 0.70, 12 Hz 0.77, and
0.80 once depth conversion moved to the GPU. Simulator VRAM on that run:
peak 2.6 GB, steady 2.3 GB (the 2026-08-20 code ran 2.7-3.6 GB).

The simulator has two rates. The *physics* rate (`TINKER_SIM_PHYSICS_HZ`,
default 120, may only be lowered) is PhysX's solver step — the thing every
contact/grasp result was validated at. The *control* rate
(`TINKER_SIM_CONTROL_HZ`, default = physics rate) is how often Isaac Lab,
the joint-target writes, the wheel slew, the gateway publish and `/clock`
run. With the control rate lowered, each control step runs
`physics_hz / control_hz` explicit solver substeps of the validated
1/120 s, so the solver trajectory is unchanged while every per-step wrapper
cost is paid 60 times a second instead of 120. The control rate must divide
the physics rate evenly and has a 30 Hz floor. Note omni.physx's
`IPhysxSimulation.simulate(elapsed)` does *not* substep on its own — it
integrates exactly `elapsed` — which is why the substeps are explicit and
why simply lowering `dt` was a fidelity change, not an optimisation.

Measured 2026-08-21 (gpsr-rcw2026, rcw2026 arena, GPU 1, RTF =
simulated / wall):

| run | physics-only (no ROS, no cameras) | sensor-rich + ROS, cameras 15 Hz, start of day | same, end of day (all fixes below) |
|---|---|---|---|
| default 120 / control 120 | 0.75 | 0.35 | **0.64** |
| `TINKER_SIM_CONTROL_HZ=60` | 1.18 | 0.44 | **0.70** |
| `TINKER_SIM_CONTROL_HZ=30` | 1.88 | 0.50 | **0.78** |
| `TINKER_SIM_PHYSICS_HZ=60` (old advice) | 1.61 | 0.51 | — (not needed any more) |

The sensor-rich numbers are with the robot safety-stopped (no bridge
attached), which is the state a run spends its start-up and every
command-loss interval in; the "end of day" column includes the lidar,
safety-hold, actuator-launch and camera fixes described further down.

Robot root position after 10 s idle agreed with the default to within
0.5 mm at control 60/30 and drifted 5 mm at physics 60 — the substepped runs
keep the validated solver trajectory, the lowered physics rate does not.
Lower control rates also lower the IMU (200 Hz parity) and base-state
(50 Hz) publish cadences, which are derived from the control step: at 60 Hz
they publish at 60 Hz, at 30 Hz they publish at 30 Hz.

What still bounds RTF under the live stack, per simulated second at control
60: the PhysX solve itself (~9 ms per control step, 32 authored position
iterations) and the Kit render pump for both RTX cameras (~30 ms per camera
frame). The pump was probed on 2026-08-21 and is *not* render-mode, GI,
async-rendering or readback bound: `RaytracedLighting` instead of the
default `RealTimePathTracing`, `/app/asyncRendering=true`, and
reflections/indirect-diffuse/AO off each changed it by <1 ms; an empty Kit
update is ~2 ms, and the cost scales with camera pixel count (~19 ms head,
~7 ms wrist). It is Kit's per-render-product frame pipeline at the parity
resolutions, and the only remaining levers are structural. Two further
opt-in knobs exist: `TINKER_SIM_SOLVER_POSITION_ITERATIONS` /
`TINKER_SIM_SOLVER_VELOCITY_ITERATIONS` override the robot USD's articulation
solver iteration counts (32 / 1); 8 position iterations measured RTF 0.52
at control 60 but *changes drive and contact convergence*, so it is not
recommended for anything that produces evidence. PhysX worker-thread count
(`--/persistent/physics/numThreads`, default 8) made no measurable
difference at 4 or 16.

With `TINKER_SIM_PROFILE=1`, every profile line now also carries
`physics_breakdown_ms.physx_substeps` and a `publish_breakdown_ms`
(clock / joint_state / imu / cloud / status / truth per `publish()` call).

## 2026-08-29: gpsr-spawn-spike — live scene-spawn viability

Ran `scripts/gpsr-spawn-spike` against the live stack (Nav2 + sim
PLAYING + bridge, `gpsr-stack up --scenario gpsr-rcw2026-bench --sim-gpu 1`)
per the command-driven-scene design
(docs/superpowers/specs/2026-08-28-command-driven-scene-and-sim-identity-design.md,
section 2.4): `tools/gpsr_spawn.py plan`+`apply` for "count the
pudding_box on the kitchen_table", checked `/clock` monotonicity, Nav2
health, and entity presence, then `clear`.

Results:
- plan: PASS
- apply (spawn_entity while PLAYING): PASS — `/get_entities` lists
  `/World/Scenario/cmd_pudding_box_0` afterwards; `GetEntityState` reads
  (2.5, -3.0, 0.734).
- arena-camera frame shows the pudding_box: INCONCLUSIVE — a 10 cm YCB
  object is ~3 px at the arena camera's 960 px; even the base scenario's
  soup/mug are not discernible. Entity-list check used instead.
- /clock monotonic across apply (20 samples before, 90 after; 65.43 -> 66.92 s): PASS
- Nav2 goto bedroom and back after the spawn: goals accepted, robot drove
  and ended at (-1.87, -1.78) ~ command point. Each leg hit the 170 s
  wall cap before reporting SUCCEEDED because RTF is ~0.2 with the arena
  camera on (sim clock 189 s after ~15 min wall) — a cadence limit, not a
  Nav2 fault.
- clear (delete_entity): PASS — entity absent from `/get_entities` afterwards.

Three things the spike caught that the unit tests could not:
1. `python3 tools/gpsr_spawn.py` failed with `ModuleNotFoundError: tools`
   when run as a script (fixed: repo root inserted on sys.path).
2. Presence-based (asset, spot) dedupe skipped instances 1-2 of a count
   command, and slot 0 coincides with the base scenario's soup pose at
   kitchen_table (fixed: count-aware dedupe + grid slots offset by the
   objects already at the spot).
3. `ros2 topic echo` via the ROS daemon intermittently dies with
   `!rclpy.ok()` — the spike uses `--no-daemon`.

**Decision:** outcome A — tier-2 uses runtime spawning per run:
`--spawn-cmd "scripts/gpsr-scene-apply --command {command} --seed {seed}
--plan {plan} --manifest {manifest}"` (plan+apply in one command) and
`--clear-cmd "python3 tools/gpsr_spawn.py clear --manifest {manifest}"`.
The bench environment must carry the vendored `simulation_interfaces`
Python overlay (`.ros-vendor/humble/opt/ros/humble/{local/lib,lib}/python3.10`)
on PYTHONPATH — the battery script already does.

Next: re-run `s2026-003`, `004`, `005` (old run dirs archived as
`*.attempt1-no-scene`). Acceptance: 003/004 no longer fail on absent
objects; 005 passes the `person_found` gate under
`GPSR_SIM_IDENTITY_RELAXED=1`.

## 2026-08-19 — Vision live-acceptance round-trip: recorded result + real tk26_vision defect

Ran the three-terminal live acceptance check (sim on `--sensor-profile
sensor-rich` with `--arena-colors`, the Humble vision overlay's `get_image`,
and `tests/ros_humble/test_vision_get_image_live.py` under
`TINKER_SIM_VISION_LIVE=1`) end to end on `ROS_DOMAIN_ID=42`. See "Live
acceptance runbook" under Vision hardware-parity cameras in the README for
the exact commands.

This was run and recorded (3/3 passed, `ROS_DOMAIN_ID=42`): direct RELIABLE
subscriptions decode both cameras with `tk26_vision` conventions, and the
wrist frame carried all six palette hues (45.3% chromatic pixels). Evidence
is under `reports/vision-roundtrip/` (gitignored, host-local).

**Real defect found in `~/tk25_ws/src/tk26_vision`** (documented here for the
record; out of scope to patch from this repo): `vision_util`'s `get_image`
and `get_point_cloud` register `async def` callbacks directly on
`message_filters`' `ApproximateTimeSynchronizer` (`get_image.py:49,65,77,81`,
`get_point_cloud.py:43,66,100,104`). Humble's `message_filters` invokes
callbacks synchronously, so those coroutines are never awaited, the node's
cached frames never update, and both services always answer "No camera
data" — on real hardware too, not only in sim. The sim run still proved that
delivery and stamp-pairing both work: the node's own "coroutine was never
awaited" `RuntimeWarning`s fired for both cameras. The acceptance test's
third case probes this service directly and will fail loudly — prompting a
test upgrade — once the node is fixed upstream.

Status: development-validated with a recorded live round-trip; **not
release-qualified**.

## 2026-08-17 through 2026-08-20 — RoboCup 2026 arena import: validation evidence and open findings

Validation performed on this branch (development-validated only):

- unit suites are green, with a stable failing/erroring name-set matched
  against this repo's pre-existing environmental failures (see "Developer
  verification" in the README);
- both artifacts hash-verify clean (`verify_asset_artifact(...) == []`) and
  re-import is a proven byte-identical no-op;
- visual/collision AABB agreement was spot-verified: within 1.4mm on three
  YCB objects (cracker box, mug, bowl); arena furniture bounds were checked
  against the upstream SDF within the configured 0.02m tolerance, with two
  documented per-model exceptions (`rcw26_door`, `rcw26_sink`) whose upstream
  SDFs deliberately under-size the collision box (a trimmed door-panel depth
  and a floor-anchored sink height, both for gripper-reach affordance, not
  data errors);
- a headless streaming smoke (`validation/run_sim.py --sensor-profile
  navigation-parity --profile parity --scenario empty --seed 7 --headless
  --livestream --arena rcw2026 --duration 45`) ran the full 45 simulated
  seconds to a clean exit, with only headless-windowing/driver diagnostic
  warnings in the log and no importer-scratch-path leakage;
- a live end-to-end `./scripts/launch-arena-streaming --arena rcw2026` run
  (2026-08-18) reached streaming readiness — ready file written, TCP 49100
  listening, `viewport_ready: true`, arena `robocup-arena3` with 38
  colliders loaded — and stayed up awaiting a client;
- sensor-rich camera imagery of the arena's furniture: head and wrist
  hardware-parity color/depth frames captured live against `--arena
  rcw2026` (shelf close-ups plus a base-rotation panorama showing the TV
  cabinet, trash bin, door, tiled floor, and plant), with per-frame content
  statistics — `reports/arena-sensor-rich-2026-08-18/`; the wrist camera's
  mount/intrinsics and arm-following viewpoint were verified separately in
  `reports/arena-arm-camera-2026-08-18/`;
- AMCL convergence on the derived map: with the spawn moved to a free cell
  (`--spawn-xy=-2.0,-2.0`) and the Humble stack pointed at the arena map
  (`map_yaml:=...`), AMCL locked to physics-truth within 0.07 m after
  seeding and, over truth-validated gentle-motion runs, contracted to
  position variance 0.035/0.050 m² (std ~0.2 m) at 0.14 rad yaw error —
  `reports/arena-amcl-2026-08-18/SUMMARY.md`, which also records two real
  findings: the default (0, 0) arena spawn sits inside `shelf_02`'s
  footprint (hence the new `--spawn-xy` override), and sustained in-place
  skid-steer rotation accumulates wheel-odometry yaw slip that drags the
  filter (a base-odometry characteristic, not a map defect).

- head pan/tilt effort override: with the backend's `effort_limit_sim`
  override extended to the `head` actuator group, commanded pan/tilt
  converged from spawn to within the 0.05 rad tolerance of a (1.0, -0.3)
  rad target — final (0.9506, -0.2853) rad at sim t=0.317 s — closing the
  2026-08-18 finding that pan/tilt drives inherited the URDF's 1.0 Nm
  effort cap. Live-proven:
  `reports/arena-fixes-2026-08-19/head-tracking.json`;
- physics interaction (an object resting on arena furniture): the
  pick-deliver-place `delivery_object` (0.08 m cube), spawned via the
  standard `spawn_entity` path at its declared z=0.8 pose, fell onto and
  came to rest statically (zero twist across 387 truth samples) on a 0.5 m
  board of `rcw26_shelf` —
  `reports/arena-scenario-spawn-2026-08-19/object-spawn-verification.md`.
  **Superseded 2026-08-20**: this evidence came from `/get_entity_state`
  polling, not physics truth; a later run under a profile that reports
  physics truth found no trace of the object in it at all. See "Scenario-
  spawned objects may not be physics-simulated" under Known arena
  limitations below — that finding is SUSPECTED, not confirmed, so treat
  this bullet as open again rather than either closed or refuted;
- scenario entity spawning in the arena: `scenario_runner` executed
  find-and-approach-person and pick-deliver-place against an `--arena
  rcw2026` sim with every operation accepted; the person capsule and task
  cube spawn at their exact declared world poses (verified via
  `/get_entities`/`/get_entity_state` and `expected_objects` truth
  correlation), and spawned entities are live rigid bodies
  (`/set_entity_state` round-trips) —
  `reports/arena-scenario-spawn-2026-08-19/`. Caveats: nothing implements
  scenario `events` (the person's `actor_path_start` walk never runs), and
  scenario poses were authored for the procedural world (the arena has no
  pedestal at the object spawn; the person's declared pose sits in
  furniture-dense space).

Not yet validated (open):

- textured-frame visual confirmation by a human viewer (the streaming
  session above is up for exactly this; connect with NVIDIA's client).

Known arena limitations (development findings, 2026-08-18 through 2026-08-20):

- the default robot spawn (0, 0) lies inside `shelf_02`'s physical and
  rasterized footprint. The launch now fails closed on this instead of
  spawning into it: `validate_arena_spawn()` exits non-zero and names the
  nearest free cell in the error (`arena spawn (0.0, 0.0) lacks 0.35 m
  clearance on the derived map; try --spawn-xy=-0.4,0.4`) — pass the
  suggested `--spawn-xy` for navigation work (see launch docs above).
  Live-proven: `reports/arena-fixes-2026-08-19/spawn-fail-closed.log`
  (exit 1, suggestion printed);
- under `sensor-rich`, an occupied dev-lidar sensor origin (for example the
  spawn-in-`shelf_02` case above) used to publish a dense ring at ~0.3 m
  for every ray. The 2026-08-18 note here misdescribed this as an RTX-lidar
  self-hit; there is no RTX lidar in this path, and the ring was the
  occupancy raycast's minimum-range floor being returned when every ray
  starts inside an occupied cell. An occupied ray origin now publishes an
  empty cloud instead. Unit-proven only, no live run has targeted this
  path: `tests/test_ros_gateway.py`
  (`RosDevelopmentLidarTest.test_development_lidar_empty_when_origin_occupied`);
- wheel-velocity commands are slew-limited to 60 rad/s² (≈3.1 m/s² linear
  at the 0.0525 m wheel radius), set deliberately ABOVE Nav2's `acc_lim`
  (~2.5 m/s²) so planner-shaped velocity profiles pass through unchanged —
  the bound exists to floor non-planner commanders and stale-target
  transients, not to shape Nav2 output. Live-proven: after a 30 s in-place
  rotation, an idle base commanded to coast drifted only 8.0e-05 m in XY
  over the following 30 s, far inside the 0.1 m bound —
  `reports/arena-fixes-2026-08-19/coast.json`;
- sustained in-place skid-steer rotation accumulates wheel-odometry yaw
  slip (a base-odometry characteristic, not a map defect — see the AMCL
  validation note above). This is a dead end for IMU fusion: the sim IMU
  publishes only world-frame angular velocity, marks
  `orientation_covariance[0] = -1.0` (REP-145 "orientation not provided"),
  and never populates linear acceleration, so fusing it into an EKF would
  only duplicate odom's own vyaw rather than correct it.

Found on 2026-08-20, while the live evidence wave was closing out the
fixes above:

1. **Scenario-spawned objects may not be physics-simulated.** Isaac logs
   `Physics tensor entity not valid for rigid body /World/Scenario/<id>`
   and the object was observed holding its exact spawn pose with
   fabricated zero velocities. This is SUSPECTED, not confirmed: it was
   seen through a run whose profile could not report objects at all
   (fixed since, commit `02d1785`), so it may prove to be an artifact of
   that. This supersedes the 2026-08-19 claim above that the "object
   rests on furniture" item was closed — that evidence came from
   `/get_entity_state`, not physics truth. Evidence:
   `reports/arena-fixes-2026-08-19/object-on-table.json`.
2. **`ROS_DOMAIN_ID` trap.** `.deployment.env` sets `ROS_DOMAIN_ID=25` and
   `scripts/launch-humble` defaults to `${ROS_DOMAIN_ID:-25}`, so sourcing
   `.deployment.env` silently overrides the 42 that live arena runs use.
   Export 42 AFTER sourcing, in every shell, including the one that runs
   `launch-humble`. `.deployment.env` must still be sourced — it carries
   the Isaac EULA acceptance variable.
3. **`scenario_runner` needs `PYTHONPATH` under a bare `ros2 run`.** It
   imports `tinker_sim_core` at module level (`scenario_runner.py:22`),
   so CLI invocations need `PYTHONPATH=$PWD/simulation:$PYTHONPATH`.
   `actor_path_driver` does NOT need this — it resolves
   `tinker_sim_core` from `--root` as of commit `bd4b553`.

Status: development-validated only, **not release-qualified**.

## 2026-09-05: fabric-off cost + opt-in spawn-yaw-via-view (#24, unvalidated)

Profiling flagged `TINKER_SIM_USE_FABRIC=0` (forced off by 13e4fdf whenever
`TINKER_SIM_SPAWN_YAW` is non-zero) as the single largest non-physics RTF
cost found: `updateToUsd` -- PhysX writing every rigid body's transform back
to USD every step, plus the Hydra scene-notice resync that write triggers --
profiles at roughly 1.1 s of wall per simulated second. 13e4fdf's own root
cause for forcing fabric off is narrower than that cost: `omni.physx.fabric`
resolves a *newly-spawned sibling body's* initial world transform against
the robot root's non-identity yaw when fabric is authoritative (i.e. when
`use_fabric=True` sets `/physics/updateToUsd=False`), so a scenario object
spawned after a yawed robot boot lands rotated by `-robot_yaw` about the
robot's origin in physics, even though USD/`get_entity_state` read back the
commanded pose correctly. Fabric-off makes USD authoritative for that
ingestion and removes the leak, at the `updateToUsd` cost above. A full
dependency audit (`/home/tinker/.claude/jobs/01ca17b4/tmp/fabric-off-dependencies.md`)
found this SPAWN_YAW mislocation is the *only* reason fabric is off anywhere
in this repo -- every other fabric-off-adjacent fix (arena/gripper/YCB
friction materials, #12's `set_entity_pose_physics`, `_apply_base_hold`) is
either a tensor-view write that is already fabric-independent by
construction, or a USD-authoring compensation for a second-order defect
(mesh colliders not inheriting the PhysicsScene default material under
`updateToUsd`) that is harmless under fabric-on.

**Hypothesis** (untested beyond static reading + CPU unit tests): the
pre-reset USD `xformOp:orient` write that authors the spawn yaw is not
itself required to be a USD write -- `_apply_base_hold` and #12's
`set_entity_pose_physics` already write the robot's root pose through the
Isaac Lab / PhysX tensor view (`write_root_pose_to_sim_index` /
`create_rigid_body_view(...).set_transforms`), which is fabric-independent.
If the spawn yaw is instead written through that same view, *after*
`sim.reset()` binds the articulation, fabric would not need to be forced
off at all for the SPAWN_YAW path. This is a **candidate fix, not a proven
one**: 13e4fdf's own diagnosis was that the defect's trigger is the robot
root carrying a non-identity yaw *in physics* at the moment a sibling body
spawns -- a condition switching the write mechanism does not obviously
change, since the robot root ends up at the same yawed pose in physics
either way, just via a different write path. It needs a live GPU A/B, not
more source reading.

**Shipped, default-off, current behaviour unchanged when unset**:
`TINKER_SIM_SPAWN_YAW_VIA_VIEW=1` (`simulation/tinker_sim_isaac/backend.py`).
When set and `TINKER_SIM_SPAWN_YAW` is non-zero: `use_fabric` stays as
`resolve_use_fabric` would otherwise compute it (normally `True`, i.e. NOT
forced off), the pre-reset USD orient authoring is skipped entirely, and
once the articulation is bound and reset, `_apply_spawn_yaw_via_view` writes
the commanded yaw as a root-view quaternion (`(x, y, z, w)` order -- NOT the
`(w, x, y, z)` order `InitialStateCfg.rot` and the USD `xformOp:orient` path
use; this tripped up the initial reading of the vendored IsaacLab source and
is called out explicitly in both the code and its tests) before the first
physics step, and seeds `_base_hold_pose`/`_base_hold_vel`/
`_base_hold_scene_sig` directly so `TINKER_SIM_FIX_BASE=1`, if also active,
holds the yawed pose immediately rather than only picking it up once its own
2 s settle-latch fires. A `{"event":"spawn_yaw","via":"view"|"usd",
"use_fabric":...}` boot-log line records which path ran, alongside the
pre-existing `{"use_fabric":...,"spawn_yaw_set":...}` line, so a live boot's
stdout says unambiguously which path it took.

**Fix round 1 (same day, code review `/home/tinker/.claude/jobs/01ca17b4/tmp/task24-yawview-review.md`, Finding 1 -- CONFIRMED)**:
the first cut of this flag only applied the yaw once, at the initial boot
bind (right after `sim.reset()`). Every standard scenario boot performs
reset -> STOP -> `spawn_entity`(s) -> PLAY (`simulation_interfaces`'
`/reset_simulation` + `/set_simulation_state`, the default
`spawn_while_playing=False` flow -- see `simulation/README.md` and
`simulation/tinker_sim_core/orchestration.py`), and Isaac Lab recreates the
articulation root view on that PLAY/PHYSICS_READY transition
(`_refresh_robot_handles` already detects and handles the new view identity
for every other piece of cached state). Since the pre-reset USD
`xformOp:orient` authoring is deliberately skipped under this flag, the
commanded yaw had no durable USD backing -- it lived only in the transient
root-view tensor buffer -- so that reset/rebind silently snapped the robot
back to the USD/`InitialStateCfg`-composed identity orientation, *right
after the spawn sequence this flag exists to make cheap*. With
`TINKER_SIM_FIX_BASE=0` (the nav-profile default) nothing ever re-applied
the yaw for the rest of the run; with `FIX_BASE=1`, `_apply_base_hold`'s
Python-cached target happened to survive and re-fix it, but only after one
physics step at identity yaw.

Fixed by moving the reapply into `_refresh_robot_handles` itself, via a new
`_reapply_spawn_yaw_after_rebind` helper: it runs on every genuine view-identity
change that method detects (the initial boot bind included, so the explicit
call the first cut made in `__init__` right after `sim.reset()` is now
redundant and was removed), covering all three call sites that can rebind
the articulation view -- `__init__`'s boot path, `step()`'s re-entrant
PHYSICS_READY rebind branch, and `_maybe_recover_simulation_view`'s manual
view recreation -- from one place instead of three. It re-reads the
*current* root position from the freshly (re)bound view rather than
replaying a cached boot-time position, so it stays correct even if the base
moved before the reset. Confirmed safe to call before any explicit
`Articulation.update(dt)`: IsaacLab's `root_link_pose_w` is a
timestamp-checked proxy that re-fetches from the physics view on every
access regardless of `.update()` (`.deps/IsaacLab/.../articulation_data.py:616-630`),
so there is no stale-buffer window. The one unavoidable residual from the
review (not fixed, and not fixable without changing Isaac's own
PHYSICS_READY dispatch order): `step()`'s rebind branch must still run one
`_sim.step()` at whatever orientation the just-recreated view reports
*before* it can call `_refresh_robot_handles()` again successfully -- so a
single physics step at the pre-reapply orientation is unavoidable on every
reset, same as it always was for every other piece of state that branch
re-synchronizes.

Non-GPU coverage (`tests/test_manipulation_runtime.py`,
`UseFabricDerivationTest` + `SpawnYawViaViewApplyTest` +
`SpawnYawViaViewRebindTest`): the `use_fabric` derivation truth table (yaw
set + flag off -> False, unchanged; yaw set + flag on -> True; no yaw ->
True regardless of flag; `TINKER_SIM_USE_FABRIC` override still wins over
the new flag), the view-write itself (issues the root-pose write exactly
once, position taken from the articulation's already-resolved `root_pos_w`,
quaternion matching a pure yaw in `(x, y, z, w)` order, and the base-hold
seeding only firing when `base_fixed` is set), and the rebind-durability
fix (`_refresh_robot_handles` reapplies the yaw and re-seeds the base-hold
target on a forced view-identity change, does so again on a second,
independent rebind, does nothing when the view identity is unchanged, and
does nothing at all when the flag is off or unset) -- all against a backend
test double. These cannot and do not exercise a real PhysX root view,
fabric's sibling-spawn ingestion, the actual PHYSICS_READY dispatch timing,
or whether the hypothesis above actually holds.

**Fix round 2 (same day, re-review `/home/tinker/.claude/jobs/01ca17b4/tmp/task24-yawview-review-r1.md`,
new finding -- CONFIRMED)**: fix round 1 closed Finding 1 correctly but
introduced a regression: `_reapply_spawn_yaw_after_rebind` ran on *every*
view-identity change `_refresh_robot_handles` detected, with no distinction
between a genuine reset (physics state reset to the initial pose anyway --
safe to reapply) and `_maybe_recover_simulation_view`'s mid-run,
state-PRESERVING view recovery (reachable any time via
`_heal_detached_scenario_bodies`, budgeted 5 uses/boot, whose own docstring
says it deliberately skips `force_load_physics_from_usd` specifically so
bodies keep their live/settled pose across the recovery). A nav-profile
robot (`FIX_BASE=0`) that had driven/turned to a live heading would get that
heading silently snapped back to the stale `TINKER_SIM_SPAWN_YAW` boot value
on the next such recovery; with `FIX_BASE=1` the base-hold reseed made the
wrong heading persistent rather than a one-off glitch. The review's point:
"is this a reset" cannot be inferred from the measured pose (a driven
robot's live heading is indistinguishable from a stale one by pose alone),
so the classification has to be explicit.

Fixed by giving `_refresh_robot_handles` an explicit
`reapply_spawn_yaw: bool = True` keyword, decided by the CALLER, never
inferred: the two genuine-reset callers (`__init__`'s boot bind and
`step()`'s PHYSICS_READY rebind branch) take the default and are unchanged;
`_maybe_recover_simulation_view`'s call now explicitly passes
`reapply_spawn_yaw=False`, so that recovery path never writes a root pose
and never touches `_spawn_yaw` or an existing base-hold target -- matching
what the USD-authoring path already does today (no code re-authors
`xformOp:orient` outside boot, so a state-preserving recovery already left a
flag-off robot's live heading alone; this makes the flag-on view-write path
behave the same way).

Non-GPU coverage added (`tests/test_manipulation_runtime.py`,
`SpawnYawViaViewRebindTest.test_boot_bind_reapplies_yaw` +
`SpawnYawViaViewRecoveryRebindTest`): boot (`_robot_view_identity=None`)
reapplies; a genuine-reset rebind (default `reapply_spawn_yaw=True`)
reapplies (already covered in round 1); a recovery-classified rebind
(`reapply_spawn_yaw=False`) issues no root-pose write at all and leaves a
pre-set `_spawn_yaw` and a driven, pre-latched `_base_hold_pose`/
`_base_hold_vel`/`_base_hold_scene_sig` completely byte-identical to what
they were before the call, while still performing the rest of the rebind
(joint index caches, clock re-anchoring, view-identity bookkeeping) exactly
as before. Full suite: 87 passed, 3 subtests passed, 0 failed.

This is a unit-tested invariant only -- no GPU harness in this repo can
currently trigger `_maybe_recover_simulation_view` on demand (it fires from
a caught tensor-view exception, not a controllable command), so there is no
live check analogous to the reset-survival one below for this path; see the
validation recipe's note on this.

**Not done here**: any GPU boot. The validation recipe --
`/home/tinker/.claude/jobs/01ca17b4/tmp/fabric-on-validation-recipe.md` --
lays out the exact A/B (spawn-yaw truth-pose comparison against 13e4fdf's
repro shape, robot root yaw from `/sim/internal/physics_truth`, and
`TINKER_SIM_PROFILE=1`'s `step_profile.kit_pump`/RTF numbers) for whoever
runs it next, now including a reset-survival check for Finding 1's fix
(spawn through the standard reset cycle, then re-read the robot's base yaw
from truth and confirm it is still the commanded value, not identity). Do
not treat this flag as validated, and do not flip any default based on this
entry alone.

## 2026-09-06: spawn-yaw-via-view + FIX_BASE held the base at its un-settled spawn height (#26)

**Symptom**: with `TINKER_SIM_SPAWN_YAW_VIA_VIEW=1` and `TINKER_SIM_FIX_BASE=1`
both active, the held base-frame z came out ~0.1954 instead of the ~0.0775
settled rest height the flag-off (USD-authored-yaw) path produces -- an
11.8 cm base-frame error, first caught on a bench round ("agu") comparing
base pose against the flag-off baseline.

**Root cause**: `_apply_spawn_yaw_via_view` (`simulation/tinker_sim_isaac/backend.py`)
runs once the articulation is bound and `self._sim.reset()` has returned,
*before the first physics step* -- exactly per its own docstring. The #24
fix round that added FIX_BASE seeding (see the 2026-09-05 entry above,
"seeds `_base_hold_pose`/`_base_hold_vel`/`_base_hold_scene_sig` directly")
took `data.root_pos_w` at that same pre-physics moment and latched it
straight into `_base_hold_pose`. That is the un-settled spawn height
(`InitialStateCfg.pos`'s z, ~0.20, minus a small articulation-resolve
offset) -- the free-base chassis has not yet had gravity drop it onto its
wheels/casters. Because `_base_hold_pose` was now non-None from step 0,
`_apply_base_hold`'s own settle-latch (`if self._base_hold_pose is None: if
self.simulation_time < self._base_hold_after_sim_s: return`, gated on a
2.0 s wait) never fired -- the branch that would otherwise have captured
the settled pose was permanently skipped, so the pre-settle height was held
for the entire run instead. The flag-off path never seeds anything here (it
is guarded out of `_apply_spawn_yaw_via_view` entirely, which flag-off never
calls), so it always went through the settle-latch and got the correct
settled height -- which is exactly why the two paths disagreed.

**Fix**: `_apply_spawn_yaw_via_view` no longer seeds `_base_hold_pose`'s
position at all. It now only remembers the commanded yaw quaternion in a
new pending field, `_base_hold_seed_quat`, and (when `base_fixed`) clears
any already-latched `_base_hold_pose`/`_base_hold_vel` back to `None` --
necessary on a genuine rebind (STOP -> spawn -> PLAY), since nothing else
re-drives the settle-latch once `_base_hold_pose` is non-None, and a
genuine rebind resets the chassis back to its unsettled spawn pose just
like boot did. `_apply_base_hold`'s existing settle-latch is otherwise
unchanged -- it still waits for `simulation_time >= _base_hold_after_sim_s`
-- except that when it fires, it now composes the freshly-read (and by then
settled) `root_pos_w` with `_base_hold_seed_quat` if one is pending, instead
of the measured `root_quat_w`, so the commanded yaw still survives into the
hold exactly as the #24 fix intended. Flag-off behaviour is unchanged
(`_base_hold_seed_quat` stays `None` for that path, so the latch falls back
to the measured orientation exactly as before). `_reapply_spawn_yaw_after_rebind`'s
rebind classification (`reapply_spawn_yaw`, #24 round 2) was not touched --
it still decides whether this whole method runs at all, independent of what
it does internally.

**Validation gap that let this ship**: the #24 GPU validation recipe
(`fabric-on-validation-recipe.md`) did exercise `FIX_BASE=1` and passed its
"spawn clean, reset survival" checks, but those checks read only
`robot.base_pose.quaternion_xyzw` -- orientation persistence across a
rebind -- never `base_pose.xyz`'s z-height against a settled baseline. It
was checking a different axis of correctness than the one that broke.

Non-GPU coverage (`tests/test_manipulation_runtime.py`,
`SpawnYawViaViewApplyTest.test_base_hold_latches_settled_height_with_seeded_yaw`,
new): seeds the yaw with `root_pos_w` z=0.20 (un-settled), mutates the mock's
`root_pos_w` to z=0.0775 (settled) and advances simulated time past
`_base_hold_after_sim_s`, then calls `_apply_base_hold` directly and asserts
the latched hold's z is 0.0775 (not 0.20) *and* its orientation is the
seeded yaw (not whatever `root_quat_w` happens to read at latch time). This
test fails against pre-fix `backend.py` (`_base_hold_pose` is not `None`
immediately after `_apply_spawn_yaw_via_view`, so the settle-latch branch
never runs). `test_seeds_base_hold_target_when_fix_base_active`,
`test_does_not_seed_base_hold_when_fix_base_inactive`, and
`test_reapplies_yaw_on_rebind_when_via_view_active` were updated to assert
the new contract (yaw seeded into `_base_hold_seed_quat`, `_base_hold_pose`
left/reset to `None`, not immediately re-latched). Full suite:
`tests/test_manipulation_runtime.py` 103 passed, 3 subtests passed, 0
failed. Still no GPU boot for this fix -- same caveat as #24 above.

**Round 2 (same day): the settle window is measured from process BOOT, not
from a rebind, so the fix above reproduced its own bug on every mid-run
reset.** Code review (`$TMP/task26-review.md`) caught it before a GPU gate:
`_apply_base_hold`'s settle-latch gates on `if self.simulation_time <
self._base_hold_after_sim_s: return` -- a one-shot, absolute 2.0 s deadline
from `simulation_time == 0`. But `_reapply_spawn_yaw_after_rebind` (->
`_apply_spawn_yaw_via_view`) runs on every genuine rebind, not just boot --
`step()`'s PHYSICS_READY branch calls it after every standard scenario
STOP -> spawn -> PLAY cycle -- and it unconditionally clears
`_base_hold_pose`/`_base_hold_vel` back to `None` each time (the round-1
fix above, needed so the settle-latch runs again after a respawn). Because
`simulation_time` is deliberately kept MONOTONIC across a rebind (#21's own
fix, `_clock_step_origin` re-anchored to the elapsed step count instead of
re-zeroed), a mid-run reset happening minutes into a live run finds
`simulation_time` already well past 2.0 s the instant the clear runs.
`_apply_base_hold`'s very next call therefore sees `_base_hold_pose is
None` *and* the boot-relative deadline already satisfied, skips the
settle-wait branch entirely, and re-latches immediately from
`data.root_pos_w` read right after the respawn -- the exact pre-settle
height bug the round-1 fix targeted, now recurring on every subsequent
reset for the rest of the run instead of once at boot. Flag-off has no
equivalent defect: it never clears `_base_hold_pose` on a rebind (its
branch of `_apply_spawn_yaw_via_view` is skipped entirely), so it just
keeps reasserting whatever was latched at boot and never re-enters the
`is None` branch.

**Fix**: track the settle deadline relative to the LAST CLEAR, not process
boot, on the via-view branch only. New field `_base_hold_settle_from`
(`__init__`, initialised to `0.0` next to `_base_hold_after_sim_s` -- boot
behaviour is unchanged, since `simulation_time - 0.0` is just
`simulation_time`). `_apply_spawn_yaw_via_view` now records
`self._base_hold_settle_from = self.simulation_time` in the same
`if getattr(self, "base_fixed", False):` block that clears
`_base_hold_pose`/`_base_hold_vel` -- so every clear (boot or a genuine
mid-run rebind) re-arms its own 2.0 s window. `_apply_base_hold`'s gate
branches on whether the via-view path owns the hold
(`_base_hold_seed_quat is not None`): when it does, the check becomes
`simulation_time - _base_hold_settle_from < _base_hold_after_sim_s`;
otherwise (flag-off, `seed_quat is None`) the original boot-relative
`simulation_time < _base_hold_after_sim_s` check runs unchanged, so
flag-off stays byte-identical.

New test (`SpawnYawViaViewRebindTest.test_rebind_after_boot_waits_for_settle_before_relatching`):
fakes 40.0 s of elapsed sim time (well past the 2.0 s deadline) *before*
triggering a rebind via `_refresh_robot_handles()` with `root_pos_w`
z=0.20 (pre-settle), then calls `_apply_base_hold()` immediately and
asserts `_base_hold_pose` is still `None` (not re-latched). It fails
against a495958 with:
```
E       AttributeError: 'IsaacWholeRobotBackend' object has no attribute '_base_hold_settle_from'. Did you mean: '_base_hold_resettle_s'?
```
(the field didn't exist pre-fix). Advancing simulated time by exactly 2.0 s
more with `root_pos_w` z=0.0775 and calling `_apply_base_hold()` again
asserts it now latches at the settled height with the seeded yaw intact.
A flag-off counterpart, `test_flag_off_rebind_keeps_previously_latched_hold`,
asserts a rebind with `_spawn_yaw_via_view = False` never touches an
already-latched `_base_hold_pose`/`_base_hold_vel` -- byte-identical to
pre-#26 behaviour, confirming this round's change doesn't give flag-off a
settle timer it never had. Full suite:
`tests/test_manipulation_runtime.py` 105 passed, 3 subtests passed, 0
failed. Still no GPU boot for either round of this fix.
