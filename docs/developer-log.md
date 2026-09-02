# Developer log

Dated engineering notes: what was measured, what was ruled out, why a fix
took the shape it did. Operational instructions live in
`docs/gpsr-sim-runbook.md`; this file is the history behind them.

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
