# Simulation handoff — 2026-08-18

This note records the state left by the most recent development session in this checkout.

## Completed

- Merged the RoboCup arena work into `task50-stage-a-repair` as `e4ff59c`; the task50 streaming work was preserved as `f255220`.
- Retained the RoboCup 2026 arena artifact under `artifacts/arena/rcw2026/` (current identity `d2b559b43207c8d54ae2609f638dca1cc36ee8b7adc7e4d94aee86e7fb56729c`) and the YCB artifact under `artifacts/objects/ycb/` (current identity `d2d5ccd2c098b68f39737f8f0490358b7fd6cbfa8080b604851e073fc758acda`).
- Added `artifacts/asset-manifest.json` for the robot USD, Isaac prewarm record, arena USD, and ten YCB object USDs; the fail-closed asset verification passed.
- Fixed the deploy launch argument path in `d8c5584`: `--arena` is now parsed and forwarded to `run_sim.py`, and launch parsing rejects silent flag-prefix matches.
- Added four regression tests in `tests/test_deploy_cli_launch.py`; the focused suite passed 4/4 and the launch-contract suites stayed green.

## Remaining validation

- The exact `./scripts/launch-arena-streaming --arena rcw2026` end-to-end run was blocked by an existing streaming-viewer singleton lock (pid 458524); no final visual confirmation was recorded.
- Still to validate: a textured frame, sensor-rich camera imagery of the arena furniture, and AMCL convergence on the derived map. Physics interaction with arena furniture was already closed by a live drop test.

## Next steps

1. Ensure no other streaming viewer is active and run `./scripts/launch-arena-streaming --arena rcw2026`.
2. Capture textured and sensor-rich evidence, then validate AMCL on the derived map.
3. Update the RoboCup status section of `README.md` with the evidence and rerun the focused arena/CLI tests (then full discovery if launch code changes).

## Update — 2026-08-18, later session

Steps 1–3 above are done; see the 2026-08-18 README changelog entry for the
full record. In brief: the arena streaming session ran end to end and is
left up awaiting a human viewer (the one still-open item); sensor-rich
head/wrist furniture imagery is in `reports/arena-sensor-rich-2026-08-18/`
and `reports/arena-arm-camera-2026-08-18/`; AMCL was validated against
physics truth on the derived map (`reports/arena-amcl-2026-08-18/`), which
required two additive fixes — a `--spawn-xy` launch override (the default
(0,0) arena spawn is inside `shelf_02`'s footprint) and a `map_yaml:=`
argument for `navigation.launch.py`. Focused suites 78/78 plus doc suites
green; full-discovery failures are environmental/external only, plus the
pre-existing uncommitted `tests/test_base_velocity_slew.py`, which imports
`slew_velocity_target` that no commit implements (in-progress task50 work —
do not delete the test; implement or land the missing backend function).
Note: `tk25_ws/src/tk25_manipulation` currently sits on branch
`collision-aware-grasp`, which breaks `test_provenance`'s pinned-commit
reads until that workspace is restored.

## Update — 2026-08-21, arena-findings plan wrap-up

The arena-findings plan (Tasks 1-14) closed out its code and docs, but the
live evidence wave it depended on did not pass. What it proved live,
what it disproved, and what remains open — full detail and evidence paths
are in the README changelog's 2026-08-21 entry:

**Proved live** (`reports/arena-fixes-2026-08-19/`): the head pan/tilt
effort-cap fix converges within tolerance
(`head-tracking.json`); the fail-closed arena-spawn check exits 1 with a
`--spawn-xy` suggestion (`spawn-fail-closed.log`); the wheel-velocity
slew/coast bound holds an idle base to ~8e-05 m of drift over 30 s
(`coast.json`).

**Disproved / reopened by the wave**: `actor_path_driver` crashed on its
first `/clock` message (`_clock` shadowed by `rclpy.node.Node`'s own
attribute) — fixed in `de95b71`, not yet re-run live
(`person-walk.json`). `pick-deliver-place`'s spawned object never appeared
in `/sim/internal/physics_truth` at all under `navigation-parity` — Task 14
(`02d1785`, `bddc0f9`) fixed the confirmed cause (H1: the backend
constructor never passed `expected_objects`/`scenario` on that branch;
unit-proven, `tests/test_scenario_object_tracking.py`, 113 passing), but a
second, still-unconfirmed cause (H2: the object may be outside PhysX's
tensor view and genuinely unsimulated, evidenced by fabricated zero
velocities and a `Physics tensor entity not valid` Isaac warning on every
query) needs its own live re-run to separate from H1
(`object-on-table.json`). This also means the 2026-08-19 "object rests on
furniture" close-out (README, `object-spawn-verification.md`) is
superseded: that evidence came from `/get_entity_state` polling, not
physics truth, and should be treated as open again, not confirmed or
refuted.

**Never validly run**: step 6, navigation-profile safety-clear without a
manual CLI heartbeat (`navigation.launch.py` now starts its own
`safety_supervisor`, code-complete). The attempt was misconfigured onto
`ROS_DOMAIN_ID=25` while Isaac ran on 42 (`.deployment.env` sets 25, and
`scripts/launch-humble` defaults to `${ROS_DOMAIN_ID:-25}`, so sourcing
`.deployment.env` silently overrides an exported 42 unless 42 is
re-exported afterward, in every shell).

**Exact command to finish it**: the fix-and-brief work for this is already
written at
`.superpowers/sdd/vast-strolling-parrot/task-12-steps-4-6-brief.md` — it
carries the corrected step-5 resting-height assertion, the exact
`scenario_runner --root/--scenario/--seed` invocations, the verified
`.ros-vendor/humble/local_setup.bash` environment recipe, and the
domain-export fix for step 6. Once the GPU is free again (another session
is actively using it as of this update — do not launch anything
GPU-bound without checking first), re-run that brief's steps 4-6 as a live
wave: step 4 re-verifies `actor_path_driver`'s walk with `de95b71` in
place; step 5 re-verifies scenario-object physics tracking with `02d1785`
in place and discriminates H1 from H2 (objects appearing but not settling
means H2 is real and separate); step 6 needs `ROS_DOMAIN_ID=42` exported
after sourcing `.deployment.env`, in every shell including the one that
runs `launch-humble`, then asserts `/sim/status/command_gateway` reports
`"safety_stop": false` with the arena nav stack up and no manual safety
heartbeat running.

The branch (`task50-stage-a-repair`) is shared: a separate workstream has
also been committing to it since 2026-08-21 (runbook docs, a
kill-domain-nodes fix, an opt-in physics rate override, a PhysX
target-write gate, a head initial-pose fix). That work is unrelated to
this plan and is not covered above. One consequence worth flagging here:
`tests/test_chassis_ballast.py` now fails 6/6 against committed code,
because that workstream raised `CHASSIS_BALLAST_ADDED_MASS_KG` from 10.0 to
30.0 without updating the test's 30 kg total assertion — not this plan's
regression, not fixed here.
