# GPSR Recorded Sim Battery (Tier 2/2+) — Design

Date: 2026-08-25. Status: approved by user (design presented in chat 2026-08-25; answers: hybrid for 40 runs + 10 full-manipulation runs; spectator + head camera contact sheets; sim-feasible-biased corpus).

Parent spec: `2026-08-23-gpsr-command-variety-testing-design.md` (the four-tier ladder). This spec delivers the sim tiers with video evidence: 50 generated commands run against the Isaac sim, each producing a reviewable per-run contact sheet.

## Goal

Run 50 GPSR commands from the official command generator against the tinker Isaac simulation:
- Runs 1–40 ("hybrid", tier 2): navigation + vision + audio live in sim, **manipulation mocked**.
- Runs 41–50 ("full", tier 2+): everything live including manipulation.
Each run is recorded from two views (arena observer camera + head camera) and condensed into one contact-sheet JPEG. A report aggregates verdicts per template/class and links every sheet.

## Non-goals

- No scripted actor motion (persons remain static; follow/guide templates with moving-person semantics are excluded from the corpus and reported as skipped).
- No WebRTC/streaming changes; the existing live-view path is untouched.
- No fixes to orchestrator findings from Phase 1 (they are inputs to interpretation, not work items here).
- No mp4 video artifacts — frames + contact sheets only (frames are kept on disk, git-ignored; only sheets + reports are committed).

## Components

### C1. Arena observer camera (tinker-sim)

- New RTX camera `arena_camera` in `simulation/tinker_sim_isaac/camera_rig.py`, world-fixed (not robot-mounted), pose computed from occupancy-map bounds by the same math as `validation/run_sim.py:_arena_camera_pose()` (3/4-elevated overview; refactor that helper into a shared module so both callers use one implementation).
- Publishes `/sim/arena_camera/image_raw` (`sensor_msgs/Image`, rgb8) + `/sim/arena_camera/camera_info`, 960×540 @ 4 Hz, QoS reliable/volatile/keep_last(1).
- Opt-in: env `TINKER_SIM_ARENA_CAMERA=1` (default off — zero cost when unset). Rate override `TINKER_SIM_ARENA_CAMERA_HZ` (may only lower 4 Hz, mirroring `TINKER_SIM_CAMERA_HZ` semantics).
- Not part of `hardware-parity.json` (it has no real-robot counterpart); it is a sim-only observer and must not appear in parity checks or the interface census.

### C2. Run recorder (tinker-sim)

- `validation/gpsr_run_recorder.py`: rclpy node, CLI: `--out DIR --topics /sim/arena_camera/image_raw=arena /camera/color/image_raw=head --interval 1.0 --max-frames 900`.
- Saves `DIR/frames/<label>/<seq:04d>_<sim_ts_ms>.jpg` (quality 85) at most one frame per label per interval; writes `DIR/recorder-meta.json` (start/end wall + sim time, frame counts, topics) on shutdown (SIGINT-safe).
- Frame-decode logic (`sensor_msgs/Image` rgb8 → JPEG) is a pure function unit-tested without ROS.

### C3. Contact sheet builder (tinker-sim)

- `tools/contact_sheet.py`: pure-PIL CLI: `--run-dir DIR --meta run.json --out sheet.jpg --columns 12`.
- Layout: header band (run id, command text wrapped, verdict [colour-coded PASS green / FAIL red / TIMEOUT amber / ERROR grey], duration, tier); row 1 = arena frames, row 2 = head frames; 12 tiles per row sampled evenly across each label's frame sequence (all frames if fewer than 12); each tile captioned `t=<sim_s>s`. Tile width 320 px, JPEG quality 80; a sheet stays under ~1.5 MB.
- `run.json` contract (written by the tier-2 runner): `{id, text, template, feasibility, tier, verdict, detail, seconds}`.
- Degrades gracefully: missing label row is replaced by a "no frames captured" band, never a crash.

### C4. Bench scenario (tinker-sim)

- `simulation/scenarios/gpsr-rcw2026-bench.json`: copy of `gpsr-rcw2026.json` plus a second person actor (`livingroom_person`, `person_standing` asset, near the sofa, ≥2 m from the waypoint and unoccluded) so person-counting > 1 is testable. Objects unchanged. Validated by the existing scenario schema tests.

### C5. Stack bring-up/teardown (tinker-sim)

- `scripts/gpsr-stack` (python, argparse: `up|down|status`, `--scenario gpsr-rcw2026-bench`, `--seed`, `--manipulation {mock,live}`, `--dry-run`): automates runbook stages 1–5 in order with per-stage logs under `gpsr_stack_logs/<ts>/` and PGID files:
  1. sim: `./scripts/tinker-sim launch --sensor-profile sensor-rich --scenario <s> --seed <n> --arena rcw2026 --spawn-xy=-2.0,-2.0 --ros --headless` with `ROS_DOMAIN_ID=42 TINKER_SIM_CAMERA_HZ=12 TINKER_SIM_CONTROL_HZ=60 TINKER_SIM_ARENA_CAMERA=1`.
  2. bridge composite `gpsr.launch.py` (scenario/seed args as runbook).
  3. tk26_vision `vision_bringup.launch.py enable_gpsr:=true` + the two runbook manual gaps (named `head_camera_server`, pan/tilt state publisher).
  4. tk25_manipulation (`--manipulation live` only): `manipulation_planning_task_only.launch.py execution_profile:=sim_cumotion` + grasp servers, on the non-sim GPU (`CUDA_VISIBLE_DEVICES` per the GPU-assignment convention: vision shares the sim GPU, manipulation gets the other).
  5. tk26_navigation extras (`approach_planner`, `orientation_angle_service`).
  Readiness gate between stages and at the end: `tools/gpsr_interface_census.py` (subset appropriate to `--manipulation mock`); `up` fails loudly with the failing census lines if not ready in 180 s.
- `down`: reverse order, SIGINT with 10 s grace then SIGKILL **by recorded PGID only** (never name patterns), explicit `cumotion_goal_set_planner_node` kill when manipulation was live, final `nvidia-smi --query-compute-apps` report. Nav teardown before sim (SHM-lock pitfall).
- `--dry-run` prints the exact commands without executing (unit-testable).

### C6. Tier-2 runner (tk25_decision)

- `GPSR/bench/tier2.py`: `run_tier2(entries, *, mock_config, constants, plan_dir, out_dir, timeout_s, launcher=DEFAULT_LAUNCHER, reset_cmd=DEFAULT_RESET_CMD, recorder_cmd=None, sheet_cmd=None)`.
- Per entry (always one command per run):
  1. `reset_cmd` (default `ros2 service call /reset_simulation simulation_interfaces/srv/ResetSimulation {}` with a 60 s bound); wait `settle_s` (default 10) for /clock to advance past re-zero.
  2. Start `recorder_cmd` (template with `{run_dir}` placeholder) as a subprocess in its own process group, if given.
  3. Launch a **fresh** orchestrator subprocess (same launcher/env contract as tier1: `BT_GPSR_CMD=<single command>`, `BT_MOCK_CONFIG=<sim-hybrid or sim>`, `GPSR_OFFLINE_PLANNER=0`, `GPSR_CONSTANTS_PATH`, absolute `BT_GPSR_PLAN_DIR`, `GPSR_DEBUG_TELEMETRY=1`); never reuse an orchestrator across runs (cached action goal handles stall after any stack hiccup).
  4. Verdict via the tier-1 telemetry parsing (`parse_events`, executor-node fallback, per-task timeout = `timeout_s`; single slot).
  5. Stop orchestrator (process-group SIGINT→SIGKILL as tier1), stop recorder (SIGINT, 15 s grace), write `run.json`, invoke `sheet_cmd` (template with `{run_dir}` `{run_json}` `{out}`), append result.
- Env var `ROS_DOMAIN_ID` is inherited (42 when driven by `scripts/gpsr-stack`); tier2 never sets it itself.
- CLI: `gpsr-bench tier2 --corpus F --out DIR --mock-config M --timeout 600 [--recorder-cmd ... --sheet-cmd ... --reset-cmd ... --limit N --start K]` (`--limit/--start` allow the 40/10 split and resume after interruption).
- Report: reuses `bench/report.py` (tier column "T2" / "T2+"); per-run line gains `sheet` path; `SUMMARY.md` gains a "Runs" section linking each contact sheet relative path.

### C7. Sim-feasible corpus (tk25_decision)

- `gpsr-bench gen --sim-feasible` flag: after normal generation, drop entries whose template is in `SIM_INFEASIBLE_TEMPLATES` (follow/guide moving-person templates: `followNameFromBeacToRoom`, `followPrsAtLoc`, `followPrsToRoom`-style, `guideNameFromBeacToBeac`, `guideClothPrsFromBeacToBeac`, `guidePrsFromBeacToBeac`, `guidePrsToBeacon` follow-ups; clothing-description templates: `greetClothDscInRm`, `countClothPrsInRoom`, `guideClothPrs*` — single person asset, no clothing variety), continuing generation until the requested count is reached; record skipped template counts in the corpus file header line (JSON comment entry `{"_skipped": {...}}` as line 1).
- Battery composition (seed chosen fresh, e.g. 2026): `--sim-feasible --count 40` general corpus for hybrid; a manipulation set of 10 drawn with `--templates takeObjFromPlcmt,bringMeObjFromPlcmt,findObjInRoom(+takeObj follow-ups)` for tier 2+.

### C8. Mock configs (tk25_decision)

- `mock_config.sim-hybrid.json`: global mock enabled, **only `manipulation` subsystem `enabled:true`** (nodes IMMEDIATE), all five other subsystems `enabled:false` (real); `keyboard_control` off; `force_mock_nodes` teleop kept.
- Runs 41–50 use the existing `mock_config.sim.json` unchanged.

## Data flow

corpus.jsonl → tier2 runner → (reset → record → orchestrate → verdict) per run → `out/runs/<id>/{frames/,run.json,sheet.jpg,orchestrator.log,debug/}` → report.py → `out/{report.json,SUMMARY.md}`. Committed: corpus, report.json, SUMMARY.md. NOT committed (git-ignored under `runs/`): frames/, logs, debug telemetry, and the `sheet.jpg` files — 50 sheets ≈ 50–75 MB, too heavy for the repo; sheets are delivered to the user directly for review and remain on disk, with `SUMMARY.md` linking their run-relative paths.

## Error handling

- Census failure at `up` → abort battery before any run.
- Reset service timeout, orchestrator spawn failure, recorder crash → verdict ERROR with detail; battery continues to the next run.
- Mid-battery stack death heuristic: 3 consecutive ERROR verdicts → runner stops and writes a `HALTED` marker with the reason (operator restarts stack, resumes with `--start`).
- Nav2 lifecycle timeout lines in the orchestrator log are captured into `detail` when a run fails (RTF-too-low signature).

## Testing

- Offline unit tests (both repos, no ROS): contact-sheet layout/sampling/degradation, recorder frame-decode + interval logic, arena-camera pose math + env parsing, scenario JSON schema, gpsr-stack `--dry-run` command assembly, tier2 orchestration with fake launcher/recorder/reset (as tier1's tests), corpus `--sim-feasible` filtering, sim-hybrid mock-config invariants.
- Smoke (live, before the battery): 2 hybrid runs + 1 full-manipulation run; verifies recording produces frames from both cameras, sheets build, reset cadence works twice in a row, and whether the anygrasp checkpoint exists (if missing: the 10 tier-2+ runs are attempted with the cumotion pipeline only, and the blocker is reported).
- Battery: 40 + 10 detached (launcher survives agent retirement), verdicts + sheets, report.

## Open risks (accepted)

- anygrasp checkpoint hardcoded/missing → tier 2+ may degrade to cumotion-only grasping or report a blocker (decided at smoke).
- B-class oracle weakness (YCB rigid bodies not resolving to PhysX views) → grasp success on video may not match truth-state; contact sheets are the mitigating evidence.
- RTF drop with the extra camera → arena camera is 4 Hz/960×540 and opt-in; if Nav2 bonds still time out at smoke, lower `TINKER_SIM_CAMERA_HZ` to 10 before lowering the arena rate.
- Reset stall until sim time passes the pre-reset stamp → mitigated by fresh orchestrator per run + `settle_s`; measured once at smoke.
- Planner latency (Phase-1 finding: 23/73 > 180 s) also applies here; 600/900 s budgets absorb it, and `seconds` records it.
