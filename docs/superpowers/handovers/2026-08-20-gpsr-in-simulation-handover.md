# Handover — GPSR in simulation

**Written:** 2026-08-20. **State:** paused after Task 1, to return to the arena-readiness workstream.

## What this is

Getting GPSR (the two-layer orchestrator in `tk25_decision`) to run against the
tinker-sim `rcw2026` arena with **real** vision and manipulation — nothing mocked.
Seven tasks, executed with superpowers:subagent-driven-development.

| Artifact | Path |
| --- | --- |
| Spec (binding authority) | `docs/superpowers/specs/2026-08-20-gpsr-in-simulation-design.md` |
| Plan (7 tasks) | `docs/superpowers/plans/2026-08-20-gpsr-in-simulation.md` |
| SDD ledger + briefs + reports | `.superpowers/sdd/2026-08-20-gpsr-in-simulation/` (git-ignored) |

The ledger is the recovery map. It holds the pre-flight conflict table, every
`Ruling:` line, and each task's completion record. Read it before the plan.
**`git clean -fdx` destroys it** — recover from `git log` if that happens.

## Two repos, two branches

| Repo | Branch | BASE at plan start | Now |
| --- | --- | --- | --- |
| `/home/tinker/tinker-sim/6.0.1` | `task50-stage-a-repair` | `08b29bb` | unchanged by this plan so far |
| `/home/tinker/tk25_ws/src/tk25_decision` | `gpsr-two-layer-orchestrator` | `3dc3ea7` | `43c0522` (Task 1) |

Tasks 1, 2 land in tk25_decision. Tasks 3, 4, 5, 6, 7 land in tinker-sim.
Never mix the two in one commit.

## Constraints in force

- **Vision and manipulation must not be mocked.** Real servers only.
- **Do not touch anything sensitive.** No reading, printing, copying, or moving
  credentials. `.env` is off limits. (Keys are already filled; existence was
  confirmed by measuring string length only.)
- **Subagent model policy (user directive):** sonnet, fable, or haiku only.
- Never `git add` `tk25_decision`'s untracked `.venv_decision.uv-project/` — teammate scratch.
- Commit messages end with `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- Never use `pkill` patterns that can match the running shell — explicit PIDs, or
  bracketed regexes like `pgrep -f 'run_sim[.]py'`.
- **Never contend for the GPU** with another session's Isaac. Check
  `nvidia-smi --query-compute-apps` first and yield.

## Task status

| # | Task | Repo | Status |
| --- | --- | --- | --- |
| 1 | Vision nodes `ServiceHandler` → `ActionHandler` | decision | Implemented (`bb71eda`, `43c0522`); **task review in flight** |
| 2 | `constants.rcw2026.json` + `GPSR_CONSTANTS_PATH` | decision | Not started — pre-derived, see below |
| 3 | Composite `gpsr.launch.py` | tinker-sim | Not started — two plan defects found, see below |
| 4 | `gpsr-rcw2026` scenario with YCB objects | tinker-sim | Not started — preconditions verified |
| 5 | `mock_config.sim.json` + runbook | tinker-sim | Not started |
| 6 | Interface census tool | tinker-sim | Not started — **needs GPU** |
| 7 | One command end-to-end | tinker-sim | Not started — **needs GPU** |

Tasks 1–5 are offline and can proceed any time. Tasks 6–7 are gated on an idle GPU.

## Task 1 — what was done and why it needed a fix round

The real-robot bug: `BtNode_FeatureExtraction` (`TemplateNodes/Vision.py`) and
`BtNode_ScanForWavingPersonNew` (`GPSR/custom_nodes.py`) subclassed
`ServiceHandler` and called `create_client`/`wait_for_service`, but tk26_vision
serves both endpoints as **ActionServers only**:

- `tk26_vision/src/tk_vision_specialized/.../waving_person_server.py:249-259` — `ActionServer(self, DetectWaving, 'detect_waving_persons')`
- `tk26_vision/src/kimi_api/kimi_api/feature_recognition.py:266-278` — `ActionServer(self, FeatureExtraction, 'feature_extraction_service')`

The `.srv` and `.action` goal/result fields are byte-identical, so the conversion
is mechanical. `BtNode_Grasp` (`TemplateNodes/Manipulation.py:152-257`) is the
copy pattern, including its mock-mode `MockFuture` guard.

The implementer flagged that `messages.py` still imported both names from
`.srv`, so the new `.Goal()` calls would `AttributeError` once mock mode is off.
Verified and fixed in round 1: those two names now come from `.action`.

### Known-open item — MUST do before any live GPSR run

The **installed** `tinker_vision_msgs_26` (built 2026-05-31) predates the
`.action` definitions (added to source 2026-08-18) and does not export
`FeatureExtraction`/`DetectWaving` as actions at all. The source is correct —
`tinker_vision_msgs_26/CMakeLists.txt:54-55` registers both. **Rebuild that
package before Task 6/7.** It was deliberately deferred: it is a cross-repo
colcon build affecting every consumer in `tk25_ws`, and another session held
the machine. This is recorded as a Task 6 prerequisite.

Related pre-existing environment gaps found while verifying (none caused by this
work, all worth a maintainer's attention): installed `tinker_vision_msgs_26.srv`
lacks `ReseedTarget`; `tinker_audio_msgs.action` lacks `NameDrinkExtraction`;
`py_trees_ros` is not installed on this host. Together they cause 8 collection
errors and 3 failures in `behavior_tree`'s suite — a baseline captured via
`git stash` confirmed the set is byte-identical before and after Task 1.

## Rulings already made (do not re-litigate)

- Work in place in both repos, no worktree — the plan spans two repos, so one worktree cannot hold it.
- The srv-era `BtNode_ScanForWavingPerson` (`TemplateNodes/Vision.py:~1609`) is dead
  code: **not converted, not deleted**, only documented as superseded. Nothing
  instantiates it — `Restaurant/restaurant.py` and `GPSR/egpsr.py` both use
  same-named local variants backed by different message types, and
  `GPSR/supervision/contracts.py:103` names it only as a string in a
  risk-classification table. Passing an action type to `ServiceHandler` now fails
  loudly at construction, which beats silently calling a service that no longer exists.
- `goal.min_waving_persons` stays at its default, matching prior service behavior.
- `possible_objects` may list more objects than the scenario spawns — it is the
  planner's vocabulary, not a spawn manifest. Naming an unspawned object yields a
  legitimate "not found", not a crash.
- Task 1 is not exercised end-to-end by Task 7: the chosen command routes through
  `find_person`, not `describe_person`. Task 1's gate is its own build + subclass assertion.
- Dispatch Tasks 1–5 offline; hold 6–7 for a free GPU.

## Pre-work already done for the next tasks

### Task 2 — standing poses derived and verified (controller ran the script)

All seven arena surfaces resolve at the brief's default standoffs; no widening
needed. `placement.json` uses keys `surface_id` and `center_xyz`; the seventh
surface is `sofa#seat`.

| surface | stand (x, y) | standoff | yaw |
| --- | --- | --- | --- |
| kitchen_table#top | (+3.294, -2.351) | 1.0 | -2.3562 |
| laundry_desk#top | (-3.237, +3.924) | 0.65 | +1.1780 |
| shelf#plate | (-3.227, +0.769) | 0.65 | -2.3562 |
| shelf_02#plate | (+0.962, -0.570) | 0.65 | +3.1416 |
| side_table#top | (-0.595, -4.444) | 0.65 | -1.1780 |
| side_table_02#top | (+1.080, +1.755) | 0.65 | +3.1416 |
| sofa#seat | (-2.184, -4.089) | 0.75 | -1.5708 |

`command_point` spawn `(-2.0, -2.0)` confirmed free at 0.35 m clearance.

### Task 3 — two plan defects found, both already ruled

1. The brief lists `audio_fixtures` under "keep from manipulation", but it is not
   in `manipulation.launch.py`. It lives in
   `ros2_ws/src/tinker_sim_bridge/launch/fixtures.launch.py:21`. **The composite
   inlines that Node** (it is eight lines and already carries the
   `scenario_file` parameter the brief wants) — `fixtures.launch.py` is the
   third merge source. The test's `audio_fixtures` assertion is correct as written.
2. `manipulation.launch.py` carries **uncommitted teammate changes** on this
   branch: a `safety_source_deadline_s` launch arg (default 1.0) feeding
   `required_source_deadline_s`, added because the collision monitor was measured
   gapping 1.2 s during gripper contact bursts with Isaac on the same host. Merge
   from the working-tree version — that is what actually runs — mirror the same
   argument in the composite, and **never stage `manipulation.launch.py`**; the
   composite commit contains only the new launch file and its test.

Remember the merge is explicit and de-duplicating, not an include:
`manipulation.launch.py` and `navigation.launch.py` both unconditionally start
same-named `command_gateway`, `safety_supervisor`, `contract_guard`, and
`robot_state_publisher` with divergent parameters.

### Task 4 — preconditions verified

All four YCB assets exist under
`artifacts/objects/ycb/d2d5ccd2c098b68f39737f8f0490358b7fd6cbfa8080b604851e073fc758acda/`
(`ycb_010_tomato_soup_can`, `ycb_025_mug`, `ycb_011_banana`, `ycb_024_bowl`;
ten objects total). No re-materialising needed.

## Why so little needs building

Every ROS interface GPSR needs is already served by something real:

- `simulation/sensors/hardware-parity.json` publishes exactly the camera topics
  tk26_vision's `camera_server_node` subscribes to, byte-identical. Note
  `hardware-parity` is a **file**, not a `--sensor-profile` value — valid profiles
  are `physics-only | sensor-rich | navigation-parity | manipulation-core`
  (`validation/run_sim.py:502-506`), and only the `sensor-rich` branch loads it
  (`run_sim.py:839-841`).
- `gripper_facade.py:82-85` serves `/xarm_gripper/gripper_action` — the stand-in
  tk25_manipulation's comments reference but never implement.
- `pan_tilt_facade.py:18,21` serves `/pan_tilt_controller/state` and `/cmd`,
  replacing the serial gimbal (`pan_tilt_controller.py:96-102` opens
  `/dev/ttyUSB0` unguarded).
- `audio_fixtures.py` already serves `listen_action`, `doorbell_action`,
  `announce`, `wait_for_start`, `question_answer_service`,
  `get_confirmation_service`, all driven by the scenario's `dialogue` array.

## Resuming

1. Read `.superpowers/sdd/2026-08-20-gpsr-in-simulation/progress.md` end to end.
2. Check whether Task 1's review landed:
   `.superpowers/sdd/2026-08-20-gpsr-in-simulation/task-1-review.md`. If it has
   findings, run the fix loop before Task 2 — both touch `tk25_decision`, so they
   must not run concurrently.
3. Dispatch Task 2 with BASE = tk25_decision `HEAD` at that moment, handing the
   implementer the pre-derived pose table above.
4. Tasks 3, 4, 5 in order (3 before 5 — the runbook documents 3's argument names).
5. When the GPU frees: rebuild `tinker_vision_msgs_26` first, then Tasks 6 and 7.
