# GPSR command-variety testing — design (2026-08-23)

Status: approved 2026-08-23. Implementation plan follows in
`docs/superpowers/plans/`.

Note on repositories: the orchestrator lives in
`~/tk25_ws/src/tk25_decision` (branch `gpsr-two-layer-orchestrator`); there
is no `tk26_decision`.

## Goal

Move GPSR-in-sim from "one hand-written command passes" (run18, 2026-08-22:
*go to the kitchen table and find a person*) to "a seeded, repeatable batch of
official-grammar commands runs through the orchestrator and the sim, and every
task is scored automatically" — so regressions in either the orchestrator
(tk25_decision, branch `gpsr-two-layer-orchestrator`) or the simulation
(tinker-sim 6.0.1) show up as a changed pass-rate, not as a surprise on the
field.

## What already exists (do not rebuild)

| Layer | Asset | Where |
|---|---|---|
| Command source | Official RoboCup `CommandGenerator`, **already vendored**, 22 top-level templates + 13 follow-ups, seedable | `GPSR/_official_cmd_gen.py` |
| Tier-0 planner batch | `cmd_understanding_test.py --seed --n --commands-file` → markdown report; `gpsr_ten_cmd/run_ten_cmd.py --count --seed --save` for the two-layer planner + BB write-map | `GPSR/` |
| Tier-1 full mock | `BT_MOCK_MODE` + `mock_config.json`, `GPSR_OFFLINE_PLANNER`, 18 `test_gpsr_*.py` | `behavior_tree/` |
| Command injection | `BT_GPSR_CMD="a\|b\|c"` (pipe-separated list runs N tasks in ONE process, all collected at command point then executed in order) | `gpsr_orchestrator.py:104` |
| Per-task outcome stream | `gpsr_runs/debug/<trajectory_id>/events.jsonl` (`task.command_received`, `plan.committed`, `task.finished` w/ terminal status, `run.finished`) | `GPSR/telemetry.py` |
| Sim command intake | `audio_fixtures` answers `listen_action` / `wait_for_start` / `get_confirmation_service` / `question_answer_service` from the scenario `dialogue` queue | `tinker_sim_bridge/audio_fixtures.py` |
| Speech oracle | every `announce` is echoed on `/sim/fixtures/audio/events` | same |
| World oracle | `truth_state`: robot base/TCP pose, object poses, contacts, `safety_stop`; `PostconditionEvaluator` with `equals / contains / set_equals / distance_less_than_or_equal` | `tinker_sim_core/evaluator.py`, `tinker_sim_isaac/backend.py:1797` |
| Arena vocabulary | `constants.rcw2026.json`: 8 poses, 10 objects (all 10 have YCB USDs), 4 rooms | `GPSR/constants.rcw2026.json` |
| Bring-up | 6-stage runbook, domain 42, `scripts/kill-domain-nodes` | `docs/gpsr-sim-runbook.md` |

Note: the GitHub repo linked in the request is the *archived Python-2
interactive* generator (no seed, no batch). The vendored module is the newer
template grammar; use it.

## The core idea: a four-tier ladder, one command corpus

Generate **one frozen corpus** of commands from the vendored generator with
the rcw2026 vocabulary and a fixed seed, and push it through four tiers of
increasing realism. A command is only "sim-robust" when it passes all four.
Each tier is cheap enough to run far more often than the one above it.

```
corpus.jsonl  (seed, template id, text, expected plan skeleton, expected world effect)
   │
   ├─ T0  planner only         (LLM, no ROS)        seconds/cmd     run on every planner change
   ├─ T1  full-mock executor   (BT, fake ROS)       seconds/cmd     run in CI
   ├─ T2  sim, mocked perception/arm  (Isaac+Nav2 real, vision+manip mocked)  ~1-3 min/cmd
   └─ T3  sim, everything real (run18 configuration)                          ~5-10 min/cmd
```

### Corpus (`gpsr_corpus/`)

- `gen_corpus.py --seed S --n N --knowledge constants.rcw2026.json` wraps the
  vendored generator with a `StubKnowledge` built from the arena constants
  (same trick `cmd_understanding_test.py` already uses) and writes
  `corpus-<seed>.jsonl`, one line per command:
  `{id, seed, template, followup, text, slots:{loc,obj,room,name,...}}`.
- Template id is recorded so pass/fail can be broken down **per template**
  (the metric that tells you *which kind* of command is weak).
- **Stratified, not random**: the generator is sampled until every supported
  template appears ≥ k times (k=3 for T0/T1, k=1 for T2/T3). Random sampling
  over-represents `goToLoc` and starves `countObjOnPlcmt`.
- Hand-curated `corpus-edge.jsonl` alongside: adversarial phrasings
  (synonyms the grammar never emits, object not at its default location,
  unknown object, two-room guide), the run18 command as a canary.

### Sim-feasibility classification of the 35 templates

Not every template can be *fulfilled* in the sim today. Tag each with a
feasibility class so a failure is reported against the right layer:

| Class | Templates | Why |
|---|---|---|
| **A — fully checkable now** | goToLoc(+atLoc), findPrsInRoom, findPrs, meetPrsAtBeac/meetName (person exists), countPrsInRoom, findObjInRoom/findObj, countObjOnPlcmt, tellObjPropOnPlcmt/tellCatPropOnPlcmt, talkInfo*, greet* | nav + detection + speech; all observable via truth pose + audio events |
| **B — executable, weak oracle** | takeObjFromPlcmt, takeObj, bringMeObjFromPlcmt, deliverObjToMe, deliverObjToPrsInRoom, deliverObjToNameAtBeac | needs real grasp; object truth pose exists but YCB objects currently "never resolve to PhysX rigid-body views" (open defect) — so the oracle `object.pose near robot tcp / near target placement` is blocked until that is fixed |
| **C — unsupported by sim content or stubbed in the executor** | followNameFromBeacToRoom, followPrs*, guide*, tellPrsInfo* (age/gender/pose of a *specific* person), countClothPrsInRoom, guideClothPrs*, putObjInTrash, placeObjOnPlcmt | scenario `events`/actor motion paths are unimplemented; no clothing-varied people; no trash bin model; executor `place` is a goto+gripper-toggle stub and `open` is a referee hand-off by design |

Class C commands still run at T0/T1 (planner + BT shape), and at T2/T3 only
to prove the orchestrator **degrades gracefully** (announces it cannot,
moves to the next task, no crash/stall). That is itself a robustness
property worth testing.

### Oracles per template (what "pass" means above T1)

Expressed as scenario `postconditions` + audio-event expectations, generated
from the corpus slot values:

- `goto X` → `robot.base_pose.xyz` within 0.5 m of `possible_poses[X]`
  (`distance_less_than_or_equal`).
- `find person / count people` → announce event matching `/found|there (is|are) \d/`
  and the scenario's actor count.
- `count objects on P` → announce contains the true count of scenario objects
  whose xyz lies inside P's footprint region (needs `regions[]` populated
  for the 7 furniture items — small scenario-data task).
- `tell me the biggest/smallest object on P` → announce names the object the
  truth state says is largest by asset bounding box.
- Class B (when rigid bodies work): `objects[obj].pose` within 0.3 m of TCP
  after grasp, or within the placement region after place/deliver.
- Every command: `robot.safety_stop == false`, no Nav2 deactivation in the
  bridge log, orchestrator emitted a terminal verdict for every task slot.

Only the sim-side evaluator sees `truth_state` (it is token-gated); the
orchestrator never does — keeping the test honest.

### Runner (`gpsr_bench.py`)

One script, tier as a flag, corpus as input, JSON report as output:

- `--tier 0|1`: no ROS; loops commands directly (reuse `cmd_understanding_test`
  / full-mock harness, just re-keyed to corpus ids).
- `--tier 2|3`: **one sim bring-up per batch, several commands per
  orchestrator process**. Uses `BT_GPSR_CMD="c1|c2|…|ck"` with k≈3–4 so a
  stall in one task cannot burn the whole batch; between orchestrator
  processes the runner calls `/reset_simulation` (robot returns to `command_point`,
  scenario re-spawned) (no Isaac
  relaunch, no Nav2 relaunch — per the teardown rules, Nav2 must never be
  SIGKILLed and the orchestrator is always the thing that restarts).
  Per-task wall-clock timeout (default 8 min at RTF≈0.1) → task scored
  `TIMEOUT`, orchestrator SIGINT'd, next group launched.
- Per task it records: corpus id, template, planner plan (from
  `events.jsonl`), executor verdict, postcondition evaluation, audio events,
  wall time, RTF, Nav2 deactivation count, and the log paths.
- Output `reports/gpsr-bench-<date>/report.json` + `SUMMARY.md` with a
  **template × tier pass matrix** — the single artifact to look at.

### What changes where

tk25_decision (small, mostly additive):
1. `GPSR/gpsr_runs/run_battery_commands.py` — corpus generator + template-stratified sampling + edge corpus. This is the module `test/test_gpsr_battery_contract.py` already imports (`COMMANDS`/`EXPECT`) but which does not exist in the repo, so landing it here also un-breaks that test.
2. `GPSR/gpsr_bench.py` — tiers 0/1 runner, and the tier 2/3 client half (launch orchestrator with env, parse `events.jsonl`, SIGINT on `run.finished` — the orchestrator idles forever after its last task, `gpsr_orchestrator.py:150`).
3. Nothing in telemetry: `task.finished` already carries `status ∈ {succeeded, failed, incomplete}` + reason; `step.finished`, `plan.committed`, `run.finished` exist. The bench only parses them.
4. **One orchestrator fix found by scouting:** `GPSR_OFFLINE_PLANNER` is honoured only by the legacy single-layer `BtNode_PlanActions`; the production `GPSRPlanner` (`planner.py:597`) consults `BT_MOCK_MODE` directly, so T1 (mock execution, real LLM) is currently impossible via the documented switch. Small fix + test.
5. T1 needs a `mock_config.bench.json` where the mocked node classes resolve `IMMEDIATE` rather than `KEYPRESS` (shipped `mock_config.json` waits for a keypress per goto/scan/grasp).

tinker-sim (small, all additive):
6. `simulation/scenarios/gpsr-rcw2026-bench.json` — same world plus `regions[]` for the 7 furniture footprints and one extra actor (so counts >1 are testable); `dialogue` left empty (commands come via `BT_GPSR_CMD`).
7. A `validation/gpsr_bench_oracle.py` that turns a corpus entry into postconditions and scores one task from a truth snapshot + `/sim/fixtures/audio/events` capture.
8. No new reset endpoint: the bridge already exposes `simulation_interfaces` `/reset_simulation`, `/spawn_entity`, `/set_simulation_state` (`scenario_runner.py:167-172`). The bench calls `/reset_simulation` between orchestrator processes. **Phase-2 check:** a reset re-zeroes `/clock`, and the base facade is known to stall until sim time passes the pre-reset stamp (developer log) — measure this once; if it bites, either relaunch only the facade or seed AMCL/reset via `/set_simulation_state` + a robot `SetEntityState` instead.
9. `scripts/gpsr-bench` wrapper that does the runbook stages 1–5 once, then drives the batch.

### Phasing

- **Phase 1 (T0+T1, no hardware, ~1 day):** corpus + runner + per-template
  matrix. Immediately tells you which templates the planner/BT mishandle.
  Also the regression gate for planner prompt changes.
- **Phase 2 (T2, sim with mocked vision/manip):** isolates nav + orchestration
  + sim physics; targets class A templates. Fixes here are sim/nav bugs.
- **Phase 3 (T3, all real):** class A with real detection (counts, person
  finding); then class B once YCB rigid bodies resolve.
- **Phase 4:** robustness perturbations on T2/T3 — randomise `initial_pose`,
  actor position, object placement within its furniture region, seeds;
  multi-command batches (k=3) to exercise the collect-all-then-execute flow.

### Success criteria

- Phase 1: ≥95 % of class A+B corpus commands produce a validator-clean plan; every class C command produces a plan or an explicit "unsupported" announce (no empty plan).
- Phase 2/3: class A pass-rate reported per template; target ≥80 % on goToLoc/findPrs/count templates, zero stalls (every task reaches a terminal verdict).
- No run ends with a leaked GPU process or a Nav2 deactivation.

## Decisions I made (override if wrong)

1. Use the vendored template generator, not the archived Py2 repo.
2. One corpus, four tiers — rather than separate ad-hoc command lists per tier.
3. Stratify by template; report per template.
4. Class C templates are *tested for graceful degradation*, not skipped.
5. Sim batches reset in-process via `/reset_simulation`, never relaunch Isaac/Nav2 per command.
6. LLM planning stays live at every tier (offline planner only for CI); costs a few cents per command.

## Resolved defaults

- T3 corpus size: ~12 commands per batch (one evening at current RTF); T2 ~24.
- Phase 3 assumes the pan/tilt `/joint_states` fix is in tk25_ws; until then it runs with the `TINKER_SIM_HEAD_CAMERA_AIM=level-forward` override only.
- Placement: corpus generator + tier 0/1 runner in tk25_decision (next to the vendored generator); tier 2/3 driver, oracle and scenario in tinker-sim.
