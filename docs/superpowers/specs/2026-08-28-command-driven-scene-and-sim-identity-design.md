# Command-driven scene population + sim-mode identity relaxation — design

Date: 2026-08-28. Status: approved in chat (user chose "approve, and also spawn a
person in the command's room").

## Problem

The 5-run hybrid battery s2026-003..007 scored 0/5 with navigation clean in
every run. Every failure was scenario content:

- `person_found(<Name>)` returned INVALID on three runs: sim persons carry no
  name identity, the detector labels them `person`, and the validator's
  labelled-mismatch branch rejects the gate even though the robot found and
  greeted a person in the right room.
- Two runs hunted objects that do not exist in the arena (`pudding_box`; a
  "kitchen item" in the living room). The bench scenario spawns a fixed 4
  objects + 2 persons regardless of the command.

## Decision (user)

1. **Sim-mode identity relaxation** for named persons.
2. **Spawn the scene the command needs**: the objects (and a person) the
   command refers to must actually be in the room/placement it names.

## Scope

Two repos:

- DEC `/home/tinker/tk25_ws/src/tk25_decision` (branch `gpsr-sim-battery`):
  validators, corpus knowledge, tier-2 bench runner.
- SIM `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec`
  (branch `worktree-gpsr-command-variety-spec`): scene planner, spawn
  backends, placement table, scenario generation.

Not in scope: real-robot behaviour (all changes are gated so a non-bench
launch is byte-identical in behaviour), clothing/gesture descriptors,
manipulation.

## Part 1 — Sim-mode identity relaxation (DEC)

**Gate:** environment variable `GPSR_SIM_IDENTITY_RELAXED=1`. Read once per
verification call in `validators.py` (`os.environ`, so tests can monkeypatch).
`tier2.py` sets it in the orchestrator subprocess env (`bench_env` result)
for every tier-2 run. Nothing else sets it; the real-robot launch never sees
it.

**Behaviour change** — in `_verify`, `person_found(<arg>)` branch only, and
only where today the code returns `INVALID` because labels are present but
`<arg>` is not among them:

- if the flag is on, AND `<arg>` is a *person name* (not a descriptor: not in
  `{"waving_person","waving_persons"}` and does not contain a space or the
  words `person`/`persons`), AND at least one label is a person-class label
  (`person`, `persons`, `people`, `human`) → `VALID`, reason
  `"sim mode: person detected; name identity is not modelled in sim"`,
  confidence 0.6.
- every other path is unchanged. `object_seen` and `counted` are not relaxed:
  objects will be genuinely present under Part 2, so those gates stay honest.

**Tests** (`test/test_sim_identity_relaxation.py`): flag off → INVALID (the
existing behaviour); flag on + `person` label → VALID with the sim reason;
flag on + labels `{"chair"}` → INVALID; flag on + descriptor arg
`waving_person` → unchanged path; flag on + `object_seen("mug")` with labels
`{"bowl"}` → INVALID (no leak into objects).

## Part 2 — Command-driven scene population

### 2.1 Scene planner (SIM, pure Python, no ROS): `tools/gpsr_scene.py`

`plan_scene(command_text, knowledge, placements, *, seed) -> ScenePlan`

Inputs:

- `command_text`: the corpus entry's `text`.
- `knowledge`: DEC's `constants.rcw2026.json` (loaded by the tool): rooms
  (`search_spots` keys), placement spots (`possible_poses` keys minus
  `command_point`), object names (`possible_objects` keys), plus the
  category map below.
- `placements`: `simulation/scenarios/rcw2026-placements.json` (2.2).
- `seed`: corpus seed, for deterministic category sampling.

Rules (first match on each dimension, tokens matched as whole words, case
and underscore/space insensitive, singular/plural tolerant):

- **Objects**
  - explicit object name in text → 1 instance of that object.
  - category word (`food`, `drink`, `kitchen item`, plurals) → 3 distinct
    members of that category (deterministic sample by seed).
  - counting template (text contains `how many` or `count`) → instances of
    the target: 3 for an explicit object, 3 category members for a category.
  - no object/category in text → no objects.
- **Where**
  - explicit placement spot in text (e.g. `kitchen_table`, `laundry_desk`,
    `shelf_02`, `side_table`) → that spot.
  - else explicit room in text → the room's first `search_spots` entry.
  - else (object named, nowhere named) → `default_locations[object]` from
    constants if present, else the first spot of `kitchen`.
- **Person**
  - text mentions a person name (`Alex, Sarah, John, Emma, Liam, Olivia`),
    or `person`/`someone`, or a greet/meet/guide/follow verb → 1 person
    actor at the **person pose** of the resolved room (2.2). Room resolution
    as above; a person with no room → the room of the first spot named, else
    `living_room`.
  - if the resolved room already has a person in the base scenario
    (`kitchen`, `living_room` in `gpsr-rcw2026-bench.json`) the planner
    still lists it, and the backend dedupes by `(asset, room)`.

Category map (added to the SIM planner as data; DEC's corpus already uses the
same three category words):

| category | members |
|---|---|
| food | banana, spam, pudding_box, sugar_box, cheez_it, soup |
| drink | mug, soup |
| kitchen item | bowl, mug, mustard, bleach |

Output `ScenePlan`: list of `SpawnItem(id, kind, name, asset_uri, room, spot,
xyz, quaternion_xyzw)`, with ids `cmd_<name>_<n>` for objects and
`cmd_person_<room>` for the actor, plus `notes` (which rule fired). The
planner never raises on an unparseable command; it returns an empty plan with
a note.

Asset resolution: object `name` → YCB directory via a fixed name map
(`soup→ycb_010_tomato_soup_can`, `mug→ycb_025_mug`, `banana→ycb_011_banana`,
`mustard→ycb_006_mustard_bottle`, `sugar_box→ycb_002_sugar_box`,
`spam→ycb_005_spam`, `cheez_it→ycb_001_cheez-it`,
`pudding_box→ycb_008_pudding_box`, `bowl→ycb_024_bowl`,
`bleach→ycb_021_bleach_cleanser`) looked up in
`artifacts/asset-manifest.json["generated_object_usds"]`; person →
the `person_standing/person.usd` entry the bench scenario already uses.

### 2.2 Placement table (SIM data): `simulation/scenarios/rcw2026-placements.json`

```json
{"schema_version": 1,
 "spots": {"kitchen_table": {"surface_xyz": [2.5, -3.0, 0.734], "grid_dx": 0.18, "grid_dy": 0.15},
           "side_table_02": {"surface_xyz": [0.43, 1.755, 0.612], ...},
           "shelf_02": {"surface_xyz": [0.312, -0.57, 1.07], ...},
           "shelf": ..., "laundry_desk": ..., "side_table": ..., "sofa": ...},
 "persons": {"kitchen": {"xyz": [1.8, -3.85, 0.0], "quaternion_xyzw": [0,0,-0.381912,0.924199]},
             "living_room": {"xyz": [-4.684, -4.089, 0.0], ...},
             "bedroom": ..., "laundry_room": ...}}
```

The three known surfaces come from the bench scenario's measured object poses.
The other four spot surfaces and two person poses are measured from the arena
USD (furniture top z + a point 0.6 m in front of the DEC spot pose, inside the
room) during Task "placement table"; the implementer records the method in the
file's `_comment`. Multiple instances at one spot are laid out on a grid
(`surface_xyz + (i % 3) * grid_dx, (i // 3) * grid_dy`) so they never overlap.

### 2.3 Spawn backends (SIM)

`tools/gpsr_spawn.py` — CLI, run inside the bridge's ROS env:

- `plan --command "<text>" --seed N --out <run_dir>/scene-plan.json` (no ROS;
  writes the ScenePlan).
- `apply --plan <scene-plan.json> --manifest <run_dir>/spawned.json`:
  calls `/spawn_entity` (`simulation_interfaces/srv/SpawnEntity`, same
  request shape as `scenario_runner.py:273-291`: `name="/World/Scenario/<id>"`,
  `entity_namespace="Scenario"`, `allow_renaming=False`, absolute USD uri,
  `frame_id="world"`), skipping items whose `(asset, spot-or-room)` already
  exists in the base scenario or in a previous manifest; writes
  `spawned.json` (entity names + poses + skipped items + service results).
- `clear --manifest <spawned.json>`: calls `/delete_entity` for every entity
  in the manifest; tolerates NOT_FOUND; rewrites the manifest with results.
- `emit-scenario --plans <plan.json>... --base gpsr-rcw2026-bench.json --out
  <scenario>.json`: the **guaranteed backend** — merges all plans' items into
  a copy of the base scenario (dedupe by `(asset, spot-or-room)`; when two
  plans want the same object at the same spot the larger count wins) so the
  stack can be launched with `--scenario <generated>` for the whole battery.

Exit codes: 0 ok, 2 service unavailable/timeout (bench treats as ERROR for
that run, same as a reset failure), 3 bad plan.

### 2.4 Live spike (gates the default backend)

Before the bench wires `apply`, one live check with the stack up:
spawn a `pudding_box` on `kitchen_table` while PLAYING, confirm (a) it
appears in an arena-camera frame at the right place, (b) `/delete_entity`
removes it, (c) `/clock` did not re-zero and Nav2 is still healthy
(`ros2 topic echo /clock` monotonic; a 3-minute nav-only goto still succeeds).
Outcome recorded in `docs/developer-log.md`:

- all three pass → tier-2 default `--spawn-cmd` uses `apply`/`clear` per run.
- spawn works but delete does not → per run `apply` with dedupe, no `clear`
  (objects accumulate within a battery; acceptable, noted in run.json).
- spawn while PLAYING fails or breaks the clock → per-battery `emit-scenario`
  + stack launch with the generated scenario; `--spawn-cmd` left unset.

### 2.5 Bench wiring (DEC `tier2.py` / `gpsr_bench.py`)

- New args `--spawn-cmd` / `--clear-cmd` (shell-split, `{run_dir}`,
  `{command}`, `{seed}`, `{plan}`, `{manifest}` substituted; both optional).
- Per run, after a successful reset and before the recorder starts:
  `spawn_cmd` runs; non-zero exit → run scored `ERROR` with the tool's stderr
  tail as detail (mirrors `_reset`). After the run (in the `finally`, after
  the recorder stops): `clear_cmd` runs; its failure is appended to
  `detail` but does not change the verdict.
- `run.json` gains `scene: {"plan": "scene-plan.json", "spawned":
  "spawned.json"}` (relative paths) when the files exist.
- `GPSR_SIM_IDENTITY_RELAXED=1` is added to the orchestrator env in
  `run_tier2` (Part 1).
- The battery script (`run-battery-t2.sh`, job tmp — not in a repo) gets the
  two flags once the spike picks the backend.

### 2.6 Contact sheet (SIM `tools/contact_sheet.py`)

The judge sheet's header block lists the scene that was placed
(`scene-plan.json` items: `object@spot ×n`, `person@room`) so a reviewer can
see what the arena contained. Missing file → no line; never fails the sheet.

## Testing

- SIM: `tests/test_gpsr_scene.py` — table of commands from the actual corpus
  (`s2026-001..007` texts) → expected items/spots/counts; category sampling
  deterministic by seed; unknown text → empty plan with note; grid layout
  never duplicates xyz; `emit-scenario` dedupe/merge; placement table
  validates (every DEC spot has a surface, every room has a person pose).
  `tests/test_gpsr_spawn_cli.py` — `plan`/`emit-scenario` subcommands
  without ROS (the `apply`/`clear` service calls are behind an injectable
  client and exercised with a fake).
- DEC: `test/test_sim_identity_relaxation.py` (Part 1);
  `test/test_tier2_spawn_hooks.py` — `run_tier2` with fake launcher/reset/
  spawn/clear commands: spawn failure → ERROR; clear failure → detail only;
  env carries the relaxation flag; `run.json.scene` written.
- Live: the 2.4 spike, then re-run `s2026-003`, `004`, `005` (archive the old
  run dirs first — the bench reuses dirs in place). Acceptance: 003/004 no
  longer fail on absent objects; 005 passes the `person_found` gate.

## Constraints carried from the session

Never read/print `.env`; ROS_DOMAIN_ID=42; PGID-only teardown, nav before
sim, never SIGKILL Nav2; colcon only `--packages-select`; no pushes to main;
tk26_vision untouched; Sonnet (not Opus) subagents; message the training
session before any stack bring-up (GPU1 is theirs when the stack is down).
