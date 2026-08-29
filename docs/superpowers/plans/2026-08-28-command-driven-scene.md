# Command-driven scene population + sim-mode identity relaxation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the GPSR sim battery's scenes match the command under test (spawn the objects/person a command actually names) and relax sim-only person-name identity checking, so the 5-run hybrid battery's content-gap failures (absent objects, name-identity mismatch on a correctly-found person) go away without touching real-robot behaviour.

**Architecture:** Two independent repos, wired by one shared contract (a `ScenePlan` JSON: a list of objects/person with id/asset/room/spot/pose). DEC (`tk25_decision`) gains an env-gated relaxation in the postcondition verifier and two new tier-2 bench hooks (`--spawn-cmd`/`--clear-cmd`) that shell out to SIM's new CLI per run. SIM (`tinker-sim`) gains a pure-Python command→scene planner, a measured placement table, a small `simulation_interfaces` CLI (`plan`/`apply`/`clear`/`emit-scenario`) behind an injectable `ServiceClient` so it's testable without ROS, and a one-line scene summary on the judge-sheet header. A live spike (not run by an implementer subagent) decides whether the bench uses live `apply`/`clear` or a pre-generated merged scenario file.

**Tech Stack:** Python 3 stdlib (SIM tools are pure Python, no ROS at import time), `pytest`, `simulation_interfaces` ROS 2 services (`SpawnEntity`, `DeleteEntity`) via `rclpy` (SIM `apply`/`clear` only, deferred import), `pxr` (USD Python bindings, available via `uv run --frozen --no-sync python` in the SIM repo's own venv — verified working during planning), Pillow (`tools/contact_sheet.py`).

**Spec:** `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/docs/superpowers/specs/2026-08-28-command-driven-scene-and-sim-identity-design.md`

## Global Constraints

- Never read or print `/home/tinker/tk25_ws/.env`.
- `ROS_DOMAIN_ID=42` for any ROS-touching command.
- `colcon` only with `--packages-select` (never a bare `colcon build`).
- No `git push` to any remote.
- Do not touch `tk26_vision`.
- No subagents dispatched from within an implementer's own session (this plan's tasks are implemented directly, not via further sub-dispatch).
- Tests must be run and pass before each commit.
- DEC commits go on branch `gpsr-sim-battery` (already checked out at `/home/tinker/tk25_ws/src/tk25_decision`); SIM commits go on the current worktree branch `worktree-gpsr-command-variety-spec` at `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec`.
- Every commit message ends with these two trailer lines:
  ```
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01DxMkrcYzHbnJNjTsAQSe46
  ```
- Scope: real-robot behaviour must be byte-identical outside a bench launch (the identity relaxation is env-gated and off by default; scene spawning is opt-in via bench flags). Clothing/gesture descriptors and manipulation are out of scope.

---

## Repo map for this plan

- **DEC** = `/home/tinker/tk25_ws/src/tk25_decision` (branch `gpsr-sim-battery`). Tests run from `$DEC/src/behavior_tree` via:
  ```bash
  cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
  PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest test/<file> -q
  ```
  (equivalent to what `/home/tinker/.claude/jobs/1462b451/tmp/run-dec-tests.sh` does for its fixed file list — append new test files to that script's list, or run the command above directly.)
- **SIM** = `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec` (branch `worktree-gpsr-command-variety-spec`). Tests run from the repo root:
  ```bash
  cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
  python3 -m pytest tests/<file> -q
  ```

---

### Task 1: DEC sim-mode identity relaxation

**Files:**
- Modify: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/validators.py`
- Test: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_sim_identity_relaxation.py`

**Interfaces:**
- Consumes: nothing from other tasks.
- Produces: the environment variable contract `GPSR_SIM_IDENTITY_RELAXED=1` that Task 6 sets in `bench_env()`'s returned dict (`tier1.py`). No new public functions are needed by later tasks; the behaviour is entirely internal to `_verify`.

- [ ] **Step 1: Read the current `_verify` person_found branch to confirm line context**

```bash
grep -n "if labels:" /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/validators.py
```
Expected: one match inside `_verify`, around line 300, immediately followed by the `_normalize(fact.args[0]) in labels` check and the `Verdict.INVALID` return for "labels do not match requested target".

- [ ] **Step 2: Write the failing tests**

Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_sim_identity_relaxation.py`:

```python
"""Sim-mode identity relaxation for person_found() (GPSR_SIM_IDENTITY_RELAXED=1).

Sim persons carry no name identity -- the detector always labels them
"person" -- so a command like greetNameInRm's person_found(<Name>) gate
was rejecting a correct sim run (battery run family s2026-005/006/007,
2026-08-28) purely because the name wasn't in the detection labels. This
flag relaxes ONLY that specific labelled-mismatch path, and only for a
genuine person NAME argument (not a waving/gesture descriptor), and only
when at least one detection label is itself a person-class label. Every
other validator path -- object_seen, counted, and person_found's other
branches -- is unchanged; the flag is off by default so a real-robot
launch never sees this behaviour.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""

from __future__ import annotations

from behavior_tree.GPSR.validators import Verdict, VerificationContext, check_all


def _check(text, *, evidence=None, env=None, monkeypatch):
    if env is not None:
        for key, value in env.items():
            monkeypatch.setenv(key, value)
    results, _facts = check_all([text], evidence or {}, VerificationContext(phase="postcondition"))
    assert len(results) == 1
    return results[0]


def test_flag_off_labelled_mismatch_is_still_invalid(monkeypatch):
    monkeypatch.delenv("GPSR_SIM_IDENTITY_RELAXED", raising=False)
    result = _check(
        "person_found(sarah)",
        evidence={"person_detection": {"objects": [{"label": "person"}]}},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.INVALID


def test_flag_on_person_label_is_valid_with_sim_reason(monkeypatch):
    result = _check(
        "person_found(sarah)",
        evidence={"person_detection": {"objects": [{"label": "person"}]}},
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.VALID
    assert result.confidence == 0.6
    assert result.evidence == "sim mode: person detected; name identity is not modelled in sim"


def test_flag_on_non_person_labels_stay_invalid(monkeypatch):
    result = _check(
        "person_found(sarah)",
        evidence={"person_detection": {"objects": [{"label": "chair"}]}},
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.INVALID


def test_flag_on_descriptor_argument_is_unaffected(monkeypatch):
    # "waving_person" is a specialist descriptor, not a name -- the
    # relaxation must not touch this path (it already has its own
    # provenance-gated VALID/UNKNOWN branch, tested in
    # test_gpsr_fact_validators.py).
    result = _check(
        "person_found(waving_person)",
        evidence={"person_detection": {"objects": [{"label": "person"}]}},
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.INVALID


def test_flag_on_does_not_leak_into_object_seen(monkeypatch):
    result = _check(
        "object_seen(mug)",
        evidence={"object_detection": {"objects": [{"label": "bowl"}]}},
        env={"GPSR_SIM_IDENTITY_RELAXED": "1"},
        monkeypatch=monkeypatch,
    )
    assert result.verdict is Verdict.INVALID
```

- [ ] **Step 3: Run the tests to verify they fail**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest test/test_sim_identity_relaxation.py -q
```
Expected: `test_flag_on_person_label_is_valid_with_sim_reason` FAILS (verdict is `Verdict.INVALID`, not `VALID`); the other four pass already (they describe today's behaviour).

- [ ] **Step 4: Add `import os` to validators.py**

```bash
grep -n "^import re$" /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/validators.py
```

Edit `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/validators.py`:

```python
# before
import re
from dataclasses import dataclass
```

```python
# after
import os
import re
from dataclasses import dataclass
```

- [ ] **Step 5: Add the relaxation constants and helpers**

Insert immediately before `def _action_verdict(fact: Fact, context: VerificationContext)`:

```python
# --- sim-mode identity relaxation (GPSR_SIM_IDENTITY_RELAXED=1) ----------
#
# Sim persons carry no name identity: the detector always labels them
# "person" regardless of who the scenario says is standing there. Without
# this flag, person_found(<Name>) rejects a correct sim run purely because
# the requested name is never among the detection labels. See _verify's
# person_found branch below for where this is consulted -- it only
# replaces the final INVALID return of the labelled-mismatch path, never
# the earlier established-fact / waving-specialist / unset-evidence paths.
_SIM_PERSON_CLASS_LABELS = {"person", "persons", "people", "human"}
_SIM_PERSON_DESCRIPTORS = {"waving_person", "waving_persons"}


def _sim_identity_relaxed_enabled() -> bool:
    # Read fresh every call (not cached at import time) so tests can
    # monkeypatch os.environ per-test without reloading the module.
    return os.environ.get("GPSR_SIM_IDENTITY_RELAXED") == "1"


def _is_person_name_arg(arg: str) -> bool:
    """True when a person_found() argument names a person rather than a
    descriptor. `arg` is already normalized (lowercase, whitespace -> "_")
    by the time _verify sees fact.args, so "waving person" and
    "waving_person" are indistinguishable here -- both are excluded.
    """
    if arg in _SIM_PERSON_DESCRIPTORS:
        return False
    if "_" in arg:
        return False
    if "person" in arg or "persons" in arg:
        return False
    return True


```

- [ ] **Step 6: Wire the relaxation into `_verify`'s `if labels:` branch**

```python
# before
        if labels:
            if _normalize(fact.args[0]) in labels:
                return _result(Verdict.VALID, f"{detection_key} label matches requested target")
            return _result(Verdict.INVALID, f"{detection_key} labels do not match requested target")
```

```python
# after
        if labels:
            if _normalize(fact.args[0]) in labels:
                return _result(Verdict.VALID, f"{detection_key} label matches requested target")
            if (
                fact.predicate == "person_found"
                and _sim_identity_relaxed_enabled()
                and _is_person_name_arg(fact.args[0])
                and (labels & _SIM_PERSON_CLASS_LABELS)
            ):
                return _result(
                    Verdict.VALID,
                    "sim mode: person detected; name identity is not modelled in sim",
                    0.6,
                )
            return _result(Verdict.INVALID, f"{detection_key} labels do not match requested target")
```

- [ ] **Step 7: Run the tests to verify they pass**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest test/test_sim_identity_relaxation.py -q
```
Expected: 5 passed.

- [ ] **Step 8: Run the full existing GPSR fact-validator suite to confirm no regression**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest test/test_gpsr_fact_validators.py test/test_sim_identity_relaxation.py -q
```
Expected: all pass.

- [ ] **Step 9: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/GPSR/validators.py src/behavior_tree/test/test_sim_identity_relaxation.py
git commit -m "$(cat <<'EOF'
feat(gpsr): relax person_found() name identity under GPSR_SIM_IDENTITY_RELAXED=1

Sim persons carry no name identity -- the detector labels them "person"
-- so person_found(<Name>) rejected correct sim runs (battery family
s2026-005/006/007, 2026-08-28) that actually found and greeted a person.
Env-gated and off by default; real-robot launches are unaffected.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01DxMkrcYzHbnJNjTsAQSe46
EOF
)"
```

---

### Task 2: SIM placement table

**Files:**
- Create: `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/simulation/scenarios/rcw2026-placements.json`
- Test: `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/tests/test_gpsr_placements.py`

**Interfaces:**
- Consumes: nothing from other tasks. Reads DEC's `constants.rcw2026.json` (absolute path, read-only, not `.env`) and this repo's own `simulation/scenarios/gpsr-rcw2026-bench.json`.
- Produces: `simulation/scenarios/rcw2026-placements.json` with top-level keys `schema_version` (int), `_comment` (str), `spots` (dict: spot name -> `{"surface_xyz": [x,y,z], "grid_dx": float, "grid_dy": float}`), `persons` (dict: room name -> `{"xyz": [x,y,z], "quaternion_xyzw": [x,y,z,w]}`). Task 3's planner reads this file's `spots`/`persons` dicts directly (no loader function needed here -- plain `json.loads`).

**Measuring procedure actually used to produce the exact numbers below (documented here for the record; already executed once during planning — the implementer's job is to write this file with these values, verify with the pxr command below if it wants to re-confirm, and write the test):**

The three "known" surfaces (`kitchen_table`, `side_table_02`, `shelf_02`) and two "known" person poses (`kitchen`, `living_room`) are exactly the poses already spawned in `simulation/scenarios/gpsr-rcw2026-bench.json` (objects `soup`/`banana`/`bowl`, actors `kitchen_person`/`livingroom_person`).

The other four spot surfaces (`side_table`, `laundry_desk`, `shelf`, `sofa`) were measured from the built rcw2026 arena USD using `pxr` (USD Python bindings), which **is available** in this repo's own `uv`-managed venv — confirmed by running, from the SIM repo root:

```bash
uv run --frozen --no-sync python -c "import pxr; print('OK', pxr.__file__)"
```
which prints `OK <repo>/.venv/lib/python3.12/site-packages/pxr/__init__.py` (this is the same `uv run --frozen --no-sync python ...` invocation `tools/tinker_sim_deploy/cli.py`'s `_launch_command` uses to run `validation/run_sim.py`). The arena USD path came from `artifacts/asset-manifest.json["generated_arena_usds"][0]["path"]`; when that exact content-hashed file is not present on disk locally (it may not be, if the artifact was regenerated since this worktree's asset cache was populated), use whichever `artifacts/arena/rcw2026/*/arena.usd` file exists instead — two independently-built copies were compared during planning and produced byte-identical furniture placements, so any build works. Furniture prim origins and world bounding boxes were read with:

```bash
uv run --frozen --no-sync python -c "
from pxr import Usd, UsdGeom
import glob
path = sorted(glob.glob('artifacts/arena/rcw2026/*/arena.usd'))[0]
stage = Usd.Stage.Open(path)
names = ['rcw26_sofa','rcw26_side_table','rcw26_side_table_02','rcw26_kitchen_table','rcw26_laundry_desk','rcw26_shelf','rcw26_shelf_02']
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_], useExtentsHint=True)
for n in names:
    prim = stage.GetPrimAtPath(f'/World/Arena/Furniture/{n}')
    xform = UsdGeom.Xformable(prim)
    t = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    bbox = cache.ComputeWorldBound(prim)
    print(n, 'origin', tuple(t), 'bbox_max_z', bbox.ComputeAlignedRange().GetMax()[2])
"
```

This produced (verbatim, 2026-08-29):

```
rcw26_sofa          origin (-2.184, -4.839, 0.0)  bbox_max_z 0.6529366685648265
rcw26_side_table     origin (-0.346, -5.045, 0.0)  bbox_max_z 0.6024761019940654
rcw26_side_table_02  origin (0.43,   1.755,  0.0)  bbox_max_z 0.6024761019940654   <- matches known 0.612 (same asset)
rcw26_kitchen_table  origin (2.587, -3.058,  0.0)  bbox_max_z 0.7238095353863205
rcw26_laundry_desk   origin (-2.988, 4.525,  0.0)  bbox_max_z 0.7336784859232828
rcw26_shelf          origin (-3.687, 0.309,  0.0)  bbox_max_z 2.0196457125231140  <- whole-unit top, not a shelf level
rcw26_shelf_02       origin (0.312, -0.57,   0.0)  bbox_max_z 2.0196457125231130  <- same asset as shelf; matches known 1.07 usable-shelf-level, not its own bbox top
```

`side_table`'s bbox top (0.6025) matches `side_table_02`'s own bbox top almost exactly (0.6025 vs known-good 0.612) -- same furniture asset reused at a second location -- so `side_table`'s surface z uses `side_table_02`'s known 0.612 rather than its own slightly-lower raw bbox top. `shelf`'s bbox top (2.0196) is the top of the whole multi-level shelf unit, not a usable shelf surface -- it is identical to `shelf_02`'s own bbox top (2.0196), which is known to actually sit objects at z=1.07 (a middle shelf level, per the bench scenario's `bowl` pose), so `shelf` reuses that same 1.07. `laundry_desk`'s bbox top (0.7337, rounds to 0.734) is used directly -- there is no twin furniture instance to cross-check against, and it lands within a centimetre of `kitchen_table`'s own known desk-height surface (0.734), which is a reasonable real-furniture-height sanity check. `sofa`'s bbox top (0.653) is the backrest, not the seat, so its surface z uses the design doc's flat fallback of 0.45m (a typical sofa seat height) rather than the bbox.

**If `pxr` is genuinely unavailable** (a future environment without the SIM repo's own `uv`-managed venv), fall back to these exact numbers instead of measuring: surface z = 0.75 for tables/desks, 1.07 for shelves, 0.45 for the sofa seat; xy = the DEC spot pose (`possible_poses` in `constants.rcw2026.json`) moved 0.45m along the spot's own facing direction (`yaw = 2*atan2(orientation.z, orientation.w)`, direction = `(cos(yaw), sin(yaw))`).

The two remaining person poses (`bedroom`, `laundry_room`) use the design doc's exact fallback formula (there is no analogous "already spawned" ground truth for these two rooms the way there is for kitchen/living_room): DEC's `possible_poses[<room's first search_spots entry>]` point, moved 0.8m along that pose's own facing direction (`yaw = 2*atan2(z,w)`), `z=0`, with the person's own orientation rotated 180 degrees from the spot's yaw so the person faces back toward the spot. Computed 2026-08-29:

- `bedroom`: first search spot is `side_table_02`, pose point `(1.08, 1.755, 0.0)`, `z=0.9999999999932537, w=-3.673205103346574e-06` (yaw = 180.0 deg, facing_dir = `(-1.0, 0.0)`) -> person `xyz = (0.28, 1.755, 0.0)`, back-facing yaw = 0 deg -> `quaternion_xyzw = (0.0, 0.0, 0.0, 1.0)`.
- `laundry_room`: first search spot is `laundry_desk`, pose point `(-3.237, 3.924, 0.0)`, `z=0.5555298041916854, w=0.8314966245600446` (yaw = 67.49 deg, facing_dir = `(0.3828, 0.9238)`) -> person `xyz = (-2.931, 4.663, 0.0)`, back-facing yaw = 247.49 deg -> `quaternion_xyzw = (0.0, 0.0, 0.831497, -0.55553)`.

- [ ] **Step 1: Write the failing test**

Create `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/tests/test_gpsr_placements.py`:

```python
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

PLACEMENTS_PATH = ROOT / "simulation" / "scenarios" / "rcw2026-placements.json"
CONSTANTS_PATH = Path(
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json"
)
BENCH_SCENARIO_PATH = ROOT / "simulation" / "scenarios" / "gpsr-rcw2026-bench.json"


def _load_placements() -> dict:
    return json.loads(PLACEMENTS_PATH.read_text())


def _load_constants() -> dict:
    return json.loads(CONSTANTS_PATH.read_text())


def test_placements_file_is_valid_json_with_schema_version():
    data = _load_placements()
    assert data["schema_version"] == 1
    assert "spots" in data and "persons" in data


def test_every_dec_search_spot_has_a_placements_surface():
    constants = _load_constants()
    placements = _load_placements()
    search_spots = constants["search_spots"]
    all_spots = {spot for room, spots in search_spots.items() if not room.startswith("_") for spot in spots}
    missing = all_spots - set(placements["spots"])
    assert missing == set(), f"placements.spots is missing: {missing}"


def test_every_dec_room_has_a_placements_person_pose():
    constants = _load_constants()
    placements = _load_placements()
    rooms = {r for r in constants["search_spots"] if not r.startswith("_")}
    missing = rooms - set(placements["persons"])
    assert missing == set(), f"placements.persons is missing: {missing}"


def test_every_spot_entry_has_a_3d_surface_xyz_and_grid_spacing():
    placements = _load_placements()
    for spot, info in placements["spots"].items():
        assert len(info["surface_xyz"]) == 3, spot
        assert all(isinstance(v, (int, float)) for v in info["surface_xyz"]), spot
        assert info["grid_dx"] > 0, spot
        assert info["grid_dy"] > 0, spot


def test_every_person_entry_has_a_3d_xyz_and_a_unit_quaternion():
    placements = _load_placements()
    for room, info in placements["persons"].items():
        assert len(info["xyz"]) == 3, room
        xyzw = info["quaternion_xyzw"]
        assert len(xyzw) == 4, room
        magnitude = sum(v * v for v in xyzw) ** 0.5
        assert abs(magnitude - 1.0) < 1e-3, room


def test_known_surfaces_match_the_bench_scenario_object_poses():
    # kitchen_table/side_table_02/shelf_02 are the three surfaces the bench
    # scenario already measures via its spawned soup/banana/bowl poses --
    # the placement table must not silently drift from what is actually
    # spawned there today.
    placements = _load_placements()
    bench = json.loads(BENCH_SCENARIO_PATH.read_text())
    by_id = {obj["id"]: obj for obj in bench["objects"]}
    assert placements["spots"]["kitchen_table"]["surface_xyz"] == by_id["soup"]["pose"]["xyz"]
    assert placements["spots"]["side_table_02"]["surface_xyz"] == by_id["banana"]["pose"]["xyz"]
    assert placements["spots"]["shelf_02"]["surface_xyz"] == by_id["bowl"]["pose"]["xyz"]


def test_known_person_poses_match_the_bench_scenario_actor_poses():
    placements = _load_placements()
    bench = json.loads(BENCH_SCENARIO_PATH.read_text())
    by_id = {actor["id"]: actor for actor in bench["actors"]}
    assert placements["persons"]["kitchen"]["xyz"] == by_id["kitchen_person"]["pose"]["xyz"]
    assert (
        placements["persons"]["kitchen"]["quaternion_xyzw"]
        == by_id["kitchen_person"]["pose"]["quaternion_xyzw"]
    )
    assert placements["persons"]["living_room"]["xyz"] == by_id["livingroom_person"]["pose"]["xyz"]


def test_measured_spots_are_within_the_arena_bounds():
    # Sanity bound on the four newly measured spots -- the rcw2026 arena is
    # roughly a 10m x 10m room; a wildly wrong measurement (e.g. an
    # un-transformed local-frame coordinate) would fall far outside this.
    placements = _load_placements()
    for spot in ("side_table", "laundry_desk", "shelf", "sofa"):
        x, y, z = placements["spots"][spot]["surface_xyz"]
        assert -8.0 < x < 8.0, spot
        assert -8.0 < y < 8.0, spot
        assert 0.0 < z < 2.5, spot


def test_measured_person_poses_are_within_the_arena_bounds():
    placements = _load_placements()
    for room in ("bedroom", "laundry_room"):
        x, y, z = placements["persons"][room]["xyz"]
        assert -8.0 < x < 8.0, room
        assert -8.0 < y < 8.0, room
        assert z == 0.0, room
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
python3 -m pytest tests/test_gpsr_placements.py -q
```
Expected: FAIL (`FileNotFoundError` / all tests error, `rcw2026-placements.json` does not exist yet).

- [ ] **Step 3: Write `simulation/scenarios/rcw2026-placements.json`**

```json
{
  "schema_version": 1,
  "_comment": "surface_xyz for kitchen_table/side_table_02/shelf_02 and persons.kitchen/persons.living_room are exactly the poses already spawned in gpsr-rcw2026-bench.json (soup, banana, bowl, kitchen_person, livingroom_person). The other four spot surfaces (side_table, laundry_desk, shelf, sofa) were measured 2026-08-29 from the built rcw2026 arena USD (artifacts/asset-manifest.json['generated_arena_usds'][0]['path'], or any artifacts/arena/rcw2026/*/arena.usd -- two independently-built copies gave identical furniture placements) using `uv run --frozen --no-sync python` (pxr is present in this repo's own uv-managed venv; see tools/tinker_sim_deploy/cli.py's _launch_command for this invocation pattern) to read /World/Arena/Furniture/rcw26_<spot> prim world-space origin xy (UsdGeom.Xformable.ComputeLocalToWorldTransform) and world bounding-box max z (UsdGeom.BBoxCache.ComputeWorldBound). side_table reuses side_table_02's known surface z (0.612) since both are the same furniture asset and their raw bbox tops match almost exactly (0.6025 vs 0.6025); shelf reuses shelf_02's known usable-shelf-level z (1.07) for the same reason (both bbox tops are 2.0196, the top of the whole multi-level unit, not a usable shelf surface); laundry_desk uses its own bbox top directly (0.7337, rounds to 0.734, within 1cm of kitchen_table's own known desk height); sofa uses the design doc's flat seat-height fallback (0.45) since its bbox top (0.653) is the backrest, not the seat. Person poses for bedroom/laundry_room use the design doc's exact fallback formula (no analogous already-spawned ground truth for these two rooms): DEC constants.rcw2026.json possible_poses[<room's first search_spots entry>] point, moved 0.8m along that pose's own facing direction (yaw = 2*atan2(orientation.z, orientation.w)), z=0, with the person's own orientation rotated 180 degrees from the spot's yaw so they face back toward the spot.",
  "spots": {
    "kitchen_table": {"surface_xyz": [2.5, -3.0, 0.734], "grid_dx": 0.18, "grid_dy": 0.15},
    "side_table_02": {"surface_xyz": [0.43, 1.755, 0.612], "grid_dx": 0.18, "grid_dy": 0.15},
    "shelf_02": {"surface_xyz": [0.312, -0.57, 1.07], "grid_dx": 0.18, "grid_dy": 0.15},
    "side_table": {"surface_xyz": [-0.346, -5.045, 0.612], "grid_dx": 0.18, "grid_dy": 0.15},
    "laundry_desk": {"surface_xyz": [-2.988, 4.525, 0.734], "grid_dx": 0.18, "grid_dy": 0.15},
    "shelf": {"surface_xyz": [-3.687, 0.309, 1.07], "grid_dx": 0.18, "grid_dy": 0.15},
    "sofa": {"surface_xyz": [-2.184, -4.839, 0.45], "grid_dx": 0.18, "grid_dy": 0.15}
  },
  "persons": {
    "kitchen": {"xyz": [1.8, -3.85, 0.0], "quaternion_xyzw": [0.0, 0.0, -0.381912, 0.924199]},
    "living_room": {"xyz": [-4.684, -4.089, 0.0], "quaternion_xyzw": [0.0, 0.0, -0.381912, 0.924199]},
    "bedroom": {"xyz": [0.28, 1.755, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]},
    "laundry_room": {"xyz": [-2.931, 4.663, 0.0], "quaternion_xyzw": [0.0, 0.0, 0.831497, -0.55553]}
  }
}
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
python3 -m pytest tests/test_gpsr_placements.py -q
```
Expected: 9 passed.

- [ ] **Step 5: Commit**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
git add simulation/scenarios/rcw2026-placements.json tests/test_gpsr_placements.py
git commit -m "$(cat <<'EOF'
feat(gpsr): measured rcw2026 placement table (spot surfaces + person poses)

Three surfaces/two person poses come from the bench scenario's already-
spawned poses; the other four surfaces and two person poses were
measured from the built arena USD via pxr (uv run --frozen --no-sync
python) and DEC's constants.rcw2026.json spot poses. Feeds the Task 3
scene planner.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01DxMkrcYzHbnJNjTsAQSe46
EOF
)"
```

---

### Task 3: SIM command-driven scene planner

**Files:**
- Create: `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/tools/gpsr_scene.py`
- Test: `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/tests/test_gpsr_scene.py`

**Interfaces:**
- Consumes: `simulation/scenarios/rcw2026-placements.json` (Task 2), `artifacts/asset-manifest.json` (existing), DEC's `constants.rcw2026.json` (existing, read by absolute path, not `.env`).
- Produces (consumed by Task 4 and Task 5):
  - `SpawnItem` frozen dataclass: `id: str, kind: str ("object"|"person"), name: str, asset_uri: str, room: str, spot: str, xyz: tuple[float,float,float], quaternion_xyzw: tuple[float,float,float,float]`.
  - `ScenePlan` frozen dataclass: `items: tuple[SpawnItem, ...], notes: tuple[str, ...]`.
  - `plan_scene(command_text: str, knowledge: dict, placements: dict, *, seed: int, asset_root: Path | None = None) -> ScenePlan`.
  - `scene_plan_to_json(plan: ScenePlan, *, command_text: str, seed: int) -> dict` and `scene_plan_from_json(data: dict) -> ScenePlan` (round-trip serialization Task 4's CLI uses to write/read `scene-plan.json`).
  - `CATEGORY_MAP: dict[str, tuple[str, ...]]`, `NAME_TO_YCB_DIR: dict[str, str]`, `PERSON_ASSET_URI: str` (module-level constants Task 4/5 may also reference).

- [ ] **Step 1: Write the failing tests**

Create `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/tests/test_gpsr_scene.py`:

```python
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.gpsr_scene import (  # noqa: E402
    CATEGORY_MAP,
    NAME_TO_YCB_DIR,
    PERSON_ASSET_URI,
    plan_scene,
    scene_plan_from_json,
    scene_plan_to_json,
)

CONSTANTS_PATH = Path(
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json"
)
PLACEMENTS_PATH = ROOT / "simulation" / "scenarios" / "rcw2026-placements.json"


@pytest.fixture(scope="module")
def knowledge():
    return json.loads(CONSTANTS_PATH.read_text())


@pytest.fixture(scope="module")
def placements():
    return json.loads(PLACEMENTS_PATH.read_text())


def _names(items):
    return [it.name for it in items]


def _ids(items):
    return [it.id for it in items]


# --- real corpus commands, s2026-001..007 (verbatim texts from
# gpsr_runs/bench/t2-2026/corpus.jsonl) ------------------------------------

def test_s2026_001_counts_three_kitchen_item_members_at_kitchen_table(knowledge, placements):
    text = "tell me how many kitchen items there are on the kitchen_table"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert [it.kind for it in plan.items] == ["object", "object", "object"]
    assert _names(plan.items) == ["bleach", "bowl", "mustard"]
    assert _ids(plan.items) == ["cmd_bleach_0", "cmd_bowl_0", "cmd_mustard_0"]
    for it in plan.items:
        assert it.spot == "kitchen_table"
        assert it.room == "kitchen"
    xs = [it.xyz[0] for it in plan.items]
    assert xs == [pytest.approx(2.5), pytest.approx(2.68), pytest.approx(2.86)]
    assert all(it.xyz[1] == pytest.approx(-3.0) for it in plan.items)
    assert all(it.xyz[2] == pytest.approx(0.734) for it in plan.items)


def test_s2026_002_counts_persons_spawns_no_objects_and_a_bedroom_person(knowledge, placements):
    text = "tell me how many persons pointing to the left are in the bedroom"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert [it.kind for it in plan.items] == ["person"]
    person = plan.items[0]
    assert person.room == "bedroom"
    assert person.id == "cmd_person_bedroom"
    assert person.xyz == pytest.approx((0.28, 1.755, 0.0))


def test_s2026_003_finds_kitchen_item_category_at_the_named_placement_spot(knowledge, placements):
    # The command names both a search room ("living_room") and a final
    # placement spot ("kitchen_table"); the literal Where rule scans every
    # known SPOT before any room, so the spot wins and objects spawn at
    # kitchen_table, not living_room.
    text = "locate a kitchen item in the living_room then grasp it and place it on the kitchen_table"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert _names(plan.items) == ["bleach", "bowl", "mustard"]
    assert all(it.spot == "kitchen_table" for it in plan.items)
    assert all(it.room == "kitchen" for it in plan.items)


def test_s2026_004_explicit_pudding_box_at_side_table_02_no_person(knowledge, placements):
    text = "go to the bedroom then locate a pudding_box and fetch it and put it on the side_table_02"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert len(plan.items) == 1
    item = plan.items[0]
    assert item.kind == "object"
    assert item.name == "pudding_box"
    assert item.id == "cmd_pudding_box_0"
    assert item.spot == "side_table_02"
    assert item.room == "bedroom"
    assert item.xyz == pytest.approx((0.43, 1.755, 0.612))


def test_s2026_005_named_person_at_the_named_room_no_objects(knowledge, placements):
    text = "introduce yourself to Liam in the living_room and tell what day is today"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert [it.kind for it in plan.items] == ["person"]
    assert plan.items[0].room == "living_room"
    assert plan.items[0].xyz == pytest.approx((-4.684, -4.089, 0.0))


def test_s2026_006_named_person_room_from_the_explicit_spot_not_the_later_room(knowledge, placements):
    # Two location words appear ("laundry_desk", "kitchen"); the explicit
    # spot's owning room (laundry_room) wins over the later plain room word.
    text = "meet Sarah at the laundry_desk then locate them in the kitchen"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert [it.kind for it in plan.items] == ["person"]
    assert plan.items[0].room == "laundry_room"
    assert plan.items[0].xyz == pytest.approx((-2.931, 4.663, 0.0))


def test_s2026_007_named_person_at_laundry_room(knowledge, placements):
    text = "meet Liam in the laundry_room and say the day of the month"
    plan = plan_scene(text, knowledge, placements, seed=2026)

    assert [it.kind for it in plan.items] == ["person"]
    assert plan.items[0].room == "laundry_room"
    assert plan.items[0].xyz == pytest.approx((-2.931, 4.663, 0.0))


# --- deterministic category sampling ---------------------------------------

def test_category_sampling_is_deterministic_by_seed(knowledge, placements):
    text = "tell me how many kitchen items there are on the kitchen_table"
    plan_a = plan_scene(text, knowledge, placements, seed=2026)
    plan_b = plan_scene(text, knowledge, placements, seed=2026)
    assert _names(plan_a.items) == _names(plan_b.items)

    plan_other_seed = plan_scene(text, knowledge, placements, seed=7)
    assert len(plan_other_seed.items) == 3


# --- unparseable / no-content text -----------------------------------------

def test_unrecognized_text_returns_an_empty_plan_with_a_note(knowledge, placements):
    plan = plan_scene("asdf qwerty zxcv", knowledge, placements, seed=1)
    assert plan.items == ()
    assert any("no object" in note for note in plan.notes)


def test_empty_text_never_raises(knowledge, placements):
    plan = plan_scene("", knowledge, placements, seed=1)
    assert plan.items == ()


# --- grid layout never duplicates xyz ---------------------------------------

def test_grid_layout_never_duplicates_xyz_for_a_counted_object(knowledge, placements):
    text = "tell me how many spam there are on the kitchen_table"
    plan = plan_scene(text, knowledge, placements, seed=1)
    assert len(plan.items) == 3
    xy_pairs = [(it.xyz[0], it.xyz[1]) for it in plan.items]
    assert len(set(xy_pairs)) == 3


# --- asset resolution --------------------------------------------------------

def test_every_category_map_member_has_a_ycb_asset_mapping():
    all_members = {name for members in CATEGORY_MAP.values() for name in members}
    assert all_members <= set(NAME_TO_YCB_DIR)


def test_resolved_object_items_carry_a_nonempty_asset_uri(knowledge, placements):
    text = "go to the bedroom then locate a pudding_box and fetch it and put it on the side_table_02"
    plan = plan_scene(text, knowledge, placements, seed=1)
    assert plan.items[0].asset_uri.endswith("ycb_008_pudding_box/object.usd")


def test_person_items_use_the_bench_scenario_person_asset(knowledge, placements):
    plan = plan_scene("meet Liam in the laundry_room", knowledge, placements, seed=1)
    assert plan.items[0].asset_uri == PERSON_ASSET_URI


# --- JSON round-trip ----------------------------------------------------------

def test_scene_plan_json_round_trips(knowledge, placements):
    text = "go to the bedroom then locate a pudding_box and fetch it and put it on the side_table_02"
    plan = plan_scene(text, knowledge, placements, seed=1)
    data = scene_plan_to_json(plan, command_text=text, seed=1)
    restored = scene_plan_from_json(data)
    assert restored == plan
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
python3 -m pytest tests/test_gpsr_scene.py -q
```
Expected: FAIL/ERROR on collection (`tools.gpsr_scene` does not exist yet).

- [ ] **Step 3: Write `tools/gpsr_scene.py`**

```python
#!/usr/bin/env python3
"""Command-driven GPSR scene planner.

Reads a GPSR command's text and DEC's constants.rcw2026.json (rooms,
placement spots, object vocabulary) plus this repo's
simulation/scenarios/rcw2026-placements.json (measured surface/person
poses) and works out which objects (and person) that command's scene
actually needs, and where. Pure stdlib, no ROS -- tools/gpsr_spawn.py is
the only thing that talks to the simulator.

Never raises on an unparseable command: an unrecognised object/category,
location, or person mention degrades to an empty (or partial) ScenePlan
with an explanatory note in `.notes`, never an exception.
"""
from __future__ import annotations

import json
import random
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parent.parent

CATEGORY_MAP: dict[str, tuple[str, ...]] = {
    "food": ("banana", "spam", "pudding_box", "sugar_box", "cheez_it", "soup"),
    "drink": ("mug", "soup"),
    "kitchen item": ("bowl", "mug", "mustard", "bleach"),
}

NAME_TO_YCB_DIR: dict[str, str] = {
    "soup": "ycb_010_tomato_soup_can",
    "mug": "ycb_025_mug",
    "banana": "ycb_011_banana",
    "mustard": "ycb_006_mustard_bottle",
    "sugar_box": "ycb_002_sugar_box",
    "spam": "ycb_005_spam",
    "cheez_it": "ycb_001_cheez-it",
    "pudding_box": "ycb_008_pudding_box",
    "bowl": "ycb_024_bowl",
    "bleach": "ycb_021_bleach_cleanser",
}

# The bench scenario's own actor asset -- there is no per-person asset
# variety modelled yet, so every spawned person uses this same USD.
PERSON_ASSET_URI = (
    "artifacts/people/sobits/d29ee5ef3b71bcbf7013ec61785a584b162bfda83a322ecce0a6a481180e531a"
    "/person_standing/person.usd"
)

_PERSON_NAMES = ("alex", "sarah", "john", "emma", "liam", "olivia")
_PERSON_TRIGGER_VERBS = ("greet", "meet", "guide", "follow")


@dataclass(frozen=True)
class SpawnItem:
    id: str
    kind: str  # "object" | "person"
    name: str
    asset_uri: str
    room: str
    spot: str  # "" for persons (they use room, not a placement spot)
    xyz: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class ScenePlan:
    items: tuple[SpawnItem, ...]
    notes: tuple[str, ...]


def _normalize_words(text: str) -> str:
    return re.sub(r"_+", " ", text).lower()


def _phrase_pattern(phrase: str) -> re.Pattern:
    words = _normalize_words(phrase).split()
    escaped = [re.escape(w) for w in words]
    if escaped:
        escaped[-1] = escaped[-1] + "s?"
    return re.compile(r"\b" + r"\s+".join(escaped) + r"\b")


def _text_contains(text: str, phrase: str) -> bool:
    return _phrase_pattern(phrase).search(_normalize_words(text)) is not None


def _is_counting_template(text: str) -> bool:
    normalized = text.lower()
    return "how many" in normalized or re.search(r"\bcount\b", normalized) is not None


def _object_names(knowledge: dict) -> list[str]:
    # Longest-first: a shorter name that is a text-prefix of a longer one
    # (there are none among the current 10, but this stays correct if that
    # ever changes) must not shadow the longer, more specific match.
    return sorted(knowledge.get("possible_objects", {}), key=lambda n: (-len(n), n))


def _match_object_name(text: str, knowledge: dict) -> Optional[str]:
    for name in _object_names(knowledge):
        if _text_contains(text, name):
            return name
    return None


def _match_category(text: str) -> Optional[str]:
    for category in sorted(CATEGORY_MAP, key=lambda c: (-len(c), c)):
        if _text_contains(text, category):
            return category
    return None


def _resolve_location(text: str, knowledge: dict) -> tuple[Optional[str], Optional[str]]:
    """Return (room, spot) from an explicit mention in `text`. An explicit
    placement spot wins over an explicit room name (its owning room is
    used); neither present returns (None, None). Longest-name-first so
    "side_table_02" is preferred over the "side_table" it starts with.
    """
    search_spots = knowledge["search_spots"]
    spot_to_room = {
        s: r for r, spots in search_spots.items() if not r.startswith("_") for s in spots
    }
    for spot in sorted(spot_to_room, key=lambda s: (-len(s), s)):
        if _text_contains(text, spot):
            return spot_to_room[spot], spot
    for room in sorted(search_spots, key=lambda r: (-len(r), r)):
        if room.startswith("_"):
            continue
        if _text_contains(text, room):
            return room, search_spots[room][0]
    return None, None


def _person_triggered(text: str) -> bool:
    normalized = text.lower()
    if re.search(r"\bpersons?\b", normalized):
        return True
    if re.search(r"\bsomeone\b", normalized):
        return True
    words = re.findall(r"[a-z]+", normalized)
    if any(name in words for name in _PERSON_NAMES):
        return True
    if any(re.search(r"\b" + verb + r"\w*\b", normalized) for verb in _PERSON_TRIGGER_VERBS):
        return True
    return False


def _resolve_person_room(text: str, knowledge: dict, first_object_spot: Optional[str]) -> str:
    room, _spot = _resolve_location(text, knowledge)
    if room is not None:
        return room
    if first_object_spot is not None:
        spot_to_room = {
            s: r
            for r, spots in knowledge["search_spots"].items()
            if not r.startswith("_")
            for s in spots
        }
        return spot_to_room.get(first_object_spot, "living_room")
    return "living_room"


def _load_object_asset_uris(asset_root: Path) -> dict[str, str]:
    manifest_path = Path(asset_root) / "artifacts" / "asset-manifest.json"
    try:
        data = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    by_name: dict[str, str] = {}
    for entry in data.get("generated_object_usds", []):
        path = entry.get("path", "")
        for name, ycb_dir in NAME_TO_YCB_DIR.items():
            if f"/{ycb_dir}/object.usd" in path:
                by_name[name] = path
    return by_name


def plan_scene(
    command_text: str,
    knowledge: dict,
    placements: dict,
    *,
    seed: int,
    asset_root: Optional[Path] = None,
) -> ScenePlan:
    """Plan the objects/person a GPSR command's scene needs. Pure Python,
    no ROS; never raises -- an unparseable or location-less command
    degrades to an empty or partial plan with an explanatory note.
    """
    notes: list[str] = []
    items: list[SpawnItem] = []
    rng = random.Random(seed)
    by_name = _load_object_asset_uris(asset_root or REPO_ROOT)

    object_name = _match_object_name(command_text, knowledge)
    category = None if object_name else _match_category(command_text)
    counting = _is_counting_template(command_text)

    object_specs: list[tuple[str, int]] = []
    if object_name:
        count = 3 if counting else 1
        object_specs = [(object_name, i) for i in range(count)]
        notes.append(
            f"explicit object '{object_name}' x{count}"
            + (" (counting template)" if counting else "")
        )
    elif category:
        members = sorted(CATEGORY_MAP[category])
        k = min(3, len(members))
        sampled = rng.sample(members, k)
        object_specs = [(name, 0) for name in sampled]
        notes.append(f"category '{category}' sampled {sampled} (seed={seed})")
    else:
        notes.append("no object or category named; no objects spawned")

    resolved_spot: Optional[str] = None
    if object_specs:
        _room, resolved_spot = _resolve_location(command_text, knowledge)
        if resolved_spot is None:
            first_name = object_specs[0][0]
            resolved_spot = knowledge.get("default_locations", {}).get(first_name)
            if resolved_spot is None:
                resolved_spot = knowledge["search_spots"]["kitchen"][0]
            notes.append(f"no location named; defaulted to '{resolved_spot}'")
        spot_info = placements["spots"].get(resolved_spot)
        if spot_info is None:
            notes.append(f"spot '{resolved_spot}' has no placement entry; no objects spawned")
        else:
            surface_xyz = spot_info["surface_xyz"]
            grid_dx = spot_info["grid_dx"]
            grid_dy = spot_info["grid_dy"]
            spot_to_room = {
                s: r
                for r, spots in knowledge["search_spots"].items()
                if not r.startswith("_")
                for s in spots
            }
            item_room = spot_to_room.get(resolved_spot, "kitchen")
            for i, (name, index) in enumerate(object_specs):
                xyz = (
                    surface_xyz[0] + (i % 3) * grid_dx,
                    surface_xyz[1] + (i // 3) * grid_dy,
                    surface_xyz[2],
                )
                asset_uri = by_name.get(name, "")
                if not asset_uri:
                    notes.append(f"asset uri not found for '{name}'")
                items.append(
                    SpawnItem(
                        id=f"cmd_{name}_{index}",
                        kind="object",
                        name=name,
                        asset_uri=asset_uri,
                        room=item_room,
                        spot=resolved_spot,
                        xyz=xyz,
                        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                    )
                )

    if _person_triggered(command_text):
        person_room = _resolve_person_room(command_text, knowledge, resolved_spot)
        person_pose = placements.get("persons", {}).get(person_room)
        if person_pose is None:
            notes.append(f"person triggered but room '{person_room}' has no known pose; skipped")
        else:
            items.append(
                SpawnItem(
                    id=f"cmd_person_{person_room}",
                    kind="person",
                    name="person",
                    asset_uri=PERSON_ASSET_URI,
                    room=person_room,
                    spot="",
                    xyz=tuple(person_pose["xyz"]),
                    quaternion_xyzw=tuple(person_pose["quaternion_xyzw"]),
                )
            )
            notes.append(f"person at '{person_room}'")

    return ScenePlan(items=tuple(items), notes=tuple(notes))


def scene_plan_to_json(plan: ScenePlan, *, command_text: str, seed: int) -> dict:
    return {
        "schema_version": 1,
        "command": command_text,
        "seed": seed,
        "notes": list(plan.notes),
        "items": [
            {
                "id": item.id,
                "kind": item.kind,
                "name": item.name,
                "asset_uri": item.asset_uri,
                "room": item.room,
                "spot": item.spot,
                "xyz": list(item.xyz),
                "quaternion_xyzw": list(item.quaternion_xyzw),
            }
            for item in plan.items
        ],
    }


def scene_plan_from_json(data: dict) -> ScenePlan:
    items = tuple(
        SpawnItem(
            id=it["id"],
            kind=it["kind"],
            name=it["name"],
            asset_uri=it["asset_uri"],
            room=it["room"],
            spot=it["spot"],
            xyz=tuple(it["xyz"]),
            quaternion_xyzw=tuple(it["quaternion_xyzw"]),
        )
        for it in data.get("items", [])
    )
    notes = tuple(data.get("notes", []))
    return ScenePlan(items=items, notes=notes)
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
python3 -m pytest tests/test_gpsr_scene.py -q
```
Expected: 16 passed.

- [ ] **Step 5: Commit**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
git add tools/gpsr_scene.py tests/test_gpsr_scene.py
git commit -m "$(cat <<'EOF'
feat(gpsr): command-driven scene planner (tools/gpsr_scene.py)

Pure-Python plan_scene() resolves a GPSR command's text into the objects
(and person) its scene actually needs, using DEC's constants.rcw2026.json
vocabulary and the Task 2 placement table. Covers the real s2026-001..007
battery corpus texts plus deterministic category sampling, unparseable
text, and grid layout.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01DxMkrcYzHbnJNjTsAQSe46
EOF
)"
```

---

### Task 4: SIM spawn/clear/emit-scenario CLI

**Files:**
- Create: `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/tools/gpsr_spawn.py`
- Test: `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/tests/test_gpsr_spawn_cli.py`

**Interfaces:**
- Consumes: `SpawnItem`, `ScenePlan`, `scene_plan_to_json`, `scene_plan_from_json`, `plan_scene` from Task 3's `tools/gpsr_scene.py`.
- Produces (consumed by Task 6's DEC bench wiring, via the CLI's argv contract, and by Task 5 via the `scene-plan.json` file format already defined in Task 3):
  - CLI subcommands `plan`, `apply`, `clear`, `emit-scenario` (see below); exit codes `0` ok, `2` service unavailable/timeout, `3` bad plan.
  - `main(argv: list[str] | None = None) -> int`.
  - Pure functions usable without ROS: `base_scenario_keys(base_scenario: dict, placements: dict) -> set[tuple[str,str]]`, `apply_plan(plan: ScenePlan, client, *, base_scenario: dict, placements: dict, previous_manifest: dict | None = None) -> dict`, `clear_manifest(manifest: dict, client) -> dict`, `emit_scenario(plans: list[ScenePlan], base_scenario: dict, placements: dict) -> dict`.
  - `ServiceClient` protocol: `.spawn(item: SpawnItem) -> str` (returns the entity name, raises on failure) and `.delete(entity: str) -> bool` (True on success/NOT_FOUND).

- [ ] **Step 1: Write the failing tests**

Create `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/tests/test_gpsr_spawn_cli.py`:

```python
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.gpsr_scene import ScenePlan, SpawnItem  # noqa: E402
from tools.gpsr_spawn import (  # noqa: E402
    apply_plan,
    base_scenario_keys,
    clear_manifest,
    emit_scenario,
    main,
)

PLACEMENTS = json.loads((ROOT / "simulation" / "scenarios" / "rcw2026-placements.json").read_text())
BASE_SCENARIO = json.loads((ROOT / "simulation" / "scenarios" / "gpsr-rcw2026-bench.json").read_text())


class FakeServiceClient:
    def __init__(self, fail_ids=()):
        self.spawned = []
        self.deleted = []
        self.fail_ids = set(fail_ids)

    def spawn(self, item):
        if item.id in self.fail_ids:
            raise RuntimeError(f"spawn failed for {item.id}")
        self.spawned.append(item.id)
        return f"/World/Scenario/{item.id}"

    def delete(self, entity):
        self.deleted.append(entity)
        return True


def _item(id_, name, spot, room, xyz, kind="object",
         asset_uri="artifacts/objects/ycb/x/object.usd"):
    return SpawnItem(id=id_, kind=kind, name=name, asset_uri=asset_uri, room=room, spot=spot,
                     xyz=xyz, quaternion_xyzw=(0.0, 0.0, 0.0, 1.0))


def test_base_scenario_keys_finds_the_four_bench_scenario_items():
    keys = base_scenario_keys(BASE_SCENARIO, PLACEMENTS)
    assert ("ycb_010_tomato_soup_can", "kitchen_table") in keys
    assert ("ycb_025_mug", "kitchen_table") in keys
    assert ("ycb_011_banana", "side_table_02") in keys
    assert ("ycb_024_bowl", "shelf_02") in keys
    assert ("person_standing", "kitchen") in keys
    assert ("person_standing", "living_room") in keys


def test_apply_plan_skips_items_already_in_the_base_scenario():
    plan = ScenePlan(
        items=(_item("cmd_soup_0", "soup", "kitchen_table", "kitchen", (2.5, -3.0, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_010_tomato_soup_can/object.usd"),),
        notes=(),
    )
    client = FakeServiceClient()
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    assert manifest["entities"] == []
    assert manifest["skipped"][0]["id"] == "cmd_soup_0"
    assert client.spawned == []


def test_apply_plan_spawns_a_new_item_and_records_the_entity_name():
    plan = ScenePlan(
        items=(_item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    client = FakeServiceClient()
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    assert manifest["skipped"] == []
    assert len(manifest["entities"]) == 1
    entity = manifest["entities"][0]
    assert entity["ok"] is True
    assert entity["entity_name"] == "/World/Scenario/cmd_spam_0"
    assert client.spawned == ["cmd_spam_0"]


def test_apply_plan_records_a_failed_spawn_without_raising():
    plan = ScenePlan(
        items=(_item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    client = FakeServiceClient(fail_ids={"cmd_spam_0"})
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS)
    assert manifest["entities"][0]["ok"] is False
    assert "spawn failed" in manifest["entities"][0]["error"]


def test_apply_plan_skips_items_already_in_a_previous_manifest():
    plan = ScenePlan(
        items=(_item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    client = FakeServiceClient()
    previous = {
        "entities": [
            {"id": "cmd_spam_0", "asset_key": "ycb_005_spam", "where": "laundry_desk",
             "entity_name": "/World/Scenario/cmd_spam_0", "ok": True}
        ],
        "skipped": [],
    }
    manifest = apply_plan(plan, client, base_scenario=BASE_SCENARIO, placements=PLACEMENTS,
                          previous_manifest=previous)
    assert manifest["skipped"][0]["id"] == "cmd_spam_0"
    assert client.spawned == []


def test_clear_manifest_deletes_every_ok_entity_and_tolerates_missing():
    manifest = {
        "entities": [
            {"id": "cmd_spam_0", "entity_name": "/World/Scenario/cmd_spam_0", "ok": True,
             "asset_key": "ycb_005_spam", "where": "laundry_desk"},
            {"id": "cmd_bad_0", "entity_name": "", "ok": False, "asset_key": "x", "where": "y"},
        ],
        "skipped": [],
    }
    client = FakeServiceClient()
    updated = clear_manifest(manifest, client)
    assert client.deleted == ["/World/Scenario/cmd_spam_0"]
    assert updated["entities"][0]["cleared"] is True
    assert "cleared" not in updated["entities"][1]


def test_emit_scenario_merges_a_new_item_into_a_copy_of_the_base_scenario():
    plan = ScenePlan(
        items=(_item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    merged = emit_scenario([plan], BASE_SCENARIO, PLACEMENTS)
    assert len(BASE_SCENARIO["objects"]) == 4  # original untouched
    assert len(merged["objects"]) == len(BASE_SCENARIO["objects"]) + 1
    new_ids = {o["id"] for o in merged["objects"]} - {o["id"] for o in BASE_SCENARIO["objects"]}
    assert len(new_ids) == 1


def test_emit_scenario_dedupes_against_the_base_scenario():
    plan = ScenePlan(
        items=(_item("cmd_soup_0", "soup", "kitchen_table", "kitchen", (2.5, -3.0, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_010_tomato_soup_can/object.usd"),),
        notes=(),
    )
    merged = emit_scenario([plan], BASE_SCENARIO, PLACEMENTS)
    assert len(merged["objects"]) == len(BASE_SCENARIO["objects"])


def test_emit_scenario_larger_count_wins_between_two_plans():
    small = ScenePlan(
        items=(_item("cmd_spam_0", "spam", "laundry_desk", "laundry_room", (-2.988, 4.525, 0.734),
                     asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd"),),
        notes=(),
    )
    big = ScenePlan(
        items=tuple(
            _item(f"cmd_spam_{i}", "spam", "laundry_desk", "laundry_room",
                 (-2.988 + 0.18 * i, 4.525, 0.734),
                 asset_uri="artifacts/objects/ycb/x/ycb_005_spam/object.usd")
            for i in range(3)
        ),
        notes=(),
    )
    merged = emit_scenario([small, big], BASE_SCENARIO, PLACEMENTS)
    spam_items = [o for o in merged["objects"] if "spam" in o["asset_uri"]]
    assert len(spam_items) == 3


def test_cli_plan_writes_a_scene_plan_json(tmp_path):
    out = tmp_path / "scene-plan.json"
    code = main([
        "plan", "--command",
        "go to the bedroom then locate a pudding_box and fetch it and put it on the side_table_02",
        "--seed", "1", "--out", str(out),
    ])
    assert code == 0
    data = json.loads(out.read_text())
    assert data["items"][0]["name"] == "pudding_box"


def test_cli_emit_scenario_merges_plans(tmp_path):
    plan_path = tmp_path / "scene-plan.json"
    main(["plan", "--command", "bring me a spam from the laundry_desk", "--seed", "1",
         "--out", str(plan_path)])
    out = tmp_path / "generated.json"
    code = main([
        "emit-scenario", "--plans", str(plan_path),
        "--base", str(ROOT / "simulation" / "scenarios" / "gpsr-rcw2026-bench.json"),
        "--placements", str(ROOT / "simulation" / "scenarios" / "rcw2026-placements.json"),
        "--out", str(out),
    ])
    assert code == 0
    merged = json.loads(out.read_text())
    assert len(merged["objects"]) >= len(BASE_SCENARIO["objects"])
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
python3 -m pytest tests/test_gpsr_spawn_cli.py -q
```
Expected: FAIL/ERROR on collection (`tools.gpsr_spawn` does not exist yet).

- [ ] **Step 3: Write `tools/gpsr_spawn.py`**

```python
#!/usr/bin/env python3
"""Spawn/clear a GPSR command's scene in the sim, or pre-generate a merged
scenario file for a whole battery.

`plan`/`emit-scenario` are pure Python (no ROS import at all). `apply`/
`clear` talk to the running sim's `/spawn_entity` and `/delete_entity`
simulation_interfaces services -- the only place this module imports
rclpy is `_make_ros_service_client`, called lazily so every other
subcommand (and every test) never needs ROS on the path.

Exit codes: 0 ok, 2 service unavailable/timeout (a run's `apply`/`clear`
failing this way is treated the same as a reset failure by the bench),
3 bad plan/scenario/placements input.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional, Protocol, Sequence

from tools.gpsr_scene import (
    REPO_ROOT,
    ScenePlan,
    SpawnItem,
    plan_scene,
    scene_plan_from_json,
    scene_plan_to_json,
)

DEFAULT_CONSTANTS = Path(
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json"
)
DEFAULT_PLACEMENTS = REPO_ROOT / "simulation" / "scenarios" / "rcw2026-placements.json"
DEFAULT_BASE_SCENARIO = REPO_ROOT / "simulation" / "scenarios" / "gpsr-rcw2026-bench.json"


class ServiceClient(Protocol):
    def spawn(self, item: SpawnItem) -> str:
        """Spawn `item`; return its entity name, or raise on failure."""
        ...

    def delete(self, entity: str) -> bool:
        """Delete `entity`; return True on success (NOT_FOUND counts as
        success -- the entity is already gone)."""
        ...


def _asset_key(asset_uri: str) -> str:
    """The distinguishing directory name of an asset_uri, e.g.
    ".../ycb_010_tomato_soup_can/object.usd" -> "ycb_010_tomato_soup_can",
    ".../person_standing/person.usd" -> "person_standing".
    """
    parts = Path(asset_uri).parts
    return parts[-2] if len(parts) >= 2 else asset_uri


def _nearest_spot_or_room(xyz, placements: dict, tolerance: float = 0.05) -> Optional[str]:
    for spot, info in placements.get("spots", {}).items():
        sx, sy, _sz = info["surface_xyz"]
        if abs(xyz[0] - sx) <= tolerance and abs(xyz[1] - sy) <= tolerance:
            return spot
    for room, info in placements.get("persons", {}).items():
        px, py, _pz = info["xyz"]
        if abs(xyz[0] - px) <= tolerance and abs(xyz[1] - py) <= tolerance:
            return room
    return None


def base_scenario_keys(base_scenario: dict, placements: dict) -> set[tuple[str, str]]:
    """(asset_key, spot-or-room) already present in the base scenario,
    derived by matching each spawned pose against the placement table
    (not hardcoded, so it stays correct if the base scenario changes).
    """
    keys: set[tuple[str, str]] = set()
    for record in (*base_scenario.get("actors", []), *base_scenario.get("objects", [])):
        asset = _asset_key(record.get("asset_uri", ""))
        xyz = record.get("pose", {}).get("xyz", [0.0, 0.0, 0.0])
        where = _nearest_spot_or_room(xyz, placements)
        if where is not None:
            keys.add((asset, where))
    return keys


def apply_plan(
    plan: ScenePlan,
    client: "ServiceClient",
    *,
    base_scenario: dict,
    placements: dict,
    previous_manifest: Optional[dict] = None,
) -> dict:
    """Spawn every item in `plan` via `client`, skipping items whose
    (asset, spot-or-room) already exists in the base scenario or a
    previous manifest. Never raises for a per-item spawn failure -- it is
    recorded in the returned manifest's entities with `"ok": False`.
    """
    seen_keys = set(base_scenario_keys(base_scenario, placements))
    entities: list[dict] = []
    if previous_manifest:
        for e in previous_manifest.get("entities", []):
            seen_keys.add((e["asset_key"], e["where"]))
            entities.append(e)

    skipped: list[dict] = []
    for item in plan.items:
        where = item.spot if item.spot else item.room
        asset_key = _asset_key(item.asset_uri)
        key = (asset_key, where)
        if key in seen_keys:
            skipped.append({"id": item.id, "reason": "already present", "where": where})
            continue
        entity_name = f"/World/Scenario/{item.id}"
        try:
            actual = client.spawn(item)
        except Exception as exc:  # noqa: BLE001 - never crash a bench run
            entities.append(
                {"id": item.id, "asset_key": asset_key, "where": where,
                 "entity_name": entity_name, "ok": False, "error": repr(exc)}
            )
            continue
        seen_keys.add(key)
        entities.append(
            {"id": item.id, "asset_key": asset_key, "where": where,
             "entity_name": actual, "ok": True}
        )
    return {"entities": entities, "skipped": skipped}


def clear_manifest(manifest: dict, client: "ServiceClient") -> dict:
    """Delete every successfully-spawned entity in `manifest` via
    `client`. Tolerates NOT_FOUND (client.delete returning True). Never
    raises for a per-item delete failure.
    """
    results: list[dict] = []
    for e in manifest.get("entities", []):
        if not e.get("ok"):
            results.append(e)
            continue
        try:
            deleted = client.delete(e["entity_name"])
        except Exception as exc:  # noqa: BLE001 - never crash a bench run
            results.append({**e, "cleared": False, "clear_error": repr(exc)})
            continue
        results.append({**e, "cleared": bool(deleted)})
    return {"entities": results, "skipped": manifest.get("skipped", [])}


def emit_scenario(plans: Sequence[ScenePlan], base_scenario: dict, placements: dict) -> dict:
    """Merge every plan's items into a COPY of base_scenario (base_scenario
    itself is never mutated). Dedupe by (asset, spot-or-room) against the
    base scenario; when two plans want the same (asset, spot-or-room),
    the plan that proposed MORE instances of it wins outright (its whole
    item group is used, the smaller plan's group is dropped).
    """
    merged = json.loads(json.dumps(base_scenario))
    existing = base_scenario_keys(base_scenario, placements)

    best: dict[tuple[str, str], list[SpawnItem]] = {}
    for plan in plans:
        per_plan: dict[tuple[str, str], list[SpawnItem]] = {}
        for item in plan.items:
            where = item.spot if item.spot else item.room
            key = (_asset_key(item.asset_uri), where)
            per_plan.setdefault(key, []).append(item)
        for key, group in per_plan.items():
            if key in existing:
                continue
            if key not in best or len(group) > len(best[key]):
                best[key] = group

    for (_asset_key_value, where), group in best.items():
        target_list = merged["actors"] if group[0].kind == "person" else merged["objects"]
        for item in group:
            target_list.append(
                {
                    # Suffixed by `where` so two different (asset, where)
                    # groups across different plans can never collide on
                    # id even if their SpawnItem ids happened to match.
                    "id": f"{item.id}__{where}",
                    "asset_uri": item.asset_uri,
                    "pose": {"xyz": list(item.xyz), "quaternion_xyzw": list(item.quaternion_xyzw)},
                }
            )
    return merged


def _resolve_asset_uri(asset_uri: str) -> str:
    if "://" in asset_uri:
        return asset_uri
    path = Path(asset_uri)
    if not path.is_absolute():
        path = REPO_ROOT / path
    return str(path.resolve())


def _make_ros_service_client() -> "ServiceClient":
    """The only place this module imports rclpy -- constructed lazily so
    `plan`/`emit-scenario` (and every test) never need ROS on the path.
    Raises if `/spawn_entity` never becomes available.
    """
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from simulation_interfaces.msg import Result
    from simulation_interfaces.srv import DeleteEntity, SpawnEntity

    rclpy.init(args=None)
    node = Node("gpsr_spawn_client")
    spawn_client = node.create_client(SpawnEntity, "/spawn_entity")
    delete_client = node.create_client(DeleteEntity, "/delete_entity")
    if not spawn_client.wait_for_service(timeout_sec=10.0):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("/spawn_entity service unavailable after 10s")

    class _RclpyServiceClient:
        def spawn(self, item: SpawnItem) -> str:
            request = SpawnEntity.Request()
            request.name = f"/World/Scenario/{item.id}"
            request.allow_renaming = False
            request.uri = _resolve_asset_uri(item.asset_uri)
            request.entity_namespace = "Scenario"
            pose = PoseStamped()
            pose.header.frame_id = "world"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = item.xyz
            (
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ) = item.quaternion_xyzw
            request.initial_pose = pose
            future = spawn_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
            if future.result() is None:
                raise RuntimeError(f"spawn_entity timed out for {item.id}")
            response = future.result()
            if response.result.result != Result.RESULT_OK:
                raise RuntimeError(
                    f"spawn_entity failed for {item.id}: {response.result.error_message}"
                )
            return str(response.entity_name)

        def delete(self, entity: str) -> bool:
            request = DeleteEntity.Request()
            request.entity = entity
            future = delete_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
            if future.result() is None:
                raise RuntimeError(f"delete_entity timed out for {entity}")
            response = future.result()
            if response.result.result == Result.RESULT_NOT_FOUND:
                return True
            return response.result.result == Result.RESULT_OK

        def shutdown(self) -> None:
            node.destroy_node()
            rclpy.shutdown()

    return _RclpyServiceClient()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Spawn/clear a GPSR command's scene in the sim.")
    sub = parser.add_subparsers(dest="cmd", required=True)

    plan_p = sub.add_parser("plan", help="plan a scene from a command (no ROS)")
    plan_p.add_argument("--command", required=True)
    plan_p.add_argument("--seed", type=int, required=True)
    plan_p.add_argument("--out", required=True)
    plan_p.add_argument("--constants", default=str(DEFAULT_CONSTANTS))
    plan_p.add_argument("--placements", default=str(DEFAULT_PLACEMENTS))

    apply_p = sub.add_parser("apply", help="spawn a plan's items via /spawn_entity")
    apply_p.add_argument("--plan", required=True)
    apply_p.add_argument("--manifest", required=True)
    apply_p.add_argument("--base-scenario", default=str(DEFAULT_BASE_SCENARIO))
    apply_p.add_argument("--placements", default=str(DEFAULT_PLACEMENTS))

    clear_p = sub.add_parser("clear", help="delete a manifest's spawned entities via /delete_entity")
    clear_p.add_argument("--manifest", required=True)

    emit_p = sub.add_parser("emit-scenario", help="merge plans into a scenario file (no ROS)")
    emit_p.add_argument("--plans", nargs="+", required=True)
    emit_p.add_argument("--base", default=str(DEFAULT_BASE_SCENARIO))
    emit_p.add_argument("--placements", default=str(DEFAULT_PLACEMENTS))
    emit_p.add_argument("--out", required=True)

    return parser


def _load_json_or_none(path: str) -> Optional[dict]:
    p = Path(path)
    if not p.is_file():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None


def _cmd_plan(args: argparse.Namespace) -> int:
    try:
        knowledge = json.loads(Path(args.constants).read_text(encoding="utf-8"))
        placements = json.loads(Path(args.placements).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"gpsr_spawn: bad plan inputs: {exc}", file=sys.stderr)
        return 3
    plan = plan_scene(args.command, knowledge, placements, seed=args.seed)
    data = scene_plan_to_json(plan, command_text=args.command, seed=args.seed)
    Path(args.out).write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    return 0


def _cmd_apply(args: argparse.Namespace) -> int:
    try:
        plan_data = json.loads(Path(args.plan).read_text(encoding="utf-8"))
        base_scenario = json.loads(Path(args.base_scenario).read_text(encoding="utf-8"))
        placements = json.loads(Path(args.placements).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"gpsr_spawn: bad plan inputs: {exc}", file=sys.stderr)
        return 3
    plan = scene_plan_from_json(plan_data)
    previous_manifest = _load_json_or_none(args.manifest)
    try:
        client = _make_ros_service_client()
    except Exception as exc:  # noqa: BLE001
        print(f"gpsr_spawn: service client unavailable: {exc}", file=sys.stderr)
        return 2
    try:
        manifest = apply_plan(plan, client, base_scenario=base_scenario, placements=placements,
                              previous_manifest=previous_manifest)
    finally:
        client.shutdown()
    Path(args.manifest).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    if any(not e.get("ok", True) for e in manifest["entities"]):
        return 2
    return 0


def _cmd_clear(args: argparse.Namespace) -> int:
    manifest = _load_json_or_none(args.manifest)
    if manifest is None:
        print(f"gpsr_spawn: bad manifest: {args.manifest}", file=sys.stderr)
        return 3
    try:
        client = _make_ros_service_client()
    except Exception as exc:  # noqa: BLE001
        print(f"gpsr_spawn: service client unavailable: {exc}", file=sys.stderr)
        return 2
    try:
        updated = clear_manifest(manifest, client)
    finally:
        client.shutdown()
    Path(args.manifest).write_text(json.dumps(updated, indent=2) + "\n", encoding="utf-8")
    if any(not e.get("cleared", True) for e in updated["entities"]):
        return 2
    return 0


def _cmd_emit_scenario(args: argparse.Namespace) -> int:
    try:
        plans = [scene_plan_from_json(json.loads(Path(p).read_text(encoding="utf-8")))
                for p in args.plans]
        base_scenario = json.loads(Path(args.base).read_text(encoding="utf-8"))
        placements = json.loads(Path(args.placements).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(f"gpsr_spawn: bad plan inputs: {exc}", file=sys.stderr)
        return 3
    merged = emit_scenario(plans, base_scenario, placements)
    Path(args.out).write_text(json.dumps(merged, indent=2) + "\n", encoding="utf-8")
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    args = _build_arg_parser().parse_args(argv)
    if args.cmd == "plan":
        return _cmd_plan(args)
    if args.cmd == "apply":
        return _cmd_apply(args)
    if args.cmd == "clear":
        return _cmd_clear(args)
    if args.cmd == "emit-scenario":
        return _cmd_emit_scenario(args)
    return 3  # pragma: no cover - argparse `required=True` prevents this


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
python3 -m pytest tests/test_gpsr_spawn_cli.py -q
```
Expected: 11 passed.

- [ ] **Step 5: Commit**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
git add tools/gpsr_spawn.py tests/test_gpsr_spawn_cli.py
git commit -m "$(cat <<'EOF'
feat(gpsr): spawn/clear/emit-scenario CLI (tools/gpsr_spawn.py)

plan/emit-scenario are pure Python; apply/clear talk to /spawn_entity
and /delete_entity behind an injectable ServiceClient (the rclpy
implementation is isolated in one lazily-imported function, exercised
only by the Task 7 live spike -- every test here uses a fake client).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01DxMkrcYzHbnJNjTsAQSe46
EOF
)"
```

---

### Task 5: SIM contact-sheet judge-sheet scene line

**Files:**
- Modify: `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/tools/contact_sheet.py`
- Modify: `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/tests/test_contact_sheet.py`

**Interfaces:**
- Consumes: the `scene-plan.json` file format from Task 3 (`scene_plan_to_json`'s output shape: `{"items": [{"id","kind","name","asset_uri","room","spot","xyz","quaternion_xyzw"}, ...]}`), read directly by path -- no import of `tools.gpsr_scene` needed.
- Produces: nothing new consumed by later tasks (this is the last consumer of the scene-plan format in this plan).

- [ ] **Step 1: Add the failing tests to `tests/test_contact_sheet.py`**

Edit the import block at the top of `tests/test_contact_sheet.py`:

```python
# before
from tools.contact_sheet import (  # noqa: E402
    build_sheet,
    build_judge_sheet,
    sample_evenly,
    select_event_rows,
    _stamp_from_name,
    _dedup_transcript,
    REPLAN_BAND_COLOR,
)
```

```python
# after
from tools.contact_sheet import (  # noqa: E402
    build_sheet,
    build_judge_sheet,
    sample_evenly,
    select_event_rows,
    _stamp_from_name,
    _dedup_transcript,
    _scene_summary_line,
    REPLAN_BAND_COLOR,
)
```

Append these tests at the end of `tests/test_contact_sheet.py` (after the existing `test_main_default_creates_judge_sheet` function):

```python
def test_scene_summary_line_formats_objects_and_person(tmp_path):
    (tmp_path / "scene-plan.json").write_text(json.dumps({
        "items": [
            {"id": "cmd_bleach_0", "kind": "object", "name": "bleach",
             "spot": "kitchen_table", "room": "kitchen"},
            {"id": "cmd_bowl_0", "kind": "object", "name": "bowl",
             "spot": "kitchen_table", "room": "kitchen"},
            {"id": "cmd_person_bedroom", "kind": "person", "name": "person",
             "spot": "", "room": "bedroom"},
        ],
    }))
    assert _scene_summary_line(tmp_path) == (
        "scene: bleach@kitchen_table x1, bowl@kitchen_table x1, person@bedroom x1"
    )


def test_scene_summary_line_missing_file_returns_none(tmp_path):
    assert _scene_summary_line(tmp_path) is None


def test_scene_summary_line_corrupt_file_returns_none(tmp_path):
    (tmp_path / "scene-plan.json").write_text("{not json")
    assert _scene_summary_line(tmp_path) is None


def test_scene_summary_line_empty_items_returns_none(tmp_path):
    (tmp_path / "scene-plan.json").write_text(json.dumps({"items": []}))
    assert _scene_summary_line(tmp_path) is None


def test_build_judge_sheet_includes_scene_summary_from_scene_plan(tmp_path):
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    _mk_judge_events(tmp_path)
    (tmp_path / "scene-plan.json").write_text(json.dumps({
        "items": [
            {"id": "cmd_bleach_0", "kind": "object", "name": "bleach",
             "spot": "kitchen_table", "room": "kitchen"},
        ],
    }))
    meta = {"id": "c013", "text": "x", "verdict": "PASS", "seconds": 1.0, "tier": "T2"}
    out = build_judge_sheet(tmp_path, meta, tmp_path / "judge-scene.jpg")
    assert out is not None
    assert out.exists()


def test_build_judge_sheet_survives_a_corrupt_scene_plan(tmp_path):
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    _mk_judge_events(tmp_path)
    (tmp_path / "scene-plan.json").write_text("{not json")
    meta = {"id": "c014", "verdict": "PASS"}
    out = build_judge_sheet(tmp_path, meta, tmp_path / "judge-badscene.jpg")
    assert out is not None
    assert out.exists()
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
python3 -m pytest tests/test_contact_sheet.py -q
```
Expected: `ImportError` (`_scene_summary_line` does not exist yet in `tools.contact_sheet`), collection fails for the whole file.

- [ ] **Step 3: Add `_scene_summary_line` and wire it into `_build_judge_sheet`**

Insert immediately before `def build_judge_sheet(run_dir: Path, meta: dict, out: Path) -> Optional[Path]:`:

```python
def _scene_summary_line(run_dir: Path) -> Optional[str]:
    """Summarize <run_dir>/scene-plan.json's spawned items for the judge-
    sheet header, e.g. "scene: bleach@kitchen_table x1, person@bedroom
    x1" -- so a reviewer can see what the arena actually contained.
    Missing/corrupt/empty file -> None; never raises.
    """
    plan_path = Path(run_dir) / "scene-plan.json"
    if not plan_path.is_file():
        return None
    try:
        data = json.loads(plan_path.read_text(encoding="utf-8"))
    except Exception:
        return None
    items = data.get("items")
    if not isinstance(items, list) or not items:
        return None
    counts: dict[str, int] = {}
    order: list[str] = []
    for item in items:
        if not isinstance(item, dict):
            continue
        if item.get("kind") == "person":
            key = f"person@{item.get('room', '?')}"
        else:
            spot = item.get("spot") or item.get("room", "?")
            key = f"{item.get('name', '?')}@{spot}"
        if key not in counts:
            order.append(key)
        counts[key] = counts.get(key, 0) + 1
    if not order:
        return None
    return "scene: " + ", ".join(f"{key} x{counts[key]}" for key in order)


```

Edit `_build_judge_sheet`:

```python
# before
    detail = str(meta.get("detail") or "").strip()
    extra_line = f"detail: {detail}" if detail else None
```

```python
# after
    detail = str(meta.get("detail") or "").strip()
    scene_summary = _scene_summary_line(run_dir)
    extra_parts = []
    if detail:
        extra_parts.append(f"detail: {detail}")
    if scene_summary:
        extra_parts.append(scene_summary)
    extra_line = " | ".join(extra_parts) if extra_parts else None
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
python3 -m pytest tests/test_contact_sheet.py -q
```
Expected: all pass (existing tests + 6 new ones).

- [ ] **Step 5: Commit**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
git add tools/contact_sheet.py tests/test_contact_sheet.py
git commit -m "$(cat <<'EOF'
feat(gpsr): judge-sheet header shows the placed scene

Reads <run_dir>/scene-plan.json (Task 3's format) and appends a
"scene: object@spot x1, ..." line to the judge sheet's header, next to
the existing bench-detail line. Missing/corrupt file -> no line; never
fails the sheet.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01DxMkrcYzHbnJNjTsAQSe46
EOF
)"
```

---

### Task 6: DEC tier2/gpsr_bench spawn/clear hooks

**Files:**
- Modify: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/bench/tier1.py`
- Modify: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/bench/tier2.py`
- Modify: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_bench.py`
- Test: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_tier2_spawn_hooks.py`

**Interfaces:**
- Consumes: nothing at import time from Task 1-5 (the CLI it shells out to, `tools/gpsr_spawn.py`, is an external process invoked via `--spawn-cmd`/`--clear-cmd`, not a Python import).
- Produces: `run_tier2(..., spawn_cmd: Sequence[str] | None = None, clear_cmd: Sequence[str] | None = None)`; `run.json` gains a `"scene": {"plan": "scene-plan.json", "spawned": "spawned.json"}` key (relative paths) when both files exist in the run dir; `bench_env(...)`'s returned dict always includes `"GPSR_SIM_IDENTITY_RELAXED": "1"` (Task 1's flag); `gpsr-bench tier2` gains `--spawn-cmd`/`--clear-cmd` CLI flags.

- [ ] **Step 1: Write the failing tests**

Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_tier2_spawn_hooks.py`:

```python
"""tier2's --spawn-cmd/--clear-cmd hooks and the GPSR_SIM_IDENTITY_RELAXED
env flag (command-driven-scene-and-sim-identity-design.md, 2026-08-28).

Mirrors test_gpsr_bench_tier2.py's fake-launcher/fake-command style: a
`sh -c` script appends to an order log so we can assert ordering without
a real orchestrator or a real gpsr_spawn.py.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""
from __future__ import annotations

import json
import sys
import textwrap
from pathlib import Path

import pytest

from behavior_tree.GPSR.bench import tier1, tier2
from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.tier2 import run_tier2


@pytest.fixture(autouse=True)
def _fake_llm_preflight_ok(monkeypatch):
    monkeypatch.setattr(tier2, "llm_preflight", lambda env: (True, ""))


def _entry(i, text):
    return CorpusEntry(id=f"c{i}", seed=7, template="goToLoc", followups=(), category="objects",
                       text=text, feasibility="A")


def _fake_orchestrator(tmp_path: Path, marker: Path | None = None) -> list[str]:
    script = tmp_path / "fake_orch_spawn.py"
    marker_line = f"open({str(marker)!r}, 'w').close()" if marker is not None else "pass"
    script.write_text(textwrap.dedent(f"""
        import json, os, time
        {marker_line}
        d = os.path.join(os.environ["BT_GPSR_PLAN_DIR"], "debug", "traj-1"); os.makedirs(d, exist_ok=True)
        f = open(os.path.join(d, "events.jsonl"), "a", buffering=1)
        def ev(t, payload):
            f.write(json.dumps({{"event_type": t, "task_id": "traj-1/task-1", "payload": payload, "occurred_at": "x"}}) + "\\n")
        ev("step.finished", {{"action": "goto", "outcome": "succeeded"}})
        ev("task.finished", {{"status": "succeeded", "reason": "r"}})
        while True:
            time.sleep(1)
    """))
    return [sys.executable, str(script)]


def test_bench_env_carries_the_sim_identity_relaxation_flag(tmp_path):
    env = tier1.bench_env(mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                          plan_dir=tmp_path / "plan", commands=["go to the sofa"], live_llm=False)
    assert env["GPSR_SIM_IDENTITY_RELAXED"] == "1"


def test_run_tier2_spawn_cmd_runs_before_the_recorder_and_writes_scene(tmp_path):
    order_log = tmp_path / "order.log"
    reset_cmd = ["sh", "-c", "echo reset >> " + str(order_log)]
    recorder_cmd = ["sh", "-c",
                    "echo recorder-start >> " + str(order_log) + "; "
                    "trap 'echo recorder-stop >> " + str(order_log) + "; exit 0' INT TERM; "
                    "while true; do sleep 1; done"]
    spawn_cmd = ["sh", "-c", "echo spawn >> " + str(order_log) + "; touch {plan} {manifest}"]
    clear_cmd = ["sh", "-c", "echo clear >> " + str(order_log)]

    entries = [_entry(0, "go to the sofa")]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=_fake_orchestrator(tmp_path),
                        reset_cmd=reset_cmd, recorder_cmd=recorder_cmd,
                        spawn_cmd=spawn_cmd, clear_cmd=clear_cmd, settle_s=0)

    assert results[0].verdict == "PASS"
    order = order_log.read_text().splitlines()
    assert order.index("reset") < order.index("spawn") < order.index("recorder-start")
    assert "clear" in order

    run_json = json.loads((tmp_path / "out" / "runs" / "c0" / "run.json").read_text())
    assert run_json["scene"] == {"plan": "scene-plan.json", "spawned": "spawned.json"}


def test_run_tier2_spawn_failure_scores_error_and_skips_the_run(tmp_path):
    marker = tmp_path / "orchestrator-marker"
    entries = [_entry(0, "go to the sofa")]
    spawn_cmd = ["sh", "-c", "echo boom 1>&2; exit 1"]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=_fake_orchestrator(tmp_path, marker=marker),
                        reset_cmd=["true"], spawn_cmd=spawn_cmd, settle_s=0)
    assert results[0].verdict == "ERROR"
    assert "spawn failed" in results[0].detail
    assert not marker.exists()


def test_run_tier2_clear_failure_is_appended_to_detail_without_changing_verdict(tmp_path):
    entries = [_entry(0, "go to the sofa")]
    clear_cmd = ["sh", "-c", "echo boom 1>&2; exit 1"]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=_fake_orchestrator(tmp_path),
                        reset_cmd=["true"], clear_cmd=clear_cmd, settle_s=0)
    assert results[0].verdict == "PASS"
    assert "clear failed" in results[0].detail


def test_run_tier2_without_spawn_cmd_writes_no_scene_key(tmp_path):
    entries = [_entry(0, "go to the sofa")]
    results = run_tier2(entries, mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                        out_dir=tmp_path / "out", timeout_s=20,
                        launcher=_fake_orchestrator(tmp_path),
                        reset_cmd=["true"], settle_s=0)
    assert results[0].verdict == "PASS"
    run_json = json.loads((tmp_path / "out" / "runs" / "c0" / "run.json").read_text())
    assert "scene" not in run_json
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest test/test_tier2_spawn_hooks.py -q
```
Expected: FAIL (`GPSR_SIM_IDENTITY_RELAXED` not in env; `run_tier2` raises `TypeError` for the unexpected `spawn_cmd`/`clear_cmd` kwargs).

- [ ] **Step 3: Add the relaxation flag to `bench_env` in `tier1.py`**

```python
# before
    env.update({
        "BT_GPSR_CMD": "|".join(commands),
        "BT_GPSR_NUM_COMMANDS": str(len(commands)),
        "BT_MOCK_MODE": "true",
        "BT_MOCK_CONFIG": str(mock_config),
        "GPSR_OFFLINE_PLANNER": "0" if live_llm else "1",
        "GPSR_CONSTANTS_PATH": str(constants),
        "BT_GPSR_PLAN_DIR": str(plan_dir),
        "GPSR_DEBUG_TELEMETRY": "1",
        "BT_LISTEN_MOCK_TYPED": "0",
    })
    return env
```

```python
# after
    env.update({
        "BT_GPSR_CMD": "|".join(commands),
        "BT_GPSR_NUM_COMMANDS": str(len(commands)),
        "BT_MOCK_MODE": "true",
        "BT_MOCK_CONFIG": str(mock_config),
        "GPSR_OFFLINE_PLANNER": "0" if live_llm else "1",
        "GPSR_CONSTANTS_PATH": str(constants),
        "BT_GPSR_PLAN_DIR": str(plan_dir),
        "GPSR_DEBUG_TELEMETRY": "1",
        "BT_LISTEN_MOCK_TYPED": "0",
        # Sim persons carry no name identity -- every tier-2 run is
        # against the sim, so this is always on here (never set for a
        # real-robot launch, which does not go through bench_env at all).
        "GPSR_SIM_IDENTITY_RELAXED": "1",
    })
    return env
```

- [ ] **Step 4: Add `_run_spawn_cmd`/`_run_clear_cmd` to `tier2.py`**

Insert immediately after the existing `_reset` function (right after its closing `return None`):

```python
def _run_spawn_cmd(spawn_cmd: Sequence[str], mapping: dict[str, str]) -> str | None:
    """Run the scene-spawn command for one run; return an error detail
    string, or None on success. Mirrors _reset's shape."""
    cmd = _substitute(spawn_cmd, mapping)
    try:
        result = subprocess.run(cmd, timeout=120, capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return "spawn failed: timed out after 120s"
    except OSError as exc:
        return f"spawn failed: {exc}"
    if result.returncode != 0:
        stderr = (result.stderr or "").strip()
        tail = stderr[-2000:]
        suffix = f": {tail}" if tail else ""
        return f"spawn failed: exit code {result.returncode}{suffix}"
    return None


def _run_clear_cmd(clear_cmd: Sequence[str], mapping: dict[str, str]) -> str | None:
    """Run the scene-clear command for one run; return an error detail
    string, or None on success."""
    cmd = _substitute(clear_cmd, mapping)
    try:
        result = subprocess.run(cmd, timeout=60, capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return "clear failed: timed out after 60s"
    except OSError as exc:
        return f"clear failed: {exc}"
    if result.returncode != 0:
        stderr = (result.stderr or "").strip()
        tail = stderr[-2000:]
        suffix = f": {tail}" if tail else ""
        return f"clear failed: exit code {result.returncode}{suffix}"
    return None
```

- [ ] **Step 5: Add `scene` to `_write_run_json`**

```python
# before
def _write_run_json(run_dir: Path, entry: CorpusEntry, tier_label: str, result: BenchResult) -> None:
    data = {
        "id": entry.id, "text": entry.text, "template": entry.template,
        "feasibility": entry.feasibility, "tier": tier_label, "verdict": result.verdict,
        "detail": result.detail, "seconds": result.seconds,
    }
    (run_dir / "run.json").write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
```

```python
# after
def _write_run_json(run_dir: Path, entry: CorpusEntry, tier_label: str, result: BenchResult) -> None:
    data = {
        "id": entry.id, "text": entry.text, "template": entry.template,
        "feasibility": entry.feasibility, "tier": tier_label, "verdict": result.verdict,
        "detail": result.detail, "seconds": result.seconds,
    }
    scene = {}
    if (run_dir / "scene-plan.json").is_file():
        scene["plan"] = "scene-plan.json"
    if (run_dir / "spawned.json").is_file():
        scene["spawned"] = "spawned.json"
    if scene:
        data["scene"] = scene
    (run_dir / "run.json").write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
```

- [ ] **Step 6: Add `spawn_cmd`/`clear_cmd` params and wiring to `run_tier2`**

```python
# before
def run_tier2(entries: Sequence[CorpusEntry], *, mock_config: Path, constants: Path, out_dir: Path,
              timeout_s: float, tier_label: str = "T2",
              launcher: Sequence[str] = DEFAULT_LAUNCHER, reset_cmd: Sequence[str] = DEFAULT_RESET_CMD,
              recorder_cmd: list[str] | None = None,
              sheet_cmd: list[str] | None = None,
              settle_s: float = 10.0, halt_after_errors: int = 3,
              live_llm: bool = True, llm_check: bool = True) -> list[BenchResult]:
```

```python
# after
def run_tier2(entries: Sequence[CorpusEntry], *, mock_config: Path, constants: Path, out_dir: Path,
              timeout_s: float, tier_label: str = "T2",
              launcher: Sequence[str] = DEFAULT_LAUNCHER, reset_cmd: Sequence[str] = DEFAULT_RESET_CMD,
              recorder_cmd: list[str] | None = None,
              sheet_cmd: list[str] | None = None,
              spawn_cmd: list[str] | None = None,
              clear_cmd: list[str] | None = None,
              settle_s: float = 10.0, halt_after_errors: int = 3,
              live_llm: bool = True, llm_check: bool = True) -> list[BenchResult]:
```

```python
# before
        if settle_s > 0:
            time.sleep(settle_s)

        recorder_proc = None
        recorder_log = None
        try:
            if recorder_cmd:
                cmd = _substitute(recorder_cmd, {"run_dir": str(run_dir)})
                recorder_log = (run_dir / "recorder.log").open("a", encoding="utf-8")
                recorder_proc = subprocess.Popen(cmd, stdout=recorder_log, stderr=subprocess.STDOUT,
                                                 start_new_session=True)

            env = bench_env(mock_config=mock_config, constants=constants, plan_dir=run_dir,
                            commands=[entry.text], live_llm=live_llm)
            verdict, detail, seconds, plan = _run_orchestrator(
                env=env, run_dir=run_dir, launcher=launcher, timeout_s=timeout_s)
        except Exception as exc:
            # Unexpected exception in a single run: score as ERROR with detail, continue batch.
            # This exception source is typically Popen(launcher) raising OSError for unexecutable
            # binary (which occurs outside _run_orchestrator's own try/finally), but guards all
            # unexpected exceptions in this span.
            verdict, detail, seconds, plan = "ERROR", f"exception: {exc!r}", 0.0, []
        finally:
            if recorder_proc is not None:
                _stop(recorder_proc)
            if recorder_log is not None:
                recorder_log.close()
```

```python
# after
        if settle_s > 0:
            time.sleep(settle_s)

        scene_mapping = {
            "run_dir": str(run_dir), "command": entry.text, "seed": str(entry.seed),
            "plan": str(run_dir / "scene-plan.json"), "manifest": str(run_dir / "spawned.json"),
        }
        if spawn_cmd:
            spawn_error = _run_spawn_cmd(spawn_cmd, scene_mapping)
            if spawn_error is not None:
                result = BenchResult(entry.id, entry.template, entry.feasibility, TIER, "ERROR",
                                     spawn_error)
                _write_run_json(run_dir, entry, tier_label, result)
                results.append(result)
                if _halted(results, out_dir, halt_after_errors):
                    break
                continue

        recorder_proc = None
        recorder_log = None
        clear_error: str | None = None
        try:
            if recorder_cmd:
                cmd = _substitute(recorder_cmd, {"run_dir": str(run_dir)})
                recorder_log = (run_dir / "recorder.log").open("a", encoding="utf-8")
                recorder_proc = subprocess.Popen(cmd, stdout=recorder_log, stderr=subprocess.STDOUT,
                                                 start_new_session=True)

            env = bench_env(mock_config=mock_config, constants=constants, plan_dir=run_dir,
                            commands=[entry.text], live_llm=live_llm)
            verdict, detail, seconds, plan = _run_orchestrator(
                env=env, run_dir=run_dir, launcher=launcher, timeout_s=timeout_s)
        except Exception as exc:
            # Unexpected exception in a single run: score as ERROR with detail, continue batch.
            # This exception source is typically Popen(launcher) raising OSError for unexecutable
            # binary (which occurs outside _run_orchestrator's own try/finally), but guards all
            # unexpected exceptions in this span.
            verdict, detail, seconds, plan = "ERROR", f"exception: {exc!r}", 0.0, []
        finally:
            if recorder_proc is not None:
                _stop(recorder_proc)
            if recorder_log is not None:
                recorder_log.close()
            if clear_cmd:
                clear_error = _run_clear_cmd(clear_cmd, scene_mapping)

        if clear_error is not None:
            detail = (detail + " | " if detail else "") + clear_error
```

- [ ] **Step 7: Add `--spawn-cmd`/`--clear-cmd` to `gpsr_bench.py`**

```python
# before
            t.add_argument("--sheet-cmd", default=None, help="shell-split contact-sheet command; {run_dir}/{run_json}/{out} are substituted")
            t.add_argument("--settle", type=float, default=10.0, help="seconds to sleep after a successful reset")
```

```python
# after
            t.add_argument("--sheet-cmd", default=None, help="shell-split contact-sheet command; {run_dir}/{run_json}/{out} are substituted")
            t.add_argument("--spawn-cmd", default=None, help="shell-split scene-spawn command; {run_dir}/{command}/{seed}/{plan}/{manifest} are substituted")
            t.add_argument("--clear-cmd", default=None, help="shell-split scene-clear command; {run_dir}/{command}/{seed}/{plan}/{manifest} are substituted")
            t.add_argument("--settle", type=float, default=10.0, help="seconds to sleep after a successful reset")
```

```python
# before
    results = run_tier2(entries, mock_config=Path(args.mock_config), constants=Path(args.constants),
                        out_dir=Path(args.out), timeout_s=args.timeout, tier_label=args.tier_label,
                        reset_cmd=reset_cmd, recorder_cmd=recorder_cmd, sheet_cmd=sheet_cmd,
                        settle_s=args.settle, live_llm=not args.offline_planner,
                        llm_check=not args.skip_llm_check)
```

```python
# after
    spawn_cmd = shlex.split(args.spawn_cmd) if args.spawn_cmd else None
    clear_cmd = shlex.split(args.clear_cmd) if args.clear_cmd else None
    results = run_tier2(entries, mock_config=Path(args.mock_config), constants=Path(args.constants),
                        out_dir=Path(args.out), timeout_s=args.timeout, tier_label=args.tier_label,
                        reset_cmd=reset_cmd, recorder_cmd=recorder_cmd, sheet_cmd=sheet_cmd,
                        spawn_cmd=spawn_cmd, clear_cmd=clear_cmd,
                        settle_s=args.settle, live_llm=not args.offline_planner,
                        llm_check=not args.skip_llm_check)
```

- [ ] **Step 8: Run the new tests to verify they pass**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest test/test_tier2_spawn_hooks.py -q
```
Expected: 5 passed.

- [ ] **Step 9: Run the existing tier2 suite to confirm no regression**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest test/test_gpsr_bench_tier2.py test/test_tier2_spawn_hooks.py -q
```
Expected: all pass.

- [ ] **Step 10: Append the new test file to the DEC test-runner script**

```bash
grep -n "test/test_gpsr_resolve_pose_rooms.py" /home/tinker/.claude/jobs/1462b451/tmp/run-dec-tests.sh
```

Edit `/home/tinker/.claude/jobs/1462b451/tmp/run-dec-tests.sh`:

```bash
# before
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_resolve_pose_rooms.py test/test_gpsr_target_planner.py test/test_scan_stores_no_match_response.py test/test_parse_count_from_answer.py -q 2>&1 | tail -3
```

```bash
# after
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_gpsr_resolve_pose_rooms.py test/test_gpsr_target_planner.py test/test_scan_stores_no_match_response.py test/test_parse_count_from_answer.py test/test_sim_identity_relaxation.py test/test_tier2_spawn_hooks.py -q 2>&1 | tail -3
```

- [ ] **Step 11: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/GPSR/bench/tier1.py \
       src/behavior_tree/behavior_tree/GPSR/bench/tier2.py \
       src/behavior_tree/behavior_tree/GPSR/gpsr_bench.py \
       src/behavior_tree/test/test_tier2_spawn_hooks.py
git commit -m "$(cat <<'EOF'
feat(gpsr): tier2 --spawn-cmd/--clear-cmd hooks + GPSR_SIM_IDENTITY_RELAXED

Per run: spawn_cmd runs after a successful reset and before the
recorder starts (non-zero exit -> ERROR, mirrors _reset); clear_cmd
runs in the finally block after the recorder stops (failure appended
to detail, verdict unchanged). run.json gains a scene block when
scene-plan.json/spawned.json exist. bench_env always sets
GPSR_SIM_IDENTITY_RELAXED=1 for tier-2 runs (Part 1's flag).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01DxMkrcYzHbnJNjTsAQSe46
EOF
)"
```

---

### Task 7: Live spike (controller-run, not an implementer subagent)

This task requires the GPU stack up (Nav2 + sim + bridge), which an implementer subagent does not have. **Do not dispatch this task to an implementer.** The controlling session runs it directly after Tasks 1-6 are merged, following the project's stack-bring-up rules (message the training session first if GPU1 is theirs; nav before sim; never SIGKILL Nav2).

**Files:**
- Create: `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/scripts/gpsr-spawn-spike`
- Modify: `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/docs/developer-log.md` (append an entry with the outcome, per `feedback-runbook-vs-devlog` — fix narratives go here, not the runbook)

**Interfaces:**
- Consumes: Task 4's `tools/gpsr_spawn.py` CLI (`plan`/`apply`/`clear`), the running stack's `/sim/arena_camera/image_raw` topic and `/clock`.
- Produces: the developer-log entry that decides which of the three 2.4 outcomes (full apply/clear; apply-only; emit-scenario-only) Task 6's real battery script (`run-battery-t2.sh`, job tmp, not in a repo) uses for `--spawn-cmd`/`--clear-cmd`. Nothing later in this plan consumes this task's code — it is a one-shot diagnostic.

- [ ] **Step 1: Write `scripts/gpsr-spawn-spike`**

```bash
#!/usr/bin/env bash
# Live spike for the command-driven-scene design (2026-08-28 spec, section
# 2.4). Requires the GPU stack up (Nav2 + sim PLAYING + bridge). Spawns a
# pudding_box on kitchen_table via tools/gpsr_spawn.py plan+apply, checks
# it shows up in an arena-camera frame and that /clock/Nav2 stay healthy,
# then clears it. Prints PASS/FAIL per check and exits 0 only if every
# check passed.
set -uo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

pass() { echo "PASS: $1"; }
fail() { echo "FAIL: $1"; STATUS=1; }
STATUS=0

echo "== gpsr-spawn-spike: plan =="
python3 "$ROOT/tools/gpsr_spawn.py" plan \
  --command "count the pudding_box on the kitchen_table" --seed 1 \
  --out "$WORK/scene-plan.json"
if [ -s "$WORK/scene-plan.json" ]; then
  pass "plan wrote scene-plan.json"
else
  fail "plan did not write scene-plan.json"
fi

echo "== gpsr-spawn-spike: apply =="
python3 "$ROOT/tools/gpsr_spawn.py" apply \
  --plan "$WORK/scene-plan.json" --manifest "$WORK/spawned.json"
APPLY_EXIT=$?
if [ "$APPLY_EXIT" -eq 0 ]; then
  pass "apply exited 0"
else
  fail "apply exited $APPLY_EXIT"
fi

echo "== gpsr-spawn-spike: settle 5s, grab an arena-camera frame =="
sleep 5
timeout 10 ros2 topic echo /sim/arena_camera/image_raw --once --field header.stamp \
  > "$WORK/frame-stamp.txt" 2>&1
if [ -s "$WORK/frame-stamp.txt" ]; then
  pass "got an /sim/arena_camera/image_raw frame"
else
  fail "no /sim/arena_camera/image_raw frame within 10s"
fi
echo "  (manual check: save+view a frame to confirm the pudding_box is visible on kitchen_table"
echo "   -- ros2 run image_view image_saver or a one-off subscriber; this script only checks liveness)"

echo "== gpsr-spawn-spike: /clock monotonic over 10s =="
timeout 10 ros2 topic echo /clock --field clock > "$WORK/clock.txt" 2>&1
python3 - "$WORK/clock.txt" <<'PYEOF'
import sys
path = sys.argv[1]
try:
    values = [int(line.strip()) for line in open(path) if line.strip().lstrip("-").isdigit()]
except Exception as exc:
    print(f"FAIL: could not parse /clock samples ({exc})")
    sys.exit(1)
if len(values) < 2:
    print("FAIL: fewer than 2 /clock samples captured")
    sys.exit(1)
if all(b >= a for a, b in zip(values, values[1:])):
    print("PASS: /clock is monotonic over 10s")
    sys.exit(0)
print("FAIL: /clock went backwards (re-zeroed) during the spawn")
sys.exit(1)
PYEOF
[ $? -ne 0 ] && STATUS=1

echo "== gpsr-spawn-spike: clear =="
python3 "$ROOT/tools/gpsr_spawn.py" clear --manifest "$WORK/spawned.json"
CLEAR_EXIT=$?
if [ "$CLEAR_EXIT" -eq 0 ]; then
  pass "clear exited 0"
else
  fail "clear exited $CLEAR_EXIT"
fi

echo
if [ "$STATUS" -eq 0 ]; then
  echo "gpsr-spawn-spike: ALL CHECKS PASSED"
else
  echo "gpsr-spawn-spike: SOME CHECKS FAILED (see FAIL lines above)"
fi
exit "$STATUS"
```

```bash
chmod +x /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/scripts/gpsr-spawn-spike
```

- [ ] **Step 2: Bring up the stack per project rules**

Message the training session before bring-up if GPU1 is theirs while the stack is down. Bring up Nav2, then the sim (PLAYING), then the bridge, per the existing runbook.

- [ ] **Step 3: Run the spike**

```bash
/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/scripts/gpsr-spawn-spike
```

Read every PASS/FAIL line. Manually confirm the pudding_box actually appears in the right place in an arena-camera frame (the script only proves a frame arrived, not its content — grab one with `ros2 run image_view image_saver` or a one-off subscriber and eyeball it against `kitchen_table`'s known pose).

- [ ] **Step 4: Also run a 3-minute nav-only goto to confirm Nav2 stayed healthy through the spawn**

Use whatever nav-goto smoke check the project already has (per the runbook) to send the robot to a distant waypoint and confirm it completes normally.

- [ ] **Step 5: Append the outcome to the developer log**

Edit `/home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec/docs/developer-log.md`, appending an entry using this template (fill in the actual PASS/FAIL results and pick exactly one outcome):

```markdown
## 2026-08-2X: gpsr-spawn-spike — live scene-spawn viability

Ran `scripts/gpsr-spawn-spike` against the live stack (Nav2 + sim
PLAYING + bridge) per the command-driven-scene design
(docs/superpowers/specs/2026-08-28-command-driven-scene-and-sim-identity-design.md,
section 2.4): `tools/gpsr_spawn.py plan`+`apply` for "count the
pudding_box on the kitchen_table", confirmed placement via an
arena-camera frame, checked `/clock` monotonicity and Nav2 health, then
`clear`.

Results:
- plan: <PASS/FAIL>
- apply (spawn_entity): <PASS/FAIL>
- arena-camera frame shows the pudding_box at kitchen_table: <PASS/FAIL, manual>
- /clock monotonic over 10s: <PASS/FAIL>
- Nav2 3-minute goto still succeeds: <PASS/FAIL>
- clear (delete_entity): <PASS/FAIL>

**Decision:** <one of the following, per the spec's three outcomes>
- All three (spawn/frame, /clock+Nav2, clear) passed -> tier-2's default
  `--spawn-cmd`/`--clear-cmd` use `apply`/`clear` per run.
- Spawn works but delete does not -> per-run `apply` with dedupe, no
  `--clear-cmd` (objects accumulate within a battery; noted in each
  run.json's detail).
- Spawn while PLAYING fails, or breaks `/clock`/Nav2 -> use
  `emit-scenario` once per battery + relaunch the stack with the
  generated scenario; `--spawn-cmd`/`--clear-cmd` left unset.

Next: wire the chosen flags into `run-battery-t2.sh` (job tmp, not in a
repo) and re-run `s2026-003`, `004`, `005` (archive the old run dirs
first — the bench reuses run dirs in place). Acceptance: 003/004 no
longer fail on absent objects; 005 passes the person_found gate.
```

- [ ] **Step 6: Commit the spike script (not the developer-log entry's factual content, which depends on the live run)**

```bash
cd /home/tinker/tinker-sim/6.0.1/.claude/worktrees/gpsr-command-variety-spec
git add scripts/gpsr-spawn-spike docs/developer-log.md
git commit -m "$(cat <<'EOF'
feat(gpsr): live spawn-entity spike script + outcome in developer log

scripts/gpsr-spawn-spike: plan+apply a pudding_box on kitchen_table
against the live stack, checks an arena-camera frame arrives, /clock
stays monotonic, then clears. Decides which of the three 2.4 outcomes
(apply/clear, apply-only, emit-scenario-only) the real battery uses.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01DxMkrcYzHbnJNjTsAQSe46
EOF
)"
```

---

## Self-review notes

- **Spec coverage:** Part 1 (identity relaxation) -> Task 1. 2.1 (scene planner) -> Task 3. 2.2 (placement table) -> Task 2. 2.3 (spawn backends CLI) -> Task 4. 2.4 (live spike) -> Task 7. 2.5 (bench wiring) -> Task 6. 2.6 (contact sheet) -> Task 5. The "Testing" section's SIM/DEC/Live bullets map 1:1 onto Tasks 2-7's own test steps.
- **Resolved ambiguities** (one line each, also inline in the relevant task):
  - Where a command names both a search room and a final placement spot (s2026-003, s2026-004), the literal Where rule scans every known SPOT before any ROOM, so objects spawn at the named spot, not the named room — documented as intentional in Task 3's tests, not a deep-NLU distinction.
  - Spot-name text matching must try longest names first (`side_table_02` before `side_table`) so the shorter name's word-boundary regex doesn't false-match inside the longer name's own text; implemented as a `(-len(name), name)` sort key everywhere a spot/room/object name is matched against text.
  - `plan_scene`'s exact 4-arg signature from the spec is preserved; asset-manifest resolution is an additional optional `asset_root` kwarg (defaulting to the repo root) rather than a required 4th positional/keyword arg, since the spec's own text describes the manifest as something "looked up" internally, not passed in by the caller.
  - The manifest-referenced arena USD content hash was not present on disk during planning (only two independently-built copies with different hashes were); both gave byte-identical furniture placements, so Task 2 documents using whichever `arena.usd` exists rather than requiring the exact manifest hash.
  - `emit-scenario`'s "larger count wins" is resolved per `(asset, spot-or-room)` key across all supplied plans (not per literal SpawnItem id), and merged item ids are suffixed with `__<where>` to avoid id collisions across different plans/keys in a battery-wide merge — not spelled out at that level of detail in the spec.
- **Placeholder scan:** no TBD/TODO/"add appropriate handling" strings; every code step has complete, runnable code; every test asserts concrete values (including the exact hand-computed numbers for the two new person poses and four new spot surfaces).
- **Type consistency:** `SpawnItem`/`ScenePlan` field names and types are identical across Task 3 (definition), Task 4 (`apply_plan`/`clear_manifest`/`emit_scenario`/CLI), and Task 5 (JSON field names read back, matching `scene_plan_to_json`'s exact keys). `plan_scene`'s signature, `bench_env`'s env dict, and `run_tier2`'s new kwargs are used identically in Task 6's tests and its `gpsr_bench.py` CLI wiring.
