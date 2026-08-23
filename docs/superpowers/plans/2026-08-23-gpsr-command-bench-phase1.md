# GPSR Command Bench — Phase 1 (tiers 0 and 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A seeded, template-stratified corpus of official-grammar GPSR commands, and a `gpsr_bench` CLI that runs that corpus through the planner alone (tier 0) and through the full-mock orchestrator process (tier 1), producing a per-template pass matrix.

**Architecture:** Everything in this plan lives in the decision repo `/home/tinker/tk25_ws/src/tk25_decision` (branch `gpsr-two-layer-orchestrator`; do the work on a new branch `gpsr-command-bench` off it). A new `behavior_tree/GPSR/bench/` package holds four small modules — corpus generation (wraps the vendored `_official_cmd_gen.CommandGenerator`), telemetry parsing (`events.jsonl` → per-task results), tier runners, and the report — plus a `gpsr_bench.py` CLI. One real orchestrator bug is fixed on the way (`GPSR_OFFLINE_PLANNER` ignored by the two-layer planner). Tiers 2–3 (simulation) are a separate plan, written after this one's matrix exists.

**Tech Stack:** Python 3.10, `py_trees`, `openai` (OpenRouter), pytest 7, ROS 2 Humble (only for tier 1, which launches `ros2 run behavior_tree gpsr-orchestrator` as a subprocess).

**Spec:** `/home/tinker/tinker-sim/6.0.1/docs/superpowers/specs/2026-08-23-gpsr-command-variety-testing-design.md` (in tinker-sim, branch `worktree-gpsr-command-variety-spec`).

## Global Constraints

- Repo: `/home/tinker/tk25_ws/src/tk25_decision`. All paths below are relative to it unless absolute. Package root for imports is `src/behavior_tree` (module path `behavior_tree.GPSR...`).
- Test command (verified 2026-08-23; the venv has no pytest and pytest≥8 breaks a ROS plugin):
  ```bash
  cd /home/tinker/tk25_ws/src/tk25_decision
  PYTHONPATH=src/behavior_tree:src/gpsr_trace:src/gpsr_debug_server ROS2_PTH_WARNED=1 \
    uv run --python .venv_decision/bin/python --no-project --with "pytest<8" \
    python -m pytest <test paths> -q -p no:cacheprovider
  ```
  Abbreviated below as `$PYTEST <paths>`.
- Tests must not need the network: every test that touches the planner injects a fake planner or sets `BT_MOCK_MODE=true`.
- The installed package is a plain copy, not symlink-installed, so **tier 1 runs require** `cd /home/tinker/tk25_ws && colcon build --packages-select behavior_tree` after any source edit. Unit tests do not (they import from `src/`).
- Vocabulary source of truth is `GPSR_CONSTANTS_PATH` (default the sim file `src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json`): 8 `possible_poses`, 10 `possible_objects`, 4 `search_spots` rooms.
- Never print or commit API keys. `/home/tinker/tk25_ws/.env` is sourced by the operator, not read by code in this plan (the planner reads it itself via `GPSR/config.py`).
- Commit after every task with a conventional-commit message; do not push to `main`.

---

## File structure

| File | Responsibility |
|---|---|
| `src/behavior_tree/behavior_tree/GPSR/bench/__init__.py` | empty package marker |
| `src/behavior_tree/behavior_tree/GPSR/bench/corpus.py` | `CorpusEntry`, `build_knowledge()`, `expand_template()`, `generate_corpus()`, `write_jsonl()/read_jsonl()`, `FEASIBILITY`, `EDGE_COMMANDS` |
| `src/behavior_tree/behavior_tree/GPSR/bench/events.py` | `TaskResult`, `parse_events(path) -> dict[int, TaskResult]` |
| `src/behavior_tree/behavior_tree/GPSR/bench/tier0.py` | `run_tier0(entries, planner, knowledge, timeout_s) -> list[BenchResult]` |
| `src/behavior_tree/behavior_tree/GPSR/bench/tier1.py` | `run_tier1(entries, *, group_size, timeout_s, env, plan_dir, launcher) -> list[BenchResult]` — subprocess driver |
| `src/behavior_tree/behavior_tree/GPSR/bench/report.py` | `BenchResult`, `matrix(results)`, `write_report(results, out_dir)` |
| `src/behavior_tree/behavior_tree/GPSR/gpsr_bench.py` | CLI: `gen`, `tier0`, `tier1`, `report` sub-commands |
| `src/behavior_tree/behavior_tree/mock_config.bench.json` | all subsystems mocked, all nodes `IMMEDIATE` |
| `src/behavior_tree/behavior_tree/GPSR/planner.py:597` | one-line fix: honour `GPSR_OFFLINE_PLANNER` |
| `src/behavior_tree/test/test_gpsr_bench_*.py` | one test file per module |

---

### Task 1: Corpus module — deterministic, template-labelled generation

**Files:**
- Create: `src/behavior_tree/behavior_tree/GPSR/bench/__init__.py` (empty)
- Create: `src/behavior_tree/behavior_tree/GPSR/bench/corpus.py`
- Test: `src/behavior_tree/test/test_gpsr_bench_corpus.py`

**Interfaces:**
- Consumes: `behavior_tree.GPSR._official_cmd_gen.CommandGenerator` (constructor takes a knowledge object with attributes `names, locations, placement_locations, rooms, objects, object_categories_singular, object_categories_plural`; public helpers `templates`, `followup_templates`, `person_cmd_list`, `object_cmd_list`, `_resolve_followups(template, category)`, `_sample_followup(key, category)`, `_generate_followup(name, category)`, `insert_all_placeholders(s)`, `_resolve_duplicates(s)`, `_fix_articles(s)`; uses the global `random` module).
- Produces:
  ```python
  @dataclass(frozen=True)
  class CorpusEntry:
      id: str            # "c042-goToLoc-findPrs"
      seed: int
      template: str      # top-level template name
      followups: tuple[str, ...]
      category: str      # "people" | "objects"
      text: str
      feasibility: str   # "A" | "B" | "C"
  def build_knowledge(constants_path: Path) -> Knowledge
  def expand_template(gen: CommandGenerator, template: str, category: str) -> tuple[str, tuple[str, ...]]
  def generate_corpus(constants_path: Path, *, seed: int, per_template: int, templates: Sequence[str] | None = None) -> list[CorpusEntry]
  def write_jsonl(entries: Iterable[CorpusEntry], path: Path) -> None
  def read_jsonl(path: Path) -> list[CorpusEntry]
  FEASIBILITY: dict[str, str]          # template -> "A"/"B"/"C"
  EDGE_COMMANDS: list[CorpusEntry]     # hand-written adversarial entries, seed=-1
  ```

- [ ] **Step 1: Write the failing tests**

```python
# src/behavior_tree/test/test_gpsr_bench_corpus.py
import json
from pathlib import Path

import pytest

from behavior_tree.GPSR.bench import corpus

CONSTANTS = Path(__file__).resolve().parents[1] / "behavior_tree" / "GPSR" / "constants.rcw2026.json"
TOP_LEVEL = [
    "goToLoc", "takeObjFromPlcmt", "findPrsInRoom", "findObjInRoom", "meetPrsAtBeac",
    "countObjOnPlcmt", "countPrsInRoom", "tellPrsInfoInLoc", "tellObjPropOnPlcmt",
    "talkInfoToGestPrsInRoom", "followNameFromBeacToRoom", "guideNameFromBeacToBeac",
    "guidePrsFromBeacToBeac", "guideClothPrsFromBeacToBeac", "bringMeObjFromPlcmt",
    "tellCatPropOnPlcmt", "greetClothDscInRm", "greetNameInRm", "meetNameAtLocThenFindInRm",
    "countClothPrsInRoom", "tellPrsInfoAtLocToPrsAtLoc", "followPrsAtLoc",
]


def test_knowledge_comes_from_constants_file():
    kb = corpus.build_knowledge(CONSTANTS)
    raw = json.loads(CONSTANTS.read_text())
    assert set(kb.rooms) == {"kitchen", "laundry_room", "living_room", "bedroom"}
    assert set(kb.locations) == set(raw["possible_poses"]) - {"command_point"}
    assert set(kb.objects) == set(raw["possible_objects"])
    assert kb.placement_locations and set(kb.placement_locations) <= set(kb.locations)


def test_expand_template_returns_text_and_followup_names():
    kb = corpus.build_knowledge(CONSTANTS)
    gen = corpus.make_generator(kb, seed=1)
    text, followups = corpus.expand_template(gen, "goToLoc", "people")
    assert text and "{" not in text and "WARNING" not in text
    assert followups and set(followups) <= set(gen.followup_templates)


def test_generate_corpus_is_deterministic_for_a_seed():
    a = corpus.generate_corpus(CONSTANTS, seed=7, per_template=1)
    b = corpus.generate_corpus(CONSTANTS, seed=7, per_template=1)
    assert [e.text for e in a] == [e.text for e in b]
    c = corpus.generate_corpus(CONSTANTS, seed=8, per_template=1)
    assert [e.text for e in a] != [e.text for e in c]


def test_generate_corpus_covers_every_template_k_times():
    entries = corpus.generate_corpus(CONSTANTS, seed=3, per_template=2)
    counts = {}
    for e in entries:
        counts[e.template] = counts.get(e.template, 0) + 1
    assert set(counts) == set(TOP_LEVEL)
    assert all(n == 2 for n in counts.values())
    assert all(e.feasibility in {"A", "B", "C"} for e in entries)
    assert len({e.id for e in entries}) == len(entries)


def test_corpus_text_stays_inside_arena_vocabulary():
    kb = corpus.build_knowledge(CONSTANTS)
    entries = corpus.generate_corpus(CONSTANTS, seed=5, per_template=1)
    allowed = set(kb.rooms) | set(kb.locations)
    for e in entries:
        for word in ("office", "bathroom", "desk_lamp"):
            assert word not in e.text, (e.template, e.text)


def test_jsonl_roundtrip(tmp_path):
    entries = corpus.generate_corpus(CONSTANTS, seed=2, per_template=1)
    path = tmp_path / "corpus.jsonl"
    corpus.write_jsonl(entries, path)
    assert corpus.read_jsonl(path) == entries


def test_feasibility_map_covers_every_template_and_followup():
    kb = corpus.build_knowledge(CONSTANTS)
    gen = corpus.make_generator(kb, seed=0)
    assert set(corpus.FEASIBILITY) == set(gen.templates) | set(gen.followup_templates)


def test_edge_commands_are_labelled():
    assert corpus.EDGE_COMMANDS
    assert all(e.seed == -1 and e.template == "edge" for e in corpus.EDGE_COMMANDS)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_corpus.py`
Expected: FAIL with `ModuleNotFoundError: No module named 'behavior_tree.GPSR.bench'`

- [ ] **Step 3: Write the implementation**

```python
# src/behavior_tree/behavior_tree/GPSR/bench/__init__.py
"""GPSR command bench: corpus generation, tier runners, reporting."""
```

```python
# src/behavior_tree/behavior_tree/GPSR/bench/corpus.py
"""Seeded, template-labelled GPSR command corpus built on the vendored official generator."""
from __future__ import annotations

import json
import random
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence

from behavior_tree.GPSR._official_cmd_gen import CommandGenerator

_FOLLOWUP_RE = re.compile(r"\{FOLLOWUP:(\w+)\}")

# Feasibility in the rcw2026 simulation (spec section "Sim-feasibility classification").
# A: checkable with truth pose + announce events today.
# B: executable but the object-pose oracle is blocked until YCB rigid bodies resolve.
# C: unsupported sim content (moving actors, clothing, trash bin) or stubbed executor (place, open).
FEASIBILITY: dict[str, str] = {
    # top-level
    "goToLoc": "A", "findPrsInRoom": "A", "findObjInRoom": "A", "meetPrsAtBeac": "A",
    "countObjOnPlcmt": "A", "countPrsInRoom": "A", "tellObjPropOnPlcmt": "A",
    "tellCatPropOnPlcmt": "A", "talkInfoToGestPrsInRoom": "A", "greetClothDscInRm": "C",
    "greetNameInRm": "A", "meetNameAtLocThenFindInRm": "A",
    "takeObjFromPlcmt": "B", "bringMeObjFromPlcmt": "B",
    "tellPrsInfoInLoc": "C", "followNameFromBeacToRoom": "C", "guideNameFromBeacToBeac": "C",
    "guidePrsFromBeacToBeac": "C", "guideClothPrsFromBeacToBeac": "C",
    "countClothPrsInRoom": "C", "tellPrsInfoAtLocToPrsAtLoc": "C", "followPrsAtLoc": "C",
    # follow-ups
    "findObj": "A", "findPrs": "A", "meetName": "A", "talkInfo": "A",
    "takeObj": "B", "deliverObjToMe": "B", "deliverObjToPrsInRoom": "B", "deliverObjToNameAtBeac": "B",
    "placeObjOnPlcmt": "C", "putObjInTrash": "C", "followPrs": "C", "followPrsToRoom": "C",
    "guidePrsToBeacon": "C",
}
_RANK = {"A": 0, "B": 1, "C": 2}


@dataclass(frozen=True)
class Knowledge:
    names: list[str]
    locations: list[str]
    placement_locations: list[str]
    rooms: list[str]
    objects: list[str]
    object_categories_singular: list[str]
    object_categories_plural: list[str]


@dataclass(frozen=True)
class CorpusEntry:
    id: str
    seed: int
    template: str
    followups: tuple[str, ...]
    category: str
    text: str
    feasibility: str


def build_knowledge(constants_path: Path) -> Knowledge:
    raw = json.loads(Path(constants_path).read_text(encoding="utf-8"))
    rooms = [r for r in raw.get("search_spots", {}) if not r.startswith("_")]
    locations = [p for p in raw["possible_poses"] if p != "command_point" and p not in rooms]
    # Flat surfaces the grammar may place/count objects on: everything that is not a seat.
    placement = [p for p in locations if p not in {"sofa"}]
    return Knowledge(
        names=["Alex", "Sarah", "John", "Emma", "Liam", "Olivia"],
        locations=locations,
        placement_locations=placement,
        rooms=rooms,
        objects=[o for o in raw["possible_objects"] if not o.startswith("_")],
        object_categories_singular=["food", "drink", "kitchen item"],
        object_categories_plural=["foods", "drinks", "kitchen items"],
    )


def make_generator(kb: Knowledge, *, seed: int) -> CommandGenerator:
    random.seed(seed)
    return CommandGenerator(kb)


def expand_template(gen: CommandGenerator, template: str, category: str) -> tuple[str, tuple[str, ...]]:
    """Expand one named template exactly as generate_command_start does, recording follow-up names."""
    text = gen.templates[template]
    followups: list[str] = []
    while True:
        match = _FOLLOWUP_RE.search(text)
        if not match:
            break
        name = gen._sample_followup(match.group(1), category)
        followups.append(name)
        expanded = gen.followup_templates[name]
        text = text.replace(match.group(0), expanded, 1)
    text = gen.insert_all_placeholders(text)
    text = gen._resolve_duplicates(text)
    text = gen._fix_articles(text)
    return text, tuple(followups)


def _feasibility(template: str, followups: Sequence[str]) -> str:
    worst = max([FEASIBILITY[template], *[FEASIBILITY[f] for f in followups]], key=_RANK.__getitem__)
    return worst


def generate_corpus(
    constants_path: Path, *, seed: int, per_template: int, templates: Sequence[str] | None = None
) -> list[CorpusEntry]:
    kb = build_knowledge(constants_path)
    gen = make_generator(kb, seed=seed)
    people = [name for name, _ in gen.person_cmd_list]
    objects = [name for name, _ in gen.object_cmd_list]
    chosen = list(templates) if templates else sorted(set(people) | set(objects))
    entries: list[CorpusEntry] = []
    index = 0
    for template in chosen:
        category = "objects" if template in objects and template not in people else "people"
        for _ in range(per_template):
            if template in people and template in objects:
                category = random.choice(["people", "objects"])
            text, followups = expand_template(gen, template, category)
            entry_id = f"c{index:03d}-{template}" + ("-" + "-".join(followups) if followups else "")
            entries.append(CorpusEntry(
                id=entry_id, seed=seed, template=template, followups=followups,
                category=category, text=text, feasibility=_feasibility(template, followups),
            ))
            index += 1
    return entries


def write_jsonl(entries: Iterable[CorpusEntry], path: Path) -> None:
    with Path(path).open("w", encoding="utf-8") as fh:
        for entry in entries:
            fh.write(json.dumps(asdict(entry), ensure_ascii=False) + "\n")


def read_jsonl(path: Path) -> list[CorpusEntry]:
    entries = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if line.strip():
            raw = json.loads(line)
            raw["followups"] = tuple(raw["followups"])
            entries.append(CorpusEntry(**raw))
    return entries


def _edge(i: int, text: str, feasibility: str) -> CorpusEntry:
    return CorpusEntry(id=f"e{i:02d}", seed=-1, template="edge", followups=(), category="edge",
                       text=text, feasibility=feasibility)


EDGE_COMMANDS: list[CorpusEntry] = [
    _edge(0, "go to the kitchen table and find a person", "A"),            # run18 canary
    _edge(1, "head over to the sofa and tell me what you see", "A"),        # synonym the grammar never emits
    _edge(2, "bring me the soup from the laundry desk", "B"),              # object not at its default location
    _edge(3, "find a unicorn in the bedroom", "A"),                        # unknown object
    _edge(4, "guide Alex from the kitchen to the bedroom then to the sofa", "C"),  # two-hop guide
    _edge(5, "count how many mugs are on the shelf and then on the side table", "A"),  # two counts
]
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_corpus.py`
Expected: 8 passed. If `test_corpus_text_stays_inside_arena_vocabulary` fails because the generator's own placeholder lists (e.g. `{talk}`, `{persInfo}`) inject words, that is fine — the assertion only checks the three foreign location names; fix `build_knowledge` rather than the test.

- [ ] **Step 5: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git checkout -b gpsr-command-bench
git add src/behavior_tree/behavior_tree/GPSR/bench/ src/behavior_tree/test/test_gpsr_bench_corpus.py
git commit -m "feat(gpsr): seeded, template-labelled command corpus on the vendored generator"
```

---

### Task 2: Honour `GPSR_OFFLINE_PLANNER` in the two-layer planner

**Files:**
- Modify: `src/behavior_tree/behavior_tree/GPSR/planner.py:597` (`GPSRPlanner.__init__`, line `self._offline_mock = is_full_mock_mode()`)
- Test: `src/behavior_tree/test/test_gpsr_target_planner.py` (append)

**Interfaces:**
- Consumes: `behavior_tree.GPSR.orchestrator._offline_planner_enabled() -> bool` (already exists; reads env, falls back to `is_full_mock_mode()`). `planner.py` already imports from `.orchestrator` at line 61, so no new import cycle.
- Produces: `GPSRPlanner()` with `BT_MOCK_MODE=true` and `GPSR_OFFLINE_PLANNER=0` now makes real LLM calls (what tier 1 needs); with `GPSR_OFFLINE_PLANNER=1` or unset it behaves exactly as before.

- [ ] **Step 1: Write the failing test**

```python
# append to src/behavior_tree/test/test_gpsr_target_planner.py
def test_two_layer_planner_honours_offline_planner_override(monkeypatch):
    import behavior_tree.GPSR.orchestrator as orch
    import behavior_tree.GPSR.planner as planner_mod

    monkeypatch.setattr(orch, "is_full_mock_mode", lambda: True)
    monkeypatch.setattr(planner_mod, "is_full_mock_mode", lambda: True)

    monkeypatch.delenv("GPSR_OFFLINE_PLANNER", raising=False)
    assert planner_mod.GPSRPlanner()._offline_mock is True

    monkeypatch.setenv("GPSR_OFFLINE_PLANNER", "0")
    assert planner_mod.GPSRPlanner()._offline_mock is False

    monkeypatch.setenv("GPSR_OFFLINE_PLANNER", "1")
    assert planner_mod.GPSRPlanner()._offline_mock is True
```

- [ ] **Step 2: Run test to verify it fails**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_target_planner.py -k offline_planner_override`
Expected: FAIL on the `"0"` case (`assert True is False`).

- [ ] **Step 3: Apply the fix**

In `planner.py`, add `_offline_planner_enabled` to the existing `from .orchestrator import (...)` block at line 61, and replace line 597:

```python
        self._offline_mock = _offline_planner_enabled()
```

- [ ] **Step 4: Run the planner tests**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_target_planner.py src/behavior_tree/test/test_gpsr_start_gate.py`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/planner.py src/behavior_tree/test/test_gpsr_target_planner.py
git commit -m "fix(gpsr): two-layer planner honours GPSR_OFFLINE_PLANNER like the legacy planner"
```

---

### Task 3: Bench mock config — every subsystem mocked, nothing waits for a key

**Files:**
- Create: `src/behavior_tree/behavior_tree/mock_config.bench.json`
- Test: `src/behavior_tree/test/test_gpsr_bench_mock_config.py`

**Interfaces:**
- Consumes: `behavior_tree.config` loads the file named by `BT_MOCK_CONFIG` (`config.py:84-88`); `is_full_mock_mode()` is true only when `mock_mode.enabled` and every subsystem has `enabled: true` (`config.py:307-320`). Allowed node values: `NO_MOCK, KEYPRESS, TELEOP, IMMEDIATE`.
- Produces: a config file tier 1 points `BT_MOCK_CONFIG` at.

- [ ] **Step 1: Write the failing test**

```python
# src/behavior_tree/test/test_gpsr_bench_mock_config.py
import json
from pathlib import Path

BENCH = Path(__file__).resolve().parents[1] / "behavior_tree" / "mock_config.bench.json"
SHIPPED = BENCH.with_name("mock_config.json")


def _nodes(cfg):
    for sub in cfg["mock_mode"]["subsystems"].values():
        yield from sub.get("nodes", {}).items()


def test_bench_config_mocks_every_subsystem_immediately():
    cfg = json.loads(BENCH.read_text())
    assert cfg["mock_mode"]["enabled"] is True
    assert all(sub["enabled"] is True for sub in cfg["mock_mode"]["subsystems"].values())
    assert all(mode == "IMMEDIATE" for _, mode in _nodes(cfg)), dict(_nodes(cfg))


def test_bench_config_covers_the_same_nodes_as_the_shipped_config():
    bench = json.loads(BENCH.read_text())
    shipped = json.loads(SHIPPED.read_text())
    assert set(bench["mock_mode"]["subsystems"]) == set(shipped["mock_mode"]["subsystems"])
    assert {n for n, _ in _nodes(bench)} == {n for n, _ in _nodes(shipped)}


def test_loader_reports_full_mock_for_bench_config(monkeypatch):
    monkeypatch.setenv("BT_MOCK_MODE", "true")
    monkeypatch.setenv("BT_MOCK_CONFIG", str(BENCH))
    import importlib
    import behavior_tree.config as config
    importlib.reload(config)
    assert config.is_full_mock_mode() is True
```

- [ ] **Step 2: Run test to verify it fails**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_mock_config.py`
Expected: FAIL with `FileNotFoundError`.

- [ ] **Step 3: Generate the file from the shipped one**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree
python3 - <<'EOF'
import json
cfg = json.load(open("mock_config.json"))
cfg["description"] = "Bench mock configuration: every subsystem mocked, every node IMMEDIATE (no keypress)."
for sub in cfg["mock_mode"]["subsystems"].values():
    sub["enabled"] = True
    for node in sub.get("nodes", {}):
        sub["nodes"][node] = "IMMEDIATE"
json.dump(cfg, open("mock_config.bench.json", "w"), indent=2)
open("mock_config.bench.json", "a").write("\n")
EOF
```

- [ ] **Step 4: Run test to verify it passes**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_mock_config.py`
Expected: 3 passed. If `test_loader_reports_full_mock_for_bench_config` fails because `config.py` caches a singleton that `reload` does not rebuild, read `config.py` lines 60-100 and construct the config class directly with the path instead of reloading; keep the assertion.

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/mock_config.bench.json src/behavior_tree/test/test_gpsr_bench_mock_config.py
git commit -m "feat(gpsr): bench mock config with IMMEDIATE mocks for unattended runs"
```

---

### Task 4: Telemetry parser — `events.jsonl` → per-task results

**Files:**
- Create: `src/behavior_tree/behavior_tree/GPSR/bench/events.py`
- Test: `src/behavior_tree/test/test_gpsr_bench_events.py`

**Interfaces:**
- Consumes: telemetry envelopes written by `GPSR/telemetry.py` — one JSON object per line with keys `event_type`, `task_id` (`"<trajectory>/task-<slot>"` or null), `phase`, `payload`, `occurred_at`, `sequence`. Relevant events: `step.finished` payload `{step_index, action, params, outcome}`; `task.finished` payload `{status: "succeeded"|"failed"|"incomplete", reason, plan_index, plan_length, corrections}`; `planner.error` (any payload).
- Produces:
  ```python
  @dataclass
  class TaskResult:
      slot: int
      status: str | None          # None = no task.finished seen (timeout/crash)
      reason: str | None
      steps: list[tuple[str, str]]  # (action, outcome) in order
      planner_errors: int
      first_seen: str | None      # occurred_at of first event for the slot
      finished_at: str | None
  def parse_events(path: Path) -> dict[int, TaskResult]
  def slot_of(task_id: str | None) -> int | None
  ```

- [ ] **Step 1: Write the failing test**

```python
# src/behavior_tree/test/test_gpsr_bench_events.py
import json

from behavior_tree.GPSR.bench.events import TaskResult, parse_events, slot_of


def _ev(event_type, task_id=None, payload=None, at="2026-08-23T00:00:00Z", seq=0):
    return {"schema": "tinker.gpsr.telemetry", "schema_version": 1, "event_id": str(seq),
            "trajectory_id": "gpsr-x", "task_id": task_id, "trace_id": None, "source_id": "t",
            "sequence": seq, "occurred_at": at, "monotonic_ns": seq, "event_type": event_type,
            "phase": None, "parent_event_id": None, "causation_ids": [], "payload": payload or {}}


def test_slot_of_parses_task_suffix():
    assert slot_of("gpsr-x/task-3") == 3
    assert slot_of(None) is None
    assert slot_of("gpsr-x") is None


def test_parse_events_groups_by_slot_and_keeps_step_order(tmp_path):
    lines = [
        _ev("run.started", seq=0),
        _ev("step.finished", "gpsr-x/task-0", {"action": "goto", "outcome": "succeeded"}, "2026-08-23T00:00:01Z", 1),
        _ev("planner.error", "gpsr-x/task-1", {"error": "boom"}, seq=2),
        _ev("step.finished", "gpsr-x/task-0", {"action": "find_person", "outcome": "failed"}, seq=3),
        _ev("task.finished", "gpsr-x/task-0", {"status": "failed", "reason": "find_person(...) failed"}, "2026-08-23T00:00:05Z", 4),
        _ev("step.finished", "gpsr-x/task-1", {"action": "announce", "outcome": "succeeded"}, seq=5),
    ]
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(l) for l in lines) + "\n")

    results = parse_events(path)
    assert set(results) == {0, 1}
    t0 = results[0]
    assert t0 == TaskResult(slot=0, status="failed", reason="find_person(...) failed",
                            steps=[("goto", "succeeded"), ("find_person", "failed")],
                            planner_errors=0, first_seen="2026-08-23T00:00:01Z",
                            finished_at="2026-08-23T00:00:05Z")
    assert results[1].status is None
    assert results[1].planner_errors == 1


def test_parse_events_tolerates_partial_last_line(tmp_path):
    path = tmp_path / "events.jsonl"
    path.write_text(json.dumps(_ev("step.finished", "g/task-0", {"action": "goto", "outcome": "succeeded"})) + "\n{\"trunc")
    assert parse_events(path)[0].steps == [("goto", "succeeded")]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_events.py`
Expected: FAIL with `ModuleNotFoundError: ... bench.events`.

- [ ] **Step 3: Implement**

```python
# src/behavior_tree/behavior_tree/GPSR/bench/events.py
"""Fold a GPSR telemetry events.jsonl into per-task results."""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

_SLOT_RE = re.compile(r"/task-(\d+)$")


@dataclass
class TaskResult:
    slot: int
    status: str | None = None
    reason: str | None = None
    steps: list[tuple[str, str]] = field(default_factory=list)
    planner_errors: int = 0
    first_seen: str | None = None
    finished_at: str | None = None


def slot_of(task_id: str | None) -> int | None:
    if not task_id:
        return None
    match = _SLOT_RE.search(task_id)
    return int(match.group(1)) if match else None


def parse_events(path: Path) -> dict[int, TaskResult]:
    results: dict[int, TaskResult] = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue  # writer may be mid-line when we read
        slot = slot_of(event.get("task_id"))
        if slot is None:
            continue
        result = results.setdefault(slot, TaskResult(slot=slot))
        if result.first_seen is None:
            result.first_seen = event.get("occurred_at")
        payload = event.get("payload") or {}
        kind = event.get("event_type")
        if kind == "step.finished":
            result.steps.append((str(payload.get("action")), str(payload.get("outcome"))))
        elif kind == "planner.error":
            result.planner_errors += 1
        elif kind == "task.finished":
            result.status = payload.get("status")
            result.reason = payload.get("reason")
            result.finished_at = event.get("occurred_at")
    return results
```

- [ ] **Step 4: Run test to verify it passes**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_events.py`
Expected: 3 passed.

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/bench/events.py src/behavior_tree/test/test_gpsr_bench_events.py
git commit -m "feat(gpsr): parse telemetry events.jsonl into per-task bench results"
```

---

### Task 5: Result model and report — template × tier matrix

**Files:**
- Create: `src/behavior_tree/behavior_tree/GPSR/bench/report.py`
- Test: `src/behavior_tree/test/test_gpsr_bench_report.py`

**Interfaces:**
- Consumes: `CorpusEntry` (Task 1).
- Produces:
  ```python
  @dataclass
  class BenchResult:
      entry_id: str
      template: str
      feasibility: str
      tier: int
      verdict: str        # "PASS" | "FAIL" | "TIMEOUT" | "ERROR"
      detail: str         # reason / first failing step / validator message
      seconds: float
      plan: list[str]     # action names in order, if known
  def matrix(results: Iterable[BenchResult]) -> dict[str, dict[int, tuple[int, int]]]   # template -> tier -> (passed, total)
  def write_report(results: Iterable[BenchResult], out_dir: Path, *, corpus_path: Path | None = None) -> Path  # returns SUMMARY.md path; also writes report.json
  ```

- [ ] **Step 1: Write the failing test**

```python
# src/behavior_tree/test/test_gpsr_bench_report.py
import json

from behavior_tree.GPSR.bench.report import BenchResult, matrix, write_report


def _r(eid, template, tier, verdict, feas="A"):
    return BenchResult(entry_id=eid, template=template, feasibility=feas, tier=tier,
                       verdict=verdict, detail="", seconds=1.0, plan=["goto"])


def test_matrix_counts_pass_over_total_per_template_and_tier():
    m = matrix([_r("a", "goToLoc", 0, "PASS"), _r("b", "goToLoc", 0, "FAIL"),
                _r("c", "goToLoc", 1, "PASS"), _r("d", "countObjOnPlcmt", 0, "TIMEOUT")])
    assert m["goToLoc"][0] == (1, 2)
    assert m["goToLoc"][1] == (1, 1)
    assert m["countObjOnPlcmt"][0] == (0, 1)


def test_write_report_emits_json_and_markdown(tmp_path):
    results = [_r("a", "goToLoc", 0, "PASS"), _r("b", "takeObjFromPlcmt", 0, "FAIL", feas="B")]
    summary = write_report(results, tmp_path)
    assert summary.name == "SUMMARY.md"
    text = summary.read_text()
    assert "| goToLoc | A |" in text and "1/1" in text
    assert "| takeObjFromPlcmt | B |" in text and "0/1" in text
    data = json.loads((tmp_path / "report.json").read_text())
    assert data["results"][1]["verdict"] == "FAIL"
    assert data["totals"]["0"] == {"PASS": 1, "FAIL": 1}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_report.py`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement**

```python
# src/behavior_tree/behavior_tree/GPSR/bench/report.py
"""Bench result model and the template x tier pass matrix."""
from __future__ import annotations

import json
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterable

VERDICTS = ("PASS", "FAIL", "TIMEOUT", "ERROR")


@dataclass
class BenchResult:
    entry_id: str
    template: str
    feasibility: str
    tier: int
    verdict: str
    detail: str = ""
    seconds: float = 0.0
    plan: list[str] = field(default_factory=list)


def matrix(results: Iterable[BenchResult]) -> dict[str, dict[int, tuple[int, int]]]:
    table: dict[str, dict[int, list[int]]] = defaultdict(lambda: defaultdict(lambda: [0, 0]))
    for r in results:
        cell = table[r.template][r.tier]
        cell[1] += 1
        if r.verdict == "PASS":
            cell[0] += 1
    return {t: {tier: (p, n) for tier, (p, n) in tiers.items()} for t, tiers in table.items()}


def write_report(results: Iterable[BenchResult], out_dir: Path, *, corpus_path: Path | None = None) -> Path:
    results = list(results)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    tiers = sorted({r.tier for r in results})
    totals = {str(t): dict(Counter(r.verdict for r in results if r.tier == t)) for t in tiers}
    (out_dir / "report.json").write_text(json.dumps({
        "corpus": str(corpus_path) if corpus_path else None,
        "totals": totals,
        "results": [asdict(r) for r in results],
    }, indent=2) + "\n", encoding="utf-8")

    feas = {r.template: r.feasibility for r in results}
    m = matrix(results)
    lines = ["# GPSR bench summary", ""]
    if corpus_path:
        lines += [f"Corpus: `{corpus_path}`", ""]
    header = "| template | class | " + " | ".join(f"T{t}" for t in tiers) + " |"
    lines += [header, "|" + "---|" * (2 + len(tiers))]
    for template in sorted(m, key=lambda t: (feas[t], t)):
        cells = [f"{m[template][t][0]}/{m[template][t][1]}" if t in m[template] else "-" for t in tiers]
        lines.append(f"| {template} | {feas[template]} | " + " | ".join(cells) + " |")
    lines += ["", "## Totals", ""]
    for t in tiers:
        lines.append(f"- T{t}: " + ", ".join(f"{v} {totals[str(t)].get(v, 0)}" for v in VERDICTS))
    lines += ["", "## Failures", ""]
    for r in results:
        if r.verdict != "PASS":
            lines.append(f"- T{r.tier} `{r.entry_id}` **{r.verdict}** — {r.detail}")
    summary = out_dir / "SUMMARY.md"
    summary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return summary
```

- [ ] **Step 4: Run test to verify it passes**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_report.py`
Expected: 2 passed.

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/bench/report.py src/behavior_tree/test/test_gpsr_bench_report.py
git commit -m "feat(gpsr): bench result model and template x tier report"
```

---

### Task 6: Tier 0 runner — planner only

**Files:**
- Create: `src/behavior_tree/behavior_tree/GPSR/bench/tier0.py`
- Test: `src/behavior_tree/test/test_gpsr_bench_tier0.py`

**Interfaces:**
- Consumes: a planner with the `GPSRPlanner` surface used by `gpsr_ten_cmd/run_ten_cmd.py:185-195`: `split_command(command) -> list[dict]`, `request_plan_all(slot, targets, command=...)`, `all_targets_ready(slot, n) -> bool`, `get_action_plan(slot, i) -> list[dict]`, `reset()`. Validation via `behavior_tree.GPSR.planner_validators.validate_plan(plan, command, known_actions, known_locations=...) -> (ok, message)`. Known action names: `behavior_tree.GPSR.small_trees.ACTION_FACTORIES` keys.
- Produces:
  ```python
  def plan_one(planner, slot: int, command: str, *, timeout_s: float) -> tuple[list[dict], list[list[dict]]]
  def judge(entry: CorpusEntry, targets, plans, *, known_actions, known_locations, seconds) -> BenchResult
  def run_tier0(entries: Sequence[CorpusEntry], planner, *, known_actions, known_locations, timeout_s=90.0) -> list[BenchResult]
  ```
  Verdict rules: `ERROR` if `split_command` raised; `TIMEOUT` if not all targets ready in `timeout_s`; `FAIL` if any target's plan is empty or `validate_plan` rejects it; else `PASS`. `detail` carries the exception text / validator message / "empty plan for target i".

- [ ] **Step 1: Write the failing test**

```python
# src/behavior_tree/test/test_gpsr_bench_tier0.py
import os
os.environ.setdefault("BT_MOCK_MODE", "true")

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.tier0 import run_tier0

KNOWN_ACTIONS = {"goto", "find_person", "announce", "count", "find_object"}
KNOWN_LOCATIONS = {"kitchen_table", "sofa", "kitchen"}


class FakePlanner:
    def __init__(self, plans_by_command, raise_for=(), never_ready=()):
        self.plans = plans_by_command
        self.raise_for = set(raise_for)
        self.never_ready = set(never_ready)
        self._slots = {}

    def reset(self):
        self._slots.clear()

    def split_command(self, command):
        if command in self.raise_for:
            raise RuntimeError("llm down")
        return [{"id": f"t{i}", "desc": command} for i in range(len(self.plans[command]))]

    def request_plan_all(self, slot, targets, command=None):
        self._slots[slot] = command

    def all_targets_ready(self, slot, n):
        return self._slots[slot] not in self.never_ready

    def get_action_plan(self, slot, i):
        return self.plans[self._slots[slot]][i]


def _entry(i, text, template="goToLoc"):
    return CorpusEntry(id=f"c{i}", seed=1, template=template, followups=(), category="people",
                       text=text, feasibility="A")


def test_tier0_verdicts():
    plans = {
        "go to the sofa then find a person": [[{"action": "goto", "params": {"location": "sofa"}},
                                               {"action": "find_person", "params": {}}]],
        "empty one": [[]],
        "bad action": [[{"action": "teleport", "params": {}}]],
        "slow one": [[{"action": "announce", "params": {"text": "hi"}}]],
    }
    planner = FakePlanner(plans, raise_for={"crash"}, never_ready={"slow one"})
    entries = [_entry(0, "go to the sofa then find a person"), _entry(1, "empty one"),
               _entry(2, "bad action"), _entry(3, "slow one"), _entry(4, "crash")]
    plans["crash"] = [[]]
    results = run_tier0(entries, planner, known_actions=KNOWN_ACTIONS,
                        known_locations=KNOWN_LOCATIONS, timeout_s=0.2)
    verdicts = {r.entry_id: r.verdict for r in results}
    assert verdicts == {"c0": "PASS", "c1": "FAIL", "c2": "FAIL", "c3": "TIMEOUT", "c4": "ERROR"}
    assert results[0].plan == ["goto", "find_person"]
    assert "empty" in results[1].detail
    assert "llm down" in results[4].detail
    assert all(r.tier == 0 for r in results)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_tier0.py`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement**

```python
# src/behavior_tree/behavior_tree/GPSR/bench/tier0.py
"""Tier 0: run corpus commands through the two-layer planner only (no ROS, no execution)."""
from __future__ import annotations

import time
from typing import Iterable, Sequence

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.report import BenchResult
from behavior_tree.GPSR.planner_validators import validate_plan


def plan_one(planner, slot: int, command: str, *, timeout_s: float):
    targets = planner.split_command(command)
    planner.request_plan_all(slot, targets, command=command)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if planner.all_targets_ready(slot, len(targets)):
            break
        time.sleep(0.05)
    else:
        raise TimeoutError(f"planner not ready after {timeout_s:.0f}s")
    plans = [planner.get_action_plan(slot, i) for i in range(len(targets))]
    return targets, plans


def judge(entry: CorpusEntry, targets, plans, *, known_actions, known_locations, seconds) -> BenchResult:
    actions = [str(step.get("action")) for plan in plans for step in plan]
    base = dict(entry_id=entry.id, template=entry.template, feasibility=entry.feasibility,
                tier=0, seconds=seconds, plan=actions)
    if not targets:
        return BenchResult(verdict="FAIL", detail="split produced no targets", **base)
    for i, plan in enumerate(plans):
        if not plan:
            return BenchResult(verdict="FAIL", detail=f"empty plan for target {i}", **base)
        ok, message = validate_plan(plan, entry.text, known_actions, known_locations=known_locations)
        if not ok:
            return BenchResult(verdict="FAIL", detail=f"target {i}: {message}", **base)
    return BenchResult(verdict="PASS", **base)


def run_tier0(entries: Sequence[CorpusEntry], planner, *, known_actions: Iterable[str],
              known_locations: Iterable[str], timeout_s: float = 90.0) -> list[BenchResult]:
    known_actions = set(known_actions)
    known_locations = set(known_locations)
    results: list[BenchResult] = []
    for slot, entry in enumerate(entries):
        planner.reset()
        started = time.monotonic()
        try:
            targets, plans = plan_one(planner, slot, entry.text, timeout_s=timeout_s)
        except TimeoutError as exc:
            results.append(BenchResult(entry.id, entry.template, entry.feasibility, 0, "TIMEOUT",
                                       str(exc), time.monotonic() - started))
            continue
        except Exception as exc:  # noqa: BLE001 - any planner failure is an ERROR verdict
            results.append(BenchResult(entry.id, entry.template, entry.feasibility, 0, "ERROR",
                                       f"{type(exc).__name__}: {exc}", time.monotonic() - started))
            continue
        results.append(judge(entry, targets, plans, known_actions=known_actions,
                             known_locations=known_locations, seconds=time.monotonic() - started))
    return results
```

- [ ] **Step 4: Run test to verify it passes**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_tier0.py`
Expected: 1 passed. If `validate_plan` rejects the `c0` plan for a reason unrelated to this test (e.g. it requires `goto` before `find_person` with a location), adjust the fake plan in the test to satisfy the validator — read `planner_validators.py:226-330` — rather than weakening `judge`.

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/bench/tier0.py src/behavior_tree/test/test_gpsr_bench_tier0.py
git commit -m "feat(gpsr): tier-0 planner-only bench runner"
```

---

### Task 7: Tier 1 runner — drive the real orchestrator process in full mock

**Files:**
- Create: `src/behavior_tree/behavior_tree/GPSR/bench/tier1.py`
- Test: `src/behavior_tree/test/test_gpsr_bench_tier1.py`

**Interfaces:**
- Consumes: `parse_events` (Task 4), `BenchResult` (Task 5), `CorpusEntry` (Task 1). The orchestrator process: `ros2 run behavior_tree gpsr-orchestrator`, reads `BT_GPSR_CMD` (`|`-separated, `gpsr_orchestrator.py:104`), `BT_GPSR_PLAN_DIR` (telemetry lands at `<plan_dir>/debug/<trajectory_id>/events.jsonl`), `BT_MOCK_MODE`, `BT_MOCK_CONFIG`, `GPSR_OFFLINE_PLANNER`, `GPSR_CONSTANTS_PATH`. **It never exits on its own** (`Running("idle")`, `gpsr_orchestrator.py:150`) and never emits `run.finished`, so completion = one `task.finished` per slot.
- Produces:
  ```python
  DEFAULT_LAUNCHER = ["ros2", "run", "behavior_tree", "gpsr-orchestrator"]
  def bench_env(*, mock_config: Path, constants: Path, plan_dir: Path, commands: Sequence[str], live_llm: bool) -> dict[str, str]
  def run_group(entries: Sequence[CorpusEntry], *, env, plan_dir: Path, launcher: Sequence[str], timeout_s: float, poll_s: float = 1.0) -> list[BenchResult]
  def run_tier1(entries, *, group_size: int, timeout_s: float, mock_config: Path, constants: Path, plan_dir: Path, launcher=DEFAULT_LAUNCHER, live_llm=True) -> list[BenchResult]
  ```
  Verdict rules per slot: `task.finished.status == "succeeded"` → `PASS`; `"failed"`/`"incomplete"` → `FAIL` with `reason`; no `task.finished` before the deadline → `TIMEOUT` (and every later slot in the group also `TIMEOUT`, detail `"group timed out at slot N"`); process exited early → `ERROR` for unfinished slots with the exit code.

- [ ] **Step 1: Write the failing test** (uses a fake launcher script instead of ROS)

```python
# src/behavior_tree/test/test_gpsr_bench_tier1.py
import json
import os
import stat
import sys
import textwrap
from pathlib import Path

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.tier1 import bench_env, run_tier1


def _entry(i, text):
    return CorpusEntry(id=f"c{i}", seed=1, template="goToLoc", followups=(), category="people",
                       text=text, feasibility="A")


def _fake_orchestrator(tmp_path: Path, behaviour: str) -> list[str]:
    """A stand-in for `ros2 run behavior_tree gpsr-orchestrator` that writes telemetry then idles."""
    script = tmp_path / "fake_orch.py"
    script.write_text(textwrap.dedent(f"""
        import json, os, time, sys
        cmds = os.environ["BT_GPSR_CMD"].split("|")
        d = os.path.join(os.environ["BT_GPSR_PLAN_DIR"], "debug", "traj-1"); os.makedirs(d, exist_ok=True)
        f = open(os.path.join(d, "events.jsonl"), "a", buffering=1)
        def ev(t, slot, payload): f.write(json.dumps({{"event_type": t, "task_id": f"traj-1/task-{{slot}}", "payload": payload, "occurred_at": "x"}}) + "\\n")
        for slot, c in enumerate(cmds):
            if "{behaviour}" == "hang" and slot == 1:
                time.sleep(60)
            if "{behaviour}" == "crash" and slot == 1:
                sys.exit(3)
            ev("step.finished", slot, {{"action": "goto", "outcome": "succeeded"}})
            ev("task.finished", slot, {{"status": "failed" if "fail" in c else "succeeded", "reason": "r"}})
        while True:
            time.sleep(1)
    """))
    return [sys.executable, str(script)]


def test_bench_env_sets_the_orchestrator_switches(tmp_path):
    env = bench_env(mock_config=tmp_path / "m.json", constants=tmp_path / "c.json",
                    plan_dir=tmp_path / "runs", commands=["a", "b"], live_llm=True)
    assert env["BT_GPSR_CMD"] == "a|b"
    assert env["BT_MOCK_MODE"] == "true"
    assert env["GPSR_OFFLINE_PLANNER"] == "0"
    assert env["BT_MOCK_CONFIG"].endswith("m.json")
    assert env["GPSR_CONSTANTS_PATH"].endswith("c.json")
    assert env["BT_GPSR_PLAN_DIR"].endswith("runs")
    assert os.environ.get("PATH", "") == env.get("PATH", "")  # inherits the shell env


def test_tier1_scores_each_slot_and_stops_the_idle_process(tmp_path):
    entries = [_entry(0, "go to the sofa"), _entry(1, "please fail"), _entry(2, "go to the shelf")]
    results = run_tier1(entries, group_size=3, timeout_s=20, mock_config=tmp_path / "m.json",
                        constants=tmp_path / "c.json", plan_dir=tmp_path / "runs",
                        launcher=_fake_orchestrator(tmp_path, "ok"))
    assert [r.verdict for r in results] == ["PASS", "FAIL", "PASS"]
    assert results[1].detail == "r"
    assert results[0].plan == ["goto"]
    assert all(r.tier == 1 for r in results)


def test_tier1_times_out_a_hung_group_and_continues(tmp_path):
    entries = [_entry(i, f"cmd {i}") for i in range(4)]
    results = run_tier1(entries, group_size=2, timeout_s=3, mock_config=tmp_path / "m.json",
                        constants=tmp_path / "c.json", plan_dir=tmp_path / "runs",
                        launcher=_fake_orchestrator(tmp_path, "hang"))
    assert [r.verdict for r in results] == ["PASS", "TIMEOUT", "PASS", "TIMEOUT"]


def test_tier1_reports_a_crashed_process(tmp_path):
    entries = [_entry(0, "cmd 0"), _entry(1, "cmd 1")]
    results = run_tier1(entries, group_size=2, timeout_s=10, mock_config=tmp_path / "m.json",
                        constants=tmp_path / "c.json", plan_dir=tmp_path / "runs",
                        launcher=_fake_orchestrator(tmp_path, "crash"))
    assert [r.verdict for r in results] == ["PASS", "ERROR"]
    assert "exit code 3" in results[1].detail
```

- [ ] **Step 2: Run test to verify it fails**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_tier1.py`
Expected: FAIL with `ModuleNotFoundError`.

- [ ] **Step 3: Implement**

```python
# src/behavior_tree/behavior_tree/GPSR/bench/tier1.py
"""Tier 1: run corpus commands through the real orchestrator process with every ROS boundary mocked."""
from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Sequence

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.events import parse_events
from behavior_tree.GPSR.bench.report import BenchResult

DEFAULT_LAUNCHER = ["ros2", "run", "behavior_tree", "gpsr-orchestrator"]


def bench_env(*, mock_config: Path, constants: Path, plan_dir: Path, commands: Sequence[str],
              live_llm: bool) -> dict[str, str]:
    env = dict(os.environ)
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


def _events_file(plan_dir: Path, since: float) -> Path | None:
    debug = Path(plan_dir) / "debug"
    if not debug.is_dir():
        return None
    candidates = [p / "events.jsonl" for p in debug.iterdir() if (p / "events.jsonl").is_file()
                  and (p / "events.jsonl").stat().st_mtime >= since]
    return max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None


def _stop(proc: subprocess.Popen) -> None:
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def run_group(entries: Sequence[CorpusEntry], *, env: dict[str, str], plan_dir: Path,
              launcher: Sequence[str], timeout_s: float, poll_s: float = 1.0) -> list[BenchResult]:
    plan_dir = Path(plan_dir)
    plan_dir.mkdir(parents=True, exist_ok=True)
    log = (plan_dir / "bench-orchestrator.log").open("a", encoding="utf-8")
    started = time.time()
    proc = subprocess.Popen(list(launcher), env=env, stdout=log, stderr=subprocess.STDOUT, cwd=str(plan_dir))
    tasks = {}
    exit_code = None
    try:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            events = _events_file(plan_dir, started - 1)
            if events:
                tasks = parse_events(events)
                if all(tasks.get(i) and tasks[i].status for i in range(len(entries))):
                    break
            exit_code = proc.poll()
            if exit_code is not None:
                break
            time.sleep(poll_s)
    finally:
        _stop(proc)
        log.close()

    results: list[BenchResult] = []
    timed_out_at = None
    for slot, entry in enumerate(entries):
        task = tasks.get(slot)
        plan = [a for a, _ in task.steps] if task else []
        seconds = time.time() - started
        if task and task.status == "succeeded":
            verdict, detail = "PASS", ""
        elif task and task.status:
            verdict, detail = "FAIL", str(task.reason or task.status)
        elif exit_code is not None:
            verdict, detail = "ERROR", f"orchestrator exited with exit code {exit_code} before slot {slot} finished"
        else:
            if timed_out_at is None:
                timed_out_at = slot
            verdict, detail = "TIMEOUT", f"group timed out at slot {timed_out_at}"
        results.append(BenchResult(entry.id, entry.template, entry.feasibility, 1, verdict, detail, seconds, plan))
    return results


def run_tier1(entries: Sequence[CorpusEntry], *, group_size: int, timeout_s: float, mock_config: Path,
              constants: Path, plan_dir: Path, launcher: Sequence[str] = DEFAULT_LAUNCHER,
              live_llm: bool = True) -> list[BenchResult]:
    results: list[BenchResult] = []
    for start in range(0, len(entries), group_size):
        group = list(entries[start:start + group_size])
        group_dir = Path(plan_dir) / f"group-{start // group_size:03d}"
        env = bench_env(mock_config=mock_config, constants=constants, plan_dir=group_dir,
                        commands=[e.text for e in group], live_llm=live_llm)
        results.extend(run_group(group, env=env, plan_dir=group_dir, launcher=launcher, timeout_s=timeout_s))
    return results
```

- [ ] **Step 4: Run test to verify it passes**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_tier1.py`
Expected: 4 passed (the hang test takes ~3 s per group; total under 15 s).

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/bench/tier1.py src/behavior_tree/test/test_gpsr_bench_tier1.py
git commit -m "feat(gpsr): tier-1 bench runner drives the orchestrator process in full mock"
```

---

### Task 8: `gpsr_bench.py` CLI

**Files:**
- Create: `src/behavior_tree/behavior_tree/GPSR/gpsr_bench.py`
- Modify: `src/behavior_tree/setup.py` (add console script `gpsr-bench = behavior_tree.GPSR.gpsr_bench:main` next to the existing `gpsr-orchestrator` entry, `setup.py:58-63`)
- Test: `src/behavior_tree/test/test_gpsr_bench_cli.py`

**Interfaces:**
- Consumes: everything above. For tier 0 the real planner is `behavior_tree.GPSR.planner.GPSRPlanner()` (requires `OPENROUTER_API_KEY` in `~/tk25_ws/.env`, loaded by `GPSR/config.py` on import) and knowledge via `behavior_tree.GPSR.orchestrator.load_knowledge_from_constants(path)` then `orchestrator.KNOWN_LOCATIONS` keys; known actions via `behavior_tree.GPSR.small_trees.ACTION_FACTORIES` keys.
- Produces the commands:
  ```
  gpsr-bench gen   --seed 42 --per-template 3 [--constants PATH] [--edge] --out corpus.jsonl
  gpsr-bench tier0 --corpus corpus.jsonl --out DIR [--timeout 90] [--only-class A,B]
  gpsr-bench tier1 --corpus corpus.jsonl --out DIR [--group-size 3] [--timeout 300] [--offline-planner]
  gpsr-bench report --out DIR    (re-renders SUMMARY.md from DIR/report.json)
  ```
  `main(argv=None) -> int`; tier sub-commands write `report.json` + `SUMMARY.md` under `--out` and return 0 if every result is `PASS`, else 1.

- [ ] **Step 1: Write the failing test**

```python
# src/behavior_tree/test/test_gpsr_bench_cli.py
import json
import os
os.environ.setdefault("BT_MOCK_MODE", "true")
from pathlib import Path

from behavior_tree.GPSR import gpsr_bench
from behavior_tree.GPSR.bench.report import BenchResult

CONSTANTS = Path(__file__).resolve().parents[1] / "behavior_tree" / "GPSR" / "constants.rcw2026.json"


def test_gen_writes_a_corpus_with_every_template(tmp_path):
    out = tmp_path / "corpus.jsonl"
    assert gpsr_bench.main(["gen", "--seed", "1", "--per-template", "1", "--constants", str(CONSTANTS),
                            "--edge", "--out", str(out)]) == 0
    lines = [json.loads(l) for l in out.read_text().splitlines()]
    assert len({l["template"] for l in lines if l["template"] != "edge"}) == 22
    assert any(l["template"] == "edge" for l in lines)


def test_tier0_uses_injected_runner_and_writes_report(tmp_path, monkeypatch):
    corpus = tmp_path / "corpus.jsonl"
    gpsr_bench.main(["gen", "--seed", "1", "--per-template", "1", "--constants", str(CONSTANTS), "--out", str(corpus)])

    def fake_run(entries, planner, **kwargs):
        return [BenchResult(e.id, e.template, e.feasibility, 0, "PASS") for e in entries]

    monkeypatch.setattr(gpsr_bench, "run_tier0", fake_run)
    monkeypatch.setattr(gpsr_bench, "_make_planner", lambda: object())
    out = tmp_path / "t0"
    assert gpsr_bench.main(["tier0", "--corpus", str(corpus), "--out", str(out), "--constants", str(CONSTANTS)]) == 0
    assert (out / "SUMMARY.md").exists() and (out / "report.json").exists()


def test_only_class_filters_entries(tmp_path, monkeypatch):
    corpus = tmp_path / "corpus.jsonl"
    gpsr_bench.main(["gen", "--seed", "1", "--per-template", "1", "--constants", str(CONSTANTS), "--out", str(corpus)])
    seen = []

    def fake_run(entries, planner, **kwargs):
        seen.extend(entries)
        return [BenchResult(e.id, e.template, e.feasibility, 0, "FAIL", "x") for e in entries]

    monkeypatch.setattr(gpsr_bench, "run_tier0", fake_run)
    monkeypatch.setattr(gpsr_bench, "_make_planner", lambda: object())
    rc = gpsr_bench.main(["tier0", "--corpus", str(corpus), "--out", str(tmp_path / "o"),
                          "--constants", str(CONSTANTS), "--only-class", "A"])
    assert rc == 1
    assert seen and all(e.feasibility == "A" for e in seen)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_cli.py`
Expected: FAIL with `ImportError: cannot import name 'gpsr_bench'`.

- [ ] **Step 3: Implement**

```python
# src/behavior_tree/behavior_tree/GPSR/gpsr_bench.py
"""GPSR command bench CLI: generate a corpus, run it through a tier, render the matrix.

    gpsr-bench gen   --seed 42 --per-template 3 --edge --out corpus-42.jsonl
    gpsr-bench tier0 --corpus corpus-42.jsonl --out gpsr_runs/bench/t0-42
    gpsr-bench tier1 --corpus corpus-42.jsonl --out gpsr_runs/bench/t1-42 --group-size 3
    gpsr-bench report --out gpsr_runs/bench/t1-42
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from behavior_tree.GPSR.bench.corpus import EDGE_COMMANDS, generate_corpus, read_jsonl, write_jsonl
from behavior_tree.GPSR.bench.report import BenchResult, write_report
from behavior_tree.GPSR.bench.tier0 import run_tier0
from behavior_tree.GPSR.bench.tier1 import run_tier1

HERE = Path(__file__).resolve().parent
DEFAULT_CONSTANTS = HERE / "constants.rcw2026.json"
DEFAULT_MOCK_CONFIG = HERE.parent / "mock_config.bench.json"


def _make_planner():
    from behavior_tree.GPSR.planner import GPSRPlanner
    return GPSRPlanner()


def _knowledge(constants: Path):
    from behavior_tree.GPSR import orchestrator, small_trees
    orchestrator.load_knowledge_from_constants(str(constants))
    return set(small_trees.ACTION_FACTORIES), set(orchestrator.KNOWN_LOCATIONS)


def _filter(entries, only_class: str | None):
    if not only_class:
        return list(entries)
    allowed = {c.strip().upper() for c in only_class.split(",")}
    return [e for e in entries if e.feasibility in allowed]


def _finish(results, out: Path, corpus: Path) -> int:
    summary = write_report(results, out, corpus_path=corpus)
    print(summary.read_text())
    return 0 if results and all(r.verdict == "PASS" for r in results) else 1


def cmd_gen(args) -> int:
    entries = generate_corpus(Path(args.constants), seed=args.seed, per_template=args.per_template)
    if args.edge:
        entries = entries + list(EDGE_COMMANDS)
    write_jsonl(entries, Path(args.out))
    print(f"wrote {len(entries)} commands to {args.out}")
    return 0


def cmd_tier0(args) -> int:
    entries = _filter(read_jsonl(Path(args.corpus)), args.only_class)
    known_actions, known_locations = _knowledge(Path(args.constants))
    results = run_tier0(entries, _make_planner(), known_actions=known_actions,
                        known_locations=known_locations, timeout_s=args.timeout)
    return _finish(results, Path(args.out), Path(args.corpus))


def cmd_tier1(args) -> int:
    entries = _filter(read_jsonl(Path(args.corpus)), args.only_class)
    results = run_tier1(entries, group_size=args.group_size, timeout_s=args.timeout,
                        mock_config=Path(args.mock_config), constants=Path(args.constants),
                        plan_dir=Path(args.out) / "runs", live_llm=not args.offline_planner)
    return _finish(results, Path(args.out), Path(args.corpus))


def cmd_report(args) -> int:
    data = json.loads((Path(args.out) / "report.json").read_text(encoding="utf-8"))
    results = [BenchResult(**r) for r in data["results"]]
    corpus = Path(data["corpus"]) if data.get("corpus") else None
    print(write_report(results, Path(args.out), corpus_path=corpus).read_text())
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="gpsr-bench", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("gen", help="generate a seeded corpus")
    g.add_argument("--seed", type=int, required=True)
    g.add_argument("--per-template", type=int, default=3)
    g.add_argument("--constants", default=str(DEFAULT_CONSTANTS))
    g.add_argument("--edge", action="store_true", help="append the hand-written edge commands")
    g.add_argument("--out", required=True)
    g.set_defaults(func=cmd_gen)

    for name, func in (("tier0", cmd_tier0), ("tier1", cmd_tier1)):
        t = sub.add_parser(name)
        t.add_argument("--corpus", required=True)
        t.add_argument("--out", required=True)
        t.add_argument("--constants", default=str(DEFAULT_CONSTANTS))
        t.add_argument("--only-class", default=None, help="comma-separated feasibility classes, e.g. A,B")
        t.add_argument("--timeout", type=float, default=90.0 if name == "tier0" else 300.0)
        if name == "tier1":
            t.add_argument("--group-size", type=int, default=3)
            t.add_argument("--mock-config", default=str(DEFAULT_MOCK_CONFIG))
            t.add_argument("--offline-planner", action="store_true", help="GPSR_OFFLINE_PLANNER=1 (no LLM)")
        t.set_defaults(func=func)

    r = sub.add_parser("report", help="re-render SUMMARY.md from report.json")
    r.add_argument("--out", required=True)
    r.set_defaults(func=cmd_report)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
```

Add to `setup.py` `entry_points["console_scripts"]`, next to `gpsr-orchestrator`:

```python
            'gpsr-bench = behavior_tree.GPSR.gpsr_bench:main',
```

- [ ] **Step 4: Run tests**

Run: `$PYTEST src/behavior_tree/test/test_gpsr_bench_cli.py src/behavior_tree/test/test_gpsr_bench_corpus.py`
Expected: all pass. If `_knowledge` fails to import because `orchestrator.py` needs ROS message packages at import time, the test already stubs `_make_planner` but not `_knowledge`; in that case also `monkeypatch.setattr(gpsr_bench, "_knowledge", lambda p: (set(), set()))` in the two tier0 tests and note it.

- [ ] **Step 5: Commit**

```bash
git add src/behavior_tree/behavior_tree/GPSR/gpsr_bench.py src/behavior_tree/setup.py src/behavior_tree/test/test_gpsr_bench_cli.py
git commit -m "feat(gpsr): gpsr-bench CLI (gen / tier0 / tier1 / report)"
```

---

### Task 9: Run tier 0 and tier 1 on the seed-42 corpus and record the first matrix

**Files:**
- Create: `src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/corpus-42.jsonl`
- Create: `src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t0-42/{report.json,SUMMARY.md}`
- Create: `src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t1-42/{report.json,SUMMARY.md}` (not the `runs/` subfolder — it holds per-group logs, add it to `.gitignore`)
- Modify: `README.md` — add a "GPSR command bench" section (commands below, one paragraph on reading the matrix)

**Interfaces:** none new; this task produces data.

- [ ] **Step 1: Generate the corpus (3 per template + edge = 72 commands)**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
PYTHONPATH=src/behavior_tree .venv_decision/bin/python -m behavior_tree.GPSR.gpsr_bench gen \
  --seed 42 --per-template 3 --edge --out src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/corpus-42.jsonl
```
Expected: `wrote 72 commands to ...`. Skim the file: every line has a `template`, no `WARNING` text, no `{placeholder}` left.

- [ ] **Step 2: Run tier 0 (live LLM; ~2 s/command, a few cents total)**

```bash
set -a; source /home/tinker/tk25_ws/.env; set +a
PYTHONPATH=src/behavior_tree .venv_decision/bin/python -m behavior_tree.GPSR.gpsr_bench tier0 \
  --corpus src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/corpus-42.jsonl \
  --out src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t0-42
```
Expected: SUMMARY.md printed with 22 template rows + `edge`; exit code 0 or 1. Do not "fix" failing commands here — record them.

- [ ] **Step 3: Rebuild and run tier 1 (needs ROS sourced; robot not needed)**

```bash
cd /home/tinker/tk25_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select behavior_tree && source install/setup.bash
cd src/tk25_decision
set -a; source /home/tinker/tk25_ws/.env; set +a
export ROS_DOMAIN_ID=77   # any free domain; nothing real listens
PYTHONPATH=src/behavior_tree python3 -m behavior_tree.GPSR.gpsr_bench tier1 \
  --corpus src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/corpus-42.jsonl \
  --out src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t1-42 --group-size 3 --timeout 300
```
Expected: 24 groups; each group's `runs/group-NNN/bench-orchestrator.log` exists; SUMMARY.md has one verdict per command. If the very first group is `TIMEOUT` with an empty `events.jsonl`, the orchestrator is blocked before task 0 — read `runs/group-000/bench-orchestrator.log`; the usual cause is a mocked node still set to `KEYPRESS` (fix Task 3's config) or `create_enter_arena()` waiting on a door detection not covered by the bench config (add that node to the config as `IMMEDIATE`).

- [ ] **Step 4: Write the README section and commit the data**

Append to `README.md`:

```markdown
## GPSR command bench

Generates a seeded corpus of official-grammar GPSR commands for the rcw2026 sim vocabulary and
scores it per template at increasing realism. Tier 0 = planner only (LLM, no ROS); tier 1 = the real
`gpsr-orchestrator` process with every ROS boundary mocked (`mock_config.bench.json`). Tiers 2/3
(simulation) live in tinker-sim.

    gpsr-bench gen   --seed 42 --per-template 3 --edge --out corpus-42.jsonl
    gpsr-bench tier0 --corpus corpus-42.jsonl --out gpsr_runs/bench/t0-42
    gpsr-bench tier1 --corpus corpus-42.jsonl --out gpsr_runs/bench/t1-42   # after colcon build

Read `SUMMARY.md`: rows are templates (class A/B/C = sim feasibility, see the spec), cells are
passed/total per tier. A template that passes T0 but fails T1 is an executor/BT problem; one that
fails T0 is a planner-prompt problem. Committed baselines: `GPSR/gpsr_runs/bench/`.
```

```bash
echo "src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/*/runs/" >> .gitignore
git add .gitignore README.md src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/
git commit -m "docs(gpsr): first command-bench baseline (seed 42) for tiers 0 and 1"
```

- [ ] **Step 5: Report the matrix back**

Paste the two SUMMARY.md tables into the final report, and list the top three failing templates with their `detail` strings. That list is the input to the tier 2/3 plan.

---

## Self-review

**Spec coverage (Phase 1 items):** corpus generator + stratification + edge corpus → Task 1; per-template reporting → Task 5; T0 runner → Task 6; T1 runner with `BT_GPSR_CMD` grouping, timeout, SIGINT → Task 7; `GPSR_OFFLINE_PLANNER` fix → Task 2; IMMEDIATE mock config → Task 3; telemetry parsing → Task 4; CLI → Task 8; first matrix + docs → Task 9. Phase 2–4 items (bench scenario, regions, oracle, `/reset_simulation` driver, `scripts/gpsr-bench`) are deliberately out of this plan and go in the tier 2/3 plan.

**Known deviation from the spec:** the spec placed the corpus at `GPSR/gpsr_runs/run_battery_commands.py` to un-break `test_gpsr_battery_contract.py`. That test expects a hand-written `COMMANDS`/`EXPECT` battery and a `validate_target_contract` function — a different artifact. The corpus lives in `GPSR/bench/corpus.py` instead and the battery test stays broken (pre-existing, tracked separately).

**Type consistency:** `CorpusEntry` fields (`id, seed, template, followups, category, text, feasibility`) are used identically in Tasks 6, 7, 8. `BenchResult` positional order `(entry_id, template, feasibility, tier, verdict, detail, seconds, plan)` is used positionally in Tasks 6 and 7 and by keyword in Task 8's tests. `TaskResult.status/reason/steps` consumed in Task 7 match Task 4.
