# Event-Driven Contact Sheets + Judge Sheet Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Contact sheets show one row per completed milestone BT node (nav / vision / audio / manipulation) with kind label + outcome info; a separate judge-sheet.jpg documents the orchestrator's judging machinery (pre/postcondition gates, supervisor barriers, replan/correction cycle) plus the bench verdict.

**Architecture:** The orchestrator's telemetry (`debug/gpsr-*/events.jsonl`) already carries per-node status transitions with feedback, wall timestamps, and an id→name map (`tree.generated`). The recorder gains a per-frame wall-time index so events join to frames exactly. The sheet builder becomes event-driven, with the current time-strip as fallback when telemetry is absent; the judge sheet renders from the same telemetry (gate/replan events) plus run.json + announcements.txt.

**Tech Stack:** Pure PIL + json (no ROS) in the sheet tools; rclpy only in the recorder node wrapper (unchanged pattern).

**User decisions (2026-08-28):** judge sheet = verdict-evidence sheet AND the in-run node-judges + replan correction nodes; old time-strip survives only as fallback.

## Global Constraints

- Sheet building must NEVER crash a bench run: any missing input (no events.jsonl, no index.jsonl, unparseable lines) degrades, never raises (existing bench contract).
- Vertical layout: 640 px wide minimum content columns; sheets read top-to-bottom (user request, commit a9921c0).
- Old runs (no frames/index.jsonl) must still build: frame pick falls back to linear wall↔sim interpolation from recorder-meta.json.
- No orchestrator (tk25_decision runtime) changes — telemetry already suffices.
- judge-sheet.jpg is written by default next to --out (no new bench wiring); --no-judge-sheet disables.
- Keepalive announce nodes ("nav keepalive", "say keepalive", "keepalive gap") are noise, never milestone rows.
- Tests run ROS-free (as today: tests/test_contact_sheet.py, tests/test_gpsr_run_recorder.py).

---

### Task 1: Recorder per-frame index

**Files:**
- Modify: `validation/gpsr_run_recorder.py` (FrameSink)
- Test: `tests/test_gpsr_run_recorder.py`

**Interfaces:**
- Produces: `<run_dir>/frames/index.jsonl`, one line per accepted frame:
  `{"label": "head", "file": "frames/head/0004_849666.jpg", "stamp_s": 849.666, "wall": "2026-08-28T10:52:33.579091+00:00"}`
- FrameSink.offer() gains keyword `wall_iso: str` (caller passes `datetime.datetime.now(datetime.timezone.utc).isoformat()`), appends the line on ACCEPTED frames only.

Steps: failing test (offer twice, one accepted → one index line with matching file/stamp/wall) → implement (append via `open(..., "a")`, create parent dir; never raise on IO error — wrap in try/except, count drops in meta) → tests pass → commit.

### Task 2: Telemetry event extraction (pure module)

**Files:**
- Create: `tools/sheet_events.py`
- Test: `tests/test_sheet_events.py`

**Interfaces (produces, consumed by Tasks 3+4):**
```python
@dataclass
class MilestoneEvent:
    wall: str          # occurred_at ISO
    kind: str          # "NAV" | "VISION" | "AUDIO" | "MANIP"
    name: str          # human node name from tree.generated
    status: str        # "SUCCESS" | "FAILURE"
    info: str          # feedback, trimmed
@dataclass
class JudgeEvent:
    wall: str
    kind: str          # "PRECONDITION" | "POSTCONDITION" | "SUPERVISOR" | "REPLAN" | "CORRECTION"
    name: str
    status: str
    info: str
def load_run_telemetry(run_dir: Path) -> tuple[list[MilestoneEvent], list[JudgeEvent], dict]:
    """Newest debug/gpsr-*/events.jsonl; ({},[],[]) shapes on absence. dict = meta
    (trajectory_id, tree_revisions: int, run_finished payload or {})."""
```
- id→name from every `tree.generated` payload (`nodes` entries carry id and name; later revisions merge over earlier).
- Milestone classification by node-name pattern (order matters, first match):
  - exclude: names containing "keepalive"
  - NAV: startswith "goto target"
  - VISION: startswith one of ("scan ", "scan to", "vlm count", "count detections", "detect ", "track ") or == "turn pantilt"
  - AUDIO: startswith ("announce", "listen", "nav keepalive"-excluded above, "say ")
  - MANIP: contains one of ("arm", "grasp", "tuck", "gripper", "place")
  - AUDIO announce rows carry info = the announced text (feedback "Finished announcing X." → "X")
- Judge classification: name startswith "precondition gate" / "postcondition gate" / "supervisor barrier" → those kinds; a `tree.generated` with `tree_revision` > 0 emits REPLAN (info = "tree revision N"); names containing "correction" (e.g. "reset correction" excluded — only status FAILURE/SUCCESS on non-reset correction nodes) → CORRECTION.
- Only transitions to SUCCESS/FAILURE enter lists (events.jsonl node entries are already changes; filter status in {"SUCCESS","FAILURE"}).

Steps: failing tests with a fixture events.jsonl (hand-built ~10 lines: tree.generated rev0 + rev1, goto SUCCESS, scan FAILURE w/ "no matches", announce SUCCESS "Finished announcing 0 persons.", postcondition gate FAILURE "postcondition unmet: ...", keepalive SUCCESS excluded) → implement → pass → commit.

### Task 3: Event-driven sheet renderer

**Files:**
- Modify: `tools/contact_sheet.py`
- Test: `tests/test_contact_sheet.py`

**Interfaces:**
- `build_sheet(run_dir, meta, out, columns=12)` unchanged signature. Internally: milestones, judge_events, tmeta = `sheet_events.load_run_telemetry(run_dir)`; if milestones non-empty → event layout, else → current time-strip layout (fallback, existing code paths kept).
- Event layout per row: `[arena 320px | head 320px | label 320px]` = 960 px wide. Label block: line 1 `HH:MM:SS  KIND` (kind color: NAV #1565c0, VISION #6a1b9a, AUDIO #2e7d32, MANIP #ef6c00; FAILURE rows tint background #ffebee), line 2 node name + status, lines 3+ wrapped info (max 4 lines).
- Frame pick: `frames/index.jsonl` → per label, frame with min |wall(frame) − wall(event)| (parse ISO). Missing index → interpolate: stamp ≈ first_stamp + (wall_event − started_wall)/(ended_wall − started_wall) × (last_stamp − first_stamp), pick nearest by filename stamp. Missing frames dir → grey tile.
- Row cap 40: sample_evenly over milestones.
- Header band unchanged (id, verdict, seconds, command).

Steps: failing tests (fixture run dir with index.jsonl + 3 fake frames + fixture events → assert 960 px wide, row count = milestone count, fallback still works when no events) → implement → pass → regenerate attempt12 sheet manually as smoke → commit.

### Task 4: Judge sheet renderer

**Files:**
- Modify: `tools/contact_sheet.py` (add `build_judge_sheet`, call from main unless --no-judge-sheet)
- Test: `tests/test_contact_sheet.py`

**Interfaces:**
- `build_judge_sheet(run_dir, meta, out) -> Path | None` — None (skip, no file) when telemetry absent.
- Content, top to bottom (960 px wide):
  1. Header: run id, VERDICT (colored), seconds, command text, bench detail line.
  2. "Plan" block: numbered steps from run.json `plan` if present (else omitted).
  3. Judge-event rows (same 3-column row layout as Task 3, frames + label) for every JudgeEvent — gates, supervisor barriers, REPLAN markers (REPLAN rows: no frames needed, full-width band "REPLAN — tree revision N" #b71c1c), CORRECTION rows.
  4. "Transcript" block: deduped announcements.txt lines (cap 25, each prefixed "🗣"), grey box.
- Bench wiring: none (written next to --out as `judge-sheet.jpg` by default).

Steps: failing tests (judge sheet exists + None-skip without telemetry; REPLAN band present for rev>0 fixture) → implement → pass → regenerate attempt12 judge sheet as smoke → commit.

### Task 5: Validate on real run data + deliver

**Files:** none (validation)

- Run `python3 tools/contact_sheet.py --run-dir <attempt12 dir> --meta run.json --out sheet.jpg` (writes sheet.jpg + judge-sheet.jpg; old-run path exercises the interpolation fallback since index.jsonl doesn't exist yet).
- Eyeball both, SendUserFile to the user for review.
- Full test suite: `tests/test_contact_sheet.py tests/test_sheet_events.py tests/test_gpsr_run_recorder.py`.
- Commit any polish; note in ledger. New-run exactness (index.jsonl) validates on the next battery run whenever the stack next comes up — not gated on GPU.
