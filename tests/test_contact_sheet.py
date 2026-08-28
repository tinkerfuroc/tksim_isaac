import datetime
import json
import sys
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.contact_sheet import (  # noqa: E402
    build_sheet,
    build_judge_sheet,
    sample_evenly,
    select_event_rows,
    _stamp_from_name,
    _dedup_transcript,
    REPLAN_BAND_COLOR,
)


def test_sample_evenly():
    assert sample_evenly([1, 2, 3], 5) == [1, 2, 3]
    assert sample_evenly(list(range(10)), 4) == [0, 3, 6, 9]
    assert sample_evenly([], 4) == []


def _mk_frames(d, label, n, size=(32, 18)):
    p = d / "frames" / label
    p.mkdir(parents=True)
    for i in range(n):
        Image.new("RGB", size, (i * 10 % 255, 80, 80)).save(p / f"{i:04d}_{i*1000}.jpg")


def test_build_sheet_is_portrait_two_columns(tmp_path):
    _mk_frames(tmp_path, "arena", 20)
    _mk_frames(tmp_path, "head", 20)
    meta = {
        "id": "c001",
        "text": "go to the kitchen table",
        "verdict": "PASS",
        "seconds": 93.2,
        "tier": "T2",
    }
    out = build_sheet(tmp_path, meta, tmp_path / "sheet.jpg", columns=12)
    img = Image.open(out)
    # Vertical layout: one column per camera (arena | head), time flowing
    # downward — 12 sampled tile rows under header + camera-label band.
    assert img.width == 2 * 320
    assert img.height > 88 + 18 + 12 * 18  # header + label band + 12 captioned rows
    assert img.height > img.width  # the sheet reads top-to-bottom


def test_build_sheet_short_column_pads_with_placeholders(tmp_path):
    # head has fewer frames than the sample count: its column ends in grey
    # placeholder tiles rather than truncating the arena column.
    _mk_frames(tmp_path, "arena", 20)
    _mk_frames(tmp_path, "head", 3)
    out = build_sheet(tmp_path, {"id": "x", "verdict": "PASS"}, tmp_path / "s.jpg")
    img = Image.open(out)
    assert img.width == 2 * 320
    assert img.height > 88 + 18 + 12 * 18  # arena still gets its 12 rows


def test_build_sheet_missing_label_degrades(tmp_path):
    _mk_frames(tmp_path, "head", 2)
    out = build_sheet(tmp_path, {"id": "x", "verdict": "ERROR"}, tmp_path / "s.jpg")
    assert out.exists()  # no crash; arena column is grey placeholder tiles


def test_stamp_from_name_parses_seq_ms_filename():
    assert _stamp_from_name("0007_1500.jpg") == "1.5"


def test_stamp_from_name_degrades_on_unparseable_name():
    # No trailing `_<ms>` integer -- must not raise (C3's "never a crash").
    assert _stamp_from_name("not-a-frame-name.jpg") == "?"
    assert _stamp_from_name("frame.jpg") == "?"


def test_build_sheet_survives_unparseable_frame_filename(tmp_path):
    # A stray non-conforming file in frames/head/ must not crash the
    # builder -- it degrades to a "t=?s" caption instead.
    p = tmp_path / "frames" / "head"
    p.mkdir(parents=True)
    Image.new("RGB", (32, 18), (10, 80, 80)).save(p / "not-a-frame-name.jpg")
    out = build_sheet(tmp_path, {"id": "x", "verdict": "ERROR"}, tmp_path / "s.jpg")
    assert out.exists()


# --- event-driven layout ------------------------------------------------
#
# Below: milestones = one NAV SUCCESS, one VISION FAILURE, one AUDIO
# SUCCESS -- built directly as a synthetic debug/gpsr-*/events.jsonl,
# matching the shape tools/sheet_events.py's load_run_telemetry() reads
# (see tests/test_sheet_events.py for the exhaustive event-shape tests;
# here we only need enough to exercise the sheet's frame-picking paths).

_EVENT_WALLS = (
    "2026-08-28T10:00:01.000000Z",
    "2026-08-28T10:00:02.000000Z",
    "2026-08-28T10:00:03.000000Z",
)


def _mk_events(run_dir):
    """Write a minimal events.jsonl: NAV SUCCESS, VISION FAILURE, AUDIO SUCCESS."""
    debug_dir = run_dir / "debug" / "gpsr-20260828T100000000000Z-fixture"
    debug_dir.mkdir(parents=True)
    nodes = [
        {"id": "n_goto", "node_id": "n_goto", "name": "goto target"},
        {"id": "n_scan", "node_id": "n_scan", "name": "scan to count"},
        {"id": "n_announce", "node_id": "n_announce", "name": "announce vlm count"},
    ]
    events = [
        {
            "trajectory_id": "traj-1",
            "occurred_at": "2026-08-28T10:00:00.000000Z",
            "event_type": "tree.generated",
            "payload": {"tree_revision": 0, "nodes": nodes},
        },
        {
            "trajectory_id": "traj-1",
            "occurred_at": _EVENT_WALLS[0],
            "event_type": "tree.node_states_changed",
            "payload": {
                "nodes": [
                    {"id": "n_goto", "node_id": "n_goto", "status": "SUCCESS", "feedback": "goal accepted"}
                ]
            },
        },
        {
            "trajectory_id": "traj-1",
            "occurred_at": _EVENT_WALLS[1],
            "event_type": "tree.node_states_changed",
            "payload": {
                "nodes": [
                    {"id": "n_scan", "node_id": "n_scan", "status": "FAILURE", "feedback": "no matches for target"}
                ]
            },
        },
        {
            "trajectory_id": "traj-1",
            "occurred_at": _EVENT_WALLS[2],
            "event_type": "tree.node_states_changed",
            "payload": {
                "nodes": [
                    {
                        "id": "n_announce",
                        "node_id": "n_announce",
                        "status": "SUCCESS",
                        "feedback": "Finished announcing 0 persons.",
                    }
                ]
            },
        },
    ]
    events_file = debug_dir / "events.jsonl"
    with events_file.open("w") as f:
        for e in events:
            f.write(json.dumps(e) + "\n")
    return debug_dir


def _mk_events_n(run_dir, n):
    """Write n synthetic NAV SUCCESS milestones, 1s apart, for cap testing."""
    debug_dir = run_dir / "debug" / "gpsr-20260828T100000000000Z-fixture"
    debug_dir.mkdir(parents=True)
    nodes = [{"id": f"n_{i}", "node_id": f"n_{i}", "name": "goto target"} for i in range(n)]
    events = [
        {
            "trajectory_id": "traj-1",
            "occurred_at": "2026-08-28T10:00:00.000000Z",
            "event_type": "tree.generated",
            "payload": {"tree_revision": 0, "nodes": nodes},
        }
    ]
    for i in range(n):
        wall = (
            datetime.datetime(2026, 8, 28, 10, 0, 1, tzinfo=datetime.timezone.utc)
            + datetime.timedelta(seconds=i)
        ).isoformat().replace("+00:00", "Z")
        events.append(
            {
                "trajectory_id": "traj-1",
                "occurred_at": wall,
                "event_type": "tree.node_states_changed",
                "payload": {
                    "nodes": [{"id": f"n_{i}", "node_id": f"n_{i}", "status": "SUCCESS", "feedback": "ok"}]
                },
            }
        )
    events_file = debug_dir / "events.jsonl"
    with events_file.open("w") as f:
        for e in events:
            f.write(json.dumps(e) + "\n")
    return debug_dir


def _mk_index(run_dir, label, n, start_wall):
    """Append n frames/index.jsonl lines for `label`'s already-written frame
    files, wall timestamps 1s apart from start_wall (a tz-aware datetime).
    """
    frames_dir = run_dir / "frames" / label
    files = sorted(frames_dir.glob("*.jpg"))
    index_path = run_dir / "frames" / "index.jsonl"
    with index_path.open("a") as f:
        for i, file in enumerate(files[:n]):
            wall = (start_wall + datetime.timedelta(seconds=i)).isoformat().replace("+00:00", "Z")
            entry = {"label": label, "file": f"frames/{label}/{file.name}", "stamp_s": float(i), "wall": wall}
            f.write(json.dumps(entry) + "\n")


def test_build_sheet_event_layout_with_index(tmp_path):
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    start = datetime.datetime(2026, 8, 28, 9, 59, 58, tzinfo=datetime.timezone.utc)
    _mk_index(tmp_path, "arena", 5, start)
    _mk_index(tmp_path, "head", 5, start)
    _mk_events(tmp_path)
    meta = {"id": "c001", "text": "count people", "verdict": "PASS", "seconds": 12.0, "tier": "T2"}
    out = build_sheet(tmp_path, meta, tmp_path / "sheet.jpg")
    assert out.exists()
    img = Image.open(out)
    # 3 tile columns (arena | head | label block) now, not the fallback's 2.
    assert img.width == 3 * 320
    # 3 milestone rows under the header band -- taller than a 1-row sheet
    # would be.
    assert img.height > 88 + 18 + 3 * 40


def test_build_sheet_event_layout_interpolates_without_index(tmp_path):
    # No frames/index.jsonl -- exercises the recorder-meta.json
    # interpolation fallback for frame picking.
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    _mk_events(tmp_path)
    recorder_meta = {
        "labels": {
            "arena": {"frames": 5, "first_stamp": 0.0, "last_stamp": 4.0},
            "head": {"frames": 5, "first_stamp": 0.0, "last_stamp": 4.0},
        },
        "started_wall": "2026-08-28T09:59:58+00:00",
        "ended_wall": "2026-08-28T10:00:05+00:00",
    }
    (tmp_path / "recorder-meta.json").write_text(json.dumps(recorder_meta))
    out = build_sheet(tmp_path, {"id": "c002", "verdict": "PASS"}, tmp_path / "sheet2.jpg")
    assert out.exists()
    img = Image.open(out)
    assert img.width == 3 * 320
    assert img.height > 88 + 18 + 3 * 40


def test_build_sheet_falls_back_to_time_strip_without_events(tmp_path):
    # No debug/gpsr-*/events.jsonl at all -- milestones is empty, so the
    # sheet must fall back to the original two-column time-strip layout.
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    out = build_sheet(tmp_path, {"id": "x", "verdict": "PASS"}, tmp_path / "s3.jpg")
    img = Image.open(out)
    assert img.width == 2 * 320


def test_select_event_rows_caps_at_40():
    milestones = list(range(100))
    rows = select_event_rows(milestones, cap=40)
    assert len(rows) == 40
    assert rows[0] == 0
    assert rows[-1] == 99


def test_select_event_rows_below_cap_returns_all():
    milestones = list(range(10))
    assert select_event_rows(milestones, cap=40) == milestones


def test_build_sheet_event_layout_caps_rows_at_40(tmp_path):
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    _mk_events_n(tmp_path, 100)
    out = build_sheet(tmp_path, {"id": "c003", "verdict": "PASS"}, tmp_path / "sheet3.jpg")
    img = Image.open(out)
    # An uncapped 100-row sheet would be far taller than a 41-row ceiling;
    # confirm the row cap actually bounds the rendered height.
    uncapped_estimate = 88 + 18 + 100 * 40
    assert img.height < uncapped_estimate
    assert img.height <= 88 + 18 + 41 * 200


# --- judge sheet ---------------------------------------------------------
#
# judge events = PRECONDITION SUCCESS, POSTCONDITION FAILURE, SUPERVISOR
# SUCCESS -- none of these node names match the milestone classifier, so
# `_mk_judge_events` fixtures have empty `milestones` but populated
# `judge_events`, matching a real gate/barrier-heavy run.

_JUDGE_WALLS = (
    "2026-08-28T11:00:01.000000Z",
    "2026-08-28T11:00:02.000000Z",
    "2026-08-28T11:00:03.000000Z",
)


def _mk_judge_events(run_dir, replan=False):
    """Write events.jsonl with PRECONDITION SUCCESS, POSTCONDITION FAILURE,
    SUPERVISOR SUCCESS judge events. With `replan=True`, also append a
    tree_revision=1 tree.generated event, which load_run_telemetry turns
    into one extra REPLAN judge event.
    """
    debug_dir = run_dir / "debug" / "gpsr-20260828T110000000000Z-judge-fixture"
    debug_dir.mkdir(parents=True)
    nodes = [
        {"id": "n_pre", "node_id": "n_pre", "name": "precondition gate:0:0"},
        {"id": "n_post", "node_id": "n_post", "name": "postcondition gate:0:0"},
        {"id": "n_barrier", "node_id": "n_barrier", "name": "supervisor barrier:0:0"},
    ]
    events = [
        {
            "trajectory_id": "traj-judge",
            "occurred_at": "2026-08-28T11:00:00.000000Z",
            "event_type": "tree.generated",
            "payload": {"tree_revision": 0, "nodes": nodes},
        },
        {
            "trajectory_id": "traj-judge",
            "occurred_at": _JUDGE_WALLS[0],
            "event_type": "tree.node_states_changed",
            "payload": {
                "nodes": [{"id": "n_pre", "node_id": "n_pre", "status": "SUCCESS", "feedback": "ok"}]
            },
        },
        {
            "trajectory_id": "traj-judge",
            "occurred_at": _JUDGE_WALLS[1],
            "event_type": "tree.node_states_changed",
            "payload": {
                "nodes": [
                    {
                        "id": "n_post",
                        "node_id": "n_post",
                        "status": "FAILURE",
                        "feedback": "postcondition unmet: counted(persons) (UNKNOWN)",
                    }
                ]
            },
        },
        {
            "trajectory_id": "traj-judge",
            "occurred_at": _JUDGE_WALLS[2],
            "event_type": "tree.node_states_changed",
            "payload": {
                "nodes": [{"id": "n_barrier", "node_id": "n_barrier", "status": "SUCCESS", "feedback": ""}]
            },
        },
    ]
    if replan:
        events.append(
            {
                "trajectory_id": "traj-judge",
                "occurred_at": "2026-08-28T11:00:04.000000Z",
                "event_type": "tree.generated",
                "payload": {"tree_revision": 1, "nodes": nodes},
            }
        )
    events_file = debug_dir / "events.jsonl"
    with events_file.open("w") as f:
        for e in events:
            f.write(json.dumps(e) + "\n")
    return debug_dir


def test_build_judge_sheet_exists_with_rows(tmp_path):
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    _mk_judge_events(tmp_path)
    meta = {
        "id": "c010",
        "text": "count people",
        "verdict": "FAIL",
        "seconds": 30.0,
        "tier": "T2",
        "detail": "postcondition unmet",
    }
    out = build_judge_sheet(tmp_path, meta, tmp_path / "judge-sheet.jpg")
    assert out is not None
    assert out.exists()
    img = Image.open(out)
    assert img.width == 960


def test_build_judge_sheet_height_grows_with_event_count(tmp_path):
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    _mk_judge_events(tmp_path)
    meta = {"id": "c010", "verdict": "FAIL"}
    out1 = build_judge_sheet(tmp_path, meta, tmp_path / "judge1.jpg")
    h1 = Image.open(out1).height

    tmp2 = tmp_path / "more"
    tmp2.mkdir()
    _mk_frames(tmp2, "arena", 5)
    _mk_frames(tmp2, "head", 5)
    _mk_judge_events(tmp2, replan=True)  # one extra (REPLAN) judge event
    out2 = build_judge_sheet(tmp2, meta, tmp2 / "judge2.jpg")
    h2 = Image.open(out2).height
    assert h2 > h1


def test_build_judge_sheet_replan_band_renders(tmp_path):
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    _mk_judge_events(tmp_path, replan=True)
    meta = {"id": "c011", "verdict": "FAIL"}
    out = build_judge_sheet(tmp_path, meta, tmp_path / "judge-replan.jpg")
    assert out is not None
    img = Image.open(out).convert("RGB")
    target = tuple(int(REPLAN_BAND_COLOR[i : i + 2], 16) for i in (1, 3, 5))
    found = False
    for x in range(0, img.width, 20):
        for y in range(0, img.height, 5):
            r, g, b = img.getpixel((x, y))
            if abs(r - target[0]) < 25 and abs(g - target[1]) < 25 and abs(b - target[2]) < 25:
                found = True
                break
        if found:
            break
    assert found, "expected a REPLAN band (#b71c1c) somewhere in the judge sheet"


def test_build_judge_sheet_returns_none_without_telemetry(tmp_path):
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    out_path = tmp_path / "judge-sheet.jpg"
    result = build_judge_sheet(tmp_path, {"id": "x", "verdict": "PASS"}, out_path)
    assert result is None
    assert not out_path.exists()


def test_dedup_transcript_preserves_first_occurrence_order_and_caps():
    lines = ["a", "b", "a", "c", "b", "d"]
    assert _dedup_transcript(lines, cap=25) == ["a", "b", "c", "d"]
    assert _dedup_transcript(lines, cap=2) == ["a", "b"]


def test_build_judge_sheet_plan_block_grows_height(tmp_path):
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    _mk_judge_events(tmp_path)
    meta_no_plan = {"id": "c012", "verdict": "PASS"}
    out1 = build_judge_sheet(tmp_path, meta_no_plan, tmp_path / "judge-noplan.jpg")
    h1 = Image.open(out1).height

    meta_with_plan = {
        "id": "c012",
        "verdict": "PASS",
        "plan": ["go to the kitchen", "find the person", "count them"],
    }
    out2 = build_judge_sheet(tmp_path, meta_with_plan, tmp_path / "judge-plan.jpg")
    h2 = Image.open(out2).height
    assert h2 > h1


def test_main_no_judge_sheet_flag_skips_judge_sheet(tmp_path):
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    _mk_judge_events(tmp_path)
    meta_path = tmp_path / "run.json"
    meta_path.write_text(json.dumps({"id": "c020", "verdict": "PASS", "text": "go", "seconds": 5.0, "tier": "T2"}))
    out_path = tmp_path / "sheet.jpg"

    from tools.contact_sheet import main as sheet_main

    rc = sheet_main(
        [
            "--run-dir",
            str(tmp_path),
            "--meta",
            str(meta_path),
            "--out",
            str(out_path),
            "--no-judge-sheet",
        ]
    )
    assert rc == 0
    assert out_path.exists()
    assert not (tmp_path / "judge-sheet.jpg").exists()


def test_main_default_creates_judge_sheet(tmp_path):
    _mk_frames(tmp_path, "arena", 5)
    _mk_frames(tmp_path, "head", 5)
    _mk_judge_events(tmp_path)
    meta_path = tmp_path / "run.json"
    meta_path.write_text(json.dumps({"id": "c021", "verdict": "PASS", "text": "go", "seconds": 5.0, "tier": "T2"}))
    out_path = tmp_path / "sheet.jpg"

    from tools.contact_sheet import main as sheet_main

    rc = sheet_main(
        [
            "--run-dir",
            str(tmp_path),
            "--meta",
            str(meta_path),
            "--out",
            str(out_path),
        ]
    )
    assert rc == 0
    assert out_path.exists()
    assert (tmp_path / "judge-sheet.jpg").exists()
