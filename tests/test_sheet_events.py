import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tools.sheet_events import MilestoneEvent, JudgeEvent, load_run_telemetry  # noqa: E402

TRAJ = "traj-fixture-001"

SCAN_FAILURE_FEEDBACK = (
    'ScanForGeneralist for persons pointing to the left failed status=1: '
    'no matches for "persons pointing to the left" via vlm'
)
POSTCONDITION_FEEDBACK = "postcondition unmet: counted(persons pointing to the left) (UNKNOWN)"


def _event(event_type, occurred_at, payload, sequence):
    return {
        "schema": "tinker.gpsr.telemetry",
        "schema_version": 1,
        "event_id": f"evt-{sequence}",
        "trajectory_id": TRAJ,
        "trace_id": "trace-fixture",
        "source_id": "gpsr-orchestrator:test",
        "sequence": sequence,
        "occurred_at": occurred_at,
        "event_type": event_type,
        "payload": payload,
    }


def _node_state(node_id, status, feedback):
    return {
        "id": node_id,
        "node_id": node_id,
        "status": status,
        "feedback": feedback,
    }


def _build_events():
    events = []
    seq = 0

    def add(event_type, occurred_at, payload):
        nonlocal seq
        seq += 1
        events.append(_event(event_type, occurred_at, payload, seq))

    # tree.generated rev0: names for every node referenced below.
    add(
        "tree.generated",
        "2026-08-28T10:00:00.000000Z",
        {
            "tree_revision": 0,
            "nodes": [
                {"id": "n_goto", "node_id": "n_goto", "name": "goto target"},
                {"id": "n_scan", "node_id": "n_scan", "name": "scan to count"},
                {"id": "n_announce", "node_id": "n_announce", "name": "announce vlm count"},
                {"id": "n_keepalive", "node_id": "n_keepalive", "name": "nav keepalive 0"},
                {"id": "n_post", "node_id": "n_post", "name": "postcondition gate:0:0"},
                {"id": "n_barrier", "node_id": "n_barrier", "name": "supervisor barrier:0:0"},
                {"id": "n_tuck", "node_id": "n_tuck", "name": "tuck arm before goto"},
                {"id": "n_running", "node_id": "n_running", "name": "announce running check"},
            ],
        },
    )

    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:01.000000Z",
        {"nodes": [_node_state("n_goto", "SUCCESS", "goal accepted :) [BtNode_GotoAction/goto target]")]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:02.000000Z",
        {"nodes": [_node_state("n_scan", "FAILURE", SCAN_FAILURE_FEEDBACK)]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:03.000000Z",
        {"nodes": [_node_state("n_announce", "SUCCESS", "Finished announcing 0 persons.")]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:04.000000Z",
        {"nodes": [_node_state("n_keepalive", "SUCCESS", "Finished announcing I am trying to go to the destination.")]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:05.000000Z",
        {"nodes": [_node_state("n_post", "FAILURE", POSTCONDITION_FEEDBACK)]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:06.000000Z",
        {"nodes": [_node_state("n_tuck", "SUCCESS", "MOCK: auto-complete finished")]},
    )
    add(
        "tree.node_states_changed",
        "2026-08-28T10:00:07.000000Z",
        {"nodes": [_node_state("n_running", "RUNNING", "MOCK: auto-completing (1/2)")]},
    )

    add(
        "tree.generated",
        "2026-08-28T10:00:08.000000Z",
        {"tree_revision": 1, "nodes": []},
    )

    add(
        "run.finished",
        "2026-08-28T10:00:09.000000Z",
        {"trajectory_id": TRAJ, "status": "complete"},
    )

    return events


def _write_events(tmp_path, events, run_name="run-000"):
    run_dir = tmp_path / run_name
    debug_dir = run_dir / "debug" / "gpsr-20260828T100000000000Z-fixture01"
    debug_dir.mkdir(parents=True)
    events_file = debug_dir / "events.jsonl"
    with events_file.open("w") as f:
        for event in events:
            f.write(json.dumps(event) + "\n")
    return run_dir


def test_load_run_telemetry_extracts_milestones_and_judge_events(tmp_path):
    run_dir = _write_events(tmp_path, _build_events())

    milestones, judge_events, meta = load_run_telemetry(run_dir)

    assert [m.kind for m in milestones] == ["NAV", "VISION", "AUDIO", "MANIP"]
    assert [m.name for m in milestones] == [
        "goto target",
        "scan to count",
        "announce vlm count",
        "tuck arm before goto",
    ]
    assert [m.status for m in milestones] == ["SUCCESS", "FAILURE", "SUCCESS", "SUCCESS"]
    assert milestones[0].info == "goal accepted :) [BtNode_GotoAction/goto target]"
    assert milestones[1].info == SCAN_FAILURE_FEEDBACK
    assert milestones[2].info == "0 persons"
    assert milestones[3].info == "MOCK: auto-complete finished"
    assert milestones[0].wall == "2026-08-28T10:00:01.000000Z"

    assert all(isinstance(m, MilestoneEvent) for m in milestones)

    assert [j.kind for j in judge_events] == ["POSTCONDITION", "REPLAN"]
    assert judge_events[0].name == "postcondition gate:0:0"
    assert judge_events[0].status == "FAILURE"
    assert judge_events[0].info == POSTCONDITION_FEEDBACK
    assert judge_events[1].name == "replan"
    assert judge_events[1].status == "SUCCESS"
    assert judge_events[1].info == "tree revision 1"
    assert judge_events[1].wall == "2026-08-28T10:00:08.000000Z"

    assert all(isinstance(j, JudgeEvent) for j in judge_events)

    assert meta["trajectory_id"] == TRAJ
    assert meta["tree_revisions"] == 1
    assert meta["run_finished"] == {"trajectory_id": TRAJ, "status": "complete"}


def test_keepalive_and_running_are_excluded(tmp_path):
    run_dir = _write_events(tmp_path, _build_events())
    milestones, _judge_events, _meta = load_run_telemetry(run_dir)
    names = [m.name for m in milestones]
    assert "nav keepalive 0" not in names
    assert "announce running check" not in names


def test_missing_debug_dir_returns_empty_shapes(tmp_path):
    run_dir = tmp_path / "run-no-debug"
    run_dir.mkdir()

    milestones, judge_events, meta = load_run_telemetry(run_dir)

    assert milestones == []
    assert judge_events == []
    assert meta == {}


def test_missing_run_dir_returns_empty_shapes(tmp_path):
    milestones, judge_events, meta = load_run_telemetry(tmp_path / "does-not-exist")
    assert (milestones, judge_events, meta) == ([], [], {})


def test_garbage_line_is_skipped_rest_still_parsed(tmp_path):
    events = _build_events()
    run_dir = tmp_path / "run-garbage"
    debug_dir = run_dir / "debug" / "gpsr-20260828T100000000000Z-fixture02"
    debug_dir.mkdir(parents=True)
    events_file = debug_dir / "events.jsonl"
    with events_file.open("w") as f:
        f.write("{this is not valid json\n")
        for event in events:
            f.write(json.dumps(event) + "\n")
        f.write("\n")  # trailing blank line should also be tolerated

    milestones, judge_events, meta = load_run_telemetry(run_dir)

    assert [m.kind for m in milestones] == ["NAV", "VISION", "AUDIO", "MANIP"]
    assert [j.kind for j in judge_events] == ["POSTCONDITION", "REPLAN"]
    assert meta["tree_revisions"] == 1


def test_newest_gpsr_debug_dir_is_selected(tmp_path):
    run_dir = tmp_path / "run-multi"
    older = run_dir / "debug" / "gpsr-20260101T000000000000Z-aaaaaaaa"
    newer = run_dir / "debug" / "gpsr-20260828T100000000000Z-bbbbbbbb"
    older.mkdir(parents=True)
    newer.mkdir(parents=True)

    (older / "events.jsonl").write_text(
        json.dumps(_event("tree.generated", "2026-01-01T00:00:00Z", {"tree_revision": 0, "nodes": []}, 1)) + "\n"
    )
    with (newer / "events.jsonl").open("w") as f:
        for event in _build_events():
            f.write(json.dumps(event) + "\n")

    milestones, judge_events, meta = load_run_telemetry(run_dir)

    assert len(milestones) == 4
    assert meta["trajectory_id"] == TRAJ
