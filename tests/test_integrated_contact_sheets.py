"""Task 9 fix-round-1 integrated contact-sheet tests (ROS-free, Python 3.12).

The suite factory and capture-path selector are defined in
``test_integrated_evidence_index`` (``make_complete_evidence_suite`` /
``required_capture_paths``).  A contact sheet is authorized only by captures
that already carry exact path+digest+event/frame metadata in
``suite_dir/evidence-index.json``; the agent and user sheets must use the same
source captures/event set with distinct embedded role metadata, and Gate F must
be able to verify sheet semantics from the sheet bytes themselves (F1.6).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from test_integrated_evidence_index import (  # noqa: E402
    AGENT_NAME,
    CAMERAS,
    CANCEL_EVENTS,
    IMAGE_SIZE,
    POSITIVE_EVENTS,
    SAFETY_EVENTS,
    make_complete_evidence_suite,
    render_sheets,
    required_capture_paths,
)
from validation.integrated_contact_sheets import (  # noqa: E402
    AGENT_NAME as AGENT_FILE,
    USER_NAME as USER_FILE,
    build_contact_sheet,
    _read_sheet_metadata,
)

REQUIRED = set(POSITIVE_EVENTS)
AGENT = AGENT_FILE
USER = USER_FILE


def _make_sheets(suite_dir: Path, *, events: set[str] | None = None) -> dict[str, object]:
    events = REQUIRED if events is None else events
    paths = required_capture_paths(suite_dir, events=events)
    agent = build_contact_sheet(suite_dir, paths, output=suite_dir / AGENT)
    user = build_contact_sheet(suite_dir, paths, output=suite_dir / USER, user=True)
    return {"agent": agent, "user": user}


def test_agent_and_user_sheets_cover_required_events(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    sheets = _make_sheets(suite_dir)
    assert set(sheets["agent"]["events"]) == REQUIRED
    assert set(sheets["user"]["events"]) == REQUIRED


def test_agent_user_roles_distinct_and_correct(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    _make_sheets(suite_dir)
    agent_meta = _read_sheet_metadata(suite_dir / AGENT)
    user_meta = _read_sheet_metadata(suite_dir / USER)
    assert agent_meta is not None and user_meta is not None
    assert agent_meta["role"] == "agent"
    assert user_meta["role"] == "user"
    assert agent_meta["role"] != user_meta["role"]
    assert agent_meta["events"] == user_meta["events"]
    assert agent_meta["captures_sha256"] == user_meta["captures_sha256"]


def test_sheets_are_not_byte_identical(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    _make_sheets(suite_dir)
    agent_bytes = (suite_dir / AGENT).read_bytes()
    user_bytes = (suite_dir / USER).read_bytes()
    assert agent_bytes != user_bytes


def test_embedded_metadata_is_deterministic(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    _make_sheets(suite_dir)
    first = _read_sheet_metadata(suite_dir / AGENT)
    _make_sheets(suite_dir)
    second = _read_sheet_metadata(suite_dir / AGENT)
    assert first == second
    assert first["diagnostic_only"] is True
    assert first["reviewed"] is False
    for capture in first["captures"]:
        assert capture["path"].startswith("E/") and "visual/source/" in capture["path"]
        assert isinstance(capture["frame_index"], int)
        assert isinstance(capture["timestamp"], float)


def test_blank_or_unindexed_image_fails_sheet(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    image_path = suite_dir / "E" / "qualification-pick-place-positive" / "visual/source/blank.png"
    image_path.parent.mkdir(parents=True, exist_ok=True)
    Image.new("RGBA", (64, 64), (0, 0, 0, 0)).save(image_path)
    with pytest.raises(ValueError, match="blank|transparent|indexed|unbound"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[image_path],
            output=suite_dir / AGENT,
        )


def test_contact_sheet_requires_existing_evidence_index(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    paths = required_capture_paths(suite_dir, events=REQUIRED)
    (suite_dir / "evidence-index.json").unlink()
    with pytest.raises(ValueError, match="evidence index"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=paths,
            output=suite_dir / AGENT,
        )


def test_contact_sheet_rejects_missing_capture_path(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    missing = suite_dir / "E" / "qualification-pick-place-positive" / "visual/source/does-not-exist.png"
    with pytest.raises(ValueError, match="missing|unindexed"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[missing],
            output=suite_dir / AGENT,
        )


def test_contact_sheet_rejects_mismatched_stale_capture(tmp_path):
    from PIL import ImageDraw

    suite_dir = make_complete_evidence_suite(tmp_path)
    entry = next(
        e for e in json.loads((suite_dir / "evidence-index.json").read_text(encoding="utf-8"))["files"]
        if e.get("category") == "capture" and e.get("event") == "approach"
    )
    path = suite_dir / entry["path"]
    original = path.read_bytes()
    image = Image.new("RGB", IMAGE_SIZE, (10, 20, 30))
    draw = ImageDraw.Draw(image)
    draw.rectangle((200, 100, 700, 400), fill=(200, 10, 10))
    draw.line((0, 100, 959, 440), fill=(240, 240, 240), width=5)
    image.save(path, format="PNG")
    with pytest.raises(ValueError, match="mismatch|stale"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[path],
            output=suite_dir / AGENT,
        )
    path.write_bytes(original)


def test_contact_sheet_rejects_blank_indexed_capture(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    entry = next(
        e for e in json.loads((suite_dir / "evidence-index.json").read_text(encoding="utf-8"))["files"]
        if e.get("category") == "capture" and e.get("event") == "readiness"
    )
    path = suite_dir / entry["path"]
    original = path.read_bytes()
    Image.new("RGB", IMAGE_SIZE, (30, 30, 30)).save(path, format="PNG")
    with pytest.raises(ValueError, match="blank|transparent"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[path],
            output=suite_dir / AGENT,
        )
    path.write_bytes(original)


def test_contact_sheet_rejects_indexed_transparent_capture(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    entry = next(
        e for e in json.loads((suite_dir / "evidence-index.json").read_text(encoding="utf-8"))["files"]
        if e.get("category") == "capture" and e.get("event") == "terminal"
    )
    path = suite_dir / entry["path"]
    original = path.read_bytes()
    Image.new("RGBA", IMAGE_SIZE, (0, 0, 0, 0)).save(path, format="PNG")
    with pytest.raises(ValueError, match="blank|transparent"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[path],
            output=suite_dir / AGENT,
        )
    path.write_bytes(original)


def test_contact_sheet_rejects_unindexed_capture(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    path = suite_dir / "E" / "qualification-pick-place-positive" / "visual/source/orphan.png"
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.new("RGB", IMAGE_SIZE, (90, 90, 90)).save(path, format="PNG")
    with pytest.raises(ValueError, match="unindexed|unbound|metadata"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[path],
            output=suite_dir / AGENT,
        )


def test_contact_sheet_rejects_path_traversal_and_symlink_escape(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    outside = tmp_path / "outside.png"
    Image.new("RGB", IMAGE_SIZE, (10, 20, 30)).save(outside, format="PNG")
    with pytest.raises(ValueError, match="outside|traversal|escape"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[outside],
            output=suite_dir / AGENT,
        )
    link = suite_dir / "E" / "qualification-pick-place-positive" / "visual/source/linked.png"
    link.parent.mkdir(parents=True, exist_ok=True)
    link.symlink_to(outside)
    with pytest.raises(ValueError, match="outside|escape|symlink"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[link],
            output=suite_dir / AGENT,
        )


def test_contact_sheet_rejects_duplicate_paths_and_events(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    paths = required_capture_paths(suite_dir, events={"readiness"})
    readiness = paths[0]
    with pytest.raises(ValueError, match="duplicate"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[readiness, readiness],
            output=suite_dir / AGENT,
        )
    index = json.loads((suite_dir / "evidence-index.json").read_text(encoding="utf-8"))
    entry = next(e for e in index["files"] if e["path"] == readiness.relative_to(suite_dir).as_posix())
    index["files"].append(dict(entry, path="E/qualification-pick-place-positive/visual/source/readiness-dup.png"))
    (suite_dir / "evidence-index.json").write_text(json.dumps(index, sort_keys=True), encoding="utf-8")
    dup = suite_dir / "E" / "qualification-pick-place-positive" / "visual/source/readiness-dup.png"
    dup.parent.mkdir(parents=True, exist_ok=True)
    Image.new("RGB", IMAGE_SIZE, (5, 5, 5)).save(dup, format="PNG")
    with pytest.raises(ValueError, match="duplicate event"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[readiness, dup],
            output=suite_dir / AGENT,
        )


def test_contact_sheet_rejects_output_as_input(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    paths = required_capture_paths(suite_dir, events={"readiness"})
    readiness = paths[0]
    with pytest.raises(ValueError, match="output-as-input|output"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[readiness],
            output=readiness,
        )
    with pytest.raises(ValueError, match="output-as-input|output|protected"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[readiness],
            output=suite_dir / "evidence-index.json",
        )


def test_agent_and_user_sheets_semantic_parity(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    sheets = _make_sheets(suite_dir)
    assert set(sheets["agent"]["events"]) == REQUIRED
    assert set(sheets["user"]["events"]) == REQUIRED
    assert sheets["agent"]["events"] == sheets["user"]["events"]
    for name in (AGENT, USER):
        output = suite_dir / name
        assert output.is_file()
        with Image.open(output) as image:
            assert image.mode in ("RGB", "RGBA")


def test_contact_sheet_deterministic_render(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    _make_sheets(suite_dir)
    first = (suite_dir / AGENT).read_bytes()
    _make_sheets(suite_dir)
    assert first == (suite_dir / AGENT).read_bytes()


def test_contact_sheet_regeneration_after_index_rebuild(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    _make_sheets(suite_dir)
    from validation.integrated_evidence_index import build_evidence_index

    build_evidence_index(suite_dir=suite_dir, output=suite_dir / "evidence-index.json")
    before = (suite_dir / USER).read_bytes()
    _make_sheets(suite_dir)
    assert before == (suite_dir / USER).read_bytes()


def test_cancel_and_safety_event_sheets(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    events = set(CANCEL_EVENTS) | set(SAFETY_EVENTS)
    sheets = _make_sheets(suite_dir, events=events)
    assert set(sheets["agent"]["events"]) == events
    assert set(sheets["user"]["events"]) == events


def test_contact_sheet_events_come_from_index_metadata_not_pixels(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    agent = build_contact_sheet(
        suite_dir=suite_dir,
        image_paths=required_capture_paths(suite_dir, events=REQUIRED),
        output=suite_dir / AGENT,
    )
    assert agent["scenario"] == "qualification-pick-place-positive"
    assert agent["attempt"] == "attempt-positive"
    assert len(agent["events"]) == len(REQUIRED)
    assert all(p.startswith("E/") and "visual/source/" in p for p in agent["paths"])


def test_cli_selects_bound_live_paths(tmp_path):
    from validation.integrated_contact_sheets import main

    suite_dir = make_complete_evidence_suite(tmp_path)
    render_sheets(suite_dir)
    exit_code = main(["--suite-dir", str(suite_dir)])
    assert exit_code == 0
    assert (suite_dir / AGENT).is_file()
    assert (suite_dir / USER).is_file()
    user_meta = _read_sheet_metadata(suite_dir / USER)
    assert user_meta["role"] == "user"
    assert all(
        isinstance(c, dict) and c.get("path", "").startswith("E/") and "visual/source/" in c.get("path", "")
        for c in user_meta["captures"]
    )


# --- Task 9 fix-round-2 tests (F2.9 output-as-input closure) -----------------


def test_contact_sheet_rejects_output_equal_other_sheet(tmp_path):
    """F2.9: an agent sheet may never overwrite the user sheet (and vice versa)."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    paths = required_capture_paths(suite_dir, events=set(POSITIVE_EVENTS))
    with pytest.raises(ValueError, match="output-as-input|protected"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=paths,
            output=suite_dir / USER,  # agent render must not clobber user sheet
        )
    with pytest.raises(ValueError, match="output-as-input|protected"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=paths,
            output=suite_dir / AGENT,
            user=True,  # user render must not clobber agent sheet
        )


def test_contact_sheet_rejects_output_equal_own_sibling_after_write(tmp_path):
    """F2.9: overwriting the sheet's own path is the only allowed protected output."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    paths = required_capture_paths(suite_dir, events={"readiness"})
    result = build_contact_sheet(suite_dir, paths, output=suite_dir / AGENT)
    assert result["role"] == "agent"
    assert (suite_dir / AGENT).is_file()


def test_contact_sheet_rejects_output_in_indexed_captures(tmp_path):
    """F2.9: output colliding with an indexed capture path is rejected."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    entry = next(
        e for e in json.loads((suite_dir / "evidence-index.json").read_text(encoding="utf-8"))["files"]
        if e.get("category") == "capture" and e.get("event") == "readiness"
    )
    paths = required_capture_paths(suite_dir, events={"readiness"})
    with pytest.raises(ValueError, match="output-as-input|indexed capture"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=paths,
            output=suite_dir / entry["path"],
        )


# --- Task 9 fix-round-3 tests (F3.7 output-as-input over EVERY evidence input)


def test_contact_sheet_rejects_output_equal_indexed_json(tmp_path):
    """F3.7: output equal to an indexed JSON artifact is rejected."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    paths = required_capture_paths(suite_dir, events={"readiness"})
    for rel in (
        "E/qualification-pick-place-positive/truth-drain.json",
        "E/qualification-pick-place-positive/gate-verdict.json",
    ):
        with pytest.raises(ValueError, match="output-as-input|indexed evidence"):
            build_contact_sheet(
                suite_dir=suite_dir,
                image_paths=paths,
                output=suite_dir / rel,
            )


def test_contact_sheet_rejects_output_equal_indexed_jsonl(tmp_path):
    """F3.7: output equal to an indexed JSONL journal is rejected."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    paths = required_capture_paths(suite_dir, events={"readiness"})
    for rel in (
        "E/qualification-pick-place-positive/physics_truth.jsonl",
        "E/qualification-pick-place-positive/visual-keyframes.jsonl",
        "E/qualification-pick-place-positive/planning-scene.jsonl",
    ):
        with pytest.raises(ValueError, match="output-as-input|indexed evidence"):
            build_contact_sheet(
                suite_dir=suite_dir,
                image_paths=paths,
                output=suite_dir / rel,
            )


def test_contact_sheet_rejects_output_equal_rosbag_db3(tmp_path):
    """F3.7: output equal to an indexed rosbag DB3 storage file is rejected."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    paths = required_capture_paths(suite_dir, events={"readiness"})
    storage = next(
        e for e in json.loads((suite_dir / "evidence-index.json").read_text(encoding="utf-8"))["files"]
        if e.get("category") == "rosbag-storage"
    )
    with pytest.raises(ValueError, match="output-as-input|indexed evidence"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=paths,
            output=suite_dir / storage["path"],
        )


def test_contact_sheet_rejects_output_equal_metadata(tmp_path):
    """F3.7: output equal to an indexed metadata.yaml is rejected."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    paths = required_capture_paths(suite_dir, events={"readiness"})
    metadata = next(
        e for e in json.loads((suite_dir / "evidence-index.json").read_text(encoding="utf-8"))["files"]
        if e.get("category") == "rosbag-metadata"
    )
    with pytest.raises(ValueError, match="output-as-input|indexed evidence"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=paths,
            output=suite_dir / metadata["path"],
        )


def test_contact_sheet_rejects_output_equal_manifest(tmp_path):
    """F3.7: output equal to an indexed manifest is rejected."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    paths = required_capture_paths(suite_dir, events={"readiness"})
    manifest = next(
        e for e in json.loads((suite_dir / "evidence-index.json").read_text(encoding="utf-8"))["files"]
        if e.get("category") == "manifest"
    )
    with pytest.raises(ValueError, match="output-as-input|indexed evidence"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=paths,
            output=suite_dir / manifest["path"],
        )


def test_contact_sheet_own_output_path_still_regenerable(tmp_path):
    """F3.7: the sheet's own expected output path remains regenerable."""
    suite_dir = make_complete_evidence_suite(tmp_path)
    paths = required_capture_paths(suite_dir, events={"readiness"})
    result = build_contact_sheet(suite_dir, paths, output=suite_dir / AGENT)
    assert result["role"] == "agent"
    second = build_contact_sheet(suite_dir, paths, output=suite_dir / AGENT)
    assert second["role"] == "agent"
