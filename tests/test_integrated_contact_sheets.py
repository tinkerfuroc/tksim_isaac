"""Task 9 integrated contact-sheet tests (ROS-free, Python 3.12).

The suite factory and capture-path selector are defined in
``test_integrated_evidence_index`` (``make_complete_evidence_suite`` /
``required_capture_paths``).  A contact sheet is authorized only by captures that
already carry exact path+digest+event/frame metadata in
``suite_dir/evidence-index.json``; PlanningScene/action/screenshot evidence is
diagnostic only and never physical pass authority.  Agent and user sheets must
agree on the covered event set without treating pixels as physical proof.
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
    CANCEL_EVENTS,
    IMAGE_SIZE,
    POSITIVE_EVENTS,
    make_complete_evidence_suite,
    required_capture_paths,
)
from validation.integrated_contact_sheets import build_contact_sheet  # noqa: E402

REQUIRED = set(POSITIVE_EVENTS)
AGENT = "contact-sheet-integrated-agent.png"
USER = "contact-sheet-integrated-user.png"


def _make_sheets(suite_dir: Path, *, events: set[str] | None = None) -> dict[str, object]:
    events = REQUIRED if events is None else events
    agent = build_contact_sheet(
        suite_dir=suite_dir,
        image_paths=required_capture_paths(suite_dir, events=events),
        output=suite_dir / AGENT,
    )
    user = build_contact_sheet(
        suite_dir=suite_dir,
        image_paths=required_capture_paths(suite_dir, events=events),
        output=suite_dir / USER,
    )
    return {"agent": agent, "user": user}


def test_blank_or_unindexed_image_fails_sheet(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    image_path = suite_dir / "captures" / "blank.png"
    Image.new("RGBA", (64, 64), (0, 0, 0, 0)).save(image_path)
    with pytest.raises(ValueError, match="blank|transparent|indexed"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[image_path],
            output=suite_dir / AGENT,
        )


def test_agent_and_user_sheets_cover_required_events(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    agent = build_contact_sheet(
        suite_dir=suite_dir,
        image_paths=required_capture_paths(suite_dir, events=REQUIRED),
        output=suite_dir / AGENT,
    )
    user = build_contact_sheet(
        suite_dir=suite_dir,
        image_paths=required_capture_paths(suite_dir, events=REQUIRED),
        output=suite_dir / USER,
    )
    assert set(agent["events"]) == REQUIRED
    assert set(user["events"]) == REQUIRED


def test_contact_sheet_requires_existing_evidence_index(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    (suite_dir / "evidence-index.json").unlink()
    with pytest.raises(ValueError, match="evidence index"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=required_capture_paths(suite_dir, events=REQUIRED),
            output=suite_dir / AGENT,
        )


def test_contact_sheet_rejects_missing_capture_path(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    missing = suite_dir / "captures/does-not-exist.png"
    with pytest.raises(ValueError, match="missing|unindexed"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[missing],
            output=suite_dir / AGENT,
        )


def test_contact_sheet_rejects_mismatched_stale_capture(tmp_path):
    from PIL import ImageDraw

    suite_dir = make_complete_evidence_suite(tmp_path)
    path = suite_dir / "captures/approach.png"
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
    indexed = suite_dir / "captures/readiness.png"
    original = indexed.read_bytes()
    Image.new("RGB", IMAGE_SIZE, (30, 30, 30)).save(indexed, format="PNG")
    with pytest.raises(ValueError, match="blank|transparent"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[indexed],
            output=suite_dir / AGENT,
        )
    indexed.write_bytes(original)


def test_contact_sheet_rejects_indexed_transparent_capture(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    indexed = suite_dir / "captures/terminal.png"
    original = indexed.read_bytes()
    Image.new("RGBA", IMAGE_SIZE, (0, 0, 0, 0)).save(indexed, format="PNG")
    with pytest.raises(ValueError, match="blank|transparent"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[indexed],
            output=suite_dir / AGENT,
        )
    indexed.write_bytes(original)


def test_contact_sheet_rejects_unindexed_capture(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    path = suite_dir / "captures/orphan.png"
    Image.new("RGB", IMAGE_SIZE, (90, 90, 90)).save(path, format="PNG")
    with pytest.raises(ValueError, match="unbound|missing.*metadata|indexed"):
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
    link = suite_dir / "captures/linked.png"
    link.symlink_to(outside)
    with pytest.raises(ValueError, match="outside|escape|symlink"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[link],
            output=suite_dir / AGENT,
        )


def test_contact_sheet_rejects_duplicate_paths_and_events(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    readiness = suite_dir / "captures/readiness.png"
    # Duplicate canonical path.
    with pytest.raises(ValueError, match="duplicate"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[readiness, readiness],
            output=suite_dir / AGENT,
        )
    # Duplicate event binding via a hand-crafted index entry (defense-in-depth:
    # the real builder already rejects duplicate event bindings at index time).
    index = json.loads((suite_dir / "evidence-index.json").read_text(encoding="utf-8"))
    entry = next(item for item in index["files"] if item["path"] == "captures/readiness.png")
    index["files"].append(dict(entry, path="captures/readiness-dup.png"))
    (suite_dir / "evidence-index.json").write_text(json.dumps(index, sort_keys=True), encoding="utf-8")
    dup_path = suite_dir / "captures/readiness-dup.png"
    Image.new("RGB", IMAGE_SIZE, (5, 5, 5)).save(dup_path, format="PNG")
    with pytest.raises(ValueError, match="duplicate event"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[readiness, dup_path],
            output=suite_dir / AGENT,
        )


def test_contact_sheet_rejects_output_as_input(tmp_path):
    suite_dir = make_complete_evidence_suite(tmp_path)
    readiness = suite_dir / "captures/readiness.png"
    with pytest.raises(ValueError, match="output-as-input|output"):
        build_contact_sheet(
            suite_dir=suite_dir,
            image_paths=[readiness],
            output=readiness,
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
            assert image.mode == "RGB"


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
    events = set(CANCEL_EVENTS) | {"safety-execution-start", "safety-trigger", "safety-velocity-compliant", "safety-post-clear"}
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
    # The returned events are bound to the evidence index metadata; they are
    # never derived from image pixels, and screenshots are diagnostic only.
    assert agent["scenario"] == "qualification-pick-place-positive"
    assert agent["attempt"] == "attempt-1"
    assert len(agent["events"]) == len(REQUIRED)
