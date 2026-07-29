from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from validation.manipulation_contact_sheets import (  # noqa: E402
    GATE_EVENTS,
    IMAGE_SIZE,
    RESULT_NAME,
    main,
    process_attempt,
    process_suite,
)


def _image(path: Path, seed: int) -> None:
    image = Image.new("RGB", IMAGE_SIZE, (25 + seed, 45 + seed, 75 + seed))
    draw = ImageDraw.Draw(image)
    draw.rectangle((120 + seed, 80, 800, 430), fill=(180, 80 + seed, 35))
    draw.line((0, seed + 10, 959, 500 - seed), fill=(240, 240, 240), width=5)
    image.save(path, format="PNG", optimize=False, compress_level=9)


def _write_journals(attempt: Path, gate: str) -> None:
    execution = []
    requests = []
    for sequence, event in enumerate(GATE_EVENTS[gate], 1):
        timestamp = float(sequence - 1)
        execution.append(
            {
                "schema_version": 1,
                "sequence": sequence,
                "event": event,
                "gate": gate,
                "simulated_timestamp": timestamp,
            }
        )
        requests.append(
            {
                "schema_version": 1,
                "sequence": sequence,
                "event": event,
                "gate": gate,
                "simulated_timestamp": timestamp,
                "source_execution_event_sequence": sequence,
            }
        )
    (attempt / "gate-execution.jsonl").write_text("\n".join(json.dumps(item) for item in execution) + "\n", encoding="utf-8")
    (attempt / "visual-capture-requests.jsonl").write_text("\n".join(json.dumps(item) for item in requests) + "\n", encoding="utf-8")


def _make_attempt(root: Path, gate: str = "free-space-fjt", *, bad: str | None = None, verdict: str = "verified-pass") -> Path:
    attempt = root / gate
    frames = attempt / "frames"
    frames.mkdir(parents=True)
    records = []
    index = 0
    for event_index, event in enumerate(GATE_EVENTS[gate]):
        for camera in ("overview", "manipulation_closeup"):
            path = frames / f"{event}-{camera}.png"
            _image(path, index + 1)
            record = {
                "gate": gate,
                "event": event,
                "camera": camera,
                "path": str(path.relative_to(attempt)),
                "sim_time": float(event_index),
                "requested_simulated_timestamp": float(event_index),
                "request_sequence": event_index + 1,
                "execution_event_sequence": event_index + 1,
                "raw_frame_id": index,
                "raw_frame_index": index,
                "requested_physics_frame_index": index,
                "capture_latency_frames": 0,
                "max_capture_latency_frames": 4,
            }
            records.append(record)
            index += 1
    payload = {
        "schema_version": 1,
        "gate": gate,
        "physics_frame_s": 1 / 150,
        "capture_latency_contract": {
            "unit": "physics_frames",
            "max_frames": 4,
            "basis": "raw_frame_index-requested_physics_frame_index",
        },
        "keyframes": records,
    }
    (attempt / "visual-keyframes.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    (attempt / "gate-verdict.json").write_text(json.dumps({"gate": gate, "status": verdict, "pass": verdict == "verified-pass", "metrics": {"max_error": 0.01}}), encoding="utf-8")
    _write_journals(attempt, gate)
    if bad == "blank":
        Image.new("RGB", IMAGE_SIZE, (30, 30, 30)).save(frames / "start-overview.png", format="PNG")
    elif bad == "stale":
        records[1]["sim_time"] = 99.0
        (attempt / "visual-keyframes.json").write_text(json.dumps(payload), encoding="utf-8")
    elif bad == "missing":
        (frames / "start-overview.png").unlink()
    elif bad == "dimension":
        Image.new("RGB", (10, 10), (100, 100, 100)).save(frames / "start-overview.png", format="PNG")
    return attempt


class ManipulationContactSheetsTest(unittest.TestCase):
    def test_valid_attempt_generates_diagnostic_and_atomic_result(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory))
            result = process_attempt(attempt)
            self.assertEqual(result["status"], "valid")
            self.assertTrue((attempt / "contact-sheet-diagnostic.png").is_file())
            self.assertEqual(Image.open(attempt / "contact-sheet-diagnostic.png").mode, "RGB")
            self.assertTrue((attempt / RESULT_NAME).is_file())
            self.assertFalse(list(attempt.glob(f".{RESULT_NAME}.*")))

    def test_valid_attempt_omits_checksum_fields(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            result = process_attempt(_make_attempt(Path(directory)))
            self.assertNotIn("source_hashes", result)
            self.assertNotIn("generated_hashes", result)

    def test_binding_journals_are_required(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory))
            (attempt / "gate-execution.jsonl").unlink()
            result = process_attempt(attempt)
            self.assertIn("missing-gate-execution-journal", {item["code"] for item in result["diagnostics"]})
            attempt = _make_attempt(Path(directory) / "missing-requests")
            (attempt / "visual-capture-requests.jsonl").unlink()
            result = process_attempt(attempt)
            self.assertIn("missing-visual-capture-request-journal", {item["code"] for item in result["diagnostics"]})

    def test_duplicate_reordered_and_extra_journal_records_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory))
            execution_path = attempt / "gate-execution.jsonl"
            execution = [json.loads(line) for line in execution_path.read_text().splitlines()]
            execution.append({**execution[0], "sequence": 5})
            execution_path.write_text("\n".join(json.dumps(item) for item in execution) + "\n")
            request_path = attempt / "visual-capture-requests.jsonl"
            requests = [json.loads(line) for line in request_path.read_text().splitlines()]
            requests.append({**requests[0], "sequence": 5})
            request_path.write_text("\n".join(json.dumps(item) for item in requests) + "\n")
            result = process_attempt(attempt)
            codes = {item["code"] for item in result["diagnostics"]}
            self.assertIn("duplicate-execution-checkpoint", codes)
            self.assertIn("duplicate-visual-request", codes)

            attempt = _make_attempt(Path(directory) / "reordered")
            execution_path = attempt / "gate-execution.jsonl"
            execution = execution_path.read_text().splitlines()
            execution[0], execution[1] = execution[1], execution[0]
            execution_path.write_text("\n".join(execution) + "\n")
            request_path = attempt / "visual-capture-requests.jsonl"
            requests = request_path.read_text().splitlines()
            requests[0], requests[1] = requests[1], requests[0]
            request_path.write_text("\n".join(requests) + "\n")
            result = process_attempt(attempt)
            codes = {item["code"] for item in result["diagnostics"]}
            self.assertIn("reordered-execution-journal", codes)
            self.assertIn("visual-request-order-mismatch", codes)

            attempt = _make_attempt(Path(directory) / "extra")
            execution_path = attempt / "gate-execution.jsonl"
            execution = [json.loads(line) for line in execution_path.read_text().splitlines()]
            execution.append({"sequence": 5, "event": "fabricated", "gate": "free-space-fjt", "simulated_timestamp": 4.0})
            execution_path.write_text("\n".join(json.dumps(item) for item in execution) + "\n")
            result = process_attempt(attempt)
            self.assertIn("extra-execution-checkpoint", {item["code"] for item in result["diagnostics"]})

    def test_mismatched_journal_gate_source_sequence_and_timestamp_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory))
            execution_path = attempt / "gate-execution.jsonl"
            execution = [json.loads(line) for line in execution_path.read_text().splitlines()]
            execution[0]["gate"] = "retention"
            execution_path.write_text("\n".join(json.dumps(item) for item in execution) + "\n")
            request_path = attempt / "visual-capture-requests.jsonl"
            requests = [json.loads(line) for line in request_path.read_text().splitlines()]
            requests[1]["source_execution_event_sequence"] = 99
            requests[2]["simulated_timestamp"] = 99.0
            request_path.write_text("\n".join(json.dumps(item) for item in requests) + "\n")
            result = process_attempt(attempt)
            codes = {item["code"] for item in result["diagnostics"]}
            self.assertIn("execution-journal-gate-mismatch", codes)
            self.assertIn("visual-request-execution-binding-mismatch", codes)
            self.assertIn("keyframe-requested-timestamp-journal-mismatch", codes)

    def test_keyframe_gate_and_event_must_be_explicit_and_exact(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory))
            payload = json.loads((attempt / "visual-keyframes.json").read_text())
            del payload["keyframes"][0]["gate"]
            payload["keyframes"][1]["event"] = "START"
            (attempt / "visual-keyframes.json").write_text(json.dumps(payload))
            result = process_attempt(attempt)
            codes = {item["code"] for item in result["diagnostics"]}
            self.assertIn("keyframe-gate-journal-mismatch", codes)
            self.assertIn("noncanonical-keyframe-event", codes)

    def test_keyframes_cannot_fabricate_journal_identity_or_timestamp(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory))
            payload = json.loads((attempt / "visual-keyframes.json").read_text())
            payload["keyframes"][0]["request_sequence"] = 99
            payload["keyframes"][1]["execution_event_sequence"] = 99
            payload["keyframes"][2]["requested_simulated_timestamp"] = 99.0
            (attempt / "visual-keyframes.json").write_text(json.dumps(payload))
            result = process_attempt(attempt)
            codes = {item["code"] for item in result["diagnostics"]}
            self.assertIn("keyframe-request-journal-mismatch", codes)
            self.assertIn("keyframe-execution-journal-mismatch", codes)
            self.assertIn("keyframe-requested-timestamp-journal-mismatch", codes)

    def test_keyframes_without_checksums_remain_valid(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            result = process_attempt(_make_attempt(Path(directory)))
            self.assertEqual(result["status"], "valid")

    def test_corrupt_and_transparent_images_block_validity(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory))
            source = attempt / "frames/start-overview.png"
            source.write_bytes(b"not a png")
            result = process_attempt(attempt)
            self.assertIn("corrupt-image", {item["code"] for item in result["diagnostics"]})
            attempt = _make_attempt(Path(directory) / "transparent")
            source = attempt / "frames/start-overview.png"
            Image.new("RGBA", IMAGE_SIZE, (0, 0, 0, 0)).save(source, format="PNG")
            result = process_attempt(attempt)
            self.assertIn("blank-or-transparent", {item["code"] for item in result["diagnostics"]})

    def test_blank_failure_blocks_validity(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            result = process_attempt(_make_attempt(Path(directory), bad="blank"))
            self.assertEqual(result["status"], "evidence-invalid")
            self.assertIn("blank-or-transparent", {item["code"] for item in result["diagnostics"]})

    def test_missing_and_dimension_failures_are_reported(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory), bad="missing")
            result = process_attempt(attempt)
            codes = {item["code"] for item in result["diagnostics"]}
            self.assertIn("missing-source-image", codes)
            with self.subTest("dimension"):
                attempt = _make_attempt(Path(directory) / "second", bad="dimension")
                result = process_attempt(attempt)
                self.assertIn("invalid-dimensions", {item["code"] for item in result["diagnostics"]})

    def test_timestamp_skew_and_non_monotonic_are_reported(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory), bad="stale")
            payload = json.loads((attempt / "visual-keyframes.json").read_text())
            payload["keyframes"][2]["sim_time"] = -1
            payload["keyframes"][3]["sim_time"] = 0.2
            (attempt / "visual-keyframes.json").write_text(json.dumps(payload))
            result = process_attempt(attempt)
            codes = {item["code"] for item in result["diagnostics"]}
            self.assertIn("non-monotonic-simulated-timestamps", codes)
            self.assertIn("timestamp-skew", codes)

    def test_capture_latency_contract_accepts_observed_boundary(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory))
            payload = json.loads((attempt / "visual-keyframes.json").read_text())
            frame = payload["keyframes"][0]
            frame["requested_physics_frame_index"] = frame["raw_frame_index"] - 4
            frame["capture_latency_frames"] = 4
            (attempt / "visual-keyframes.json").write_text(json.dumps(payload))
            result = process_attempt(attempt)
            codes = {item["code"] for item in result["diagnostics"]}
            self.assertNotIn("capture-latency-out-of-bounds", codes)
            self.assertNotIn("capture-latency-frame-mismatch", codes)

    def test_capture_latency_beyond_contract_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory))
            payload = json.loads((attempt / "visual-keyframes.json").read_text())
            frame = payload["keyframes"][0]
            frame["requested_physics_frame_index"] = frame["raw_frame_index"] - 5
            frame["capture_latency_frames"] = 5
            (attempt / "visual-keyframes.json").write_text(json.dumps(payload))
            result = process_attempt(attempt)
            self.assertIn("capture-latency-out-of-bounds", {item["code"] for item in result["diagnostics"]})

    def test_event_sequence_metadata_must_match_between_cameras_and_be_ordered(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory))
            payload = json.loads((attempt / "visual-keyframes.json").read_text())
            payload["keyframes"][1]["execution_event_sequence"] = 99
            payload["keyframes"][2]["request_sequence"] = 1
            (attempt / "visual-keyframes.json").write_text(json.dumps(payload))
            result = process_attempt(attempt)
            codes = {item["code"] for item in result["diagnostics"]}
            self.assertIn("execution-event-sequence-mismatch", codes)
            self.assertIn("non-monotonic-visual-request-sequence", codes)

    def test_metadata_identity_and_unindexed_source_failures(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory))
            payload = json.loads((attempt / "visual-keyframes.json").read_text())
            payload["keyframes"][0]["camera"] = "side"
            del payload["keyframes"][1]["raw_frame_id"]
            del payload["keyframes"][1]["raw_frame_index"]
            del payload["physics_frame_s"]
            extra = attempt / "frames/unindexed.png"
            _image(extra, 99)
            (attempt / "visual-keyframes.json").write_text(json.dumps(payload))
            result = process_attempt(attempt)
            codes = {item["code"] for item in result["diagnostics"]}
            self.assertIn("unexpected-camera", codes)
            self.assertIn("missing-raw-frame-id-index", codes)
            self.assertIn("missing-physics-frame", codes)
            self.assertIn("unindexed-source-image", codes)

    def test_suite_generates_both_sheets_and_requires_all_gates(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for gate in GATE_EVENTS:
                _make_attempt(root, gate)
            result = process_suite(root)
            self.assertEqual(result["status"], "valid")
            for name in ("contact-sheet-agent.png", "contact-sheet-user.png", RESULT_NAME):
                self.assertTrue((root / name).is_file())
            with Image.open(root / "contact-sheet-agent.png") as image:
                self.assertEqual(image.mode, "RGB")
                self.assertEqual(image.size, (1812, 1480))
            with Image.open(root / "contact-sheet-user.png") as image:
                self.assertEqual(image.size, (1422, 1372))
            missing = root / "retention"
            for path in missing.rglob("*"):
                if path.is_file():
                    path.unlink()
            for path in sorted(missing.rglob("*"), reverse=True):
                if path.is_dir():
                    path.rmdir()
            result = process_suite(root)
            self.assertEqual(result["status"], "evidence-invalid")
            self.assertIn("missing-gate-attempt", {item["code"] for item in result["diagnostics"]})

    def test_cli_returns_nonzero_for_invalid_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory), bad="missing")
            self.assertEqual(main(["--attempt-dir", str(attempt)]), 1)

    def test_repeatable_render_bytes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            attempt = _make_attempt(Path(directory))
            process_attempt(attempt)
            first = (attempt / "contact-sheet-diagnostic.png").read_bytes()
            process_attempt(attempt)
            self.assertEqual(first, (attempt / "contact-sheet-diagnostic.png").read_bytes())


if __name__ == "__main__":
    unittest.main()
