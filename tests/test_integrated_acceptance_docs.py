"""Task 10: docs-acceptance contract tests for the integrated OMPL qualification.

Python 3.12, ROS-free: this suite only reads the three tracked documentation
files and asserts the exact integrated-qualification CLI contract documented by
Task 10.  It never imports ``rclpy``, generated ROS messages, geometry
packages, or the validation tooling, and it never launches any process or
writes any file.  It asserts the exact command blocks/paths, fresh-suite
retention wording, bounded build command, three-lock sequence, live-only
caveats, and cuMotion prohibition so a documentation regression fails closed.
"""
from __future__ import annotations

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

# The Task 10 integrated-qualification CLI section is documented verbatim in
# exactly these three tracked files. The README links to
# docs/integrated-ompl-qualification.md rather than inlining this content, so
# that file is the tracked copy in place of README.md.
DOC_PATHS = {
    "acceptance": ROOT / "docs" / "acceptance.md",
    "manipulation": ROOT / "integration" / "MANIPULATION.md",
    "integrated_ompl_qualification": ROOT / "docs" / "integrated-ompl-qualification.md",
}

DOC_NAMES = tuple(DOC_PATHS)


@pytest.fixture(scope="module")
def doc_texts() -> dict[str, str]:
    texts: dict[str, str] = {}
    for name, path in DOC_PATHS.items():
        assert path.is_file(), f"documentation file missing: {path}"
        texts[name] = path.read_text(encoding="utf-8")
    return texts


def assert_in_all(doc_texts: dict[str, str], snippet: str) -> None:
    for name in DOC_NAMES:
        assert snippet in doc_texts[name], (
            f"expected snippet not present in {name}: {snippet!r}"
        )


# --- exact command blocks and paths -----------------------------------------


def test_documentation_files_exist_and_are_readable(doc_texts):
    for name, text in doc_texts.items():
        assert text.strip(), f"documentation file is empty: {name}"


def test_suite_dir_variable_exact(doc_texts):
    assert_in_all(
        doc_texts,
        "SUITE_DIR=outputs/integrated/integrated-ompl-seed-7",
    )


def test_gate_a_command_exact(doc_texts):
    assert_in_all(
        doc_texts,
        "./.venv/bin/python validation/integrated_qualification.py \\\n"
        '  --attempt-root "$SUITE_DIR" --stage A',
    )


def test_gate_b_offline_command_exact(doc_texts):
    assert_in_all(
        doc_texts,
        "./.venv/bin/python validation/integrated_qualification.py \\\n"
        '  --attempt-root "$SUITE_DIR" --stage B --offline',
    )


def test_gate_c_command_exact(doc_texts):
    assert_in_all(
        doc_texts,
        "./.venv/bin/python validation/integrated_qualification.py \\\n"
        '  --attempt-root "$SUITE_DIR" --stage C',
    )


def test_gate_d_command_exact(doc_texts):
    assert_in_all(
        doc_texts,
        "./.venv/bin/python validation/integrated_qualification.py \\\n"
        '  --attempt-root "$SUITE_DIR" --stage D',
    )


def test_gate_e_command_exact(doc_texts):
    assert_in_all(
        doc_texts,
        "./.venv/bin/python validation/integrated_qualification.py \\\n"
        '  --attempt-root "$SUITE_DIR" --stage E',
    )


def test_gate_f_command_exact(doc_texts):
    assert_in_all(
        doc_texts,
        "./.venv/bin/python validation/integrated_qualification.py \\\n"
        '  --attempt-root "$SUITE_DIR" --stage F',
    )


def test_stage_all_command_exact(doc_texts):
    assert_in_all(
        doc_texts,
        "./.venv/bin/python validation/integrated_qualification.py \\\n"
        '  --attempt-root "$SUITE_DIR" --stage all',
    )


def test_verifier_replay_selects_exact_single_attempt(doc_texts):
    # The replay must bind a runnable deterministic selection that finds the
    # exact single immutable matching attempt under $SUITE_DIR, fails unless
    # exactly one match is found, and passes --attempt-dir "$ATTEMPT_DIR".
    assert_in_all(
        doc_texts,
        "The selection\n"
        "below finds the exact single immutable matching attempt under `$SUITE_DIR`,\n"
        "fails unless exactly one match is found, and binds it to `ATTEMPT_DIR`:",
    )
    assert_in_all(
        doc_texts,
        'ATTEMPT_DIR="$(find "$SUITE_DIR" -maxdepth 1 -type d \\\n'
        "  -name 'C-qualification-moveit-plan-joint-*' | sort)\"",
    )
    assert_in_all(
        doc_texts,
        "test \"$(printf '%s\\n' \"$ATTEMPT_DIR\" | sed '/^$/d' | wc -l)\" -eq 1",
    )
    assert_in_all(
        doc_texts,
        "./.venv/bin/python validation/integrated_gate_verifier.py \\\n"
        "  --scenario qualification-moveit-plan-joint \\\n"
        '  --attempt-dir "$ATTEMPT_DIR" \\\n'
        "  --config simulation/qualification/integrated-ompl.json",
    )


def test_verifier_replay_has_no_fake_attempt_path(doc_texts):
    # The stale fake replay path must not appear anywhere.
    for text in doc_texts.values():
        assert "C-qualification-moveit-plan-joint-1-0" not in text


def test_contact_sheet_regeneration_command_exact(doc_texts):
    assert_in_all(
        doc_texts,
        './.venv/bin/python validation/integrated_contact_sheets.py --suite-dir "$SUITE_DIR"',
    )


def test_evidence_index_validate_command_exact(doc_texts):
    assert_in_all(
        doc_texts,
        "./.venv/bin/python validation/integrated_evidence_index.py \\\n"
        '  --suite-dir "$SUITE_DIR" --summary "$SUITE_DIR/qualification-summary.json" \\\n'
        "  --validate",
    )


def test_attempt_root_is_the_suite_dir(doc_texts):
    assert_in_all(
        doc_texts,
        "`SUITE_DIR` is passed as the runner's `--attempt-root`.",
    )


def test_core_suite_sibling_root_exact(doc_texts):
    assert_in_all(
        doc_texts,
        "outputs/integrated/integrated-ompl-seed-7-core/",
    )


# --- fresh-suite retention wording ------------------------------------------


def test_fresh_suite_retention_wording(doc_texts):
    assert_in_all(
        doc_texts,
        "never delete or reuse a failed, stale, or\n"
        "old attempt or suite",
    )
    assert_in_all(
        doc_texts,
        "repeated allocation yields distinct preserved paths",
    )
    assert_in_all(
        doc_texts,
        "choose a fresh suite path (a fresh `--attempt-root`)",
    )
    assert_in_all(
        doc_texts,
        "never merge a new\n"
        "run into an old suite",
    )


def test_suite_dir_is_exact_not_appended(doc_texts):
    assert_in_all(
        doc_texts,
        "It is the exact\n"
        "integrated suite directory; nothing is silently appended below it.",
    )


def test_repeated_run_chooses_fresh_attempt_root(doc_texts):
    assert_in_all(
        doc_texts,
        "A repeated\n"
        "qualification run must choose a fresh `--attempt-root`; never merge a new run\n"
        "into an old suite.",
    )


# --- bounded build command ---------------------------------------------------


def test_bounded_build_command_exact(doc_texts):
    assert_in_all(
        doc_texts,
        "MAKEFLAGS='-j2 -l2' ./scripts/build-humble-overlay",
    )


def test_bounded_build_never_raw_colcon(doc_texts):
    # The wrapper ignores CLI args and internally executes colcon with
    # --parallel-workers 2; no external --parallel-workers is appended.
    assert_in_all(
        doc_texts,
        "Bounded build command (never raw colcon).  The wrapper ignores CLI args and\n"
        "internally executes colcon with `--parallel-workers 2`:",
    )


# --- three-lock sequence -----------------------------------------------------


def test_three_source_lock_roles(doc_texts):
    assert_in_all(
        doc_texts,
        "`simulator_overlay` / `production` / `qualification_tooling`",
    )
    assert_in_all(
        doc_texts,
        "The runtime config has three source-lock roles",
    )


def test_qualification_tooling_lock_created_after_review_clean(doc_texts):
    assert_in_all(
        doc_texts,
        "qualification-tooling source-lock role is created only after Task 10 is\n"
        "  review-clean, in a separate lock-only commit, and only before live attempts.",
    )


# --- live-only caveats -------------------------------------------------------


def test_no_live_claim_from_task_10_offline_tests(doc_texts):
    assert_in_all(
        doc_texts,
        "No live Gate F/OMPL/cuMotion claim comes from Task 10's offline tests",
    )


def test_image_stats_requires_live_rtx_calibration(doc_texts):
    assert_in_all(
        doc_texts,
        "`_image_stats` thresholds still require live RTX calibration",
    )


# --- cuMotion prohibition ----------------------------------------------------


def test_cumotion_prohibition_until_task_37(doc_texts):
    assert_in_all(
        doc_texts,
        "cuMotion remains prohibited until Task 37's live OMPL qualification "
        "passes",
    )


# --- offline flag scoping ----------------------------------------------------


def test_offline_flag_is_stage_b_compatibility_only(doc_texts):
    assert_in_all(
        doc_texts,
        "`--offline` is an explicit compatibility flag\n"
        "for the already-offline B implementation and must not make any live stage\n"
        "offline or bypass checks",
    )


# --- standalone sequence vs --stage all alternatives -------------------------


def test_stage_all_is_alternative_to_standalone_sequence(doc_texts):
    assert_in_all(
        doc_texts,
        "The standalone Gates A through F above and `--stage all` are alternatives.",
    )
    assert_in_all(
        doc_texts,
        "`--stage all` must use a fresh suite path and must not be run against a suite\n"
        "that already has write-once A-E stage records.",
    )


# --- integrated suite path immutability --------------------------------------


def test_exact_immutable_suite_path_used_everywhere(doc_texts):
    # The immutable suite path must be used with the one consistent variable
    # form and must never appear with an appended nested suite directory.
    for name, text in doc_texts.items():
        assert "integrated-ompl-seed-7-core" in text
        # No silently-appended nested suite directory below the documented one.
        assert "integrated-ompl-seed-7/integrated" not in text
