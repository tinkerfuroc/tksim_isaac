"""Pure maths of tools/rtf_monitor.py (no rclpy needed)."""
import importlib.util
import json
from pathlib import Path

_PATH = Path(__file__).resolve().parents[1] / "tools" / "rtf_monitor.py"
_spec = importlib.util.spec_from_file_location("rtf_monitor", _PATH)
rtf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rtf)


def _series(rate, wall0=1000.0, sim0=0.0, n=61, dt=0.5):
    return [(wall0 + i * dt, sim0 + i * dt * rate) for i in range(n)]


def test_window_rtf_steady_rate():
    s = _series(0.4)
    assert abs(rtf.window_rtf(s, now_wall=s[-1][0], window_s=10.0) - 0.4) < 1e-9


def test_window_rtf_only_uses_the_window():
    s = _series(1.0, n=40) + [(1000.0 + 20.0 + i * 0.5, 20.0 + i * 0.5 * 0.2) for i in range(1, 41)]
    assert abs(rtf.window_rtf(s, now_wall=s[-1][0], window_s=10.0) - 0.2) < 1e-9


def test_window_rtf_needs_one_sample_and_positive_wall_span():
    assert rtf.window_rtf([], now_wall=5.0, window_s=10.0) is None
    assert rtf.window_rtf([(5.0, 1.0)], now_wall=5.0, window_s=10.0) is None  # zero span


def test_window_rtf_sees_a_stalled_clock():
    # 5 s of samples at RTF 0.5, then /clock goes silent for 25 s (now = last sample + 25).
    s = [(1000.0 + i * 0.5, i * 0.5 * 0.5) for i in range(11)]  # wall 1000..1005, sim 0..2.5
    got = rtf.window_rtf(s, now_wall=1030.0, window_s=30.0)
    assert abs(got - 2.5 / 30.0) < 1e-9
    assert got < rtf.RTF_WARN_THRESHOLD


def test_window_rtf_single_sample_with_elapsed_wall_is_zero():
    assert rtf.window_rtf([(1000.0, 7.0)], now_wall=1010.0, window_s=10.0) == 0.0


def test_window_rtf_restarts_after_clock_rewind():
    before = [(1000.0 + i, 500.0 + i * 0.5) for i in range(6)]       # sim 500..502.5
    after = [(1006.0 + i, 0.0 + i * 0.3) for i in range(6)]          # reset rewinds to 0
    got = rtf.window_rtf(before + after, now_wall=1011.0, window_s=30.0)
    assert abs(got - 0.3) < 1e-9


def test_warn_state_crossing_reminder_and_recovery():
    w = rtf.WarnState(threshold=0.5, remind_s=60.0)
    assert w.update(0.6, 0.0) is None
    assert "WARN rtf_30s=0.40 < 0.50" in w.update(0.4, 10.0)
    assert w.update(0.4, 20.0) is None                 # no spam
    assert "WARN" in w.update(0.45, 71.0)              # 60 s reminder
    assert w.update(0.55, 80.0).startswith("OK")       # recovery is logged once
    assert w.update(0.55, 90.0) is None
    assert w.update(None, 95.0) is None                # no data yet: silent


def test_summarise_time_weighted_mean_and_min():
    recs = [
        {"wall": 0.0, "sim": 0.0, "rtf_10s": None, "rtf_30s": None},
        {"wall": 10.0, "sim": 6.0, "rtf_10s": 0.6, "rtf_30s": 0.6},
        {"wall": 20.0, "sim": 8.0, "rtf_10s": 0.2, "rtf_30s": 0.4},
        {"wall": 30.0, "sim": 14.0, "rtf_10s": 0.6, "rtf_30s": 0.47},
    ]
    out = rtf.summarise(recs, t_from=0.0, t_to=30.0)
    assert abs(out["mean"] - 14.0 / 30.0) < 1e-9
    assert out["min_30s"] == 0.4
    assert out["samples"] == 4


def test_summarise_skips_rewind_pairs():
    recs = [
        {"wall": 0.0, "sim": 100.0, "rtf_10s": None, "rtf_30s": None},
        {"wall": 10.0, "sim": 105.0, "rtf_10s": 0.5, "rtf_30s": 0.5},
        {"wall": 20.0, "sim": 1.0, "rtf_10s": None, "rtf_30s": None},   # reset
        {"wall": 30.0, "sim": 6.0, "rtf_10s": 0.5, "rtf_30s": 0.5},
    ]
    out = rtf.summarise(recs, t_from=0.0, t_to=30.0)
    assert abs(out["mean"] - 0.5) < 1e-9


def test_summarise_empty_window():
    out = rtf.summarise([{"wall": 0.0, "sim": 0.0, "rtf_10s": None, "rtf_30s": None}], 50.0, 60.0)
    assert out == {"mean": None, "min_30s": None, "samples": 0}


def test_default_out_path_uses_run_dir(tmp_path):
    assert rtf.default_out_path({"GPSR_RUN_DIR": str(tmp_path)}) == tmp_path / "rtf.jsonl"
    assert rtf.default_out_path({}) == Path("rtf.jsonl")


def test_summarise_cli(tmp_path, capsys):
    path = tmp_path / "rtf.jsonl"
    path.write_text("\n".join(json.dumps(r) for r in [
        {"wall": 0.0, "sim": 0.0, "rtf_10s": None, "rtf_30s": None},
        {"wall": 10.0, "sim": 5.0, "rtf_10s": 0.5, "rtf_30s": 0.5},
    ]) + "\n")
    assert rtf.main(["--summarise", str(path), "--from", "0", "--to", "10"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out == {"mean": 0.5, "min_30s": 0.5, "samples": 2}


def test_summarise_cli_skips_bad_lines(tmp_path, capsys):
    path = tmp_path / "rtf.jsonl"
    path.write_text("\n".join([
        json.dumps({"wall": 0.0, "sim": 0.0, "rtf_10s": None, "rtf_30s": None}),
        json.dumps({"wall": 10.0, "sim": 5.0, "rtf_10s": 0.5, "rtf_30s": 0.5}),
        '{"wall": 20.0, "si',                                        # truncated mid-write
        json.dumps({"wall": 30.0, "rtf_10s": 0.5, "rtf_30s": 0.5}),  # no "sim"
    ]) + "\n")
    assert rtf.main(["--summarise", str(path), "--from", "0", "--to", "40"]) == 0
    captured = capsys.readouterr()
    assert json.loads(captured.out) == {"mean": 0.5, "min_30s": 0.5, "samples": 2}
    assert "skipped 2" in captured.err


def test_summarise_cli_missing_file(tmp_path, capsys):
    assert rtf.main(["--summarise", str(tmp_path / "nope.jsonl"), "--from", "0", "--to", "10"]) == 1
    assert "cannot read" in capsys.readouterr().err


def test_load_records_counts_only_unusable_lines():
    lines = ['{"wall": 0.0, "sim": 0.0}', "", '{"wall": 10.0, "si', "[1, 2]",
             '{"wall": 1.0}', '{"wall": "x", "sim": 1.0}']
    records, skipped = rtf.load_records(lines)
    assert records == [{"wall": 0.0, "sim": 0.0}]
    assert skipped == 4


def test_append_record_creates_nested_dir_and_appends(tmp_path):
    out = tmp_path / "a" / "b" / "rtf.jsonl"
    assert rtf.append_record(out, {"wall": 1.0, "sim": 2.0}) is None
    assert rtf.append_record(out, {"wall": 3.0, "sim": 4.0}) is None
    lines = out.read_text(encoding="utf-8").splitlines()
    assert [json.loads(line) for line in lines] == [{"wall": 1.0, "sim": 2.0}, {"wall": 3.0, "sim": 4.0}]


def test_append_record_reports_unwritable_target(tmp_path):
    blocker = tmp_path / "file"
    blocker.write_text("x", encoding="utf-8")
    err = rtf.append_record(blocker / "rtf.jsonl", {"wall": 1.0, "sim": 2.0})
    assert isinstance(err, str) and err
