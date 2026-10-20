import json

import pytest

from map_check.bench import BenchStore, JobStatus, RenderRecord
from map_check.poses import Pose, export_code

POSES_YAML = """map: "m1"
frame_id: map
poses:
  "diningroom_table_2_0": {x: 12.8350, y: 5.4500, yaw: -1.5708}
  "kitchen_a": {x: 1.0000, y: 2.0000, yaw: 0.0000}
  "kitchen_b": {x: 3.0000, y: 2.0000, yaw: 0.5000}
"""


@pytest.fixture
def maps_dir(tmp_path):
    d = tmp_path / "maps"
    d.mkdir()
    (d / "m1.poses.yaml").write_text(POSES_YAML)
    return d


@pytest.fixture
def store(tmp_path):
    return BenchStore(tmp_path / "state")


def job(status):
    return JobStatus(id="j1", map="m1", status=status, queued_at=1.0, started_at=None,
                     finished_at=None, done_views=0, total_views=3, position=None, message="")


def test_defaults_and_persist(tmp_path):
    s = BenchStore(tmp_path)
    st = s.get("m1")
    assert st.version == 0 and st.arm_pose == "arm_pos_navigating" and st.rows == {}
    st, errs = s.apply("m1", [{"op": "set_row", "row": "saved:kitchen_a", "fields": {"included": True}}])
    assert st.version == 1 and errs == []
    again = BenchStore(tmp_path).get("m1")
    assert again == st and again.rows["saved:kitchen_a"].included is True


def test_set_rows_bulk(store):
    keys = ["saved:a", "saved:b", "saved:c"]
    st, _ = store.apply("m1", [{"op": "set_rows", "rows": keys, "fields": {"behind": False}}])
    st, _ = store.apply("m1", [{"op": "set_row", "row": "saved:d", "fields": {"top": True}}])
    assert [st.rows[k].behind for k in keys] == [False] * 3
    assert st.rows["saved:d"].behind is True and st.rows["saved:d"].top is True


def test_field_validation(store):
    with pytest.raises(ValueError):
        store.apply("m1", [{"op": "bogus"}])
    with pytest.raises(ValueError):
        store.apply("m1", [{"op": "set_row", "row": "saved:a", "fields": {"nope": 1}}])
    with pytest.raises(ValueError):
        store.apply("m1", [{"op": "set_row", "row": "saved:a", "fields": {"behind": "yes"}}])
    with pytest.raises(ValueError):
        store.apply("../x", [])
    st, _ = store.apply("m1", [{"op": "set_row", "row": "saved:a",
                                "fields": {"pan_deg": 500, "tilt_deg": -99}}])
    assert st.rows["saved:a"].pan_deg == 180 and st.rows["saved:a"].tilt_deg == -30
    assert store.get("m1").version == 1  # failed batches did not bump


@pytest.mark.parametrize("field", ["pan_deg", "tilt_deg"])
@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_pan_tilt_rejected(store, field, value):
    with pytest.raises(ValueError, match=f"{field}.*finite"):
        store.apply("m1", [{"op": "set_row", "row": "saved:a", "fields": {field: value}}])
    assert store.get("m1").version == 0


def test_remove_and_restore_saved(store, maps_dir):
    assert len(store.view("m1", maps_dir).rows) == 3
    store.apply("m1", [{"op": "remove", "row": "saved:kitchen_a"}])
    v = store.view("m1", maps_dir)
    assert "saved:kitchen_a" not in [r.key for r in v.rows] and v.removed_count == 1
    store.apply("m1", [{"op": "restore_saved"}])
    v = store.view("m1", maps_dir)
    assert "saved:kitchen_a" in [r.key for r in v.rows] and v.removed_count == 0


def test_remove_pasted_deletes(store):
    text = export_code([Pose("POS_NEW_ONE", 1.0, 1.0, 0.0)])
    st, _ = store.apply("m1", [{"op": "add_pasted", "text": text}])
    assert [p.name for p in st.pasted] == ["POS_NEW_ONE"]
    st, _ = store.apply("m1", [{"op": "remove", "row": "pasted:POS_NEW_ONE"}])
    assert st.pasted == [] and st.removed_saved == []
    st, _ = store.apply("m1", [{"op": "add_pasted", "text": text}, {"op": "clear_pasted"}])
    assert st.pasted == []


def test_add_pasted_errors_reported(store):
    good = export_code([Pose("POS_GOOD", 1.0, 2.0, 0.0)])
    bad = "POS_BAD = PoseStamped(header=Header(), pose=Pose())\n"
    st, errs = store.apply("m1", [{"op": "add_pasted", "text": good + "\n" + bad}])
    assert [p.name for p in st.pasted] == ["POS_GOOD"] and len(errs) == 1
    # re-paste replaces
    st, _ = store.apply("m1", [{"op": "add_pasted", "text": export_code([Pose("POS_GOOD", 9.0, 2.0, 0.0)])}])
    assert len(st.pasted) == 1 and st.pasted[0].x == 9.0


def test_view_follows_map_studio(store, maps_dir):
    store.apply("m1", [{"op": "remove", "row": "saved:kitchen_b"}])
    (maps_dir / "m1.poses.yaml").write_text(POSES_YAML + '  "hall": {x: 0.0, y: 0.0, yaw: 0.0}\n')
    v = store.view("m1", maps_dir)
    names = [r.pose.name for r in v.rows]
    assert "hall" in names and "kitchen_b" not in names
    assert all(r.group == "saved" for r in v.rows) and v.rows[0].state.included is False


def test_running_job_becomes_interrupted(tmp_path):
    s = BenchStore(tmp_path)
    s.set_job("m1", job("running"))
    s.set_job("m2", job("done"))
    s2 = BenchStore(tmp_path)
    assert s2.get("m1").job.status == "interrupted"
    assert s2.get("m2").job.status == "done"
    assert s.get("m1").version == 1


def test_render_record_roundtrip(tmp_path):
    from map_check.overlap import Flag
    rec = RenderRecord(job_id="j1", finished_at=2.0, arm_pose="arm_pos_navigating",
                       items=[{"key": "saved:a", "name": "a",
                               "flags": [Flag("overlap", "head", "wall", -0.03, "t")],
                               "views": [{"view": "behind", "url": "/u", "caption": "c"}]}],
                       overview_url=None)
    s = BenchStore(tmp_path)
    s.set_last_render("m1", rec)
    got = BenchStore(tmp_path).get("m1").last_render
    assert got == rec
    json.loads((tmp_path / "bench" / "m1.json").read_text())


def test_clash_in_view(store, maps_dir):
    text = export_code([Pose("POS_DININGROOM_TABLE_2_0", 12.95, 5.45, -1.5708)])
    store.apply("m1", [{"op": "add_pasted", "text": text}])
    v = store.view("m1", maps_dir)
    row = next(r for r in v.rows if r.key == "pasted:POS_DININGROOM_TABLE_2_0")
    assert row.clash_m == pytest.approx(0.115)
    assert next(r for r in v.rows if r.group == "saved").clash_m is None
