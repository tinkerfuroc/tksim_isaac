"""HTTP API tests: FastAPI TestClient, fake render command, fake GPU probe, stub robot."""
import hashlib
import io
import json
import sys
import textwrap
import threading
from pathlib import Path

import numpy as np
import pytest
from fastapi.testclient import TestClient
from PIL import Image

from map_check.config import Settings
from map_check.gpu import BusyReason, GpuStatus
from map_check.overlap import Flag
from map_check.server import create_app, origin_allowed

ARM = [-87.4, -58.8, -2.4, 8.0, 12.3, -68.5, -8.2]
HEADERS = {"Origin": "http://localhost:9081", "Host": "localhost:9081"}

FAKE = textwrap.dedent("""
    import json, os, sys
    job = json.load(open(sys.argv[-1]))
    os.makedirs(job["out_dir"], exist_ok=True)
    views = [("overview", "overview")] + [(p["key"], v) for p in job["poses"] for v in p["views"]]
    for i, (key, view) in enumerate(views):
        name = f"{i:02d}_{view}.png"
        open(os.path.join(job["out_dir"], name), "wb").close()
        print(json.dumps({"event": "view", "key": key, "view": view, "file": name}), flush=True)
    print(json.dumps({"event": "done"}), flush=True)
""")


class StubRobot:
    def flags(self, pose, arm_deg, pan_deg, tilt_deg, boxes, bounds):
        if pose.name == "a":
            return [Flag("overlap", "base", "wall", -0.03, "base overlaps wall by 3 cm")]
        return []


def free_gpu(own_pids=frozenset()):
    return GpuStatus(gpus=[0, 1], busy=[])


def busy_gpu(own_pids=frozenset()):
    return GpuStatus(gpus=[0, 1], busy=[BusyReason(0, 4242, "gpsr-stack", "scripts/gpsr-stack")])


def map_files(size=24):
    """A ring of walls with a closed room in it, as (yaml bytes, pgm bytes)."""
    a = np.full((size, size), 254, dtype=np.uint8)
    a[2, 2:-2] = a[-3, 2:-2] = a[2:-2, 2] = a[2:-2, -3] = 0
    a[8:12, 8:12] = 0
    a[9:11, 9:11] = 254  # a closed outline
    buf = io.BytesIO()
    Image.fromarray(a).save(buf, format="PNG")
    yaml = (b"image: m.png\nresolution: 0.05\norigin: [-1.0, -1.0, 0.0]\n"
            b"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return yaml, buf.getvalue()


@pytest.fixture
def settings(tmp_path):
    arm = tmp_path / "constants.json"
    arm.write_text(json.dumps({"arm_pos_navigating": ARM, "arm_pos_other": ARM}))
    (tmp_path / "maps").mkdir()
    return Settings(maps_dir=tmp_path / "maps", arena_root=tmp_path / "arena",
                    state_dir=tmp_path / "state", repo_root=tmp_path, arm_poses_json=arm)


def fake_command(tmp_path):
    script = tmp_path / "render.py"
    script.write_text(FAKE)
    return lambda jobfile: [sys.executable, str(script), str(jobfile)]


@pytest.fixture
def make_client(settings, tmp_path):
    made = []

    def make(gpu=free_gpu, **kw):
        c = TestClient(create_app(settings, fake_command(tmp_path), robot_model=StubRobot(),
                                  gpu_probe=gpu, poll_interval=0.02, **kw))
        c.__enter__()
        made.append(c)
        return c

    yield make
    for c in made:
        c.__exit__(None, None, None)


@pytest.fixture
def client(make_client):
    return make_client()


def upload(client, name_file="foo.yaml"):
    yaml, img = map_files()
    r = client.post("/api/map/upload", files={
        "yaml": (name_file, yaml, "text/yaml"), "image": ("m.png", img, "image/png")})
    assert r.status_code == 200, r.text
    return r.json()


def save(client, token, name, **extra):
    body = {"token": token, "name": name, "wall_height": 0.6, "shapes": {}, "erased": []}
    body.update(extra)
    return client.post("/api/map/save", json=body)


def test_cross_origin_refused(client):
    assert client.get("/api/maps", headers={"Origin": "http://evil.example"}).status_code == 403
    assert client.get("/api/maps").status_code == 200
    assert client.get("/api/maps", headers=HEADERS).status_code == 200
    assert not origin_allowed("http://localhost.evil.example", "localhost.evil.example")


def test_upload_then_save_new_map(client, settings):
    up = upload(client)
    assert up["name_suggestion"] == "foo" and up["identical_to"] is None
    assert up["grid"]["W"] == 24 and up["outlines"] and up["outlines"][0]["id"] == 1
    r = save(client, up["token"], "bar")
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["name"] == "bar" and body["boxes"] > 0
    assert (settings.maps_dir / "bar.yaml").is_file()
    assert (settings.arena_root / "bar" / "current.json").is_file()
    maps = client.get("/api/maps").json()
    assert "bar" in maps["studio"] and "bar" in maps["arenas"]


def test_upload_identical_reports_existing(client):
    up = upload(client)
    save(client, up["token"], "bar")
    assert upload(client)["identical_to"] == "bar"


def test_save_taken_name_409(client):
    up = upload(client)
    assert save(client, up["token"], "bar").status_code == 200
    r = save(client, up["token"], "bar")
    assert r.status_code == 409
    assert "already exists" in r.json()["detail"]


@pytest.mark.parametrize("bad", ["", "../x", ".hidden", "a b", "x.poses", "a/b"])
def test_save_bad_name_400(client, settings, bad):
    up = upload(client)
    r = save(client, up["token"], bad)
    assert r.status_code == 400 and isinstance(r.json()["detail"], str)
    assert list(settings.maps_dir.iterdir()) == []


def test_save_unknown_token_and_bad_body(client):
    assert save(client, "nope", "bar").status_code == 404
    up = upload(client)
    r = save(client, up["token"], "bar", shapes={"99": {"fill": True, "height": 0.5}})
    assert r.status_code == 400
    r = save(client, up["token"], "bar", erased=[10 ** 9])
    assert r.status_code == 400
    r = client.post("/api/map/save", json={"token": 1})
    assert r.status_code == 422 and isinstance(r.json()["detail"], str)


def test_upload_bad_files(client):
    r = client.post("/api/map/upload", files={
        "yaml": ("a.yaml", b"image: gone.png\nresolution: 0.05\norigin: [0, 0, 0]\n", "text/yaml"),
        "image": ("m.png", b"x", "image/png")})
    assert r.status_code == 422 and "gone.png" in r.json()["detail"]


def test_picked_map_save_versions(client, settings):
    yaml, img = map_files()
    (settings.maps_dir / "pick.yaml").write_bytes(yaml)
    (settings.maps_dir / "m.png").write_bytes(img)
    before = [(p.stat().st_mtime_ns, hashlib.sha256(p.read_bytes()).hexdigest())
              for p in sorted(settings.maps_dir.iterdir())]
    st = client.get("/api/map/studio/pick")
    assert st.status_code == 200 and st.json()["name_suggestion"] == "pick"
    token = st.json()["token"]
    assert save(client, token, "pick").status_code == 200
    assert save(client, token, "pick", wall_height=0.9).status_code == 200
    versions = [d for d in (settings.arena_root / "pick").iterdir() if d.is_dir()]
    assert len(versions) == 2
    after = [(p.stat().st_mtime_ns, hashlib.sha256(p.read_bytes()).hexdigest())
             for p in sorted(settings.maps_dir.iterdir())]
    assert before == after
    assert "pick" in client.get("/api/maps").json()["arenas"]
    assert client.get("/api/map/studio/missing").status_code == 404
    assert client.get("/api/map/studio/..x").status_code == 400
    # a picked-map token cannot be saved under another name
    assert save(client, token, "other").status_code == 400


def bench_ready(client, settings):
    """Arena 'bar' with Map Studio poses a, b."""
    up = upload(client)
    assert save(client, up["token"], "bar").status_code == 200
    (settings.maps_dir / "bar.poses.yaml").write_text(
        "map: bar\nframe_id: map\nposes:\n  a: {x: 1.0, y: 1.0, yaw: 0.0}\n"
        "  b: {x: 2.0, y: 1.0, yaw: 1.5}\n")


def test_bench_view_flags_and_tilt(client, settings):
    bench_ready(client, settings)
    r = client.get("/api/bench/bar")
    assert r.status_code == 200, r.text
    d = r.json()
    assert d["arena"] is True and d["tilt_down_sign"] in (1, -1)
    assert set(d["arm_poses"]) == {"arm_pos_navigating", "arm_pos_other"}
    rows = {x["key"]: x for x in d["rows"]}
    assert rows["saved:a"]["flags"][0]["text"] == "base overlaps wall by 3 cm"
    assert rows["saved:b"]["flags"] == []
    assert rows["saved:a"]["pose"]["name"] == "a"


def test_bench_without_arena(client, settings):
    (settings.maps_dir / "solo.poses.yaml").write_text(
        "map: solo\nframe_id: map\nposes:\n  a: {x: 1.0, y: 1.0, yaw: 0.0}\n")
    d = client.get("/api/bench/solo").json()
    assert d["arena"] is False and d["rows"][0]["flags"] == []


def test_bench_ops_paste_and_errors(client, settings):
    bench_ready(client, settings)
    r = client.post("/api/bench/bar/ops", json={"ops": [
        {"op": "set_row", "row": "saved:a", "fields": {"included": True, "head": True}},
        {"op": "add_pasted", "text": "POS_BAD = PoseStamped(header=None)"}]})
    assert r.status_code == 200, r.text
    d = r.json()
    assert d["rows"][0]["state"]["included"] is True
    assert d["paste_errors"] and "reason" in d["paste_errors"][0]
    bad = client.post("/api/bench/bar/ops", json={"ops": [{"op": "nope"}]})
    assert bad.status_code == 400 and "nope" in bad.json()["detail"]
    assert client.get("/api/bench/..bad").status_code in (400, 404)


def read_events(client, url, n, out, ready=None):
    """Read the first n SSE events (event name, data) from a bounded stream."""
    events = []
    with client.stream("GET", url) as r:
        name = None
        for line in r.iter_lines():
            if line.startswith("event:"):
                name = line.split(":", 1)[1].strip()
            elif line.startswith("data:"):
                events.append((name, json.loads(line.split(":", 1)[1])))
                if ready is not None and len(events) == 1:
                    ready.set()
                if len(events) >= n:
                    break
    out.extend(events)


def test_bench_ops_and_events(client, settings):
    bench_ready(client, settings)
    v0 = client.get("/api/bench/bar").json()["version"]
    out, ready = [], threading.Event()
    t = threading.Thread(target=read_events,
                         args=(client, "/api/events?map=bar&max_events=2", 2, out, ready))
    t.start()
    assert ready.wait(10)
    r = client.post("/api/bench/bar/ops", json={"ops": [
        {"op": "set_row", "row": "saved:a", "fields": {"tilt_deg": 30}}]})
    assert r.status_code == 200
    t.join(10)
    assert not t.is_alive()
    bench = [d for n, d in out if n == "bench"]
    assert bench[0] == v0 and bench[-1] > v0


def test_events_bad_map_400(client):
    assert client.get("/api/events?map=..x&max_events=1").status_code == 400


def test_export_routes(client, settings):
    bench_ready(client, settings)
    client.post("/api/bench/bar/ops", json={"ops": [
        {"op": "set_row", "row": "saved:b", "fields": {"included": True}}]})
    code = client.get("/api/bench/bar/export?format=code&which=all")
    assert code.status_code == 200 and code.headers["content-type"].startswith("text/plain")
    assert code.text.startswith("from geometry_msgs.msg import")
    assert "POS_A" in code.text and "POS_B" in code.text
    y = client.get("/api/bench/bar/export?format=yaml&which=ticked").text
    assert "frame_id: map" in y and '"b"' in y and '"a"' not in y
    assert client.get("/api/bench/bar/export?format=pdf").status_code == 400
    assert client.get("/api/bench/bar/export?format=code&which=some").status_code == 400


def test_gpu_route(make_client):
    assert make_client(free_gpu).get("/api/gpu").json() == {"busy": False, "text": ""}


def test_gpu_route_busy(make_client):
    d = make_client(busy_gpu).get("/api/gpu").json()
    assert d["busy"] is True and "gpsr-stack" in d["text"]


def test_render_busy_409_then_override(make_client, settings):
    c = make_client(busy_gpu)
    bench_ready(c, settings)
    c.post("/api/bench/bar/ops", json={"ops": [
        {"op": "set_rows", "rows": ["saved:a", "saved:b"], "fields": {"included": True}}]})
    r = c.post("/api/bench/bar/render", json={"override": False})
    assert r.status_code == 409
    assert "gpsr-stack" in r.json()["busy_text"] and isinstance(r.json()["detail"], str)
    r = c.post("/api/bench/bar/render", json={"override": True})
    assert r.status_code == 200, r.text
    assert r.json()["map"] == "bar" and r.json()["total_views"] == 3
    # nothing ticked / unknown map is a 400, not a crash
    assert c.post("/api/bench/nomap/render", json={"override": True}).status_code == 400


def test_job_event_streams(client, settings):
    bench_ready(client, settings)
    client.post("/api/bench/bar/ops", json={"ops": [
        {"op": "set_row", "row": "saved:a", "fields": {"included": True}}]})
    assert client.post("/api/bench/bar/render", json={"override": False}).status_code == 200
    out = []
    read_events(client, "/api/events?map=bar&max_events=2", 2, out)
    assert any(n == "job" and d["map"] == "bar" for n, d in out)


def test_renders_static(client, settings):
    d = settings.state_dir / "renders" / "bar"
    d.mkdir(parents=True)
    (d / "x.png").write_bytes(b"png")
    assert client.get("/renders/bar/x.png").content == b"png"
    assert client.get("/renders/bar/missing.png").status_code == 404


def test_static_not_built_message(client):
    r = client.get("/")
    assert r.status_code == 200 and "no web UI build" in r.text


def test_static_serves_built_ui(settings, tmp_path):
    web = tmp_path / "dist"
    web.mkdir()
    (web / "index.html").write_text("<h1>hi</h1>")
    with TestClient(create_app(settings, fake_command(tmp_path), robot_model=StubRobot(),
                               gpu_probe=free_gpu, web_dir=web)) as c:
        assert "<h1>hi</h1>" in c.get("/").text
        assert c.get("/api/maps").status_code == 200


def test_upload_token_expires(settings, tmp_path):
    clock = [1000.0]
    with TestClient(create_app(settings, fake_command(tmp_path), robot_model=StubRobot(),
                               gpu_probe=free_gpu, clock=lambda: clock[0])) as c:
        up = upload(c)
        clock[0] += 3599
        assert save(c, up["token"], "bar").status_code == 200
        clock[0] += 3601
        assert save(c, up["token"], "bar2").status_code == 404
