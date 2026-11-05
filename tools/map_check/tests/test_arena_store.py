import hashlib
import io
import json
import sys
from dataclasses import replace
from pathlib import Path

import pytest
from PIL import Image

from map_check.arena_build import BuildSettings, Box, ShapeSetting
from map_check.arena_store import ArenaStore, NameTaken
from map_check.config import Settings
from map_check.occupancy import parse_map

WORKTREE = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WORKTREE / "validation"))
import run_sim as rs  # noqa: E402


def png_bytes(seed=0):
    img = Image.new("L", (8, 8), 254)
    for i in range(8):
        img.putpixel((i, seed % 8), 0)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()


def yaml_for(image):
    return (f"image: {image}\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n").encode()


@pytest.fixture
def store(tmp_path):
    s = replace(Settings(), maps_dir=tmp_path / "maps", repo_root=tmp_path,
                arena_root=tmp_path / "artifacts" / "arena")
    s.maps_dir.mkdir()
    return ArenaStore(s)


def snapshot(root):
    return sorted(str(p.relative_to(root)) for p in root.rglob("*"))


def test_upload_refuses_taken_names(store):
    s = store.settings
    (s.maps_dir / "taken.yaml").write_text("x")
    (s.arena_root / "arenaname").mkdir(parents=True)
    before = (snapshot(s.maps_dir), snapshot(s.arena_root))
    for n in ("taken", "arenaname"):
        with pytest.raises(NameTaken):
            store.save_uploaded_map(n, yaml_for("a.png"), {"a.png": png_bytes()})
    assert (snapshot(s.maps_dir), snapshot(s.arena_root)) == before


def test_upload_refuses_bad_names(store):
    with pytest.raises(ValueError):
        store.save_uploaded_map("../x", yaml_for("a.png"), {"a.png": png_bytes()})
    assert snapshot(store.settings.maps_dir) == []
    assert not (store.settings.maps_dir.parent / "x.yaml").exists()


def test_upload_rewrites_image_field(store):
    img = png_bytes()
    out = store.save_uploaded_map("bar", yaml_for("foo.png"), {"foo.png": img})
    assert out == store.settings.maps_dir / "bar.yaml"
    assert "image: bar.png" in out.read_text()
    assert (store.settings.maps_dir / "bar.png").read_bytes() == img
    assert store.map_names() == ["bar"]


def test_identical_existing(store):
    img = png_bytes()
    store.save_uploaded_map("orig", yaml_for("foo.png"), {"foo.png": img})
    assert store.identical_existing(yaml_for("other_name.png"), {"other_name.png": img}) == "orig"
    assert store.identical_existing(yaml_for("o.png"), {"o.png": png_bytes(3)}) is None


def _map(seed=0):
    return parse_map(yaml_for("m.png"), {"m.png": png_bytes(seed)})


def _bs():
    return BuildSettings(wall_height=0.6, shapes={1: ShapeSetting(True, 0.5)},
                         erased=frozenset({3, 1}))


def _boxes():
    return [Box("wall", None, 0, 1, 0, 1, 0, 0.6), Box("shape", 1, 1, 2, 1, 2, 0, 0.5)]


def hashes(d):
    return {str(p.relative_to(d)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in d.rglob("*") if p.is_file()}


def test_arena_versions_accumulate(store):
    v1 = store.save_arena("a1", _bs(), _boxes(), _map())
    h1 = hashes(v1.version_dir)
    v2 = store.save_arena("a1", _bs(), _boxes(), _map())
    assert v1.version_dir != v2.version_dir
    assert hashes(v1.version_dir) == h1
    assert set(h1) == {"arena.usd", "map.yaml", "map.png", "boxes.json", "manifest.json"}
    cur = json.loads((store.settings.arena_root / "a1" / "current.json").read_text())
    assert cur["schema_version"] == 1
    assert (store.settings.repo_root / cur["manifest"]).parent == v2.version_dir
    man, boxes, vdir = store.load_current("a1")
    assert vdir == v2.version_dir and boxes == _boxes()
    assert man["built_by"] == "map_check" and man["boxes"] == 2
    assert man["settings"] == {"wall_height": 0.6, "shapes": {"1": {"fill": True, "height": 0.5}},
                               "erased": [1, 3], "outline_rule": "shape-height-v1"}
    assert (v2.version_dir / "map.yaml").read_text().count("image: map.png") == 1
    assert not [p for p in (store.settings.arena_root / "a1").iterdir()
                if p.name.startswith(".tmp-")]
    assert store.arenas() == ["a1"]


def test_resolves_through_run_sim(store):
    v = store.save_arena("viaRunSim", _bs(), _boxes(), _map())
    got = rs.resolve_arena_artifact(store.settings.repo_root, "viaRunSim")
    assert got == v.version_dir
    assert (got / "arena.usd").is_file() and (got / "map.yaml").is_file()


def test_arenas_only_map_check(store):
    d = store.settings.arena_root / "rcw2026"
    (d / "v1").mkdir(parents=True)
    (d / "v1" / "manifest.json").write_text(json.dumps({"kind": "arena", "id": "rcw2026"}))
    (d / "current.json").write_text(json.dumps({"manifest": "artifacts/arena/rcw2026/v1/manifest.json"}))
    store.save_arena("mine", _bs(), _boxes(), _map())
    assert store.arenas() == ["mine"]


def test_arena_root_outside_repo_writes_absolute_pointer(tmp_path):
    """MAP_CHECK_ARENA_ROOT outside the repo: the save works and leaves no orphan version."""
    s = replace(Settings(), maps_dir=tmp_path / "maps", repo_root=tmp_path / "repo",
                arena_root=tmp_path / "elsewhere" / "arena")
    store = ArenaStore(s)
    v = store.save_arena("outside", _bs(), _boxes(), _map())
    cur = json.loads((s.arena_root / "outside" / "current.json").read_text())
    assert Path(cur["manifest"]).is_absolute()
    assert Path(cur["manifest"]) == (v.version_dir / "manifest.json").resolve()
    assert [p.name for p in (s.arena_root / "outside").iterdir() if p.is_dir()] == [v.version_dir.name]
    assert store.load_current("outside")[2] == v.version_dir.resolve()
    # run_sim reads only <root>/artifacts/arena; linked there, it accepts the absolute manifest
    root = tmp_path / "simroot"
    (root / "artifacts").mkdir(parents=True)
    (root / "artifacts" / "arena").symlink_to(s.arena_root)
    assert rs.resolve_arena_artifact(root, "outside") == v.version_dir.resolve()


def test_arena_root_inside_repo_keeps_relative_pointer(store):
    store.save_arena("rel", _bs(), _boxes(), _map())
    cur = json.loads((store.settings.arena_root / "rel" / "current.json").read_text())
    assert cur["manifest"].startswith("artifacts/arena/rel/")


def test_upload_image_suffix_restricted(store):
    for bad in ("m.jpg", "m", "m.PGM.bak"):
        with pytest.raises(ValueError, match=r"\.pgm or \.png"):
            store.save_uploaded_map("bar", yaml_for(bad), {bad: png_bytes()})
    assert snapshot(store.settings.maps_dir) == []
    store.save_uploaded_map("bar", yaml_for("M.PNG"), {"M.PNG": png_bytes()})
    assert snapshot(store.settings.maps_dir) == ["bar.png", "bar.yaml"]


def test_upload_existing_image_file_is_name_taken(store):
    s = store.settings
    (s.maps_dir / "bar.png").write_bytes(b"someone else's image")
    with pytest.raises(NameTaken, match="bar.png"):
        store.save_uploaded_map("bar", yaml_for("foo.png"), {"foo.png": png_bytes()})
    assert snapshot(s.maps_dir) == ["bar.png"]
    assert (s.maps_dir / "bar.png").read_bytes() == b"someone else's image"


@pytest.mark.parametrize("yaml_text", [
    '"image": foo.png\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n',
    '{image: foo.png, resolution: 0.05, origin: [0.0, 0.0, 0.0]}\n',
])
def test_upload_image_rewrite_is_verified(store, yaml_text):
    with pytest.raises(ValueError, match="could not point the map yaml's image at 'bar.png'"):
        store.save_uploaded_map("bar", yaml_text.encode(), {"foo.png": png_bytes()})
    assert snapshot(store.settings.maps_dir) == []


def _foreign_arena(store, name, manifest):
    d = store.settings.arena_root / name
    (d / "v1").mkdir(parents=True)
    (d / "v1" / "manifest.json").write_text(json.dumps(manifest))
    (d / "current.json").write_text(json.dumps({"manifest": f"artifacts/arena/{name}/v1/manifest.json"}))
    return d


def test_save_refuses_arena_not_built_by_map_check(store):
    d = _foreign_arena(store, "rcw2026", {"kind": "arena", "id": "rcw2026"})
    before = (snapshot(d), (d / "current.json").read_text())
    with pytest.raises(NameTaken, match="not built by Map Check"):
        store.save_arena("rcw2026", _bs(), _boxes(), _map())
    assert (snapshot(d), (d / "current.json").read_text()) == before


def test_save_refuses_dir_without_pointer_but_reuses_empty_one(store):
    d = store.settings.arena_root / "loose"
    (d / "something").mkdir(parents=True)
    with pytest.raises(NameTaken, match="without a current.json"):
        store.save_arena("loose", _bs(), _boxes(), _map())
    (store.settings.arena_root / "empty").mkdir()
    store.save_arena("empty", _bs(), _boxes(), _map())
    store.save_arena("empty", _bs(), _boxes(), _map())  # its own arena: a new version
    assert store.arenas() == ["empty"]


def test_manifest_source_is_saved_name(store):
    img = png_bytes()
    store.save_uploaded_map("aaa_orig", yaml_for("f.png"), {"f.png": img})
    m = parse_map(yaml_for("f.png"), {"f.png": img})
    man = store.save_arena("zzz_copy", _bs(), _boxes(), m).manifest
    assert man["source_map"]["name"] == "zzz_copy"
    assert man["source_map"]["identical_to"] == "aaa_orig"
    own = store.save_arena("aaa_orig", _bs(), _boxes(), m).manifest
    assert own["source_map"]["name"] == "aaa_orig" and "identical_to" not in own["source_map"]
