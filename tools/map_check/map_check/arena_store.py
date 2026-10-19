"""Saving uploaded maps and versioned arena artifacts (never overwriting)."""
import hashlib
import json
import os
import re
import shutil
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Mapping

from .arena_build import Box, BuildSettings, arena_usda
from .names import check_map_name
from .occupancy import MapFileError, OccupancyMap, load_map_dir, parse_map

_IMAGE_LINE = re.compile(rb"^image\s*:.*$", re.MULTILINE)


class NameTaken(ValueError):
    """A map or arena with this name already exists."""


@dataclass(frozen=True)
class ArenaVersion:
    name: str
    version_dir: Path
    manifest: dict


def _set_image(yaml_bytes: bytes, image: str) -> bytes:
    return _IMAGE_LINE.sub(b"image: " + image.encode(), yaml_bytes, count=1)


def _ext(image_name: str) -> str:
    return PurePosixPath(image_name).suffix or ".png"


def _write_new(path: Path, data: bytes) -> None:
    with open(path, "xb") as f:
        f.write(data)


class ArenaStore:
    def __init__(self, settings):
        self.settings = settings

    def map_names(self) -> list:
        names = []
        for p in sorted(self.settings.maps_dir.glob("*.yaml")):
            if p.name.endswith(".poses.yaml"):
                continue
            try:
                load_map_dir(p)
            except (MapFileError, OSError):
                continue
            names.append(p.name[:-len(".yaml")])
        return names

    def identical_existing(self, yaml_bytes: bytes, files: Mapping) -> "str | None":
        try:
            m = parse_map(yaml_bytes, files)
        except MapFileError:
            return None
        want = _set_image(yaml_bytes, "x")
        for name in self.map_names():
            try:
                other = load_map_dir(self.settings.maps_dir / f"{name}.yaml")
            except (MapFileError, OSError):
                continue
            if other.image_bytes == m.image_bytes and _set_image(other.yaml_bytes, "x") == want:
                return name
        return None

    def save_uploaded_map(self, name: str, yaml_bytes: bytes, files: Mapping) -> Path:
        check_map_name(name)
        m = parse_map(yaml_bytes, files)
        maps_dir, arena_root = self.settings.maps_dir, self.settings.arena_root
        yaml_path = maps_dir / f"{name}.yaml"
        if yaml_path.exists() or (arena_root / name).exists():
            raise NameTaken(f"the name '{name}' is already taken")
        image_path = maps_dir / f"{name}{_ext(m.image_name)}"
        _write_new(image_path, m.image_bytes)
        try:
            _write_new(yaml_path, _set_image(yaml_bytes, image_path.name))
        except BaseException:
            image_path.unlink(missing_ok=True)
            raise
        return yaml_path

    def save_arena(self, name: str, settings_used: BuildSettings, boxes: list,
                   m: OccupancyMap) -> ArenaVersion:
        check_map_name(name)
        sha = hashlib.sha256(m.yaml_bytes + m.image_bytes).hexdigest()
        now = datetime.now(timezone.utc)
        base = f"{now:%Y%m%dT%H%M%SZ}-{sha[:8]}"
        arena_dir = self.settings.arena_root / name
        arena_dir.mkdir(parents=True, exist_ok=True)
        source = self.identical_existing(m.yaml_bytes, {m.image_name: m.image_bytes}) or name
        image = "map" + _ext(m.image_name)
        manifest = {
            "kind": "arena", "built_by": "map_check", "id": name,
            "source_map": {"name": source, "sha256": sha},
            "settings": {
                "wall_height": settings_used.wall_height,
                "shapes": {str(k): {"fill": s.fill, "height": s.height}
                           for k, s in sorted(settings_used.shapes.items())},
                "erased": sorted(int(i) for i in settings_used.erased),
            },
            "boxes": len(boxes),
            "created_utc": now.strftime("%Y-%m-%dT%H:%M:%SZ"),
        }
        tmp = Path(tempfile.mkdtemp(prefix=".tmp-", dir=arena_dir))
        try:
            (tmp / "arena.usd").write_text(arena_usda(boxes))
            (tmp / "map.yaml").write_bytes(_set_image(m.yaml_bytes, image))
            (tmp / image).write_bytes(m.image_bytes)
            (tmp / "boxes.json").write_text(json.dumps([
                {"kind": b.kind, "shape_id": b.shape_id, "x0": b.x0, "x1": b.x1,
                 "y0": b.y0, "y1": b.y1, "z0": b.z0, "z1": b.z1} for b in boxes]))
            (tmp / "manifest.json").write_text(json.dumps(manifest, indent=2))
            n = 1
            while True:
                vdir = arena_dir / (base if n == 1 else f"{base}-{n}")
                try:
                    os.rename(tmp, vdir)  # fails if a non-empty dir exists
                except OSError:
                    if not vdir.exists():
                        raise
                    n += 1
                    continue
                break
        except BaseException:
            shutil.rmtree(tmp, ignore_errors=True)
            raise
        rel = (vdir / "manifest.json").relative_to(self.settings.repo_root)
        ptr_tmp = arena_dir / ".tmp-current.json"
        ptr_tmp.write_text(json.dumps({"manifest": str(rel), "schema_version": 1}))
        os.replace(ptr_tmp, arena_dir / "current.json")
        return ArenaVersion(name, vdir, manifest)

    def _current_manifest(self, name: str):
        ptr = json.loads((self.settings.arena_root / name / "current.json").read_text())
        path = Path(ptr["manifest"])
        if not path.is_absolute():
            path = self.settings.repo_root / path
        return json.loads(path.read_text()), path.parent

    def arenas(self) -> list:
        out = []
        root = self.settings.arena_root
        if not root.is_dir():
            return out
        for d in sorted(root.iterdir()):
            try:
                man, _ = self._current_manifest(d.name)
            except (OSError, ValueError, KeyError, TypeError):
                continue
            if isinstance(man, dict) and man.get("built_by") == "map_check":
                out.append(d.name)
        return out

    def load_current(self, name: str) -> tuple:
        man, vdir = self._current_manifest(name)
        raw = json.loads((vdir / "boxes.json").read_text())
        return man, [Box(**r) for r in raw], vdir
