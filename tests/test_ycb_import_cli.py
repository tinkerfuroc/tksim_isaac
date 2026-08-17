"""Task 10: YCB object importer CLI orchestration -- ``run_import`` with stub
Kit hooks.

Covers only the pure-Python orchestration in ``tools/ycb_import.py``: pin
verification, resolving each allowlisted object's DAE/STL by convention,
publishing through ``arena_artifact``, and assembling ``ATTRIBUTION.md``. The
real ``arena_convert.convert_object_to_usd`` implementation is exercised only
by the live import (see the Task 10 report), never here -- mirrors Task 9's
``tests/test_arena_import_cli.py`` split between pure-orchestration tests and
live-only Kit/pxr code.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import ycb_import  # noqa: E402
from tinker_sim_deploy import arena_artifact  # noqa: E402

_MODEL_CONFIG = b"""<?xml version="1.0"?>
<model>
  <name>ycb_003_cracker_box</name>
  <version>1.0</version>
  <sdf version="1.8">model-1_4.sdf</sdf>
  <description>
    This model has been converted from Yale-CMU-Berkeley(YCB) Object and Model set. Distributed under Creative Commons Attribution 4.0 International (CC BY 4.0) license.
  </description>
</model>
"""

# The real upstream license filename is LICENSE.txt (confirmed live against
# the pinned https://github.com/TeamSOBITS/tmc_wrs_gz checkout -- "The Clear
# BSD License", (c) Toyota Motor Corporation), not the bare "LICENSE" Task
# 9's sobits_gazebo_worlds repo happens to use, so the fixture mirrors the
# real filename rather than Task 9's.
_LICENSE = (
    b"The Clear BSD License\n\n"
    b"Copyright (c) 2020 TOYOTA MOTOR CORPORATION\nAll rights reserved.\n"
)


def _git(cwd: Path, *args: str) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        ["git", "-C", str(cwd), *args],
        capture_output=True,
        text=True,
        env={**os.environ, "LC_ALL": "C"},
        check=False,
    )
    if result.returncode != 0:
        raise AssertionError(f"git {' '.join(args)} failed: {result.stderr}")
    return result


def _git_head(cwd: Path) -> str:
    return _git(cwd, "rev-parse", "HEAD").stdout.strip()


def _build_checkout(root: Path, object_ids: tuple[str, ...] = ("ycb_003_cracker_box",)) -> Path:
    checkout = root / "checkout"
    checkout.mkdir(parents=True)
    for object_id in object_ids:
        model_dir = checkout / "tmc_wrs_gz_worlds" / "models" / object_id
        (model_dir / "meshes").mkdir(parents=True)
        (model_dir / "model.config").write_bytes(_MODEL_CONFIG)
        (model_dir / "meshes" / "textured.dae").write_bytes(f"fake-dae-bytes:{object_id}".encode())
        (model_dir / "meshes" / "nontextured.stl").write_bytes(f"fake-stl-bytes:{object_id}".encode())
    (checkout / "LICENSE.txt").write_bytes(_LICENSE)
    _git(checkout, "init", "-q")
    _git(checkout, "config", "user.email", "fixture@example.invalid")
    _git(checkout, "config", "user.name", "Fixture")
    _git(checkout, "add", "-A")
    _git(checkout, "commit", "-q", "-m", "fixture ycb checkout")
    return checkout


def _base_config(commit: str, object_allowlist: tuple[str, ...] = ("ycb_003_cracker_box",)) -> dict[str, object]:
    return {
        "repository": "https://example.invalid/tmc_wrs_gz",
        "branch": "jazzy-devel",
        "commit": commit,
        "models_root": "tmc_wrs_gz_worlds/models",
        "object_allowlist": list(object_allowlist),
    }


class StubHooks:
    """Mirrors Task 9's ``StubHooks`` shape: never imports Kit/pxr, so the
    orchestration in ``ycb_import.run_import`` is exercised without a GPU.
    """

    def __init__(self, write_texture: bool = False):
        self._write_texture = write_texture
        self.convert_calls: list[dict[str, object]] = []

    def convert_object_to_usd(self, dae_path: Path, stl_path: Path, usd_path: Path) -> None:
        self.convert_calls.append({"dae": dae_path, "stl": stl_path, "usd": usd_path})
        usd_path.parent.mkdir(parents=True, exist_ok=True)
        usd_path.write_bytes(b"stub-usd:" + dae_path.name.encode() + b":" + stl_path.name.encode())
        if self._write_texture:
            # Mirrors arena_convert._relocate_local_assets's real layout:
            # usd_path.parent / "textures" / <object_id>.
            object_id = usd_path.parent.name
            texture_dir = usd_path.parent / "textures" / object_id
            texture_dir.mkdir(parents=True, exist_ok=True)
            (texture_dir / "tex0.png").write_bytes(b"fake-png-bytes")


class RunImportTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = Path(self._tmp.name)
        self.checkout = _build_checkout(self.root)
        self.repo_root = self.root / "repo"
        self.commit = _git_head(self.checkout)

    def test_import_publishes_artifact(self):
        config = _base_config(self.commit)
        hooks = StubHooks()

        publication = ycb_import.run_import(config, self.repo_root, self.checkout, hooks)

        self.assertTrue(publication.created)
        artifact_dir = publication.artifact_dir
        self.assertTrue((artifact_dir / "ycb_003_cracker_box" / "object.usd").is_file())
        self.assertTrue((artifact_dir / "source-lock.json").is_file())
        self.assertTrue((artifact_dir / "manifest.json").is_file())
        self.assertTrue((artifact_dir / "ATTRIBUTION.md").is_file())
        self.assertEqual(arena_artifact.verify_asset_artifact(artifact_dir), [])

        self.assertEqual(len(hooks.convert_calls), 1)
        call = hooks.convert_calls[0]
        self.assertEqual(call["dae"].name, "textured.dae")
        self.assertEqual(call["stl"].name, "nontextured.stl")

        lock = json.loads((artifact_dir / "source-lock.json").read_bytes())
        self.assertEqual(lock["repository"], config["repository"])
        self.assertEqual(lock["branch"], config["branch"])
        self.assertEqual(lock["commit"], self.commit)
        recorded_paths = {record["path"] for record in lock["records"]}
        self.assertIn("tmc_wrs_gz_worlds/models/ycb_003_cracker_box/meshes/textured.dae", recorded_paths)
        self.assertIn("tmc_wrs_gz_worlds/models/ycb_003_cracker_box/meshes/nontextured.stl", recorded_paths)
        self.assertIn("LICENSE.txt", recorded_paths)

    def test_multiple_objects_each_get_their_own_usd(self):
        object_ids = ("ycb_003_cracker_box", "ycb_002_sugar_box")
        checkout = _build_checkout(self.root / "multi", object_ids=object_ids)
        commit = _git_head(checkout)
        config = _base_config(commit, object_allowlist=object_ids)
        hooks = StubHooks()

        publication = ycb_import.run_import(config, self.repo_root, checkout, hooks)

        self.assertEqual(len(hooks.convert_calls), 2)
        for object_id in object_ids:
            self.assertTrue((publication.artifact_dir / object_id / "object.usd").is_file())
        self.assertEqual(arena_artifact.verify_asset_artifact(publication.artifact_dir), [])

    def test_pin_mismatch_fails_closed(self):
        config = _base_config("f" * 40)
        hooks = StubHooks()

        with self.assertRaises(arena_artifact.AssetArtifactError):
            ycb_import.run_import(config, self.repo_root, self.checkout, hooks)

        self.assertFalse((self.repo_root / "artifacts" / "objects").exists())

    def test_missing_object_directory_fails_closed(self):
        config = _base_config(self.commit, object_allowlist=("ycb_does_not_exist",))
        hooks = StubHooks()

        with self.assertRaises(arena_artifact.AssetArtifactError):
            ycb_import.run_import(config, self.repo_root, self.checkout, hooks)

    def test_attribution_contains_required_sections(self):
        config = _base_config(self.commit)
        hooks = StubHooks()

        publication = ycb_import.run_import(config, self.repo_root, self.checkout, hooks)

        attribution = (publication.artifact_dir / "ATTRIBUTION.md").read_text(encoding="utf-8")
        self.assertIn("YCB Object and Model Set — CC BY 4.0", attribution)
        self.assertIn("tmc_wrs_gz — Clear BSD", attribution)
        self.assertIn("CC BY 4.0", attribution)
        # The Clear BSD license bytes must be reproduced verbatim.
        self.assertIn("The Clear BSD License", attribution)
        self.assertIn("TOYOTA MOTOR CORPORATION", attribution)

    def test_relocated_textures_are_published_alongside_their_usd(self):
        # arena_convert.convert_object_to_usd relocates any texture/material
        # files Kit extracted to usd_path.parent/"textures"/<object_id>/ (see
        # arena_convert._relocate_local_assets) -- run_import must republish
        # those alongside the referencing USD or the relative path it
        # rewrote them to resolves to nothing in the artifact.
        config = _base_config(self.commit)
        hooks = StubHooks(write_texture=True)

        publication = ycb_import.run_import(config, self.repo_root, self.checkout, hooks)

        texture_path = (
            publication.artifact_dir
            / "ycb_003_cracker_box"
            / "textures"
            / "ycb_003_cracker_box"
            / "tex0.png"
        )
        self.assertTrue(texture_path.is_file())
        self.assertEqual(texture_path.read_bytes(), b"fake-png-bytes")
        self.assertEqual(arena_artifact.verify_asset_artifact(publication.artifact_dir), [])

    def test_second_run_is_a_no_op_when_content_is_unchanged(self):
        config = _base_config(self.commit)

        first = ycb_import.run_import(config, self.repo_root, self.checkout, StubHooks())
        second = ycb_import.run_import(config, self.repo_root, self.checkout, StubHooks())

        self.assertTrue(first.created)
        self.assertFalse(second.created)
        self.assertEqual(first.identity, second.identity)


if __name__ == "__main__":
    unittest.main()
