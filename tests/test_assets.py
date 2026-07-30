from __future__ import annotations

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.assets import verify_assets
from tinker_sim_deploy.config import Config, sha256_file


class AssetManifestTest(unittest.TestCase):
    def test_portable_template_is_outside_ignored_artifact_output(self) -> None:
        self.assertTrue((ROOT / "config/asset-manifest.example.json").is_file())
        self.assertFalse((ROOT / "config/asset-manifest.example.json").is_symlink())

    def test_requires_complete_hash_verified_groups(self) -> None:
        base = Config.load(ROOT)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            raw = copy.deepcopy(base.raw)
            config = Config(root=root, raw=raw)
            artifacts = config.path("artifacts")
            artifacts.mkdir(parents=True)
            robot = artifacts / "tinker.usd"
            cache = config.path("isaac_cache") / "assets" / "room.usd"
            cache.parent.mkdir(parents=True)
            robot.write_text("robot", encoding="utf-8")
            cache.write_text("room", encoding="utf-8")
            manifest = {
                "schema_version": 1,
                "generated_robot_usds": [
                    {
                        "path": robot.relative_to(root).as_posix(),
                        "sha256": sha256_file(robot),
                    }
                ],
                "warmed_isaac_assets": [
                    {
                        "path": cache.relative_to(root).as_posix(),
                        "sha256": sha256_file(cache),
                    }
                ],
            }
            (artifacts / "asset-manifest.json").write_text(json.dumps(manifest))
            self.assertEqual(verify_assets(config), manifest)

    def test_rejects_empty_asset_group(self) -> None:
        base = Config.load(ROOT)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            config = Config(root=root, raw=copy.deepcopy(base.raw))
            config.path("artifacts").mkdir(parents=True)
            (config.path("artifacts") / "asset-manifest.json").write_text(
                json.dumps(
                    {
                        "generated_robot_usds": [],
                        "warmed_isaac_assets": [],
                    }
                )
            )
            with self.assertRaises(RuntimeError):
                verify_assets(config)


if __name__ == "__main__":
    unittest.main()
