from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.config import Config
from tinker_sim_deploy.provenance import verify


class ProvenanceTest(unittest.TestCase):
    def test_checked_in_release_inputs_match_manifest(self) -> None:
        manifest = verify(Config.load(ROOT), require_python=True)
        self.assertEqual(manifest["environment"]["resolved_packages"], 219)


if __name__ == "__main__":
    unittest.main()
