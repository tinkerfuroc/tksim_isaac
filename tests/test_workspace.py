from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy.config import sha256_file
from tinker_sim_deploy.workspace import verify_workspace_lock


class WorkspaceLockTest(unittest.TestCase):
    def test_detects_source_drift_without_writing_workspace(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / "workspace"
            workspace.mkdir()
            source = workspace / "source"
            source.write_text("one", encoding="utf-8")
            lock = Path(temporary) / "lock.json"
            lock.write_text(json.dumps({"files": [{"path": "source", "size": 3, "sha256": sha256_file(source)}]}))
            with mock.patch("tinker_sim_deploy.workspace.SOURCE_GLOBS", ("**/*",)):
                self.assertEqual(verify_workspace_lock(workspace, lock), [])
                source.write_text("two", encoding="utf-8")
                self.assertEqual(verify_workspace_lock(workspace, lock), ["changed:source"])


if __name__ == "__main__":
    unittest.main()
