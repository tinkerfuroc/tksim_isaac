"""Final review Finding 3: ``import_common._verify_pin`` must fail closed on
a dirty checkout, not just a HEAD mismatch -- an uncommitted local edit
(tampered or merely dirty) is invisible to a bare ``git rev-parse HEAD``
check but would silently feed different bytes into an importer than the
source lock claims to record.
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from tinker_sim_deploy import arena_artifact, import_common  # noqa: E402


def _git(cwd: Path, *args: str) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        ["git", "-C", str(cwd), *args],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise AssertionError(f"git {' '.join(args)} failed: {result.stderr}")
    return result


def _git_head(cwd: Path) -> str:
    return _git(cwd, "rev-parse", "HEAD").stdout.strip()


class VerifyPinDirtyCheckoutTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.checkout = Path(self._tmp.name)
        (self.checkout / "tracked.txt").write_text("original\n")
        _git(self.checkout, "init", "-q")
        _git(self.checkout, "config", "user.email", "fixture@example.invalid")
        _git(self.checkout, "config", "user.name", "Fixture")
        _git(self.checkout, "add", "-A")
        _git(self.checkout, "commit", "-q", "-m", "fixture commit")
        self.commit = _git_head(self.checkout)

    def test_clean_checkout_at_pinned_commit_passes(self):
        import_common._verify_pin(self.checkout, self.commit)

    def test_head_mismatch_fails_closed(self):
        with self.assertRaises(arena_artifact.AssetArtifactError):
            import_common._verify_pin(self.checkout, "f" * 40)

    def test_dirty_checkout_at_pinned_commit_fails_closed(self):
        # HEAD still equals the pin, but the working tree has a local edit
        # a bare rev-parse HEAD check would never see.
        (self.checkout / "tracked.txt").write_text("tampered\n")
        with self.assertRaises(arena_artifact.AssetArtifactError):
            import_common._verify_pin(self.checkout, self.commit)

    def test_untracked_file_at_pinned_commit_fails_closed(self):
        (self.checkout / "untracked.txt").write_text("surprise\n")
        with self.assertRaises(arena_artifact.AssetArtifactError):
            import_common._verify_pin(self.checkout, self.commit)


if __name__ == "__main__":
    unittest.main()
