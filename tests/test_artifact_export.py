from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))


class ExportedArtifactTest(unittest.TestCase):
    def test_map_yaml_references_portable_image_name(self) -> None:
        current = __import__("json").loads((ROOT / "artifacts/robot/tinker2/current.json").read_text())
        manifest = Path(current["manifest"])
        self.assertFalse(manifest.is_absolute())
        artifact = (ROOT / manifest).parent
        self.assertIn("image: map.pgm", (artifact / "map.yaml").read_text())
        self.assertTrue((artifact / "map.pgm").is_file())


if __name__ == "__main__":
    unittest.main()
