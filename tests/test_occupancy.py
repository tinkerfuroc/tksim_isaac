from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.occupancy import OccupancyMap


class OccupancyMapTest(unittest.TestCase):
    def test_parses_flips_and_merges_pgm(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "map.pgm"
            # PGM top row has two occupied cells; bottom row has one.
            path.write_bytes(b"P5\n3 2\n255\n" + bytes([0, 0, 255, 0, 255, 255]))
            grid = OccupancyMap.from_pgm(path, resolution=1.0, origin_x=0.0, origin_y=0.0)
            self.assertEqual(grid.occupied[0], (True, False, False))
            self.assertEqual(grid.occupied[1], (True, True, False))
            self.assertEqual(len(grid.rectangles()), 2)
            self.assertLess(grid.raycast(2.5, 0.5, 3.1415926535), 2.0)


if __name__ == "__main__":
    unittest.main()
