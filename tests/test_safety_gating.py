from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.safety_gating import effective_stop


class EffectiveStopTest(unittest.TestCase):
    def test_managed_mode_preserves_all_latches(self):
        self.assertTrue(effective_stop(False, False, False, False, True))
        self.assertTrue(effective_stop(False, True, True, False, True))
        self.assertTrue(effective_stop(False, True, False, True, True))
        self.assertFalse(effective_stop(False, True, False, False, True))
        self.assertTrue(effective_stop(True, True, False, False, True))

    def test_unmanaged_mode_reduces_to_desired(self):
        self.assertFalse(effective_stop(False, False, True, True, False))
        self.assertTrue(effective_stop(True, False, False, False, False))


if __name__ == "__main__":
    unittest.main()
