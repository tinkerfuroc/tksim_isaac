# tests/test_design_two_arm_demo.py
"""designs/two_arm_demo is a committed, pretty-printed copy of tests/design_fixtures.py's
two_arm_urdf()/two_arm_design(); this guards that the committed file stays in sync with the
fixture that generated it, and that it still loads as a valid two-armed design.
"""
from __future__ import annotations

import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

from design_fixtures import two_arm_urdf
from tinker_designs.model import parse_urdf
from tinker_designs.schema import load_design

DESIGN = ROOT / "designs" / "two_arm_demo"


def _structural_bytes(root: ET.Element) -> bytes:
    """Canonicalize ignoring insignificant (indentation) whitespace.

    Unlike tinker_designs.clean.canonical_bytes (strip_text=False, by design: it must
    preserve a published URDF's exact formatting), this is strip_text=True and exists only
    for this test's own comparison of a pretty-printed file against an unindented one --
    ET.indent() adds whitespace-only text/tail nodes that canonical_bytes treats as
    significant, so a pretty-printed file can never canonical_bytes-equal a single-line one
    even when they parse to the same document.
    """
    xml = ET.tostring(root, encoding="unicode")
    canonical = ET.canonicalize(xml_data=xml, with_comments=False, strip_text=True)
    return (canonical.rstrip("\n") + "\n").encode("utf-8")


class TwoArmDemoDesignTest(unittest.TestCase):
    def test_committed_urdf_matches_the_fixture_structurally(self) -> None:
        committed = (DESIGN / "robot.urdf").read_bytes()
        self.assertEqual(
            _structural_bytes(parse_urdf(committed)),
            _structural_bytes(parse_urdf(two_arm_urdf())),
            "designs/two_arm_demo/robot.urdf has drifted from tests/design_fixtures.py's two_arm_urdf()",
        )

    def test_loads_as_a_design_with_two_arms(self) -> None:
        design = load_design(DESIGN)
        self.assertEqual(design.name, "two_arm_demo")
        self.assertEqual(len(design.arms), 2)
        self.assertEqual({arm.name for arm in design.arms}, {"left", "right"})

    def test_both_arms_are_flagged_estimated(self) -> None:
        """Both arms are sketched primitives (spec §3A), so estimated: true on each."""
        design = load_design(DESIGN)
        self.assertTrue(all(arm.estimated for arm in design.arms), [(arm.name, arm.estimated) for arm in design.arms])


if __name__ == "__main__":
    unittest.main()
