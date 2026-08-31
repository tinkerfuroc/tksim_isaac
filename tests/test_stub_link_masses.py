"""The stub-link mass correction must target exactly the URDF's frame links.

The URDF -> USD importer leaves ``physics:mass`` unauthored on links that
declare no ``<inertial>``, and PhysX then assigns each one its 1.0 kg default.
The tinker2 description uses 22 such links as pure attachment frames, eleven
of them on the wrist -- ~11 kg of phantom mass at the elbow's full moment arm.
Measured 2026-08-31: that load alone saturates joint4's 50 Nm effort cap at
the orchestrator's tuck posture, so the elbow stalls short of its target and
every tuck trajectory aborts on the 0.01 rad goal tolerance.

``massless_stub_links`` is the pure selector for the correction: it must pick
links with neither ``<inertial>`` nor ``<collision>`` (frames), never links
that merely author unusual inertials, never colliding bodies, and never
``world``.  Selecting too much would zero out real dynamics; selecting too
little leaves phantom kilograms on the arm.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.backend import STUB_LINK_MASS_KG, massless_stub_links  # noqa: E402


def _urdf(links: str) -> bytes:
    return f'<robot name="t">{links}</robot>'.encode()


class MasslessStubLinkSelection(unittest.TestCase):
    def test_selects_only_frame_links(self) -> None:
        urdf = _urdf(
            """
            <link name="world"/>
            <link name="link5">
              <inertial><mass value="1.32"/></inertial>
              <collision><geometry><box size="1 1 1"/></geometry></collision>
            </link>
            <link name="link_tcp"/>
            <link name="xarm_camera_color_optical_frame"/>
            <link name="bumper">
              <collision><geometry><box size="1 1 1"/></geometry></collision>
            </link>
            <link name="counterweight">
              <inertial><mass value="9.0"/></inertial>
            </link>
            """
        )
        self.assertEqual(
            massless_stub_links(urdf),
            ("link_tcp", "xarm_camera_color_optical_frame"),
        )

    def test_world_is_never_selected(self) -> None:
        self.assertEqual(massless_stub_links(_urdf('<link name="world"/>')), ())

    def test_declaration_order_is_preserved(self) -> None:
        urdf = _urdf('<link name="b_frame"/><link name="a_frame"/>')
        self.assertEqual(massless_stub_links(urdf), ("b_frame", "a_frame"))

    def test_malformed_urdf_fails_closed(self) -> None:
        with self.assertRaises(Exception):
            massless_stub_links(b"<robot><link")

    def test_correction_mass_is_negligible(self) -> None:
        # 22 frame links on tinker2: the whole correction must stay below
        # any real link's mass (lightest authored link is 0.072 kg).
        self.assertLess(22 * STUB_LINK_MASS_KG, 0.072)
        self.assertGreater(STUB_LINK_MASS_KG, 0.0)


class TinkerArtifactContract(unittest.TestCase):
    """Pin the selector against the real artifact description when present."""

    URDF = (
        ROOT
        / "artifacts/robot/tinker2/347aef747d5f0d39dac1f5c9a5229b2aaa1b45bd86c729efed2f641b693a7417/robot.urdf"
    )

    def test_wrist_and_head_frames_are_selected(self) -> None:
        if not self.URDF.is_file():
            self.skipTest("content-addressed robot artifact not present")
        stubs = set(massless_stub_links(self.URDF.read_bytes()))
        self.assertIn("link_tcp", stubs)
        self.assertIn("link_eef", stubs)
        self.assertIn("xarm_camera_color_optical_frame", stubs)
        # Real bodies must never be selected.
        for name in ("link4", "link5", "front_left_wheel", "ballast"):
            self.assertNotIn(name, stubs)


if __name__ == "__main__":
    unittest.main()
