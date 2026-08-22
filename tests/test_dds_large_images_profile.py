"""A Fast DDS profile with shared memory sized for 720p camera streams.

The sim publishes a 2.76 MB colour frame and a 1.84 MB depth frame back to
back and the vision stack attaches about six subscribers to each. Against
Fast DDS's default 512 KB shared-memory segment the second topic published in
each iteration starves: measured at one consumer under full load, colour held
3.28 Hz while depth collapsed to 0.05 Hz, and the generalist failed with
``No orbbec camera data within sync threshold`` on 65% of GPSR run14's scans.

Pairing against subscriber count showed the mechanism plainly -- one
subscriber per topic in separate processes paired 90% of frames, two
subscribers in one process 25%, the full vision stack close to zero.

``local`` deliberately sets no profile at all (Fast DDS's own defaults, with
shared memory on) and ``lan`` swaps shared memory for UDP. Neither can carry
these streams, hence a third profile: shared memory kept, given room.
"""
from __future__ import annotations

import json
import sys
import unittest
import xml.etree.ElementTree as ElementTree
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

PROFILE_NAME = "large-images"
PROFILE_PATH = "config/fastdds-large-images.xml"
NS = {"d": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}

# Fast DDS's built-in shared-memory segment, which is what starved depth.
DEFAULT_SEGMENT_BYTES = 512 * 1024
# One 1280x720 rgb8 frame.
COLOUR_FRAME_BYTES = 1280 * 720 * 3


class DeploymentDeclarationTest(unittest.TestCase):
    def setUp(self):
        self.deployment = json.loads((ROOT / "deployment.json").read_text(encoding="utf-8"))
        self.profiles = self.deployment["ros"]["dds_profiles"]

    def test_profile_is_declared(self):
        self.assertIn(PROFILE_NAME, self.profiles)
        self.assertEqual(self.profiles[PROFILE_NAME], PROFILE_PATH)

    def test_profile_file_exists(self):
        self.assertTrue((ROOT / PROFILE_PATH).is_file())

    def test_existing_profiles_are_untouched(self):
        """local and lan keep their meanings; this is an addition."""
        self.assertIsNone(self.profiles["local"])
        self.assertEqual(self.profiles["lan"], "config/fastdds-lan.xml")


class ProfileContentTest(unittest.TestCase):
    def setUp(self):
        path = ROOT / PROFILE_PATH
        if not path.is_file():
            self.fail(f"missing {PROFILE_PATH}")
        self.root = ElementTree.parse(path).getroot()
        self.transports = {
            descriptor.findtext("d:transport_id", namespaces=NS): descriptor
            for descriptor in self.root.iterfind(
                "d:profiles/d:transport_descriptors/d:transport_descriptor", NS
            )
        }

    def test_shared_memory_stays_enabled(self):
        """Unlike the lan profile, this must not swap SHM for UDP."""
        kinds = {d.findtext("d:type", namespaces=NS) for d in self.transports.values()}
        self.assertIn("SHM", kinds)

    def test_udp_remains_available_for_discovery(self):
        kinds = {d.findtext("d:type", namespaces=NS) for d in self.transports.values()}
        self.assertIn("UDPv4", kinds)

    def _shm(self):
        for descriptor in self.transports.values():
            if descriptor.findtext("d:type", namespaces=NS) == "SHM":
                return descriptor
        self.fail("no SHM transport descriptor")

    def test_segment_has_room_for_many_frames(self):
        """The whole point: the segment must dwarf the default."""
        segment = int(self._shm().findtext("d:segment_size", namespaces=NS))
        self.assertGreater(segment, DEFAULT_SEGMENT_BYTES * 32)

    def test_a_whole_colour_frame_fits_in_one_message(self):
        largest = int(self._shm().findtext("d:maxMessageSize", namespaces=NS))
        self.assertGreater(largest, COLOUR_FRAME_BYTES)

    def test_participant_uses_the_declared_transports(self):
        participant = self.root.find("d:profiles/d:participant", NS)
        self.assertIsNotNone(participant)
        self.assertEqual(
            participant.findtext("d:rtps/d:useBuiltinTransports", namespaces=NS), "false"
        )
        used = {
            element.text
            for element in participant.iterfind("d:rtps/d:userTransports/d:transport_id", NS)
        }
        self.assertEqual(used, set(self.transports))

    def test_it_is_the_default_profile(self):
        participant = self.root.find("d:profiles/d:participant", NS)
        self.assertEqual(participant.get("is_default_profile"), "true")


class ConfigResolutionTest(unittest.TestCase):
    def test_config_resolves_the_profile_to_an_existing_file(self):
        from tinker_sim_deploy.config import Config

        config = Config.load(ROOT)
        resolved = config.dds_profile(PROFILE_NAME)
        self.assertIsNotNone(resolved)
        self.assertTrue(Path(resolved).is_file())
        self.assertEqual(Path(resolved), (ROOT / PROFILE_PATH).resolve())

    def test_local_still_means_no_override(self):
        from tinker_sim_deploy.config import Config

        self.assertIsNone(Config.load(ROOT).dds_profile("local"))


class CliChoiceTest(unittest.TestCase):
    def test_launch_accepts_the_profile(self):
        """A profile the launcher will not accept is unreachable."""
        source = (ROOT / "tools/tinker_sim_deploy/cli.py").read_text(encoding="utf-8")
        self.assertIn(f'"{PROFILE_NAME}"', source)


if __name__ == "__main__":
    unittest.main()
