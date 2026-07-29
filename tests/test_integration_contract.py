from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_core.profile import BEHAVIOR_PROFILES, SimulationProfile


class IntegrationContractTest(unittest.TestCase):
    def test_behavior_profiles_force_cpu_physics(self) -> None:
        for name in BEHAVIOR_PROFILES:
            profile = SimulationProfile.load(ROOT, name)
            self.assertEqual(profile.physics_device, "cpu")

    def test_standard_control_replaces_custom_services(self) -> None:
        contract = (ROOT / "contracts/simulation.yaml").read_text(encoding="utf-8")
        self.assertIn("provider: isaacsim.ros2.sim_control", contract)
        for endpoint in (
            "/get_simulation_state",
            "/reset_simulation",
            "/step_simulation",
            "/simulate_steps",
            "/load_world",
            "/spawn_entities",
        ):
            self.assertIn(endpoint, contract)
        cmake = (
            ROOT / "ros2_ws/src/tinker_sim_interfaces/CMakeLists.txt"
        ).read_text(encoding="utf-8")
        self.assertNotIn(".srv", cmake)
        self.assertFalse(
            (ROOT / "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge/scenario_gateway.py").exists()
        )

    def test_pinned_ros_dependencies(self) -> None:
        deployment = json.loads((ROOT / "deployment.json").read_text(encoding="utf-8"))
        self.assertEqual(
            deployment["dependencies"]["isaacsim_ros_workspaces"]["commit"],
            "dd3eeede7912755996a18f4884285d9f50843f79",
        )
        manifest = json.loads(
            (ROOT / "artifacts/provenance/ros-debs.json").read_text(encoding="utf-8")
        )
        versions = {item["name"]: item["version"] for item in manifest["packages"]}
        self.assertEqual(
            versions["ros-humble-simulation-interfaces"],
            "1.4.0-1jammy.20260605.131229",
        )
        self.assertEqual(
            versions["ros-humble-topic-based-ros2-control"],
            "0.2.0-1jammy.20260605.160608",
        )

    def test_one_isaac_joint_command_publisher_in_source(self) -> None:
        sources = list((ROOT / "ros2_ws/src").rglob("*.py"))
        publishers = []
        for path in sources:
            text = path.read_text(encoding="utf-8")
            if 'create_publisher(\n            JointState, "/isaac_joint_commands"' in text:
                publishers.append(path.name)
        self.assertEqual(publishers, ["command_gateway.py"])

    def test_local_dds_uses_shared_memory_default(self) -> None:
        deployment = json.loads((ROOT / "deployment.json").read_text(encoding="utf-8"))
        self.assertIsNone(deployment["ros"]["dds_profiles"]["local"])
        self.assertEqual(
            deployment["ros"]["dds_profiles"]["lan"], "config/fastdds-lan.xml"
        )

    def test_ros_vendor_exports_debian_python_install_path(self) -> None:
        vendor = (
            ROOT / "tools/tinker_sim_deploy/ros_vendor.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            "/local/lib/python3.10/dist-packages",
            vendor,
        )


if __name__ == "__main__":
    unittest.main()
