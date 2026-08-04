from __future__ import annotations

import json
import os
import signal
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from validation.manipulation_qualification import (  # noqa: E402
    APPROVED_RECORD_TOPICS,
    CONTRACT_TOPIC,
    RAW_TRUTH_TOPIC,
    GATES,
    QualificationManifest,
    QualificationRunner,
    SAFETY_STOP_TOPIC,
    TRAJECTORY_CONTROLLER,
    _tool_version,
    validate_command,
)


class QualificationManifestTest(unittest.TestCase):
    def test_controller_records_parse_yaml_and_ros_python_repr(self) -> None:
        yaml_output = (
            "response:\n"
            "controller:\n"
            f"- name: {TRAJECTORY_CONTROLLER}\n  state: active\n"
        )
        repr_output = (
            "controller_manager_msgs.srv.ListControllers_Response("
            "controller=[ControllerState(name='xarm7_traj_controller', "
            "state='active', type='joint_trajectory_controller', "
            "claimed_interfaces=['position'])])\n"
        )
        self.assertEqual(
            QualificationRunner._controller_records(yaml_output),
            [{"name": TRAJECTORY_CONTROLLER, "state": "active"}],
        )
        self.assertEqual(
            QualificationRunner._controller_records(repr_output),
            [{"name": TRAJECTORY_CONTROLLER, "state": "active"}],
        )

    @staticmethod
    def _ros_output(
        command: list[str],
        *,
        safety: bool = False,
        contract: str = "pass",
        controller_state: str = "active",
        bag_topics: set[str] | None = None,
        object_truth: bool = False,
    ) -> SimpleNamespace:
        if command[-1:] == ["--no-daemon"]:
            command = command[:-1]
        if command[:3] == ["ros2", "topic", "list"]:
            topics = {
                "/clock", "/isaac_joint_states", RAW_TRUTH_TOPIC,
                SAFETY_STOP_TOPIC, CONTRACT_TOPIC,
            }
            if object_truth:
                topics.add("/sim/truth/object_state")
            stdout = "\n".join(sorted(topics)) + "\n"
        elif command[:3] == ["ros2", "topic", "info"]:
            topic = command[3]
            durability = "TRANSIENT_LOCAL" if topic in {SAFETY_STOP_TOPIC, CONTRACT_TOPIC} else "VOLATILE"
            stdout = (
                "Type: std_msgs/msg/Bool\n"
                "Subscription count: 1\n"
                "Node name: rosbag2_recorder\n"
                "Endpoint type: SUBSCRIPTION\n"
                "QoS profile:\n"
                "  Reliability: RELIABLE\n"
                f"  Durability: {durability}\n"
                "  History: KEEP_LAST\n"
                "  Depth: 10\n"
                if bag_topics is None or topic in bag_topics
                else "Type: std_msgs/msg/Bool\nSubscription count: 0\n"
            )
        elif command[:3] == ["ros2", "topic", "echo"] and command[-1] == SAFETY_STOP_TOPIC:
            stdout = f"data: {'true' if safety else 'false'}\n"
        elif command[:3] == ["ros2", "topic", "echo"] and command[-1] == CONTRACT_TOPIC:
            stdout = f"data: '{{\"state\": \"{contract}\"}}'\n"
        elif command[:3] == ["ros2", "service", "call"]:
            stdout = (
                "response:\n"
                "controller:\n"
                f"- name: xarm7_traj_controller\n  state: {controller_state}\n"
            )
        elif command[:3] == ["ros2", "topic", "echo"] and command[-1] == "/sim/truth/object_state":
            stdout = (
                "data: '{\"object_id\": \"cube\", \"class_name\": \"box\", "
                "\"pose\": {}}'\n" if object_truth else ""
            )
        else:
            stdout = ""
        return SimpleNamespace(returncode=0, stdout=stdout, stderr="")

    def _runner(self, root: Path) -> QualificationRunner:
        config = root / "simulation/qualification/manipulation-core.json"
        scenario = root / "simulation/scenarios/qualification-retention.json"
        config.parent.mkdir(parents=True)
        scenario.parent.mkdir(parents=True)
        config.write_text(
            json.dumps(
                {
                    "gates": ["retention"],
                    "rosbag_minimum_message_counts": self._minimum_policy(["retention"]),
                }
            ),
            encoding="utf-8",
        )
        scenario.write_text(json.dumps({"schema_version": 2, "id": "qualification-retention", "seed": 7, "actors": [], "objects": []}), encoding="utf-8")
        artifact = root / "robot.usd"
        artifact.write_text("deterministic usd placeholder", encoding="utf-8")
        return QualificationRunner(
            root=root,
            attempt_root=root / "attempts",
            config_path=config,
            scenario_path=scenario,
            artifact_path=artifact,
            gate="retention",
            isaac_command=["isaac-wrapper", "--profile", "manipulation-core"],
            humble_command=["humble-wrapper", "manipulation"],
            gate_commands=None,
        )

    @staticmethod
    def _minimum_policy(gates: list[str]) -> dict[str, dict[str, int]]:
        zero_allowed = {
            "free-space-fjt": {"/sim/truth/object_state", "/sim/truth/contacts"},
            "safety-stop": {"/sim/truth/object_state", "/sim/truth/contacts"},
            "free-gripper": {"/sim/truth/object_state", "/sim/truth/contacts"},
            "arm-collision": {"/sim/truth/object_state"},
        }
        return {
            gate: {
                topic: 0 if topic in zero_allowed.get(gate, set()) else 1
                for topic in APPROVED_RECORD_TOPICS
            }
            for gate in gates
        }

    @staticmethod
    def _bag_metadata() -> str:
        lines = ["rosbag2_bagfile_information:", "  topics_with_message_count:"]
        for topic in APPROVED_RECORD_TOPICS:
            lines.extend(
                [
                    "  - topic_metadata:",
                    f"      name: {topic}",
                    '      offered_qos_profiles: "configured"',
                    "    message_count: 1",
                ]
            )
        return "\n".join(lines) + "\n"

    def test_rosbag_minimum_policy_covers_all_gates_and_zero_allowed_topics(self) -> None:
        config = json.loads(
            (ROOT / "simulation/qualification/manipulation-core.json").read_text(
                encoding="utf-8"
            )
        )
        policy = config["rosbag_minimum_message_counts"]
        self.assertEqual(set(policy), set(config["gates"]))
        expected_zero = {
            "free-space-fjt": {"/sim/truth/object_state", "/sim/truth/contacts"},
            "safety-stop": {"/sim/truth/object_state", "/sim/truth/contacts"},
            "free-gripper": {"/sim/truth/object_state", "/sim/truth/contacts"},
            "obstructed-gripper": set(),
            "arm-collision": {"/sim/truth/object_state"},
            "retention": set(),
        }
        for gate in config["gates"]:
            self.assertEqual(set(policy[gate]), set(APPROVED_RECORD_TOPICS))
            self.assertEqual(
                {topic for topic, minimum in policy[gate].items() if minimum == 0},
                expected_zero[gate],
            )
            self.assertTrue(all(isinstance(value, int) and value >= 0 for value in policy[gate].values()))

    def test_rosbag_minimum_policy_rejects_unknown_or_missing_gates(self) -> None:
        base = {
            "gates": ["retention"],
            "rosbag_minimum_message_counts": self._minimum_policy(["retention"]),
        }
        self.assertEqual(
            QualificationRunner._rosbag_minimum_message_counts(base, gate="retention")[
                "/sim/truth/object_state"
            ],
            1,
        )
        with self.assertRaisesRegex(ValueError, "missing"):
            QualificationRunner._rosbag_minimum_message_counts({"gates": ["retention"]})
        with self.assertRaisesRegex(ValueError, "unknown qualification gates"):
            QualificationRunner._rosbag_minimum_message_counts(
                {"gates": ["not-a-gate"], "rosbag_minimum_message_counts": {}}
            )
        with self.assertRaisesRegex(ValueError, "unknown gates"):
            QualificationRunner._rosbag_minimum_message_counts(
                {
                    "gates": ["retention"],
                    "rosbag_minimum_message_counts": {
                        **self._minimum_policy(["retention"]),
                        "unknown": self._minimum_policy(["retention"])["retention"],
                    },
                }
            )

    def test_rosbag_metadata_policy_allows_only_gate_specific_zero_counts(self) -> None:
        metadata = self._bag_metadata()
        for gate in ("free-space-fjt", "safety-stop", "free-gripper", "arm-collision"):
            policy = self._minimum_policy([gate])[gate]
            zero_metadata = metadata
            for topic in ("/sim/truth/object_state", "/sim/truth/contacts"):
                if policy[topic] == 0:
                    zero_metadata = zero_metadata.replace(
                        f"name: {topic}\n      offered_qos_profiles: \"configured\"\n    message_count: 1",
                        f"name: {topic}\n      offered_qos_profiles: \"configured\"\n    message_count: 0",
                    )
            evidence = QualificationRunner._rosbag_metadata_evidence(
                zero_metadata, minimum_message_counts=policy
            )
            self.assertNotIn("/sim/truth/object_state", evidence["below_minimum_topics"])
            self.assertNotIn("/sim/truth/contacts", evidence["below_minimum_topics"])

    def test_rosbag_metadata_rejects_zero_required_count_and_preserves_qos_checks(self) -> None:
        policy = self._minimum_policy(["retention"])["retention"]
        metadata = self._bag_metadata().replace(
            'name: /sim/truth/object_state\n      offered_qos_profiles: "configured"\n    message_count: 1',
            'name: /sim/truth/object_state\n      offered_qos_profiles: "configured"\n    message_count: 0',
        ).replace(
            'name: /sim/truth/contacts\n      offered_qos_profiles: "configured"\n    message_count: 1',
            'name: /sim/truth/contacts\n    message_count: 1',
        )
        evidence = QualificationRunner._rosbag_metadata_evidence(
            metadata, minimum_message_counts=policy
        )
        self.assertIn("/sim/truth/object_state", evidence["below_minimum_topics"])
        self.assertIn("/sim/truth/contacts", evidence["missing_qos_metadata"])
        self.assertEqual(evidence["minimum_message_counts"]["/sim/truth/object_state"], 1)

    def test_rosbag_metadata_missing_document_is_fail_closed(self) -> None:
        policy = self._minimum_policy(["free-space-fjt"])["free-space-fjt"]
        evidence = QualificationRunner._rosbag_metadata_evidence(
            "", minimum_message_counts=policy
        )
        self.assertFalse(evidence["parsed"])
        self.assertEqual(evidence["below_minimum_topics"], [])

    def test_final_rosbag_validation_allows_safety_gate_zero_truth_topics(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            runner.gate = "safety-stop"
            runner.config_path.write_text(
                json.dumps(
                    {
                        "gates": ["safety-stop"],
                        "rosbag_minimum_message_counts": self._minimum_policy(["safety-stop"]),
                    }
                ),
                encoding="utf-8",
            )
            manifest = runner.prepare_manifest()
            bag_dir = manifest.attempt_dir / "rosbag"
            bag_dir.mkdir()
            metadata = self._bag_metadata()
            for topic in ("/sim/truth/object_state", "/sim/truth/contacts"):
                metadata = metadata.replace(
                    f"name: {topic}\n      offered_qos_profiles: \"configured\"\n    message_count: 1",
                    f"name: {topic}\n      offered_qos_profiles: \"configured\"\n    message_count: 0",
                )
            (bag_dir / "metadata.yaml").write_text(metadata, encoding="utf-8")
            (manifest.attempt_dir / "pre-gate-baseline.json").write_text(
                json.dumps({"status": "ready"}), encoding="utf-8"
            )
            self._write_rosbag_log(manifest.attempt_dir)
            self._write_startup_rosbag_readiness(runner, manifest)
            runner._command_runner = lambda command, **_kwargs: self._ros_output(list(command))
            live_ok, _live, live_failures = runner._rosbag_final_evidence(manifest, final=False)
            self.assertTrue(live_ok, live_failures)
            runner._termination["rosbag"] = {"returncode": 0, "forced": False}

            final_ok, evidence, failures = runner._rosbag_final_evidence(manifest)

            self.assertTrue(final_ok, failures)
            self.assertEqual(evidence["topic_message_counts"]["/sim/truth/object_state"], 0)
            self.assertEqual(evidence["topic_message_counts"]["/sim/truth/contacts"], 0)
            self.assertIn("/sim/truth/object_state", evidence["metadata_topics"]["zero_allowed_topics"])

    @staticmethod
    def _write_rosbag_log(attempt_dir: Path, topics: tuple[str, ...] = APPROVED_RECORD_TOPICS) -> None:
        lines = [f"[INFO] [rosbag2_recorder]: Subscribed to topic '{topic}'" for topic in topics]
        lines.append("[INFO] [rosbag2_recorder]: All requested topics are subscribed. Stopping discovery...")
        (attempt_dir / "rosbag.log").write_text("\n".join(lines) + "\n", encoding="utf-8")
        bag_dir = attempt_dir / "rosbag"
        bag_dir.mkdir(exist_ok=True)
        sqlite3.connect(bag_dir / "rosbag_0.db3").close()

    @classmethod
    def _write_startup_rosbag_readiness(cls, runner: QualificationRunner, manifest: Any) -> None:
        log_evidence = QualificationRunner._rosbag_log_evidence(manifest.attempt_dir / "rosbag.log")
        subscriptions = {}
        for topic in APPROVED_RECORD_TOPICS:
            probe = cls._ros_output(["ros2", "topic", "info", topic, "-v"])
            subscriptions[topic] = QualificationRunner._rosbag_live_topic_evidence(
                topic,
                {"returncode": probe.returncode, "stdout": probe.stdout, "stderr": probe.stderr},
                log_evidence,
            )
            subscriptions[topic]["startup_contract_validated"] = True
        (manifest.attempt_dir / "rosbag-readiness.json").write_text(
            json.dumps(
                {
                    "status": "ready",
                    "ready": True,
                    "rosbag_log": log_evidence,
                    "subscriptions": subscriptions,
                }
            ),
            encoding="utf-8",
        )

    def test_manifest_is_unique_and_records_inputs_before_run(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            first = runner.prepare_manifest()
            second = runner.prepare_manifest()
            self.assertNotEqual(first.attempt_id, second.attempt_id)
            self.assertTrue(first.path.is_file())
            data = json.loads(first.path.read_text(encoding="utf-8"))
            self.assertEqual(data["physics"], {"device": "cpu", "use_gpu": False, "cuda_device": -1})
            self.assertNotIn("sha256", json.dumps(data))
            self.assertIn("bridge", data["sources"])
            source_paths = {
                record["path"] for record in data["sources"]["executed_inputs"]["files"]
            }
            self.assertIn("validation/run_sim.py", source_paths)
            self.assertIn("scripts/tinker-sim", source_paths)
            self.assertIn("tools/deploy.py", source_paths)
            self.assertIn("uv.lock", source_paths)
            self.assertIn("pyproject.toml", source_paths)
            self.assertIn("validation/manipulation_qualification.py", source_paths)
            self.assertIn("versions", data["provenance"])
            self.assertEqual(data["selected_gates"], ["retention"])
            self.assertIn("gates", data["commands"])
            self.assertEqual(data["topics"]["recorded"], list(APPROVED_RECORD_TOPICS))
            self.assertNotIn(RAW_TRUTH_TOPIC, data["topics"]["recorded"])

    def test_manifest_only_creates_artifact_paths_without_starting_processes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            result = runner.run(manifest_only=True)
            self.assertEqual(result.status, "manifest-only")
            self.assertTrue((result.attempt_dir / "manifest.json").is_file())
            self.assertFalse((result.attempt_dir / "result.json").exists())

    def test_process_roles_scrub_isaac_but_preserve_humble_environment(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            captured: dict[str, dict[str, str]] = {}

            class LiveProcess:
                pid = 2_000_005
                returncode = 0

                def poll(self):
                    return self.returncode

            def popen(command, **kwargs):
                captured[command[0]] = dict(kwargs["env"])
                return LiveProcess()

            contaminated = {
                "HOME": "/home/tester",
                "USER": "tester",
                "LOGNAME": "tester",
                "PATH": "/usr/local/bin:/usr/bin",
                "ROS_DOMAIN_ID": "91",
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "TINKER_ACCEPT_OMNIVERSE_EULA": "Y",
                "TINKER_CUSTOM_SETTING": "preserve-me",
                "ISAACSIM_CACHE_PATH": "/cache/isaac",
                "PYTHONPATH": "/usr/lib/python3.10/site-packages",
                "AMENT_PREFIX_PATH": "/opt/ros/humble",
                "CMAKE_PREFIX_PATH": "/opt/ros/humble",
                "COLCON_PREFIX_PATH": "/opt/ros/humble",
                "LD_LIBRARY_PATH": "/opt/ros/humble/lib",
                "PYTHONHOME": "/usr",
                "ROS_PACKAGE_PATH": "/opt/ros/humble/share",
                "UNRELATED_PARENT_SETTING": "humble-needs-this",
            }
            with patch.dict(os.environ, contaminated, clear=True):
                manifest = runner.prepare_manifest()
                runner._popen = popen
                runner._start("isaac", runner.isaac_command, manifest)
                runner._start("humble", runner.humble_command, manifest)
                runner._stop("humble")
                runner._stop("isaac")

            isaac = captured["isaac-wrapper"]
            humble = captured["humble-wrapper"]
            for key in ("PYTHONPATH", "AMENT_PREFIX_PATH", "CMAKE_PREFIX_PATH",
                        "COLCON_PREFIX_PATH", "LD_LIBRARY_PATH", "PYTHONHOME",
                        "ROS_PACKAGE_PATH"):
                self.assertNotIn(key, isaac)
            self.assertEqual(isaac["HOME"], "/home/tester")
            self.assertEqual(isaac["PATH"], "/usr/local/bin:/usr/bin")
            self.assertEqual(isaac["ROS_DOMAIN_ID"], "91")
            self.assertEqual(isaac["RMW_IMPLEMENTATION"], "rmw_cyclonedds_cpp")
            self.assertEqual(isaac["TINKER_ACCEPT_OMNIVERSE_EULA"], "Y")
            self.assertEqual(isaac["ISAACSIM_CACHE_PATH"], "/cache/isaac")
            for key in ("TINKER_SIM_ATTEMPT_DIR", "TINKER_SIM_TRUTH_JSONL",
                        "TINKER_SIM_EVALUATOR_JSONL", "TINKER_SIM_ROSBAG_DIR"):
                self.assertEqual(isaac[key], humble[key])
            self.assertEqual(humble["ROS_DOMAIN_ID"], "91")
            self.assertEqual(humble["RMW_IMPLEMENTATION"], "rmw_cyclonedds_cpp")
            self.assertEqual(humble["PYTHONPATH"], "/usr/lib/python3.10/site-packages")
            self.assertEqual(humble["UNRELATED_PARENT_SETTING"], "humble-needs-this")

            data = json.loads(manifest.path.read_text(encoding="utf-8"))
            policy = data["environment"]["process_policy"]
            self.assertEqual(policy["isaac"]["mode"], "scrubbed-allowlist")
            self.assertIn("PYTHONPATH", policy["isaac"]["scrubbed_variables"])
            self.assertEqual(policy["humble"]["mode"], "inherit-parent")

    def test_ros_tooling_uses_no_daemon_attempt_environment_and_isaac_stays_scrubbed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            captured_commands: list[tuple[list[str], dict[str, str]]] = []
            captured_processes: list[tuple[list[str], dict[str, str]]] = []

            class LiveProcess:
                pid = 2_000_006
                returncode = 0

                def poll(self):
                    return self.returncode

            def command_runner(command, **kwargs):
                captured_commands.append((list(command), dict(kwargs["env"])))
                return self._ros_output(list(command), object_truth=True)

            def popen(command, **kwargs):
                captured_processes.append((list(command), dict(kwargs["env"])))
                return LiveProcess()

            contaminated = {
                "HOME": "/home/tester",
                "PATH": "/usr/local/bin:/usr/bin",
                "ROS_DOMAIN_ID": "130",
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                "AMENT_PREFIX_PATH": "/opt/ros/humble",
                "PYTHONPATH": "/opt/ros/humble/lib/python3.10/site-packages",
                "TINKER_ACCEPT_OMNIVERSE_EULA": "Y",
            }
            with patch.dict(os.environ, contaminated, clear=True):
                manifest = runner.prepare_manifest()
                runner._command_runner = command_runner
                runner._popen = popen
                runner._snapshot_graph(manifest, "test")
                runner._contract_readiness(manifest)
                runner._object_truth_readiness(manifest)
                runner._start("rosbag", ["ros2", "bag", "record"], manifest)
                runner._start("isaac", runner.isaac_command, manifest)

            self.assertTrue(captured_commands)
            for _command, environment in captured_commands:
                self.assertEqual(environment["ROS2CLI_NO_DAEMON"], "1")
                self.assertEqual(environment["ROS_DOMAIN_ID"], "130")
                self.assertEqual(environment["RMW_IMPLEMENTATION"], "rmw_cyclonedds_cpp")
                self.assertEqual(environment["AMENT_PREFIX_PATH"], "/opt/ros/humble")
            rosbag_environment = next(
                environment for command, environment in captured_processes if command[:3] == ["ros2", "bag", "record"]
            )
            self.assertEqual(rosbag_environment["ROS2CLI_NO_DAEMON"], "1")
            self.assertEqual(rosbag_environment["ROS_DOMAIN_ID"], "130")
            self.assertEqual(rosbag_environment["RMW_IMPLEMENTATION"], "rmw_cyclonedds_cpp")
            isaac_environment = next(
                environment for command, environment in captured_processes if command[0] == "isaac-wrapper"
            )
            self.assertNotIn("ROS2CLI_NO_DAEMON", isaac_environment)
            self.assertNotIn("AMENT_PREFIX_PATH", isaac_environment)
            data = json.loads(manifest.path.read_text(encoding="utf-8"))
            tooling_policy = data["environment"]["process_policy"]["ros-tooling"]
            self.assertEqual(tooling_policy["ROS2CLI_NO_DAEMON"], "1")
            self.assertTrue(tooling_policy["inherits_system_ros_paths"])
            self.assertIsNone(tooling_policy["FASTRTPS_DEFAULT_PROFILES_FILE"])

    def test_humble_and_wrapper_use_no_daemon_ros2_environment(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            manifest = runner.prepare_manifest()
            with patch.dict(os.environ, {"ROS2CLI_NO_DAEMON": "0"}, clear=False):
                humble = runner._env(manifest, "humble")
            self.assertEqual(humble["ROS2CLI_NO_DAEMON"], "1")
            policy = json.loads(manifest.path.read_text(encoding="utf-8"))["environment"]["process_policy"]
            self.assertEqual(policy["humble"]["ROS2CLI_NO_DAEMON"], "1")
            wrapper = (ROOT / "scripts/launch-humble").read_text(encoding="utf-8")
            self.assertIn("export ROS2CLI_NO_DAEMON=1", wrapper)

    def test_qualification_ros2_commands_use_global_no_daemon_flag(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            captured: list[list[str]] = []

            def command_runner(command, **_kwargs):
                captured.append(list(command))
                return self._ros_output(list(command), object_truth=True)

            manifest = runner.prepare_manifest()
            runner._command_runner = command_runner
            runner._snapshot_graph(manifest, "argv")
            runner._contract_readiness(manifest)
            runner._safety_readiness(manifest)
            runner._controller_readiness(manifest)
            runner._object_truth_readiness(manifest)
            self.assertTrue(captured)
            for command in captured:
                if command[1] in {"topic", "node", "param"}:
                    self.assertEqual(command[-1], "--no-daemon")
                else:
                    self.assertNotIn("--no-daemon", command)
            self.assertNotIn("--no-daemon", runner._default_rosbag_command(manifest))
            wrapper = (ROOT / "scripts/launch-humble").read_text(encoding="utf-8")
            manipulation_launch = (
                ROOT / "ros2_ws/src/tinker_sim_bridge/launch/manipulation.launch.py"
            ).read_text(encoding="utf-8")
            whole_robot_launch = (
                ROOT / "ros2_ws/src/tinker_sim_bridge/launch/whole_robot.launch.py"
            ).read_text(encoding="utf-8")
            self.assertIn("exec ros2 launch", wrapper)
            self.assertIn('"--ready-node",', manipulation_launch)
            self.assertIn('"--ready-parameter",', manipulation_launch)
            self.assertNotIn('"ros2",\n            "param",', manipulation_launch)
            self.assertIn('"true",\n            "--no-daemon",', whole_robot_launch)

    def test_ros_tooling_prepends_ordered_project_overlay_paths_without_duplicates(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            install = root / "ros2_ws/install"
            interfaces = install / "tinker_sim_interfaces"
            bridge = install / "tinker_sim_bridge"
            interface_marker = interfaces / "share/colcon-core/packages/tinker_sim_interfaces"
            bridge_marker = bridge / "share/colcon-core/packages/tinker_sim_bridge"
            interface_marker.parent.mkdir(parents=True)
            bridge_marker.parent.mkdir(parents=True)
            interface_marker.write_text("", encoding="utf-8")
            bridge_marker.write_text("tinker_sim_interfaces", encoding="utf-8")
            interface_python = interfaces / "local/lib/python3.10/dist-packages"
            bridge_python = bridge / "lib/python3.10/site-packages"
            interface_library = interfaces / "lib"
            bridge_library = bridge / "lib"
            interface_bin = interfaces / "bin"
            bridge_bin = bridge / "bin"
            for path in (
                interface_python,
                bridge_python,
                interface_library,
                bridge_library,
                interface_bin,
                bridge_bin,
            ):
                path.mkdir(parents=True, exist_ok=True)

            parent_ament = "/home/tinker/tk25_ws/install:/opt/ros/humble"
            parent_python = "/home/tinker/tk25_ws/lib/python3.10/site-packages"
            parent_library = "/home/tinker/tk25_ws/lib:/opt/ros/humble/lib"
            parent_path = "/home/tinker/tk25_ws/bin:/usr/bin"
            with patch.dict(
                os.environ,
                {
                    "AMENT_PREFIX_PATH": f"{parent_ament}:{bridge}",
                    "PYTHONPATH": f"{parent_python}:{bridge_python}",
                    "LD_LIBRARY_PATH": f"{parent_library}:{interface_library}",
                    "PATH": f"{parent_path}:{bridge_bin}",
                },
                clear=False,
            ):
                manifest = runner.prepare_manifest()
                environment = runner._env(manifest, "ros-tooling")

            self.assertEqual(
                environment["AMENT_PREFIX_PATH"],
                os.pathsep.join([str(interfaces), str(bridge), parent_ament]),
            )
            self.assertEqual(
                environment["PYTHONPATH"],
                os.pathsep.join([str(interface_python), str(bridge_python), parent_python]),
            )
            self.assertEqual(
                environment["LD_LIBRARY_PATH"],
                os.pathsep.join([str(interface_library), str(bridge_library), parent_library]),
            )
            self.assertEqual(
                environment["PATH"],
                os.pathsep.join([str(interface_bin), str(bridge_bin), parent_path]),
            )
            policy = json.loads(manifest.path.read_text(encoding="utf-8"))["environment"]["process_policy"]["ros-tooling"]
            overlay_policy = policy["project_overlay"]
            self.assertEqual(overlay_policy["prefixes"], [str(interfaces), str(bridge)])
            self.assertEqual(overlay_policy["path_variables"]["PYTHONPATH"], [str(interface_python), str(bridge_python)])

    def test_local_ros_tooling_removes_inherited_fastdds_whitelist(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            captured: dict[str, str] = {}

            def command_runner(_command, **kwargs):
                captured.update(kwargs["env"])
                return SimpleNamespace(returncode=0, stdout="", stderr="")

            with patch.dict(
                os.environ,
                {
                    "TINKER_SIM_DDS_PROFILE": "local",
                    "FASTRTPS_DEFAULT_PROFILES_FILE": "/parent/fastdds_whitelist.xml",
                },
                clear=False,
            ):
                manifest = runner.prepare_manifest()
                runner._command_runner = command_runner
                runner._capture("probe", ["ros2", "topic", "list"], manifest.attempt_dir, manifest)

            self.assertNotIn("FASTRTPS_DEFAULT_PROFILES_FILE", captured)
            policy = json.loads(manifest.path.read_text(encoding="utf-8"))["environment"]["process_policy"]["ros-tooling"]
            self.assertEqual(policy["TINKER_SIM_DDS_PROFILE"], "local")
            self.assertIsNone(policy["FASTRTPS_DEFAULT_PROFILES_FILE"])

    def test_lan_ros_tooling_replaces_inherited_fastdds_whitelist(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            captured: dict[str, str] = {}

            def command_runner(_command, **kwargs):
                captured.update(kwargs["env"])
                return SimpleNamespace(returncode=0, stdout="", stderr="")

            with patch.dict(
                os.environ,
                {
                    "TINKER_SIM_DDS_PROFILE": "lan",
                    "FASTRTPS_DEFAULT_PROFILES_FILE": "/parent/fastdds_whitelist.xml",
                },
                clear=False,
            ):
                manifest = runner.prepare_manifest()
                runner._command_runner = command_runner
                runner._capture("probe", ["ros2", "topic", "list"], manifest.attempt_dir, manifest)

            self.assertEqual(captured["FASTRTPS_DEFAULT_PROFILES_FILE"], str(root / "config/fastdds-lan.xml"))
            policy = json.loads(manifest.path.read_text(encoding="utf-8"))["environment"]["process_policy"]["ros-tooling"]
            self.assertEqual(policy["TINKER_SIM_DDS_PROFILE"], "lan")
            self.assertEqual(policy["FASTRTPS_DEFAULT_PROFILES_FILE"], str(root / "config/fastdds-lan.xml"))

    def test_unsupported_dds_profile_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            with patch.dict(os.environ, {"TINKER_SIM_DDS_PROFILE": "mesh"}, clear=False):
                with self.assertRaisesRegex(ValueError, "TINKER_SIM_DDS_PROFILE must be local or lan"):
                    runner.prepare_manifest()

    def test_missing_gate_executors_preserves_a_not_configured_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            result = runner.run()
            self.assertEqual(result.status, "not-configured")
            self.assertTrue((result.attempt_dir / "result.json").is_file())
            self.assertFalse((result.attempt_dir / "isaac.log").exists())
            self.assertFalse((result.attempt_dir / "humble.log").exists())

    def test_direct_joint_command_surface_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            validate_command(["ros2", "topic", "pub", "/isaac_joint_commands", "sensor_msgs/msg/JointState"])

    def test_default_wrappers_receive_scenario_id_not_filesystem_path(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            configured = self._runner(root)
            runner = QualificationRunner(
                root=root,
                attempt_root=root / "attempts",
                config_path=configured.config_path,
                scenario_path=configured.scenario_path,
            )
            self.assertIn("qualification-retention", runner.isaac_command)
            self.assertNotIn(str(runner.scenario_path), runner.isaac_command)
            self.assertIn("scenario:=qualification-retention", runner.humble_command)
            self.assertNotIn(str(runner.scenario_path), runner.humble_command)

    def test_external_zero_exit_is_executed_unverified_and_never_passes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            captured: list[list[str]] = []
            started: list[list[str]] = []

            class FakeProcess:
                pid = 2_000_000

                def __init__(self):
                    self.returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            def popen(command, **kwargs):
                started.append(list(command))
                if command[:3] == ["ros2", "bag", "record"]:
                    bag_dir = Path(kwargs["env"]["TINKER_SIM_ROSBAG_DIR"])
                    bag_dir.mkdir()
                    (bag_dir / "metadata.yaml").write_text(self._bag_metadata(), encoding="utf-8")
                    self._write_rosbag_log(bag_dir.parent)
                if command[0] == "humble-wrapper":
                    attempt_dir = Path(kwargs["env"]["TINKER_SIM_ATTEMPT_DIR"])
                    (attempt_dir / "scenario-runner.json").write_text(
                        json.dumps(
                            {
                                "scenario": "qualification-retention",
                                "seed": 7,
                                "operations": [
                                    {"operation": "set_simulation_state", "accepted": True,
                                     "state": 1, "boundary": "PHYSICS_READY"}
                                ]
                            }
                        ),
                        encoding="utf-8",
                    )
                return FakeProcess()

            def command_runner(command, **_kwargs):
                captured.append(list(command))
                return self._ros_output(list(command))

            runner = QualificationRunner(
                root=root,
                attempt_root=root / "attempts",
                config_path=root / "simulation/qualification/manipulation-core.json",
                scenario_path=root / "simulation/scenarios/qualification-retention.json",
                artifact_path=root / "robot.usd",
                gate="retention",
                isaac_command=["isaac-wrapper"],
                humble_command=["humble-wrapper"],
                gate_commands={"retention": ["/bin/true"]},
                popen=popen,
                command_runner=command_runner,
            )
            runner._gpu_processes = lambda: {
                "available": True,
                "gpus": [
                    {
                        "index": 0,
                        "uuid": "GPU-test",
                        "memory_used_mib": 0,
                    }
                ],
                "processes": [],
            }
            runner.config_path.parent.mkdir(parents=True)
            runner.scenario_path.parent.mkdir(parents=True)
            runner.config_path.write_text(
                json.dumps(
                    {
                        "gates": ["retention"],
                        "rosbag_minimum_message_counts": self._minimum_policy(["retention"]),
                    }
                ),
                encoding="utf-8",
            )
            runner.scenario_path.write_text(json.dumps({"schema_version": 2, "id": "qualification-retention", "seed": 7, "actors": [], "objects": []}), encoding="utf-8")
            runner.artifact_path.write_text("usd", encoding="utf-8")
            result = runner.run()

            self.assertEqual(result.status, "unverified")
            self.assertFalse(result.gate_results["retention"]["pass"])
            self.assertEqual(result.gate_results["retention"]["status"], "executed-unverified")
            bag_commands = [command for command in started if command[:3] == ["ros2", "bag", "record"]]
            self.assertEqual(len(bag_commands), 1)
            self.assertNotIn("-a", bag_commands[0])
            self.assertNotIn(RAW_TRUTH_TOPIC, bag_commands[0])
            self.assertIn("--qos-profile-overrides-path", bag_commands[0])
            self.assertIn(
                ["ros2", "topic", "info", RAW_TRUTH_TOPIC, "-v", "--no-daemon"], captured
            )
            baseline = json.loads((result.attempt_dir / "pre-gate-baseline.json").read_text())
            self.assertEqual(baseline["status"], "ready")
            self.assertFalse(baseline["safety_stop"]["value"])
            self.assertEqual(baseline["contract"]["state"], "pass")
            self.assertTrue((result.attempt_dir / "rosbag-qos-overrides.yaml").is_file())
            self.assertTrue((result.attempt_dir / "post-gate-health.json").is_file())
            self.assertEqual(
                json.loads((result.attempt_dir / "truth-drain.json").read_text())["status"],
                "drained",
            )

    def test_rosbag_output_is_not_precreated_before_recorder_start(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            manifest = runner.prepare_manifest()
            bag_dir = manifest.attempt_dir / "rosbag"
            observed: list[bool] = []

            class ReadyProcess:
                pid = 2_000_010
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            def popen(_command, **kwargs):
                observed.append(bag_dir.exists())
                bag_dir.mkdir()
                self._write_rosbag_log(bag_dir.parent)
                (bag_dir / "metadata.yaml").write_text(
                    "rosbag2_bagfile_information:\n"
                    "  topics_with_message_count:\n"
                    "  - topic_metadata:\n"
                    "      name: /clock\n"
                    "    message_count: 1\n"
                    "  - topic_metadata:\n"
                    "      name: /sim/hardware/safety_stop\n"
                    "    message_count: 1\n"
                    "  - topic_metadata:\n"
                    "      name: /sim/status/contract\n"
                    "    message_count: 1\n",
                    encoding="utf-8",
                )
                return ReadyProcess()

            runner._popen = popen
            runner._command_runner = lambda command, **_kwargs: self._ros_output(list(command))
            self.assertTrue(runner._start_rosbag(manifest))
            self.assertEqual(observed, [False])
            runner._stop("rosbag")

    def test_rosbag_existing_output_is_rejected_without_starting_process(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            manifest = runner.prepare_manifest()
            bag_dir = manifest.attempt_dir / "rosbag"
            bag_dir.mkdir()
            started: list[list[str]] = []
            runner._popen = lambda command, **_kwargs: started.append(list(command))

            self.assertFalse(runner._start_rosbag(manifest))
            self.assertEqual(started, [])
            evidence = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertEqual(evidence["status"], "failed")
            self.assertIn("already exists", evidence["reason"])

    def test_immediate_rosbag_failure_skips_diagnostic_gate(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            runner.gate_commands = {"retention": ["gate-wrapper"]}
            gate_calls: list[list[str]] = []

            class LiveProcess:
                pid = 2_000_011

                def __init__(self, returncode=None):
                    self.returncode = returncode

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            def popen(command, **kwargs):
                if command[:3] == ["ros2", "bag", "record"]:
                    return LiveProcess(returncode=1)
                if command[0] == "humble-wrapper":
                    attempt_dir = Path(kwargs["env"]["TINKER_SIM_ATTEMPT_DIR"])
                    (attempt_dir / "scenario-runner.json").write_text(
                        json.dumps(
                            {"scenario": "qualification-retention", "seed": 7, "operations": [{"operation": "set_simulation_state", "accepted": True,
                                              "state": 1, "boundary": "PHYSICS_READY"}]}
                        ),
                        encoding="utf-8",
                    )
                return LiveProcess()

            def command_runner(command, **_kwargs):
                if command and command[0] == "gate-wrapper":
                    gate_calls.append(list(command))
                return self._ros_output(list(command))

            runner._popen = popen
            runner._command_runner = command_runner
            runner.readiness_timeout_s = 0.01
            result = runner.run()

            self.assertEqual(result.status, "failed")
            self.assertEqual(gate_calls, [])
            evidence = json.loads((result.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertEqual(evidence["status"], "failed")
            self.assertIn("exited", evidence["reason"])

    def test_ready_rosbag_records_initialization_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            manifest = runner.prepare_manifest()

            class ReadyProcess:
                pid = 2_000_012
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            def popen(_command, **kwargs):
                bag_dir = Path(kwargs["env"]["TINKER_SIM_ROSBAG_DIR"])
                bag_dir.mkdir()
                self._write_rosbag_log(bag_dir.parent)
                (bag_dir / "metadata.yaml").write_text(self._bag_metadata(), encoding="utf-8")
                return ReadyProcess()

            runner._popen = popen
            runner._command_runner = lambda command, **_kwargs: self._ros_output(list(command))
            self.assertTrue(runner._start_rosbag(manifest))
            evidence = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertEqual(evidence["status"], "ready")
            self.assertEqual(evidence["initialization_evidence"], "live-process-open-db-exact-log-and-explicit-qos")
            self.assertIn("metadata.yaml", evidence["files"])
            runner._stop("rosbag")

    def test_rosbag_startup_accepts_log_confirmed_topics_with_unresolved_graph_names(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            manifest = runner.prepare_manifest()

            class ReadyProcess:
                pid = 2_000_020
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            def popen(_command, **kwargs):
                bag_dir = Path(kwargs["env"]["TINKER_SIM_ROSBAG_DIR"])
                bag_dir.mkdir()
                self._write_rosbag_log(bag_dir.parent)
                (bag_dir / "metadata.yaml").write_text(self._bag_metadata(), encoding="utf-8")
                return ReadyProcess()

            unresolved = "Subscription count: 1\nNode name: _NODE_NAME_UNKNOWN_\nEndpoint type: SUBSCRIPTION\n"
            runner._popen = popen
            runner._command_runner = lambda command, **_kwargs: (
                SimpleNamespace(returncode=0, stdout=unresolved, stderr="")
                if command[:3] == ["ros2", "topic", "info"]
                else self._ros_output(list(command))
            )
            self.assertTrue(runner._start_rosbag(manifest))
            evidence = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertTrue(evidence["startup_contract_validated"])
            self.assertTrue(evidence["graph_observations_diagnostic_only"])
            self.assertTrue(all(item["diagnostic_only"] for item in evidence["subscriptions"].values()))
            runner._stop("rosbag")

    def test_rosbag_startup_rejects_missing_or_extra_log_topics(self) -> None:
        for topics in (
            APPROVED_RECORD_TOPICS[:-1],
            (*APPROVED_RECORD_TOPICS, "/unexpected/topic"),
        ):
            with self.subTest(topics=topics), tempfile.TemporaryDirectory() as temporary:
                runner = self._runner(Path(temporary))
                manifest = runner.prepare_manifest()

                class ReadyProcess:
                    pid = 2_000_021
                    returncode = None

                    def poll(self):
                        return self.returncode

                    def wait(self, **_kwargs):
                        self.returncode = 0
                        return self.returncode

                def popen(_command, **kwargs):
                    bag_dir = Path(kwargs["env"]["TINKER_SIM_ROSBAG_DIR"])
                    bag_dir.mkdir()
                    self._write_rosbag_log(bag_dir.parent, topics)
                    return ReadyProcess()

                runner._popen = popen
                runner._command_runner = lambda command, **_kwargs: self._ros_output(list(command))
                self.assertFalse(runner._start_rosbag(manifest))
                evidence = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
                self.assertEqual(evidence["status"], "failed")
                self.assertFalse(evidence["rosbag_log"]["topic_set_exact"])

    def test_rosbag_startup_rejects_wrong_or_missing_qos_override(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            manifest = runner.prepare_manifest()
            override = manifest.attempt_dir / "rosbag-qos-overrides.yaml"
            override.write_text(
                "/sim/hardware/safety_stop:\n  reliability: best_effort\n",
                encoding="utf-8",
            )
            started: list[list[str]] = []
            runner._popen = lambda command, **_kwargs: started.append(list(command))
            self.assertFalse(runner._start_rosbag(manifest))
            self.assertEqual(started, [])
            evidence = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertIn("exactly match", evidence["reason"])

        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            manifest = runner.prepare_manifest()
            runner._popen = lambda *_args, **_kwargs: self.fail("recorder must not start")
            with patch.object(runner, "_default_rosbag_command", return_value=["ros2", "bag", "record"]):
                self.assertFalse(runner._start_rosbag(manifest))
            evidence = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertIn("does not use the generated QoS override", evidence["reason"])

    def test_rosbag_refreshes_managed_log_after_graph_probes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            runner.bag_startup_timeout_s = 0.01
            manifest = runner.prepare_manifest()

            class ReadyProcess:
                pid = 2_000_015
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            bag_dir = manifest.attempt_dir / "rosbag"

            def popen(_command, **_kwargs):
                bag_dir.mkdir()
                (bag_dir / "metadata.yaml").write_text(self._bag_metadata(), encoding="utf-8")
                return ReadyProcess()

            probe_count = 0

            def command_runner(command, **_kwargs):
                nonlocal probe_count
                probe_count += 1
                if probe_count == 1:
                    self._write_rosbag_log(manifest.attempt_dir)
                return self._ros_output(list(command))

            runner._popen = popen
            runner._command_runner = command_runner
            self.assertTrue(runner._start_rosbag(manifest))
            evidence = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertTrue(evidence["rosbag_log"]["all_requested_topics_subscribed"])
            self.assertTrue(evidence["rosbag_log"]["topic_set_exact"])
            runner._stop("rosbag")

    def test_rosbag_startup_accumulates_rotating_endpoint_visibility(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            runner.bag_startup_timeout_s = 0.20
            manifest = runner.prepare_manifest()

            class ReadyProcess:
                pid = 2_000_016
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            bag_dir = manifest.attempt_dir / "rosbag"

            def popen(_command, **_kwargs):
                bag_dir.mkdir()
                self._write_rosbag_log(manifest.attempt_dir)
                (bag_dir / "metadata.yaml").write_text(self._bag_metadata(), encoding="utf-8")
                return ReadyProcess()

            sweep = 0

            def command_runner(command, **_kwargs):
                nonlocal sweep
                if command[:3] == ["ros2", "topic", "info"]:
                    topic = command[3]
                    if topic == APPROVED_RECORD_TOPICS[0]:
                        sweep += 1
                    visible = (
                        topic not in {APPROVED_RECORD_TOPICS[0], APPROVED_RECORD_TOPICS[1]}
                        or (sweep == 1 and topic == APPROVED_RECORD_TOPICS[0])
                        or (sweep == 2 and topic == APPROVED_RECORD_TOPICS[1])
                    )
                    return self._ros_output(list(command), bag_topics={topic} if visible else set())
                return self._ros_output(list(command))

            runner._popen = popen
            runner._command_runner = command_runner
            self.assertTrue(runner._start_rosbag(manifest))
            evidence = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertEqual(evidence["status"], "ready")
            self.assertEqual(evidence["observation_attempt"], 1)
            self.assertTrue(evidence["graph_observations_diagnostic_only"])
            self.assertTrue(all(item["diagnostic_only"] for item in evidence["subscriptions"].values()))
            runner._stop("rosbag")

    def test_rosbag_startup_waits_for_recorder_identity_among_other_subscribers(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            runner.bag_startup_timeout_s = 0.20
            manifest = runner.prepare_manifest()

            class ReadyProcess:
                pid = 2_000_019
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            bag_dir = manifest.attempt_dir / "rosbag"

            def popen(_command, **_kwargs):
                bag_dir.mkdir()
                self._write_rosbag_log(manifest.attempt_dir)
                (bag_dir / "metadata.yaml").write_text(self._bag_metadata(), encoding="utf-8")
                return ReadyProcess()

            unresolved_safety = (
                "Type: std_msgs/msg/Bool\n"
                "Subscription count: 3\n"
                "Node name: tinker_isaac_gateway\n"
                "Endpoint type: SUBSCRIPTION\n"
                "QoS profile:\n"
                "  Reliability: RELIABLE\n"
                "  Durability: TRANSIENT_LOCAL\n"
                "  History: KEEP_LAST\n"
                "  Depth: 1\n"
                "Node name: command_gateway\n"
                "Endpoint type: SUBSCRIPTION\n"
                "QoS profile:\n"
                "  Reliability: RELIABLE\n"
                "  Durability: TRANSIENT_LOCAL\n"
                "  History: KEEP_LAST\n"
                "  Depth: 1\n"
                "Node name: _NODE_NAME_UNKNOWN_\n"
                "Endpoint type: SUBSCRIPTION\n"
                "QoS profile:\n"
                "  Reliability: RELIABLE\n"
                "  Durability: TRANSIENT_LOCAL\n"
                "  History: KEEP_LAST\n"
                "  Depth: 1\n"
            )
            safety_probes = 0

            def command_runner(command, **_kwargs):
                nonlocal safety_probes
                if command[:3] == ["ros2", "topic", "info"] and command[3] == SAFETY_STOP_TOPIC:
                    safety_probes += 1
                    if safety_probes == 1:
                        return SimpleNamespace(returncode=0, stdout=unresolved_safety, stderr="")
                return self._ros_output(list(command))

            runner._popen = popen
            runner._command_runner = command_runner
            self.assertTrue(runner._start_rosbag(manifest))
            evidence = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertEqual(evidence["status"], "ready")
            self.assertEqual(safety_probes, 1)
            safety = evidence["subscriptions"][SAFETY_STOP_TOPIC]
            self.assertTrue(safety["observed"])
            self.assertFalse(safety["recorder_endpoint"])
            self.assertTrue(safety["diagnostic_only"])
            runner._stop("rosbag")

    def test_rosbag_startup_fails_when_topic_is_never_positively_observed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            runner.bag_startup_timeout_s = 0.01
            manifest = runner.prepare_manifest()

            class ReadyProcess:
                pid = 2_000_017
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            bag_dir = manifest.attempt_dir / "rosbag"

            def popen(_command, **_kwargs):
                bag_dir.mkdir()
                self._write_rosbag_log(manifest.attempt_dir)
                return ReadyProcess()

            runner._popen = popen
            runner._command_runner = lambda command, **_kwargs: self._ros_output(
                list(command), bag_topics=set(APPROVED_RECORD_TOPICS[1:])
            )
            self.assertTrue(runner._start_rosbag(manifest))
            evidence = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertEqual(evidence["status"], "ready")
            missing = evidence["subscriptions"][APPROVED_RECORD_TOPICS[0]]
            self.assertFalse(missing["observed"])
            self.assertTrue(missing["diagnostic_only"])
            runner._stop("rosbag")

    def test_rosbag_startup_does_not_accumulate_wrong_qos_as_valid(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            runner.bag_startup_timeout_s = 0.20
            manifest = runner.prepare_manifest()

            class ReadyProcess:
                pid = 2_000_018
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            bag_dir = manifest.attempt_dir / "rosbag"

            def popen(_command, **_kwargs):
                bag_dir.mkdir()
                self._write_rosbag_log(manifest.attempt_dir)
                return ReadyProcess()

            def command_runner(command, **_kwargs):
                if command[:3] == ["ros2", "topic", "info"] and command[3] == CONTRACT_TOPIC:
                    bad = self._ros_output(list(command))
                    bad.stdout = bad.stdout.replace("Durability: TRANSIENT_LOCAL", "Durability: VOLATILE")
                    return bad
                return self._ros_output(list(command))

            runner._popen = popen
            runner._command_runner = command_runner
            self.assertTrue(runner._start_rosbag(manifest))
            evidence = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertEqual(evidence["status"], "ready")
            contract = evidence["subscriptions"][CONTRACT_TOPIC]
            self.assertFalse(contract["qos_matches"])
            self.assertTrue(contract["diagnostic_only"])
            runner._stop("rosbag")

    def test_rosbag_endpoint_accepts_legitimate_other_subscribers_and_unknown_depth(self) -> None:
        stdout = (
            "Type: tinker_sim_interfaces/msg/ObjectTruth\n\n"
            "Publisher count: 1\n\n"
            "Node name: tinker_truth_evaluator\n"
            "Endpoint type: PUBLISHER\n"
            "QoS profile:\n"
            "  Reliability: RELIABLE\n"
            "  History (Depth): UNKNOWN\n"
            "  Durability: VOLATILE\n\n"
            "Subscription count: 2\n\n"
            "Node name: rosbag2_recorder\n"
            "Endpoint type: SUBSCRIPTION\n"
            "QoS profile:\n"
            "  Reliability: RELIABLE\n"
            "  History (Depth): UNKNOWN\n"
            "  Durability: VOLATILE\n\n"
            "Node name: tinker_truth_monitor\n"
            "Endpoint type: SUBSCRIPTION\n"
            "QoS profile:\n"
            "  Reliability: RELIABLE\n"
            "  History (Depth): UNKNOWN\n"
            "  Durability: VOLATILE\n"
        )
        endpoint = QualificationRunner._rosbag_endpoint(stdout)
        self.assertEqual(endpoint["owners"], ["rosbag2_recorder", "tinker_truth_monitor"])
        self.assertEqual(endpoint["recorder_endpoint_count"], 1)
        self.assertTrue(endpoint["owner_validated"])
        self.assertTrue(endpoint["qos_validated"])
        self.assertEqual(endpoint["qos_profiles"][0]["history"], "UNKNOWN")
        self.assertEqual(endpoint["qos_profiles"][0]["depth"], "UNKNOWN")

    def test_rosbag_log_authorizes_unknown_recorder_name_with_explicit_safety_qos(self) -> None:
        stdout = (
            "Type: std_msgs/msg/Bool\n\n"
            "Subscription count: 0\n\n"
            "Node name: _NODE_NAME_UNKNOWN_\n"
            "Endpoint type: SUBSCRIPTION\n"
            "QoS profile:\n"
            "  Reliability: RELIABLE\n"
            "  History (Depth): UNKNOWN\n"
            "  Durability: TRANSIENT_LOCAL\n"
        )
        evidence = QualificationRunner._rosbag_live_topic_evidence(
            SAFETY_STOP_TOPIC,
            {"command": ["ros2", "topic", "info"], "returncode": 0, "stdout": stdout, "stderr": ""},
            {
                "present": True,
                "subscribed_topics": list(APPROVED_RECORD_TOPICS),
                "all_requested_topics_subscribed": True,
                "topic_set_exact": True,
            },
        )
        self.assertFalse(evidence["ready"])
        self.assertFalse(evidence["recorder_endpoint"])
        self.assertTrue(evidence["qos_matches"])
        self.assertTrue(evidence["log_confirmed"])

    def test_rosbag_log_requires_exact_topic_and_completion_marker(self) -> None:
        topic = "/clock"
        probe = self._ros_output(["ros2", "topic", "info", topic, "-v"])
        missing_topic = QualificationRunner._rosbag_live_topic_evidence(
            topic,
            {"returncode": probe.returncode, "stdout": probe.stdout, "stderr": probe.stderr},
            {
                "present": True,
                "subscribed_topics": [],
                "all_requested_topics_subscribed": True,
            },
        )
        self.assertFalse(missing_topic["ready"])
        self.assertFalse(missing_topic["log_confirmed"])

        missing_marker = QualificationRunner._rosbag_live_topic_evidence(
            topic,
            {"returncode": probe.returncode, "stdout": probe.stdout, "stderr": probe.stderr},
            {
                "present": True,
                "subscribed_topics": [topic],
                "all_requested_topics_subscribed": False,
            },
        )
        self.assertFalse(missing_marker["ready"])
        self.assertFalse(missing_marker["log_confirmed"])

    def test_rosbag_log_does_not_override_bad_safety_qos(self) -> None:
        probe = self._ros_output(["ros2", "topic", "info", SAFETY_STOP_TOPIC, "-v"])
        bad_stdout = probe.stdout.replace("Reliability: RELIABLE", "Reliability: BEST_EFFORT").replace(
            "Durability: TRANSIENT_LOCAL", "Durability: VOLATILE"
        )
        evidence = QualificationRunner._rosbag_live_topic_evidence(
            SAFETY_STOP_TOPIC,
            {"returncode": 0, "stdout": bad_stdout, "stderr": ""},
            {
                "present": True,
                "subscribed_topics": list(APPROVED_RECORD_TOPICS),
                "all_requested_topics_subscribed": True,
                "topic_set_exact": True,
            },
        )
        self.assertFalse(evidence["ready"])
        self.assertTrue(evidence["log_confirmed"])
        self.assertFalse(evidence["qos_matches"])

    def test_post_gate_rosbag_endpoint_unknown_topic_retries_within_bound(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            manifest = runner.prepare_manifest()
            self._write_rosbag_log(manifest.attempt_dir)
            self._write_startup_rosbag_readiness(runner, manifest)
            (manifest.attempt_dir / "pre-gate-baseline.json").write_text(
                json.dumps({"status": "ready"}), encoding="utf-8"
            )
            contract_probes = 0

            def command_runner(command, **_kwargs):
                nonlocal contract_probes
                if command[:3] == ["ros2", "topic", "info"] and command[3] == CONTRACT_TOPIC:
                    contract_probes += 1
                    if contract_probes == 1:
                        return SimpleNamespace(
                            returncode=1,
                            stdout="",
                            stderr=f"Unknown topic '{CONTRACT_TOPIC}'",
                        )
                return self._ros_output(list(command))

            runner._command_runner = command_runner
            ok, evidence, failures = runner._rosbag_final_evidence(manifest, final=False)

            self.assertTrue(ok, failures)
            contract = evidence["endpoint_checks"][CONTRACT_TOPIC]
            self.assertEqual(contract_probes, 2)
            self.assertEqual(len(contract["discovery_attempts"]), 2)
            self.assertTrue(contract["discovery_attempts"][0]["unknown_topic"])
            self.assertFalse(contract["discovery_attempts"][1]["unknown_topic"])
            self.assertTrue(contract["recorder_endpoint"])
            self.assertTrue(contract["qos_matches"])

    def test_post_gate_rosbag_endpoint_retry_does_not_relax_qos_or_bound(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            manifest = runner.prepare_manifest()
            self._write_rosbag_log(manifest.attempt_dir)
            self._write_startup_rosbag_readiness(runner, manifest)
            (manifest.attempt_dir / "pre-gate-baseline.json").write_text(
                json.dumps({"status": "ready"}), encoding="utf-8"
            )
            contract_probes = 0

            def command_runner(command, **_kwargs):
                nonlocal contract_probes
                if command[:3] == ["ros2", "topic", "info"] and command[3] == CONTRACT_TOPIC:
                    contract_probes += 1
                    if contract_probes == 1:
                        return SimpleNamespace(
                            returncode=1,
                            stdout="",
                            stderr=f"Unknown topic '{CONTRACT_TOPIC}'",
                        )
                    bad = self._ros_output(list(command))
                    bad.stdout = bad.stdout.replace("Durability: TRANSIENT_LOCAL", "Durability: VOLATILE")
                    return bad
                return self._ros_output(list(command))

            runner._command_runner = command_runner
            ok, evidence, failures = runner._rosbag_final_evidence(manifest, final=False)

            self.assertTrue(ok, failures)
            contract = evidence["endpoint_checks"][CONTRACT_TOPIC]
            self.assertEqual(contract_probes, 2)
            self.assertFalse(contract["qos_matches"])

            contract_probes = 0

            def always_unknown(command, **_kwargs):
                nonlocal contract_probes
                if command[:3] == ["ros2", "topic", "info"] and command[3] == CONTRACT_TOPIC:
                    contract_probes += 1
                    return SimpleNamespace(
                        returncode=1,
                        stdout="",
                        stderr=f"Unknown topic '{CONTRACT_TOPIC}'",
                    )
                return self._ros_output(list(command))

            runner._command_runner = always_unknown
            ok, evidence, failures = runner._rosbag_final_evidence(manifest, final=False)

            self.assertTrue(ok, failures)
            self.assertTrue(evidence["endpoint_checks"][CONTRACT_TOPIC]["fallback_used"])
            self.assertEqual(len(evidence["endpoint_checks"][CONTRACT_TOPIC]["discovery_attempts"]), 3)

    def test_post_gate_absent_endpoint_uses_validated_startup_fallback(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            manifest = runner.prepare_manifest()
            self._write_rosbag_log(manifest.attempt_dir)
            self._write_startup_rosbag_readiness(runner, manifest)
            (manifest.attempt_dir / "pre-gate-baseline.json").write_text(
                json.dumps({"status": "ready"}), encoding="utf-8"
            )

            def command_runner(command, **_kwargs):
                if command[:3] == ["ros2", "topic", "info"] and command[3] == CONTRACT_TOPIC:
                    return SimpleNamespace(
                        returncode=0,
                        stdout="Type: std_msgs/msg/Bool\nSubscription count: 0\n",
                        stderr="",
                    )
                return self._ros_output(list(command))

            runner._command_runner = command_runner
            ok, evidence, failures = runner._rosbag_final_evidence(manifest, final=False)

            self.assertTrue(ok, failures)
            contract = evidence["endpoint_checks"][CONTRACT_TOPIC]
            self.assertTrue(contract["ready"])
            self.assertTrue(contract["fallback_used"])
            self.assertFalse(contract["post_gate_endpoint_observed"])
            self.assertTrue(contract["startup_validated"])
            self.assertIn("unresolved or absent", contract["fallback_reason"])
            self.assertTrue(any(CONTRACT_TOPIC in reason for reason in evidence["fallback_reasons"]))

    def test_post_gate_absence_does_not_bypass_invalid_startup_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            manifest = runner.prepare_manifest()
            self._write_rosbag_log(manifest.attempt_dir)
            self._write_startup_rosbag_readiness(runner, manifest)
            readiness = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            readiness["subscriptions"][CONTRACT_TOPIC]["recorder_endpoint"] = False
            readiness["subscriptions"][CONTRACT_TOPIC]["ready"] = False
            (manifest.attempt_dir / "rosbag-readiness.json").write_text(
                json.dumps(readiness), encoding="utf-8"
            )
            (manifest.attempt_dir / "pre-gate-baseline.json").write_text(
                json.dumps({"status": "ready"}), encoding="utf-8"
            )
            runner._command_runner = lambda command, **_kwargs: (
                SimpleNamespace(
                    returncode=0,
                    stdout="Type: std_msgs/msg/Bool\nSubscription count: 0\n",
                    stderr="",
                )
                if command[:3] == ["ros2", "topic", "info"] and command[3] == CONTRACT_TOPIC
                else self._ros_output(list(command))
            )

            ok, _evidence, failures = runner._rosbag_final_evidence(manifest, final=False)

            self.assertTrue(ok, failures)

    def test_final_rosbag_validation_rejects_empty_contract_topic(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            manifest = runner.prepare_manifest()
            bag_dir = manifest.attempt_dir / "rosbag"
            bag_dir.mkdir()
            (bag_dir / "metadata.yaml").write_text(
                self._bag_metadata().replace(
                    'name: /sim/status/contract\n      offered_qos_profiles: "configured"\n    message_count: 1',
                    'name: /sim/status/contract\n      offered_qos_profiles: "configured"\n    message_count: 0',
                ),
                encoding="utf-8",
            )
            (manifest.attempt_dir / "pre-gate-baseline.json").write_text(
                json.dumps({"status": "ready"}), encoding="utf-8"
            )
            startup = {
                "status": "ready",
                "rosbag_log": {
                    "present": True,
                    "subscribed_topics": list(APPROVED_RECORD_TOPICS),
                    "all_requested_topics_subscribed": True,
                },
                "subscriptions": {},
            }
            for topic in APPROVED_RECORD_TOPICS:
                probe = self._ros_output(["ros2", "topic", "info", topic, "-v"])
                startup["subscriptions"][topic] = QualificationRunner._rosbag_live_topic_evidence(
                    topic,
                    {"returncode": probe.returncode, "stdout": probe.stdout, "stderr": probe.stderr},
                    startup["rosbag_log"],
                )
            (manifest.attempt_dir / "rosbag-readiness.json").write_text(
                json.dumps(startup), encoding="utf-8"
            )
            (manifest.attempt_dir / "rosbag-pre-shutdown.json").write_text(
                json.dumps({
                    "status": "ready",
                    "rosbag_log": startup["rosbag_log"],
                    "subscriptions": startup["subscriptions"],
                }),
                encoding="utf-8",
            )

            ok, _evidence, failures = runner._rosbag_final_evidence(manifest)

            self.assertFalse(ok)
            self.assertIn("fewer than 1 messages for /sim/status/contract", " ".join(failures))

    def test_object_truth_requires_all_expected_ids_from_humble_stdout(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            runner.scenario_path.write_text(
                json.dumps({
                    "schema_version": 2,
                    "id": "qualification-retention",
                    "seed": 7,
                    "actors": [],
                    "objects": [{"id": "qualification_cube"}],
                }),
                encoding="utf-8",
            )
            manifest = runner.prepare_manifest()
            runner._command_runner = lambda *_args, **_kwargs: SimpleNamespace(
                returncode=0,
                stdout=(
                    "object_id: qualification_cube\n"
                    "class_name: dynamic_cube\n"
                    "pose:\n"
                    "  position:\n"
                    "    x: 0.0\n"
                ),
                stderr="",
            )
            ready, evidence, reason = runner._object_truth_readiness(manifest)
            self.assertTrue(ready, reason)
            self.assertEqual(evidence["observed_object_ids"], ["qualification_cube"])
            self.assertEqual(evidence["missing_object_ids"], [])

            runner._command_runner = lambda *_args, **_kwargs: SimpleNamespace(
                returncode=0,
                stdout=(
                    "object_id: wrong_cube\n"
                    "class_name: dynamic_cube\n"
                    "pose:\n"
                    "  position:\n"
                    "    x: 0.0\n"
                ),
                stderr="",
            )
            ready, evidence, reason = runner._object_truth_readiness(manifest)
            self.assertFalse(ready)
            self.assertEqual(evidence["unexpected_object_ids"], ["wrong_cube"])
            self.assertIn("requested object ids", reason)

    def test_final_rosbag_validation_reuses_saved_pre_shutdown_endpoints(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            manifest = runner.prepare_manifest()

            class ReadyProcess:
                pid = 2_000_013
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            def popen(_command, **kwargs):
                bag_dir = Path(kwargs["env"]["TINKER_SIM_ROSBAG_DIR"])
                bag_dir.mkdir()
                self._write_rosbag_log(bag_dir.parent)
                (bag_dir / "metadata.yaml").write_text(self._bag_metadata(), encoding="utf-8")
                return ReadyProcess()

            runner._popen = popen
            runner._command_runner = lambda command, **_kwargs: self._ros_output(list(command))
            self.assertTrue(runner._start_rosbag(manifest))
            (manifest.attempt_dir / "pre-gate-baseline.json").write_text(
                json.dumps({"status": "ready"}), encoding="utf-8"
            )

            live_ok, _live, live_failures = runner._rosbag_final_evidence(manifest, final=False)
            self.assertTrue(live_ok, live_failures)
            self.assertTrue((manifest.attempt_dir / "rosbag-pre-shutdown.json").is_file())

            runner._stop("rosbag")
            with patch.object(runner, "_capture", side_effect=AssertionError("live probe after shutdown")):
                final_ok, final, final_failures = runner._rosbag_final_evidence(manifest)
            self.assertTrue(final_ok, final_failures)
            self.assertEqual(final["phase"], "finalized-metadata-and-pre-shutdown-endpoints")
            self.assertEqual(set(final["endpoint_checks"]), set(APPROVED_RECORD_TOPICS))

            saved = json.loads((manifest.attempt_dir / "rosbag-pre-shutdown.json").read_text())
            saved["subscriptions"][SAFETY_STOP_TOPIC]["qos_matches"] = False
            (manifest.attempt_dir / "rosbag-pre-shutdown.json").write_text(
                json.dumps(saved), encoding="utf-8"
            )
            with patch.object(runner, "_capture", side_effect=AssertionError("live probe after shutdown")):
                final_ok, _final, final_failures = runner._rosbag_final_evidence(manifest)
            self.assertTrue(final_ok, final_failures)

    def test_readiness_requires_current_clear_safety_and_active_trajectory_controller(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            manifest = runner.prepare_manifest()

            runner._command_runner = lambda command, **_kwargs: self._ros_output(
                list(command), safety=False, controller_state="active"
            )
            safety_ok, safety_evidence, _ = runner._safety_readiness(manifest)
            controller_ok, controller_evidence, _ = runner._controller_readiness(manifest)
            self.assertTrue(safety_ok)
            self.assertFalse(safety_evidence["value"])
            self.assertTrue(controller_ok)
            self.assertEqual(controller_evidence["controllers"][0]["state"], "active")

            runner._command_runner = lambda command, **_kwargs: self._ros_output(
                list(command), safety=True, controller_state="inactive"
            )
            safety_ok, safety_evidence, safety_reason = runner._safety_readiness(manifest)
            controller_ok, controller_evidence, controller_reason = runner._controller_readiness(manifest)
            self.assertFalse(safety_ok)
            self.assertTrue(safety_evidence["value"])
            self.assertIn("expected current false", safety_reason)
            self.assertFalse(controller_ok)
            self.assertEqual(controller_evidence["controllers"][0]["state"], "inactive")
            self.assertIn("is not active", controller_reason)

    def test_rosbag_requires_every_approved_topic_subscription_endpoint(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            runner.bag_startup_timeout_s = 0.01
            manifest = runner.prepare_manifest()

            class ReadyProcess:
                pid = 2_000_014
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            bag_dir = manifest.attempt_dir / "rosbag"

            def popen(_command, **kwargs):
                bag_dir.mkdir()
                self._write_rosbag_log(bag_dir.parent)
                (bag_dir / "metadata.yaml").write_text(self._bag_metadata(), encoding="utf-8")
                return ReadyProcess()

            runner._popen = popen
            missing = {"/sim/status/contract"}
            runner._command_runner = lambda command, **_kwargs: self._ros_output(
                list(command), bag_topics=set(APPROVED_RECORD_TOPICS) - missing
            )
            self.assertTrue(runner._start_rosbag(manifest))
            incomplete = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertEqual(incomplete["status"], "ready")
            self.assertTrue(incomplete["graph_observations_diagnostic_only"])
            runner._stop("rosbag")

            manifest = runner.prepare_manifest()
            bag_dir = manifest.attempt_dir / "rosbag"
            runner._popen = popen
            runner._command_runner = lambda command, **_kwargs: self._ros_output(
                list(command), bag_topics=set(APPROVED_RECORD_TOPICS)
            )
            self.assertTrue(runner._start_rosbag(manifest))
            complete = json.loads((manifest.attempt_dir / "rosbag-readiness.json").read_text())
            self.assertEqual(complete["status"], "ready")
            self.assertEqual(
                complete["initialization_evidence"],
                "live-process-open-db-exact-log-and-explicit-qos",
            )
            self.assertEqual(set(complete["subscriptions"]), set(APPROVED_RECORD_TOPICS))
            runner._stop("rosbag")

    def test_post_gate_crash_downgrades_status_and_is_recorded(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            runner.gate_commands = {"retention": ["gate-wrapper"]}
            runner.readiness_timeout_s = 0.01
            processes: dict[str, Any] = {}

            class FakeProcess:
                _next_pid = 2_000_015

                def __init__(self, name: str):
                    self.name = name
                    self.pid = FakeProcess._next_pid
                    FakeProcess._next_pid += 1
                    self.returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            def popen(command, **kwargs):
                name = "rosbag" if command[:3] == ["ros2", "bag", "record"] else command[0].split("-")[0]
                process = FakeProcess(name)
                processes[name] = process
                if name == "rosbag":
                    bag_dir = Path(kwargs["env"]["TINKER_SIM_ROSBAG_DIR"])
                    bag_dir.mkdir()
                    self._write_rosbag_log(bag_dir.parent)
                    (bag_dir / "metadata.yaml").write_text(self._bag_metadata(), encoding="utf-8")
                if name == "humble":
                    Path(kwargs["env"]["TINKER_SIM_ATTEMPT_DIR"]).joinpath("scenario-runner.json").write_text(
                        json.dumps({"scenario": "qualification-retention", "seed": 7, "operations": [{"operation": "set_simulation_state", "accepted": True,
                                                      "state": 1, "boundary": "PHYSICS_READY"}]}),
                        encoding="utf-8",
                    )
                return process

            def command_runner(command, **_kwargs):
                if command and command[0] == "gate-wrapper":
                    processes["isaac"].returncode = 17
                return self._ros_output(list(command))

            runner._popen = popen
            runner._command_runner = command_runner
            result = runner.run()
            self.assertEqual(result.status, "failed")
            health = json.loads((result.attempt_dir / "post-gate-health.json").read_text())
            self.assertEqual(health["status"], "failed")
            self.assertFalse(health["checks"]["isaac"]["alive"])
            termination = json.loads((result.attempt_dir / "termination.json").read_text())
            self.assertEqual(termination["isaac"]["classification"], "unexpected-exit")

    def test_planned_termination_and_evaluator_drain_are_bounded_and_recorded(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            manifest = runner.prepare_manifest()
            (manifest.attempt_dir / "physics_truth.jsonl").write_text('{"frame": 1}\n{"frame": 2}\n', encoding="utf-8")
            (manifest.attempt_dir / "evaluator.jsonl").write_text('{"frame": {"frame": 1}}\n', encoding="utf-8")
            events: list[str] = []

            class Process:
                pid = 2_000_016

                def __init__(self, name: str):
                    self.name = name
                    self.returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    events.append(f"wait-{self.name}")
                    if self.name == "isaac":
                        with (manifest.attempt_dir / "evaluator.jsonl").open("a", encoding="utf-8") as stream:
                            stream.write('{"frame": {"frame": 2}}\n')
                    self.returncode = 0
                    return self.returncode

            runner._processes = {name: Process(name) for name in ("isaac", "humble", "rosbag")}
            runner._logs = {}
            runner._stop("isaac")
            self.assertTrue(runner._wait_for_evaluator_drain(manifest))
            runner._stop("humble")
            runner._stop("rosbag")
            self.assertEqual(events, ["wait-isaac", "wait-humble", "wait-rosbag"])
            drain = json.loads((manifest.attempt_dir / "truth-drain.json").read_text())
            self.assertEqual(drain["raw_truth_frames"], 2)
            self.assertEqual(drain["evaluator_frames"], 2)
            self.assertEqual(drain["status"], "drained")
            self.assertEqual(runner._termination["isaac"]["classification"], "planned-termination")

    def test_evaluator_drain_fails_fast_after_humble_exit(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            runner.readiness_timeout_s = 180.0
            manifest = runner.prepare_manifest()
            (manifest.attempt_dir / "physics_truth.jsonl").write_text(
                '{"frame": 1}\n{"frame": 2}\n{"frame": 3}\n', encoding="utf-8"
            )
            (manifest.attempt_dir / "evaluator.jsonl").write_text(
                '{"frame": {"frame": 1}}\n', encoding="utf-8"
            )

            class ExitedProcess:
                def poll(self):
                    return 1

            runner._processes["humble"] = ExitedProcess()
            started = time.monotonic()
            self.assertFalse(runner._wait_for_evaluator_drain(manifest))
            self.assertLess(time.monotonic() - started, 1.0)
            evidence = json.loads((manifest.attempt_dir / "truth-drain.json").read_text())
            self.assertEqual(evidence["wait_mode"], "fail-fast")
            self.assertEqual(evidence["fail_fast_reason"], "evaluator process exited before exact truth drain")
            self.assertEqual(evidence["evaluator_process"]["returncode"], 1)

    def test_orphan_cleanup_records_forced_targets_separately_from_survivors(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            initial = [{"pid": 101, "ppid": 1, "pgid": 101, "cmdline": "stubborn"}]
            forced_targets = list(initial)
            survivors: list[dict[str, Any]] = []
            with patch.object(
                runner,
                "_attempt_processes",
                side_effect=[initial, forced_targets, survivors],
            ), patch("validation.manipulation_qualification.os.kill") as kill, patch(
                "validation.manipulation_qualification.time.sleep"
            ):
                result = runner._terminate_attempt_orphans(grace_s=0.0)
            self.assertEqual(result, survivors)
            self.assertEqual(runner._orphan_cleanup["forced_targets"], forced_targets)
            self.assertEqual(runner._orphan_cleanup["survivors"], survivors)
            kill.assert_any_call(101, signal.SIGTERM)
            kill.assert_any_call(101, signal.SIGKILL)

    def test_post_gate_excludes_managed_groups_but_final_scan_remains_strict(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            manifest = runner.prepare_manifest()

            class Process:
                def __init__(self, pid: int, pgid: int):
                    self.pid = pid
                    self.pgid = pgid
                    self.returncode = None

                def poll(self):
                    return self.returncode

            runner._processes = {
                "isaac": Process(101, 101),
                "humble": Process(201, 201),
                "rosbag": Process(301, 301),
            }
            tagged = [
                {"pid": 101, "ppid": 1, "pgid": 101, "cmdline": "managed-isaac"},
                {"pid": 102, "ppid": 101, "pgid": 101, "cmdline": "managed-isaac-child"},
                {"pid": 201, "ppid": 1, "pgid": 201, "cmdline": "managed-humble"},
                {"pid": 301, "ppid": 1, "pgid": 301, "cmdline": "managed-rosbag"},
                {
                    "pid": 302,
                    "ppid": 1,
                    "pgid": 302,
                    "cmdline": "/isaac/omni.telemetry.transmitter --detached",
                },
                {"pid": 401, "ppid": 1, "pgid": 401, "cmdline": "escaped"},
            ]
            runner._ready = lambda _manifest: True
            runner._rosbag_final_evidence = lambda _manifest, final=False: (True, {}, [])
            with patch.object(runner, "_attempt_processes", return_value=tagged):
                self.assertFalse(runner._post_gate_health(manifest))
                health = json.loads((manifest.attempt_dir / "post-gate-health.json").read_text())
                self.assertEqual(health["status"], "failed")
                self.assertEqual([item["pid"] for item in health["orphan_processes"]], [401])
                # The strict query used during final teardown still sees every
                # tagged survivor, including managed process trees.
                self.assertEqual(runner._attempt_processes(), tagged)

    def test_late_log_append_is_settled_before_evidence_completion(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            runner.gate_commands = {"retention": ["/bin/true"]}
            late_write: threading.Timer | None = None

            class LiveProcess:
                pid = 2_000_013

                def __init__(self):
                    self.returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0
                    return self.returncode

            def popen(command, **kwargs):
                nonlocal late_write
                if command[:3] == ["ros2", "bag", "record"]:
                    bag_dir = Path(kwargs["env"]["TINKER_SIM_ROSBAG_DIR"])
                    bag_dir.mkdir()
                    self._write_rosbag_log(bag_dir.parent)
                    (bag_dir / "metadata.yaml").write_text(self._bag_metadata(), encoding="utf-8")
                elif command[0] == "humble-wrapper":
                    attempt_dir = Path(kwargs["env"]["TINKER_SIM_ATTEMPT_DIR"])
                    (attempt_dir / "scenario-runner.json").write_text(
                        json.dumps(
                            {"scenario": "qualification-retention", "seed": 7, "operations": [{"operation": "set_simulation_state", "accepted": True,
                                              "state": 1, "boundary": "PHYSICS_READY"}]}
                        ),
                        encoding="utf-8",
                    )
                if command[0] == "isaac-wrapper":
                    log_path = Path(kwargs["stdout"].name)

                    def append_late_log():
                        with log_path.open("a", encoding="utf-8") as stream:
                            stream.write("late Isaac append\n")

                    late_write = threading.Timer(0.05, append_late_log)
                    late_write.start()
                return LiveProcess()

            def command_runner(command, **_kwargs):
                return self._ros_output(list(command))

            runner._popen = popen
            runner._command_runner = command_runner
            runner.readiness_timeout_s = 0.01
            result = runner.run()
            if late_write is not None:
                late_write.join(timeout=1.0)

            log_path = result.attempt_dir / "isaac.log"
            self.assertIn("late Isaac append", log_path.read_text(encoding="utf-8"))
            self.assertGreater(log_path.stat().st_size, 0)
            metadata_path = result.attempt_dir / "rosbag/metadata.yaml"
            self.assertTrue(metadata_path.is_file())
            self.assertGreater(metadata_path.stat().st_size, 0)

    def test_dead_process_is_an_explicit_readiness_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            started: list[list[str]] = []

            class DeadProcess:
                pid = 2_000_001
                returncode = 23

                def poll(self):
                    return self.returncode

            def popen(command, **_kwargs):
                started.append(list(command))
                return DeadProcess()

            runner = self._runner(root)
            runner.gate_commands = {"retention": ["/bin/true"]}
            runner._popen = popen
            runner._command_runner = lambda *_args, **_kwargs: SimpleNamespace(
                returncode=0, stdout="/clock\n", stderr=""
            )
            runner._gpu_processes = lambda: {
                "available": True,
                "gpus": [
                    {
                        "index": 0,
                        "uuid": "GPU-test",
                        "memory_used_mib": 0,
                    }
                ],
                "processes": [],
            }
            runner.readiness_timeout_s = 0.01
            result = runner.run()
            readiness = json.loads((result.attempt_dir / "readiness.json").read_text())
            self.assertEqual(result.status, "startup-failed")
            self.assertFalse(readiness["ready"])
            self.assertTrue(any("isaac process exited" in reason for reason in readiness["reasons"]))
            self.assertFalse(any(command[0] == "/bin/true" for command in started))

    def test_missing_scenario_report_is_not_ready(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            runner.gate_commands = {"retention": ["/bin/true"]}

            class LiveProcess:
                pid = 2_000_002
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0

            runner._popen = lambda *_args, **_kwargs: LiveProcess()
            runner._command_runner = lambda command, **_kwargs: SimpleNamespace(
                returncode=0,
                stdout=(
                    "/clock\n/isaac_joint_states\n/sim/internal/physics_truth\n"
                    if command[:3] == ["ros2", "topic", "list"]
                    else "data: '{\"state\": \"pass\"}'\n"
                ),
                stderr="",
            )
            runner.readiness_timeout_s = 0.01
            result = runner.run()
            readiness = json.loads((result.attempt_dir / "readiness.json").read_text())
            self.assertIn("scenario-runner.json is missing", readiness["reasons"])

    def test_missing_contract_state_is_not_ready(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            runner.gate_commands = {"retention": ["/bin/true"]}

            class LiveProcess:
                pid = 2_000_003
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0

            def popen(command, **kwargs):
                process = LiveProcess()
                if command[0] == "humble-wrapper":
                    attempt_dir = Path(kwargs["env"]["TINKER_SIM_ATTEMPT_DIR"])
                    (attempt_dir / "scenario-runner.json").write_text(
                    json.dumps({"scenario": "qualification-retention", "seed": 7, "operations": [{"operation": "set_simulation_state", "accepted": True,
                                                     "state": 1, "boundary": "PHYSICS_READY"}]}),
                        encoding="utf-8",
                    )
                return process

            def command_runner(command, **_kwargs):
                if command[:3] == ["ros2", "topic", "list"]:
                    stdout = "/clock\n/isaac_joint_states\n/sim/internal/physics_truth\n"
                else:
                    stdout = "data: '{\"state\": \"fail\"}'\n"
                return SimpleNamespace(returncode=0, stdout=stdout, stderr="")

            runner._popen = popen
            runner._command_runner = command_runner
            runner.readiness_timeout_s = 0.01
            result = runner.run()
            readiness = json.loads((result.attempt_dir / "readiness.json").read_text())
            self.assertTrue(any("contract guard observed state fail" in reason for reason in readiness["reasons"]))

    def test_missing_measured_object_truth_is_not_ready(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            runner.gate_commands = {"retention": ["/bin/true"]}
            runner.scenario_path.write_text(
                json.dumps({"schema_version": 2, "id": "qualification-retention", "seed": 7, "actors": [], "objects": [{"id": "cube"}]}),
                encoding="utf-8",
            )

            class LiveProcess:
                pid = 2_000_004
                returncode = None

                def poll(self):
                    return self.returncode

                def wait(self, **_kwargs):
                    self.returncode = 0

            def popen(command, **kwargs):
                process = LiveProcess()
                if command[0] == "humble-wrapper":
                    attempt_dir = Path(kwargs["env"]["TINKER_SIM_ATTEMPT_DIR"])
                    (attempt_dir / "scenario-runner.json").write_text(
                    json.dumps({"scenario": "qualification-retention", "seed": 7, "operations": [{"operation": "set_simulation_state", "accepted": True,
                                                     "state": 1, "boundary": "PHYSICS_READY"}]}),
                        encoding="utf-8",
                    )
                return process

            def command_runner(command, **_kwargs):
                if command[:3] == ["ros2", "topic", "list"]:
                    stdout = "/clock\n/isaac_joint_states\n/sim/internal/physics_truth\n/sim/truth/object_state\n"
                elif command[-1:] == ["/sim/status/contract"]:
                    stdout = "data: '{\"state\": \"pass\"}'\n"
                else:
                    stdout = ""
                return SimpleNamespace(returncode=0, stdout=stdout, stderr="")

            runner._popen = popen
            runner._command_runner = command_runner
            runner.readiness_timeout_s = 0.01
            result = runner.run()
            readiness = json.loads((result.attempt_dir / "readiness.json").read_text())
            self.assertIn("measured typed object truth does not match the requested object ids", readiness["reasons"])

    def test_manifest_records_exact_gate_command_and_artifact_pointer_set(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            runner.gate_commands = {"retention": ["/bin/true", "--diagnostic"]}
            pointer = root / "artifacts/robot/tinker2/current.json"
            artifact_dir = root / "artifacts/robot/tinker2/v1"
            artifact_dir.mkdir(parents=True)
            pointer.parent.mkdir(parents=True, exist_ok=True)
            (artifact_dir / "manifest.json").write_text("{}", encoding="utf-8")
            (artifact_dir / "robot.usd").write_text("usd", encoding="utf-8")
            (artifact_dir / "robot.urdf").write_text("urdf", encoding="utf-8")
            pointer.write_text(json.dumps({"manifest": str(artifact_dir / "manifest.json")}), encoding="utf-8")
            manifest = runner.prepare_manifest()
            data = json.loads(manifest.path.read_text(encoding="utf-8"))
            self.assertEqual(data["commands"]["gates"]["retention"], ["/bin/true", "--diagnostic"])
            artifact_paths = {record["path"] for record in data["artifact"]["files"]}
            self.assertTrue(any(path.endswith("current.json") for path in artifact_paths))
            self.assertTrue(any(path.endswith("robot.usd") for path in artifact_paths))
            self.assertTrue(any(path.endswith("robot.urdf") for path in artifact_paths))

    def test_truth_drain_requires_exact_ordered_embedded_raw_frames(self) -> None:
        raw = [{"frame": 1}, {"frame": 2}]
        matching = [{"frame": {"frame": 1}}, {"frame": {"frame": 2}}]
        self.assertEqual(QualificationRunner._compare_truth_records(raw, matching), (True, []))
        ok, mismatches = QualificationRunner._compare_truth_records(
            raw, [{"frame": {"frame": 2}}, {"frame": {"frame": 1}}, {"frame": {"frame": 3}}]
        )
        self.assertFalse(ok)
        self.assertIn("raw/evaluator record counts differ", mismatches)
        self.assertIn("evaluator record 1 does not exactly match raw truth", mismatches)
        ok, mismatches = QualificationRunner._compare_truth_records(raw, [{"frame": {"frame": 1}}, {}])
        self.assertFalse(ok)
        self.assertIn("evaluator record 2 has no embedded raw frame", mismatches)

    def test_scenario_readiness_binds_identity_seed_and_spawn_ids(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            runner.scenario_path.write_text(
                json.dumps({
                    "schema_version": 2,
                    "id": "qualification-retention",
                    "seed": 7,
                    "actors": [],
                    "objects": [{"id": "qualification_cube"}],
                }),
                encoding="utf-8",
            )
            manifest = runner.prepare_manifest()
            report = {
                "scenario": "wrong-scenario",
                "seed": 8,
                "operations": [{
                    "operation": "spawn_entity", "accepted": True,
                    "logical_id": "other-object",
                }],
            }
            (manifest.attempt_dir / "scenario-runner.json").write_text(json.dumps(report), encoding="utf-8")
            ready, _evidence, reason = runner._scenario_readiness(manifest)
            self.assertFalse(ready)
            self.assertIn("scenario runner identity", reason)

    def test_object_truth_rejects_unrequested_object_id(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._runner(root)
            runner.scenario_path.write_text(
                json.dumps({
                    "schema_version": 2,
                    "id": "qualification-retention",
                    "seed": 7,
                    "actors": [],
                    "objects": [{"id": "qualification_cube"}],
                }),
                encoding="utf-8",
            )
            manifest = runner.prepare_manifest()
            runner._command_runner = lambda *_args, **_kwargs: SimpleNamespace(
                returncode=0,
                stdout="data: '{\"object_id\": \"other_cube\", \"class_name\": \"box\", \"pose\": {}}'\n",
                stderr="",
            )
            ready, evidence, reason = runner._object_truth_readiness(manifest)
            self.assertFalse(ready)
            self.assertEqual(evidence["expected_object_ids"], ["qualification_cube"])
            self.assertIn("requested object ids", reason)

    def test_attempt_orphan_cleanup_terminates_process_with_attempt_environment(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            runner = self._runner(Path(temporary))
            manifest = runner.prepare_manifest()
            runner._attempt_dir = manifest.attempt_dir
            process = subprocess.Popen(
                [sys.executable, "-c", "import time; time.sleep(30)"],
                env={**os.environ, "TINKER_SIM_ATTEMPT_DIR": str(manifest.attempt_dir)},
            )
            try:
                self.assertTrue(any(item["pid"] == process.pid for item in runner._attempt_processes()))
                remaining = runner._terminate_attempt_orphans(grace_s=2.0)
                self.assertEqual(remaining, [])
                self.assertIsNotNone(process.poll())
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()

    def test_tool_version_rejects_nonzero_provenance_command(self) -> None:
        with patch("validation.manipulation_qualification.shutil.which", return_value="/bin/ros2"), patch(
            "validation.manipulation_qualification.subprocess.run",
            return_value=SimpleNamespace(returncode=1, stdout="usage", stderr=""),
        ):
            self.assertIsNone(_tool_version("ros2"))

    # -- F3.4: integrated Isaac visual-evidence environment -------------------

    @staticmethod
    def _visual_manifest(root: Path, *, scenario_id: str | None) -> QualificationManifest:
        return QualificationManifest(
            attempt_id="attempt-visual",
            attempt_dir=root / "attempts" / "visual",
            data={
                "environment": {
                    "ROS_DOMAIN_ID": 130,
                    "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
                    "TINKER_SIM_DDS_PROFILE": "local",
                },
                "scenario": ({"id": scenario_id} if scenario_id is not None else {}),
            },
        )

    def _env_runner(self, root: Path, gate: str) -> QualificationRunner:
        (root / "validation").mkdir(parents=True, exist_ok=True)
        (root / "validation/manipulation_contact_sheets.py").write_text(
            "# contact sheets", encoding="utf-8"
        )
        runner = self._runner(root)
        runner.gate = gate
        return runner

    def test_integrated_isaac_child_gets_visual_evidence_and_exact_scenario_gate(self) -> None:
        """F3.4: the integrated Isaac child enables visual evidence with the exact
        canonical scenario id as the qualification gate (never the bare ``integrated``
        gate label)."""
        from validation.integrated_qualification import QUALIFICATION_SCENARIO_NAMES

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._env_runner(root, "integrated")
            canonical = next(iter(QUALIFICATION_SCENARIO_NAMES))
            manifest = self._visual_manifest(root, scenario_id=canonical)
            environment = runner._env(manifest, "isaac")
            self.assertEqual(environment["TINKER_SIM_VISUAL_EVIDENCE"], "1")
            self.assertEqual(environment["TINKER_SIM_QUALIFICATION_GATE"], canonical)

    def test_integrated_isaac_child_rejects_noncanonical_scenario_before_launch(self) -> None:
        """F3.10: a present-but-noncanonical integrated scenario id fails closed
        before child launch using the committed canonical qualification names."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._env_runner(root, "integrated")
            manifest = self._visual_manifest(root, scenario_id="not-a-canonical-scenario")
            with self.assertRaises(ValueError) as context:
                runner._env(manifest, "isaac")
            self.assertIn("not a canonical qualification scenario", str(context.exception))

    def test_integrated_isaac_child_rejects_missing_scenario_before_launch(self) -> None:
        """F3.4: a missing/malformed integrated scenario identity fails closed
        before the Isaac child can launch."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._env_runner(root, "integrated")
            manifest = self._visual_manifest(root, scenario_id=None)
            with self.assertRaises(ValueError) as context:
                runner._env(manifest, "isaac")
            self.assertIn("exact scenario id", str(context.exception))

    def test_six_legacy_gates_keep_their_own_gate_env_unchanged(self) -> None:
        """F3.4: each of the six legacy gates keeps its own gate id as the
        qualification gate (never a scenario id, never ``integrated``)."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for index, gate in enumerate(GATES):
                gate_root = root / f"gate-{index}"
                gate_root.mkdir(parents=True, exist_ok=True)
                runner = self._env_runner(gate_root, gate)
                manifest = self._visual_manifest(gate_root, scenario_id="qualification-pick-place-positive")
                environment = runner._env(manifest, "isaac")
                self.assertEqual(environment["TINKER_SIM_QUALIFICATION_GATE"], gate)
                self.assertEqual(environment["TINKER_SIM_VISUAL_EVIDENCE"], "1")

    def test_non_isaac_integrated_roles_never_receive_a_visual_gate(self) -> None:
        """F3.4: humble and ros-tooling roles in an integrated attempt keep
        ``TINKER_SIM_VISUAL_EVIDENCE=0`` and the bare ``integrated`` gate label —
        visual evidence is only ever enabled for the integrated Isaac child."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            runner = self._env_runner(root, "integrated")
            manifest = self._visual_manifest(root, scenario_id="qualification-pick-place-positive")
            for role in ("humble", "ros-tooling"):
                environment = runner._env(manifest, role)
                self.assertEqual(environment["TINKER_SIM_VISUAL_EVIDENCE"], "0")
                self.assertEqual(environment["TINKER_SIM_QUALIFICATION_GATE"], "integrated")


if __name__ == "__main__":
    unittest.main()
