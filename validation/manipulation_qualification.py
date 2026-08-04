#!/usr/bin/env python3
"""Create and run auditable, development-only manipulation attempts.

Built-in gate executors can establish a development qualification pass only
after independent raw-evidence recomputation. External gate commands remain
diagnostics and are always unverified.
"""
from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
import platform
import re
import shlex
import shutil
import signal
import sqlite3
import subprocess
import sys
import tempfile
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence

import yaml

ROOT = Path(__file__).resolve().parents[1]
CPU_PHYSICS = {"device": "cpu", "use_gpu": False, "cuda_device": -1}
THRESHOLDS = {
    "fjt_final_max_error_rad": 0.01,
    "fjt_rms_error_rad": 0.05,
    "safety_stop_velocity_rad_s": 0.02,
    "safety_stop_frames": 5,
    "retention_lift_m": 0.10,
    "retention_translation_m": 0.20,
    "retention_hold_s": 1.0,
    "retention_drift_m": 0.02,
    "retention_drift_deg": 5.0,
    "stable_speed_m_s": 0.02,
    "free_gripper_min_travel_rad": 0.75,
    "obstructed_gripper_min_gap_rad": 0.02,
    "safety_stop_position_creep_rad": 0.005,
}
# Driver accounting can fluctuate slightly while CUDA contexts are destroyed.
# This is deliberately small: unexplained growth above this allowance is a leak.
GPU_MEMORY_TOLERANCE_MIB = 32
GPU_CLEANUP_RETRIES = 5
GPU_CLEANUP_RETRY_DELAY_S = 0.25
# DDS graph discovery can briefly report a transient-local topic as unknown
# immediately after the recorder remains alive. Keep this retry short and
# bounded; endpoint ownership and QoS are still validated on the final probe.
ROSBAG_ENDPOINT_DISCOVERY_MAX_ATTEMPTS = 3
ROSBAG_ENDPOINT_DISCOVERY_RETRY_DELAY_S = 0.10
GATES = (
    "free-space-fjt",
    "safety-stop",
    "free-gripper",
    "obstructed-gripper",
    "arm-collision",
    "retention",
)
FORBIDDEN_COMMAND_SURFACES = ("/isaac_joint_commands",)
APPROVED_RECORD_TOPICS = (
    "/clock",
    "/isaac_joint_states",
    "/isaac_joint_commands",
    "/sim/truth/robot_state",
    "/sim/truth/object_state",
    "/sim/truth/contacts",
    "/sim/truth/task_state",
    "/sim/safety/collision",
    "/sim/hardware/safety_stop",
    "/sim/status/contract",
    "/sim/status/command_gateway",
)
RAW_TRUTH_TOPIC = "/sim/internal/physics_truth"
SAFETY_STOP_TOPIC = "/sim/hardware/safety_stop"
CONTRACT_TOPIC = "/sim/status/contract"
COMMAND_GATEWAY_STATUS_TOPIC = "/sim/status/command_gateway"
TRAJECTORY_CONTROLLER = "xarm7_traj_controller"
ROSBAG_QOS_OVERRIDE_PROFILES = {
    SAFETY_STOP_TOPIC: {
        "history": "keep_last",
        "depth": 1,
        "reliability": "reliable",
        "durability": "transient_local",
    },
    CONTRACT_TOPIC: {
        "history": "keep_last",
        "depth": 1,
        "reliability": "reliable",
        "durability": "transient_local",
    },
}
LIST_CONTROLLERS_SERVICE = "/controller_manager/list_controllers"
_CONTROLLER_REPR = re.compile(
    r"\bControllerState\s*\(\s*name\s*=\s*(['\"])(?P<name>.*?)\1"
    r"\s*,\s*state\s*=\s*(['\"])(?P<state>.*?)\3",
    re.DOTALL,
)
DDS_PROFILE_VARIABLE = "TINKER_SIM_DDS_PROFILE"
FASTRTPS_PROFILE_VARIABLE = "FASTRTPS_DEFAULT_PROFILES_FILE"
SUPPORTED_DDS_PROFILES = ("local", "lan")


def _ros2_command(command: Sequence[str]) -> list[str]:
    """Apply Humble's per-command no-daemon option where supported."""
    values = list(command)
    if (
        len(values) >= 2
        and values[0] == "ros2"
        and values[1] in {"topic", "node", "param"}
        and "--no-daemon" not in values[2:]
    ):
        return [*values, "--no-daemon"]
    return values

# The Isaac wrapper performs its own internal ROS/Python setup.  Passing the
# parent shell's ROS overlay into it makes the wrapper reject the launch before
# Isaac starts, so keep the boundary deliberately small.  Prefixes are used for
# deployment-specific Tinker and Isaac settings; path-like ROS/Python state is
# never inherited through this boundary.
ISAAC_ENV_ALLOWLIST = (
    "HOME",
    "USER",
    "LOGNAME",
    "PATH",
    "ROS_DOMAIN_ID",
    "RMW_IMPLEMENTATION",
    "ACCEPT_EULA",
    "OMNI_KIT_ACCEPT_EULA",
    "XDG_RUNTIME_DIR",
    "DISPLAY",
    "WAYLAND_DISPLAY",
    "XAUTHORITY",
    "CUDA_VISIBLE_DEVICES",
    "NVIDIA_VISIBLE_DEVICES",
    "NVIDIA_DRIVER_CAPABILITIES",
    "UV_CACHE_DIR",
    "UV_PYTHON_INSTALL_DIR",
    "XDG_CACHE_HOME",
    "OV_CACHE_ROOT",
)
ISAAC_ENV_ALLOW_PREFIXES = ("TINKER_", "ISAAC_", "ISAACSIM_")
ISAAC_ENV_SCRUBBED_VARIABLES = (
    "AMENT_PREFIX_PATH",
    "CMAKE_PREFIX_PATH",
    "COLCON_CURRENT_PREFIX",
    "COLCON_PREFIX_PATH",
    "LD_LIBRARY_PATH",
    "PKG_CONFIG_PATH",
    "PYTHONHOME",
    "PYTHONPATH",
    "ROS_PACKAGE_PATH",
    "ROS_PYTHON_VERSION",
    "ROS_VERSION",
    "VIRTUAL_ENV",
)


def _file_records(root: Path, paths: Iterable[Path]) -> dict[str, Any]:
    records: list[dict[str, Any]] = []
    for candidate in sorted({path.resolve() for path in paths}, key=str):
        try:
            relative = candidate.relative_to(root.resolve()).as_posix()
        except ValueError:
            relative = str(candidate)
        if not candidate.is_file():
            records.append({"path": relative, "missing": True})
            continue
        records.append({"path": relative, "size": candidate.stat().st_size})
    return {"files": records}


def _file_tree(root: Path, relative: str, *, include_generated: bool = False) -> dict[str, Any]:
    directory = root / relative
    paths = (
        path
        for path in directory.rglob("*")
        if path.is_file()
        and not any(part in {"__pycache__", ".pytest_cache", "log"} for part in path.parts)
        and (include_generated or not any(part in {"build", "install"} for part in path.parts))
    ) if directory.is_dir() else (directory,)
    return _file_records(root, paths)


def _command_path(root: Path, command: Sequence[str]) -> Path | None:
    """Resolve a command executable when it names a local file."""
    if not command:
        return None
    token = str(command[0])
    candidate = Path(token)
    if not candidate.is_absolute():
        candidate = root / candidate if "/" in token else Path(shutil.which(token) or "")
    return candidate.resolve() if candidate.is_file() else None


def _package_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def _resolve_dds_profile(profile: str | None = None) -> str:
    value = str(profile or os.environ.get(DDS_PROFILE_VARIABLE, "local"))
    if value not in SUPPORTED_DDS_PROFILES:
        raise ValueError(f"{DDS_PROFILE_VARIABLE} must be local or lan")
    return value


def _effective_fastdds_profile_path(root: Path, profile: str) -> str | None:
    return str(root / "config/fastdds-lan.xml") if profile == "lan" else None


def _unique_path_values(values: Iterable[Path | str]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for value in values:
        rendered = str(value)
        if rendered and rendered not in seen:
            seen.add(rendered)
            result.append(rendered)
    return result


def _project_overlay_paths(root: Path) -> dict[str, list[str]]:
    """Derive the built project's isolated ROS install paths.

    This mirrors the path-bearing parts of sourcing ``ros2_ws/install`` while
    leaving the parent Humble/Tinker environment intact.  Package markers are
    used when available so dependencies are ordered before dependents, as
    colcon does for the generated setup script.
    """
    install_root = (root / "ros2_ws/install").resolve()
    packages: dict[str, tuple[Path, set[str]]] = {}
    if not install_root.is_dir():
        return {"prefixes": [], "pythonpath": [], "library_path": [], "path": []}

    for prefix in sorted((path for path in install_root.iterdir() if path.is_dir()), key=str):
        package_markers = prefix / "share/colcon-core/packages"
        marker_files = sorted((path for path in package_markers.iterdir() if path.is_file()), key=lambda path: path.name) if package_markers.is_dir() else []
        if marker_files:
            for marker in marker_files:
                dependencies = {
                    dependency for dependency in marker.read_text(encoding="utf-8").split(os.pathsep) if dependency
                }
                packages[marker.name] = (prefix, dependencies)
            continue
        package_files = sorted((path for path in (prefix / "share").glob("*/package.xml") if path.is_file()), key=str)
        for package_file in package_files:
            packages[package_file.parent.name] = (prefix, set())

    pending = set(packages)
    ordered_names: list[str] = []
    while pending:
        ready = sorted(
            name for name in pending
            if not (packages[name][1] & pending)
        )
        if not ready:
            ready = sorted(pending)
        ordered_names.extend(ready)
        pending.difference_update(ready)

    prefixes = _unique_path_values(packages[name][0] for name in ordered_names)
    pythonpath: list[Path] = []
    library_path: list[Path] = []
    executable_path: list[Path] = []
    for prefix_value in prefixes:
        prefix = Path(prefix_value)
        for relative in (
            "lib/python3.10/site-packages",
            "lib/python3.10/dist-packages",
            "local/lib/python3.10/site-packages",
            "local/lib/python3.10/dist-packages",
        ):
            candidate = prefix / relative
            if candidate.is_dir():
                pythonpath.append(candidate)
        for relative in ("lib",):
            candidate = prefix / relative
            if candidate.is_dir():
                library_path.append(candidate)
        candidate = prefix / "bin"
        if candidate.is_dir():
            executable_path.append(candidate)

    return {
        "prefixes": prefixes,
        "pythonpath": _unique_path_values(pythonpath),
        "library_path": _unique_path_values(library_path),
        "path": _unique_path_values(executable_path),
    }


def _prepend_unique_environment_paths(
    environment: dict[str, str], variable: str, values: Iterable[str]
) -> None:
    existing = environment.get(variable, "").split(os.pathsep) if environment.get(variable) else []
    environment[variable] = os.pathsep.join(_unique_path_values([*values, *existing]))


def _ros_tooling_environment(
    *,
    root: Path = ROOT,
    dds_profile: str | None = None,
    domain_id: str | None = None,
    rmw_implementation: str | None = None,
) -> dict[str, str]:
    """Return inherited ROS tooling state with the wrapper's DDS policy."""
    profile = _resolve_dds_profile(dds_profile)
    environment = os.environ.copy()
    fastdds_profile = _effective_fastdds_profile_path(root, profile)
    if fastdds_profile is None:
        environment.pop(FASTRTPS_PROFILE_VARIABLE, None)
    else:
        environment[FASTRTPS_PROFILE_VARIABLE] = fastdds_profile
    environment.update(
        {
            "ROS2CLI_NO_DAEMON": "1",
            "ROS_DOMAIN_ID": str(domain_id or os.environ.get("ROS_DOMAIN_ID", "25")),
            "RMW_IMPLEMENTATION": str(
                rmw_implementation or os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
            ),
        }
    )
    overlay = _project_overlay_paths(root)
    _prepend_unique_environment_paths(environment, "AMENT_PREFIX_PATH", overlay["prefixes"])
    _prepend_unique_environment_paths(environment, "PYTHONPATH", overlay["pythonpath"])
    _prepend_unique_environment_paths(environment, "LD_LIBRARY_PATH", overlay["library_path"])
    _prepend_unique_environment_paths(environment, "PATH", overlay["path"])
    return environment


def _tool_version(executable: str, *, env: Mapping[str, str] | None = None) -> str | None:
    resolved = shutil.which(executable, path=(env or os.environ).get("PATH"))
    if not resolved:
        return None
    try:
        completed = subprocess.run(
            [resolved, "--version"],
            capture_output=True, text=True, timeout=2, check=False,
            env=dict(env) if env is not None else None,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if completed.returncode != 0:
        return None
    output = (completed.stdout or completed.stderr).strip()
    return output.splitlines()[0] if output else None


def _runtime_versions(*, root: Path = ROOT) -> dict[str, Any]:
    """Capture local runtime/tool versions without resolving or downloading anything."""
    ros_tooling_env = _ros_tooling_environment(root=root)
    return {
        "python": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "ros_distro": ros_tooling_env.get("ROS_DISTRO"),
        "rmw_implementation": ros_tooling_env.get("RMW_IMPLEMENTATION"),
        "isaacsim": _package_version("isaacsim"),
        "isaaclab": _package_version("isaaclab"),
        "ros2": _tool_version("ros2", env=ros_tooling_env),
        "uv": _tool_version("uv"),
        "git": _tool_version("git"),
    }


def _json_payloads(text: str) -> list[Mapping[str, Any]]:
    """Extract JSON objects from ros2 echo output and wrapper diagnostics."""
    payloads: list[Mapping[str, Any]] = []
    candidates = [text.strip()]
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("data:"):
            candidates.append(stripped.split(":", 1)[1].strip())
    for candidate in candidates:
        if len(candidate) >= 2 and candidate[0] == candidate[-1] and candidate[0] in {"'", '"'}:
            candidates.append(candidate[1:-1])
        try:
            value = json.loads(candidate)
        except (TypeError, ValueError, json.JSONDecodeError):
            continue
        if isinstance(value, Mapping):
            payloads.append(value)
            data = value.get("data")
            if isinstance(data, str):
                payloads.extend(_json_payloads(data))
        elif isinstance(value, str):
            try:
                nested = json.loads(value)
            except (TypeError, ValueError, json.JSONDecodeError):
                continue
            if isinstance(nested, Mapping):
                payloads.append(nested)
    return payloads


def _json_file(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _count_valid_jsonl_records(path: Path) -> int:
    """Count nonblank lines that contain one valid JSONL record each."""
    if not path.is_file():
        return 0
    count = 0
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                json.loads(line)
            except json.JSONDecodeError:
                continue
            count += 1
    return count


def _resolve_artifact(root: Path, explicit: Path | None) -> list[Path]:
    pointer = root / "artifacts/robot/tinker2/current.json"
    candidates: list[Path] = [pointer]
    if explicit is not None:
        artifact = explicit if explicit.is_absolute() else root / explicit
        candidates.extend(
            [artifact, artifact.parent / "robot.urdf", artifact.parent / "manifest.json"]
        )
    if pointer.is_file():
        current = _json_file(pointer)
        manifest_value = Path(str(current.get("manifest", "")))
        manifest = manifest_value if manifest_value.is_absolute() else root / manifest_value
        candidates.extend([pointer, manifest, manifest.parent / "robot.usd", manifest.parent / "robot.urdf"])
    return list(dict.fromkeys(candidates))


def _command_tokens(command: Sequence[str] | str) -> list[str]:
    return shlex.split(command) if isinstance(command, str) else [str(item) for item in command]


def validate_command(command: Sequence[str] | str) -> list[str]:
    tokens = _command_tokens(command)
    rendered = " ".join(tokens)
    for forbidden in FORBIDDEN_COMMAND_SURFACES:
        if forbidden in rendered:
            raise ValueError(f"direct joint command publishing is forbidden: {forbidden}")
    return tokens


@dataclass(frozen=True)
class QualificationManifest:
    attempt_id: str
    attempt_dir: Path
    data: Mapping[str, Any]

    @property
    def path(self) -> Path:
        return self.attempt_dir / "manifest.json"


@dataclass(frozen=True)
class QualificationResult:
    attempt_dir: Path
    status: str
    gate_results: Mapping[str, Any]
    exit_codes: Mapping[str, int | None]


class QualificationRunner:
    """Runner with injectable process functions for manifest-only testing."""

    def __init__(
        self,
        root: Path | None = None,
        attempt_root: Path | None = None,
        config_path: Path | None = None,
        scenario_path: Path | None = None,
        artifact_path: Path | None = None,
        seed: int = 7,
        gate: str = "all",
        readiness_timeout_s: float = 30.0,
        bag_startup_timeout_s: float = 5.0,
        isaac_command: Sequence[str] | str | None = None,
        humble_command: Sequence[str] | str | None = None,
        gate_commands: Mapping[str, Sequence[str] | str] | None = None,
        ros_domain_id: int | str | None = None,
        popen: Callable[..., Any] = subprocess.Popen,
        command_runner: Callable[..., Any] = subprocess.run,
    ) -> None:
        self.root = (root or ROOT).resolve()
        self.attempt_root = (attempt_root or self.root / "qualification-runs").resolve()
        self.config_path = (config_path or self.root / "simulation/qualification/manipulation-core.json").resolve()
        if scenario_path is None and gate != "all" and self.config_path.is_file():
            try:
                scenarios = _json_file(self.config_path).get("scenarios", {})
                configured_scenario = (
                    scenarios.get(gate) if isinstance(scenarios, Mapping) else None
                )
            except (OSError, ValueError, json.JSONDecodeError):
                configured_scenario = None
            if isinstance(configured_scenario, str) and configured_scenario:
                scenario_name = (
                    configured_scenario
                    if configured_scenario.endswith(".json")
                    else f"{configured_scenario}.json"
                )
                scenario_path = self.root / "simulation/scenarios" / scenario_name
        self.scenario_path = (
            scenario_path
            or self.root / "simulation/scenarios/qualification-retention.json"
        ).resolve()
        if artifact_path is None:
            self.artifact_path = None
        else:
            self.artifact_path = (artifact_path if artifact_path.is_absolute() else self.root / artifact_path).resolve()
        self.seed = int(seed)
        self.gate = gate
        self.readiness_timeout_s = float(readiness_timeout_s)
        self.bag_startup_timeout_s = float(bag_startup_timeout_s)
        self.isaac_command = validate_command(isaac_command or self._default_isaac_command())
        self.humble_command = validate_command(humble_command or self._default_humble_command())
        self.gate_commands = {
            name: validate_command(command) for name, command in (gate_commands or {}).items()
        }
        self.ros_domain_id = (
            str(ros_domain_id) if ros_domain_id is not None else None
        )
        self._popen = popen
        self._command_runner = command_runner
        self._processes: dict[str, Any] = {}
        self._logs: dict[str, Any] = {}
        self._termination: dict[str, Any] = {}
        self._attempt_dir: Path | None = None
        self._orphan_cleanup: dict[str, Any] = {}
        self._owned_pids: set[int] = set()

    def _default_isaac_command(self) -> list[str]:
        scenario_id = self._scenario_id()
        return [
            str(self.root / "scripts/launch-isaac"),
            "--sensor-profile", "manipulation-core",
            "--profile", "parity",
            "--scenario", scenario_id,
            "--seed", str(self.seed),
            "--headless", "--ros", "--qualification",
        ]

    def _default_humble_command(self) -> list[str]:
        scenario_id = self._scenario_id()
        return [
            str(self.root / "scripts/launch-humble"),
            "manipulation",
            f"scenario:={scenario_id}",
            f"seed:={self.seed}",
        ]

    def _scenario_id(self) -> str:
        try:
            value = _json_file(self.scenario_path).get("id")
        except (OSError, ValueError, json.JSONDecodeError):
            value = None
        return str(value) if value else self.scenario_path.stem

    def _scenario_spec(self) -> dict[str, Any]:
        """Return the schema-level identity used to validate live evidence."""
        raw = _json_file(self.scenario_path)
        if raw.get("schema_version") != 2:
            raise ValueError("qualification scenario must use schema_version 2")
        scenario_id = raw.get("id")
        if not isinstance(scenario_id, str) or not scenario_id:
            raise ValueError("qualification scenario id must be a non-empty string")
        entities: list[dict[str, Any]] = []
        for group in ("actors", "objects"):
            records = raw.get(group, [])
            if not isinstance(records, list):
                raise ValueError(f"qualification scenario {group} must be an array")
            for record in records:
                if not isinstance(record, Mapping) or not isinstance(record.get("id"), str):
                    raise ValueError(f"qualification scenario {group} entries require an id")
                entities.append({"id": str(record["id"]), "group": group, **dict(record)})
        identifiers = [str(item["id"]) for item in entities]
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("qualification scenario entity ids must be unique")
        return {
            "id": scenario_id,
            "schema_version": 2,
            "seed": raw.get("seed"),
            "entities": entities,
            "entity_ids": identifiers,
            "object_ids": [item["id"] for item in entities if item["group"] == "objects"],
        }

    def _default_rosbag_command(self, manifest: QualificationManifest) -> list[str]:
        qos_overrides = self._write_rosbag_qos_overrides(manifest)
        return [
            "ros2", "bag", "record", "-o", str(manifest.attempt_dir / "rosbag"),
            "--qos-profile-overrides-path", str(qos_overrides),
            *APPROVED_RECORD_TOPICS,
        ]

    @staticmethod
    def _qos_override_text() -> str:
        return (
            f"{SAFETY_STOP_TOPIC}:\n"
            "  history: keep_last\n"
            "  depth: 1\n"
            "  reliability: reliable\n"
            "  durability: transient_local\n"
            f"{CONTRACT_TOPIC}:\n"
            "  history: keep_last\n"
            "  depth: 1\n"
            "  reliability: reliable\n"
            "  durability: transient_local\n"
        )

    def _write_rosbag_qos_overrides(self, manifest: QualificationManifest) -> Path:
        path = manifest.attempt_dir / "rosbag-qos-overrides.yaml"
        if not path.exists():
            path.write_text(self._qos_override_text(), encoding="utf-8")
        return path

    @staticmethod
    def _rosbag_qos_override_evidence(path: Path) -> dict[str, Any]:
        evidence: dict[str, Any] = {
            "path": str(path),
            "present": path.is_file(),
            "parsed": False,
            "profiles": {},
            "expected_profiles": dict(ROSBAG_QOS_OVERRIDE_PROFILES),
            "exact": False,
        }
        if not path.is_file():
            evidence["error"] = "generated QoS override file is missing"
            return evidence
        try:
            parsed = yaml.safe_load(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, yaml.YAMLError) as error:
            evidence["error"] = f"generated QoS override file is not valid YAML: {error}"
            return evidence
        if not isinstance(parsed, Mapping) or not all(
            isinstance(value, Mapping) for value in parsed.values()
        ):
            evidence["error"] = "generated QoS override file must map topics to QoS profiles"
            return evidence
        profiles = {str(topic): dict(profile) for topic, profile in parsed.items()}
        evidence["parsed"] = True
        evidence["profiles"] = profiles
        evidence["exact"] = profiles == ROSBAG_QOS_OVERRIDE_PROFILES
        if not evidence["exact"]:
            evidence["error"] = "QoS override profiles do not exactly match the required configuration"
        return evidence

    @staticmethod
    def _rosbag_command_override_path(command: Sequence[str]) -> str | None:
        try:
            index = list(command).index("--qos-profile-overrides-path")
        except ValueError:
            return None
        if index + 1 >= len(command):
            return None
        return str(command[index + 1])

    @staticmethod
    def _rosbag_output_evidence(path: Path) -> dict[str, Any]:
        databases = sorted(path.glob("*.db3")) if path.is_dir() else []
        evidence: dict[str, Any] = {
            "directory": str(path),
            "present": path.is_dir(),
            "databases": [str(database) for database in databases],
            "database_path": None,
            "open": False,
        }
        for database in databases:
            try:
                uri = f"file:{database}?mode=ro"
                with sqlite3.connect(uri, uri=True, timeout=0.2) as connection:
                    connection.execute("PRAGMA schema_version").fetchone()
            except (OSError, sqlite3.Error) as error:
                evidence["error"] = f"output database is not openable: {error}"
                continue
            evidence.update({"database_path": str(database), "open": True})
            break
        if not databases:
            evidence["error"] = "rosbag output database is missing"
        return evidence

    @staticmethod
    def _rosbag_path(manifest: QualificationManifest) -> Path:
        return manifest.attempt_dir / "rosbag"

    def _config(self) -> dict[str, Any]:
        return _json_file(self.config_path)

    def _selected_gates(self, config: Mapping[str, Any]) -> list[str]:
        configured = [str(item) for item in config.get("gates", GATES)]
        self._rosbag_minimum_message_counts(config, gate="all")
        if self.gate == "all":
            return configured
        if self.gate not in configured:
            raise ValueError(f"unknown gate {self.gate!r}; choose one of {configured}")
        return [self.gate]

    @staticmethod
    def _rosbag_minimum_message_counts(
        config: Mapping[str, Any], *, gate: str = "all"
    ) -> dict[str, int]:
        """Resolve the explicit, fail-closed final rosbag count policy."""
        configured = [str(item) for item in config.get("gates", GATES)]
        unknown_gates = sorted(set(configured) - set(GATES))
        if unknown_gates:
            raise ValueError(f"unknown qualification gates: {', '.join(unknown_gates)}")
        policy = config.get("rosbag_minimum_message_counts")
        if not isinstance(policy, Mapping):
            raise ValueError("rosbag_minimum_message_counts policy is missing")
        policy_gates = {str(name) for name in policy}
        configured_gates = set(configured)
        missing_gates = sorted(configured_gates - policy_gates)
        extra_gates = sorted(policy_gates - configured_gates)
        if missing_gates:
            raise ValueError(
                "rosbag minimum-count policy is missing gates: "
                + ", ".join(missing_gates)
            )
        if extra_gates:
            raise ValueError(
                "rosbag minimum-count policy has unknown gates: "
                + ", ".join(extra_gates)
            )
        selected = configured if gate == "all" else [gate]
        if gate != "all" and gate not in configured_gates:
            raise ValueError(f"unknown gate {gate!r}; no minimum-count policy is configured")
        resolved: dict[str, int] = {}
        for selected_gate in selected:
            gate_policy = policy.get(selected_gate)
            if not isinstance(gate_policy, Mapping):
                raise ValueError(
                    f"rosbag minimum-count policy for gate {selected_gate!r} is missing"
                )
            observed_topics = {str(topic) for topic in gate_policy}
            expected_topics = set(APPROVED_RECORD_TOPICS)
            missing_topics = sorted(expected_topics - observed_topics)
            extra_topics = sorted(observed_topics - expected_topics)
            if missing_topics:
                raise ValueError(
                    f"rosbag minimum-count policy for gate {selected_gate!r} "
                    f"is missing topics: {', '.join(missing_topics)}"
                )
            if extra_topics:
                raise ValueError(
                    f"rosbag minimum-count policy for gate {selected_gate!r} "
                    f"has unknown topics: {', '.join(extra_topics)}"
                )
            for topic in APPROVED_RECORD_TOPICS:
                minimum = gate_policy.get(topic)
                if isinstance(minimum, bool) or not isinstance(minimum, int) or minimum < 0:
                    raise ValueError(
                        f"rosbag minimum count for {selected_gate}/{topic} "
                        "must be a non-negative integer"
                    )
                resolved[topic] = max(resolved.get(topic, 0), minimum)
        return resolved

    def _source_inventory(self) -> dict[str, Any]:
        bridge_root = self.root / "ros2_ws/src/tinker_sim_bridge"
        overlay = self.root / "ros2_ws/install"
        fixed_inputs = [
            self.root / "validation/manipulation_qualification.py",
            self.root / "validation/manipulation_gate_executor.py",
            self.root / "validation/manipulation_gate_verifier.py",
            self.root / "validation/manipulation_contact_sheets.py",
            self.root / "validation/run_sim.py",
            self.root
            / "simulation/tinker_sim_isaac/qualification_visual_capture.py",
            self.root / "scripts/tinker-sim",
            self.root / "scripts/launch-humble",
            self.root / "scripts/launch-isaac",
            self.root / "tools/deploy.py",
            self.root / "uv.lock",
            self.root / "pyproject.toml",
            self.config_path,
            self.scenario_path,
        ]
        executed_inputs = list(fixed_inputs)
        commands = [self.isaac_command, self.humble_command, ["ros2"]]
        commands.extend(self.gate_commands.values())
        for command in commands:
            executable = _command_path(self.root, command)
            if executable is not None:
                executed_inputs.append(executable)
        return {
            "bridge": _file_tree(self.root, "ros2_ws/src/tinker_sim_bridge/tinker_sim_bridge"),
            "simulation": _file_tree(self.root, "simulation"),
            "validation": _file_records(self.root, [self.root / "validation/manipulation_qualification.py"]),
            "validation_run_sim": _file_records(self.root, [self.root / "validation/run_sim.py"]),
            "lock_and_config": _file_records(
                self.root,
                [self.root / "uv.lock", self.root / "pyproject.toml", self.config_path, self.scenario_path],
            ),
            "bridge_launch": _file_tree(self.root, "ros2_ws/src/tinker_sim_bridge/launch"),
            "bridge_config": _file_tree(self.root, "ros2_ws/src/tinker_sim_bridge/config"),
            "bridge_package": _file_records(
                self.root,
                [bridge_root / "package.xml", bridge_root / "setup.py", bridge_root / "setup.cfg"],
            ),
            "interfaces": _file_tree(self.root, "ros2_ws/src/tinker_sim_interfaces"),
            "wrappers": _file_records(
                self.root,
                [self.root / "scripts/launch-humble", self.root / "scripts/launch-isaac"],
            ),
            "installed_overlay": _file_tree(self.root, "ros2_ws/install", include_generated=True)
            if overlay.exists()
            else {"present": False, "files": []},
            "executed_inputs": _file_records(self.root, executed_inputs),
        }

    def _new_attempt_dir(self) -> tuple[str, Path]:
        self.attempt_root.mkdir(parents=True, exist_ok=True)
        for _ in range(20):
            timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
            attempt_id = f"{timestamp}-{os.getpid()}-{uuid.uuid4().hex[:10]}"
            path = self.attempt_root / attempt_id
            try:
                path.mkdir()
                return attempt_id, path
            except FileExistsError:
                continue
        raise RuntimeError("could not allocate a unique qualification attempt directory")

    def _build_manifest_data(
        self,
        *,
        attempt_id: str,
        attempt_dir: Path,
        selected_gates: Sequence[str],
        scenario: Mapping[str, Any],
        gate: str,
    ) -> dict[str, Any]:
        """Build the canonical manifest data dict (shared by the six-gate and
        integrated manifest paths).  ``selected_gates`` is the exact gate list
        for the six-gate path and empty for the integrated path, so the
        integrated manifest carries no core-gate selection semantics."""
        dds_profile = _resolve_dds_profile()
        config = self._config()
        artifact_files = _resolve_artifact(self.root, self.artifact_path)
        sources = self._source_inventory()
        return {
            "schema_version": 1,
            "attempt_id": attempt_id,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "root": str(self.root),
            "scenario": {
                "id": scenario["id"],
                "schema_version": scenario["schema_version"],
                "seed": scenario["seed"],
                "entity_ids": scenario["entity_ids"],
                "object_ids": scenario["object_ids"],
                "path": str(self.scenario_path.relative_to(self.root)),
            },
            "config": {"path": str(self.config_path.relative_to(self.root))},
            "artifact": _file_records(self.root, artifact_files),
            "sources": sources,
            "provenance": {
                "versions": _runtime_versions(root=self.root),
                "executed_input_paths": [
                    record["path"] for record in sources["executed_inputs"]["files"]
                ],
            },
            "seed": self.seed,
            "gate": gate,
            "selected_gates": [str(item) for item in selected_gates],
            "physics": dict(CPU_PHYSICS),
            "thresholds": {
                **THRESHOLDS,
                **{
                    str(name): value
                    for name, value in config.get("thresholds", {}).items()
                },
            },
            "rosbag_minimum_message_counts": {
                gate: self._rosbag_minimum_message_counts(config, gate=gate)
                for gate in selected_gates
            },
            "commands": {
                "isaac": self.isaac_command,
                "humble": self.humble_command,
                "gates": {
                    name: list(self.gate_commands[name])
                    for name in selected_gates
                    if name in self.gate_commands
                },
            },
            "environment": {
                "ROS_DOMAIN_ID": (
                    self.ros_domain_id
                    if self.ros_domain_id is not None
                    else os.environ.get("ROS_DOMAIN_ID", "25")
                ),
                "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
                DDS_PROFILE_VARIABLE: dds_profile,
                "process_policy": self._environment_policy(dds_profile=dds_profile),
            },
            "topics": {
                "physics_truth": RAW_TRUTH_TOPIC,
                "truth": ["/sim/truth/robot_state", "/sim/truth/object_state", "/sim/truth/contacts", "/sim/truth/task_state"],
                "command_gateway": "/isaac_joint_commands",
                "recorded": list(APPROVED_RECORD_TOPICS),
            },
        }

    def prepare_manifest(self) -> QualificationManifest:
        if not self.config_path.is_file():
            raise FileNotFoundError(self.config_path)
        if not self.scenario_path.is_file():
            raise FileNotFoundError(self.scenario_path)
        config = self._config()
        scenario = self._scenario_spec()
        selected_gates = self._selected_gates(config)
        attempt_id, attempt_dir = self._new_attempt_dir()
        manifest_data = self._build_manifest_data(
            attempt_id=attempt_id,
            attempt_dir=attempt_dir,
            selected_gates=selected_gates,
            scenario=scenario,
            gate=self.gate,
        )
        manifest_path = attempt_dir / "manifest.json"
        manifest_path.write_text(json.dumps(manifest_data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return QualificationManifest(attempt_id, attempt_dir, manifest_data)

    def prepare_manifest_at(
        self,
        attempt_id: str,
        attempt_dir: Path,
        *,
        scenario_id: str | None = None,
    ) -> QualificationManifest:
        """Build a manifest at an externally allocated attempt directory.

        This is the narrow additive integrated path.  It accepts an externally
        allocated (freshly created) attempt directory and records the scenario
        id with ``gate="integrated"`` and an empty ``selected_gates``, so an
        integrated scenario id is never passed into the core six-gate selection
        (integrated scenario ids are not members of the six-gate config).  The
        ``QualificationRunner`` six-gate ``--gate``/``prepare_manifest``
        behavior is unchanged.
        """
        if not self.config_path.is_file():
            raise FileNotFoundError(self.config_path)
        if not self.scenario_path.is_file():
            raise FileNotFoundError(self.scenario_path)
        scenario = self._scenario_spec()
        if scenario_id is not None and scenario["id"] != scenario_id:
            raise ValueError(
                "scenario id {!r} != requested {!r}".format(scenario["id"], scenario_id)
            )
        manifest_data = self._build_manifest_data(
            attempt_id=attempt_id,
            attempt_dir=attempt_dir,
            selected_gates=(),
            scenario=scenario,
            gate="integrated",
        )
        manifest_path = attempt_dir / "manifest.json"
        manifest_path.write_text(json.dumps(manifest_data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return QualificationManifest(attempt_id, attempt_dir, manifest_data)

    @staticmethod
    def _group_alive(process: Any) -> bool:
        pid = getattr(process, "pid", None)
        if not pid:
            return False
        try:
            os.killpg(int(pid), 0)
        except OSError:
            return False
        return True

    def _wait_for_group_exit(self, process: Any, timeout_s: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            if not self._group_alive(process):
                return True
            time.sleep(0.05)
        return not self._group_alive(process)

    def _settle_evidence_files(self, manifest: QualificationManifest) -> None:
        """Wait briefly for descendants and redirected logs to stop changing."""
        deadline = time.monotonic() + 3.0
        stable_for = 0.15
        previous: tuple[tuple[str, int, int], ...] | None = None
        stable_since: float | None = None
        while time.monotonic() < deadline:
            snapshot: list[tuple[str, int, int]] = []
            for path in sorted(manifest.attempt_dir.rglob("*")):
                if not path.is_file():
                    continue
                try:
                    stat = path.stat()
                except OSError:
                    continue
                snapshot.append((path.relative_to(manifest.attempt_dir).as_posix(), stat.st_size, stat.st_mtime_ns))
            current = tuple(snapshot)
            now = time.monotonic()
            if current == previous:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= stable_for:
                    return
            else:
                previous = current
                stable_since = now
            time.sleep(0.05)

    @staticmethod
    def _isaac_key_allowed(key: str) -> bool:
        return key in ISAAC_ENV_ALLOWLIST or key.startswith(ISAAC_ENV_ALLOW_PREFIXES)

    def _environment_policy(self, *, dds_profile: str | None = None) -> dict[str, Any]:
        observed = set(os.environ)
        ros_domain_id = os.environ.get("ROS_DOMAIN_ID", "25")
        rmw_implementation = os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
        profile = _resolve_dds_profile(dds_profile)
        fastdds_profile = _effective_fastdds_profile_path(self.root, profile)
        overlay = _project_overlay_paths(self.root)
        return {
            "isaac": {
                "mode": "scrubbed-allowlist",
                "allowlist": list(ISAAC_ENV_ALLOWLIST),
                "prefix_allowlist": list(ISAAC_ENV_ALLOW_PREFIXES),
                "scrubbed_variables": sorted(observed & set(ISAAC_ENV_SCRUBBED_VARIABLES)),
            },
            "humble": {"mode": "inherit-parent", "ROS2CLI_NO_DAEMON": "1"},
            "ros-tooling": {
                "mode": "inherit-parent",
                "ROS2CLI_NO_DAEMON": "1",
                "ROS_DOMAIN_ID": ros_domain_id,
                "RMW_IMPLEMENTATION": rmw_implementation,
                DDS_PROFILE_VARIABLE: profile,
                FASTRTPS_PROFILE_VARIABLE: fastdds_profile,
                "inherits_system_ros_paths": True,
                "project_overlay": {
                    "install_root": str((self.root / "ros2_ws/install").resolve()),
                    "prefixes": overlay["prefixes"],
                    "path_variables": {
                        "AMENT_PREFIX_PATH": overlay["prefixes"],
                        "PYTHONPATH": overlay["pythonpath"],
                        "LD_LIBRARY_PATH": overlay["library_path"],
                        "PATH": overlay["path"],
                    },
                },
            },
        }

    def _env(self, manifest: QualificationManifest, role: str = "humble") -> dict[str, str]:
        if role == "isaac":
            environment = {
                key: value
                for key, value in os.environ.items()
                if self._isaac_key_allowed(key)
            }
        elif role == "humble":
            environment = os.environ.copy()
            # The qualification owns its ROS graph and must never create or
            # reuse a ros2 CLI daemon outside the attempt.
            environment["ROS2CLI_NO_DAEMON"] = "1"
        elif role == "ros-tooling":
            configured = manifest.data["environment"]
            environment = _ros_tooling_environment(
                root=self.root,
                dds_profile=str(configured[DDS_PROFILE_VARIABLE]),
                domain_id=str(configured["ROS_DOMAIN_ID"]),
                rmw_implementation=str(configured["RMW_IMPLEMENTATION"]),
            )
        else:
            raise ValueError(f"unknown qualification process role: {role!r}")
        environment.update(
            {
                "ROS_DOMAIN_ID": str(
                    manifest.data["environment"]["ROS_DOMAIN_ID"]
                ),
                "RMW_IMPLEMENTATION": str(
                    manifest.data["environment"]["RMW_IMPLEMENTATION"]
                ),
                "TINKER_SIM_ROOT": str(self.root),
                "TINKER_SIM_ATTEMPT_DIR": str(manifest.attempt_dir),
                "TINKER_SIM_TRUTH_JSONL": str(manifest.attempt_dir / "physics_truth.jsonl"),
                "TINKER_SIM_EVALUATOR_JSONL": str(manifest.attempt_dir / "evaluator.jsonl"),
                "TINKER_SIM_ROSBAG_DIR": str(manifest.attempt_dir / "rosbag"),
                "TINKER_SIM_PHYSICS_DEVICE": "cpu",
                "TINKER_SIM_QUALIFICATION_GATE": self.gate,
                "TINKER_SIM_VISUAL_EVIDENCE": (
                    "1"
                    if self.gate in GATES
                    and (self.root / "validation/manipulation_contact_sheets.py").is_file()
                    else "0"
                ),
                "ISAACSIM_HEADLESS": "1",
            }
        )
        return environment

    def _capture(
        self, name: str, command: Sequence[str], directory: Path, manifest: QualificationManifest
    ) -> dict[str, Any]:
        command = _ros2_command(command)
        try:
            completed = self._command_runner(
                list(command), cwd=self.root, env=self._env(manifest, "ros-tooling"),
                text=True, capture_output=True, timeout=10, check=False
            )
            result = {
                "command": list(command),
                "returncode": completed.returncode,
                "stdout": completed.stdout,
                "stderr": completed.stderr,
            }
        except (OSError, subprocess.SubprocessError) as error:
            result = {"command": list(command), "returncode": None, "error": str(error)}
        (directory / f"{name}.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return result

    def _observe(self, command: Sequence[str], manifest: QualificationManifest, *, timeout: float = 5.0) -> dict[str, Any]:
        """Run a ROS observation and retain the complete subprocess result."""
        command = _ros2_command(command)
        try:
            completed = self._command_runner(
                list(command), cwd=self.root, env=self._env(manifest, "ros-tooling"),
                text=True, capture_output=True, timeout=timeout, check=False,
            )
            return {
                "command": list(command),
                "returncode": getattr(completed, "returncode", None),
                "stdout": str(getattr(completed, "stdout", "") or ""),
                "stderr": str(getattr(completed, "stderr", "") or ""),
            }
        except (OSError, subprocess.SubprocessError) as error:
            return {"command": list(command), "returncode": None, "stdout": "", "stderr": "", "error": str(error)}

    def _snapshot_graph(self, manifest: QualificationManifest, suffix: str) -> None:
        self._capture(f"graph-nodes-{suffix}", ["ros2", "node", "list"], manifest.attempt_dir, manifest)
        self._capture(f"graph-topics-{suffix}", ["ros2", "topic", "list", "-t"], manifest.attempt_dir, manifest)
        self._capture(f"graph-command-ownership-{suffix}", ["ros2", "topic", "info", "/isaac_joint_commands", "-v"], manifest.attempt_dir, manifest)
        self._capture(
            f"graph-raw-truth-ownership-{suffix}",
            ["ros2", "topic", "info", RAW_TRUTH_TOPIC, "-v"],
            manifest.attempt_dir,
            manifest,
        )

    def _process_failures(self) -> list[str]:
        failures: list[str] = []
        for name in ("isaac", "humble"):
            process = self._processes.get(name)
            if process is None:
                failures.append(f"{name} process was not started")
                continue
            try:
                returncode = process.poll()
            except Exception as error:  # pragma: no cover - defensive fake/process boundary
                failures.append(f"{name} process liveness check failed: {error}")
                continue
            if returncode is not None:
                failures.append(f"{name} process exited before readiness (returncode={returncode})")
        return failures

    def _scenario_readiness(self, manifest: QualificationManifest) -> tuple[bool, dict[str, Any], str | None]:
        path = manifest.attempt_dir / "scenario-runner.json"
        if not path.is_file():
            return False, {"path": str(path), "present": False}, "scenario-runner.json is missing"
        try:
            report = _json_file(path)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            return False, {"path": str(path), "present": True, "error": str(error)}, "scenario-runner.json is invalid"
        try:
            expected = self._scenario_spec()
        except (OSError, ValueError, json.JSONDecodeError) as error:
            return False, {"path": str(path), "present": True, "error": str(error)}, "qualification scenario schema is invalid"
        expected_seed = self.seed
        # Schema-tolerant identity: the legacy scenario-runner report carries a
        # top-level ``scenario`` string + ``seed``; the integrated overlay
        # canonical report carries ``scenario`` as {id, seed, declaration} with
        # ``seed`` nested inside.  Both must agree with the requested identity.
        scenario_value = report.get("scenario")
        observed_scenario = scenario_value
        observed_seed = report.get("seed")
        if isinstance(scenario_value, Mapping):
            observed_scenario = scenario_value.get("id")
            observed_seed = scenario_value.get("seed", observed_seed)
        identity = {
            "expected_scenario": expected["id"],
            "observed_scenario": observed_scenario,
            "expected_seed": expected_seed,
            "observed_seed": observed_seed,
            "expected_entity_ids": expected["entity_ids"],
        }
        if observed_scenario != expected["id"] or observed_seed != expected_seed:
            return False, {"path": str(path), "present": True, "identity": identity, "report": report}, "scenario runner identity does not match requested scenario/seed"
        operations = report.get("operations")
        if (
            report.get("error")
            or report.get("success") is False
            or report.get("status") in {"failed", "error"}
            or not isinstance(operations, list)
            or not operations
        ):
            return False, {"path": str(path), "present": True, "report": report}, "scenario runner did not report successful operations"
        if any(not isinstance(operation, Mapping) or operation.get("accepted") is not True for operation in operations):
            return False, {"path": str(path), "present": True, "report": report}, "scenario runner contains a rejected operation"
        spawned_ids = [
            str(operation.get("logical_id"))
            for operation in operations
            if isinstance(operation, Mapping) and operation.get("operation") == "spawn_entity"
        ]
        identity["observed_entity_ids"] = spawned_ids
        if spawned_ids != expected["entity_ids"]:
            return False, {"path": str(path), "present": True, "identity": identity, "report": report}, "scenario runner spawn ids do not match the requested scenario"
        final = operations[-1]
        playing = final.get("state") in (1, "1", "PLAYING", "playing")
        physics_ready = final.get("boundary") == "PHYSICS_READY"
        successful = final.get("operation") == "set_simulation_state" and playing and physics_ready
        if not successful:
            return (
                False,
                {"path": str(path), "present": True, "identity": identity, "final_operation": final},
                "scenario runner final operation is not accepted PHYSICS_READY/PLAYING",
            )
        return True, {"path": str(path), "present": True, "identity": identity, "final_operation": final}, None

    def _contract_readiness(self, manifest: QualificationManifest) -> tuple[bool, dict[str, Any], str | None]:
        command = [
            "ros2", "topic", "echo", "--once", "--qos-durability", "transient_local",
            "--qos-reliability", "reliable", "--qos-depth", "1", CONTRACT_TOPIC,
        ]
        observed = self._observe(command, manifest)
        stdout = observed["stdout"]
        payloads = _json_payloads(stdout)
        state = next((str(payload.get("state")) for payload in payloads if payload.get("state") is not None), None)
        if state is None:
            match = re.search(r"(?:[\"']?state[\"']?\s*:\s*[\"']?)(pass|fail|starting)", stdout, re.IGNORECASE)
            state = match.group(1).lower() if match else None
        observed["state"] = state
        if observed.get("returncode", 0) != 0 or state != "pass":
            return False, observed, f"contract guard observed state {state or 'missing'}, expected pass"
        return True, observed, None

    def _safety_readiness(self, manifest: QualificationManifest) -> tuple[bool, dict[str, Any], str | None]:
        command = [
            "ros2", "topic", "echo", "--once", "--qos-durability", "transient_local",
            "--qos-reliability", "reliable", "--qos-depth", "1", SAFETY_STOP_TOPIC,
        ]
        observed = self._observe(command, manifest)
        stdout = observed["stdout"]
        value: bool | None = None
        payloads = _json_payloads(stdout)
        for payload in payloads:
            candidate = payload.get("data", payload.get("value"))
            if isinstance(candidate, bool):
                value = candidate
                break
        if value is None:
            match = re.search(r"(?:^|\n)\s*(?:data|value)\s*:\s*(true|false)\s*$", stdout, re.IGNORECASE | re.MULTILINE)
            if match:
                value = match.group(1).lower() == "true"
        observed["value"] = value
        observed["qos_required"] = {"durability": "transient_local", "reliability": "reliable", "depth": 1}
        if observed.get("returncode", 0) != 0 or value is not False:
            return False, observed, f"safety stop observed {value!r}, expected current false"
        return True, observed, None

    @staticmethod
    def _controller_records(stdout: str) -> list[dict[str, str]]:
        records: list[dict[str, str]] = []
        for payload in _json_payloads(stdout):
            values = payload.get("controller", payload.get("controllers", []))
            if isinstance(values, Mapping):
                values = [values]
            if isinstance(values, list):
                for value in values:
                    if isinstance(value, Mapping) and value.get("name") is not None:
                        records.append({"name": str(value.get("name")), "state": str(value.get("state", ""))})
        for match in _CONTROLLER_REPR.finditer(stdout):
            records.append(
                {
                    "name": match.group("name"),
                    "state": match.group("state"),
                }
            )
        current: dict[str, str] | None = None
        for line in stdout.splitlines():
            name_match = re.search(r"(?:^|\s)-?\s*name\s*:\s*([^#\s]+)", line)
            if name_match:
                current = {"name": name_match.group(1).strip("'\""), "state": ""}
                records.append(current)
                continue
            state_match = re.search(r"(?:^|\s)state\s*:\s*([^#\s]+)", line)
            if state_match and current is not None:
                current["state"] = state_match.group(1).strip("'\"")
        unique: dict[tuple[str, str], dict[str, str]] = {}
        for record in records:
            unique[(record["name"], record["state"])] = record
        return list(unique.values())

    def _controller_readiness(self, manifest: QualificationManifest) -> tuple[bool, dict[str, Any], str | None]:
        command = [
            "ros2", "service", "call", LIST_CONTROLLERS_SERVICE,
            "controller_manager_msgs/srv/ListControllers", "{}",
        ]
        observed = self._observe(command, manifest)
        controllers = self._controller_records(observed["stdout"])
        observed["controllers"] = controllers
        active = any(
            item["name"] == TRAJECTORY_CONTROLLER and item["state"].lower() == "active"
            for item in controllers
        )
        observed["required_controller"] = TRAJECTORY_CONTROLLER
        observed["required_state"] = "active"
        if observed.get("returncode", 0) != 0 or not active:
            return False, observed, f"{TRAJECTORY_CONTROLLER} is not active"
        return True, observed, None

    def _object_truth_readiness(self, manifest: QualificationManifest) -> tuple[bool, dict[str, Any], str | None]:
        command = _ros2_command(["ros2", "topic", "echo", "--once", "/sim/truth/object_state"])
        try:
            completed = self._command_runner(
                command, cwd=self.root, env=self._env(manifest, "ros-tooling"),
                text=True, capture_output=True, timeout=5, check=False
            )
        except (OSError, subprocess.SubprocessError) as error:
            return False, {"command": command, "error": str(error)}, "measured typed object truth could not be observed"
        stdout = str(getattr(completed, "stdout", "") or "")
        payloads = _json_payloads(stdout)
        expected = self._scenario_spec()
        expected_ids = set(str(item) for item in expected["object_ids"])
        observed_ids = {
            str(payload.get("object_id", payload.get("id", ""))).strip()
            for payload in payloads
            if str(payload.get("object_id", payload.get("id", ""))).strip()
        }
        object_id_values = [
            value.strip().strip("'\"")
            for match in re.finditer(r"^\s*object_id\s*:\s*([^#\n]+)", stdout, re.MULTILINE)
            if (value := match.group(1).strip())
        ]
        observed_ids.update(value for value in object_id_values if value)
        typed = any(
            str(payload.get("object_id", payload.get("id", ""))).strip() in expected_ids
            and str(payload.get("class_name", "")).strip()
            and isinstance(payload.get("pose"), Mapping)
            for payload in payloads
        )
        object_id = re.search(r"^\s*object_id\s*:\s*([^#\n]+)", stdout, re.MULTILINE)
        class_name = re.search(r"^\s*class_name\s*:\s*([^#\n]+)", stdout, re.MULTILINE)
        pose = re.search(r"^\s*pose\s*:", stdout, re.MULTILINE)
        if object_id and class_name and pose:
            typed = all(value.strip().strip("'\"") for value in (object_id.group(1), class_name.group(1)))
        observed = {
            "command": command,
            "returncode": getattr(completed, "returncode", None),
            "typed_object_sample": typed,
            "expected_object_ids": sorted(expected_ids),
            "observed_object_ids": sorted(observed_ids),
        }
        if object_id and object_id.group(1).strip().strip("'\"") not in expected_ids:
            typed = False
            observed["typed_object_sample"] = False
        unexpected_ids = observed_ids - expected_ids
        missing_ids = expected_ids - observed_ids
        observed["missing_object_ids"] = sorted(missing_ids)
        observed["unexpected_object_ids"] = sorted(unexpected_ids)
        if getattr(completed, "returncode", 0) != 0 or not typed or missing_ids or unexpected_ids:
            return False, observed, "measured typed object truth does not match the requested object ids"
        return True, observed, None

    def _write_readiness(self, manifest: QualificationManifest, evidence: Mapping[str, Any]) -> None:
        (manifest.attempt_dir / "readiness.json").write_text(
            json.dumps(dict(evidence), indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    def _ready(self, manifest: QualificationManifest) -> bool:
        deadline = time.monotonic() + self.readiness_timeout_s
        expected = {
            "/clock", "/isaac_joint_states", "/sim/internal/physics_truth",
            SAFETY_STOP_TOPIC, CONTRACT_TOPIC,
        }
        object_required = bool(_json_file(self.scenario_path).get("objects", []))
        if object_required:
            expected.add("/sim/truth/object_state")
        evidence: dict[str, Any] = {
            "ready": False,
            "expected_topics": sorted(expected),
            "object_required": object_required,
            "reasons": [],
            "checks": {},
        }
        while time.monotonic() < deadline:
            reasons = self._process_failures()
            checks: dict[str, Any] = {
                "processes": {"isaac": "alive", "humble": "alive"},
            }
            if reasons:
                evidence.update({"reasons": reasons, "checks": checks})
                self._write_readiness(manifest, evidence)
                return False
            try:
                completed = self._command_runner(
                    _ros2_command(["ros2", "topic", "list"]), cwd=self.root, text=True,
                    env=self._env(manifest, "ros-tooling"), capture_output=True, timeout=5, check=False
                )
                last = str(getattr(completed, "stdout", "") or "")
                topics = set(last.split())
                missing_topics = sorted(expected - topics)
                checks["topics"] = {"missing": missing_topics, "observed": sorted(expected & topics)}
            except (OSError, subprocess.SubprocessError) as error:
                last = ""
                missing_topics = sorted(expected)
                checks["topics"] = {"missing": missing_topics, "error": str(error)}
            scenario_ok, scenario_evidence, scenario_reason = self._scenario_readiness(manifest)
            checks["scenario_runner"] = scenario_evidence
            contract_ok, contract_evidence, contract_reason = self._contract_readiness(manifest)
            checks["contract_guard"] = contract_evidence
            safety_ok, safety_evidence, safety_reason = self._safety_readiness(manifest)
            checks["safety_stop"] = safety_evidence
            controller_ok, controller_evidence, controller_reason = self._controller_readiness(manifest)
            checks["controller"] = controller_evidence
            object_ok = True
            object_evidence: dict[str, Any] = {"required": False}
            object_reason = None
            if object_required:
                object_ok, object_evidence, object_reason = self._object_truth_readiness(manifest)
                object_evidence["required"] = True
            checks["object_truth"] = object_evidence
            reasons = []
            if missing_topics:
                reasons.append(f"required ROS topics missing: {', '.join(missing_topics)}")
            if scenario_reason:
                reasons.append(scenario_reason)
            if contract_reason:
                reasons.append(contract_reason)
            if safety_reason:
                reasons.append(safety_reason)
            if controller_reason:
                reasons.append(controller_reason)
            if object_reason:
                reasons.append(object_reason)
            evidence.update({"checks": checks, "reasons": reasons})
            if not reasons and scenario_ok and contract_ok and safety_ok and controller_ok and object_ok:
                evidence["ready"] = True
                evidence["reasons"] = []
                self._write_readiness(manifest, evidence)
                return True
            time.sleep(0.25)
        if not evidence["reasons"]:
            evidence["reasons"] = ["readiness timeout before all prerequisites were observed"]
        self._write_readiness(manifest, evidence)
        return False

    def _write_pre_gate_baseline(self, manifest: QualificationManifest) -> bool:
        safety_ok, safety, safety_reason = self._safety_readiness(manifest)
        contract_ok, contract, contract_reason = self._contract_readiness(manifest)
        controller_ok, controller, controller_reason = self._controller_readiness(manifest)
        evidence: dict[str, Any] = {
            "status": "ready" if safety_ok and contract_ok and controller_ok else "failed",
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "safety_stop": safety,
            "contract": contract,
            "controller": controller,
            "reasons": [reason for reason in (safety_reason, contract_reason, controller_reason) if reason],
        }
        (manifest.attempt_dir / "pre-gate-baseline.json").write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return evidence["status"] == "ready"

    def _write_gate_window(self, name: str, manifest: QualificationManifest) -> None:
        """Record the append-only evidence boundary immediately before a gate."""
        window = {
            "schema_version": 1,
            "gate": name,
            "attempt_id": manifest.attempt_id,
            "raw_start_index": _count_valid_jsonl_records(
                manifest.attempt_dir / "physics_truth.jsonl"
            ),
            "evaluator_start_index": _count_valid_jsonl_records(
                manifest.attempt_dir / "evaluator.jsonl"
            ),
            "wall_timestamp": datetime.now(timezone.utc).isoformat(),
        }
        _write_json_atomic(manifest.attempt_dir / "gate-window.json", window)

    def _start(self, name: str, command: Sequence[str], manifest: QualificationManifest) -> None:
        self._attempt_dir = manifest.attempt_dir
        command = _ros2_command(command)
        log_path = manifest.attempt_dir / f"{name}.log"
        stream = log_path.open("w", encoding="utf-8")
        self._logs[name] = stream
        role = {
            "isaac": "isaac",
            "humble": "humble",
            "rosbag": "ros-tooling",
        }.get(name)
        if role is None:
            stream.close()
            self._logs.pop(name, None)
            raise ValueError(f"unknown qualification process name: {name!r}")
        self._processes[name] = self._popen(
            list(command), cwd=self.root, env=self._env(manifest, role), stdout=stream,
            stderr=subprocess.STDOUT, text=True, start_new_session=True,
        )
        pid = getattr(self._processes[name], "pid", None)
        if pid is not None:
            self._owned_pids.add(int(pid))

    def _gpu_processes(self) -> dict[str, Any]:
        executable = shutil.which("nvidia-smi")
        if not executable:
            return {
                "available": False,
                "gpus": [],
                "processes": [],
                "error": "nvidia-smi not found",
            }

        def run(arguments: Sequence[str]) -> subprocess.CompletedProcess[str]:
            return self._command_runner(
                [executable, *arguments],
                text=True,
                capture_output=True,
                timeout=5.0,
                check=False,
            )

        errors: list[str] = []
        try:
            gpu_query = run(
                [
                    "--query-gpu=index,uuid,memory.used",
                    "--format=csv,noheader,nounits",
                ]
            )
            compute_query = run(
                [
                    "--query-compute-apps=gpu_uuid,pid,process_name,used_gpu_memory",
                    "--format=csv,noheader,nounits",
                ]
            )
            pmon_query = run(["pmon", "-c", "1", "-s", "um"])
        except (OSError, subprocess.SubprocessError) as error:
            return {
                "available": False,
                "gpus": [],
                "processes": [],
                "error": str(error),
            }

        if gpu_query.returncode != 0:
            errors.append(f"gpu query failed with return code {gpu_query.returncode}")
        if compute_query.returncode != 0:
            errors.append(
                f"compute process query failed with return code {compute_query.returncode}"
            )
        if pmon_query.returncode != 0:
            errors.append(f"pmon query failed with return code {pmon_query.returncode}")

        gpus: list[dict[str, Any]] = []
        if not errors:
            for line in gpu_query.stdout.splitlines():
                if not line.strip():
                    continue
                fields = [field.strip() for field in line.split(",", 2)]
                if len(fields) != 3:
                    errors.append(f"malformed GPU snapshot row: {line!r}")
                    continue
                try:
                    gpus.append(
                        {
                            "index": int(fields[0]),
                            "uuid": fields[1],
                            "memory_used_mib": int(fields[2]),
                        }
                    )
                except (TypeError, ValueError):
                    errors.append(f"malformed GPU snapshot row: {line!r}")

        processes_by_pid: dict[int, dict[str, Any]] = {}
        if not errors:
            uuid_to_index = {record["uuid"]: record["index"] for record in gpus}
            for line in compute_query.stdout.splitlines():
                if not line.strip():
                    continue
                fields = [field.strip() for field in line.split(",", 3)]
                if len(fields) != 4:
                    errors.append(f"malformed compute process row: {line!r}")
                    continue
                try:
                    gpu_uuid, pid_text, process_name, memory_text = fields
                    pid = int(pid_text)
                    record = {
                        "pid": pid,
                        "process_name": process_name,
                        "used_gpu_memory_mib": int(memory_text),
                        "process_type": "compute",
                        "gpu_uuid": gpu_uuid,
                    }
                    if gpu_uuid in uuid_to_index:
                        record["gpu_index"] = uuid_to_index[gpu_uuid]
                    processes_by_pid[pid] = record
                except (TypeError, ValueError):
                    errors.append(f"malformed compute process row: {line!r}")

            # pmon reports both graphics (G) and compute (C) clients, while the
            # compute query supplies the more useful per-process memory value.
            for line in pmon_query.stdout.splitlines():
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue
                fields = stripped.split()
                if len(fields) < 3:
                    errors.append(f"malformed pmon row: {line!r}")
                    continue
                try:
                    gpu_index = int(fields[0])
                    pid = int(fields[1])
                except ValueError:
                    # pmon emits '-' for an idle row; it is not a process.
                    if fields[1] == "-":
                        continue
                    errors.append(f"malformed pmon row: {line!r}")
                    continue
                process_type = fields[2]
                if process_type not in {"C", "G", "C+G"}:
                    errors.append(f"unknown pmon process type: {line!r}")
                    continue
                process_name = fields[-1] if len(fields) > 3 else "unknown"
                framebuffer_memory = 0
                if len(fields) >= 12 and fields[9] != "-":
                    try:
                        framebuffer_memory = int(fields[9])
                    except ValueError:
                        errors.append(f"malformed pmon framebuffer memory: {line!r}")
                        continue
                record = processes_by_pid.setdefault(
                    pid,
                    {
                        "pid": pid,
                        "process_name": process_name,
                        "used_gpu_memory_mib": framebuffer_memory,
                    },
                )
                record.update({"gpu_index": gpu_index, "process_type": process_type})

        available = not errors and bool(gpus)
        return {
            "available": available,
            "gpus": gpus if available else [],
            "processes": sorted(processes_by_pid.values(), key=lambda record: record["pid"])
            if available
            else [],
            "errors": errors,
            "stderr": "; ".join(
                value.strip()
                for value in (
                    str(getattr(gpu_query, "stderr", "") or ""),
                    str(getattr(compute_query, "stderr", "") or ""),
                    str(getattr(pmon_query, "stderr", "") or ""),
                )
                if value.strip()
            ),
        }

    def _write_resource_evidence(
        self,
        manifest: QualificationManifest,
        baseline: Mapping[str, Any],
    ) -> bool:
        owned = sorted(self._owned_pids)
        attempts: list[dict[str, Any]] = []
        final: dict[str, Any] = {
            "available": False,
            "gpus": [],
            "processes": [],
            "errors": ["no cleanup snapshot captured"],
        }
        memory_leaks: list[dict[str, Any]] = []
        leaked: list[dict[str, Any]] = []

        baseline_gpus = {
            str(record.get("uuid") or f"index:{record.get('index')}"): record
            for record in baseline.get("gpus", [])
        }

        def assess(snapshot: Mapping[str, Any]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
            survivors = [
                record
                for record in snapshot.get("processes", [])
                if int(record.get("pid", -1)) in self._owned_pids
            ]
            leaks: list[dict[str, Any]] = []
            if snapshot.get("available"):
                final_gpus = {
                    str(record.get("uuid") or f"index:{record.get('index')}"): record
                    for record in snapshot.get("gpus", [])
                }
                for key, record in final_gpus.items():
                    baseline_record = baseline_gpus.get(key)
                    baseline_memory = int((baseline_record or {}).get("memory_used_mib", 0))
                    final_memory = int(record.get("memory_used_mib", 0))
                    if final_memory > baseline_memory + GPU_MEMORY_TOLERANCE_MIB:
                        leaks.append(
                            {
                                "gpu": key,
                                "baseline_memory_used_mib": baseline_memory,
                                "final_memory_used_mib": final_memory,
                                "unexplained_growth_mib": final_memory - baseline_memory,
                            }
                        )
                if set(final_gpus) != set(baseline_gpus):
                    leaks.append(
                        {
                            "gpu": "topology",
                            "baseline_gpu_keys": sorted(baseline_gpus),
                            "final_gpu_keys": sorted(final_gpus),
                        }
                    )
            return survivors, leaks

        for attempt in range(1, GPU_CLEANUP_RETRIES + 1):
            final = self._gpu_processes()
            leaked, memory_leaks = assess(final)
            attempts.append(
                {
                    "attempt": attempt,
                    "available": bool(final.get("available")),
                    "owned_gpu_survivors": leaked,
                    "unexplained_gpu_memory": memory_leaks,
                }
            )
            if final.get("available") and not leaked and not memory_leaks:
                break
            if attempt < GPU_CLEANUP_RETRIES:
                time.sleep(GPU_CLEANUP_RETRY_DELAY_S)

        clean = bool(final.get("available")) and not leaked and not memory_leaks
        evidence = {
            "schema_version": 2,
            "baseline": dict(baseline),
            "final": final,
            "attempt_owned_pids": owned,
            "attempt_owned_gpu_survivors": leaked,
            "unexplained_gpu_memory": memory_leaks,
            "memory_tolerance_mib": GPU_MEMORY_TOLERANCE_MIB,
            "settle_attempts": attempts,
            "clean": clean,
            "captured_at": datetime.now(timezone.utc).isoformat(),
        }
        (manifest.attempt_dir / "resource-cleanup.json").write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return bool(evidence["clean"])

    def _write_rosbag_readiness(
        self, manifest: QualificationManifest, evidence: Mapping[str, Any]
    ) -> None:
        (manifest.attempt_dir / "rosbag-readiness.json").write_text(
            json.dumps(dict(evidence), indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    @staticmethod
    def _rosbag_log_evidence(path: Path) -> dict[str, Any]:
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError as error:
            return {
                "path": str(path),
                "present": False,
                "subscribed_topics": [],
                "all_requested_topics_subscribed": False,
                "error": str(error),
            }
        topics = sorted(
            set(
                match.group(1).strip()
                for match in re.finditer(r"Subscribed to topic\s+['\"]([^'\"]+)['\"]", text)
            )
        )
        expected_topics = set(APPROVED_RECORD_TOPICS)
        observed_topics = set(topics)
        missing_topics = sorted(expected_topics - observed_topics)
        extra_topics = sorted(observed_topics - expected_topics)
        all_requested_topics_subscribed = bool(
            re.search(r"All requested topics are subscribed\.", text, re.IGNORECASE)
        )
        return {
            "path": str(path),
            "present": True,
            "subscribed_topics": topics,
            "missing_topics": missing_topics,
            "extra_topics": extra_topics,
            "topic_set_exact": not missing_topics and not extra_topics,
            "all_requested_topics_subscribed": all_requested_topics_subscribed,
            "startup_contract": (
                all_requested_topics_subscribed
                and not missing_topics
                and not extra_topics
            ),
        }

    @staticmethod
    def _rosbag_endpoint(stdout: str) -> dict[str, Any]:
        blocks = re.split(r"(?=\bNode name\s*:)", stdout, flags=re.IGNORECASE)
        subscriptions = [
            block for block in blocks
            if re.search(r"\bEndpoint type\s*:\s*SUBSCRIPTION\b", block, re.IGNORECASE)
        ]
        owners = [
            match.group(1).strip()
            for block in subscriptions
            if (match := re.search(r"\bNode name\s*:\s*([^\s]+)", block, re.IGNORECASE))
        ]
        recorder_blocks = [
            block
            for block in subscriptions
            if (
                node_match := re.search(r"\bNode name\s*:\s*([^\s]+)", block, re.IGNORECASE)
            )
            and node_match.group(1).strip() == "rosbag2_recorder"
        ]
        subscription_qos_profiles = []
        recorder_qos_profiles = []
        for block in subscriptions:
            history_match = re.search(
                r"\bHistory\s*(?:\(Depth\))?\s*:\s*([^\s]+)", block, re.IGNORECASE
            )
            depth_match = re.search(r"\bDepth\s*:\s*(\d+)", block, re.IGNORECASE)
            profile = {
                "reliability": (re.search(r"\bReliability\s*:\s*([^\s]+)", block, re.IGNORECASE) or [None, None])[1],
                "durability": (re.search(r"\bDurability\s*:\s*([^\s]+)", block, re.IGNORECASE) or [None, None])[1],
                "history": history_match.group(1) if history_match else "UNKNOWN",
                "depth": depth_match.group(1) if depth_match else "UNKNOWN",
            }
            subscription_qos_profiles.append(profile)
            node_match = re.search(r"\bNode name\s*:\s*([^\s]+)", block, re.IGNORECASE)
            if node_match and node_match.group(1).strip() == "rosbag2_recorder":
                recorder_qos_profiles.append(profile)
        return {
            "subscription_endpoint_count": len(subscriptions),
            "owners": owners,
            "recorder_endpoint_count": len(recorder_blocks),
            "qos_profiles": recorder_qos_profiles,
            "subscription_qos_profiles": subscription_qos_profiles,
            "owner_validated": len(recorder_blocks) == 1,
            "qos_validated": bool(recorder_qos_profiles) and all(
                all(profile.get(key) is not None for key in ("reliability", "durability"))
                for profile in recorder_qos_profiles
            ),
        }

    @staticmethod
    def _rosbag_qos_expectation(topic: str) -> dict[str, str]:
        return (
            {"reliability": "RELIABLE", "durability": "TRANSIENT_LOCAL"}
            if topic in {SAFETY_STOP_TOPIC, CONTRACT_TOPIC}
            else {}
        )

    @classmethod
    def _rosbag_live_topic_evidence(
        cls,
        topic: str,
        probe: Mapping[str, Any],
        log_evidence: Mapping[str, Any],
        *,
        discovery_attempts: Sequence[Mapping[str, Any]] | None = None,
    ) -> dict[str, Any]:
        stdout = str(probe.get("stdout", ""))
        count_match = re.search(r"Subscription count\s*:\s*(\d+)", stdout, re.IGNORECASE)
        endpoint = cls._rosbag_endpoint(stdout)
        endpoint_count = int(endpoint["subscription_endpoint_count"])
        qos_expectation = cls._rosbag_qos_expectation(topic)
        recorder_profiles = endpoint["qos_profiles"]
        qos_candidates = recorder_profiles or endpoint["subscription_qos_profiles"]
        qos_matches = not qos_expectation or (
                bool(qos_candidates)
                and (
                len(recorder_profiles) == 1
                and all(
                    str(qos_candidates[0].get(key, "")).upper() == value
                    for key, value in qos_expectation.items()
                )
                if recorder_profiles
                else any(
                    all(str(profile.get(key, "")).upper() == value for key, value in qos_expectation.items())
                    for profile in qos_candidates
                )
            )
        )
        log_topics = set(str(value) for value in log_evidence.get("subscribed_topics", []))
        log_confirmed = (
            bool(log_evidence.get("present"))
            and bool(log_evidence.get("all_requested_topics_subscribed"))
            and bool(log_evidence.get("topic_set_exact", log_topics == set(APPROVED_RECORD_TOPICS)))
            and topic in log_topics
        )
        return {
            "command": probe.get("command"),
            "returncode": probe.get("returncode"),
            "stdout": stdout,
            "stderr": probe.get("stderr", ""),
            "subscription_count": int(count_match.group(1)) if count_match else 0,
            "endpoint_count": endpoint_count,
            "recorder_endpoint": bool(endpoint["owner_validated"]),
            "endpoint": endpoint,
            "qos_expected": qos_expectation,
            "qos_matches": qos_matches,
            "log_confirmed": log_confirmed,
            "discovery_attempts": [dict(attempt) for attempt in (discovery_attempts or ())],
            "ready": (
                log_confirmed
                and bool(endpoint["owner_validated"])
                and bool(endpoint["qos_validated"])
                and (probe.get("returncode") == 0)
                and qos_matches
            ),
        }

    @staticmethod
    def _rosbag_startup_topic_valid(evidence: Mapping[str, Any]) -> bool:
        """Validate the saved startup contract, independent of graph identity."""
        if evidence.get("startup_contract_validated") is not None:
            return bool(evidence.get("startup_contract_validated"))
        endpoint = evidence.get("endpoint", {})
        return bool(
            evidence.get("ready")
            and evidence.get("log_confirmed")
            and evidence.get("recorder_endpoint")
            and endpoint.get("owner_validated")
            and endpoint.get("qos_validated")
            and evidence.get("qos_matches")
        )

    @staticmethod
    def _rosbag_probe_is_unknown_topic(probe: Mapping[str, Any]) -> bool:
        output = "\n".join(
            str(probe.get(field, "")) for field in ("stdout", "stderr", "error")
        )
        return bool(re.search(r"\bunknown\s+topic\b", output, re.IGNORECASE))

    @classmethod
    def _rosbag_probe_is_transiently_absent(cls, probe: Mapping[str, Any]) -> bool:
        """Classify only a graph absence; observed bad endpoints remain fatal."""
        if cls._rosbag_probe_is_unknown_topic(probe):
            return True
        if probe.get("returncode") != 0:
            return False
        endpoint = cls._rosbag_endpoint(str(probe.get("stdout", "")))
        count_match = re.search(
            r"Subscription count\s*:\s*(\d+)",
            str(probe.get("stdout", "")),
            re.IGNORECASE,
        )
        return (
            endpoint["subscription_endpoint_count"] == 0
            and (count_match is None or int(count_match.group(1)) == 0)
        )

    def _rosbag_endpoint_probe(
        self,
        topic: str,
        manifest: QualificationManifest,
        name: str,
    ) -> tuple[dict[str, Any], list[dict[str, Any]]]:
        """Retry only transient unknown-topic discovery, with a hard bound."""
        attempts: list[dict[str, Any]] = []
        probe: dict[str, Any] = {}
        for attempt_number in range(1, ROSBAG_ENDPOINT_DISCOVERY_MAX_ATTEMPTS + 1):
            capture_name = name if attempt_number == 1 else f"{name}-retry-{attempt_number}"
            probe = self._capture(
                capture_name,
                ["ros2", "topic", "info", topic, "-v"],
                manifest.attempt_dir,
                manifest,
            )
            unknown_topic = self._rosbag_probe_is_unknown_topic(probe)
            attempts.append(
                {
                    "attempt": attempt_number,
                    "returncode": probe.get("returncode"),
                    "unknown_topic": unknown_topic,
                }
            )
            if not unknown_topic or attempt_number == ROSBAG_ENDPOINT_DISCOVERY_MAX_ATTEMPTS:
                break
            time.sleep(ROSBAG_ENDPOINT_DISCOVERY_RETRY_DELAY_S)
        return probe, attempts

    @classmethod
    def _rosbag_probe_has_invalid_observed_endpoint(
        cls, _probe: Mapping[str, Any], evidence: Mapping[str, Any]
    ) -> bool:
        """Reject only positively identified recorder endpoints that violate policy."""
        endpoint = evidence.get("endpoint", {})
        recorder_count = int(endpoint.get("recorder_endpoint_count", 0) or 0)
        if recorder_count == 0:
            return False
        if recorder_count != 1:
            return True
        return bool(
            not endpoint.get("owner_validated")
            or not endpoint.get("qos_validated")
            or not evidence.get("qos_matches")
        )

    def _start_rosbag(self, manifest: QualificationManifest) -> bool:
        output_path = self._rosbag_path(manifest)
        command = self._default_rosbag_command(manifest)
        expected_qos_path = manifest.attempt_dir / "rosbag-qos-overrides.yaml"
        command_qos_path = self._rosbag_command_override_path(command)
        qos_overrides = self._rosbag_qos_override_evidence(expected_qos_path)
        command_uses_expected_qos = bool(
            command_qos_path
            and Path(command_qos_path).resolve() == expected_qos_path.resolve()
        )
        startup_contract_error = None
        if not command_uses_expected_qos:
            startup_contract_error = "rosbag command does not use the generated QoS override file"
        elif not qos_overrides.get("exact"):
            startup_contract_error = str(
                qos_overrides.get("error", "generated QoS override file is invalid")
            )
        if output_path.exists():
            self._write_rosbag_readiness(
                manifest,
                {
                    "ready": False,
                    "status": "failed",
                    "command": command,
                    "output_path": str(output_path),
                    "reason": "rosbag output path already exists before recorder startup",
                    "qos_overrides": qos_overrides,
                },
            )
            return False
        if startup_contract_error:
            self._write_rosbag_readiness(
                manifest,
                {
                    "ready": False,
                    "status": "failed",
                    "command": command,
                    "output_path": str(output_path),
                    "qos_overrides": qos_overrides,
                    "command_qos_override_path": command_qos_path,
                    "reason": startup_contract_error,
                },
            )
            return False
        try:
            self._start("rosbag", command, manifest)
        except (OSError, RuntimeError, ValueError) as error:
            self._write_rosbag_readiness(
                manifest,
                {
                    "ready": False,
                    "status": "failed",
                    "command": command,
                    "output_path": str(output_path),
                    "reason": "rosbag process could not be started",
                    "error": str(error),
                },
            )
            return False

        deadline = time.monotonic() + max(0.0, self.bag_startup_timeout_s)
        last_evidence: dict[str, Any] = {}
        observation_attempt = 0
        graph_diagnostics_captured = False
        while True:
            process = self._processes.get("rosbag")
            returncode = None
            if process is not None:
                returncode = process.poll()
            files: list[str] = []
            if output_path.is_dir():
                files = sorted(
                    path.relative_to(output_path).as_posix()
                    for path in output_path.rglob("*")
                    if path.is_file()
                )
            log_evidence = self._rosbag_log_evidence(manifest.attempt_dir / "rosbag.log")
            last_evidence = {
                "ready": False,
                "status": "starting",
                "command": command,
                "output_path": str(output_path),
                "process_returncode": returncode,
                "output_directory": output_path.is_dir(),
                "files": files,
                "qos_overrides": qos_overrides,
                "command_qos_override_path": command_qos_path,
                "command_uses_expected_qos": command_uses_expected_qos,
                "output_database": self._rosbag_output_evidence(output_path),
                "rosbag_log": log_evidence,
            }
            if returncode is not None:
                last_evidence.update(
                    {
                        "status": "failed",
                        "reason": "rosbag process exited before recording initialized",
                    }
                )
                self._write_rosbag_readiness(manifest, last_evidence)
                return False
            if output_path.is_dir():
                if process is None or process.poll() is not None:
                    last_evidence.update(
                        {
                            "status": "failed",
                            "process_returncode": None if process is None else process.poll(),
                            "reason": "rosbag process exited while recording initialized",
                        }
                    )
                    self._write_rosbag_readiness(manifest, last_evidence)
                    return False
                if not graph_diagnostics_captured:
                    observation_attempt += 1
                    probes: dict[str, Mapping[str, Any]] = {}
                    for topic in APPROVED_RECORD_TOPICS:
                        safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", topic.strip("/")) or "root"
                        probes[topic] = self._capture(
                            f"rosbag-topic-info-{safe_name}",
                            ["ros2", "topic", "info", topic, "-v"],
                            manifest.attempt_dir,
                            manifest,
                        )
                    log_evidence = self._rosbag_log_evidence(manifest.attempt_dir / "rosbag.log")
                    last_evidence["rosbag_log"] = log_evidence
                    subscriptions: dict[str, Any] = {}
                    for topic, probe in probes.items():
                        current = self._rosbag_live_topic_evidence(topic, probe, log_evidence)
                        current.update(
                            {
                                "observed": bool(current.get("endpoint_count")),
                                "observation_attempt": observation_attempt,
                                "observation_time": datetime.now(timezone.utc).isoformat(),
                                "diagnostic_only": True,
                            }
                        )
                        subscriptions[topic] = current
                    last_evidence["subscriptions"] = subscriptions
                    last_evidence["graph_observations_diagnostic_only"] = True
                    last_evidence["observation_attempt"] = observation_attempt
                    graph_diagnostics_captured = True
                post_probe_returncode = process.poll()
                last_evidence["process_returncode"] = post_probe_returncode
                if post_probe_returncode is not None:
                    last_evidence.update(
                        {
                            "status": "failed",
                            "reason": "rosbag process exited during startup endpoint discovery",
                        }
                    )
                    self._write_rosbag_readiness(manifest, last_evidence)
                    return False
                output_database = last_evidence["output_database"] = self._rosbag_output_evidence(output_path)
                log_evidence = last_evidence["rosbag_log"] = self._rosbag_log_evidence(
                    manifest.attempt_dir / "rosbag.log"
                )
                if log_evidence.get("all_requested_topics_subscribed") and not log_evidence.get(
                    "topic_set_exact"
                ):
                    last_evidence.update(
                        {
                            "status": "failed",
                            "reason": "rosbag log topic set is not exactly the approved topic set",
                        }
                    )
                    self._write_rosbag_readiness(manifest, last_evidence)
                    return False
                if output_database.get("open") and log_evidence.get("startup_contract"):
                    for observation in last_evidence.get("subscriptions", {}).values():
                        observation["startup_contract_validated"] = True
                    last_evidence.update(
                        {
                            "ready": True,
                            "status": "ready",
                            "initialization_evidence": "live-process-open-db-exact-log-and-explicit-qos",
                            "startup_contract_validated": True,
                            "observation_strategy": "graph-endpoints-diagnostic-only",
                        }
                    )
                    self._write_rosbag_readiness(manifest, last_evidence)
                    return True
            if time.monotonic() >= deadline:
                last_evidence.update(
                    {
                        "status": "failed",
                        "reason": (
                            "rosbag startup contract was not confirmed: "
                            "live process, open output database, exact log topic set, and QoS override are required"
                        ),
                    }
                )
                self._write_rosbag_readiness(manifest, last_evidence)
                return False
            time.sleep(0.05)

    def _stop(self, name: str) -> int | None:
        process = self._processes.get(name)
        if process is None:
            stream = self._logs.pop(name, None)
            if stream is not None:
                stream.close()
            return None
        initial_returncode = process.poll()
        planned = initial_returncode is None
        signals_sent: list[str] = []
        forced = False
        if planned:
            for sig, label, timeout in (
                (signal.SIGINT, "SIGINT", 5),
                (signal.SIGTERM, "SIGTERM", 3),
            ):
                if process.poll() is not None:
                    break
                try:
                    os.killpg(process.pid, sig)
                except OSError:
                    try:
                        process.send_signal(sig)
                    except (AttributeError, OSError):
                        pass
                signals_sent.append(label)
                try:
                    process.wait(timeout=timeout)
                except (OSError, subprocess.TimeoutExpired):
                    pass
            if process.poll() is None or self._group_alive(process):
                forced = True
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except OSError:
                    try:
                        process.kill()
                    except (AttributeError, OSError):
                        pass
                try:
                    process.wait(timeout=3)
                except (OSError, subprocess.TimeoutExpired):
                    pass
                self._wait_for_group_exit(process, 1.0)
        stream = self._logs.pop(name, None)
        if stream is not None:
            stream.close()
        returncode = getattr(process, "returncode", None)
        self._termination[name] = {
            "classification": "planned-termination" if planned else "unexpected-exit",
            "initial_returncode": initial_returncode,
            "returncode": returncode,
            "signals": signals_sent,
            "forced": forced,
        }
        return returncode

    def _attempt_processes(self) -> list[dict[str, Any]]:
        """Find descendants that escaped a launcher process group."""
        attempt_dir = self._attempt_dir
        if attempt_dir is None:
            return []
        marker = f"TINKER_SIM_ATTEMPT_DIR={attempt_dir}"
        records: list[dict[str, Any]] = []
        proc_root = Path("/proc")
        for entry in proc_root.iterdir() if proc_root.is_dir() else ():
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            if pid == os.getpid():
                continue
            try:
                environ = (entry / "environ").read_bytes().split(b"\0")
                if marker.encode() not in environ:
                    continue
                cmdline = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace").strip()
                stat = (entry / "stat").read_text(encoding="utf-8")
                fields = stat.rsplit(")", 1)[1].split()
                records.append({"pid": pid, "ppid": int(fields[1]), "pgid": int(fields[2]), "cmdline": cmdline})
            except (OSError, ValueError, IndexError):
                continue
        return sorted(records, key=lambda record: int(record["pid"]))

    def _managed_process_scope(self) -> tuple[set[int], set[int]]:
        """Return live managed PIDs and groups for the pre-teardown health check."""
        pids: set[int] = set()
        pgids: set[int] = set()
        for process in self._processes.values():
            try:
                if process.poll() is not None:
                    continue
                pid = int(process.pid)
            except (AttributeError, TypeError, ValueError):
                continue
            pids.add(pid)
            configured_pgid = getattr(process, "pgid", None)
            if configured_pgid is not None:
                try:
                    pgids.add(int(configured_pgid))
                    continue
                except (TypeError, ValueError):
                    pass
            try:
                pgids.add(os.getpgid(pid))
            except (OSError, ProcessLookupError):
                # Popen uses start_new_session=True, so a live managed leader
                # has its PID as its process group even when a fake process
                # cannot be queried through /proc.
                pgids.add(pid)
        return pids, pgids

    def _post_gate_attempt_processes(self) -> list[dict[str, Any]]:
        """Find escaped attempt processes while managed launch trees are live."""
        managed_pids, managed_pgids = self._managed_process_scope()
        isaac = self._processes.get("isaac")
        isaac_live = isaac is not None and isaac.poll() is None
        return [
            record
            for record in self._attempt_processes()
            if int(record["pid"]) not in managed_pids
            and int(record["pgid"]) not in managed_pgids
            and not (
                isaac_live
                and "omni.telemetry.transmitter" in str(record.get("cmdline", ""))
            )
        ]

    def _terminate_attempt_orphans(self, *, grace_s: float = 1.0) -> list[dict[str, Any]]:
        initial = self._attempt_processes()
        for record in initial:
            try:
                os.kill(int(record["pid"]), signal.SIGTERM)
            except OSError:
                pass
        deadline = time.monotonic() + max(0.0, grace_s)
        while time.monotonic() < deadline and self._attempt_processes():
            time.sleep(0.05)
        remaining = self._attempt_processes()
        for record in remaining:
            try:
                os.kill(int(record["pid"]), signal.SIGKILL)
            except OSError:
                pass
        if remaining:
            time.sleep(0.1)
        survivors = self._attempt_processes()
        self._orphan_cleanup = {
            "initial": initial,
            "forced_targets": remaining,
            "survivors": survivors,
        }
        return survivors

    @staticmethod
    def _rosbag_metadata_evidence(
        metadata: str, *, minimum_message_counts: Mapping[str, int] | None = None
    ) -> dict[str, Any]:
        minimums = (
            {topic: 1 for topic in APPROVED_RECORD_TOPICS}
            if minimum_message_counts is None
            else dict(minimum_message_counts)
        )
        evidence: dict[str, Any] = {
            "parsed": False,
            "topics": {},
            "missing_topics": [],
            "empty_topics": [],
            "below_minimum_topics": [],
            "missing_qos_metadata": [],
            "minimum_message_counts": minimums,
            "zero_allowed_topics": sorted(
                topic for topic, minimum in minimums.items() if minimum == 0
            ),
        }
        try:
            document = yaml.safe_load(metadata)
        except yaml.YAMLError as error:
            evidence["error"] = f"rosbag metadata is not valid YAML: {error}"
            return evidence
        root = document.get("rosbag2_bagfile_information") if isinstance(document, Mapping) else None
        records = root.get("topics_with_message_count") if isinstance(root, Mapping) else None
        if not isinstance(records, list):
            evidence["error"] = "rosbag metadata has no topics_with_message_count list"
            return evidence
        topics: dict[str, Any] = {}
        for record in records:
            if not isinstance(record, Mapping):
                continue
            metadata_record = record.get("topic_metadata")
            if not isinstance(metadata_record, Mapping) or not isinstance(metadata_record.get("name"), str):
                continue
            topic = str(metadata_record["name"])
            topics[topic] = {
                "message_count": record.get("message_count"),
                "topic_metadata": dict(metadata_record),
                "qos_metadata_present": bool(
                    isinstance(metadata_record.get("offered_qos_profiles"), str)
                    and metadata_record.get("offered_qos_profiles", "").strip()
                ),
            }
        missing = sorted(set(APPROVED_RECORD_TOPICS) - set(topics))
        empty = sorted(
            topic for topic in APPROVED_RECORD_TOPICS
            if topic in topics
            and (not isinstance(topics[topic].get("message_count"), int) or topics[topic]["message_count"] <= 0)
        )
        below_minimum = sorted(
            topic for topic in APPROVED_RECORD_TOPICS
            if topic in topics
            and (
                not isinstance(topics[topic].get("message_count"), int)
                or topics[topic]["message_count"] < minimums.get(topic, 1)
            )
        )
        missing_qos = sorted(
            topic for topic in APPROVED_RECORD_TOPICS
            if topic in topics and not topics[topic].get("qos_metadata_present")
        )
        evidence.update(
            {
                "parsed": True,
                "topics": topics,
                "missing_topics": missing,
                "empty_topics": empty,
                "below_minimum_topics": below_minimum,
                "missing_qos_metadata": missing_qos,
                "extra_topics": sorted(set(topics) - set(APPROVED_RECORD_TOPICS)),
            }
        )
        return evidence

    def _rosbag_final_evidence(
        self, manifest: QualificationManifest, *, final: bool = True
    ) -> tuple[bool, dict[str, Any], list[str]]:
        path = self._rosbag_path(manifest)
        metadata_path = path / "metadata.yaml"
        failures: list[str] = []
        metadata = metadata_path.read_text(encoding="utf-8") if metadata_path.is_file() else ""
        try:
            minimums = self._rosbag_minimum_message_counts(
                self._config(), gate=self.gate
            )
        except ValueError as error:
            minimums = {}
            failures.append(str(error))
        metadata_evidence = self._rosbag_metadata_evidence(
            metadata, minimum_message_counts=minimums
        )
        metadata_structured = bool(metadata_evidence.get("parsed"))
        if final and not metadata_structured:
            failures.append("rosbag metadata is missing or not structured rosbag2 metadata")
        baseline: dict[str, Any] = {}
        baseline_path = manifest.attempt_dir / "pre-gate-baseline.json"
        if baseline_path.is_file():
            try:
                baseline = _json_file(baseline_path)
            except (OSError, ValueError, json.JSONDecodeError):
                failures.append("pre-gate baseline is invalid")
        else:
            failures.append("pre-gate baseline is missing")
        if final and baseline.get("status") != "ready":
            failures.append("pre-gate baseline was not ready")
        topic_counts: dict[str, int | None] = {}
        endpoint_checks: dict[str, Any] = {}
        rosbag_log: dict[str, Any] = {}
        if final:
            live_path = manifest.attempt_dir / "rosbag-pre-shutdown.json"
            try:
                live_evidence = _json_file(live_path)
            except (OSError, ValueError, json.JSONDecodeError):
                live_evidence = {}
                failures.append("pre-shutdown rosbag endpoint evidence is invalid")
            if live_evidence.get("status") != "ready":
                failures.append("pre-shutdown rosbag endpoint evidence was not ready")
            rosbag_log = dict(live_evidence.get("rosbag_log", {}))
            if not rosbag_log.get("startup_contract"):
                failures.append("rosbag log does not confirm the exact approved topic set")
            endpoint_checks = dict(live_evidence.get("subscriptions", {}))
            for topic in APPROVED_RECORD_TOPICS:
                endpoint = endpoint_checks.get(topic, {})
                if not endpoint.get("startup_validated"):
                    failures.append(f"rosbag startup contract is invalid for {topic}")
            recorder_exit = dict(self._termination.get("rosbag", {}))
            if recorder_exit.get("returncode") != 0 or recorder_exit.get("forced"):
                failures.append("rosbag recorder did not exit cleanly")
        else:
            readiness_path = manifest.attempt_dir / "rosbag-readiness.json"
            try:
                startup_readiness = _json_file(readiness_path)
            except (OSError, ValueError, json.JSONDecodeError):
                startup_readiness = {}
                failures.append("startup rosbag readiness evidence is missing or invalid")
            if startup_readiness.get("status") != "ready":
                failures.append("startup rosbag endpoint evidence was not ready")
            startup_subscriptions = dict(startup_readiness.get("subscriptions", {}))
            rosbag_log = dict(startup_readiness.get("rosbag_log", {}))
            if not rosbag_log:
                rosbag_log = self._rosbag_log_evidence(manifest.attempt_dir / "rosbag.log")
            fallback_reasons: list[str] = []
            subscriptions: dict[str, Any] = {}
            for topic in APPROVED_RECORD_TOPICS:
                safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", topic.strip("/")) or "root"
                probe, discovery_attempts = self._rosbag_endpoint_probe(
                    topic,
                    manifest,
                    f"rosbag-pre-shutdown-topic-info-{safe_name}",
                )
                startup_evidence = startup_subscriptions.get(topic, {})
                evidence = self._rosbag_live_topic_evidence(
                    topic,
                    probe,
                    rosbag_log,
                    discovery_attempts=discovery_attempts,
                )
                startup_valid = self._rosbag_startup_topic_valid(startup_evidence)
                endpoint_absent = self._rosbag_probe_is_transiently_absent(probe)
                endpoint_observed = not endpoint_absent
                evidence.update(
                    {
                        "startup_validated": startup_valid,
                        "startup_evidence": dict(startup_evidence),
                        "post_gate_endpoint_observed": endpoint_observed,
                        "fallback_used": False,
                    }
                )
                if startup_valid:
                    reason = (
                        "post-gate graph endpoint was unresolved or absent; using validated "
                        "startup recorder process/DB/log/QoS contract pending finalized bag count"
                    )
                    evidence.update(
                        {
                            "ready": True,
                            "fallback_used": endpoint_absent,
                            "fallback_reason": reason,
                            "graph_observation_diagnostic_only": True,
                        }
                    )
                    if endpoint_absent:
                        fallback_reasons.append(f"{topic}: {reason}")
                subscriptions[topic] = evidence
                if not evidence["ready"]:
                    failures.append(f"rosbag startup contract is invalid for {topic}")
            endpoint_checks = subscriptions
        for topic in APPROVED_RECORD_TOPICS:
            record = metadata_evidence.get("topics", {}).get(topic, {})
            count = record.get("message_count")
            topic_counts[topic] = count if isinstance(count, int) else None
        if final and metadata_structured:
            for topic in metadata_evidence.get("missing_topics", []):
                failures.append(f"rosbag metadata is missing approved topic {topic}")
            for topic in metadata_evidence.get("below_minimum_topics", []):
                minimum = minimums.get(topic, 1)
                if minimum:
                    failures.append(
                        f"rosbag metadata has fewer than {minimum} messages for {topic}"
                    )
            for topic in metadata_evidence.get("missing_qos_metadata", []):
                failures.append(f"rosbag metadata is missing QoS metadata for {topic}")
        evidence = {
            "status": "ready" if not failures else "failed",
            "phase": "finalized-metadata-and-pre-shutdown-endpoints" if final else "pre-shutdown-live-endpoints",
            "metadata": {"path": str(metadata_path), "present": metadata_path.is_file(), "structured": metadata_structured},
            "metadata_topics": metadata_evidence,
            "rosbag_log": rosbag_log,
            "topic_message_counts": topic_counts,
            "endpoint_checks": endpoint_checks,
            "subscriptions": endpoint_checks if not final else {},
            "fallback_reasons": fallback_reasons if not final else [],
            "baseline": baseline,
            "failures": failures,
        }
        output_name = "rosbag-final.json" if final else "rosbag-pre-shutdown.json"
        (manifest.attempt_dir / output_name).write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return not failures, evidence, failures

    def _post_gate_health(self, manifest: QualificationManifest) -> bool:
        checks: dict[str, Any] = {}
        failures: list[str] = []
        for name in ("isaac", "humble", "rosbag"):
            process = self._processes.get(name)
            returncode = None if process is None else process.poll()
            alive = process is not None and returncode is None
            checks[name] = {"alive": alive, "returncode": returncode}
            if not alive:
                failures.append(f"{name} exited before intentional teardown (returncode={returncode})")
        readiness: dict[str, Any] = {}
        if self.gate == "arm-collision":
            _clear, safety, _reason = self._safety_readiness(manifest)
            readiness_ok = safety.get("value") is True
            readiness = {
                "ready": readiness_ok,
                "mode": "expected-terminal-collision-stop",
                "safety_stop": safety,
            }
        else:
            readiness_ok = self._ready(manifest)
            readiness_path = manifest.attempt_dir / "readiness.json"
            if readiness_path.is_file():
                try:
                    readiness = _json_file(readiness_path)
                except (OSError, ValueError, json.JSONDecodeError):
                    readiness = {"ready": False}
        if not readiness_ok:
            failures.append("post-gate readiness revalidation failed")
        rosbag_ok, rosbag, rosbag_failures = self._rosbag_final_evidence(manifest, final=False)
        if not rosbag_ok:
            failures.extend(f"post-gate recorder: {failure}" for failure in rosbag_failures)
        orphan_processes = self._post_gate_attempt_processes()
        if orphan_processes:
            failures.append("qualification attempt still has descendant processes")
        evidence = {
            "status": "ready" if not failures else "failed",
            "captured_at": datetime.now(timezone.utc).isoformat(),
            "checks": checks,
            "readiness": readiness,
            "rosbag": rosbag,
            "orphan_processes": orphan_processes,
            "failures": failures,
        }
        (manifest.attempt_dir / "post-gate-health.json").write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return not failures

    @staticmethod
    def _jsonl_count(path: Path) -> int:
        if not path.is_file():
            return 0
        try:
            return sum(1 for line in path.read_text(encoding="utf-8").splitlines() if line.strip())
        except OSError:
            return 0

    @staticmethod
    def _jsonl_records(path: Path) -> tuple[list[Mapping[str, Any]], list[str]]:
        records: list[Mapping[str, Any]] = []
        errors: list[str] = []
        if not path.is_file():
            return records, errors
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError as error:
            return records, [str(error)]
        for line_number, line in enumerate(lines, 1):
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except (TypeError, ValueError, json.JSONDecodeError) as error:
                errors.append(f"line {line_number}: invalid JSON ({error})")
                continue
            if not isinstance(value, Mapping):
                errors.append(f"line {line_number}: record is not a JSON object")
                continue
            records.append(value)
        return records, errors

    @staticmethod
    def _canonical_json(value: Any) -> str:
        return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)

    @classmethod
    def _compare_truth_records(
        cls,
        raw_records: Sequence[Mapping[str, Any]],
        evaluator_records: Sequence[Mapping[str, Any]],
        raw_errors: Sequence[str] = (),
        evaluator_errors: Sequence[str] = (),
    ) -> tuple[bool, list[str]]:
        mismatches: list[str] = []
        if len(evaluator_records) != len(raw_records):
            mismatches.append("raw/evaluator record counts differ")
        for index, (raw, evaluated) in enumerate(zip(raw_records, evaluator_records), 1):
            frame = evaluated.get("frame")
            if not isinstance(frame, Mapping):
                mismatches.append(f"evaluator record {index} has no embedded raw frame")
                continue
            if cls._canonical_json(frame) != cls._canonical_json(raw):
                mismatches.append(f"evaluator record {index} does not exactly match raw truth")
        return not raw_errors and not evaluator_errors and not mismatches, mismatches

    def _wait_for_evaluator_drain(self, manifest: QualificationManifest) -> bool:
        raw_path = manifest.attempt_dir / "physics_truth.jsonl"
        evaluator_path = manifest.attempt_dir / "evaluator.jsonl"
        deadline = time.monotonic() + max(5.0, self.readiness_timeout_s)
        raw_records, raw_errors = self._jsonl_records(raw_path)
        evaluator_records, evaluator_errors = self._jsonl_records(evaluator_path)
        correlation = False
        mismatches: list[str] = []
        evaluator_process = self._processes.get("humble")
        fail_fast_reason: str | None = None
        while time.monotonic() < deadline:
            raw_records, raw_errors = self._jsonl_records(raw_path)
            evaluator_records, evaluator_errors = self._jsonl_records(evaluator_path)
            correlation, mismatches = self._compare_truth_records(
                raw_records, evaluator_records, raw_errors, evaluator_errors
            )
            if correlation:
                break
            evaluator_returncode = None if evaluator_process is None else evaluator_process.poll()
            if evaluator_process is not None and evaluator_returncode is not None:
                fail_fast_reason = "evaluator process exited before exact truth drain"
                break
            time.sleep(0.05)
        raw_count = len(raw_records)
        evaluator_count = len(evaluator_records)
        drained = correlation
        evidence = {
            "status": "drained" if drained else "timeout",
            "raw_truth_frames": raw_count,
            "evaluator_frames": evaluator_count,
            "counts_match_or_exceed": evaluator_count >= raw_count,
            "exact_correlation": drained,
            "raw_errors": raw_errors,
            "evaluator_errors": evaluator_errors,
            "mismatches": mismatches,
            "bounded_timeout_s": max(5.0, self.readiness_timeout_s),
            "wait_mode": "fail-fast" if fail_fast_reason else "bounded",
            "fail_fast_reason": fail_fast_reason,
            "evaluator_process": {
                "present": evaluator_process is not None,
                "alive": evaluator_process is not None and evaluator_process.poll() is None,
                "returncode": None if evaluator_process is None else evaluator_process.poll(),
            },
            "captured_at": datetime.now(timezone.utc).isoformat(),
        }
        (manifest.attempt_dir / "truth-drain.json").write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        return drained

    def _builtin_gate_command(
        self, name: str, manifest: QualificationManifest
    ) -> list[str] | None:
        executable = self.root / "validation/manipulation_gate_executor.py"
        if not executable.is_file():
            return None
        return [
            "/usr/bin/python3",
            str(executable),
            "--gate",
            name,
            "--attempt-dir",
            str(manifest.attempt_dir),
            "--config",
            str(self.config_path),
        ]

    def _run_gate(self, name: str, manifest: QualificationManifest) -> dict[str, Any]:
        diagnostic = name in self.gate_commands
        command = (
            self.gate_commands.get(name)
            if diagnostic
            else self._builtin_gate_command(name, manifest)
        )
        if command is None:
            return {"gate": name, "status": "not-configured", "pass": False}
        command = _ros2_command(command)
        log_path = manifest.attempt_dir / f"gate-{name}.log"
        timeout = float(self._config().get("execution_timeout_s", 90.0))
        try:
            with log_path.open("w", encoding="utf-8") as stream:
                completed = self._command_runner(
                    list(command), cwd=self.root, env=self._env(manifest, "ros-tooling"), stdout=stream,
                    stderr=subprocess.STDOUT, text=True, timeout=timeout, check=False,
                )
            return {
                "gate": name,
                "status": (
                    "executed-unverified"
                    if diagnostic and completed.returncode == 0
                    else "executed-pending-verification"
                    if completed.returncode == 0
                    else "failed"
                ),
                "pass": False,
                "verified": False,
                "diagnostic": diagnostic,
                "command": list(command),
                "returncode": completed.returncode,
            }
        except (OSError, subprocess.SubprocessError) as error:
            return {"gate": name, "status": "error", "pass": False, "error": str(error)}

    def _verify_gate(
        self,
        name: str,
        manifest: QualificationManifest,
        execution: Mapping[str, Any],
    ) -> dict[str, Any]:
        if execution.get("diagnostic"):
            return {
                **dict(execution),
                "status": "executed-unverified",
                "pass": False,
                "verified": False,
                "reason": "external gate commands are diagnostic-only",
            }
        if execution.get("returncode") != 0:
            return dict(execution)
        verifier = self.root / "validation/manipulation_gate_verifier.py"
        if not verifier.is_file():
            return {
                **dict(execution),
                "status": "evidence-invalid",
                "reason": "built-in gate verifier is missing",
            }
        log_path = manifest.attempt_dir / f"verifier-{name}.log"
        command = [
            sys.executable,
            str(verifier),
            "--gate",
            name,
            "--attempt-dir",
            str(manifest.attempt_dir),
            "--config",
            str(self.config_path),
        ]
        try:
            with log_path.open("w", encoding="utf-8") as stream:
                completed = self._command_runner(
                    command,
                    cwd=self.root,
                    env=self._env(manifest, "humble"),
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=30.0,
                    check=False,
                )
            verdict_path = manifest.attempt_dir / "gate-verdict.json"
            verdict = _json_file(verdict_path) if verdict_path.is_file() else {}
            status = str(verdict.get("status", "evidence-invalid"))
            if status not in {"verified-pass", "verified-fail", "evidence-invalid"}:
                status = "evidence-invalid"
            expected_returncode = {
                "verified-pass": 0,
                "verified-fail": 1,
                "evidence-invalid": 2,
            }[status]
            identity_valid = (
                verdict.get("gate") == name
                and verdict.get("attempt_id") == manifest.attempt_id
                and verdict.get("pass") is (status == "verified-pass")
                and verdict.get("verified")
                is (status in {"verified-pass", "verified-fail"})
            )
            if completed.returncode != expected_returncode or not identity_valid:
                status = "evidence-invalid"
            return {
                **dict(execution),
                "status": status,
                "pass": status == "verified-pass",
                "verified": status in {"verified-pass", "verified-fail"},
                "verifier_returncode": completed.returncode,
                "verdict": verdict,
            }
        except (OSError, subprocess.SubprocessError, ValueError, json.JSONDecodeError) as error:
            return {
                **dict(execution),
                "status": "evidence-invalid",
                "pass": False,
                "verified": False,
                "reason": f"verifier failed: {error}",
            }

    def _build_contact_sheet(
        self, name: str, manifest: QualificationManifest
    ) -> dict[str, Any]:
        generator = self.root / "validation/manipulation_contact_sheets.py"
        if not generator.is_file():
            return {"status": "evidence-invalid", "errors": ["contact-sheet generator is missing"]}
        log_path = manifest.attempt_dir / f"contact-sheet-{name}.log"
        command = [
            sys.executable,
            str(generator),
            "--attempt-dir",
            str(manifest.attempt_dir),
        ]
        try:
            with log_path.open("w", encoding="utf-8") as stream:
                completed = self._command_runner(
                    command,
                    cwd=self.root,
                    env=self._env(manifest, "humble"),
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    text=True,
                    timeout=30.0,
                    check=False,
                )
            result_path = manifest.attempt_dir / "visual-evidence-result.json"
            result = _json_file(result_path) if result_path.is_file() else {}
            if completed.returncode != 0 or result.get("status") != "valid":
                return {
                    **result,
                    "status": "evidence-invalid",
                    "returncode": completed.returncode,
                }
            return {**result, "returncode": completed.returncode}
        except (OSError, subprocess.SubprocessError, ValueError, json.JSONDecodeError) as error:
            return {"status": "evidence-invalid", "errors": [str(error)]}

    def _wait_for_visual_capture(
        self, manifest: QualificationManifest, expected_records: int = 8
    ) -> bool:
        path = manifest.attempt_dir / "visual-keyframes.jsonl"
        deadline = time.monotonic() + 20.0
        count = 0
        while time.monotonic() < deadline:
            try:
                count = sum(
                    1
                    for line in path.read_text(encoding="utf-8").splitlines()
                    if line.strip()
                )
            except OSError:
                count = 0
            if count >= expected_records:
                break
            failures = self._process_failures()
            if failures:
                break
            time.sleep(0.05)
        evidence = {
            "schema_version": 1,
            "expected_records": expected_records,
            "observed_records": count,
            "drained": count >= expected_records,
            "captured_at": datetime.now(timezone.utc).isoformat(),
        }
        (manifest.attempt_dir / "visual-drain.json").write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return bool(evidence["drained"])

    def run(self, manifest: QualificationManifest | None = None, manifest_only: bool = False) -> QualificationResult:
        manifest = manifest or self.prepare_manifest()
        self._attempt_dir = manifest.attempt_dir
        for filename in ("physics_truth.jsonl", "evaluator.jsonl"):
            (manifest.attempt_dir / filename).touch(exist_ok=True)
        if manifest_only:
            return QualificationResult(manifest.attempt_dir, "manifest-only", {}, {})
        gpu_baseline = self._gpu_processes()
        gate_results: dict[str, Any] = {}
        exit_codes: dict[str, int | None] = {}
        if (
            not self.gate_commands
            and not (self.root / "validation/manipulation_gate_executor.py").is_file()
        ):
            gate_results = {
                gate: {"gate": gate, "status": "not-configured", "pass": False}
                for gate in manifest.data["selected_gates"]
            }
            (manifest.attempt_dir / "exit_codes.json").write_text("{}\n", encoding="utf-8")
            (manifest.attempt_dir / "gate_results.json").write_text(
                json.dumps(gate_results, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            (manifest.attempt_dir / "result.json").write_text(
                json.dumps(
                    {"status": "not-configured", "attempt_id": manifest.attempt_id, "gates": gate_results},
                    indent=2,
                    sort_keys=True,
                ) + "\n",
                encoding="utf-8",
            )
            return QualificationResult(manifest.attempt_dir, "not-configured", gate_results, exit_codes)
        status = "failed"
        try:
            self._start("isaac", self.isaac_command, manifest)
            self._start("humble", self.humble_command, manifest)
            self._snapshot_graph(manifest, "startup")
            ready = self._ready(manifest)
            if ready:
                pre_gate_failures = self._process_failures()
                if pre_gate_failures:
                    readiness_path = manifest.attempt_dir / "readiness.json"
                    try:
                        readiness = _json_file(readiness_path)
                    except (OSError, ValueError, json.JSONDecodeError):
                        readiness = {}
                    readiness.update({"ready": False, "phase": "pre-gate", "reasons": pre_gate_failures})
                    self._write_readiness(manifest, readiness)
                    ready = False
            if ready:
                if self._start_rosbag(manifest):
                    if self._write_pre_gate_baseline(manifest):
                        for gate in manifest.data["selected_gates"]:
                            self._write_gate_window(gate, manifest)
                            gate_results[gate] = self._run_gate(gate, manifest)
                        successful = gate_results and all(
                            result.get("status")
                            in {
                                "executed-unverified",
                                "executed-pending-verification",
                            }
                            and result.get("returncode") == 0
                            for result in gate_results.values()
                        )
                        status = (
                            "verification-pending" if successful else "failed"
                        )
                        built_in_gates = [
                            gate
                            for gate, result in gate_results.items()
                            if not result.get("diagnostic")
                            and result.get("returncode") == 0
                        ]
                        if built_in_gates and not self._wait_for_visual_capture(
                            manifest, expected_records=8 * len(built_in_gates)
                        ):
                            status = "evidence-invalid"
                        if not self._post_gate_health(manifest):
                            status = "failed"
                    else:
                        for gate in manifest.data["selected_gates"]:
                            gate_results[gate] = {
                                "gate": gate,
                                "status": "not-run",
                                "pass": False,
                                "verified": False,
                                "reason": "pre-gate safety/contract/controller baseline was not ready",
                            }
                        status = "failed"
                else:
                    for gate in manifest.data["selected_gates"]:
                        gate_results[gate] = {
                            "gate": gate,
                            "status": "not-run",
                            "pass": False,
                            "verified": False,
                            "reason": "rosbag recording did not initialize",
                        }
                    status = "failed"
            else:
                status = "startup-failed"
        except (OSError, RuntimeError, ValueError) as error:
            (manifest.attempt_dir / "runner-error.txt").write_text(str(error) + "\n", encoding="utf-8")
            status = "runner-error"
        finally:
            self._snapshot_graph(manifest, "shutdown")
            # Stop the producer first, let the evaluator consume its final raw
            # frames, then stop the evaluator and recorder in that order.
            if "isaac" in self._processes:
                exit_codes["isaac"] = self._stop("isaac")
            drained = self._wait_for_evaluator_drain(manifest)
            if not drained and status in {
                "verification-pending",
                "unverified",
                "manifest-only",
            }:
                status = "evidence-invalid"
            if "humble" in self._processes:
                exit_codes["humble"] = self._stop("humble")
            if "rosbag" in self._processes:
                exit_codes["rosbag"] = self._stop("rosbag")
            rosbag_ok, _rosbag_final, rosbag_failures = self._rosbag_final_evidence(manifest)
            if not rosbag_ok and status in {
                "verification-pending",
                "unverified",
            }:
                status = "evidence-invalid"
            orphan_initial = self._attempt_processes()
            orphan_remaining = self._terminate_attempt_orphans()
            orphan_cleanup = dict(self._orphan_cleanup)
            self._termination["orphan-cleanup"] = {
                "classification": "forced-descendant-cleanup" if orphan_initial else "none-observed",
                "initial": orphan_cleanup.get("initial", orphan_initial),
                "forced_targets": orphan_cleanup.get("forced_targets", []),
                "survivors": orphan_cleanup.get("survivors", orphan_remaining),
                "remaining": orphan_cleanup.get("survivors", orphan_remaining),
                "forced": bool(orphan_cleanup.get("forced_targets", [])),
            }
            if orphan_initial:
                status = "failed"
            teardown_failures = [
                name for name, record in self._termination.items()
                if name != "orphan-cleanup"
                and (record.get("forced") or (record.get("classification") == "planned-termination" and record.get("returncode") not in (0, None)))
            ]
            if teardown_failures:
                status = "failed"
            resources_clean = self._write_resource_evidence(manifest, gpu_baseline)
            if not resources_clean:
                status = "failed"
            evidence_ready = drained and rosbag_ok and not orphan_initial and not teardown_failures
            prior_evidence_invalid = status == "evidence-invalid"
            if status in {
                "verification-pending",
                "unverified",
                "evidence-invalid",
            } and evidence_ready:
                for gate, execution in list(gate_results.items()):
                    verified = self._verify_gate(gate, manifest, execution)
                    if not execution.get("diagnostic"):
                        visual = self._build_contact_sheet(gate, manifest)
                        verified["visual_evidence"] = visual
                        if (
                            prior_evidence_invalid
                            or visual.get("status") != "valid"
                        ):
                            verified.update(
                                {
                                    "status": "evidence-invalid",
                                    "pass": False,
                                    "verified": False,
                                }
                            )
                    gate_results[gate] = verified
                statuses = {
                    str(result.get("status")) for result in gate_results.values()
                }
                if statuses == {"verified-pass"}:
                    status = "verified-pass"
                elif statuses == {"executed-unverified"}:
                    status = "unverified"
                elif "evidence-invalid" in statuses:
                    status = "evidence-invalid"
                elif "verified-fail" in statuses:
                    status = "verified-fail"
                else:
                    status = "failed"
            (manifest.attempt_dir / "termination.json").write_text(
                json.dumps(self._termination, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            (manifest.attempt_dir / "exit_codes.json").write_text(json.dumps(exit_codes, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            (manifest.attempt_dir / "gate_results.json").write_text(json.dumps(gate_results, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            (manifest.attempt_dir / "result.json").write_text(json.dumps({"status": status, "attempt_id": manifest.attempt_id, "gates": gate_results}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            self._settle_evidence_files(manifest)
        return QualificationResult(manifest.attempt_dir, status, gate_results, exit_codes)


class QualificationProcessHelpers:
    """Task 4 additive thin wrapper around the six-gate runner's process,
    recorder, and provenance mechanics.

    This is a purely additive exposure for the integrated gate executor and
    later tasks.  It reuses the existing ``QualificationRunner._start`` /
    ``_stop`` / ``_env`` / ``_popen`` / ``_snapshot_graph`` / ``_capture``,
    the module's atomic JSON writer, and the source-inventory/provenance
    methods without duplicating process ownership or cleanup logic.  The
    six-gate ``run()`` behavior, artifact schema, and command ordering are
    unchanged.
    """

    def __init__(self, runner: QualificationRunner) -> None:
        if not isinstance(runner, QualificationRunner):
            raise TypeError("QualificationProcessHelpers requires a QualificationRunner")
        self.runner = runner

    # -- process ownership ------------------------------------------------

    def start(self, name: str, command: Sequence[str], manifest: QualificationManifest) -> None:
        """Start a managed process under the runner's ownership/cleanup scope."""
        self.runner._start(name, command, manifest)

    def stop(self, name: str) -> int | None:
        """Stop a managed process using the runner's graceful-then-forced teardown."""
        return self.runner._stop(name)

    @property
    def popen(self) -> Callable[..., Any]:
        """The injectable ``subprocess.Popen`` used by the runner."""
        return self.runner._popen

    @property
    def command_runner(self) -> Callable[..., Any]:
        """The injectable ``subprocess.run`` used by the runner."""
        return self.runner._command_runner

    def env(self, manifest: QualificationManifest, role: str = "humble") -> dict[str, str]:
        """Build the per-role process environment exactly as the runner does."""
        return self.runner._env(manifest, role)

    def capture(
        self, name: str, command: Sequence[str], directory: Path, manifest: QualificationManifest
    ) -> dict[str, Any]:
        """Run a ROS observation subprocess and retain the complete result."""
        return self.runner._capture(name, command, directory, manifest)

    def observe(
        self, command: Sequence[str], manifest: QualificationManifest, *, timeout: float = 5.0
    ) -> dict[str, Any]:
        """Run a ROS observation and return the complete subprocess result."""
        return self.runner._observe(command, manifest, timeout=timeout)

    def snapshot_graph(self, manifest: QualificationManifest, suffix: str) -> None:
        """Capture the graph-nodes/topics/command-ownership evidence files."""
        self.runner._snapshot_graph(manifest, suffix)

    # -- attempt/provenance mechanics --------------------------------------

    def new_attempt_dir(self) -> tuple[str, Path]:
        """Allocate a fresh attempt directory using the runner's unique naming."""
        return self.runner._new_attempt_dir()

    def source_inventory(self) -> dict[str, Any]:
        """Return the runner's complete source/provenance inventory."""
        return self.runner._source_inventory()

    def write_json_atomic(self, path: Path, value: Mapping[str, Any]) -> None:
        """Atomically write a JSON mapping via the module's canonical writer."""
        _write_json_atomic(path, value)

    def ros_tooling_environment(
        self,
        *,
        root: Path = ROOT,
        dds_profile: str | None = None,
        domain_id: str | None = None,
        rmw_implementation: str | None = None,
    ) -> dict[str, str]:
        """Return inherited ROS tooling state with the wrapper's DDS policy."""
        return _ros_tooling_environment(
            root=root,
            dds_profile=dds_profile,
            domain_id=domain_id,
            rmw_implementation=rmw_implementation,
        )


def qualification_ros_tooling_environment(
    *,
    root: Path = ROOT,
    dds_profile: str | None = None,
    domain_id: str | None = None,
    rmw_implementation: str | None = None,
) -> dict[str, str]:
    """Task 4 additive: expose the runner's ROS-tooling environment builder."""
    return _ros_tooling_environment(
        root=root,
        dds_profile=dds_profile,
        domain_id=domain_id,
        rmw_implementation=rmw_implementation,
    )


def qualification_write_json_atomic(path: Path, value: Mapping[str, Any]) -> None:
    """Task 4 additive: expose the module's atomic JSON writer."""
    _write_json_atomic(path, value)


def qualification_new_suite_dir(attempt_root: Path) -> tuple[str, Path]:
    """Task 4 additive: expose the suite-directory allocator."""
    return _new_suite_dir(attempt_root)


def qualification_source_inventory(
    *,
    root: Path = ROOT,
    config_path: Path | None = None,
    scenario_path: Path | None = None,
) -> dict[str, Any]:
    """Task 4 additive: source/provenance inventory for an integrated attempt.

    Builds a minimal ``QualificationRunner`` scoped to the caller's config and
    scenario paths and returns its ``_source_inventory()`` without starting any
    process.  The returned schema is identical to the six-gate manifest
    ``sources`` section.
    """
    runner = QualificationRunner(
        root=root,
        config_path=config_path,
        scenario_path=scenario_path,
        gate="all",
    )
    return runner._source_inventory()


# ---------------------------------------------------------------------------
# Task 8 additive: reusable core lifecycle helpers for the integrated runner.
#
# These are thin, semantically identical exposures of the six-gate runner's
# process/rosbag/drain/resource mechanics so the integrated qualification
# orchestrator can reuse them without duplicating lifecycle logic.  None of
# them changes ``QualificationRunner`` behavior or the ``--gate`` CLI.
# ---------------------------------------------------------------------------


def qualification_source_identity(
    *,
    root: Path = ROOT,
    config_path: Path | None = None,
    scenario_path: Path | None = None,
) -> dict[str, Any]:
    """Capture the immutable source identity for a qualification attempt.

    Combines the runner's source/provenance inventory with the runtime/tool
    versions.  Reuses the exact six-gate inventory and version builders; the
    integrated runner records this before Gate B so a later source-lock check
    can never fall back to capturing and trusting current state.
    """
    return {
        "sources": qualification_source_inventory(
            root=root, config_path=config_path, scenario_path=scenario_path
        ),
        "versions": _runtime_versions(root=root),
    }


def qualification_record_topics(
    record_topics: Sequence[str] | None = None,
) -> tuple[str, ...]:
    """Expose the approved record-topic set (configurable default).

    The integrated runner passes its own topic set (identical to
    ``APPROVED_RECORD_TOPICS`` by default) through the same QoS/rosbag helpers.
    """
    return tuple(record_topics or APPROVED_RECORD_TOPICS)


def qualification_rosbag_qos_profiles(
    profiles: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Expose the configurable rosbag QoS override profiles."""
    return dict(profiles or ROSBAG_QOS_OVERRIDE_PROFILES)


def qualification_rosbag_output_evidence(path: Path) -> dict[str, Any]:
    """Open/validate a rosbag output database without touching the runner."""
    return QualificationRunner._rosbag_output_evidence(path)


def qualification_rosbag_metadata_evidence(
    metadata: str,
    *,
    minimum_message_counts: Mapping[str, int] | None = None,
) -> dict[str, Any]:
    """Parse/validate rosbag2 metadata fail-closed, reusing the gate policy."""
    return QualificationRunner._rosbag_metadata_evidence(
        metadata, minimum_message_counts=minimum_message_counts
    )


def qualification_jsonl_records(path: Path) -> tuple[list[Mapping[str, Any]], list[str]]:
    """Read raw/evaluator JSONL records with per-line errors, unchanged."""
    return QualificationRunner._jsonl_records(path)


def qualification_jsonl_count(path: Path) -> int:
    """Count nonblank JSONL records, unchanged."""
    return QualificationRunner._jsonl_count(path)


def qualification_compare_truth_records(
    raw_records: Sequence[Mapping[str, Any]],
    evaluator_records: Sequence[Mapping[str, Any]],
    raw_errors: Sequence[str] = (),
    evaluator_errors: Sequence[str] = (),
) -> tuple[bool, list[str]]:
    """Require exact raw/evaluator correlation, unchanged."""
    return QualificationRunner._compare_truth_records(
        raw_records,
        evaluator_records,
        raw_errors=raw_errors,
        evaluator_errors=evaluator_errors,
    )


def qualification_start_process(
    runner: QualificationRunner,
    name: str,
    command: Sequence[str],
    manifest: QualificationManifest,
) -> None:
    """Start a managed child process under the runner's ownership/cleanup."""
    runner._start(name, command, manifest)


def qualification_observe(
    runner: QualificationRunner,
    command: Sequence[str],
    manifest: QualificationManifest,
    *,
    timeout: float = 5.0,
) -> dict[str, Any]:
    """Run a ROS observation via the runner, retaining the full result."""
    return runner._observe(command, manifest, timeout=timeout)


def qualification_wait_for_ready(runner: QualificationRunner, manifest: QualificationManifest) -> bool:
    """Run the runner's bounded readiness loop unchanged."""
    return runner._ready(manifest)


def qualification_wait_for_evaluator_drain(
    runner: QualificationRunner, manifest: QualificationManifest
) -> bool:
    """Wait for exact raw/evaluator drain using the runner's bounded wait."""
    return runner._wait_for_evaluator_drain(manifest)


def qualification_rosbag_final_evidence(
    runner: QualificationRunner,
    manifest: QualificationManifest,
    *,
    final: bool = True,
) -> tuple[bool, dict[str, Any], list[str]]:
    """Finalize/validate the rosbag evidence unchanged."""
    return runner._rosbag_final_evidence(manifest, final=final)


def qualification_stop_process(runner: QualificationRunner, name: str) -> int | None:
    """Stop a managed process using graceful-then-forced teardown."""
    return runner._stop(name)


def qualification_attempt_processes(runner: QualificationRunner) -> list[dict[str, Any]]:
    """Find descendants that escaped a launcher process group."""
    return runner._attempt_processes()


def qualification_terminate_attempt_orphans(
    runner: QualificationRunner, *, grace_s: float = 1.0
) -> list[dict[str, Any]]:
    """Force-terminate escaped attempt descendants and report survivors."""
    return runner._terminate_attempt_orphans(grace_s=grace_s)


def qualification_gpu_processes(runner: QualificationRunner) -> dict[str, Any]:
    """Snapshot GPU processes using the runner's nvidia-smi accounting."""
    return runner._gpu_processes()


def qualification_write_resource_evidence(
    runner: QualificationRunner,
    manifest: QualificationManifest,
    baseline: Mapping[str, Any],
) -> bool:
    """Assess owned-PID survivors and unexplained GPU memory growth."""
    return runner._write_resource_evidence(manifest, baseline)


def qualification_settle_evidence_files(
    runner: QualificationRunner, manifest: QualificationManifest
) -> None:
    """Wait briefly for descendant logs/evidence to stop changing."""
    runner._settle_evidence_files(manifest)


def _new_suite_dir(attempt_root: Path) -> tuple[str, Path]:
    attempt_root.mkdir(parents=True, exist_ok=True)
    for _ in range(20):
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        suite_id = f"suite-{timestamp}-{os.getpid()}-{uuid.uuid4().hex[:10]}"
        path = attempt_root / suite_id
        try:
            path.mkdir()
            return suite_id, path
        except FileExistsError:
            continue
    raise RuntimeError("could not allocate a unique qualification suite directory")


def _write_json_atomic(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary_name = stream.name
            stream.write(json.dumps(dict(value), indent=2, sort_keys=True) + "\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
        temporary_name = None
    finally:
        if temporary_name is not None:
            try:
                Path(temporary_name).unlink()
            except FileNotFoundError:
                pass


def _run_suite(
    *,
    root: Path,
    attempt_root: Path | None,
    config_path: Path | None,
    artifact_path: Path | None,
    seed: int,
    readiness_timeout_s: float,
    isaac_command: Sequence[str] | str | None,
    humble_command: Sequence[str] | str | None,
    gate_commands: Mapping[str, Sequence[str] | str],
    base_domain_id: int,
) -> QualificationResult:
    root = root.resolve()
    config_path = (
        config_path or root / "simulation/qualification/manipulation-core.json"
    ).resolve()
    config = _json_file(config_path)
    gates = [str(value) for value in config.get("gates", GATES)]
    scenarios = config.get("scenarios", {})
    if not isinstance(scenarios, Mapping):
        raise ValueError("manipulation qualification config scenarios must be an object")
    suite_id, suite_dir = _new_suite_dir(
        (attempt_root or root / "qualification-runs").resolve()
    )
    children: dict[str, Any] = {}
    preliminary = {
        "schema_version": 1,
        "suite_id": suite_id,
        "status": "running",
        "gates": children,
        "created_at": datetime.now(timezone.utc).isoformat(),
    }
    _write_json_atomic(suite_dir / "suite-result.json", preliminary)

    for index, gate in enumerate(gates):
        scenario_value = scenarios.get(gate)
        if not isinstance(scenario_value, str) or not scenario_value:
            children[gate] = {
                "gate": gate,
                "status": "runner-error",
                "reason": "gate has no configured scenario",
            }
            continue
        scenario_name = (
            scenario_value
            if scenario_value.endswith(".json")
            else f"{scenario_value}.json"
        )
        scenario_path = (root / "simulation/scenarios" / scenario_name).resolve()
        domain_id = (int(base_domain_id) + index) % 233
        child = QualificationRunner(
            root=root,
            attempt_root=suite_dir / "gates" / gate,
            config_path=config_path,
            scenario_path=scenario_path,
            artifact_path=artifact_path,
            seed=seed,
            gate=gate,
            readiness_timeout_s=readiness_timeout_s,
            isaac_command=isaac_command,
            humble_command=humble_command,
            gate_commands=(
                {gate: gate_commands[gate]} if gate in gate_commands else {}
            ),
            ros_domain_id=domain_id,
        ).run()
        children[gate] = {
            "gate": gate,
            "status": child.status,
            "attempt_dir": str(child.attempt_dir),
            "ros_domain_id": domain_id,
            "gate_result": dict(child.gate_results.get(gate, {})),
            "exit_codes": dict(child.exit_codes),
        }
        _write_json_atomic(
            suite_dir / "suite-result.json",
            {**preliminary, "gates": children},
        )

    child_statuses = {str(record.get("status")) for record in children.values()}
    if child_statuses == {"verified-pass"} and len(children) == len(gates):
        status = "verified-pass"
    elif "evidence-invalid" in child_statuses:
        status = "evidence-invalid"
    elif "verified-fail" in child_statuses:
        status = "verified-fail"
    elif child_statuses == {"unverified"}:
        status = "unverified"
    else:
        status = "failed"
    suite_result: dict[str, Any] = {
        **preliminary,
        "status": status,
        "gates": children,
        "completed_at": datetime.now(timezone.utc).isoformat(),
    }
    _write_json_atomic(suite_dir / "suite-result.json", suite_result)

    generator = root / "validation/manipulation_contact_sheets.py"
    visual: dict[str, Any] = {
        "status": "evidence-invalid",
        "errors": ["suite contact-sheet generator is missing"],
    }
    if generator.is_file():
        completed = subprocess.run(
            [sys.executable, str(generator), "--suite-dir", str(suite_dir)],
            cwd=root,
            text=True,
            capture_output=True,
            timeout=60.0,
            check=False,
        )
        for candidate in (
            suite_dir / "visual-evidence-result.json",
            suite_dir / "suite-visual-evidence-result.json",
        ):
            if candidate.is_file():
                visual = _json_file(candidate)
                break
        visual["returncode"] = completed.returncode
        if completed.returncode != 0:
            visual["status"] = "evidence-invalid"
            visual.setdefault("errors", []).append(
                completed.stderr.strip() or "suite contact-sheet generation failed"
            )
    suite_result["visual_evidence"] = visual
    if status == "verified-pass" and visual.get("status") != "valid":
        suite_result["status"] = "evidence-invalid"
        status = "evidence-invalid"
    _write_json_atomic(suite_dir / "suite-result.json", suite_result)

    gate_results = {
        gate: dict(record.get("gate_result", record))
        for gate, record in children.items()
    }
    return QualificationResult(suite_dir, status, gate_results, {})


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--attempt-root", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--scenario", type=Path)
    parser.add_argument("--artifact", type=Path)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--gate", default="all")
    parser.add_argument("--base-domain-id", type=int)
    parser.add_argument("--readiness-timeout", type=float, default=30.0)
    parser.add_argument("--isaac-command", help="override Isaac wrapper command")
    parser.add_argument("--humble-command", help="override Humble wrapper command")
    parser.add_argument("--gate-command", action="append", metavar="NAME=COMMAND", help="gate command override; may be repeated")
    parser.add_argument("--manifest-only", "--dry-run", action="store_true")
    args = parser.parse_args(argv)
    gate_commands: dict[str, str] = {}
    for specification in args.gate_command or []:
        name, separator, command = specification.partition("=")
        if not separator or not name or not command:
            parser.error("--gate-command must have the form NAME=COMMAND")
        gate_commands[name] = command
    base_domain_id = (
        args.base_domain_id
        if args.base_domain_id is not None
        else int(os.environ.get("ROS_DOMAIN_ID", "25"))
    )
    if args.gate == "all" and not args.manifest_only:
        result = _run_suite(
            root=args.root,
            attempt_root=args.attempt_root,
            config_path=args.config,
            artifact_path=args.artifact,
            seed=args.seed,
            readiness_timeout_s=args.readiness_timeout,
            isaac_command=args.isaac_command,
            humble_command=args.humble_command,
            gate_commands=gate_commands,
            base_domain_id=base_domain_id,
        )
        print(
            json.dumps(
                {"attempt_dir": str(result.attempt_dir), "status": result.status},
                sort_keys=True,
            )
        )
        return 0 if result.status == "verified-pass" else 1
    runner = QualificationRunner(
        root=args.root,
        attempt_root=args.attempt_root,
        config_path=args.config,
        scenario_path=args.scenario,
        artifact_path=args.artifact,
        seed=args.seed,
        gate=args.gate,
        readiness_timeout_s=args.readiness_timeout,
        isaac_command=args.isaac_command,
        humble_command=args.humble_command,
        gate_commands=gate_commands,
        ros_domain_id=base_domain_id,
    )
    result = runner.run(manifest_only=args.manifest_only)
    print(json.dumps({"attempt_dir": str(result.attempt_dir), "status": result.status}, sort_keys=True))
    return 0 if result.status in {"verified-pass", "manifest-only"} else 1
    return 0 if result.status == "manifest-only" else 1


if __name__ == "__main__":
    raise SystemExit(main())
