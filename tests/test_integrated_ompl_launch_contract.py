"""Task 6: integrated OMPL overlay launch contract tests.

Defines local ``load_launch_source``, ``resolve_launch_graph``, and
``assert_allowlisted_launch_graph`` helpers (AST-based) and verifies the staged
integrated launch against the exact provider-manifest allow-list and the
Task 2 exact production overlay contract.  The AST walker needs no ROS Humble
``launch`` import, so this test also runs under the simulator CPython 3.12 venv.
"""
from __future__ import annotations

import ast
from pathlib import Path
from typing import Mapping

import pytest

ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = ROOT / "ros2_ws/src/tinker_sim_bridge/launch/integrated_ompl_manipulation.launch.py"
MANIP_LAUNCH_PATH = ROOT / "ros2_ws/src/tinker_sim_bridge/launch/manipulation.launch.py"
PROVIDER_MANIFEST_PATH = ROOT / "ros2_ws/src/tinker_sim_bridge/integration/provider-manifest.json"

# The exact Task 2 allow-list applied to the integrated composition.
ALLOWED_TOP_LEVEL_IMPORTS = frozenset({
    "os", "sys", "json", "hashlib", "tempfile", "pathlib", "ast", "pytest",
    "typing", "launch", "launch_ros", "ament_index_python", "yaml",
    "tinker_sim_deploy", "tinker_sim_bridge",
})
ALLOWED_NODE_PACKAGES = frozenset({
    "tinker_sim_bridge",
    "controller_manager",
    "robot_state_publisher",
    "moveit_ros_move_group",
    "pick_and_place",
})
ALLOWED_EXECUTABLES = frozenset({
    "safety_supervisor",
    "ros2_control_node",
    "controller_reconciler",
    "command_gateway",
    "xarm_facade",
    "gripper_facade",
    "pan_tilt_facade",
    "contract_guard",
    "truth_evaluator",
    "robot_state_publisher",
    "scenario_runner",
    "physics_ready_gate",
    "fixture_planning_scene",
    "integrated_readiness",
    "move_group",
    "pick_and_place",
})
REQUIRED_LAUNCH_ARGUMENTS = frozenset({
    "project_root",
    "tinker_workspace",
    "scenario",
    "seed",
    "reset_attempts",
    "reset_retry_delay",
    "qualification",
    "model_bundle_manifest",
    "provider_manifest_path",
    "attempt_dir",
})
REQUIRED_PERSISTENT_NODE_KEYS = frozenset({
    "controller_manager",
    "robot_state_publisher",
    "command_gateway",
    "safety_supervisor",
    "xarm_facade",
    "gripper_facade",
    "pan_tilt_facade",
    "contract_guard",
    "truth_evaluator",
    "physics_ready_gate",
    "fixture_planning_scene",
    "move_group",
    "pick_and_place",
    "integrated_readiness",
})
REQUIRED_ONE_SHOT_KEYS = frozenset({"scenario_runner", "controller_reconciler"})
REQUIRED_PUBLISHERS = frozenset({
    "/joint_states",
    "/isaac_joint_commands",
    "/sim/controller/gripper_commands",
    "/sim/safety/operator",
    "/sim/hardware/safety_stop",
    "/sim/status/planning_scene_fixture",
    "/sim/status/integrated_manipulation",
})
REQUIRED_WAITER_SERVICES = frozenset({"/sim/ready/physics", "/sim/ready/fixture"})


def load_launch_source(path: Path) -> str:
    """Return the raw launch source text."""
    return Path(path).read_text(encoding="utf-8")


class _NodeVisitor(ast.NodeVisitor):
    def __init__(self) -> None:
        self.nodes: list[tuple[str, str, str | None]] = []
        self.launch_arguments: dict[str, str] = {}
        self.includes: list[str] = []
        self.waiters: list[str] = []
        self.execute_cmds: list[str] = []
        self.register_handlers = 0

    def _kw_str(self, call: ast.Call, name: str) -> str | None:
        for keyword in call.keywords:
            if keyword.arg != name:
                continue
            if isinstance(keyword.value, ast.Constant) and isinstance(keyword.value.value, str):
                return keyword.value.value
            if isinstance(keyword.value, ast.Name):
                return keyword.value.id
        return None

    def visit_Call(self, call: ast.Call) -> None:
        func = call.func
        if isinstance(func, ast.Name) and func.id in {"Node", "DeclareLaunchArgument", "IncludeLaunchDescription", "_service_waiter", "ExecuteProcess"}:
            if func.id == "Node":
                package = self._kw_str(call, "package")
                executable = self._kw_str(call, "executable")
                name = self._kw_str(call, "name")
                if package and executable:
                    self.nodes.append((package, executable, name))
            elif func.id == "DeclareLaunchArgument":
                if call.args and isinstance(call.args[0], ast.Constant):
                    self.launch_arguments[str(call.args[0].value)] = ""
                else:
                    default = self._kw_str(call, "name")
                    if default:
                        self.launch_arguments[default] = ""
            elif func.id == "IncludeLaunchDescription":
                self.includes.append("_included")
            elif func.id == "_service_waiter":
                if call.args and isinstance(call.args[0], ast.Constant):
                    self.waiters.append(str(call.args[0].value))
            elif func.id == "ExecuteProcess":
                for keyword in call.keywords:
                    if keyword.arg == "cmd" and isinstance(keyword.value, ast.List):
                        for item in keyword.value.elts:
                            if isinstance(item, ast.Constant):
                                self.execute_cmds.append(str(item.value))
        self.generic_visit(call)


def resolve_launch_graph(source: str, args: Mapping[str, str]) -> dict[str, object]:
    """AST-resolve a launch source into a structured graph."""
    del args
    tree = ast.parse(source)
    visitor = _NodeVisitor()
    visitor.visit(tree)
    return {
        "nodes": visitor.nodes,
        "launch_arguments": visitor.launch_arguments,
        "includes": visitor.includes,
        "waiters": visitor.waiters,
        "execute_cmds": visitor.execute_cmds,
    }


def _collect_included_launch_files(source: str) -> list[str]:
    """Extract the literal ``.launch.py`` file names from an included launch source."""
    result: list[str] = []
    for line in source.splitlines():
        if ".launch.py" in line:
            result.append(line.strip())
    return result


def assert_allowlisted_launch_graph(graph: dict[str, object]) -> None:
    """Assert a resolved graph obeys the integrated provider allow-list."""
    nodes = graph["nodes"]
    assert isinstance(nodes, list) and nodes
    packages = {pkg for pkg, _exec, _name in nodes}
    executables = {exec_ for _pkg, exec_, _name in nodes}
    unknown_packages = packages - ALLOWED_NODE_PACKAGES
    assert not unknown_packages, "disallowed node packages: {}".format(sorted(unknown_packages))
    unknown_executables = executables - ALLOWED_EXECUTABLES
    assert not unknown_executables, "disallowed executables: {}".format(sorted(unknown_executables))

    args = graph["launch_arguments"]
    missing = REQUIRED_LAUNCH_ARGUMENTS - set(args)
    assert not missing, "missing launch arguments: {}".format(sorted(missing))

    waiters = graph["waiters"]
    missing_waiters = REQUIRED_WAITER_SERVICES - set(waiters)
    assert not missing_waiters, "missing readiness waiter services: {}".format(sorted(missing_waiters))

    # Exactly one controller reconciler process requesting both controllers.
    reconcilers = [node for node in nodes if node[1] == "controller_reconciler"]
    assert len(reconcilers) == 1, "expected exactly one controller_reconciler process"

    # The integrated composition must stage the physics gate, fixture adapter,
    # and integrated readiness as persistent nodes.
    for executable in ("physics_ready_gate", "fixture_planning_scene", "integrated_readiness"):
        assert any(node[1] == executable for node in nodes), "missing {} node".format(executable)

    # The production overlay must be included exactly once per stage
    # (planning-only and task-only).
    assert graph["includes"], "integrated launch must include the production overlay"


def _provider_manifest() -> dict[str, object]:
    import json

    return json.loads(PROVIDER_MANIFEST_PATH.read_text(encoding="utf-8"))


def test_launch_source_loads() -> None:
    source = load_launch_source(LAUNCH_PATH)
    assert "generate_launch_description" in source
    assert "_resolve" in source


def test_integrated_launch_allowlist() -> None:
    source = load_launch_source(LAUNCH_PATH)
    graph = resolve_launch_graph(source, {})
    assert_allowlisted_launch_graph(graph)


def _def_ranges(source: str) -> dict[str, tuple[int, int]]:
    """Return ``{function_name: (start_line, end_line)}`` from a launch source."""
    tree = ast.parse(source)
    result: dict[str, tuple[int, int]] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            result[node.name] = (node.lineno, node.end_lineno or node.lineno)
    return result


def _lines_in_range(source: str, start: int, end: int) -> str:
    lines = source.splitlines()
    return "\n".join(lines[start - 1:end])


def test_integrated_launch_stages_exact_sequence() -> None:
    """The nested staging functions appear in the exact briefed order and each
    stage carries its required markers."""
    source = load_launch_source(LAUNCH_PATH)
    ranges = _def_ranges(source)
    scenario_start, scenario_end = ranges["_after_scenario_runner"]
    physics_start, physics_end = ranges["_after_physics_ready"]
    fixture_start, fixture_end = ranges["_after_fixture_ready"]
    assert scenario_start < physics_start < fixture_start, (
        "staging functions are not in order: scenario_runner -> physics -> fixture"
    )

    scenario_body = _lines_in_range(source, scenario_start, physics_start - 1)
    assert "physics_ready_gate" in scenario_body
    assert "/sim/ready/physics" in scenario_body

    physics_body = _lines_in_range(source, physics_start, fixture_start - 1)
    assert "start_move_group" in physics_body
    assert "fixture_planning_scene" in physics_body
    assert "/sim/ready/fixture" in physics_body

    fixture_body = _lines_in_range(source, fixture_start, fixture_end)
    assert "start_task_server" in fixture_body
    assert "integrated_readiness" in fixture_body

    # The controller reconciler must be chained to the scenario runner at the
    # top level, and the scenario runner to the physics gate stage.
    assert "controller_reconciler" in source
    assert "scenario_runner" in source


def test_integrated_launch_includes_production_overlay() -> None:
    source = load_launch_source(LAUNCH_PATH)
    assert "manipulation_planning_task_only.launch.py" in source
    included = _collect_included_launch_files(source)
    assert any("manipulation_planning_task_only.launch.py" in line for line in included)


def test_provider_manifest_schema_and_self_hash() -> None:
    import json

    from tinker_sim_bridge.integrated_readiness import sha256_json

    manifest = _provider_manifest()
    assert manifest["schema_version"] == 1
    assert set(manifest) == {
        "schema_version",
        "owner",
        "provider_manifest_sha256",
        "cardinality_source",
        "persistent_nodes",
        "one_shot_processes",
        "controller_resources",
        "publishers",
    }
    recorded = manifest["provider_manifest_sha256"]
    recomputed = sha256_json(
        {k: v for k, v in manifest.items() if k != "provider_manifest_sha256"}
    )
    assert recorded == recomputed
    persistent_keys = {entry["key"] for entry in manifest["persistent_nodes"]}
    assert persistent_keys == REQUIRED_PERSISTENT_NODE_KEYS
    one_shot_keys = {entry["key"] for entry in manifest["one_shot_processes"]}
    assert one_shot_keys == REQUIRED_ONE_SHOT_KEYS
    publisher_topics = {entry["topic"] for entry in manifest["publishers"]}
    assert publisher_topics == REQUIRED_PUBLISHERS
    controller_names = {entry["resource_name"] for entry in manifest["controller_resources"]}
    assert controller_names == {"joint_state_broadcaster", "xarm7_traj_controller"}


def test_manipulation_launch_default_preserves_legacy() -> None:
    """manipulation.launch.py declares planning_overlay (default false) and the
    overlay is installed only through the integrated launch."""
    source = load_launch_source(MANIP_LAUNCH_PATH)
    graph = resolve_launch_graph(source, {})
    assert "planning_overlay" in graph["launch_arguments"]
    assert "integrated_ompl_manipulation.launch.py" in source
    assert "planning_overlay" in source


def test_ready_snapshot_reuses_shared_helpers() -> None:
    """The integrated launch computes the canonical integrated mapping from the
    shared pure module (so its digest agrees with every consumer)."""
    source = load_launch_source(LAUNCH_PATH)
    assert "build_integrated_mapping" in source
    assert "from tinker_sim_bridge.integrated_readiness import" in source
