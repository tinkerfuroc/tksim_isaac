from setuptools import find_packages, setup

package_name = "tinker_sim_bridge"

# The canonical OMPL acceptance contract and the full 17-scenario integrated
# matrix live in the simulator checkout (integration/ and simulation/scenarios/),
# outside the bridge package directory.  They are exposed to the build through
# tracked source symlinks under integration/ and scenarios/ so the build's
# ``data_files`` sources stay relative (colcon rejects absolute sources) while
# the canonical source bytes remain the single authoritative copy.  The
# provenance suite verifies installed bytes equal the canonical source bytes.
_scenario_sources = [
    "scenarios/" + name
    for name in (
        "qualification-moveit-plan-joint.json",
        "qualification-moveit-plan-pose.json",
        "qualification-moveit-execute-joint.json",
        "qualification-moveit-execute-pose.json",
        "qualification-moveit-cartesian-retreat.json",
        "qualification-moveit-gripper.json",
        "qualification-moveit-cancel.json",
        "qualification-moveit-safety.json",
        "qualification-pick-place-positive.json",
        "qualification-pick-place-blocked-approach.json",
        "qualification-pick-place-unreachable-grasp.json",
        "qualification-pick-place-malformed-back.json",
        "qualification-pick-place-cancel-approach.json",
        "qualification-pick-place-cancel-transport.json",
        "qualification-pick-place-safety-transport.json",
        "qualification-pick-place-occupied-place.json",
    )
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/fixtures.launch.py",
                "launch/integrated_ompl_manipulation.launch.py",
                "launch/manipulation.launch.py",
                "launch/navigation.launch.py",
                "launch/whole_robot.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/base_facade.yaml",
                "config/command_gateway.yaml",
                "config/controllers.yaml",
                "config/pointcloud_to_laserscan.yaml",
                "config/tinker_topic_control.ros2_control.xacro",
            ],
        ),
        (
            "share/" + package_name + "/integration",
            [
                "integration/provider-manifest.json",
                "integration/ompl-overlay-contract.json",
            ],
        ),
        (
            "share/" + package_name + "/scenarios",
            _scenario_sources,
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tinker Team",
    maintainer_email="tinker@example.invalid",
    description="External Humble hardware-parity gateways for Tinker simulation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "audio_fixtures = tinker_sim_bridge.audio_fixtures:main",
            "base_facade = tinker_sim_bridge.base_facade:main",
            "command_gateway = tinker_sim_bridge.command_gateway:main",
            "contract_guard = tinker_sim_bridge.contract_guard:main",
            "controller_reconciler = tinker_sim_bridge.controller_reconciler:main",
            "fixture_planning_scene = tinker_sim_bridge.fixture_planning_scene_node:main",
            "gripper_facade = tinker_sim_bridge.gripper_facade:main",
            "initial_pose = tinker_sim_bridge.initial_pose:main",
            "integrated_readiness = tinker_sim_bridge.integrated_readiness_node:main",
            "joint_state_probe = tinker_sim_bridge.joint_state_probe:main",
            "model_bundle = tinker_sim_bridge.model_bundle:main",
            "model_limits = tinker_sim_bridge.model_limits:main",
            "model_preflight = tinker_sim_bridge.model_preflight:main",
            "pan_tilt_facade = tinker_sim_bridge.pan_tilt_facade:main",
            "physics_ready_gate = tinker_sim_bridge.physics_ready_gate:main",
            "readiness_waiter = tinker_sim_bridge.readiness_waiter:main",
            "scenario_runner = tinker_sim_bridge.scenario_runner:main",
            "safety_supervisor = tinker_sim_bridge.safety_supervisor:main",
            "truth_evaluator = tinker_sim_bridge.truth_evaluator:main",
            "xarm_facade = tinker_sim_bridge.xarm_facade:main",
        ]
    },
)
