from setuptools import find_packages, setup

package_name = "tinker_sim_bridge"

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
            "gripper_facade = tinker_sim_bridge.gripper_facade:main",
            "initial_pose = tinker_sim_bridge.initial_pose:main",
            "model_bundle = tinker_sim_bridge.model_bundle:main",
            "model_limits = tinker_sim_bridge.model_limits:main",
            "model_preflight = tinker_sim_bridge.model_preflight:main",
            "pan_tilt_facade = tinker_sim_bridge.pan_tilt_facade:main",
            "scenario_runner = tinker_sim_bridge.scenario_runner:main",
            "safety_supervisor = tinker_sim_bridge.safety_supervisor:main",
            "truth_evaluator = tinker_sim_bridge.truth_evaluator:main",
            "xarm_facade = tinker_sim_bridge.xarm_facade:main",
        ]
    },
)
