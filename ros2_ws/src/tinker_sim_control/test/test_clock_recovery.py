"""Real controller-manager recovery under a paused and rewound ROS clock."""
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import LoadController, ConfigureController, SwitchController
import yaml

URDF = """<robot name="clock_test"><link name="base"/><link name="tip"/>
<joint name="joint" type="revolute"><parent link="base"/><child link="tip"/>
<axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="10" velocity="2"/></joint>
<ros2_control name="mock" type="system"><hardware>
<plugin>mock_components/GenericSystem</plugin></hardware><joint name="joint">
<command_interface name="position"/><state_interface name="position">
<param name="initial_value">0</param></state_interface><state_interface name="velocity"/>
</joint></ros2_control></robot>"""

def test_controller_recovers_after_rewind_and_holds_paused_clock(tmp_path):
    executable = os.environ["CONTROLLER_MANAGER_EXECUTABLE"]
    config = {
        "controller_manager": {"ros__parameters": {
            "use_sim_time": True, "update_rate": 100, "robot_description": URDF,
            "state": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
            "arm": {"type": "joint_trajectory_controller/JointTrajectoryController"}}},
        "arm": {"ros__parameters": {"joints": ["joint"],
            "command_interfaces": ["position"],
            "state_interfaces": ["position", "velocity"]}}}
    params = tmp_path / "controllers.yaml"
    params.write_text(yaml.safe_dump(config))
    log = (tmp_path / "manager.log").open("w")
    process = subprocess.Popen([executable, "--ros-args", "--params-file", str(params)],
                               stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node("clock_recovery_test")
    publisher = node.create_publisher(Clock, "/clock", 10)
    samples = []
    node.create_subscription(JointState, "/joint_states",
        lambda msg: samples.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9),
        qos_profile_sensor_data)
    stamp = 1000.0
    def pump(duration=.02, advance=True):
        nonlocal stamp
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if advance:
                stamp += .01
            message = Clock()
            message.clock.sec = int(stamp)
            message.clock.nanosec = int((stamp - int(stamp)) * 1e9)
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=.01)
    def call(service_type, name, request):
        client = node.create_client(service_type, "/controller_manager/" + name)
        deadline = time.monotonic() + 8
        while not client.service_is_ready() and time.monotonic() < deadline:
            pump()
        assert client.service_is_ready(), (tmp_path / "manager.log").read_text()
        future = client.call_async(request)
        deadline = time.monotonic() + 4
        while not future.done() and time.monotonic() < deadline:
            pump()
        assert future.done(), (tmp_path / "manager.log").read_text()
        result = future.result()
        node.destroy_client(client)
        assert result.ok, (tmp_path / "manager.log").read_text()
    def switch(start=(), stop=()):
        request = SwitchController.Request()
        request.activate_controllers = list(start)
        request.deactivate_controllers = list(stop)
        request.strictness = 2
        request.timeout.sec = 2
        call(SwitchController, "switch_controller", request)
    try:
        for name in ("state", "arm"):
            request = LoadController.Request(); request.name = name
            call(LoadController, "load_controller", request)
            request = ConfigureController.Request(); request.name = name
            call(ConfigureController, "configure_controller", request)
        switch(("state", "arm"))
        pump(.3)
        assert samples and samples[-1] > 1000
        pump(.2, advance=False)
        count = len(samples)
        pump(.2, advance=False)
        assert len(samples) == count, "Paused simulation clock advanced the controller"
        stamp = 100.0
        pump(.3)
        assert samples[-1] < 200, "Joint publication did not resume in the new epoch"
        switch(stop=("arm",))
        switch(start=("arm",))
        pump(.2)
        assert samples[-1] < 200
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait(timeout=5)
        log.close()
