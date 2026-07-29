# Navigation-first Isaac integration

Tinker's existing Gazebo/Nav2 stack remains the reference regression backend.
Isaac replaces only the simulator-facing boundary:

| Owner | Interface |
| --- | --- |
| Isaac Python 3.12 | `/clock`, `/isaac_joint_states`, `/livox/lidar`, `/livox/imu` |
| Command gateway | sole publisher of `/isaac_joint_commands` |
| Base facade | `/cmd_vel` to four wheel targets; wheel-derived `/tracer/odom` |
| Point-cloud adapter | `/livox/lidar` to `/scan` |
| EKF | `odom -> base_link` from parity odometry |
| AMCL | `map -> odom` |
| Robot-state publisher | robot joint/link transforms |

The `navigation-parity` profile uses deterministic CPU raycasting only for
fast behavior development. Sensor-rich and qualification profiles use Isaac's
RTX LiDAR graph; the development cloud is not a parity claim.

Build and launch:

```bash
cd /home/tinker/tinker-sim/6.0.1
./scripts/build-humble-overlay

# Terminal A: no system ROS sourced
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
./scripts/launch-isaac --sensor-profile navigation-parity --profile parity \
  --scenario find-and-approach-person --seed 7 --ros

# Terminal B
./scripts/launch-humble navigation
```

Standard lifecycle examples:

```bash
ros2 service call /set_simulation_state \
  simulation_interfaces/srv/SetSimulationState \
  '{state: {state: 2}}'
ros2 service call /step_simulation \
  simulation_interfaces/srv/StepSimulation '{}'
ros2 service call /set_simulation_state \
  simulation_interfaces/srv/SetSimulationState \
  '{state: {state: 1}}'
```

The contract guard fails on legacy `/sim/control/*` or `/sim/scenario/*`
services, duplicate articulation-command publishers, missing standard services,
wrong topic types, or conflicting dynamic TF ownership.
