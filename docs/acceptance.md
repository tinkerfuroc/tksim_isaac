# Release acceptance checklist

## Environment and provenance

- Fresh online bootstrap succeeds without skip flags.
- Fresh offline bootstrap succeeds from an empty destination.
- A second bootstrap makes no dependency or source changes.
- Two servers report identical package versions and release hashes.
- Isaac Lab `HEAD`, Git tree, and clean status match `release-manifest.json`.
- Isaac `sys.path` contains no `/opt/ros/humble` or Python 3.10 package path.

## GPU and streaming

- Compatibility checker exits successfully.
- Headless PhysX completes 10,000 steps.
- RTX camera and LiDAR each produce a GPU-backed frame/buffer.
- NVENC hardware encode succeeds.
- A fresh temporary environment synchronizes with `--frozen --offline` from
  the populated versioned cache.
- One viewer connects through trusted LAN/VPN and a second viewer is rejected.
- NVIDIA's native Ubuntu Omniverse Streaming Client remains connected for at
  least 60 continuous seconds, then client and simulator terminate cleanly.

## ROS boundary

- `/clock`, TF, `cmd_vel`, joint trajectory, camera, and LiDAR data cross the
  Python 3.12/Python 3.10 DDS boundary.
- External FJT and custom Tinker actions control the hardware facade.
- NVIDIA's standard `simulation_interfaces` services and `/simulate_steps`
  action are present; legacy `/sim/control/*` and `/sim/scenario/*` aliases are
  absent.
- The command gateway is the only `/isaac_joint_commands` publisher.
- Each dynamic TF edge has exactly one owner.
- No non-evaluator node can subscribe to `/sim/truth/*`.

## Simulation

- Gazebo and Isaac pass the same base contract suite.
- Parity odometry is scored against hidden truth.
- Arm tracking, collision, effort/contact, safety stops, and retained objects
  are scored.
- A claimed task success fails whenever a hidden world postcondition fails.
- Person approach, pick/deliver/place, and reception/seat assignment each pass
  deterministic seeded scenarios.
