# Isolated system-Humble overlay

Build this overlay only with ROS 2 Humble/Python 3.10. Isaac remains in the
locked Python 3.12 environment and communicates with it through DDS.

The overlay contains:

- `tinker_sim_interfaces`: evaluator-only truth messages; it defines no
  lifecycle or scenario services.
- `tinker_sim_bridge`: wheel odometry, a single articulation-command gateway,
  external `ros2_control`, xArm/gripper/pan-tilt facades, deterministic audio
  fixtures, contract guards, and the standard-API scenario runner.

The exact `simulation_interfaces` 1.4.0 and `topic_based_ros2_control` 0.2.0
Debian packages are extracted below `.ros-vendor/humble`; the host installation
is not changed.

```bash
cd /home/tinker/tinker-sim/6.0.1
./scripts/build-humble-overlay
```

Launch Isaac from a clean unsourced terminal:

```bash
export TINKER_ACCEPT_OMNIVERSE_EULA=Y
./scripts/launch-isaac --sensor-profile navigation-parity --scenario \
  find-and-approach-person --seed 7 --ros
```

Then launch external Tinker integration from a second terminal:

```bash
./scripts/launch-humble navigation
# Or after navigation acceptance:
./scripts/launch-humble whole-robot
```

`TINKER_SIM_DDS_PROFILE=local` is the default and retains Fast DDS shared
memory. Set it to `lan` on every participating process to select the committed
UDP-only profile for multi-machine operation.
