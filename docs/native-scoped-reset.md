# Native idle scoped reset

The project-owned adapter in
`simulation/tinker_sim_isaac/scoped_reset.py` implements the standard
`/reset_simulation` TIME (1), STATE (2), and TIME|STATE (3) scopes for the
idle manipulation qualification graph. It is installed by
`validation/run_sim.py::_enable_sim_control_services` before the native
provider registers its callbacks. Default/spawned reset scopes retain the
upstream provider behavior.

This implementation has CPU validation. Native lifecycle acceptance is still
pending. It does not turn historical full-reset failures into successful
scoped-reset evidence.

## State and command boundary

The initial robot baseline is captured once when the native services become
ready, after backend and camera warm-up. It includes the root link pose, root
center-of-mass velocity, joint positions, and joint velocities. Free rigid
bodies present then retain their initial bound transforms and velocities.
Successful standard spawn calls register new bodies without replacing existing
baselines. Unregistered bodies and additional articulations outside the
supported robot are rejected rather than assigned an invented initial state.

The reset observes commanded arm, gripper, pan/tilt and root poses over a
0.25-second wall-clock window. Joint spans must remain within 0.0005 radians,
root component spans within 0.0005 meters, and quaternion distance within
0.0005. All measured positions and velocities must be finite. Passive caster
rotation and noisy mimic-joint velocities do not establish commanded-arm
motion; root-pose observation still detects actual base travel.
The reset also observes arm and gripper action status publishers. An active action or an action arriving while establishing
the stop rejects the reset before restoring physics. The scope is the idle
manipulation qualification graph; active navigation or other external motion
workflows are not qualified by this adapter.

The provider asserts only `/sim/safety/reset`. This is a distinct optional
input to the existing safety supervisor, so clearing it cannot clear
`/sim/safety/operator`. The supervisor uses its existing controller-manager
query and switch path to deactivate the arm controller. Its reset-specific
`/sim/safety/reset_ready` acknowledgement is true only for a confirmed inactive
controller, enabled controller management, an asserted reset source, and no
switch in flight. Isaac's bundled Python ROS distribution has no
`controller_manager_msgs`; the acknowledgement keeps controller queries in
the existing Humble supervisor.

The provider requires that acknowledgement, effective hardware stop, and the
backend's physical hold before pausing the timeline. It never calls timeline
STOP, deletes prims, resets the world, or replaces live PhysX views. The
articulation's reset API clears actuator memory and external wrenches. Direct
articulation and rigid-body writers restore the retained state in place.
Staged snapshots, gripper slew commands, wheel slew state, held targets, effort
and velocity targets, and stale contact records are cleared or reseeded.

The existing command mux clears its pre-stop source commands and starts a new
generation through the existing safety transition. Restored joint measurements
are published while stopped before release. Controller reactivation and a fresh
accepted mux generation are required before success. Other safety sources remain
authoritative.

## Clock, capture, and failures

The backend's internal physics step clock remains monotonic. TIME reset changes
the origin of the existing public ROS clock to the process's initial epoch.
The gateway's sole clock publisher and joint publisher emit that epoch together.
A current-state render occurs after restoration, and camera publication is
suppressed until reset release; subsequent RGB-D publications use the same
gateway clock. STATE-only reset preserves public time and still produces the
visible pause/invalidation boundary.

A 1.2-second pause exceeds the manipulation lifecycle policy's one-second pause
watchdog. This is the actual timeline pause, not an injected clock sample.
The external lifecycle harness must still observe clock behavior, reject the
old exact capture, verify no motion/action dispatch for that rejection, and
accept a fresh capture into the normal planner path.

A reset cannot overlap a provider service mutation. Scoped-reset failures after stop
assertion leave the simulation paused and the provider's reset stop asserted,
and the service error reports both facts explicitly. No timeout silently falls
back to a full reset. Recovery requires resolving the reported cause and
explicitly recovering or restarting the qualification graph; teardown does
not publish a stop clear.

## Validation and remaining native checks

CPU evidence on 2026-09-09:

- 104 focused reset, gateway, supervisor, provider bootstrap, and clock-deadline
  tests passed under the Humble Python environment.
- 181 backend runtime tests and 5 subtests passed under native Python 3.12,
  using bundled Humble message modules. These tests used CPU tensors and mocks,
  not a simulator/GPU process.
- `colcon build --packages-select tinker_sim_bridge` completed: one package in
  5.24 seconds, no stderr; remote job
  `e3b8c5065be7aa21fa9ba1acc3a04210`.
- The installed supervisor was imported and checked for the separate reset
  source and reset acknowledgement.

The focused command is:

```bash
source /opt/ros/humble/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH=simulation:ros2_ws/src/tinker_sim_bridge:$PYTHONPATH
/home/tinker/tk25_ws/src/tk27_manipulation/.venv-tools/bin/python -m pytest \
  tests/test_scoped_reset.py tests/test_set_entity_state_physics.py \
  tests/test_ros_gateway.py tests/test_safety_contract.py \
  tests/test_safety_supervisor_logging.py tests/test_run_sim_services_readiness.py \
  tests/test_gateway_simtime_deadlines.py -q
```

For native launch, source `/opt/ros/humble/setup.bash`, then
`/home/tinker/tinker-sim/6.0.1/ros2_ws/install/setup.bash`, and include
`/home/tinker/tinker-sim/6.0.1/simulation` in `PYTHONPATH`.

The coordinated native run must verify initial baseline capture, service feature
advertisement and scope dispatch, supervisor acknowledgement, actual robot and
spawned-body state restoration without view invalidation, clock rewind followed
by progress, controller reactivation, coherent fresh RGB-D, rejection of an old
capture without motion, and the normal fresh-capture planner response. Repeat
pause/resume and reset qualification with the existing assertions unchanged.
Check explicit rejection for active motion and preservation of an independent
operator stop. CPU success alone does not establish any of these live results.

The first native06 scope-3 attempt was rejected before stop assertion or state
mutation with an idle-observation timeout. A read-only follow-up confirmed both
action status publishers, terminal status 4 for their retained goals, and a
clear effective stop. Over 33 samples in three wall seconds, arm joint spans
were below 0.000009 radians and gripper spans below 0.000066 radians, while the
left finger reported 0.041518 rad/s, an outer knuckle 0.029937 rad/s, and a
passive rear wheel 0.025360 rad/s. The initial blanket 0.02 rad/s threshold
therefore rejected this settled contact state. The pose-window revision has
CPU regressions for those noise values and for actual slow joint/root motion.
The existing native acceptance evaluator limits are unchanged. This failed
attempt remains failed evidence; a new live scoped reset is required.

The subsequent native07 scope-3 attempt established the idle and stop boundary,
then failed because restoration assigned the backend's read-only
`gripper_effort_limit` property. It remained paused with the reset stop asserted.
Restoration now uses the existing backend effort setter, converting saved Nm
back to its configured commanded-force units (N). That setter updates the
implicit actuator model and the live PhysX drive; a failed PhysX write aborts
the reset. The regression uses the real backend property and setter with CPU
tensors and a PhysX-view double, including unequal force/torque scales and
repeated restoration. No other restoration assignment targets a read-only
backend property. Native07's failed attempt is preserved; this fix still
requires a new live reset.
Follow-up CPU verification passed 182 runtime tests plus 7 subtests and 26
scoped-reset tests. No simulator was launched for those checks.
