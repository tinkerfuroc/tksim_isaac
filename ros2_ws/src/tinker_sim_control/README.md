# Native simulation controller scheduling

The manipulation launch uses the standard `controller_manager` library through
this C++17 runner. A steady-clock wakeup observes simulation time once per
configured update period. Paused simulation time does not advance the control
loop; backward jumps rebase its deadline and resume with a positive period.

The Humble 2.50.0 `ros2_control_node` runner sleeps on an absolute ROS-time
deadline. A scoped TIME reset can leave that deadline in the previous epoch,
blocking hardware read, controller update, hardware write, and reactivation.
This package owns the simulator scheduling policy without replacing controller
plugins, their services, or their action interfaces.

Validation on 2026-09-10: three schedule unit cases and a ROS integration test
using `mock_components/GenericSystem`, `joint_state_broadcaster`, and the joint
trajectory controller passed. The same integration test failed with the stock
runner after rewinding the published clock from 1000 seconds to 100 seconds.
The test also checks paused time and controller reactivation after rewind.
Full Isaac scoped-reset acceptance is recorded separately; these tests alone
do not establish physical reset acceptance.
