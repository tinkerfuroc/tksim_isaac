from __future__ import annotations

from functools import partial
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class SafetySourceTracker:
    """Track one required source with a wall-clock freshness deadline.

    A required source is stopped until it has supplied an explicit sample. A
    false sample clears the source only for the configured freshness window;
    after that window, the source is stopped again until another false sample
    arrives. Wall-clock time is intentional here because this is a transport
    liveness contract, not simulation time.
    """

    def __init__(self, deadline_s: float) -> None:
        if deadline_s <= 0.0:
            raise ValueError("safety source deadline must be positive")
        self.deadline_s = float(deadline_s)
        self.value: bool | None = None
        self.received_at: float | None = None

    def update(self, value: bool, received_at: float) -> None:
        self.value = bool(value)
        self.received_at = float(received_at)

    def requires_stop(self, now: float) -> bool:
        if self.value is None or self.received_at is None:
            return True
        return self.value or now - self.received_at >= self.deadline_s


class SafetySupervisor(Node):
    """Own the effective stop and the trajectory-controller lifecycle.

    ``/sim/hardware/safety_stop`` is a transient-local, reliable heartbeat:
    this node republishes the effective Bool every reconciliation period
    (0.25 s), and consumers must fail closed when no sample is received for
    ``required_source_deadline_s``. Consumer integration points are the
    safety-stop subscriptions in ``command_gateway.py``, ``backend.py``, and
    ``gripper_facade.py``; those consumers should use the same deadline and
    treat supervisor loss as an active stop.
    """

    REQUIRED_SOURCES = ("xarm", "collision")
    OPTIONAL_SOURCES = ("operator",)
    SOURCES = {
        "xarm": "/sim/safety/xarm",
        "collision": "/sim/safety/collision",
        "operator": "/sim/safety/operator",
    }

    def __init__(self) -> None:
        super().__init__("tinker_sim_safety_supervisor")
        self.declare_parameter("controller", "xarm7_traj_controller")
        self.declare_parameter("controller_management_ready", False)
        self.declare_parameter("required_source_deadline_s", 1.0)
        self._controller = str(self.get_parameter("controller").value)
        self._required_source_deadline_s = float(
            self.get_parameter("required_source_deadline_s").value
        )
        if self._required_source_deadline_s <= 0.0:
            raise ValueError("required_source_deadline_s must be positive")
        # Required sources start unknown until their transient-local state is
        # received. Optional operator input defaults clear until it is used.
        self._sources: dict[str, bool | None] = {
            name: None for name in self.REQUIRED_SOURCES
        }
        self._sources.update({name: False for name in self.OPTIONAL_SOURCES})
        self._source_trackers = {
            name: SafetySourceTracker(self._required_source_deadline_s)
            for name in self.REQUIRED_SOURCES
        }
        self._desired_stop = True
        self._published_stop: bool | None = None
        self._management_ready = False
        self._startup_hold = True
        self._controller_active: bool | None = None
        self._controller_was_active: bool | None = None
        self._restore_pending = False
        self._stop_episode_recorded = False
        self._controllers_inflight = False
        self._switch_inflight = False
        source_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        stop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._stop = self.create_publisher(
            Bool, "/sim/hardware/safety_stop", stop_qos
        )
        for name, topic in self.SOURCES.items():
            self.create_subscription(
                Bool, topic, partial(self._source, name), source_qos
            )
        self._controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._switch = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.create_timer(
            0.25,
            self._reconcile,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self._publish(True)

    def _source(self, name: str, message: Bool) -> None:
        self._sources[name] = bool(message.data)
        if name in self._source_trackers:
            self._source_trackers[name].update(bool(message.data), time.monotonic())
        self._reconcile()

    def _refresh_desired_stop(self) -> None:
        desired = any(
            tracker.requires_stop(time.monotonic())
            for tracker in self._source_trackers.values()
        ) or any(self._sources[name] for name in self.OPTIONAL_SOURCES)
        if desired == self._desired_stop:
            return
        if desired:
            # A new stop must never discard an activation which is still in
            # flight. Keep the prior baseline until list_controllers confirms
            # the post-stop state.
            self._startup_hold = True
            if self._controller_was_active is None and self._controller_active is not None:
                self._controller_was_active = self._controller_active is True
            # If state is unknown, let the next list_controllers response
            # establish the baseline instead of recording a false one.
            self._stop_episode_recorded = self._controller_was_active is not None
        else:
            self._restore_pending = self._restore_pending or self._controller_was_active is True
        self._desired_stop = desired

    def _publish_effective(self) -> None:
        active = (
            self._desired_stop
            or not self._management_ready
            or self._startup_hold
            or self._restore_pending
        )
        # Repeated publication is the supervisor liveness heartbeat. A
        # consumer that misses this stream must assert its own stop.
        self._publish(active)

    def _publish(self, active: bool) -> None:
        message = Bool()
        message.data = active
        self._stop.publish(message)
        self._published_stop = active

    def _reconcile(self) -> None:
        self._refresh_desired_stop()
        management_ready = bool(
            self.get_parameter("controller_management_ready").value
        )
        if management_ready != self._management_ready:
            self._management_ready = management_ready
            if not management_ready:
                self._startup_hold = True
        self._publish_effective()
        if not self._management_ready:
            return
        if self._controllers_inflight or self._switch_inflight:
            return
        try:
            ready = self._controllers.service_is_ready()
        except Exception as error:
            self.get_logger().error(
                f"controller manager list readiness check failed: {error}"
            )
            return
        if not ready:
            return
        try:
            future = self._controllers.call_async(ListControllers.Request())
        except Exception as error:
            self.get_logger().error(f"controller state request failed: {error}")
            return
        self._controllers_inflight = True
        future.add_done_callback(self._controllers_listed)

    def _controllers_listed(self, future) -> None:
        self._controllers_inflight = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"controller state query failed: {error}")
            return
        if response is None:
            self.get_logger().error("controller manager returned no controller state")
            return
        controller = next(
            (
                item
                for item in getattr(response, "controller", ())
                if str(getattr(item, "name", "")) == self._controller
            ),
            None,
        )
        if controller is None:
            self._controller_active = None
            self.get_logger().warning(
                f"configured controller is not listed: {self._controller}"
            )
            return
        active = str(getattr(controller, "state", "")).lower() == "active"
        self._controller_active = active
        if self._desired_stop and not self._stop_episode_recorded:
            # The initial fail-safe stop also needs an episode baseline.
            self._controller_was_active = active
            self._stop_episode_recorded = True

        if self._desired_stop:
            self._startup_hold = True
        elif self._restore_pending:
            if active:
                # Do not clear the stop until the controller manager has
                # confirmed that the restoration request actually took effect.
                self._restore_pending = False
                self._controller_was_active = None
                self._stop_episode_recorded = False
                self._startup_hold = False
                self._publish_effective()
            elif not active:
                self._request_switch(activate=True)
        elif active:
            # This is either the first successful post-spawner observation or
            # a controller which did not need restoration.
            self._startup_hold = False
            self._publish_effective()
        else:
            # An inactive controller is never activated by the safety node
            # without a confirmed pre-stop active baseline.
            self._startup_hold = True
            self._publish_effective()

        if self._desired_stop and active:
            self._request_switch(activate=False)

    def _request_switch(self, *, activate: bool) -> None:
        try:
            ready = self._switch.service_is_ready()
        except Exception as error:
            self.get_logger().error(f"controller manager readiness check failed: {error}")
            return
        if not ready:
            return
        request = SwitchController.Request()
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = Duration(seconds=2.0).to_msg()
        if activate:
            request.activate_controllers = [self._controller]
        else:
            request.deactivate_controllers = [self._controller]
        try:
            future = self._switch.call_async(request)
        except Exception as error:
            self.get_logger().error(f"controller switch request failed: {error}")
            return
        self._switch_inflight = True
        future.add_done_callback(partial(self._switched, activate=activate))

    def _switched(self, future, *, activate: bool) -> None:
        self._switch_inflight = False
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"controller switch failed: {error}")
            return
        if response is None or not response.ok:
            self.get_logger().error("controller manager rejected safety switch")
            return
        # The switch response is only an acknowledgement. The next
        # list_controllers response is the source of truth for actual state.
        # In particular, an activation acknowledgement must not clear the
        # restoration intent while a stop transition may be racing it.
        self._controller_active = None
        self._reconcile()


def main() -> None:
    rclpy.init()
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
