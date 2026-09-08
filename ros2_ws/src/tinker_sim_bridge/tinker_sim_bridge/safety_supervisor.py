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

from tinker_sim_core.observability import format_duration
from tinker_sim_core.safety_gating import effective_stop


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
        self.declare_parameter("manage_controllers", True)
        self.declare_parameter("required_sources", list(self.REQUIRED_SOURCES))
        self._controller = str(self.get_parameter("controller").value)
        self._required_source_deadline_s = float(
            self.get_parameter("required_source_deadline_s").value
        )
        if self._required_source_deadline_s <= 0.0:
            raise ValueError("required_source_deadline_s must be positive")
        self._manage_controllers = bool(self.get_parameter("manage_controllers").value)
        required_sources = list(self.get_parameter("required_sources").value)
        for name in required_sources:
            if name not in self.SOURCES:
                raise ValueError(f"unknown required safety source: {name}")
        # Shadow the class constant with the parameter-derived tuple so the
        # dict comprehensions below (and any future reference to
        # self.REQUIRED_SOURCES) build from the configured sources rather
        # than the manipulation-default constant.
        self.REQUIRED_SOURCES = tuple(required_sources)
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
        # #33 observability: last-seen requires_stop() per required source, so
        # a flip can be reported without changing _refresh_desired_stop's
        # existing computation.
        self._source_stop_state: dict[str, bool] = {}
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

    def _log_source_transitions(self) -> None:
        """Observability only (#33): announce when a required source's
        requires_stop() state flips. _desired_stop below is the OR of every
        source plus optional operator input, so on its own it cannot say
        which source moved; this reads each tracker again (pure, no side
        effect) purely to report the per-source edge.
        """
        for name, tracker in self._source_trackers.items():
            now = time.monotonic()
            current = tracker.requires_stop(now)
            previous = self._source_stop_state.get(name)
            if previous is not None and previous != current:
                # ``received_at`` is only ``None`` before this source's very
                # first sample ever, which cannot itself flip requires_stop()
                # from a prior state -- but the "recovered" transition after
                # a heartbeat that landed while this tracker was still fresh
                # from an even earlier sample degrades the same way, so this
                # never assumes a start time exists.
                age = (
                    now - tracker.received_at
                    if tracker.received_at is not None
                    else None
                )
                self.get_logger().info(
                    "safety_source source=%s state=%s age_s=%s deadline_s=%.3f"
                    % (
                        name,
                        "expired" if current else "recovered",
                        format_duration(age),
                        tracker.deadline_s,
                    )
                )
            self._source_stop_state[name] = current

    def _stop_reasons(self) -> list[str]:
        """Observability only (#33): the source names presently requiring a
        stop, for the safety_stop_published line. An empty list means the
        active value (if True) comes from a controller-management hold
        rather than any source.
        """
        now = time.monotonic()
        reasons = [
            name
            for name, tracker in self._source_trackers.items()
            if tracker.requires_stop(now)
        ]
        reasons.extend(name for name in self.OPTIONAL_SOURCES if self._sources.get(name))
        return reasons

    def _refresh_desired_stop(self) -> None:
        self._log_source_transitions()
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
        if self._manage_controllers:
            # Managed mode (manipulation): compute the effective stop with
            # the exact expression the pre-navigation-mode supervisor used,
            # verbatim, so the "default behavior is unchanged" guarantee is
            # inspectable by diff rather than by trusting that
            # effective_stop's managed-mode branch matches. This is also the
            # same formula effective_stop() applies when manage_controllers
            # is True (see tinker_sim_core.safety_gating).
            active = (
                self._desired_stop
                or not self._management_ready
                or self._startup_hold
                or self._restore_pending
            )
        else:
            active = effective_stop(
                self._desired_stop,
                self._management_ready,
                self._startup_hold,
                self._restore_pending,
                self._manage_controllers,
            )
        # Repeated publication is the supervisor liveness heartbeat. A
        # consumer that misses this stream must assert its own stop.
        self._publish(active)

    def _publish(self, active: bool) -> None:
        message = Bool()
        message.data = active
        changed = active != self._published_stop
        self._stop.publish(message)
        if changed:
            # Observability only (#33): announce every value change, not
            # every republish (the 0.25 s heartbeat repeats the unchanged
            # value far more often than it actually flips).
            reasons = self._stop_reasons()
            self.get_logger().info(
                f"safety_stop_published value={active} "
                f"reason={','.join(reasons) if reasons else 'none'}"
            )
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
        if not self._manage_controllers:
            return
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
        # 30 s, not 2 s: the controller manager applies switches from its
        # stepped update loop, and Isaac's stepping gaps out for multiple
        # wall seconds during RTX camera strides / gripper contact bursts.
        # A 2 s window then times out every attempt, leaving the arm
        # controller stuck inactive after any transient safety stop.
        request.timeout = Duration(seconds=30.0).to_msg()
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
