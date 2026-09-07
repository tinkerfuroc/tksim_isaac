"""#33 observability: the safety supervisor must announce a required
source's requires_stop() edge and every changed /sim/hardware/safety_stop
publish, without changing the desired-stop / effective-stop computation
those paths already had.
"""

from __future__ import annotations

from unittest.mock import patch

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("std_msgs")

from tinker_sim_bridge.safety_supervisor import (  # noqa: E402
    SafetySourceTracker,
    SafetySupervisor,
)


class _LogRecorder:
    def __init__(self) -> None:
        self.info_lines: list[str] = []

    def info(self, message: str) -> None:
        self.info_lines.append(message)

    def error(self, message: str) -> None:
        pass

    def warning(self, message: str) -> None:
        pass


class _Recorder:
    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, message) -> None:
        self.messages.append(message)


def _supervisor(logger: _LogRecorder) -> SafetySupervisor:
    """A lightweight double: only the source-tracking / publish path this
    observability change touches, not the controller-manager machinery
    (list_controllers/switch_controller), which needs a live rclpy client.
    Unmanaged mode (manage_controllers=False) exercises the same
    _publish_effective -> effective_stop path with the simplest formula
    (bool(desired_stop)), so a bare source recovery is directly visible on
    /sim/hardware/safety_stop without a controller-manager round trip.
    """
    supervisor = object.__new__(SafetySupervisor)
    supervisor._source_trackers = {"collision": SafetySourceTracker(1.0)}
    supervisor._sources = {"collision": None, "operator": False}
    supervisor.REQUIRED_SOURCES = ("collision",)
    supervisor.OPTIONAL_SOURCES = ("operator",)
    supervisor._source_stop_state = {}
    supervisor._desired_stop = True
    supervisor._published_stop = None
    supervisor._management_ready = True
    supervisor._manage_controllers = False
    supervisor._startup_hold = False
    supervisor._restore_pending = False
    supervisor._controller_active = None
    supervisor._controller_was_active = None
    supervisor._stop_episode_recorded = False
    supervisor._stop = _Recorder()
    supervisor.get_logger = lambda: logger
    return supervisor


def test_source_expiry_and_recovery_flip_publish_with_a_fake_clock() -> None:
    clock = {"t": 0.0}

    with patch(
        "tinker_sim_bridge.safety_supervisor.time.monotonic", lambda: clock["t"]
    ):
        logger = _LogRecorder()
        supervisor = _supervisor(logger)
        tracker = supervisor._source_trackers["collision"]

        # An initial heartbeat clears the source; the first observation only
        # seeds _source_stop_state (nothing to flip against yet), but this is
        # still a real /sim/hardware/safety_stop transition (None -> False).
        tracker.update(False, clock["t"])
        supervisor._refresh_desired_stop()
        supervisor._publish_effective()
        assert [m for m in logger.info_lines if m.startswith("safety_source")] == []
        published = [
            m for m in logger.info_lines if m.startswith("safety_stop_published")
        ]
        assert published == ["safety_stop_published value=False reason=none"]

        # 1.1 s of silence past the 1.0 s deadline: the source expires.
        clock["t"] += 1.1
        supervisor._refresh_desired_stop()
        supervisor._publish_effective()
        expired = [m for m in logger.info_lines if m.startswith("safety_source")]
        assert expired == [
            "safety_source source=collision state=expired age_s=1.100 deadline_s=1.000"
        ]
        published = [
            m for m in logger.info_lines if m.startswith("safety_stop_published")
        ]
        assert published[-1] == "safety_stop_published value=True reason=collision"

        # A fresh heartbeat recovers the source and clears the publish.
        tracker.update(False, clock["t"])
        supervisor._refresh_desired_stop()
        supervisor._publish_effective()
        recovered = [m for m in logger.info_lines if "recovered" in m]
        assert recovered == [
            "safety_source source=collision state=recovered age_s=0.000 deadline_s=1.000"
        ]
        published = [
            m for m in logger.info_lines if m.startswith("safety_stop_published")
        ]
        assert published[-1] == "safety_stop_published value=False reason=none"


def test_first_heartbeat_ever_transition_does_not_raise() -> None:
    """#33: a source that starts already-expired (requires_stop() True at
    the very first _log_source_transitions call, before _source_stop_state
    has any prior value for it) and then recovers on its first-ever sample
    must not raise formatting an age from a tracker whose received_at was
    still None moments earlier.
    """
    with patch("tinker_sim_bridge.safety_supervisor.time.monotonic", lambda: 0.0):
        logger = _LogRecorder()
        supervisor = _supervisor(logger)

        # Before any sample, requires_stop() is True (received_at is None)
        # and _source_stop_state has no prior entry: the transition guard
        # must not crash formatting that unset state, and (by design) does
        # not log a line since there is nothing to compare against yet.
        supervisor._refresh_desired_stop()
        assert [m for m in logger.info_lines if m.startswith("safety_source")] == []
        assert supervisor._source_stop_state["collision"] is True

        # The first-ever heartbeat now flips it: state=recovered, and the
        # age is computed from a receive time that only just stopped being
        # None -- this must render a real duration, not raise or print nan.
        supervisor._source_trackers["collision"].update(False, 0.0)
        supervisor._refresh_desired_stop()
        recovered = [m for m in logger.info_lines if "recovered" in m]
        assert recovered == [
            "safety_source source=collision state=recovered age_s=0.000 deadline_s=1.000"
        ]


def test_transition_with_no_received_at_recorded_does_not_raise() -> None:
    """#33 defensive path: _log_source_transitions must not raise even if a
    tracker somehow flips to "expired" with received_at still None (e.g. a
    future caller mutating tracker state directly, bypassing update()).
    requires_stop() forces True whenever received_at is None, so this is the
    only state ``current`` can take there -- but the age formatting must
    still degrade rather than assume a start time exists.
    """
    with patch("tinker_sim_bridge.safety_supervisor.time.monotonic", lambda: 0.0):
        logger = _LogRecorder()
        supervisor = _supervisor(logger)
        tracker = supervisor._source_trackers["collision"]

        supervisor._source_stop_state["collision"] = False  # seed a "previous"
        tracker.value = False
        tracker.received_at = None  # deliberately decoupled from value

        supervisor._log_source_transitions()

        expired = [m for m in logger.info_lines if "expired" in m]
        assert expired == [
            "safety_source source=collision state=expired age_s=n/a deadline_s=1.000"
        ]


def test_unchanged_publish_is_not_logged_again() -> None:
    with patch("tinker_sim_bridge.safety_supervisor.time.monotonic", lambda: 5.0):
        logger = _LogRecorder()
        supervisor = _supervisor(logger)
        supervisor._source_trackers["collision"].update(False, 5.0)

        supervisor._refresh_desired_stop()
        supervisor._publish_effective()
        supervisor._publish_effective()
        supervisor._publish_effective()

        published = [
            m for m in logger.info_lines if m.startswith("safety_stop_published")
        ]
        assert len(published) == 1
