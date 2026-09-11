"""``_has_listeners`` decides whether an expensive payload gets BUILT.

The gateway used to build three grasp-bench payloads on EVERY control tick,
unconditionally: ``contact_state()``, ``parity_tcp_frame()`` and a
``json.dumps()`` of the entire physics-truth frame. Their only consumers are
the manipulation/qualification scenarios, so a navigation run computed all of
it 120 times a second and handed it to DDS to drop.

The gate is deliberately conservative, because publishing nothing is a far
worse failure than publishing into the void:

* a publisher that does not exist cannot have a listener, but that is not an
  error -- partial gateways exist in tests and some parity publishers are
  env-gated. An eager ``self.pub`` tuple raised AttributeError on exactly
  those, which broke 8 manipulation tests;
* a publisher with no ``get_subscription_count`` fails OPEN;
* a counter that raises fails OPEN;
* if NOTHING resolves, behave as though the gate were not there.

Only an affirmative "every publisher resolved and every one reports zero
subscribers" suppresses the work.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))

from tinker_sim_isaac.ros_gateway import RosStandardGateway  # noqa: E402


class _Pub:
    def __init__(self, count):
        self._count = count

    def get_subscription_count(self):
        if isinstance(self._count, Exception):
            raise self._count
        return self._count


class _PubNoCounter:
    """A publisher-like object that predates/omits the count API."""


def _gateway(**publishers):
    gateway = object.__new__(RosStandardGateway)
    for name, pub in publishers.items():
        setattr(gateway, name, pub)
    return gateway


class HasListenersTest(unittest.TestCase):
    def test_zero_subscribers_suppresses(self) -> None:
        gateway = _gateway(a=_Pub(0), b=_Pub(0))
        self.assertFalse(gateway._has_listeners("a", "b"))

    def test_any_subscriber_keeps_the_work(self) -> None:
        gateway = _gateway(a=_Pub(0), b=_Pub(3))
        self.assertTrue(gateway._has_listeners("a", "b"))

    def test_missing_publisher_is_not_an_error(self) -> None:
        """The exact failure that broke 8 manipulation tests.

        A partially-built gateway has some parity publishers and not others.
        Resolving them eagerly raised AttributeError from inside publish().
        """
        gateway = _gateway(a=_Pub(0))
        try:
            result = gateway._has_listeners("a", "does_not_exist")
        except AttributeError as error:  # pragma: no cover - the regression
            self.fail(f"a missing publisher must not raise: {error}")
        self.assertFalse(result, "a=0 and b absent means nothing is listening")

    def test_publisher_without_the_count_api_fails_open(self) -> None:
        gateway = _gateway(a=_PubNoCounter())
        self.assertTrue(gateway._has_listeners("a"))

    def test_counter_that_raises_fails_open(self) -> None:
        gateway = _gateway(a=_Pub(RuntimeError("rmw exploded")))
        self.assertTrue(gateway._has_listeners("a"))

    def test_nothing_resolves_fails_open(self) -> None:
        """No publishers at all: behave exactly as before the gate existed."""
        gateway = _gateway()
        self.assertTrue(gateway._has_listeners("nope", "also_nope"))

    def test_env_escape_hatch_forces_open(self) -> None:
        """TINKER_SIM_PUBLISH_GATE=0 restores unconditional publishing.

        This is what makes an honest A/B possible: both arms are one build,
        differing only in this predicate.
        """
        import os

        gateway = _gateway(a=_Pub(0))
        previous = os.environ.get("TINKER_SIM_PUBLISH_GATE")
        os.environ["TINKER_SIM_PUBLISH_GATE"] = "0"
        try:
            self.assertTrue(gateway._has_listeners("a"))
        finally:
            if previous is None:
                os.environ.pop("TINKER_SIM_PUBLISH_GATE", None)
            else:
                os.environ["TINKER_SIM_PUBLISH_GATE"] = previous
        self.assertFalse(gateway._has_listeners("a"), "hatch must not persist")


if __name__ == "__main__":
    unittest.main()
