"""Skip re-sending joint targets that PhysX already holds.

Physics runs at 120 Hz while commands arrive far slower, so most steps push
byte-identical joint targets through Isaac Lab into PhysX. Profiled on
2026-08-20 that redundant plumbing was 6.8 ms of a 12.2 ms physics step --
more than the PhysX solve itself (5.0 ms).

PhysX drive targets persist until changed, and this backend uses implicit
(stateless) actuators and no external wrenches, so not re-writing an unchanged
target is semantically identical to re-writing it.

The gate is deliberately biased toward writing: the expensive failure is a
wasted write, the incorrect one is a missed write.

Note that a safety-stopped backend writes on *every* step by design:
``_apply_safety_actuator_hold`` recomputes a gravity-compensating arm effort
from the current joint state, so the effort target genuinely changes each step.
The gate only pays off once a command stream is live and the stop is cleared,
which is the state a real run spends its time in.
"""

from __future__ import annotations

from typing import Any, Callable, Sequence


def _default_equal(a: Any, b: Any) -> bool:
    """Exact equality for torch tensors, falling back to ``==``."""
    try:
        import torch

        if torch.is_tensor(a) or torch.is_tensor(b):
            return bool(torch.equal(a, b))
    except Exception:
        pass
    return bool(a == b)


class TargetWriteGate:
    """Decide whether this step's joint targets must be pushed to the sim."""

    def __init__(
        self,
        *,
        always_write: bool = False,
        equal: Callable[[Any, Any], bool] | None = None,
    ) -> None:
        self._always_write = bool(always_write)
        self._equal = equal or _default_equal
        self._written: tuple[Any, ...] | None = None
        # Nothing has been written yet, so the first step must write.
        self._forced = True

    @property
    def always_write(self) -> bool:
        return self._always_write

    def force_next(self) -> None:
        """Require a write regardless of equality.

        Used when the articulation handles were re-resolved (a fresh PhysX view
        holds no targets) or the safety stop changed state.
        """
        self._forced = True

    def should_write(self, targets: Sequence[Any]) -> bool:
        if self._always_write or self._forced or self._written is None:
            return True
        if len(targets) != len(self._written):
            return True
        return not all(
            self._equal(current, written)
            for current, written in zip(targets, self._written)
        )

    def note_written(self, snapshots: Sequence[Any]) -> None:
        """Record what was actually pushed; clears any pending force."""
        self._written = tuple(snapshots)
        self._forced = False
