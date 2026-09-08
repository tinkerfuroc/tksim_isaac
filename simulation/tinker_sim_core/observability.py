from __future__ import annotations

import math

"""Shared formatting for the #33 observability log lines.

Every line introduced for #33 reports a duration measured from some "start"
event: when a hold began, when a source was last heard from, when the safety
gate last armed. The very first such event a process ever sees has no start
yet -- the gateway boots ARMED with no prior arm time, a mux source can go
stale before it was ever fresh, a supervisor source can flip before its
first heartbeat ever arrives. A log line must never be the reason a node
exits, so every duration is rendered through this helper: a missing or
non-finite start degrades to ``"n/a"`` instead of raising.
"""


def format_duration(value: float | None) -> str:
    """Render an elapsed-seconds duration for a log line.

    ``None`` and non-finite values (``nan``/``inf``, which a bad clock read
    could still produce) render as ``"n/a"``. Any other formatting failure
    also degrades to ``"n/a"`` rather than propagating -- this is called
    from logging paths only, and logging must not be able to crash the
    caller.
    """
    if value is None:
        return "n/a"
    try:
        if not math.isfinite(value):
            return "n/a"
        return f"{value:.3f}"
    except (TypeError, ValueError):
        return "n/a"
