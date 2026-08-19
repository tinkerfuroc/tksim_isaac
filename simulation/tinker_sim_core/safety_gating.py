from __future__ import annotations


def effective_stop(
    desired_stop: bool,
    management_ready: bool,
    startup_hold: bool,
    restore_pending: bool,
    manage_controllers: bool,
) -> bool:
    """Effective /sim/hardware/safety_stop value.

    Managed mode (manipulation): the controller-lifecycle latches gate the
    clear exactly as before. Unmanaged mode (navigation — no
    /controller_manager exists): only the fused source state matters; the
    fail-closed source-freshness contract lives upstream in
    SafetySourceTracker and still asserts desired_stop on any stale source.
    """
    if not manage_controllers:
        return bool(desired_stop)
    return bool(
        desired_stop or not management_ready or startup_hold or restore_pending
    )
