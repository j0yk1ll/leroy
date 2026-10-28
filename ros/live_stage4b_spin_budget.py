"""Hardware-free bounded ROS subscription service policy for Stage 4B.

The owning serial loop MUST read the MCU first and call this helper AFTER
each read, not spin for an unlimited time. No ROS, serial or motor imports.
"""
import time

MAX_SPINS = 8
MAX_BUDGET_S = 0.008


def service_callbacks(spin_one, check_faults, *, monotonic=time.monotonic,
                      max_spins=MAX_SPINS, budget_s=MAX_BUDGET_S):
    """Attempt multiple nonblocking spins while limiting serial starvation.

    The callback itself cannot be preempted, so budget is checked BETWEEN
    spins. Return (spin_attempts, elapsed_seconds); these are not guaranteed
    to be ROS message callback counts. Callers must use timeout_sec=0.0.
    """
    if not callable(spin_one) or not callable(check_faults):
        raise ValueError("callbacks_must_be_callables")
    if type(max_spins) is not int or not 1 <= max_spins <= MAX_SPINS:
        raise ValueError("invalid_spin_limit")
    if type(budget_s) not in (float, int) or not 0 < budget_s <= MAX_BUDGET_S:
        raise ValueError("invalid_spin_budget")
    started = monotonic()
    attempts = 0
    for _ in range(max_spins):
        if attempts and monotonic() - started >= budget_s:
            break
        spin_one()
        attempts += 1
        # Never suppress a graph, sensor provenance or odometry fault to
        # complete the rest of a burst: propagate to the MCU STOP finally.
        check_faults()
    elapsed = monotonic() - started
    return attempts, elapsed
