"""Offline-only M6 sensor fault schedule and diagnostic freshness policy.

This is a TEST AUDIT, NOT a command interlock, safety certification, or
authorization to publish any robot velocity.
"""

import math


CYCLE_S = 22.0
PAUSE_START_S = 10.0
PAUSE_END_S = 16.0
MAX_AGE_S = 1.5
FUTURE_TOLERANCE_S = 0.1


def in_blackout(elapsed_s):
    """Synthetic publisher pause from [10, 16) seconds of each 22s cycle."""
    if not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError('Invalid synthetic fault elapsed time')
    phase = elapsed_s % CYCLE_S
    return PAUSE_START_S <= phase < PAUSE_END_S


def _recent(now_mono, now_ros, recv_mono, acquisition_ros,
            max_age_s=MAX_AGE_S):
    if recv_mono is None or acquisition_ros is None:
        return False
    numbers = (now_mono, now_ros, recv_mono, acquisition_ros, max_age_s)
    if not all(math.isfinite(v) for v in numbers) or max_age_s <= 0:
        return False
    recv_age = now_mono - recv_mono
    stamp_age = now_ros - acquisition_ros
    return (0.0 <= recv_age <= max_age_s
            and -FUTURE_TOLERANCE_S <= stamp_age <= max_age_s)


def diagnostic_state(*, now_mono, now_ros, odom_recv, odom_stamp,
                     depth_recv, depth_stamp, map_recv, map_valid):
    """Ready only if every relevant TEST observation is fresh and consistent.

    Fresh rolling costmaps alone NEVER substitute for missing fresh depth.
    This function does not control Nav2 or veto any motor-compatible topic.
    """
    if not math.isfinite(now_mono) or not math.isfinite(now_ros):
        return 'BLOCKED_CLOCK'
    odom_fresh = _recent(now_mono, now_ros, odom_recv, odom_stamp)
    depth_fresh = _recent(now_mono, now_ros, depth_recv, depth_stamp)
    # An isolated fault must never be attributed to only one stream when
    # both streams have dropped. Keep this evidence explicit.
    if not odom_fresh and not depth_fresh:
        return 'BLOCKED_BOTH'
    if not odom_fresh:
        return 'BLOCKED_ODOM'
    if not depth_fresh:
        return 'BLOCKED_DEPTH'
    if not map_valid or not _recent(now_mono, now_ros, map_recv, now_ros):
        return 'BLOCKED_COSTMAP'
    return 'READY'
