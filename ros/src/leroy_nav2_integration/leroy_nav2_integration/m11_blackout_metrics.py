"""M11 pure synthetic blackout metrics; no ROS or actuator imports.

CLOCK_MONOTONIC is shared by processes in one isolated Podman time
namespace. The returned trigger window represents the *scheduled*
5-second fake-source pause, not proof of motor behavior or actual
sensor suppression at each timer tick.
"""

import json
import math

from .fault_contract import MAX_AGE_S


INNER_START_GUARD_S = 0.75  # depth fixture timer fires every 0.5s
INNER_END_GUARD_S = 0.25
EXPECTED_DURATION_S = 5.0


def encode_window(*, start, end):
    if (not all(math.isfinite(v) for v in (start, end))
            or start <= 0 or abs((end - start) - EXPECTED_DURATION_S) > 0.001):
        raise ValueError('Invalid 5s synthetic blackout schedule')
    return json.dumps({
        'm11_synthetic_only': True,
        'scheduled_start_mono': start,
        'scheduled_end_mono': end,
    }, sort_keys=True)


def parse_window(message):
    try:
        payload = json.loads(message)
        if payload.get('m11_synthetic_only') is not True:
            raise ValueError('not an M11 fake source schedule')
        start = payload['scheduled_start_mono']
        end = payload['scheduled_end_mono']
        if (type(start) not in (int, float)
                or type(end) not in (int, float)):
            raise ValueError('non-numeric monotonic timestamps')
        # Roundtrip schema and fixed duration validate both numbers.
        encode_window(start=start, end=end)
        return float(start), float(end)
    except (TypeError, KeyError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError('Invalid synthetic blackout trigger evidence') from exc


def summarize(samples, *, start, end, first_veto, sensor_recv):
    """Classify read-only receipt-timed MPPI proposals strictly by window.

    samples: sequence of (rx_monotonic, is_nonzero), recorded ONLY from
    accepted-goal, finite, bounded actual MPPI diagnostic Twist callbacks.
    A message received during the schedule may have been generated
    earlier; source Twist is unstamped. Neither count implies movement.
    """
    encode_window(start=start, end=end)
    if (first_veto is None or not math.isfinite(first_veto)
            or not start <= first_veto < end):
        raise ValueError('No first correct shadow veto inside fake outage')
    if (sensor_recv is None or not math.isfinite(sensor_recv)
            or sensor_recv > start):
        raise ValueError('No pre-trigger fake sensor receipt baseline')
    result = {
        'scheduled_zero': 0, 'scheduled_nonzero': 0,
        'inner_zero': 0, 'inner_nonzero': 0,
        'after_veto_zero': 0, 'after_veto_nonzero': 0,
    }
    inner_start = start + INNER_START_GUARD_S
    inner_end = end - INNER_END_GUARD_S
    for time_received, nonzero in samples:
        if (not math.isfinite(time_received)
                or type(nonzero) is not bool):
            raise ValueError('Malformed MPPI diagnostic sample')
        if start <= time_received < end:
            key = 'nonzero' if nonzero else 'zero'
            result['scheduled_' + key] += 1
            if inner_start <= time_received < inner_end:
                result['inner_' + key] += 1
            if first_veto <= time_received < end:
                result['after_veto_' + key] += 1
    result['veto_from_schedule_s'] = round(first_veto - start, 4)
    result['veto_from_last_sensor_rx_s'] = round(first_veto - sensor_recv, 4)
    result['veto_vs_recv_age_threshold_s'] = round(
        first_veto - (sensor_recv + MAX_AGE_S), 4)
    return result
