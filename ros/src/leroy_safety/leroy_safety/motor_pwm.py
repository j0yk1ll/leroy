"""ROS-independent open-loop mecanum Twist-to-PWM conversion.

wheel_velocity_to_pwm_gain converts requested wheel rad/s into duty units
(PWM per rad/s), NOT a measured physical wheel speed limit.
max_pwm independently caps the absolute signed duty. Zero gain disables
actuation. Gain and cap are fixed at bridge startup.
"""
import math
from typing import Optional


def mecanum_twist_to_pwm(
    vx: float,
    vy: float,
    wz: float,
    wheel_radius_m: float,
    half_wheelbase_m: float,
    half_track_m: float,
    wheel_velocity_to_pwm_gain: float,
    max_pwm: int,
) -> Optional[list[int]]:
    """Return signed [FL, FR, RL, RR] integer duties, or None if disabled/invalid."""
    values = (vx, vy, wz, wheel_radius_m, half_wheelbase_m, half_track_m,
              wheel_velocity_to_pwm_gain)
    if not all(math.isfinite(value) for value in values):
        return None
    if (wheel_radius_m <= 0.0 or half_wheelbase_m <= 0.0 or half_track_m <= 0.0
            or wheel_velocity_to_pwm_gain <= 0.0
            or not isinstance(max_pwm, int) or not 1 <= max_pwm <= 255):
        return None

    lever = half_wheelbase_m + half_track_m
    # Mecanum inverse kinematics, in firmware FL/FR/RL/RR order.
    wheel = [
        (vx - vy - lever * wz) / wheel_radius_m,
        (vx + vy + lever * wz) / wheel_radius_m,
        (vx + vy - lever * wz) / wheel_radius_m,
        (vx - vy + lever * wz) / wheel_radius_m,
    ]
    if not all(math.isfinite(value) for value in wheel):
        return None
    raw_pwm = [omega * wheel_velocity_to_pwm_gain for omega in wheel]
    if not all(math.isfinite(value) for value in raw_pwm):
        return None

    # Scale together to preserve the mecanum wheel ratios if the requested
    # output exceeds the independent PWM cap. Never clip individual wheels.
    peak = max(abs(value) for value in raw_pwm)
    if peak > max_pwm:
        raw_pwm = [value * max_pwm / peak for value in raw_pwm]

    return [max(-max_pwm, min(max_pwm, int(round(value)))) for value in raw_pwm]
