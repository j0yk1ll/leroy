"""Open-loop mecanum Twist-to-PWM mapping (ROS-independent).

The full-scale wheel-velocity value is a conversion reference at hardware
PWM=255, *not* a validated physical wheel-speed limit. max_pwm separately
bounds absolute duty. A zero full-scale reference disables actuation.
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
    pwm_full_scale_wheel_velocity_rad_s: float,
    max_pwm: int,
) -> Optional[list[int]]:
    """Return [FL, FR, RL, RR] integer duty commands or None when disabled/invalid."""
    values = (vx, vy, wz, wheel_radius_m, half_wheelbase_m, half_track_m,
              pwm_full_scale_wheel_velocity_rad_s)
    if not all(math.isfinite(value) for value in values):
        return None
    if (wheel_radius_m <= 0.0 or half_wheelbase_m <= 0.0 or half_track_m <= 0.0
            or pwm_full_scale_wheel_velocity_rad_s <= 0.0
            or not isinstance(max_pwm, int) or not 1 <= max_pwm <= 255):
        return None

    lever = half_wheelbase_m + half_track_m
    # Mecanum inverse kinematics, in firmware's FL/FR/RL/RR order.
    wheel = [
        (vx - vy - lever * wz) / wheel_radius_m,
        (vx + vy + lever * wz) / wheel_radius_m,
        (vx + vy - lever * wz) / wheel_radius_m,
        (vx - vy + lever * wz) / wheel_radius_m,
    ]
    if not all(math.isfinite(value) for value in wheel):
        return None

    # Preserve four-wheel velocity ratios when a requested Twist would exceed
    # the independent configured PWM ceiling; do not clip wheels separately.
    allowable_rad_s = pwm_full_scale_wheel_velocity_rad_s * (max_pwm / 255.0)
    peak = max(abs(value) for value in wheel)
    if peak > allowable_rad_s:
        wheel = [value * (allowable_rad_s / peak) for value in wheel]

    pwm = [int(round(value / pwm_full_scale_wheel_velocity_rad_s * 255.0))
           for value in wheel]
    return [max(-max_pwm, min(max_pwm, value)) for value in pwm]
