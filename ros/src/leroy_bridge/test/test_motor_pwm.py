"""ROS-free regression tests for the open-loop mecanum PWM conversion."""
import math

from leroy_bridge.motor_pwm import mecanum_twist_to_pwm


def convert(vx=0.0, vy=0.0, wz=0.0, gain=12.75, cap=64):
    return mecanum_twist_to_pwm(
        vx=vx,
        vy=vy,
        wz=wz,
        wheel_radius_m=0.020,
        half_wheelbase_m=0.180,
        half_track_m=0.275,
        wheel_velocity_to_pwm_gain=gain,
        max_pwm=cap,
    )


def test_default_zero_gain_disables_motors():
    assert convert(wz=0.16, gain=0.0) is None


def test_zero_twist_commands_all_wheels_stopped():
    assert convert() == [0, 0, 0, 0]


def test_yaw_pwm_and_independent_cap():
    # 0.16 rad/s yaw needs 3.64 wheel rad/s; gain 12.75 => 46 PWM.
    assert convert(wz=0.16) == [-46, 46, -46, 46]
    assert convert(vx=0.025) == [16, 16, 16, 16]
    assert convert(wz=0.16, cap=255) == [-46, 46, -46, 46]
    assert convert(wz=0.16, cap=24) == [-24, 24, -24, 24]


def test_large_commands_preserve_wheel_ratios_when_saturated():
    assert convert(vx=0.1, wz=0.7) == [-33, 64, -33, 64]
    assert all(abs(v) <= 64 for v in convert(vx=3.0, vy=-4.0, wz=2.0))


def test_higher_gain_means_higher_pwm_until_cap():
    assert convert(wz=0.16, gain=6.375) == [-23, 23, -23, 23]
    assert convert(wz=0.16, gain=25.5) == [-64, 64, -64, 64]


def test_reject_nonfinite_inputs_and_invalid_cap():
    assert convert(vx=float('nan')) is None
    assert convert(wz=math.inf) is None
    assert convert(gain=float('nan')) is None
    assert convert(gain=-0.1) is None
    assert convert(cap=0) is None
    assert convert(cap=256) is None
