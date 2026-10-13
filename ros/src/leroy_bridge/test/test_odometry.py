from leroy_bridge.odometry import integrate_yaw


def test_positive_gyro_z_increases_yaw_with_first_sample() -> None:
    yaw = integrate_yaw(0.0, None, 1.25, 0.2)

    assert yaw == 0.25


def test_positive_gyro_z_increases_yaw_with_trapezoid_integration() -> None:
    yaw = integrate_yaw(0.0, 1.0, 1.5, 0.2)

    assert yaw == 0.25
