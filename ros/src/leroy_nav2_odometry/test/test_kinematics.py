"""M3 test matrix: ROS-free synthetic mecanum and fail-closed geometry."""

from math import pi

import pytest

from leroy_nav2_odometry.kinematics import (
    Geometry, Pose2D, Twist2D, integrate, synthetic_twist,
    twist_from_wheels, wheels_from_twist,
)


G = Geometry(0.05, 0.25, 0.20)


@pytest.mark.parametrize('expected', [
    Twist2D(0., 0., 0.),
    Twist2D(0.08, 0., 0.),
    Twist2D(-0.03, 0., 0.),
    Twist2D(0., 0.06, 0.),
    Twist2D(0., -0.06, 0.),
    Twist2D(0., 0., 0.2),
    Twist2D(0., 0., -0.2),
    Twist2D(0.03, -0.02, 0.1),
])
def test_synthetic_mecanum_inverse_forward_exact(expected):
    actual = twist_from_wheels(wheels_from_twist(expected, G), G)
    assert actual.vx == pytest.approx(expected.vx)
    assert actual.vy == pytest.approx(expected.vy)
    assert actual.wz == pytest.approx(expected.wz)


def test_wheel_sign_convention_independent_lateral_and_yaw():
    # X-pattern: lateral [-,+,+,-], yaw [-,+,-,+].
    lateral = twist_from_wheels((-1., 1., 1., -1.), G)
    yaw = twist_from_wheels((-1., 1., -1., 1.), G)
    assert lateral.vx == pytest.approx(0.)
    assert lateral.vy > 0
    assert lateral.wz == pytest.approx(0.)
    assert yaw.vx == pytest.approx(0.)
    assert yaw.vy == pytest.approx(0.)
    assert yaw.wz > 0



@pytest.mark.parametrize('values', [
    (0., .25, .2), (.05, 0., .2), (.05, .25, 0.),
    (-.05, .25, .2), (float('nan'), .25, .2),
    (.05, float('inf'), .2),
])
def test_invalid_synthetic_geometry_rejected(values):
    with pytest.raises(ValueError):
        Geometry(*values)


@pytest.mark.parametrize('wheels', [
    (0., 0., 0.), (0., 0., 0., 0., 0.),
    (float('nan'), 0., 0., 0.),
    (0., float('inf'), 0., 0.),
    (21., 0., 0., 0.),
])
def test_invalid_wheel_measurement_rejected(wheels):
    with pytest.raises(ValueError):
        twist_from_wheels(wheels, G)


def test_planar_integration_strafe_uses_current_yaw():
    p = integrate(Pose2D(yaw=pi / 2), Twist2D(0., .06, 0.), .2)
    assert p.x == pytest.approx(-.012)
    assert p.y == pytest.approx(0., abs=1e-9)
    assert p.yaw == pytest.approx(pi / 2)


def test_rotation_midpoint_integrator_and_yaw_wrap():
    p = Pose2D(yaw=pi - .02)
    p = integrate(p, Twist2D(.08, .06, .2), .2)
    assert p.x < 0 and p.y < 0
    assert -pi <= p.yaw <= pi


@pytest.mark.parametrize('dt', [0., -0.1, 0.26, float('nan'), float('inf')])
def test_stale_or_nonmonotonic_interval_rejected_without_pose_update(dt):
    pose = Pose2D(x=1., y=2., yaw=.3)
    with pytest.raises(ValueError):
        integrate(pose, Twist2D(.1, 0., 0.), dt)
    assert pose == Pose2D(x=1., y=2., yaw=.3)


def test_nonfinite_twist_rejected():
    with pytest.raises(ValueError):
        wheels_from_twist(Twist2D(float('nan'), 0., 0.), G)
    with pytest.raises(ValueError):
        integrate(Pose2D(), Twist2D(0., float('inf'), 0.), .1)


@pytest.mark.parametrize(('t', 'expected'), [
    (0., Twist2D(0., 0., 0.)),
    (3., Twist2D(.08, 0., 0.)),
    (6., Twist2D(0., .06, 0.)),
    (9., Twist2D(0., 0., .2)),
    (12., Twist2D(0., 0., 0.)),
])
def test_deterministic_profile(t, expected):
    assert synthetic_twist(t) == expected
