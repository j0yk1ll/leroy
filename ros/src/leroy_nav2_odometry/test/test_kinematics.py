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


def test_motion_profile_never_depends_on_remote_velocity_commands():
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    pub = (root / 'leroy_nav2_odometry/synthetic_odom.py').read_text()
    launch = (root / 'launch/odom_fixture.launch.py').read_text()
    probe = (root / 'leroy_nav2_odometry/odom_probe.py').read_text()
    assert 'create_subscription' not in pub
    assert 'create_publisher(' in pub
    assert "'/leroy_nav2/diagnostics/synthetic_odom'" in pub
    assert "'odom'" in pub and "'base_link'" in pub
    assert 'ROS_DOMAIN_ID' in pub and 'ROS_DOMAIN_ID' in launch
    for forbidden in ("'/cmd_vel'", "'/odom'", "'controller_server'"):
        assert forbidden not in pub and forbidden not in launch
    assert 'cmd_vel' in probe  # fail-closed graph assertion
    assert 'leroy_bridge' not in launch

def test_probe_requires_measured_pose_differences_not_only_twist():
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    probe = (root / 'leroy_nav2_odometry/odom_probe.py').read_text()
    assert 'moved_forward = max(self.x_samples)' in probe
    assert 'moved_lateral = max(self.y_samples)' in probe
    assert 'rotated = max(self.yaw_samples)' in probe
