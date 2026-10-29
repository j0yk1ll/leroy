"""Pure Python, synthetic X-pattern mecanum geometry and planar integration.

Wheel order: front-left, front-right, rear-left, rear-right.
All parameters are fictitious laboratory values, NOT Leroy calibration.
No ROS, device, motor, or command topic access.
"""

# TODO(AUTONOMY) [measured mecanum odometry]: Replace this synthetic
# fixture's values with separately tested encoder counts/signs, measured
# wheel radius, track/wheelbase, roller orientation and slip compensation.
# TODO(AUTONOMY) [pose fusion]: Fuse wheel velocities and real IMU yaw
# with validated camera/metric localization, published covariances and
# single ownership of odom->base_link. Detect jumps, encoder loss and
# nonholonomic slip without silently trusting guessed physical scale.
# Acceptance: measured driving/rotation against external ground truth,
# repeated stop/start and encoder faults remain within recorded error bounds.


from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Geometry:
    wheel_radius_m: float
    half_length_m: float
    half_width_m: float

    def __post_init__(self):
        values = (self.wheel_radius_m, self.half_length_m, self.half_width_m)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Synthetic geometry must be finite and positive')

    @property
    def lever_arm(self):
        return self.half_length_m + self.half_width_m


@dataclass(frozen=True)
class Twist2D:
    vx: float
    vy: float
    wz: float


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def _finite_four(wheel_rad_s):
    if len(wheel_rad_s) != 4:
        raise ValueError('Expected exactly FL, FR, RL, RR wheel speeds')
    wheels = tuple(float(v) for v in wheel_rad_s)
    if not all(math.isfinite(v) and abs(v) <= 20.0 for v in wheels):
        raise ValueError('Nonfinite or excessive synthetic wheel speed')
    return wheels


def twist_from_wheels(wheel_rad_s, geometry):
    """X-roller sign convention for FL, FR, RL, RR angular velocities."""
    fl, fr, rl, rr = _finite_four(wheel_rad_s)
    q = geometry.wheel_radius_m / 4.0
    return Twist2D(
        vx=q * (fl + fr + rl + rr),
        vy=q * (-fl + fr + rl - rr),
        wz=(q / geometry.lever_arm) * (-fl + fr - rl + rr),
    )


def wheels_from_twist(twist, geometry):
    """Inverse, used only to synthesize known test wheel measurements."""
    vals = (twist.vx, twist.vy, twist.wz)
    if not all(math.isfinite(v) for v in vals):
        raise ValueError('Invalid synthetic twist')
    k = geometry.lever_arm
    r = geometry.wheel_radius_m
    return _finite_four((
        (twist.vx - twist.vy - k * twist.wz) / r,
        (twist.vx + twist.vy + k * twist.wz) / r,
        (twist.vx + twist.vy - k * twist.wz) / r,
        (twist.vx - twist.vy + k * twist.wz) / r,
    ))


def integrate(pose, twist, dt_s):
    """SE(2) midpoint step, with a bounded timestamp gap (no extrapolation)."""
    if not math.isfinite(dt_s) or not 0.0 < dt_s <= 0.25:
        raise ValueError('Invalid or stale synthetic odometry interval')
    if not all(math.isfinite(v) for v in (
            pose.x, pose.y, pose.yaw, twist.vx, twist.vy, twist.wz)):
        raise ValueError('Nonfinite planar odometry')
    mid_yaw = pose.yaw + 0.5 * twist.wz * dt_s
    return Pose2D(
        x=pose.x + dt_s * (
            math.cos(mid_yaw) * twist.vx - math.sin(mid_yaw) * twist.vy),
        y=pose.y + dt_s * (
            math.sin(mid_yaw) * twist.vx + math.cos(mid_yaw) * twist.vy),
        yaw=math.atan2(
            math.sin(pose.yaw + twist.wz * dt_s),
            math.cos(pose.yaw + twist.wz * dt_s)),
    )


def synthetic_twist(elapsed_s):
    """Timed motion sequence independent of any subscribed command topic."""
    if not math.isfinite(elapsed_s) or elapsed_s < 0:
        raise ValueError('Invalid synthetic elapsed time')
    if elapsed_s < 2:
        return Twist2D(0.0, 0.0, 0.0)
    if elapsed_s < 5:
        return Twist2D(0.08, 0.0, 0.0)
    if elapsed_s < 8:
        return Twist2D(0.0, 0.06, 0.0)
    if elapsed_s < 11:
        return Twist2D(0.0, 0.0, 0.2)
    return Twist2D(0.0, 0.0, 0.0)
