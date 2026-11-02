"""Non-actuating validation of metric odometry continuity.

This checks *plausibility*, not physical metric calibration or accuracy.
It never publishes transforms, changes pose authority or authorizes motion.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    position_variance: float
    yaw_variance: float
    frame: str = "odom"
    child_frame: str = "base_link"


@dataclass(frozen=True)
class Quality:
    valid: bool
    reason: str
    consecutive: int


class OdometryQualityGate:
    def __init__(self, *, max_gap_s=0.30, max_speed_mps=1.0,
                 max_yaw_rate_radps=2.0, max_position_variance=0.25,
                 max_yaw_variance=0.25, warmup_samples=3):
        limits = (max_gap_s, max_speed_mps, max_yaw_rate_radps,
                  max_position_variance, max_yaw_variance)
        if (any(type(x) not in (int, float) or not math.isfinite(x) or x <= 0
                for x in limits) or type(warmup_samples) is not int
                or warmup_samples < 1):
            raise ValueError("invalid quality gate configuration")
        self.max_gap_ns = int(max_gap_s * 1e9)
        self.max_speed = float(max_speed_mps)
        self.max_yaw_rate = float(max_yaw_rate_radps)
        self.max_position_variance = float(max_position_variance)
        self.max_yaw_variance = float(max_yaw_variance)
        self.warmup_samples = warmup_samples
        self.previous = None
        self.consecutive = 0

    def reset(self):
        self.previous = None
        self.consecutive = 0

    def observe(self, sample, *, now_ns):
        def reject(reason):
            self.reset()
            return Quality(False, reason, 0)
        if type(sample) is not Sample or type(now_ns) is not int or now_ns <= 0:
            return reject("invalid_input")
        if sample.frame != "odom" or sample.child_frame != "base_link":
            return reject("frame_mismatch")
        values = (sample.x, sample.y, sample.yaw, sample.vx, sample.vy,
                  sample.wz, sample.position_variance, sample.yaw_variance)
        if (type(sample.stamp_ns) is not int or sample.stamp_ns <= 0 or
                any(type(v) not in (float, int) or not math.isfinite(v)
                    for v in values)):
            return reject("nonfinite_or_invalid")
        if not 0 <= now_ns - sample.stamp_ns <= self.max_gap_ns:
            return reject("stale_or_future")
        if (not 0 <= sample.position_variance <= self.max_position_variance
                or not 0 <= sample.yaw_variance <= self.max_yaw_variance):
            return reject("covariance_unqualified")
        if (math.hypot(sample.vx, sample.vy) > self.max_speed
                or abs(sample.wz) > self.max_yaw_rate):
            return reject("velocity_exceeds_bounds")
        if self.previous is not None:
            dt_ns = sample.stamp_ns - self.previous.stamp_ns
            if not 0 < dt_ns <= self.max_gap_ns:
                return reject("timestamp_discontinuity")
            dt = dt_ns / 1e9
            distance = math.hypot(sample.x - self.previous.x,
                                  sample.y - self.previous.y)
            delta_yaw = math.atan2(math.sin(sample.yaw - self.previous.yaw),
                                   math.cos(sample.yaw - self.previous.yaw))
            if (distance > self.max_speed * dt + 0.02 or
                    abs(delta_yaw) > self.max_yaw_rate * dt + 0.02):
                return reject("pose_discontinuity")
        self.previous = sample
        self.consecutive += 1
        if self.consecutive < self.warmup_samples:
            return Quality(False, "warming_up", self.consecutive)
        return Quality(True, "plausible_not_calibrated", self.consecutive)
