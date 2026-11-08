"""Planar holonomic point-goal control; pure geometry and no motor authority."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Command:
    vx: float
    vy: float
    wz: float
    arrived: bool


class GoalController:
    def __init__(self, *, max_linear=0.10, max_angular=0.20,
                 kp_linear=0.8, kp_angular=1.2,
                 position_tolerance=0.05, yaw_tolerance=0.08):
        values = (max_linear, max_angular, kp_linear, kp_angular,
                  position_tolerance, yaw_tolerance)
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in values):
            raise ValueError("invalid controller gains or limits")
        self.max_linear = float(max_linear)
        self.max_angular = float(max_angular)
        self.kp_linear = float(kp_linear)
        self.kp_angular = float(kp_angular)
        self.position_tolerance = float(position_tolerance)
        self.yaw_tolerance = float(yaw_tolerance)

    def calculate(self, current: Pose, target: Pose) -> Command:
        if not isinstance(current, Pose) or not isinstance(target, Pose):
            raise ValueError("pose required")
        if not all(math.isfinite(v) for v in (
                current.x, current.y, current.yaw, target.x, target.y, target.yaw)):
            raise ValueError("nonfinite pose")
        dx, dy = target.x-current.x, target.y-current.y
        error_yaw = math.atan2(math.sin(target.yaw-current.yaw),
                               math.cos(target.yaw-current.yaw))
        if math.hypot(dx, dy) <= self.position_tolerance:
            vx, vy = 0.0, 0.0
        else:
            c, s = math.cos(current.yaw), math.sin(current.yaw)
            vx = self.kp_linear*(c*dx+s*dy)
            vy = self.kp_linear*(-s*dx+c*dy)
            speed = math.hypot(vx, vy)
            if speed > self.max_linear:
                scale = self.max_linear/speed
                vx, vy = vx*scale, vy*scale
        wz = 0.0 if abs(error_yaw) <= self.yaw_tolerance else max(
            -self.max_angular, min(self.max_angular, self.kp_angular*error_yaw))
        arrived = vx == 0.0 and vy == 0.0 and wz == 0.0
        return Command(vx, vy, wz, arrived)
