"""Pure mission queue: ordered poses, cancellation and finite recovery budget.

No ROS, motor authority or diagnostic interfaces. The controller must call
arrived() only after independently checking the final target pose.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        if not all(type(v) in (float, int) and math.isfinite(v)
                   for v in (self.x, self.y, self.yaw)):
            raise ValueError("invalid waypoint")


class Mission:
    def __init__(self, *, max_waypoints=64, max_replans=3):
        if (type(max_waypoints) is not int or max_waypoints < 1 or
                type(max_replans) is not int or max_replans < 0):
            raise ValueError("invalid mission limits")
        self.max_waypoints = max_waypoints
        self.max_replans = max_replans
        self.cancel()

    def load(self, waypoints):
        points = tuple(waypoints)
        if (not points or len(points) > self.max_waypoints
                or any(type(p) is not Waypoint for p in points)):
            raise ValueError("invalid mission plan")
        self._points = points
        self.index = 0
        self.replans = 0
        self.state = "RUNNING"
        return self.current

    @property
    def current(self):
        if self.state != "RUNNING":
            return None
        return self._points[self.index]

    def arrived(self):
        if self.state != "RUNNING":
            return None
        self.index += 1
        self.replans = 0
        if self.index == len(self._points):
            self.state = "SUCCEEDED"
            return None
        return self.current

    def unreachable(self):
        if self.state != "RUNNING":
            return False
        self.replans += 1
        if self.replans > self.max_replans:
            self.state = "FAILED"
            return False
        return True

    def cancel(self):
        self._points = ()
        self.index = 0
        self.replans = 0
        self.state = "CANCELLED"
