"""Bounded rolling evidence map, independent of ROS and actuation.

Retains observed cells across frames, expires old evidence, and shifts the
grid with the robot. Unknown stays unknown; occupancy is never inferred
outside observed depth rays. Evidence timestamps use monotonic nanoseconds.
"""
import math


class EvidenceMap:
    def __init__(self, settings, *, evidence_ttl_ns=3_000_000_000):
        if type(evidence_ttl_ns) is not int or evidence_ttl_ns <= 0:
            raise ValueError("invalid evidence lifetime")
        self.settings = settings
        self.ttl_ns = evidence_ttl_ns
        self.origin = None
        self.cells = {}
        self.last_update_ns = None

    def reset(self):
        self.cells.clear()
        self.origin = None
        self.last_update_ns = None

    def update(self, *, origin_x, origin_y, observed, now_ns):
        s = self.settings
        if (type(now_ns) is not int or now_ns <= 0 or
                (self.last_update_ns is not None and now_ns <= self.last_update_ns)
                or len(observed) != s.width * s.height or
                not math.isfinite(origin_x) or not math.isfinite(origin_y)):
            self.reset()
            raise ValueError("invalid evidence frame or clock regression")
        # Use world-aligned integer cell coordinates, so moving windows do not
        # shift or smear prior obstacles due to floating point rounding.
        base_x = round(origin_x / s.resolution)
        base_y = round(origin_y / s.resolution)
        self.origin = (base_x, base_y)
        expiry = now_ns - self.ttl_ns
        self.cells = {key: val for key, val in self.cells.items()
                      if val[1] >= expiry
                      and base_x <= key[0] < base_x+s.width
                      and base_y <= key[1] < base_y+s.height}
        for i, value in enumerate(observed):
            if value not in (-1, 0, 100):
                self.reset()
                raise ValueError("unexpected occupancy value")
            if value == -1:
                continue
            key = (base_x + i % s.width, base_y + i // s.width)
            # Occupied wins within a simultaneous frame. In later frames a
            # directly measured free traversal can clear an old obstacle.
            self.cells[key] = (value, now_ns)
        self.last_update_ns = now_ns
        return self.snapshot(now_ns=now_ns)

    def snapshot(self, *, now_ns):
        if self.origin is None or type(now_ns) is not int or now_ns <= 0:
            raise ValueError("no valid mapping snapshot")
        if self.last_update_ns is not None and now_ns < self.last_update_ns:
            raise ValueError("regressed clock")
        bx, by = self.origin
        grid = [-1] * (self.settings.width * self.settings.height)
        for (x, y), (value, stamp) in self.cells.items():
            if (0 <= now_ns-stamp <= self.ttl_ns and
                    bx <= x < bx+self.settings.width and
                    by <= y < by+self.settings.height):
                grid[(y-by)*self.settings.width+(x-bx)] = value
        return (bx*self.settings.resolution, by*self.settings.resolution,
                tuple(grid))
