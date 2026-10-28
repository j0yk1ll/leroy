"""ROS-independent contract for an actual stamped depth-to-cloud chain.

A link PASS proves message transport/shape and matching acquisition stamps;
it deliberately does not prove obstacle geometry, depth accuracy or safety.
"""
from collections import OrderedDict
import math

CACHE_SIZE = 128
MAX_AGE_S = 2.0


def stamp_key(stamp):
    sec = int(stamp.sec)
    ns = int(stamp.nanosec)
    if sec <= 0 or not 0 <= ns < 1_000_000_000:
        raise ValueError("invalid acquisition timestamp")
    return sec * 1_000_000_000 + ns


class LiveChain:
    def __init__(self):
        self.info = OrderedDict()
        self.depth = OrderedDict()
        self.cloud = OrderedDict()
        self.matched = set()
        self.ages = []
        self.faults = []

    @staticmethod
    def _remember(cache, stamp, value):
        cache[stamp] = value
        cache.move_to_end(stamp)
        while len(cache) > CACHE_SIZE:
            cache.popitem(last=False)

    @staticmethod
    def _header(frame, width, height):
        if not frame or width < 1 or height < 1:
            raise ValueError("invalid image frame or dimensions")
        return frame, width, height

    def on_info(self, stamp, frame, width, height, p):
        if len(p) != 12 or not all(math.isfinite(float(x)) for x in p):
            raise ValueError("invalid intrinsics")
        if p[0] <= 0 or p[5] <= 0:
            raise ValueError("uncalibrated camera")
        self._remember(self.info, stamp, self._header(frame, width, height))
        return self._link()

    def on_depth(self, stamp, frame, width, height, encoding, step, size,
                 age_s):
        if (encoding != "32FC1" or step < width * 4 or step % 4
                or size < step * height):
            raise ValueError("invalid depth image layout")
        if not math.isfinite(age_s) or not 0 <= age_s <= MAX_AGE_S:
            raise ValueError("stale/future depth frame")
        self._remember(self.depth, stamp, (
            self._header(frame, width, height), age_s))
        return self._link()

    def on_cloud(self, stamp, frame, width, height, point_step, size):
        if (not frame or width < 1 or height < 1
                or point_step < 12 or size < width * height * point_step):
            raise ValueError("empty/malformed optical point cloud")
        self._remember(self.cloud, stamp, frame)
        return self._link()

    def _link(self):
        # Arrival order does not matter. Match EXACT acquisition timestamps
        # without substituting recent timestamps to make results look good.
        common = self.info.keys() & self.depth.keys() & self.cloud.keys()
        for key in common:
            if key in self.matched:
                continue
            info = self.info[key]
            depth, age = self.depth[key]
            if info != depth or self.cloud[key] != depth[0]:
                self.faults.append("camera-depth-cloud frame/dimension mismatch")
                continue
            self.matched.add(key)
            self.ages.append(age)
        return len(self.matched)
