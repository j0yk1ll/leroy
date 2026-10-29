"""Conservative bridge-owned freshness from REAL sensor/nav messages.

A valid ROS message is not proof that physical depth or localization is
calibrated. These gates only block absent/stale/malformed evidence.
"""

# TODO(AUTONOMY) [trusted dynamic health]: Current checks prove message
# shape/freshness only. Add measured acquisition-to-receipt timestamps,
# transform history, odometry pose covariance, tracking scale confidence,
# independent sensor provenance and complete obstacle field coverage.
# TODO(AUTONOMY) [unlocalized boot policy]: Distinguish missing *metric*
# localization from otherwise verified bounded startup motion; do not
# fake localization_valid just to unlock an initializing SLAM system.
# Acceptance: timestamps from stale/duplicate publishers, invalid geometry,
# monocular scale ambiguity and sensor blindness make health unavailable.

import math
import time

ODOM_MAX_NS = 300_000_000
TF_MAX_NS = 300_000_000
DEPTH_MAX_NS = 1_000_000_000
COSTMAP_MAX_NS = 500_000_000


class MotorSensorHealth:
    def __init__(self):
        self.timestamps = {}
        self.tracking = False
        self.localization = False

    def tracking_state(self, msg):
        self.tracking = msg.data == "tracking"

    def localization_state(self, msg):
        self.localization = msg.data is True

    def odom(self, msg):
        p = msg.pose.pose
        v = (p.position.x, p.position.y, p.position.z,
             p.orientation.x, p.orientation.y,
             p.orientation.z, p.orientation.w,
             msg.twist.twist.linear.x, msg.twist.twist.linear.y,
             msg.twist.twist.angular.z)
        qnorm = sum(x*x for x in v[3:7])
        if (msg.header.frame_id == "odom"
                and msg.child_frame_id == "base_link"
                and msg.header.stamp.sec > 0
                and all(math.isfinite(x) for x in v)
                and abs(qnorm - 1.0) < 0.01):
            self.timestamps["odom"] = time.monotonic_ns()
        else:
            self.timestamps.pop("odom", None)

    def depth(self, msg):
        if (msg.encoding == "32FC1" and msg.width > 0
                and msg.height > 0 and msg.header.frame_id
                and msg.header.stamp.sec > 0
                and msg.step >= msg.width * 4
                and len(msg.data) >= msg.step * msg.height):
            self.timestamps["depth"] = time.monotonic_ns()
        else:
            self.timestamps.pop("depth", None)

    def dynamic_tf(self, msg):
        for t in msg.transforms:
            q = t.transform.rotation
            v = (q.x, q.y, q.z, q.w)
            if (t.header.frame_id == "odom" and
                    t.child_frame_id == "base_link" and
                    t.header.stamp.sec > 0 and
                    all(math.isfinite(x) for x in v) and
                    abs(sum(x*x for x in v) - 1.0) < 0.01):
                self.timestamps["tf"] = time.monotonic_ns()
                return

    def costmap(self, msg):
        info = msg.info
        if (msg.header.frame_id == "odom" and
                msg.header.stamp.sec > 0 and
                info.width > 0 and info.height > 0 and
                math.isfinite(info.resolution) and
                info.resolution > 0.0 and
                len(msg.data) == info.width * info.height and
                any(0 <= x <= 100 for x in msg.data)):
            self.timestamps["costmap"] = time.monotonic_ns()
        else:
            self.timestamps.pop("costmap", None)

    def state(self, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        def fresh(kind, limit):
            t = self.timestamps.get(kind)
            return t is not None and 0 <= now_ns - t <= limit
        odom = fresh("odom", ODOM_MAX_NS)
        depth = fresh("depth", DEPTH_MAX_NS)
        tf = fresh("tf", TF_MAX_NS)
        costmap = fresh("costmap", COSTMAP_MAX_NS)
        return {
            "odom_fresh": odom,
            "depth_fresh": depth,
            "tf_fresh": tf,
            "costmap_valid": costmap,
            "localization_valid": self.tracking and self.localization and odom,
        }
