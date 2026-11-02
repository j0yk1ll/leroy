"""Odometry-frame goal follower.

Publishes on isolated /leroy/navigation/cmd_vel; never on /cmd_vel or
the signed motor topic. Requires metric, calibrated odometry to be useful.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node

from leroy_navigation.goal_controller import GoalController, Pose
from leroy_navigation.grid_planner import Grid, plan


def yaw_of(q):
    norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    if not math.isfinite(norm) or abs(norm-1.0)>1e-3:
        raise ValueError("invalid quaternion")
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class GoalNode(Node):
    def __init__(self):
        super().__init__("leroy_goal_controller")
        self.declare_parameter("max_linear_mps", 0.10)
        self.declare_parameter("max_angular_radps", 0.20)
        self.declare_parameter("odom_timeout_s", 0.25)
        self.controller = GoalController(
            max_linear=float(self.get_parameter("max_linear_mps").value),
            max_angular=float(self.get_parameter("max_angular_radps").value))
        self.timeout_ns = int(float(self.get_parameter("odom_timeout_s").value)*1e9)
        if not 0 < self.timeout_ns <= 500_000_000:
            raise ValueError("invalid odometry timeout")
        self.odom = None
        self.odom_received = None
        self.odom_stamp = None
        self.goal = None
        self.grid = None
        self.map_received = None
        self.route = None
        self.route_index = 0
        self.route_planned_at = None
        self.declare_parameter("map_timeout_s", 2.0)
        self.declare_parameter("replan_period_s", 0.5)
        self.declare_parameter("inflation_radius_m", 0.30)
        self.map_timeout_ns = int(float(self.get_parameter("map_timeout_s").value) * 1e9)
        self.replan_ns = int(float(self.get_parameter("replan_period_s").value) * 1e9)
        self.inflation_radius = float(self.get_parameter("inflation_radius_m").value)
        if self.map_timeout_ns <= 0 or self.replan_ns <= 0:
            raise ValueError("invalid map timing")
        self.pub = self.create_publisher(Twist, "/leroy/navigation/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(OccupancyGrid, "/leroy/navigation/occupancy_grid", self.on_map, 10)
        self.create_subscription(PoseStamped, "/leroy/navigation/goal", self.on_goal, 10)
        self.create_timer(0.05, self.tick)

    def on_odom(self, msg):
        try:
            if msg.header.frame_id != "odom" or msg.child_frame_id != "base_link":
                raise ValueError("wrong odometry frames")
            stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
            if stamp <= 0 or (self.odom_stamp is not None and stamp <= self.odom_stamp):
                raise ValueError("nonmonotonic odometry")
            p = msg.pose.pose.position
            pose = Pose(p.x, p.y, yaw_of(msg.pose.pose.orientation))
            if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw)):
                raise ValueError("invalid pose")
            now = self.get_clock().now().nanoseconds
            if not 0 <= now - stamp <= self.timeout_ns:
                raise ValueError("stale odometry")
            self.odom = pose
            self.odom_stamp = stamp
            self.odom_received = time.monotonic_ns()
        except ValueError:
            self.odom = None
            self.odom_received = None

    def on_goal(self, msg):
        try:
            if msg.header.frame_id != "odom":
                raise ValueError("goal must be in odom frame")
            p = msg.pose.position
            target = Pose(p.x, p.y, yaw_of(msg.pose.orientation))
            if not all(math.isfinite(v) for v in (target.x, target.y, target.yaw)):
                raise ValueError("nonfinite goal")
            self.goal = target
            self.route = None
            self.route_index = 0
        except ValueError:
            self.goal = None
            self.route = None

    def on_map(self, msg):
        try:
            if msg.header.frame_id != "odom":
                raise ValueError("map must use odom frame")
            q = msg.info.origin.orientation
            if abs(yaw_of(q)) > 1e-5:
                raise ValueError("rotated map origins not supported")
            p = msg.info.origin.position
            self.grid = Grid(
                int(msg.info.width), int(msg.info.height),
                float(msg.info.resolution), float(p.x), float(p.y),
                tuple(int(x) for x in msg.data),
                inflation_radius_m=self.inflation_radius)
            self.map_received = time.monotonic_ns()
            self.route = None
        except (ValueError, TypeError, OverflowError):
            self.grid = None
            self.route = None
            self.map_received = None

    def tick(self):
        cmd = Twist()
        now = time.monotonic_ns()
        if (self.goal is not None and self.odom is not None
                and self.grid is not None
                and self.odom_received is not None and self.map_received is not None
                and 0 <= now-self.odom_received <= self.timeout_ns
                and 0 <= now-self.map_received <= self.map_timeout_ns):
            try:
                if (self.route is None or self.route_planned_at is None
                        or now-self.route_planned_at >= self.replan_ns):
                    self.route = plan(
                        self.grid, (self.odom.x, self.odom.y),
                        (self.goal.x, self.goal.y))
                    self.route_index = 0
                    self.route_planned_at = now
                if self.route:
                    while (self.route_index < len(self.route) and
                           math.hypot(self.route[self.route_index][0]-self.odom.x,
                                      self.route[self.route_index][1]-self.odom.y)
                           <= self.controller.position_tolerance):
                        self.route_index += 1
                    if self.route_index < len(self.route):
                        x, y = self.route[self.route_index]
                        target = Pose(x, y, self.odom.yaw)
                    else:
                        target = self.goal
                    result = self.controller.calculate(self.odom, target)
                    cmd.linear.x, cmd.linear.y, cmd.angular.z = (
                        result.vx, result.vy, result.wz)
                    if target is self.goal and result.arrived:
                        self.goal = None
                        self.route = None
            except ValueError:
                self.route = None
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
