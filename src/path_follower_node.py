#!/usr/bin/env python3
"""ROS 2 node: drive the Audibot model along /planned_path with pure pursuit (kinematic, RViz only)."""

import bisect
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


FRAME_ID = "map"
CHILD_FRAME_ID = "base_footprint"

# Audibot
WHEELBASE = 2.652
FRONT_TRACK = 1.638
REAR_TRACK = 1.6
WHEEL_RADIUS = 0.36
MAX_STEER = 0.6
WHEEL_JOINTS = ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")
STEER_JOINTS = ("steer_fl", "steer_fr")
MAX_SUBSTEP = 0.02  # seconds of simulated time per integration step
SEARCH_AHEAD = 50.0  # meters of path searched when projecting the car onto the route
BRAKE_DECEL = 2.0  # m/s^2 used to slow down before the goal
MIN_SPEED = 0.5


def ackermann_angles(delta):
    """Split a bicycle-model steering angle into (left, right) front wheel angles."""
    if abs(delta) < 1e-6:
        return 0.0, 0.0
    radius = WHEELBASE / math.tan(delta)
    left = math.atan(WHEELBASE / (radius - FRONT_TRACK / 2.0))
    right = math.atan(WHEELBASE / (radius + FRONT_TRACK / 2.0))
    clamp = lambda a: max(-MAX_STEER, min(MAX_STEER, a))
    return clamp(left), clamp(right)


def wheel_rolling_speeds(speed, yaw_rate, left, right):
    """Ground speed each wheel rolls at, in WHEEL_JOINTS order.

    Each wheel sits at (x, y) in the rear-axle frame and rolls along its own heading. Its contact
    velocity is (speed - yaw_rate * y, yaw_rate * x), so inner wheels roll slower and outer or
    front wheels faster than the chassis speed while turning.
    """
    wheels = (
        (WHEELBASE, FRONT_TRACK / 2.0, left),    # wheel_fl
        (WHEELBASE, -FRONT_TRACK / 2.0, right),  # wheel_fr
        (0.0, REAR_TRACK / 2.0, 0.0),            # wheel_rl
        (0.0, -REAR_TRACK / 2.0, 0.0),           # wheel_rr
    )
    return [
        (speed - yaw_rate * y) * math.cos(phi) + yaw_rate * x * math.sin(phi)
        for x, y, phi in wheels
    ]


class PurePursuit:
    """Kinematic bicycle model, referenced at the rear axle (base_footprint), tracking a polyline."""

    def __init__(self, points, lookahead_min, lookahead_time, max_lateral_accel, goal_tolerance):
        pts = [points[0]]
        for p in points[1:]:
            if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-3:
                pts.append(p)
        self.points = pts
        self.cum = [0.0]
        for a, b in zip(pts, pts[1:]):
            self.cum.append(self.cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = self.cum[-1]
        self.lookahead_min = lookahead_min
        self.lookahead_time = lookahead_time
        self.max_lateral_accel = max_lateral_accel
        self.goal_tolerance = goal_tolerance

        self.x, self.y = pts[0]
        if len(pts) > 1:
            self.yaw = math.atan2(pts[1][1] - pts[0][1], pts[1][0] - pts[0][0])
        else:
            self.yaw = 0.0
        self.segment = 0
        self.progress = 0.0
        self.done = len(pts) < 2

    def point_at(self, s):
        s = max(0.0, min(self.length, s))
        i = min(bisect.bisect_right(self.cum, s) - 1, len(self.points) - 2)
        seg_len = self.cum[i + 1] - self.cum[i]
        t = (s - self.cum[i]) / seg_len if seg_len > 0 else 0.0
        (ax, ay), (bx, by) = self.points[i], self.points[i + 1]
        return ax + t * (bx - ax), ay + t * (by - ay)

    def _project(self):
        """Advance self.progress to the closest path point ahead of the previous projection."""
        best_d2, best_seg, best_s = None, self.segment, self.progress
        i = self.segment
        while i < len(self.points) - 1 and self.cum[i] <= self.progress + SEARCH_AHEAD:
            (ax, ay), (bx, by) = self.points[i], self.points[i + 1]
            vx, vy = bx - ax, by - ay
            seg_len2 = vx * vx + vy * vy
            t = ((self.x - ax) * vx + (self.y - ay) * vy) / seg_len2 if seg_len2 > 0 else 0.0
            t = max(0.0, min(1.0, t))
            px, py = ax + t * vx, ay + t * vy
            d2 = (self.x - px) ** 2 + (self.y - py) ** 2
            if best_d2 is None or d2 < best_d2:
                best_d2, best_seg, best_s = d2, i, self.cum[i] + t * math.sqrt(seg_len2)
            i += 1
        self.segment = best_seg
        self.progress = max(self.progress, best_s)

    def step(self, dt, cruise_speed):
        """Advance the car by dt seconds. Returns (speed, steering angle, yaw rate)."""
        if self.done:
            return 0.0, 0.0, 0.0
        self._project()
        remaining = self.length - self.progress
        if remaining <= self.goal_tolerance:
            self.done = True
            return 0.0, 0.0, 0.0

        lookahead = max(self.lookahead_min, self.lookahead_time * cruise_speed)
        tx, ty = self.point_at(self.progress + lookahead)
        dx, dy = tx - self.x, ty - self.y
        dist = max(math.hypot(dx, dy), 1e-6)
        alpha = math.atan2(math.sin(math.atan2(dy, dx) - self.yaw), math.cos(math.atan2(dy, dx) - self.yaw))
        if abs(alpha) > math.pi / 2.0:
            # Target is behind the car: turn as hard as possible toward it.
            delta = math.copysign(MAX_STEER, alpha)
        else:
            delta = math.atan(WHEELBASE * 2.0 * math.sin(alpha) / dist)
        delta = max(-MAX_STEER, min(MAX_STEER, delta))

        speed = cruise_speed
        curvature = abs(math.tan(delta)) / WHEELBASE
        if curvature > 1e-6:
            speed = min(speed, math.sqrt(self.max_lateral_accel / curvature))
        speed = min(speed, max(MIN_SPEED, math.sqrt(2.0 * BRAKE_DECEL * remaining)))

        self.x += speed * math.cos(self.yaw) * dt
        self.y += speed * math.sin(self.yaw) * dt
        yaw_rate = speed / WHEELBASE * math.tan(delta)
        self.yaw += yaw_rate * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        return speed, delta, yaw_rate


class PathFollower(Node):
    def __init__(self):
        super().__init__("audibot_path_follower")
        self.declare_parameter("speed", 15.0)
        self.declare_parameter("time_scale", 1.0)
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("lookahead_min", 6.0)
        self.declare_parameter("lookahead_time", 0.8)
        self.declare_parameter("max_lateral_accel", 3.0)
        self.declare_parameter("goal_tolerance", 1.0)

        self.path_sub = self.create_subscription(Path, "/planned_path", self.path_callback, 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tracker = None
        self.x = self.y = self.yaw = 0.0
        self.steer = 0.0
        self.wheel_angles = [0.0] * len(WHEEL_JOINTS)
        self.distance = 0.0
        self.goal_reported = True
        self.last_time = self.get_clock().now()
        rate = float(self.get_parameter("rate").value)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info("Audibot path follower ready. Waiting on /planned_path.")

    def path_callback(self, msg):
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not points:
            self.get_logger().warn("Received an empty /planned_path; ignoring it.")
            return
        if msg.header.frame_id and msg.header.frame_id != FRAME_ID:
            self.get_logger().warn("Path frame '%s' is not '%s'; using it as-is." % (msg.header.frame_id, FRAME_ID))
        self.tracker = PurePursuit(
            points,
            float(self.get_parameter("lookahead_min").value),
            float(self.get_parameter("lookahead_time").value),
            float(self.get_parameter("max_lateral_accel").value),
            float(self.get_parameter("goal_tolerance").value),
        )
        self.x, self.y, self.yaw = self.tracker.x, self.tracker.y, self.tracker.yaw
        self.steer = 0.0
        self.distance = 0.0
        self.goal_reported = False
        self.get_logger().info("Following new route: %d poses, %.2f km." % (len(points), self.tracker.length / 1000.0))

    def tick(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if self.tracker is not None and not self.tracker.done and elapsed > 0.0:
            cruise = float(self.get_parameter("speed").value)
            sim_dt = min(elapsed, 0.5) * float(self.get_parameter("time_scale").value)
            steps = max(1, int(math.ceil(sim_dt / MAX_SUBSTEP)))
            h = sim_dt / steps
            for _ in range(steps):
                speed, self.steer, yaw_rate = self.tracker.step(h, cruise)
                left, right = ackermann_angles(self.steer)
                rolls = wheel_rolling_speeds(speed, yaw_rate, left, right)
                for i, roll in enumerate(rolls):
                    self.wheel_angles[i] += roll * h / WHEEL_RADIUS
                self.distance += speed * h
                if self.tracker.done:
                    break
            self.wheel_angles = [math.remainder(a, 2.0 * math.pi) for a in self.wheel_angles]
            self.x, self.y, self.yaw = self.tracker.x, self.tracker.y, self.tracker.yaw

        if self.tracker is not None and self.tracker.done and not self.goal_reported:
            self.goal_reported = True
            self.steer = 0.0
            self.get_logger().info("Reached goal after %.2f km." % (self.distance / 1000.0))

        self.publish_state(now.to_msg())

    def publish_state(self, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = FRAME_ID
        tf.child_frame_id = CHILD_FRAME_ID
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = math.sin(self.yaw / 2.0)
        tf.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.tf_broadcaster.sendTransform(tf)

        left, right = ackermann_angles(self.steer)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = list(WHEEL_JOINTS) + list(STEER_JOINTS)
        joints.position = list(self.wheel_angles) + [left, right]
        self.joint_pub.publish(joints)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
