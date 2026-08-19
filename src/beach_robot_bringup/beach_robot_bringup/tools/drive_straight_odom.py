import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(rad):
    return math.atan2(math.sin(rad), math.cos(rad))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class DriveStraightOdom(Node):
    def __init__(self, args):
        super().__init__("drive_straight_odom")
        self.args = args
        self.latest_odom = None
        self.start_pose = None
        self.last_progress_report_m = -1
        self.done = False
        self.result = "not_started"

        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 20)

        self.get_logger().info(
            "Drive straight by odom: "
            f"odom={args.odom_topic} cmd={args.cmd_topic} "
            f"distance={args.distance_m:.3f}m speed={args.speed_mps:.3f}m/s"
        )

    def odom_cb(self, msg):
        self.latest_odom = msg

    def stop_robot(self, count=10):
        stop = Twist()
        for _ in range(count):
            self.cmd_pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def wait_for_odom(self):
        deadline = time.monotonic() + self.args.wait_odom_sec
        while rclpy.ok() and self.latest_odom is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.latest_odom is not None

    def capture_start(self):
        pose = self.latest_odom.pose.pose
        self.start_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
        )
        self.get_logger().info(
            "Captured start pose: "
            f"x={self.start_pose[0]:.3f} y={self.start_pose[1]:.3f} "
            f"yaw={math.degrees(self.start_pose[2]):.1f}deg"
        )

    def progress(self):
        pose = self.latest_odom.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = yaw_from_quaternion(pose.orientation)

        x0, y0, yaw0 = self.start_pose
        dx = x - x0
        dy = y - y0
        direction = -1.0 if self.args.reverse else 1.0
        along = direction * (dx * math.cos(yaw0) + dy * math.sin(yaw0))
        cross = direction * (-dx * math.sin(yaw0) + dy * math.cos(yaw0))
        yaw_error = normalize_angle(yaw0 - yaw)
        return along, cross, yaw_error

    def command_for_progress(self, along, yaw_error):
        remaining = max(0.0, self.args.distance_m - along)
        speed = abs(self.args.speed_mps)
        if remaining < self.args.slowdown_distance_m:
            scale = max(self.args.min_speed_mps / max(speed, 1e-6), remaining / self.args.slowdown_distance_m)
            speed *= scale
        if self.args.reverse:
            speed = -speed

        cmd = Twist()
        cmd.linear.x = speed
        if not self.args.no_yaw_hold:
            cmd.angular.z = max(
                -self.args.max_angular_radps,
                min(self.args.max_angular_radps, self.args.yaw_kp * yaw_error),
            )
        return cmd

    def run(self):
        if not self.wait_for_odom():
            self.result = "no_odom"
            self.get_logger().error(f"No odom received on {self.args.odom_topic}")
            self.stop_robot()
            return False

        self.capture_start()
        time.sleep(max(0.0, self.args.settle_sec))

        start_time = time.monotonic()
        period = 1.0 / max(1.0, self.args.rate_hz)
        next_tick = start_time

        while rclpy.ok():
            now = time.monotonic()
            if now < next_tick:
                rclpy.spin_once(self, timeout_sec=min(0.02, next_tick - now))
                continue
            next_tick += period

            rclpy.spin_once(self, timeout_sec=0.0)
            if self.latest_odom is None:
                continue

            along, cross, yaw_error = self.progress()
            meter = int(math.floor(max(0.0, along)))
            if meter > self.last_progress_report_m:
                self.last_progress_report_m = meter
                self.get_logger().info(
                    f"progress={along:.2f}/{self.args.distance_m:.2f}m "
                    f"cross={cross:.2f}m yaw_err={math.degrees(yaw_error):.1f}deg"
                )

            if along >= self.args.distance_m:
                self.result = "target_reached"
                break

            if (now - start_time) > self.args.timeout_sec:
                self.result = "timeout"
                self.get_logger().error(
                    f"Timeout before target: progress={along:.2f}/{self.args.distance_m:.2f}m"
                )
                break

            self.cmd_pub.publish(self.command_for_progress(along, yaw_error))

        self.stop_robot()
        along, cross, yaw_error = self.progress()
        self.get_logger().info(
            f"Finished: result={self.result} progress={along:.3f}m "
            f"cross={cross:.3f}m yaw_err={math.degrees(yaw_error):.2f}deg"
        )
        return self.result == "target_reached"


def main():
    parser = argparse.ArgumentParser(description="Drive straight until odometry reports the requested distance.")
    parser.add_argument("--odom-topic", default="/odometry/fusion_bno")
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--distance-m", type=float, default=10.0)
    parser.add_argument("--speed-mps", type=float, default=0.15)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--wait-odom-sec", type=float, default=5.0)
    parser.add_argument("--settle-sec", type=float, default=0.5)
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--no-yaw-hold", action="store_true")
    parser.add_argument("--yaw-kp", type=float, default=1.2)
    parser.add_argument("--max-angular-radps", type=float, default=0.12)
    parser.add_argument("--slowdown-distance-m", type=float, default=0.5)
    parser.add_argument("--min-speed-mps", type=float, default=0.05)
    args = parser.parse_args()

    rclpy.init()
    node = DriveStraightOdom(args)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
