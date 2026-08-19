import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

import math


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


class WheelOdometryNode(Node):
    """
    Input:
      /enc_vel  std_msgs/Float32MultiArray [FL, FR, RL, RR] (m/s)

    Output:
      /wheel/odom nav_msgs/Odometry (odom -> base_link pose + twist)

    Model:
      Asymmetric 4WD skid-steer with a narrower front axle and a wider rear axle.
    """

    def __init__(self):
        super().__init__('wheel_odometry')

        # ---- Parameters ----
        self.declare_parameter('enc_vel_topic', 'enc_vel')
        self.declare_parameter('odom_topic', 'wheel/odom')

        self.declare_parameter('front_track_width', 0.75)
        self.declare_parameter('rear_track_width', 1.17)
        self.declare_parameter('wheel_scale_fl', 1.0)
        self.declare_parameter('wheel_scale_fr', 1.0)
        self.declare_parameter('wheel_scale_rl', 1.0)
        self.declare_parameter('wheel_scale_rr', 1.0)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('timeout_sec', 0.5)

        self.enc_vel_topic = self.get_parameter('enc_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.front_track_width = float(self.get_parameter('front_track_width').value)
        self.rear_track_width = float(self.get_parameter('rear_track_width').value)
        self.wheel_scales = (
            float(self.get_parameter('wheel_scale_fl').value),
            float(self.get_parameter('wheel_scale_fr').value),
            float(self.get_parameter('wheel_scale_rl').value),
            float(self.get_parameter('wheel_scale_rr').value),
        )
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        # ---- State ----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_enc_time = None
        self.last_update_time = self.get_clock().now()

        self.v_fl = 0.0
        self.v_fr = 0.0
        self.v_rl = 0.0
        self.v_rr = 0.0

        # ---- ROS I/O ----
        self.sub = self.create_subscription(
            Float32MultiArray,
            self.enc_vel_topic,
            self.enc_cb,
            20
        )

        self.pub = self.create_publisher(Odometry, self.odom_topic, 20)

        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self.update)

        self.get_logger().info(
            'Wheel odometry: '
            f'sub={self.enc_vel_topic} '
            f'pub={self.odom_topic} '
            f'front_track_width={self.front_track_width:.3f} '
            f'rear_track_width={self.rear_track_width:.3f} '
            f'wheel_scales={self.wheel_scales} '
            f'linear_scale={self.linear_scale:.3f} '
            f'angular_scale={self.angular_scale:.3f}'
        )

    def enc_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('enc_vel must contain 4 elements [FL, FR, RL, RR]')
            return

        self.v_fl = float(msg.data[0]) * self.wheel_scales[0]
        self.v_fr = float(msg.data[1]) * self.wheel_scales[1]
        self.v_rl = float(msg.data[2]) * self.wheel_scales[2]
        self.v_rr = float(msg.data[3]) * self.wheel_scales[3]

        self.last_enc_time = self.get_clock().now()

    def compute_body_twist(self):
        vx = 0.25 * (self.v_fl + self.v_fr + self.v_rl + self.v_rr)

        front_delta = self.v_fr - self.v_fl
        rear_delta = self.v_rr - self.v_rl
        denom = (self.front_track_width ** 2) + (self.rear_track_width ** 2)
        if denom <= 1e-9:
            wz = 0.0
        else:
            wz = (
                (self.front_track_width * front_delta) +
                (self.rear_track_width * rear_delta)
            ) / denom

        return vx * self.linear_scale, wz * self.angular_scale

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_update_time = now

        if self.last_enc_time is None:
            v = 0.0
            w = 0.0
        else:
            age = (now - self.last_enc_time).nanoseconds / 1e9
            if age > self.timeout_sec:
                v = 0.0
                w = 0.0
            else:
                v, w = self.compute_body_twist()

        self.yaw += w * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        q = quaternion_from_euler(0.0, 0.0, self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = w

        odom.pose.covariance[0] = 0.05 * 0.05
        odom.pose.covariance[7] = 0.05 * 0.05
        odom.pose.covariance[35] = 0.10 * 0.10

        odom.twist.covariance[0] = 0.10 * 0.10
        odom.twist.covariance[35] = 0.20 * 0.20

        self.pub.publish(odom)



def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
