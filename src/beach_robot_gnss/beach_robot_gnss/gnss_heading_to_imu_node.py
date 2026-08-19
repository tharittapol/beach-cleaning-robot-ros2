import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu


class GnssHeadingToImu(Node):
    """
    Input:
      /gnss/heading_quat (QuaternionStamped)

    Output:
      /gnss/imu_heading (sensor_msgs/Imu) with orientation only.
    """

    def __init__(self):
        super().__init__('gnss_heading_to_imu')

        self.declare_parameter('input_topic', 'gnss/heading_quat')
        self.declare_parameter('output_topic', 'gnss/imu_heading')
        self.declare_parameter('frame_id', 'gnss_heading_link')
        self.declare_parameter('yaw_std_deg', 2.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        yaw_std_deg = float(self.get_parameter('yaw_std_deg').value)

        self.yaw_var = math.radians(yaw_std_deg) ** 2

        self.pub = self.create_publisher(Imu, self.output_topic, 10)
        self.sub = self.create_subscription(QuaternionStamped, self.input_topic, self.cb, 10)

        self.get_logger().info(
            f'Publishing {self.output_topic} from {self.input_topic} (yaw_std_deg={yaw_std_deg})'
        )

    def cb(self, msg: QuaternionStamped):
        imu = Imu()
        imu.header.stamp = msg.header.stamp
        imu.header.frame_id = self.frame_id

        imu.orientation = msg.quaternion

        # GNSS heading gives yaw only; roll/pitch are unknown -> huge covariance
        roll_var = 1e6
        pitch_var = 1e6
        yaw_var = self.yaw_var
        imu.orientation_covariance = [
            roll_var, 0.0,     0.0,
            0.0,     pitch_var, 0.0,
            0.0,     0.0,     yaw_var
        ]

        # mark others unknown so EKF ignores them
        imu.angular_velocity_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 1e6
        ]
        imu.linear_acceleration_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, 1e6
        ]

        self.pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = GnssHeadingToImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
