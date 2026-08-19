import math
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import QuaternionStamped
from tf_transformations import quaternion_from_euler


class UM982HeadingNode(Node):
    """
    Reads $GNHPR from UM982 and publishes:
      /gnss/heading_quat (QuaternionStamped)

    Assumption:
      - GNHPR heading: 0 deg = North, 90 deg = East, clockwise positive
      - ROS ENU yaw: CCW from +X (East)

    Conversion:
      yaw_rad = radians(90 - heading_deg)
    """

    def __init__(self):
        super().__init__('um982_heading')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'gnss_heading_link')
        self.declare_parameter('min_qf', 1)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.min_qf = int(self.get_parameter('min_qf').value)

        self.pub = self.create_publisher(QuaternionStamped, 'gnss/heading_quat', 10)

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open UM982 serial {self.port}: {e}')
            raise

        self.timer = self.create_timer(0.02, self.read_serial)  # 50 Hz poll
        self.get_logger().info(f'UM982 heading reading from {self.port} @ {self.baudrate}')

    def read_serial(self):
        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline()
                if not raw:
                    return
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line.startswith('$GNHPR'):
                    continue
                self.parse_gnhpr(line)
        except serial.SerialException as e:
            self.get_logger().error(f'UM982 serial read error: {e}')

    def parse_gnhpr(self, line: str):
        # remove checksum
        if '*' in line:
            line = line.split('*', 1)[0]

        fields = line.split(',')
        if len(fields) < 6:
            return

        try:
            heading_deg = float(fields[2])
        except ValueError:
            return

        try:
            qf = int(fields[5])
        except ValueError:
            qf = 0

        if qf < self.min_qf:
            return

        yaw_rad = math.radians(90.0 - heading_deg)
        q = quaternion_from_euler(0.0, 0.0, yaw_rad)

        msg = QuaternionStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.quaternion.x = q[0]
        msg.quaternion.y = q[1]
        msg.quaternion.z = q[2]
        msg.quaternion.w = q[3]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UM982HeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
