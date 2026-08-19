#!/usr/bin/env python3
"""Publish the real-robot footprint + scoop as RViz markers (base_link frame).

Two shapes, latched so RViz picks them up at any time:
  * WHEEL QUAD  - a quadrilateral whose 4 corners are the wheel centres. The robot
    is asymmetric (front caterpillar track narrow 0.734 m, rear round wheels wide
    1.179 m, wheelbase 0.950 m) so this is a trapezoid, NOT a rectangle. Matches the
    Nav2 costmap `footprint` set in nav2_params_tile.yaml.
  * SCOOP       - the sand scoop box, default 0.60 m wide (= tool_width), centred
    under the chassis. Length/offset are params (true fore-aft size not measured yet).

base_link is the chassis centre (URDF: mid-wheelbase, mid-width), so all corners are
symmetric about the origin in x and split by the per-axle half-track in y.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _pt(x, y, z=0.0):
    p = Point(); p.x = float(x); p.y = float(y); p.z = float(z); return p


class FootprintViz(Node):
    def __init__(self):
        super().__init__('footprint_viz')
        # real-robot geometry (matches beach_wheel_mixer/config + CLAUDE.md)
        self.declare_parameter('front_track_width', 0.734)
        self.declare_parameter('rear_track_width', 1.179)
        self.declare_parameter('wheelbase', 0.950)
        # scoop box (under chassis, centred). width given (0.6 m); length/offset are guesses
        self.declare_parameter('scoop_width', 0.60)
        self.declare_parameter('scoop_length', 0.30)
        self.declare_parameter('scoop_x_offset', 0.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('topic', 'robot_footprint_viz')
        self.declare_parameter('publish_period_sec', 0.05)  # 20 Hz so RViz tracks base_link smoothly

        self.ft = float(self.get_parameter('front_track_width').value)
        self.rt = float(self.get_parameter('rear_track_width').value)
        self.wb = float(self.get_parameter('wheelbase').value)
        self.sw = float(self.get_parameter('scoop_width').value)
        self.sl = float(self.get_parameter('scoop_length').value)
        self.sx = float(self.get_parameter('scoop_x_offset').value)
        self.frame = self.get_parameter('frame_id').value
        topic = self.get_parameter('topic').value

        # latched: keep last sample for late-joining RViz
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(MarkerArray, topic, qos)

        self.msg = self._build()
        self.pub.publish(self.msg)
        period = max(0.5, float(self.get_parameter('publish_period_sec').value))
        self.create_timer(period, lambda: self.pub.publish(self.msg))
        self.get_logger().info(
            f'footprint_viz: wheel-quad front={self.ft:.3f} rear={self.rt:.3f} '
            f'wheelbase={self.wb:.3f}; scoop {self.sw:.2f}x{self.sl:.2f} @x={self.sx:+.2f} '
            f'on {self.frame} -> /{topic}')

    def _line_strip(self, mid, ns, corners, color, width=0.03):
        m = Marker()
        m.header.frame_id = self.frame
        m.ns = ns; m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = width
        m.color = color
        m.pose.orientation.w = 1.0
        m.points = [_pt(*c) for c in corners] + [_pt(*corners[0])]  # close loop
        return m

    def _text(self, mid, ns, xy, txt, color):
        m = Marker()
        m.header.frame_id = self.frame
        m.ns = ns; m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.scale.z = 0.12
        m.color = color
        m.pose.position = _pt(xy[0], xy[1], 0.05)
        m.pose.orientation.w = 1.0
        m.text = txt
        return m

    def _build(self):
        hb = self.wb / 2.0
        hf = self.ft / 2.0
        hr = self.rt / 2.0
        # wheel quad: FL, FR, RR, RL  (x forward, y left)
        quad = [(hb, hf), (hb, -hf), (-hb, -hr), (-hb, hr)]
        # scoop rectangle centred at x_offset
        hl = self.sl / 2.0
        hw = self.sw / 2.0
        scoop = [(self.sx + hl, hw), (self.sx + hl, -hw),
                 (self.sx - hl, -hw), (self.sx - hl, hw)]

        green = ColorRGBA(r=0.1, g=0.9, b=0.2, a=1.0)
        orange = ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0)

        arr = MarkerArray()
        arr.markers.append(self._line_strip(0, 'wheel_quad', quad, green, 0.035))
        arr.markers.append(self._line_strip(1, 'scoop', scoop, orange, 0.03))
        arr.markers.append(self._text(2, 'labels', (hb + 0.12, 0.0), 'front', green))
        arr.markers.append(self._text(3, 'labels', (self.sx, 0.0), 'scoop', orange))
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = FootprintViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
