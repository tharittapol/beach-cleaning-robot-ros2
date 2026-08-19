#!/usr/bin/env python3
import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32


class GpsJitterNode(Node):
    def __init__(self):
        super().__init__('gps_jitter_node')

        # Params
        self.fix_topic = self.declare_parameter('fix_topic', '/gps/fix').value
        self.window_sec = float(self.declare_parameter('window_sec', 60.0).value)
        self.min_samples = int(self.declare_parameter('min_samples', 30).value)
        self.outlier_m = float(self.declare_parameter('outlier_m', 2.0).value)
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 1.0).value)

        # Buffer of (t, lat, lon)
        self.buf = deque()

        # Pubs (meters)
        self.pub_r95 = self.create_publisher(Float32, 'gps/jitter_r95_m', 10)
        self.pub_rms = self.create_publisher(Float32, 'gps/jitter_rms_m', 10)
        self.pub_max = self.create_publisher(Float32, 'gps/jitter_max_m', 10)

        # Sub
        self.sub = self.create_subscription(NavSatFix, self.fix_topic, self.on_fix, 20)

        # Timer to publish periodically (avoid spamming each fix)
        period = 1.0 / max(0.1, self.publish_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(f"Subscribing {self.fix_topic}, window={self.window_sec}s, min_samples={self.min_samples}")

    def on_fix(self, msg: NavSatFix):
        # Only accept valid-ish points
        if msg.status.status < 0:
            return
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        t = time.time()

        self.buf.append((t, lat, lon))

        # drop old
        while self.buf and (t - self.buf[0][0]) > self.window_sec:
            self.buf.popleft()

    def _ll_to_en_m(self, lat0_deg, lon0_deg, lat_deg, lon_deg):
        # small-area approximation (good for local jitter)
        R = 6378137.0
        lat0 = math.radians(lat0_deg)
        dlat = math.radians(lat_deg - lat0_deg)
        dlon = math.radians(lon_deg - lon0_deg)
        dN = dlat * R
        dE = dlon * R * math.cos(lat0)
        return dE, dN

    def on_timer(self):
        if len(self.buf) < self.min_samples:
            return

        # Reference point = first in buffer
        _, lat0, lon0 = self.buf[0]

        EN = []
        for _, la, lo in self.buf:
            e, n = self._ll_to_en_m(lat0, lon0, la, lo)
            EN.append((e, n))

        en = np.array(EN, dtype=np.float64)
        mean = en.mean(axis=0)
        d = en - mean
        r = np.sqrt(d[:, 0] ** 2 + d[:, 1] ** 2)

        # Outlier reject (single-pass)
        r2 = r[r <= self.outlier_m]
        if r2.size < self.min_samples:
            return

        rms = float(np.sqrt(np.mean(r2 ** 2)))
        r95 = float(np.percentile(r2, 95.0))
        rmax = float(np.max(r2))

        self.pub_r95.publish(Float32(data=r95))
        self.pub_rms.publish(Float32(data=rms))
        self.pub_max.publish(Float32(data=rmax))


def main():
    rclpy.init()
    node = GpsJitterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
