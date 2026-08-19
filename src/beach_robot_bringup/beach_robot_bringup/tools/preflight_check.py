import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from diagnostic_updater import Updater, DiagnosticStatusWrapper

REQUIRED_TOPICS = [
    ('/enc_vel', 'std_msgs/msg/Float32MultiArray'),
    ('/imu/data', 'sensor_msgs/msg/Imu'),
    ('/wheel_cmd', 'std_msgs/msg/Float32MultiArray'),
    ('/cmd_vel', 'geometry_msgs/msg/Twist'),
]

OPTIONAL_TOPICS = [
    ('/gps/fix', 'sensor_msgs/msg/NavSatFix'),
    ('/zed/filtered_cloud', 'sensor_msgs/msg/PointCloud2'),
]

REQUIRED_TF = [
    ('map', 'base_link'),
    ('odom', 'base_link'),
]

class Preflight(Node):
    def __init__(self):
        super().__init__('preflight_check')

        self.updater = Updater(self)
        self.updater.setHardwareID("beach_robot")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.seen = {}  # topic -> last_time
        self.topic_types = {}

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Use generic subscription by topic name only (we track if messages arrive via /rosout introspection)
        # Simpler: rely on topic list + echo in practice.
        # Here: we just track topic existence + TF availability.

        self.timer = self.create_timer(1.0, self.tick)

        self.updater.add("Topics", self.diag_topics)
        self.updater.add("TF", self.diag_tf)

        self.get_logger().info("Preflight running: checking topics and TF...")

    def diag_topics(self, stat: DiagnosticStatusWrapper):
        ok = True
        # Check topic existence via graph
        names_and_types = self.get_topic_names_and_types()

        existing = {n: t for n, t in names_and_types}
        missing = []

        for tname, _ in REQUIRED_TOPICS:
            if tname not in existing:
                ok = False
                missing.append(tname)

        if ok:
            stat.summary(stat.OK, "Required topics exist")
        else:
            stat.summary(stat.ERROR, f"Missing topics: {missing}")

        return stat

    def diag_tf(self, stat: DiagnosticStatusWrapper):
        ok = True
        missing = []

        for parent, child in REQUIRED_TF:
            try:
                self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            except Exception:
                ok = False
                missing.append(f"{parent}->{child}")

        if ok:
            stat.summary(stat.OK, "TF links available")
        else:
            stat.summary(stat.ERROR, f"Missing TF: {missing}")

        return stat

    def tick(self):
        self.updater.update()

def main():
    rclpy.init()
    node = Preflight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
