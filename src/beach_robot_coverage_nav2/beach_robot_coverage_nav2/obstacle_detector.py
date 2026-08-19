#!/usr/bin/env python3
"""Front-box obstacle detection from the ZED filtered point cloud.

`/zed/filtered_cloud` is already range- and height-filtered (0.25-6.0 m, z 0.05-1.5 m)
and decimated to ~10 Hz by `zed_nav2_cloud_filter`, and is published in the `base_link`
frame (x forward, y left, z up). This module looks only at a forward box in front of the
robot and reports whether something is close enough to stop for.

Two pieces:
  * ``evaluate_front_box(cloud, ...)`` — pure function doing the numpy math.
  * ``FrontBoxMonitor`` — wires a subscription on a given node and calls back on every
    cloud with ``(present, nearest_x)``; the coverage node uses this to drive its
    pause/resume state machine.

Run standalone for bring-up testing::

    ros2 run beach_robot_coverage_nav2 obstacle_detector
"""
from typing import Callable, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray


DEBUG_CLOUD_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]


def extract_front_box_points(
    cloud: PointCloud2,
    *,
    min_forward_distance: float,
    stop_distance: float,
    box_width: float,
    min_z: float,
    max_z: float,
) -> np.ndarray:
    """Return XYZ points inside the straight rectangular front box.

    A point counts as an obstacle if it is ahead of the robot
    (``min_forward_distance <= x <= stop_distance``), inside the lateral half-width
    (``|y| <= box_width / 2``) and within the height band (``min_z <= z <= max_z``).
    The Y limit is constant at every X distance, so this is a straight rectangular box,
    not an angle-based cone.
    ``min_z`` sits above the cloud filter's 0.05 m floor to keep ground points from
    triggering a stop when the robot pitches on soft sand.
    """
    try:
        arr = point_cloud2.read_points_numpy(
            cloud, field_names=('x', 'y', 'z'), skip_nans=True)
    except AttributeError:  # older sensor_msgs_py without read_points_numpy
        # read_points may yield tuples or a structured array; row-index works for both.
        arr = np.array(
            [(p[0], p[1], p[2]) for p in point_cloud2.read_points(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True)],
            dtype=np.float32,
        )

    if arr.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    arr = np.asarray(arr, dtype=np.float32).reshape(-1, 3)
    xs, ys, zs = arr[:, 0], arr[:, 1], arr[:, 2]
    box_half_width = max(0.0, float(box_width)) / 2.0

    mask = (
        (xs >= min_forward_distance)
        & (xs <= stop_distance)
        & (np.abs(ys) <= box_half_width)
        & (zs >= min_z)
        & (zs <= max_z)
    )
    return arr[mask]


def evaluate_front_box(
    cloud: PointCloud2,
    *,
    min_forward_distance: float,
    stop_distance: float,
    box_width: float,
    min_z: float,
    max_z: float,
    min_points: int,
) -> Tuple[bool, Optional[float]]:
    """Return ``(obstacle_present, nearest_x)`` for the forward box."""
    points = extract_front_box_points(
        cloud,
        min_forward_distance=min_forward_distance,
        stop_distance=stop_distance,
        box_width=box_width,
        min_z=min_z,
        max_z=max_z,
    )
    count = len(points)
    if count >= min_points:
        return True, float(np.min(points[:, 0]))
    return False, None


class FrontBoxMonitor:
    """Subscribes to the filtered cloud and reports front-box obstacles to a callback."""

    def __init__(
        self,
        node: Node,
        on_update: Callable[[bool, Optional[float]], None],
        *,
        cloud_topic: str = '/zed/filtered_cloud',
        min_forward_distance: float = 0.25,
        stop_distance: float = 2.0,
        box_width: float = 0.8,
        min_z: float = 0.12,
        max_z: float = 1.5,
        min_points: int = 1000,
        expected_frame: str = 'base_link',
        debug_cloud_topic: str = '/safety/obstacle_points',
        debug_marker_topic: str = '/safety/obstacle_markers',
        publish_debug_cloud: bool = True,
    ):
        self._node = node
        self._on_update = on_update
        self.min_forward_distance = max(0.0, float(min_forward_distance))
        self.stop_distance = max(self.min_forward_distance, float(stop_distance))
        self.box_width = max(0.0, float(box_width))
        self.min_z = float(min_z)
        self.max_z = max(self.min_z, float(max_z))
        self.min_points = max(1, int(min_points))
        self.expected_frame = expected_frame.lstrip('/')
        self._bad_frame_warned: Optional[str] = None

        self.present: bool = False
        self.nearest_x: Optional[float] = None
        self.point_count: int = 0
        self.point_min: Optional[np.ndarray] = None
        self.point_max: Optional[np.ndarray] = None
        self._debug_pub = (
            node.create_publisher(PointCloud2, debug_cloud_topic, qos_profile_sensor_data)
            if publish_debug_cloud else None
        )
        self._marker_pub = (
            node.create_publisher(MarkerArray, debug_marker_topic, 10)
            if publish_debug_cloud else None
        )

        self._sub = node.create_subscription(
            PointCloud2, cloud_topic, self._cloud_cb, qos_profile_sensor_data)

    def _cloud_cb(self, msg: PointCloud2):
        frame_id = msg.header.frame_id.lstrip('/')
        if frame_id != self.expected_frame:
            if frame_id != self._bad_frame_warned:
                self._node.get_logger().warn(
                    f'Ignoring obstacle cloud in frame {msg.header.frame_id!r}; '
                    f'expected {self.expected_frame!r}. Transform it before publishing.')
                self._bad_frame_warned = frame_id
            return
        self._bad_frame_warned = None

        try:
            points = extract_front_box_points(
                msg,
                min_forward_distance=self.min_forward_distance,
                stop_distance=self.stop_distance,
                box_width=self.box_width,
                min_z=self.min_z,
                max_z=self.max_z,
            )
        except Exception as exc:  # never let a malformed cloud kill the node
            self._node.get_logger().warn(f'Obstacle scan failed: {exc}')
            return

        self.point_count = len(points)
        self.point_min = np.min(points, axis=0) if self.point_count else None
        self.point_max = np.max(points, axis=0) if self.point_count else None
        present = self.point_count >= self.min_points
        nearest_x = float(np.min(points[:, 0])) if present else None
        if self._debug_pub is not None:
            # A bright intensity field makes the selected points visible with RViz's
            # default Intensity color transformer, instead of requiring FlatColor.
            debug_points = [
                (float(x), float(y), float(z), 4096.0) for x, y, z in points
            ]
            debug_cloud = point_cloud2.create_cloud(
                msg.header, DEBUG_CLOUD_FIELDS, debug_points)
            self._debug_pub.publish(debug_cloud)
        if self._marker_pub is not None:
            self._publish_debug_markers(msg.header.frame_id, points)

        self.present = present
        self.nearest_x = nearest_x
        self._on_update(present, nearest_x)

    def _publish_debug_markers(self, frame_id: str, points: np.ndarray):
        stamp = self._node.get_clock().now().to_msg()

        selected = Marker()
        selected.header.frame_id = frame_id
        selected.header.stamp = stamp
        selected.ns = 'obstacle_points'
        selected.id = 0
        selected.type = Marker.POINTS
        selected.action = Marker.ADD
        selected.pose.orientation.w = 1.0
        selected.scale.x = 0.06
        selected.scale.y = 0.06
        selected.color.r = 1.0
        selected.color.g = 0.05
        selected.color.b = 0.05
        selected.color.a = 1.0
        selected.points = [
            Point(x=float(x), y=float(y), z=float(z)) for x, y, z in points
        ]

        box = Marker()
        box.header.frame_id = frame_id
        box.header.stamp = stamp
        box.ns = 'obstacle_box'
        box.id = 1
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x = (self.min_forward_distance + self.stop_distance) / 2.0
        box.pose.position.z = (self.min_z + self.max_z) / 2.0
        box.pose.orientation.w = 1.0
        box.scale.x = self.stop_distance - self.min_forward_distance
        box.scale.y = self.box_width
        box.scale.z = self.max_z - self.min_z
        box.color.r = 0.1
        box.color.g = 0.8
        box.color.b = 0.2
        box.color.a = 0.15

        self._marker_pub.publish(MarkerArray(markers=[box, selected]))


def main(args=None):
    """Standalone diagnostic: log front-box obstacle state from the live cloud."""
    rclpy.init(args=args)
    node = Node('obstacle_detector')
    node.declare_parameter('cloud_topic', '/zed/filtered_cloud')
    node.declare_parameter('min_forward_distance', 0.25)
    node.declare_parameter('stop_distance', 2.0)
    node.declare_parameter('box_width', 0.8)
    node.declare_parameter('min_z', 0.12)
    node.declare_parameter('max_z', 1.5)
    node.declare_parameter('min_points', 1000)

    def _log(present: bool, nearest_x: Optional[float]):
        if present:
            node.get_logger().info(f'OBSTACLE  nearest_x={nearest_x:.2f} m')
        else:
            node.get_logger().info('clear')

    FrontBoxMonitor(
        node, _log,
        cloud_topic=str(node.get_parameter('cloud_topic').value),
        min_forward_distance=float(node.get_parameter('min_forward_distance').value),
        stop_distance=float(node.get_parameter('stop_distance').value),
        box_width=float(node.get_parameter('box_width').value),
        min_z=float(node.get_parameter('min_z').value),
        max_z=float(node.get_parameter('max_z').value),
        min_points=int(node.get_parameter('min_points').value),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
