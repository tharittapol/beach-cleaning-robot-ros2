import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # ---- Set these roughly first, then measure and tune ----
    # base_link -> imu_link
    imu_xyz = (0.31685, 0.00, 0.646)   # meters
    imu_rpy = (0.0, 0.0, 0.0)      # radians

    # base_link -> gps_link (antenna position)
    gps_xyz = (0.30, 0.00, 0.50)   # meters
    gps_rpy = (0.0, 0.0, 0.0)

    # base_link -> gnss_heading_link
    # If UM982 heading is “robot yaw”, usually place at the main antenna.
    head_xyz = gps_xyz
    head_rpy = (0.0, 0.0, 0.0)

    # base_link -> ultrasonic links (front sensors)
    ultra_mid_xyz = (0.25, 0.00, 0.12)
    ultra_mid_rpy = (0.0, 0.0, 0.0)

    ultra_left_xyz = (0.25, 0.15, 0.12)
    ultra_left_rpy = (0.0, 0.0, 0.0)

    ultra_right_xyz = (0.25, -0.15, 0.12)
    ultra_right_rpy = (0.0, 0.0, 0.0)

    def static_tf(name, xyz, rpy, parent, child):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            output='screen',
            arguments=[
                str(xyz[0]), str(xyz[1]), str(xyz[2]),
                str(rpy[0]), str(rpy[1]), str(rpy[2]),
                parent, child
            ]
        )

    zed_static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('zed_nav2_cloud_filter'),
            'launch',
            'zedm_static_tf.launch.py',
        )),
    )

    return LaunchDescription([
        static_tf('tf_base_to_imu', imu_xyz, imu_rpy, 'base_link', 'imu_link'),
        static_tf('tf_base_to_gps', gps_xyz, gps_rpy, 'base_link', 'gps_link'),
        static_tf('tf_base_to_gnss_heading', head_xyz, head_rpy, 'base_link', 'gnss_heading_link'),
        zed_static_tf,
        static_tf('tf_base_to_ultra_left', ultra_left_xyz, ultra_left_rpy, 'base_link', 'ultra_left_link'),
        static_tf('tf_base_to_ultra_middle', ultra_mid_xyz, ultra_mid_rpy, 'base_link', 'ultra_middle_link'),
        static_tf('tf_base_to_ultra_right', ultra_right_xyz, ultra_right_rpy, 'base_link', 'ultra_right_link'),
    ])
