from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('beach_robot_localization')
    ekf_local = os.path.join(pkg, 'config', 'ekf_local.yaml')
    navsat = os.path.join(pkg, 'config', 'navsat.yaml')
    ekf_global = os.path.join(pkg, 'config', 'ekf_global_yaw_from_GNSS.yaml')

    return LaunchDescription([
        Node(
            package='beach_robot_localization',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='screen',
            parameters=[{
                'enc_vel_topic': 'enc_vel',
                'odom_topic': 'wheel/odom',
                'front_track_width': 0.75,
                'rear_track_width': 1.17,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
            }]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf_local],
            remappings=[
                ('odometry/filtered', 'odometry/local'),
                ('wheel/odom', 'wheel/odom'),
                ('imu/data', 'imu/data'),
            ],
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat],
            remappings=[
                ('imu', 'gnss/imu_heading'),
                ('gps/fix', 'gps/fix'),
                ('odometry/filtered', 'odometry/local'),
                ('odometry/gps', 'odometry/gps'),
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf_global],
            remappings=[
                ('odometry/filtered', 'odometry/global'),
                ('odometry/gps', 'odometry/gps'),
                ('imu/data', 'imu/data'),
                ('gnss/imu_heading', 'gnss/imu_heading'),
            ],
        ),
    ])
