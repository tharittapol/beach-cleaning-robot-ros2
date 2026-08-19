from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('beach_robot_localization')
    ekf_local = os.path.join(pkg, 'config', 'ekf_local.yaml')
    navsat = os.path.join(pkg, 'config', 'navsat.yaml')
    ekf_global = os.path.join(pkg, 'config', 'ekf_global.yaml')

    return LaunchDescription([
        Node(
            package='beach_robot_localization',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='screen',
            parameters=[{
                'enc_vel_topic': 'enc_vel',
                'odom_topic': 'wheel/odom',
                'front_track_width': 0.734,
                'rear_track_width': 1.179,
                'wheel_scale_fl': 1.0,
                'wheel_scale_fr': 1.0,
                'wheel_scale_rl': 0.64,
                'wheel_scale_rr': 0.64,
                'linear_scale': 1.19,  # sand 2026-06-10: 10 m odom vs 8.245 m tape (~21% slip)
                'angular_scale': 1.0,
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
                ('imu', 'imu/data'),
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
            ],
        ),
    ])
