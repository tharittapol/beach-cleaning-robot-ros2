import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('beach_robot_localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_static_tf = LaunchConfiguration('use_static_tf')
    enc_vel_topic = LaunchConfiguration('enc_vel_topic')
    wheel_odom_topic = LaunchConfiguration('wheel_odom_topic')
    bno_imu_topic = LaunchConfiguration('bno_imu_topic')
    zed_imu_topic = LaunchConfiguration('zed_imu_topic')
    front_track_width = LaunchConfiguration('front_track_width')
    rear_track_width = LaunchConfiguration('rear_track_width')
    wheel_scale_fl = LaunchConfiguration('wheel_scale_fl')
    wheel_scale_fr = LaunchConfiguration('wheel_scale_fr')
    wheel_scale_rl = LaunchConfiguration('wheel_scale_rl')
    wheel_scale_rr = LaunchConfiguration('wheel_scale_rr')
    linear_scale = LaunchConfiguration('linear_scale')
    angular_scale = LaunchConfiguration('angular_scale')
    publish_fusion_bno_tf = LaunchConfiguration('publish_fusion_bno_tf')

    static_tf_launch = os.path.join(pkg, 'launch', 'static_sensors_tf.launch.py')
    ekf_compare = os.path.join(pkg, 'config', 'ekf_imu_compare.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_static_tf', default_value='true'),
        DeclareLaunchArgument('enc_vel_topic', default_value='/enc_vel'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/wheel/odom'),
        DeclareLaunchArgument('bno_imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('zed_imu_topic', default_value='/zed/zed_node/imu/data'),
        DeclareLaunchArgument('front_track_width', default_value='0.734'),
        DeclareLaunchArgument('rear_track_width', default_value='1.179'),
        DeclareLaunchArgument('wheel_scale_fl', default_value='1.0'),
        DeclareLaunchArgument('wheel_scale_fr', default_value='1.0'),
        DeclareLaunchArgument('wheel_scale_rl', default_value='1.0'),
        DeclareLaunchArgument('wheel_scale_rr', default_value='1.0'),
        DeclareLaunchArgument('linear_scale', default_value='1.0'),
        DeclareLaunchArgument('angular_scale', default_value='1.0'),
        DeclareLaunchArgument(
            'publish_fusion_bno_tf',
            default_value='false',
            description='Publish odom->base_link from fusion_bno only. Keep false for pure comparison bags.',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(static_tf_launch),
            condition=IfCondition(use_static_tf),
        ),

        Node(
            package='beach_robot_localization',
            executable='wheel_odometry',
            name='wheel_odometry_compare',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'enc_vel_topic': enc_vel_topic,
                'odom_topic': wheel_odom_topic,
                'front_track_width': front_track_width,
                'rear_track_width': rear_track_width,
                'wheel_scale_fl': wheel_scale_fl,
                'wheel_scale_fr': wheel_scale_fr,
                'wheel_scale_rl': wheel_scale_rl,
                'wheel_scale_rr': wheel_scale_rr,
                'linear_scale': linear_scale,
                'angular_scale': angular_scale,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
            }],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_wheel_only_compare',
            output='screen',
            parameters=[ekf_compare, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/wheel_only'),
                ('wheel/odom', wheel_odom_topic),
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_bno_imu_only',
            output='screen',
            parameters=[ekf_compare, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/bno_imu_only'),
                ('imu/data', bno_imu_topic),
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_zed_imu_only',
            output='screen',
            parameters=[ekf_compare, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/zed_imu_only'),
                ('imu/data', zed_imu_topic),
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_wheel_bno',
            output='screen',
            parameters=[ekf_compare, {
                'use_sim_time': use_sim_time,
                'publish_tf': publish_fusion_bno_tf,
            }],
            remappings=[
                ('odometry/filtered', '/odometry/fusion_bno'),
                ('wheel/odom', wheel_odom_topic),
                ('imu/data', bno_imu_topic),
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_wheel_zed',
            output='screen',
            parameters=[ekf_compare, {'use_sim_time': use_sim_time}],
            remappings=[
                ('odometry/filtered', '/odometry/fusion_zed'),
                ('wheel/odom', wheel_odom_topic),
                ('imu/data', zed_imu_topic),
            ],
        ),
    ])
