import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    zed_filter_pkg = get_package_share_directory('zed_nav2_cloud_filter')

    launch_zed = LaunchConfiguration('launch_zed')
    use_static_tf = LaunchConfiguration('use_static_tf')
    zed_params_override = LaunchConfiguration('zed_params_override')
    cloud_topic = LaunchConfiguration('cloud_topic')
    camera_forward_offset = LaunchConfiguration('camera_forward_offset')
    min_forward_distance = LaunchConfiguration('min_forward_distance')
    stop_distance = LaunchConfiguration('stop_distance')
    box_width = LaunchConfiguration('box_width')
    cone_half_width = LaunchConfiguration('cone_half_width')
    min_z = LaunchConfiguration('min_z')
    max_z = LaunchConfiguration('max_z')
    min_points = LaunchConfiguration('min_points')
    clear_time_sec = LaunchConfiguration('clear_time_sec')
    fail_safe_on_cloud_timeout = LaunchConfiguration('fail_safe_on_cloud_timeout')
    filter_min_range = LaunchConfiguration('filter_min_range')
    filter_max_range = LaunchConfiguration('filter_max_range')
    filter_min_z = LaunchConfiguration('filter_min_z')
    filter_max_z = LaunchConfiguration('filter_max_z')

    zed_filter_launch = os.path.join(
        zed_filter_pkg, 'launch', 'zedm_nav2_filtered.launch.py')
    default_depth_params = os.path.join(
        zed_filter_pkg, 'config', 'zedm_orin_nano_depth.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_zed',
            default_value='true',
            description='Launch the ZED camera and filtered point-cloud stack. Set false if already running.',
        ),
        DeclareLaunchArgument(
            'use_static_tf',
            default_value='true',
            description='Publish base_link to ZED static TF when running standalone.',
        ),
        DeclareLaunchArgument('zed_params_override', default_value=default_depth_params),
        DeclareLaunchArgument('cloud_topic', default_value='/zed/filtered_cloud'),
        DeclareLaunchArgument(
            'camera_forward_offset',
            default_value='0.71245',
            description='ZED forward position from base_link; used to convert camera distances (m).',
        ),
        DeclareLaunchArgument(
            'min_forward_distance',
            default_value='0.25',
            description='Nearest forward distance from the ZED camera included in the box (m).',
        ),
        DeclareLaunchArgument(
            'stop_distance',
            default_value='2.0',
            description='Obstacle stop boundary measured forward from the ZED camera (m).',
        ),
        DeclareLaunchArgument(
            'box_width',
            default_value='0.8',
            description='Constant total width of the straight obstacle box (m).',
        ),
        DeclareLaunchArgument(
            'cone_half_width',
            default_value='-1.0',
            description='Deprecated compatibility argument; non-negative values override box_width.',
        ),
        DeclareLaunchArgument('min_z', default_value='0.12'),
        DeclareLaunchArgument('max_z', default_value='1.50'),
        DeclareLaunchArgument('min_points', default_value='1000'),
        DeclareLaunchArgument('clear_time_sec', default_value='3.0'),
        DeclareLaunchArgument(
            'fail_safe_on_cloud_timeout',
            default_value='false',
            description='When false, a missing/stale cloud does not trigger the safety E-stop.',
        ),
        DeclareLaunchArgument(
            'filter_min_range',
            default_value='0.25',
            description='Upstream cloud filter minimum XY radial range (m).',
        ),
        DeclareLaunchArgument(
            'filter_max_range',
            default_value='6.0',
            description='Upstream cloud filter maximum XY radial range (m).',
        ),
        DeclareLaunchArgument('filter_min_z', default_value='0.05'),
        DeclareLaunchArgument('filter_max_z', default_value='1.50'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_filter_launch),
            launch_arguments={
                'use_static_tf': use_static_tf,
                'target_frame': 'base_link',
                'zed_params_override': zed_params_override,
                'output_cloud_topic': cloud_topic,
                'filter_min_range': filter_min_range,
                'filter_max_range': filter_max_range,
                'filter_min_z': filter_min_z,
                'filter_max_z': filter_max_z,
            }.items(),
            condition=IfCondition(launch_zed),
        ),

        Node(
            package='beach_robot_coverage_nav2',
            executable='zed_obstacle_stop',
            name='zed_obstacle_stop',
            output='screen',
            parameters=[{
                'cloud_topic': cloud_topic,
                'camera_forward_offset': camera_forward_offset,
                'min_forward_distance': min_forward_distance,
                'stop_distance': stop_distance,
                'box_width': box_width,
                'cone_half_width': cone_half_width,
                'min_z': min_z,
                'max_z': max_z,
                'min_points': min_points,
                'clear_time_sec': clear_time_sec,
                'fail_safe_on_cloud_timeout': fail_safe_on_cloud_timeout,
            }],
        ),
    ])
