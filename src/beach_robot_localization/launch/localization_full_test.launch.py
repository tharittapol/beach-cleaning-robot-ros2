import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _include(pkg_name, relative_path, launch_args=None, condition=None):
    launch_file = os.path.join(
        get_package_share_directory(pkg_name),
        relative_path,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=(launch_args or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    mixer_pkg = get_package_share_directory('beach_wheel_mixer')
    localization_pkg = get_package_share_directory('beach_robot_localization')
    navsat_params = os.path.join(localization_pkg, 'config', 'navsat.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_esp32 = LaunchConfiguration('use_esp32')
    use_teleop = LaunchConfiguration('use_teleop')
    use_mixer = LaunchConfiguration('use_mixer')
    use_zed = LaunchConfiguration('use_zed')
    zed_params_override = LaunchConfiguration('zed_params_override')
    use_gnss = LaunchConfiguration('use_gnss')
    launch_gnss_driver = LaunchConfiguration('launch_gnss_driver')
    gnss_gga_send_period = LaunchConfiguration('gnss_gga_send_period')
    gnss_force_gpgga = LaunchConfiguration('gnss_force_gpgga')

    esp32_port = LaunchConfiguration('esp32_port')
    esp32_baudrate = LaunchConfiguration('esp32_baudrate')
    esp32_timeout = LaunchConfiguration('esp32_timeout')
    enc_vel_max_abs_mps = LaunchConfiguration('enc_vel_max_abs_mps')
    enc_vel_max_step_mps = LaunchConfiguration('enc_vel_max_step_mps')
    wheel_cmd_send_rate_hz = LaunchConfiguration('wheel_cmd_send_rate_hz')
    wheel_cmd_stale_timeout_sec = LaunchConfiguration('wheel_cmd_stale_timeout_sec')
    esp32_debug_enabled = LaunchConfiguration('esp32_debug_enabled')
    publish_raw_json = LaunchConfiguration('publish_raw_json')

    joy_device = LaunchConfiguration('joy_device')
    max_linear = LaunchConfiguration('max_linear')
    max_angular = LaunchConfiguration('max_angular')
    axis_linear = LaunchConfiguration('axis_linear')
    axis_angular = LaunchConfiguration('axis_angular')

    mixer_params_file = LaunchConfiguration('mixer_params_file')
    linear_scale = LaunchConfiguration('linear_scale')
    angular_scale = LaunchConfiguration('angular_scale')
    wheel_scale_fl = LaunchConfiguration('wheel_scale_fl')
    wheel_scale_fr = LaunchConfiguration('wheel_scale_fr')
    wheel_scale_rl = LaunchConfiguration('wheel_scale_rl')
    wheel_scale_rr = LaunchConfiguration('wheel_scale_rr')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_esp32', default_value='true'),
        DeclareLaunchArgument('use_teleop', default_value='true'),
        DeclareLaunchArgument('use_mixer', default_value='true'),
        DeclareLaunchArgument('use_zed', default_value='true'),
        DeclareLaunchArgument(
            'zed_params_override',
            default_value=os.path.join(
                get_package_share_directory('zed_nav2_cloud_filter'),
                'config',
                'zedm_orin_nano_imu_only.yaml',
            ),
            description='ZED wrapper profile. Use a depth profile when obstacle detection is needed.',
        ),
        DeclareLaunchArgument(
            'use_gnss',
            default_value='true',
            description='Run navsat_transform → /odometry/gps and (if launch_gnss_driver) the UM982 driver.',
        ),
        DeclareLaunchArgument(
            'launch_gnss_driver',
            default_value='true',
            description='Launch the UM982 bridge here. Set false to keep GNSS as a SEPARATE persistent '
                        'node (run um982_fix_nema.launch.py once and leave it converging) so restarting '
                        'this stack does not reset the RTK fix; navsat_transform still runs under use_gnss.',
        ),
        DeclareLaunchArgument(
            'gnss_gga_send_period',
            default_value='5.0',
            description='NTRIP upstream GGA period in seconds. RTK2GO is often more stable around 5s than sub-second rates.',
        ),
        DeclareLaunchArgument(
            'gnss_force_gpgga',
            default_value='false',
            description='Convert GNGGA to GPGGA before sending to the NTRIP caster.',
        ),

        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyESP32'),
        DeclareLaunchArgument('esp32_baudrate', default_value='230400'),
        DeclareLaunchArgument('esp32_timeout', default_value='0.05'),
        DeclareLaunchArgument('enc_vel_max_abs_mps', default_value='3.0'),
        DeclareLaunchArgument('enc_vel_max_step_mps', default_value='1.0'),
        DeclareLaunchArgument(
            'wheel_cmd_send_rate_hz',
            default_value='20.0',
            description='Rate-limit wheel_cmd serial writes from the ESP32 bridge.',
        ),
        DeclareLaunchArgument(
            'wheel_cmd_stale_timeout_sec',
            default_value='0.5',
            description='Send zero wheel_cmd if the mixer stops publishing fresh wheel commands.',
        ),
        DeclareLaunchArgument(
            'esp32_debug_enabled',
            default_value='false',
            description='Enable continuous low-level ESP32 debug telemetry. Keep false while driving.',
        ),
        DeclareLaunchArgument(
            'publish_raw_json',
            default_value='false',
            description='Publish every raw ESP32 JSON line. Keep false while driving.',
        ),

        DeclareLaunchArgument('joy_device', default_value='/dev/input/js_joy'),
        DeclareLaunchArgument('max_linear', default_value='0.35'),   # sand: match mixer max_v
        DeclareLaunchArgument('max_angular', default_value='0.35'),  # sand: match mixer max_w
        DeclareLaunchArgument('axis_linear', default_value='1'),
        DeclareLaunchArgument('axis_angular', default_value='0'),

        DeclareLaunchArgument(
            'mixer_params_file',
            default_value=os.path.join(mixer_pkg, 'config', 'mixer.yaml'),
        ),
        DeclareLaunchArgument(
            'linear_scale',
            default_value='1.625',
            description='Wheel odom linear scale. Tile 2026-06-19: 4.87 m tape @ odom 5.0 (converged from sand 1.19, which under-reported ~40% on tile).',
        ),
        DeclareLaunchArgument(
            'angular_scale',
            default_value='1.0',
            description='Wheel odometry yaw scale. Start neutral for no-spin arc tests.',
        ),
        DeclareLaunchArgument(
            'wheel_scale_fl',
            default_value='1.0',
            description='Per-wheel encoder scale applied before wheel odometry integration.',
        ),
        DeclareLaunchArgument(
            'wheel_scale_fr',
            default_value='1.0',
            description='Per-wheel encoder scale applied before wheel odometry integration.',
        ),
        DeclareLaunchArgument(
            'wheel_scale_rl',
            default_value='0.64',
            description='Rear-left encoder scale from chain-fixed straight response tests.',
        ),
        DeclareLaunchArgument(
            'wheel_scale_rr',
            default_value='0.64',
            description='Rear-right encoder scale from chain-fixed straight response tests.',
        ),

        _include(
            'beach_robot_esp32_bridge',
            'launch/esp32_bridge.launch.py',
            launch_args={
                'port': esp32_port,
                'baudrate': esp32_baudrate,
                'timeout': esp32_timeout,
                'enc_vel_max_abs_mps': enc_vel_max_abs_mps,
                'enc_vel_max_step_mps': enc_vel_max_step_mps,
                'wheel_cmd_send_rate_hz': wheel_cmd_send_rate_hz,
                'wheel_cmd_stale_timeout_sec': wheel_cmd_stale_timeout_sec,
                'esp32_debug_enabled': esp32_debug_enabled,
                'publish_raw_json': publish_raw_json,
            },
            condition=IfCondition(use_esp32),
        ),

        _include(
            'beach_robot_teleop',
            'launch/teleop.launch.py',
            launch_args={
                'joy_device': joy_device,
                'max_linear': max_linear,
                'max_angular': max_angular,
                'axis_linear': axis_linear,
                'axis_angular': axis_angular,
            },
            condition=IfCondition(use_teleop),
        ),

        _include(
            'beach_wheel_mixer',
            'launch/wheel_mps_mixer.launch.py',
            launch_args={
                'params_file': mixer_params_file,
                'use_sim_time': use_sim_time,
                'input_topic': '/cmd_vel',
                'output_topic': '/wheel_cmd',
            },
            condition=IfCondition(use_mixer),
        ),

        _include(
            'zed_nav2_cloud_filter',
            'launch/zedm_nav2_filtered.launch.py',
            launch_args={
                # localization_imu_compare publishes the static sensor TFs.
                'use_static_tf': 'false',
                'target_frame': 'base_link',
                'zed_params_override': zed_params_override,
            },
            condition=IfCondition(use_zed),
        ),

        # UM982 driver — only when launch_gnss_driver is true. Keep it false to run the GNSS as a
        # separate persistent node (um982_fix_nema.launch.py) so the RTK fix stays converged.
        _include(
            'beach_robot_gnss',
            'launch/um982_fix_nema.launch.py',
            launch_args={
                'gga_send_period': gnss_gga_send_period,
                'force_gpgga': gnss_force_gpgga,
            },
            condition=IfCondition(PythonExpression(
                ["'", use_gnss, "' == 'true' and '", launch_gnss_driver, "' == 'true'"])),
        ),

        _include(
            'beach_robot_localization',
            'launch/localization_imu_compare.launch.py',
            launch_args={
                'use_sim_time': use_sim_time,
                'use_static_tf': 'true',
                'enc_vel_topic': '/enc_vel',
                'wheel_odom_topic': '/wheel/odom',
                'bno_imu_topic': '/imu/data',
                'zed_imu_topic': '/zed/zed_node/imu/data',
                'linear_scale': linear_scale,
                'angular_scale': angular_scale,
                'wheel_scale_fl': wheel_scale_fl,
                'wheel_scale_fr': wheel_scale_fr,
                'wheel_scale_rl': wheel_scale_rl,
                'wheel_scale_rr': wheel_scale_rr,
                'publish_fusion_bno_tf': 'true',
            },
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_report',
            output='screen',
            parameters=[navsat_params, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odometry/fusion_bno'),
                ('odometry/gps', '/odometry/gps'),
            ],
            condition=IfCondition(use_gnss),
        ),
    ])
