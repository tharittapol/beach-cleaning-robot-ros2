from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def include_launch(pkg, rel_path, launch_args=None, condition=None):
    launch_file = os.path.join(FindPackageShare(pkg).find(pkg), rel_path)
    action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments=(launch_args or {}).items()
    )
    if condition is not None:
        action.condition = condition
    return action


def generate_launch_description():
    robot_yaml = LaunchConfiguration('robot_yaml')
    use_gnss = LaunchConfiguration('use_gnss')
    use_zed = LaunchConfiguration('use_zed')
    use_coverage = LaunchConfiguration('use_coverage')

    esp32_port = LaunchConfiguration('esp32_port')
    esp32_baud = LaunchConfiguration('esp32_baud')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'robot_yaml',
        default_value=os.path.join(FindPackageShare('beach_robot_bringup').find('beach_robot_bringup'), 'config', 'robot.yaml')
    ))
    ld.add_action(DeclareLaunchArgument('use_gnss', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_zed', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_coverage', default_value='true'))

    ld.add_action(DeclareLaunchArgument('esp32_port', default_value='/dev/esp32_beach'))
    ld.add_action(DeclareLaunchArgument('esp32_baud', default_value='115200'))

    # 1) ESP32 bridge
    ld.add_action(include_launch(
        'beach_robot_esp32_bridge',
        'launch/esp32_bridge.launch.py',
        launch_args={
            'port': esp32_port,
            'baudrate': esp32_baud,
            'timeout': '0.05',
        }
    ))

    # 2) wheel mixer
    mixer_params = os.path.join(
        FindPackageShare('beach_wheel_mixer').find('beach_wheel_mixer'),
        'config',
        'mixer.yaml'
    )

    ld.add_action(include_launch(
        'beach_wheel_mixer',
        'launch/wheel_mps_mixer.launch.py',
        launch_args={
            'params_file': mixer_params,
            'use_sim_time': 'false',
        }
    ))

    # 2.5) Static sensor TFs (IMU/GPS/ZED/Ultrasonic frames)
    ld.add_action(include_launch(
        'beach_robot_localization',
        'launch/static_sensors_tf.launch.py'
    ))

    # 3) localization (EKF + odom)
    ld.add_action(include_launch(
        'beach_robot_localization',
        'launch/localization.launch.py'
    ))

    # 4) GNSS (optional)
    ld.add_action(include_launch(
        'beach_robot_gnss',
        'launch/um982_fix_nema.launch.py',
        condition=IfCondition(use_gnss)
    ))

    # 5) ZED filter (optional)
    ld.add_action(include_launch(
        'zed_nav2_cloud_filter',
        'launch/zedm_nav2_filtered.launch.py',
        launch_args={
            'use_static_tf': 'false',
        },
        condition=IfCondition(use_zed)
    ))

    # 6) Nav2 + Coverage bringup (when use_coverage=true)
    ld.add_action(include_launch(
        'beach_robot_coverage_nav2',
        'launch/beach_cleaning_bringup.launch.py',
        condition=IfCondition(use_coverage)
    ))

    # 7) Preflight check helper (optional: run manually)
    ld.add_action(Node(
        package='beach_robot_bringup',
        executable='preflight_check',
        name='preflight_check',
        output='screen',
        parameters=[robot_yaml],
        # comment out if you don't want it auto-run:
        # condition=IfCondition('false')
    ))

    return ld
