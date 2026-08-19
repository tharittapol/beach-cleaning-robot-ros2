from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joy_device = LaunchConfiguration('joy_device')
    max_linear = LaunchConfiguration('max_linear')
    max_angular = LaunchConfiguration('max_angular')
    axis_linear = LaunchConfiguration('axis_linear')
    axis_angular = LaunchConfiguration('axis_angular')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_device',
            default_value='/dev/input/js_joy',
            description='Joystick device path. Use /dev/input/js0 if no stable symlink is configured.',
        ),
        DeclareLaunchArgument(
            'max_linear',
            default_value='0.35',
            description='Maximum manual linear speed in m/s (sand: match mixer max_v).',
        ),
        DeclareLaunchArgument(
            'max_angular',
            default_value='0.35',
            description='Maximum manual angular speed in rad/s (sand: match mixer max_w).',
        ),
        DeclareLaunchArgument(
            'axis_linear',
            default_value='1',
            description='Joystick axis for linear.x. F710/XInput left stick Y is usually 1.',
        ),
        DeclareLaunchArgument(
            'axis_angular',
            default_value='0',
            description='Joystick axis for angular.z. F710/XInput left stick X is usually 0.',
        ),

        # joy node (must install joy package first: sudo apt install ros-<distro>-joy)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'device': joy_device,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        # teleop node
        Node(
            package='beach_robot_teleop',
            executable='teleop_4wd',
            name='teleop_4wd',
            output='screen',
            parameters=[{
                'max_linear': max_linear,
                'max_angular': max_angular,

                # F710/XInput default: left stick Y drives, left stick X rotates.
                'axis_linear': axis_linear,
                'axis_angular': axis_angular,
                'scale_linear': 1.0,
                'scale_angular': 1.0,

                'invert_linear': False,
                'invert_angular': False,     # set True if turning direction feels reversed
                'deadzone_linear': 0.08,
                'deadzone_angular': 0.08,
                'expo_linear': 1.0,
                'expo_angular': 1.0,
                'publish_rate_hz': 20.0,
                'joy_timeout_sec': 0.5,
                'linear_slew_rate': 0.35,
                'angular_slew_rate': 0.60,
                'linear_decel_rate': 3.0,
                'angular_decel_rate': 4.0,
                'spin_snap_enabled': True,
                'spin_snap_angular_min': 0.25,
                'spin_snap_linear_ratio': 0.45,

                'cmd_vel_out': '/cmd_vel',
            }]
        )
    ])
