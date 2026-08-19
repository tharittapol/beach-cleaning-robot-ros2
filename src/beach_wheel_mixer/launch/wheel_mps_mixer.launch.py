from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = "beach_wheel_mixer"   # <-- change to your ament_cmake package name

    default_params = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        "config",
        "mixer.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to mixer.yaml"
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time"
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/cmd_vel",
            description="Twist input topic"
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/wheel_cmd",
            description="Wheel speeds output topic (Float32MultiArray, m/s)"
        ),

        Node(
            package=pkg_name,
            executable="wheel_mps_mixer", 
            name="wheel_mps_mixer",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {
                    "input_topic": LaunchConfiguration("input_topic"),
                    "output_topic": LaunchConfiguration("output_topic"),
                },
            ],
        ),
    ])
