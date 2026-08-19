from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyESP32",
            description="Serial port for ESP32 bridge"
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="230400",
            description="Baud rate"
        ),
        DeclareLaunchArgument(
            "timeout",
            default_value="0.05",
            description="Serial read timeout (seconds)"
        ),
        DeclareLaunchArgument(
            "enc_vel_max_abs_mps",
            default_value="3.0",
            description="Drop encoder velocity samples with any wheel above this absolute speed. <=0 disables."
        ),
        DeclareLaunchArgument(
            "enc_vel_max_step_mps",
            default_value="1.0",
            description="Drop encoder velocity samples with any wheel jump above this speed. <=0 disables."
        ),
        DeclareLaunchArgument(
            "wheel_cmd_send_rate_hz",
            default_value="30.0",
            description="Rate-limit wheel_cmd serial writes to keep ESP32 telemetry/debug responsive."
        ),
        DeclareLaunchArgument(
            "wheel_cmd_stale_timeout_sec",
            default_value="0.5",
            description="Send zero wheel_cmd when no fresh /wheel_cmd has arrived within this time."
        ),
        DeclareLaunchArgument(
            "safety_estop_topic",
            default_value="/safety/e_stop",
            description="Independent safety E-stop topic ORed with the manual /e_stop topic."
        ),
        DeclareLaunchArgument(
            "publish_raw_json",
            default_value="false",
            description="Publish every raw ESP32 JSON line. Keep false while driving to avoid debug backpressure."
        ),
        DeclareLaunchArgument(
            "esp32_debug_enabled",
            default_value="false",
            description="Enable continuous low-level ESP32 debug telemetry. Keep false for normal driving."
        ),

        Node(
            package="beach_robot_esp32_bridge",
            executable="esp32_bridge",
            name="esp32_bridge",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "baudrate": LaunchConfiguration("baudrate"),
                "timeout": LaunchConfiguration("timeout"),
                "enc_vel_max_abs_mps": LaunchConfiguration("enc_vel_max_abs_mps"),
                "enc_vel_max_step_mps": LaunchConfiguration("enc_vel_max_step_mps"),
                "wheel_cmd_send_rate_hz": LaunchConfiguration("wheel_cmd_send_rate_hz"),
                "wheel_cmd_stale_timeout_sec": LaunchConfiguration("wheel_cmd_stale_timeout_sec"),
                "safety_estop_topic": LaunchConfiguration("safety_estop_topic"),
                "esp32_debug_enabled": LaunchConfiguration("esp32_debug_enabled"),
                "publish_raw_json": LaunchConfiguration("publish_raw_json"),
            }],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
