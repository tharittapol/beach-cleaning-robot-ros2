from launch import LaunchDescription
from launch_ros.actions import Node


def _static_tf(name, xyz, rpy, child_frame):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="screen",
        arguments=[
            str(xyz[0]), str(xyz[1]), str(xyz[2]),
            str(rpy[0]), str(rpy[1]), str(rpy[2]),
            "base_link", child_frame,
        ],
    )


def generate_launch_description():
    zed_xyz = (0.71245, 0.0, 0.394)
    zed_rpy = (0.0, 0.0, 0.0)

    return LaunchDescription([
        _static_tf("tf_base_to_zed", zed_xyz, zed_rpy, "zed_camera_link"),
        _static_tf("tf_base_to_zed_imu", zed_xyz, zed_rpy, "zed_imu_link"),
    ])
