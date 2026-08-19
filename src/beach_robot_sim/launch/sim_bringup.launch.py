import os
import re

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import xacro


def _xacro_to_robot_description(xacro_file: str, mappings: dict) -> dict:
    """
    Generate robot_description from xacro using Python (not shell Command),
    and clean it so gazebo_ros2_control won't choke on "--param robot_description:=<xml...>" parsing.

    - Remove XML header: <?xml version="1.0" ?>
    - Remove XML comments: <!-- ... -->
    - Keep as one string (ParameterValue str)
    """
    doc = xacro.process_file(xacro_file, mappings=mappings)
    xml = doc.toxml()

    # remove xml header
    xml = re.sub(r"<\?xml[^>]*\?>", "", xml).strip()

    # remove comments (non-greedy)
    xml = re.sub(r"<!--.*?-->", "", xml, flags=re.DOTALL).strip()

    # collapse excessive whitespace (optional but helps)
    xml = re.sub(r">\s+<", "><", xml)

    return {
        "robot_description": ParameterValue(xml, value_type=str)
    }


def generate_launch_description():
    desc_pkg = get_package_share_directory("beach_robot_description")
    sim_pkg = get_package_share_directory("beach_robot_sim")

    world = os.path.join(sim_pkg, "worlds", "beach_flat.world")
    xacro_file = os.path.join(desc_pkg, "urdf", "beach_robot.urdf.xacro")
    controllers_yaml = os.path.join(desc_pkg, "config", "controllers.yaml")

    robot_description = _xacro_to_robot_description(
        xacro_file,
        mappings={
            "controllers_yaml": controllers_yaml
        }
    )

    # TF from URDF
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Gazebo Classic + gazebo_ros
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    # Spawn robot from robot_description topic
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "beach_robot",
            "-topic", "robot_description",
            "-x", "0", "-y", "0", "-z", "0.05",
        ],
    )

    # Spawners (controller_manager should be created by gazebo_ros2_control plugin)
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawner_ddc = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn,
            on_exit=[
                TimerAction(period=2.0, actions=[spawner_jsb]),
                TimerAction(period=3.0, actions=[spawner_ddc]),
            ],
        )
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        start_controllers,
    ])
