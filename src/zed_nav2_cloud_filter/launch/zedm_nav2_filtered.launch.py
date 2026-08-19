from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("zed_nav2_cloud_filter")
    use_static_tf = LaunchConfiguration("use_static_tf")
    target_frame = LaunchConfiguration("target_frame")
    zed_params_override = LaunchConfiguration("zed_params_override")
    output_cloud_topic = LaunchConfiguration("output_cloud_topic")
    filter_min_range = LaunchConfiguration("filter_min_range")
    filter_max_range = LaunchConfiguration("filter_max_range")
    filter_min_z = LaunchConfiguration("filter_min_z")
    filter_max_z = LaunchConfiguration("filter_max_z")
    filter_decimate_n = LaunchConfiguration("filter_decimate_n")
    filter_throttle_hz = LaunchConfiguration("filter_throttle_hz")

    # ZED wrapper launch file path
    zed_launch = (
        get_package_share_directory("zed_wrapper")
        + "/launch/zed_camera.launch.py"
    )
    static_tf_launch = pkg_share + "/launch/zedm_static_tf.launch.py"

    static_tf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(static_tf_launch),
        condition=IfCondition(use_static_tf),
    )

    zed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch),
        launch_arguments={
            "camera_model": "zedm",
            "camera_name": "zed",

            # IMPORTANT: avoid TF duplication with your static TF broadcaster
            "publish_tf": "false",
            "publish_map_tf": "false",

            # Keep Orin Nano launches stable by default. The override disables
            # depth, point cloud, and positional tracking for IMU-only tests.
            "ros_params_override_path": zed_params_override,
        }.items(),
    )

    cloud_filter_node = Node(
        package="zed_nav2_cloud_filter",
        executable="zed_cloud_filter_node",
        name="zed_cloud_filter_node",
        output="screen",
        parameters=[{
            # ZED wrapper typical registered cloud topic
            "input_cloud_topic": "/zed/zed_node/point_cloud/cloud_registered",

            # Filtered output cloud for Nav2 costmap and obstacle detection
            "output_cloud_topic": output_cloud_topic,

            # Transform cloud into this frame before filtering
            "target_frame": target_frame,

            # Range gate (XY distance in target_frame)
            "min_range": filter_min_range,
            "max_range": filter_max_range,

            # Height gate (Z in target_frame) to remove floor/sky
            "min_z": filter_min_z,
            "max_z": filter_max_z,

            # Cheap downsample: keep 1 of every N points
            "decimate_n": filter_decimate_n,

            # Limit publish rate (0 = no throttle)
            "throttle_hz": filter_throttle_hz,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_static_tf",
            default_value="true",
            description="Publish only the base_link -> ZED static TFs when running standalone.",
        ),
        DeclareLaunchArgument(
            "target_frame",
            default_value="base_link",
            description="Frame to transform the filtered cloud into.",
        ),
        DeclareLaunchArgument(
            "zed_params_override",
            default_value=pkg_share + "/config/zedm_orin_nano_imu_only.yaml",
            description="ZED wrapper YAML override. Default is a low-memory IMU-only profile.",
        ),
        DeclareLaunchArgument("output_cloud_topic", default_value="/zed/filtered_cloud"),
        DeclareLaunchArgument("filter_min_range", default_value="0.25"),
        DeclareLaunchArgument("filter_max_range", default_value="6.0"),
        DeclareLaunchArgument("filter_min_z", default_value="0.05"),
        DeclareLaunchArgument("filter_max_z", default_value="1.50"),
        DeclareLaunchArgument("filter_decimate_n", default_value="8"),
        DeclareLaunchArgument("filter_throttle_hz", default_value="10.0"),
        static_tf_node,
        zed_node,
        cloud_filter_node,
    ])
