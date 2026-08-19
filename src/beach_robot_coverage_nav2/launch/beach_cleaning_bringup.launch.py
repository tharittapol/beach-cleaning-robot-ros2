from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    keepout_mask_yaml = LaunchConfiguration('keepout_mask_yaml')
    use_robot_stack = LaunchConfiguration('use_robot_stack')
    use_zed = LaunchConfiguration('use_zed')
    zed_params_override = LaunchConfiguration('zed_params_override')
    use_gnss = LaunchConfiguration('use_gnss')
    launch_gnss_driver = LaunchConfiguration('launch_gnss_driver')
    use_teleop = LaunchConfiguration('use_teleop')
    esp32_port = LaunchConfiguration('esp32_port')
    wheel_cmd_send_rate_hz = LaunchConfiguration('wheel_cmd_send_rate_hz')
    esp32_debug_enabled = LaunchConfiguration('esp32_debug_enabled')
    publish_raw_json = LaunchConfiguration('publish_raw_json')
    linear_scale = LaunchConfiguration('linear_scale')
    angular_scale = LaunchConfiguration('angular_scale')
    mixer_params_file = LaunchConfiguration('mixer_params_file')
    publish_map_to_odom_tf = LaunchConfiguration('publish_map_to_odom_tf')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    start_coverage = LaunchConfiguration('start_coverage')
    start_delay_sec = LaunchConfiguration('start_delay_sec')
    enable_turn_yaw_cut = LaunchConfiguration('enable_turn_yaw_cut')
    coverage_pattern = LaunchConfiguration('coverage_pattern')
    area_origin_x = LaunchConfiguration('area_origin_x')
    area_origin_y = LaunchConfiguration('area_origin_y')
    area_width = LaunchConfiguration('area_width')
    area_height = LaunchConfiguration('area_height')
    area_yaw = LaunchConfiguration('area_yaw')
    tool_width = LaunchConfiguration('tool_width')
    overlap = LaunchConfiguration('overlap')
    lane_spacing = LaunchConfiguration('lane_spacing')
    lane_center_offset = LaunchConfiguration('lane_center_offset')
    auto_widen_lanes_for_turn = LaunchConfiguration('auto_widen_lanes_for_turn')
    boundary_margin = LaunchConfiguration('boundary_margin')
    waypoint_step = LaunchConfiguration('waypoint_step')
    turn_style = LaunchConfiguration('turn_style')
    turn_radius = LaunchConfiguration('turn_radius')
    minimum_turn_radius = LaunchConfiguration('minimum_turn_radius')
    # Obstacle distances are still forwarded to the coverage node's (disabled) internal
    # stop block for reference; the live detector runs SEPARATELY (zed_obstacle_stop.launch.py).
    obstacle_stop_distance = LaunchConfiguration('obstacle_stop_distance')
    obstacle_box_width = LaunchConfiguration('obstacle_box_width')
    obstacle_clear_time_sec = LaunchConfiguration('obstacle_clear_time_sec')
    num_passes = LaunchConfiguration('num_passes')
    coverage_path_mode = LaunchConfiguration('coverage_path_mode')
    upper_return_count = LaunchConfiguration('upper_return_count')
    deadhead_style = LaunchConfiguration('deadhead_style')
    deadhead_clearance = LaunchConfiguration('deadhead_clearance')
    use_keepout = LaunchConfiguration('use_keepout')

    pkg = get_package_share_directory('beach_robot_coverage_nav2')
    localization_pkg = get_package_share_directory('beach_robot_localization')
    mixer_pkg = get_package_share_directory('beach_wheel_mixer')
    default_keepout_params = os.path.join(pkg, 'config', 'nav2_params_keepout.yaml')
    default_nokeepout_params = os.path.join(pkg, 'config', 'nav2_params_nokeepout.yaml')
    default_keepout_mask_yaml = os.path.join(pkg, 'config', 'keepout_mask.yaml')
    default_zed_depth_params = os.path.join(
        get_package_share_directory('zed_nav2_cloud_filter'),
        'config',
        'zedm_orin_nano_depth.yaml',
    )

    # Nav2 params: explicit nav2_params override wins; otherwise pick by use_keepout.
    nav2_params = PythonExpression([
        "'", LaunchConfiguration('nav2_params'), "' if '",
        LaunchConfiguration('nav2_params'), "' != '' else ('",
        default_keepout_params, "' if '", use_keepout,
        "'.lower() == 'true' else '", default_nokeepout_params, "')",
    ])
    localization_launch = os.path.join(localization_pkg, 'launch', 'localization_full_test.launch.py')
    default_mixer_params = os.path.join(mixer_pkg, 'config', 'mixer.yaml')

    declare = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'nav2_params', default_value='',
            description='Explicit Nav2 params file; empty = auto-pick by use_keepout.'),
        DeclareLaunchArgument(
            'use_keepout', default_value='false',
            description='false (default): no keepout boundary, deadheads may loop outside the '
                        'work area. true: use nav2_params_keepout.yaml + keepout mask servers.'),
        DeclareLaunchArgument('keepout_mask_yaml', default_value=default_keepout_mask_yaml),
        DeclareLaunchArgument(
            'use_robot_stack',
            default_value='true',
            description='Launch ESP32, mixer, sensors, and EKF so Nav2 has odom->base_link TF.',
        ),
        DeclareLaunchArgument('use_zed', default_value='true'),
        DeclareLaunchArgument(
            'zed_params_override',
            default_value=default_zed_depth_params,
            description='ZED depth profile used when use_zed=true. Must publish point cloud for obstacle stop.',
        ),
        DeclareLaunchArgument('use_gnss', default_value='false'),
        DeclareLaunchArgument('launch_gnss_driver', default_value='true',
                              description='false → GNSS runs as a SEPARATE persistent node '
                                          '(um982_fix_nema.launch.py, kept on so RTK stays converged); '
                                          'the stack then only runs navsat_transform.'),
        DeclareLaunchArgument('use_teleop', default_value='true',
                              description='Joystick teleop during coverage. Needed for the Y E-STOP '
                                          '(teleop → /e_stop → bridge force-stops). It yields /cmd_vel to '
                                          'Nav2 in auto mode, so it does not fight the controller.'),
        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyESP32'),
        DeclareLaunchArgument('wheel_cmd_send_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('esp32_debug_enabled', default_value='false'),
        DeclareLaunchArgument('publish_raw_json', default_value='false'),
        DeclareLaunchArgument('linear_scale', default_value='1.625'),  # tile 2026-06-19: 4.87 m tape @ odom 5.0 (converged from 1.19). Sand was 1.19 (10 m odom vs 8.245 m tape)
        DeclareLaunchArgument('angular_scale', default_value='1.0'),
        DeclareLaunchArgument('mixer_params_file', default_value=default_mixer_params),
        DeclareLaunchArgument(
            'publish_map_to_odom_tf',
            default_value='true',
            description='Publish map->odom using the requested spawn pose. Disable if a global localization source publishes map->odom.',
        ),
        DeclareLaunchArgument(
            'spawn_x',
            default_value='0.0',
            description='Initial robot X in map when odometry starts at zero.',
        ),
        DeclareLaunchArgument(
            'spawn_y',
            default_value='0.3',
            description='Initial robot Y in map; default matches the first coverage lane.',
        ),
        DeclareLaunchArgument(
            'spawn_yaw',
            default_value='0.0',
            description='Initial robot yaw in map; default points straight along +X.',
        ),
        DeclareLaunchArgument('start_coverage', default_value='true'),
        DeclareLaunchArgument('start_delay_sec', default_value='15.0'),
        DeclareLaunchArgument('enable_turn_yaw_cut', default_value='false',
                              description='false → follow the full arc/teardrop path to its end '
                                          '(no early yaw cut). Cleaner when turn_radius matches the '
                                          'robot; avoids the cancel+resend race that skips a lane.'),
        DeclareLaunchArgument('coverage_pattern', default_value='boustrophedon'),
        DeclareLaunchArgument('area_origin_x', default_value='0.0'),
        DeclareLaunchArgument('area_origin_y', default_value='0.0'),
        DeclareLaunchArgument('area_width', default_value='5.0'),
        DeclareLaunchArgument('area_height', default_value='3.6'),
        DeclareLaunchArgument('area_yaw', default_value='0.0'),
        DeclareLaunchArgument('tool_width', default_value='0.60'),
        DeclareLaunchArgument('overlap', default_value='0.0'),
        # Six 5 m lanes at y=0.3,0.9,...,3.3. Every lane runs x=0→5.
        DeclareLaunchArgument('lane_spacing', default_value='0.60'),
        DeclareLaunchArgument('lane_center_offset', default_value='0.30'),
        DeclareLaunchArgument('auto_widen_lanes_for_turn', default_value='false'),
        DeclareLaunchArgument('boundary_margin', default_value='0.0'),  # First lane runs x=0→5; spawn_y places the robot on its y=0.3 centerline.
        DeclareLaunchArgument('waypoint_step', default_value='0.30'),  # tile 2026-06-19: dense waypoints pin NavFn to the lane (0.50 let /plan diagonal-offset up to 0.3 m off-lane when the robot was off-track)
        DeclareLaunchArgument('turn_style', default_value='arc'),
        DeclareLaunchArgument('turn_radius', default_value='2.10'),
        DeclareLaunchArgument('minimum_turn_radius', default_value='2.10'),
        DeclareLaunchArgument(
            'num_passes', default_value='1',
            description='Use one pass: six physical lanes spaced by the 0.60 m tool width.'),
        DeclareLaunchArgument(
            'coverage_path_mode', default_value='same_direction_loops',
            description='Every lane runs x=0 to x=5; exact-radius loops return to x=0.'),
        DeclareLaunchArgument(
            'upper_return_count', default_value='3',
            description='Number of first lane transitions that loop above the area.'),
        DeclareLaunchArgument('deadhead_style', default_value='outside'),  # outside|direct|rounded
        DeclareLaunchArgument('deadhead_clearance', default_value='2.10'),  # ≥ turn_radius for outside loops
        DeclareLaunchArgument('deadhead_settle_sec', default_value='0.8'),  # direct deadhead: hold cmd_vel=0 between pivot/drive segments so the robot fully stops & heading settles
        # --- obstacle-stop tuning (reference only here) ---
        # The live ZED obstacle detector is NOT spawned by this bringup. Launch it separately
        # (persistent, like the GNSS node):
        #   ros2 launch beach_robot_coverage_nav2 zed_obstacle_stop.launch.py launch_zed:=false
        # This bringup already opens the ZED camera (use_zed) and the ESP32 bridge already
        # ORs the manual /e_stop (joystick) with the detector's /safety/e_stop.
        DeclareLaunchArgument('obstacle_stop_distance', default_value='2.0'),
        DeclareLaunchArgument('obstacle_box_width', default_value='0.8'),
        DeclareLaunchArgument('obstacle_clear_time_sec', default_value='3.0'),
    ]

    robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_teleop': use_teleop,
            'use_zed': use_zed,
            'zed_params_override': zed_params_override,
            'use_gnss': use_gnss,
            'launch_gnss_driver': launch_gnss_driver,
            'esp32_port': esp32_port,
            'wheel_cmd_send_rate_hz': wheel_cmd_send_rate_hz,
            'esp32_debug_enabled': esp32_debug_enabled,
            'publish_raw_json': publish_raw_json,
            'linear_scale': linear_scale,
            'angular_scale': angular_scale,
            'mixer_params_file': mixer_params_file,
        }.items(),
        condition=IfCondition(use_robot_stack),
    )

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        # The local EKF starts odom->base_link at zero. Offset map->odom so the robot
        # starts exactly on the first lane and can drive straight immediately.
        arguments=[
            '--x', spawn_x,
            '--y', spawn_y,
            '--z', '0',
            '--yaw', spawn_yaw,
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        condition=IfCondition(publish_map_to_odom_tf),
    )

    keepout_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='keepout_filter_mask_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'topic_name': '/keepout_mask',
            'yaml_filename': keepout_mask_yaml,
        }],
        condition=IfCondition(use_keepout),
    )

    keepout_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='keepout_costmap_filter_info_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'type': 0,
            'filter_info_topic': '/costmap_filter_info',
            'mask_topic': '/keepout_mask',
            'base': 0.0,
            'multiplier': 1.0,
        }],
        condition=IfCondition(use_keepout),
    )

    # Lifecycle manager for the keepout servers — nav2_map_server nodes are
    # lifecycle nodes and must be transitioned to active before Nav2 costmap
    # filters can receive the mask.
    keepout_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'keepout_filter_mask_server',
                'keepout_costmap_filter_info_server',
            ],
        }],
        condition=IfCondition(use_keepout),
    )

    nav2_pkg = get_package_share_directory('nav2_bringup')
    navigation_launch = os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
    coverage_bt_xml = os.path.join(pkg, 'config', 'coverage_navigate_to_pose.xml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'map': keepout_mask_yaml,
            'autostart': 'true',
            'default_bt_xml_filename': coverage_bt_xml,
        }.items()
    )

    coverage = Node(
        package='beach_robot_coverage_nav2',
        executable='coverage_follow_waypoints',
        name='coverage_follow_waypoints',
        output='screen',
        parameters=[{
            # --- area (rectangle) ---
            'area.origin_x': area_origin_x,
            'area.origin_y': area_origin_y,
            'area.width': area_width,     # ด้านยาว (แนววิ่งขนานชายหาด)
            'area.height': area_height,   # ด้านกว้าง (แนว shift lane)
            'area.yaw': area_yaw,         # ถ้าแกน X ของ map ขนานชายหาดอยู่แล้ว

            # --- coverage ---
            'pattern': coverage_pattern,
            'tool_width': tool_width,
            'overlap': overlap,
            'lane_spacing': lane_spacing,
            'lane_center_offset': lane_center_offset,
            'auto_widen_lanes_for_turn': auto_widen_lanes_for_turn,
            'boundary_margin': boundary_margin,
            'waypoint_step': waypoint_step,

            # --- turn ---
            'turn_style': turn_style,
            'turn_radius': turn_radius,
            'minimum_turn_radius': minimum_turn_radius,
            'enable_turn_yaw_cut': enable_turn_yaw_cut,

            # --- multipass (interleaved passes for 100% coverage) ---
            'num_passes': num_passes,
            'coverage_path_mode': coverage_path_mode,
            'upper_return_count': upper_return_count,
            'deadhead_style': deadhead_style,
            'deadhead_clearance': deadhead_clearance,
            'deadhead_settle_sec': LaunchConfiguration('deadhead_settle_sec'),

            # --- auto-mode obstacle stop (ZED straight front box) ---
            # Disabled here: the SEPARATE zed_obstacle_stop node owns the hardware E-stop and
            # buzzer for every Nav2 phase (via /safety/e_stop → ESP32 bridge).
            'obstacle_stop.enabled': False,
            'obstacle_stop.stop_distance': obstacle_stop_distance,
            'obstacle_stop.box_width': obstacle_box_width,
            'obstacle_stop.clear_time_sec': obstacle_clear_time_sec,

            # --- run ---
            'autostart': start_coverage,
            'start_delay_sec': start_delay_sec,
        }],
    )

    # Real-robot footprint + scoop markers for RViz (base_link frame). Wheel quad matches
    # the costmap footprint in nav2_params_tile.yaml; scoop is the 0.6 m sand tool, centred.
    footprint_viz = Node(
        package='beach_robot_coverage_nav2',
        executable='footprint_viz',
        name='footprint_viz',
        output='screen',
        parameters=[{
            'front_track_width': 0.734,
            'rear_track_width': 1.179,
            'wheelbase': 0.950,
            'scoop_width': tool_width,   # 0.6 m sand scoop, matches coverage tool_width
            'scoop_length': 0.30,        # fore-aft size not measured yet — adjust if known
            'scoop_x_offset': 0.0,       # centred under chassis
        }],
    )

    return LaunchDescription(declare + [
        robot_stack,
        map_to_odom_tf,
        keepout_mask_server,
        keepout_info_server,
        keepout_lifecycle,
        nav2,
        coverage,
        footprint_viz,
    ])
