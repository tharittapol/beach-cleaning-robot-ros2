from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    gga_send_period = LaunchConfiguration('gga_send_period')
    force_gpgga = LaunchConfiguration('force_gpgga')
    # Explicit value_type so an empty override (no NTRIP account set) stays a
    # string instead of being coerced to None.
    ntrip_host = ParameterValue(LaunchConfiguration('ntrip_host'), value_type=str)
    ntrip_port = ParameterValue(LaunchConfiguration('ntrip_port'), value_type=int)
    mountpoint = ParameterValue(LaunchConfiguration('mountpoint'), value_type=str)
    username = ParameterValue(LaunchConfiguration('username'), value_type=str)
    password = ParameterValue(LaunchConfiguration('password'), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gga_send_period', default_value='5.0'),
        DeclareLaunchArgument('force_gpgga', default_value='true'),

        # NTRIP caster settings. Defaults come from the environment so no account
        # details live in the repo:
        #   export NTRIP_HOST=rtk2go.com NTRIP_MOUNTPOINT=<your-base-station>
        #   export NTRIP_USERNAME=<your-caster-login> NTRIP_PASSWORD=<your-password>
        # (rtk2go uses your e-mail with @ -> -at- and . -> -d-, password "none".)
        DeclareLaunchArgument(
            'ntrip_host',
            default_value=EnvironmentVariable('NTRIP_HOST', default_value='rtk2go.com'),
        ),
        DeclareLaunchArgument(
            'ntrip_port',
            default_value=EnvironmentVariable('NTRIP_PORT', default_value='2101'),
        ),
        DeclareLaunchArgument(
            'mountpoint',
            default_value=EnvironmentVariable('NTRIP_MOUNTPOINT', default_value=''),
            description='NTRIP mountpoint (base station) to pull RTCM corrections from.',
        ),
        DeclareLaunchArgument(
            'username',
            default_value=EnvironmentVariable('NTRIP_USERNAME', default_value=''),
            description='NTRIP caster login. Empty disables the RTCM stream.',
        ),
        DeclareLaunchArgument(
            'password',
            default_value=EnvironmentVariable('NTRIP_PASSWORD', default_value='none'),
        ),

        Node(
            package='beach_robot_gnss',
            executable='um982_bridge_node',
            name='um982_ntrip_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/ttyGNSS',
                'baud': 115200,
                'frame_id': 'gps_link',

                'ntrip_host': ntrip_host,
                'ntrip_port': ntrip_port,
                'mountpoint': mountpoint,

                'username': username,
                'password': password,
                'user_agent': 'NTRIP UM982/1.0',

                'gga_send_period': gga_send_period,
                'force_gpgga': force_gpgga,
            }],
        ),
    ])
