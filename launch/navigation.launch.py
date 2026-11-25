from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('osm_robot_navigation')
    rviz_config = os.path.join(share, 'rviz', 'navigation.rviz')
    urdf_file = os.path.join(share, 'urdf', 'audibot_description.urdf')

    blue = LaunchConfiguration('blue')
    speed = LaunchConfiguration('speed')
    time_scale = LaunchConfiguration('time_scale')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' blue:=', blue]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'blue',
            default_value='true',
            description='Use the blue Audibot body mesh (false = orange)',
        ),
        DeclareLaunchArgument(
            'speed',
            default_value='15.0',
            description='Audibot cruise speed along the route in m/s',
        ),
        DeclareLaunchArgument(
            'time_scale',
            default_value='1.0',
            description='Simulation speed-up factor for the route playback',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='osm_robot_navigation',
            executable='path_follower',
            name='audibot_path_follower',
            parameters=[{
                'speed': ParameterValue(speed, value_type=float),
                'time_scale': ParameterValue(time_scale, value_type=float),
            }],
            output='screen',
        ),
        Node(
            package='osm_robot_navigation',
            executable='route_planner',
            name='osm_astar_route_planner',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
        Node(
            package='osm_robot_navigation',
            executable='location_gui',
            name='osm_navigation_gui',
            output='screen',
        ),
    ])
