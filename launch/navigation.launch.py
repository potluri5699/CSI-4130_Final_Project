from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory('osm_robot_navigation')
    rviz_config = os.path.join(share, 'config', 'navigation.rviz')
    return LaunchDescription([
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
