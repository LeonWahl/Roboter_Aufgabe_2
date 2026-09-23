from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

   
    return LaunchDescription([

        

        # Online Line Finder
        Node(
            package='online_line_finder',
            executable='online_line_finder',
            name='online_line_finder',
            output='screen'
        ),

        # Graph Provider
        Node(
            package='robotik_pfadplannung',
            executable='graph_provider',
            name='graph_provider',
            output='screen'
        ),

        # Path Planner
        Node(
            package='robotik_pfadplannung',
            executable='path_planner',
            name='path_planner',
            output='screen'
        )
    ])
