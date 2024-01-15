from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():



    return LaunchDescription([
        DeclareLaunchArgument("topic", description="a pointcloud topic to process", default_value="nonground"),
        Node(
            package='lidar_cluster',
            executable='dblane_f1s',
            output='screen',
            parameters=[
                {'points_in_topic': LaunchConfiguration("topic")},
                {'points_out_topic': 'clustered_points'},
                {'marker_out_topic': 'clustered_marker'},
                {'verbose1': True},
                {'search_start_width_x': 20.0},
                {'search_start_width_y': 6.5},
                {'eps_min': 1.5},
                {'esp_max': 4.0},
                {'ang_threshold_deg': 35.0},
            ]
        )
    ])