from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():



    return LaunchDescription([
        Node(
            package='lidar_cluster',
            executable='dbscan_spatial',
            output='screen',
            parameters=[
                {'points_in_topic': 'nonground'},
                {'points_out_topic': 'clustered_points'},
                {'marker_out_topic': 'clustered_marker'},
                {'verbose1': True},
            ]
        )
    ])
