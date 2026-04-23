import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_cluster')

    return LaunchDescription([
        DeclareLaunchArgument("topic", description="a pointcloud topic to process", default_value="nonground"),
        DeclareLaunchArgument("visualization", description="start RViz with cluster config", default_value="true"),
        Node(
            package='lidar_cluster',
            executable='dblane_f1s',
            output='screen',
            parameters=[
                {'points_in_topic': LaunchConfiguration("topic")},
                {'points_out_topic': 'clustered_points'},
                {'marker_out_topic': 'clustered_marker'},
                {'minX': -20.0},
                {'minY': -20.0},
                {'maxX': 0.0},
                {'maxY': 20.0},
                {'maxZ': 0.2},
                {'minZ': 0.0},
                {'verbose1': False},
                {'search_start_width_x': 20.0},
                {'search_start_width_y': 6.5},
                {'eps_min': 0.2},
                {'eps_max': 5.0},
                {'ang_threshold_deg': 50.0},
                {'origin_filter_radius': 0.25},
            ]
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('visualization')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'rviz_cluster_sim.rviz')],
        )
    ])