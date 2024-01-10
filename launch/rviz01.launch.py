from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_name = 'lidar_cluster'
    pkg_dir = get_package_share_directory(pkg_name)

    return LaunchDescription([
        Node(
            package='rviz2',
            # namespace='',
            executable='rviz2',
            arguments=['-d', [os.path.join(pkg_dir, 'config', 'rviz01.rviz')]]
        )
    ])
