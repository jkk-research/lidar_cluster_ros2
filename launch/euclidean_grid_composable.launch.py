
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='euclidean_grid_container',
        namespace='cluster',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_cluster',
                plugin='cluster::EuclideanGrid',
                name='euclidean_grid',
                parameters=[{
                    'minX': -80.0,
                    'minY': -25.0,
                    'minZ': -2.0,
                    'maxX': 80.0,
                    'maxY': 25.0,
                    'maxZ': -0.15,
                    'points_in_topic': '/lexus3/os_center/points',
                    'points_out_topic': 'clustered_points',
                    'marker_out_topic': 'clustered_marker',
                    'verbose1': False,
                    'verbose2': False,
                    'pub_undecided': False,
                    'voxel_leaf_size': 3.0,
                    'tolerance': 5.0,
                    'max_cluster_size': 400,
                    'min_points_number_per_voxel': 5
                }]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])