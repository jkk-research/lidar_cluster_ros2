from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    container = ComposableNodeContainer(
        name='euclidean_grid_container',
        namespace='/euc',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='lidar_cluster',
                plugin='cluster::EuclideanGridCore',
                name='euclidean_grid_filter_composable_unique',
                #namespace='euclidean_grid_filter_ns',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[
                {'points_in_topic': 'lexus3/os_center/points',
                'points_out_topic': 'clustered_points',
                'marker_out_topic': 'clustered_marker',
                'tolerance': 5.0,
                'max_cluster_size': 4000,
                'voxel_leaf_size': 3.0,
                'min_points_number_per_voxel': 5,
                'verbose1': False,
                'verbose2': False,}
            ],
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])