# `lidar_cluster` ROS 2 package
LIDAR pointcloud clustering in `ROS 2` `Humble`

[![Static Badge](https://img.shields.io/badge/ROS_2-Humble-blue)](https://docs.ros.org/en/humble/)


## Build this `ROS 2` package

In the following `~/ros2_ws` is assumed as the ROS 2 workspace:

``` bash
cd ~/ros2_ws/src
```

``` bash
git clone https://github.com/jkk-research/lidar_cluster_ros2
```

``` bash
cd ~/ros2_ws
```

Build either with:

``` bash
colcon build --packages-select lidar_cluster
```

or with optimized build:

``` bash
MAKEFLAGS="-j4" colcon build --packages-select lidar_cluster --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## ROS 2 graph


``` mermaid
graph LR;

    p[ /input_points<br/>sensor_msgs::PointCloud2]:::white --> cluster([ /cluster_node]):::light
    tf[ /tf <br/>optional topic]:::dash -.-> cluster
    cluster --> f1[ /filtered_points<br/>sensor_msgs::PointCloud2]:::white
    cluster --> f2[ /filtered_markers<br/>visualization_msgs::MarkerArray]:::white
    classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
    classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
    classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#15274
    classDef dash fill:#ffffff,stroke:#152742,stroke-width:2px,color:#15274, stroke-dasharray: 5 5
    classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff
```
## Cluster nodes
- `dbscan_spatial`: DBSCAN non-grid implementation.
- `dbscan_grid`: DBSCAN grid implementation.
- `dblane_spatial`: DBlane non-grid implementation. The [flowchart](https://github.com/jkk-research/lidar_cluster_ros2/blob/ros2/notebooks/flowchart.md) and the [notebooks](https://github.com/jkk-research/lidar_cluster_ros2/tree/ros2/notebooks) can help to understand the working principles.

``` bash
ros2 run lidar_cluster dblane_spatial
```