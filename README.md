# `lidar_cluster` ROS 2 package
LIDAR pointcloud clustering in `ROS 2` `Humble`

[![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)](https://docs.ros.org/en/humble/)

<center>
<img src="img/lidar_cluster01.png" width="80%"> 
</center>

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

``` bash
sudo rosdep install --from-paths src --ignore-src -r -y
```

Build either with:

``` bash
colcon build --packages-select lidar_cluster --symlink-install
```

or with optimized build:

``` bash
MAKEFLAGS="-j4" colcon build --packages-select lidar_cluster --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> [!TIP]
> There is a detailded decription how to make this package work with ground segmentation: [jkk-research.github.io/workshops/clustering_a](https://jkk-research.github.io/workshops/clustering_a/)

## ROS 2 graph


``` mermaid
graph LR;

    p[ /input_points<br/>sensor_msgs::PointCloud2]:::white --> cluster([ /cluster_node]):::light
    tf[ /tf <br/>optional topic]:::dash -.-> cluster
    cluster --> f1[ /clustered_points<br/>sensor_msgs::PointCloud2]:::white
    cluster --> f2[ /clustered_marker<br/>visualization_msgs::MarkerArray]:::white
    classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
    classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
    classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#15274
    classDef dash fill:#ffffff,stroke:#152742,stroke-width:2px,color:#15274, stroke-dasharray: 5 5
    classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff
```
## Cluster nodes

| Node | Clustering | Implementation | Description | Additional Resources |
| --- | --- | --- | --- | --- |
| `dbscan_spatial` | DBSCAN | spatial| DBSCAN non-grid implementation | |
| `dbscan_grid` | DBSAN | grid | DBSCAN voxel-grid-based implementation | |
| `dblane_spatial` | DBlane | spatial| DBlane non-grid implementation | [flowchart](https://github.com/jkk-research/lidar_cluster_ros2/blob/ros2/notebooks/flowchart.md), [notebooks](https://github.com/jkk-research/lidar_cluster_ros2/tree/ros2/notebooks) |
| `dblane_f1s` | DBlane | formula | DBlane formula 1 student implementation | [notebooks](https://github.com/jkk-research/lidar_cluster_ros2/tree/ros2/notebooks) |
| `euclidean_spatial` | Euclidean | spatial| PCL implementation of Euclidean clustering non-grid implementation | [PCL docs](https://pointclouds.org/documentation/group__segmentation.html) |
| `euclidean_grid` | Euclidean | grid| PCL implementation of Euclidean clustering voxel-grid-based implementation | [PCL docs](https://pointclouds.org/documentation/group__segmentation.html) |


<details>
<summary> Don't forget to source before ROS commands.</summary>

``` bash
source ~/ros2_ws/install/setup.bash
```
</details>

``` bash
ros2 run lidar_cluster dblane_spatial
```

``` bash
ros2 launch lidar_cluster dblane_spatial.launch.py
```

``` bash
ros2 launch lidar_cluster dblane_f1s.launch.py topic:=/input_points
```

# Dblane-g launch with simulator

To run `dblane-g` with the racetrack simulator, use the Formula Student simulator package from:

https://github.com/szenergy/formula_student_packages/tree/racetracksim-extradata

> [!IMPORTANT]
> Switch to the `racetracksim-extradata` branch before building.

Example setup in `~/ros2_ws/src`:

``` bash
cd ~/ros2_ws/src
git clone https://github.com/szenergy/formula_student_packages.git
cd formula_student_packages
git checkout racetracksim-extradata
```

Build the racetrack simulator package:

``` bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select racetrack_simulator --symlink-install
source ~/ros2_ws/install/setup.bash
```

Start the simulator:

``` bash
ros2 launch racetrack_simulator lidar_cone_sim.launch.py
```

Start `dblane_f1s` in a new terminal (this command starts the cluster node and RViz by default):

``` bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch lidar_cluster dblane_f1s.launch.py
```

# Start the evaluation node in another terminal:

``` bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run lidar_cluster evaluation
```

## Evaluation node controls:

- Press `e`: evaluate the currently available (already received) input data snapshot, print the metrics, and start publishing evaluation preview topics.
- Press `Ctrl+C`: exit the evaluator immediately.
- Published preview topics include `/point_true_positives`, `/point_false_negatives`, `/point_false_positives`, `/edge_true_positives`, `/edge_false_negatives`, `/edge_false_positives`, and `/gt_edges`.

### Evaluator inputs (exact topics)

- `/nonground_odom` (`sensor_msgs/msg/PointCloud2`): Ground-truth cones are initialized from this stream (left/right/noise classes), so this defines what is considered the reference set.
- `/interpolated_marker_map_odom` (`visualization_msgs/msg/MarkerArray`): Predicted map points and line segments are read from this stream, so this defines what is compared against GT.

When `e` is pressed, the evaluator computes results only from the data that has already arrived on the two input topics at that moment.

### Evaluator outputs

- Terminal output: point-level, hybrid, and edge-level metrics are printed (TP/FP/FN, precision, sensitivity, F-measure, etc.) for the current snapshot.
- `/point_true_positives`: GT cone points that are matched by prediction within threshold.
- `/point_false_negatives`: GT cone points that are currently unmatched.
- `/point_false_positives`: predicted points that currently do not match any GT cone.
- `/edge_true_positives`: GT consecutive edges that are matched by predicted edges.
- `/edge_false_negatives`: GT edges that are currently unmatched.
- `/edge_false_positives`: predicted edges that do not match GT edges.
- `/gt_edges`: the GT left/right edge chains in `odom`; useful as a stable reference overlay in RViz.

## Remarks

In VS code it is advised to add the following to include path:

``` r
${workspaceFolder}/**
/opt/ros/humble/include/**
/usr/include/pcl-1.12/**
/usr/include/eigen3/**
```

If you are not sure where your header files are use e.g.:
``` r
find /usr/include -name point_cloud.h
find /usr/include -name crop_box.h
```

## Images

> [!NOTE]  
> The following images shows two possible usage of clustering: an urban scenario and a race scenario.
> The first row shows a camera image that corresponds to the LIDAR image in the second row. In the third row an example is shown of the clustered LIDAR pointcloud with yellow.


| Urban scenario | Race scenario |
|:---:|:---:|
<img src="img/lidar_cluster_a2_urban.gif" width="100%"> | <img src="img/lidar_cluster_a1_race.gif" width="100%"> 
<img src="img/lidar_cluster_b2_urban.gif" width="100%"> | <img src="img/lidar_cluster_b1_race.gif" width="100%">
<img src="img/lidar_cluster_c2_urban.gif" width="100%"> | <img src="img/lidar_cluster_c1_race.gif" width="100%"> 


# Citation

If you use any of this code please consider citing the [paper](https://ieeexplore.ieee.org/document/10607072):

``` bibtex
@INPROCEEDINGS{10607072,
  author={Unger, Miklós and Horváth, Ernő and Pup, Dániel and Pozna, Claudiu Radu},
  booktitle={2024 IEEE International Conference on Mobility, Operations, Services and Technologies (MOST)}, 
  title={Towards Robust LIDAR Lane Clustering for Autonomous Vehicle Perception in ROS 2}, 
  year={2024},
  pages={229-234},
  keywords={Laser radar;Lane detection;Source coding;Clustering algorithms;Robustness;Pattern recognition;Autonomous vehicles;self-driving;autonomous;point cloud;LIDAR;proceeding;filter;geometric patterns},
  doi={10.1109/MOST60774.2024.00031}}
```