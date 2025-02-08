// Eucledian voxel grid clustering filter for Point Cloud data
// Somewhat based on https://autowarefoundation.github.io/autoware.universe/main/perception/euclidean_cluster/ (Apache 2.0 License)

#include <functional>
#include <memory>
#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
// ROS
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/point_tests.h>
// ROS package
#include "lidar_cluster/marker.hpp"
// Benchmarking
#include "benchmark.hpp"
// TBB
#include <tbb/tbb.h>

#include "cluster_outline.hpp"

// Function to create a voxel grid from a point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr createVoxelGrid(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud, // Input point cloud
  pcl::VoxelGrid<pcl::PointXYZ>& voxel_grid,
  float voxel_leaf_size,
  int min_points_number_per_voxel)
{
  // Create a new point cloud to store the voxel grid
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_grid.setMinimumPointsNumberPerVoxel(min_points_number_per_voxel);
  voxel_grid.setInputCloud(pointcloud);
  // Save the leaf layout (this will allow you to later use the getCentroidIndexAt method)
  voxel_grid.setSaveLeafLayout(true);
  voxel_grid.filter(*voxel_map_ptr);
  
  return voxel_map_ptr;
}

// Zero out the z coordinate and project the point cloud to 2D
pcl::PointCloud<pcl::PointXYZ>::Ptr projectTo2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr& voxel_map_ptr)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& point : voxel_map_ptr->points) {
    pcl::PointXYZ point2d;
    point2d.x = point.x;
    point2d.y = point.y;
    point2d.z = 0.0;
    pointcloud_2d_ptr->push_back(point2d);
  }

  return pointcloud_2d_ptr;
}

// Perform clustering
std::vector<pcl::PointIndices> performClustering(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud_2d_ptr, 
  float tolerance, 
  int max_cluster_size)
{
  // We'll store the cluster indices, so we can access the points in the original point cloud later
  std::vector<pcl::PointIndices> cluster_indices;
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_2d_ptr);

  // Euclidean clustering object
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(tolerance);
  pcl_euclidean_cluster.setMinClusterSize(1);
  pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_2d_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);

  return cluster_indices;
}

// Map cluster indices to the original point cloud, so we don't lose the Z coordinate in the clustering process, but still save time by clustering in 2D
std::unordered_map<int, int> mapClusterIndices(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud,
  const pcl::VoxelGrid<pcl::PointXYZ>& voxel_grid,
  const std::vector<pcl::PointIndices>& cluster_indices,
  int &num_of_clusters)
{
  std::unordered_map<int, int> cluster_index_map;
  int cluster_id = 1;

  // Map the cluster indices to the original point cloud
  for (const auto& cluster : cluster_indices) {
    for (const auto& point_idx : cluster.indices) {
      // Get the point index in the original point cloud, based on the voxel grid
      int original_idx = voxel_grid.getCentroidIndexAt(voxel_grid.getGridCoordinates(
          pointcloud->points[point_idx].x, 
          pointcloud->points[point_idx].y, 
          pointcloud->points[point_idx].z
      ));
      if (original_idx >= 0) {
        cluster_index_map[original_idx] = cluster_id;
      }
    }
    ++cluster_id;
    num_of_clusters = cluster_id;
  }

  return cluster_index_map;
}

namespace cluster {

class EuclideanGridCore : public rclcpp::Node
{
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
      if (param.get_name() == "minX")
      {
        minX = param.as_double();
      }
      if (param.get_name() == "minY")
      {
        minY = param.as_double();
      }
      if (param.get_name() == "minZ")
      {
        minZ = param.as_double();
      }
      if (param.get_name() == "maxX")
      {
        maxX = param.as_double();
      }
      if (param.get_name() == "maxY")
      {
        maxY = param.as_double();
      }
      if (param.get_name() == "maxZ")
      {
        maxZ = param.as_double();
      }
      if (param.get_name() == "points_in_topic")
      {
        points_in_topic = param.as_string();
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_in_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&EuclideanGridCore::lidar_callback, this, std::placeholders::_1));
      }
      if (param.get_name() == "points_out_topic")
      {
        points_out_topic = param.as_string();
        pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic, 10);
      }
      if (param.get_name() == "marker_out_topic")
      {
        marker_out_topic = param.as_string();
        pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_out_topic, 10);
      }
      if (param.get_name() == "verbose1")
      {
        verbose1 = param.as_bool();
      }
      if (param.get_name() == "verbose2")
      {
        verbose2 = param.as_bool();
      }
      if (param.get_name() == "pub_undecided")
      {
        pub_undecided = param.as_bool();
      }
      if (param.get_name() == "voxel_leaf_size")
      {
        voxel_leaf_size = param.as_double();
      }
      if (param.get_name() == "tolerance")
      {
        tolerance = param.as_double();
      }
      if (param.get_name() == "max_cluster_size")
      {
        max_cluster_size = param.as_int();
      }
      if (param.get_name() == "min_points_number_per_voxel")
      {
        min_points_number_per_voxel = param.as_int();
      }
    }
    return result;
  }

public:
  EuclideanGridCore(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("euclidean_grid",options), count_(0)
  {
    parse_parameters();

    set_publisher_and_callback();
  }

private:
  benchmark::Timer fullbenchmark;
  benchmark::Timer clustering;
  benchmark::Timer convex_hull;
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    fullbenchmark.start("fullbenchmark", verbose2);
    clustering.start("clustering", verbose2);

    visualization_msgs::msg::MarkerArray mark_array;

    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_msg, *cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    int original_size = cloud->width * cloud->height;

    // Filter out points outside of the box
    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    crop.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    crop.filter(*cloud);

    // Create PointXYZ version of pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_xyz);

    if (verbose1)
    {
      // Print the length of the pointcloud
      RCLCPP_INFO_STREAM(this->get_logger(), "PointCloud in: " << original_size << " reduced size before cluster: " << cloud->width * cloud->height);
    }

    // if any point in the pointcloud is not a proper number, modify that value to zero
    for (auto &point : cloud_xyz->points) {
        if (!pcl::isFinite(point)) {
            point.x = 0.0;
            point.y = 0.0;
            point.z = 0.0;
        }
    }

    // Create voxel grid and project to 2D
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_map_ptr = createVoxelGrid(cloud_xyz, voxel_grid, voxel_leaf_size, min_points_number_per_voxel);
    auto pointcloud_2d_ptr = projectTo2D(voxel_map_ptr);


    // Perform clustering
    auto cluster_indices = performClustering(pointcloud_2d_ptr, tolerance, max_cluster_size);


    int num_of_clusters = 0;

    // Map cluster indices
    auto cluster_index_map = mapClusterIndices(voxel_map_ptr, voxel_grid, cluster_indices, num_of_clusters);

    // Assign intensities and construct the filtered cloud
    for (const auto& point : cloud->points) {
      pcl::PointXYZI point_i;
      point_i.x = point.x;
      point_i.y = point.y;
      point_i.z = point.z;
      auto voxel_idx = voxel_grid.getCentroidIndexAt(voxel_grid.getGridCoordinates(point.x, point.y, point.z));
      if (cluster_index_map.find(voxel_idx) != cluster_index_map.end()) {
        point_i.intensity = static_cast<float>(cluster_index_map[voxel_idx]);
        cloud_filtered->points.push_back(point_i);
      } else if (pub_undecided) {
        point_i.intensity = 0; // Or some value indicating no cluster
        cloud_filtered->points.push_back(point_i);
      }
    }
    max_clust_reached = std::max(max_clust_reached, num_of_clusters);
    
    std::vector<double> center_x(num_of_clusters + 1), center_y(num_of_clusters + 1);
    std::vector<int> count(num_of_clusters + 1);
    for (int i = 0; i <= num_of_clusters; i++) {
      center_x[i] = 0.0;
      center_y[i] = 0.0;
      count[i] = 0;
    }

    for (const auto& point : cloud_filtered->points) {
      int cluster_id = static_cast<int>(point.intensity);
      center_x[cluster_id + 1] += point.x;
      center_y[cluster_id + 1] += point.y;
      count[cluster_id + 1]++;
    }

    for (int i = 1; i <= num_of_clusters; i++) {
      if (count[i] > 0) {
        center_x[i] /= count[i];
        center_y[i] /= count[i];
        visualization_msgs::msg::Marker center_marker;
        init_center_marker(center_marker, center_x[i], center_y[i], i);
        center_marker.header.frame_id = input_msg->header.frame_id;
        center_marker.header.stamp = this->now();
        mark_array.markers.push_back(center_marker);
      }
    }
    // Add markers for clusters that are not present in the current frame to avoid ghost markers
    for(int i = num_of_clusters + 1; i <= max_clust_reached; i++) {
      visualization_msgs::msg::Marker center_marker;
      init_center_marker(center_marker, 0, 0, i);
      center_marker.header.frame_id = input_msg->header.frame_id;
      center_marker.header.stamp = this->now();
      center_marker.color.a = 0.0;
      mark_array.markers.push_back(center_marker);
    }

    clustering.finish();
    convex_hull.start("convex_hull", verbose2);

    // Compute and draw an outline around the clusters
    cluster_outline.computeOutline(cloud_filtered, mark_array, 20, max_clust_reached, input_msg->header.frame_id);

    convex_hull.finish();
    fullbenchmark.finish();

    // Convert to ROS data type and publish
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_filtered, output_msg);
    output_msg.header.frame_id = input_msg->header.frame_id;
    pub_lidar_->publish(output_msg);
    pub_marker_->publish(mark_array);
  } // EuclideanGridCore::lidar_callback

  void parse_parameters(){
        this->declare_parameter<float>("minX", minX);
    this->declare_parameter<float>("minY", minY);
    this->declare_parameter<float>("minZ", minZ);
    this->declare_parameter<float>("maxX", maxX);
    this->declare_parameter<float>("maxY", maxY);
    this->declare_parameter<float>("maxZ", maxZ);
    this->declare_parameter<std::string>("points_in_topic", "/lexus3/os_center/points");
    this->declare_parameter<std::string>("points_out_topic", "clustered_points");
    this->declare_parameter<std::string>("marker_out_topic", "clustered_marker");
    this->declare_parameter<bool>("verbose1", verbose1);
    this->declare_parameter<bool>("verbose2", verbose2);
    this->declare_parameter<bool>("pub_undecided", pub_undecided);
    this->declare_parameter<float>("voxel_leaf_size", voxel_leaf_size);
    this->declare_parameter<float>("tolerance", tolerance);
    this->declare_parameter<int>("max_cluster_size", max_cluster_size);
    this->declare_parameter<int>("min_points_number_per_voxel", min_points_number_per_voxel);

    this->get_parameter("minX", minX);
    this->get_parameter("minY", minY);
    this->get_parameter("minZ", minZ);
    this->get_parameter("maxX", maxX);
    this->get_parameter("maxY", maxY);
    this->get_parameter("maxZ", maxZ);
    this->get_parameter("points_in_topic", points_in_topic);
    this->get_parameter("points_out_topic", points_out_topic);
    this->get_parameter("marker_out_topic", marker_out_topic);
    this->get_parameter("verbose1", verbose1);
    this->get_parameter("verbose2", verbose2);
    this->get_parameter("pub_undecided", pub_undecided);
    this->get_parameter("voxel_leaf_size", voxel_leaf_size);
    this->get_parameter("tolerance", tolerance);
    this->get_parameter("max_cluster_size", max_cluster_size);
    this->get_parameter("min_points_number_per_voxel", min_points_number_per_voxel);
  }

  void set_publisher_and_callback(){
    pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic, 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_out_topic, 10);
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_in_topic, 10, std::bind(&EuclideanGridCore::lidar_callback, this, std::placeholders::_1));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&EuclideanGridCore::parametersCallback, this, std::placeholders::_1));


    RCLCPP_INFO(this->get_logger(), "EuclideanGrid node has been started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: '%s'", points_in_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: '%s' and '%s'", points_out_topic.c_str(), marker_out_topic.c_str());
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  outline::ClusterOutline cluster_outline;

  float minX = -80.0, minY = -25.0, minZ = -2.0;
  float maxX = +80.0, maxY = +25.0, maxZ = -0.15;
  float tolerance = 5;
  int max_cluster_size = 400, max_clust_reached = 0;
  float voxel_leaf_size = 3.0;
  int min_points_number_per_voxel = 5;
  bool verbose1 = false, verbose2 = false, pub_undecided = false;
  std::string points_in_topic, points_out_topic, marker_out_topic;
  size_t count_;
};

} // namespace cluster