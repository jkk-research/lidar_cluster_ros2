// DBlane filter for point cloud data
// formula 1 student version

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <limits>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <iostream>
// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
// Package
#include "lidar_cluster/dblane.hpp"
#include "lidar_cluster/marker.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// Silence verbose debug/warn logs; keep only startup RCLCPP_INFO.
#ifdef RCLCPP_INFO_STREAM
#undef RCLCPP_INFO_STREAM
#endif
#define RCLCPP_INFO_STREAM(...) ((void)0)

#ifdef RCLCPP_INFO_THROTTLE
#undef RCLCPP_INFO_THROTTLE
#endif
#define RCLCPP_INFO_THROTTLE(...) ((void)0)

#ifdef RCLCPP_WARN
#undef RCLCPP_WARN
#endif
#define RCLCPP_WARN(...) ((void)0)

#ifdef RCLCPP_WARN_STREAM
#undef RCLCPP_WARN_STREAM
#endif
#define RCLCPP_WARN_STREAM(...) ((void)0)

class DblaneFormula : public rclcpp::Node
{
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Param update: " << param.get_name().c_str() << ": " << param.value_to_string().c_str());
        
       if (param.get_name() == "eps_min")
      {
        eps_min = param.as_double();
      }
      if (param.get_name() == "eps_max")
      {
        eps_max = param.as_double();
      }
      if (param.get_name() == "ang_threshold_deg")
      {
        ang_threshold_deg = param.as_double();
      }
      if (param.get_name() == "cluster_num")
      {
        cluster_num = param.as_int();
      }
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
      if (param.get_name() == "verbose1")
      {
        verbose1 = param.as_bool();
      }
      if (param.get_name() == "verbose2")
      {
        verbose2 = param.as_bool();
      }
      if (param.get_name() == "search_start_width_x")
      {
        search_start_width_x = param.as_double();
      }
      if (param.get_name() == "search_start_width_y")
      {
        search_start_width_y = param.as_double();
      }
      if (param.get_name() == "origin_filter_radius")
      {
        origin_filter_radius = param.as_double();
      }
      if (param.get_name() == "global_point_merge_radius")
      {
        global_point_merge_radius_ = param.as_double();
      }
      if (param.get_name() == "points_in_topic")
      {
        points_in_topic = param.as_string();
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_in_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&DblaneFormula::lidar_callback, this, std::placeholders::_1));
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
      if (param.get_name() == "marker_odom_out_topic")
      {
        marker_odom_out_topic_ = param.as_string();
        pub_marker_odom_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_odom_out_topic_, 10);
      }
      if (param.get_name() == "interp_points_out_topic")
      {
        interp_points_out_topic_ = param.as_string();
        pub_interp_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(interp_points_out_topic_, 10);
      }
      if (param.get_name() == "interp_marker_map_out_topic")
      {
        interp_marker_map_out_topic_ = param.as_string();
        pub_interp_marker_map_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(interp_marker_map_out_topic_, 10);
      }
      if (param.get_name() == "interp_point_merge_radius")
      {
        interp_point_merge_radius_ = param.as_double();
      }
      if (param.get_name() == "odom_frame")
      {
        odom_frame_ = param.as_string();
      }
    }
    return result;
  }

public:
  DblaneFormula() : Node("dblane_f1s"), count_(0)
  {
    this->declare_parameter<float>("minX", minX);
    this->declare_parameter<float>("minY", minY);
    this->declare_parameter<float>("minZ", minZ);
    this->declare_parameter<float>("maxX", maxX);
    this->declare_parameter<float>("maxY", maxY);
    this->declare_parameter<float>("maxZ", maxZ);
    this->declare_parameter<std::string>("points_in_topic", "/lexus3/os_center/points");
    this->declare_parameter<std::string>("points_out_topic", "clustered_points");
    this->declare_parameter<std::string>("marker_out_topic", "clustered_marker");
    this->declare_parameter<std::string>("marker_odom_out_topic", "clustered_marker_odom");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<bool>("verbose1", verbose1);
    this->declare_parameter<bool>("verbose2", verbose2);
    this->declare_parameter<float>("search_start_width_x", search_start_width_x);
    this->declare_parameter<float>("search_start_width_y", search_start_width_y);
    this->declare_parameter<int>("cluster_num", cluster_num);
    this->declare_parameter<float>("eps_min", eps_min);
    this->declare_parameter<float>("eps_max", eps_max);
    this->declare_parameter<float>("ang_threshold_deg", ang_threshold_deg);
    this->declare_parameter<float>("origin_filter_radius", origin_filter_radius);
    this->declare_parameter<float>("global_point_merge_radius", global_point_merge_radius_);
    this->declare_parameter<std::string>("interp_points_out_topic", interp_points_out_topic_);
    this->declare_parameter<std::string>("interp_marker_map_out_topic", interp_marker_map_out_topic_);
    this->declare_parameter<float>("interp_point_merge_radius", interp_point_merge_radius_);
    this->get_parameter("minX", minX);
    this->get_parameter("minY", minY);
    this->get_parameter("minZ", minZ);
    this->get_parameter("maxX", maxX);
    this->get_parameter("maxY", maxY);
    this->get_parameter("maxZ", maxZ);
    this->get_parameter("points_in_topic", points_in_topic);
    this->get_parameter("points_out_topic", points_out_topic);
    this->get_parameter("marker_out_topic", marker_out_topic);
    this->get_parameter("marker_odom_out_topic", marker_odom_out_topic_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("verbose1", verbose1);
    this->get_parameter("verbose2", verbose2);
    this->get_parameter("search_start_width_x", search_start_width_x);
    this->get_parameter("search_start_width_y", search_start_width_y);
    this->get_parameter("cluster_num", cluster_num);
    this->get_parameter("eps_min", eps_min);
    this->get_parameter("eps_max", eps_max);
    this->get_parameter("ang_threshold_deg", ang_threshold_deg);
    this->get_parameter("origin_filter_radius", origin_filter_radius);
    this->get_parameter("global_point_merge_radius", global_point_merge_radius_);
    this->get_parameter("interp_points_out_topic", interp_points_out_topic_);
    this->get_parameter("interp_marker_map_out_topic", interp_marker_map_out_topic_);
    this->get_parameter("interp_point_merge_radius", interp_point_merge_radius_);

    pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic, 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_out_topic, 10);
    pub_marker_odom_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_odom_out_topic_, 10);
    pub_interp_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(interp_points_out_topic_, 10);
    pub_interp_marker_map_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(interp_marker_map_out_topic_, 10);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_in_topic, 10, std::bind(&DblaneFormula::lidar_callback, this, std::placeholders::_1));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DblaneFormula::parametersCallback, this, std::placeholders::_1));
    sub_marker_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("clustered_marker_euc", 10, std::bind(&DblaneFormula::marker_callback, this, std::placeholders::_1));
    sub_vehicle_speed_ = this->create_subscription<std_msgs::msg::Float32>("vehicle_speed", 10, std::bind(&DblaneFormula::vehicle_speed_callback, this, std::placeholders::_1));
    sub_steering_angle_ = this->create_subscription<std_msgs::msg::Float32>("steering_angle", 10, std::bind(&DblaneFormula::steering_angle_callback, this, std::placeholders::_1));
    sub_free_space_ = this->create_subscription<visualization_msgs::msg::Marker>("free_space_marker", 10, std::bind(&DblaneFormula::free_space_callback, this, std::placeholders::_1));
    sub_free_space_convex_ = this->create_subscription<visualization_msgs::msg::Marker>("free_space_convex_marker", 10, std::bind(&DblaneFormula::free_space_convex_callback, this, std::placeholders::_1));

    // Initialize motion tracking
    last_update_time_ = this->now();
    motion_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),  // 20 Hz = 50ms period
      std::bind(&DblaneFormula::update_motion, this)
    );

    RCLCPP_INFO(this->get_logger(), "DblaneFormula node has been started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: '%s'", points_in_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: '%s' and '%s'", points_out_topic.c_str(), marker_out_topic.c_str());
  }

private:
  bool is_new_global_point(const Point &candidate) const
  {
    const double merge_radius_sq = global_point_merge_radius_ * global_point_merge_radius_;
    for (const auto &existing : global_points_)
    {
      const double dx = existing.x - candidate.x;
      const double dy = existing.y - candidate.y;
      if ((dx * dx + dy * dy) <= merge_radius_sq)
      {
        return false;
      }
    }
    return true;
  }

  void ingest_global_points(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &local_cloud,
    const geometry_msgs::msg::TransformStamped &sensor_to_global)
  {
    tf2::Transform tf_transform;
    tf2::fromMsg(sensor_to_global.transform, tf_transform);

    for (const auto &local_point : local_cloud->points)
    {
      const tf2::Vector3 transformed = tf_transform * tf2::Vector3(local_point.x, local_point.y, local_point.z);
      Point global_point(transformed.x(), transformed.y());
      if (is_new_global_point(global_point))
      {
        global_points_.push_back(global_point);
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr global_points_as_cloud() const
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud->points.reserve(global_points_.size());
    for (const auto &point : global_points_)
    {
      pcl::PointXYZI pcl_point;
      pcl_point.x = point.x;
      pcl_point.y = point.y;
      pcl_point.z = 0.0f;
      pcl_point.intensity = 0.0f;
      cloud->points.push_back(pcl_point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr crop_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, double min_x_, double min_y_, double max_x_, double max_y_)
  {
    pcl::CropBox<pcl::PointXYZI> crop_fwd;
    crop_fwd.setInputCloud(cloud_in);
    crop_fwd.setMin(Eigen::Vector4f(min_x_, min_y_, -5.0, 1.0));
    crop_fwd.setMax(Eigen::Vector4f(max_x_, max_y_, 5.0, 1.0));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZI>);
    crop_fwd.filter(*cloud_cropped);
    // RCLCPP_INFO_STREAM(this->get_logger(), "crop_fwd: " << cloud_cropped->width * cloud_cropped->height);
    return cloud_cropped;
  }

  // Helper function to resolve crossing between clusters
  // Returns: 0 = no crossing, 1 = keep left remove right points after crossing, 2 = keep right remove left points after crossing
  int resolve_cluster_crossing(Cluster &cluster, int idA, int idB)
  {
    if (cluster.get_size(idA) < 2 || cluster.get_size(idB) < 2) return 0;
    
    // Find crossing segments
    int crossing_seg_a = -1, crossing_seg_b = -1;
    
    for (int i = 0; i < cluster.get_size(idA) - 1; ++i)
    {
      Point a1 = cluster.get_cluster_point(idA, i);
      Point a2 = cluster.get_cluster_point(idA, i + 1);
      for (int j = 0; j < cluster.get_size(idB) - 1; ++j)
      {
        Point b1 = cluster.get_cluster_point(idB, j);
        Point b2 = cluster.get_cluster_point(idB, j + 1);
        if (segments_intersect(a1, a2, b1, b2))
        {
          crossing_seg_a = i;
          crossing_seg_b = j;
          break;
        }
      }
      if (crossing_seg_a != -1) break;
    }
    
    if (crossing_seg_a == -1) return 0; // No crossing found
    
    RCLCPP_INFO_STREAM(this->get_logger(), "Crossing detected at segment " << crossing_seg_a 
      << " (cluster " << idA << ") and segment " << crossing_seg_b << " (cluster " << idB << ")");
    
    // Go back one segment in each cluster
    int check_seg_a = (crossing_seg_a > 0) ? crossing_seg_a - 1 : 0;
    int check_seg_b = (crossing_seg_b > 0) ? crossing_seg_b - 1 : 0;
    
    if (check_seg_a < cluster.get_size(idA) - 1 && check_seg_b < cluster.get_size(idB) - 1)
    {
      Point tail_a = cluster.get_cluster_point(idA, check_seg_a + 1);
      Point tail_b = cluster.get_cluster_point(idB, check_seg_b + 1);
      double tail_angle_a = cluster.get_tail_angle(idA);
      double tail_angle_b = cluster.get_tail_angle(idB);
      
      // Find best candidate for each cluster at the check point
      double best_angle_diff_a = std::numeric_limits<double>::infinity();
      double best_angle_diff_b = std::numeric_limits<double>::infinity();
      
      for (const Point& cand : cluster.candidate_points)
      {
        double dist_a = cluster.distance(tail_a, cand);
        double dist_b = cluster.distance(tail_b, cand);
        
        if (eps_min <= dist_a && dist_a <= eps_max)
        {
          double ang_a = cluster.calculate_angle(tail_a, cand);
          double diff_a = std::abs(cluster.angle_diff(ang_a, tail_angle_a));
          if (diff_a < best_angle_diff_a) best_angle_diff_a = diff_a;
        }
        
        if (eps_min <= dist_b && dist_b <= eps_max)
        {
          double ang_b = cluster.calculate_angle(tail_b, cand);
          double diff_b = std::abs(cluster.angle_diff(ang_b, tail_angle_b));
          if (diff_b < best_angle_diff_b) best_angle_diff_b = diff_b;
        }
      }
      
      if (best_angle_diff_a < best_angle_diff_b)
      {
        return 1; // Keep idA, trim idB
      }
      else
      {
        return 2; // Keep idB, trim idA
      }
    }
    
    return 0;
  }

  visualization_msgs::msg::MarkerArray crop_markers(const visualization_msgs::msg::MarkerArray& input_markers, double minX, double minY, double maxX, double maxY)
  {
    visualization_msgs::msg::MarkerArray filtered_markers;

    for (const auto& marker : input_markers.markers)
    {
      if (marker.ns == "cluster_center" &&
          marker.pose.position.x >= minX && marker.pose.position.x <= maxX &&
          marker.pose.position.y >= minY && marker.pose.position.y <= maxY)
      {
        filtered_markers.markers.push_back(marker);
      }
    }

    return filtered_markers;
  }

  // Helper function to calculate candidate score
  double calculate_score(double angle_difference, double distance)
  {
    return angle_difference + 0.001 * distance;
  }

  struct NongroundScatterStats
  {
    size_t count = 0;
    double mean_x = 0.0;
    double mean_y = 0.0;
    double std_x = 0.0;
    double std_y = 0.0;
    double radial_std = 0.0;
  };

  NongroundScatterStats compute_nonground_scatter_stats(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) const
  {
    NongroundScatterStats stats;
    if (!cloud || cloud->points.empty())
    {
      return stats;
    }

    stats.count = cloud->points.size();
    for (const auto &p : cloud->points)
    {
      stats.mean_x += p.x;
      stats.mean_y += p.y;
    }
    stats.mean_x /= static_cast<double>(stats.count);
    stats.mean_y /= static_cast<double>(stats.count);

    double var_x = 0.0;
    double var_y = 0.0;
    double radial_var = 0.0;
    for (const auto &p : cloud->points)
    {
      const double dx = p.x - stats.mean_x;
      const double dy = p.y - stats.mean_y;
      var_x += dx * dx;
      var_y += dy * dy;
      radial_var += dx * dx + dy * dy;
    }

    const double inv_n = 1.0 / static_cast<double>(stats.count);
    stats.std_x = std::sqrt(var_x * inv_n);
    stats.std_y = std::sqrt(var_y * inv_n);
    stats.radial_std = std::sqrt(radial_var * inv_n);
    return stats;
  }

  bool points_near(
    const geometry_msgs::msg::Point &a,
    const geometry_msgs::msg::Point &b,
    double tolerance) const
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return (dx * dx + dy * dy + dz * dz) <= (tolerance * tolerance);
  }

  bool marker_has_segment(
    const visualization_msgs::msg::Marker &marker,
    const geometry_msgs::msg::Point &p1,
    const geometry_msgs::msg::Point &p2,
    double tolerance) const
  {
    for (size_t i = 0; i + 1 < marker.points.size(); i += 2)
    {
      const auto &a = marker.points[i];
      const auto &b = marker.points[i + 1];
      const bool same_order = points_near(a, p1, tolerance) && points_near(b, p2, tolerance);
      const bool swapped_order = points_near(a, p2, tolerance) && points_near(b, p1, tolerance);
      if (same_order || swapped_order)
      {
        return true;
      }
    }
    return false;
  }

  void add_unique_segment(
    visualization_msgs::msg::Marker &marker,
    const geometry_msgs::msg::Point &p1,
    const geometry_msgs::msg::Point &p2,
    double tolerance)
  {
    if (!marker_has_segment(marker, p1, p2, tolerance))
    {
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  }

  geometry_msgs::msg::Point transform_point(
    const geometry_msgs::msg::Point &point,
    const geometry_msgs::msg::TransformStamped &transform) const
  {
    tf2::Transform tf_transform;
    tf2::fromMsg(transform.transform, tf_transform);

    const tf2::Vector3 transformed = tf_transform * tf2::Vector3(point.x, point.y, point.z);

    geometry_msgs::msg::Point output;
    output.x = transformed.x();
    output.y = transformed.y();
    output.z = transformed.z();
    return output;
  }

  bool transform_marker_to_frame(
    const visualization_msgs::msg::Marker &marker,
    const std::string &target_frame,
    visualization_msgs::msg::Marker &transformed_marker)
  {
    const std::string source_frame = marker.header.frame_id;
    if (source_frame.empty())
    {
      return false;
    }

    transformed_marker = marker;
    if (source_frame == target_frame)
    {
      transformed_marker.header.frame_id = target_frame;
      return true;
    }

    try
    {
      const auto transform = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
      transformed_marker.header.frame_id = target_frame;
      transformed_marker.header.stamp = transform.header.stamp;

      if (marker.points.empty())
      {
        tf2::doTransform(marker.pose, transformed_marker.pose, transform);
      }
      else
      {
        transformed_marker.points.clear();
        transformed_marker.points.reserve(marker.points.size());
        for (const auto &point : marker.points)
        {
          transformed_marker.points.push_back(transform_point(point, transform));
        }
      }

      return true;
    }
    catch (const tf2::TransformException &exception)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Failed to transform marker from '%s' to '%s': %s",
        source_frame.c_str(),
        target_frame.c_str(),
        exception.what());
      return false;
    }
  }

  void publish_odom_marker_array(const visualization_msgs::msg::MarkerArray &markers)
  {
    if (!pub_marker_odom_ || markers.markers.empty())
    {
      return;
    }

    visualization_msgs::msg::MarkerArray odom_markers;
    odom_markers.markers.reserve(markers.markers.size());

    for (const auto &marker : markers.markers)
    {
      visualization_msgs::msg::Marker transformed_marker;
      if (transform_marker_to_frame(marker, odom_frame_, transformed_marker))
      {
        odom_markers.markers.push_back(transformed_marker);
      }
    }

    if (!odom_markers.markers.empty())
    {
      pub_marker_odom_->publish(odom_markers);
    }
  }

  void publish_marker_topics(const visualization_msgs::msg::MarkerArray &markers)
  {
    pub_marker_->publish(markers);
    publish_odom_marker_array(markers);
  }

  void initialize_interp_map_markers_if_needed(const std::string &frame_id, const rclcpp::Time &stamp)
  {
    if (interp_map_markers_initialized_)
    {
      interp_left_map_marker_.header.frame_id = frame_id;
      interp_left_map_marker_.header.stamp = stamp;
      interp_right_map_marker_.header.frame_id = frame_id;
      interp_right_map_marker_.header.stamp = stamp;
      return;
    }

    interp_left_map_marker_.header.frame_id = frame_id;
    interp_left_map_marker_.header.stamp = stamp;
    interp_left_map_marker_.ns = "parallel_left_interpolated_map";
    interp_left_map_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    interp_left_map_marker_.action = visualization_msgs::msg::Marker::ADD;
    interp_left_map_marker_.scale.x = 0.6;
    interp_left_map_marker_.color.r = 0.35;
    interp_left_map_marker_.color.g = 0.35;
    interp_left_map_marker_.color.b = 1.0;
    interp_left_map_marker_.color.a = 0.95;
    interp_left_map_marker_.id = 162;
    interp_left_map_marker_.pose.orientation.w = 1.0;

    interp_right_map_marker_.header.frame_id = frame_id;
    interp_right_map_marker_.header.stamp = stamp;
    interp_right_map_marker_.ns = "parallel_right_interpolated_map";
    interp_right_map_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    interp_right_map_marker_.action = visualization_msgs::msg::Marker::ADD;
    interp_right_map_marker_.scale.x = 0.6;
    interp_right_map_marker_.color.r = 0.35;
    interp_right_map_marker_.color.g = 0.95;
    interp_right_map_marker_.color.b = 0.35;
    interp_right_map_marker_.color.a = 0.95;
    interp_right_map_marker_.id = 163;
    interp_right_map_marker_.pose.orientation.w = 1.0;

    interp_map_markers_initialized_ = true;
  }

  void publish_interpolated_marker_map(
    const visualization_msgs::msg::Marker &left_interp_marker,
    const visualization_msgs::msg::Marker &right_interp_marker,
    const std::string &frame_id,
    const rclcpp::Time &stamp)
  {
    if (!pub_interp_marker_map_)
    {
      return;
    }

    initialize_interp_map_markers_if_needed(frame_id, stamp);

    const double interpolation_dedup_tolerance = 0.05;
    for (size_t i = 0; i + 1 < left_interp_marker.points.size(); i += 2)
    {
      add_unique_segment(
        interp_left_map_marker_,
        left_interp_marker.points[i],
        left_interp_marker.points[i + 1],
        interpolation_dedup_tolerance);
    }

    for (size_t i = 0; i + 1 < right_interp_marker.points.size(); i += 2)
    {
      add_unique_segment(
        interp_right_map_marker_,
        right_interp_marker.points[i],
        right_interp_marker.points[i + 1],
        interpolation_dedup_tolerance);
    }

    visualization_msgs::msg::MarkerArray map_markers;
    if (!interp_left_map_marker_.points.empty())
    {
      map_markers.markers.push_back(interp_left_map_marker_);
    }
    if (!interp_right_map_marker_.points.empty())
    {
      map_markers.markers.push_back(interp_right_map_marker_);
    }

    visualization_msgs::msg::Marker left_points_map_marker;
    left_points_map_marker.header.frame_id = frame_id;
    left_points_map_marker.header.stamp = stamp;
    left_points_map_marker.ns = "parallel_left_interpolated_map_points";
    left_points_map_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    left_points_map_marker.action = visualization_msgs::msg::Marker::ADD;
    left_points_map_marker.scale.x = 0.35;
    left_points_map_marker.scale.y = 0.35;
    left_points_map_marker.scale.z = 0.35;
    left_points_map_marker.color.r = 0.20;
    left_points_map_marker.color.g = 0.20;
    left_points_map_marker.color.b = 1.00;
    left_points_map_marker.color.a = 0.95;
    left_points_map_marker.id = 164;
    left_points_map_marker.pose.orientation.w = 1.0;
    left_points_map_marker.points = interp_left_global_pts_;

    visualization_msgs::msg::Marker right_points_map_marker;
    right_points_map_marker.header.frame_id = frame_id;
    right_points_map_marker.header.stamp = stamp;
    right_points_map_marker.ns = "parallel_right_interpolated_map_points";
    right_points_map_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    right_points_map_marker.action = visualization_msgs::msg::Marker::ADD;
    right_points_map_marker.scale.x = 0.35;
    right_points_map_marker.scale.y = 0.35;
    right_points_map_marker.scale.z = 0.35;
    right_points_map_marker.color.r = 0.20;
    right_points_map_marker.color.g = 1.00;
    right_points_map_marker.color.b = 0.20;
    right_points_map_marker.color.a = 0.95;
    right_points_map_marker.id = 165;
    right_points_map_marker.pose.orientation.w = 1.0;
    right_points_map_marker.points = interp_right_global_pts_;

    if (!left_points_map_marker.points.empty())
    {
      map_markers.markers.push_back(left_points_map_marker);
    }
    if (!right_points_map_marker.points.empty())
    {
      map_markers.markers.push_back(right_points_map_marker);
    }

    if (!map_markers.markers.empty())
    {
      pub_interp_marker_map_->publish(map_markers);
    }
  }

  void marker_callback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr input_msg)
  {
      RCLCPP_WARN_STREAM(this->get_logger(), "Frame: init_heading_left=" << init_heading_left_ * 180.0 / M_PI 
        << "° init_heading_right=" << init_heading_right_ * 180.0 / M_PI << "°");

      visualization_msgs::msg::MarkerArray markers;

      for (const auto& marker : input_msg->markers)
      {
        if (marker.ns == "cluster_center")
        {
          markers.markers.push_back(marker);
        }
   
      }

    visualization_msgs::msg::MarkerArray mark_array;

    visualization_msgs::msg::MarkerArray markers_left = crop_markers(markers, -8.0, -4.5, -0.001, 0.0);
    visualization_msgs::msg::MarkerArray markers_right = crop_markers(markers, -8.0, 0.0, -0.001, +4.5);   

    visualization_msgs::msg::MarkerArray markers_fwd = crop_markers(markers,-8.0, -1.5, -0.1, +1.5);     

    double ang_threshold = ang_threshold_deg * M_PI / 180.0;

    Cluster cluster1(std::vector<Point>(), cluster_num, eps_min, eps_max, ang_threshold_deg);
    std::vector<Point> candidate_points;

    for (const auto& marker : markers.markers)
    {
      Point point(marker.pose.position.x, marker.pose.position.y);
      point.x = marker.pose.position.x;
      point.y = marker.pose.position.y;
      
      candidate_points.push_back(point);
    }
    cluster1.candidate_points = candidate_points;

    //print out the size of the candidate points
    RCLCPP_INFO_STREAM(this->get_logger(), "Candidate points size MARKERS__: " << cluster1.get_candidate_size());
    RCLCPP_INFO_STREAM(this->get_logger(), "Markers points size, MARKERS_LEFT: " << markers_left.markers.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "Markers points size, MARKERS_RIGHT: " << markers_right.markers.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "Markers points size, MARKERS_FWD: " << markers_fwd.markers.size());
    

    float min_x = -10.0;
    for (const auto& marker : markers.markers)
    {
      if (marker.pose.position.x > min_x)
      {
        min_x = marker.pose.position.x;
      }
    }

     Point left_start(-50.0, -10.0);
     Point right_start(-50.0, +10.0);


    for (const auto& marker : markers_left.markers)
    {
      if (marker.pose.position.x > left_start.x) 
      {
        // Check if point is inside free space polygon
        if (latest_free_space_.points.size() >= 3 && 
            point_in_polygon(marker.pose.position.x, marker.pose.position.y, latest_free_space_.points))
        {
          left_start.y = marker.pose.position.y;
          left_start.x = marker.pose.position.x;
        }
      }
    }
    if (!markers_left.markers.empty())
    {
      cluster1.add_back(left_start, 1);
    }
    
    for (const auto& marker : markers_right.markers)
    {
      if (marker.pose.position.x > right_start.x) 
      {
        // Check if point is inside free space polygon
        if (latest_free_space_.points.size() >= 3 && 
            point_in_polygon(marker.pose.position.x, marker.pose.position.y, latest_free_space_.points))
        {
          right_start.y = marker.pose.position.y;
          right_start.x = marker.pose.position.x;
        }
      }
    }
  
    cluster1.add_back(right_start, 1);

    if (!markers.markers.empty())
    {
      cluster1.add_back(right_start, 2);
    }
    if (cluster1.get_size(1) >= 1)
    {
      // Evaluate all candidates and pick the best by smallest absolute angle difference
      if (cluster1.get_size(1) >= 1)
      {
        double best_score = std::numeric_limits<double>::infinity();
          Point best_p(0.0, 0.0);
        bool found = false;
        for (const auto& marker : markers.markers)
        {
          Point p(marker.pose.position.x, marker.pose.position.y);
          double dist = cluster1.distance(p, left_start);
          if (eps_min <= dist && dist <= eps_max)
          {
            double candidate_ang = cluster1.calculate_angle(p, left_start);
            double angle_difference = std::abs(cluster1.angle_diff(candidate_ang, init_heading_left_)); // use carried heading
            if (angle_difference < ang_threshold)
            {
              // Simple scoring: prefer smaller angle difference, break ties with smaller distance
              double score = calculate_score(angle_difference, dist);
              if (score < best_score)
              {
                best_score = score;
                best_p = p;
                found = true;
              }
            }
          }
        }
        if (found)
        {
          cluster1.add_back(best_p.x, best_p.y, 1);
        }
      }
    }
    // For right cluster: evaluate all candidates and pick best
    if (cluster1.get_size(2) >= 1)
    {
  double best_score_r = std::numeric_limits<double>::infinity();
  Point best_pr(0.0, 0.0);
      bool found_r = false;
      for (const auto& marker : markers.markers)
      {
        Point p(marker.pose.position.x, marker.pose.position.y);
        double dist = cluster1.distance(p, right_start);
        if (eps_min <= dist && dist <= eps_max)
        {
          double candidate_ang = cluster1.calculate_angle(p, right_start);
          double angle_difference = std::abs(cluster1.angle_diff(candidate_ang, init_heading_right_));
          if (angle_difference < ang_threshold)
          {
            double score = calculate_score(angle_difference, dist);
            if (score < best_score_r)
            {
              best_score_r = score;
              best_pr = p;
              found_r = true;
            }
          }
        }
      }
      if (found_r)
      {
        cluster1.add_back(best_pr.x, best_pr.y, 2);
      }
    }
     // Grow clusters in parallel/interleaved manner
    bool left_extending = true;
    bool right_extending = true;
    
    if (cluster1.get_size(1) < 2) left_extending = false;
    if (cluster1.get_size(2) < 2) right_extending = false;
    
    while (left_extending || right_extending)
    {
      if (left_extending)
      {
        left_extending = cluster1.next_tail(1);
      }
      if (right_extending)
      {
        right_extending = cluster1.next_tail(2);
      }
    }

    if (cluster1.get_size(1) >= 1)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "cluster1(1) size_MARKERS: " << cluster1.get_size(1));
    }
    if (cluster1.get_size(2) >= 1)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "cluster1(2) size_MARKERS: " << cluster1.get_size(2));
    }

    // Clean up crossings and spikes for marker-based clusters as well
    int points_removed_markers = remove_crossing_between_clusters(cluster1, 1, 2);
    //int spikes_removed_markers_l = remove_spikes(cluster1, 1, 150.0, eps_max);
    //int spikes_removed_markers_r = remove_spikes(cluster1, 2, 150.0, eps_max);
    // if (points_removed_markers + spikes_removed_markers_l + spikes_removed_markers_r > 0)
    // {
    //   RCLCPP_INFO_STREAM(this->get_logger(), "Marker-based removals: crossings=" << points_removed_markers <<
    //     ", spikes L=" << spikes_removed_markers_l << " R=" << spikes_removed_markers_r);
    // }

    // visualization_msgs::msg::Marker debug1_marker, debug2_marker, debug_text_marker;
    // init_debug_marker(debug1_marker, left_start.x, left_start.y, 1);
    // debug1_marker.header.frame_id = input_msg->header.frame_id;
    // debug1_marker.header.stamp = this->now();
    // init_debug_marker(debug2_marker, right_start.x, right_start.y, 2);
    // debug2_marker.header.frame_id = input_msg->header.frame_id;
    // debug2_marker.header.stamp = this->now();
    // init_text_debug_marker(debug_text_marker);
    // debug_text_marker.header.frame_id = input_msg->header.frame_id;
    // debug_text_marker.header.stamp = this->now();
    // debug_text_marker.text = std::to_string(tmp_angle_difference);

    visualization_msgs::msg::Marker cluster3_marker;
    cluster3_marker.header.frame_id = "laser_data_frame"; //markers.header.frame_id;
    cluster3_marker.header.stamp = this->now();
    cluster3_marker.ns = "cluster3";
    cluster3_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    cluster3_marker.action = visualization_msgs::msg::Marker::MODIFY;
    cluster3_marker.scale.x = 0.4;
    cluster3_marker.color.r = 0.0;
    cluster3_marker.color.g = 1.0;
    cluster3_marker.color.b = 0.0;
    cluster3_marker.color.a = 1.0;
    cluster3_marker.id = 3;
    cluster3_marker.pose.position.x = 0.0;
    cluster3_marker.pose.position.y = 0.0;
    cluster3_marker.pose.position.z = 0.0;
    cluster3_marker.points.clear();
    for (int i = 0; i < cluster1.get_size(1) - 1; i++)
    {
      geometry_msgs::msg::Point p;
      p.x = cluster1.get_cluster_point(1, i).x;
      p.y = cluster1.get_cluster_point(1, i).y;
      p.z = 0.0;
      cluster3_marker.points.push_back(p);
      p.x = cluster1.get_cluster_point(1, i + 1).x;
      p.y = cluster1.get_cluster_point(1, i + 1).y;
      p.z = 0.0;
      cluster3_marker.points.push_back(p);
    }

    visualization_msgs::msg::Marker cluster4_marker;
    cluster4_marker.header.frame_id ="laser_data_frame";                  //markers.header.frame_id;
    cluster4_marker.header.stamp = this->now();
    cluster4_marker.ns = "cluster4";
    cluster4_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    cluster4_marker.action = visualization_msgs::msg::Marker::MODIFY;
    cluster4_marker.scale.x = 0.4;
    cluster4_marker.color.r = 0.75;
    cluster4_marker.color.g = 0.25;
    cluster4_marker.color.b = 0.0;
    cluster4_marker.color.a = 1.0;
    cluster4_marker.id = 4;
    cluster4_marker.pose.position.x = 0.0;
    cluster4_marker.pose.position.y = 0.0;
    cluster4_marker.pose.position.z = 0.0;
    cluster4_marker.points.clear();
    for (int i = 0; i < cluster1.get_size(2) - 1; i++)
    {
      geometry_msgs::msg::Point p;
      p.x = cluster1.get_cluster_point(2, i).x;
      p.y = cluster1.get_cluster_point(2, i).y;
      p.z = 0.0;
      cluster4_marker.points.push_back(p);
      p.x = cluster1.get_cluster_point(2, i + 1).x;
      p.y = cluster1.get_cluster_point(2, i + 1).y;
      p.z = 0.0;
      cluster4_marker.points.push_back(p);
    }

   
    mark_array.markers.push_back(cluster3_marker);
    mark_array.markers.push_back(cluster4_marker);
    // Also add sphere markers for each cluster point so individual points are visible
    visualization_msgs::msg::Marker cluster3_points;
    cluster3_points.header.frame_id = "laser_data_frame"; // markers.header.frame_id;
    cluster3_points.header.stamp = this->now();
    cluster3_points.ns = "cluster3_points";
    cluster3_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    cluster3_points.action = visualization_msgs::msg::Marker::MODIFY;
  // make spheres larger for better visibility
    cluster3_points.scale.x = 0.3;
    cluster3_points.scale.y = 0.3;
    cluster3_points.scale.z = 0.3;
    cluster3_points.color.r = 0.0;
    cluster3_points.color.g = 1.0;
    cluster3_points.color.b = 0.0;
    cluster3_points.color.a = 1.0;
    cluster3_points.id = 30;
    cluster3_points.points.clear();
    for (int i = 0; i < cluster1.get_size(1); ++i)
    {
      geometry_msgs::msg::Point p;
      p.x = cluster1.get_cluster_point(1, i).x;
      p.y = cluster1.get_cluster_point(1, i).y;
      p.z = 0.0;
      cluster3_points.points.push_back(p);
    }

    visualization_msgs::msg::Marker cluster4_points;
    cluster4_points.header.frame_id = "laser_data_frame"; // markers.header.frame_id;
    cluster4_points.header.stamp = this->now();
    cluster4_points.ns = "cluster4_points";
    cluster4_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    cluster4_points.action = visualization_msgs::msg::Marker::MODIFY;
    cluster4_points.scale.x = 0.3;
    cluster4_points.scale.y = 0.3;
    cluster4_points.scale.z = 0.3;
    cluster4_points.color.r = 0.75;
    cluster4_points.color.g = 0.25;
    cluster4_points.color.b = 0.0;
    cluster4_points.color.a = 1.0;
    cluster4_points.id = 31;
    cluster4_points.points.clear();
    for (int i = 0; i < cluster1.get_size(2); ++i)
    {
      geometry_msgs::msg::Point p;
      p.x = cluster1.get_cluster_point(2, i).x;
      p.y = cluster1.get_cluster_point(2, i).y;
      p.z = 0.0;
      cluster4_points.points.push_back(p);
    }

    // Only push sphere markers if they contain points
    if (!cluster3_points.points.empty()) mark_array.markers.push_back(cluster3_points);
    if (!cluster4_points.points.empty()) mark_array.markers.push_back(cluster4_points);

    // Debug text: angles between consecutive points for cluster3
    if (cluster1.get_size(1) >= 2 && cluster3_points.points.size() > 0)
    {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1);
      for (int i = 0; i < cluster1.get_size(1) - 1; ++i)
      {
        Point a = cluster1.get_cluster_point(1, i);
        Point b = cluster1.get_cluster_point(1, i + 1);
        double ang = std::atan2(b.y - a.y, b.x - a.x) * 180.0 / M_PI;
        oss << ang;
  if (i < cluster1.get_size(1) - 2) oss << "\n";
      }
      visualization_msgs::msg::Marker angle_text;
      angle_text.header.frame_id = "laser_data_frame";
      angle_text.header.stamp = this->now();
      angle_text.ns = "angles_cluster3";
      angle_text.id = 40;
      angle_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      angle_text.action = visualization_msgs::msg::Marker::MODIFY;
      angle_text.pose.position.x = cluster1.get_cluster_point(1, 0).x;
      angle_text.pose.position.y = cluster1.get_cluster_point(1, 0).y;
      angle_text.pose.position.z = 1.0;
      angle_text.scale.z = 0.4; // text height
      angle_text.color.r = 1.0;
      angle_text.color.g = 1.0;
      angle_text.color.b = 1.0;
      angle_text.color.a = 1.0;
      angle_text.text = oss.str();
      mark_array.markers.push_back(angle_text);
    }

    // Debug text: angles between consecutive points for cluster4
    if (cluster1.get_size(2) >= 2 && cluster4_points.points.size() > 0)
    {
      std::ostringstream oss2;
      oss2 << std::fixed << std::setprecision(1);
      for (int i = 0; i < cluster1.get_size(2) - 1; ++i)
      {
        Point a = cluster1.get_cluster_point(2, i);
        Point b = cluster1.get_cluster_point(2, i + 1);
        double ang = std::atan2(b.y - a.y, b.x - a.x) * 180.0 / M_PI;
        oss2 << ang;
  if (i < cluster1.get_size(2) - 2) oss2 << "\n";
      }
      visualization_msgs::msg::Marker angle_text2;
      angle_text2.header.frame_id = "laser_data_frame";
      angle_text2.header.stamp = this->now();
      angle_text2.ns = "angles_cluster4";
      angle_text2.id = 41;
      angle_text2.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      angle_text2.action = visualization_msgs::msg::Marker::MODIFY;
      angle_text2.pose.position.x = cluster1.get_cluster_point(2, 0).x;
      angle_text2.pose.position.y = cluster1.get_cluster_point(2, 0).y;
      angle_text2.pose.position.z = 1.0;
      angle_text2.scale.z = 0.4;
      angle_text2.color.r = 1.0;
      angle_text2.color.g = 1.0;
      angle_text2.color.b = 1.0;
      angle_text2.color.a = 1.0;
      angle_text2.text = oss2.str();
      mark_array.markers.push_back(angle_text2);
    }

    latest_markers_ = mark_array;
    publish_marker_topics(mark_array);
  }
  
  
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "Frame: init_heading_left=" << init_heading_left_ * 180.0 / M_PI 
      << "° init_heading_right=" << init_heading_right_ * 180.0 / M_PI << "°");

    // Collect special segments to highlight (e.g., after trimming crossings)
    std::vector<std::pair<Point, Point>> red_segments_left;
    std::vector<std::pair<Point, Point>> red_segments_right;
    // Convert incoming local frame points, then append only new points into global map.
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_msg, *local_cloud);

    geometry_msgs::msg::TransformStamped sensor_to_global;
    try
    {
      sensor_to_global = tf_buffer_->lookupTransform(odom_frame_, input_msg->header.frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &exception)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Skipping lidar frame: failed transform '%s' -> '%s': %s",
        input_msg->header.frame_id.c_str(),
        odom_frame_.c_str(),
        exception.what());
      return;
    }

    tf2::Transform sensor_to_global_tf;
    tf2::fromMsg(sensor_to_global.transform, sensor_to_global_tf);

    pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    global_cloud->points.reserve(local_cloud->points.size());
    for (const auto &local_point : local_cloud->points)
    {
      const tf2::Vector3 transformed =
        sensor_to_global_tf * tf2::Vector3(local_point.x, local_point.y, local_point.z);
      pcl::PointXYZI global_point;
      global_point.x = transformed.x();
      global_point.y = transformed.y();
      global_point.z = transformed.z();
      global_point.intensity = local_point.intensity;
      global_cloud->points.push_back(global_point);
    }
    global_cloud->width = global_cloud->points.size();
    global_cloud->height = 1;
    global_cloud->is_dense = true;

    if (global_cloud->points.empty())
    {
      return;
    }

    const double vehicle_x = sensor_to_global.transform.translation.x;
    const double vehicle_y = sensor_to_global.transform.translation.y;
    tf2::Quaternion vehicle_orientation;
    tf2::fromMsg(sensor_to_global.transform.rotation, vehicle_orientation);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(vehicle_orientation).getRPY(roll, pitch, yaw);

    // Seed first search direction from current vehicle heading.
    init_heading_left_ = yaw;
    init_heading_right_ = yaw;

    auto to_global_xy = [&](double forward, double lateral, double &x_out, double &y_out)
    {
      const double cy = std::cos(yaw);
      const double sy = std::sin(yaw);
      x_out = vehicle_x + forward * cy - lateral * sy;
      y_out = vehicle_y + forward * sy + lateral * cy;
    };

    auto point_in_oriented_box = [&](const pcl::PointXYZI &point,
                                     double cx,
                                     double cy,
                                     double box_yaw,
                                     double half_x,
                                     double half_y)
    {
      const double dx = point.x - cx;
      const double dy = point.y - cy;
      const double cyaw = std::cos(box_yaw);
      const double syaw = std::sin(box_yaw);
      const double local_x = dx * cyaw + dy * syaw;
      const double local_y = -dx * syaw + dy * cyaw;
      return std::abs(local_x) <= half_x && std::abs(local_y) <= half_y;
    };

    const int original_size = global_cloud->width * global_cloud->height;

    const double box_center_forward = 0.5 * (minX + maxX);
    const double box_center_lateral = 0.5 * (minY + maxY);
    const double box_half_forward = 0.5 * (maxX - minX);
    const double box_half_lateral = 0.5 * (maxY - minY);
    double box_center_global_x = 0.0;
    double box_center_global_y = 0.0;
    to_global_xy(box_center_forward, box_center_lateral, box_center_global_x, box_center_global_y);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_box_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &point : global_cloud->points)
    {
      if (!point_in_oriented_box(point, box_center_global_x, box_center_global_y, yaw, box_half_forward, box_half_lateral))
      {
        continue;
      }
      if (point.z < minZ || point.z > maxZ)
      {
        continue;
      }
      cloud_box_filtered->points.push_back(point);
    }
    cloud_box_filtered->width = cloud_box_filtered->points.size();
    cloud_box_filtered->height = 1;
    cloud_box_filtered->is_dense = true;
    const int after_box_filter = cloud_box_filtered->width * cloud_box_filtered->height;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &point : cloud_box_filtered->points)
    {
      const double dx = point.x - vehicle_x;
      const double dy = point.y - vehicle_y;
      const double distance_from_vehicle = std::sqrt(dx * dx + dy * dy);
      if (distance_from_vehicle > origin_filter_radius)
      {
        cloud->points.push_back(point);
      }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
    const int after_origin_filter = cloud->width * cloud->height;

    const NongroundScatterStats nonground_scatter = compute_nonground_scatter_stats(cloud);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "nonground scatter (frame=%s n=%zu): mean=(%.3f, %.3f) std=(%.3f, %.3f) radial_std=%.3f m",
      odom_frame_.c_str(),
      nonground_scatter.count,
      nonground_scatter.mean_x,
      nonground_scatter.mean_y,
      nonground_scatter.std_x,
      nonground_scatter.std_y,
      nonground_scatter.radial_std);

    if (verbose1)
    {
      // print the length of the pointcloud
      RCLCPP_INFO_STREAM(this->get_logger(), "PointCloud in: " << original_size 
        << " after box filter: " << after_box_filter 
        << " after origin filter: " << after_origin_filter 
        << " (removed " << (after_box_filter - after_origin_filter) << " points around origin)");
      
      // Print all point coordinates
      RCLCPP_INFO_STREAM(this->get_logger(), "All points in cloud:");
      for (int i = 0; i < cloud->points.size(); i++) {
        RCLCPP_INFO_STREAM(this->get_logger(), "  Point " << i << ": x=" << cloud->points[i].x << " y=" << cloud->points[i].y << " z=" << cloud->points[i].z);
      }
    }

    // create marker array
    visualization_msgs::msg::MarkerArray mark_array;
    const std::string marker_frame = odom_frame_;

    double blue_left_global_x = 0.0;
    double blue_left_global_y = 0.0;
    to_global_xy(-4.0, -2.25, blue_left_global_x, blue_left_global_y);

    double amber_right_global_x = 0.0;
    double amber_right_global_y = 0.0;
    to_global_xy(-4.0, 2.25, amber_right_global_x, amber_right_global_y);

    visualization_msgs::msg::Marker blue_left;
    blue_left.header.frame_id = marker_frame;
    blue_left.header.stamp = this->now();
    blue_left.ns = "search_start";
    blue_left.type = visualization_msgs::msg::Marker::CUBE;
    blue_left.action = visualization_msgs::msg::Marker::MODIFY;
    blue_left.scale.x = 8.0;
    blue_left.scale.y = 4.5;

    //blue_left.scale.x = search_start_width_x;
    //blue_left.scale.y = search_start_width_y;
    blue_left.scale.z = 0.2;
    blue_left.color.r = md_blue_500_r;
    blue_left.color.g = md_blue_500_g;
    blue_left.color.b = md_blue_500_b;
    blue_left.color.a = 0.8;
    blue_left.id = 0;
    blue_left.pose.position.x = blue_left_global_x;
    blue_left.pose.position.y = blue_left_global_y;
    blue_left.pose.position.z = 0.0;
    {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      blue_left.pose.orientation = tf2::toMsg(q);
    }

    visualization_msgs::msg::Marker amber_right;
    amber_right.header.frame_id = marker_frame;
    amber_right.header.stamp = this->now();
    amber_right.ns = "search_start";
    amber_right.type = visualization_msgs::msg::Marker::CUBE;
    amber_right.action = visualization_msgs::msg::Marker::MODIFY;
    //amber_right.scale.x = search_start_width_x;
    //amber_right.scale.y = search_start_width_y;
    amber_right.scale.x = 8.0;
    amber_right.scale.y = 4.5;
    amber_right.scale.z = 0.2;
    amber_right.color.r = md_amber_500_r;
    amber_right.color.g = md_amber_500_g;
    amber_right.color.b = md_amber_500_b;
    amber_right.color.a = 0.8;
    amber_right.id = 1;
    amber_right.pose.position.x = amber_right_global_x;
    amber_right.pose.position.y = amber_right_global_y;
    amber_right.pose.position.z = 0.0;
    {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      amber_right.pose.orientation = tf2::toMsg(q);
    }

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_start(new pcl::PointCloud<pcl::PointXYZI>);
    // cloud_start = crop_pcl(cloud, -0.5 * search_start_width_x, -1.0 * search_start_width_y, +0.5 * search_start_width_x, +1.0 * search_start_width_y);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Start size: " << cloud_start->width * cloud_start->height);

    // test DBlane
    Cluster cluster1(std::vector<Point>(), cluster_num, eps_min, eps_max, ang_threshold_deg);
    double ang_threshold = ang_threshold_deg * M_PI / 180.0;
    std::vector<Point> candidate_points;
    for (pcl::PointXYZI p : cloud->points)
    {
      candidate_points.push_back(Point(p.x, p.y));
    }
    cluster1.candidate_points = candidate_points;

    // Build directional windows directly in global frame using oriented boxes.
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fwd(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZI>);

    double fwd_center_x = 0.0;
    double fwd_center_y = 0.0;
    to_global_xy(-4.05, 0.0, fwd_center_x, fwd_center_y);

    for (const auto &p : cloud->points)
    {
      if (point_in_oriented_box(p, fwd_center_x, fwd_center_y, yaw, 3.95, 1.5))
      {
        cloud_fwd->points.push_back(p);
      }
      if (point_in_oriented_box(p, blue_left_global_x, blue_left_global_y, yaw, 4.0, 2.25))
      {
        cloud_left->points.push_back(p);
      }
      if (point_in_oriented_box(p, amber_right_global_x, amber_right_global_y, yaw, 4.0, 2.25))
      {
        cloud_right->points.push_back(p);
      }
    }

    cloud_fwd->width = cloud_fwd->points.size();
    cloud_fwd->height = 1;
    cloud_fwd->is_dense = true;
    cloud_left->width = cloud_left->points.size();
    cloud_left->height = 1;
    cloud_left->is_dense = true;
    cloud_right->width = cloud_right->points.size();
    cloud_right->height = 1;
    cloud_right->is_dense = true;

    RCLCPP_INFO_STREAM(this->get_logger(), "crop_fwd: " << cloud_fwd->width * cloud_fwd->height);

    // get the smallest x value from cloud_fwd
    float min_x = -10.0;
    for (pcl::PointXYZI p : cloud_fwd->points)
    {
      if (p.x > min_x)
      {
        min_x = p.x;
      }
    }
    // get the largest y value from cloud_left
    Point left_start(-50.0, -10.0);
   
      RCLCPP_INFO_STREAM(this->get_logger(), "cloud_left: " << cloud_left->width * cloud_left->height);
      RCLCPP_INFO_STREAM(this->get_logger(), "cloud_right: " << cloud_right->width * cloud_right->height);
      
      // // Print all points in cloud_left
      // RCLCPP_INFO_STREAM(this->get_logger(), "Points in cloud_left:");
      // for (const auto& p : cloud_left->points) {
      //   RCLCPP_INFO_STREAM(this->get_logger(), "  x=" << p.x << " y=" << p.y << " z=" << p.z);
      // }
      // // Print all points in cloud_right
      // RCLCPP_INFO_STREAM(this->get_logger(), "Points in cloud_right:");
      // for (const auto& p : cloud_right->points) {
      //   RCLCPP_INFO_STREAM(this->get_logger(), "  x=" << p.x << " y=" << p.y << " z=" << p.z);
      // }
      
    // Pick closest point from cloud_left
    double min_dist_left = std::numeric_limits<double>::infinity();
    for (pcl::PointXYZI p : cloud_left->points)
    {
      const double dx = p.x - vehicle_x;
      const double dy = p.y - vehicle_y;
      const double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist_left)
      {
        min_dist_left = dist;
        left_start.x = p.x;
        left_start.y = p.y;
      }
    }
    if (cloud_left->width * cloud_left->height > 0.0) 
    {
    cluster1.add_back(left_start, 1);
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "left_start computed: " << left_start.x << ", " << left_start.y);
    // get the smallest y value from cloud_right
    Point right_start(-50.0, +10.0);
    // Pick closest point from cloud_right
    double min_dist_right = std::numeric_limits<double>::infinity();
    for (pcl::PointXYZI p : cloud_right->points)
    {
      const double dx = p.x - vehicle_x;
      const double dy = p.y - vehicle_y;
      const double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist_right)
      {
        min_dist_right = dist;
        right_start.x = p.x;
        right_start.y = p.y;
      }
    }
    if (cloud_right->width * cloud_right->height > 0.0)
    {
    cluster1.add_back(right_start, 2);
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "right_start computed: " << right_start.x << ", " << right_start.y);
    // Evaluate all LIDAR candidates for left cluster and pick the best
    if (cluster1.get_size(1) >= 1)
    {
      double best_score_l = std::numeric_limits<double>::infinity();
      Point best_pl(0.0, 0.0);
      bool found_l = false;
      
      for (pcl::PointXYZI pxyzi : cloud->points)
      {
        Point p(pxyzi.x, pxyzi.y);
        double dist = cluster1.distance(p, left_start);
        if (eps_min <= dist && dist <= eps_max)
        {
          double candidate_ang = cluster1.calculate_angle(p, left_start);
          double angle_difference = std::abs(cluster1.angle_diff(candidate_ang, init_heading_left_));
          
          if (angle_difference < ang_threshold)
          {
            double score = calculate_score(angle_difference, dist);
            if (score < best_score_l)
            {
              best_score_l = score;
              best_pl = p;
              found_l = true;
            }
          }
        }
      }
      if (found_l)
      {
        cluster1.add_back(best_pl.x, best_pl.y, 1);
      }
    }
    double tmp_angle_difference = -1.0;
    // Evaluate all LIDAR candidates for right cluster and pick the best
    if (cluster1.get_size(2) >= 1)
    {
      double best_score_r_lidar = std::numeric_limits<double>::infinity();
      Point best_pr_lidar(0.0, 0.0);
      bool found_r_lidar = false;
      
      for (pcl::PointXYZI pxyzi : cloud->points)
      {
        Point p(pxyzi.x, pxyzi.y);
        double dist = cluster1.distance(p, right_start);
        if (eps_min <= dist && dist <= eps_max)
        {
          double candidate_ang = cluster1.calculate_angle(p, right_start);
          double angle_difference = std::abs(cluster1.angle_diff(candidate_ang, init_heading_right_));
          
          if (angle_difference < ang_threshold)
          {
            double score = calculate_score(angle_difference, dist);
            if (score < best_score_r_lidar)
            {
              best_score_r_lidar = score;
              best_pr_lidar = p;
              found_r_lidar = true;
            }
          }
        }
      }
      if (found_r_lidar)
      {
        cluster1.add_back(best_pr_lidar.x, best_pr_lidar.y, 2);
      }
    }
    // Grow clusters in parallel/interleaved manner
    bool left_extending = true;
    bool right_extending = true;
    
    if (cluster1.get_size(1) < 2) left_extending = false;
    if (cluster1.get_size(2) < 2) right_extending = false;
    
    while (left_extending || right_extending)
    {
      if (left_extending)
      {
        left_extending = cluster1.next_tail(1);
      }
      if (right_extending)
      {
        right_extending = cluster1.next_tail(2);
      }
    }
      // Debug: print info for last point in right cluster and its candidates
      if (cluster1.get_size(2) >= 2) {
        Point last = cluster1.get_cluster_point(2, cluster1.get_size(2) - 1);
        Point prev = cluster1.get_cluster_point(2, cluster1.get_size(2) - 2);
        double tail_angle = cluster1.get_tail_angle(2);
        //RCLCPP_INFO_STREAM(this->get_logger(), "RIGHT cluster last point: x=" << last.x << " y=" << last.y 
        //  << " tail_angle=" << tail_angle * 180.0 / M_PI << "°");
        
        // Check potential next candidates for this last point
        //RCLCPP_INFO_STREAM(this->get_logger(), "  Potential next candidates for RIGHT cluster:");
        for (const Point& p : cluster1.candidate_points)
        {
          if (p.cluster_id == -1) // unassigned
          {
            double dist = cluster1.distance(last, p);
            if (eps_min <= dist && dist <= eps_max)
            {
              double candidate_ang = cluster1.calculate_angle(last, p);
              double angle_difference = std::abs(cluster1.angle_diff(candidate_ang, tail_angle));
              // RCLCPP_INFO_STREAM(this->get_logger(), "    x=" << p.x << " y=" << p.y 
              //   << " dist=" << dist << " angle=" << candidate_ang * 180.0 / M_PI 
              //   << "° rel_angle_diff=" << angle_difference * 180.0 / M_PI << "°");
            }
          }
        }
      }

    // Remove points that create crossings between left(1) and right(2) clusters
    //int points_removed = remove_crossing_between_clusters(cluster1, 1, 2);
    
    // New approach: Check which cluster is pointing toward the other cluster
    // For each segment, compute if it's oriented toward points in the other cluster
    
    auto check_pointing_toward = [&]() {
      if (cluster1.get_size(1) < 2 || cluster1.get_size(2) < 2) return;
    };
    
    check_pointing_toward();
    
    // Check for crossing detection for all cluster sizes
    bool crossing_detected = false;
    bool trimmed_cluster = false; // track if we trimmed to avoid regrowing
    int crossing_seg_left = -1, crossing_seg_right = -1;
    bool proximity_crossing = false; // flag for proximity-based crossing
    int proximity_left_idx = -1, proximity_right_idx = -1; // indices of too-close points

    // Check for crossings between all segments (geometric intersection)
    for (int i = 0; i < cluster1.get_size(1) - 1; ++i)
    {
      Point a1 = cluster1.get_cluster_point(1, i);
      Point a2 = cluster1.get_cluster_point(1, i + 1);
      for (int j = 0; j < cluster1.get_size(2) - 1; ++j)
      {
        Point b1 = cluster1.get_cluster_point(2, j);
        Point b2 = cluster1.get_cluster_point(2, j + 1);
        if (segments_intersect(a1, a2, b1, b2))
        {
          crossing_detected = true;
          crossing_seg_left = i;
          crossing_seg_right = j;
          break;
        }
      }
      if (crossing_detected) break;
    }

    // Check for proximity-based crossing (points within 0.2m from different clusters)
    // Only valid when at least one cluster is longer than 2 points (prevents 2-point clusters from trimming each other)
    const double proximity_threshold = 0.2;
    bool proximity_check_enabled = (cluster1.get_size(1) > 2 || cluster1.get_size(2) > 2);
    
    if (proximity_check_enabled)
    {
      for (int i = 0; i < cluster1.get_size(1); ++i)
      {
        Point left_point = cluster1.get_cluster_point(1, i);
        for (int j = 0; j < cluster1.get_size(2); ++j)
        {
          Point right_point = cluster1.get_cluster_point(2, j);
          double dist = cluster1.distance(left_point, right_point);
          if (dist <= proximity_threshold)
          {
            proximity_crossing = true;
            proximity_left_idx = i;
            proximity_right_idx = j;
            RCLCPP_WARN_STREAM(this->get_logger(), "Proximity crossing detected: LEFT[" << i << "] and RIGHT[" << j 
              << "] are " << dist << " m apart (threshold: " << proximity_threshold << " m)");
            break;
          }
        }
        if (proximity_crossing) break;
      }
    }

    // Combine both crossing types
    if (proximity_crossing)
    {
      crossing_detected = true;
      // For proximity crossing, use the indices as segment reference points
      crossing_seg_left = proximity_left_idx;
      crossing_seg_right = proximity_right_idx;
    }
    
    if (crossing_detected)
    {
      // Crossing found; keep logs minimal and defer repair reporting to adjustment step
      std::string crossing_type = proximity_crossing ? "PROXIMITY" : "GEOMETRIC";
      RCLCPP_INFO_STREAM(this->get_logger(), "Crossing detected (" << crossing_type << ") between LEFT segment " << crossing_seg_left << " and RIGHT segment " << crossing_seg_right);
      
      // For proximity crossings, ensure we don't go out of bounds
      int left_seg_end = proximity_crossing ? crossing_seg_left : (crossing_seg_left + 1);
      int right_seg_end = proximity_crossing ? crossing_seg_right : (crossing_seg_right + 1);
      
      // Clamp to valid indices
      left_seg_end = std::min(left_seg_end, cluster1.get_size(1) - 1);
      right_seg_end = std::min(right_seg_end, cluster1.get_size(2) - 1);
      
      Point left_p1 = cluster1.get_cluster_point(1, crossing_seg_left);
      Point left_p2 = (left_seg_end > crossing_seg_left) ? cluster1.get_cluster_point(1, left_seg_end) : left_p1;
      Point right_p1 = cluster1.get_cluster_point(2, crossing_seg_right);
      Point right_p2 = (right_seg_end > crossing_seg_right) ? cluster1.get_cluster_point(2, right_seg_end) : right_p1;
      
      double left_orientation = cluster1.calculate_angle(left_p1, left_p2) * 180.0 / M_PI;
      double right_orientation = cluster1.calculate_angle(right_p1, right_p2) * 180.0 / M_PI;
      double orientation_diff = std::abs(cluster1.angle_diff(
        cluster1.calculate_angle(left_p1, left_p2), 
        cluster1.calculate_angle(right_p1, right_p2))) * 180.0 / M_PI;
      
      // Orientation between segment starts and orientation deltas
      double left_to_right_orientation = cluster1.calculate_angle(left_p1, right_p1) * 180.0 / M_PI;
      double right_to_left_orientation = cluster1.calculate_angle(right_p1, left_p1) * 180.0 / M_PI;
      double left_before_diff = std::abs(cluster1.angle_diff(
        cluster1.calculate_angle(left_p1, left_p2),
        cluster1.calculate_angle(right_p1, left_p1))) * 180.0 / M_PI;
      double right_before_diff = std::abs(cluster1.angle_diff(
        cluster1.calculate_angle(right_p1, right_p2),
        cluster1.calculate_angle(left_p1, right_p1))) * 180.0 / M_PI;

      // Decide which side to report and log only that side; trimming will happen later if no adjustment found
      bool decision_left = left_before_diff > right_before_diff;
      if (decision_left)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Crossing decision: left");
      }
      else
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "Crossing decision: right");
      }

      // If a cluster has only one segment (2 points), try to pick a different second point that avoids crossing
      auto choose_alt_second = [&](int target_id, int other_id)
      {
        if (cluster1.get_size(target_id) != 2) return false; // only handle the simple 2-point cluster
        Point start = cluster1.get_cluster_point(target_id, 0);
        double expected_angle = cluster1.get_tail_angle(target_id); // current direction
        double best_score = std::numeric_limits<double>::infinity();
        Point best_p = cluster1.get_cluster_point(target_id, 1);
        Point old_second = best_p;
        bool found = false;

        for (const Point& cand : cluster1.candidate_points)
        {
          // Prefer unassigned points only to avoid reuse
          if (cand.cluster_id != -1) continue;
          // Enforce minimum separation from the old second point (0.5 m)
          if (cluster1.distance(cand, old_second) <= 0.5) continue;
          double dist = cluster1.distance(start, cand);
          if (dist < eps_min || dist > eps_max) continue;
          double ang = cluster1.calculate_angle(start, cand);
          double diff = std::abs(cluster1.angle_diff(ang, expected_angle));
          if (diff >= ang_threshold) continue;

          // Check if this segment would intersect any segment of the other cluster
          bool would_cross = false;
          for (int j = 0; j < cluster1.get_size(other_id) - 1; ++j)
          {
            Point o1 = cluster1.get_cluster_point(other_id, j);
            Point o2 = cluster1.get_cluster_point(other_id, j + 1);
            if (segments_intersect(start, cand, o1, o2))
            {
              would_cross = true;
              break;
            }
          }
          if (would_cross) continue;

          // Check if candidate is within 0.5 m of any point in the other cluster
          bool too_close_to_other = false;
          for (int j = 0; j < cluster1.get_size(other_id); ++j)
          {
            Point other_point = cluster1.get_cluster_point(other_id, j);
            if (cluster1.distance(cand, other_point) <= 0.5)
            {
              too_close_to_other = true;
              break;
            }
          }
          if (too_close_to_other) continue;

          double score = calculate_score(diff, dist);
          if (score < best_score)
          {
            best_score = score;
            best_p = cand;
            found = true;
          }
        }

        if (found)
        {
          double old_heading = (target_id == 1) ? init_heading_left_ : init_heading_right_;
          double new_heading = cluster1.calculate_angle(start, best_p);
          std::vector<Point> new_points = {start, best_p};
          cluster1.set_cluster_points(new_points, target_id);
          if (target_id == 1)
          {
            red_segments_left.push_back({start, best_p});
            init_heading_left_ = new_heading; // carry repaired heading forward
          }
          else if (target_id == 2)
          {
            red_segments_right.push_back({start, best_p});
            init_heading_right_ = new_heading; // carry repaired heading forward
          }
          // Single warning: report repair with old/new headings
          RCLCPP_WARN_STREAM(this->get_logger(), "Crossing repaired for cluster " << target_id
            << " old_heading=" << old_heading * 180.0 / M_PI << "° new_heading=" << new_heading * 180.0 / M_PI << "°");
          return true;
        }
        return false;
      };

      bool adjusted = false;
      // Prefer adjusting the cluster that has only one segment; if both, adjust right by default
      if (cluster1.get_size(1) == 2 && cluster1.get_size(2) > 2)
      {
        adjusted = choose_alt_second(1, 2);
      }
      else if (cluster1.get_size(2) == 2 && cluster1.get_size(1) > 2)
      {
        adjusted = choose_alt_second(2, 1);
      }
      else if (cluster1.get_size(1) == 2 && cluster1.get_size(2) == 2)
      {
        adjusted = choose_alt_second(2, 1);
        if (!adjusted) adjusted = choose_alt_second(1, 2);
      }

      if (adjusted)
      {
        // Re-run a quick crossing check after adjustment
        crossing_detected = false;
        crossing_seg_left = -1;
        crossing_seg_right = -1;
        for (int i = 0; i < cluster1.get_size(1) - 1; ++i)
        {
          Point a1 = cluster1.get_cluster_point(1, i);
          Point a2 = cluster1.get_cluster_point(1, i + 1);
          for (int j = 0; j < cluster1.get_size(2) - 1; ++j)
          {
            Point b1 = cluster1.get_cluster_point(2, j);
            Point b2 = cluster1.get_cluster_point(2, j + 1);
            if (segments_intersect(a1, a2, b1, b2))
            {
              crossing_detected = true;
              crossing_seg_left = i;
              crossing_seg_right = j;
              break;
            }
          }
          if (crossing_detected) break;
        }

      }

      // If no alternative point could fix the crossing, trim the decided side now
      if (!adjusted)
      {
        if (decision_left)
        {
          // For proximity crossing, keep points before the offending point
          int keep_points = proximity_crossing ? crossing_seg_left : (crossing_seg_left + 1);
          if (cluster1.get_size(1) > keep_points)
          {
            std::vector<Point> kept;
            kept.insert(kept.end(), cluster1.cluster_points[1].begin(), cluster1.cluster_points[1].begin() + keep_points);
            cluster1.set_cluster_points(kept, 1);
            std::string reason = proximity_crossing ? "proximity violation" : "crossing segment";
            RCLCPP_WARN_STREAM(this->get_logger(), "  Trimmed LEFT cluster to " << keep_points << " point(s) to remove " << reason << " and followers");
            trimmed_cluster = true;
          }
        }
        else
        {
          // For proximity crossing, keep points before the offending point
          int keep_points = proximity_crossing ? crossing_seg_right : (crossing_seg_right + 1);
          if (cluster1.get_size(2) > keep_points)
          {
            std::vector<Point> kept;
            kept.insert(kept.end(), cluster1.cluster_points[2].begin(), cluster1.cluster_points[2].begin() + keep_points);
            cluster1.set_cluster_points(kept, 2);
            std::string reason = proximity_crossing ? "proximity violation" : "crossing segment";
            RCLCPP_WARN_STREAM(this->get_logger(), "  Trimmed RIGHT cluster to " << keep_points << " point(s) to remove " << reason << " and followers");
            trimmed_cluster = true;
          }
        }
      }

      // After trimming, try to find an alternative second point to avoid future crossings
      if (trimmed_cluster && cluster1.get_size(1) == 2)
      {
        // Trimmed left cluster - try to find alternative second point
        auto try_alt_second_after_trim = [&](int target_id, int other_id)
        {
          if (cluster1.get_size(target_id) != 2) return false;
          Point start = cluster1.get_cluster_point(target_id, 0);
          Point old_second = cluster1.get_cluster_point(target_id, 1);
          double best_score = std::numeric_limits<double>::infinity();
          Point best_p = old_second;
          bool found = false;

          for (const Point& cand : cluster1.candidate_points)
          {
            // Prefer unassigned points
            if (cand.cluster_id != -1) continue;
            // Don't reuse the old second point
            if (cluster1.distance(cand, old_second) <= 0.1) continue;
            // Check distance constraints
            double dist = cluster1.distance(start, cand);
            if (dist < eps_min || dist > eps_max) continue;
            
            // Check if this segment would intersect any segment of the other cluster
            bool would_cross = false;
            for (int j = 0; j < cluster1.get_size(other_id) - 1; ++j)
            {
              Point o1 = cluster1.get_cluster_point(other_id, j);
              Point o2 = cluster1.get_cluster_point(other_id, j + 1);
              if (segments_intersect(start, cand, o1, o2))
              {
                would_cross = true;
                break;
              }
            }
            if (would_cross) continue;

            // Check proximity to other cluster
            bool too_close_to_other = false;
            for (int j = 0; j < cluster1.get_size(other_id); ++j)
            {
              Point other_point = cluster1.get_cluster_point(other_id, j);
              if (cluster1.distance(cand, other_point) <= 0.5)
              {
                too_close_to_other = true;
                break;
              }
            }
            if (too_close_to_other) continue;

            // Prefer points aligned with start-first_point direction
            Point first_point = cluster1.get_cluster_point(target_id, 1);
            double original_angle = cluster1.calculate_angle(start, first_point);
            double candidate_angle = cluster1.calculate_angle(start, cand);
            double angle_diff = std::abs(cluster1.angle_diff(candidate_angle, original_angle));
            
            double score = calculate_score(angle_diff, dist);
            if (score < best_score)
            {
              best_score = score;
              best_p = cand;
              found = true;
            }
          }

          if (found && cluster1.distance(best_p, old_second) > 0.1)
          {
            double old_heading = (target_id == 1) ? init_heading_left_ : init_heading_right_;
            double new_heading = cluster1.calculate_angle(start, best_p);
            std::vector<Point> new_points = {start, best_p};
            cluster1.set_cluster_points(new_points, target_id);
            if (target_id == 1)
            {
              red_segments_left.push_back({start, best_p});
              init_heading_left_ = new_heading;
            }
            else if (target_id == 2)
            {
              red_segments_right.push_back({start, best_p});
              init_heading_right_ = new_heading;
            }
            RCLCPP_WARN_STREAM(this->get_logger(), "After trim: alternative second point found for cluster " << target_id
              << " old_heading=" << old_heading * 180.0 / M_PI << "° new_heading=" << new_heading * 180.0 / M_PI << "°");
            return true;
          }
          return false;
        };

        if (decision_left)
        {
          try_alt_second_after_trim(1, 2);
        }
        else
        {
          try_alt_second_after_trim(2, 1);
        }
      }
      else if (trimmed_cluster && cluster1.get_size(2) == 2)
      {
        // Trimmed right cluster - try alternative second point
        auto try_alt_second_after_trim = [&](int target_id, int other_id)
        {
          if (cluster1.get_size(target_id) != 2) return false;
          Point start = cluster1.get_cluster_point(target_id, 0);
          Point old_second = cluster1.get_cluster_point(target_id, 1);
          double best_score = std::numeric_limits<double>::infinity();
          Point best_p = old_second;
          bool found = false;

          for (const Point& cand : cluster1.candidate_points)
          {
            if (cand.cluster_id != -1) continue;
            if (cluster1.distance(cand, old_second) <= 0.1) continue;
            double dist = cluster1.distance(start, cand);
            if (dist < eps_min || dist > eps_max) continue;
            
            bool would_cross = false;
            for (int j = 0; j < cluster1.get_size(other_id) - 1; ++j)
            {
              Point o1 = cluster1.get_cluster_point(other_id, j);
              Point o2 = cluster1.get_cluster_point(other_id, j + 1);
              if (segments_intersect(start, cand, o1, o2))
              {
                would_cross = true;
                break;
              }
            }
            if (would_cross) continue;

            bool too_close_to_other = false;
            for (int j = 0; j < cluster1.get_size(other_id); ++j)
            {
              Point other_point = cluster1.get_cluster_point(other_id, j);
              if (cluster1.distance(cand, other_point) <= 0.5)
              {
                too_close_to_other = true;
                break;
              }
            }
            if (too_close_to_other) continue;

            Point first_point = cluster1.get_cluster_point(target_id, 1);
            double original_angle = cluster1.calculate_angle(start, first_point);
            double candidate_angle = cluster1.calculate_angle(start, cand);
            double angle_diff = std::abs(cluster1.angle_diff(candidate_angle, original_angle));
            
            double score = calculate_score(angle_diff, dist);
            if (score < best_score)
            {
              best_score = score;
              best_p = cand;
              found = true;
            }
          }

          if (found && cluster1.distance(best_p, old_second) > 0.1)
          {
            double old_heading = (target_id == 1) ? init_heading_left_ : init_heading_right_;
            double new_heading = cluster1.calculate_angle(start, best_p);
            std::vector<Point> new_points = {start, best_p};
            cluster1.set_cluster_points(new_points, target_id);
            if (target_id == 1)
            {
              red_segments_left.push_back({start, best_p});
              init_heading_left_ = new_heading;
            }
            else if (target_id == 2)
            {
              red_segments_right.push_back({start, best_p});
              init_heading_right_ = new_heading;
            }
            RCLCPP_WARN_STREAM(this->get_logger(), "After trim: alternative second point found for cluster " << target_id
              << " old_heading=" << old_heading * 180.0 / M_PI << "° new_heading=" << new_heading * 180.0 / M_PI << "°");
            return true;
          }
          return false;
        };

        if (!decision_left)
        {
          try_alt_second_after_trim(2, 1);
        }
      }
    }
    else
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "No crossings detected");
    }
    
    // Regrow clusters only if we did not trim due to crossing
    if (!trimmed_cluster)
    {
      bool regrowing = true;
      int regrow_iteration = 0;
      
      while (regrowing && regrow_iteration < 100) // Safety limit for regrow iterations
      {
        regrow_iteration++;
        regrowing = false; // Will be set to true if any cluster grows or crossing is fixed
        
        // Try to grow left cluster
        if (cluster1.get_size(1) >= 2)
        {
          bool extended = cluster1.next_tail(1);
          if (extended)
          {
            regrowing = true;
          }
        }
        
        // Try to grow right cluster
        if (cluster1.get_size(2) >= 2)
        {
          bool extended = cluster1.next_tail(2);
          if (extended)
          {
            regrowing = true;
          }
        }
        
        // Temporary: crossing checks during regrowth remain disabled
      }
    }
    
    // Temporary: Skip crossing check after regrowth (disabled)
    //int spikes_removed_l = remove_spikes(cluster1, 1, 150.0, eps_max);
    //int spikes_removed_r = remove_spikes(cluster1, 2, 150.0, eps_max);
    // if (points_removed + spikes_removed_l + spikes_removed_r > 0)
    // {
    //   RCLCPP_INFO_STREAM(this->get_logger(), "Crossing removals: " << points_removed << ", spikes removed L:" << spikes_removed_l << " R:" << spikes_removed_r);
    // }
    if (cluster1.get_size(1) >= 1)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "cluster1(left) size: " << cluster1.get_size(1));
    }
    if (cluster1.get_size(2) >= 1)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "cluster1(right) size: " << cluster1.get_size(2));
    }

    visualization_msgs::msg::Marker left_start_marker, right_start_marker;
    init_debug_marker(left_start_marker, left_start.x, left_start.y, 1);
    left_start_marker.header.frame_id = marker_frame;
    left_start_marker.header.stamp = this->now();
    left_start_marker.ns = "start_left_point";
    init_debug_marker(right_start_marker, right_start.x, right_start.y, 2);
    right_start_marker.header.frame_id = marker_frame;
    right_start_marker.header.stamp = this->now();
    right_start_marker.ns = "start_right_point";
    // init_text_debug_marker(debug_text_marker);
    // debug_text_marker.header.frame_id = input_msg->header.frame_id;
    // debug_text_marker.header.stamp = this->now();
    // debug_text_marker.text = std::to_string(tmp_angle_difference);

    visualization_msgs::msg::Marker cluster1_marker;
    cluster1_marker.header.frame_id = marker_frame;
    cluster1_marker.header.stamp = this->now();
    cluster1_marker.ns = "cluster1";
    cluster1_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    cluster1_marker.action = visualization_msgs::msg::Marker::MODIFY;
    cluster1_marker.scale.x = 0.4;
    cluster1_marker.color.r = md_blue_500_r;
    cluster1_marker.color.g = md_blue_500_g;
    cluster1_marker.color.b = md_blue_500_b;
    cluster1_marker.color.a = 1.0;
    cluster1_marker.id = 2;
    cluster1_marker.pose.position.x = 0.0;
    cluster1_marker.pose.position.y = 0.0;
    cluster1_marker.pose.position.z = 0.0;
    cluster1_marker.points.clear();
    for (int i = 0; i < cluster1.get_size(1) - 1; i++)
    {
      geometry_msgs::msg::Point p;
      p.x = cluster1.get_cluster_point(1, i).x;
      p.y = cluster1.get_cluster_point(1, i).y;
      p.z = 0.0;
      cluster1_marker.points.push_back(p);
      p.x = cluster1.get_cluster_point(1, i + 1).x;
      p.y = cluster1.get_cluster_point(1, i + 1).y;
      p.z = 0.0;
      cluster1_marker.points.push_back(p);
    }

    visualization_msgs::msg::Marker cluster2_marker;
    cluster2_marker.header.frame_id = marker_frame;
    cluster2_marker.header.stamp = this->now();
    cluster2_marker.ns = "cluster2";
    cluster2_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    cluster2_marker.action = visualization_msgs::msg::Marker::MODIFY;
    cluster2_marker.scale.x = 0.4;
    cluster2_marker.color.r = md_amber_500_r;
    cluster2_marker.color.g = md_amber_500_g;
    cluster2_marker.color.b = md_amber_500_b;
    cluster2_marker.color.a = 1.0;
    cluster2_marker.id = 2;
    cluster2_marker.pose.position.x = 0.0;
    cluster2_marker.pose.position.y = 0.0;
    cluster2_marker.pose.position.z = 0.0;
    cluster2_marker.points.clear();
    for (int i = 0; i < cluster1.get_size(2) - 1; i++)
    {
      geometry_msgs::msg::Point p;
      p.x = cluster1.get_cluster_point(2, i).x;
      p.y = cluster1.get_cluster_point(2, i).y;
      p.z = 0.0;
      cluster2_marker.points.push_back(p);
      p.x = cluster1.get_cluster_point(2, i + 1).x;
      p.y = cluster1.get_cluster_point(2, i + 1).y;
      p.z = 0.0;
      cluster2_marker.points.push_back(p);
    }

    mark_array.markers.push_back(blue_left);
    mark_array.markers.push_back(amber_right);
    mark_array.markers.push_back(cluster1_marker);
    mark_array.markers.push_back(cluster2_marker);
    
    // Add red segments for trimmed areas (crossing resolution)
    if (!red_segments_left.empty()) {
      visualization_msgs::msg::Marker red_left;
      red_left.header.frame_id = marker_frame;
      red_left.header.stamp = this->now();
      red_left.ns = "trimmed_left";
      red_left.type = visualization_msgs::msg::Marker::LINE_LIST;
      red_left.action = visualization_msgs::msg::Marker::MODIFY;
      red_left.scale.x = 0.5;
      red_left.color.r = 1.0;
      red_left.color.g = 0.0;
      red_left.color.b = 0.0;
      red_left.color.a = 1.0;
      red_left.id = 200;
      for (auto seg : red_segments_left) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = seg.first.x; p1.y = seg.first.y; p1.z = 0.0;
        p2.x = seg.second.x; p2.y = seg.second.y; p2.z = 0.0;
        red_left.points.push_back(p1);
        red_left.points.push_back(p2);
      }
      mark_array.markers.push_back(red_left);
    }
    if (!red_segments_right.empty()) {
      visualization_msgs::msg::Marker red_right;
      red_right.header.frame_id = marker_frame;
      red_right.header.stamp = this->now();
      red_right.ns = "trimmed_right";
      red_right.type = visualization_msgs::msg::Marker::LINE_LIST;
      red_right.action = visualization_msgs::msg::Marker::MODIFY;
      red_right.scale.x = 0.5;
      red_right.color.r = 1.0;
      red_right.color.g = 0.0;
      red_right.color.b = 0.0;
      red_right.color.a = 1.0;
      red_right.id = 201;
      for (auto seg : red_segments_right) {
        geometry_msgs::msg::Point p1, p2;
        p1.x = seg.first.x; p1.y = seg.first.y; p1.z = 0.0;
        p2.x = seg.second.x; p2.y = seg.second.y; p2.z = 0.0;
        red_right.points.push_back(p1);
        red_right.points.push_back(p2);
      }
      mark_array.markers.push_back(red_right);
    }
    
    // --- PÁRHUZAMOS SZEGMENSEK DETEKTÁLÁSA ---
    // ±10 fok tolerancia párhuzamosság ellenőrzéshez
    const double parallel_angle_tolerance = 10.0 * M_PI / 180.0;
    
    // Szegmens info struktúra
    struct SegmentInfo {
      geometry_msgs::msg::Point p1, p2;
      double angle;
      int index;
    };
    
    std::vector<SegmentInfo> left_segments, right_segments;
    
    // Bal oldali (cluster1) szegmensek gyűjtése
    for (int i = 0; i < cluster1.get_size(1) - 1; i++) {
      SegmentInfo seg;
      seg.p1.x = cluster1.get_cluster_point(1, i).x;
      seg.p1.y = cluster1.get_cluster_point(1, i).y;
      seg.p1.z = 0.0;
      seg.p2.x = cluster1.get_cluster_point(1, i + 1).x;
      seg.p2.y = cluster1.get_cluster_point(1, i + 1).y;
      seg.p2.z = 0.0;
      seg.angle = std::atan2(seg.p2.y - seg.p1.y, seg.p2.x - seg.p1.x);
      seg.index = i;
      left_segments.push_back(seg);
    }
    
    // Jobb oldali (cluster2) szegmensek gyűjtése
    for (int i = 0; i < cluster1.get_size(2) - 1; i++) {
      SegmentInfo seg;
      seg.p1.x = cluster1.get_cluster_point(2, i).x;
      seg.p1.y = cluster1.get_cluster_point(2, i).y;
      seg.p1.z = 0.0;
      seg.p2.x = cluster1.get_cluster_point(2, i + 1).x;
      seg.p2.y = cluster1.get_cluster_point(2, i + 1).y;
      seg.p2.z = 0.0;
      seg.angle = std::atan2(seg.p2.y - seg.p1.y, seg.p2.x - seg.p1.x);
      seg.index = i;
      right_segments.push_back(seg);
    }
    
    // Párhuzamos szegmensek markerek
    visualization_msgs::msg::Marker parallel_left_marker;
    parallel_left_marker.header.frame_id = marker_frame;
    parallel_left_marker.header.stamp = this->now();
    parallel_left_marker.ns = "parallel_left";
    parallel_left_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    parallel_left_marker.action = visualization_msgs::msg::Marker::MODIFY;
    parallel_left_marker.scale.x = 0.6;
    parallel_left_marker.color.r = 1.0;
    parallel_left_marker.color.g = 0.0;
    parallel_left_marker.color.b = 1.0;  // Magenta
    parallel_left_marker.color.a = 1.0;
    parallel_left_marker.id = 60;
    parallel_left_marker.pose.position.x = 0.0;
    parallel_left_marker.pose.position.y = 0.0;
    parallel_left_marker.pose.position.z = 0.0;
    parallel_left_marker.points.clear();
    
    visualization_msgs::msg::Marker parallel_right_marker;
    parallel_right_marker.header.frame_id = marker_frame;
    parallel_right_marker.header.stamp = this->now();
    parallel_right_marker.ns = "parallel_right";
    parallel_right_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    parallel_right_marker.action = visualization_msgs::msg::Marker::MODIFY;
    parallel_right_marker.scale.x = 0.6;
    parallel_right_marker.color.r = 0.0;
    parallel_right_marker.color.g = 1.0;
    parallel_right_marker.color.b = 1.0;  // Cián
    parallel_right_marker.color.a = 1.0;
    parallel_right_marker.id = 61;
    parallel_right_marker.pose.position.x = 0.0;
    parallel_right_marker.pose.position.y = 0.0;
    parallel_right_marker.pose.position.z = 0.0;
    parallel_right_marker.points.clear();
    
    std::vector<bool> right_used(right_segments.size(), false);
    std::vector<int> matched_left_indices;
    std::vector<int> matched_right_indices;
    
    for (size_t i = 0; i < left_segments.size(); ++i) {
      for (size_t j = 0; j < right_segments.size(); ++j) {
        if (right_used[j]) continue;
        
        double angle_diff = std::abs(left_segments[i].angle - right_segments[j].angle);
        // Kezelés a 180 fokos esetre (ellentétes irányú, de párhuzamos vonalak)
        double effective_diff = std::min(angle_diff, std::abs(M_PI - angle_diff));
        
        if (effective_diff < parallel_angle_tolerance) {
          // Párhuzamos szegmenspár találat
          parallel_left_marker.points.push_back(left_segments[i].p1);
          parallel_left_marker.points.push_back(left_segments[i].p2);
          parallel_right_marker.points.push_back(right_segments[j].p1);
          parallel_right_marker.points.push_back(right_segments[j].p2);
          matched_left_indices.push_back(i);
          matched_right_indices.push_back(j);
          right_used[j] = true;
          break;  // Következő bal oldali szegmenshez lépünk
        }
      }
    }
    
    // Create interpolated segment markers
    visualization_msgs::msg::Marker parallel_left_interpolated_marker;
    parallel_left_interpolated_marker.header.frame_id = marker_frame;
    parallel_left_interpolated_marker.header.stamp = this->now();
    parallel_left_interpolated_marker.ns = "parallel_left_interpolated";
    parallel_left_interpolated_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    parallel_left_interpolated_marker.action = visualization_msgs::msg::Marker::MODIFY;
    parallel_left_interpolated_marker.scale.x = 0.6;
    parallel_left_interpolated_marker.color.r = 0.5;  // Different from magenta (1.0, 0.0, 1.0)
    parallel_left_interpolated_marker.color.g = 0.5;
    parallel_left_interpolated_marker.color.b = 1.0;
    parallel_left_interpolated_marker.color.a = 0.6;  // Slightly transparent
    parallel_left_interpolated_marker.id = 62;
    parallel_left_interpolated_marker.pose.position.x = 0.0;
    parallel_left_interpolated_marker.pose.position.y = 0.0;
    parallel_left_interpolated_marker.pose.position.z = 0.0;
    parallel_left_interpolated_marker.points.clear();

    visualization_msgs::msg::Marker parallel_right_interpolated_marker;
    parallel_right_interpolated_marker.header.frame_id = marker_frame;
    parallel_right_interpolated_marker.header.stamp = this->now();
    parallel_right_interpolated_marker.ns = "parallel_right_interpolated";
    parallel_right_interpolated_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    parallel_right_interpolated_marker.action = visualization_msgs::msg::Marker::MODIFY;
    parallel_right_interpolated_marker.scale.x = 0.6;
    parallel_right_interpolated_marker.color.r = 0.5;  // Different from cyan (0.0, 1.0, 1.0)
    parallel_right_interpolated_marker.color.g = 0.8;
    parallel_right_interpolated_marker.color.b = 0.5;
    parallel_right_interpolated_marker.color.a = 0.6;  // Slightly transparent
    parallel_right_interpolated_marker.id = 63;
    parallel_right_interpolated_marker.pose.position.x = 0.0;
    parallel_right_interpolated_marker.pose.position.y = 0.0;
    parallel_right_interpolated_marker.pose.position.z = 0.0;
    parallel_right_interpolated_marker.points.clear();
    
    const double interpolation_dedup_tolerance = 0.05;

    // Add matched segments to interpolated markers
    for (size_t idx : matched_left_indices) {
      add_unique_segment(
        parallel_left_interpolated_marker,
        left_segments[idx].p1,
        left_segments[idx].p2,
        interpolation_dedup_tolerance);
    }
    for (size_t idx : matched_right_indices) {
      add_unique_segment(
        parallel_right_interpolated_marker,
        right_segments[idx].p1,
        right_segments[idx].p2,
        interpolation_dedup_tolerance);
    }
    
    // Interpolate gaps in left cluster matched segments
    if (!matched_left_indices.empty()) {
      int min_left_idx = *std::min_element(matched_left_indices.begin(), matched_left_indices.end());
      int max_left_idx = *std::max_element(matched_left_indices.begin(), matched_left_indices.end());
      
      // Rebuild interpolated marker with sequential ordering for continuity
      parallel_left_interpolated_marker.points.clear();
      
      for (int i = min_left_idx; i <= max_left_idx; ++i) {
        bool is_matched = std::find(matched_left_indices.begin(), matched_left_indices.end(), i) != matched_left_indices.end();
        if (is_matched && i >= 0 && i < (int)left_segments.size()) {
          // Add matched segment
          add_unique_segment(
            parallel_left_interpolated_marker,
            left_segments[i].p1,
            left_segments[i].p2,
            interpolation_dedup_tolerance);
        } else if (!is_matched && i >= 0 && i < (int)left_segments.size()) {
          // Find adjacent matched segments for interpolation
          int before_idx = -1, after_idx = -1;
          for (int m : matched_left_indices) {
            if (m < i && m > before_idx) before_idx = m;
            if (m > i && (after_idx == -1 || m < after_idx)) after_idx = m;
          }
          
          if (before_idx != -1 && after_idx != -1) {
            // Interpolate between segments
            geometry_msgs::msg::Point p1, p2;
            p1.x = left_segments[before_idx].p2.x + (left_segments[after_idx].p1.x - left_segments[before_idx].p2.x) * (i - before_idx) / (after_idx - before_idx);
            p1.y = left_segments[before_idx].p2.y + (left_segments[after_idx].p1.y - left_segments[before_idx].p2.y) * (i - before_idx) / (after_idx - before_idx);
            p1.z = 0.0;
            
            p2.x = left_segments[before_idx].p2.x + (left_segments[after_idx].p1.x - left_segments[before_idx].p2.x) * (i + 1 - before_idx) / (after_idx - before_idx);
            p2.y = left_segments[before_idx].p2.y + (left_segments[after_idx].p1.y - left_segments[before_idx].p2.y) * (i + 1 - before_idx) / (after_idx - before_idx);
            p2.z = 0.0;
            
            add_unique_segment(
              parallel_left_interpolated_marker,
              p1,
              p2,
              interpolation_dedup_tolerance);
          }
        }
      }
    }
    
    // Interpolate gaps in right cluster matched segments
    if (!matched_right_indices.empty()) {
      int min_right_idx = *std::min_element(matched_right_indices.begin(), matched_right_indices.end());
      int max_right_idx = *std::max_element(matched_right_indices.begin(), matched_right_indices.end());
      
      // Rebuild interpolated marker with sequential ordering for continuity
      parallel_right_interpolated_marker.points.clear();
      
      for (int i = min_right_idx; i <= max_right_idx; ++i) {
        bool is_matched = std::find(matched_right_indices.begin(), matched_right_indices.end(), i) != matched_right_indices.end();
        if (is_matched && i >= 0 && i < (int)right_segments.size()) {
          // Add matched segment
          add_unique_segment(
            parallel_right_interpolated_marker,
            right_segments[i].p1,
            right_segments[i].p2,
            interpolation_dedup_tolerance);
        } else if (!is_matched && i >= 0 && i < (int)right_segments.size()) {
          // Find adjacent matched segments for interpolation
          int before_idx = -1, after_idx = -1;
          for (int m : matched_right_indices) {
            if (m < i && m > before_idx) before_idx = m;
            if (m > i && (after_idx == -1 || m < after_idx)) after_idx = m;
          }
          
          if (before_idx != -1 && after_idx != -1) {
            // Interpolate between segments
            geometry_msgs::msg::Point p1, p2;
            p1.x = right_segments[before_idx].p2.x + (right_segments[after_idx].p1.x - right_segments[before_idx].p2.x) * (i - before_idx) / (after_idx - before_idx);
            p1.y = right_segments[before_idx].p2.y + (right_segments[after_idx].p1.y - right_segments[before_idx].p2.y) * (i - before_idx) / (after_idx - before_idx);
            p1.z = 0.0;
            
            p2.x = right_segments[before_idx].p2.x + (right_segments[after_idx].p1.x - right_segments[before_idx].p2.x) * (i + 1 - before_idx) / (after_idx - before_idx);
            p2.y = right_segments[before_idx].p2.y + (right_segments[after_idx].p1.y - right_segments[before_idx].p2.y) * (i + 1 - before_idx) / (after_idx - before_idx);
            p2.z = 0.0;
            
            add_unique_segment(
              parallel_right_interpolated_marker,
              p1,
              p2,
              interpolation_dedup_tolerance);
          }
        }
      }
    }
    
    // Párhuzamos markerek hozzáadása, ha vannak találatok
    if (!parallel_left_marker.points.empty()) {
      mark_array.markers.push_back(parallel_left_marker);
    }
    if (!parallel_right_marker.points.empty()) {
      mark_array.markers.push_back(parallel_right_marker);
    }
    
    // Add interpolated markers (which contain both matched and interpolated segments)
    if (!parallel_left_interpolated_marker.points.empty()) {
      mark_array.markers.push_back(parallel_left_interpolated_marker);
    }
    if (!parallel_right_interpolated_marker.points.empty()) {
      mark_array.markers.push_back(parallel_right_interpolated_marker);
    }
    
    mark_array.markers.push_back(left_start_marker);
    mark_array.markers.push_back(right_start_marker);
    // Also add sphere markers for cluster points so individual points are visible along with lines
    visualization_msgs::msg::Marker cluster1_points_marker;
    cluster1_points_marker.header.frame_id = marker_frame;
    cluster1_points_marker.header.stamp = this->now();
    cluster1_points_marker.ns = "cluster1_points";
    cluster1_points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    cluster1_points_marker.action = visualization_msgs::msg::Marker::MODIFY;
    // larger spheres for visibility
    cluster1_points_marker.scale.x = 1.0; // diameter of each sphere
    cluster1_points_marker.scale.y = 1.0;
    cluster1_points_marker.scale.z = 1.0;
    cluster1_points_marker.color.r = md_blue_500_r;
    cluster1_points_marker.color.g = md_blue_500_g;
    cluster1_points_marker.color.b = md_blue_500_b;
    cluster1_points_marker.color.a = 1.0;
    cluster1_points_marker.id = 20;
    cluster1_points_marker.points.clear();
    for (int i = 0; i < cluster1.get_size(1); ++i)
    {
      geometry_msgs::msg::Point p;
      p.x = cluster1.get_cluster_point(1, i).x;
      p.y = cluster1.get_cluster_point(1, i).y;
      p.z = 0.0;
      cluster1_points_marker.points.push_back(p);
    }

    visualization_msgs::msg::Marker cluster2_points_marker;
    cluster2_points_marker.header.frame_id = marker_frame;
    cluster2_points_marker.header.stamp = this->now();
    cluster2_points_marker.ns = "cluster2_points";
    cluster2_points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    cluster2_points_marker.action = visualization_msgs::msg::Marker::MODIFY;
    cluster2_points_marker.scale.x = 1.0;
    cluster2_points_marker.scale.y = 1.0;
    cluster2_points_marker.scale.z = 1.0;
    cluster2_points_marker.color.r = md_amber_500_r;
    cluster2_points_marker.color.g = md_amber_500_g;
    cluster2_points_marker.color.b = md_amber_500_b;
    cluster2_points_marker.color.a = 1.0;
    cluster2_points_marker.id = 21;
    cluster2_points_marker.points.clear();
    for (int i = 0; i < cluster1.get_size(2); ++i)
    {
      geometry_msgs::msg::Point p;
      p.x = cluster1.get_cluster_point(2, i).x;
      p.y = cluster1.get_cluster_point(2, i).y;
      p.z = 0.0;
      cluster2_points_marker.points.push_back(p);
    }

    if (!cluster1_points_marker.points.empty()) mark_array.markers.push_back(cluster1_points_marker);
    if (!cluster2_points_marker.points.empty()) mark_array.markers.push_back(cluster2_points_marker);

    // Debug text: angles between consecutive points for cluster1
    if (cluster1.get_size(1) >= 2 && cluster1_marker.points.size() > 0)
    {
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(1);
      for (int i = 0; i < cluster1.get_size(1) - 1; ++i)
      {
        Point a = cluster1.get_cluster_point(1, i);
        Point b = cluster1.get_cluster_point(1, i + 1);
        double ang = std::atan2(b.y - a.y, b.x - a.x) * 180.0 / M_PI;
        oss << ang;
  if (i < cluster1.get_size(1) - 2) oss << "\n";
      }
      visualization_msgs::msg::Marker angle_text1;
      angle_text1.header.frame_id = marker_frame;
      angle_text1.header.stamp = this->now();
      angle_text1.ns = "angles_cluster1";
      angle_text1.id = 50;
      angle_text1.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      angle_text1.action = visualization_msgs::msg::Marker::MODIFY;
      angle_text1.pose.position.x = cluster1.get_cluster_point(1, 0).x;
      angle_text1.pose.position.y = cluster1.get_cluster_point(1, 0).y;
      angle_text1.pose.position.z = 1.0;
      angle_text1.scale.z = 0.4;
      angle_text1.color.r = 1.0;
      angle_text1.color.g = 1.0;
      angle_text1.color.b = 1.0;
      angle_text1.color.a = 1.0;
      angle_text1.text = oss.str();
      mark_array.markers.push_back(angle_text1);
    }

    // Debug text: angles between consecutive points for cluster2
    if (cluster1.get_size(2) >= 2 && cluster2_marker.points.size() > 0)
    {
      std::ostringstream oss2;
      oss2 << std::fixed << std::setprecision(1);
      for (int i = 0; i < cluster1.get_size(2) - 1; ++i)
      {
        Point a = cluster1.get_cluster_point(2, i);
        Point b = cluster1.get_cluster_point(2, i + 1);
        double ang = std::atan2(b.y - a.y, b.x - a.x) * 180.0 / M_PI;
        oss2 << ang;
  if (i < cluster1.get_size(2) - 2) oss2 << "\n";
      }
      visualization_msgs::msg::Marker angle_text2;
      angle_text2.header.frame_id = marker_frame;
      angle_text2.header.stamp = this->now();
      angle_text2.ns = "angles_cluster2";
      angle_text2.id = 51;
      angle_text2.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      angle_text2.action = visualization_msgs::msg::Marker::MODIFY;
      angle_text2.pose.position.x = cluster1.get_cluster_point(2, 0).x;
      angle_text2.pose.position.y = cluster1.get_cluster_point(2, 0).y;
      angle_text2.pose.position.z = 1.0;
      angle_text2.scale.z = 0.4;
      angle_text2.color.r = 1.0;
      angle_text2.color.g = 1.0;
      angle_text2.color.b = 1.0;
      angle_text2.color.a = 1.0;
      angle_text2.text = oss2.str();
      mark_array.markers.push_back(angle_text2);
    }

    publish_marker_topics(mark_array);

    // --- Collect unique interpolated lane points into global accumulators ---
    const double imerge_sq = interp_point_merge_radius_ * interp_point_merge_radius_;

    auto add_interp_point = [&](std::vector<geometry_msgs::msg::Point> &acc,
                                const geometry_msgs::msg::Point &pt)
    {
      for (const auto &existing : acc)
      {
        const double dx = existing.x - pt.x;
        const double dy = existing.y - pt.y;
        if ((dx * dx + dy * dy) <= imerge_sq)
        {
          return; // already have a nearby point
        }
      }
      acc.push_back(pt);
    };

    // Extract all unique endpoints from the interpolated LEFT marker
    for (const auto &pt : parallel_left_interpolated_marker.points)
    {
      add_interp_point(interp_left_global_pts_, pt);
    }
    // Extract all unique endpoints from the interpolated RIGHT marker
    for (const auto &pt : parallel_right_interpolated_marker.points)
    {
      add_interp_point(interp_right_global_pts_, pt);
    }

    publish_interpolated_marker_map(
      parallel_left_interpolated_marker,
      parallel_right_interpolated_marker,
      marker_frame,
      this->now());

    // Build and publish a combined PointCloud2 (left=intensity 0, right=intensity 1)
    pcl::PointCloud<pcl::PointXYZI> interp_cloud;
    for (const auto &pt : interp_left_global_pts_)
    {
      pcl::PointXYZI p;
      p.x = pt.x; p.y = pt.y; p.z = 0.0f; p.intensity = 0.0f;
      interp_cloud.points.push_back(p);
    }
    for (const auto &pt : interp_right_global_pts_)
    {
      pcl::PointXYZI p;
      p.x = pt.x; p.y = pt.y; p.z = 0.0f; p.intensity = 1.0f;
      interp_cloud.points.push_back(p);
    }
    interp_cloud.width = interp_cloud.points.size();
    interp_cloud.height = 1;
    interp_cloud.is_dense = true;
    if (!interp_cloud.points.empty())
    {
      sensor_msgs::msg::PointCloud2 interp_msg;
      pcl::toROSMsg(interp_cloud, interp_msg);
      interp_msg.header.frame_id = marker_frame;
      interp_msg.header.stamp = this->now();
      pub_interp_points_->publish(interp_msg);
    }

    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    // Add the same frame_id as the input, it is not included in pcl PointXYZI
    output_msg.header.frame_id = marker_frame;
    // Publish the data as a ROS message
    pub_lidar_->publish(output_msg);
  }

  // add callback functions for vehicle speed and steering angle if needed in future
  void vehicle_speed_callback(const std_msgs::msg::Float32::ConstSharedPtr speed_msg)
  {
    current_speed_ = speed_msg->data;
    if (verbose2)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Vehicle speed: " << current_speed_);
    }   
  }

  void steering_angle_callback(const std_msgs::msg::Float32::ConstSharedPtr steering_msg)
  {
    current_steering_angle_ = steering_msg->data;
    if (verbose2)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Steering angle: " << current_steering_angle_);
    }   
  }

  void free_space_callback(const visualization_msgs::msg::Marker::ConstSharedPtr free_space_msg)
  {
    latest_free_space_ = *free_space_msg;
    if (verbose2)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Received free space marker with " << free_space_msg->points.size() << " points");
    }
  }

  void free_space_convex_callback(const visualization_msgs::msg::Marker::ConstSharedPtr free_space_convex_msg)
  {
    latest_free_space_convex_ = *free_space_convex_msg;
    if (verbose2)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Received free space convex marker with " << free_space_convex_msg->points.size() << " points");
    }
  }

  // Motion update callback at 20 Hz
  void update_motion()
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_update_time_).seconds();
    
    // Update marker positions based on vehicle motion
    visualization_msgs::msg::MarkerArray::SharedPtr mark_array_ptr = 
      std::make_shared<visualization_msgs::msg::MarkerArray>();
    
    // Clear any existing markers
    mark_array_ptr->markers.clear();
    
    // Update markers using the latest vehicle state
    for (const auto& marker : latest_markers_.markers)
    {
      visualization_msgs::msg::Marker updated_marker = marker;
      
      // Only update position for our tracked markers
      if (marker.ns == "cluster3" || marker.ns == "cluster4" ||
          marker.ns == "cluster3_points" || marker.ns == "cluster4_points")
      {
        if (marker.type == visualization_msgs::msg::Marker::LINE_LIST)
        {
          // Update line list points
          for (size_t i = 0; i < marker.points.size(); i += 2)
          {
            Point p1(marker.points[i].x, marker.points[i].y);
            Point p2(marker.points[i + 1].x, marker.points[i + 1].y);
            
            Point pred1 = predict_point_motion(p1, dt);
            Point pred2 = predict_point_motion(p2, dt);
            
            updated_marker.points[i].x = pred1.x;
            updated_marker.points[i].y = pred1.y;
            updated_marker.points[i + 1].x = pred2.x;
            updated_marker.points[i + 1].y = pred2.y;
          }
        }
        else if (marker.type == visualization_msgs::msg::Marker::SPHERE_LIST)
        {
          // Update sphere list points
          for (size_t i = 0; i < marker.points.size(); ++i)
          {
            Point p(marker.points[i].x, marker.points[i].y);
            Point pred = predict_point_motion(p, dt);
            updated_marker.points[i].x = pred.x;
            updated_marker.points[i].y = pred.y;
          }
        }
      }
      
      mark_array_ptr->markers.push_back(updated_marker);
    }
    
    // Publish updated markers
    if (!mark_array_ptr->markers.empty())
    {
      latest_markers_ = *mark_array_ptr;
      publish_marker_topics(*mark_array_ptr);
    }
    
    last_update_time_ = current_time;
  }

  // Helper function to predict point position based on vehicle motion
  Point predict_point_motion(const Point& p, double dt)
  {
    // Simple bicycle model for prediction
    // Assuming vehicle's origin at rear axle
    const double wheelbase = 1.7;  // meters - adjust based on your vehicle
    
    if (std::abs(current_speed_) < 0.1) {
      return p;  // No movement if speed is negligible
    }

    // Calculate turning radius (positive for left turn, negative for right turn)
    double turn_radius;
    if (std::abs(current_steering_angle_) < 0.001) {
      // Straight line motion
      Point predicted(
        p.x + current_speed_ * dt,  // Move forward in x
        p.y  // y position unchanged
      );
      return predicted;
    } else {
      // Invert steering convention: what used to be left turn becomes right turn.
      turn_radius = wheelbase / std::tan(-current_steering_angle_);
      
      // Convert point to vehicle-relative coordinates (assuming vehicle at origin)
      double px_rel = p.x;
      double py_rel = p.y;
      
      // Calculate angular velocity
      double angular_vel = current_speed_ / turn_radius;
      double angle = angular_vel * dt;
      
      // Rotate point around center of rotation
      double cos_angle = std::cos(angle);
      double sin_angle = std::sin(angle);
      
      Point predicted(
        px_rel * cos_angle - py_rel * sin_angle + current_speed_ * dt,
        px_rel * sin_angle + py_rel * cos_angle
      );
      
      return predicted;
    }
  }

  // Helper function to update cluster points based on vehicle motion
  void update_cluster_points_with_motion(Cluster& cluster, double dt)
  {
    for (int cluster_id = 1; cluster_id <= 2; ++cluster_id) {
      std::vector<Point> updated_points;
      for (int i = 0; i < cluster.get_size(cluster_id); ++i) {
        Point current = cluster.get_cluster_point(cluster_id, i);
        Point predicted = predict_point_motion(current, dt);
        updated_points.push_back(predicted);
      }
      cluster.set_cluster_points(updated_points, cluster_id);
    }
  }

  // Helper: check if a point is inside a polygon using ray casting algorithm
  bool point_in_polygon(double x, double y, const std::vector<geometry_msgs::msg::Point>& polygon)
  {
    if (polygon.size() < 3) return false;  // Need at least 3 points for a polygon
    
    int intersections = 0;
    size_t n = polygon.size();
    
    for (size_t i = 0; i < n - 1; ++i) {
      double x1 = polygon[i].x;
      double y1 = polygon[i].y;
      double x2 = polygon[i + 1].x;
      double y2 = polygon[i + 1].y;
      
      // Check if horizontal ray from point intersects edge
      if (((y1 <= y && y < y2) || (y2 <= y && y < y1))) {
        double x_intersect = x1 + (y - y1) * (x2 - x1) / (y2 - y1);
        if (x < x_intersect) {
          intersections++;
        }
      }
    }
    
    return (intersections % 2) == 1;  // Odd number of intersections means inside
  }

  // Helper: check if two line segments (p1,q1) and (p2,q2) intersect
  static int orientation(const Point &p, const Point &q, const Point &r)
  {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (std::abs(val) < 1e-9) return 0; // colinear
    return (val > 0) ? 1 : 2; // 1: clockwise, 2: counterclockwise
  }

  static bool on_segment(const Point &p, const Point &q, const Point &r)
  {
    return (q.x <= std::max(p.x, r.x) + 1e-9 && q.x + 1e-9 >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) + 1e-9 && q.y + 1e-9 >= std::min(p.y, r.y));
  }

  static bool segments_intersect(const Point &p1, const Point &q1, const Point &p2, const Point &q2)
  {
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    if (o1 != o2 && o3 != o4) return true;
    // Special Cases
    if (o1 == 0 && on_segment(p1, p2, q1)) return true;
    if (o2 == 0 && on_segment(p1, q2, q1)) return true;
    if (o3 == 0 && on_segment(p2, p1, q2)) return true;
    if (o4 == 0 && on_segment(p2, q1, q2)) return true;
    return false;
  }

  // Remove points from clusters that create crossings between cluster idA and idB.
  // When a crossing between segment (A[i],A[i+1]) and (B[j],B[j+1]) is detected
  // the later point of each segment (A[i+1] and B[j+1]) is removed.
  // Returns total number of points removed.
  int remove_crossing_between_clusters(Cluster &cluster, int idA, int idB)
  {
    int total_removed = 0;
    bool points_removed;
    int iteration = 0;

    do {
      points_removed = false;
      int nA = cluster.get_size(idA);
      int nB = cluster.get_size(idB);
      if (nA < 2 || nB < 2) break;

      std::vector<char> keepA(nA, 1);
      std::vector<char> keepB(nB, 1);
      // Always keep first point
      keepA[0] = 1;
      keepB[0] = 1;

      int removed_this_iter = 0;
      for (int i = 0; i < nA - 1; ++i)
      {
        Point a1 = cluster.get_cluster_point(idA, i);
        Point a2 = cluster.get_cluster_point(idA, i + 1);
        for (int j = 0; j < nB - 1; ++j)
        {
          Point b1 = cluster.get_cluster_point(idB, j);
          Point b2 = cluster.get_cluster_point(idB, j + 1);
          if (segments_intersect(a1, a2, b1, b2))
          {
            // mark the later points for removal (i+1 and j+1)
            if (keepA[i + 1]) {
              keepA[i + 1] = 0;
              removed_this_iter++;
              points_removed = true;
            }
            if (keepB[j + 1]) {
              keepB[j + 1] = 0;
              removed_this_iter++;
              points_removed = true;
            }
          }
        }
      }

      // Rebuild cluster points for A
      std::vector<Point> newA;
      for (int i = 0; i < nA; ++i)
      {
        if (keepA[i]) newA.push_back(cluster.get_cluster_point(idA, i));
      }
      if (!newA.empty()) cluster.set_cluster_points(newA, idA);

      // Rebuild cluster points for B
      std::vector<Point> newB;
      for (int i = 0; i < nB; ++i)
      {
        if (keepB[i]) newB.push_back(cluster.get_cluster_point(idB, i));
      }
      if (!newB.empty()) cluster.set_cluster_points(newB, idB);

      total_removed += removed_this_iter;
      if (removed_this_iter > 0) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("dblane_f1s"), 
          "Iteration " << iteration << ": Removed " << removed_this_iter << 
          " points (" << (nA - newA.size()) << " from cluster " << idA << 
          ", " << (nB - newB.size()) << " from cluster " << idB << ")");
      }
      iteration++;
    } while (points_removed);

    if (total_removed > 0) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("dblane_f1s"), 
        "Total points removed: " << total_removed << " in " << iteration << " iterations");
    }
    return total_removed;
  }

  // Remove 'spike' points from a single cluster that stick out compared to neighbors.
  // A point is removed if the turning angle at that point is larger than angle_thresh_deg
  // OR its perpendicular distance to the line between its neighbors is greater than dist_thresh.
  // Returns number of points removed.
  // int remove_spikes(Cluster &cluster, int id, double angle_thresh_deg = 150.0, double dist_thresh = -1.0)
  // {
  //   int total_removed = 0;
  //   bool removed_any = false;
  //   int iter = 0;
  //   if (dist_thresh < 0) dist_thresh = eps_max; // default to eps_max if not provided

  //   do {
  //     removed_any = false;
  //     int n = cluster.get_size(id);
  //     if (n < 3) break; // need at least 3 points to consider a middle spike
  //     std::vector<char> keep(n, 1);
  //     int removed_this_iter = 0;

  //     for (int i = 1; i < n - 1; ++i)
  //     {
  //       Point prev = cluster.get_cluster_point(id, i - 1);
  //       Point cur = cluster.get_cluster_point(id, i);
  //       Point next = cluster.get_cluster_point(id, i + 1);

  //       // vectors
  //       double v1x = cur.x - prev.x;
  //       double v1y = cur.y - prev.y;
  //       double v2x = next.x - cur.x;
  //       double v2y = next.y - cur.y;
  //       double norm1 = std::sqrt(v1x * v1x + v1y * v1y);
  //       double norm2 = std::sqrt(v2x * v2x + v2y * v2y);
  //       if (norm1 < 1e-6 || norm2 < 1e-6) continue; // degenerate, skip

  //       // angle between v1 and v2
  //       double dot = (v1x * v2x + v1y * v2y) / (norm1 * norm2);
  //       dot = std::max(-1.0, std::min(1.0, dot));
  //       double ang_rad = std::acos(dot);
  //       double ang_deg = ang_rad * 180.0 / M_PI;

  //       // perpendicular distance from cur to line(prev,next)
  //       double lx = next.x - prev.x;
  //       double ly = next.y - prev.y;
  //       double lnorm = std::sqrt(lx * lx + ly * ly);
  //       double perp_dist = 0.0;
  //       if (lnorm > 1e-6) {
  //         double area2 = std::abs((cur.x - prev.x) * (next.y - prev.y) - (cur.y - prev.y) * (next.x - prev.x));
  //         perp_dist = area2 / lnorm;
  //       }

  //       if (ang_deg > angle_thresh_deg || perp_dist > dist_thresh)
  //       {
  //         keep[i] = 0;
  //         removed_this_iter++;
  //         removed_any = true;
  //       }
  //     }

  //     if (removed_this_iter > 0)
  //     {
  //       // rebuild
  //       std::vector<Point> newPts;
  //       for (int i = 0; i < n; ++i) if (keep[i]) newPts.push_back(cluster.get_cluster_point(id, i));
  //       if (!newPts.empty()) cluster.set_cluster_points(newPts, id);
  //       total_removed += removed_this_iter;
  //       RCLCPP_INFO_STREAM(this->get_logger(), "remove_spikes cluster " << id << " iteration " << iter << ": removed " << removed_this_iter << " points");
  //     }
  //     iter++;
  //   } while (removed_any);

  //   if (total_removed > 0) RCLCPP_INFO_STREAM(this->get_logger(), "remove_spikes cluster " << id << ": total removed " << total_removed << " in " << iter << " iterations");
  //   return total_removed;
  // }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_odom_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_marker_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_vehicle_speed_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_steering_angle_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_free_space_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_free_space_convex_;

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  
  // Store latest markers for motion updates
  visualization_msgs::msg::MarkerArray latest_markers_;
  visualization_msgs::msg::Marker latest_free_space_;
  visualization_msgs::msg::Marker latest_free_space_convex_;
  float minX = -80.0, minY = -25.0, minZ = -2.0;
  float maxX = +80.0, maxY = +25.0, maxZ = -0.15;
  bool verbose1 = false, verbose2 = false;
  float search_start_width_x = 8.0, search_start_width_y = 3.5;
  std::string points_in_topic, points_out_topic, marker_out_topic;
  int cluster_num = 5;
  float eps_min = 1.2, eps_max = 3.4, ang_threshold_deg = 30.0;
  double origin_filter_radius = 0.25;
  double global_point_merge_radius_ = 0.15;
  std::vector<Point> global_points_;
  double init_heading_left_ = 0.0;  // carried initial heading for left cluster
  double init_heading_right_ = 0.0; // carried initial heading for right cluster

  // Accumulated unique interpolated lane points (odom frame)
  std::vector<geometry_msgs::msg::Point> interp_left_global_pts_;
  std::vector<geometry_msgs::msg::Point> interp_right_global_pts_;
  double interp_point_merge_radius_ = 0.2;
  std::string interp_points_out_topic_ = "interpolated_lane_points";
  std::string interp_marker_map_out_topic_ = "interpolated_marker_map_odom";
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_interp_points_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_interp_marker_map_;
  visualization_msgs::msg::Marker interp_left_map_marker_;
  visualization_msgs::msg::Marker interp_right_map_marker_;
  bool interp_map_markers_initialized_ = false;
  
  // Motion tracking variables
  float current_speed_ = 0.0;  // m/s
  float current_steering_angle_ = 0.0;  // radians
  rclcpp::Time last_update_time_;
  rclcpp::TimerBase::SharedPtr motion_update_timer_;
  bool first_update_ = true;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string marker_odom_out_topic_ = "clustered_marker_odom";
  std::string odom_frame_ = "odom";
  
  // colors from https://github.com/jkk-research/colors
  const float md_amber_500_r = 1.00, md_amber_500_g = 0.76, md_amber_500_b = 0.03;
  const float md_blue_500_r = 0.13, md_blue_500_g = 0.59, md_blue_500_b = 0.95;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DblaneFormula>());
  rclcpp::shutdown();
  return 0;
}