// DBlane filter for point cloud data
// formula 1 student version

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
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
    this->declare_parameter<bool>("verbose1", verbose1);
    this->declare_parameter<bool>("verbose2", verbose2);
    this->declare_parameter<float>("search_start_width_x", search_start_width_x);
    this->declare_parameter<float>("search_start_width_y", search_start_width_y);
    this->declare_parameter<int>("cluster_num", cluster_num);
    this->declare_parameter<float>("eps_min", eps_min);
    this->declare_parameter<float>("eps_max", eps_max);
    this->declare_parameter<float>("ang_threshold_deg", ang_threshold_deg);
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
    this->get_parameter("search_start_width_x", search_start_width_x);
    this->get_parameter("search_start_width_y", search_start_width_y);
    this->get_parameter("cluster_num", cluster_num);
    this->get_parameter("eps_min", eps_min);
    this->get_parameter("eps_max", eps_max);
    this->get_parameter("ang_threshold_deg", ang_threshold_deg);

    pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic, 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_out_topic, 10);
    // TODO: QoS // rclcpp::SensorDataQoS().keep_last(1)
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_in_topic, 10, std::bind(&DblaneFormula::lidar_callback, this, std::placeholders::_1));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DblaneFormula::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "DblaneFormula node has been started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: '%s'", points_in_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: '%s' and '%s'", points_out_topic.c_str(), marker_out_topic.c_str());
  }

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr crop_pcl(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, double min_x_, double min_y_, double max_x_, double max_y_)
  {
    pcl::CropBox<pcl::PointXYZI> crop_fwd;
    crop_fwd.setInputCloud(cloud_in);
    crop_fwd.setMin(Eigen::Vector4f(min_x_, min_y_, -2.0, 1.0));
    crop_fwd.setMax(Eigen::Vector4f(max_x_, max_y_, -0.1, 1.0));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZI>);
    crop_fwd.filter(*cloud_cropped);
    // RCLCPP_INFO_STREAM(this->get_logger(), "crop_fwd: " << cloud_cropped->width * cloud_cropped->height);
    return cloud_cropped;
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_msg, *cloud);
    int original_size = cloud->width * cloud->height;

    // Filter out points outside of the box
    pcl::CropBox<pcl::PointXYZI> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    crop.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    crop.filter(*cloud);

    if (verbose1)
    {
      // print the length of the pointcloud
      RCLCPP_INFO_STREAM(this->get_logger(), "PointCloud in: " << original_size << " reduced size before cluster: " << cloud->width * cloud->height);
    }

    // create marker array
    visualization_msgs::msg::MarkerArray mark_array;
    visualization_msgs::msg::Marker blue_left;
    blue_left.header.frame_id = input_msg->header.frame_id;
    blue_left.header.stamp = this->now();
    blue_left.ns = "search_start";
    blue_left.type = visualization_msgs::msg::Marker::CUBE;
    blue_left.action = visualization_msgs::msg::Marker::MODIFY;
    blue_left.scale.x = search_start_width_x;
    blue_left.scale.y = search_start_width_y;
    blue_left.scale.z = 0.2;
    blue_left.color.r = md_blue_500_r;
    blue_left.color.g = md_blue_500_g;
    blue_left.color.b = md_blue_500_b;
    blue_left.color.a = 0.8;
    blue_left.id = 0;
    blue_left.pose.position.x = 0.0;
    blue_left.pose.position.y = -0.5 * search_start_width_y - 0.1;
    blue_left.pose.position.z = 0.0;

    visualization_msgs::msg::Marker amber_right;
    amber_right.header.frame_id = input_msg->header.frame_id;
    amber_right.header.stamp = this->now();
    amber_right.ns = "search_start";
    amber_right.type = visualization_msgs::msg::Marker::CUBE;
    amber_right.action = visualization_msgs::msg::Marker::MODIFY;
    amber_right.scale.x = search_start_width_x;
    amber_right.scale.y = search_start_width_y;
    amber_right.scale.z = 0.2;
    amber_right.color.r = md_amber_500_r;
    amber_right.color.g = md_amber_500_g;
    amber_right.color.b = md_amber_500_b;
    amber_right.color.a = 0.8;
    amber_right.id = 1;
    amber_right.pose.position.x = 0.0;
    amber_right.pose.position.y = 0.5 * search_start_width_y + 0.1;
    amber_right.pose.position.z = 0.0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_start(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_start = crop_pcl(cloud, -0.5 * search_start_width_x, -1.0 * search_start_width_y, +0.5 * search_start_width_x, +1.0 * search_start_width_y);
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

    // LIDAR looks backward, so the left side is positive y and the right side is negative y TODO: parameterize
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_fwd(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_fwd = crop_pcl(cloud, -8.0, -1.5, -0.1, +1.5); // cloud, min_x, min_y, max_x, max_y
    // RCLCPP_INFO_STREAM(this->get_logger(), "crop_fwd: " << cloud_fwd->width * cloud_fwd->height);

    // get the smallest x value from cloud_fwd
    float min_x = -10.0;
    for (pcl::PointXYZI p : cloud_fwd->points)
    {
      if (p.x > min_x)
      {
        min_x = p.x;
      }
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_left = crop_pcl(cloud, -4.0, -3.5, -0.001, 0.0); // cloud, min_x, min_y, max_x, max_y
    // RCLCPP_INFO_STREAM(this->get_logger(), "crop_left: " << cloud_left->width * cloud_left->height);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_righ(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_righ = crop_pcl(cloud, -4.0, 0.0, -0.001, +3.5);

    // get the largest y value from cloud_left
    Point left_start(0.0, -10.0);
    for (pcl::PointXYZI p : cloud_left->points)
    {
      if (p.y > left_start.y)
      {
        left_start.y = p.y;
        left_start.x = p.x;
      }
    }
    cluster1.add_back(left_start, 1);
    // RCLCPP_INFO_STREAM(this->get_logger(), "left_start: " << left_start.x << ", " << left_start.y);
    // get the smallest y value from cloud_righ
    Point right_start(0.0, +10.0);
    for (pcl::PointXYZI p : cloud_righ->points)
    {
      if (p.y < right_start.y)
      {
        right_start.y = p.y;
        right_start.x = p.x;
      }
    }
    cluster1.add_back(right_start, 2);
    if (cluster1.get_size(1) >= 1)
    {
      for (pcl::PointXYZI pxyzi : cloud->points)
      {
        Point p(pxyzi.x, pxyzi.y);
        if (eps_min <= cluster1.distance(p, left_start) && cluster1.distance(p, left_start) <= eps_max)
        {
          double candidate_ang = cluster1.calculate_angle(p, left_start);
          double angle_difference = cluster1.angle_diff(candidate_ang, 0.0); // 0.0 rad is up on X axis
          if (angle_difference < ang_threshold)
          {
            cluster1.add_back(p.x, p.y, 1);
            break;
          }
        }
      }
    }
    double tmp_angle_difference = -1.0;
    if (cluster1.get_size(2) >= 1)
    {
      for (pcl::PointXYZI pxyzi : cloud->points)
      {
        Point p(pxyzi.x, pxyzi.y);
        if (eps_min <= cluster1.distance(p, right_start) && cluster1.distance(p, right_start) <= eps_max)
        {
          double candidate_ang = cluster1.calculate_angle(p, right_start);
          double angle_difference = cluster1.angle_diff(candidate_ang, 0.0); // 0.0 rad up on X axis
          tmp_angle_difference = angle_difference;
          if (angle_difference < ang_threshold)
          {
            cluster1.add_back(p.x, p.y, 2);

            break;
          }
        }
      }
    }
    if (cluster1.get_size(1) >= 2)
    {
      bool extending = true;
      while (extending == true)
      {
        extending = cluster1.next_tail(1);
      }
    }
    if (cluster1.get_size(2) >= 2)
    {
      bool extending = true;
      while (extending == true)
      {
        extending = cluster1.next_tail(2);
      }
    }

    // if (cluster1.get_size(1) >= 1)
    // {
    //   RCLCPP_INFO_STREAM(this->get_logger(), "cluster1(1) size: " << cluster1.get_size(1));
    // }
    // if (cluster1.get_size(2) >= 1)
    // {
    //   RCLCPP_INFO_STREAM(this->get_logger(), "cluster1(2) size: " << cluster1.get_size(2));
    // }

    visualization_msgs::msg::Marker debug1_marker, debug2_marker, debug_text_marker;
    init_debug_marker(debug1_marker, left_start.x, left_start.y, 1);
    debug1_marker.header.frame_id = input_msg->header.frame_id;
    debug1_marker.header.stamp = this->now();
    init_debug_marker(debug2_marker, right_start.x, right_start.y, 2);
    debug2_marker.header.frame_id = input_msg->header.frame_id;
    debug2_marker.header.stamp = this->now();
    init_text_debug_marker(debug_text_marker);
    debug_text_marker.header.frame_id = input_msg->header.frame_id;
    debug_text_marker.header.stamp = this->now();
    debug_text_marker.text = std::to_string(tmp_angle_difference);

    visualization_msgs::msg::Marker cluster1_marker;
    cluster1_marker.header.frame_id = input_msg->header.frame_id;
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
    cluster2_marker.header.frame_id = input_msg->header.frame_id;
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
    for (int i = 0; i < cluster1.get_size(1) - 1; i++)
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
    mark_array.markers.push_back(debug_text_marker);
    mark_array.markers.push_back(debug1_marker);
    mark_array.markers.push_back(debug2_marker);
    pub_marker_->publish(mark_array);

    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    // Add the same frame_id as the input, it is not included in pcl PointXYZI
    output_msg.header.frame_id = input_msg->header.frame_id;
    // Publish the data as a ROS message
    pub_lidar_->publish(output_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  float minX = -80.0, minY = -25.0, minZ = -2.0;
  float maxX = +80.0, maxY = +25.0, maxZ = -0.15;
  bool verbose1 = false, verbose2 = false;
  float search_start_width_x = 20.0, search_start_width_y = 6.5;
  std::string points_in_topic, points_out_topic, marker_out_topic;
  int cluster_num = 5;
  float eps_min = 1.2, eps_max = 3.4, ang_threshold_deg = 30.0;
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