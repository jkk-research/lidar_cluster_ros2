// Non-grid (spatial) DBSCAN filter for point cloud data
// The DBSCAN (Density-Based Spatial Clustering of Applications with Noise) algorithm is a popular clustering algorithm in machine learning

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
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

using namespace std::chrono_literals;
using std::placeholders::_1;

class Point
{
public:
  double x, y;
  int neighbor_pts = 0;
  bool core = false;
  int cluster_id = -1;
};

double distance(const Point &p1, const Point &p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

void find_neighbors(std::vector<Point> &points, double eps)
{
  for (Point &p : points)
  {
    for (const Point &q : points)
    {
      if (distance(p, q) <= eps)
      {
        p.neighbor_pts++;
      }
    }
    p.core = p.neighbor_pts >= 3;
  }
}

void find_clusters(std::vector<Point> &points, double eps)
{
  int actual_cluster_id = 0;
  for (int i = 0; i < 10; i++)
  {
    for (Point &p : points)
    {
      if (p.core && p.cluster_id == -1)
      {
        p.cluster_id = actual_cluster_id;
        break;
      }
    }
    for (Point &p : points)
    {
      if (p.cluster_id == actual_cluster_id)
      {
        for (Point &q : points)
        {
          if (q.core && q.cluster_id == -1 && distance(p, q) <= eps)
          {
            q.cluster_id = actual_cluster_id;
          }
        }
      }
    }
    actual_cluster_id++;
  }
  for (Point &p : points)
  {
    if (!p.core)
    {
      for (const Point &q : points)
      {
        if (q.core && q.cluster_id != -1 && distance(p, q) <= eps)
        {
          p.cluster_id = q.cluster_id;
        }
      }
    }
  }
}
class DbscanSpatial : public rclcpp::Node
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
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_in_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&DbscanSpatial::lidar_callback, this, std::placeholders::_1));
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
  DbscanSpatial() : Node("dbscan_spatial"), count_(0)
  {
    this->declare_parameter<float>("minX", minX);
    this->declare_parameter<float>("minY", minY);
    this->declare_parameter<float>("minZ", minZ);
    this->declare_parameter<float>("maxX", maxX);
    this->declare_parameter<float>("maxY", maxY);
    this->declare_parameter<float>("maxZ", maxZ);
    this->declare_parameter<std::string>("points_in_topic", "/lexus3/os_center/points");
    this->declare_parameter<std::string>("points_out_topic", "cluster_points");
    this->declare_parameter<std::string>("marker_out_topic", "cluster_marker");
    this->declare_parameter<bool>("verbose1", verbose1);
    this->declare_parameter<bool>("verbose2", verbose2);
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

    pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic, 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_out_topic, 10);
    // TODO: QoS // rclcpp::SensorDataQoS().keep_last(1)
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_in_topic, 10, std::bind(&DbscanSpatial::lidar_callback, this, std::placeholders::_1));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DbscanSpatial::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "DbscanSpatial node has been started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: '%s'", points_in_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: '%s' and '%s'", points_out_topic.c_str(), marker_out_topic.c_str());
  }

private:
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input_msg, *cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    
    // print the length of the point cloud
    if (verbose1)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "PointCloud in: " << cloud->width * cloud->height << " data points");
    }

    // DBSCAN
    // find neighbors in cloud 
    std::vector<Point> points;

    int i = 0;

    for (const pcl::PointXYZI &p : cloud->points)
    {
      // temporary filter only out a small portion of the point cloud, to work in real time // TODO: fix
      if (i > 60000 and i < 70000)
      {
        Point point;
        point.x = p.x;
        point.y = p.y;
        points.push_back(point);
      }
      i++;
    }


    find_neighbors(points, 3.5);
    // find clusters in cloud
    find_clusters(points, 3.5);
    
    // convert to PointXYZI
    for (const Point &p : points)
    {
      pcl::PointXYZI point;
      point.x = p.x;
      point.y = p.y;
      point.z = 0.0;
      point.intensity = p.cluster_id;
      cloud_filtered->points.push_back(point);
    }
    

    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_filtered, output_msg);
    // Add the same frame_id as the input, it is not included in pcl PointXYZI
    output_msg.header.frame_id = input_msg->header.frame_id;
    // Publish the data as a ROS message
    pub_lidar_->publish(output_msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  float minX = 0.0, minY = -5.0, minZ = -2.0;
  float maxX = 40.0, maxY = +5.0, maxZ = -0.15;
  bool verbose1 = false, verbose2 = false;
  std::string points_in_topic, points_out_topic, marker_out_topic;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DbscanSpatial>());
  rclcpp::shutdown();
  return 0;
}