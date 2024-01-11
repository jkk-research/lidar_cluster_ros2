// Non-grid (spatial) euclidean cluster filter for point cloud data
// based on https://github.com/autowarefoundation/autoware.universe/blob/main/perception/euclidean_cluster/lib/euclidean_cluster.cpp
// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

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
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class EucledianSpatial : public rclcpp::Node
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
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_in_topic, rclcpp::SensorDataQoS().keep_last(1), std::bind(&EucledianSpatial::lidar_callback, this, std::placeholders::_1));
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
  EucledianSpatial() : Node("eucledian_spatial"), count_(0)
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
    this->declare_parameter<float>("tolerance", tolerance_);
    this->declare_parameter<int>("min_cluster_size", min_cluster_size_);
    this->declare_parameter<int>("max_cluster_size", max_cluster_size_);
    this->declare_parameter<bool>("use_height", use_height_);
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
    this->get_parameter("tolerance", tolerance_);
    this->get_parameter("min_cluster_size", min_cluster_size_);
    this->get_parameter("max_cluster_size", max_cluster_size_);
    this->get_parameter("use_height", use_height_);

    pub_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(points_out_topic, 10);
    pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_out_topic, 10);
    // TODO: QoS // rclcpp::SensorDataQoS().keep_last(1)
    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_in_topic, 10, std::bind(&EucledianSpatial::lidar_callback, this, std::placeholders::_1));
    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&EucledianSpatial::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "EucledianSpatial node has been started.");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: '%s'", points_in_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: '%s' and '%s'", points_out_topic.c_str(), marker_out_topic.c_str());
  }

private:
  void lidar_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
  {
    // Convert to PCL data type
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>); // not PointXYZI
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input_msg, *pointcloud);
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    int original_size = pointcloud->width * pointcloud->height;



    // Filter out points outside of the box
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setInputCloud(pointcloud);
    crop.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    crop.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    crop.filter(*pointcloud);

    if (verbose1)
    {
      // print the length of the point pointcloud
      RCLCPP_INFO_STREAM(this->get_logger(), "PointCloud in: " << original_size << " Reduced size: " << pointcloud->width * pointcloud->height);
    }


    // convert 2d pointcloud
    if (!use_height_)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto &point : pointcloud->points)
      {
        {
          pcl::PointXYZ point2d;
          point2d.x = point.x;
          point2d.y = point.y;
          point2d.z = 0.0;
          pointcloud_2d_ptr->push_back(point2d);
        }
      }
      pointcloud_ptr = pointcloud_2d_ptr;
    }
    else
    {
      pointcloud_ptr = pointcloud;
    }

    // create tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pointcloud_ptr);
    // clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
    pcl_euclidean_cluster.setClusterTolerance(tolerance_);
    pcl_euclidean_cluster.setMinClusterSize(min_cluster_size_);
    pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size_);
    pcl_euclidean_cluster.setSearchMethod(tree);
    pcl_euclidean_cluster.setInputCloud(pointcloud_ptr);
    pcl_euclidean_cluster.extract(cluster_indices);
    // build output
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    {
      for (const auto &cluster : cluster_indices)
      {
        
        for (const auto &point_idx : cluster.indices)
        {
          cloud_cluster->points.push_back(pointcloud->points[point_idx]);
        }
        clusters.push_back(*cloud_cluster);
        clusters.back().width = cloud_cluster->points.size();
        clusters.back().height = 1;
        clusters.back().is_dense = false;
      }
    }

    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*pointcloud, output_msg);
    // Add the same frame_id as the input, it is not included in pcl PointXYZ
    output_msg.header.frame_id = input_msg->header.frame_id;
    // Publish the data as a ROS message
    pub_lidar_->publish(output_msg);
    // RCLCPP_INFO_STREAM(this->get_logger(), "PointCloud published");
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  float minX = -80.0, minY = -25.0, minZ = -2.0;
  float maxX = 80.0, maxY = +25.0, maxZ = -0.15;
  float tolerance_ = 0.02;
  int min_cluster_size_ = 10;
  int max_cluster_size_ = 500;
  bool use_height_ = false;
  bool verbose1 = false, verbose2 = false;
  std::string points_in_topic, points_out_topic, marker_out_topic;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EucledianSpatial>());
  rclcpp::shutdown();
  return 0;
}