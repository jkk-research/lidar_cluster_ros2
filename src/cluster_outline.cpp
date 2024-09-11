// ROS
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

#include "cluster_outline.hpp"

namespace outline {


double ClusterOutline::calculateDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

pcl::PointXYZ ClusterOutline::findClosestPoint(const pcl::PointXYZ& mid_point, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster) {
    double min_distance = std::numeric_limits<double>::max();
    pcl::PointXYZ closest_point;
    for (const auto& point : cluster->points) {
        double distance = calculateDistance(mid_point, point);
        if (distance < min_distance) {
        min_distance = distance;
        closest_point = point;
        }
    }
    closest_point.z = mid_point.z;
    return closest_point;
}


void ClusterOutline::addPointsIfNecessary(pcl::PointCloud<pcl::PointXYZ>::Ptr& hull, 
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster, 
                            double max_distance, int& counter, int max_added_points) {

    bool added_points = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_hull(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < hull->points.size(); ++i) {
        const auto& p1 = hull->points[i];
        const auto& p2 = hull->points[(i + 1) % hull->points.size()];  // wrap around to the first point
        new_hull->points.push_back(p1);
        double distance = calculateDistance(p1, p2);
        if (distance > max_distance) {
        pcl::PointXYZ mid_point;
        mid_point.x = (p1.x + p2.x) / 2.0;
        mid_point.y = (p1.y + p2.y) / 2.0;
        mid_point.z = (p1.z + p2.z) / 2.0;
        pcl::PointXYZ closest_point = findClosestPoint(mid_point, cluster);
        new_hull->points.push_back(closest_point);
        added_points = true;
        }
    }
    hull = new_hull;
    if (added_points && counter < max_added_points) {
        addPointsIfNecessary(hull, cluster, max_distance, ++counter, max_added_points);
    }
}


void ClusterOutline::computeOutline(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud, 
                    visualization_msgs::msg::MarkerArray& hull_markers, int max_added_points, int max_clust_reached, std::string frame_id) {


    // Map to store clusters, with the intensity as the key and the points as the value
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

    // Separate the point cloud into individual clusters
    for (const auto& point : pointcloud->points) {
        pcl::PointXYZ point_xyz;
        point_xyz.x = point.x;
        point_xyz.y = point.y;
        // pressing the z value to 0, for the outline height doesn't matter
        point_xyz.z = 0;

        // If the cluster doesn't exist in the map, create it
        if (clusters.find(point.intensity) == clusters.end()) {
            clusters[point.intensity] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }

        // Add the point to the appropriate cluster
        clusters[point.intensity]->points.push_back(point_xyz);
    }
    
    // Create the Convex Hull for each cluster
    int cluster_id = 0;
    for (const auto& cluster : clusters) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(cluster.second);
        chull.reconstruct(*cloud_hull);

        // Add intermediate points if necessary
        double max_distance = 4.0;  // maximum allowed distance between consecutive points 
        int counter = 0;
        addPointsIfNecessary(cloud_hull, cluster.second, max_distance, counter, max_added_points);

        // Create a marker for this cluster
        visualization_msgs::msg::Marker hull_marker;
        hull_marker.header.frame_id = frame_id;
        hull_marker.header.stamp = rclcpp::Clock().now();
        hull_marker.ns = "hull";
        hull_marker.id = cluster_id++;
        hull_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        hull_marker.action = visualization_msgs::msg::Marker::ADD;
        hull_marker.scale.x = 0.2;
        hull_marker.color.a = 1.0;
        hull_marker.color.r = 0.30; // 0.30 0.69 0.31 md_green_500 https://github.com/jkk-research/colors
        hull_marker.color.g = 0.69;
        hull_marker.color.b = 0.31;

        // Add the points of the Convex Hull to the marker
        for (const auto& point : cloud_hull->points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = -1; // a z value of -1 is around the bumper height
            hull_marker.points.push_back(p);
        }
        // Close the loop by adding the first point to the end
        if (!cloud_hull->points.empty()) {
            geometry_msgs::msg::Point p;
            p.x = cloud_hull->points[0].x;
            p.y = cloud_hull->points[0].y;
            p.z = -1;
            hull_marker.points.push_back(p);
        }
        // Add the marker to the array
        hull_markers.markers.push_back(hull_marker);
        // **Add a TEXT_VIEW_FACING marker for distance**

        // Find the center of the cluster (mean of all points)
        pcl::PointXYZ centroid(0.0, 0.0, 0.0);
        for (const auto& point : cluster.second->points) {
            centroid.x += point.x;
            centroid.y += point.y;
        }
        centroid.x /= cluster.second->points.size();
        centroid.y /= cluster.second->points.size();

        // Calculate the distance from the origin (0,0) to the cluster center
        double distance = calculateDistance(pcl::PointXYZ(0, 0, 0), centroid);

        // Create the TEXT_VIEW_FACING marker
        visualization_msgs::msg::Marker distance_marker;
        distance_marker.header.frame_id = frame_id;
        distance_marker.header.stamp = rclcpp::Clock().now();
        distance_marker.ns = "distance";
        distance_marker.id = cluster_id++;  // Increment the ID for unique markers
        distance_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        distance_marker.action = visualization_msgs::msg::Marker::ADD;
        distance_marker.scale.z = 2;  
        distance_marker.color.a = 1.0;
        distance_marker.color.r = 1.0;
        distance_marker.color.g = 1.0;
        distance_marker.color.b = 1.0;

        // Set the position of the text marker to the cluster centroid with an offset in z
        distance_marker.pose.position.x = centroid.x;
        distance_marker.pose.position.y = centroid.y;
        distance_marker.pose.position.z = 1;  // Place text slightly above the hull marker

        // Format the distance to one decimal place and add 'm' for meters
        std::stringstream stream;
        stream << std::fixed << std::setprecision(1) << distance;
        distance_marker.text = stream.str() + "m";

        // Add the distance marker to the array
        hull_markers.markers.push_back(distance_marker);
    }
    // Add markers for clusters that are not present in the current frame to avoid ghost markers   
    visualization_msgs::msg::Marker hull_marker;
    hull_marker.header.stamp = rclcpp::Clock().now();
    hull_marker.header.frame_id = frame_id;
    hull_marker.ns = "hull";
    hull_marker.color.a = 0.0;    // alpha = 0.0 makes the marker invisible
    hull_marker.scale.x = 0.2;
    hull_marker.scale.y = 0.2;
    hull_marker.scale.z = 0.2;
    for(int i = clusters.size() + 1 ; i < max_clust_reached; i++) {
        hull_marker.id = cluster_id++;
        hull_markers.markers.push_back(hull_marker);
    }

}


} // namespace outline
