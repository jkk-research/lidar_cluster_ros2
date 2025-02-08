#ifndef CLUSTER_OUTLINE_HPP
#define CLUSTER_OUTLINE_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "visualization_msgs/msg/marker_array.hpp"

namespace outline {

class ClusterOutline {
private:

// Calculate the distance between two points, X and Y coordinates only
double calculateDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);

// Find the closest point to the mid point of two points
pcl::PointXYZ findClosestPoint(const pcl::PointXYZ& mid_point, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);

// Add points to the hull if the distance between two points is greater than max_distance
void addPointsIfNecessary(pcl::PointCloud<pcl::PointXYZ>::Ptr& hull, 
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster, 
                              double max_distance, int& counter, int max_added_points);

public:
// Compute the outline of a cluster. 
// The input pointcloud has to be PointXYZI where Intensity is the cluster ID.
// max_added_points is the maximum number of points that can be added to the hull
void computeOutline(pcl::PointCloud<pcl::PointXYZI>::Ptr& pointcloud, 
                        visualization_msgs::msg::MarkerArray& hull_markers, 
                        int max_added_points, int max_clust_reached, 
                        std::string frame_id, float marker_height = -1.0);


}; // class ClusterOutline

} // namespace outline

#endif // CLUSTER_OUTLINE_HPP
