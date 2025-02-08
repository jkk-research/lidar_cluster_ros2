#include "euclidean_grid_core.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cluster::EuclideanGridCore>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}