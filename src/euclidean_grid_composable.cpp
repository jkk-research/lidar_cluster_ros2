
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "rclcpp_components/component_manager_isolated.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto manager = std::make_shared<rclcpp_components::ComponentManager>(options);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(manager);

  manager->load_component("cluster::EuclideanGrid", options);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}