#include <memory>

#include "nav2_planner/planner_server.hpp"
#include "nav2_controller/controller_server.hpp"
#include "nav2_behaviors/behavior_server.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create costmaps to be shared
  auto global_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap",
      true);

  auto local_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", true);

  auto planner_node = std::make_shared<nav2_planner::PlannerServer>(global_costmap_ros_);
  auto controller_node = std::make_shared<nav2_controller::ControllerServer>(local_costmap_ros_);
  auto behaviors_node = std::make_shared<behavior_server::BehaviorServer>(global_costmap_ros_,
                                                                          local_costmap_ros_);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planner_node->get_node_base_interface());
  executor.add_node(controller_node->get_node_base_interface());
  executor.add_node(behaviors_node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
