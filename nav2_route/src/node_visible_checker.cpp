/*
 * Copyright (C) 2023, Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Licensed under Proprietary and confidential
 */

#include "nav2_route/node_visible_checker.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
namespace nav2_route
{

NodeVisibleChecker::NodeVisibleChecker(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    const std::string &topic_name)
    : costmap_sub_(parent, topic_name)
{
}

bool NodeVisibleChecker::isNodeValid(const geometry_msgs::msg::PoseStamped &node,
                                     const geometry_msgs::msg::PoseStamped &pose)
{
  // Assume it is the map frame
  unsigned int x0, x1, y0, y1;
  costmap_sub_.getCostmap()->worldToMap(node.pose.position.x, node.pose.position.y, x0, y0);
  costmap_sub_.getCostmap()->worldToMap(pose.pose.position.x, node.pose.position.y, x1, y1);

  for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    double cost = costmap_sub_.getCostmap()->getCost(line.getX(), line.getY());

    if (cost == static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
      return false;
    }
  }
  return true;
}

}