/*
 * Copyright (C) 2023, Locus Robotics. All rights reserved.
 * Unauthorized copying of this file, via any medium, is strictly prohibited
 * Licensed under Proprietary and confidential
 */

#ifndef NAV2_WS_SRC_NAVIGATION2_NAV2_ROUTE_NODE_VISABLE_CHECKER_HPP_
#define NAV2_WS_SRC_NAVIGATION2_NAV2_ROUTE_NODE_VISABLE_CHECKER_HPP_

#include "nav2_util/line_iterator.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_route
{

class NodeVisibleChecker
{
 public:
  NodeVisibleChecker(
      const nav2_util::LifecycleNode::WeakPtr & parent,
      const std::string & topic_name);

  bool isNodeValid(
    const geometry_msgs::msg::PoseStamped & node,
    const geometry_msgs::msg::PoseStamped & pose);

 private:
  nav2_costmap_2d::CostmapSubscriber costmap_sub_;
};

}

#endif //NAV2_WS_SRC_NAVIGATION2_NAV2_ROUTE_NODE_VISABLE_CHECKER_HPP_
