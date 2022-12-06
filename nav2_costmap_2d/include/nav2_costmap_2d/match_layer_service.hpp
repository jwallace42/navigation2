// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_COSTMAP_2D__MATCH_LAYER_SERVICE_HPP_
#define NAV2_COSTMAP_2D__MATCH_LAYER_SERVICE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/srv/match_layer.hpp"

namespace nav2_costmap_2d
{

class Costmap2DROS;

/**
 * @class MatchLayerService
 * @brief Exposes services to match the costmap layer
 */
class MatchLayerService
{
public:
  /**
   * @brief Constructor
   */
  MatchLayerService(const nav2_util::LifecycleNode::WeakPtr & parent, Costmap2DROS & costmap);

private:
  using MatchLayer = nav2_msgs::srv::MatchLayer;

  rclcpp::Logger logger_{rclcpp::get_logger("nav2_costmap_2d")};

  Costmap2DROS & costmap_;

  rclcpp::Service<MatchLayer>::SharedPtr match_layer_service_;

  void matchLayerCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<MatchLayer::Request>,
    const std::shared_ptr<MatchLayer::Response>);
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__MATCH_LAYER_SERVICE_HPP_
