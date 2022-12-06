// Copyright (c) 2022 Joshua Wallace
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

#include <memory>

#include "nav2_costmap_2d/match_layer_service.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_costmap_2d
{

MatchLayerService::MatchLayerService(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  Costmap2DROS & costmap)
: costmap_(costmap)
{
  auto node = parent.lock();
  logger_ = node->get_logger();

  match_layer_service_ = node->create_service<MatchLayer>(
    "match_layer",
    std::bind(
      &MatchLayerService::matchLayerCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void MatchLayerService::matchLayerCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<MatchLayer::Request>,
  const std::shared_ptr<MatchLayer::Response>)
{
  RCLCPP_INFO(logger_, "Running match layer callback");
}

}  // namespace nav2_costmap_2d
