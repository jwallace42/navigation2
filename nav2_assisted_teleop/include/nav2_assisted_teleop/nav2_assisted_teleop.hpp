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


#ifndef NAV2_ASSISTED_TELEOP__NAV2__NAV2_ASSISTED_TELEOP_HPP_
#define NAV2_ASSISTED_TELEOP__NAV2__NAV2_ASSISTED_TELEOP_HPP_

#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace nav2_assisted_teleop
{

/**
 * @class nav2_assisted_teleop::AssistedTeleop
 * @brief This class contians the core functionality for
 * calculating the reduced/refined velocity command
 */
class AssistedTeleop
{
public:
  /**
   * @brief Reduce velocity if a collision is imminent
   * @param twist current speed
   * @return geometry_msgs::msg::Twist reduced velocity
   */
  geometry_msgs::msg::Twist computeVelocity(geometry_msgs::msg::Twist twist);

  /**
   * @brief preform a linear projection of a pose
   * @param pose the pose to project
   * @param twist speed to project the pose by
   * @param projected_time amount of time to project the pose by
   * @return geometry_msgs::msg::Pose the projected pose
   */
  geometry_msgs::msg::Pose projectPose(geometry_msgs::msg::Pose pose,
                                          geometry_msgs::msg::Twist twist,
                                          double projected_time);

  /**
   * @brief publish velocity to the base
   * @param twist velocity to publish
   */
  void publishTwist(geometry_msgs::msg::Twist twist);

  /**
   * @brief publish zero velocity
   */
  void stop();

private:
  /**
   * @brief Callback function for the twist subcriber
   * @param msg twist message
   */
  void twistCallback(const geometry_msgs::msg::Twist msg);
};

/**
 * @class nav2_assisted_teleop::AssistedTeleopServer
 * @brief This class provides a action server refine/reduce commanded
 * velocities from a teleoperator to prevent collisions
 */
class AssistedTeleopServer : public nav2_util::LifecycleNode
{
  public:


  protected:
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  private:
    AssistedTeleop assisted_teleop;

};

}

#endif  // NAV2_ASSISTED_TELEOP__NAV2__NAV2_ASSISTED_TELEOP_HPP_
