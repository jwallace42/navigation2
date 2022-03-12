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


#include <buffer.h>
#include "tf2_ros/transform_listener.h"

#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

//#include <action message def>

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
  // void twistCallback(const geometry_msgs::msg::Twist msg);
};

/**
 * @class nav2_assisted_teleop::AssistedTeleopServer
 * @brief This class provides a action server refine/reduce commanded
 * velocities from a teleoperator to prevent collisions
 */
class AssistedTeleopServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for nav2_assisted_teleop::AssistedTeleopServer
   * @param options Additional options to control creation of the node.
   */
  explicit AssistedTeleopServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor for the nav2_assisted_teleop::AssistedTeleopServer
   */
  ~AssistedTeleopServer();

protected:
  /**
   * @brief Configures assisted teleop params and member variable
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Callback function for velocity subscriber
   * @param msg received Twist message
   */
  void velCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief AssistedTeleop action server callback. Handles action server updates and
   * spins server until goal is reached
   *
   * While this action is running, commands from joystick are filtered and then published to the base.
   */
  // TODO(jwallace42): not sure what a good name is
  void execute();

private:
  AssistedTeleop assisted_teleop;

  rclcpp::Logger logger_;

  // Transforms
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  // Sub/Pubs
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;

  // Parameters
  double projection_time_;
};

}

#endif  // NAV2_ASSISTED_TELEOP__NAV2__NAV2_ASSISTED_TELEOP_HPP_
