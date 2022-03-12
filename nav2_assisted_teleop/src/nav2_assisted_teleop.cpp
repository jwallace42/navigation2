// Copyright (c) 2021 Anushree Sabnis
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

#include "nav2_assisted_teleop/nav2_assisted_teleop.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_util/node_utils.hpp"

namespace nav2_assisted_teleop
{
  AssistedTeleopServer::AssistedTeleopServer(const rclcpp::NodeOptions & options)
  : LifecycleNode("assisted_teleop_server", "", false, options),
  logger_(get_logger())
  {
    RCLCPP_INFO(logger_, "Creating assisted teleop server");

    declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
  }

  AssistedTeleop::~AssistedTeleop()
  {
    // do nothing
  }

  nav2_util::CallbackReturn
  AssistedTeleopServer::on_configure(const rclcpp_lifecycle::State &)
  {
    auto node_ = shared_from_this();

    // Set up parameters
    nav2_util::declare_parameter_if_not_declared(
    node, "projection_time", rclcpp::ParameterValue(1.0));

    // nav2_util::declare_parameter_if_not_declared(
    //   node,
    //   "linear_velocity_threshold_", rclcpp::ParameterValue(0.06));

    nav2_util::declare_parameter_if_not_declared(
      this,
      "input_vel_topic", rclcpp::ParameterValue(std::string("input_vel_topic")));

    nav2_util::declare_parameter_if_not_declared(
      node,
      "output_vel_topic", rclcpp::ParameterValue(std::string("cmd_vel")));

    // Get parameters
    this->get_parameter("projection_time", projection_time_);

    std::string input_vel_topic;
    this->get_parameter("input_vel_topic", input_vel_topic);

    std::string output_vel_topic;
    this->get_parameter("output_vel_topic", output_vel_topic);

    //     std::string input_vel_topic;
    // this->get_parameter("input_vel_topic", input_vel_topic);

    tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf_->setCreateTimerInterface(timer_interface);
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    // Initialize pubs & subs
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_vel_topic, 1);
  }


  void AssistedTeleopServer::execute()
  {
    RCLCPP_INFO(logger_, "Running teleop action");
  //   RCLCPP_INFO(logger_, "Attempting %s", recovery_name_.c_str());

    auto node = node_.lock();
    // Log a message every second
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

  // auto timer = node->create_wall_timer(
  //   1s,
  //   [&]()
  //   {RCLCPP_INFO(logger_, "%s running...", recovery_name_.c_str());});


  // auto start_time = node->now();

  // // Initialize the ActionT goal and result
  // auto at_goal = action_server_->get_current_goal();
  // auto result = std::make_shared<AssistedTeleopAction::Result>();

  // rclcpp::WallRate loop_rate(cycle_frequency_);
  // auto assisted_teleop_end_ = (node->now() + at_goal->time).nanoseconds();

  // vel_sub_ = node->create_subscription<geometry_msgs::msg::Twist>(
  //   input_vel_topic_, rclcpp::SystemDefaultsQoS(),
  //   std::bind(
  //     &AssistedTeleop::velCallback,
  //     this, std::placeholders::_1));

  // while (rclcpp::ok()) {
  //   if (action_server_->is_cancel_requested()) {
  //     RCLCPP_INFO(logger_, "Canceling %s", recovery_name_.c_str());
  //     stopRobot();
  //     vel_sub_.reset();
  //     result->total_elapsed_time = node->now() - start_time;
  //     action_server_->terminate_all(result);
  //     return;
  //   }

  //   if (action_server_->is_preempt_requested()) {
  //     RCLCPP_ERROR( "Received a preemption request for %s,"
  //       " however feature is currently not implemented. Aborting and stopping.",
  //       recovery_name_.c_str());
  //     stopRobot();
  //     vel_sub_.reset();
  //     result->total_elapsed_time = node->now() - start_time;
  //     action_server_->terminate_current(result);
  //     return;
  //   }

  //   auto current_point = node->now().nanoseconds();

  //   auto time_left =
  //     assisted_teleop_end_ - current_point;

  //   // Enable recovery behavior if we haven't run out of time
  //   if (time_left < 0) {
  //     vel_sub_.reset();
  //     result->total_elapsed_time = node->now() - start_time;
  //     action_server_->succeeded_current(result);
  //     RCLCPP_INFO("%s completed successfully", recovery_name_.c_str());
  //     return;
  //   }
  // }
  // loop_rate.sleep();
  }



}  // nav2_assisted_teleop
