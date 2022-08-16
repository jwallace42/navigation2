//
// Created by josh on 8/16/22.
//

#include "nav2_engine/engine_server.hpp"

namespace nav2_engine {

EngineServer::EngineServer(const rclcpp::NodeOptions &options)
: LifecycleNode("engine_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Constructing Engine Server");
}

nav2_util::CallbackReturn EngineServer::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Configuring Engine Server");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn EngineServer::on_activate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Activating Engine Server");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn EngineServer::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Deactivating Engine Server");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn EngineServer::on_cleanup(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Cleaning up Engine Server");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn EngineServer::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down Engine Server");
  return nav2_util::CallbackReturn::SUCCESS;
}

EngineServer::~EngineServer() {
  RCLCPP_INFO(get_logger(), "Destructing Engine Server");
}

} // nav2_engine