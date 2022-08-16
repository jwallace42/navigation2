//
// Created by josh on 8/16/22.
//

#ifndef NAV2_WS_ENGINESERVER_HPP
#define NAV2_WS_ENGINESERVER_HPP

#include "nav2_util/lifecycle_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav2_engine/muxer.hpp"

namespace nav2_engine {

class EngineServer : public nav2_util::LifecycleNode {
public:

  /**
   * \brief A contructor for nav2_engine::SmootherServer
   * \param options Additional option to control creation of the node
   */
  explicit EngineServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~EngineServer() override;
protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn  on_deactivate(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  Muxer action_mux_;
};

} // nav2_engine

#endif //NAV2_WS_ENGINESERVER_HPP
