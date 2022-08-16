//
// Created by josh on 8/16/22.
//

#ifndef NAV2_WS_ENGINEINTERFACE_HPP
#define NAV2_WS_ENGINEINTERFACE_HPP

#include <string>
#include <functional>

#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace nav2_engine {

template<class ActionT>
class EngineInterface {
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  typedef std::function<bool (typename ActionT::Goal::ConstSHaredPtr)> OnGoalReceivedCallback;
  typedef std::function<void ()> OnLoopCallback;
  typedef std::function<void (typename ActionT::Goal::ConstSharedPtr)> OnPreemptCallback;
  typedef std::function<void (typename ActionT::Result::SharedPtr)> OnCompletionCallback;

  explicit EngineInterface(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      const std::string & action_name,
      OnGoalReceivedCallback on_goal_received_callback,
      OnLoopCallback on_loop_callback,
      OnPreemptCallback on_preempt_callback,
      OnCompletionCallback on_completion_callback);

  ~EngineInterface();


  bool on_configure();

  bool on_activate();

  bool on_deactivate();

  bool on_cleanup();

protected:
  std::string action_name_;
  std::shared_ptr<ActionServer> action_server_;
};

} // nav2_engine

#endif //NAV2_WS_ENGINEINTERFACE_HPP
