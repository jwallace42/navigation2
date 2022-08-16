//
// Created by josh on 8/16/22.
//

#ifndef NAV2_WS_MUXER_HPP
#define NAV2_WS_MUXER_HPP

#include <optional>
#include <map>
#include <string>
#include <memory>

#include "nav2_engine/engine_interface.hpp"

namespace nav2_engine {

class Muxer {
public:
  /**
   * \brief
   */
  void run(std::string &action_name);

  void addAction(std::string &name, std::shared_ptr<EngineInterface> &action);

protected:
  void preemptActiveAction();

  void cancelActiveAction();


  std::optional<std::string> active_action_;
  std::map<std::string, std::shared_ptr<EngineInterface>> action_map_;
};

} // nav2_engine

#endif //NAV2_WS_MUXER_HPP
