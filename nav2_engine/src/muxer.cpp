//
// Created by josh on 8/16/22.
//

#include "nav2_engine/muxer.hpp"

namespace nav2_engine {
void Muxer::addAction(std::string &name, std::shared_ptr<EngineInterface>& action) {
  action_map_.insert( std::pair<std::string, std::shared_ptr<EngineInterface>>(name, action) );
}

void Muxer::preemptActiveAction() {

}


void Muxer::run(std::string &action_name) {
  if (active_action_)
  {
    if (action_name == active_action_.value()) {
      preemptActiveAction();
      return;
    } else {
      cancelCurrentAction();
    }
  }
//  action_map_[action_name]->run();
}

} // nav2_engine