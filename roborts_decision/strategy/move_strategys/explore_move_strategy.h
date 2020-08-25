//
// Created by lq on 2020/8/25.
//

#ifndef ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_STRATEGY_MOVE_STRATEGYS_EXPLORE_MOVE_STRATEGY_H_
#define ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_STRATEGY_MOVE_STRATEGYS_EXPLORE_MOVE_STRATEGY_H_

#include "../abstract_common_strategy.h"

namespace roborts_decision {

class ExploreMoveStrategy : public AbstractCommonStrategy {

 public:
  ExploreMoveStrategy(const std::shared_ptr<MyRobot> &p_my_robot, const std::shared_ptr<Blackboard> &p_blackboard)
      : AbstractCommonStrategy(p_my_robot, p_blackboard) {}

  void run() override;

  StrategyID getID() override { return StrategyID::exploreMove; }

  bool CanExecuteMe() override { return true; }

  BehaviorState Update() override { return behavior_state_; }

  void Reset() override { behavior_state_ = BehaviorState::IDLE; }
};

}


#endif //ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_STRATEGY_MOVE_STRATEGYS_EXPLORE_MOVE_STRATEGY_H_
