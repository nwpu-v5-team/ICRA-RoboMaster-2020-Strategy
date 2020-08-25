//
// Created by lq on 2020/7/26.
//

#ifndef ICRA_FIREFLY_ROBORTS_STOP_MOVE_STRATEGY_H
#define ICRA_FIREFLY_ROBORTS_STOP_MOVE_STRATEGY_H

#include "../abstract_common_strategy.h"

namespace roborts_decision {

class StopMove : public AbstractCommonStrategy {
 public:
  StopMove(const std::shared_ptr<MyRobot> &pMyRobot,
           const std::shared_ptr<Blackboard> &pBlackboard);

  void run() override;

  StrategyID getID() override { return StrategyID::stopMove; }

  bool CanExecuteMe() override;

  BehaviorState Update() override;

  void Reset() override;

 private:
};

}



#endif //ICRA_FIREFLY_ROBORTS_STOP_MOVE_STRATEGY_H
