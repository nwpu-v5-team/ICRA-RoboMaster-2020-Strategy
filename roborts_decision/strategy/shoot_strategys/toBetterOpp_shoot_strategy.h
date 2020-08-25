//
// Created by lq on 2020/7/25.
//

#ifndef ICRA_FIREFLY_ROBORTS_TOBETTEROPP_SHOOT_STRATEGY_H
#define ICRA_FIREFLY_ROBORTS_TOBETTEROPP_SHOOT_STRATEGY_H

#include "../abstract_common_strategy.h"

namespace roborts_decision {

class ToBetterOppShootStrategy : public AbstractCommonStrategy {
 public:
  ToBetterOppShootStrategy(const std::shared_ptr<MyRobot> &pMyRobot,
                           const std::shared_ptr<Blackboard> &pBlackboard);

  void run() override;

  StrategyID getID() override { return StrategyID::toBetterOppShoot; }

  // TODO
  bool CanExecuteMe() override;

  // TODO
  BehaviorState Update() override;

  // TODO
  void Reset() override;

 private:

  int GetBetterTarget();
};

}
#endif //ICRA_FIREFLY_ROBORTS_TOBETTEROPP_SHOOT_STRATEGY_H
