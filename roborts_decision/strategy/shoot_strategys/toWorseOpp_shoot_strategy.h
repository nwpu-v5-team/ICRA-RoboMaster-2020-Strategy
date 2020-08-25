//
// Created by lq on 2020/7/25.
//

#ifndef ICRA_FIREFLY_ROBORTS_TOWORSEOPP_SHOOT_STRATEGY_H
#define ICRA_FIREFLY_ROBORTS_TOWORSEOPP_SHOOT_STRATEGY_H

#include "../abstract_common_strategy.h"

namespace roborts_decision {

class ToWorseOppShootStrategy : public AbstractCommonStrategy {

public:
  ToWorseOppShootStrategy(const std::shared_ptr<MyRobot> &pMyRobot, const std::shared_ptr<Blackboard> &pBlackboard);

  void run();

  StrategyID getID() override { return StrategyID::toWorseOppShoot;}
  // TODO
  virtual bool CanExecuteMe() {
    return true;
  }
  // TODO
  virtual BehaviorState Update(){
    return BehaviorState::SUCCESS;
  }
  // TODO
  virtual void Reset(){

  }

};
}

#endif //ICRA_FIREFLY_ROBORTS_TOWORSEOPP_SHOOT_STRATEGY_H
