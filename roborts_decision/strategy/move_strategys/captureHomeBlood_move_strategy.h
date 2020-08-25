//
// Created by lq on 2020/7/25.
//

#ifndef ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_STRATEGY_MOVE_STRATEGYS_CAPTUREHOMEBLOOD_MOVE_STRATEGY_H_
#define ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_STRATEGY_MOVE_STRATEGYS_CAPTUREHOMEBLOOD_MOVE_STRATEGY_H_

#include "../abstract_common_strategy.h"

namespace roborts_decision {

class CaptureOurBloodStrategy : public AbstractCommonStrategy {
 public:
  CaptureOurBloodStrategy(std::shared_ptr<MyRobot> p_my_robot, std::shared_ptr<Blackboard> p_blackboard);

  void run() override;

  StrategyID getID() override { return StrategyID::captureHomeBloodMove; }

  bool CanExecuteMe() override;

  BehaviorState Update() override { return behavior_state_; }

  void Reset() override { behavior_state_ = BehaviorState::IDLE; }

 private:
  bool JudgeBloodZoneIsOurs(const BuffZoneStatus &bzs);
};

}

#endif //ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_STRATEGY_MOVE_STRATEGYS_CAPTUREHOMEBLOOD_MOVE_STRATEGY_H_
