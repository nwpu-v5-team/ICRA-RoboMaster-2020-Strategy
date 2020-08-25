//
// Created by lq on 2020/7/25.
//

#ifndef ICRA_FIREFLY_ROBORTS_CAPTUREENEMYBLOOD_MOVE_STRATEGY_H
#define ICRA_FIREFLY_ROBORTS_CAPTUREENEMYBLOOD_MOVE_STRATEGY_H

#include "../abstract_common_strategy.h"

namespace roborts_decision {
class CaptureEnemyBloodMoveStrategy : public AbstractCommonStrategy {
 public:
  CaptureEnemyBloodMoveStrategy(std::shared_ptr<MyRobot> _p_my_robot_,
                                std::shared_ptr<Blackboard> _p_blackboard_);

  void run() override;

  StrategyID getID() override { return StrategyID::captureEnemyBloodMove; };

  BehaviorState Update() override;

  void Reset() override;

  bool CanExecuteMe() override;

 private:

  bool judgeBloodZoneIsEnemy(const BuffZoneStatus &bzs);
};
}  // namespace roborts_decision

#endif  // ICRA_FIREFLY_ROBORTS_CAPTUREENEMYBLOOD_MOVE_STRATEGY_H
