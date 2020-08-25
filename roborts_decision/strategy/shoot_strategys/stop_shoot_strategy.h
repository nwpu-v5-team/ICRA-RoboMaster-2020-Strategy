//
// Created by lq on 2020/7/25.
//

#ifndef ICRA_FIREFLY_ROBORTS_STOP_SHOOT_STRATEGY_H
#define ICRA_FIREFLY_ROBORTS_STOP_SHOOT_STRATEGY_H

#include "../abstract_common_strategy.h"

namespace roborts_decision {

class StopShootStrategy : public AbstractCommonStrategy {
 public:
  StopShootStrategy(const std::shared_ptr<MyRobot> &pMyRobot,
                    const std::shared_ptr<Blackboard> &pBlackboard);

  void run() override;

  StrategyID getID() override { return StrategyID::stopShoot; }

  BehaviorState Update() override;

  void Reset() override;

  bool CanExecuteMe();

 private:

  double GetLongerDistanceFromTwoRobot();

  bool IsObstacleOntheTrack();
};

}  // namespace roborts_decision

#endif  // ICRA_FIREFLY_ROBORTS_STOP_SHOOT_STRATEGY_H
