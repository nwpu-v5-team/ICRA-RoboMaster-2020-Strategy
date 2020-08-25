//
// Created by lq on 2020/7/25.
//

#ifndef ICRA_FIREFLY_ROBORTS_CAPTUREBULLETS_MOVE_STRATEGY_H
#define ICRA_FIREFLY_ROBORTS_CAPTUREBULLETS_MOVE_STRATEGY_H

#include "../abstract_common_strategy.h"

namespace roborts_decision {

class CaptureBulletsMoveStrategy : public AbstractCommonStrategy {
 public:
  CaptureBulletsMoveStrategy(const std::shared_ptr<MyRobot> &pMyRobot,
                             const std::shared_ptr<Blackboard> &pBlackboard);

  void run() override;

  StrategyID getID() override { return StrategyID::captureBulletsMove; }

  BehaviorState Update() override;

  void Reset() override;

  bool CanExecuteMe() override;

 private:
  int init_blood{0};


  double GetDistanceBetweenRobotAndBuff(BuffStatus buff_status);

  bool IsLossBloodQuick();

};

}



#endif //ICRA_FIREFLY_ROBORTS_CAPTUREBULLETS_MOVE_STRATEGY_H
