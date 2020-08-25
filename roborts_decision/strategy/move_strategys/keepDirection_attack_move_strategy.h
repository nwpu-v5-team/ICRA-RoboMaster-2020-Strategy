//
// Created by wpy on 2020/8/2.
//

#ifndef ROBOTRTS_WS_KEEPDIRECTION_ATTACK_MOVE_STRATEGY_H
#define ROBOTRTS_WS_KEEPDIRECTION_ATTACK_MOVE_STRATEGY_H

#include "../abstract_common_strategy.h"

namespace roborts_decision {

class KeepDirectionAttackMoveStrategy : public AbstractCommonStrategy{

public:
  KeepDirectionAttackMoveStrategy(const std::shared_ptr<MyRobot> &pMyRobot,
                                   const std::shared_ptr<Blackboard> &pBlackboard);

  void run() override;
  StrategyID getID() override {return StrategyID::keepDirectionAttackMove;}

  virtual bool CanExecuteMe();

  virtual BehaviorState Update();

  virtual void Reset();



};

}
#endif //ROBOTRTS_WS_KEEPDIRECTION_ATTACK_MOVE_STRATEGY_H
