//
// Created by lq on 2020/7/25.
//

/**
 * 选取合适的点
 * 1. 射击分打强的和不强的，跑应与此有对应（
 * 2. 跑分为防御跑点和进攻跑点（依据障碍物防御或避开障碍物进攻）
 * 3. 跑点应考虑与自身位置的关系(从自身出发的扩展树或遗产算法）
 * 4. 跑点应考虑同伴（优先分散）
 */

#ifndef ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_STRATEGY_MOVE_STRATEGYS_KEEPDIRECTION_DEFENSE_MOVE_STRATEGY_H_
#define ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_STRATEGY_MOVE_STRATEGYS_KEEPDIRECTION_DEFENSE_MOVE_STRATEGY_H_

#include "../abstract_common_strategy.h"

namespace roborts_decision {

class KeepDirectionDefenseMoveStrategy : public AbstractCommonStrategy {

public:
  KeepDirectionDefenseMoveStrategy(const std::shared_ptr<MyRobot> &p_my_robot,
                                     const std::shared_ptr<Blackboard> &p_blackboard);

  void run() override;

  StrategyID getID() override {return StrategyID::keepDirectionDefendMove;}

  bool CanExecuteMe() override { return true; }

  BehaviorState Update() override { return behavior_state_; }

  void Reset() override { behavior_state_ = BehaviorState::IDLE; }
};

}


#endif //ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_STRATEGY_MOVE_STRATEGYS_KEEPDIRECTION_DEFENSE_MOVE_STRATEGY_H_
