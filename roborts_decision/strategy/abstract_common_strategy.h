//
// Created by wpy on 2020/7/26.
//

#ifndef ROBOTRTS_WS_ABSTRACT_COMMON_STRATEGY_H
#define ROBOTRTS_WS_ABSTRACT_COMMON_STRATEGY_H

#include "../blackboard/my_robot.h"
#include "../blackboard/blackboard.h"
#include <typeinfo>

namespace roborts_decision {

/**
 * The id of Strategy. One Id corresponds to one class.
 */
enum class StrategyID{
  stopMove,
  keepDirectionAttackMove,
  keepDirectionDefendMove,
  captureHomeBloodMove,
  captureEnemyBloodMove,
  captureBulletsMove,
  exploreMove,

  stopShoot,
  toBetterOppShoot,
  toWorseOppShoot
};

/*
 * 普通策略的抽象类，其他策略都应继承
 * 注意需要在子类定义构造函数
 */
class AbstractCommonStrategy {
public:
  AbstractCommonStrategy(std::shared_ptr<MyRobot> _p_my_robot_,
                         std::shared_ptr<Blackboard> _p_blackboard_)
      : p_my_robot_(std::move(_p_my_robot_)),
        p_blackboard_(std::move(_p_blackboard_)){
    behavior_state_ = roborts_decision::BehaviorState::IDLE;
  }

  virtual bool CanExecuteMe() = 0;

  virtual void run() = 0;

  virtual StrategyID getID() = 0;

  virtual BehaviorState Update() = 0;

  virtual void Reset() = 0;

 protected:
    std::shared_ptr<MyRobot> p_my_robot_;
    std::shared_ptr<Blackboard> p_blackboard_;
    BehaviorState behavior_state_;
};

}
#endif //ROBOTRTS_WS_ABSTRACT_COMMON_STRATEGY_H
