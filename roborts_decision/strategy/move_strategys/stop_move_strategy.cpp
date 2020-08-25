//
// Created by lq on 2020/7/26.
//

#include "stop_move_strategy.h"

roborts_decision::StopMove::StopMove(
    const std::shared_ptr<MyRobot> &pMyRobot,
    const std::shared_ptr<Blackboard> &pBlackboard)
    : AbstractCommonStrategy(pMyRobot, pBlackboard) {
}

void roborts_decision::StopMove::run() {
  this->p_my_robot_->stayStill();

  this->behavior_state_ = BehaviorState::RUNNING;
}

bool roborts_decision::StopMove::CanExecuteMe() {
  return this->p_my_robot_->isNoMove();
}

roborts_decision::BehaviorState roborts_decision::StopMove::Update() {
  if (!this->p_my_robot_->isNoMove()) {
    this->behavior_state_ = BehaviorState::IDLE;
    return BehaviorState::SUCCESS;
  }

  return BehaviorState::RUNNING;
}

void roborts_decision::StopMove::Reset() {
  ROS_ERROR("StrategyID NoMove has reset!");
  this->behavior_state_ = BehaviorState::IDLE;
}