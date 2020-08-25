//
// Created by wpy on 2020/8/2.
//

#include "keepDirection_attack_move_strategy.h"

roborts_decision::KeepDirectionAttackMoveStrategy::KeepDirectionAttackMoveStrategy(const std::shared_ptr<MyRobot> &pMyRobot, const std::shared_ptr<Blackboard> &pBlackboard)
  :  AbstractCommonStrategy(pMyRobot,pBlackboard)
{}

void roborts_decision::KeepDirectionAttackMoveStrategy::run(){

  roborts_common::Point2D tarPoint = this->p_blackboard_->GetAttackRunningPoint(this->p_my_robot_,
                                                                                this->p_blackboard_->GetAnotherRobot(
                                                                                    this->p_my_robot_)->getNextPoint());
  this->p_my_robot_->setNextPoint(tarPoint);
  const auto &enemy = p_blackboard_->GetCloserEnemyRobot(this->p_my_robot_);
  std::cout<<"robot:" << this->p_my_robot_->GetId() << "start execute action "<<std::endl;
  this->p_my_robot_->keepDirectionMove(tarPoint.X(), tarPoint.Y(), this->p_my_robot_->getSafeDirection(enemy.GetPose()));
}


bool roborts_decision::KeepDirectionAttackMoveStrategy::CanExecuteMe() {
  double home_state_1 = this->p_my_robot_->getState();
  double home_state_2 = this->p_blackboard_->GetAnotherRobot(this->p_my_robot_)->getState();
  double enemy_state_1 = this->p_blackboard_->GetEnemyRobot1().getState();
  double enemy_state_2 = this->p_blackboard_->GetEnemyRobot2().getState();
  const double kI = 0.95;

  const int kBullets = this->p_my_robot_->getRemainingProjectiles();

  if(home_state_1 + home_state_2 >= (enemy_state_1 + enemy_state_2) * kI && kBullets >  0){
    return true;
  }else{
    return false;
  }

  return false;
}
roborts_decision::BehaviorState roborts_decision::KeepDirectionAttackMoveStrategy::Update(){
  double home_state_1 = this->p_my_robot_->getState();
  double home_state_2 = this->p_blackboard_->GetAnotherRobot(this->p_my_robot_)->getState();
  double enemy_state_1 = this->p_blackboard_->GetEnemyRobot1().getState();
  double enemy_state_2 = this->p_blackboard_->GetEnemyRobot2().getState();
  const double kI = 0.95;

  const int kBullets = this->p_my_robot_->getRemainingProjectiles();
  BehaviorState state = roborts_decision::BehaviorState::IDLE;

  if (kBullets <= 0){
    state = roborts_decision::BehaviorState::FAILURE;
  }else if (home_state_1 + home_state_2 < (enemy_state_1 + enemy_state_2) * kI){
    state = roborts_decision::BehaviorState::FAILURE;
  }else{
    state = roborts_decision::BehaviorState::RUNNING;
  }

  this->behavior_state_ = state;

  return state;
}

void roborts_decision::KeepDirectionAttackMoveStrategy::Reset(){

  ROS_ERROR("StrategyID  KeepDirectionAttackMoveStrategy has reset");
  this->behavior_state_ = BehaviorState::IDLE;
}


