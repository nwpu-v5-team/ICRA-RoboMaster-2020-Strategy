//
// Created by lq on 2020/7/25.
//

#include "stop_shoot_strategy.h"

roborts_decision::StopShootStrategy::StopShootStrategy(
    const std::shared_ptr<MyRobot> &pMyRobot,
    const std::shared_ptr<Blackboard> &pBlackboard)
    : AbstractCommonStrategy(pMyRobot, pBlackboard) {
}

void roborts_decision::StopShootStrategy::run() {
  this->behavior_state_ = BehaviorState::RUNNING;

  auto tarOppId = this->p_blackboard_->GetPrepareShootOpp(this->p_my_robot_);
  auto tarOppPose = tarOppId.GetPose().pose.position;

  // 只调整云台方位，不射击
  this->p_my_robot_->TowardWPosShoot(tarOppPose.x, tarOppPose.y, tarOppPose.z);
}

roborts_decision::BehaviorState roborts_decision::StopShootStrategy::Update() {
  if (this->p_my_robot_->getRemainingProjectiles() <= 0) {
    this->behavior_state_ = BehaviorState::RUNNING;
  } else if (this->p_my_robot_->isNoShoot()) {
    this->behavior_state_ = BehaviorState::RUNNING;
  } else if (this->GetLongerDistanceFromTwoRobot() > 5.0) {
    this->behavior_state_ = BehaviorState::RUNNING;
  } else {
    this->behavior_state_ = BehaviorState::IDLE;
    return BehaviorState::SUCCESS;
  }

  // TODO Need to add the condition
  return this->behavior_state_;
}

void roborts_decision::StopShootStrategy::Reset() {
  ROS_ERROR("StrategyID StopShoot has reset!");
  this->behavior_state_ = BehaviorState::IDLE;
}

bool roborts_decision::StopShootStrategy::CanExecuteMe() {
  if (this->p_my_robot_->isNoShoot()) {
    return true;
  } else if (this->p_my_robot_->getRemainingProjectiles() <= 0) {
    return true;
  } else if (this->GetLongerDistanceFromTwoRobot() > 5.0) {
    return true;
  } else if (this->IsObstacleOntheTrack()) {
    return true;
  }
  return false;
}

double roborts_decision::StopShootStrategy::GetLongerDistanceFromTwoRobot() {
  double enemy_distance_1 = 0;
  double enemy_distance_2 = 0;
  enemy_distance_1 = roborts_common::PointDistance(
      this->p_my_robot_->getMyRobotPoint(),
      this->p_blackboard_->GetEnemyRobot1().getEnemyPoint());
  enemy_distance_2 = roborts_common::PointDistance(
      this->p_my_robot_->getMyRobotPoint(),
      this->p_blackboard_->GetEnemyRobot2().getEnemyPoint());

  return enemy_distance_1 > enemy_distance_2 ? enemy_distance_1
                                             : enemy_distance_2;
}

bool roborts_decision::StopShootStrategy::IsObstacleOntheTrack() {
  return this->p_blackboard_->GetDistanceWithObstacles(
             this->p_my_robot_->getMyRobotPoint(),
             this->p_blackboard_->GetEnemyPolygon()[0]) == data::MAX or
         this->p_blackboard_->GetDistanceWithObstacles(
             this->p_my_robot_->getMyRobotPoint(),
             this->p_blackboard_->GetEnemyPolygon()[1]) == data::MAX;
}
