//
// Created by lq on 2020/7/25.
//

#include "toBetterOpp_shoot_strategy.h"

roborts_decision::ToBetterOppShootStrategy::ToBetterOppShootStrategy(
    const std::shared_ptr<MyRobot>& pMyRobot,
    const std::shared_ptr<Blackboard>& pBlackboard)
    : AbstractCommonStrategy(pMyRobot, pBlackboard) {
}

void roborts_decision::ToBetterOppShootStrategy::run() {
  this->behavior_state_ = BehaviorState::RUNNING;
  // const EnemyRobot& enemyRobot =
  //     (this->p_blackboard_->GetEnemyRobot1().getState() >
  //      this->p_blackboard_->GetEnemyRobot2().getState())
  //         ? this->p_blackboard_->GetEnemyRobot1()
  //         : this->p_blackboard_->GetEnemyRobot2();

  if (this->GetBetterTarget() == ENEMY_ROBOT_1) {
    const EnemyRobot& enemy_robot = this->p_blackboard_->GetEnemyRobot1();
    this->p_my_robot_->SetShootEnemyId(ENEMY_ROBOT_1);
    this->p_my_robot_->SetShootNulletsNumber(0);
    this->p_my_robot_->TowardWPosShoot(enemy_robot.GetPose().pose.position.x,
                                       enemy_robot.GetPose().pose.position.y,
                                       enemy_robot.GetPose().pose.position.z);
    this->p_my_robot_->shoot(0);
  } else if (this->GetBetterTarget() == ENEMY_ROBOT_2) {
    const EnemyRobot& enemy_robot = this->p_blackboard_->GetEnemyRobot2();
    this->p_my_robot_->SetShootEnemyId(ENEMY_ROBOT_2);
    this->p_my_robot_->SetShootNulletsNumber(0);
    this->p_my_robot_->TowardWPosShoot(enemy_robot.GetPose().pose.position.x,
                                       enemy_robot.GetPose().pose.position.y,
                                       enemy_robot.GetPose().pose.position.z);
    this->p_my_robot_->shoot(0);
  } else {
    this->behavior_state_ = BehaviorState::FAILURE;
    return;
  }

  // const auto& enemyPos = enemy_robot.GetPose().pose.position;
  // const auto& enemyOrientation = enemy_robot.GetPose().pose.orientation;
  //
  // const double kSpeed = this->p_blackboard_->GetShootSpeed(
  //      this->p_my_robot_, roborts_common::Point2D(enemyPos.x, enemyPos.y));
  // this->p_my_robot_->SetShootEnemyId(enemy_robot.GetId());
  // this->p_my_robot_->SetShootNulletsNumber(this->p_my_robot_->shootBulletsMaxNumber(kSpeed));
  // this->p_my_robot_->TowardWPosShoot(enemyPos.x, enemyPos.y, enemyPos.z);
  // this->p_my_robot_->shoot(kSpeed);
}

bool roborts_decision::ToBetterOppShootStrategy::CanExecuteMe() {
  return this->GetBetterTarget() != 0;
}

void roborts_decision::ToBetterOppShootStrategy::Reset() {
  this->behavior_state_ = BehaviorState::IDLE;
  ROS_ERROR("StrategyID ToBetterOppShoot has reset!");
}

roborts_decision::BehaviorState
roborts_decision::ToBetterOppShootStrategy::Update() {
  if (this->behavior_state_ == BehaviorState::FAILURE) {
    this->behavior_state_ = BehaviorState::IDLE;
    return BehaviorState::FAILURE;
  } else if (this->GetBetterTarget() == 0) {
    this->behavior_state_ = BehaviorState::IDLE;
    return BehaviorState::FAILURE;
  }

  return this->behavior_state_;
}

int roborts_decision::ToBetterOppShootStrategy::GetBetterTarget() {
  roborts_common::Point2D my_robot(this->p_my_robot_->getMyRobotPoint());

  if (this->p_blackboard_->GetDistanceWithObstacles(
          my_robot, this->p_blackboard_->GetEnemyPolygon()[0]) >
          static_cast<double>(data::MAX - 1) and
      this->p_blackboard_->GetDistanceWithObstacles(
          my_robot, this->p_blackboard_->GetEnemyPolygon()[1]) >
          static_cast<double>(data::MAX - 1)) {
    return false;
  } else if (this->p_blackboard_->GetDistanceWithObstacles(
                 my_robot, this->p_blackboard_->GetEnemyPolygon()[0]) <
             this->p_blackboard_->GetDistanceWithObstacles(
                 my_robot, this->p_blackboard_->GetEnemyPolygon()[1])) {
    return ENEMY_ROBOT_1;
  } else {
    return ENEMY_ROBOT_2;
  }
}
