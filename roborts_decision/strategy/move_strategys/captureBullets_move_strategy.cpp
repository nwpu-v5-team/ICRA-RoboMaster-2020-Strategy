//
// Created by lq on 2020/7/25.
//

#include "captureBullets_move_strategy.h"

using namespace roborts_decision;

CaptureBulletsMoveStrategy::CaptureBulletsMoveStrategy(
    const std::shared_ptr<MyRobot> &pMyRobot,
    const std::shared_ptr<Blackboard> &pBlackboard)
    : AbstractCommonStrategy(pMyRobot, pBlackboard) {
}

void CaptureBulletsMoveStrategy::run() {
  double tar_x = -1;
  double tar_y = -1;

  this->behavior_state_ = BehaviorState::RUNNING;

  MyColor myColor = this->p_blackboard_->GetMyColor();

  for (auto zone : Field::GetField()->GetBuffZoneStatus()) {
    // TODO：此处应用正方形判断方式,但此处暂时使用中心点
    if (myColor == MyColor::RED &&
        zone.first.buff_status == BuffStatus::RED_PROJECTILE_SUPPLIER) {
      tar_x = zone.second.GetZoneX();
      tar_y = zone.second.GetZoneY();
      break;
    } else if (myColor == MyColor::BLUE &&
               zone.first.buff_status == BuffStatus::BLUE_PROJECTILE_SUPPLIER) {
      tar_x = zone.second.GetZoneX();
      tar_y = zone.second.GetZoneY();
      break;
    }
  }

  if (tar_x == -1 && tar_y == -1) {
    std::cerr << "captureBullets_move_strategy error: don't have our "
                 "projectile supplier Zone"
              << std::endl;
  }

  /*if (this->p_blackboard_->GetCommonMoveType(this->p_my_robot_) ==
  RobotBehavior::avoidanceMove) { this->p_my_robot_->avoidanceMove(tar_x,
  tar_y); } else*/
  if (this->p_blackboard_->GetCommonMoveType(this->p_my_robot_) ==
      RobotBehavior::KEEP_DIRECTION_MOVE) {
    const auto &enemy = p_blackboard_->GetCloserEnemyRobot(this->p_my_robot_);
    this->p_my_robot_->keepDirectionMove(
        tar_x, tar_y, this->p_my_robot_->getSafeDirection(enemy.GetPose()));
  } else {
    std::cerr << "captureBullets_move_strategy error: moveType error"
              << std::endl;
  }
}

BehaviorState CaptureBulletsMoveStrategy::Update() {
  // TODO Need to add adjust the point is same ?
  if (this->IsLossBloodQuick()) {
    this->behavior_state_ = BehaviorState::IDLE;
    return BehaviorState::FAILURE;
  } else if (!Field::GetField()->IsZoneActive(RED_PROJECTILE_SUPPLIER) and
             this->p_blackboard_->GetMyColor() == RED) {
    this->behavior_state_ = BehaviorState::IDLE;
    return BehaviorState::SUCCESS;
  } else if (!Field::GetField()->IsZoneActive(BLUE_PROJECTILE_SUPPLIER) and
             this->p_blackboard_->GetMyColor() == BLUE) {
    this->behavior_state_ = BehaviorState::IDLE;
    return BehaviorState::SUCCESS;
  } else {
    this->behavior_state_ = BehaviorState::RUNNING;
    return BehaviorState::RUNNING;
  }
}

bool CaptureBulletsMoveStrategy::CanExecuteMe() {
  this->init_blood = this->p_my_robot_->getHp();
  if (!Field::GetField()->IsZoneActive(RED_PROJECTILE_SUPPLIER) and
      this->p_blackboard_->GetMyColor() == RED) {
    return false;
  } else if (!Field::GetField()->IsZoneActive(BLUE_PROJECTILE_SUPPLIER) and
             this->p_blackboard_->GetMyColor() == BLUE) {
    return false;
  }

  if (this->GetDistanceBetweenRobotAndBuff(RED_PROJECTILE_SUPPLIER) <= 2.3 and
      this->p_blackboard_->GetMyColor() == RED) {
    return true;
  } else if (this->GetDistanceBetweenRobotAndBuff(BLUE_PROJECTILE_SUPPLIER) <=
                 2.3 and
             this->p_blackboard_->GetMyColor() == BLUE) {
    return true;
  } else if (this->p_my_robot_->getRemainingProjectiles() < 50) {
    return true;
  }

  return false;
}

double CaptureBulletsMoveStrategy::GetDistanceBetweenRobotAndBuff(
    BuffStatus buff_status) {
  double tar_x = 0;
  double tar_y = 0;
  for (auto buff_zone_status : Field::GetField()->GetBuffZoneStatus()) {
    if (buff_zone_status.first.buff_status == buff_status) {
      tar_x = buff_zone_status.second.GetZoneX();
      tar_y = buff_zone_status.second.GetZoneY();
      break;
    }
  }

  return roborts_common::PointDistance(this->p_my_robot_->getMyRobotPoint().X(),
                                       this->p_my_robot_->getMyRobotPoint().Y(),
                                       tar_x, tar_y);
}

bool CaptureBulletsMoveStrategy::IsLossBloodQuick() {
  return static_cast<double>(this->init_blood - this->p_my_robot_->getHp()) /
             this->init_blood <
         0.15;
}

void CaptureBulletsMoveStrategy::Reset() {
  ROS_ERROR("StrategyID CaptureBulletsMove has reset");
  this->behavior_state_ = BehaviorState::IDLE;
}
