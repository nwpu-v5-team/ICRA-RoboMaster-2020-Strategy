//
// Created by lq on 2020/7/25.
//

#include "keepDirection_defense_move_strategy.h"

roborts_decision::KeepDirectionDefenseMoveStrategy::KeepDirectionDefenseMoveStrategy(const std::shared_ptr<MyRobot> &p_my_robot,
                                                                                     const std::shared_ptr<Blackboard> &p_blackboard)
        : AbstractCommonStrategy(p_my_robot, p_blackboard) {}

void roborts_decision::KeepDirectionDefenseMoveStrategy::run() {

  roborts_common::Point2D tar_point = this->p_blackboard_->GetDefenseRunningPoint(this->p_my_robot_,
                                                                                  this->p_blackboard_->GetAnotherRobot(
                                                                                      this->p_my_robot_)->getNextPoint());
  this->p_my_robot_->setNextPoint(tar_point);
  const auto &enemy = p_blackboard_->GetCloserEnemyRobot(this->p_my_robot_);

  ROS_INFO("Robot %d move target point is (%lf, %lf).", p_my_robot_->GetId(), tar_point.X(), tar_point.Y());

  std::cout<<"robot:" << this->p_my_robot_->GetId() << "start execute action "<<std::endl;
  this->p_my_robot_->keepDirectionMove(tar_point.X(), tar_point.Y(), this->p_my_robot_->getSafeDirection(enemy.GetPose()));
}
