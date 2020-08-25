//
// Created by lq on 2020/8/25.
//

#include "explore_move_strategy.h"
#include <random>

void roborts_decision::ExploreMoveStrategy::run() {

  std::vector<roborts_common::Point2D> explore_point = {{1.21, 1.87},
                                                        {1.29, -1.71},
                                                        {-1.27, -1.78},
                                                        {-1.45, 1.72}};

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 3);
  const auto &tar_point = explore_point.at(dis(gen));

  this->p_my_robot_->setNextPoint(tar_point);
  const auto &enemy = p_blackboard_->GetCloserEnemyRobot(this->p_my_robot_);

  ROS_INFO("Robot %d move target point is (%lf, %lf).", p_my_robot_->GetId(), tar_point.X(), tar_point.Y());

  std::cout<<"robot:" << this->p_my_robot_->GetId() << "start execute action "<<std::endl;
  this->p_my_robot_->keepDirectionMove(tar_point.X(), tar_point.Y(), this->p_my_robot_->getSafeDirection(enemy.GetPose()));
}
