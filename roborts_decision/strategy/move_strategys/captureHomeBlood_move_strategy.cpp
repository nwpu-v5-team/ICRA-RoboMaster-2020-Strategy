//
// Created by lq on 2020/7/25.
//

#include "captureHomeBlood_move_strategy.h"

using namespace roborts_decision;


CaptureOurBloodStrategy::CaptureOurBloodStrategy(
    std::shared_ptr<MyRobot> p_my_robot,
    std::shared_ptr<Blackboard> p_blackboard)
    : AbstractCommonStrategy(p_my_robot, p_blackboard)
{}

/*
 * 吃我方加成区血包
 * 仅获取我方血包的位置，没有判断血包是否有效，可以有外部判断
 */
void CaptureOurBloodStrategy::run(){

    //获取目标点
    double tarX{0},tarY{0};
    for (auto zero : Field::GetField()->GetBuffZoneStatus()){
        if(this->JudgeBloodZoneIsOurs(zero.first)){
            tarX = zero.second.GetZoneX();
            tarY = zero.second.GetZoneY();
        }
    }

    if(tarX == 0 && tarY == 0){
        std::cerr << "captureBlood_move_strategy error: don't have our restoration Zone" << std::endl;
    }

    //赋动作
    /*if (this->p_blackboard_->GetCommonMoveType(this->p_my_robot_) == RobotBehavior::avoidanceMove) {
        this->p_my_robot_->avoidanceMove(tarX, tarY);
    } else */
    if (this->p_blackboard_->GetCommonMoveType(this->p_my_robot_) == RobotBehavior::KEEP_DIRECTION_MOVE) {
      const auto &enemy = p_blackboard_->GetCloserEnemyRobot(this->p_my_robot_);
      this->p_my_robot_->keepDirectionMove(tarX, tarY, this->p_my_robot_->getSafeDirection(enemy.GetPose()));
    } else {
        std::cerr << "captureBlood_move_strategy error: moveType error" << std::endl;
    }
}

bool  CaptureOurBloodStrategy::JudgeBloodZoneIsOurs(const BuffZoneStatus &bzs){

    MyColor mc = p_blackboard_->GetMyColor();

    if (mc == RED && bzs.buff_status == RED_RESTORATION){
        return true;
    }else if(mc == BLUE && bzs.buff_status == BLUE_RESTORATION){
        return true;
    }else{
        return false;
    }
    return false;
}

bool CaptureOurBloodStrategy::CanExecuteMe() {

  BuffStatus buff_status = BuffStatus::UNKNOWN_STATUS;
  if (p_blackboard_->GetMyColor() == MyColor::RED) {
    buff_status = BuffStatus::RED_RESTORATION;
  } else if (p_blackboard_->GetMyColor() == MyColor::BLUE) {
    buff_status = BuffStatus::BLUE_RESTORATION;
  } else {
    ROS_ERROR("Don't know my color!");
    return false;
  }

  /// 未找到对应buff
  roborts_common::Zone buff_zone{0, 0, 0, 0};
  const auto p_field = Field::GetField();
  for (const auto &buff : p_field->GetBuffZoneStatus()) {
    if(buff.first.buff_status == buff_status){
      buff_zone = buff.second;
    }
  }
  if(buff_zone.max_x_ < 0.001 && buff_zone.min_x_ < 0.001) return false;
  roborts_common::Point2D buff_point{buff_zone.max_x_ + buff_zone.min_x_, buff_zone.max_y_ + buff_zone.min_y_};

  /// 没激活
  if(!Field::GetField()->IsZoneActive(buff_status)) return false;

  /// 队友要去
  const auto &partner_tar = p_blackboard_->GetAnotherRobot(p_my_robot_)->getNextPoint();
  if(partner_tar.X() < 0.01 && partner_tar.Y() < 0.01
      && roborts_common::PointDistance(partner_tar, buff_point) < 0.5) {
    return false;
  }

  /// 敌方阻挡
  const auto &my_point = p_my_robot_->getMyRobotPoint();
  roborts_common::LineSegment2D path_line{my_point, buff_point};
  for (const auto &enemy : p_blackboard_->GetEnemyPolygon()) {
    if(roborts_common::DistanceSegmentToPolygon2D(path_line, enemy) < data::ROBOTSIZE / 2
        && roborts_common::DistancePointToPolygon2D(my_point, enemy) < data::ROBOTSIZE * 3)
      return false;
  }

  /// 血少要去
  if(p_my_robot_->getHp() < data::HP / 3) return true;

  /// 靠近要去
  if(p_my_robot_->getHp() < (data::HP - data::BLOODPACKAGE)
      && roborts_common::PointDistance(buff_point, my_point) < data::ROBOTSIZE * 3)
    return true;

  /// 默认不去
  return false;
}
