//
// Created by lq on 2020/7/25.
//

#include "captureEnemyBlood_move_strategy.h"

using namespace roborts_decision;

CaptureEnemyBloodMoveStrategy::CaptureEnemyBloodMoveStrategy(
        std::shared_ptr<MyRobot> _p_my_robot_,
        std::shared_ptr<Blackboard> _p_blackboard_)
        : AbstractCommonStrategy(_p_my_robot_, _p_blackboard_) {
}

/*
 * 吃敌方加成区血包
 * 仅获取敌方血包的位置，没有判断血包是否有效，可以有外部判断
 */
void CaptureEnemyBloodMoveStrategy::run(){
  this->behavior_state_ = BehaviorState::RUNNING;
  //获取目标点
    double tarX{0},tarY{0};
    for (auto zero : Field::GetField()->GetBuffZoneStatus()){
        if(this->judgeBloodZoneIsEnemy(zero.first)){
            tarX = zero.second.GetZoneX();
            tarY = zero.second.GetZoneY();
        }
    }

    if(tarX == 0 && tarY == 0){
        std::cerr << "captureBlood_move_strategy error: don't have enemy restoration Zone" << std::endl;
    }

    //赋动作
/*    if (this->p_blackboard_->GetCommonMoveType(this->p_my_robot_) == RobotBehavior::avoidanceMove) {
        this->p_my_robot_->avoidanceMove(tarX, tarY);
    } else */
    if (this->p_blackboard_->GetCommonMoveType(this->p_my_robot_) == RobotBehavior::KEEP_DIRECTION_MOVE) {
      const auto &enemy = p_blackboard_->GetCloserEnemyRobot(this->p_my_robot_);
      this->p_my_robot_->keepDirectionMove(tarX, tarY, this->p_my_robot_->getSafeDirection(enemy.GetPose()));
    } else {
        std::cerr << "captureBlood_move_strategy error: moveType error" << std::endl;
    }
}

/*
 * 判断是否是敌方血包
 * 没有判断血包是否有效，也没有判断血包是否可以吃
 */
bool CaptureEnemyBloodMoveStrategy::judgeBloodZoneIsEnemy(const BuffZoneStatus &bzs){

    MyColor homeColor = p_blackboard_->GetMyColor();

    // TODO 暂时先用此方法，，后期再改
    auto getOppColor = [=]()->MyColor{
        if(homeColor == RED){
            return BLUE;
        }else{
            return RED;
        }
    };
    MyColor oppColor= getOppColor();

    if (oppColor == RED && bzs.buff_status == RED_RESTORATION) {
      return true;
    } else if (oppColor == BLUE && bzs.buff_status == BLUE_RESTORATION) {
      return true;
    } else {
      return false;
    }
    return false;
}

BehaviorState CaptureEnemyBloodMoveStrategy::Update() {
  return BehaviorState::IDLE;
}

void CaptureEnemyBloodMoveStrategy::Reset() {
  ROS_ERROR("StrategyID CaptureEnemyBloodMove has reset!");
  this->behavior_state_ = BehaviorState::IDLE;
}

bool CaptureEnemyBloodMoveStrategy::CanExecuteMe() { return false; }
