//
// Created by wpy on 2020/8/15.
//

#include "strategy_execute.h"

using namespace roborts_decision;

std::shared_ptr<StrategyExecute> StrategyExecute::p_strategy_execute_ = nullptr;

StrategyExecute::StrategyExecute(const ros::NodeHandle &nh) :nh_(nh) {
  this->p_blackboard_ = Blackboard::GetBlackboard();

  next_decision_sub_ = nh_.subscribe<std_msgs::Int32>(
      "action_id", 1, &StrategyExecute::NextDecisionCallBack, this);


}

void StrategyExecute::ChooseStrategyToExceed(const std::shared_ptr<MyRobot> &p_my_robot, const int data){

  const auto chassis_action = data % 6;
  int gimbal_action = data / 6;
  std::shared_ptr<AbstractCommonStrategy> p_common_strategy;

  // 操作底盘

  // 如果无法移动, 直接调用无法移动函数
  if (p_my_robot->isNoMove()){
    ROS_WARN("Robot %d No move!", p_my_robot->GetId());
    p_common_strategy = std::make_shared<StopMove>(p_my_robot, this->p_blackboard_);
    p_common_strategy->run();
  }else{
    ROS_WARN("Robot %d StrategyID is %d", p_my_robot->GetId(), chassis_action);
    switch (static_cast<StrategyID>(chassis_action)) {
      case StrategyID::stopMove: {
        p_common_strategy = std::make_shared<StopMove>(p_my_robot, this->p_blackboard_);
        p_common_strategy->run();
        break;
      }
      case StrategyID::keepDirectionAttackMove: {
        p_common_strategy = std::make_shared<KeepDirectionAttackMoveStrategy>(p_my_robot, this->p_blackboard_);
        p_common_strategy->run();
        break;
      }
      case StrategyID::keepDirectionDefendMove: {
        p_common_strategy = std::make_shared<KeepDirectionDefenseMoveStrategy>(p_my_robot, this->p_blackboard_);
        p_common_strategy->run();
        break;
      }
      case StrategyID::captureHomeBloodMove: {
        p_common_strategy = std::make_shared<CaptureOurBloodStrategy>(p_my_robot, this->p_blackboard_);
        p_common_strategy->run();
        break;
      }
      case StrategyID::captureEnemyBloodMove: {
        p_common_strategy = std::make_shared<CaptureEnemyBloodMoveStrategy>(p_my_robot, this->p_blackboard_);
        p_common_strategy->run();
        break;
      }
      case StrategyID::captureBulletsMove: {
        p_common_strategy = std::make_shared<CaptureBulletsMoveStrategy>(p_my_robot, this->p_blackboard_);
        p_common_strategy->run();
        break;
      }
      default:
        std::cout<< "解析失败"<<std::endl;
    }
  }

  // 操纵云台
  gimbal_action += 6;
  if (p_my_robot->isNoShoot()){
    p_common_strategy = std::make_shared<StopShootStrategy>(p_my_robot, this->p_blackboard_);
    p_common_strategy->run();
  }else{
    switch (static_cast<StrategyID>(gimbal_action)) {
      case StrategyID::stopShoot: {
        p_common_strategy = std::make_shared<StopShootStrategy>(p_my_robot, this->p_blackboard_);
        p_common_strategy->run();
        break;
      }
      case StrategyID::toBetterOppShoot: {
        p_common_strategy = std::make_shared<ToBetterOppShootStrategy>(p_my_robot, this->p_blackboard_);
        p_common_strategy->run();
        break;
      }
      case StrategyID::toWorseOppShoot: {
        p_common_strategy = std::make_shared<ToWorseOppShootStrategy>(p_my_robot, this->p_blackboard_);
        p_common_strategy->run();
        break;
      }
      default:
        std::cout<< "解析失败"<<std::endl;
    }
  }
}

void StrategyExecute::NextDecisionCallBack(const std_msgs::Int32::ConstPtr &msg) {
  //
  // 17 = 5(共11个底层动作策略,从0开始,到5) + 2(共3个云台动作,从0开始到2,所以后来switch需要加上6) * 6
  // 两个机器人总共17*18+17中情况
  //
  const auto all_data = msg->data;
  const auto operate_robot_1_data = all_data  % 18;
  const auto operate_robot_2_data = all_data  / 18;
  std::cout << "all_data:" << all_data << std::endl;
  std::cout << "operate_robot_1_data :" << operate_robot_1_data  << std::endl;
  std::cout << "operate_robot_2_data:" << operate_robot_2_data << std::endl;

  this->ChooseStrategyToExceed(p_blackboard_->GetMyRobot1(), operate_robot_1_data);
  std::cout << "robot_1 action is chosen and executed"<<std::endl;
  this->ChooseStrategyToExceed(p_blackboard_->GetMyRobot2(), operate_robot_2_data);
  std::cout << "robot_2 action is chosen and executed"<<std::endl;
}

std::shared_ptr<StrategyExecute>  StrategyExecute::GetStrategyExecute(){
  if (p_strategy_execute_ == nullptr){
    p_strategy_execute_ = std::make_shared<StrategyExecute>();
  }

  return p_strategy_execute_;
}
