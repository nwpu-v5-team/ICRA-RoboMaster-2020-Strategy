//
// Created by lq on 2020/7/25.
//

#include "toWorseOpp_shoot_strategy.h"

roborts_decision::ToWorseOppShootStrategy::ToWorseOppShootStrategy(const std::shared_ptr<MyRobot> &pMyRobot, const std::shared_ptr<Blackboard> &pBlackboard)
  :AbstractCommonStrategy(pMyRobot, pBlackboard){

}

void roborts_decision::ToWorseOppShootStrategy::run(){

  // 获取需要打击的机器人
  const EnemyRobot& worseEnemyToAttack = [this](){
    if (this->p_blackboard_->GetEnemyRobot1().getState() > this->p_blackboard_->GetEnemyRobot2().getState()){
      return this->p_blackboard_->GetEnemyRobot2();
    }
    else {
      return this->p_blackboard_->GetEnemyRobot1();
    }
  }();

  /**
    * 评估函数，评估具体的射击速度，射击子弹数量
    * 1. 考虑敌方机器人位姿
    * 1.1 敌方机器人为倾斜角跑位
    * 1.1.1 根据距离设置射击速度，距离越大，速度越小
    * 1.1.2 如果姿态固定，尽可能多射击
    * 1.2 敌方机器人为旋转跑
    * 1.2.1 尽可能降低射击速度
    * 1.2.2 如果可以捕捉装甲板，多射击，否则点射
    *
    * 2. 不考虑敌方位姿
    * 2.1 根据距离进行判断，距离近射击速度越大
    * 2.2 射击数量均匀
    *
    * 注：暂选方案一
    *
    */

  const auto& enemyPos = worseEnemyToAttack.GetPose().pose.position;
  const double kSpeed = this->p_blackboard_->GetShootSpeed(this->p_my_robot_, worseEnemyToAttack.getEnemyPoint());

  this->p_my_robot_->SetShootEnemyId(worseEnemyToAttack.GetId());
  this->p_my_robot_->SetShootNulletsNumber(this->p_my_robot_->shootBulletsMaxNumber(kSpeed));

  this->p_my_robot_->TowardWPosShoot(enemyPos.x, enemyPos.y, enemyPos.z);
  this->p_my_robot_->shoot(kSpeed);
}



