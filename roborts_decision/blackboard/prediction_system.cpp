//
// Created by wpy on 2020/8/5.
//

#include "prediction_system.h"

using namespace roborts_decision;

PredictionSystem::PredictionSystem(std::shared_ptr<MyRobot> p_my_robot_1,std::shared_ptr<MyRobot> p_my_robot_2, EnemyRobot &enemy_robot_1 ,EnemyRobot &enemy_robot_2)
  : p_my_robot_1_(p_my_robot_1), p_my_robot_2_(p_my_robot_2), enemy_robot_1_(enemy_robot_1), enemy_robot_2_(enemy_robot_2), all_home_({p_my_robot_1, p_my_robot_2}){

  field_ = Field::GetField();

  // 预测射击的子弹数
  this->bullet_shoot_enemy_1_to_home_1_ = PredictBulletNumbersWithDistance(p_my_robot_1_->getMyRobotPoint(), enemy_robot_1.getEnemyPoint());
  this->bullet_shoot_enemy_2_to_home_1_ = PredictBulletNumbersWithDistance(p_my_robot_1_->getMyRobotPoint(), enemy_robot_2.getEnemyPoint());
  this->bullet_shoot_enemy_1_to_home_2_ = PredictBulletNumbersWithDistance(p_my_robot_2_->getMyRobotPoint(), enemy_robot_1.getEnemyPoint());
  this->bullet_shoot_enemy_2_to_home_2_ = PredictBulletNumbersWithDistance(p_my_robot_2_->getMyRobotPoint(), enemy_robot_2.getEnemyPoint());

  // 预测射击速度
  this->speed_shoot_enemy_1_to_home_1_ = this -> PredictSpeedWithDistance(p_my_robot_1_->getMyRobotPoint(), enemy_robot_1.getEnemyPoint());
  this->speed_shoot_enemy_2_to_home_1_ = this -> PredictSpeedWithDistance(p_my_robot_1_->getMyRobotPoint(), enemy_robot_2.getEnemyPoint());
  this->speed_shoot_enemy_1_to_home_2_ = this -> PredictSpeedWithDistance(p_my_robot_2_->getMyRobotPoint(), enemy_robot_1.getEnemyPoint());
  this->speed_shoot_enemy_2_to_home_2_ = this -> PredictSpeedWithDistance(p_my_robot_2_->getMyRobotPoint(), enemy_robot_2.getEnemyPoint());

}

// 获取剩余子弹数
void PredictionSystem::UpdatePredictedRemainingProjectiles(){

  auto remaining_bullets_1 = enemy_robot_1_.getRemainingProjectiles();
  auto remaining_bullets_2 = enemy_robot_2_.getRemainingProjectiles();

  for(const auto& iter : all_home_){

    // 考虑敌方射击
    // 我方机器人被攻击
    if (iter->isShot() == true) {
      std::vector<roborts_decision::EnemyRobot> enemy_to_attack_home;
      enemy_to_attack_home = PredictWhoAttackHome(iter);

      if (enemy_to_attack_home.empty() != true){
        for (const auto& k_iter : enemy_to_attack_home){
          // 被攻击的是我方一号机器人
          if (iter->GetId() == MY_ROBOT_1){
            if (k_iter == enemy_robot_1_){
              remaining_bullets_1 -= this->bullet_shoot_enemy_1_to_home_1_;
            }else if (k_iter == enemy_robot_2_){
              remaining_bullets_2 -= this->bullet_shoot_enemy_2_to_home_1_;
            }
          //
          }else if(iter->GetId() == MY_ROBOT_2){
            if (k_iter == enemy_robot_1_){
              remaining_bullets_1 -= this->bullet_shoot_enemy_1_to_home_2_;
            }else if (k_iter == enemy_robot_2_){
              remaining_bullets_2 -= this->bullet_shoot_enemy_2_to_home_2_;
            }
          }
        }
      }
    }
  }


  if (remaining_bullets_1 < 0){
    remaining_bullets_1 = 0;
  }
  if (remaining_bullets_2 < 0){
    remaining_bullets_2 = 0;
  }

  // 考虑敌方buff加成
  if (GetEnemyBulletBuffStatus() == false){
    remaining_bullets_1+=100;
    remaining_bullets_2+=100;
  }

  enemy_robot_1_.SetRemainingProjectiles(remaining_bullets_1);
  enemy_robot_2_.SetRemainingProjectiles(remaining_bullets_2);

}

// 获得剩余血量
void PredictionSystem::UpdatePredictedHp(){

  double hp_remaining_1 = enemy_robot_1_.getHp();
  double hp_remaining_2 = enemy_robot_2_.getHp();

  // 暂定每射一次,掉10滴血,之后再改,以后可以获取射击的子弹数,就能做预测
  int shoot_bullets_number;
  int damage = 0;
  if (this->p_my_robot_1_->GetShootEnemyId() == enemy_robot_1_.GetId()){
    shoot_bullets_number = this->p_my_robot_1_->GetShootNulletsNumber();
    damage = this->PredictDamageToEnemyWithBullets(shoot_bullets_number);
    hp_remaining_1 -= damage;
  } else if (this->p_my_robot_1_->GetShootEnemyId() == enemy_robot_2_.GetId()){
    shoot_bullets_number = this->p_my_robot_1_->GetShootNulletsNumber();
    damage = this->PredictDamageToEnemyWithBullets(shoot_bullets_number);
    hp_remaining_2 -= damage;
  }

  if (p_my_robot_2_->GetShootEnemyId() == enemy_robot_1_.GetId()){
    shoot_bullets_number = this->p_my_robot_2_->GetShootNulletsNumber();
    damage = this->PredictDamageToEnemyWithBullets(shoot_bullets_number);
    hp_remaining_1 -= damage;
  } else if (p_my_robot_2_->GetShootEnemyId() == enemy_robot_2_.GetId()){
    shoot_bullets_number = this->p_my_robot_2_->GetShootNulletsNumber();
    damage = this->PredictDamageToEnemyWithBullets(shoot_bullets_number);
    hp_remaining_2 -= damage;
  }

  if (GetEnemyBloodBuffStatus() == false){
    hp_remaining_1 += 200;
    hp_remaining_2 += 200;
  }
  if (hp_remaining_1 > 2000){
    hp_remaining_1 = 2000;
  }
  if (hp_remaining_2 > 2000){
    hp_remaining_2 = 2000;
  }

  enemy_robot_1_.SetHp(hp_remaining_1);
  enemy_robot_2_.SetHp(hp_remaining_2);
}

// 获得枪口热量
void PredictionSystem::UpdatePredictedCurrentHeat() {
  double heat_remaining_1 = enemy_robot_1_.getCurrentHeat();
  double heat_remaining_2 = enemy_robot_2_.getCurrentHeat();

  std::vector<roborts_decision::EnemyRobot> enemy_to_attack_home_1,enemy_to_attack_home_2;
  enemy_to_attack_home_1 = PredictWhoAttackHome(this->p_my_robot_1_);
  enemy_to_attack_home_2 = PredictWhoAttackHome(this->p_my_robot_2_);

  if (enemy_to_attack_home_1.empty() == true ){

    if(enemy_to_attack_home_2.empty() == true){
      if (enemy_robot_1_.getHp() < 400){
        heat_remaining_1 -= 24;
      }else{
        heat_remaining_1 -= 12;
      }

      if (enemy_robot_2_.getHp() < 400){
        heat_remaining_2 -= 24;
      }else{
        heat_remaining_2 -= 12;
      }
    }else{
        for (const auto& k_iter : enemy_to_attack_home_2) {
          if (k_iter == enemy_robot_1_) {
            heat_remaining_1 += this->bullet_shoot_enemy_1_to_home_2_ * roborts_decision::data::averagespeed_;
          } else if (k_iter == enemy_robot_2_) {
            heat_remaining_2 += bullet_shoot_enemy_2_to_home_2_ * roborts_decision::data::averagespeed_;
          }
        }
    }
  }else if (enemy_to_attack_home_1.empty() != true){
    if (enemy_to_attack_home_2.empty() == true){
      for (const auto& k_iter : enemy_to_attack_home_1) {
        if (k_iter == enemy_robot_1_) {
          heat_remaining_1 += bullet_shoot_enemy_1_to_home_1_ * roborts_decision::data::averagespeed_;
        } else if (k_iter == enemy_robot_2_) {
          heat_remaining_2 += bullet_shoot_enemy_2_to_home_1_ * roborts_decision::data::averagespeed_;
        }
      }
    }else{
      // 简化版本,直接一对一,二对二
      // 暂时不归一化,简单处理
      //this->NormalizedEnemyAttacker(enemy_to_attack_home_1, enemy_to_attack_home_2);
      heat_remaining_1 += bullet_shoot_enemy_1_to_home_1_ * speed_shoot_enemy_1_to_home_1_;
      heat_remaining_2 += bullet_shoot_enemy_2_to_home_2_ * speed_shoot_enemy_2_to_home_2_;
    }

  }
  enemy_robot_1_.SetCurrentHeat(heat_remaining_1);
  enemy_robot_2_.SetCurrentHeat(heat_remaining_2);




}


// 获取当前敌方子弹buff状态
bool PredictionSystem::GetEnemyBulletBuffStatus(){
  for(const auto& iter : field_->GetBuffZoneStatus()){
    if (enemy_robot_1_.GetColor() == roborts_decision::RED && iter.first.buff_status == roborts_decision::RED_PROJECTILE_SUPPLIER ){
      return iter.first.is_active;
    }else if(enemy_robot_1_.GetColor() == roborts_decision::BLUE && iter.first.buff_status == roborts_decision::BLUE_PROJECTILE_SUPPLIER){
      return iter.first.is_active;
    }else{
      // Do nothing
    }
  }

}

// 获取当前敌方血量buff状态
bool PredictionSystem::GetEnemyBloodBuffStatus(){

  for(const auto& iter :field_->GetBuffZoneStatus()){
    if (enemy_robot_1_.GetColor() == roborts_decision::RED && iter.first.buff_status == roborts_decision::RED_RESTORATION){
      return iter.first.is_active;
    }else if(enemy_robot_1_.GetColor() == roborts_decision::BLUE && iter.first.buff_status == roborts_decision::BLUE_RESTORATION){
      return iter.first.is_active;
    }else{
      // Do nothing
    }
  }
}

// 预测敌方打击我方特定机器人的机器人
std::vector<roborts_decision::EnemyRobot> PredictionSystem::PredictWhoAttackHome(const std::shared_ptr<roborts_decision::MyRobot> &p_my_robot){
  /**
   * 如果同时有两块夹板被攻击,则敌方两机器人均在攻击
   * 如果只有一块,考虑范围
   *    包括装甲板在内的 2PI/3 范围内,如果敌方机器人在内,并且与我方几何中心没有障碍物
   *    ,则应预测其正在攻击
   *    具体方法:
   *    1. 如果与2PI/3边界的两条直线夹角均小于2PI/3 则可判断位于范围内
   *    2. 位于范围内,还需要求与我方机器人之间没有障碍物
   */

  std::vector<roborts_decision::EnemyRobot> enemy_to_attack_home;

  std::vector<roborts_decision::ArmorId>armors_under_attack =  p_my_robot->getArmorsUnderAttack();

  if (armors_under_attack.size() == 0){
    // Do nothing
  }else if (armors_under_attack.size() == 2){
    enemy_to_attack_home.push_back(enemy_robot_1_);
    enemy_to_attack_home.push_back(enemy_robot_2_);
  }else{
    const auto& enemy_robot_1 = enemy_robot_1_;
    const auto& enemy_robot_2 = enemy_robot_2_;

    // 获取我方机器人与对方机器人连线角度
    const double kRelativeX1 = p_my_robot->getChassisMapPose().pose.position.x - enemy_robot_1.GetPose().pose.position.x;
    const double kRelativeY1 = p_my_robot->getChassisMapPose().pose.position.y - enemy_robot_1.GetPose().pose.position.y;

    const double kArmorAngle = p_my_robot->GetArmorTowards(armors_under_attack.at(0));
    const double kArmorX = cos(kArmorAngle);
    const double kArmorY = sin(kArmorAngle);

    const double kRelativeAngle1 = roborts_common::AngleVectorToVector(std::make_tuple(kRelativeX1, kRelativeY1), std::make_tuple(kArmorX, kArmorY));

    // 角度在-PI/3 ~ PI/3, 并且两车之间没有障碍物
    if (fabs(kRelativeAngle1) < roborts_decision::data::PI / 3){
      double dis = roborts_common::PointDistance(p_my_robot->getMyRobotPoint(), enemy_robot_1_.getEnemyPoint());
      if (dis != roborts_decision::data::MAX){
        enemy_to_attack_home.push_back(enemy_robot_1);
      }
    }

    const double kRelativeX2 = p_my_robot->getChassisMapPose().pose.position.x - enemy_robot_2.GetPose().pose.position.x;
    const double kRelativeY2 = p_my_robot->getChassisMapPose().pose.position.y - enemy_robot_2.GetPose().pose.position.y;

    const double kRelativeAngle2 = roborts_common::AngleVectorToVector(std::make_tuple(kRelativeX2, kRelativeY2), std::make_tuple(kArmorX, kArmorY));

    // 角度在-PI/3 ~ PI/3, 并且两车之间没有障碍物
    if (fabs(kRelativeAngle2) < roborts_decision::data::PI / 3){
      double dis = roborts_common::PointDistance(p_my_robot->getMyRobotPoint(), enemy_robot_2_.getEnemyPoint());
      if (dis != roborts_decision::data::MAX){
        enemy_to_attack_home.push_back(enemy_robot_2);
      }
    }
  }
  return enemy_to_attack_home;
}


int PredictionSystem::PredictBulletNumbersWithDistance(const roborts_common::Point2D& pose_1, const roborts_common::Point2D& pose_2){

  // 暂定最远距离为8m
  const double kDistance = roborts_common::PointDistance(pose_1, pose_2) / 8;
  int bullet_max;

  if (kDistance > 0.8){
    bullet_max = 1;
  }else if(kDistance <= 0.8 && kDistance > 0.6){
    bullet_max = 2;
  }else if(kDistance <= 0.6 && kDistance > 0.4){
    bullet_max = 3;
  }else if(kDistance <= 0.4 && kDistance > 0.2){
    bullet_max = 4;
  }else{
    bullet_max = 5;
  }
  return bullet_max;
}
int PredictionSystem::PredictSpeedWithDistance(const roborts_common::Point2D &pose_1, const roborts_common::Point2D &pose_2) {

  // 暂定最远距离为8m
  const double kDistance = roborts_common::PointDistance(pose_1, pose_2);
  int speed;

  if (kDistance > 6){
    speed = 24;
  }else if(kDistance <= 6 && kDistance > 4){
    speed = 20;
  }else if(kDistance <= 4 && kDistance > 2){
    speed = 15;
  }else{// 距离过近,假设速度增大,因为特别近,更容易倾斜一定角度躲子弹,这样速度大一点才行
    speed = 20;
  }
  return speed;

}

/**
 * 规范化进攻我方机器人的数据
 * @param enemy_to_attack_home_1 进攻我方1号球员的机器人
 * @param enemy_to_attack_home_2 进攻我方2号球员的机器人
 */
void PredictionSystem::NormalizedEnemyAttacker(std::vector<roborts_decision::EnemyRobot> &enemy_to_attack_home_1, std::vector<roborts_decision::EnemyRobot>&enemy_to_attack_home_2) {

  // 1. 如果攻击我方的1号和2号机器人均为1个
  if (enemy_to_attack_home_1.size() == 1 && enemy_to_attack_home_2.size() == 1){
    // 预测bug, 攻击我方两个的机器人为同一个机器人, 这是不可能事件,根据距离进行修正
    if (enemy_to_attack_home_1.at(0) == enemy_to_attack_home_2.at(0)){
      const double kDis1 = roborts_common::PointDistance(p_my_robot_1_->getMyRobotPoint(), enemy_to_attack_home_1.at(0).getEnemyPoint());
      const double kDis2 = roborts_common::PointDistance(p_my_robot_2_->getMyRobotPoint(), enemy_to_attack_home_1.at(0).getEnemyPoint());
      if (kDis1 > kDis2){
        enemy_to_attack_home_1.clear();
      }else{
        enemy_to_attack_home_2.clear();
      }
    }else{
      // Do nothing
    }
  }else if (enemy_to_attack_home_1.size() == 2 && enemy_to_attack_home_2.size() == 1){
    // 攻击我方1号有两个机器人,2号有一个,此时删除同时攻击我方1号和2号的机器人
    for(const auto & iter : enemy_to_attack_home_1){
      if (iter == enemy_to_attack_home_2.at(0)){
        //std::remove_if()
        auto find_iter = std::find(enemy_to_attack_home_1.begin(), enemy_to_attack_home_1.end(), iter);
        enemy_to_attack_home_1.erase(find_iter);
      }
    }
  }else if(enemy_to_attack_home_1.size() == 1 && enemy_to_attack_home_2.size() == 2){
    // 攻击我方2号有两个机器人,1号有一个,此时删除同时攻击我方1号和2号的机器人
    for(const auto & iter : enemy_to_attack_home_2){
      if (iter == enemy_to_attack_home_1.at(0)){
        //std::remove_if()
        auto find_iter = std::find(enemy_to_attack_home_2.begin(), enemy_to_attack_home_2.end(), iter);
        enemy_to_attack_home_2.erase(find_iter);
      }
    }
  }else if(enemy_to_attack_home_1.size() == 2 && enemy_to_attack_home_2.size() == 2){
    // 攻击我方的1号,2号均是两个机器人,此时按照距离各删除一个
      const double kDisHome1ToEnemy1 = roborts_common::PointDistance(p_my_robot_1_->getMyRobotPoint(), enemy_robot_1_.getEnemyPoint());
      const double kDisHome1ToEnemy2 = roborts_common::PointDistance(p_my_robot_1_->getMyRobotPoint(), enemy_robot_2_.getEnemyPoint());
      const double kDisHome2ToEnemy1 = roborts_common::PointDistance(p_my_robot_2_->getMyRobotPoint(), enemy_robot_1_.getEnemyPoint());
      const double kDisHome2ToEnemy2 = roborts_common::PointDistance(p_my_robot_2_->getMyRobotPoint(), enemy_robot_2_.getEnemyPoint());
      if (kDisHome1ToEnemy1 > kDisHome1ToEnemy2){
        for (auto iter = enemy_to_attack_home_1.begin(); iter != enemy_to_attack_home_1.end(); iter++){
          if (iter->GetId() == enemy_robot_1_.GetId()){
            enemy_to_attack_home_1.erase(iter);
          }else{
            continue;
          }
        }
      }else{
        for (auto iter = enemy_to_attack_home_1.begin(); iter != enemy_to_attack_home_1.end(); iter++){
          if (iter->GetId() == enemy_robot_2_.GetId()){
            enemy_to_attack_home_1.erase(iter);
          }else{
            continue;
          }
        }
      }

      if(kDisHome2ToEnemy1 > kDisHome2ToEnemy2){
        for (auto iter = enemy_to_attack_home_2.begin(); iter != enemy_to_attack_home_2.end(); iter++){
          if (iter->GetId() == enemy_robot_1_.GetId()){
            enemy_to_attack_home_2.erase(iter);
          }else{
            continue;
          }
        }
      }else{
        for (auto iter = enemy_to_attack_home_2.begin(); iter != enemy_to_attack_home_2.end(); iter++){
          if (iter->GetId() == enemy_robot_2_.GetId()){
            enemy_to_attack_home_2.erase(iter);
          }else{
            continue;
          }
        }
      }
    }else{
      // Do nothing
    }
}

/**
 * 根据我方射击的子弹数预测敌方掉血
 * @param bullets
 * @return
 */
int PredictionSystem::PredictDamageToEnemyWithBullets(const int bullets) {
  int damage = 0;
  if (bullets > 10){
    damage = 80;
  }else if (bullets <= 10 && bullets > 7){
    damage = 60;
  }else if(bullets <= 7 && bullets > 4){
    damage = 40;
  }else if (bullets <= 4 && bullets >1){
    damage = 20;
  }else{
    damage = 0;
  }

  return damage;
}
