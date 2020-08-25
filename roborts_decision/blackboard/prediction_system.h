//
// Created by wpy on 2020/8/5.
//

#ifndef ROBOTRTS_WS_SRC_ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_BLACKBOARD_PREDICTION_SYSTEM_H_
#define ROBOTRTS_WS_SRC_ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_BLACKBOARD_PREDICTION_SYSTEM_H_
/**
 * Prediction system: responsible for predicting all enemy and our robot information
 */

#include "my_robot.h"
#include "enemy_robot.h"
#include "field.h"

namespace roborts_decision {
class PredictionSystem {
 public:

  PredictionSystem(std::shared_ptr<MyRobot> p_my_robot_1,std::shared_ptr<MyRobot> p_my_robot_2, EnemyRobot &enemy_robot_1 ,EnemyRobot &enemy_robot_2);

  PredictionSystem() = delete ;
  ~PredictionSystem() = default;


  // 获取剩余子弹数
  void UpdatePredictedRemainingProjectiles();

  // 获得剩余血量
  void UpdatePredictedHp();

  // 获得枪口热量
  void UpdatePredictedCurrentHeat();

  // 获取当前敌方子弹buff状态
  bool GetEnemyBulletBuffStatus();

  // 获取当前敌方血量buff状态
  bool GetEnemyBloodBuffStatus();

  // 预测敌方打击我方的机器人
  std::vector<roborts_decision::EnemyRobot> PredictWhoAttackHome(const std::shared_ptr<roborts_decision::MyRobot> &p_my_robot);
 private:

  std::shared_ptr<MyRobot> p_my_robot_1_;
  std::shared_ptr<MyRobot> p_my_robot_2_;
  std::array<std::shared_ptr<MyRobot>, 2> all_home_;

  std::shared_ptr<Field> field_;

  EnemyRobot &enemy_robot_1_;
  EnemyRobot &enemy_robot_2_;
  int bullet_shoot_enemy_1_to_home_1_;
  int bullet_shoot_enemy_1_to_home_2_;
  int bullet_shoot_enemy_2_to_home_1_;
  int bullet_shoot_enemy_2_to_home_2_;

  int speed_shoot_enemy_1_to_home_1_;
  int speed_shoot_enemy_1_to_home_2_;
  int speed_shoot_enemy_2_to_home_1_;
  int speed_shoot_enemy_2_to_home_2_;

  // 根据距离预测敌方可能发射的子弹
  int PredictBulletNumbersWithDistance(const roborts_common::Point2D &pose_1, const roborts_common::Point2D &pose_2);

  int PredictSpeedWithDistance(const roborts_common::Point2D &pose_1, const roborts_common::Point2D &pose_2);

  int PredictDamageToEnemyWithBullets(const int bullets);

  void NormalizedEnemyAttacker(std::vector<roborts_decision::EnemyRobot> &enemy_to_attack_home_1, std::vector<roborts_decision::EnemyRobot>&enemy_to_attack_home_2);

};
}
#endif //ROBOTRTS_WS_SRC_ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_BLACKBOARD_PREDICTION_SYSTEM_H_
