//
// Created by kehan on 2020/2/28.
//

#ifndef ROBORTS_ROBORTS_DECISION_BLACKBOARD_ENEMY_ROBOT_H_
#define ROBORTS_ROBORTS_DECISION_BLACKBOARD_ENEMY_ROBOT_H_

#include "../../roborts_common/math/math.h"
#include "blackboard_common.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <roborts_msgs/ShootInfo.h>
#include <ros/ros.h>

namespace roborts_decision {
class EnemyRobot {
public:
  EnemyRobot(const RobotId &robot_id, MyColor my_color);
  virtual ~EnemyRobot();
  void Init();

  RobotId GetId() const;

  RobotType GetRobotType() const;
  void SetRobotType(RobotType robot_type);

  void SetColor(MyColor color);
  MyColor GetColor() const;

  bool IsSurvival() const;
  void SetIsSurvival(bool is_survival);

  const geometry_msgs::PoseStamped &GetPose() const;
  void SetPose(const ros::Time &stamp, const geometry_msgs::Point &position);

  // 获取剩余子弹数
  int getRemainingProjectiles() const;
  // 更新子弹数
  void SetRemainingProjectiles(const int bullets);

  // 获得剩余血量
  int getHp() const;
  //设置当前血量
  void SetHp(const int hp);

  // 获得枪口热量
  int getCurrentHeat() const;
  void SetCurrentHeat(const int heat);

  // 是否可移动
  bool canMove() const;
  bool SetCanMove(const bool canMove);

  // 是否可射击
  bool canShoot() const;
  void SetCanShoot(const bool canShoot);

  bool IsDetected() const;
  void SetIsDetected(bool is_detected);

  // 获取敌方机器人的状态评估值
  double getState() const;
  double getStanderdState() const; // 一般状态阈值

  // 获得敌方机器人正向相对某点的偏角
  double getRelativeAngle(roborts_common::Point2D point) const;

  // 获取敌方机器人的坐标，以Point2D形式返回
  roborts_common::Point2D getEnemyPoint() const;

  // 获取敌方机器人当前位姿
  Posture getCurrentPosture() const;

  // 设置敌方机器人射击参数,用于状态估计(getState)
  void SetEnemyShootParameter();

  // 设置敌方机器人移动参数,用于状态估计(getState)
  void SetEnemyMoveParameter();

  void EnemyRobotPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);



  bool operator==(const EnemyRobot &e_r)const;

private:
  RobotId id_;
  RobotType robot_type_;
  MyColor color_;

  ros::NodeHandle nh_;
  ros::Subscriber enemy_pose_sub_;

  int num_of_bullets_;
  int hp_;
  int current_heat_;
  bool can_move_;
  bool can_shoot_;

  bool is_survival_;
  bool is_detected_;
  geometry_msgs::PoseStamped pose_;

  // 状态评估的系数
  constexpr static double kBlood = 0.9;
  constexpr static double kBullet = 0.8;
  constexpr static double kHeat = 0.1;
  double tNoShoot = 0;
  double tNoMove = 0;
};
} // namespace roborts_decision

#endif // ROBORTS_ROBORTS_DECISION_BLACKBOARD_ENEMY_ROBOT_H_
