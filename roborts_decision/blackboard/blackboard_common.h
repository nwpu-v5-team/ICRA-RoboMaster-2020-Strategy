//
// Created by kehan on 2020/2/29.
//

#ifndef ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_COMMON_H_
#define ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_COMMON_H_

#include <geometry_msgs/PoseStamped.h>
#include <roborts_msgs/ArmorsDetected.h>

namespace roborts_decision {

class data {
 public:
  //暂定频率为10Hz
  constexpr static int Hz = 2;   ///< 决策频率
  constexpr static double PI = 3.1415926535;
  constexpr static int HP = 2000; ///< 最高血量
  constexpr static int MAXHEAT = 240; ///< 最高热量
  constexpr static int STARTPROJECTILES = 50; ///< 初始子弹数
  constexpr static int PROJECTILESPACKAGE = 100;  ///< 子弹包子弹数
  constexpr static int BLOODPACKAGE = 200;  ///< 血包回血数量
  constexpr static double ROBOTSIZE = 0.6;    ///< 车长
  constexpr static double SITELENGTH = 8.08;  ///< 赛场长
  constexpr static double SITEHEIGHT = 4.48;  ///< 赛场高
  constexpr static int MAX = 1e7; ///< 最大数
  constexpr static double SHOOTMAX = 25;  ///< 最大射速
  constexpr static double DANGERSHOOTMIN = 12;    ///< 最小有效射速
  constexpr static int averagespeed_ = 20;///< 预估敌方射击子弹的速度
  // 状态评估的系数
  constexpr static double kBlood = 0.9;
  constexpr static double kBullet = 0.8;
  constexpr static double kHeat = 0.1;

};

/**
 * 当前机器人的位姿
 */
enum class Posture {
  /// 以固定角度跑位，包括倾斜一定角度，或者没有倾斜
  TILTANGLE,
  /// 在进行旋转跑位
  TWIST
};

enum MyRobotBehavior {
  CHASE,
  ESCAPE,
  GOAL,
  SUPPLY
};

enum ArmorId {
  FRONT = 0,
  RIGHT = 1,
  BACK = 2,
  LEFT = 3
};

enum BuffStatus {
  RED_RESTORATION = 1,//血包
  RED_PROJECTILE_SUPPLIER = 2,//子弹
  BLUE_RESTORATION = 3,
  BLUE_PROJECTILE_SUPPLIER = 4,
  NO_SHOOTING = 5,
  NO_MOVING = 6,
  UNKNOWN_STATUS = 7
};

struct BuffZoneStatus {
  bool is_active = false;
  BuffStatus buff_status = UNKNOWN_STATUS;
};

enum EnemyBuffZone {
  PROJECTILE_SUPPLIER,
  RESTORATION
};

enum RobotType {
  RED_1 = 3,
  RED_2 = 4,
  BLUE_1 = 13,
  BLUE_2 = 14,
  UNKNOWN_TYPE
};

enum RobotId {
  MY_ROBOT_1 = 0,
  MY_ROBOT_2 = 1,
  ENEMY_ROBOT_1 = 2,
  ENEMY_ROBOT_2 = 3,
};

enum MyColor {
  RED,
  BLUE,
  UNKNOWN_COLOR
};

enum Decision {
  OFFENSIVE
};

/// Basic action of robot
enum class RobotBehavior {
  SPIN_FORWARD,
  SPIN_IN_PLACE,
  STAY_STILL,
  KEEP_DIRECTION_MOVE,
  // AVOIDANCE_MOVE,
  SET_GIMBAL_ODOM_POSE,
  SHOOT_AIMED,
  TOWARD_W_POS_SHOOT
};

/// Signal for interaction between robot
enum class Signal {
  NONE,
  HELP
};
}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_COMMON_H_
