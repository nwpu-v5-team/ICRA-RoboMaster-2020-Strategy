#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "roborts_msgs/ArmorDetectionAction.h"

#include "io/io.h"
#include "../proto/decision.pb.h"
#include "costmap/costmap_interface.h"
#include "../../roborts_common/math/math.h"
#include "blackboard_common.h"


namespace roborts_decision {

class BlackboardRaw {
 public:
  typedef roborts_costmap::CostmapInterface CostMap;
  typedef roborts_costmap::Costmap2D CostMap2D;

  explicit BlackboardRaw(const std::string &proto_file_path);
  virtual ~BlackboardRaw();

  // Enemy
  void ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr &feedback);
  geometry_msgs::PoseStamped GetEnemy() const;
  bool IsEnemyDetected() const;

  // Goal
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal);
  geometry_msgs::PoseStamped GetGoal() const;
  bool IsNewGoal();

  /*---------------------------------- Tools ------------------------------------------*/

  static double GetDistance(const geometry_msgs::PoseStamped &pose1,
                            const geometry_msgs::PoseStamped &pose2);
  static double GetAngle(const geometry_msgs::PoseStamped &pose1,
                         const geometry_msgs::PoseStamped &pose2);

  // 方向向量转化四元数
  static std::tuple<double,double,double,double> turnAngleToPose(double alpha, double beta, double gamma);

  // 方向向量转化水平方向角（-180-180）
  static double vectorToHorizontalDirection(double dx, double dy);
  // 方向向量转化竖直方向角（-90-90）
  static double vectorToVerticalDirection(double dx, double dy, double dz);

  // n/x 函数模型
  static double inverseRatioFunction(double x, double max = static_cast<double>(roborts_decision::data::MAX), double min = 0.0, double n = 1);

  // kx + b 函数模型
  static double linearFunction(double x, double k = -1, double b = 1, double max = static_cast<double>(roborts_decision::data::MAX), double min = 0);

  // a^(k * x) + b 函数模型
  static double exponentialFunction(double x, double k = -1, double a = exp(1), double b = 0, double min = 0);

  const geometry_msgs::PoseStamped GetChassisMapPose();
  const geometry_msgs::PoseStamped GetGimbalMapPose();
  const std::shared_ptr<CostMap> GetCostMap();
  const CostMap2D* GetCostMap2D();
  const unsigned char* GetCharMap();

 private:
  void UpdateChassisPose();
  void UpdateGimbalPose();

  //tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  //Enenmy detection
  ros::Subscriber enemy_sub_;

  //Goal info
  geometry_msgs::PoseStamped goal_;
  bool new_goal_{};

  //Enemy info
  actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction> armor_detection_actionlib_client_;
  roborts_msgs::ArmorDetectionGoal armor_detection_goal_;
  geometry_msgs::PoseStamped enemy_pose_;
  bool enemy_detected_;

  //cost map
  std::shared_ptr<CostMap> costmap_ptr_;
  CostMap2D* costmap_2d_;
  unsigned char* charmap_;

  // Chassis map pose.
  geometry_msgs::PoseStamped chassis_map_pose_;

  // Gimbal map pose.
  geometry_msgs::PoseStamped gimbal_map_pose_;

  // Chassis odom pose.
  geometry_msgs::PoseStamped chassis_odom_pose_;

  // Gimbal odom pose.
  geometry_msgs::PoseStamped gimbal_odom_pose_;

};
} //namespace roborts_decision
#endif //ROBORTS_DECISION_BLACKBOARD_H
