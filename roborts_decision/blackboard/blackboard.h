//
// Created by kehan on 2020/3/1.
//

#ifndef ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_H_
#define ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_H_

#include <roborts_msgs/RobotStatus.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>

#include <std_msgs/Int32.h>
#include "roborts_msgs/BuffZoneStatus.h"
#include "roborts_msgs/GameSurvivor.h"
#include "roborts_msgs/OutpostDetected.h"

#include "../../roborts_common/math/math.h"

#include "blackboard_common.h"
#include "enemy_robot.h"
#include "my_robot.h"
#include "prediction_system.h"
#include "field.h"

namespace roborts_decision {

struct PointValue {
    roborts_common::Point2D point;
    double value = 0;
};

/// Contains all information needed by the strategy
/// The class implements single instance pattern, the instance is obtained by Blackboard::GetBlackboard().
class Blackboard {
public:
  typedef std::shared_ptr<Blackboard> Ptr;
  /// Construct Blackboard using two MyRobot
  explicit Blackboard(std::shared_ptr<MyRobot> &p_myrobot1,
                      std::shared_ptr<MyRobot> &p_myrobot2,
                      const ros::NodeHandle &nh = ros::NodeHandle("~"));
  virtual ~Blackboard();

  Blackboard(const Blackboard&) = delete;
  Blackboard(Blackboard&&) = delete;

  /// Create or get the single instance of Blackboard
  static std::shared_ptr<Blackboard> GetBlackboard();


  /// Get number 1 of MyRobot
  const std::shared_ptr<MyRobot> &GetMyRobot1();
  /// Get number 2 of MyRobot
  const std::shared_ptr<MyRobot> &GetMyRobot2();

  /// Get number 1 of EnemyRobot
  EnemyRobot &GetEnemyRobot1();
  /// Get number 2 of EnemyRobot
  EnemyRobot &GetEnemyRobot2();

  /// Given an EnemyRobot, returns its companion.
  const EnemyRobot &GetAnotherRobot(const EnemyRobot &enemy_robot);
  /// Given an MyRobot, returns its companion.
  const std::shared_ptr<MyRobot> &GetAnotherRobot(const std::shared_ptr<MyRobot>& p_my_robot) const;


  /// Update all the information. Need to be called on every frame.
  void Update();

  /// Get the color of MyRobot
  MyColor GetMyColor() const;

  /// \brief Whether the non-defensive run (spinning run) is obstacle avoidance run or run at a certain angle
  /// \param my_robot `std::shared_ptr` to MyRobot
  RobotBehavior GetCommonMoveType(const std::shared_ptr<MyRobot> &my_robot);

  /// \brief Get `EnemyRobot`s that can attack us directly
  /// \returns a `std::vector` of dangerous EnemyRobot
  std::vector<EnemyRobot> GetDangerousOpp(const std::shared_ptr<MyRobot> &p_my_robot);

  /// \brief Get the point to run when attacking
  /// \param p_my_robot The robot attacking
  /// \param another_partner_point The partner of the attacking robot
  roborts_common::Point2D GetAttackRunningPoint(const std::shared_ptr<MyRobot> &p_my_robot,
                                                const roborts_common::Point2D &another_partner_point);

  /// \brief Get the point to run to, used by GetAttackRunningPoint
  /// \param running_interval 所有符合条件的跑位点
  /// \param p_my_robot 我方执行此函数的机器人机器人
  /// \param another_point 我方另一个机器人的跑位点
  /// \return 返回具体跑位点
  roborts_common::Point2D GetSpecificRunPoint(const std::vector<PointValue> &running_interval,
                                              std::shared_ptr<MyRobot> p_my_robot, const roborts_common::Point2D &another_point);

  /// \brief Use analytic hierarchy process to obtain robots that need to attack
  const EnemyRobot &GetEnemyRobotToAttack(const std::shared_ptr<MyRobot> &p_my_robot);

  /// \brief Get the EnemyRobot that closest to our robots
  const EnemyRobot &GetCloserEnemyRobot(const std::shared_ptr<MyRobot> &p_my_robot);

  /// \brief Get max safe distance
  double GetKeepDirectionSafeDistance() const;
  /// \brief Obtain the damage percentage of a certain distance from the enemy
  /// and a certain deflection angle relative to the enemy
  /// (the angle formed by the line between the normal direction and the center of the robot)
  double GetKeepDirectionEffectWithDistance(double distance, double angle = 45) const;
  // 获取射击速度
  double GetShootSpeed(const std::shared_ptr<MyRobot>&p_my_robot, const roborts_common::Point2D &enemy_pose);

  // 获得准备射击时的敌方目标机器人
  const EnemyRobot &GetPrepareShootOpp(const std::shared_ptr<MyRobot> &my_robot);

  // 计算防御下我方某机器人的跑位点(若给第二个参数，会考虑另一机器人的跑点）
  roborts_common::Point2D GetDefenseRunningPoint(const std::shared_ptr<MyRobot> &p_my_robot,
                                                 roborts_common::Point2D another_point = roborts_common::Point2D(0,
                                                                                                                 0));

  // 更新防御势场全部影响因素
  void UpdateDefenseAllPotentialField(std::vector<std::vector<PointValue>> &security_matrix);

  // 对矩形范围内的点统一用value更新
  void UpdateAreaOfPotentialField(std::vector<std::vector<PointValue>> &security_matrix,
                                  const roborts_common::Polygon2D &area, double value) const;

  // 获得防御势场判断安全的阈值
  double GetDefensePotentialFieldSafeValue() const;

  // 考虑同伴的情况后判断该点是否合适
  bool CanCooperatePartner(const roborts_common::Point2D &tar_point,
                           const roborts_common::Point2D &partner_tar_point,
                           const std::shared_ptr<MyRobot> &me,
                           const std::shared_ptr<MyRobot> &partner) const;

  void FindSafePoint(PointValue &safe_point,
                     const std::shared_ptr<MyRobot> &p_my_robot,
                     const std::vector<std::vector<PointValue>> &security_matrix,
                     roborts_common::Point2D another_point);

  // 炮台势场(direction：相对炮台方向位置偏角（0-180）)
  double GetFortPotentialField(double distance, double direction);

  // 敌方位置势场
  double GetPlacePotentialField(double distance);

  // 墙体势场(包括障碍物)
  double GetWallPotentialField(double distance);

  // 计算跑位点中的更新势场函数
  void UpdatePotentialField(std::vector<std::vector<PointValue>> &potential_field,
                            const roborts_common::Polygon2D &object,
                            const std::function<double(const roborts_common::Point2D &,
                                                 const roborts_common::Polygon2D &)>& fun);

  // 根据离散单位生成地图
  std::vector<std::vector<PointValue>> DiscretizeMap(double discrete_unit) const;

  // 是否相撞(撞墙、撞车
  bool IsCollide(std::shared_ptr<MyRobot> p_my_robot) const;

  // 计算进攻跑位点函数更新势场
  std::vector<std::vector<PointValue>> UpdatePotentialFieldInAttack(const std::shared_ptr<MyRobot> &p_my_robot, const EnemyRobot &enemy_to_attack);

  // 敌方机器人更新势场的策略，具体由updatePotentialField中的函数指针调用
  // 计算所攻击敌方机器人的势场
  double CalculateEnemyToAttackPotentialField(const roborts_common::Point2D &pose_point,
                                              const roborts_common::Polygon2D &pose_polygon);

  // 计算另一个敌方机器人的势场
  double CalculateEnemyAnotherPotentialField(const roborts_common::Point2D &pose_point,
                                             const roborts_common::Polygon2D &pose_polygon);

  // 选取合适的跑位点
  std::vector<PointValue> CalculateSuitRunPoint(const std::vector<std::vector<PointValue>> &potential_field);

  // 计算点到多边形间的距离（若两者间有障碍物，距离无穷大）
  double GetDistanceWithObstacles(const roborts_common::Point2D &pose,
                                  const roborts_common::Polygon2D &polygon_2_d);

  // 计算点到实体矩形的距离（若在实体中间，返回0）
  double GetPointToEntityDistance(const roborts_common::Point2D &point_2_d,
                                  const roborts_common::Polygon2D &polygon_2_d) const;

  // 获取我方所有机器人的矩形
  std::vector<roborts_common::Polygon2D> GetHomePolygon();

  // 预测距离
  double PredictedDistance(const roborts_common::Point2D &_point, const std::shared_ptr<MyRobot>& p_my_robot);

  // 获取我方某个机器人的矩形
  roborts_common::Polygon2D GetHomePolygon(const std::shared_ptr<MyRobot> &p_my_robot) const;

  // 获取敌方所有机器人的矩形
  std::vector<roborts_common::Polygon2D> GetEnemyPolygon() const;

  // 获取敌方某个机器人的矩形
  roborts_common::Polygon2D GetEnemyPolygon(const EnemyRobot &enemy_robot) const;

  void WriteFile(std::string file_name, std::vector<std::vector<PointValue>> &map, roborts_common::Point2D &most_suitable_point);
private:
  void OutpostDetectedCallback(const roborts_msgs::OutpostDetected::ConstPtr &msg);
  void MyColorCallback(const roborts_msgs::RobotStatus::ConstPtr &msg);
  void GameSurvivorCallback(const roborts_msgs::GameSurvivor::ConstPtr &msg);

  [[deprecated]]
  void NextDecisionCallBackDeprecated(const std_msgs::Int32::ConstPtr &msg);


  ros::NodeHandle nh_;
  ros::Subscriber outpose_camera_sub_;
  ros::Subscriber color_sub_;
  ros::Subscriber game_survivor_sub_;
  ros::Publisher is_collide_pub_;

  MyColor my_color_;

  EnemyRobot enemy_robot_1_;
  EnemyRobot enemy_robot_2_;
  std::shared_ptr<Field> field_;

  std::shared_ptr<MyRobot> p_my_robot1_;
  std::shared_ptr<MyRobot> p_my_robot2_;

  PredictionSystem prediction_system_;

  static std::shared_ptr<Blackboard> p_blackboard_;

  // strategy
  std::string LoadValue(const std::string& value_name, const std::string& target_name);
  std::string FindValueInFile(const std::string& value_name, const std::string& file_name);
  bool UpdataValue(const std::string &value_name, const std::string &file_name, const std::string &value);
  bool AddValueInFile(const std::string &value_name, const std::string &value, const std::string &file_path);
  bool ChangeValueInFile(const std::string& value_name, const std::string& value, const std::string& file_path);
  double defense_safe_value_ = 100.0;

};
} // namespace roborts_decision

#endif // ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_H_
