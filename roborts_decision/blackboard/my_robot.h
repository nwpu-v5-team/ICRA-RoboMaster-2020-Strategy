//
// Created by kehan on 2020/2/28.
//

#ifndef ROBORTS_ROBORTS_DECISION_BLACKBOARD_RMROBOT_H_
#define ROBORTS_ROBORTS_DECISION_BLACKBOARD_RMROBOT_H_

#include <vector>
#include <thread>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <roborts_msgs/ArmorsDetected.h>
#include <roborts_msgs/RobotStatus.h>

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <roborts_msgs/RobotHeat.h>
#include <roborts_msgs/RobotDamage.h>
#include <roborts_msgs/ShootInfo.h>

#include "blackboard_common.h"
#include "blackboard_raw.h"
#include "../executor/gimbal_executor.h"
#include "../executor/chassis_executor.h"
#include "../../roborts_common/math/math.h"

namespace roborts_decision {

class MyRobot {
public:
    explicit MyRobot(RobotId id, const ros::NodeHandle &nh = ros::NodeHandle("~"));

    virtual ~MyRobot();

    bool operator==(const MyRobot &rhs) const;
    bool operator!=(const MyRobot &rhs) const;

    /****************定义MyRobot获取赛场信息和自身属性方法********************/
    void update(); // 每拍进行的更新

    RobotId GetId() const;

    // 获取机器人编号: red1/2, blue1/2
    RobotType getRobotType() const;
    //设定机器人编号
    void setRobotType(RobotType robot_type);
    //将机器人编号数据类型由enum转换成string
    std::string robotTypeToString(const RobotType robot_type);

    //获取当前血量
    int getHp() const;

    // 获取当前热量
    int getCurrentHeat() const;

    // 获取当前弹丸数
    int getRemainingProjectiles() const;

    // 是否存活
    bool isSurvival() const;
    void setIsSurvival(bool is_survival);

    // 是否可以移动、射击
    bool isNoMove() const;
    bool isNoShoot() const;

    // 是否被攻击
    bool isShot() const;

    // 获取被攻击的装甲板编号
    const std::vector<ArmorId> &getArmorsUnderAttack() const;

    // 获取视线范围内的装甲板
    const roborts_msgs::ArmorsDetected &getArmorsInEyes() const;

    // 获取底盘在地图的姿态，世界坐标
    const geometry_msgs::PoseStamped &getChassisMapPose() const;

    // OK = 0, Error = 1, FAILURE >= 2
    uint32_t getStatusCode();

    // 获取底盘odom坐标系下姿态
    const geometry_msgs::PoseStamped &getChassisOdomPose();

    // 获取云台世界坐标系下姿态
    const geometry_msgs::PoseStamped &getGimbalMapPose();

    // 获取云台Odom坐标系下姿态
    const geometry_msgs::PoseStamped &getGimbalOdomPose();

    roborts_common::Point2D getMyRobotPoint() const;
    // 获取当前目标
    // const geometry_msgs::PoseStamped &GetCurrentGoal() const;

    // 获取当前行为状态
    MyRobotBehavior getCurrentBehavior() const;
    void setCurrentBehavior(MyRobotBehavior current_behavior);

    double getEffectiveShootDistance() const;

    double getState() const;

  /**********************通讯方法******************************/
    // 获取下一个跑位点
    const roborts_common::Point2D &getNextPoint() const;

    // 设置下一个跑位点
    void setNextPoint(const roborts_common::Point2D &nextPoint);

    // 获取信号量
    Signal getSignal() const;

    /****************定义MyRobot底盘动作方法********************/
    //实现机器人旋转前进
    // 只需传跑位点，转速在内部确定
    void spinForword(const double &tarX, const double &tarY);

    // 实现机器人原地旋转
    void spinInPlace(const double &angularVelocity);

    //实现机器人原地静止
    void stayStill();

    /** 实现机器人以某个朝向移动
     * @param tar_x 目标点x坐标
     * @param tar_y 目标点y坐标
     * @param direction 到达点位点目标方向
     */
    void keepDirectionMove(double tar_x, double tar_y, double direction);
    double getSafeDirection(const geometry_msgs::PoseStamped &pose_enemy);  // 获得相对地方最安全的姿态偏角

    // 避障跑 此为空函数不推荐使用
    [[deprecated("当前跑位默认使用避障，暂无不适用避障的接口")]]
    void avoidanceMove(double x, double y);

    // 获取特定装甲板的朝向角度,范围 -PI - PI
    double GetArmorTowards(ArmorId armor_id);

    /****************定义MyRobot云台及射击方法********************/
    // TODO:实现机器人以某种速度射击一定数量的子弹(返回是否在一拍内射击了指定数量的子弹）(不满足射击条件自动停止射击并返回false）
    bool shoot(double speed, int number = data::MAX);
    int shootBulletsMaxNumber(double speed) const; // TODO:返回一拍内该速度射击的最大子弹数量

    // 将枪口朝向某个坐标点
    double TowardWPosShoot(double tar_x, double tar_y, double tar_z);

    //获取需要攻击的目标
    int GetShootEnemyId();
    void SetShootEnemyId(const int id);

    // 获取一帧射击的子弹数
    int GetShootNulletsNumber();
    void  SetShootNulletsNumber(const int num_bullets);

  // 设置我方机器人射击参数,用于状态估计(getState)
  void SetHomeShootParameter();

  // 设置我方机器人移动参数,用于状态估计(getState)
  void SetHomeMoveParameter();



private:
    // 获取底盘/云台控制器指针
    std::shared_ptr<ChassisExecutor> getPChassisExecutor();
    std::shared_ptr<GimbalExecutor> getPGimbalExecutor();

    //设置云台位置
    void setGimbalOdomPose(const double &gimbal_goal_map_pitch, const double &gimbal_goal_map_yaw);

    // 原学长写的射击函数，提供宝贵指导作用
    roborts_msgs::GimbalAngle shootAimed(geometry_msgs::Point32 &target);

    //暂时废弃shoot函数
    void shoot();

    // 与其他机器人通讯（不保证对方一定会接收）
    void setSignal(Signal signal);

    // TODO: 射击时转轮和实际射击速度的关系
    double changeFrictionWheelSpeedToShootSpeed(double frictionWhellSpeed);

    // ros消息回调函数
    void ArmorsUnderAttackCallback(const roborts_msgs::RobotDamage::ConstPtr &msg);
    void HeatCallback(const roborts_msgs::RobotHeat::ConstPtr &msg);
    void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &msg);
    void ArmorsInEyesCallback(const roborts_msgs::ArmorsDetected::ConstPtr &msg);
    void RemainingProjectilesCallback(const roborts_msgs::ShootInfo::ConstPtr &msg);

    void ChassisMapPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    // void UpdateChassisMapPose();
    void UpdateChassisOdomPose();
    void UpdateGimbalMapPose();
    void UpdateGimbalOdomPose();

    ros::NodeHandle nh_;

    ros::Subscriber remaining_projectiles_sub_;
    ros::Subscriber armors_under_attack_sub_;
    ros::Subscriber heat_sub_;
    ros::Subscriber armors_in_eyes_sub_;
    ros::Subscriber robot_status_sub_;
    ros::Subscriber robot_map_pose_sub_;

    std::shared_ptr<tf::TransformListener> tf_ptr_;
    std::shared_ptr<std::thread> tf_thread_ptr_;

    std::shared_ptr<std::thread> p_ros_spin_thread_;

    RobotId id_;
    RobotType robot_type_;

    int remaining_hp_;
    int max_hp_;
    int current_heat_;

    bool no_move_;
    bool no_shoot_;

    // 状态评估的系数
    constexpr static double kBlood = 0.9;
    constexpr static double kBullet = 0.8;
    constexpr static double kHeat = 0.1;
    double tNoShoot = 0;
    double tNoMove = 0;


    // TODO: 子弹数，不知为何要加TODO
    int remaining_projectiles_;

    bool is_survival_;

    int shoot_enemy_id_;

    int shoot_bullets_this_time_;

    std::vector<ArmorId> armors_under_attack_;
    roborts_msgs::ArmorsDetected armors_in_eyes_;

    geometry_msgs::PoseStamped chassis_map_pose_;
    geometry_msgs::PoseStamped chassis_odom_pose_;

    geometry_msgs::PoseStamped gimbal_map_pose_;
    geometry_msgs::PoseStamped gimbal_odom_pose_;

    // geometry_msgs::PoseStamped current_goal_;
    MyRobotBehavior current_behavior_;

    std::shared_ptr<ChassisExecutor> p_chassis_executor_;
    std::shared_ptr<GimbalExecutor> p_gimbal_executor_;

    ros::Publisher shoot_pub_;

private:
    // 策略数据
    roborts_common::Point2D nextPoint_;  // 下一个跑位点

    roborts_decision::Signal signal_;    // 通讯信号量（求救、进攻等，其他机器人会在满足自己的动作的基础上查看同伴的情况）
};

}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_RMROBOT_H_
