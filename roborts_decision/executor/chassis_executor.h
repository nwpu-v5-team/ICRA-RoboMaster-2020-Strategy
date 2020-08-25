#ifndef ROBORTS_DECISION_CHASSIS_EXECUTOR_H
#define ROBORTS_DECISION_CHASSIS_EXECUTOR_H
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>

#include "roborts_msgs/GlobalPlannerAction.h"
#include "roborts_msgs/LocalPlannerAction.h"
#include "roborts_msgs/PIDControllerTowardAngularAction.h"
#include "roborts_msgs/TwistAccel.h"
#include "geometry_msgs/Twist.h"

#include "../behavior_tree/behavior_state.h"
#include "pid_controller/pid_controller.h"

#include <thread>

namespace roborts_decision {
/***
 * @brief Chassis Executor to execute different abstracted task for chassis module
 */
class ChassisExecutor {

  typedef actionlib::SimpleActionClient<roborts_msgs::GlobalPlannerAction> GlobalActionClient;
  typedef actionlib::SimpleActionClient<roborts_msgs::LocalPlannerAction> LocalActionClient;
  typedef actionlib::SimpleActionClient<roborts_msgs::PIDControllerTowardAngularAction> PIDControllerClient;

 public:
  /**
   * @brief Chassis execution mode for different tasks
   */
  enum class ExcutionMode {
    IDLE_MODE,            ///< Default idle mode with no task
    GOAL_USE_PLANNER_MODE,            ///< Goal-targeted task mode using global and local planner
    GOAL_FROM_ODOM_MODE,  ///< Goal-targeted task mode using odom data
    SPEED_MODE,           ///< Velocity task mode
    SPEED_WITH_ACCEL_MODE ///< Velocity with acceleration task mode
  };

  enum class GoalMode {
    GOAL_MODE_USE_GOLBAL_LOCAL_PLANNER,
    GOAL_MODE_USE_ODOM_DATA
  };
  /**
   * @brief Constructor of ChassisExecutor
   */
  explicit ChassisExecutor(const ros::NodeHandle &nh = ros::NodeHandle("~"));
  ~ChassisExecutor() = default;
  /**
   * @brief Execute the goal-targeted task using global and local planner with actionlib
   * @param goal Given target goal
   */
  void Execute(const geometry_msgs::PoseStamped &goal);
  /**
   * @brief
   * @param goal Given target goal
   * @param _goal_mode Given goal mode
   */
  void Execute(const geometry_msgs::PoseStamped &goal, GoalMode _goal_mode);
  /**
   * @brief Execute the velocity task with publisher
   * @param twist Given velocity
   */
  void Execute(const geometry_msgs::Twist &twist);
  /**
   * @brief Execute the velocity with acceleration task with publisher
   * @param twist_accel Given velocity with acceleration
   */
  void Execute(const roborts_msgs::TwistAccel &twist_accel);
  /**
   * @brief Update the current chassis executor state
   * @return Current chassis executor state(same with behavior state)
   */
  BehaviorState Update();
  /**
   * @brief Cancel the current task and deal with the mode transition
   */
  void Cancel();

  uint32_t GetErrorCode() const;

 private:

  ros::NodeHandle nh_;
  /***
   * @brief Global planner actionlib feedback callback function to send the global planner path to local planner
   * @param global_planner_feedback  Global planner actionlib feedback, which mainly consists of global planner path output
   */
  void GlobalPlannerFeedbackCallback(const roborts_msgs::GlobalPlannerFeedbackConstPtr &global_planner_feedback);
  void GlobalPlannerDoneCallback(const actionlib::SimpleClientGoalState &state,
                                 const roborts_msgs::GlobalPlannerResultConstPtr &global_planner_result);
  void PIDControllerFeedbackCallback(const roborts_msgs::PIDControllerTowardAngularFeedbackConstPtr &pid_controller_toward_angular_feedback);
  //! execution mode of the executor
  ExcutionMode execution_mode_;
  //! execution state of the executor (same with behavior state)
  BehaviorState execution_state_;

  //! global planner actionlib client
  actionlib::SimpleActionClient<roborts_msgs::GlobalPlannerAction> global_planner_client_;
  //! local planner actionlib client
  actionlib::SimpleActionClient<roborts_msgs::LocalPlannerAction> local_planner_client_;
  //! pid controller actionlib client
  actionlib::SimpleActionClient<roborts_msgs::PIDControllerTowardAngularAction> pid_controller_client_;
  //! global planner actionlib goal
  roborts_msgs::GlobalPlannerGoal global_planner_goal_;
  //! local planner actionlib goal
  roborts_msgs::LocalPlannerGoal local_planner_goal_;
  //! pid controller actionlib goal
  roborts_msgs::PIDControllerTowardAngularGoal pid_controller_toward_angular_goal_;

  //! velocity control publisher in ROS
  ros::Publisher cmd_vel_pub_;
  //! zero twist in form of ROS geometry_msgs::Twist
  geometry_msgs::Twist zero_twist_;

  //! velocity with accel publisher in ROS
  ros::Publisher cmd_vel_acc_pub_;
  //! zero twist with acceleration in form of ROS roborts_msgs::TwistAccel
  roborts_msgs::TwistAccel zero_twist_accel_;

  uint32_t error_code_;

  // std::thread update_thread_;
//  bool LoadParam(const std::string &proto_file_path);
//  double chassis_v2p_pid_kp;
//  double chassis_v2p_pid_ki;
//  double chassis_v2p_pid_kd;
//  bool chassis_v2p_pid_has_threshold;
//  double chassis_v2p_pid_threshold;

};
}

#endif //ROBORTS_DECISION_CHASSIS_EXECUTOR_H