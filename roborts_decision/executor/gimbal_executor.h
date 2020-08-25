#ifndef ROBORTS_DECISION_GIMBAL_EXECUTOR_H
#define ROBORTS_DECISION_GIMBAL_EXECUTOR_H
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point32.h>
#include "roborts_msgs/PIDControllerTowardAngularAction.h"
#include "gimbal_control.h"
#include "ros/ros.h"

#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalRate.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"

#include "../behavior_tree/behavior_state.h"
namespace roborts_decision{
/***
 * @brief Gimbal Executor to execute different abstracted task for gimbal module
 */
class GimbalExecutor{

  typedef actionlib::SimpleActionClient<roborts_msgs::PIDControllerTowardAngularAction> PIDControllerClient;

 public:
  /**
   * @brief Gimbal execution mode for different tasks
   */
  enum class ExcutionMode{
    IDLE_MODE,   ///< Default idle mode with no task
    ANGLE_MODE,  ///< Angle task mode
    RATE_MODE,    ///< Rate task mode
    PID_MODE
  };

  enum class GoalMode {
    GOAL_MODE_USE_GIMBAL_ANGLE,
    GOAL_MODE_USE_GIMBAL_RATE,
    GOAL_MODE_USE_PID
  };

  /**
   * @brief Constructor of GimbalExecutor
   */
  explicit GimbalExecutor(const ros::NodeHandle &nh = ros::NodeHandle("~"));
  ~GimbalExecutor() = default;

  void Execute(const geometry_msgs::PoseStamped &gimbal_angle, GoalMode goal_mode);
  /***
   * @brief Execute the gimbal angle task with publisher
   * @param gimbal_angle Given gimbal angle
   */
  void Execute(geometry_msgs::Point32 &target, roborts_msgs::GimbalAngle &executor_gimbal_angle_);
  /***
   * @brief transform the target point to gimbal angle with publisher
   * @param gimbal_angle Given gimbal angle
   */
  void Execute(const roborts_msgs::GimbalAngle &gimbal_angle);
  /***
   * @brief Execute the gimbal rate task with publisher
   * @param gimbal_rate Given gimbal rate
   */
  void Execute(const roborts_msgs::GimbalRate &gimbal_rate);
  /**
   * @brief Update the current gimbal executor state
   * @return Current gimbal executor state(same with behavior state)
   */

   void Execute_ShootCmd(const uint16_t &mode_cmd, const uint16_t &num);
   void Execute_FricWhl(const bool key, const double speed);

  BehaviorState Update();
  /**
   * @brief Cancel the current task and deal with the mode transition
   */
  void Cancel();
  void PublishMsgs();

 private:

  ros::NodeHandle nh_;

  //! execution mode of the executor
  ExcutionMode excution_mode_;
  //! execution state of the executor (same with behavior state)
  BehaviorState execution_state_;

  //! gimbal rate control publisher in ROS
  ros::Publisher cmd_gimbal_rate_pub_;
  //! zero gimbal rate in form of ROS roborts_msgs::GimbalRate
  roborts_msgs::GimbalRate zero_gimbal_rate_;

  //! gimbal angle control publisher in ROS
  ros::Publisher cmd_gimbal_angle_pub_;

  ros::ServiceClient shoot_client_;

  ros::ServiceClient fric_wheel_client_;

  //! pid controller actionlib client
  actionlib::SimpleActionClient<roborts_msgs::PIDControllerTowardAngularAction> pid_controller_client_;
  roborts_msgs::PIDControllerTowardAngularGoal pid_controller_toward_angular_goal_;


  void PIDControllerFeedbackCallback(const roborts_msgs::PIDControllerTowardAngularFeedbackConstPtr &pid_controller_toward_angular_feedback);
};
}


#endif //ROBORTS_DECISION_GIMBAL_EXECUTOR_H