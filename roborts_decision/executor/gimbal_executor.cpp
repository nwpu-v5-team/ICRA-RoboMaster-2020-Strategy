#include "gimbal_executor.h"
namespace roborts_decision {
GimbalExecutor::GimbalExecutor(const ros::NodeHandle &nh)
    : nh_(nh),
      excution_mode_(ExcutionMode::IDLE_MODE),
      execution_state_(BehaviorState::IDLE),
      pid_controller_client_("pid_planner_gimbal_node_action",
                             true) {

  cmd_gimbal_angle_pub_ =
      nh_.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);
  cmd_gimbal_rate_pub_ =
      nh_.advertise<roborts_msgs::GimbalRate>("cmd_gimbal_rate", 1);

  shoot_client_ = nh_.serviceClient<roborts_msgs::ShootCmd>("cmd_shoot");
  fric_wheel_client_ =
      nh_.serviceClient<roborts_msgs::FricWhl>("cmd_fric_wheel");

/*
  pid_controller_client_.waitForServer();
*/
  ROS_INFO("PID controller gimbal server start!");
}

void GimbalExecutor::Execute(const roborts_msgs::GimbalAngle &gimbal_angle) {

  if (excution_mode_ == ExcutionMode::PID_MODE) {
    Cancel();
  }

  excution_mode_ = ExcutionMode::ANGLE_MODE;
    ROS_ERROR("pitch_anggel = %f, yaw_angle = %f",
              gimbal_angle.pitch_angle,
              gimbal_angle.yaw_angle);

  cmd_gimbal_angle_pub_.publish(gimbal_angle);

}

void GimbalExecutor::Execute(const roborts_msgs::GimbalRate &gimbal_rate) {

  if (excution_mode_ == ExcutionMode::PID_MODE) {
    Cancel();
  }

  excution_mode_ = ExcutionMode::RATE_MODE;
  cmd_gimbal_rate_pub_.publish(gimbal_rate);
}

void GimbalExecutor::Execute(const geometry_msgs::PoseStamped &gimbal_angle, GoalMode goal_mode) {

  if (goal_mode == GoalMode::GOAL_MODE_USE_PID) {

    excution_mode_ = ExcutionMode::PID_MODE;

    ROS_ERROR("Execute gimbal send goal! \n");
    pid_controller_toward_angular_goal_.goal = gimbal_angle;
    pid_controller_client_.sendGoal(pid_controller_toward_angular_goal_,
                                    PIDControllerClient::SimpleDoneCallback(),
                                    PIDControllerClient::SimpleActiveCallback(),
                                    boost::bind(&GimbalExecutor::PIDControllerFeedbackCallback, this, _1));
  }
}

void GimbalExecutor::Execute(geometry_msgs::Point32 &target, roborts_msgs::GimbalAngle &executor_gimbal_angle_){
    //geometry_msgs::Point32 target_3d;

    GimbalContrl gimbal_control_;
    float pitch, yaw;
   // ROS_INFO("target.x = %lf, target.y = %lf, target.z = %lf",
   //          target.x,
   //          target.y,
   //          target.z);

   gimbal_control_.Transform(target, pitch, yaw);

   // ROS_WARN("pitch_angle = %lf, yaw_angle = %lf",
   //         pitch,
   //         yaw);

   executor_gimbal_angle_.yaw_mode = true;
   executor_gimbal_angle_.pitch_mode = false;
   executor_gimbal_angle_.yaw_angle = yaw ;   //原本是 * 0.7
   executor_gimbal_angle_.pitch_angle = pitch;

//   PublishMsgs();
//    cmd_gimbal_angle_pub_.publish()

}
void GimbalExecutor::Execute_ShootCmd(const uint16_t &mode_cmd, const uint16_t &num) {
    roborts_msgs::ShootCmd shoot_cmd;
    shoot_cmd.request.mode = mode_cmd;
    shoot_cmd.request.number = num;

    if(shoot_client_.call(shoot_cmd))
    {
        ROS_INFO("shootcmd_response: %d", (int)(shoot_cmd.response.received));
    }
    else
    {
        ROS_ERROR("Failed to call service cmd_shoot");
    }

}

void GimbalExecutor::Execute_FricWhl(const bool key, const double speed) {
    roborts_msgs::FricWhl fricwhl_ctrl;
    fricwhl_ctrl.request.open = key;
    fricwhl_ctrl.request.speed = speed;
    if(fric_wheel_client_.call(fricwhl_ctrl))
    {
        ROS_INFO("fricwhl_response: %d", (int)(fricwhl_ctrl.response.received));
    }
    else
    {
        ROS_ERROR("Failed to call service cmd_fric_wheel");
    }
}

//void GimbalExecutor::PublishMsgs() {
//    cmd_gimbal_angle_pub_.publish(executor_gimbal_angle_);
//}

BehaviorState GimbalExecutor::Update() {
  actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::LOST;
  switch (excution_mode_) {
    case ExcutionMode::IDLE_MODE:execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::PID_MODE:state = pid_controller_client_.getState();
      if (state == actionlib::SimpleClientGoalState::ACTIVE) {
        ROS_INFO("%s : pid_controller_gimbal_client ACTIVE", __FUNCTION__);
        execution_state_ = BehaviorState::RUNNING;
      } else if (state == actionlib::SimpleClientGoalState::PENDING) {
        ROS_INFO("%s : pid_controller_gimbal_client PENDING", __FUNCTION__);
        execution_state_ = BehaviorState::RUNNING;

      } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("%s : pid_controller_gimbal_client SUCCEEDED", __FUNCTION__);
        execution_state_ = BehaviorState::SUCCESS;

      } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("%s : pid_controller_gimbal_client ABORTED", __FUNCTION__);
        execution_state_ = BehaviorState::FAILURE;

      } else {
        ROS_ERROR("pid_controller_gimbal_client Error: %s", state.toString().c_str());
        execution_state_ = BehaviorState::FAILURE;
      }
      break;

    case ExcutionMode::ANGLE_MODE:execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::RATE_MODE:execution_state_ = BehaviorState::RUNNING;
      break;

    default:ROS_ERROR("Wrong Execution Mode");
  }
  return execution_state_;
}

void GimbalExecutor::Cancel() {
  switch (excution_mode_) {
    case ExcutionMode::IDLE_MODE:ROS_WARN("Nothing to be canceled.");
      break;

    case ExcutionMode::ANGLE_MODE:cmd_gimbal_rate_pub_.publish(zero_gimbal_rate_);
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::RATE_MODE:cmd_gimbal_rate_pub_.publish(zero_gimbal_rate_);
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::PID_MODE:pid_controller_client_.cancelGoal();
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    default:ROS_ERROR("Wrong Execution Mode");
  }

}

void GimbalExecutor::PIDControllerFeedbackCallback(const roborts_msgs::PIDControllerTowardAngularFeedbackConstPtr &pid_controller_toward_angular_feedback) {
  ROS_INFO("The differ angle between gimbal and goal is %lf", pid_controller_toward_angular_feedback->differ_angle);
  //TODO
}

}
