//
// Created by kehan on 2020/2/28.
//


#include <tuple>
#include "my_robot.h"

using namespace roborts_decision;

MyRobot::MyRobot(RobotId id, const ros::NodeHandle &nh) :
    nh_(nh),
    id_(id),
    remaining_hp_(2000),
    max_hp_(2000),
    current_heat_(0),
    is_survival_(true),
    remaining_projectiles_(0),
    no_move_(false),
    no_shoot_(false) {

  robot_type_ = RobotType::UNKNOWN_TYPE;
//  if (id_ == RED1 || id_ == BLUE1) {
//    remaining_projectiles_ = 50;
//  } else {
//    remaining_projectiles_ = 0;
//  }

  p_chassis_executor_ = std::make_shared<ChassisExecutor>(nh_);
  p_gimbal_executor_ = std::make_shared<GimbalExecutor>(nh_);

  // TODO
  current_behavior_ = MyRobotBehavior::GOAL;

  std::string armors_under_attack_topic("armors_under_attack");
  nh_.param("armors_under_attack_topic", armors_under_attack_topic, armors_under_attack_topic);
  armors_under_attack_sub_ =
      nh_.subscribe<roborts_msgs::RobotDamage>(armors_under_attack_topic, 1, &MyRobot::ArmorsUnderAttackCallback, this);

  std::string heat_topic("heat");
  nh_.param("heat_topic", heat_topic, heat_topic);
  heat_sub_ = heat_sub_ = nh_.subscribe<roborts_msgs::RobotHeat>(
      heat_topic, 1, &MyRobot::HeatCallback, this);

  std::string armors_in_eyes_topic("armors_in_eyes");
  nh_.param("armors_in_eyes_topic", armors_in_eyes_topic, armors_in_eyes_topic);
  armors_in_eyes_sub_ =
      nh_.subscribe<roborts_msgs::ArmorsDetected>(armors_in_eyes_topic, 1, &MyRobot::ArmorsInEyesCallback, this);

  std::string robot_status_topic("robot_status");
  nh_.param("robot_status_topic", robot_status_topic, robot_status_topic);
  robot_status_sub_ = nh_.subscribe<roborts_msgs::RobotStatus>(
      robot_status_topic, 1, &MyRobot::RobotStatusCallback, this);

  robot_map_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "true_pose", 1, &MyRobot::ChassisMapPoseCallback, this);

  shoot_pub_ = nh_.advertise<std_msgs::Bool>("shoot", 10);

  remaining_projectiles_sub_ = nh_.subscribe<roborts_msgs::ShootInfo>(
      "shoot_info", 1, &MyRobot::RemainingProjectilesCallback, this);
  tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
  tf_thread_ptr_ = std::make_shared<std::thread>([&]() {
    ros::Rate loop_rate(30);
    while (ros::ok()) {
      // UpdateChassisMapPose();
      UpdateChassisOdomPose();
      UpdateGimbalMapPose();
      UpdateGimbalOdomPose();
      loop_rate.sleep();
    }
  });
  tf_thread_ptr_->detach();
  p_ros_spin_thread_ = std::make_shared<std::thread>([&]() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
  });
  p_ros_spin_thread_->detach();

  this->setNextPoint({0, 0});
  this->setSignal(Signal::NONE);
}

MyRobot::~MyRobot() = default;

void MyRobot::ArmorsUnderAttackCallback(const roborts_msgs::RobotDamage::ConstPtr &msg) {
  // TODO
}

void MyRobot::HeatCallback(const roborts_msgs::RobotHeat::ConstPtr &msg) {
  current_heat_ = msg->shooter_heat;
}

void MyRobot::RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &msg) {
  robot_type_ = static_cast<RobotType>(msg->id);
  remaining_hp_ = msg->remain_hp;
  max_hp_ = msg->max_hp;
  no_move_ = !msg->chassis_output;
  no_shoot_ = !msg->shooter_output;
}

void MyRobot::ArmorsInEyesCallback(const roborts_msgs::ArmorsDetected::ConstPtr &msg) {
  armors_in_eyes_ = *msg;
}

void MyRobot::RemainingProjectilesCallback(const roborts_msgs::ShootInfo::ConstPtr &msg) {
  remaining_projectiles_ = msg->remain_bullet;
}

RobotId MyRobot::GetId() const {
  return id_;
}

RobotType MyRobot::getRobotType() const {
  return robot_type_;
}

std::string MyRobot::robotTypeToString(const RobotType robot_type) {
  switch (robot_type) {
  case roborts_decision::RED_1: {
    return "red1";
    break;
  }
  case roborts_decision::RED_2: {
    return "red2";
    break;
  }
  case roborts_decision::BLUE_1: {
    return "blue1";
    break;
  }
  case roborts_decision::BLUE_2: {
    return "blue2";
    break;
  }
  default: {
    return "unknown";
  }
  }
}

void MyRobot::setRobotType(RobotType robot_type) {
    robot_type_ = robot_type;
}

int MyRobot::getHp() const {
  return remaining_hp_;
}

int MyRobot::getCurrentHeat() const {
  return current_heat_;
}

int MyRobot::getRemainingProjectiles() const {
  return remaining_projectiles_;
}

bool MyRobot::isSurvival() const {
  return is_survival_;
}

void MyRobot::setIsSurvival(bool is_survival) {
  is_survival_ = is_survival;
}

/**
 * 暂定使用方法为如果有被攻击的装甲板,size不为0,并且返回ArmorId
 * @return
 */
const std::vector<ArmorId> &MyRobot::getArmorsUnderAttack() const {
  return armors_under_attack_;
}

const roborts_msgs::ArmorsDetected &MyRobot::getArmorsInEyes() const {
  return armors_in_eyes_;
}

const geometry_msgs::PoseStamped &MyRobot::getChassisMapPose() const {
  return chassis_map_pose_;
}

const geometry_msgs::PoseStamped &MyRobot::getChassisOdomPose() {
  return chassis_odom_pose_;
}

const geometry_msgs::PoseStamped &MyRobot::getGimbalMapPose() {
  return gimbal_map_pose_;
}

const geometry_msgs::PoseStamped &MyRobot::getGimbalOdomPose() {
  return gimbal_odom_pose_;
}

// const geometry_msgs::PoseStamped &MyRobot::GetCurrentGoal() const {
//   return current_goal_;
// }

MyRobotBehavior MyRobot::getCurrentBehavior() const {
  return current_behavior_;
}

void MyRobot::setCurrentBehavior(MyRobotBehavior current_behavior) {
  current_behavior_ = current_behavior;
}

bool MyRobot::operator==(const MyRobot &rhs) const {
  return id_ == rhs.id_;
}

bool MyRobot::operator!=(const MyRobot &rhs) const {
  return !(rhs == *this);
}

// ChassisExecutor* MyRobot::GetChassisExecutor() {
//   return &chassis_executor_;
// }
//
// GimbalExecutor* MyRobot::GetGimbalExecutor() {
//   return &gimbal_executor_;
// }

// void MyRobot::UpdateChassisMapPose() {
//   tf::Stamped<tf::Pose> chassis_tf_pose;
//   chassis_tf_pose.setIdentity();
//
//   chassis_tf_pose.frame_id_ = "base_link";
//   chassis_tf_pose.stamp_ = ros::Time();
//   try {
//     geometry_msgs::PoseStamped chassis_pose;
//     tf::poseStampedTFToMsg(chassis_tf_pose, chassis_pose);
//     tf_ptr_->transformPose("map", chassis_pose, chassis_map_pose_);
//   }
//   catch (tf::LookupException &ex) {
//     ROS_ERROR("Transform Error looking up chassis pose: %s", ex.what());
//   }
// }

void MyRobot::ChassisMapPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  chassis_map_pose_ = *msg;
}

bool MyRobot::isNoMove() const {
  return no_move_;
}

bool MyRobot::isNoShoot() const {
  return no_shoot_;
}

/** TODO
 * 返回我方机器人是否被攻击
 * @return
 */
bool MyRobot::isShot() const{
  return false;
}

std::shared_ptr<ChassisExecutor> MyRobot::getPChassisExecutor() {
  return p_chassis_executor_;
}

std::shared_ptr<GimbalExecutor> MyRobot::getPGimbalExecutor() {
  return p_gimbal_executor_;
}

uint32_t MyRobot::getStatusCode() {
  return p_chassis_executor_->GetErrorCode();
}

void MyRobot::shoot() {
  std_msgs::Bool shoot;
  shoot.data = true;
  shoot_pub_.publish(shoot);
}

void MyRobot::UpdateChassisOdomPose() {
  if (robot_type_ == UNKNOWN_TYPE) {
    return;
  }
  tf::Stamped<tf::Pose> chassis_tf_pose;
  chassis_tf_pose.setIdentity();

  chassis_tf_pose.frame_id_ = robotTypeToString(robot_type_) + "/base_link";
  chassis_tf_pose.stamp_ = ros::Time();
  try {
    geometry_msgs::PoseStamped chassis_pose;
    tf::poseStampedTFToMsg(chassis_tf_pose, chassis_pose);
    tf_ptr_->transformPose(robotTypeToString(robot_type_) + "/odom",
                           chassis_pose, chassis_odom_pose_);
  } catch (tf::LookupException &ex) {
    ROS_ERROR("Transform Error looking up chassis odom pose: %s", ex.what());
  } catch (tf2::ExtrapolationException &ex) {
    ROS_ERROR("Transform Error Extrapolation chassis odom pose: %s", ex.what());
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Transform Error Connectivity chassis odom pose: %s", ex.what());
  }
}

void MyRobot::UpdateGimbalMapPose() {
  if (robot_type_ == UNKNOWN_TYPE) {
    return;
  }
  tf::Stamped<tf::Pose> gimbal_tf_pose;
  gimbal_tf_pose.setIdentity();

  gimbal_tf_pose.frame_id_ = robotTypeToString(robot_type_) + "/shoot_link";
  gimbal_tf_pose.stamp_ = ros::Time();
  try {
    geometry_msgs::PoseStamped gimbal_pose;
    tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
    tf_ptr_->transformPose("map", gimbal_pose, gimbal_map_pose_);
  } catch (tf::LookupException &ex) {
    ROS_ERROR("Transform Error looking up gimbal map pose: %s", ex.what());
  } catch (tf2::ExtrapolationException &ex) {
    ROS_ERROR("Transform Error Extrapolation chassis map pose: %s", ex.what());
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Transform Error Connectivity chassis map pose: %s", ex.what());
  }
}

void MyRobot::UpdateGimbalOdomPose() {
  if (robot_type_ == UNKNOWN_TYPE) {
    return;
  }
  tf::Stamped<tf::Pose> gimbal_tf_pose;
  gimbal_tf_pose.setIdentity();

  gimbal_tf_pose.frame_id_ = robotTypeToString(robot_type_) + "/shoot_link";
  gimbal_tf_pose.stamp_ = ros::Time();
  try {
    geometry_msgs::PoseStamped gimbal_pose;
    tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
    tf_ptr_->transformPose(robotTypeToString(robot_type_) + "/odom",
                           gimbal_pose, gimbal_odom_pose_);
  } catch (tf::LookupException &ex) {
    ROS_ERROR("Transform Error looking up gimbal odom pose: %s", ex.what());
  } catch (tf2::ExtrapolationException &ex) {
    ROS_ERROR("Transform Error Extrapolation gimbal odom pose: %s", ex.what());
  } catch (tf::ConnectivityException &ex) {
    ROS_ERROR("Transform Error Connectivity gimbal odom pose: %s", ex.what());
  }
}

/*
 * 设置机器人进行原地旋转，具体旋转参数由调用函数确定
 */
void MyRobot::spinInPlace(const double &angularVelocity){
    geometry_msgs::Twist twist;
    twist.angular.z = angularVelocity;
    this->getPChassisExecutor()->Execute(twist);
}

/*
* 实现机器人旋转前进
* 只需传跑位点，转速在内部确定
*/
void MyRobot::spinForword(const double &tarX, const double &tarY){
    const double PI = 3.1415926535;
    geometry_msgs::PoseStamped goal;

    //目标点位置
    goal.pose.position.x = tarX;
    goal.pose.position.y = tarY;
    goal.pose.position.z = 0;

    //四元数转化
    //暂时定1s转0.5圈，之后可以调试,1s是10Hz，故一帧转PI/10
    auto &orientation = goal.pose.orientation;
    std::tie(orientation.w, orientation.x, orientation.y, orientation.z)
        =BlackboardRaw::turnAngleToPose(0,0, PI / (data::Hz) / 2 + this->getChassisMapPose().pose.orientation.w);
    this->getPChassisExecutor()->Execute(goal);

}

void MyRobot::keepDirectionMove(double tar_x, double tar_y, double direction) {

    geometry_msgs::PoseStamped chassisGoal;
    auto &orientation = chassisGoal.pose.orientation;
    std::tie(orientation.w, orientation.x, orientation.y, orientation.z) = BlackboardRaw::turnAngleToPose(0, 0, direction);

    chassisGoal.pose.position.x = tar_x;
    chassisGoal.pose.position.y = tar_y;
    chassisGoal.pose.position.z = 0;

    this->getPChassisExecutor()->Execute(chassisGoal);
}

double MyRobot::TowardWPosShoot(double tar_x, double tar_y, double tar_z) {

  roborts_msgs::GimbalAngle gimbal_angle;
  double deviation_angle = 0;

  geometry_msgs::PoseStamped gimbal_pose = this->getGimbalMapPose();
  double dx = tar_x - gimbal_pose.pose.position.x;
  double dy = tar_y - gimbal_pose.pose.position.y;
  double dz = tar_z - gimbal_pose.pose.position.z;

  gimbal_angle.pitch_mode = true;
  gimbal_angle.yaw_mode = true;
  // TODO:云台横向旋转（-90-90），纵向（-90-90）
  gimbal_angle.pitch_angle = atan2(dy, dx);
  gimbal_angle.yaw_angle = atan2(dz, sqrt(dx*dx+dy*dy));

  auto sign = [](double x) {return x > 0 ? 1 : -1;};

  if(abs(gimbal_angle.pitch_angle) > 90) {
    deviation_angle = gimbal_angle.pitch_angle - sign(gimbal_angle.pitch_angle) * 90;
    gimbal_angle.pitch_angle = sign(gimbal_angle.pitch_angle) * 90;
  }

//  this->getPGimbalExecutor()->Execute(gimbalAngle);

  roborts_msgs::GimbalAngle executor_gimbal_angle;

  geometry_msgs::Point32 target;
  target.x = tar_x;
  target.y = tar_y;
  target.z = tar_z;

  this->getPGimbalExecutor()->Execute(target, executor_gimbal_angle); //计算位姿
  setGimbalOdomPose(executor_gimbal_angle.pitch_angle, executor_gimbal_angle.yaw_angle);

  return deviation_angle;
}


roborts_msgs::GimbalAngle MyRobot::shootAimed(geometry_msgs::Point32 &target){

  ros::Rate loop_rate(data::Hz);

  roborts_msgs::GimbalAngle executor_gimbal_angle;

  this->getPGimbalExecutor()->Execute(target, executor_gimbal_angle); //计算位姿
  // SetGimbalMapPose(executor_gimbal_angle.pitch_angle, executor_gimbal_angle.yaw_angle);
  setGimbalOdomPose(executor_gimbal_angle.pitch_angle, executor_gimbal_angle.yaw_angle);
  // gimbal_executor_->Execute_FricWhl(true,1230);

  // gimbal_executor_->Execute_ShootCmd(1,1);
  //这个函数似乎没什么作用，可能就是用来传递消息，暂时保留
  this->getPGimbalExecutor()->Execute_ShootCmd(0, 0);
  // gimbal_executor_->Execute_FricWhl(true);

  // 设置摩擦轮速度，如果需要关闭射击，速度调至1100即可
  this->getPGimbalExecutor()->Execute_FricWhl(true, 1200);
  loop_rate.sleep();
  // 关闭射击
  this->getPGimbalExecutor()->Execute_FricWhl(true, 1100);


  return executor_gimbal_angle;
}

/*
 * 在odom坐标系下设立位姿
 */
void MyRobot::setGimbalOdomPose(const double &gimbal_goal_map_pitch, const double &gimbal_goal_map_yaw) {

    try {
        geometry_msgs::PoseStamped gimbal_pose;
        gimbal_pose.header.frame_id = "map";
        gimbal_pose.header.stamp = ros::Time();
        gimbal_pose.pose.orientation =
                tf::createQuaternionMsgFromRollPitchYaw(0, gimbal_goal_map_pitch, gimbal_goal_map_yaw);

        geometry_msgs::PoseStamped gimbal_odom_pose_;
        //世界坐标转换成odom坐标
        tf_ptr_->transformPose("odom", gimbal_pose, gimbal_odom_pose_);
        ROS_WARN("gimbal pose %lf", tf::getYaw(gimbal_odom_pose_.pose.orientation));
        this->getPGimbalExecutor()->Execute(gimbal_odom_pose_, GimbalExecutor::GoalMode::GOAL_MODE_USE_PID);
    }
    catch (tf::LookupException &ex) {
        ROS_ERROR("Transform Error looking up gimbal pose: %s", ex.what());
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("Transform Error gimbal pose: %s", ex.what());
    }

}

/*
 * 保持机器人原地静止
 */
void MyRobot::stayStill(){
  const geometry_msgs::Twist twist;
  this->getPChassisExecutor()->Execute(twist);
}

void MyRobot::avoidanceMove(double x, double y) {
}

double MyRobot::getSafeDirection(const geometry_msgs::PoseStamped &pose_enemy) {
  const geometry_msgs::PoseStamped &pose_home = this->getChassisMapPose();
  double angle = roborts_decision::BlackboardRaw::GetAngle(pose_home, pose_enemy);
  angle = angle + roborts_decision::data::PI / 4;
  if (angle > 180){
    angle = angle-360;
  }
  // TODO 暂定返回为与敌方机器人正面相对的角,可能会出问题
  return angle;
}

// TODO: 未完成
double MyRobot::getEffectiveShootDistance() const {
    return 5;
}

const roborts_common::Point2D &MyRobot::getNextPoint() const {
    return nextPoint_;
}

void MyRobot::setNextPoint(const roborts_common::Point2D &nextPoint) {
    MyRobot::nextPoint_ = nextPoint;
}

Signal MyRobot::getSignal() const {
    return signal_;
}

void MyRobot::setSignal(Signal signal) {
    MyRobot::signal_ = signal;
}

void MyRobot::update() {

    this->setNextPoint({0, 0});
    this->setSignal(Signal::NONE);
}

// TODO: 未完成
bool MyRobot::shoot(double speed, int number) {

  ros::Rate loop_rate(data::Hz);

  if (number == data::MAX) {
    number = this->shootBulletsMaxNumber(speed);
  }

  std_msgs::Bool msg;
  msg.data = true;
  for (int kI = 0; kI < 8; kI ++) {
    std::cout<<"子弹发射次数"<<kI << std::endl;
    shoot_pub_.publish(msg);
  }




  // TODO 此部分代码应用与实物,仿真环境暂时删除此部分
/*  //这个函数似乎没什么作用，可能就是用来传递消息，暂时保留
  this->getPGimbalExecutor()->Execute_ShootCmd(0, 0);

  // 设置摩擦轮速度，如果需要关闭射击，速度调至1100即可
  this->getPGimbalExecutor()->Execute_FricWhl(true, 1200);
  loop_rate.sleep();
  // 关闭射击
  this->getPGimbalExecutor()->Execute_FricWhl(true, 1100);*/

  return true;
}

roborts_common::Point2D MyRobot::getMyRobotPoint() const{
  roborts_common::Point2D posePoint(this->getChassisMapPose().pose.position.x, this->getChassisMapPose().pose.position.y);
  return posePoint;
}


int MyRobot::shootBulletsMaxNumber(double speed) const {
  int num=10;
  return num;
}

double MyRobot::changeFrictionWheelSpeedToShootSpeed(double frictionWhellSpeed) {
    return 0;
}
double MyRobot::GetArmorTowards(ArmorId armor_id) {
  double robot_angle = tf::getYaw(this->getChassisMapPose().pose.orientation);
  double armor_angle{robot_angle};
  switch (armor_id) {
    case FRONT: armor_angle = armor_angle; break;
    case RIGHT: armor_angle -= data::PI/2; break;
    case BACK: armor_angle -= data::PI; break;
    case LEFT: armor_angle -= data::PI*3/2; break;
    default: ;
  }
  if (armor_angle < -data::PI){
    armor_angle += data::PI * 2;
  }else if (armor_angle > data::PI){
    armor_angle -= data::PI * 2;
  }

  return armor_angle;
}

int MyRobot::GetShootEnemyId(){
  return this->shoot_enemy_id_;
}
void MyRobot::SetShootEnemyId(const int id){
  this->shoot_enemy_id_ = id;
}
int MyRobot::GetShootNulletsNumber() {
  return this->shoot_bullets_this_time_;
}
void MyRobot::SetShootNulletsNumber(const int num_bullets) {
  this->shoot_bullets_this_time_ = num_bullets;
}


// value = 血量×k1+子弹×k2×有无子弹+热量×k3+不动惩罚系数
double roborts_decision::MyRobot::getState() const {
  using roborts_decision::data;
  return this->getHp() /data::HP * data::kBlood
      + this->getRemainingProjectiles() /data::PROJECTILESPACKAGE * data::kBullet * this->tNoShoot
      + this->getCurrentHeat() /data::MAXHEAT * data::kHeat
      + this->tNoMove;
}

void roborts_decision::MyRobot::SetHomeShootParameter() {
  this->tNoShoot = this->isNoShoot() ? 0.0 : 1.0;
}

void roborts_decision::MyRobot::SetHomeMoveParameter() {
  this->tNoMove = this->isNoMove() ? -0.52 : 0.0;
}

