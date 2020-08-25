//
// Created by kehan on 2020/2/28.
//

#include <tf/transform_datatypes.h>
#include "enemy_robot.h"

roborts_decision::EnemyRobot::EnemyRobot(const RobotId &robot_id,
                                         const MyColor my_color)
    : id_(robot_id), is_survival_(true), color_(my_color) {

}

roborts_decision::EnemyRobot::~EnemyRobot() = default;

void roborts_decision::EnemyRobot::Init() {
  if (color_ == RED and id_ == ENEMY_ROBOT_1) {
    this->SetRobotType(RED_1);
    enemy_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/red1/ground_truth/state", 1, &EnemyRobot::EnemyRobotPoseCallback,
        this);
  } else if (color_ == RED and id_ == ENEMY_ROBOT_2) {
    this->SetRobotType(RED_2);
    enemy_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/red2/ground_truth/state", 1, &EnemyRobot::EnemyRobotPoseCallback,
        this);
  } else if (color_ == BLUE and id_ == ENEMY_ROBOT_1) {
    this->SetRobotType(BLUE_1);
    enemy_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/blue1/ground_truth/state", 1, &EnemyRobot::EnemyRobotPoseCallback,
        this);
  } else if (color_ == BLUE and id_ == ENEMY_ROBOT_2) {
    this->SetRobotType(BLUE_2);
    enemy_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/blue2/ground_truth/state", 1, &EnemyRobot::EnemyRobotPoseCallback,
        this);
  } else if (color_ == UNKNOWN_COLOR) {
    ROS_ERROR("Color has problem in Enemy init!");
    color_ = UNKNOWN_COLOR;
  }

  this->hp_ = 2000;
  this->num_of_bullets_ = id_ == ENEMY_ROBOT_1? 50 : 0;
  this->can_move_ = true;
  this->can_shoot_ = this->num_of_bullets_ == 50 ? true: false;

}

roborts_decision::RobotId roborts_decision::EnemyRobot::GetId() const {
  return id_;
}

roborts_decision::RobotType roborts_decision::EnemyRobot::GetRobotType() const {
  return robot_type_;
}

void roborts_decision::EnemyRobot::SetRobotType(roborts_decision::RobotType robot_type) {
  robot_type_ = robot_type;
}

void roborts_decision::EnemyRobot::SetColor(MyColor color) {
  color_ = color;
}

roborts_decision::MyColor roborts_decision::EnemyRobot::GetColor() const {
  return this->color_;
}

bool roborts_decision::EnemyRobot::IsSurvival() const {
  return is_survival_;
}

void roborts_decision::EnemyRobot::SetIsSurvival(bool is_survival) {
  is_survival_ = is_survival;
}

const geometry_msgs::PoseStamped &
roborts_decision::EnemyRobot::GetPose() const {
  return pose_;
}

void roborts_decision::EnemyRobot::SetPose(
    const ros::Time &stamp, const geometry_msgs::Point &position) {
  pose_.header.stamp = stamp;
  pose_.pose.position = position;
}

void roborts_decision::EnemyRobot::EnemyRobotPoseCallback(
    const nav_msgs::Odometry::ConstPtr &msg) {

  pose_.pose.position.x = msg->pose.pose.position.x;
  pose_.pose.position.y = msg->pose.pose.position.y;
  pose_.pose.position.z = msg->pose.pose.position.z;
  pose_.pose.orientation.x = msg->pose.pose.orientation.x;
  pose_.pose.orientation.y = msg->pose.pose.orientation.y;
  pose_.pose.orientation.z = msg->pose.pose.orientation.z;
  pose_.pose.orientation.w = msg->pose.pose.orientation.w;
  pose_.header.frame_id = msg->header.frame_id;
  pose_.header.seq = msg->header.seq;
  pose_.header.stamp = msg->header.stamp;

/*
  ROS_ERROR("%lf %lf",pose_.pose.position.x, pose_.pose.position.y);
*/
}

bool roborts_decision::EnemyRobot::IsDetected() const { return is_detected_; }

void roborts_decision::EnemyRobot::SetIsDetected(bool is_detected) {
  is_detected_ = is_detected;
}

// value = 血量×k1+子弹×k2×有无子弹+热量×k3+不动惩罚系数
double roborts_decision::EnemyRobot::getState() const {
  using roborts_decision::data;
  return this->getHp() /data::HP * data::kBlood
      + this->getRemainingProjectiles() /data::PROJECTILESPACKAGE * data::kBullet * this->tNoShoot
      + this->getCurrentHeat() /data::MAXHEAT * data::kHeat
      + this->tNoMove;
}


double roborts_decision::EnemyRobot::getStanderdState() const {

    constexpr static double KStanderedHp = 0.5;
    constexpr static double KStanderedProjectiles = 0.42;
    constexpr static double KStanderedHeat = 0.66;

    return this->getHp() * KStanderedHp / roborts_decision::data::HP * roborts_decision::data::kBlood
           + this->getRemainingProjectiles() * KStanderedProjectiles / roborts_decision::data::PROJECTILESPACKAGE * roborts_decision::data::kBullet * this->tNoShoot
           + this->getCurrentHeat() * KStanderedHeat / roborts_decision::data::MAXHEAT * roborts_decision::data::kHeat
           + this->tNoMove;

}

// TODO: 未完成，获得子弹
int roborts_decision::EnemyRobot::getRemainingProjectiles() const {
  return this->num_of_bullets_;
}
void roborts_decision::EnemyRobot::SetRemainingProjectiles(const int bullets){
  this->num_of_bullets_ = bullets;
}

// TODO: 未完成
int roborts_decision::EnemyRobot::getHp() const {
  return this->hp_;
}

void roborts_decision::EnemyRobot::SetHp(const int hp){
  this->hp_ = hp;
}

// TODO: 未完成
int roborts_decision::EnemyRobot::getCurrentHeat() const {
    return this->current_heat_;
}

void roborts_decision::EnemyRobot::SetCurrentHeat(const int heat){
  this->current_heat_ = heat;
}

// TODO: 未完成
bool roborts_decision::EnemyRobot::canMove() const {
  return this->can_move_;
}

bool roborts_decision::EnemyRobot::SetCanMove(const bool canMove){
  this->can_move_ = canMove;
}

// TODO: 未完成
bool roborts_decision::EnemyRobot::canShoot() const {
  return this->can_shoot_;
}

void roborts_decision::EnemyRobot::SetCanShoot(const bool canShoot){
  this->can_shoot_ = canShoot;
}

double roborts_decision::EnemyRobot::getRelativeAngle(roborts_common::Point2D point) const {

  const auto& enemyPos = this->GetPose().pose.position;
  roborts_common::LineSegment2D towardLine(roborts_common::Point2D(enemyPos.x, enemyPos.y), point);
  roborts_common::Point2D unitDirection1 = towardLine.UnitDirection();

  double angleEnemy = tf::getYaw(this->GetPose().pose.orientation);
  roborts_common::Point2D unitDirection2(cos(angleEnemy), sin(angleEnemy));

  roborts_common::Point2D tarVector = unitDirection1 + unitDirection2;

  return atan2(tarVector.Y(), tarVector.X());
}

// TODO: 未完成
roborts_decision::Posture roborts_decision::EnemyRobot::getCurrentPosture() const{
  return roborts_decision::Posture::TILTANGLE;
}

roborts_common::Point2D roborts_decision::EnemyRobot::getEnemyPoint() const{
  roborts_common::Point2D posePoint(this->GetPose().pose.position.x, this->GetPose().pose.position.y);
  return posePoint;
}

void roborts_decision::EnemyRobot::SetEnemyShootParameter() {
  this->tNoShoot = this->canShoot() ? 1.0 : 0.0;
}

void roborts_decision::EnemyRobot::SetEnemyMoveParameter() {
  this->tNoMove = this->canMove() ? 0.0 : -0.52;
}

bool roborts_decision::EnemyRobot::operator==(const EnemyRobot &e_r)const{
    return e_r.GetId() == this->GetId();
}



