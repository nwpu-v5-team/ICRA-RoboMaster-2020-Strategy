//
// Created by kehan on 2020/3/1.
//

#include <fstream>
#include <utility>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include "roborts_msgs/RobotStrategyState.h"
#include "blackboard.h"
#include "blackboard_raw.h"
#include "field.h"

using namespace roborts_decision;

std::shared_ptr<Blackboard> Blackboard::p_blackboard_ = nullptr;

Blackboard::Blackboard(std::shared_ptr<MyRobot> &p_myrobot1,
                       std::shared_ptr<MyRobot> &p_myrobot2,
                       const ros::NodeHandle &nh)
    : nh_(nh),
      enemy_robot_1_(RobotId::ENEMY_ROBOT_1, UNKNOWN_COLOR),
      enemy_robot_2_(RobotId::ENEMY_ROBOT_2, UNKNOWN_COLOR),
      my_color_(UNKNOWN_COLOR), p_my_robot1_(p_myrobot1), p_my_robot2_(p_myrobot2),
      prediction_system_(this->p_my_robot1_, this->p_my_robot2_, this->enemy_robot_1_, this->enemy_robot_2_) {
  //TODO color has problem
  while (true){
    if (p_my_robot2_->getRobotType() != UNKNOWN_TYPE){
      break;
    }
    ros::spinOnce();
  }
  if (p_my_robot1_->getRobotType() < 10 and p_my_robot2_->getRobotType() < 10) {
    my_color_ = RED;
    enemy_robot_1_.SetColor(BLUE);
    enemy_robot_2_.SetColor(BLUE);
    enemy_robot_1_.Init();
    enemy_robot_2_.Init();
  } else if (p_my_robot1_->getRobotType() > 10 and
          p_my_robot2_->getRobotType() > 10) {
    my_color_ = BLUE;
    enemy_robot_1_.SetColor(RED);
    enemy_robot_2_.SetColor(RED);
    enemy_robot_1_.Init();
    enemy_robot_2_.Init();
  //  enemy_robot_1_ = EnemyRobot(RobotId::ENEMY_ROBOT_1, RED);
  //  enemy_robot_2_ = EnemyRobot(RobotId::ENEMY_ROBOT_2, RED);
  } else {
    my_color_ = UNKNOWN_COLOR;
  }

  std::string outpost_camera_topic("/outpost_camera");
  nh_.param("outpost_camera_topic", outpost_camera_topic, outpost_camera_topic);
  outpose_camera_sub_ = nh_.subscribe<roborts_msgs::OutpostDetected>(
      outpost_camera_topic, 1, &Blackboard::OutpostDetectedCallback, this);

  // std::string robot_status_for_color_topic("/robot_status_for_color");
  // nh_.param("robot_status_for_color_topic", robot_status_for_color_topic,
  // robot_status_for_color_topic); color_sub_ =
  //     nh_.subscribe<roborts_msgs::RobotStatus>(robot_status_for_color_topic,
  //     1, &Blackboard::MyColorCallback, this);

  std::string game_survivor_topic("/game_survivor");
  nh_.param("game_survivor_topic", game_survivor_topic, game_survivor_topic);
  game_survivor_sub_ = nh_.subscribe<roborts_msgs::GameSurvivor>(
      game_survivor_topic, 1, &Blackboard::GameSurvivorCallback, this);

  field_ = Field::GetField();

  is_collide_pub_ = nh_.advertise<roborts_msgs::RobotStrategyState>("/IsCollide", 10);

  std::string defense_safe_value_string = LoadValue("defense_safe_value", "defense_field.data");
  this->defense_safe_value_ = std::stod(defense_safe_value_string.empty() ? "15.9" : defense_safe_value_string);
}

Blackboard::~Blackboard() = default;

std::shared_ptr<Blackboard> Blackboard::GetBlackboard(){

  if(p_blackboard_ == nullptr) {
    // TODO 写死我方机器人恒为红方,后期需要修改
    ros::NodeHandle nh;
    std::string robot_color;
    nh.getParam("robot_color", robot_color);

    auto p_my_robot1 = std::make_shared<MyRobot>(
        MY_ROBOT_1, ros::NodeHandle("/" + robot_color + "1"));
    auto p_my_robot2 = std::make_shared<MyRobot>(
        MY_ROBOT_2, ros::NodeHandle("/" + robot_color + "2"));
    sleep(1);

    p_blackboard_ = std::make_shared<Blackboard>(p_my_robot1, p_my_robot2);

    while ((fabs(p_blackboard_->GetEnemyRobot1().GetPose().pose.position.x)) <
           0.001) {
      ros::spinOnce();
      usleep(50e3);
    }

    return p_blackboard_;
  }else{
    return p_blackboard_;
  }
}



const std::shared_ptr<MyRobot> &Blackboard::GetMyRobot1() {
  return p_my_robot1_;
}

const std::shared_ptr<MyRobot> &Blackboard::GetMyRobot2() {
  return p_my_robot2_;
}

EnemyRobot &Blackboard::GetEnemyRobot1() {
  return enemy_robot_1_;
}

EnemyRobot &Blackboard::GetEnemyRobot2() {
  return enemy_robot_2_;
}

const std::shared_ptr<MyRobot> &Blackboard::GetAnotherRobot(const std::shared_ptr<MyRobot>& p_my_robot) const{
    if(p_my_robot1_ == p_my_robot){
        return p_my_robot2_;
    }else{
        return p_my_robot1_;
    }
}
const EnemyRobot &Blackboard::GetAnotherRobot(const EnemyRobot &enemy_robot){
    if(enemy_robot_1_ == enemy_robot){
        return enemy_robot_2_;
    }else{
        return enemy_robot_1_;
    }
}

MyColor Blackboard::GetMyColor() const {
  return my_color_;
}

void Blackboard::OutpostDetectedCallback(const roborts_msgs::OutpostDetected::ConstPtr &msg) {

  if (my_color_ == UNKNOWN_COLOR) {
    return;
  }

  if (my_color_ == RED) {
    enemy_robot_1_.SetIsDetected(msg->blue1_detected);
    enemy_robot_1_.SetPose(msg->header.stamp, msg->blue1);
    enemy_robot_2_.SetIsDetected(msg->blue2_detected);
    enemy_robot_2_.SetPose(msg->header.stamp, msg->blue2);
  } else if (my_color_ == BLUE) {
    enemy_robot_1_.SetIsDetected(msg->red1_detected);
    enemy_robot_1_.SetPose(msg->header.stamp, msg->red1);
    enemy_robot_2_.SetIsDetected(msg->red2_detected);
    enemy_robot_2_.SetPose(msg->header.stamp, msg->red2);
  }
}

void Blackboard::MyColorCallback(
    const roborts_msgs::RobotStatus::ConstPtr &msg) {
  // TODO
}

void Blackboard::GameSurvivorCallback(const roborts_msgs::GameSurvivor::ConstPtr &msg) {
  if (my_color_ == RED) {

    p_my_robot1_->setIsSurvival(msg->red3);
    p_my_robot2_->setIsSurvival(msg->red4);
    enemy_robot_1_.SetIsSurvival(msg->blue3);
    enemy_robot_2_.SetIsSurvival(msg->blue4);
  } else if (my_color_ == BLUE) {
    p_my_robot1_->setIsSurvival(msg->blue3);
    p_my_robot2_->setIsSurvival(msg->blue4);
    enemy_robot_1_.SetIsSurvival(msg->red3);
    enemy_robot_2_.SetIsSurvival(msg->red4);
  }
}

void Blackboard::NextDecisionCallBackDeprecated(const std_msgs::Int32::ConstPtr &msg){
  // TODO 所有情况 (17*9+11*11) * 2 * 4
  // TODO (大方格+小方格)*两个机器人*四种射击

  const int kParaToParse = msg->data;
  // 判断机器人
  // 对于跑位点编号从0开始,则到273即可
  const int kChooseHome = (kParaToParse%548)>273?2:1;
  int shoot_mode = (kParaToParse/548);
  const int kChassisAction = (kParaToParse%548)%273;

  // 获取射击速度
  double kSpeed = 0;
  switch (shoot_mode) {
    case 0:
      kSpeed = 0;
      break; // 不射击
    case 1: kSpeed = 15;
      break;
    case 2: kSpeed = 20;
      break;
    case 3: kSpeed = 24;
      break;
  }
  // 获取跑位点
  double x = 0;
  double y = 0;
  if (kChassisAction <= 152){
    // 跑大格
    x = (kChassisAction%17-8)*0.5;
    y = (kChassisAction/17-4)*0.5;

  }else{
    // 跑小格
    double chassis_action_small = kChassisAction - 152;
    x = (kChassisAction%11-5)*0.05;
    y = (kChassisAction/11-5)*0.05;
  }

  if (kChooseHome == 1){
    // 针对云台
      this->GetMyRobot1()->shoot(kSpeed);

    // 针对底盘
    const auto enemy = this->GetCloserEnemyRobot(this->GetMyRobot1());
    const auto safe_direction = this->GetMyRobot1()->getSafeDirection(enemy.GetPose());
    this->GetMyRobot1()->keepDirectionMove(x, y, safe_direction);
  }else{

    this->GetMyRobot2()->shoot(kSpeed);

    // 针对底盘
    const auto enemy = this->GetCloserEnemyRobot(this->GetMyRobot2());
    const auto angle = this->GetMyRobot2()->getSafeDirection(enemy.GetPose());
    this->GetMyRobot2()->keepDirectionMove(x, y, angle);
  }
}


RobotBehavior Blackboard::GetCommonMoveType(const std::shared_ptr<MyRobot> &my_robot) {
    // 敌方可直接攻击使用合适姿态跑
    return RobotBehavior::KEEP_DIRECTION_MOVE;
/*    if (this->GetDangerousOpp(my_robot).size() != 0) {
        return RobotBehavior::KEEP_DIRECTION_MOVE;
    } else { // 其他使用避障
        return RobotBehavior::avoidanceMove;
    }*/
}

std::vector<EnemyRobot> Blackboard::GetDangerousOpp(const std::shared_ptr<MyRobot> &p_my_robot) {

    std::vector<EnemyRobot> dangerEnemy;
    const auto myRobotPos = p_my_robot->getChassisMapPose().pose.position;
    roborts_common::Point2D myRobotPoint(myRobotPos.x, myRobotPos.y);

    const auto& enemy1 = this->GetEnemyRobot1();
    const auto& enemy2 = this->GetEnemyRobot2();
    double distanceToEnemy1 = this->GetDistanceWithObstacles(myRobotPoint, this->GetEnemyPolygon(enemy1));
    double distanceToEnemy2 = this->GetDistanceWithObstacles(myRobotPoint, this->GetEnemyPolygon(enemy2));
    if (distanceToEnemy1 < data::SITELENGTH + data::SITEHEIGHT) dangerEnemy.push_back(enemy1);
    if (distanceToEnemy2 < data::SITELENGTH + data::SITEHEIGHT) dangerEnemy.push_back(enemy2);

    return dangerEnemy;
}

const EnemyRobot & Blackboard::GetPrepareShootOpp(const std::shared_ptr<MyRobot> &my_robot) {

    double distance1 = my_robot->getEffectiveShootDistance() - BlackboardRaw::GetDistance(this->GetEnemyRobot1().GetPose(), my_robot->getChassisMapPose());
    double distance2 = my_robot->getEffectiveShootDistance() - BlackboardRaw::GetDistance(this->GetEnemyRobot2().GetPose(), my_robot->getChassisMapPose());

    // 无有效射击范围内的，朝向最近的敌方机器人
    if (distance1 < 0 && distance2 < 0) {
        if (distance1 > distance2) {
            return this->GetEnemyRobot1();
        } else {
            return this->GetEnemyRobot2();
        }

    } else if (distance1 > 0 && distance2 < 0) { // 枪口朝向在有效射击范围内的单个敌方
        return this->GetEnemyRobot1();

    } else if (distance1 < 0 && distance2 > 0) {
        return this->GetEnemyRobot2();

    } else {    // 若有两个，朝向后甲板漏出、侧夹板、离得近的

        auto armors = my_robot->getArmorsInEyes();
        // 如果没有观察到机器人,直接根据距离选人
        if (armors.armors.empty()) {
          if (distance1 > distance2) {
            return this->GetEnemyRobot1();
          } else {
            return this->GetEnemyRobot2();
          }
        }else if(armors.armors.front().armor_id == 3) {
            return (roborts_decision::RobotId)armors.armors.front().robot_id == this->GetEnemyRobot1().GetId() ? this->GetEnemyRobot1() : this->GetEnemyRobot2();

        } else {
            if (distance1 > distance2) {
                return this->GetEnemyRobot1();
            } else {
                return this->GetEnemyRobot2();
            }
        }
    }
}

// 1. 根据敌方机器人状态，判断敌方机器人是否处于强势期
// 2. 根据敌方机器人和我放机器人距离及敌方机器人状态判断是否应该防御或者进攻
// 3. 适当结合同伴跑位点，可以考虑吸引火力，优先分散，适度结合等策略
// 4. 计算跑位点


/// 计算我方机器人进攻跑位点
/// 1. 打敌方其中一个机器人，远离另一个
///   1.1 通过层次分析法计算
///   1.2 总共考虑了三要素，距离，对敌方机器人状态估计，敌方机器人与我方机器人之间的障碍物个数
/// 2. 尽可能大的获取敌方视野
///   2.1 通过我方机器人和敌方机器人的势场以及障碍物的势场进行判断，尝试采用RRT算法，获取跑位点
///   2.2 敌方机器人势场在半径４ｍ以内为负，，４ｍ以外为正，但是势场无法穿越障碍物，
///      2.2.1　我方势场为负，但是我方势场可以穿越障碍物，尽可能选择势场绝对值小的地方
///      2.2.2　考虑我方势场前提：我方机器人状态较好
///   2.3 首先确定跑位半径，然后获得相应的离散的跑位点
///   2.4 半径扩大原则：如果跑位点不满足，扩大跑位范围
///   2.5 就近原则：跑点就近，但是要考虑绕行障碍物所带来的距离损耗
///   2.6 易防守原则：尽量靠近障碍物
/// 3. 考虑我方机器人
///   3.1 不损害同伴，跑位点不能重合，并且适当远离同伴
///   3.2 完成自己的跑位点
///   3.3 在同伴发出请求的情况下，尽可能帮助同伴
roborts_common::Point2D Blackboard::GetAttackRunningPoint(const std::shared_ptr<MyRobot> &p_my_robot, const roborts_common::Point2D &another_partner_point){

    double tarX = 0, tarY = 0;
    /**
     * 0. 获取进攻的对象
    */
    auto enemyRobotToAttack = this->GetEnemyRobotToAttack(p_my_robot);

    /**
     *1. 获取势场
     */
    auto potentialField = this->UpdatePotentialFieldInAttack(p_my_robot, enemyRobotToAttack);

    /**
     * 2. 确定跑位点区间
     * 直接选择value最小的1000个点
     */
    std::vector<PointValue> runningInterval;
    runningInterval = this->CalculateSuitRunPoint(potentialField);

    /**
    * 3. 对跑位点进行评估，选取最优跑位点
    *  3.1 获取距离最近的点
    *  3.2 需要考虑对墙的距离，不能离墙太近
    *  3.3 考虑另一个机器人的跑位点，在我方机器人状态良好的情况下，适当远离
    */
    roborts_common::Point2D mostSuitablePoint =
        this->GetSpecificRunPoint(runningInterval, p_my_robot, another_partner_point);

    // this->WriteFile("out.txt", potentialField, mostSuitablePoint);

    return mostSuitablePoint;
}

std::vector<PointValue> Blackboard::CalculateSuitRunPoint(const std::vector<std::vector<PointValue>> &potential_field){

  std::vector<PointValue> runningInterval;
  for(const auto& line : potential_field){
    for(const auto & _point : line){
      if (_point.value <= 5){
        runningInterval.push_back(_point);
      }
    }
  }

  // 对数组排序,按照value值从小到大排序
  std::sort(runningInterval.begin(), runningInterval.end(),[](const PointValue &p1,const PointValue &p2)->bool{
      return p1.value <= p2.value;
  });

  runningInterval.resize(1000);
/*  for(const auto &iter : runningInterval){
    std::cout<<iter.value << std::endl;
  }*/

  return runningInterval;
}

/**
 * 更新势场
 * 1. 所有点全部更新，暂时利用赛场上的个点，势场均为正
 * 2. 对于敌方需要打击的机器人势场４ｍ以内势场逐渐减小，４ｍ以外逐渐增加，绝对值从１－１００
 * 3. 对于敌方不需要打击的机器人算法同上
 * 4. 对于我方另一个机器人，算法同上。
 * 5. 有障碍物阻挡，距离无限远，可以考虑取倒数，即势场影响为０
 */
std::vector<std::vector<PointValue>> Blackboard::UpdatePotentialFieldInAttack(const std::shared_ptr<MyRobot> &p_my_robot, const EnemyRobot & enemy_to_attack){


  using std::placeholders::_1;
  using std::placeholders::_2;
  //
  constexpr double interval = 0.04;
  auto potentialField = this->DiscretizeMap(interval);

  // 采用bing和mem_fn解决非静态问题
  const auto anotherEnemy = this->GetAnotherRobot(enemy_to_attack);

  this->UpdatePotentialField(potentialField, this->GetEnemyPolygon(enemy_to_attack),
                             std::bind(std::mem_fn(&Blackboard::CalculateEnemyToAttackPotentialField), this, _1, _2));

  this->UpdatePotentialField(potentialField, this->GetEnemyPolygon(anotherEnemy),
                             std::bind(std::mem_fn(&Blackboard::CalculateEnemyAnotherPotentialField), this, _1, _2));


  return potentialField;
}


/**
 * 计算敌方被进攻的机器人的势场，根据距离确定, 并结合障碍物位置
 * @param pose_point ,位置点
 * @param pose_polygon ，机器人矩形
 * @return 返回获得的value
 */
double Blackboard::CalculateEnemyToAttackPotentialField(const roborts_common::Point2D &pose_point, const roborts_common::Polygon2D &pose_polygon){

    const auto rate = 1;
    const auto valueMax = 10;
    double distance = this->GetDistanceWithObstacles(pose_point, pose_polygon);
    if (distance == data::MAX){
      return rate * valueMax;
    }
    else{
        return rate * (fabs(distance-2.5));
    }
}


/**
 * 计算敌方另一个机器人（相对于被进攻的机器人）机器人的势场，根据距离确定, 直接取倒数即可
 * @param pose_point ,位置点
 * @param pose_polygon ，机器人矩形
 * @return 返回获得的value
 */
double Blackboard::CalculateEnemyAnotherPotentialField(const roborts_common::Point2D &pose_point, const roborts_common::Polygon2D &pose_polygon){

  const auto rate = 0.1;
  const auto valueMax = 10;
  const auto valueMin = 0;
  double distance = this->GetDistanceWithObstacles(pose_point, pose_polygon);
  if (distance == data::MAX){
    return valueMin;
  }
  else{
    return rate *std::abs(valueMax - distance);
  }
}

/**
 * 获取最终的跑位点，结合到我方机器人的距离，与障碍物的距离和另一个机器人的距离
 */
roborts_common::Point2D Blackboard::GetSpecificRunPoint(const std::vector<PointValue> &running_interval, std::shared_ptr<MyRobot> p_my_robot, const roborts_common::Point2D &another_point){

    using namespace roborts_common;
    Point2D mostSuitablePoint;
    double disMin{data::MAX};
    for(auto _point : running_interval){
        bool disToObstacleSuitable = field_->GetDisToObstacleSuitable(_point.point);
        double disToMyRobot = this->PredictedDistance(_point.point, p_my_robot);
        double disToAnotherHomeRobot = PointDistance(_point.point, another_point);

        bool can_choose_this_point = this->p_blackboard_->CanCooperatePartner(_point.point, another_point, p_my_robot, this->p_blackboard_->GetAnotherRobot(p_my_robot));
        if (can_choose_this_point && disToObstacleSuitable && disToMyRobot < disMin && disToAnotherHomeRobot > 1 ){
            disMin = disToMyRobot;
            mostSuitablePoint = _point.point;
        }
    }
    return mostSuitablePoint;
}

double Blackboard::PredictedDistance(const roborts_common::Point2D &_point, const std::shared_ptr<MyRobot>& p_my_robot){

    double dis = this->GetDistanceWithObstacles(_point, this->GetHomePolygon(p_my_robot));
    if (dis == static_cast<double>(data::MAX)){
        dis = fabs(_point.X() - p_my_robot->getChassisMapPose().pose.position.x) +fabs(_point.Y() - p_my_robot->getChassisMapPose().pose.position.y);
    }
    return dis;
}

/*
 * 简化版本的层次分析法选人
 * 没有考虑一致性检验
 */
const EnemyRobot& Blackboard::GetEnemyRobotToAttack(const std::shared_ptr<MyRobot> &p_my_robot){

    double enemyRobot1State = this->GetEnemyRobot1().getState();
    double enemyRobot2State = this->GetEnemyRobot2().getState();
    double disToEnemy1{0};
    double disToEnemy2{0};
    double nearDistance{0};
    double numOfObstaclesTo1{0};
    double numOfObstaclesTo2{0};

    // 获取我方机器人和敌方机器人连线上的障碍物
    numOfObstaclesTo1 = field_->GetObstaclesBetweenTwoRobots(p_my_robot->getMyRobotPoint(), enemy_robot_1_.getEnemyPoint()).size();
    numOfObstaclesTo2 = field_->GetObstaclesBetweenTwoRobots(p_my_robot->getMyRobotPoint(), enemy_robot_2_.getEnemyPoint()).size();

    // 计算我方到敌方机器人距离
    disToEnemy1 = roborts_common::PointDistance(p_my_robot->getMyRobotPoint(), enemy_robot_1_.getEnemyPoint());
    disToEnemy2 = roborts_common::PointDistance(p_my_robot->getMyRobotPoint(), enemy_robot_2_.getEnemyPoint());

  // 采用层次分析法进行选择
    // 1. 建立成对比较矩阵, 第一层矩阵值人为定义，第二层根据实际因素确定
    // 第一层矩阵及权向量
    std::array<std::array<double, 3>, 3>pairComMatrixA{{{1,3,0.5}, {1.0/3, 1, 0.25}, {2, 4, 1}}};

    std::array<double, 3> pairComMatrixAW{};
    for (auto iterator = 0; iterator < 3; iterator++){
        for(auto it : pairComMatrixA[iterator]){
            pairComMatrixAW.at(iterator) += it;
        }
    }
    //对W矩阵归一化
    auto sum =(pairComMatrixAW.at(0) + pairComMatrixAW.at(1) + pairComMatrixAW.at(2));
    for (auto& iter:pairComMatrixAW){
        iter = iter/sum;
    }

    // 第二层矩阵及权向量
    std::array<std::array<double, 2>, 2>pairComMatrixBInDistance{{{1, disToEnemy1/disToEnemy2}, {disToEnemy2/disToEnemy1, 1}}};
    std::array<std::array<double, 2>, 2>pairComMatrixBInState{{{1,enemyRobot1State/enemyRobot2State}, {enemyRobot2State/enemyRobot1State, 1}}};
    std::array<std::array<double, 2>, 2>pairComMatrixBInReach{{{1,numOfObstaclesTo2 != 0 ? numOfObstaclesTo1/numOfObstaclesTo2 : 1}, {numOfObstaclesTo1 != 0 ? numOfObstaclesTo2/numOfObstaclesTo1 : 1, 1}}};

    // 分别对其进行归一化
    // lambda 表达式
    auto normalized = [](std::array<std::array<double, 2>, 2>pCBD)->std::array<double,2>{
        std::array<double, 2> pCB{0, 0};
        for (auto iterator = 0; iterator < 2; iterator++){
            for(auto it :pCBD[iterator]){
                pCB.at(iterator) += it;
            }
        }
        //对W矩阵归一化
        auto sum =(pCB.at(0) +pCB.at(1));
        for (auto& iter:pCB){
            iter = iter/sum;
        }
        return pCB;
    };

    std::array<double,2> pairComMatrixBInDistanceW = normalized(pairComMatrixBInDistance);
    std::array<double,2> pairComMatrixBInStateW = normalized(pairComMatrixBInState);
    std::array<double,2> pairComMatrixBInReachW = normalized(pairComMatrixBInReach);


    //进行权重计算
    double enemy1B = pairComMatrixBInDistanceW.at(0) * pairComMatrixAW.at(0)
                    +pairComMatrixBInStateW.at(0) * pairComMatrixAW.at(1)
                    +pairComMatrixBInReachW.at(0) * pairComMatrixAW.at(2);

    double enemy2B = pairComMatrixBInDistanceW.at(1) * pairComMatrixAW.at(0)
                     +pairComMatrixBInStateW.at(1) * pairComMatrixAW.at(1)
                     +pairComMatrixBInReachW.at(1) * pairComMatrixAW.at(2);

    if (enemy1B >= enemy2B){
        return enemy_robot_1_;
    }else{
        return enemy_robot_2_;
    }
}

//获取距离当前机器人更近的敌方机器人
const EnemyRobot& Blackboard::GetCloserEnemyRobot(const std::shared_ptr<MyRobot> &p_my_robot){

    double disToEnemy1 = BlackboardRaw::GetDistance(p_my_robot->getChassisMapPose(), this->GetEnemyRobot1().GetPose());
    double disToEnemy2 = BlackboardRaw::GetDistance(p_my_robot->getChassisMapPose(), this->GetEnemyRobot2().GetPose());
    int closerDistanceID = disToEnemy1 > disToEnemy2 ? 2: 1;

    if (closerDistanceID == 1){
        return this->GetEnemyRobot1();
    }else{
        return this->GetEnemyRobot2();
    }

}

/** 不考虑同伴
 * 根据两个敌方位置、自己位置、障碍物位置进行躲藏保障安全
 * 统一：成倍扩张寻找范围，直到寻找到一个满足安全系数的位置
 * 寻找算法：将地图离散（80mm单位离散），通过势场算法将获得各离散点的安全度，找到一个范围内满足安全性要求的最安全的位置。无满足的，进行范围扩散
 * 障碍物问题：给障碍物添加势场影响，加范围，防止离墙过近
 *
 * 考虑同伴（原则法）
 * 不损害同伴、完成自己的目的，尽力帮助同伴
 * 不损害同伴：1. 跑点与当前位置的连线不相交；2. 跑点不重合（距离小于两个车长）；
 * 完成自己的目的：1. 进攻、防御的目的不会因为同伴而改变（若重合，选其他点）；
 * 尽力帮助同伴：1. 响应同伴的信号（有余力才响应）；
 * @param p_my_robot
 * @param another_point
 * @return
 */
roborts_common::Point2D Blackboard::GetDefenseRunningPoint(const std::shared_ptr<MyRobot> &p_my_robot,
                                                           roborts_common::Point2D another_point) {

  double discreteUnit = 0.08;   // 离散化单位长度
  auto securityMatrix = DiscretizeMap(discreteUnit);

  // 更新势场
  this->UpdateDefenseAllPotentialField(securityMatrix);

  // 逐步扩散范围，找安全点
  PointValue safePoint = {{-1, -1}, data::MAX};   // 安全的点
  this->FindSafePoint(safePoint, p_my_robot, securityMatrix, another_point);

  return safePoint.point;
}

std::vector<std::vector<PointValue>> Blackboard::DiscretizeMap(double discrete_unit) const {

  const int LENGTHUNIT =  static_cast<const int>(data::SITELENGTH / discrete_unit);
  const int WIDTHUNIT = static_cast<const int>(data::SITEHEIGHT / discrete_unit);
  std::vector<std::vector<PointValue>> securityMatrix(WIDTHUNIT, std::vector<PointValue>(LENGTHUNIT));

  double lineWidth = -WIDTHUNIT / 2 * discrete_unit + discrete_unit / 2;
  for (int i = 0; i < securityMatrix.size(); i++) {
    double columnWidth = -(LENGTHUNIT - 1) / 2 * discrete_unit;
    for (int j = 0; j < securityMatrix.at(0).size(); j++) {
      securityMatrix.at(i).at(j) = {{columnWidth, lineWidth}, 0};
      columnWidth += discrete_unit;
    }
    lineWidth += discrete_unit;
  }
  return securityMatrix;
}

void Blackboard::FindSafePoint(PointValue &safe_point,
                               const std::shared_ptr<MyRobot> &p_my_robot,
                               const std::vector<std::vector<PointValue>> &security_matrix,
                               roborts_common::Point2D another_point) {

  constexpr static double diffusionRadius = 0.6;    // 扩散半径

  int rangeMultiples = 1; // 单次寻找扩散倍数
  while (true) {
    safe_point = {{-1, -1}, data::MAX};

    for (const auto &line : security_matrix) {
      for (const auto &point : line) {

        if (roborts_common::PointDistance(point.point, roborts_common::Point2D(p_my_robot->getChassisMapPose().pose.position.x, p_my_robot->getChassisMapPose().pose.position.y)) < rangeMultiples * diffusionRadius) {   // 在范围内
          if ((point.value < GetDefensePotentialFieldSafeValue())
              && this->CanCooperatePartner(point.point, another_point, p_my_robot, this->GetAnotherRobot(p_my_robot))) {  // 满足安全性要求和配合要求
            if (point.value < safe_point.value) {    // 找到更安全的点
              safe_point.point = point.point;
              safe_point.value = point.value;
            }
          }
        }
      }
    }
    if (abs(safe_point.value - data::MAX) > 1.0 / data::MAX) {    // 如果找到了
      break;
    }
    rangeMultiples++;

    if (rangeMultiples * diffusionRadius > data::SITELENGTH) {  // 未找到安全的地方（安全不合理）
      for (const auto &line : security_matrix) {   // 保险，寻找最安全的地方
        for (const auto &point : line) {
          if (point.value < safe_point.value) {
            safe_point.point = point.point;
            safe_point.value = point.value;
          }
        }
      }
      ROS_INFO("roborts_decision::Blackboard::FindSafePoint error: there is no safe defense point!\n"
               "The savest point value is %lf.\n", safe_point.value);

      // 通过写文件的方法将场上最安全的位置的value更新到最小安全标准
      UpdataValue("defense_safe_value", "defense_field.data", std::to_string(safe_point.value));
      break;
    }
  }
}

void Blackboard::UpdateDefenseAllPotentialField(std::vector<std::vector<PointValue>> &security_matrix) {

  // 敌方机器人位置势场
  for (const auto &opp : GetEnemyPolygon()) {
    this->UpdatePotentialField(security_matrix,
                               opp,
                               [this](const roborts_common::Point2D &point,
                                      const roborts_common::Polygon2D &polygon_2_d) {
                                 return GetPlacePotentialField(this->GetPointToEntityDistance(point, polygon_2_d));
                               });
  }

  // 敌方机器人炮台势场（通过墙将距离无限大可以描述墙阻碍炮台威胁）
  this->UpdatePotentialField(security_matrix,
                             GetEnemyPolygon(GetEnemyRobot1()),
                             [this](const roborts_common::Point2D &point, const roborts_common::Polygon2D &polygon2D) {
                               return GetFortPotentialField(GetDistanceWithObstacles(point, polygon2D),
                                                            abs(GetEnemyRobot1().getRelativeAngle(point)));
                             });
  this->UpdatePotentialField(security_matrix,
                             GetEnemyPolygon(GetEnemyRobot2()),
                             [this](const roborts_common::Point2D &point, const roborts_common::Polygon2D &polygon2D) {
                               return GetFortPotentialField(GetDistanceWithObstacles(point, polygon2D),
                                                            GetEnemyRobot2().getRelativeAngle(point));
                             });

  // 墙体、障碍物势场（防止离墙过近）
  for (const auto &wall : Field::TurnRectangulars(this->field_->getWalls())) {
    this->UpdatePotentialField(security_matrix,
                               wall,
                               [this](const roborts_common::Point2D &point,
                                      const roborts_common::Polygon2D &polygon2D) {
                                 return GetWallPotentialField(this->GetPointToEntityDistance(point, polygon2D));
                               });
  }
  for (const auto &obstacle : Field::TurnRectangulars(this->field_->getObstacles())) {
    this->UpdatePotentialField(security_matrix,
                               obstacle,
                               [this](const roborts_common::Point2D &point,
                                      const roborts_common::Polygon2D &polygon2D) {
                                 return GetWallPotentialField(this->GetPointToEntityDistance(point, polygon2D));
                               });
  }

  // 可以通过将势场在墙反射的方法描述敌方机器人将我方逼到死角的情况
  //this->UpdateAreaOfPotentialField(security_matrix, , -1);
}

void Blackboard::UpdatePotentialField(
    std::vector<std::vector<PointValue>> &potential_field,
    const roborts_common::Polygon2D &object,
    const std::function<double(const roborts_common::Point2D&, const roborts_common::Polygon2D&)>& fun) {

  for (auto& line : potential_field) {
    for (auto& point : line) {
      double value = fun(point.point, object);
      point.value += value;//fun(point.point, object);
    }
  }
}

double Blackboard::GetFortPotentialField(double distance, double direction) {

    // 不加权重应该保证势场影响大小为0-1
    constexpr static double PROPORTION = 1.2;

    const double k = -1.0 / 12;
    const double b = 1;
    const double MAX = 1;

    distance = abs(distance);
    direction = abs(direction);

    // 幂函数变化过快，不满足要求
    //double power = roborts_decision::BlackboardRaw::exponentialFunction(abs(distance * direction));
    // 使用线性函数
    double power = roborts_decision::BlackboardRaw::linearFunction(distance * direction, k, b, MAX);

    return power * PROPORTION;
}

double Blackboard::GetKeepDirectionSafeDistance() const {

    double angle = asin(data::DANGERSHOOTMIN / (data::SHOOTMAX - 1));
    double distance = (data::ROBOTSIZE / 2) / sin(data::PI / 4 - angle) * sin(angle);

    return distance;    // 约0.58
}

double Blackboard::GetPlacePotentialField(double distance) {

  // 不加权重应该保证势场影响大小为0-1
  constexpr static double PROPORTION = 1.8;

  constexpr static double POWERMAX = 1;   // 最大影响
  constexpr static double POWERSIZE = 1.6;    // 影响范围
  constexpr static double N = POWERMAX * data::ROBOTSIZE / 2; // 0.3
  constexpr static double POWERMIN = N / POWERSIZE; // 0.1875

  //double power = BlackboardRaw::exponentialFunction(distance, -1, exp(1), 0, 0.3);
  double power = BlackboardRaw::exponentialFunction(distance);

  return power * PROPORTION;
}

double Blackboard::GetWallPotentialField(double distance) {

  // 不加权重应该保证势场影响大小为0-1
  constexpr static double PROPORTION = 2.4;

  constexpr static double POWERMAX = 1;
  constexpr static double DANGERDISTANCE = 1; // 墙影响的扩散范围 3.5 (1.5=0)
  constexpr static double POWERSIZE = data::ROBOTSIZE / 2 * 1.414 + DANGERDISTANCE;
  constexpr static double N = POWERMAX * data::ROBOTSIZE / 2;
  constexpr static double POWERMIN = N / POWERSIZE;

  distance = abs(distance);// 3->0.10259,1.48->0.21

  //double power = BlackboardRaw::inverseRatioFunction(distance, POWERMAX, POWERMIN, N);
  // double power = BlackboardRaw::inverseRatioFunction(distance, POWERMAX);
  double power = BlackboardRaw::exponentialFunction(distance, -1, exp(1), 0, 0.49); // 0.45

  return power * PROPORTION;
}

// 获取我方所有机器人的矩形
std::vector<roborts_common::Polygon2D> Blackboard::GetHomePolygon(){
    std::vector<roborts_common::Polygon2D> homePolygon;
    homePolygon.push_back(this->GetHomePolygon(p_my_robot1_));
    homePolygon.push_back(this->GetHomePolygon(p_my_robot2_));
    return homePolygon;
}

// 获取我方某个机器人的矩形
roborts_common::Polygon2D Blackboard::GetHomePolygon(const std::shared_ptr<MyRobot>& p_my_robot) const{

    std::vector<roborts_common::Point2D> hPoint;
    double angleHome = tf::getYaw(p_my_robot->getChassisMapPose().pose.orientation);

    if (angleHome < 0){
        angleHome += data::PI;
    }

    if (angleHome > data::PI/2){
        angleHome = data::PI / 2 - angleHome;
    }

    angleHome -= angleHome-data::PI/4;

    double robotX = cos(angleHome) * data::ROBOTSIZE / 2 * sqrt(2);
    double robotY = sin(angleHome) * data::ROBOTSIZE;

    hPoint.emplace_back(p_my_robot->getChassisMapPose().pose.position.x + robotX,p_my_robot->getChassisMapPose().pose.position.y + robotY);
    hPoint.emplace_back(p_my_robot->getChassisMapPose().pose.position.x + robotY,p_my_robot->getChassisMapPose().pose.position.y - robotX);
    hPoint.emplace_back(p_my_robot->getChassisMapPose().pose.position.x - robotX,p_my_robot->getChassisMapPose().pose.position.y - robotY);
    hPoint.emplace_back(p_my_robot->getChassisMapPose().pose.position.x - robotY,p_my_robot->getChassisMapPose().pose.position.y + robotX);
    return roborts_common::Polygon2D(hPoint);
}

// 获取敌方所有机器人的矩形
std::vector<roborts_common::Polygon2D> Blackboard::GetEnemyPolygon() const {

    std::vector<roborts_common::Polygon2D> enemyPolygon;
    enemyPolygon.push_back(this->GetEnemyPolygon(enemy_robot_1_));
    enemyPolygon.push_back(this->GetEnemyPolygon(enemy_robot_2_));
    return enemyPolygon;
}

// 获取敌方某个机器人的矩形
roborts_common::Polygon2D Blackboard:: GetEnemyPolygon(const EnemyRobot &enemy_robot) const {

    std::vector<roborts_common::Point2D> hPoint;
    double angleEnemy = tf::getYaw(enemy_robot.GetPose().pose.orientation);

    if (angleEnemy < 0){
        angleEnemy += data::PI;
    }

    if (angleEnemy > data::PI/2){
        angleEnemy = data::PI / 2 - angleEnemy;
    }

    angleEnemy -= angleEnemy-data::PI/4;

    double robotX = cos(angleEnemy) * data::ROBOTSIZE / 2 * sqrt(2);
    double robotY = sin(angleEnemy) * data::ROBOTSIZE;

    hPoint.emplace_back(enemy_robot.GetPose().pose.position.x + robotX, enemy_robot.GetPose().pose.position.y + robotY);
    hPoint.emplace_back(enemy_robot.GetPose().pose.position.x + robotY, enemy_robot.GetPose().pose.position.y - robotX);
    hPoint.emplace_back(enemy_robot.GetPose().pose.position.x - robotX, enemy_robot.GetPose().pose.position.y - robotY);
    hPoint.emplace_back(enemy_robot.GetPose().pose.position.x - robotY, enemy_robot.GetPose().pose.position.y + robotX);
    return roborts_common::Polygon2D(hPoint);
}

double Blackboard::GetDistanceWithObstacles(const roborts_common::Point2D &pose,
                                            const roborts_common::Polygon2D &polygon_2_d) {

    double distance = this->GetPointToEntityDistance(pose, polygon_2_d);

    roborts_common::LineSegment2D tar_line(roborts_common::Point2D(pose.X(), pose.Y()),
        roborts_common::Point2D((polygon_2_d.MaxX() + polygon_2_d.MinX()) / 2, (polygon_2_d.MaxY() + polygon_2_d.MinY()) / 2));

    for (const auto &obstacleRectangular : Field::TurnRectangulars(this->field_->getObstacles())) {
        for(auto line : obstacleRectangular.Lines()) {
            if (roborts_common::CheckLineSegmentsIntersection2D(line, tar_line)) {
                distance = static_cast<double >(data::MAX);
            }
        }
    }
    return distance;
}

double Blackboard::GetDefensePotentialFieldSafeValue() const {
   return this->defense_safe_value_;
}

/**
 * 判断是否可以配合同伴（原则法）
 * 不损害同伴：1. 跑点与当前位置的连线不相交；2. 跑点不重合（距离小于两个车长）；
 * 尽力帮助同伴：1. 响应同伴的信号（有余力才响应）；
 * @param tar_point
 * @param partner_tar_point
 * @param partner
 * @return
 */
bool Blackboard::CanCooperatePartner(const roborts_common::Point2D &tar_point,
                                     const roborts_common::Point2D &partner_tar_point,
                                     const std::shared_ptr<MyRobot> &me,
                                     const std::shared_ptr<MyRobot> &partner) const {

    using namespace roborts_common;

    constexpr static double minDistance = roborts_decision::data::ROBOTSIZE * 2;   // 跑点不重合距离

    constexpr static bool RUNINTERSECT = true;

    // 不能选到队友很近的位置
    if (this->GetPointToEntityDistance(tar_point, this->GetHomePolygon(partner)) < data::ROBOTSIZE / 2 * 1.4) {
      return false;
    }

    // 无配合点, 满足配合
    if (partner_tar_point == roborts_common::Point2D(0, 0)) {
        return true;
    }

    // 跑点重合，不满足配合
    if (PointDistance(tar_point, partner_tar_point) < minDistance) {
        return false;
    }

    // 跑点连线相交，不满足配合
    const auto myPoint = me->getChassisMapPose().pose.position;
    const auto partnerPoint = partner->getChassisMapPose().pose.position;
    if (RUNINTERSECT &&
        CheckLineSegmentsIntersection2D(
            LineSegment2D(tar_point, Point2D(myPoint.x, myPoint.y)),
            LineSegment2D(partner_tar_point,
                          Point2D(partnerPoint.x, partnerPoint.y)))) {
      return false;
    }

    // 相应同伴信号

    return true;
}

void Blackboard::Update() {

  this->GetMyRobot1()->update();
  this->GetMyRobot2()->update();

  prediction_system_.UpdatePredictedRemainingProjectiles();
  prediction_system_.UpdatePredictedCurrentHeat();
  prediction_system_.UpdatePredictedHp();

  roborts_msgs::RobotStrategyState robot_strategy_state;
  robot_strategy_state.my_1_is_collide = this->IsCollide(this->GetMyRobot1());
  robot_strategy_state.my_2_is_collide = this->IsCollide(this->GetMyRobot2());

  this->GetEnemyRobot1().SetEnemyMoveParameter();
  this->GetEnemyRobot1().SetEnemyShootParameter();
  this->GetEnemyRobot2().SetEnemyMoveParameter();
  this->GetEnemyRobot2().SetEnemyShootParameter();

  this->GetMyRobot1()->SetHomeMoveParameter();
  this->GetMyRobot1()->SetHomeShootParameter();
  this->GetMyRobot2()->SetHomeMoveParameter();
  this->GetMyRobot2()->SetHomeShootParameter();

  is_collide_pub_.publish(robot_strategy_state);
}

double Blackboard::GetKeepDirectionEffectWithDistance(double distance, double angle) const {

    double height = distance * cos(angle);
    double base = distance * sin(angle) + data::ROBOTSIZE / 2;
    double tarAngle = atan2(base, height);

    return cos(tarAngle);
}

double Blackboard::GetShootSpeed(const std::shared_ptr<MyRobot>&p_my_robot, const roborts_common::Point2D &enemy_pose){

  roborts_common::Point2D myRobotPoint = p_my_robot->getMyRobotPoint();
  double dis = roborts_common::PointDistance(myRobotPoint, enemy_pose);
  const double loss = dis/8 * 0.05;
  double shootV = (13 / (this->GetKeepDirectionEffectWithDistance(dis))) / (1-loss);
  return shootV;
}

double Blackboard::GetPointToEntityDistance(const roborts_common::Point2D &point_2_d,
                                            const roborts_common::Polygon2D &polygon_2_d) const {

  if (polygon_2_d.MinY() < point_2_d.Y() && polygon_2_d.MaxY() > point_2_d.Y()) {
    if (polygon_2_d.MinX() < point_2_d.X() && polygon_2_d.MaxX() > point_2_d.X()) {
      return 0.0;
    }
  }

  return roborts_common::DistancePointToPolygon2D(point_2_d, polygon_2_d);
}

/**
 * 加载目标文件的某个值，若找不到，返回空字符串
 * 若目标是目录：在默认文件查找
 * 若目标是文件：在该文件查找
 * @param value_name
 * @param target_name
 * @return 返回value的字符串
 */
std::string Blackboard::LoadValue(const std::string& value_name, const std::string& target_name) {
  namespace fs = boost::filesystem;

  fs::path father("..");
  fs::path src_path = father/father/father/father/"roborts_decision";
  fs::path p = src_path/"data"/target_name;

  if(fs::is_directory(p)) {
    p /= "default.data";
  } else if(!fs::is_regular_file(p)) {  // 若非目录或常规文件
    return std::string("");
  }

  std::string value{FindValueInFile(value_name, p.string())};

  return value;
}

/**
 * 查找目标路径的所有文件，将数据更新到对应文件中，在单个文件中不允许重名
 * @param value_name
 * @param file_name
 * @return
 * @author lq
 */
bool Blackboard::UpdataValue(const std::string &value_name, const std::string &file_name, const std::string &value) {
  namespace fs = boost::filesystem;

  fs::path father("..");
  fs::path src_path = father/father/father/father/"roborts_decision";
  fs::path p = src_path/"data"/file_name;


  if(!fs::is_regular_file(p)) { // 如果不是常规文件，返回false
    return false;
  }

  if(!FindValueInFile(value_name, p.string()).empty()) { // 如果找到了该字段
    ChangeValueInFile(value_name, value, p.string());
  } else {
    AddValueInFile(value_name, value, p.string());
  }

  return true;
}

/**
 * 在指定文件中查找目标字段
 * @param value_name
 * @param file_name
 * @return 返回目标字段的value（string）,若没找到，返回空字符串
 */
std::string Blackboard::FindValueInFile(const std::string& value_name, const std::string& file_name) {

  std::ifstream file;
  file.open(file_name, std::ios::in);

  if(file.is_open()) {
    std::string name;
    std::string value;
    while(file.good()) {
      file >> name >> value;
      if (name == value_name) {
        ROS_INFO("Load value %s successful!", value_name.c_str());
        file.close();
        return value;
      }
    }
    file.close();
  }
  return std::string("");
}

/**
 * 在目标文件添加键值对（不检查是否字段存在）
 * @param value_name
 * @param value
 * @param file_path
 * @return
 */
bool Blackboard::AddValueInFile(const std::string &value_name, const std::string &value, const std::string &file_path) {

  std::ofstream file;
  file.open(file_path, std::ios::app);

  file << std::endl << value_name << ' ' << value << std::endl;

  return true;
}

/**
 * 更新目标文件目标字段的值（若有重复，全部更新；若无，也不检查）
 * @param value_name
 * @param value
 * @param file_path
 * @return
 */
bool Blackboard::ChangeValueInFile(const std::string &value_name,
                                   const std::string &value,
                                   const std::string &file_path) {

  std::vector<std::pair<std::string, std::string>> key_value;

  std::ifstream file_in;
  file_in.open(file_path, std::fstream::in);
  while(file_in.good()) {
    std::string a, b;
    file_in >> a >> b;
    key_value.emplace_back(a, b);
  }
  file_in.close();

  std::fstream file_out;
  file_out.open(file_path, std::ios::out);
  for(const auto& each : key_value) {
    if(each.first == value_name) {
      file_out << std::endl << each.first << ' ' << value << std::endl;
    } else {
      file_out << std::endl << each.first << ' ' << each.second << std::endl;
    }
  }
  return true;
}
void Blackboard::WriteFile(std::string file_name, std::vector<std::vector<PointValue>> &map, roborts_common::Point2D &most_suitable_point){
  std::fstream file;
  file.open("/home/wpy/v5/robotMaster/log/"+file_name, std::ios::out);
  if (file.is_open() == false){
    std::cout<<"$$$$$$$$$$$$$$$$$$$$$打开失败"<<std::endl;
  }
  for (auto line_reverse = map.rbegin(); line_reverse != map.rend(); line_reverse++) {
    for (auto & point_reverse : *line_reverse) {
      if (point_reverse.point.X() == most_suitable_point.X() && point_reverse.point.Y() == most_suitable_point.Y()){
        point_reverse.value = static_cast<double>(15);
      }
      if (fabs(point_reverse.point.X() - this->GetMyRobot1()->getChassisMapPose().pose.position.x) < 0.04 && fabs(point_reverse.point.Y() - this->GetMyRobot1()->getChassisMapPose().pose.position.y) < 0.04){
        point_reverse.value = static_cast<double>(18);
      }
      if (fabs(point_reverse.point.X() - this->GetMyRobot2()->getChassisMapPose().pose.position.x) < 0.04 && fabs(point_reverse.point.Y() - this->GetMyRobot2()->getChassisMapPose().pose.position.y) < 0.04){
        point_reverse.value = static_cast<double>(19);
      }
      if (fabs(point_reverse.point.X() - this->GetEnemyRobot1().GetPose().pose.position.x) < 0.04 && fabs(point_reverse.point.Y() - this->GetEnemyRobot1().GetPose().pose.position.y) < 0.04){
        point_reverse.value = static_cast<double>(25);
      }
      if (fabs(point_reverse.point.X() - this->GetEnemyRobot2().GetPose().pose.position.x) < 0.04 && fabs(point_reverse.point.Y() - this->GetEnemyRobot2().GetPose().pose.position.y) < 0.04){
        point_reverse.value = static_cast<double>(26);
      }
      file << point_reverse.value << ' ';
    }
    file<<std::endl;
  }
  file.close();
}

void Blackboard::UpdateAreaOfPotentialField(std::vector<std::vector<PointValue>> &security_matrix,
                                            const roborts_common::Polygon2D &area,
                                            double value) const {
  for(auto& line : security_matrix) {
    for(auto& point : line) {
      if(this->GetPointToEntityDistance(point.point, area) < 0.01) {
        point.value += value;
      }
    }
  }
}

bool Blackboard::IsCollide(const std::shared_ptr<MyRobot> p_my_robot) const {

  const roborts_common::Polygon2D kMyPolygon = this->GetHomePolygon(p_my_robot);

  // 撞墙
  for(const auto& wall : this->field_->TurnRectangulars(this->field_->getWalls())) {
    if(roborts_common::DistancePolygonToPolygon2D(kMyPolygon, wall) < 0.01)
      return true;
  }
  for(const auto& obstacle : this->field_->TurnRectangulars(this->field_->getObstacles())) {
    if(roborts_common::DistancePolygonToPolygon2D(kMyPolygon, obstacle) < 0.01)
      return true;
  }
  // 撞车
  const auto& partner = this->GetHomePolygon(this->GetAnotherRobot(p_my_robot));
  if(roborts_common::DistancePolygonToPolygon2D(kMyPolygon, partner) < 0.01) return true;

  for(const auto& enemy : this->GetEnemyPolygon()) {
    if(roborts_common::DistancePolygonToPolygon2D(kMyPolygon, enemy) < 0.01)
      return true;
  }

  return false;
}
