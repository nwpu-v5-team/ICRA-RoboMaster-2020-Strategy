//
// Created by wpy on 2020/8/15.
//

#ifndef ROBOTRTS_WS_SRC_ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_STRATEGY_STRATEGY_EXECUTE_H_
#define ROBOTRTS_WS_SRC_ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_STRATEGY_STRATEGY_EXECUTE_H_

#include "abstract_common_strategy.h"
#include "./move_strategys/captureBullets_move_strategy.h"
#include "./move_strategys/captureEnemyBlood_move_strategy.h"
#include "./move_strategys/captureHomeBlood_move_strategy.h"
#include "./move_strategys/keepDirection_attack_move_strategy.h"
#include "./move_strategys/keepDirection_defense_move_strategy.h"
#include "./move_strategys/stop_move_strategy.h"
#include "./shoot_strategys/stop_shoot_strategy.h"
#include "./shoot_strategys/toBetterOpp_shoot_strategy.h"
#include "./shoot_strategys/toWorseOpp_shoot_strategy.h"
namespace roborts_decision {


/**
 * The class executes strategy selected by dqn_network.
 * It implements the single instance mode, the instance can be obtained by StrategyExecute::GetStrategyExecute()
 */
class StrategyExecute {
 public:
  /// \brief the constructor
  explicit StrategyExecute(const ros::NodeHandle &nh = ros::NodeHandle("~"));
  ~StrategyExecute() = default;

  static std::shared_ptr<StrategyExecute> GetStrategyExecute();

  /// \brief Convert the strategy id to specific strategy
  void ChooseStrategyToExceed(const std::shared_ptr<MyRobot> &p_my_robot, int data);

 private:
  std::shared_ptr<Blackboard> p_blackboard_;

  ros::NodeHandle nh_;
  ros::Subscriber next_decision_sub_;

  static std::shared_ptr<StrategyExecute> p_strategy_execute_;

  /// The callback will be called on every policy choice message dqn_network sends.
  /// \param msg the specific type of msg
  void NextDecisionCallBack(const std_msgs::Int32::ConstPtr &msg);


};
}
#endif //ROBOTRTS_WS_SRC_ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_STRATEGY_STRATEGY_EXECUTE_H_
