//
// Created by wpy on 2020/8/23.
//

#ifndef ROBOTRTS_WS_SRC_ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_BEHAVIOR_TEST_NODE_H_
#define ROBOTRTS_WS_SRC_ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_BEHAVIOR_TEST_NODE_H_

#include <ros/ros.h>
#include "blackboard/blackboard.h"
#include "blackboard/my_robot.h"
#include "./strategy/strategy_execute.h"
#include"../roborts_decision/behavior_tree/behavior_tree.h"

namespace roborts_decision {
class BehaviorTestNode {
 public:
  BehaviorTestNode();
  ~BehaviorTestNode() = default;
  BehaviorTestNode(const BehaviorTestNode&) = delete;
  BehaviorTestNode(BehaviorTestNode&&) = delete;

  static std::shared_ptr<BehaviorTestNode> GetBehaviorTestNode();

  std::shared_ptr<BehaviorNode> CreatRootNode();

 private:
  static std::shared_ptr<BehaviorTestNode> ptr_behavior_test_node;
};
}
#endif //ROBOTRTS_WS_SRC_ICRA_FIREFLY_ROBORTS_ROBORTS_DECISION_BEHAVIOR_TEST_NODE_H_
