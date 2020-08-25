//
// Created by song on 2020/4/13.
//

#ifndef ROBORTS_DECISION_TWOROBORTS_CHASE_H
#define ROBORTS_DECISION_TWOROBORTS_CHASE_H

#include <ros/ros.h>

#include "../blackboard/blackboard.h"
#include "../behavior/robot_behaviors.h"

namespace roborts_decision {
class TwoRobortsChase {
public:
  TwoRobortsChase(std::shared_ptr<MyRobot> p_my_robot1,
                  std::shared_ptr<MyRobot> p_my_robot2,
                  std::shared_ptr<Blackboard> p_blackboard,
                  std::shared_ptr<RobotBehaviors> p_robot1_behaviors,
                  std::shared_ptr<RobotBehaviors> p_robot2_behaviors)
      : p_my_robot1(std::move(p_my_robot1)),
        p_my_robot2(std::move(p_my_robot2)),
        p_blackboard_(std::move(p_blackboard)),
        p_robot1_behaviors(std::move(p_robot1_behaviors)),
        p_robot2_behaviors(std::move(p_robot2_behaviors)) {}

  void run(RobotId emeny_id) {

    auto current_behavior = p_my_robot1->GetCurrentBehavior();
    if (current_behavior != CHASE) {
      CancelBehavior(p_my_robot1, p_robot1_behaviors);
    }

    current_behavior = p_my_robot2->GetCurrentBehavior();
    if (current_behavior != CHASE) {
      CancelBehavior(p_my_robot2, p_robot2_behaviors);
    }

    p_my_robot1->SetCurrentBehavior(CHASE);
    p_my_robot2->SetCurrentBehavior(CHASE);
    p_robot1_behaviors->GetPursueAttackBehavior()->Run(emeny_id, 1.2);
    p_robot2_behaviors->GetPursueAttackBehavior()->Run(emeny_id, 1.2);
  }

  static void
  CancelBehavior(const std::shared_ptr<MyRobot> &myRobot,
                 const std::shared_ptr<RobotBehaviors> &robotBehaviors) {
    auto current_behavior = myRobot->GetCurrentBehavior();
    if (current_behavior == ESCAPE) {
      robotBehaviors->GetEscapeBehavior()->Cancel();
    } else if (current_behavior == SUPPLY) {
      robotBehaviors->GetSupplyBehavior()->Cancel();
    } else if (current_behavior == GOAL) {
      robotBehaviors->GetGoalBehavior()->Cancel();
    }
  }

  void cancel() {
    p_robot1_behaviors->GetPursueAttackBehavior()->Cancel();
    p_robot2_behaviors->GetPursueAttackBehavior()->Cancel();
  }

private:
  std::shared_ptr<MyRobot> p_my_robot1;
  std::shared_ptr<MyRobot> p_my_robot2;
  std::shared_ptr<Blackboard> p_blackboard_;
  std::shared_ptr<RobotBehaviors> p_robot1_behaviors;
  std::shared_ptr<RobotBehaviors> p_robot2_behaviors;
};
} // namespace roborts_decision

#endif // ROBORTS_DECISION_TWOROBORTS_CHASE_H
