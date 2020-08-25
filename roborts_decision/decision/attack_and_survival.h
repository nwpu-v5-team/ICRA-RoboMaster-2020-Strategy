//
// Created by noorall on 2020/4/12.
//

#ifndef SRC_ATTACK_AND_SURVIVAL_H
#define SRC_ATTACK_AND_SURVIVAL_H

#include "../behavior/robot_behaviors.h"

enum AASType {
  FIND_RESTORATION = 0,
  FIND_PROJECTILE
};
namespace roborts_decision {
class AttackAndSurvival {
 public:
  AttackAndSurvival(std::shared_ptr<MyRobot> p_my_robot1,
                    std::shared_ptr<MyRobot> p_my_robot2,
                    std::shared_ptr<Blackboard> p_blackboard,
                    std::shared_ptr<RobotBehaviors> p_robot1_behaviors,
                    std::shared_ptr<RobotBehaviors> p_robot2_behaviors) :
      p_my_robot1(std::move(p_my_robot1)),
      p_my_robot2(std::move(p_my_robot2)),
      p_blackboard_(std::move(p_blackboard)),
      p_robot1_behaviors(std::move(p_robot1_behaviors)),
      p_robot2_behaviors(std::move(p_robot2_behaviors)) {
  }

  void run(AASType aasType) {
    int supplyId = getSupplyId(aasType);
    if (!supplyId) {
      ROS_ERROR("The Zone is not active! id %d", supplyId);
    } else {
      taskAllocation(aasType);
      p_attack_robot_behavior->GetPursueAttackBehavior()->Run(p_blackboard_->GetEnemyRobot1().GetId(), 1.2);
      p_survival_robot_behavior->GetGoalBehavior()->Run(getGalPose(supplyId));
    }
  }

  geometry_msgs::PoseStamped getGalPose(int supplyId) {
    geometry_msgs::PoseStamped galPose;
    galPose.pose.orientation.z = 1;
    galPose.pose.position.x = p_blackboard_->GetBuffZoneStatus().at(supplyId).second.zone_x_;
    galPose.pose.position.y = p_blackboard_->GetBuffZoneStatus().at(supplyId).second.zone_y_;
    galPose.pose.position.z = 0;
    return galPose;
  }

  int getSupplyId(AASType aasType) {
    int supplyId = -1;
    for (auto buffZoneStatus : p_blackboard_->GetBuffZoneStatus()) {
      supplyId++;
      if (aasType == FIND_RESTORATION && buffZoneStatus.first.is_active_ &&
          buffZoneStatus.first.buff_status_ == RED_RESTORATION) {
        return supplyId;
      } else if (buffZoneStatus.first.is_active_ &&
          buffZoneStatus.first.buff_status_ == RED_PROJECTILE_SUPPLIER) {
        return supplyId;
      }
    }
    return 0;
  }

  static void CancelBehavior(const std::shared_ptr<MyRobot> &myRobot,
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

  void taskAllocation(AASType aasType) {
    if (aasType == FIND_RESTORATION) {
      if (p_my_robot1->GetHp() > p_my_robot2->GetHp()) {
        p_attack_robot_behavior = p_robot1_behaviors;
        p_survival_robot_behavior = p_robot2_behaviors;

        auto current_behavior = p_my_robot1->GetCurrentBehavior();
        if (current_behavior != CHASE) {
          CancelBehavior(p_my_robot1, p_attack_robot_behavior);
          p_my_robot1->SetCurrentBehavior(CHASE);
        }

        current_behavior = p_my_robot2->GetCurrentBehavior();
        if (current_behavior != GOAL) {
          CancelBehavior(p_my_robot2, p_survival_robot_behavior);
          p_my_robot1->SetCurrentBehavior(GOAL);
        }

      } else {
        p_attack_robot_behavior = p_robot2_behaviors;
        p_survival_robot_behavior = p_robot1_behaviors;

        auto current_behavior = p_my_robot2->GetCurrentBehavior();
        if (current_behavior != CHASE) {
          CancelBehavior(p_my_robot2, p_attack_robot_behavior);
          p_my_robot2->SetCurrentBehavior(CHASE);
        }

        current_behavior = p_my_robot1->GetCurrentBehavior();
        if (current_behavior != GOAL) {
          CancelBehavior(p_my_robot1, p_survival_robot_behavior);
          p_my_robot1->SetCurrentBehavior(GOAL);
        }
      }
    } else {
      if (p_my_robot1->GetRemainingProjectiles() > p_my_robot2->GetRemainingProjectiles()) {
        p_attack_robot_behavior = p_robot1_behaviors;
        p_survival_robot_behavior = p_robot2_behaviors;

        auto current_behavior = p_my_robot1->GetCurrentBehavior();
        if (current_behavior != CHASE) {
          CancelBehavior(p_my_robot1, p_attack_robot_behavior);
          p_my_robot1->SetCurrentBehavior(CHASE);
        }

        current_behavior = p_my_robot2->GetCurrentBehavior();
        if (current_behavior != GOAL) {
          CancelBehavior(p_my_robot2, p_survival_robot_behavior);
          p_my_robot2->SetCurrentBehavior(GOAL);
        }

      } else {
        p_attack_robot_behavior = p_robot2_behaviors;
        p_survival_robot_behavior = p_robot1_behaviors;

        auto current_behavior = p_my_robot2->GetCurrentBehavior();
        if (current_behavior != CHASE) {
          CancelBehavior(p_my_robot2, p_attack_robot_behavior);
          p_my_robot2->SetCurrentBehavior(CHASE);
        }

        current_behavior = p_my_robot1->GetCurrentBehavior();
        if (current_behavior != GOAL) {
          CancelBehavior(p_my_robot1, p_survival_robot_behavior);
          p_my_robot1->SetCurrentBehavior(GOAL);
        }
      }
    }
  }

  void cancel() {
    p_attack_robot_behavior->GetPursueAttackBehavior()->Cancel();
    p_survival_robot_behavior->GetSupplyBehavior()->Cancel();
  }

 private:
  std::shared_ptr<MyRobot> p_my_robot1;
  std::shared_ptr<MyRobot> p_my_robot2;
  std::shared_ptr<Blackboard> p_blackboard_;
  std::shared_ptr<RobotBehaviors> p_attack_robot_behavior;
  std::shared_ptr<RobotBehaviors> p_survival_robot_behavior;
  std::shared_ptr<RobotBehaviors> p_robot1_behaviors;
  std::shared_ptr<RobotBehaviors> p_robot2_behaviors;
};
}
#endif //SRC_ATTACK_AND_SURVIVAL_H
