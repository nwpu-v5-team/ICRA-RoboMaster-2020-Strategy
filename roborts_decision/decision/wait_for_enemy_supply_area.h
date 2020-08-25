//
// Created by heihei on 2020/4/13.
//

#ifndef ICRA_FIREFLY_ROBORTS_WAIT_FOR_ENEMY_SUPPLY_AREA_H_
#define ICRA_FIREFLY_ROBORTS_WAIT_FOR_ENEMY_SUPPLY_AREA_H_

#include "../blackboard/blackboard.h"
#include "../blackboard/my_robot.h"
#include "../behavior/robot_behaviors.h"

namespace roborts_decision {

class WaitForEnemySupplyArea {
public:
  WaitForEnemySupplyArea(std::shared_ptr<MyRobot> p_my_robot1,
                         std::shared_ptr<MyRobot> p_my_robot2,
                         std::shared_ptr<Blackboard> p_blackboard,
                         std::shared_ptr<RobotBehaviors> p_robot1_behaviors,
                         std::shared_ptr<RobotBehaviors> p_robot2_behaviors)
      : p_my_robot1(std::move(p_my_robot1)),
        p_my_robot2(std::move(p_my_robot2)),
        p_blackboard_(std::move(p_blackboard)),
        p_robot1_behaviors(std::move(p_robot1_behaviors)),
        p_robot2_behaviors(std::move(p_robot2_behaviors)) {
    if (this->p_my_robot1->GetRobotType() > 10) {
      this->buff_status_projection_supplier = RED_PROJECTILE_SUPPLIER;
      this->buff_status_restoration = RED_RESTORATION;
    } else {
      this->buff_status_projection_supplier = BLUE_PROJECTILE_SUPPLIER;
      this->buff_status_restoration = BLUE_RESTORATION;
    }
  }

  bool Run(EnemyBuffZone enemy_zone) {
    int zone_id = 0;
    if (enemy_zone == PROJECTILE_SUPPLIER) {
      zone_id = this->p_blackboard_->isZoneActive(
          this->buff_status_projection_supplier);
      if (!zone_id) {
        ROS_ERROR("The Zone is not active!");
        return false;
      } else {

        auto current_behavior = p_my_robot1->GetCurrentBehavior();
        if (current_behavior != GOAL) {
          CancelBehavior(p_my_robot1, p_robot1_behaviors);
        }

        current_behavior = p_my_robot2->GetCurrentBehavior();
        if (current_behavior != GOAL) {
          CancelBehavior(p_my_robot2, p_robot2_behaviors);
        }

        p_my_robot1->SetCurrentBehavior(GOAL);
        p_my_robot2->SetCurrentBehavior(GOAL);
        this->p_robot1_behaviors->GetGoalBehavior()->Run(
            getGoalPose(zone_id).first);
        this->p_robot2_behaviors->GetGoalBehavior()->Run(
            getGoalPose(zone_id).second);
      }
    } else if (enemy_zone == RESTORATION) {
      zone_id =
          this->p_blackboard_->isZoneActive(this->buff_status_restoration);
      if (!zone_id) {
        ROS_ERROR("The Zone is not active!");
        return false;
      } else {

        auto current_behavior = p_my_robot1->GetCurrentBehavior();
        if (current_behavior != GOAL) {
          CancelBehavior(p_my_robot1, p_robot1_behaviors);
        }

        current_behavior = p_my_robot2->GetCurrentBehavior();
        if (current_behavior != GOAL) {
          CancelBehavior(p_my_robot2, p_robot2_behaviors);
        }

        p_my_robot1->SetCurrentBehavior(GOAL);
        p_my_robot2->SetCurrentBehavior(GOAL);
        this->p_robot1_behaviors->GetGoalBehavior()->Run(
            getGoalPose(zone_id).first);
        this->p_robot2_behaviors->GetGoalBehavior()->Run(
            getGoalPose(zone_id).second);
      }
    }
    return true;
  }

  void Cancel() {
    this->p_robot1_behaviors->GetGoalBehavior()->Cancel();
    this->p_robot2_behaviors->GetGoalBehavior()->Cancel();
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

private:
  std::shared_ptr<MyRobot> p_my_robot1;
  std::shared_ptr<MyRobot> p_my_robot2;
  std::shared_ptr<Blackboard> p_blackboard_;

  std::shared_ptr<RobotBehaviors> p_robot1_behaviors;
  std::shared_ptr<RobotBehaviors> p_robot2_behaviors;

  BuffStatus buff_status_restoration;
  BuffStatus buff_status_projection_supplier;

  std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>
  getGoalPose(int zone_id) {
    geometry_msgs::PoseStamped zone_pose_1;
    geometry_msgs::PoseStamped zone_pose_2;

    switch (zone_id) {
    case 1: {
      zone_pose_1.pose.position.x =
          this->p_blackboard_->GetBuffZoneStatus().at(1).second.max_x_ + 1;
      zone_pose_1.pose.position.y =
          this->p_blackboard_->GetBuffZoneStatus().at(1).second.zone_y_;
      zone_pose_1.pose.orientation.z = 1;

      zone_pose_2.pose.position.x =
          this->p_blackboard_->GetBuffZoneStatus().at(1).second.zone_x_;
      zone_pose_2.pose.position.y =
          this->p_blackboard_->GetBuffZoneStatus().at(1).second.min_y_ - 1;
      zone_pose_2.pose.orientation.z = 0.707;
      zone_pose_2.pose.orientation.w = 0.707;
      break;
    }

    case 2: {
      zone_pose_1.pose.position.x =
          this->p_blackboard_->GetBuffZoneStatus().at(2).second.min_x_ - 1;
      zone_pose_1.pose.position.y =
          this->p_blackboard_->GetBuffZoneStatus().at(2).second.zone_y_;
      zone_pose_1.pose.orientation.w = 1;

      zone_pose_2.pose.position.x =
          this->p_blackboard_->GetBuffZoneStatus().at(2).second.max_x_ + 1;
      zone_pose_2.pose.position.y =
          this->p_blackboard_->GetBuffZoneStatus().at(2).second.zone_y_;
      zone_pose_2.pose.orientation.z = 1;
      break;
    }

    case 6: {
      zone_pose_1.pose.position.x =
          this->p_blackboard_->GetBuffZoneStatus().at(6).second.min_x_ - 1;
      zone_pose_1.pose.position.y =
          this->p_blackboard_->GetBuffZoneStatus().at(6).second.zone_y_;
      zone_pose_1.pose.orientation.w = 1;

      zone_pose_2.pose.position.x =
          this->p_blackboard_->GetBuffZoneStatus().at(6).second.max_x_ + 1;
      zone_pose_2.pose.position.y =
          this->p_blackboard_->GetBuffZoneStatus().at(6).second.zone_y_;
      zone_pose_2.pose.orientation.z = 1;
      break;
    }

    case 3: {
      zone_pose_1.pose.position.x =
          this->p_blackboard_->GetBuffZoneStatus().at(3).second.min_x_ - 1;
      zone_pose_1.pose.position.y =
          this->p_blackboard_->GetBuffZoneStatus().at(3).second.zone_y_;
      zone_pose_1.pose.orientation.w = 1;

      zone_pose_2.pose.position.x =
          this->p_blackboard_->GetBuffZoneStatus().at(3).second.max_x_ + 1;
      zone_pose_2.pose.position.y =
          this->p_blackboard_->GetBuffZoneStatus().at(3).second.zone_y_;
      zone_pose_2.pose.orientation.z = 1;
      break;
    }

    case 5: {
      zone_pose_1.pose.position.x =
          this->p_blackboard_->GetBuffZoneStatus().at(5).second.min_x_ - 1;
      zone_pose_1.pose.position.y =
          this->p_blackboard_->GetBuffZoneStatus().at(5).second.zone_y_;
      zone_pose_1.pose.orientation.w = 1;

      zone_pose_2.pose.position.x =
          this->p_blackboard_->GetBuffZoneStatus().at(5).second.max_x_ + 1;
      zone_pose_2.pose.position.y =
          this->p_blackboard_->GetBuffZoneStatus().at(5).second.zone_y_;
      zone_pose_2.pose.orientation.z = 1;
      break;
    }

    case 4: {
      zone_pose_1.pose.position.x =
          this->p_blackboard_->GetBuffZoneStatus().at(4).second.min_x_ - 1;
      zone_pose_1.pose.position.y =
          this->p_blackboard_->GetBuffZoneStatus().at(4).second.zone_y_;
      zone_pose_1.pose.orientation.w = 1;

      zone_pose_2.pose.position.x =
          this->p_blackboard_->GetBuffZoneStatus().at(4).second.zone_x_;
      zone_pose_2.pose.position.y =
          this->p_blackboard_->GetBuffZoneStatus().at(4).second.max_y_ + 1;
      zone_pose_2.pose.orientation.z = -0.707;
      zone_pose_2.pose.orientation.w = 0.707;
      break;
    }

    default: {
      ROS_ERROR("Error Zone id !");
      break;
    }
    }

    //处理位置

    return std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>(
        zone_pose_1, zone_pose_2);
  }
};
} // namespace roborts_decision

#endif // ICRA_FIREFLY_ROBORTS_WAIT_FOR_ENEMY_SUPPLY_AREA_H_
