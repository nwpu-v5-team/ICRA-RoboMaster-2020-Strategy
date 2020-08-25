#pragma once

#include <vector>
#include <ros/ros.h>

#include "roborts_msgs/BuffZoneStatus.h"
#include "../../roborts_common/math/geometry.h"

#include "blackboard_common.h"

namespace roborts_decision {
/**
 * 与障碍物有关的信息
 */
class Field {
 public:
  Field(const ros::NodeHandle &nh = ros::NodeHandle("~"));
  Field(const Field&) = delete;
  Field(Field &&) = delete;

  static std::shared_ptr<Field> GetField();

  // 获得障碍物、墙的信息
  const std::vector<std::pair<BuffZoneStatus, roborts_common::Zone>> &GetBuffZoneStatus() const;
  const std::vector<roborts_common::Zone> &getObstacles() const;
  const std::vector<roborts_common::Zone> &getWalls() const;
  static std::vector<roborts_common::Polygon2D> TurnRectangulars(const std::vector<roborts_common::Zone> &zones) ;

  /**
   * Adjust the buff Zone is active
   * @param buff_status The Buff Zone status
   * @return When the buff Zone is active return the Zone id, and the other one
   * return false
   */
  int IsZoneActive(BuffStatus buff_status);

  //获取我方机器人与敌方机器人之间障碍物
  std::vector<roborts_common::Polygon2D>
  GetObstaclesBetweenTwoRobots(const roborts_common::Point2D point1, const roborts_common::Point2D point2);

  // 获取坐标点到障碍物的最短距离是否合适
  bool GetDisToObstacleSuitable(const roborts_common::Point2D &point);

 private:

  static std::shared_ptr<Field> field_;
  ros::NodeHandle nh_;
  ros::Subscriber buff_zone_sub_;

  std::vector<std::pair<BuffZoneStatus, roborts_common::Zone>> vec_buff_zone_status_;
  std::vector<roborts_common::Zone> obstacles_;
  std::vector<roborts_common::Zone> walls_;

  void buffZoneStatusCallback(const roborts_msgs::BuffZoneStatus::ConstPtr &msg);
};
}