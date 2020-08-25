#include "field.h"

#include <algorithm>

#include "../../roborts_common/math/math.h"

using namespace std::string_literals;
using namespace roborts_decision;

std::shared_ptr<Field> Field::field_ = nullptr;

Field::Field(const ros::NodeHandle &nh)
    : nh_{nh},
      vec_buff_zone_status_{
          {
              {},
              {
                  .max_x_ = -3.270,
                  .max_y_ = 0.790,
                  .min_x_ = -3.810,
                  .min_y_ = 0.310,
              }
          },
          {
              {},
              {
                  .max_x_ = -1.870,
                  .max_y_ = -0.350,
                  .min_x_ = -2.410,
                  .min_y_ = -0.830,
              }
          },
          {
              {},
              {
                  .max_x_ = 0.270,
                  .max_y_ = 2.035,
                  .min_x_ = -0.270,
                  .min_y_ = 1.555,
              }
          },
          {
              {},
              {
                  .max_x_ = 3.810,
                  .max_y_ = -0.310,
                  .min_x_ = 3.270,
                  .min_y_ = -0.790,
              }
          },
          {
              {},
              {
                  .max_x_ = 2.410,
                  .max_y_ = 0.830,
                  .min_x_ = 1.870,
                  .min_y_ = 0.350,
              }
          },
          {
              {},
              {
                  .max_x_ = 0.270,
                  .max_y_ = -1.555,
                  .min_x_ = -0.270,
                  .min_y_ = -2.035,
              }
          }
      },
      obstacles_{
          {
              .max_x_ = -3.04,
              .max_y_ = 1.24,
              .min_x_ = -4.04,
              .min_y_ = 1.04,
          },
          {
              .max_x_ = -1.74,
              .max_y_ = 0.1,
              .min_x_ = -2.54,
              .min_y_ = -0.1,
          },
          {
              .max_x_ = -2.44,
              .max_y_ = -1.24,
              .min_x_ = -2.54,
              .min_y_ = -2.24,
          },
          {
              .max_x_ = 0.5,
              .max_y_ = 1.305,
              .min_x_ = -0.5,
              .min_y_ = 1.105,
          },
          {
              .max_x_ = 0.125,
              .max_y_ = 0.125,
              .min_x_ = -0.125,
              .min_y_ = -0.125,
          },
          {
              .max_x_ = 0.5,
              .max_y_ = -1.105,
              .min_x_ = -0.5,
              .min_y_ = -1.305,
          },
          {
              .max_x_ = 2.54,
              .max_y_ = 2.24,
              .min_x_ = 2.44,
              .min_y_ = 1.24,
          },
          {
              .max_x_ = 2.54,
              .max_y_ = 0.1,
              .min_x_ = 1.74,
              .min_y_ = -0.1,
          },
          {
              .max_x_ = 4.04,
              .max_y_ = -1.04,
              .min_x_ = 3.04,
              .min_y_ = -1.24,
          },
      },

    // 墙更新,逆时针更新,从左方第一个开始更新
      walls_{
          {
              .max_x_ = -4.04,
              .max_y_ = 2.445,
              .min_x_ = -4.245,
              .min_y_ = -2.445,
          },
          {
              .max_x_ = 4.245,
              .max_y_ = -2.24,
              .min_x_ = -4.245,
              .min_y_ = -2.445,
          },
          {
              .max_x_ = 4.245,
              .max_y_ = 2.445,
              .min_x_ = 4.04,
              .min_y_ = -2.445,
          },
          {
              .max_x_ = 4.245,
              .max_y_ = 2.445,
              .min_x_ = -4.245,
              .min_y_ = 2.24,
          },
      } {
  auto buff_zone_topic = "/buff_zone_status"s;
  nh_.param("buff_zone_status_topic", buff_zone_topic, buff_zone_topic);
  Field* p = const_cast<Field*>(this);
  buff_zone_sub_ = nh_.subscribe<roborts_msgs::BuffZoneStatus>(
      buff_zone_topic, 1, &Field::buffZoneStatusCallback, p);
}

const std::vector<std::pair<BuffZoneStatus, roborts_common::Zone>> &Field::GetBuffZoneStatus() const {
  return vec_buff_zone_status_;
}

const std::vector<roborts_common::Zone> &Field::getObstacles() const {
  return obstacles_;
}

const std::vector<roborts_common::Zone> &Field::getWalls() const {
  return walls_;
}

std::vector<roborts_common::Polygon2D> roborts_decision::Field::TurnRectangulars(const std::vector<roborts_common::Zone> &zones) {

  std::vector<roborts_common::Polygon2D> rectangulars;

  for (const auto &zone : zones) {

    std::vector<roborts_common::Point2D> points;

    if (zone.GetZoneX() == 0.001 && zone.GetZoneY() < 0.001) { // 判断是否是中间的障碍物(避免0判断错误）
      points.emplace_back(zone.min_x_, (zone.min_y_ + zone.max_y_) / 2);    // 左
      points.emplace_back((zone.max_x_ + zone.min_x_) / 2, zone.min_y_);    // 下
      points.emplace_back(zone.max_x_, (zone.min_y_ + zone.max_y_) / 2);    // 右
      points.emplace_back((zone.max_x_ + zone.min_x_) / 2, zone.max_y_);    // 上
    } else {
      points.emplace_back(zone.max_x_, zone.min_y_);
      points.emplace_back(zone.max_x_, zone.max_y_);
      points.emplace_back(zone.min_x_, zone.max_y_);
      points.emplace_back(zone.min_x_, zone.min_y_);
    }

    rectangulars.emplace_back(points);
  }

  return rectangulars;
}

int Field::IsZoneActive(BuffStatus buff_status) {
  for (int i = 0; i < 6; i++) {
    if (this->vec_buff_zone_status_.at(i).first.buff_status == buff_status
        and this->vec_buff_zone_status_.at(i).first.is_active) {
      return i + 1;
    }
  }
  return false;
}

std::vector<roborts_common::Polygon2D>
Field::GetObstaclesBetweenTwoRobots(const roborts_common::Point2D point1, const roborts_common::Point2D point2) {

  std::vector<roborts_common::Polygon2D> tarObstacles;

  roborts_common::LineSegment2D tarLine(point1, point2);

  for (const auto& obstacle : TurnRectangulars(this->getObstacles())) {
    for (const auto& line : obstacle.Lines()) {
      if (roborts_common::CheckLineSegmentsIntersection2D(tarLine, line)) {
        tarObstacles.push_back(obstacle);
        break;
      }
    }
  }

  return tarObstacles;
}

/**
 * 判断当前跑位点知否合适，如果距离大于sqrt(2)/2个车长，则合适，否则不合适
 * @param point
 * @return
 */
bool Field::GetDisToObstacleSuitable(const roborts_common::Point2D &point){
  std::vector<roborts_common::Zone> allObstacle;//包括墙在内所有的障碍物
  allObstacle.insert(allObstacle.begin(), this->getWalls().begin(), this->getWalls().end());
  allObstacle.insert(allObstacle.begin(), this->getObstacles().begin(), this->getObstacles().end());

  std::vector<roborts_common::Polygon2D> posePolygon = Field::TurnRectangulars(allObstacle);

  double disMin = 1e5;
  for (const auto &obstacleRectangular : posePolygon) {
    double dis = roborts_common::DistancePointToPolygon2D(point, obstacleRectangular);
    if (dis < disMin){
      disMin = dis;
    }
  }
  if (disMin > data::ROBOTSIZE/2*sqrt(2)){
    return true;
  }else{
    return false;
  }
}

void Field::buffZoneStatusCallback(const roborts_msgs::BuffZoneStatus::ConstPtr &msg) {
  std::array<uint8_t, 6> active_status = {
      msg->F1_zone_status,
      msg->F2_zone_status,
      msg->F3_zone_status,
      msg->F4_zone_status,
      msg->F5_zone_status,
      msg->F6_zone_status,
  };

  for (int i = 0; i < active_status.size(); ++i) {
    this->vec_buff_zone_status_.at(i).first.is_active = !active_status.at(i);
  }

  std::array<uint8_t, 6> debuff_status = {
      msg->F1_zone_buff_debuff_status,
      msg->F2_zone_buff_debuff_status,
      msg->F3_zone_buff_debuff_status,
      msg->F4_zone_buff_debuff_status,
      msg->F5_zone_buff_debuff_status,
      msg->F6_zone_buff_debuff_status,
  };

  for (int i = 0; i < debuff_status.size(); ++i) {
    this->vec_buff_zone_status_.at(i).first.buff_status = static_cast<BuffStatus>(debuff_status.at(i));
  }
}
std::shared_ptr<Field> Field::GetField() {
  if (field_ == nullptr){
    field_ = std::make_shared<Field>();
  }else{
    // do nothing
  }
  return field_;
}
