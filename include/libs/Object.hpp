#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <Eigen/Eigen>
#include <cmath>

#include "Polygon.hpp"
#include "config.h"
#include "log.hpp"

using namespace config::alphabot;
using namespace config::env;

enum ObjectType {
  ENV,
  ARM,
  ARM_ALGAE,
  ARM_CORAL,
  ARM_ALGAE_CORAL,
  ARM_EXP,
  ARM_EXP_ALGAE,
  ARM_EXP_CORAL,
  ARM_EXP_ALGAE_CORAL
};

class Object {
 private:
  std::vector<Geometry::Polygon> polygons;

 public:
  Object(std::vector<Geometry::Polygon> polygons) : polygons(polygons) {}
  Object(ObjectType type) {
    switch (type) {
      case ObjectType::ENV: {
        this->polygons = std::vector<Geometry::Polygon>{
            Geometry::Polygon(
                {Eigen::Vector2d(-ROBOT_2_L1_FRONT, ROBOT_HEIGHT),
                 Eigen::Vector2d(-ROBOT_2_L1_FRONT - ROBOT_WIDTH, ROBOT_HEIGHT),
                 Eigen::Vector2d(-ROBOT_2_L1_FRONT - ROBOT_WIDTH, 0),
                 Eigen::Vector2d(-ROBOT_2_L1_FRONT, 0)}),
            Geometry::Polygon({Eigen::Vector2d(0, 0), Eigen::Vector2d(0, L1_FRONT_HEIGHT),
                               Eigen::Vector2d(L1_FRONT_2_L1_BACK, L1_BACK_HEIGHT),
                               Eigen::Vector2d(L1_FRONT_2_L1_BACK, 0)},
                              false),
            Geometry::Polygon({Eigen::Vector2d(L2_UL_X, L2_UL_Y),
                               Eigen::Vector2d(L2_UR_X, L2_UR_Y),
                               Eigen::Vector2d(L2_LR_X, L2_LR_Y),
                               Eigen::Vector2d(L2_LL_X, L2_LL_Y)},
                              false),
            Geometry::Polygon({Eigen::Vector2d(L3_UL_X, L3_UL_Y),
                               Eigen::Vector2d(L3_UR_X, L3_UR_Y),
                               Eigen::Vector2d(L3_LR_X, L3_LR_Y),
                               Eigen::Vector2d(L3_LL_X, L3_LL_Y)},
                              false),
            Geometry::Polygon({Eigen::Vector2d(L4_UL_X, L4_UL_Y),
                               Eigen::Vector2d(L4_LL_X, L4_LL_Y),
                               Eigen::Vector2d(L4_LR_X, L4_LR_Y),
                               Eigen::Vector2d(L4_UR_X, L4_UR_Y)}),
            Geometry::Polygon({Eigen::Vector2d(L4_LL_X, L4_LL_Y),
                               Eigen::Vector2d(L4_LR_X, L4_LR_Y),
                               Eigen::Vector2d(BRANCH_UR_X, BRANCH_UR_Y),
                               Eigen::Vector2d(BRANCH_UL_X, BRANCH_UL_Y)}),
        };
        break;
      }
      case ObjectType::ARM: {
        this->polygons = std::vector<Geometry::Polygon>{
            Geometry::Polygon({Eigen::Vector2d(-.2, .05), Eigen::Vector2d(.2, .05),
                               Eigen::Vector2d(.2, -.05), Eigen::Vector2d(-.2, -.05)},
                              false)};
        break;
      }
      case ObjectType::ARM_EXP: {
        this->polygons = std::vector<Geometry::Polygon>{
            Geometry::Polygon({Eigen::Vector2d(-.25, .1), Eigen::Vector2d(.25, .1),
                               Eigen::Vector2d(.25, -.1), Eigen::Vector2d(-.25, -.1)},
                              false)};
        break;
      }
      default: {
        this->polygons = std::vector<Geometry::Polygon>{};
        break;
      }
    }
  }

  bool intersect(Object object) {
    for (Geometry::Polygon p1 : this->polygons) {
      for (Geometry::Polygon p2 : object.polygons) {
        if (p1.isPolygonIntersect(p2)) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * @param dt in meter
   * @param dr in degree
   */
  Object armTransform(double dt, double dr) {
    std::vector<Geometry::Polygon> new_polygons;
    for (Geometry::Polygon p : this->polygons) {
      new_polygons.push_back(p.transform(
          Eigen::Vector2d(-ELEVATOR_2_L1_FRONT + dt * ELEVATOR_COS_ANGLE,
                          ELEVATOR_2_GROUND + dt * ELEVATOR_SIN_ANGLE),
          dr / 180 * M_PI));
    }
    return Object(new_polygons);
  }

  std::vector<Geometry::Polygon> getPolygons() { return this->polygons; }
  void getPolygons(std::vector<Geometry::Polygon>& polygons) {
    polygons = this->polygons;
  }

  double distanceToObject(Object object) {
    double result = INFINITY;
    for (Geometry::Polygon p1 : this->polygons) {
      for (Geometry::Polygon p2 : object.polygons) {
        result = std::min(result, p1.distanceToPolygon(p2));
      }
    }
    return result;
  }
};

#endif  // OBJECT_HPP