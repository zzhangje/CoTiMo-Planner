#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <eigen3/Eigen/Eigen>

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

class Segment {
 private:
  Eigen::Vector2d pts1, pts2;

 public:
  Segment(Eigen::Vector2d pts1, Eigen::Vector2d pts2) {
    this->pts1 = pts1;
    this->pts2 = pts2;
  }
  Segment(double x1, double y1, double x2, double y2) {
    this->pts1 = Eigen::Vector2d(x1, y1);
    this->pts2 = Eigen::Vector2d(x2, y2);
  }
  bool intersect(Segment seg) {
    Eigen::Vector2d p1 = this->pts1;
    Eigen::Vector2d p2 = this->pts2;
    Eigen::Vector2d q1 = seg.pts1;
    Eigen::Vector2d q2 = seg.pts2;
    double d = (p1.x() - p2.x()) * (q1.y() - q2.y()) -
               (p1.y() - p2.y()) * (q1.x() - q2.x());
    if (d == 0) return false;
    double t = ((p1.x() - q1.x()) * (q1.y() - q2.y()) -
                (p1.y() - q1.y()) * (q1.x() - q2.x())) /
               d;
    double u = -((p1.x() - p2.x()) * (p1.y() - q1.y()) -
                 (p1.y() - p2.y()) * (p1.x() - q1.x())) /
               d;
    return t >= 0 && t <= 1 && u >= 0 && u <= 1;
  }
  double getPts1X() { return this->pts1.x(); }
  double getPts1Y() { return this->pts1.y(); }
  double getPts2X() { return this->pts2.x(); }
  double getPts2Y() { return this->pts2.y(); }
};

class Object {
 private:
  std::vector<Segment> segments;
  double x, y, theta;

 public:
  Object(ObjectType type, double t = 0, double r = 0) {
    switch (type) {
      case ObjectType::ENV: {
        this->segments = std::vector<Segment>{
            Segment(-ROBOT_2_L1_FRONT, ROBOT_HEIGHT,
                    -ROBOT_2_L1_FRONT - ROBOT_WIDTH,
                    ROBOT_HEIGHT),
            Segment(0, 0, 0, L1_FRONT_HEIGHT),
            Segment(0, L1_FRONT_HEIGHT, L1_FRONT_2_L1_BACK,
                    L1_BACK_HEIGHT),
            Segment(L2_UL_X, L2_UL_Y, L2_UR_X,
                    L2_UR_Y),
            Segment(L2_LL_X, L2_LL_Y, L2_LR_X,
                    L2_LR_Y),
            Segment(L3_UL_X, L3_UL_Y, L3_UR_X,
                    L3_UR_Y),
            Segment(L3_LL_X, L3_LL_Y, L3_LR_X,
                    L3_LR_Y),
            Segment(L4_UL_X, L4_UL_Y, L4_LL_X,
                    L4_LL_Y),
            Segment(BRANCH_UL_X, BRANCH_UL_Y, BRANCH_LL_X,
                    BRANCH_LL_Y),
            Segment(BRANCH_UL_X, BRANCH_UL_Y, L4_LL_X,
                    L4_LL_Y),
        };
        this->x = 0;
        this->y = 0;
        this->theta = 0;
        return;
      }
      case ObjectType::ARM: {
        this->segments = std::vector<Segment>{
            Segment(-.2, .05, .2, .05),
            Segment(.2, .05, .2, -.05),
            Segment(.2, -.05, -.2, -.05),
            Segment(-.2, -.05, -.2, .05),
        };
        break;
      }
      case ObjectType::ARM_EXP: {
        this->segments = std::vector<Segment>{
            Segment(-.25, .1, .25, .1),
            Segment(.25, .1, .25, -.1),
            Segment(.25, -.1, -.25, -.1),
            Segment(-.25, -.1, -.25, .1),
        };
        break;
      }
      default: {
        this->segments = std::vector<Segment>();
        break;
      }
    }
    this->x = 0;
    this->y = 0;
    this->theta = 0;
    this->armTransform(t, r);
  }
  Object(std::vector<Segment> segments, double x = 0, double y = 0, double theta = 0) {
    this->segments = segments;
    this->x = 0;
    this->y = 0;
    this->theta = 0;
    this->transform(x, y, theta);
  }
  Object(Object& obj) {
    this->segments = obj.segments;
    this->x = obj.x;
    this->y = obj.y;
    this->theta = obj.theta;
  }
  bool intersect(Object segs) {
    for (Segment s1 : this->segments) {
      for (Segment s2 : segs.segments) {
        if (s1.intersect(s2)) return true;
      }
    }
    return false;
  }

  void transform(double dx, double dy, double dtheta) {
    std::vector<Segment> new_segments;
    double cos_theta = cos(dtheta);
    double sin_theta = sin(dtheta);
    for (Segment s : this->segments) {
      new_segments.push_back(
          Segment(Eigen::Vector2d(this->x + cos_theta * s.getPts1X() -
                                      sin_theta * s.getPts1Y() + dx,
                                  this->y + sin_theta * s.getPts1X() +
                                      cos_theta * s.getPts1Y() + dy),
                  Eigen::Vector2d(this->x + cos_theta * s.getPts2X() -
                                      sin_theta * s.getPts2Y() + dx,
                                  this->y + sin_theta * s.getPts2X() +
                                      cos_theta * s.getPts2Y() + dy)));
    }
    this->segments = new_segments;
    this->x += dx;
    this->y += dy;
    this->theta += dtheta;
  }

  void armTransform(double dt, double dr) {
    return this->transform(
        dt * ELEVATOR_COS_ANGLE - ELEVATOR_2_L1_FRONT,
        dt * ELEVATOR_SIN_ANGLE + ELEVATOR_2_GROUND, dr);
  }

  std::vector<Segment> getSegments() { return this->segments; }
  double getX() { return this->x; }
  double getY() { return this->y; }
  double getTheta() { return this->theta; }
};

#endif  // OBJECT_HPP