#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <eigen3/Eigen/Eigen>

#include "config.h"

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
  // env
  Object() {
    this->segments = std::vector<Segment>{
        Segment(-config::ROBOT_2_L1_FRONT, config::ROBOT_HEIGHT,
                -config::ROBOT_2_L1_FRONT - config::ROBOT_WIDTH,
                config::ROBOT_HEIGHT),
        Segment(0, 0, 0, config::L1_FRONT_HEIGHT),
        Segment(0, config::L1_FRONT_HEIGHT, config::L1_FRONT_2_L1_BACK,
                config::L1_BACK_HEIGHT),
        Segment(config::L2_UL_X, config::L2_UL_Y, config::L2_UR_X,
                config::L2_UR_Y),
        Segment(config::L2_LL_X, config::L2_LL_Y, config::L2_LR_X,
                config::L2_LR_Y),
        Segment(config::L3_UL_X, config::L3_UL_Y, config::L3_UR_X,
                config::L3_UR_Y),
        Segment(config::L3_LL_X, config::L3_LL_Y, config::L3_LR_X,
                config::L3_LR_Y),
        Segment(config::L4_UL_X, config::L4_UL_Y, config::L4_LL_X,
                config::L4_LL_Y),
        Segment(config::BRANCH_UL_X, config::BRANCH_UL_Y, config::BRANCH_LL_X,
                config::BRANCH_LL_Y),
        Segment(config::BRANCH_UL_X, config::BRANCH_UL_Y, config::L4_LL_X,
                config::L4_LL_Y),
    };
    this->x = 0;
    this->y = 0;
    this->theta = 0;
  }
  // arm
  Object(bool isSafe, bool withAlgae, bool withCoral) {
    if (isSafe) {
      if (withAlgae && withCoral) {
      } else if (withAlgae) {
      } else if (withCoral) {
      } else {
        this->segments = std::vector<Segment>{
            Segment(-.25, .1, .25, .1),
            Segment(.25, .1, .25, -.1),
            Segment(.25, -.1, -.25, -.1),
            Segment(-.25, -.1, -.25, .1),
        };
      }
    } else {
      if (withAlgae && withCoral) {
      } else if (withAlgae) {
      } else if (withCoral) {
      } else {
        this->segments = std::vector<Segment>{
            Segment(-.2, .05, .2, .05),
            Segment(.2, .05, .2, -.05),
            Segment(.2, -.05, -.2, -.05),
            Segment(-.2, -.05, -.2, .05),
        };
      }
    }
    this->x = 0;
    this->y = 0;
    this->theta = 0;
  }
  Object(std::vector<Segment> segments) {
    this->segments = segments;
    this->x = 0;
    this->y = 0;
    this->theta = 0;
  }
  Object(std::vector<Segment> segments, double x, double y, double theta) {
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
      for (Segment s2 : segs.getSegments()) {
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
        dt * config::ELEVATOR_COS_ANGLE - config::ELEVATOR_2_L1_FRONT,
        dt * config::ELEVATOR_SIN_ANGLE + config::ELEVATOR_2_GROUND, dr);
  }

  std::vector<Segment> getSegments() { return this->segments; }
  double getX() { return this->x; }
  double getY() { return this->y; }
  double getTheta() { return this->theta; }
};

#endif  // OBJECT_HPP