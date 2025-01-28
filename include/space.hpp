#ifndef SPACE_HPP
#define SPACE_HPP

#include <Eigen/Eigen>
#include <vector>

#include "Object.hpp"
#include "astar.hpp"
#include "config.h"
#include "log.hpp"

using namespace nextinnovation::alphabot;
namespace nextinnovation {
double clamp(double x, double lowerBound, double upperBound) {
  if (x < lowerBound) return lowerBound;
  if (x > upperBound) return upperBound;
  return x;
}

int clamp(int x, int lowerBound, int upperBound) {
  if (x < lowerBound) return lowerBound;
  if (x > upperBound) return upperBound;
  return x;
}

Eigen::Vector2i getGridIdx(const double t, const double r) {
  return Eigen::Vector2i(
      floor(
          (clamp(t, ELEVATOR_MIN_POSITION_METER, ELEVATOR_MAX_POSITION_METER) -
           ELEVATOR_MIN_POSITION_METER) /
          ELEVATOR_GRID_SIZE),
      floor((clamp(r, ARM_MIN_THETA_DEGREE, ARM_MAX_THETA_DEGREE) -
             ARM_MIN_THETA_DEGREE) /
            ARM_GRID_SIZE));
}

Eigen::Vector2i getGridIdx(const Eigen::Vector2d& tr) {
  return getGridIdx(tr[0], tr[1]);
}

std::vector<Eigen::Vector2i> getGridIdxs(
    const std::vector<Eigen::Vector2d>& trs) {
  std::vector<Eigen::Vector2i> idxs;
  for (auto tr : trs) {
    idxs.push_back(getGridIdx(tr));
  }
  return idxs;
}

Eigen::Vector2d getTR(const int t, const int r) {
  return Eigen::Vector2d(t * ELEVATOR_GRID_SIZE + ELEVATOR_MIN_POSITION_METER,
                         r * ARM_GRID_SIZE + ARM_MIN_THETA_DEGREE);
}

Eigen::Vector2d getTR(const Eigen::Vector2i& idx) {
  return getTR(idx[0], idx[1]);
}

std::vector<Eigen::Vector2d> getTRs(const std::vector<Eigen::Vector2i>& idxs) {
  std::vector<Eigen::Vector2d> trs;
  for (auto idx : idxs) {
    trs.push_back(getTR(idx));
  }
  return trs;
}

void getGridMap(ObjectType type, std::vector<std::vector<bool>>& map) {
  assert(type != ObjectType::ENV);
  Object env = Object(ObjectType::ENV);
  Object arm = Object(type);
  map = std::vector<std::vector<bool>>(ELEVATOR_GRID_NUMS,
                                       std::vector<bool>(ARM_GRID_NUMS, true));
  for (int tt = 0; tt < ELEVATOR_GRID_NUMS; ++tt) {
    for (int rr = 0; rr < ARM_GRID_NUMS; ++rr) {
      Eigen::Vector2d tr = getTR(tt, rr);
      map[tt][rr] = env.intersect(arm.armTransform(tr.x(), tr.y()));
    }
  }
  return;
}

void getGridMap(ObjectType type, std::vector<std::vector<double>>& map) {
  assert(type != ObjectType::ENV);
  Object env = Object(ObjectType::ENV);
  Object arm = Object(type);

  const double obstacle = OBSTACLE_OFFSET;
  const double reduce = OBSTACLE_FIELD_REDUCTION;
  map = std::vector<std::vector<double>>(
      ELEVATOR_GRID_NUMS, std::vector<double>(ARM_GRID_NUMS, true));
  // bound of the map
  for (int tt = 0; tt < ELEVATOR_GRID_NUMS; ++tt) {
    map[tt][0] = obstacle * reduce;
    map[tt][ARM_GRID_NUMS - 1] = obstacle * reduce;
  }
  for (int rr = 0; rr < ARM_GRID_NUMS; ++rr) {
    map[0][rr] = obstacle * reduce;
    map[ELEVATOR_GRID_NUMS - 1][rr] = obstacle * reduce;
  }
  for (int tt = 1; tt < ELEVATOR_GRID_NUMS - 1; ++tt) {
    for (int rr = 1; rr < ARM_GRID_NUMS - 1; ++rr) {
      Eigen::Vector2d tr = getTR(tt, rr);
      if (env.intersect(arm.armTransform(tr.x(), tr.y()))) {
        map[tt][rr] = obstacle;
      } else {
        double max_val = map[tt - 1][rr - 1];
        if (map[tt - 1][rr] > max_val) max_val = map[tt - 1][rr];
        if (map[tt][rr - 1] > max_val) max_val = map[tt][rr - 1];
        map[tt][rr] = max_val * reduce;
      }
    }
  }
  for (int tt = ELEVATOR_GRID_NUMS - 2; tt > 0; --tt) {
    for (int rr = ARM_GRID_NUMS - 2; rr > 0; --rr) {
      if (map[tt][rr] == obstacle) continue;
      double max_val = map[tt + 1][rr + 1];
      if (map[tt + 1][rr] > max_val) max_val = map[tt + 1][rr];
      if (map[tt][rr + 1] > max_val) max_val = map[tt][rr + 1];
      map[tt][rr] =
          map[tt][rr] > max_val * reduce ? map[tt][rr] : max_val * reduce;
    }
  }
  for (int tt = 0; tt < ELEVATOR_GRID_NUMS; ++tt) {
    for (int rr = 0; rr < ARM_GRID_NUMS; ++rr) {
      if (map[tt][rr] < 1) {
        map[tt][rr] = 0;
      }
    }
  }
  return;
}
}  // namespace nextinnovation

#endif