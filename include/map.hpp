#ifndef MAP_HPP
#define MAP_HPP

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "Object.hpp"
#include "config.h"
#include "log.hpp"

using namespace config::dynamic;
using namespace config::alphabot;

double clamp(double x, double min, double max) {
  return std::min(std::max(x, min), max);
}

int clamp(int x, int min, int max) { return std::min(std::max(x, min), max); }

Eigen::Vector2i getGridIdx(const double t, const double r) {
  return Eigen::Vector2i(floor((clamp(t, ELEVATOR_MIN_POSITION,
                                      ELEVATOR_MAX_POSITION) -
                                ELEVATOR_MIN_POSITION) /
                               ELEVATOR_GRID_SIZE),
                         floor((clamp(r, ARM_MIN_THETA_ROTATION,
                                      ARM_MAX_THETA_ROTATION) -
                                ARM_MIN_THETA_ROTATION) /
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
  return Eigen::Vector2d(
      t * ELEVATOR_GRID_SIZE + ELEVATOR_MIN_POSITION,
      r * ARM_GRID_SIZE + ARM_MIN_THETA_ROTATION);
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
  auto env = Object(ObjectType::ENV);
  map = std::vector<std::vector<bool>>(
      ELEVATOR_GRID_NUMS,
      std::vector<bool>(ARM_GRID_NUMS, true));
  for (int tt = 0; tt < ELEVATOR_GRID_NUMS; ++tt) {
    for (int rr = 0; rr < ARM_GRID_NUMS; ++rr) {
      Eigen::Vector2d tr = getTR(tt, rr);
      Object arm = Object(type, tr.x(), tr.y());
      // true if no intersection
      map[tt][rr] = !env.intersect(arm);
    }
  }
  log_debug("Grid map size: %d x %d", ELEVATOR_GRID_NUMS, ARM_GRID_NUMS);
  return;
}

void getGridMap(ObjectType type,
                std::vector<std::vector<double>>& map) {
  assert(type != ObjectType::ENV);
  map = std::vector<std::vector<double>>(
      ELEVATOR_GRID_NUMS,
      std::vector<double>(ARM_GRID_NUMS, true));
  auto env = Object(ObjectType::ENV);
  const double obstacle = OBSTACLE_OFFSET;
  const double reduce = OBSTACLE_FIELD_REDUCTION;
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
      Object arm = Object(type, tr.x(), tr.y());
      if (arm.intersect(env)) {
        map[tt][rr] = obstacle;
      } else {
        map[tt][rr] = std::max(map[tt - 1][rr - 1],
                               std::max(map[tt - 1][rr], map[tt][rr - 1])) *
                      reduce;
      }
    }
  }
  for (int tt = ELEVATOR_GRID_NUMS - 2; tt > 0; --tt) {
    for (int rr = ARM_GRID_NUMS - 2; rr > 0; --rr) {
      if (map[tt][rr] == obstacle) continue;
      map[tt][rr] = std::max(
          map[tt][rr], std::max(map[tt + 1][rr + 1],
                                std::max(map[tt + 1][rr], map[tt][rr + 1])) *
                           reduce);
    }
  }
  for (int tt = 0; tt < ELEVATOR_GRID_NUMS; ++tt) {
    for (int rr = 0; rr < ARM_GRID_NUMS; ++rr) {
      if (map[tt][rr] < 1) {
        map[tt][rr] = 0;
      }
    }
  }
  log_debug("Grid map size: %d x %d", ELEVATOR_GRID_NUMS, ARM_GRID_NUMS);
  return;
}

#endif