#ifndef MAP_HPP
#define MAP_HPP

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "config.h"
#include "object.hpp"

double clamp(double x, double min, double max) {
  return std::min(std::max(x, min), max);
}

int clamp(int x, int min, int max) { return std::min(std::max(x, min), max); }

Eigen::Vector2i getGridIdx(const double t, const double r) {
  return Eigen::Vector2i(floor((clamp(t, config::ELEVATOR_MIN_POSITION,
                                      config::ELEVATOR_MAX_POSITION) -
                                config::ELEVATOR_MIN_POSITION) /
                               config::ELEVATOR_GRID_SIZE),
                         floor((clamp(r, config::ARM_MIN_THETA_RADIAN,
                                      config::ARM_MAX_THETA_RADIAN) -
                                config::ARM_MIN_THETA_RADIAN) /
                               config::ARM_GRID_SIZE));
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
      t * config::ELEVATOR_GRID_SIZE + config::ELEVATOR_MIN_POSITION,
      r * config::ARM_GRID_SIZE + config::ARM_MIN_THETA_RADIAN);
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

void getGridMap(Object& env, Object& arm, std::vector<std::vector<bool>>& map) {
  map = std::vector<std::vector<bool>>(
      config::ELEVATOR_GRID_NUMS,
      std::vector<bool>(config::ARM_GRID_NUMS, true));
  for (int tt = 0; tt < config::ELEVATOR_GRID_NUMS; ++tt) {
    for (int rr = 0; rr < config::ARM_GRID_NUMS; ++rr) {
      Object arm_copy = Object(arm);
      Eigen::Vector2d tr = getTR(tt, rr);
      arm_copy.armTransform(tr[0], tr[1]);
      // true if no intersection
      map[tt][rr] = !arm_copy.intersect(env);
    }
  }
  return;
}

void getGridMap(Object& env, Object& arm,
                std::vector<std::vector<double>>& map) {
  map = std::vector<std::vector<double>>(
      config::ELEVATOR_GRID_NUMS,
      std::vector<double>(config::ARM_GRID_NUMS, true));
  const double obstacle = config::OBSTACLE_OFFSET;
  const double reduce = config::OBSTACLE_FIELD_REDUCTION;
  // bound of the map
  for (int tt = 0; tt < config::ELEVATOR_GRID_NUMS; ++tt) {
    map[tt][0] = obstacle * reduce;
    map[tt][config::ARM_GRID_NUMS - 1] = obstacle * reduce;
  }
  for (int rr = 0; rr < config::ARM_GRID_NUMS; ++rr) {
    map[0][rr] = obstacle * reduce;
    map[config::ELEVATOR_GRID_NUMS - 1][rr] = obstacle * reduce;
  }
  for (int tt = 1; tt < config::ELEVATOR_GRID_NUMS - 1; ++tt) {
    for (int rr = 1; rr < config::ARM_GRID_NUMS - 1; ++rr) {
      Object arm_copy = Object(arm);
      Eigen::Vector2d tr = getTR(tt, rr);
      arm_copy.armTransform(tr[0], tr[1]);
      if (arm_copy.intersect(env)) {
        map[tt][rr] = obstacle;
      } else {
        map[tt][rr] = std::max(map[tt - 1][rr - 1],
                               std::max(map[tt - 1][rr], map[tt][rr - 1])) *
                      reduce;
      }
    }
  }
  for (int tt = config::ELEVATOR_GRID_NUMS - 2; tt > 0; --tt) {
    for (int rr = config::ARM_GRID_NUMS - 2; rr > 0; --rr) {
      if (map[tt][rr] == obstacle) continue;
      map[tt][rr] = std::max(
          map[tt][rr], std::max(map[tt + 1][rr + 1],
                                std::max(map[tt + 1][rr], map[tt][rr + 1])) *
                           reduce);
    }
  }
  for (int tt = 0; tt < config::ELEVATOR_GRID_NUMS; ++tt) {
    for (int rr = 0; rr < config::ARM_GRID_NUMS; ++rr) {
      if (map[tt][rr] < 1) {
        map[tt][rr] = 0;
      }
    }
  }
  return;
}

#endif