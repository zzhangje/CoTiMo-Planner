#ifndef MAP_HPP
#define MAP_HPP

#include <nav_msgs/GridCells.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

void renderMap(nav_msgs::GridCells& map_msg,
               const std::vector<std::vector<bool>>& map,
               const double height = 0) {
  map_msg.header.frame_id = "map";
  map_msg.cell_width = config::ELEVATOR_GRID_SIZE;
  map_msg.cell_height = config::ARM_GRID_SIZE;
  for (int i = 0; i < config::ELEVATOR_GRID_NUMS; ++i) {
    for (int j = 0; j < config::ARM_GRID_NUMS; ++j) {
      if (!map[i][j]) {
        // mark if obstacle
        geometry_msgs::Point p;
        p.x = getTR(i, j)[0];
        p.y = getTR(i, j)[1];
        p.z = height;
        map_msg.cells.push_back(p);
      }
    }
  }
  return;
}

void renderMap(nav_msgs::GridCells& map_msg,
               const std::vector<std::vector<double>>& map,
               const double height = 0) {
  map_msg.header.frame_id = "map";
  map_msg.cell_width = config::ELEVATOR_GRID_SIZE;
  map_msg.cell_height = config::ARM_GRID_SIZE;
  for (int i = 0; i < config::ELEVATOR_GRID_NUMS; ++i) {
    for (int j = 0; j < config::ARM_GRID_NUMS; ++j) {
      if (map[i][j] > config::OBSTACLE_OFFSET - .1) {
        // mark if obstacle
        geometry_msgs::Point p;
        p.x = getTR(i, j)[0];
        p.y = getTR(i, j)[1];
        p.z = height;
        map_msg.cells.push_back(p);
      }
    }
  }
  return;
}

void renderBound(nav_msgs::GridCells& boundary_msg) {
  boundary_msg.header.frame_id = "map";
  boundary_msg.cell_width = config::ELEVATOR_GRID_SIZE;
  boundary_msg.cell_height = config::ARM_GRID_SIZE;
  for (int i = 0; i < config::ELEVATOR_GRID_NUMS; ++i) {
    geometry_msgs::Point p1, p2;
    p1.x = getTR(i, 0)[0];
    p1.y = getTR(i, 0)[1];
    p1.z = 0;
    p2.x = getTR(i, config::ARM_GRID_NUMS - 1)[0];
    p2.y = getTR(i, config::ARM_GRID_NUMS - 1)[1];
    p2.z = 0;
    boundary_msg.cells.push_back(p1);
    boundary_msg.cells.push_back(p2);
  }
  for (int j = 0; j < config::ARM_GRID_NUMS; ++j) {
    geometry_msgs::Point p1, p2;
    p1.x = getTR(0, j)[0];
    p1.y = getTR(0, j)[1];
    p1.z = 0;
    p2.x = getTR(config::ELEVATOR_GRID_NUMS - 1, j)[0];
    p2.y = getTR(config::ELEVATOR_GRID_NUMS - 1, j)[1];
    p2.z = 0;
    boundary_msg.cells.push_back(p1);
    boundary_msg.cells.push_back(p2);
  }
  return;
}

void renderPoints(visualization_msgs::Marker& points_msg,
                  const std::vector<Eigen::Vector2d>& points) {
  points_msg.header.frame_id = "map";
  points_msg.ns = "points";
  points_msg.id = 0;
  points_msg.type = visualization_msgs::Marker::SPHERE_LIST;
  points_msg.action = visualization_msgs::Marker::ADD;
  points_msg.scale.x = 2 * config::ELEVATOR_GRID_SIZE;
  points_msg.scale.y = 2 * config::ARM_GRID_SIZE;
  points_msg.scale.z = 1;
  points_msg.pose.orientation.w = 1.0;
  points_msg.color.a = .9;
  points_msg.color.r = 1.;
  points_msg.color.g = 1.;
  points_msg.color.b = 0;

  geometry_msgs::Point p;
  for (int i = 0; i < points.size(); ++i) {
    p.x = points[i][0];
    p.y = points[i][1];
    p.z = 1.;
    points_msg.points.push_back(p);
  }
  return;
}

void renderPath(visualization_msgs::Marker& path_msg,
                const std::vector<Eigen::Vector2d>& path) {
  path_msg.header.frame_id = "map";
  path_msg.ns = "path";
  path_msg.id = 0;
  path_msg.type = visualization_msgs::Marker::LINE_STRIP;
  path_msg.action = visualization_msgs::Marker::ADD;
  path_msg.scale.x = config::ELEVATOR_GRID_SIZE;
  path_msg.pose.orientation.w = 1.0;
  path_msg.color.a = .6;
  path_msg.color.r = 1;
  path_msg.color.g = 0;
  path_msg.color.b = 0;

  geometry_msgs::Point p;
  for (auto& x : path) {
    p.x = x[0];
    p.y = x[1];
    p.z = 0;
    path_msg.points.push_back(p);
  }
  return;
}

#endif