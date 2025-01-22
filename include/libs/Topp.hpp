#ifndef TOPP_HPP
#define TOPP_HPP

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "Logger.hpp"
#include "lbfgs.hpp"
#include "spline.hpp"

class Topp {
 private:
  std::vector<Eigen::Vector2d> points;

 public:
  Topp(const std::vector<Eigen::Vector2d>& points, bool setup = true,
       bool solve = true, int maxIter = 30) {
    this->points = points;
    if (setup) {
      this->setup();
      if (solve) {
        this->solve(maxIter);
      }
    }
  }
  void setup() {
    Logger::getInstance()->log(LogLevel::INFO, "TOPP",
                               "Setting up TOPP problem...");

    Logger::getInstance()->log(LogLevel::INFO, "TOPP",
                               "TOPP problem setup complete.");
  }
  void solve(int maxIter) {
    Logger::getInstance()->log(LogLevel::INFO, "TOPP",
                               "Solving TOPP problem...");

    Logger::getInstance()->log(LogLevel::INFO, "TOPP", "TOPP problem solved.");
  }
};

#endif