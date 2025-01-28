#ifndef SMOOTH_HPP
#define SMOOTH_HPP

#include <Eigen/Eigen>

#include "Object.hpp"

class Smooth {
 private:
  int n;
  Eigen::VectorXd x;  // [t0, t1, ..., tn, r0, r1, ..., rn]'

  Object arm, env;

 public:
  Smooth(const std::vector<Eigen::Vector2d>& heuristicPath, ObjectType type,
         int maxIter = 30) {
    this->n = heuristicPath.size();
    this->x = Eigen::VectorXd::Zero(2 * this->n);
    for (int i = 0; i <= this->n; ++i) {
      this->x(i) = heuristicPath[i](0);
      this->x(i + this->n) = heuristicPath[i](1);
    }
    this->arm = Object(type);
    this->env = Object(ObjectType::ENV);
  }

  void getPath(std::vector<Eigen::Vector2d>& path) {
    path.clear();
    for (int i = 0; i <= this->n; ++i) {
      path.push_back(Eigen::Vector2d(this->x(i), this->x(i + this->n)));
    }
  }

  std::vector<Eigen::Vector2d> getPath() {
    std::vector<Eigen::Vector2d> path;
    for (int i = 0; i <= this->n; ++i) {
      path.push_back(Eigen::Vector2d(this->x(i), this->x(i + this->n)));
    }
    return path;
  }

 private:
  static double loss(void* instance, const Eigen::VectorXd& optX,
                     Eigen::VectorXd& optG) {
    double res = 0;
  }

  void solve(int maxIter = 30) {}
};

#endif  // SMOOTH_HPP