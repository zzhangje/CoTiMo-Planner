#ifndef SMOOTH_HPP
#define SMOOTH_HPP

#include <Eigen/Eigen>
#include <chrono>

#include "Object.hpp"
#include "lbfgs.hpp"

#define GRAD_GAP 1e-20
namespace nextinnovation {
class Smooth {
 private:
  int n;
  Eigen::VectorXd x;  // [t0, t1, ..., tn, r0, r1, ..., rn]'
  std::vector<Eigen::Vector2d> path;

  Object arm, env;
  int iter;
  lbfgs::lbfgs_parameter_t params;

 public:
  Smooth(const std::vector<Eigen::Vector2d>& heuristicPath, ObjectType type,
         int maxIter = 30) {
    this->n = heuristicPath.size();

    this->path = heuristicPath;
    this->x = Eigen::VectorXd::Zero(2 * this->n);
    this->convertToVariables();

    this->arm = Object(type);
    this->env = Object(ObjectType::ENV);
  }

  void getPath(std::vector<Eigen::Vector2d>& path) {
    this->convertToPath();
    path = this->path;
  }

  std::vector<Eigen::Vector2d> getPath() {
    this->convertToPath();
    return this->path;
  }

 private:
  static double loss(void* instance, const Eigen::VectorXd& optX,
                     Eigen::VectorXd& optG) {
    Smooth* smooth = static_cast<Smooth*>(instance);
    double res = 0;
    optG = Eigen::VectorXd::Zero(optX.size());

    // distance potential field
    for (int i = 1; i < smooth->n - 1; ++i) {
      res += exp(-smooth->env.distanceToObject(smooth->arm.armTransform(optX(i), optX(i + smooth->n))));

      optG(i) += (exp(-smooth->env.distanceToObject(smooth->arm.armTransform(optX(i) + GRAD_GAP, optX(i + smooth->n)))) - exp(-smooth->env.distanceToObject(smooth->arm.armTransform(optX(i) - GRAD_GAP, optX(i + smooth->n))))) / 2 / GRAD_GAP;

      optG(i + smooth->n) += (exp(-smooth->env.distanceToObject(smooth->arm.armTransform(optX(i), optX(i + smooth->n) + GRAD_GAP))) - exp(-smooth->env.distanceToObject(smooth->arm.armTransform(optX(i), optX(i + smooth->n) - GRAD_GAP)))) / 2 / GRAD_GAP;
    }

    // smoothness potential field
    for (int i = 1; i < smooth->n - 1; ++i) {
      res += pow(optX(i - 1) - 2 * optX(i) + optX(i + 1), 2);
      res += pow(optX(i + smooth->n) - 2 * optX(i - 1 + smooth->n) + optX(i - 2 + smooth->n), 2);

      if (i > 1) {
        optG(i - 1) += 2 * (optX(i - 1) - 2 * optX(i) + optX(i + 1));
      }
      if (i < smooth->n - 2) {
        optG(i + 1) += 2 * (optX(i - 1) - 2 * optX(i) + optX(i + 1));
      }
      optG(i) += -4 * (optX(i - 1) - 2 * optX(i) + optX(i + 1));
    }
  }

  void convertToPath() {
    for (int i = 0; i <= this->n; ++i) {
      this->path.push_back(Eigen::Vector2d(this->x(i), this->x(i + this->n)));
    }
  }

  void convertToVariables() {
    for (int i = 0; i <= this->n; ++i) {
      this->x(i) = this->path[i](0);
      this->x(i + this->n) = this->path[i](1);
    }
  }

  void solve(int maxIter = 30) {
    log_info("Solving Smooth problem...");

    double cost;

    auto now = std::chrono::steady_clock::now();
    auto lastMillis = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch())
                          .count();
    auto lastMicros = std::chrono::duration_cast<std::chrono::microseconds>(
                          now.time_since_epoch())
                          .count();
    auto beginTime = lastMillis;

    for (int iter = 0; iter < maxIter; ++iter) {
      this->iter++;
      Eigen::VectorXd x0 = x;
      int ret = lbfgs::lbfgs_optimize(x, cost, loss, NULL, NULL, this, params);
      if (ret < 0) {
        log_error("L-BFGS optimization failed with code %d.", ret);
        break;
      }

      now = std::chrono::steady_clock::now();
      auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch())
                        .count();
      auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                        now.time_since_epoch())
                        .count();

      if (true || iter % 10 == 0) {
        log_debug("iter: %2d, duration: %3d.%3dms, loss: %f, dx: %f", iter, (millis - lastMillis) % 1000, (micros - lastMicros) % 1000, cost, (x - x0).norm());
      }

      lastMillis = millis;
      lastMicros = micros;
    }

    now = std::chrono::steady_clock::now();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch())
                      .count();
    log_info("Smooth problem solved, total duration: %3d,%3dms, iterations: %d, loss: %f", (millis - beginTime) / 1000, (millis - beginTime) % 1000, maxIter, cost);
    return;
  }
};
};  // namespace nextinnovation

#endif  // SMOOTH_HPP