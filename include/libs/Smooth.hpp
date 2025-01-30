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
  Eigen::VectorXd x, x0;  // [t0, t1, ..., tn, r0, r1, ..., rn]'
  std::vector<Eigen::Vector2d> path;

  Object arm, env;
  int iter;
  lbfgs::lbfgs_parameter_t params;

 public:
  Smooth(const std::vector<Eigen::Vector2d>& heuristicPath, ObjectType type,
         int maxIter = 30)
      : arm(type), env(ObjectType::ENV) {
    this->n = heuristicPath.size();

    this->path = heuristicPath;
    this->x = Eigen::VectorXd::Zero(2 * this->n);
    this->convertToVariables();
    this->x0 = x;

    this->params = lbfgs::lbfgs_parameter_t();
    // this->params.g_epsilon = 1.0e-8;
    // this->params.past = 3;
    // this->params.delta = 1.0e-8;

    this->solve(maxIter);
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

    // gravity potential field
    // for (int i = 1; i < smooth->n - 1; ++i) {
    //   double d = smooth->env.distanceToObject(
    //       smooth->arm.armTransform(optX(i), optX(i + smooth->n)));

    //   res += exp(-d * 50);

    //   optG(i) += -exp(-d * 50) * (smooth->env.distanceToObject(smooth->arm.armTransform(optX(i) + GRAD_GAP, optX(i + smooth->n))) - d) / GRAD_GAP;

    //   optG(i + smooth->n) += -exp(-d) * (smooth->env.distanceToObject(smooth->arm.armTransform(optX(i), optX(i + smooth->n) + GRAD_GAP)) - d) / GRAD_GAP;
    // }

    // distance potential field
    // for (int i = 0; i < smooth->n - 1; ++i) {
    //   double d = pow(optX(i) - optX(i + 1), 2);
    //   optG(i) += 2 * d;
    //   optG(i + 1) += -2 * d;
    //   res += d;

    //   d = pow(optX(i + smooth->n) - optX(i + smooth->n + 1), 2);
    //   optG(i + smooth->n) += 2 * d;
    //   optG(i + smooth->n + 1) += -2 * d;
    //   res += d;
    // }

    // smoothness potential field
    for (int i = 1; i < smooth->n - 1; ++i) {
      double d = pow(optX(i - 1) - 2 * optX(i) + optX(i + 1), 2);
      optG(i - 1) += 2 * d;
      optG(i) += -4 * d;
      optG(i + 1) += 2 * d;
      res += d;

      d = pow(optX(i + smooth->n - 1) - 2 * optX(i + smooth->n) + optX(i + smooth->n + 1), 2);
      optG(i + smooth->n - 1) += 2 * d;
      optG(i + smooth->n) += -4 * d;
      optG(i + smooth->n + 1) += 2 * d;
      res += d;
    }

    // penalty potential field
    // for (int i = 1; i < smooth->n - 1; ++i) {
    //   res += .5 * (pow(optX(i) - smooth->x0(i), 2) + pow(optX(i + smooth->n) - smooth->x0(i + smooth->n), 2));
    //   optG(i) += (optX(i) - smooth->x0(i));
    //   optG(i + smooth->n) += (optX(i + smooth->n) - smooth->x0(i + smooth->n));
    // }

    optG(0) = 0;
    optG(smooth->n - 1) = 0;
    optG(smooth->n) = 0;
    optG(2 * smooth->n - 1) = 0;

    return res;
  }

  void convertToPath() {
    this->path.clear();
    for (int i = 0; i < this->n; ++i) {
      this->path.push_back(Eigen::Vector2d(this->x(i), this->x(i + this->n)));
    }
  }

  void convertToVariables() {
    for (int i = 0; i < this->n; ++i) {
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
        // break;
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