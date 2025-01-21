#ifndef TOPP_HPP
#define TOPP_HPP

#include <ros/ros.h>

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "config.h"
#include "lbfgs.hpp"
#include "spline.hpp"

class Topp {
 private:
  void projection(Eigen::VectorXd& P_k, const Eigen::VectorXd& v) {
    if (v.rows() == 1) {
      P_k(0) = v(0);
      return;
    }
    if (v(0) > v.tail(v.rows() - 1).norm()) {
      P_k(0) = v(0);
      P_k.tail(v.rows() - 1) = v.tail(v.rows() - 1);
      return;
    } else if (v(0) < -v.tail(v.rows() - 1).norm()) {
      return;
    } else {
      P_k(0) = v.tail(v.rows() - 1).norm();
      P_k.tail(v.rows() - 1) = v.tail(v.rows() - 1);
      P_k *= (v(0) + v.tail(v.rows() - 1).norm()) / 2 /
             v.tail(v.rows() - 1).norm();
      return;
    }
    return;
  }

  void gradient() {
    grad = f;
    int n_conic = A[0].rows(), n_eq = G.rows(), n_neq = P.rows();
    grad += rho * G.transpose() * (G * x - h + lambda / rho);
    for (int i = 0; i < A.size(); ++i) {
      Eigen::VectorXd Pconic = Eigen::VectorXd::Zero(n_conic),
                      conic = mu[i] / rho - A[i] * x - b[i];
      projection(Pconic, conic);
      grad -= rho * A[i].transpose() * Pconic;
    }
    Eigen::VectorXd Px_q = P * x - q + eta / rho;
    for (int i = 0; i < P.rows(); ++i) {
      Px_q(i) = std::max(0.0, Px_q(i));
    }
    grad += rho * P.transpose() * Px_q;
  }

  double loss() {
    double res = 0;
    res += f.dot(x);
    int n_conic = A[0].rows(), n_eq = G.rows(), n_neq = P.rows();
    res += rho / 2 * (G * x - h + lambda / rho).squaredNorm();
    for (int i = 0; i < A.size(); ++i) {
      Eigen::VectorXd Pconic = Eigen::VectorXd::Zero(n_conic),
                      conic = mu[i] / rho - A[i] * x - b[i];
      projection(Pconic, conic);
      res += rho / 2 * Pconic.squaredNorm();
    }
    for (int i = 0; i < P.rows(); ++i) {
      res += rho / 2 * std::max(0.0, (P.row(i) * x - q(i) + eta(i) / rho));
    }
    return res;
  }

  /**
   * a:    0 ~  n-1
   * b:  n   ~ 2n
   * c: 2n+1 ~ 3n+1
   * d: 3n+2 ~ 4n+1
   */
  Eigen::VectorXd x, f, h, q, lambda, eta, grad, arc, qx1, qx2, qy1, qy2, ax,
      ay, bx, by, cx, cy, dx, dy;
  Eigen::MatrixXd G, P;
  std::vector<Eigen::MatrixXd> A;
  std::vector<Eigen::VectorXd> b, mu;
  double rho;
  int iter = 0;

  static double loss(void* instance, const Eigen::VectorXd& x,
                     Eigen::VectorXd& grad) {
    Topp* topp = (Topp*)instance;
    topp->x = x;
    topp->gradient();
    grad = topp->grad;
    return topp->loss();
  }

 public:
  Topp() {}
  Topp(const std::vector<Eigen::Vector2d>& points) {
    int n = points.size() - 1;

    Eigen::VectorXd qx(n + 1), qy(n + 1);
    for (int i = 0; i <= n; ++i) {
      qx(i) = points[i](0), qy(i) = points[i](1);
    }
    spline::spline(qx, ax, bx, cx, dx);
    spline::spline(qy, ay, by, cy, dy);

    qx1 = Eigen::VectorXd::Zero(n + 1), qy1 = Eigen::VectorXd::Zero(n + 1),
    qx2 = Eigen::VectorXd::Zero(n + 1), qy2 = Eigen::VectorXd::Zero(n + 1);
    arc = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
      qx1(i) = cx(i);
      qy1(i) = cy(i);
      qx2(i) = 2 * bx(i);
      qy2(i) = 2 * by(i);
      for (double u = .01; u < 1; u += .01) {
        arc(i) += .01 * sqrt(pow(3 * ax(i) * u * u + 2 * bx(i) * u + cx(i), 2) +
                             pow(3 * ay(i) * u * u + 2 * by(i) * u + cy(i), 2));
      }
      arc(i) += .005 * (sqrt(pow(3 * ax(i) + 2 * bx(i) + cx(i), 2) +
                             pow(3 * ay(i) + 2 * by(i) + cy(i), 2)) +
                        sqrt(cx(i) * cx(i) + cy(i) * cy(i)));
    }
    qx1(n) = 3 * ax(n - 1) + 2 * bx(n - 1) + cx(n - 1);
    qy1(n) = 3 * ay(n - 1) + 2 * by(n - 1) + cy(n - 1);
    qx2(n) = 6 * ax(n - 1) + 2 * bx(n - 1);
    qy2(n) = 6 * ax(n - 1) + 2 * by(n - 1);

    f = Eigen::VectorXd::Zero(4 * n + 2);
    x = Eigen::VectorXd::Zero(4 * n + 2);
    G = Eigen::MatrixXd::Zero(n + 4, 4 * n + 2);
    h = Eigen::VectorXd::Zero(n + 4);
    P = Eigen::MatrixXd::Zero(7 * n + 3, 4 * n + 2);
    q = Eigen::VectorXd::Zero(7 * n + 3);

    Eigen::MatrixXd A_;
    Eigen::VectorXd b_, c_;
    A.clear(), b.clear();

    /**
     * a:    0 ~  n-1
     * b:  n   ~ 2n
     * c: 2n+1 ~ 3n+1
     * d: 3n+2 ~ 4n+1
     */
    for (int i = 0; i < n; ++i) {
      /**
       * \Sum 2(s_{k+1} - s_k) * d_k
       */
      f(3 * n + 2 + i) = 2 * arc(i);

      /**
       * ||  2*c_k  ||
       * ||         || <= b_k + 1
       * || b_k - 1 ||
       *
       * || b_k + 1 ||
       * ||  2*c_k  || in Q
       * || b_k - 1 ||
       */
      A_ = Eigen::MatrixXd::Zero(3, 4 * n + 2);
      b_ = Eigen::VectorXd::Zero(3);
      c_ = Eigen::VectorXd::Zero(4 * n + 2);
      A_(0, n + i) = 1;
      A_(1, 2 * n + 1 + i) = 2;
      A_(2, n + i) = 1;
      b_(0) = 1;
      b_(2) = -1;
      A.push_back(A_), b.push_back(b_);
      /**
       * ||          2          ||
       * ||                     || <= c_{k+1} + c_k + d_k
       * || c_{k+1} + c_k - d_k ||
       *
       * || c_{k+1} + c_k + d_k ||
       * ||          2          || in Q
       * || c_{k+1} + c_k - d_k ||
       */
      A_ = Eigen::MatrixXd::Zero(3, 4 * n + 2);
      b_ = Eigen::VectorXd::Zero(3);
      A_(0, 2 * n + 1 + i) = 1;
      A_(0, 2 * n + 2 + i) = 1;
      A_(0, 3 * n + 2 + i) = 1;
      A_(2, 2 * n + 1 + i) = 1;
      A_(2, 2 * n + 2 + i) = 1;
      A_(2, 3 * n + 2 + i) = -1;
      b_(1) = 2;
      A.push_back(A_), b.push_back(b_);

      /**
       * 2(s_{k+1} - s_k) * a_k + b_k - b_{k+1} = 0
       */
      G(i, i) = 2 * arc(i);
      G(i, n + i) = 1;
      G(i, n + i + 1) = -1;
      // h(i) = 0;

      /**
       * -b_k <= 0
       */
      P(7 * i, n + i) = -1;
      // q(7 * i) = 0;

      /**
       * q'_x(s_k)^2 * b_k <= vmax^2
       * q'_y(s_k)^2 * b_k <= vmax^2
       */
      P(7 * i + 1, n + i) = qx1(i) * qx1(i);
      P(7 * i + 2, n + i) = qy1(i) * qy1(i);
      q(7 * i + 1) = config::ELEVATOR_VMAX * config::ELEVATOR_VMAX;
      q(7 * i + 2) = config::ARM_VMAX * config::ARM_VMAX;

      /**
       *  q''_x(s_k)^2 * b_k + q'_x(s_k) * a_k <= amax
       *  q''_y(s_k)^2 * b_k + q'_y(s_k) * a_k <= amax
       * -q''_x(s_k)^2 * b_k - q'_x(s_k) * a_k <= amax
       * -q''_y(s_k)^2 * b_k - q'_y(s_k) * a_k <= amax
       */
      P(7 * i + 3, n + i) = qx2(i) * qx2(i);
      P(7 * i + 4, n + i) = qy2(i) * qy2(i);
      P(7 * i + 5, n + i) = -qx2(i) * qx2(i);
      P(7 * i + 6, n + i) = -qy2(i) * qy2(i);
      P(7 * i + 3, i) = qx1(i);
      P(7 * i + 4, i) = qy1(i);
      P(7 * i + 5, i) = -qx1(i);
      P(7 * i + 6, i) = -qy1(i);
      q(7 * i + 3) = config::ELEVATOR_AMAX;
      q(7 * i + 4) = config::ARM_AMAX;
      q(7 * i + 5) = config::ELEVATOR_AMAX;
      q(7 * i + 6) = config::ARM_AMAX;
    }

    /**
     * ||  2*c_k  ||
     * ||         || <= b_k + 1
     * || b_k - 1 ||
     *
     * || b_k + 1 ||
     * ||  2*c_k  || in Q
     * || b_k - 1 ||
     */
    A_ = Eigen::MatrixXd::Zero(3, 4 * n + 2);
    b_ = Eigen::VectorXd::Zero(3);
    A_(0, 2 * n) = 1;
    A_(1, 3 * n + 1) = 2;
    A_(2, 2 * n) = 1;
    b_(2) = -1;
    A.push_back(A_);
    b.push_back(b_);

    /**
     * (q'_x(s_0)^2 + q'_y(s_0)^2) * b_0 = vstart^2
     * (q'_x(s_k)^2 + q'_y(s_k)^2) * b_n = vend^2
     */
    G(n, n) = qx1(0) * qx1(0);
    G(n + 1, n) = qy1(0) * qy1(0);
    G(n + 2, 2 * n) = qx1(n) * qx1(n);
    G(n + 3, 2 * n) = qy1(n) * qy1(n);

    /**
     * -b_k <= 0
     * q'_x(s_k)^2 * b_k <= vmax^2
     * q'_y(s_k)^2 * b_k <= vmax^2
     */
    P(7 * n, 2 * n) = -1;
    // q(7 * n) = 0;
    P(7 * n + 1, 2 * n) = qx1(n) * qx1(n);
    P(7 * n + 2, 2 * n) = qy1(n) * qy1(n);
    q(7 * n + 1) = config::ELEVATOR_VMAX * config::ELEVATOR_VMAX;
    q(7 * n + 2) = config::ARM_VMAX * config::ARM_VMAX;
    rho = 1;

    x = Eigen::VectorXd::Zero(f.rows());
    mu = std::vector<Eigen::VectorXd>(A.size(), Eigen::VectorXd::Zero(3));
    lambda = Eigen::VectorXd::Zero(G.rows());
    eta = Eigen::VectorXd::Zero(P.rows());
  }

  bool step() {
    ++iter;
    double cost;
    lbfgs::lbfgs_parameter_t params;
    params.g_epsilon = 1.0e-8;
    params.past = 3;
    params.delta = 1.0e-8;
    lbfgs::lbfgs_optimize(x, cost, loss, NULL, NULL, this, params);
    lambda = lambda + rho * (G * x - h);
    for (int i = 0; i < A.size(); ++i) {
      projection(mu[i], mu[i] - rho * (A[i] * x + b[i]));
    }
    eta = eta + rho * (P * x - q);
    for (int i = 0; i < eta.rows(); ++i) {
      eta(i) = std::max(0.0, eta(i));
    }
    rho = std::min(config::TOPP_BETA, rho * (1 + config::TOPP_GAMMA));
    return iter >= config::TOPP_ITER;
  }

  double getLoss() { return loss(); }
  int getIter() { return iter; }

  void get(std::vector<double>& timestamp,
           std::vector<Eigen::Vector2d>& velocity,
           std::vector<Eigen::Vector2d>& acceleration) {
    // TODO
    timestamp.clear(), velocity.clear(), acceleration.clear();
    int n = arc.rows();
    Eigen::VectorXd a_k = x.segment(0, n);
    Eigen::VectorXd b_k = x.segment(n, n + 1);
    Eigen::VectorXd c_k = x.segment(2 * n + 1, n + 1);
    Eigen::VectorXd d_k = x.segment(3 * n + 1, n);
    timestamp.push_back(0);
    for (int i = 0; i < n; ++i) {
      timestamp.push_back(timestamp.back() +
                          std::max(0.0, 2 * arc(i) * d_k(i)));
      velocity.push_back(
          Eigen::Vector2d(qx1(i) * sqrt(b_k(i)), qy1(i) * sqrt(b_k(i))));
      acceleration.push_back(
          Eigen::Vector2d(qx2(i) * b_k(i) + qx1(i) * a_k(i),
                          qy2(i) * b_k(i) + qy1(i) * a_k(i)));
    }
    velocity.push_back(
        Eigen::Vector2d(qx1(n) * sqrt(b_k(n)), qy1(n) * sqrt(b_k(n))));
    return;
  }
};

#endif