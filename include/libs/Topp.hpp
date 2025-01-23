#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <thread>

#include "config.h"
#include "lbfgs.hpp"
#include "log.hpp"
#include "spline.hpp"
#include "utils.hpp"

#define BETA 1e3
#define GAMMA .1

class Topp {
 private:
  std::vector<Eigen::Vector2d> points;
  int n;
  int iter;
  lbfgs::lbfgs_parameter_t params;

  /**
   * spline parameters
   */
  Eigen::VectorXd qt, qr, qt1, qr1, qt2, qr2, arc;
  Eigen::VectorXd qta, qtb, qtc, qtd, qra, qrb, qrc, qrd;

  /**
   * optimization variables
   * a:  0   ~  n-1  (n)
   * b:  n   ~  2n   (n+1)
   * c: 2n+1 ~ 3n+1  (n+1)
   * d: 3n+2 ~ 4n+1  (n)
   * e: 4n+2 ~ 5n+2  (n+1)
   */
  Eigen::VectorXd x, g;
  int n_var;

  /**
   * linear coefficients
   */
  Eigen::VectorXd f;

  /**
   * second order cone constraints
   */
  std::vector<Eigen::MatrixXd> As;
  std::vector<Eigen::VectorXd> bs;
  int n_soc;

  /**
   * linear equality constraints
   */
  Eigen::MatrixXd G;
  Eigen::VectorXd h;
  int n_eq;

  /**
   * linear inequality constraints
   */
  Eigen::MatrixXd P;
  Eigen::VectorXd q;
  int n_ineq;

  /**
   * quadratic equality constraints
   */
  std::vector<Eigen::MatrixXd> Js;
  std::vector<Eigen::VectorXd> rs;
  int n_quadeq;

  /**
   * dual variables
   */
  double rho;
  Eigen::VectorXd lambda, eta;
  std::vector<Eigen::VectorXd> mus;

 public:
  Topp(const std::vector<Eigen::Vector2d>& points, bool solve = true,
       int maxIter = 30) {
    this->points = points;
    this->n = points.size() - 1;
    this->iter = 0;
    this->setup();
    if (solve) {
      this->solve(maxIter);
    }
  }

  void solve(int maxIter = 30) {
    log_info("Solving TOPP problem...");

    double cost;

    auto now = std::chrono::high_resolution_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto lastMillis = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch())
                          .count();
    auto lastMicros = std::chrono::duration_cast<std::chrono::microseconds>(
                          now.time_since_epoch())
                          .count();

    for (int iter = 0; iter < maxIter; ++iter) {
      this->iter++;
      lbfgs::lbfgs_optimize(x, cost, loss, NULL, NULL, this, params);

      now = std::chrono::high_resolution_clock::now();
      time_t_now = std::chrono::system_clock::to_time_t(now);
      auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch())
                        .count();
      auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                        now.time_since_epoch())
                        .count();

      // update dual variables
      mus = socProjections(mus - rho * (As * x + bs));
      lambda = lambda + rho * (G * x - h);
      eta = max(eta + rho * (P * x - q), 0);
      rho = std::min(rho * (1 + GAMMA), BETA);

      log_debug("iter: %2d, duration: %3d.%3dms, loss: %f", this->iter, (millis - lastMillis) % 1000, (micros - lastMicros) % 1000, cost);
      lastMillis = millis;
      lastMicros = micros;
    }

    log_info("TOPP problem solved.");
  }

  int getIter() { return iter; }
  double getLoss() {
    return f.dot(x) +
           rho / 2 * ((G * x - h - lambda / rho).squaredNorm() + squaredNorm(x.transpose() * Js * x - transpose(rs) * x) + (max(P * x - q + eta / rho, 0)).squaredNorm() + sum(squaredNorm(socProjections(mus / rho - As * x - bs))));
  }
  void get() {}

 private:
  void setup() {
    log_info("Setting up TOPP problem...");

    // get spline parameters
    qt = Eigen::VectorXd::Zero(n + 1), qr = Eigen::VectorXd::Zero(n + 1);
    for (int i = 0; i <= n; ++i) {
      qt(i) = points[i](0), qr(i) = points[i](1);
    }
    spline::cubic(qt, qta, qtb, qtc, qtd);
    spline::cubic(qr, qra, qrb, qrc, qrd);

    qt1 = Eigen::VectorXd::Zero(n + 1), qr1 = Eigen::VectorXd::Zero(n + 1),
    qt2 = Eigen::VectorXd::Zero(n + 1), qr2 = Eigen::VectorXd::Zero(n + 1);
    arc = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
      qt1(i) = qtc(i), qr1(i) = qrc(i);
      qt2(i) = 2 * qtb(i), qr2(i) = 2 * qrb(i);
      for (double u = .01; u < 1; u += .01) {
        arc(i) +=
            .01 * sqrt(pow(3 * qta(i) * u * u + 2 * qtb(i) * u + qtc(i), 2) +
                       pow(3 * qra(i) * u * u + 2 * qrb(i) * u + qrc(i), 2));
      }
      arc(i) += .005 * (sqrt(pow(3 * qta(i) + 2 * qtb(i) + qtc(i), 2) +
                             pow(3 * qra(i) + 2 * qrb(i) + qrc(i), 2)) +
                        sqrt(qtc(i) * qtc(i) + qrc(i) * qrc(i)));
    }
    qt1(n) = 3 * qta(n - 1) + 2 * qtb(n - 1) + qtc(n - 1);
    qr1(n) = 3 * qra(n - 1) + 2 * qrb(n - 1) + qrc(n - 1);
    qt2(n) = 6 * qta(n - 1) + 2 * qtb(n - 1);
    qr2(n) = 6 * qra(n - 1) + 2 * qrb(n - 1);

    // initialize lbfgs params
    params.g_epsilon = 1.0e-8;
    params.past = 3;
    params.delta = 1.0e-8;

    // initialize optimization variables
    n_var = 5 * n + 3;
    x = Eigen::VectorXd::Zero(n_var);
    g = Eigen::VectorXd::Zero(n_var);

    /**
     * initialize linear coefficients
     * \sum 2 * (s_{k+1} - s_k) * d_k
     */
    f = Eigen::VectorXd::Zero(n_var);
    for (int i = 0; i < n; ++i) {
      f(3 * n + 2 + i) = 2 * arc(i);
    }

    /**
     * initialize second order cone constraints
     */
    As.clear(), bs.clear();
    n_soc = 2 * n + 1;

    /**
     * ||  2*c_k  ||
     * ||         || <= b_k + 1
     * || b_k - 1 ||
     *
     * || b_k + 1 ||
     * ||  2*c_k  || in Q
     * || b_k - 1 ||
     */
    for (int i = 0; i <= n; ++i) {
      Eigen::MatrixXd A_(3, n_var);
      Eigen::VectorXd b_(3);
      A_(0, n + i) = 1;          // b_k
      A_(1, 2 * n + 1 + i) = 2;  // c_k
      A_(2, n + i) = 1;          // b_k
      b_(0) = 1;
      b_(2) = -1;
      As.push_back(A_);
      bs.push_back(b_);
    }

    /**
     * ||          2          ||
     * ||                     || <= c_{k+1} + c_k + d_k
     * || c_{k+1} + c_k - d_k ||
     *
     * || c_{k+1} + c_k + d_k ||
     * ||          2          || in Q
     * || c_{k+1} + c_k - d_k ||
     */
    for (int i = 0; i < n; ++i) {
      Eigen::MatrixXd A_(3, n_var);
      Eigen::VectorXd b_(3);
      A_(0, 2 * n + 1 + i) = 1;   // c_k
      A_(0, 2 * n + 2 + i) = 1;   // c_{k+1}
      A_(0, 3 * n + 2 + i) = 1;   // d_k
      A_(2, 2 * n + 1 + i) = 1;   // c_k
      A_(2, 2 * n + 2 + i) = 1;   // c_{k+1}
      A_(2, 3 * n + 2 + i) = -1;  // d_k
      b_(1) = 2;
      As.push_back(A_);
      bs.push_back(b_);
    }

    /**
     * initialize linear equality constraints
     */
    n_eq = n + 4;
    G = Eigen::MatrixXd::Zero(n_eq, n_var);
    h = Eigen::VectorXd::Zero(n_eq);

    /**
     * kinematic constraints
     * 2(s_{k+1} - s_k) * a_k + b_k - b_{k+1} = 0
     */
    for (int i = 0; i < n; ++i) {
      G(i, i) = 2 * arc(i);  // 2(s_{k+1} - s_k)
      G(i, n + i) = 1;       // b_k
      G(i, n + i + 1) = -1;  // b_{k+1}
    }

    /**
     * boundary constraints
     * b_0 = 0
     * b_n = 0
     * e_0 = 0
     * e_n = 0
     */
    G(n, n) = 1;              // b_0
    G(n + 1, 2 * n) = 1;      // b_n
    G(n + 2, 4 * n + 2) = 1;  // e_0
    G(n + 3, 5 * n + 2) = 1;  // e_n

    /**
     * initialize linear inequality constraints
     */
    n_ineq = 11 * n + 3;
    P = Eigen::MatrixXd::Zero(n_ineq, n_var);
    q = Eigen::VectorXd::Zero(n_ineq);

    /**
     * always forward
     * -b_k <= 0
     */
    for (int i = 0; i <= n; ++i) {
      P(i, n + i) = -1;  // b_k
    }  // end at n

    /**
     * velocity constraints
     * q'_t(s_k)^2 * b_k <= vt_max^2
     * q'_r(s_k)^2 * b_k <= vr_max^2
     */
    for (int i = 0; i <= n; ++i) {
      P(n + 1 + 2 * i, n + i) = qt1(i) * qt1(i);  // b_k
      P(n + 2 + 2 * i, n + i) = qr1(i) * qr1(i);  // b_k
      q(n + 1 + 2 * i) = alphabot::ELEVATOR_VMAX * alphabot::ELEVATOR_VMAX;
      q(n + 2 + 2 * i) = alphabot::ARM_VMAX * alphabot::ARM_VMAX;
    }  // end at 3n+2

    /**
     * acceleration constraints
     * +q''_t(s_k)^2 * b_k + q'_t(s_k) * a_k <= at_max
     * -q''_t(s_k)^2 * b_k - q'_t(s_k) * a_k <= at_max
     * +q''_r(s_k)^2 * b_k + q'_r(s_k) * a_k <= ar_max
     * -q''_r(s_k)^2 * b_k - q'_r(s_k) * a_k <= ar_max
     */
    for (int i = 0; i < n; ++i) {
      P(3 * n + 3 + 4 * i, n + i) = qt2(i) * qt2(i);   // b_k
      P(3 * n + 4 + 4 * i, n + i) = -qt2(i) * qt2(i);  // b_k
      P(3 * n + 5 + 4 * i, n + i) = qr2(i) * qr2(i);   // b_k
      P(3 * n + 6 + 4 * i, n + i) = -qr2(i) * qr2(i);  // b_k
      P(3 * n + 3 + 4 * i, i) = qt1(i);                // a_k
      P(3 * n + 4 + 4 * i, i) = -qt1(i);               // a_k
      P(3 * n + 5 + 4 * i, i) = qr1(i);                // a_k
      P(3 * n + 6 + 4 * i, i) = -qr1(i);               // a_k
      q(3 * n + 3 + 4 * i) = alphabot::ELEVATOR_AMAX;
      q(3 * n + 4 + 4 * i) = alphabot::ELEVATOR_AMAX;
      q(3 * n + 5 + 4 * i) = alphabot::ARM_AMAX;
      q(3 * n + 6 + 4 * i) = alphabot::ARM_AMAX;
    }  // end at 7n+2

    /**
     * voltage constraints
     * +Kv * q'_t(s_k) * e_k + Ka * q''_t(s_k) * a_k + Ka * q'_t(s_k) * b_k <= V_max
     * -Kv * q'_t(s_k) * e_k - Ka * q''_t(s_k) * a_k - Ka * q'_t(s_k) * b_k <= V_max
     * +Kv * q'_r(s_k) * e_k + Ka * q''_r(s_k) * a_k + Ka * q'_r(s_k) * b_k <= V_max
     * -Kv * q'_r(s_k) * e_k - Ka * q''_r(s_k) * a_k - Ka * q'_r(s_k) * b_k <= V_max
     */
    for (int i = 0; i < n; ++i) {
      P(7 * n + 3 + 4 * i, 4 * n + 2 + i) = alphabot::ELEVATOR_Kv * qt1(i);   // e_k
      P(7 * n + 4 + 4 * i, 4 * n + 2 + i) = -alphabot::ELEVATOR_Kv * qt1(i);  // e_k
      P(7 * n + 5 + 4 * i, 4 * n + 2 + i) = alphabot::ARM_Kv * qr1(i);        // e_k
      P(7 * n + 6 + 4 * i, 4 * n + 2 + i) = -alphabot::ARM_Kv * qr1(i);       // e_k
      P(7 * n + 3 + 4 * i, i) = alphabot::ARM_Ka * qt2(i);                    // a_k
      P(7 * n + 4 + 4 * i, i) = -alphabot::ARM_Ka * qt2(i);                   // a_k
      P(7 * n + 5 + 4 * i, i) = alphabot::ELEVATOR_Ka * qr2(i);               // a_k
      P(7 * n + 6 + 4 * i, i) = -alphabot::ELEVATOR_Ka * qr2(i);              // a_k
      P(7 * n + 3 + 4 * i, n + i) = alphabot::ELEVATOR_Ka * qt1(i);           // b_k
      P(7 * n + 4 + 4 * i, n + i) = -alphabot::ELEVATOR_Ka * qt1(i);          // b_k
      P(7 * n + 5 + 4 * i, n + i) = alphabot::ARM_Ka * qr1(i);                // b_k
      P(7 * n + 6 + 4 * i, n + i) = -alphabot::ARM_Ka * qr1(i);               // b_k
      q(7 * n + 3 + 4 * i) = alphabot::ELEVATOR_MAX_VOLTAGE;
      q(7 * n + 4 + 4 * i) = alphabot::ELEVATOR_MAX_VOLTAGE;
      q(7 * n + 5 + 4 * i) = alphabot::ARM_MAX_VOLTAGE;
      q(7 * n + 6 + 4 * i) = alphabot::ARM_MAX_VOLTAGE;
    }  // end at 11n+2

    /**
     * initialize quadratic equality constraints
     */
    n_quadeq = n + 1;
    Js.clear(), rs.clear();

    /**
     * e_k * e_k = b_k
     */
    for (int i = 0; i <= n; ++i) {
      Eigen::MatrixXd J_ = Eigen::MatrixXd::Zero(n_var, n_var);
      Eigen::VectorXd r_ = Eigen::VectorXd::Zero(n_var);
      J_(4 * n + 2 + i, 4 * n + 2 + i) = 1;  // e_k
      r_(n + i) = 1;                         // b_k
      Js.push_back(J_);
      rs.push_back(r_);
    }

    // initialize dual variables
    mus = std::vector<Eigen::VectorXd>(n_soc, Eigen::VectorXd::Zero(3));
    lambda = Eigen::VectorXd::Zero(n_eq);
    eta = Eigen::VectorXd::Zero(n_ineq);
    rho = 1;

    log_info("TOPP problem set up.");
  }

  /**
   * @deprecated too slow
   */
  // static double loss(void* instance, const Eigen::VectorXd& optX,
  //                    Eigen::VectorXd& optG) {
  //   Topp* self = static_cast<Topp*>(instance);

  //   // x^T * Js * x - rs * x
  //   std::vector<double> resQuadeqs = optX.transpose() * self->Js * optX - transpose(self->rs) * optX;
  //   // Pk(mu / rho - As * x - bs)
  //   std::vector<Eigen::VectorXd> resSocs = socProjections(self->mus / self->rho - self->As * optX - self->bs);
  //   // G * x - h + lambda / rho
  //   Eigen::VectorXd resEq = self->G * optX - self->h + self->lambda / self->rho;
  //   // max[P * x - q + eta / rho, 0]
  //   Eigen::VectorXd resIneq = max(self->P * optX - self->q + self->eta / self->rho, 0);

  //   optG = self->f;
  //   // G^T * (G * x - h + lambda / rho)
  //   optG += self->rho * self->G.transpose() * resEq;
  //   // p^T * max[P * x - q + eta / rho, 0]
  //   optG += self->rho * self->P.transpose() * resIneq;
  //   // As^T * Pk(mu / rho - As * x - bs)
  //   optG += self->rho * sum(transpose(self->As) * resSocs);
  //   // x^T * Js * x * Js * x - Js * x * rs^T - x^T * Js * rs^T + rs^T * x * rs
  //   optG += self->rho *
  //           sum(resQuadeqs * (2 * self->Js * optX - self->rs));

  //   return self->f.dot(optX) + self->rho / 2 * (sum(resQuadeqs) + sum(squaredNorm(resSocs)) + resEq.squaredNorm() + resIneq.squaredNorm());
  // }

  static double loss(void* instance, const Eigen::VectorXd& optX,
                     Eigen::VectorXd& optG) {
    Topp* topp = static_cast<Topp*>(instance);
    double res = topp->f.dot(optX);
    optG = topp->f;

    // linear equality constraints
    Eigen::VectorXd resEq = topp->G * optX - topp->h - topp->lambda / topp->rho;
    res += topp->rho / 2 * resEq.squaredNorm();
    optG += topp->rho * topp->G.transpose() * resEq;

    // linear inequality constraints
    Eigen::VectorXd resIneq = max(topp->P * optX - topp->q + topp->eta / topp->rho, 0);
    res += topp->rho / 2 * resIneq.squaredNorm();
    optG += topp->rho * topp->P.transpose() * resIneq;

    // second order cone constraints
    for (int i = 0; i < topp->n_soc; ++i) {
      Eigen::VectorXd resSoc = socProjection(topp->mus[i] / topp->rho - topp->As[i] * optX - topp->bs[i]);
      res += topp->rho / 2 * resSoc.squaredNorm();
      optG += topp->rho * topp->As[i].transpose() * resSoc;
    }

    // quadratic equality constraints
    for (int i = 0; i < topp->n_quadeq; ++i) {
      double resQuadeq = (optX.transpose() * topp->Js[i]).dot(optX) - topp->rs[i].dot(optX);
      res += topp->rho / 2 * resQuadeq * resQuadeq;
      optG += topp->rho * (2 * topp->Js[i] * optX - topp->rs[i]);
    }

    return res;
  }
};