#ifndef TOPP_HPP
#define TOPP_HPP

#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Sparse>
#include <iostream>
#include <thread>

#include "config.h"
#include "lbfgs.hpp"
#include "log.hpp"
#include "proto/ArmTrajectoryService.grpc.pb.h"
#include "spline.hpp"

#define BETA 1e4
#define GAMMA .5
#define RHO 1e1
#define ARC_LEN 1.

using namespace com::nextinnovation::armtrajectoryservice;
using namespace nextinnovation::alphabot;

namespace nextinnovation {
Eigen::VectorXd max(const Eigen::VectorXd& v, const double num) {
  Eigen::VectorXd res = v;
  for (int i = 0; i < v.rows(); ++i) {
    res(i) = std::max(num, v(i));
  }
  return res;
}

Eigen::VectorXd socProjection(const Eigen::VectorXd& v) {
  if (v.rows() == 1) {
    return v;
  }
  double v0 = v(0);
  Eigen::VectorXd v1 = v.tail(v.rows() - 1);
  if (v0 > v1.norm()) {
    return v;
  } else if (v0 < -v1.norm()) {
    return Eigen::VectorXd::Zero(v.rows());
  } else {
    Eigen::VectorXd Pk = Eigen::VectorXd::Zero(v.rows());
    Pk(0) = v1.norm();
    Pk.tail(v.rows() - 1) = v1;
    Pk *= (v0 + v1.norm()) / 2 / v1.norm();
    return Pk;
  }
}

double sign(double x) {
  if (x > 1e-20) {
    return 1;
  }
  if (x < -1e-20) {
    return -1;
  }
  return 0;
}

class Topp {
 private:
  std::vector<Eigen::Vector2d> points;
  // number of segments
  int n;
  int iter;
  lbfgs::lbfgs_parameter_t params;

  /**
   * spline parameters
   */
  Eigen::VectorXd qt, qr, qt1, qr1, qt2, qr2;
  Eigen::VectorXd qta, qtb, qtc, qtd, qra, qrb, qrc, qrd;

  /**
   * optimization variables
   * a:  0   ~  n-1  (n)
   * b:  n   ~  2n   (n+1)
   * c: 2n+1 ~ 3n+1  (n+1)
   * d: 3n+2 ~ 4n+1  (n)
   */
  Eigen::VectorXd x, g;
  int n_var;

  /**
   * linear coefficients
   */
  Eigen::SparseVector<double> f;

  /**
   * second order cone constraints
   */
  std::vector<Eigen::SparseMatrix<double>> As;
  std::vector<Eigen::SparseVector<double>> bs;
  int n_soc;

  /**
   * linear equality constraints
   */
  Eigen::SparseMatrix<double> G;
  Eigen::SparseVector<double> h;
  int n_eq;

  /**
   * linear inequality constraints
   */
  Eigen::SparseMatrix<double> P;
  Eigen::SparseVector<double> q;
  int n_ineq;

  /**
   * quadratic equality constraints
   */
  std::vector<Eigen::SparseMatrix<double>> Js;
  std::vector<Eigen::SparseVector<double>> rs;
  int n_quadeq;

  /**
   * dual variables
   */
  double rho;
  Eigen::VectorXd lambda, eta;
  std::vector<Eigen::VectorXd> mus;
  std::vector<double> nus;

 public:
  Topp(const std::vector<Eigen::Vector2d>& points,
       int maxIter = 30) {
    if (points.size() < 3) {
      log_error("The number of points should be at least 3.");
      throw std::invalid_argument("The number of points should be at least 3.");
    }
    this->points = points;
    this->n = points.size() - 1;
    this->iter = 0;
    this->setup();
    this->solve(maxIter);
  }

  void solve(int maxIter = 30) {
    log_info("Solving TOPP problem...");

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

      // update dual variables
      for (int i = 0; i < n_soc; ++i) {
        mus[i] = socProjection(mus[i] - rho * (As[i] * x + bs[i]));
      }
      for (int i = 0; i < n_quadeq; ++i) {
        nus[i] = nus[i] + rho * (x.transpose() * Js[i] * x - rs[i].dot(x));
      }
      lambda = lambda + rho * (G * x - h);
      eta = max(eta + rho * (P * x - q), 0);
      rho = std::min(rho * (1 + GAMMA), BETA);

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
    log_info("TOPP problem solved, total duration: %3d,%3dms, iterations: %d, loss: %f", (millis - beginTime) / 1000, (millis - beginTime) % 1000, maxIter, cost);
    return;
  }

  int getIter() { return iter; }
  double getLoss() {
    return loss(this, x, g);
  }
  void getTrajectory(ArmTrajectory* trajectory, std::vector<Eigen::Vector2d>& voltage, std::vector<Eigen::Vector2d>& velocity) {
    Eigen::VectorXd ak = x.segment(getA(0), lenA());
    Eigen::VectorXd bk = x.segment(getB(0), lenB());
    Eigen::VectorXd ck = x.segment(getC(0), lenC());
    Eigen::VectorXd dk = x.segment(getD(0), lenD());
    voltage.clear(), velocity.clear();
    ArmTrajectoryState* state;
    double t = 0;

    log_debug("qt1: %f", qt1(0));
    log_debug("qt2: %f", qt2(0));
    log_debug("qr1: %f", qr1(0));
    log_debug("qr2: %f", qr2(0));
    log_debug("ak1: %f", ak(0));
    log_debug("bk1: %f", bk(0));
    log_debug("ck1: %f", ck(0));

    for (int i = 0; i < n; ++i) {
      state = trajectory->add_states();
      state->set_timestamp(t);
      state->mutable_position()->set_shoulderheightmeter(points[i](0));
      state->mutable_position()->set_elbowpositionradian(points[i](1));
      state->mutable_current()->set_shouldercurrentampere(((-ELEVATOR_METER_2_MOTOR_RADIAN * qt1(i) / ELEVATOR_Kv + ELEVATOR_Kv * qt2(i)) * ck(i) + ELEVATOR_Ka * qt2(i) * ak(i) + ELEVATOR_Ka * qt1(i) * bk(i) + ELEVATOR_Kg + ELEVATOR_Ks * sign(qt1(i))) / ELEVATOR_R);
      state->mutable_current()->set_elbowcurrentampere(((-ARM_RADIAN_2_MOTOR_RADIAN * qr1(i) / ARM_Kv + ARM_Kv * qr2(i)) * ck(i) + ARM_Ka * qr2(i) * ak(i) + ARM_Ka * qr1(i) * bk(i) + ARM_Kg * cos(qr(i)) + ARM_Ks * sign(qr1(i))) / ARM_R);
      voltage.push_back(Eigen::Vector2d(ELEVATOR_Kv * qt1(i) * ck(i) + ELEVATOR_Ka * qt2(i) * ak(i) + ELEVATOR_Ka * qt1(i) * bk(i) + ELEVATOR_Kg + ELEVATOR_Ks * sign(qt1(i)), ARM_Kv * qr1(i) * ck(i) + ARM_Ka * qr2(i) * ak(i) + ARM_Ka * qr1(i) * bk(i) + ARM_Kg * cos(qr(i)) + ARM_Ks * sign(qr1(i))));
      velocity.push_back(Eigen::Vector2d(ELEVATOR_Kv * qt1(i) * ck(i), ARM_Kv * qr1(i) * ck(i)));
      t += ARC_LEN * 2 / (ck(i) + ck(i + 1));
    }

    state = trajectory->add_states();
    state->set_timestamp(t);
    state->mutable_position()->set_shoulderheightmeter(points[n](0));
    state->mutable_position()->set_elbowpositionradian(points[n](1));
    state->mutable_current()->set_shouldercurrentampere(((-ELEVATOR_METER_2_MOTOR_RADIAN * qt1(n) / ELEVATOR_Kv + ELEVATOR_Kv * qt2(n)) * ck(n) + ELEVATOR_Ka * qt1(n) * bk(n) + ELEVATOR_Kg + ELEVATOR_Ks * sign(qt1(n))) / ELEVATOR_R);
    state->mutable_current()->set_elbowcurrentampere(((-ARM_RADIAN_2_MOTOR_RADIAN * qr1(n) / ARM_Kv + ARM_Kv * qr2(n)) * ck(n) + ARM_Ka * qr1(n) * bk(n) + ARM_Kg * cos(qr(n)) + ARM_Ks * sign(qr1(n))) / ARM_R);
    voltage.push_back(Eigen::Vector2d(ELEVATOR_Kv * qt1(n) * ck(n) + ELEVATOR_Ka * qt1(n) * bk(n) + ELEVATOR_Kg + ELEVATOR_Ks * sign(qt1(n)), ARM_Kv * qr1(n) * ck(n) + ARM_Ka * qr1(n) * bk(n) + ARM_Kg * cos(qr(n)) + ARM_Ks * sign(qr1(n))));
    velocity.push_back(Eigen::Vector2d(ELEVATOR_Kv * qt1(n) * ck(n), ARM_Kv * qr1(n) * ck(n)));
  }

 private:
  void setup() {
    log_info("Setting up TOPP problem with %d segments.", n);
    log_debug("Setting up TOPP problem[0/] with %d segments.", n);

    // get spline parameters
    qt = Eigen::VectorXd::Zero(n + 1), qr = Eigen::VectorXd::Zero(n + 1);
    for (int i = 0; i <= n; ++i) {
      qt(i) = points[i](0), qr(i) = points[i](1);
    }
    nextinnovation::cubic(qt, qta, qtb, qtc, qtd);
    nextinnovation::cubic(qr, qra, qrb, qrc, qrd);

    qt1 = Eigen::VectorXd::Zero(n + 1), qr1 = Eigen::VectorXd::Zero(n + 1),
    qt2 = Eigen::VectorXd::Zero(n + 1), qr2 = Eigen::VectorXd::Zero(n + 1);
    for (int i = 0; i < n; ++i) {
      qt1(i) = qtc(i), qr1(i) = qrc(i);
      qt2(i) = 2 * qtb(i), qr2(i) = 2 * qrb(i);
    }
    qt1(n) = 3 * qta(n - 1) + 2 * qtb(n - 1) + qtc(n - 1);
    qr1(n) = 3 * qra(n - 1) + 2 * qrb(n - 1) + qrc(n - 1);
    qt2(n) = 6 * qta(n - 1) + 2 * qtb(n - 1);
    qr2(n) = 6 * qra(n - 1) + 2 * qrb(n - 1);
    log_debug("Setting up TOPP problem[1/] spline parameters generated.");

    // initialize lbfgs params
    params.g_epsilon = 1.0e-8;
    params.past = 3;
    params.delta = 1.0e-8;
    log_debug("Setting up TOPP problem[2/] lbfgs parameters generated.");

    // initialize optimization variables
    n_var = 4 * n + 2;
    // x = Eigen::VectorXd::Zero(n_var);
    x = Eigen::VectorXd::Random(n_var);
    g = Eigen::VectorXd::Zero(n_var);
    log_debug("Setting up TOPP problem[3/] optimization variables initialized.");

    /**
     * initialize linear coefficients
     * \sum 2 * (s_{k+1} - s_k) * d_k
     */
    f = Eigen::SparseVector<double>(n_var);
    for (int i = 0; i < n; ++i) {
      f.insert(getD(i)) = 2 * ARC_LEN;
    }
    log_debug("Setting up TOPP problem[4/] linear coefficients initialized.");

    /**
     * initialize second order cone constraints
     */
    As.clear(), bs.clear();
    n_soc = n;
    int n_soc_cnt = 0;

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
      Eigen::SparseMatrix<double> A_(3, n_var);
      Eigen::SparseVector<double> b_(3);
      A_.insert(0, getC(i)) = 1;      // c_k
      A_.insert(0, getC(i + 1)) = 1;  // c_{k+1}
      A_.insert(0, getD(i)) = 1;      // d_k
      A_.insert(2, getC(i)) = 1;      // c_k
      A_.insert(2, getC(i + 1)) = 1;  // c_{k+1}
      A_.insert(2, getD(i)) = -1;     // d_k
      b_.insert(1) = 2;
      As.push_back(A_);
      bs.push_back(b_);
      n_soc_cnt += 1;
    }
    log_debug("Setting up TOPP problem[5/] second order cone constraints initialized, %d/%d.", n_soc_cnt, n_soc);

    /**
     * initialize linear equality constraints
     */
    n_eq = n + 4;
    int n_eq_cnt = 0;
    G = Eigen::SparseMatrix<double>(n_eq, n_var);
    h = Eigen::SparseVector<double>(n_eq);

    /**
     * kinematic constraints
     * 2(s_{k+1} - s_k) * a_k + b_k - b_{k+1} = 0
     */
    for (int i = 0; i < n; ++i) {
      G.insert(n_eq_cnt, getA(i)) = 2 * ARC_LEN;
      G.insert(n_eq_cnt, getB(i)) = 1;
      G.insert(n_eq_cnt, getB(i + 1)) = -1;
      ++n_eq_cnt;
    }

    /**
     * boundary constraints
     * b_0 = 0
     * b_n = 0
     * c_0 = 0
     * c_n = 0
     */
    G.insert(n_eq_cnt, getB(0)) = 1;
    ++n_eq_cnt;

    G.insert(n_eq_cnt, getB(lenB() - 1)) = 1;
    ++n_eq_cnt;

    G.insert(n_eq_cnt, getC(0)) = 1;
    n_eq_cnt++;

    G.insert(n_eq_cnt, getC(lenC() - 1)) = 1;
    n_eq_cnt++;

    log_debug("Setting up TOPP problem[6/] linear equality constraints initialized, %d/%d.", n_eq_cnt, n_eq);

    /**
     * initialize quadratic equality constraints
     */
    n_quadeq = n + 1;
    int n_quadeq_cnt = 0;
    Js.clear(), rs.clear();

    /**
     * c_k * c_k = b_k
     */
    for (int i = 0; i <= n; ++i) {
      Eigen::SparseMatrix<double> J_(n_var, n_var);
      Eigen::SparseVector<double> r_(n_var);
      J_.insert(getC(i), getC(i)) = 1;
      r_.insert(getB(i)) = 1;
      Js.push_back(J_);
      rs.push_back(r_);
      n_quadeq_cnt += 1;
    }

    log_debug("Setting up TOPP problem[7/] quadratic equality constraints initialized, %d/%d.", n_quadeq_cnt, n_quadeq);

    /**
     * initialize linear inequality constraints
     */
    n_ineq = 10 * n + 2;
    int n_ineq_cnt = 0;
    P = Eigen::SparseMatrix<double>(n_ineq, n_var);
    q = Eigen::SparseVector<double>(n_ineq);

    /**
     * always forward
     * -b_k <= 0
     * -c_k <= 0
     */
    for (int i = 0; i <= n; ++i) {
      P.insert(n_ineq_cnt, getB(i)) = -1;
      ++n_ineq_cnt;

      P.insert(n_ineq_cnt, getC(i)) = -1;
      ++n_ineq_cnt;
    }

    /**
     * voltage constraints
     */
    for (int i = 0; i < n; ++i) {
      P.insert(n_ineq_cnt, getC(i)) = ELEVATOR_Kv * qt1(i);
      P.insert(n_ineq_cnt, getA(i)) = ELEVATOR_Ka * qt2(i);
      P.insert(n_ineq_cnt, getB(i)) = ELEVATOR_Ka * qt1(i);
      q.insert(n_ineq_cnt) = ELEVATOR_V_MAX - ELEVATOR_Kg - ELEVATOR_Ks * sign(qt1(i));
      ++n_ineq_cnt;

      P.insert(n_ineq_cnt, getC(i)) = -ELEVATOR_Kv * qt1(i);
      P.insert(n_ineq_cnt, getA(i)) = -ELEVATOR_Ka * qt2(i);
      P.insert(n_ineq_cnt, getB(i)) = -ELEVATOR_Ka * qt1(i);
      q.insert(n_ineq_cnt) = ELEVATOR_V_MAX + ELEVATOR_Kg + ELEVATOR_Ks * sign(qt1(i));
      ++n_ineq_cnt;

      P.insert(n_ineq_cnt, getC(i)) = ARM_Kv * qr1(i);
      P.insert(n_ineq_cnt, getA(i)) = ARM_Ka * qr2(i);
      P.insert(n_ineq_cnt, getB(i)) = ARM_Ka * qr1(i);
      q.insert(n_ineq_cnt) = ARM_V_MAX - ARM_Kg * cos(qr(i)) - ARM_Ks * sign(qr1(i));
      ++n_ineq_cnt;

      P.insert(n_ineq_cnt, getC(i)) = -ARM_Kv * qr1(i);
      P.insert(n_ineq_cnt, getA(i)) = -ARM_Ka * qr2(i);
      P.insert(n_ineq_cnt, getB(i)) = -ARM_Ka * qr1(i);
      q.insert(n_ineq_cnt) = ARM_V_MAX + ARM_Kg * cos(qr(i)) + ARM_Ks * sign(qr1(i));
      ++n_ineq_cnt;
    }

    /**
     * current constraints
     */
    for (int i = 0; i < n; ++i) {
      P.insert(n_ineq_cnt, getC(i)) = -ELEVATOR_METER_2_MOTOR_RADIAN * ELEVATOR_R * qt1(i) / ELEVATOR_Kv + ELEVATOR_Kv * qt1(i);
      P.insert(n_ineq_cnt, getA(i)) = ELEVATOR_Ka * qt2(i);
      P.insert(n_ineq_cnt, getB(i)) = ELEVATOR_Ka * qt1(i);
      q.insert(n_ineq_cnt) = ELEVATOR_I_MAX * ELEVATOR_R - ELEVATOR_Kg - ELEVATOR_Ks * sign(qt1(i));
      ++n_ineq_cnt;

      P.insert(n_ineq_cnt, getC(i)) = ELEVATOR_METER_2_MOTOR_RADIAN * ELEVATOR_R * qt1(i) / ELEVATOR_Kv - ELEVATOR_Kv * qt1(i);
      P.insert(n_ineq_cnt, getA(i)) = -ELEVATOR_Ka * qt2(i);
      P.insert(n_ineq_cnt, getB(i)) = -ELEVATOR_Ka * qt1(i);
      q.insert(n_ineq_cnt) = ELEVATOR_I_MAX * ELEVATOR_R + ELEVATOR_Kg + ELEVATOR_Ks * sign(qt1(i));
      ++n_ineq_cnt;

      P.insert(n_ineq_cnt, getC(i)) = -ARM_RADIAN_2_MOTOR_RADIAN * ARM_R * qr1(i) / ARM_Kv + ARM_Kv * qr1(i);
      P.insert(n_ineq_cnt, getA(i)) = ARM_Ka * qr2(i);
      P.insert(n_ineq_cnt, getB(i)) = ARM_Ka * qr1(i);
      q.insert(n_ineq_cnt) = ARM_I_MAX * ARM_R - ARM_Kg * cos(qr(i)) - ARM_Ks * sign(qr1(i));
      ++n_ineq_cnt;

      P.insert(n_ineq_cnt, getC(i)) = ARM_RADIAN_2_MOTOR_RADIAN * ARM_R * qr1(i) / ARM_Kv - ARM_Kv * qr1(i);
      P.insert(n_ineq_cnt, getA(i)) = -ARM_Ka * qr2(i);
      P.insert(n_ineq_cnt, getB(i)) = -ARM_Ka * qr1(i);
      q.insert(n_ineq_cnt) = ARM_I_MAX * ARM_R + ARM_Kg * cos(qr(i)) + ARM_Ks * sign(qr1(i));
      ++n_ineq_cnt;
    }

    log_debug("Setting up TOPP problem[8/] linear inequality constraints initialized, %d/%d.", n_ineq_cnt, n_ineq);

    // initialize dual variables
    mus = std::vector<Eigen::VectorXd>(n_soc, Eigen::VectorXd::Zero(3));
    nus = std::vector<double>(n_quadeq, 0);
    lambda = Eigen::VectorXd::Zero(n_eq);
    eta = Eigen::VectorXd::Zero(n_ineq);
    rho = RHO;
    log_debug("Setting up TOPP problem[9/] dual variables initialized.");
  }

  static double loss(void* instance, const Eigen::VectorXd& optX,
                     Eigen::VectorXd& optG) {
    Topp* topp = static_cast<Topp*>(instance);
    double res = topp->f.dot(optX);
    optG = topp->f;

    // linear equality constraints
    Eigen::VectorXd resEq = topp->G * optX - topp->h + topp->lambda / topp->rho;
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
      optG -= topp->rho * topp->As[i].transpose() * resSoc;
    }

    // quadratic equality constraints
    for (int i = 0; i < topp->n_quadeq; ++i) {
      double resQuadeq = (optX.transpose() * topp->Js[i]).dot(optX) - topp->rs[i].dot(optX) + topp->nus[i] / topp->rho;
      res += topp->rho / 2 * resQuadeq * resQuadeq;
      optG += topp->rho * resQuadeq * (2 * topp->Js[i] * optX - topp->rs[i]);
    }

    return res;
  }

  inline int lenA() { return n; }
  inline int lenB() { return n + 1; }
  inline int lenC() { return n + 1; }
  inline int lenD() { return n; }

  inline int getA(int i) {
    assert(i >= 0 && i < lenA());
    return i;
  }
  inline int getB(int i) {
    assert(i >= 0 && i < lenB());
    return n + i;
  }
  inline int getC(int i) {
    assert(i >= 0 && i < lenC());
    return 2 * n + 1 + i;
  }
  inline int getD(int i) {
    assert(i >= 0 && i < lenD());
    return 3 * n + 2 + i;
  }
};
};  // namespace nextinnovation

#endif