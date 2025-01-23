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

#define BETA 1e3
#define GAMMA .1

using com::nextinnovation::armtrajectoryservice::ArmCurrentState;
using com::nextinnovation::armtrajectoryservice::ArmPositionState;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryState;
using namespace config::alphabot;

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

    // auto now = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto lastMillis = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch())
                          .count();
    auto lastMicros = std::chrono::duration_cast<std::chrono::microseconds>(
                          now.time_since_epoch())
                          .count();
    auto beginTime = lastMicros;

    for (int iter = 0; iter < maxIter; ++iter) {
      this->iter++;
      lbfgs::lbfgs_optimize(x, cost, loss, NULL, NULL, this, params);

      // now = std::chrono::high_resolution_clock::now();
      now = std::chrono::steady_clock::now();
      auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch())
                        .count();
      auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                        now.time_since_epoch())
                        .count();

      // update dual variables
      for (int i = 0; i < n_soc; ++i) {
        mus[i] = socProjection(mus[i] / rho - As[i] * x - bs[i]);
      }
      lambda = lambda + rho * (G * x - h);
      eta = max(eta + rho * (P * x - q), 0);
      rho = std::min(rho * (1 + GAMMA), BETA);

      log_debug("iter: %2d, duration: %3d.%3dms, loss: %f", this->iter, (millis - lastMillis) % 1000, (micros - lastMicros) % 1000, cost);
      lastMillis = millis;
      lastMicros = micros;
    }

    log_info("TOPP problem solved, total duration: %3d.%3dms.", (lastMicros - beginTime) / 1000, (lastMicros - beginTime) % 1000);
  }

  int getIter() { return iter; }
  double getLoss() {
    return loss(this, x, g);
  }
  void getStates(std::vector<ArmTrajectoryState>& states) {
    Eigen::VectorXd ak = x.segment(getA(0), lenA());
    Eigen::VectorXd bk = x.segment(getB(0), lenB());
    Eigen::VectorXd ck = x.segment(getC(0), lenC());
    Eigen::VectorXd dk = x.segment(getD(0), lenD());
    Eigen::VectorXd ek = x.segment(getE(0), lenE());

    states.clear();
    ArmTrajectoryState state;
    ArmPositionState position;
    ArmCurrentState current;

    position.set_shoulderheightmeter(points[0].x());
    position.set_elbowpositiondegree(points[0].y());
    current.set_shouldercurrentamp(ELEVATOR_Kv * qt1(0) * ek(0) + ELEVATOR_Ka * (qt2(0) * ak(0) + qt1(0) * bk(0)));
    current.set_elbowcurrentamp(ARM_Kv * qr1(0) * ek(0) + ARM_Ka * (qr2(0) * ak(0) + qr1(0) * bk(0)));
    state.set_timestamp(0);
    state.set_allocated_position(&position);
    state.set_allocated_current(&current);
    states.push_back(state);

    for (int i = 1; i < n; ++i) {
      position.set_shoulderheightmeter(points[i].x());
      position.set_elbowpositiondegree(points[i].y());
      current.set_shouldercurrentamp(ELEVATOR_Kv * qt1(i) * ek(i) + ELEVATOR_Ka * (qt2(i) * ak(i) + qt1(i) * bk(i)));
      current.set_elbowcurrentamp(ARM_Kv * qr1(i) * ek(i) + ARM_Ka * (qr2(i) * ak(i) + qr1(i) * bk(i)));
      state.set_timestamp(arc(i - 1) * 2 / (ek(i) + ek(i - 1)));
      state.set_allocated_position(&position);
      state.set_allocated_current(&current);
      states.push_back(state);
    }

    position.set_shoulderheightmeter(points[n].x());
    position.set_elbowpositiondegree(points[n].y());
    current.set_shouldercurrentamp(ELEVATOR_Kv * qt1(n) * ek(n) + ELEVATOR_Ka * qt1(n) * bk(n));
    current.set_elbowcurrentamp(ARM_Kv * qr1(n) * ek(n) + ARM_Ka * qr1(n) * bk(n));
    state.set_timestamp(arc(n - 1) * 2 / (ek(n) + ek(n - 1)));
    state.set_allocated_position(&position);
    state.set_allocated_current(&current);
    states.push_back(state);
  }

 private:
  void setup() {
    log_info("Setting up TOPP problem[0/] with %d segments.", n);

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
    log_debug("Setting up TOPP problem[1/] spline parameters generated.");

    // initialize lbfgs params
    params.g_epsilon = 1.0e-8;
    params.past = 3;
    params.delta = 1.0e-8;
    log_debug("Setting up TOPP problem[2/] lbfgs parameters generated.");

    // initialize optimization variables
    n_var = 5 * n + 3;
    x = Eigen::VectorXd::Zero(n_var);
    g = Eigen::VectorXd::Zero(n_var);
    log_debug("Setting up TOPP problem[3/] optimization variables initialized.");

    /**
     * initialize linear coefficients
     * \sum 2 * (s_{k+1} - s_k) * d_k
     */
    f = Eigen::SparseVector<double>(n_var);
    for (int i = 0; i < n; ++i) {
      f.insert(getD(i)) = 2 * arc(i);
    }
    log_debug("Setting up TOPP problem[4/] linear coefficients initialized.");

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
      Eigen::SparseMatrix<double> A_(3, n_var);
      Eigen::SparseVector<double> b_(3);
      A_.insert(0, getB(i)) = 1;  // b_k
      A_.insert(1, getC(i)) = 2;  // c_k
      A_.insert(2, getB(i)) = 1;  // b_k
      b_.insert(0) = 1;
      b_.insert(2) = -1;
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
    }
    log_debug("Setting up TOPP problem[5/] second order cone constraints initialized.");

    /**
     * initialize linear equality constraints
     */
    n_eq = n + 4;
    G = Eigen::SparseMatrix<double>(n_eq, n_var);
    h = Eigen::SparseVector<double>(n_eq);

    /**
     * kinematic constraints
     * 2(s_{k+1} - s_k) * a_k + b_k - b_{k+1} = 0
     */
    for (int i = 0; i < n; ++i) {
      G.insert(i, getA(i)) = 2 * arc(i);  // 2(s_{k+1} - s_k)
      G.insert(i, getB(i)) = 1;           // b_k
      G.insert(i, getB(i + 1)) = -1;      // b_{k+1}
    }

    /**
     * boundary constraints
     * b_0 = 0
     * b_n = 0
     * e_0 = 0
     * e_n = 0
     */
    G.insert(n, getB(0)) = 1;               // b_0
    G.insert(n + 1, getB(lenB() - 1)) = 1;  // b_n
    G.insert(n + 2, getE(0)) = 1;           // e_0
    G.insert(n + 3, getE(lenB() - 1)) = 1;  // e_n

    log_debug("Setting up TOPP problem[6/] linear equality constraints initialized.");

    /**
     * initialize linear inequality constraints
     */
    n_ineq = 11 * n + 3;
    P = Eigen::SparseMatrix<double>(n_ineq, n_var);
    q = Eigen::SparseVector<double>(n_ineq);

    /**
     * always forward
     * -b_k <= 0
     */
    for (int i = 0; i <= n; ++i) {
      P.insert(i, getB(i)) = -1;  // b_k
    }  // end at n

    /**
     * velocity constraints
     * q'_t(s_k)^2 * b_k <= vt_max^2
     * q'_r(s_k)^2 * b_k <= vr_max^2
     */
    for (int i = 0; i <= n; ++i) {
      P.insert(n + 1 + 2 * i, getB(i)) = qt1(i) * qt1(i);  // b_k
      P.insert(n + 2 + 2 * i, getB(i)) = qr1(i) * qr1(i);  // b_k
      q.insert(n + 1 + 2 * i) = ELEVATOR_VMAX * ELEVATOR_VMAX;
      q.insert(n + 2 + 2 * i) = ARM_VMAX * ARM_VMAX;
    }  // end at 3n+2

    /**
     * acceleration constraints
     * +q''_t(s_k)^2 * b_k + q'_t(s_k) * a_k <= at_max
     * -q''_t(s_k)^2 * b_k - q'_t(s_k) * a_k <= at_max
     * +q''_r(s_k)^2 * b_k + q'_r(s_k) * a_k <= ar_max
     * -q''_r(s_k)^2 * b_k - q'_r(s_k) * a_k <= ar_max
     */
    for (int i = 0; i < n; ++i) {
      P.insert(3 * n + 3 + 4 * i, getB(i)) = qt2(i) * qt2(i);   // b_k
      P.insert(3 * n + 4 + 4 * i, getB(i)) = -qt2(i) * qt2(i);  // b_k
      P.insert(3 * n + 5 + 4 * i, getB(i)) = qr2(i) * qr2(i);   // b_k
      P.insert(3 * n + 6 + 4 * i, getB(i)) = -qr2(i) * qr2(i);  // b_k
      P.insert(3 * n + 3 + 4 * i, getA(i)) = qt1(i);            // a_k
      P.insert(3 * n + 4 + 4 * i, getA(i)) = -qt1(i);           // a_k
      P.insert(3 * n + 5 + 4 * i, getA(i)) = qr1(i);            // a_k
      P.insert(3 * n + 6 + 4 * i, getA(i)) = -qr1(i);           // a_k
      q.insert(3 * n + 3 + 4 * i) = ELEVATOR_AMAX;
      q.insert(3 * n + 4 + 4 * i) = ELEVATOR_AMAX;
      q.insert(3 * n + 5 + 4 * i) = ARM_AMAX;
      q.insert(3 * n + 6 + 4 * i) = ARM_AMAX;
    }  // end at 7n+2

    /**
     * voltage constraints
     * +Kv * q'_t(s_k) * e_k + Ka * q''_t(s_k) * a_k + Ka * q'_t(s_k) * b_k <= V_max
     * -Kv * q'_t(s_k) * e_k - Ka * q''_t(s_k) * a_k - Ka * q'_t(s_k) * b_k <= V_max
     * +Kv * q'_r(s_k) * e_k + Ka * q''_r(s_k) * a_k + Ka * q'_r(s_k) * b_k <= V_max
     * -Kv * q'_r(s_k) * e_k - Ka * q''_r(s_k) * a_k - Ka * q'_r(s_k) * b_k <= V_max
     */
    for (int i = 0; i < n; ++i) {
      P.insert(7 * n + 3 + 4 * i, getE(i)) = ELEVATOR_Kv * qt1(i);   // e_k
      P.insert(7 * n + 4 + 4 * i, getE(i)) = -ELEVATOR_Kv * qt1(i);  // e_k
      P.insert(7 * n + 5 + 4 * i, getE(i)) = ARM_Kv * qr1(i);        // e_k
      P.insert(7 * n + 6 + 4 * i, getE(i)) = -ARM_Kv * qr1(i);       // e_k
      P.insert(7 * n + 3 + 4 * i, getA(i)) = ARM_Ka * qt2(i);        // a_k
      P.insert(7 * n + 4 + 4 * i, getA(i)) = -ARM_Ka * qt2(i);       // a_k
      P.insert(7 * n + 5 + 4 * i, getA(i)) = ELEVATOR_Ka * qr2(i);   // a_k
      P.insert(7 * n + 6 + 4 * i, getA(i)) = -ELEVATOR_Ka * qr2(i);  // a_k
      P.insert(7 * n + 3 + 4 * i, getB(i)) = ELEVATOR_Ka * qt1(i);   // b_k
      P.insert(7 * n + 4 + 4 * i, getB(i)) = -ELEVATOR_Ka * qt1(i);  // b_k
      P.insert(7 * n + 5 + 4 * i, getB(i)) = ARM_Ka * qr1(i);        // b_k
      P.insert(7 * n + 6 + 4 * i, getB(i)) = -ARM_Ka * qr1(i);       // b_k
      q.insert(7 * n + 3 + 4 * i) = ELEVATOR_MAX_VOLTAGE;
      q.insert(7 * n + 4 + 4 * i) = ELEVATOR_MAX_VOLTAGE;
      q.insert(7 * n + 5 + 4 * i) = ARM_MAX_VOLTAGE;
      q.insert(7 * n + 6 + 4 * i) = ARM_MAX_VOLTAGE;
    }  // end at 11n+2

    log_debug("Setting up TOPP problem[7/] linear inequality constraints initialized.");

    /**
     * initialize quadratic equality constraints
     */
    n_quadeq = n + 1;
    Js.clear(), rs.clear();

    /**
     * e_k * e_k = b_k
     */
    for (int i = 0; i <= n; ++i) {
      Eigen::SparseMatrix<double> J_(n_var, n_var);
      Eigen::SparseVector<double> r_(n_var);
      J_.insert(getE(i), getE(i)) = 1;  // e_k
      r_.insert(getB(i)) = 1;           // b_k
      Js.push_back(J_);
      rs.push_back(r_);
    }
    log_debug("Setting up TOPP problem[8/] quadratic equality constraints initialized.");

    // initialize dual variables
    mus = std::vector<Eigen::VectorXd>(n_soc, Eigen::VectorXd::Zero(3));
    lambda = Eigen::VectorXd::Zero(n_eq);
    eta = Eigen::VectorXd::Zero(n_ineq);
    rho = 1;
    log_debug("Setting up TOPP problem[9/] dual variables initialized.");

    log_info("TOPP problem set up.");
  }

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

  inline int lenA() { return n; }
  inline int lenB() { return n + 1; }
  inline int lenC() { return n + 1; }
  inline int lenD() { return n; }
  inline int lenE() { return n + 1; }

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
  inline int getE(int i) {
    assert(i >= 0 && i < lenE());
    return 4 * n + 2 + i;
  }
};