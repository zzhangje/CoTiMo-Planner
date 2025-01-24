#include <iomanip>
#include <iostream>

#include "Object.hpp"
#include "config.h"
#include "lbfgs.hpp"
#include "map.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace config::alphabot;

// int main() {
//   auto env = Object(ObjectType::ENV);
//   auto c_arm = Object(ObjectType::ARM, 1, 2);
//   auto e_arm = Object(ObjectType::ARM_EXP, 1, 2);

//   auto env_segments = env.getSegments();
//   auto c_arm_segments = c_arm.getSegments();
//   auto e_arm_segments = e_arm.getSegments();
//   for (int i = 0; i < env_segments.size(); i++) {
//     plt::plot({env_segments[i].getPts1X(), env_segments[i].getPts2X()}, {env_segments[i].getPts1Y(), env_segments[i].getPts2Y()}, "g");
//   }
//   for (int i = 0; i < c_arm_segments.size(); i++) {
//     plt::plot({c_arm_segments[i].getPts1X(), c_arm_segments[i].getPts2X()}, {c_arm_segments[i].getPts1Y(), c_arm_segments[i].getPts2Y()}, "b");
//   }
//   for (int i = 0; i < e_arm_segments.size(); i++) {
//     plt::plot({e_arm_segments[i].getPts1X(), e_arm_segments[i].getPts2X()}, {e_arm_segments[i].getPts1Y(), e_arm_segments[i].getPts2Y()}, "r:");
//   }
//   plt::plot({-ELEVATOR_2_L1_FRONT + ELEVATOR_MIN_POSITION * ELEVATOR_COS_ANGLE, -ELEVATOR_2_L1_FRONT + ELEVATOR_MAX_POSITION * ELEVATOR_COS_ANGLE}, {ELEVATOR_2_GROUND + ELEVATOR_MIN_POSITION * ELEVATOR_SIN_ANGLE, ELEVATOR_2_GROUND + ELEVATOR_MAX_POSITION * ELEVATOR_SIN_ANGLE}, "k:");
//   plt::axis("equal");
//   plt::xlim(-ROBOT_2_L1_FRONT - ROBOT_WIDTH, L1_FRONT_2_L1_BACK + L1_BACK_2_REEF_CENTER);
//   plt::ylim(0., L4_UL_Y + .5);
//   // plt::subplot(2, 1, 1);
//   // plt::title("Environment and Arms");
//   // plt::subplot(2, 1, 2);
//   // plt::title("Elevator Path");
//   plt::show();
//   return 0;
// }
class Example {
 public:
  void run() {
    double cost;
    lbfgs::lbfgs_parameter_t params;
    params.g_epsilon = 1.0e-8;
    params.past = 3;
    params.delta = 1.0e-8;
    Eigen::VectorXd x = Eigen::VectorXd::Random(4);
    std::cout << x.transpose() << std::endl;
    for (int i = 0; i < 20; ++i) {
      std::cout << lbfgs::lbfgs_optimize(x, cost, loss, NULL, monitorProgress, this, params) << std::endl;
    }
    std::cout << x.transpose() << std::endl;
    std::cout << "Cost: " << cost << std::endl;
  }
  static double loss(void* instance, const Eigen::VectorXd& x,
                     Eigen::VectorXd& g) {
    Eigen::MatrixXd G, H;
    Eigen::VectorXd f, c;
    G = Eigen::MatrixXd::Zero(4, 4);
    H = Eigen::MatrixXd::Zero(4, 4);
    f = Eigen::VectorXd::Zero(4);
    c = Eigen::VectorXd::Zero(4);
    G(0, 0) = 1;
    H(1, 1) = 1;
    f(2) = 1;
    c(3) = 1;
    g = 2 * ((x.transpose() * G).dot(x) + f.dot(x)) * (G * x - f) + 2 * ((x.transpose() * H * x) + c.dot(x)) * (H * x - c);
    // std::cout << "Gradient: " << g.transpose() << std::endl;
    return (x.transpose() * G * x - f.transpose() * x).squaredNorm() + (x.transpose() * H * x - c.transpose() * x).squaredNorm();
  }
  static int monitorProgress(void* instance,
                             const Eigen::VectorXd& x,
                             const Eigen::VectorXd& g,
                             const double fx,
                             const double step,
                             const int k,
                             const int ls) {
    std::cout << std::setprecision(4)
              << "================================" << std::endl
              << "Iteration: " << k << std::endl
              << "Function Value: " << fx << std::endl
              << "Gradient Inf Norm: " << g.cwiseAbs().maxCoeff() << std::endl
              << "Variables: " << std::endl
              << x.transpose() << std::endl;
    return 0;
  }
};

int main() {
  Example example;
  example.run();
  return 0;
}