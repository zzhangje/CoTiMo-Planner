#include <Eigen/Eigen>
#include <iomanip>
#include <iostream>

#include "Object.hpp"
#include "config.h"
#include "lbfgs.hpp"
#include "map.hpp"
#include "spline.hpp"

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

int main() {
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(10, 0, 9);
  Eigen::VectorXd a, b, c, d;
  spline::cubic(x, a, b, c, d);
  std::cout << a.transpose() << std::endl;
  std::cout << b.transpose() << std::endl;
  std::cout << c.transpose() << std::endl;
  std::cout << d.transpose() << std::endl;
  return 0;
}