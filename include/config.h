#ifndef CONFIG_H
#define CONFIG_H

#include <cmath>
#include <string>

namespace config {

constexpr double toRadians(const double degree) {
  return degree / 180 * M_PI;
}

constexpr double toDegrees(const double radian) {
  return radian / M_PI * 180;
}

constexpr double constexprCos(const double x) {
  return x - x * x * x / 6 + x * x * x * x * x / 120 - x * x * x * x * x * x * x / 5040 + x * x * x * x * x * x * x * x * x / 362880 - x * x * x * x * x * x * x * x * x * x * x / 39916800 + x * x * x * x * x * x * x * x * x * x * x * x * x / 6227020800;
}

constexpr double constexprSin(const double x) {
  return 1 - x * x / 2 + x * x * x * x / 24 - x * x * x * x * x * x / 720 + x * x * x * x * x * x * x * x / 40320 - x * x * x * x * x * x * x * x * x * x / 3628800 + x * x * x * x * x * x * x * x * x * x * x * x / 479001600;
}

constexpr int constexprFloor(double x) {
  return (x < 0) ? static_cast<int>(x) - 1 : static_cast<int>(x);
}

namespace dynamic {
constexpr double OBSTACLE_OFFSET = 100.0;
constexpr double OBSTACLE_FIELD_REDUCTION = 0.6;
constexpr double ASTAR_HEURISTIC_COEFFICIENT = 1.0;
}  // namespace dynamic

namespace robot {
using namespace alphabot;
};  // namespace robot

namespace alphabot {
const std::string VERSION = "8214.0.0";

const double ROBOT_WIDTH = 0.7;
const double ROBOT_HEIGHT = 0.2;
const double ROBOT_2_L1_FRONT = 0.05;

const double ELEVATOR_2_L1_FRONT = 0.4;
const double ELEVATOR_2_GROUND = 0.2;
const double ELEVATOR_2_GROUND_ANGLE = 80.0;
constexpr double ELEVATOR_SIN_ANGLE = constexprSin(toRadians(ELEVATOR_2_GROUND_ANGLE));
constexpr double ELEVATOR_COS_ANGLE = constexprCos(toRadians(ELEVATOR_2_GROUND_ANGLE));

const double ELEVATOR_MAX_VOLTAGE = 12;
const double ELEVATOR_Kv = 1;
const double ELEVATOR_Ka = 1;
const double ELEVATOR_MIN_POSITION = 0;
const double ELEVATOR_MAX_POSITION = 2.0;
const double ELEVATOR_MAX_RPS = 2;
const double ELEVATOR_MAX_RPSS = 0.01;
const double ELEVATOR_REDUCTION = 1.0;
const double ELEVATOR_ROTATION_2_POSITION = 1;

const double ARM_MAX_VOLTAGE = 12;
const double ARM_Kv = 1;
const double ARM_Ka = 1;
const double ARM_MIN_THETA_ROTATION = 0;
const double ARM_MAX_THETA_ROTATION = 350;
const double ARM_MAX_RPS = 2;
const double ARM_MAX_RPSS = 0.01;
const double ARM_REDUCTION = 1.0;

constexpr double ARM_MIN_THETA_RADIAN = toRadians(ARM_MIN_THETA_ROTATION);
constexpr double ARM_MAX_THETA_RADIAN = toRadians(ARM_MAX_THETA_ROTATION);
constexpr double ARM_VMAX = ARM_MAX_RPS / ARM_REDUCTION;
constexpr double ARM_AMAX = ARM_MAX_RPSS / ARM_REDUCTION;
constexpr double ELEVATOR_VMAX = ELEVATOR_MAX_RPS / ELEVATOR_REDUCTION * ELEVATOR_ROTATION_2_POSITION;
constexpr double ELEVATOR_AMAX = ELEVATOR_MAX_RPSS / ELEVATOR_REDUCTION * ELEVATOR_ROTATION_2_POSITION;

const double ELEVATOR_GRID_SIZE = 0.05;
const double ARM_GRID_SIZE = 0.1;
constexpr int ELEVATOR_GRID_NUMS = constexprFloor((ELEVATOR_MAX_POSITION - ELEVATOR_MIN_POSITION) / ELEVATOR_GRID_SIZE) + 1;
constexpr int ARM_GRID_NUMS = constexprFloor((ARM_MAX_THETA_ROTATION - ARM_MIN_THETA_ROTATION) / ARM_GRID_SIZE) + 1;
};  // namespace alphabot

namespace env {
const double L1_FRONT_HEIGHT = 0.454;
const double L1_BACK_HEIGHT = 0.514;
const double L1_FRONT_2_L1_BACK = 0.248;
constexpr double L1_BACK_2_REEF_CENTER = 0.832 - L1_FRONT_2_L1_BACK;
const double REEF_CENTER_HEIGHT = 1.2;

const double BRANCH_DIAMETER = 0.042;
const double BRANCH_UL_X = 0.296;
const double BRANCH_UL_Y = 1.35;
const double BRANCH_LR_Y = 0.0;
constexpr double BRANCH_UR_X = BRANCH_UL_X + BRANCH_DIAMETER;
constexpr double BRANCH_UR_Y = BRANCH_UL_Y;
constexpr double BRANCH_LR_X = BRANCH_UR_X;
constexpr double BRANCH_LL_X = BRANCH_UL_X;
constexpr double BRANCH_LL_Y = BRANCH_LR_Y;

const double L4_UL_X = 0.030;
const double L4_UL_Y = 1.810;
constexpr double L4_UR_X = L4_UL_X + BRANCH_DIAMETER;
constexpr double L4_UR_Y = L4_UL_Y;
constexpr double L4_LR_X = L4_UR_X;
constexpr double L4_LR_Y = L4_UR_Y - 0.173;
constexpr double L4_LL_X = L4_UL_X;
constexpr double L4_LL_Y = L4_LR_Y;

const double L3_UL_X = 0.053;
const double L3_UL_Y = 1.194;
const double L3_LL_X = 0.029;
const double L3_LL_Y = 1.16;
const double L3_UR_Y = 1.024;
const double L3_LR_Y = 0.973;
constexpr double L3_UR_X = BRANCH_LL_X;
constexpr double L3_LR_X = BRANCH_LL_X;

const double L2_UL_Y = 0.79;
constexpr double L2_UL_X = L3_UL_X;
constexpr double L2_LL_X = L3_LL_X;
constexpr double L2_LL_Y = L2_UL_Y - (L3_UL_Y - L3_LL_Y);
constexpr double L2_UR_X = L3_UR_X;
constexpr double L2_UR_Y = L2_UL_Y - (L3_UL_Y - L3_UR_Y);
constexpr double L2_LR_X = L3_LR_X;
constexpr double L2_LR_Y = L2_LL_Y - (L3_LL_Y - L3_LR_Y);

const double ALGAE_HIGH_CENTER_X = 0.7;
const double ALGAE_HIGH_CENTER_Y = 1.2;
const double ALGAE_LOW_CENTER_Y = 0.4;
const double ALGAE_RADIUS = 0.41;
constexpr double ALGAE_LOW_CENTER_X = ALGAE_HIGH_CENTER_X;
};  // namespace env

};  // namespace config

#endif  // CONFIG_H
