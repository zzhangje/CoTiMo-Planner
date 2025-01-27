#ifndef CONFIG_H
#define CONFIG_H

#include <cmath>
#include <string>

namespace config {

constexpr double toRadians(double degree) { return degree / 180 * M_PI; }

constexpr double toDegrees(double radian) { return radian / M_PI * 180; }

constexpr double constexprCos(double x) {
  return 1 - x * x / 2 + x * x * x * x / 24 - x * x * x * x * x * x / 720 +
         x * x * x * x * x * x * x * x / 40320 -
         x * x * x * x * x * x * x * x * x * x / 3628800 +
         x * x * x * x * x * x * x * x * x * x * x * x / 479001600;
}

constexpr double constexprSin(double x) {
  return x - x * x * x / 6 + x * x * x * x * x / 120 -
         x * x * x * x * x * x * x / 5040 +
         x * x * x * x * x * x * x * x * x / 362880 -
         x * x * x * x * x * x * x * x * x * x * x / 39916800 +
         x * x * x * x * x * x * x * x * x * x * x * x * x / 6227020800;
}

constexpr int constexprFloor(double x) {
  return (x < 0) ? static_cast<int>(x) - 1 : static_cast<int>(x);
}

namespace params {
const std::string GRPC_PORT = "58214";
const bool IS_DEBUG = true;
}  // namespace params

namespace alphabot {
const std::string ROBOT_PARAMS_VERSION =
    "8214.0.0";  // for cache version control

// the prohitbit area of the drivetrain
constexpr double ROBOT_WIDTH = 0.7;  // the width of the prohitbit area
constexpr double ROBOT_HEIGHT =
    0.2;  // the height of the prohitbit area to the ground
constexpr double ROBOT_2_L1_FRONT =
    0.05;  // the distance from the prohitbit area to the front of the reef

// the properties of the elevator
constexpr double ELEVATOR_2_L1_FRONT =
    0.4;  // the distance from the elevator to the front of the reef
constexpr double ELEVATOR_2_GROUND =
    0.2;  // the distance from the elevator to the ground
constexpr double ELEVATOR_2_GROUND_ANGLE =
    80.0;  // the angle from the elevator to the ground
constexpr double ELEVATOR_SIN_ANGLE =
    constexprSin(toRadians(ELEVATOR_2_GROUND_ANGLE));
constexpr double ELEVATOR_COS_ANGLE =
    constexprCos(toRadians(ELEVATOR_2_GROUND_ANGLE));
constexpr double ELEVATOR_MIN_POSITION_METER = 0;
constexpr double ELEVATOR_MAX_POSITION_METER = 2.0;
constexpr double ARM_MIN_THETA_DEGREE = 0;
constexpr double ARM_MAX_THETA_DEGREE = 350;

// the properties of the elevator motor
// constexpr double ELEVATOR_Kv = 50.8;
constexpr double ELEVATOR_Kv = .001;
constexpr double ELEVATOR_Ka = .001;
constexpr double ELEVATOR_MAX_RPS = 0x3f3f3f3f;
constexpr double ELEVATOR_MAX_RPSS = 0x3f3f3f3f;
constexpr double ELEVATOR_REDUCTION = 1.0;
constexpr double ELEVATOR_ROUNDS_2_POSITION = 1.0;

// the properties of the arm motor
// constexpr double ARM_Kv = 50.8;
constexpr double ARM_Kv = .001;
constexpr double ARM_Ka = .001;
constexpr double ARM_MAX_RPS = 0x3f3f3f3f;
constexpr double ARM_MAX_RPSS = 0x3f3f3f3f;
constexpr double ARM_REDUCTION = 1.0;
constexpr double ARM_ROUNDS_2_DEGREE = 360.0;

// the limit of the motor
constexpr double ELEVATOR_MAX_VOLTAGE = 12;
constexpr double ELEVATOR_VMAX =
    ELEVATOR_MAX_RPS / ELEVATOR_REDUCTION * ELEVATOR_ROUNDS_2_POSITION;
constexpr double ELEVATOR_AMAX =
    ELEVATOR_MAX_RPSS / ELEVATOR_REDUCTION * ELEVATOR_ROUNDS_2_POSITION;
constexpr double ARM_MAX_VOLTAGE = 12;
constexpr double ARM_VMAX = ARM_MAX_RPS / ARM_REDUCTION * ARM_ROUNDS_2_DEGREE;
constexpr double ARM_AMAX = ARM_MAX_RPSS / ARM_REDUCTION * ARM_ROUNDS_2_DEGREE;

// the properties of the grid map
constexpr double ELEVATOR_GRID_SIZE = .01;
constexpr double ARM_GRID_SIZE = 2.;
constexpr int ELEVATOR_GRID_NUMS =
    constexprFloor((ELEVATOR_MAX_POSITION_METER - ELEVATOR_MIN_POSITION_METER) /
                   ELEVATOR_GRID_SIZE) +
    1;
constexpr int ARM_GRID_NUMS =
    constexprFloor((ARM_MAX_THETA_DEGREE - ARM_MIN_THETA_DEGREE) /
                   ARM_GRID_SIZE) +
    1;
};  // namespace alphabot

namespace env {
constexpr double L1_FRONT_HEIGHT = 0.454;
constexpr double L1_BACK_HEIGHT = 0.514;
constexpr double L1_FRONT_2_L1_BACK = 0.248;
constexpr double L1_BACK_2_REEF_CENTER = 0.832 - L1_FRONT_2_L1_BACK;
constexpr double REEF_CENTER_HEIGHT = 1.2;

constexpr double BRANCH_DIAMETER = 0.042;
constexpr double BRANCH_UL_X = 0.296;
constexpr double BRANCH_UL_Y = 1.35;
constexpr double BRANCH_LR_Y = 0.0;
constexpr double BRANCH_UR_X = BRANCH_UL_X + BRANCH_DIAMETER;
constexpr double BRANCH_UR_Y = BRANCH_UL_Y;
constexpr double BRANCH_LR_X = BRANCH_UR_X;
constexpr double BRANCH_LL_X = BRANCH_UL_X;
constexpr double BRANCH_LL_Y = BRANCH_LR_Y;

constexpr double L4_UL_X = 0.030;
constexpr double L4_UL_Y = 1.810;
constexpr double L4_UR_X = L4_UL_X + BRANCH_DIAMETER;
constexpr double L4_UR_Y = L4_UL_Y;
constexpr double L4_LR_X = L4_UR_X;
constexpr double L4_LR_Y = L4_UR_Y - 0.173;
constexpr double L4_LL_X = L4_UL_X;
constexpr double L4_LL_Y = L4_LR_Y;

constexpr double L3_UL_X = 0.053;
constexpr double L3_UL_Y = 1.194;
constexpr double L3_LL_X = 0.029;
constexpr double L3_LL_Y = 1.16;
constexpr double L3_UR_Y = 1.024;
constexpr double L3_LR_Y = 0.973;
constexpr double L3_UR_X = BRANCH_LL_X;
constexpr double L3_LR_X = BRANCH_LL_X;

constexpr double L2_UL_Y = 0.79;
constexpr double L2_UL_X = L3_UL_X;
constexpr double L2_LL_X = L3_LL_X;
constexpr double L2_LL_Y = L2_UL_Y - (L3_UL_Y - L3_LL_Y);
constexpr double L2_UR_X = L3_UR_X;
constexpr double L2_UR_Y = L2_UL_Y - (L3_UL_Y - L3_UR_Y);
constexpr double L2_LR_X = L3_LR_X;
constexpr double L2_LR_Y = L2_LL_Y - (L3_LL_Y - L3_LR_Y);

constexpr double ALGAE_HIGH_CENTER_X = 0.7;
constexpr double ALGAE_HIGH_CENTER_Y = 1.2;
constexpr double ALGAE_LOW_CENTER_Y = 0.4;
constexpr double ALGAE_RADIUS = 0.41;
constexpr double ALGAE_LOW_CENTER_X = ALGAE_HIGH_CENTER_X;
};  // namespace env

};  // namespace config

#endif  // CONFIG_H
