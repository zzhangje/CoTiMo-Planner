#ifndef CONFIG_H
#define CONFIG_H

#include <cmath>
#include <string>

namespace nextinnovation {

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

namespace config {
const std::string GRPC_PORT = "58214";
const bool IS_DEBUG = true;
}  // namespace config

namespace alphabot {
const std::string ROBOT_PARAMS_VERSION =
    "8214.0.0";  // for cache version control

// the prohitbit area of the drivetrain
constexpr double ROBOT_WIDTH = 0.7;  // the width of the prohitbit area
constexpr double ROBOT_HEIGHT =
    0.1;  // the height of the prohitbit area to the ground
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

// the properties of the elevator motor
const double ELEVATOR_MIN_POSITION_METER = 0;
const double ELEVATOR_MAX_POSITION_METER = 2.0;
const double ELEVATOR_V_MAX = 12;
const double ELEVATOR_I_MAX = 25;
const double ELEVATOR_Kg = 1;
const double ELEVATOR_Ks = 1;
const double ELEVATOR_Kv = 57;
const double ELEVATOR_Ka = 50;
const double ELEVATOR_R = 0.1;
const double ELEVATOR_METER_2_MOTOR_RADIAN = .1;

// the properties of the arm motor
constexpr double ARM_MIN_THETA_DEGREE = 0;
constexpr double ARM_MAX_THETA_DEGREE = 350;
constexpr double ARM_MIN_THETA_RADIAN = toRadians(ARM_MIN_THETA_DEGREE);
constexpr double ARM_MAX_THETA_RADIAN = toRadians(ARM_MAX_THETA_DEGREE);
const double ARM_V_MAX = 12;
const double ARM_I_MAX = 25;
const double ARM_Kg = 1;
const double ARM_Ks = 1;
const double ARM_Kv = 57;
const double ARM_Ka = .50;
const double ARM_R = 0.1;
const double ARM_RADIAN_2_MOTOR_RADIAN = .1;

// the properties of the grid map
constexpr double ELEVATOR_GRID_SIZE = .01;
constexpr double ARM_GRID_SIZE = .01;
constexpr int ELEVATOR_GRID_NUMS =
    constexprFloor((ELEVATOR_MAX_POSITION_METER - ELEVATOR_MIN_POSITION_METER) /
                   ELEVATOR_GRID_SIZE) +
    1;
constexpr int ARM_GRID_NUMS =
    constexprFloor((ARM_MAX_THETA_RADIAN - ARM_MIN_THETA_RADIAN) /
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

};  // namespace nextinnovation

#endif  // CONFIG_H
