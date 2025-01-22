#ifndef CONFIG_H
#define CONFIG_H

#include <cmath>

namespace config {

// dynamic params
const double DT = .05;
const double TOPP_GAMMA = .5;
const double TOPP_BETA = 1e3;
const int TOPP_ITER = 3;

const double OBSTACLE_OFFSET = 100;
const double OBSTACLE_FIELD_REDUCTION = .6;
const double ASTAR_HEURISTIC_COEFFICIENT = 1;

const double ARM_MAX_RPS = 2 * M_PI;
const double ELEVATOR_MAX_RPS = 2 * M_PI;
const double ARM_MAX_RPSS = .01;
const double ELEVATOR_MAX_RPSS = .01;

const double ROBOT_WIDTH = .7;
const double ROBOT_HEIGHT = .2;
const double ROBOT_2_L1_FRONT = .05;

const double ELEVATOR_2_L1_FRONT = .4;
const double ELEVATOR_2_GROUND = .2;
const double ELEVATOR_2_GROUND_ANGLE = 80.;
const double ELEVATOR_SIN_ANGLE = sin(ELEVATOR_2_GROUND_ANGLE / 180 * M_PI);
const double ELEVATOR_COS_ANGLE = cos(ELEVATOR_2_GROUND_ANGLE / 180 * M_PI);

const double ELEVATOR_MIN_POSITION = 0;
const double ELEVATOR_MAX_POSITION = 2.;
const double ELEVATOR_GRID_SIZE = .05;
const int ELEVATOR_GRID_NUMS =
    floor((ELEVATOR_MAX_POSITION - ELEVATOR_MIN_POSITION) /
          ELEVATOR_GRID_SIZE) +
    1;
const double ELEVATOR_REDUCTION = 1.;
const double ELEVATOR_ROTATION_2_POSITION = 1. / 2 / M_PI;
const double ELEVATOR_VMAX =
    ELEVATOR_MAX_RPS / ELEVATOR_REDUCTION * ELEVATOR_ROTATION_2_POSITION;
const double ELEVATOR_AMAX =
    ELEVATOR_MAX_RPSS / ELEVATOR_REDUCTION * ELEVATOR_ROTATION_2_POSITION;

const double ARM_MIN_THETA_ROTATION = 0;
const double ARM_MAX_THETA_ROTATION = 350;
const double ARM_MIN_THETA_RADIAN = ARM_MIN_THETA_ROTATION / 180 * M_PI;
const double ARM_MAX_THETA_RADIAN = ARM_MAX_THETA_ROTATION / 180 * M_PI;
const double ARM_GRID_SIZE = .1;
const int ARM_GRID_NUMS =
    floor((ARM_MAX_THETA_RADIAN - ARM_MIN_THETA_RADIAN) / ARM_GRID_SIZE) + 1;
const double ARM_REDUCTION = 1.;
const double ARM_VMAX = ARM_MAX_RPS / ARM_REDUCTION;
const double ARM_AMAX = ARM_MAX_RPSS / ARM_REDUCTION;

// static params
const double L1_FRONT_HEIGHT = .454;
const double L1_BACK_HEIGHT = .514;
const double L1_FRONT_2_L1_BACK = .248;
const double L1_BACK_2_REEF_CENTER = .832 - L1_FRONT_2_L1_BACK;
const double REEF_CENTER_HEIGHT = 1.2;

const double BRANCH_DIAMETER = .042;
const double BRANCH_UL_X = .296;
const double BRANCH_UL_Y = 1.35;
const double BRANCH_UR_X = BRANCH_UL_X + BRANCH_DIAMETER;
const double BRANCH_UR_Y = BRANCH_UL_Y;
const double BRANCH_LR_X = BRANCH_UR_X;
const double BRANCH_LR_Y = .0;
const double BRANCH_LL_X = BRANCH_UL_X;
const double BRANCH_LL_Y = BRANCH_LR_Y;

const double L4_UL_X = .030;
const double L4_UL_Y = 1.810;
const double L4_UR_X = L4_UL_X + BRANCH_DIAMETER;
const double L4_UR_Y = L4_UL_Y;
const double L4_LR_X = L4_UR_X;
const double L4_LR_Y = L4_UR_Y - .173;
const double L4_LL_X = L4_UL_X;
const double L4_LL_Y = L4_LR_Y;

const double L3_UL_X = .053;
const double L3_UL_Y = 1.194;
const double L3_LL_X = .029;
const double L3_LL_Y = 1.16;
const double L3_UR_X = BRANCH_LL_X;
const double L3_UR_Y = 1.024;
const double L3_LR_X = BRANCH_LL_X;
const double L3_LR_Y = .973;

const double L2_UL_X = L3_UL_X;
const double L2_UL_Y = .79;
const double L2_LL_X = L3_LL_X;
const double L2_LL_Y = L2_UL_Y - (L3_UL_Y - L3_LL_Y);
const double L2_UR_X = L3_UR_X;
const double L2_UR_Y = L2_UL_Y - (L3_UL_Y - L3_UR_Y);
const double L2_LR_X = L3_LR_X;
const double L2_LR_Y = L2_LL_Y - (L3_LL_Y - L3_LR_Y);

const double ALGAE_HIGH_CENTER_X = .7;
const double ALGAE_HIGH_CENTER_Y = 1.2;
const double ALGAE_LOW_CENTER_X = ALGAE_HIGH_CENTER_X;
const double ALGAE_LOW_CENTER_Y = .4;
const double ALGAE_RADIUS = .41;

};  // namespace config

#endif  // CONFIG_H