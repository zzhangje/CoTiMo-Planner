cmake_minimum_required(VERSION 3.0)
project(Cyber-Planner)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/sys
    ${PROJECT_SOURCE_DIR}/include/libs
)

add_executable(Cyber_Planner
    src/server.cpp
)