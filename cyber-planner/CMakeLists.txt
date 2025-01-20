cmake_minimum_required(VERSION 3.0)
project(cyber-planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    rospy
    cmake_modules
    std_msgs
)
catkin_package()
find_package(Eigen3 REQUIRED)

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

add_executable(plan_node
    src/utils/astar.hpp
    src/utils/spline.hpp
    src/utils/lbfgs.hpp
    src/utils/topp.hpp
    src/plan_node.cpp
)

catkin_install_python(PROGRAMS scripts/plot_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(plan_node ${catkin_LIBRARIES})