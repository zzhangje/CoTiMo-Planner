cmake_minimum_required(VERSION 3.0)
project(Cyber-Planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/libs
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils
)

find_package(Eigen3 REQUIRED)
find_package(protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(Threads)

add_library(trajectory_native_proto proto/ArmTrajectoryService.proto)
target_link_libraries(trajectory_native_proto PUBLIC protobuf::libprotobuf gRPC::grpc gRPC::grpc++ gRPC::grpc++_reflection)
target_include_directories(trajectory_native_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

get_target_property(grpc_cpp_plugin_location gRPC::grpc_cpp_plugin LOCATION)
protobuf_generate(TARGET trajectory_native_proto LANGUAGE cpp)
protobuf_generate(TARGET trajectory_native_proto LANGUAGE grpc GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc PLUGIN "protoc-gen-grpc=${grpc_cpp_plugin_location}")

file(GLOB_RECURSE HEADER_FILES 
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp"
)

add_executable(Cyber_Planner
    src/server.cpp
    ${HEADER_FILES}
)

target_link_libraries(Cyber_Planner
    trajectory_native_proto
    Eigen3::Eigen
)

add_executable(test
    src/test.cpp
    ${HEADER_FILES}
)

target_compile_features(test PRIVATE cxx_std_11)
find_package(Python COMPONENTS Development NumPy REQUIRED)
target_include_directories(test PRIVATE ${Python_INCLUDE_DIRS} ${Python_NumPy_INCLUDE_DIRS})
target_link_libraries(test PRIVATE Python::Python Python::NumPy)