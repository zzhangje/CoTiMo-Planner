cmake_minimum_required(VERSION 3.0)
project(Cyber-Planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(absl_DIR ${PROTOBUF}/lib/cmake/absl)
set(utf8_range_DIR ${PROTOBUF}/lib/cmake/utf8_range)

find_package(Eigen3 REQUIRED)
find_package(absl CONFIG REQUIRED)

get_filename_component(rg_proto "proto/ArmTrajectoryService.proto" ABSOLUTE)
get_filename_component(rg_proto_path "${rg_proto}" PATH)

set(rg_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/ArmTrajectoryService.pb.cc")
set(rg_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/ArmTrajectoryService.pb.h")
set(rg_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/ArmTrajectoryService.grpc.pb.cc")
set(rg_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/ArmTrajectoryService.grpc.pb.h")

find_program(_PROTOBUF_PROTOC protoc)
find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)

add_custom_command(
    OUTPUT  "${rg_proto_srcs}" "${rg_proto_hdrs}" "${rg_grpc_srcs}" "${rg_grpc_hdrs}"
    COMMAND ${_PROTOBUF_PROTOC}
    ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
    --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
    -I "${rg_proto_path}"
    --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
    "${rg_proto}"
    DEPENDS "${rg_proto}"
)

include_directories(
    /usr/local/include
    ${EIGEN3_INCLUDE_DIR}
    ${gRPC_INCLUDE_DIR}
    ${Protobuf_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/sys
    ${PROJECT_SOURCE_DIR}/include/libs
)
add_definitions(
    ${Protobuf_DEFINITIONS}
)
link_directories(
    ${Protobuf_LIBRARY_DIRS}
)

add_library(arm_grpc_proto
    ${rg_proto_srcs}
    ${rg_grpc_srcs}
    ${rg_proto_hdrs}
    ${rg_grpc_hdrs}
)
target_link_libraries(arm_grpc_proto
    absl::absl_log
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF}
)

add_executable(Cyber_Planner
    src/server.cpp
)
target_link_libraries(Cyber_Planner
    arm_grpc_proto
    absl::absl_log
    absl::flags_parse
    absl::log_initialize
    absl::log_globals
    ${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF}
    ${Protobuf_LIBRARIES}
    ${gRPC_LIBRARIES}
)