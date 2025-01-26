cmake_minimum_required(VERSION 3.15)

if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING "")
endif()

project(Cyber-Planner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_definitions(-D_USE_MATH_DEFINES)

find_package(Eigen3 CONFIG REQUIRED)
find_package(protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(Threads)
find_package(glfw3 REQUIRED)
find_package(gl3w REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)

add_subdirectory(imgui)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/libs
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/include/implot
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)

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

add_executable(server WIN32
    src/server.cpp
    src/implot/implot.cpp
    src/implot/implot_items.cpp
    ${HEADER_FILES}
)

target_include_directories(server PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/proto
)

target_link_libraries(server PRIVATE
    trajectory_native_proto
    Eigen3::Eigen
    imgui
    glfw
    unofficial::gl3w::gl3w
    OpenGL::GL
)

set_target_properties(server PROPERTIES
    LINK_FLAGS "/SUBSYSTEM:WINDOWS"
)

add_executable(client
    src/client.cpp
    ${HEADER_FILES}
)

target_include_directories(client PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/proto)

target_link_libraries(client
    trajectory_native_proto
    Eigen3::Eigen
)

add_executable(cachee
    src/cache.cpp
    ${HEADER_FILES}
)

target_include_directories(cachee PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/proto)

target_link_libraries(cachee
    trajectory_native_proto
    Eigen3::Eigen
)