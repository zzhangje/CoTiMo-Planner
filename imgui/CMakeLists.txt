cmake_minimum_required(VERSION 3.10)
project(imgui)

file(GLOB IMGUI_SOURCES "imgui*.cpp" "imgui*.h")

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

# Create ImGui library
add_library(imgui STATIC ${IMGUI_SOURCES})

# Include directories
target_include_directories(imgui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${GLFW_INCLUDE_DIRS})
target_link_libraries(imgui PUBLIC glfw ${OPENGL_LIBRARIES})
