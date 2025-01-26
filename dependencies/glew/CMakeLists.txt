cmake_minimum_required(VERSION 3.10)
project(glew)

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

file(GLOB GLEW_SOURCES "include/*.h" "src/*.c")

# Create ImGui library
add_library(imgui STATIC ${GLEW_SOURCES})

# Include directories
target_include_directories(glew PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GLFW_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(glew PUBLIC
    glfw
    ${OPENGL_LIBRARIES}
)
