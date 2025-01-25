#include <GLFW/glfw3.h>  // Include GLFW
#include <windows.h>

#include <cmath>
#include <thread>

#include "Object.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "map.hpp"

int renderThread() {
  FreeConsole();

  // Initialize GLFW
  if (!glfwInit())
    return -1;

  // Create window
  GLFWwindow* window = glfwCreateWindow(1280, 720, "Cyber Planner 2025", NULL, NULL);
  if (window == NULL) {
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // Enable vsync

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;
  ImGui::StyleColorsDark();

  // Initialize ImGui backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 150");

  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Create a window
    ImGui::Begin("Hello, world!");
    ImGui::Text("This is some useful text.");
    ImGui::End();

    static float data[100];
    for (int i = 0; i < 100; i++) {
      data[i] = (float)std::sin(i * 0.1f);
    }

    // Create a second window
    ImGui::Begin("Cyber Planner 2025");
    ImGui::PlotLines("", data, IM_ARRAYSIZE(data), 0, nullptr);
    ImGui::End();

    ImGui::Begin("Cyber Planner 2023");
    glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
    glPointSize(5);

    std::vector<std::vector<double>> emap, amap;
    std::vector<Eigen::Vector2i> path = {};
    getGridMap(ObjectType::ARM, amap);
    getGridMap(ObjectType::ARM_EXP, emap);
    GLubyte* map;
    getRenderMap(map, amap, emap, path);
    glDrawPixels(128, 72, GL_LUMINANCE, GL_UNSIGNED_BYTE, map);

    glPopAttrib();
    ImGui::End();

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}

int main(int, char**) {
  std::thread t(renderThread);
  while (1) {
  }
  return 0;
}