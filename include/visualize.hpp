#ifndef VISUALIZE_HPP
#define VISUALIZE_HPP

#include <GLFW/glfw3.h>
#include <implot.h>

#include "config.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
using namespace config::alphabot;

namespace visualize {

void renderMap(const std::vector<std::vector<double>>& amap, const std::vector<std::vector<double>>& emap, const std::vector<Eigen::Vector2i>& path) {
  if (ImPlot::BeginPlot("Map", "X", "Y")) {
    // ImPlot::PlotHeatmap("Algae", amap.data(), ELEVATOR_GRID_SIZE, ARM_GRID_SIZE);
    // ImPlot::PlotHeatmap("Coral", emap.data(), ELEVATOR_GRID_SIZE, ARM_GRID_SIZE);
    ImPlot::EndPlot();
  }
}

}  // namespace visualize

#endif  // VISUALIZE_HPP