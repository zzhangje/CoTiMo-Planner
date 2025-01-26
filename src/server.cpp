#include <GLFW/glfw3.h>
#include <grpcpp/grpcpp.h>
#include <implot.h>
#include <windows.h>

#include <Eigen/Eigen>
#include <atomic>
#include <mutex>
#include <thread>

#include "Object.hpp"
#include "Topp.hpp"
#include "astar.hpp"
#include "config.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "log.hpp"
#include "map.hpp"
#include "proto/ArmTrajectoryService.grpc.pb.h"
#include "visualize.hpp"

using com::nextinnovation::armtrajectoryservice::ArmPositionState;
using com::nextinnovation::armtrajectoryservice::ArmTrajectory;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryParameter;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryService;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryState;
using google::protobuf::RepeatedPtrField;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

ArmTrajectory* sharedTrajectory = new ArmTrajectory();
bool hasNewTrajectory = false;
int requestCount = 0;
std::mutex trajectoryMutex;

class Service final : public ArmTrajectoryService::Service {
  Status generate(ServerContext* context, const ArmTrajectoryParameter* request,
                  Response* response) override {
    const ArmPositionState& start = request->start();
    const ArmPositionState& end = request->end();
    log_info("Received request: start(%.2f, %.2f), end(%.2f, %.2f)",
             start.shoulderheightmeter(), start.elbowpositiondegree(),
             end.shoulderheightmeter(), end.elbowpositiondegree());

    Eigen::Vector2d startTR(start.shoulderheightmeter(), start.elbowpositiondegree());
    Eigen::Vector2d endTR(end.shoulderheightmeter(), end.elbowpositiondegree());
    Eigen::Vector2i startGridIdx = getGridIdx(startTR);
    Eigen::Vector2i endGridIdx = getGridIdx(endTR);
    log_info("Start grid index: (%d, %d), end grid index: (%d, %d)",
             startGridIdx(0), startGridIdx(1), endGridIdx(0), endGridIdx(1));

    ObjectType armType, expType;
    if (request->hasalgae() && request->hascoral()) {
      armType = ObjectType::ARM_ALGAE_CORAL;
      expType = ObjectType::ARM_EXP_ALGAE_CORAL;
    } else if (request->hasalgae()) {
      armType = ObjectType::ARM_ALGAE;
      expType = ObjectType::ARM_EXP_ALGAE;
    } else if (request->hascoral()) {
      armType = ObjectType::ARM_CORAL;
      expType = ObjectType::ARM_EXP_CORAL;
    } else {
      armType = ObjectType::ARM;
      expType = ObjectType::ARM_EXP;
    }

    std::vector<std::vector<double>> grid;
    getGridMap(expType, grid);
    std::vector<Eigen::Vector2i> path, visited, sampledPath;
    if (!astar::astar(grid, startGridIdx, endGridIdx, path, visited)) {
      log_warn("Failed to find a path in the expanded map.");
      getGridMap(armType, grid);
      if (!astar::astar(grid, startGridIdx, endGridIdx, path, visited)) {
        log_error("Failed to find a path in the original map.");
        return Status::CANCELLED;
      }
    }
    log_info("Found a path with %d points.", path.size());
    // astar::samplePath(path, sampledPath, 1);
    sampledPath = path;

    ArmTrajectory* trajectory = response->mutable_trajectory();
    Topp topp(getTRs(sampledPath));
    topp.getTrajectory(trajectory);
    *trajectory->mutable_parameter() = *request;

    {
      std::unique_lock<std::mutex> lock(trajectoryMutex);
      sharedTrajectory->CopyFrom(*trajectory);
      hasNewTrajectory = true;
      ++requestCount;
    }

    log_info("Generated trajectory with %d points", trajectory->states_size());
    return Status::OK;
  }
};

// render thread
int guiRender() {
  log_info("Starting GUI render thread...");

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
  ImPlot::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;
  ImGui::StyleColorsDark();

  // Initialize ImGui backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 150");

  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  // thread variables
  int count = 0;
  ArmTrajectory renderTrajectory;

  std::vector<std::vector<double>> emap, amap;
  std::vector<Eigen::Vector2d> doublePath;
  ObjectType armType, expType;
  double t = 0, r = 0;
  int simIndex = 0;
  auto simStart = std::chrono::high_resolution_clock::now();

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (hasNewTrajectory) {
      // thread variables update
      {
        hasNewTrajectory = false;
        renderTrajectory.CopyFrom(*sharedTrajectory);
        count = requestCount;
      }

      // update map and path
      std::vector<Eigen::Vector2i> path = {};
      if (renderTrajectory.parameter().hasalgae() && renderTrajectory.parameter().hascoral()) {
        armType = ObjectType::ARM_ALGAE_CORAL;
        expType = ObjectType::ARM_EXP_ALGAE_CORAL;
      } else if (renderTrajectory.parameter().hasalgae()) {
        armType = ObjectType::ARM_ALGAE;
        expType = ObjectType::ARM_EXP_ALGAE;
      } else if (renderTrajectory.parameter().hascoral()) {
        armType = ObjectType::ARM_CORAL;
        expType = ObjectType::ARM_EXP_CORAL;
      } else {
        armType = ObjectType::ARM;
        expType = ObjectType::ARM_EXP;
      }
      getGridMap(armType, amap);
      getGridMap(expType, emap);

      doublePath.clear();
      for (const ArmTrajectoryState& state : renderTrajectory.states()) {
        doublePath.push_back(Eigen::Vector2d(state.position().shoulderheightmeter(),
                                             state.position().elbowpositiondegree()));
      }
    }

    // update simulation
    auto simNow = std::chrono::high_resolution_clock::now();
    if (doublePath.size() > 0) {
      double simTime = std::chrono::duration_cast<std::chrono::milliseconds>(simNow - simStart).count() / 1000.0;
      if (simTime > renderTrajectory.states(simIndex + 1).timestamp()) {
        simIndex++;
        log_info("--- simIndex: %d", simIndex);
        if (simIndex >= renderTrajectory.states_size() - 1) {
          simIndex = 0;
          simStart = simNow;
        }
      }
      double leftTime = simTime - renderTrajectory.states(simIndex).timestamp();
      double leftRatio = leftTime / (renderTrajectory.states(simIndex + 1).timestamp() - renderTrajectory.states(simIndex).timestamp());
      log_info("simTime: %.3f, leftTime: %.3f, leftRatio: %.3f", simTime, leftTime, leftRatio);
      t = renderTrajectory.states(simIndex).position().shoulderheightmeter() * (1 - leftRatio) +
          renderTrajectory.states(simIndex + 1).position().shoulderheightmeter() * leftRatio;
      r = renderTrajectory.states(simIndex).position().elbowpositiondegree() * (1 - leftRatio) +
          renderTrajectory.states(simIndex + 1).position().elbowpositiondegree() * leftRatio;
    }

    ImGui::Begin("Arm Trajectory");
    if (ImPlot::BeginPlot("Configuration Space", "Shoulder Height (m)", "Elbow Position (degree)")) {
      std::vector<double> obsX, obsY;
      for (int t = 0; t < emap.size(); t++) {
        for (int r = 0; r < emap[t].size(); r++) {
          if (emap[t][r] > config::params::OBSTACLE_OFFSET - .01) {
            Eigen::Vector2d tr = getTR(t, r);
            obsX.push_back(tr(0));
            obsY.push_back(tr(1));
          }
        }
      }
      ImPlot::PlotScatter("obstacle", obsX.data(), obsY.data(), obsX.size());

      double* trajX = new double[doublePath.size()];
      double* trajY = new double[doublePath.size()];
      for (int i = 0; i < doublePath.size(); ++i) {
        trajX[i] = doublePath[i](0);
        trajY[i] = doublePath[i](1);
      }
      ImPlot::PlotLine("trajectory", trajX, trajY, doublePath.size());

      double currentX[] = {t};
      double currentY[] = {r};
      ImPlot::PlotScatter("current", currentX, currentY, 1);
      ImPlot::EndPlot();
    }
    ImGui::End();

    ImGui::Begin("Trajectory Params");
    if (ImPlot::BeginPlot("Shoulder Velocity", "Time (s)", "Velocity (m/s)")) {
      double* velocityT = new double[renderTrajectory.states_size()];
      double* timestamp = new double[renderTrajectory.states_size()];
      for (int i = 0; i < renderTrajectory.states_size(); ++i) {
        velocityT[i] = renderTrajectory.states(i).velocity().shouldervelocitymeterpersecond();
        timestamp[i] = renderTrajectory.states(i).timestamp();
      }
      ImPlot::PlotLine("Shoulder Velocity", timestamp, velocityT, renderTrajectory.states_size());
      std::vector<double> vmaxX = {0, timestamp[renderTrajectory.states_size() - 1]};
      std::vector<double> vmaxY = {ELEVATOR_VMAX, ELEVATOR_VMAX};
      std::vector<double> vminY = {-ELEVATOR_VMAX, -ELEVATOR_VMAX};
      ImPlot::PlotLine("Shoulder Max Velocity", vmaxX.data(), vmaxY.data(), 2);
      ImPlot::PlotLine("Shoulder Min Velocity", vmaxX.data(), vminY.data(), 2);
      ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Elbow Velocity", "Time (s)", "Velocity (degree/s)")) {
      double* velocityT = new double[renderTrajectory.states_size()];
      double* timestamp = new double[renderTrajectory.states_size()];
      for (int i = 0; i < renderTrajectory.states_size(); ++i) {
        velocityT[i] = renderTrajectory.states(i).velocity().elbowvelocitydegreepersecond();
        timestamp[i] = renderTrajectory.states(i).timestamp();
      }
      ImPlot::PlotLine("Elbow Velocity", timestamp, velocityT, renderTrajectory.states_size());
      std::vector<double> vmaxX = {0, timestamp[renderTrajectory.states_size() - 1]};
      std::vector<double> vmaxY = {ARM_VMAX, ARM_VMAX};
      std::vector<double> vminY = {-ARM_VMAX, -ARM_VMAX};
      ImPlot::PlotLine("Elbow Max Velocity", vmaxX.data(), vmaxY.data(), 2);
      ImPlot::PlotLine("Elbow Min Velocity", vmaxX.data(), vminY.data(), 2);
      ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Voltage", "Time (s)", "Voltage (V)")) {
      double* elbowVoltageT = new double[renderTrajectory.states_size()];
      double* shoulderVoltageT = new double[renderTrajectory.states_size()];
      double* timestamp = new double[renderTrajectory.states_size()];
      for (int i = 0; i < renderTrajectory.states_size(); ++i) {
        timestamp[i] = renderTrajectory.states(i).timestamp();
        shoulderVoltageT[i] = renderTrajectory.states(i).voltage().shouldervoltagevolt();
        elbowVoltageT[i] = renderTrajectory.states(i).voltage().elbowvoltagevolt();
      }
      ImPlot::PlotLine("Shoulder Voltage", timestamp, shoulderVoltageT, renderTrajectory.states_size());
      ImPlot::PlotLine("Elbow Voltage", timestamp, elbowVoltageT, renderTrajectory.states_size());
      std::vector<double> vmaxX = {0, timestamp[renderTrajectory.states_size() - 1]};
      std::vector<double> vmaxY = {ARM_MAX_VOLTAGE, ARM_MAX_VOLTAGE};
      std::vector<double> vminY = {-ARM_MAX_VOLTAGE, -ARM_MAX_VOLTAGE};
      ImPlot::PlotLine("Max Voltage", vmaxX.data(), vmaxY.data(), 2);
      ImPlot::PlotLine("Min Voltage", vmaxX.data(), vminY.data(), 2);
      ImPlot::EndPlot();
    }
    ImGui::End();

    ImGui::Begin("2D Projection");
    if (ImPlot::BeginPlot("2D Projection", "X", "Y")) {
      Object env = Object(ObjectType::ENV);
      for (int i = 0; i < env.getSegments().size(); ++i) {
        double plotX[2] = {env.getSegments()[i].getPts1X(), env.getSegments()[i].getPts2X()};
        double plotY[2] = {env.getSegments()[i].getPts1Y(), env.getSegments()[i].getPts2Y()};
        ImPlot::PlotLine("obstacle", plotX, plotY, 2);
      }
      Object arm = Object(armType, t, r);
      for (int i = 0; i < arm.getSegments().size(); ++i) {
        double plotX[2] = {arm.getSegments()[i].getPts1X(), arm.getSegments()[i].getPts2X()};
        double plotY[2] = {arm.getSegments()[i].getPts1Y(), arm.getSegments()[i].getPts2Y()};
        ImPlot::PlotLine("arm", plotX, plotY, 2);
      }
      Object exp = Object(expType, t, r);
      for (int i = 0; i < exp.getSegments().size(); ++i) {
        double plotX[2] = {exp.getSegments()[i].getPts1X(), exp.getSegments()[i].getPts2X()};
        double plotY[2] = {exp.getSegments()[i].getPts1Y(), exp.getSegments()[i].getPts2Y()};
        ImPlot::PlotLine("expanded arm", plotX, plotY, 2);
      }
      std::vector<double> elevatorX = {-ELEVATOR_2_L1_FRONT + ELEVATOR_MIN_POSITION_METER * ELEVATOR_COS_ANGLE, -ELEVATOR_2_L1_FRONT + ELEVATOR_MAX_POSITION_METER * ELEVATOR_COS_ANGLE};
      std::vector<double> elevatorY = {ELEVATOR_2_GROUND + ELEVATOR_MIN_POSITION_METER * ELEVATOR_SIN_ANGLE, ELEVATOR_2_GROUND + ELEVATOR_MAX_POSITION_METER * ELEVATOR_SIN_ANGLE};
      ImPlot::PlotLine("elevator", elevatorX.data(), elevatorY.data(), 2);
      ImPlot::EndPlot();
    }
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
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
  log_warn("GUI render thread terminated.");
  return 0;
}

int main(int argc, char* argv[]) {
  // FreeConsole()

  log_info(
      "Welcome to Cyber Planner 2025!"
      "\n"
      "  ______   ______  _____ ____  \n"
      " / ___\\ \\ / / __ )| ____|  _ \\ \n"
      "| |    \\ V /|  _ \\|  _| | |_) |\n"
      "| |___  | | | |_) | |___|  _ < \n"
      " \\____| |_| |____/|_____|_| \\_\\\n"
      " ____  _        _    _   _ _   _ _____ ____  \n"
      "|  _ \\| |      / \\  | \\ | | \\ | | ____|  _ \\\n"
      "| |_) | |     / _ \\ |  \\| |  \\| |  _| | |_) |\n"
      "|  __/| |___ / ___ \\| |\\  | |\\  | |___|  _ < \n"
      "|_|   |_____/_/   \\_\\_| \\_|_| \\_|_____|_| \\_\\\n");

  Service service;
  ServerBuilder builder;
  std::thread windowThread(guiRender);

  builder.AddChannelArgument(GRPC_ARG_MAX_CONCURRENT_STREAMS, 1);
  builder.AddListeningPort("0.0.0.0:" + config::params::GRPC_PORT, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  log_info(("Server is running on 0.0.0.0:" + config::params::GRPC_PORT).c_str());
  builder.BuildAndStart()->Wait();

  return 0;
}