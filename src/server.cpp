// #include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <fmt/chrono.h>
#include <fmt/color.h>
#include <fmt/core.h>
#include <grpcpp/grpcpp.h>
#include <implot.h>

#include <Eigen/Eigen>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "Object.hpp"
#include "Polygon.hpp"
#include "Smooth.hpp"
#include "Topp.hpp"
#include "astar.hpp"
#include "config.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "log.hpp"
#include "proto/ArmTrajectoryService.grpc.pb.h"
#include "space.hpp"

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#define LOCK_WINDOW_RESIZE

#ifdef LOCK_WINDOW_RESIZE
#define WINDOW_FLAGS ImGuiWindowFlags_NoResize
#else
#define WINDOW_FLAGS 0
#endif

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

using namespace nextinnovation::alphabot;

// shared variables
ArmTrajectory* g_trajectory = new ArmTrajectory();
std::vector<Eigen::Vector2d>* g_voltage = new std::vector<Eigen::Vector2d>();
std::vector<Eigen::Vector2d>* g_velocity = new std::vector<Eigen::Vector2d>();
std::vector<Eigen::Vector2d>* g_path = new std::vector<Eigen::Vector2d>();
bool g_hasNewTrajectory = false;
bool g_isRunning = true;
std::mutex g_trajectoryMutex;

void CompatibleFreeConsole() {
#ifdef _WIN32
  FreeConsole();
#else
  close(STDIN_FILENO);
  close(STDOUT_FILENO);
  close(STDERR_FILENO);
#endif
}

void CompatibleAllocConsole() {
#ifdef _WIN32
  AllocConsole();
#else
  int fd = open("/dev/tty", O_RDWR);
  dup2(fd, STDIN_FILENO);
  dup2(fd, STDOUT_FILENO);
  dup2(fd, STDERR_FILENO);
  close(fd);
#endif
}

struct ImGuiMessage {
  ImVec4 color;
  std::string level;
  std::string message;
  std::chrono::system_clock::time_point timestamp;
};
std::deque<ImGuiMessage> consoleOutputs;
std::mutex consoleOutputMutex;
const size_t consoleOutputSize = 500;

enum class LogLevel {
  LOG_INFO,
  LOG_WARN,
  LOG_ERROR
};

void ShowLog(LogLevel level, const char* format, ...) {
  static const std::unordered_map<LogLevel, std::pair<ImVec4, const char*>> logConfig = {
      {LogLevel::LOG_INFO, {{0.0f, 1.0f, 0.0f, 1.0f}, "INFO "}},
      {LogLevel::LOG_WARN, {{1.0f, 1.0f, 0.0f, 1.0f}, "WARN "}},
      {LogLevel::LOG_ERROR, {{1.0f, 0.0f, 0.0f, 1.0f}, "ERROR"}}};

  va_list args;
  va_start(args, format);
  char buffer[2048];
  vsnprintf(buffer, sizeof(buffer), format, args);
  buffer[sizeof(buffer) - 1] = '\0';
  va_end(args);

  const auto& config = logConfig.at(level);
  {
    std::lock_guard<std::mutex> lock(consoleOutputMutex);
    consoleOutputs.push_back({config.first, config.second, buffer, std::chrono::system_clock::now()});
    if (consoleOutputs.size() > consoleOutputSize) {
      consoleOutputs.pop_front();
    }
  }
}

inline void ShowError(const char* format, ...) {
  va_list args;
  va_start(args, format);
  char buffer[2048];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  ShowLog(LogLevel::LOG_ERROR, "%s", buffer);
}

inline void ShowWarn(const char* format, ...) {
  va_list args;
  va_start(args, format);
  char buffer[2048];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  ShowLog(LogLevel::LOG_WARN, "%s", buffer);
}

inline void ShowInfo(const char* format, ...) {
  va_list args;
  va_start(args, format);
  char buffer[2048];
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  ShowLog(LogLevel::LOG_INFO, "%s", buffer);
}

// gRPC service
class Service final : public ArmTrajectoryService::Service {
  Status generate(ServerContext* context, const ArmTrajectoryParameter* request,
                  Response* response) override {
    const ArmPositionState& start = request->start();
    const ArmPositionState& end = request->end();
    log_info("Received request: start(%.2f, %.2f), end(%.2f, %.2f)",
             start.shoulderheightmeter(), start.elbowpositionradian(),
             end.shoulderheightmeter(), end.elbowpositionradian());
    ShowInfo("Receive a request");

    // get the grid index of the start and end points
    Eigen::Vector2d startTR(start.shoulderheightmeter(), start.elbowpositionradian());
    Eigen::Vector2d endTR(end.shoulderheightmeter(), end.elbowpositionradian());
    Eigen::Vector2i startGridIdx = nextinnovation::getGridIdx(startTR);
    Eigen::Vector2i endGridIdx = nextinnovation::getGridIdx(endTR);
    log_info("Start grid index: (%d, %d), end grid index: (%d, %d)",
             startGridIdx(0), startGridIdx(1), endGridIdx(0), endGridIdx(1));

    // judge the type of the arm
    nextinnovation::ObjectType armType, expType;
    if (request->hasalgae() && request->hascoral()) {
      armType = nextinnovation::ObjectType::ARM_ALGAE_CORAL;
      expType = nextinnovation::ObjectType::ARM_EXP_ALGAE_CORAL;
    } else if (request->hasalgae()) {
      armType = nextinnovation::ObjectType::ARM_ALGAE;
      expType = nextinnovation::ObjectType::ARM_EXP_ALGAE;
    } else if (request->hascoral()) {
      armType = nextinnovation::ObjectType::ARM_CORAL;
      expType = nextinnovation::ObjectType::ARM_EXP_CORAL;
    } else {
      armType = nextinnovation::ObjectType::ARM;
      expType = nextinnovation::ObjectType::ARM_EXP;
    }

    // find the path in the grid map
    std::vector<std::vector<double>> grid;
    getGridMap(expType, grid);
    std::vector<Eigen::Vector2i> gridPath, visited, sampledPath;
    if (!nextinnovation::astar(grid, startGridIdx, endGridIdx, gridPath, visited)) {
      log_warn("Failed to find a path in the expanded map.");
      ShowWarn("Failed to find a path in the expanded map.");
      getGridMap(armType, grid);
      if (!nextinnovation::astar(grid, startGridIdx, endGridIdx, gridPath, visited)) {
        log_error("Failed to find a path in the original map.");
        ShowError("Failed to find a path in the original map.");
        return Status::CANCELLED;
      }
    }
    log_info("Found a path with %d points.", gridPath.size());
    nextinnovation::samplePath(gridPath, sampledPath, 15);
    std::vector<Eigen::Vector2d> path = nextinnovation::getTRs(sampledPath);

    // optimize the path
    nextinnovation::Smooth smooth(path, armType);
    path = smooth.getPath();

    // generate the trajectory
    ArmTrajectory* trajectory = response->mutable_trajectory();
    std::vector<Eigen::Vector2d> voltage, velocity;
    nextinnovation::Topp topp(path);
    topp.getTrajectory(trajectory, voltage, velocity);
    *trajectory->mutable_parameter() = *request;

    // write the trajectory to the shared variable
    {
      std::unique_lock<std::mutex> lock(g_trajectoryMutex);
      g_trajectory->CopyFrom(*trajectory);
      g_voltage->clear();
      g_velocity->clear();
      g_path->clear();
      for (const Eigen::Vector2d& v : voltage) {
        g_voltage->push_back(v);
      }
      for (const Eigen::Vector2d& v : velocity) {
        g_velocity->push_back(v);
      }
      for (const Eigen::Vector2d& p : nextinnovation::getTRs(sampledPath)) {
        g_path->push_back(p);
      }
      g_hasNewTrajectory = true;
    }

    log_info("Generated trajectory with %d points", trajectory->states_size());
    ShowInfo("Path successfully generated");
    return Status::OK;
  }
};

void RunGrpcServer() {
  std::string serverAddress("0.0.0.0:" + nextinnovation::config::GRPC_PORT);
  Service service;

  ServerBuilder builder;
  builder.AddChannelArgument(GRPC_ARG_MAX_CONCURRENT_STREAMS, 1);
  builder.AddListeningPort(serverAddress, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  std::unique_ptr<Server> server(builder.BuildAndStart());
  log_info("Server is running on %s", serverAddress.c_str());
  ShowInfo("Server is running on %s", serverAddress.c_str());

  while (g_isRunning) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  server->Shutdown();
  ShowInfo("Server is shutting down.");
  log_info("Server is shutting down.");
}

int main(int argc, char* argv[]) {
  // free console
  // CompatibleFreeConsole();

  log_set_quiet(false);

  ShowInfo("Welcome to Cyber Planner 2025");
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

  // initialize glfw
  if (!glfwInit()) {
    ShowError("Failed to initialize GLFW.");
    log_error("Failed to initialize GLFW.");
    return -1;
  }

  // create window
  glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);  // 设置窗口启动时最大化
  GLFWwindow* window = glfwCreateWindow(1920, 1080, "Cyber Planner 2025", NULL, NULL);
  if (!window) {
    ShowError("Failed to create window.");
    log_error("Failed to create window.");
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize glew
  // glewExperimental = GL_TRUE;
  // if (glewInit() != GLEW_OK) {
  //   ShowError("Failed to initialize GLEW.");
  //   log_error("Failed to initialize GLEW.");
  //   return -1;
  // }

  // initialize imgui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;
  io.IniFilename = "imgui.ini";
  ImGui::StyleColorsClassic();
  ImGui::GetStyle().ScaleAllSizes(2.0f);
  io.FontGlobalScale = 2.0f;

  // initialize imgui for glfw
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 150");

  // initialize implot
  ImPlot::CreateContext();

  // run gRPC server
  std::thread serverThread(RunGrpcServer);

  // thread variables copy
  ArmTrajectory trajectory;
  std::vector<Eigen::Vector2d> voltage, velocity;

  std::vector<std::vector<double>> emap, amap;
  std::vector<Eigen::Vector2d> path, astarPath;
  nextinnovation::ObjectType armType = nextinnovation::ObjectType::ARM;
  nextinnovation::ObjectType expType = nextinnovation::ObjectType::ARM_EXP;
  getGridMap(armType, amap);
  getGridMap(expType, emap);
  double simT = 0, simR = 0;
  int simIndex = 0;
  auto simStart = std::chrono::high_resolution_clock::now();

  // main loop
  while (!glfwWindowShouldClose(window)) {
    if (!serverThread.joinable()) {
      ShowError("Server thread may dead, please restart the program.");
      log_error("Server thread is not joinable!");
    }

    // start the frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // handle new trajectory
    if (g_hasNewTrajectory) {
      voltage.clear();
      velocity.clear();
      astarPath.clear();
      {
        g_hasNewTrajectory = false;
        trajectory.CopyFrom(*g_trajectory);
        for (const Eigen::Vector2d& v : *g_voltage) {
          voltage.push_back(v);
        }
        for (const Eigen::Vector2d& v : *g_velocity) {
          velocity.push_back(v);
        }
        for (const Eigen::Vector2d& p : *g_path) {
          astarPath.push_back(p);
        }
      }

      // update map
      if (trajectory.parameter().hasalgae() && trajectory.parameter().hascoral()) {
        armType = nextinnovation::ObjectType::ARM_ALGAE_CORAL;
        expType = nextinnovation::ObjectType::ARM_EXP_ALGAE_CORAL;
      } else if (trajectory.parameter().hasalgae()) {
        armType = nextinnovation::ObjectType::ARM_ALGAE;
        expType = nextinnovation::ObjectType::ARM_EXP_ALGAE;
      } else if (trajectory.parameter().hascoral()) {
        armType = nextinnovation::ObjectType::ARM_CORAL;
        expType = nextinnovation::ObjectType::ARM_EXP_CORAL;
      } else {
        armType = nextinnovation::ObjectType::ARM;
        expType = nextinnovation::ObjectType::ARM_EXP;
      }
      getGridMap(armType, amap);
      getGridMap(expType, emap);

      path.clear();
      for (const ArmTrajectoryState& state : trajectory.states()) {
        path.push_back(Eigen::Vector2d(state.position().shoulderheightmeter(),
                                       state.position().elbowpositionradian()));
      }

      // reset simulation
      simIndex = 0;
      simStart = std::chrono::high_resolution_clock::now();
    }

    // handle simulation
    auto simNow = std::chrono::high_resolution_clock::now();
    double simTime = std::chrono::duration_cast<std::chrono::milliseconds>(simNow - simStart).count() / 1000.0;
    if (path.size() > 0) {
      if (simTime > trajectory.states(simIndex + 1).timestamp()) {
        simIndex++;
        if (simIndex >= trajectory.states_size() - 1) {
          simIndex = 0;
          simStart = simNow;
        }
      }
      double dt = simTime - trajectory.states(simIndex).timestamp();
      double factor = dt / (trajectory.states(simIndex + 1).timestamp() - trajectory.states(simIndex).timestamp());
      simT = trajectory.states(simIndex).position().shoulderheightmeter() * (1 - factor) +
             trajectory.states(simIndex + 1).position().shoulderheightmeter() * factor;
      simR = trajectory.states(simIndex).position().elbowpositionradian() * (1 - factor) +
             trajectory.states(simIndex + 1).position().elbowpositionradian() * factor;
    }

    // handle console output
    while (consoleOutputs.size() > consoleOutputSize) {
      consoleOutputs.pop_front();
    }

    /**
     * Window 1: Console Log
     */
    ImGui::Begin("Console Output", nullptr, WINDOW_FLAGS);
    ImGui::BeginChild("console", ImVec2(0, -ImGui::GetFrameHeightWithSpacing()), true);
    {
      std::lock_guard<std::mutex> lock(consoleOutputMutex);
      ImGuiListClipper clipper;
      clipper.Begin(consoleOutputs.size());
      while (clipper.Step()) {
        for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; i++) {
          const auto& message = consoleOutputs[i];
          ImGui::Text(fmt::format("{:%H:%M:%S}", fmt::localtime(std::chrono::system_clock::to_time_t(message.timestamp))).c_str());
          ImGui::SameLine();
          ImGui::PushStyleColor(ImGuiCol_Text, message.color);
          ImGui::Text(message.level.c_str());
          ImGui::SameLine();
          ImGui::PopStyleColor();
          ImGui::TextWrapped(message.message.c_str());
        }
      }
      if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 10.0f) {
        ImGui::SetScrollHereY(1.0f);
      }
    }
    ImGui::EndChild();
    ImGui::End();

    /**
     * Window 2: Configuration Space
     */
    ImGui::Begin("Arm Trajectory", nullptr, WINDOW_FLAGS);
    if (ImPlot::BeginPlot("Configuration Space", "Shoulder Height (m)", "Elbow Position (rad)", ImVec2(-1, 400))) {
      ImPlot::SetupAxisLimits(ImAxis_X1, ELEVATOR_MIN_POSITION_METER, ELEVATOR_MAX_POSITION_METER);
      ImPlot::SetupAxisLimits(ImAxis_Y1, ARM_MIN_THETA_RADIAN, ARM_MAX_THETA_RADIAN);

      std::vector<double> obsX, obsY, expX, expY;
      for (int t = 0; t < emap.size(); t++) {
        for (int r = 0; r < emap[t].size(); r++) {
          if (amap[t][r] > 99.9) {
            Eigen::Vector2d tr = nextinnovation::getTR(t, r);
            obsX.push_back(tr(0));
            obsY.push_back(tr(1));
          } else if (emap[t][r] > 99.9) {
            Eigen::Vector2d tr = nextinnovation::getTR(t, r);
            expX.push_back(tr(0));
            expY.push_back(tr(1));
          }
        }
      }
      ImPlot::PlotScatter("obstacle", obsX.data(), obsY.data(), obsX.size());
      ImPlot::PlotScatter("expanded", expX.data(), expY.data(), expX.size());

      double* trajX = new double[path.size()];
      double* trajY = new double[path.size()];
      for (int i = 0; i < path.size(); ++i) {
        trajX[i] = path[i](0);
        trajY[i] = path[i](1);
      }
      ImPlot::PlotLine("trajectory", trajX, trajY, path.size());

      double* astarX = new double[astarPath.size()];
      double* astarY = new double[astarPath.size()];
      for (int i = 0; i < astarPath.size(); ++i) {
        astarX[i] = astarPath[i](0);
        astarY[i] = astarPath[i](1);
      }
      ImPlot::PlotLine("astar", astarX, astarY, astarPath.size());

      double currentX[] = {simT};
      double currentY[] = {simR};
      ImPlot::PlotScatter("current", currentX, currentY, 1);
      ImPlot::EndPlot();
    }
    ImGui::End();

    /**
     * Window 3: Trajectory Params
     */
    ImGui::Begin("Trajectory Params");
    if (velocity.size() > 0 && trajectory.states_size() > 0 && voltage.size() > 0) {
      std::vector<double> simX = {simTime, simTime};
      std::vector<double> simY = {-0x3f3f3f3f, 0x3f3f3f3f};
      if (ImPlot::BeginPlot("Current")) {
        ImPlot::SetupAxes("Time (s)", "Current (A)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_None);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -1.3 * ARM_I_MAX, 1.3 * ARM_I_MAX);
        double* elbowCurrentT = new double[trajectory.states_size()];
        double* shoulderCurrentT = new double[trajectory.states_size()];
        double* timestamp = new double[trajectory.states_size()];
        for (int i = 0; i < trajectory.states_size(); ++i) {
          timestamp[i] = trajectory.states(i).timestamp();
          shoulderCurrentT[i] = trajectory.states(i).current().shouldercurrentampere();
          elbowCurrentT[i] = trajectory.states(i).current().elbowcurrentampere();
        }
        ImPlot::PlotLine("Shoulder Current", timestamp, shoulderCurrentT, trajectory.states_size());
        ImPlot::PlotLine("Elbow Current", timestamp, elbowCurrentT, trajectory.states_size());
        std::vector<double> vmaxX = {0, timestamp[trajectory.states_size() - 1]};
        std::vector<double> vmaxY = {ARM_I_MAX, ARM_I_MAX};
        std::vector<double> vminY = {-ARM_I_MAX, -ARM_I_MAX};
        ImPlot::PlotLine("Max Current", vmaxX.data(), vmaxY.data(), 2);
        ImPlot::PlotLine("Min Current", vmaxX.data(), vminY.data(), 2);
        ImPlot::PlotLine("Current Time", simX.data(), simY.data(), 2);
        ImPlot::EndPlot();
      }
      if (ImPlot::BeginPlot("Voltage")) {
        ImPlot::SetupAxes("Time (s)", "Voltage (V)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_None);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -1.3 * ARM_V_MAX, 1.3 * ARM_V_MAX);
        double* elbowVoltageT = new double[trajectory.states_size()];
        double* shoulderVoltageT = new double[trajectory.states_size()];
        double* timestamp = new double[trajectory.states_size()];
        for (int i = 0; i < trajectory.states_size(); ++i) {
          timestamp[i] = trajectory.states(i).timestamp();
          shoulderVoltageT[i] = voltage[i].x();
          elbowVoltageT[i] = voltage[i].y();
        }
        ImPlot::PlotLine("Shoulder Voltage", timestamp, shoulderVoltageT, trajectory.states_size());
        ImPlot::PlotLine("Elbow Voltage", timestamp, elbowVoltageT, trajectory.states_size());
        std::vector<double> vmaxX = {0, timestamp[trajectory.states_size() - 1]};
        std::vector<double> vmaxY = {ARM_V_MAX, ARM_V_MAX};
        std::vector<double> vminY = {-ARM_V_MAX, -ARM_V_MAX};
        ImPlot::PlotLine("Max Voltage", vmaxX.data(), vmaxY.data(), 2);
        ImPlot::PlotLine("Min Voltage", vmaxX.data(), vminY.data(), 2);
        ImPlot::PlotLine("Voltage Time", simX.data(), simY.data(), 2);
        ImPlot::EndPlot();
      }
      if (ImPlot::BeginPlot("Shoulder Velocity")) {
        ImPlot::SetupAxes("Time (s)", "Velocity (m/s)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_None);
        double* velocityT = new double[trajectory.states_size()];
        double* timestamp = new double[trajectory.states_size()];
        double maxVelocity = 1, minVelocity = -1;
        for (int i = 0; i < trajectory.states_size(); ++i) {
          velocityT[i] = velocity[i].x();
          if (velocity[i].x() > maxVelocity) {
            maxVelocity = velocity[i].x();
          }
          if (velocity[i].x() < minVelocity) {
            minVelocity = velocity[i].x();
          }
          timestamp[i] = trajectory.states(i).timestamp();
        }
        ImPlot::SetupAxisLimits(ImAxis_Y1, 1.3 * minVelocity, 1.3 * maxVelocity);
        ImPlot::PlotLine("Shoulder Velocity", timestamp, velocityT, trajectory.states_size());
        ImPlot::PlotLine("Velocity Time", simX.data(), simY.data(), 2);
        ImPlot::EndPlot();
      }
      if (ImPlot::BeginPlot("Elbow Velocity")) {
        ImPlot::SetupAxes("Time (s)", "Velocity (rad/s)", ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_None);
        double* velocityT = new double[trajectory.states_size()];
        double* timestamp = new double[trajectory.states_size()];
        double maxVelocity = 1, minVelocity = -1;
        for (int i = 0; i < trajectory.states_size(); ++i) {
          velocityT[i] = velocity[i].y();
          if (velocity[i].y() > maxVelocity) {
            maxVelocity = velocity[i].y();
          }
          if (velocity[i].y() < minVelocity) {
            minVelocity = velocity[i].y();
          }
          timestamp[i] = trajectory.states(i).timestamp();
        }
        ImPlot::SetupAxisLimits(ImAxis_Y1, 1.3 * minVelocity, 1.3 * maxVelocity);
        ImPlot::PlotLine("Elbow Velocity", timestamp, velocityT, trajectory.states_size());
        ImPlot::PlotLine("Velocity Time", simX.data(), simY.data(), 2);
        ImPlot::EndPlot();
      }
    }
    ImGui::End();

    /**
     * Window 4: 2D Projection
     */
    ImGui::Begin("2D Projection", nullptr, WINDOW_FLAGS);
    if (ImPlot::BeginPlot("2D Projection", "X (m)", "Z (m)", ImVec2(-1, 400), ImPlotFlags_Equal)) {
      nextinnovation::Object env = nextinnovation::Object(nextinnovation::ObjectType::ENV);
      nextinnovation::Object arm = nextinnovation::Object(armType).armTransform(simT, simR);
      nextinnovation::Object exp = nextinnovation::Object(expType).armTransform(simT, simR);
      for (nextinnovation::Polygon& polygon : env.getPolygons()) {
        for (int i = 0; i < polygon.getPoints().size(); ++i) {
          Eigen::Vector2d pts1 = polygon.getPoints()[i];
          Eigen::Vector2d pts2 = polygon.getPoints()[(i + 1) % polygon.getPoints().size()];
          double plotX[2] = {pts1(0), pts2(0)};
          double plotY[2] = {pts1(1), pts2(1)};
          ImPlot::PlotLine("obstacle", plotX, plotY, 2);
        }
      }
      for (nextinnovation::Polygon& polygon : arm.getPolygons()) {
        for (int i = 0; i < polygon.getPoints().size(); ++i) {
          Eigen::Vector2d pts1 = polygon.getPoints()[i];
          Eigen::Vector2d pts2 = polygon.getPoints()[(i + 1) % polygon.getPoints().size()];
          double plotX[2] = {pts1(0), pts2(0)};
          double plotY[2] = {pts1(1), pts2(1)};
          ImPlot::PlotLine("arm", plotX, plotY, 2);
        }
      }
      for (nextinnovation::Polygon& polygon : exp.getPolygons()) {
        for (int i = 0; i < polygon.getPoints().size(); ++i) {
          Eigen::Vector2d pts1 = polygon.getPoints()[i];
          Eigen::Vector2d pts2 = polygon.getPoints()[(i + 1) % polygon.getPoints().size()];
          double plotX[2] = {pts1(0), pts2(0)};
          double plotY[2] = {pts1(1), pts2(1)};
          ImPlot::PlotLine("expanded", plotX, plotY, 2);
        }
      }
      if (path.size() > 0) {
        arm = nextinnovation::Object(armType).armTransform(path[0](0), path[0](1));
        for (nextinnovation::Polygon& polygon : arm.getPolygons()) {
          for (int i = 0; i < polygon.getPoints().size(); ++i) {
            Eigen::Vector2d pts1 = polygon.getPoints()[i];
            Eigen::Vector2d pts2 = polygon.getPoints()[(i + 1) % polygon.getPoints().size()];
            double plotX[2] = {pts1(0), pts2(0)};
            double plotY[2] = {pts1(1), pts2(1)};
            ImPlot::PlotLine("target", plotX, plotY, 2);
          }
        }
        arm = nextinnovation::Object(armType).armTransform(path[path.size() - 1](0), path[path.size() - 1](1));
        for (nextinnovation::Polygon& polygon : arm.getPolygons()) {
          for (int i = 0; i < polygon.getPoints().size(); ++i) {
            Eigen::Vector2d pts1 = polygon.getPoints()[i];
            Eigen::Vector2d pts2 = polygon.getPoints()[(i + 1) % polygon.getPoints().size()];
            double plotX[2] = {pts1(0), pts2(0)};
            double plotY[2] = {pts1(1), pts2(1)};
            ImPlot::PlotLine("target", plotX, plotY, 2);
          }
        }
      }
      std::vector<double> elevatorX = {-ELEVATOR_2_L1_FRONT + ELEVATOR_MIN_POSITION_METER * ELEVATOR_COS_ANGLE, -ELEVATOR_2_L1_FRONT + ELEVATOR_MAX_POSITION_METER * ELEVATOR_COS_ANGLE};
      std::vector<double> elevatorY = {ELEVATOR_2_GROUND + ELEVATOR_MIN_POSITION_METER * ELEVATOR_SIN_ANGLE, ELEVATOR_2_GROUND + ELEVATOR_MAX_POSITION_METER * ELEVATOR_SIN_ANGLE};
      ImPlot::PlotLine("elevator", elevatorX.data(), elevatorY.data(), 2);
      ImPlot::EndPlot();
    }
    ImGui::End();

    // render the frame
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // mark the server as not running
  g_isRunning = false;

  // cleanup implot
  ImPlot::DestroyContext();

  // cleanup imgui
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  // cleanup glfw
  glfwDestroyWindow(window);
  glfwTerminate();

  // wait for the server thread
  if (serverThread.joinable()) {
    serverThread.join();
  }

  return 0;
}