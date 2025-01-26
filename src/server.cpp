// #include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <grpcpp/grpcpp.h>
#include <implot.h>
#include <windows.h>

#include <Eigen/Eigen>
#include <atomic>
#include <chrono>
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
#include "proto/ArmTrajectoryService.grpc.pb.h"
#include "space.hpp"
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

using namespace config::alphabot;

// shared variables
ArmTrajectory* g_trajectory = new ArmTrajectory();
bool g_hasNewTrajectory = false;
bool g_isRunning = true;
std::mutex g_trajectoryMutex;

// redirect stdout to imgui stream
std::stringstream g_ss;
std::streambuf* g_coutbuf = std::cout.rdbuf();
std::streambuf* g_cerrbuf = std::cerr.rdbuf();
std::deque<std::string> consoleOutput;
const size_t consoleOutputSize = 500;

void RedirectStdout() {
  g_coutbuf = std::cout.rdbuf();
  std::cout.rdbuf(g_ss.rdbuf());
  std::streambuf* g_cerrbuf = std::cerr.rdbuf();
  std::cerr.rdbuf(g_ss.rdbuf());
}
void RestoreStdout() {
  std::cout.rdbuf(g_coutbuf);
  std::cerr.rdbuf(g_cerrbuf);
}

// gRPC service
class Service final : public ArmTrajectoryService::Service {
  Status generate(ServerContext* context, const ArmTrajectoryParameter* request,
                  Response* response) override {
    const ArmPositionState& start = request->start();
    const ArmPositionState& end = request->end();
    log_info("Received request: start(%.2f, %.2f), end(%.2f, %.2f)",
             start.shoulderheightmeter(), start.elbowpositiondegree(),
             end.shoulderheightmeter(), end.elbowpositiondegree());

    // get the grid index of the start and end points
    Eigen::Vector2d startTR(start.shoulderheightmeter(), start.elbowpositiondegree());
    Eigen::Vector2d endTR(end.shoulderheightmeter(), end.elbowpositiondegree());
    Eigen::Vector2i startGridIdx = getGridIdx(startTR);
    Eigen::Vector2i endGridIdx = getGridIdx(endTR);
    log_info("Start grid index: (%d, %d), end grid index: (%d, %d)",
             startGridIdx(0), startGridIdx(1), endGridIdx(0), endGridIdx(1));

    // judge the type of the arm
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

    // find the path in the grid map
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
    astar::samplePath(path, sampledPath, 17);
    sampledPath = path;

    // generate the trajectory
    ArmTrajectory* trajectory = response->mutable_trajectory();
    Topp topp(getTRs(sampledPath));
    topp.getTrajectory(trajectory);
    *trajectory->mutable_parameter() = *request;

    // write the trajectory to the shared variable
    {
      std::unique_lock<std::mutex> lock(g_trajectoryMutex);
      g_trajectory->CopyFrom(*trajectory);
      g_hasNewTrajectory = true;
    }

    log_info("Generated trajectory with %d points", trajectory->states_size());
    return Status::OK;
  }
};

void RunGrpcServer() {
  std::string serverAddress("0.0.0.0:" + config::params::GRPC_PORT);
  Service service;

  ServerBuilder builder;
  builder.AddChannelArgument(GRPC_ARG_MAX_CONCURRENT_STREAMS, 1);
  builder.AddListeningPort(serverAddress, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  std::unique_ptr<Server> server(builder.BuildAndStart());
  log_info(("Server is running on 0.0.0.0:" + config::params::GRPC_PORT).c_str());

  while (g_isRunning) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  server->Shutdown();
  log_info("Server is shutting down.");
}

// main thread
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
  // redirect stdout
  RedirectStdout();
  log_set_quiet(false);
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
    log_error("Failed to initialize GLFW.");
    return -1;
  }

  // create window
  GLFWwindow* window = glfwCreateWindow(1280, 720, "Cyber Planner 2025", NULL, NULL);
  if (!window) {
    log_error("Failed to create window.");
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize glew
  // glewExperimental = GL_TRUE;
  // if (glewInit() != GLEW_OK) {
  //   log_error("Failed to initialize GLEW.");
  //   return -1;
  // }

  // initialize imgui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;
  ImGui::StyleColorsClassic();

  // initialize imgui for glfw
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 150");

  // initialize implot
  ImPlot::CreateContext();

  // run gRPC server
  std::thread serverThread(RunGrpcServer);

  // thread variables copy
  ArmTrajectory trajectory;

  std::vector<std::vector<double>> emap, amap;
  std::vector<Eigen::Vector2d> path;
  ObjectType armType, expType;
  double t = 0, r = 0;
  int simIndex = 0;
  auto simStart = std::chrono::high_resolution_clock::now();

  // main loop
  while (!glfwWindowShouldClose(window)) {
    if (!serverThread.joinable()) {
      MessageBox(NULL, "Server thread is not joinable!", "Error!", MB_OK | MB_ICONERROR);
    }

    // start the frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // handle outstream
    std::string line;
    while (std::getline(g_ss, line)) {
      consoleOutput.push_back(line);
      if (consoleOutput.size() > consoleOutputSize) {
        consoleOutput.pop_front();
      }
    }
    g_ss.str(std::string());
    g_ss.clear();

    // handle new trajectory
    if (g_hasNewTrajectory) {
      {
        g_hasNewTrajectory = false;
        trajectory.CopyFrom(*g_trajectory);
      }

      // update map
      if (trajectory.parameter().hasalgae() && trajectory.parameter().hascoral()) {
        armType = ObjectType::ARM_ALGAE_CORAL;
        expType = ObjectType::ARM_EXP_ALGAE_CORAL;
      } else if (trajectory.parameter().hasalgae()) {
        armType = ObjectType::ARM_ALGAE;
        expType = ObjectType::ARM_EXP_ALGAE;
      } else if (trajectory.parameter().hascoral()) {
        armType = ObjectType::ARM_CORAL;
        expType = ObjectType::ARM_EXP_CORAL;
      } else {
        armType = ObjectType::ARM;
        expType = ObjectType::ARM_EXP;
      }
      getGridMap(armType, amap);
      getGridMap(expType, emap);

      path.clear();
      for (const ArmTrajectoryState& state : trajectory.states()) {
        path.push_back(Eigen::Vector2d(state.position().shoulderheightmeter(),
                                       state.position().elbowpositiondegree()));
      }
    }

    // handle simulation
    auto simNow = std::chrono::high_resolution_clock::now();
    if (path.size() > 0) {
      double simTime = std::chrono::duration_cast<std::chrono::milliseconds>(simNow - simStart).count() / 1000.0;
      if (simTime > trajectory.states(simIndex + 1).timestamp()) {
        simIndex++;
        if (simIndex >= trajectory.states_size() - 1) {
          simIndex = 0;
          simStart = simNow;
        }
      }
      double dt = simTime - trajectory.states(simIndex).timestamp();
      double factor = dt / (trajectory.states(simIndex + 1).timestamp() - trajectory.states(simIndex).timestamp());
      t = trajectory.states(simIndex).position().shoulderheightmeter() + factor *
                                                                             (trajectory.states(simIndex + 1).position().shoulderheightmeter() - trajectory.states(simIndex).position().shoulderheightmeter());
      r = trajectory.states(simIndex).position().elbowpositiondegree() + factor *
                                                                             (trajectory.states(simIndex + 1).position().elbowpositiondegree() - trajectory.states(simIndex).position().elbowpositiondegree());
    }

    /**
     * Window 1: Console Log
     */
    ImGui::Begin("Console Output");
    ImGui::BeginChild("console", ImVec2(0, -ImGui::GetFrameHeightWithSpacing()), true);
    for (const std::string& line : consoleOutput) {
      ImGui::TextUnformatted(line.c_str());
    }
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
      ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();
    ImGui::End();

    /**
     * Window 2: Arm Trajectory
     */
    ImGui::Begin("Arm Trajectory");
    if (ImPlot::BeginPlot("Configuration Space", "Shoulder Height (m)", "Elbow Position (degree)")) {
      ImPlot::SetupAxisLimits(ImAxis_X1, ELEVATOR_MIN_POSITION_METER, ELEVATOR_MAX_POSITION_METER);
      ImPlot::SetupAxisLimits(ImAxis_Y1, ARM_MIN_THETA_DEGREE, ARM_MAX_THETA_DEGREE);

      std::vector<double> obsX, obsY, expX, expY;
      for (int t = 0; t < emap.size(); t++) {
        for (int r = 0; r < emap[t].size(); r++) {
          if (amap[t][r] > config::params::OBSTACLE_OFFSET - .01) {
            Eigen::Vector2d tr = getTR(t, r);
            obsX.push_back(tr(0));
            obsY.push_back(tr(1));
          } else if (emap[t][r] > config::params::OBSTACLE_OFFSET - .01) {
            Eigen::Vector2d tr = getTR(t, r);
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

      double currentX[] = {t};
      double currentY[] = {r};
      ImPlot::PlotScatter("current", currentX, currentY, 1);
      ImPlot::EndPlot();
    }
    ImGui::End();

    /**
     * Window 3: Trajectory Params
     */
    ImGui::Begin("Trajectory Params");
    if (ImPlot::BeginPlot("Shoulder Velocity", "Time (s)", "Velocity (m/s)")) {
      ImPlot::SetupAxisLimits(ImAxis_Y1, -1.2 * ELEVATOR_VMAX, 1.2 * ELEVATOR_VMAX);
      double* velocityT = new double[trajectory.states_size()];
      double* timestamp = new double[trajectory.states_size()];
      for (int i = 0; i < trajectory.states_size(); ++i) {
        velocityT[i] = trajectory.states(i).velocity().shouldervelocitymeterpersecond();
        timestamp[i] = trajectory.states(i).timestamp();
      }
      ImPlot::PlotLine("Shoulder Velocity", timestamp, velocityT, trajectory.states_size());
      std::vector<double> vmaxX = {0, timestamp[trajectory.states_size() - 1]};
      std::vector<double> vmaxY = {ELEVATOR_VMAX, ELEVATOR_VMAX};
      std::vector<double> vminY = {-ELEVATOR_VMAX, -ELEVATOR_VMAX};
      ImPlot::PlotLine("Shoulder Max Velocity", vmaxX.data(), vmaxY.data(), 2);
      ImPlot::PlotLine("Shoulder Min Velocity", vmaxX.data(), vminY.data(), 2);
      ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Elbow Velocity", "Time (s)", "Velocity (degree/s)")) {
      ImPlot::SetupAxisLimits(ImAxis_Y1, -1.2 * ARM_VMAX, 1.2 * ARM_VMAX);
      double* velocityT = new double[trajectory.states_size()];
      double* timestamp = new double[trajectory.states_size()];
      for (int i = 0; i < trajectory.states_size(); ++i) {
        velocityT[i] = trajectory.states(i).velocity().elbowvelocitydegreepersecond();
        timestamp[i] = trajectory.states(i).timestamp();
      }
      ImPlot::PlotLine("Elbow Velocity", timestamp, velocityT, trajectory.states_size());
      std::vector<double> vmaxX = {0, timestamp[trajectory.states_size() - 1]};
      std::vector<double> vmaxY = {ARM_VMAX, ARM_VMAX};
      std::vector<double> vminY = {-ARM_VMAX, -ARM_VMAX};
      ImPlot::PlotLine("Elbow Max Velocity", vmaxX.data(), vmaxY.data(), 2);
      ImPlot::PlotLine("Elbow Min Velocity", vmaxX.data(), vminY.data(), 2);
      ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Voltage", "Time (s)", "Voltage (V)")) {
      ImPlot::SetupAxisLimits(ImAxis_Y1, -1.2 * ARM_MAX_VOLTAGE, 1.2 * ARM_MAX_VOLTAGE);
      double* elbowVoltageT = new double[trajectory.states_size()];
      double* shoulderVoltageT = new double[trajectory.states_size()];
      double* timestamp = new double[trajectory.states_size()];
      for (int i = 0; i < trajectory.states_size(); ++i) {
        timestamp[i] = trajectory.states(i).timestamp();
        shoulderVoltageT[i] = trajectory.states(i).voltage().shouldervoltagevolt();
        elbowVoltageT[i] = trajectory.states(i).voltage().elbowvoltagevolt();
      }
      ImPlot::PlotLine("Shoulder Voltage", timestamp, shoulderVoltageT, trajectory.states_size());
      ImPlot::PlotLine("Elbow Voltage", timestamp, elbowVoltageT, trajectory.states_size());
      std::vector<double> vmaxX = {0, timestamp[trajectory.states_size() - 1]};
      std::vector<double> vmaxY = {ARM_MAX_VOLTAGE, ARM_MAX_VOLTAGE};
      std::vector<double> vminY = {-ARM_MAX_VOLTAGE, -ARM_MAX_VOLTAGE};
      ImPlot::PlotLine("Max Voltage", vmaxX.data(), vmaxY.data(), 2);
      ImPlot::PlotLine("Min Voltage", vmaxX.data(), vminY.data(), 2);
      ImPlot::EndPlot();
    }
    ImGui::End();

    /**
     * Window 4: 2D Projection
     */
    ImGui::Begin("2D Projection");
    if (ImPlot::BeginPlot("2D Projection", "X", "Z")) {
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

  // restore stdout
  RestoreStdout();

  return 0;
}