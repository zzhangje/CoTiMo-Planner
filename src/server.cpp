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
ArmTrajectory* sharedTrajectory = new ArmTrajectory();
bool hasNewTrajectory = false;
int requestCount = 0;
std::mutex trajectoryMutex;

// redirect stdout to imgui stream
std::stringstream g_ss;
std::streambuf* g_coutbuf = std::cout.rdbuf();
std::streambuf* g_cerrbuf = std::cerr.rdbuf();
std::deque<std::string> consoleOutput;
const size_t consoleOutputSize = 500;

void RedirectStdout() {
  g_coutbuf = std::cout.rdbuf();
  std::cout.rdbuf(g_ss.rdbuf());
}

void RestoreStdout() {
  std::cout.rdbuf(g_coutbuf);
}

void RedirectStderr() {
  std::streambuf* g_cerrbuf = std::cerr.rdbuf();
  std::cerr.rdbuf(g_ss.rdbuf());
}

void RestoreStderr() {
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
    astar::samplePath(path, sampledPath, 4);
    sampledPath = path;

    // generate the trajectory
    ArmTrajectory* trajectory = response->mutable_trajectory();
    Topp topp(getTRs(sampledPath));
    topp.getTrajectory(trajectory);
    *trajectory->mutable_parameter() = *request;

    // write the trajectory to the shared variable
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

void RunGrpcServer() {
  std::string serverAddress("0.0.0.0:" + config::params::GRPC_PORT);
  Service service;

  ServerBuilder builder;
  builder.AddChannelArgument(GRPC_ARG_MAX_CONCURRENT_STREAMS, 1);
  builder.AddListeningPort(serverAddress, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  std::unique_ptr<Server> server(builder.BuildAndStart());
  log_info(("Server is running on 0.0.0.0:" + config::params::GRPC_PORT).c_str());

  server->Wait();
}

// window process function
extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
IMGUI_IMPL_API void ImGui_ImplWin32_Shutdown();

LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam) {
  if (ImGui_ImplWin32_WndProcHandler(hwnd, uMsg, wParam, lParam)) {
    return true;
  }
  switch (uMsg) {
    case WM_CLOSE:
      PostQuitMessage(0);
      return 0;
    case WM_DESTROY:
      PostQuitMessage(0);
      return 0;
  }
  return DefWindowProc(hwnd, uMsg, wParam, lParam);
}

// main thread
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow) {
  // redirect stdout
  RedirectStdout();
  RedirectStderr();
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

  // initialize GLFW
  if (!glfwInit()) {
    log_error("Failed to initialize GLFW.");
    return -1;
  }

  // create window
  GLFWwindow* window = glfwCreateWindow(1280, 720, "Cyber Planner 2025", NULL, NULL);
  if (window == NULL) {
    log_error("Failed to create GLFW window.");
    glfwTerminate();
    return -1;
  }

  // initialize ImGui
  glfwMakeContextCurrent(window);
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  (void)io;
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 150");

  // initialize ImPlot
  ImPlot::CreateContext();

  // run gRPC server
  std::thread serverThread(RunGrpcServer);

  // register window process function
  WNDCLASS wc = {};
  wc.lpfnWndProc = WindowProc;
  wc.hInstance = GetModuleHandle(NULL);
  wc.lpszClassName = "CyberPlanner2025";
  RegisterClass(&wc);

  // create window
  HWND hwnd = CreateWindow(wc.lpszClassName, "Cyber Planner 2025", WS_OVERLAPPEDWINDOW, 100, 100, 1280, 720, NULL, NULL,
                           wc.hInstance, NULL);
  if (hwnd == NULL) {
    MessageBox(NULL, "Window Creation Failed!", "Error!", MB_OK | MB_ICONERROR);
    return -1;
  }

  // main loop
  MSG msg;
  while (true) {
    if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
      if (msg.message == WM_QUIT) {
        break;
      }
      TranslateMessage(&msg);
      DispatchMessage(&msg);
    }

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

    // console window
    ImGui::Begin("Console Output");
    ImGui::BeginChild("console", ImVec2(0, -ImGui::GetFrameHeightWithSpacing()), true);
    for (const std::string& line : consoleOutput) {
      ImGui::TextUnformatted(line.c_str());
    }
    ImGui::EndChild();
    ImGui::End();

    std::cout << "time" << std::endl;
    std::cout << std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) << std::endl;
    log_info("asdfa");

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

  // cleanup
  ImPlot::DestroyContext();

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui_ImplWin32_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  // wait for the server thread
  // serverThread.join();

  // restore stdout
  RestoreStdout();
  RestoreStderr();

  return 0;
}