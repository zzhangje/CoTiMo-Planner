#include <grpcpp/grpcpp.h>

#include <csignal>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <memory>
#include <string>

#include "Object.hpp"
#include "Topp.hpp"
#include "Toppp.hpp"
#include "astar.hpp"
#include "log.hpp"
#include "map.hpp"
#include "proto/ArmTrajectoryService.grpc.pb.h"

using com::nextinnovation::armtrajectoryservice::ArmTrajectoryParameter;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryService;
using com::nextinnovation::armtrajectoryservice::Response;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

volatile std::sig_atomic_t signalStatus = 0;
void signalHandler(int signal) {
  log_debug(("Received signal " + std::to_string(signal) + ".").c_str());
  signalStatus = 1;
}

int main(int argc, char **argv) {
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

  std::signal(SIGINT, signalHandler);

  log_info("Cyber Planner 2025 is running, press Ctrl+C to exit.");

  std::vector<std::vector<double>> map;
  std::vector<Eigen::Vector2i> path, visited, samplePath;
  getGridMap(ObjectType::ARM, map);
  astar::astar(map, Eigen::Vector2i(0, 0), Eigen::Vector2i(60, 20), path, visited);
  // astar::samplePath(path, samplePath, 3);

  std::vector<Eigen::Vector2d> points;
  for (int i = 0; i < 10; ++i) {
    points.push_back(Eigen::Vector2d(i, i));
  }
  // Toppp toppp(points);
  // for (int i = 0; i < 30; ++i) {
  //   toppp.step();
  // }

  Topp topp(points);

  while (!signalStatus) {
  }

  log_info("Shutting down Cyber Planner 2025...");

  // Do some cleanup here

  log_info("Cyber Planner 2025 has been shut down.");
  return 0;
}