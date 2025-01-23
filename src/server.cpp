#include <grpcpp/grpcpp.h>

#include <csignal>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <memory>
#include <string>

#include "Topp.hpp"
#include "log.hpp"
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

  std::vector<Eigen::Vector2d> points;
  points.push_back(Eigen::Vector2d(0, 0));
  points.push_back(Eigen::Vector2d(1, 1));
  points.push_back(Eigen::Vector2d(2, 2));
  points.push_back(Eigen::Vector2d(3, 3));
  points.push_back(Eigen::Vector2d(4, 4));
  points.push_back(Eigen::Vector2d(5, 5));

  Topp topp(points);

  while (!signalStatus) {
  }

  log_info("Shutting down Cyber Planner 2025...");

  // Do some cleanup here

  log_info("Cyber Planner 2025 has been shut down.");
  return 0;
}