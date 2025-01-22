#include <grpcpp/grpcpp.h>

#include <csignal>
#include <iostream>
#include <memory>
#include <string>

#include "ArmTrajectoryService.grpc.pb.h"
#include "Logger.hpp"

using com::nextinnovation::armtrajectoryservice::ArmTrajectoryParameter;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryService;
using com::nextinnovation::armtrajectoryservice::Response;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

volatile std::sig_atomic_t signalStatus = 0;
void signalHandler(int signal) {
  Logger::getInstance()->log(LogLevel::WARN, "Signal Handler",
                             "Received signal, shutting down...");
  signalStatus = 1;
}

class ArmTrajectoryServiceImpl final : public ArmTrajectoryService::Service {
  Status generate(ServerContext *context, const ArmTrajectoryParameter *request,
                  Response *response) override {
    Logger::getInstance()->log(
        LogLevel::INFO, "gRPC Server",
        "Received a request to generate arm trajectory.");
    // Do some processing here

    return Status::OK;
  }
};

void runServer() {
  std::string serverAddress("0.0.0.0:50051");
  ArmTrajectoryServiceImpl service;

  //   ServerBuilder builder;
  //   builder.AddListeningPort(serverAddress,
  //   grpc::InsecureServerCredentials()); builder.RegisterService(&service);

  //   std::unique_ptr<Server> server(builder.BuildAndStart());
  //   Logger::getInstance()->log(LogLevel::INFO, "gRPC Server",
  //                              "Service listening on " + serverAddress);

  //   server->Wait();
}

int main(int argc, char **argv) {
  Logger::getInstance()->log(
      LogLevel::INFO, "Cyber Planner",
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
  runServer();

  Logger::getInstance()->log(
      LogLevel::INFO, "Cyber Planner",
      "Cyber Planner 2025 is running, press Ctrl+C to exit.");

  while (!signalStatus) {
  }

  Logger::getInstance()->log(LogLevel::INFO, "Cyber Planner",
                             "Shutting down Cyber Planner 2025...");

  // Do some cleanup here

  Logger::getInstance()->log(
      LogLevel::INFO, "Cyber Planner",
      "Program terminated successfully, logs are saved in " +
          Logger::getInstance()->getLogFileName());
  return 0;
}