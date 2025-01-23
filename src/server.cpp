#include <grpcpp/grpcpp.h>
#include "proto/ArmTrajectoryService.grpc.pb.h"
#include "log.hpp"

using namespace com::nextinnovation::armtrajectoryservice;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

class Service final : public ArmTrajectoryService::Service {
  Status generate(ServerContext* context, const ArmTrajectoryParameter* request,
                 Response* response) override {
    auto* trajectory = response->mutable_trajectory();
    *trajectory->mutable_parameter() = *request;
    
    for (int i = 0; i < 5; i++) {
      auto* state = trajectory->add_states();
      state->set_timestamp(i * 0.1);
      auto* pos = state->mutable_position();
      pos->set_shoulderheightmeter(0.5 + i * 0.1);
      pos->set_elbowpositiondegree(45.0 + i * 10.0);
    }
    return Status::OK;
  }
};

int main() {
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
  builder.AddListeningPort("0.0.0.0:50051", grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  builder.BuildAndStart()->Wait();
  
  return 0;
}