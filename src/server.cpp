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
    
    const auto& start = request->start();
    const auto& end = request->end();
    
    for (int i = 0; i < 5; i++) {
      double t = i / 4.0;
      auto* state = trajectory->add_states();
      state->set_timestamp(t);
      auto* pos = state->mutable_position();
      pos->set_shoulderheightmeter(start.shoulderheightmeter() + (end.shoulderheightmeter() - start.shoulderheightmeter()) * t);
      pos->set_elbowpositiondegree(start.elbowpositiondegree() + (end.elbowpositiondegree() - start.elbowpositiondegree()) * t);
      
      auto* current = state->mutable_current();
      current->set_shouldercurrentamp(1.0);
      current->set_elbowcurrentamp(0.8);
    }
    log_info("Generated trajectory with %d points", trajectory->states_size());
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
  log_info("Server is running on 0.0.0.0:50051");
  builder.BuildAndStart()->Wait();
  
  return 0;
}