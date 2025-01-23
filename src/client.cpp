#include <grpcpp/grpcpp.h>
#include "proto/ArmTrajectoryService.grpc.pb.h"
#include "log.hpp"

using namespace com::nextinnovation::armtrajectoryservice;
using grpc::Channel;
using grpc::ClientContext;

struct TrajectoryConfig {
  struct State {
    double shoulder;
    double elbow;
  };
  State start{0.5, 45.0};
  State end{1.0, 90.0};
  bool hasAlgae{true};
  bool hasCoral{false};
};

class Client {
public:
  explicit Client(const std::string& addr) 
    : stub_(ArmTrajectoryService::NewStub(
        grpc::CreateChannel(addr, grpc::InsecureChannelCredentials()))) {
    
    const TrajectoryConfig config;
    ArmTrajectoryParameter request;
    request.mutable_start()->set_shoulderheightmeter(config.start.shoulder);
    request.mutable_start()->set_elbowpositiondegree(config.start.elbow);
    request.mutable_end()->set_shoulderheightmeter(config.end.shoulder);
    request.mutable_end()->set_elbowpositiondegree(config.end.elbow);
    request.set_hasalgae(config.hasAlgae);
    request.set_hascoral(config.hasCoral);

    ClientContext context;
    Response response;
    
    if (auto status = stub_->generate(&context, request, &response); status.ok()) {
      if (response.has_trajectory()) {
        for (const auto& state : response.trajectory().states()) {
          log_info("t=%.2f, h=%.2f, θ=%.2f", 
            state.timestamp(),
            state.position().shoulderheightmeter(),
            state.position().elbowpositiondegree());
        }
      }
    }
  }

private:
  std::unique_ptr<ArmTrajectoryService::Stub> stub_;
};

int main() {
  Client("localhost:50051");
  return 0;
} 