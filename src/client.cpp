#include <grpcpp/grpcpp.h>
#include "proto/ArmTrajectoryService.grpc.pb.h"
#include "log.hpp"

using namespace com::nextinnovation::armtrajectoryservice;
using grpc::Channel;
using grpc::ClientContext;

class Client {
public:
  explicit Client(const std::string& addr) 
    : stub_(ArmTrajectoryService::NewStub(
        grpc::CreateChannel(addr, grpc::InsecureChannelCredentials()))) {
    
    ArmTrajectoryParameter request;
    request.mutable_start()->set_shoulderheightmeter(0.5);
    request.mutable_start()->set_elbowpositiondegree(45.0);
    request.mutable_end()->set_shoulderheightmeter(1.0);
    request.mutable_end()->set_elbowpositiondegree(90.0);

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