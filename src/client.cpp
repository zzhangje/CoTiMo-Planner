#include <grpcpp/grpcpp.h>

#include "config.h"
#include "log.hpp"
#include "proto/ArmTrajectoryService.grpc.pb.h"

using com::nextinnovation::armtrajectoryservice::ArmTrajectoryParameter;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryService;
using com::nextinnovation::armtrajectoryservice::Response;
using grpc::Channel;
using grpc::ClientContext;

class Client {
 public:
  explicit Client(const std::string& addr)
      : stub_(ArmTrajectoryService::NewStub(
            grpc::CreateChannel(addr, grpc::InsecureChannelCredentials()))) {
    ArmTrajectoryParameter request;
    request.mutable_start()->set_shoulderheightmeter(0.5);
    request.mutable_start()->set_elbowpositiondegree(180.0);
    request.mutable_end()->set_shoulderheightmeter(1.0);
    request.mutable_end()->set_elbowpositiondegree(180.0);
    request.set_hasalgae(false);
    request.set_hascoral(false);

    ClientContext context;
    Response response;

    if (auto status = stub_->generate(&context, request, &response); status.ok()) {
      if (response.has_trajectory()) {
        for (const auto& state : response.trajectory().states()) {
          log_info("t=%.3f, h=%.2f, θ=%.2f, vel_h=%.2f, vel_θ=%.2f, V_h=%.2f, V_θ=%.2f",
                   state.timestamp(),
                   state.position().shoulderheightmeter(),
                   state.position().elbowpositiondegree(),
                   state.velocity().shouldervelocitymeterpersecond(),
                   state.velocity().elbowvelocitydegreepersecond(),
                   state.voltage().shouldervoltagevolt(),
                   state.voltage().elbowvoltagevolt());
        }
      }
    }
  }

 private:
  std::unique_ptr<ArmTrajectoryService::Stub> stub_;
};

int main() {
  Client("localhost:" + config::params::GRPC_PORT);
  return 0;
}