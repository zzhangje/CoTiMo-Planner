#include <grpcpp/grpcpp.h>
#include <matplot/matplot.h>

#include "config.h"
#include "log.hpp"
#include "map.hpp"
#include "proto/ArmTrajectoryService.grpc.pb.h"
#include "visualize.hpp"

using com::nextinnovation::armtrajectoryservice::ArmTrajectoryParameter;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryService;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryState;
using com::nextinnovation::armtrajectoryservice::Response;
using grpc::Channel;
using grpc::ClientContext;

class Client {
 public:
  explicit Client(const std::string& addr, const ArmTrajectoryParameter& request)
      : stub_(ArmTrajectoryService::NewStub(
            grpc::CreateChannel(addr, grpc::InsecureChannelCredentials()))) {
    ClientContext context;
    Response response;

    if (auto status = stub_->generate(&context, request, &response); status.ok()) {
      if (response.has_trajectory()) {
        auto states = response.trajectory().states();

        // print the trajectory
        for (const ArmTrajectoryState& state : states) {
          log_info("t=%.3f, h=%.2f, θ=%.2f, vel_h=%.2f, vel_θ=%.2f, V_h=%.2f, V_θ=%.2f",
                   state.timestamp(),
                   state.position().shoulderheightmeter(),
                   state.position().elbowpositiondegree(),
                   state.velocity().shouldervelocitymeterpersecond(),
                   state.velocity().elbowvelocitydegreepersecond(),
                   state.voltage().shouldervoltagevolt(),
                   state.voltage().elbowvoltagevolt());
        }
      } else {
        log_warn("The response does not contain the trajectory.");
      }
    } else {
      log_error("The response status is not OK: %s", status.error_message().c_str());
    }
  }

 private:
  std::unique_ptr<ArmTrajectoryService::Stub> stub_;
};

int main() {
  std::cout << "=== Press Ctrl+C to exit ===" << std::endl
            << std::endl;

  while (1) {
    auto request = std::make_shared<ArmTrajectoryParameter>();

    std::cout << "$ Please enter the shoulder height in meters: ";
    double shoulderHeight;
    std::cin >> shoulderHeight;
    request->mutable_start()->set_shoulderheightmeter(shoulderHeight);

    std::cout << "$ Please enter the elbow position in degrees: ";
    double elbowPosition;
    std::cin >> elbowPosition;
    request->mutable_start()->set_elbowpositiondegree(elbowPosition);

    std::cout << "$ Please enter the end shoulder height in meters: ";
    double endShoulderHeight;
    std::cin >> endShoulderHeight;
    request->mutable_end()->set_shoulderheightmeter(endShoulderHeight);

    std::cout << "$ Please enter the end elbow position in degrees: ";
    double endElbowPosition;
    std::cin >> endElbowPosition;
    request->mutable_end()->set_elbowpositiondegree(endElbowPosition);

    request->set_hasalgae(false);
    request->set_hascoral(false);

    log_info("Sending request: start(%.2f, %.2f), end(%.2f, %.2f), algae(%d), coral(%d)",
             shoulderHeight, elbowPosition, endShoulderHeight, endElbowPosition,
             request->hasalgae(), request->hascoral());
    Client("localhost:" + config::params::GRPC_PORT, *request);
  }
  return 0;
}