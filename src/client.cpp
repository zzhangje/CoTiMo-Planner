#include <grpcpp/grpcpp.h>

#include "config.h"
#include "log.hpp"
#include "proto/ArmTrajectoryService.grpc.pb.h"
#include "space.hpp"

using com::nextinnovation::armtrajectoryservice::ArmTrajectoryParameter;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryService;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryState;
using com::nextinnovation::armtrajectoryservice::Response;
using grpc::Channel;
using grpc::ClientContext;

using namespace nextinnovation::alphabot;

class Client {
 public:
  explicit Client(const std::string& addr, const ArmTrajectoryParameter& request)
      : stub_(ArmTrajectoryService::NewStub(
            grpc::CreateChannel(addr, grpc::InsecureChannelCredentials()))) {
    auto start = std::chrono::high_resolution_clock::now();

    ClientContext context;
    Response response;

    if (auto status = stub_->generate(&context, request, &response); status.ok()) {
      if (response.has_trajectory()) {
        auto states = response.trajectory().states();

        // print the trajectory
        for (const ArmTrajectoryState& state : states) {
          log_info("t=%.3f, h=%.2f, theta=%.2f, I_h=%.2f, I_theta=%.2f",
                   state.timestamp(),
                   state.position().shoulderheightmeter(),
                   state.position().elbowpositionradian(),
                   state.current().shouldercurrentampere(),
                   state.current().elbowcurrentampere());
        }
      } else {
        log_warn("The response does not contain the trajectory.");
      }
    } else {
      log_error("The response status is not OK: %s", status.error_message().c_str());
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    log_info("Total duration: %d,%dms", duration.count() / 1000, duration.count() % 1000);
  }

 private:
  std::unique_ptr<ArmTrajectoryService::Stub> stub_;
};

int main() {
  while (1) {
    auto request = std::make_shared<ArmTrajectoryParameter>();

    std::cout << "$ Please enter the shoulder height in meters, (" << ELEVATOR_MIN_POSITION_METER << ',' << ELEVATOR_MAX_POSITION_METER << "): ";
    double shoulderHeight;
    std::cin >> shoulderHeight;
    request->mutable_start()->set_shoulderheightmeter(shoulderHeight);

    std::cout << "$ Please enter the elbow position in degrees, (" << ARM_MIN_THETA_RADIAN << ',' << ARM_MAX_THETA_RADIAN << "): ";
    double elbowPosition;
    std::cin >> elbowPosition;
    request->mutable_start()->set_elbowpositionradian(elbowPosition);

    std::cout << "$ Please enter the end shoulder height in meters, (" << ELEVATOR_MIN_POSITION_METER << ',' << ELEVATOR_MAX_POSITION_METER << "): ";
    double endShoulderHeight;
    std::cin >> endShoulderHeight;
    request->mutable_end()->set_shoulderheightmeter(endShoulderHeight);

    std::cout << "$ Please enter the end elbow position in degrees, (" << ARM_MIN_THETA_RADIAN << ',' << ARM_MAX_THETA_RADIAN << "): ";
    double endElbowPosition;
    std::cin >> endElbowPosition;
    request->mutable_end()->set_elbowpositionradian(endElbowPosition);

    request->set_hasalgae(false);
    request->set_hascoral(false);

    log_info("Sending request: start(%.2f, %.2f), end(%.2f, %.2f), algae(%d), coral(%d)",
             shoulderHeight, elbowPosition, endShoulderHeight, endElbowPosition,
             request->hasalgae(), request->hascoral());
    Client("localhost:" + nextinnovation::config::GRPC_PORT, *request);
    std::cout << std::endl
              << "=== Press Ctrl+C to exit ===" << std::endl
              << std::endl;
  }
  return 0;
}