#include <grpcpp/grpcpp.h>

#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <string>

#include "config.h"
#include "log.hpp"
#include "map.hpp"
#include "proto/ArmTrajectoryService.grpc.pb.h"

using com::nextinnovation::armtrajectoryservice::ArmTrajectoryParameter;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryService;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryState;
using com::nextinnovation::armtrajectoryservice::Response;
using grpc::Channel;
using grpc::ClientContext;

using namespace config::alphabot;

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
        Eigen::Vector2i ij = getGridIdx(response.trajectory().parameter().start().shoulderheightmeter(), response.trajectory().parameter().start().elbowpositiondegree());
        Eigen::Vector2i tr = getGridIdx(response.trajectory().parameter().end().shoulderheightmeter(), response.trajectory().parameter().end().elbowpositiondegree());

        std::string filename = "cache/" + ROBOT_PARAMS_VERSION + "/" + (response.trajectory().parameter().hasalgae() ? "1" : "0") + (response.trajectory().parameter().hascoral() ? "0" : "1") + "_" + std::to_string(ij(0)) + "_" + std::to_string(ij(1)) + "_" + std::to_string(tr(0)) + "_" + std::to_string(tr(1)) + ".txt";
        std::ofstream file(filename);
        if (!file.is_open()) {
          log_error("Failed to open file: %s", filename.c_str());
          return;
        }
        for (auto state : states) {
          file << state.timestamp() << ' ' << state.position().shoulderheightmeter() << ' ' << state.position().elbowpositiondegree() << ' ' << state.velocity().shouldervelocitymeterpersecond() << ' ' << state.velocity().elbowvelocitydegreepersecond() << ' ' << state.voltage().shouldervoltagevolt() << ' ' << state.voltage().elbowvoltagevolt() << std::endl;
        }
        file.close();
        log_info("Saved trajectory to %s", filename.c_str());
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
  for (double i = ELEVATOR_MIN_POSITION_METER; i < ELEVATOR_MAX_POSITION_METER; i += ELEVATOR_GRID_SIZE) {
    for (double j = ARM_MIN_THETA_DEGREE; j < ARM_MAX_THETA_DEGREE; j += ARM_GRID_SIZE) {
      for (double t = ELEVATOR_MIN_POSITION_METER; t < ELEVATOR_MAX_POSITION_METER; t += ELEVATOR_GRID_SIZE) {
        for (double r = ARM_MIN_THETA_DEGREE; r < ARM_MAX_THETA_DEGREE; r += ARM_GRID_SIZE) {
          auto request = std::make_shared<ArmTrajectoryParameter>();
          request->mutable_start()->set_shoulderheightmeter(i);
          request->mutable_start()->set_elbowpositiondegree(j);
          request->mutable_end()->set_shoulderheightmeter(t);
          request->mutable_end()->set_elbowpositiondegree(r);
          request->set_hasalgae(true);
          request->set_hascoral(true);
          log_info("Sending request: start(%.2f, %.2f), end(%.2f, %.2f), algae(%d), coral(%d)",
                   i, j, t, r,
                   request->hasalgae(), request->hascoral());
          Client("localhost:" + config::params::GRPC_PORT, *request);
        }
      }
    }
  }
  return 0;
}