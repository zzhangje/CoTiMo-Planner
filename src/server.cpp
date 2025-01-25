#include <grpcpp/grpcpp.h>
#include <windows.h>

#include <atomic>
#include <eigen3/Eigen/Eigen>
#include <mutex>
#include <thread>

#include "Object.hpp"
#include "Topp.hpp"
#include "astar.hpp"
#include "config.h"
#include "log.hpp"
#include "map.hpp"
#include "proto/ArmTrajectoryService.grpc.pb.h"

using com::nextinnovation::armtrajectoryservice::ArmPositionState;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryParameter;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryService;
using google::protobuf::RepeatedPtrField;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

RepeatedPtrField<ArmTrajectoryState>* sharedTrajectory = new RepeatedPtrField<ArmTrajectoryState>();
bool hasNewTrajectory = false;
std::mutex trajectoryMutex;

class Service final : public ArmTrajectoryService::Service {
  Status generate(ServerContext* context, const ArmTrajectoryParameter* request,
                  Response* response) override {
    const ArmPositionState& start = request->start();
    const ArmPositionState& end = request->end();
    log_info("Received request: start(%.2f, %.2f), end(%.2f, %.2f)",
             start.shoulderheightmeter(), start.elbowpositiondegree(),
             end.shoulderheightmeter(), end.elbowpositiondegree());

    Eigen::Vector2d startTR(start.shoulderheightmeter(), start.elbowpositiondegree());
    Eigen::Vector2d endTR(end.shoulderheightmeter(), end.elbowpositiondegree());
    Eigen::Vector2i startGridIdx = getGridIdx(startTR);
    Eigen::Vector2i endGridIdx = getGridIdx(endTR);
    log_info("Start grid index: (%d, %d), end grid index: (%d, %d)",
             startGridIdx(0), startGridIdx(1), endGridIdx(0), endGridIdx(1));

    ObjectType armType, expType;
    if (request->hasalgae() && request->hascoral()) {
      armType = ObjectType::ARM_ALGAE_CORAL;
      expType = ObjectType::ARM_EXP_ALGAE_CORAL;
    } else if (request->hasalgae()) {
      armType = ObjectType::ARM_ALGAE;
      expType = ObjectType::ARM_EXP_ALGAE;
    } else if (request->hascoral()) {
      armType = ObjectType::ARM_CORAL;
      expType = ObjectType::ARM_EXP_CORAL;
    } else {
      armType = ObjectType::ARM;
      expType = ObjectType::ARM_EXP;
    }

    std::vector<std::vector<double>> grid;
    getGridMap(expType, grid);
    std::vector<Eigen::Vector2i> path, visited, sampledPath;
    if (!astar::astar(grid, startGridIdx, endGridIdx, path, visited)) {
      log_warn("Failed to find a path in the expanded map.");
      getGridMap(armType, grid);
      if (!astar::astar(grid, startGridIdx, endGridIdx, path, visited)) {
        log_error("Failed to find a path in the original map.");
        return Status::CANCELLED;
      }
    }
    log_info("Found a path with %d points.", path.size());
    // astar::samplePath(path, sampledPath, 1);
    sampledPath = path;

    ArmTrajectory* trajectory = response->mutable_trajectory();
    Topp topp(getTRs(sampledPath));
    topp.getTrajectory(trajectory);
    *trajectory->mutable_parameter() = *request;

    {
      std::unique_lock<std::mutex> lock(trajectoryMutex);
      sharedTrajectory->CopyFrom(trajectory->states());
      hasNewTrajectory = true;
    }

    log_info("Generated trajectory with %d points", trajectory->states_size());
    return Status::OK;
  }
};

int main(int argc, char* argv[]) {
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

  builder.AddChannelArgument(GRPC_ARG_MAX_CONCURRENT_STREAMS, 1);
  builder.AddListeningPort("0.0.0.0:" + config::params::GRPC_PORT, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  log_info(("Server is running on 0.0.0.0:" + config::params::GRPC_PORT).c_str());
  builder.BuildAndStart()->Wait();

  return 0;
}