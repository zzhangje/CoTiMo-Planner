#include <grpcpp/grpcpp.h>

#include "log.hpp"
#include "proto/ArmTrajectoryService.grpc.pb.h"

#define SERVER_ADDRESS "0.0.0.0:58214"

using com::nextinnovation::armtrajectoryservice::ArmTrajectoryParameter;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryService;
using com::nextinnovation::armtrajectoryservice::Response;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

class ArmTrajectoryServiceImpl final : public ArmTrajectoryService::Service {
  Status generate(ServerContext *context, const ArmTrajectoryParameter *request,
                  Response *response) override {
    log_info("gRPC Server Received a request.");
    // Do some processing here

    return Status::OK;
  }
};

/**
 * @warning The function will block current thread
 */
void runServer() {
  ArmTrajectoryServiceImpl service;

  ServerBuilder builder;
  builder.AddListeningPort(SERVER_ADDRESS,
                           grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  std::unique_ptr<Server> server(builder.BuildAndStart());
  log_info(("gRPC Server listening on " + std::string(SERVER_ADDRESS)).c_str());

  // server->Wait();
}