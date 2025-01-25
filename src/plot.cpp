#include <grpcpp/grpcpp.h>

#include <iostream>

#include "proto/ArmTrajectoryService.grpc.pb.h"

using com::nextinnovation::armtrajectoryservice::ArmPositionState;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryParameter;
using com::nextinnovation::armtrajectoryservice::ArmTrajectoryService;
using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

int main() {
  std::cout << "Hello, World!" << std::endl;
  std::cout << "This is a simple C++ program." << std::endl;
  return 0;
}