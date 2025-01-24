#include <grpcpp/grpcpp.h>
#include <thread>
#include <vector>
#include <chrono>

#include "config.h"
#include "log.hpp"
#include "proto/ArmTrajectoryService.grpc.pb.h"

using namespace com::nextinnovation::armtrajectoryservice;
using grpc::Channel;
using grpc::ClientContext;
using namespace config::dynamic;

struct TrajectoryConfig {
  struct State {
    double shoulder;
    double elbow;
  };
  State start{0.5, 180.0};
  State end{1.0, 180.0};
  bool hasAlgae{true};
  bool hasCoral{false};
};

class Client {
 public:
  explicit Client(const std::string& addr)
      : stub_(ArmTrajectoryService::NewStub(
            grpc::CreateChannel(addr, grpc::InsecureChannelCredentials()))) {}

  void sendRequest(const TrajectoryConfig& config, int requestId) {
    ArmTrajectoryParameter request;
    request.mutable_start()->set_shoulderheightmeter(config.start.shoulder);
    request.mutable_start()->set_elbowpositiondegree(config.start.elbow);
    request.mutable_end()->set_shoulderheightmeter(config.end.shoulder);
    request.mutable_end()->set_elbowpositiondegree(config.end.elbow);
    request.set_hasalgae(config.hasAlgae);
    request.set_hascoral(config.hasCoral);

    ClientContext context;
    Response response;

    auto start = std::chrono::high_resolution_clock::now();
    auto status = stub_->generate(&context, request, &response);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    if (status.ok()) {
      if (response.has_trajectory()) {
        log_info("Request %d completed in %lld ms with %d states", 
                requestId, duration.count(), response.trajectory().states_size());
      }
    } else {
      log_info("Request %d failed: %s", requestId, status.error_message().c_str());
    }
  }

  void runConcurrentTests(int numRequests) {
    std::vector<std::thread> threads;
    std::vector<TrajectoryConfig> configs;

    for (int i = 0; i < numRequests; i++) {
      TrajectoryConfig config;
      config.start.shoulder = 0.5 + (i * 0.1);
      config.end.shoulder = 1.0 + (i * 0.1);
      config.hasAlgae = (i % 2 == 0);
      config.hasCoral = (i % 3 == 0);
      configs.push_back(config);
    }

    for (int i = 0; i < numRequests; i++) {
      threads.emplace_back(&Client::sendRequest, this, configs[i], i + 1);
    }

    for (auto& thread : threads) {
      thread.join();
    }
  }

 private:
  std::unique_ptr<ArmTrajectoryService::Stub> stub_;
};

int main() {
  Client client("localhost:" + GRPC_PORT);
  const int NUM_CONCURRENT_REQUESTS = 10;
  log_info("Starting concurrent test with %d requests...", NUM_CONCURRENT_REQUESTS);
  client.runConcurrentTests(NUM_CONCURRENT_REQUESTS);
  return 0;
}