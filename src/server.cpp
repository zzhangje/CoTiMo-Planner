#include <eigen3/Eigen/Eigen>

#include "Logger.hpp"

int main() {
  Logger::getInstance()->log(LogLevel::INFO, "Server", "Server started");
  Logger::getInstance()->log(LogLevel::ERROR, "Server", "Server crashed");
  Logger::getInstance()->log(LogLevel::INFO, "Server", "Server restarted");
  return 0;
}
