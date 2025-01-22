#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

enum LogLevel { DEBUG, INFO, WARN, ERROR };

std::string toString(LogLevel level) {
  switch (level) {
    case LogLevel::DEBUG:
      return " DEBUG ";
    case LogLevel::INFO:
      return "  INFO ";
    case LogLevel::WARN:
      return "  WARN ";
    case LogLevel::ERROR:
      return " ERROR ";
    default:
      return "UNKNOWN";
  }
}

void getTime(std::string &timestamp) {
  auto now = std::chrono::system_clock::now();
  auto now_time_t = std::chrono::system_clock::to_time_t(now);
  auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) %
                std::chrono::seconds(1);

  char buffer[20];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localtime(&now_time_t));
  timestamp = std::string(buffer) + "." + std::to_string(now_ms.count());
}

class Logger {
 private:
  static Logger *instance;
  std::ofstream logFile;
  std::string logFileName;

  Logger() {
    time_t now = time(0);
    std::string logDir = "/workspaces/cyber-planner-2025/logs/";
    logFileName = logDir + std::to_string(now) + ".log";
    logFile.open(logFileName, std::ios::out);
    if (!logFile.is_open()) {
      std::cerr << "Failed to create log file, path: " << logFileName
                << std::endl;
    } else {
      std::cout << "Successfully created log file, path: " << logFileName
                << std::endl;
    }
  }

  ~Logger() {
    if (logFile.is_open()) {
      logFile.close();
    }
  }

 public:
  static Logger *getInstance() {
    if (instance == nullptr) {
      instance = new Logger();
    }
    return instance;
  }

  void log(LogLevel level, const std::string &header,
           const std::string &message) {
    std::string timestamp;
    getTime(timestamp);
    std::ostringstream logEntry;
    logEntry << "[" << toString(level) << "] " << timestamp << " [" << header
             << "] " << message << std::endl;
    std::cout << logEntry.str();
    if (logFile.is_open()) {
      logFile << logEntry.str();
      logFile.flush();
    }
  }

  std::string getLogFileName() { return logFileName; }
};

Logger *Logger::instance = nullptr;

#endif  // LOGGER_HPP