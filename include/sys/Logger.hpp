#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

enum class LogLevel { DEBUG, INFO, WARN, ERROR };

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
  char buffer[20];
  time_t now = time(0);
  tm *ltm = localtime(&now);
  strftime(buffer, 20, "%Y-%m-%d %H:%M:%S", ltm);
  timestamp = buffer;
}

class Logger {
 private:
  static Logger *instance;
  Logger() {
    time_t now = time(0);
    const char *homeDirCStr = getenv("HOME");
    if (homeDirCStr == nullptr) {
      std::cerr << "HOME environment variable is not set" << std::endl;
      return;
    }
    std::string homeDir = homeDirCStr;
    std::string logDir = homeDir + "/cyber-planner/logs/";
    logFile.open(logDir + std::to_string(now) + ".log", std::ios::out);

    if (!logFile.is_open()) {
      std::cerr << "Failed to create log file, path: "
                << logDir + std::to_string(now) + ".log" << std::endl;
    } else {
      std::cout << "Log file created successfully, path: "
                << logDir + std::to_string(now) + ".log" << std::endl;
    }
  }
  std::ofstream logFile;

 public:
  static Logger *getInstance() {
    if (instance == nullptr) {
      instance = new Logger();
    }
    return instance;
  }
  ~Logger() {
    if (logFile.is_open()) {
      logFile.close();
    }
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
};

Logger *Logger::instance = nullptr;

#endif  // LOGGER_HPP