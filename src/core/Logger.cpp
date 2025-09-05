#include "unlook/core/Logger.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace unlook {
namespace core {

Logger::Logger() : minLevel_(LogLevel::INFO), consoleOutput_(true) {
    // Constructor implementation
}

Logger::~Logger() {
    closeLogFile();
}

Logger& Logger::getInstance() {
    static Logger instance;
    return instance;
}

bool Logger::setLogFile(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (logFile_.is_open()) {
        logFile_.close();
    }
    
    logFile_.open(filename, std::ios::app);
    return logFile_.is_open();
}

void Logger::closeLogFile() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (logFile_.is_open()) {
        logFile_.close();
    }
}

bool Logger::initialize(LogLevel level, bool consoleOutput, bool fileOutput, const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    minLevel_ = level;
    consoleOutput_ = consoleOutput;
    
    if (fileOutput && !filename.empty()) {
        if (logFile_.is_open()) {
            logFile_.close();
        }
        logFile_.open(filename, std::ios::app);
        if (!logFile_.is_open()) {
            return false;
        }
    }
    
    return true;
}

void Logger::flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (consoleOutput_) {
        std::cout.flush();
        std::cerr.flush();
    }
    if (logFile_.is_open()) {
        logFile_.flush();
    }
}

void Logger::log(LogLevel level, const std::string& message, const std::string& file, int line) {
    if (level < minLevel_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::string formatted = formatMessage(level, message, file, line);
    
    if (consoleOutput_) {
        if (level >= LogLevel::ERROR) {
            std::cerr << formatted << std::endl;
        } else {
            std::cout << formatted << std::endl;
        }
    }
    
    if (logFile_.is_open()) {
        logFile_ << formatted << std::endl;
    }
}

void Logger::log(LogLevel level, const std::string& message, const std::string& file) {
    log(level, message, file, 0);
}

std::string Logger::levelToString(LogLevel level) const {
    switch (level) {
        case LogLevel::TRACE:    return "TRACE";
        case LogLevel::DEBUG:    return "DEBUG";
        case LogLevel::INFO:     return "INFO";
        case LogLevel::WARNING:  return "WARNING";
        case LogLevel::ERROR:    return "ERROR";
        case LogLevel::CRITICAL: return "CRITICAL";
        default:                 return "UNKNOWN";
    }
}

std::string Logger::getTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    oss << "." << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

std::string Logger::formatMessage(LogLevel level, const std::string& message, 
                                  const std::string& file, int line) const {
    std::ostringstream oss;
    oss << "[" << getTimestamp() << "] [" << levelToString(level) << "] " << message;
    
    if (!file.empty() && line > 0) {
        // Extract just the filename from the full path
        size_t pos = file.find_last_of("/\\");
        std::string filename = (pos != std::string::npos) ? file.substr(pos + 1) : file;
        oss << " (" << filename << ":" << line << ")";
    }
    
    return oss.str();
}

} // namespace core
} // namespace unlook