#include "unlook/core/Logger.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <cerrno>
#include <cstring>

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

std::string Logger::generateTimestampedFilename(const std::string& directory) const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::ostringstream oss;
    oss << directory;
    if (!directory.empty() && directory.back() != '/') {
        oss << '/';
    }
    oss << "log_unlook_";
    oss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H-%M-%S");
    oss << ".txt";

    return oss.str();
}

bool Logger::createDirectoryIfNeeded(const std::string& directory) const {
    struct stat st;

    // Check if directory exists
    if (stat(directory.c_str(), &st) == 0) {
        if (S_ISDIR(st.st_mode)) {
            return true; // Directory exists
        } else {
            std::cerr << "[Logger] Error: " << directory << " exists but is not a directory" << std::endl;
            return false;
        }
    }

    // Try to create directory (with parents if needed)
    // Using mode 0755 (rwxr-xr-x)
    if (mkdir(directory.c_str(), 0755) == 0) {
        return true;
    }

    // If mkdir failed, it might be because parent doesn't exist
    // Try to create parent directories recursively
    if (errno == ENOENT) {
        size_t pos = directory.find_last_of('/');
        if (pos != std::string::npos && pos > 0) {
            std::string parent = directory.substr(0, pos);
            if (createDirectoryIfNeeded(parent)) {
                // Parent created, now try again
                return mkdir(directory.c_str(), 0755) == 0;
            }
        }
    }

    std::cerr << "[Logger] Failed to create directory " << directory
              << ": " << std::strerror(errno) << std::endl;
    return false;
}

bool Logger::initializeWithTimestamp(const std::string& logDirectory, LogLevel level) {
    std::lock_guard<std::mutex> lock(mutex_);

    // Create log directory if needed
    if (!createDirectoryIfNeeded(logDirectory)) {
        std::cerr << "[Logger] Warning: Could not create log directory, file logging disabled" << std::endl;
        minLevel_ = level;
        consoleOutput_ = true;
        return false; // Continue without file logging
    }

    // Generate timestamped filename
    currentLogFile_ = generateTimestampedFilename(logDirectory);

    // Close existing log file if open
    if (logFile_.is_open()) {
        logFile_.close();
    }

    // Open new log file
    logFile_.open(currentLogFile_, std::ios::out | std::ios::app);
    if (!logFile_.is_open()) {
        std::cerr << "[Logger] Error: Failed to open log file " << currentLogFile_
                  << ": " << std::strerror(errno) << std::endl;
        currentLogFile_.clear();
        minLevel_ = level;
        consoleOutput_ = true;
        return false;
    }

    // Configure logger settings
    minLevel_ = level;
    consoleOutput_ = true; // Always enable console for dual output

    // Write header to log file
    logFile_ << "===========================================" << std::endl;
    logFile_ << "Unlook 3D Scanner Log" << std::endl;
    logFile_ << "Started: " << getTimestamp() << std::endl;
    logFile_ << "Log Level: " << levelToString(level) << std::endl;
    logFile_ << "===========================================" << std::endl;
    logFile_.flush();

    return true;
}

std::string Logger::getCurrentLogFile() const {
    // No lock needed for reading a string that's only written during initialization
    // If thread-safety is critical, make mutex_ mutable
    return currentLogFile_;
}

} // namespace core
} // namespace unlook