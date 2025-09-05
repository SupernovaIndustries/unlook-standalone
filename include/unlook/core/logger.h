#pragma once

#include <string>
#include <memory>
#include <sstream>
#include <mutex>
#include <fstream>

/**
 * @file logger.h
 * @brief Thread-safe logging system for the Unlook 3D Scanner API
 */

namespace unlook {
namespace core {

/**
 * @brief Log severity levels
 */
enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    CRITICAL = 4
};

/**
 * @brief Thread-safe logger implementation
 * 
 * Provides structured logging with timestamps, thread IDs,
 * and configurable output targets (console, file, both).
 */
class Logger {
public:
    /**
     * @brief Get the singleton logger instance
     * @return Reference to the global logger
     */
    static Logger& getInstance();
    
    /**
     * @brief Initialize logger with configuration
     * @param min_level Minimum log level to output
     * @param log_to_console Enable console output
     * @param log_to_file Enable file output
     * @param log_file_path Path to log file (if file output enabled)
     */
    void initialize(LogLevel min_level = LogLevel::INFO,
                   bool log_to_console = true,
                   bool log_to_file = true,
                   const std::string& log_file_path = "unlook.log");
    
    /**
     * @brief Log a message with specified level
     * @param level Log severity level
     * @param message Message to log
     * @param component Optional component identifier
     */
    void log(LogLevel level, 
             const std::string& message, 
             const std::string& component = "");
    
    /**
     * @brief Set minimum log level
     * @param level Minimum level to output
     */
    void setLogLevel(LogLevel level);
    
    /**
     * @brief Get current minimum log level
     * @return Current minimum log level
     */
    LogLevel getLogLevel() const;
    
    /**
     * @brief Enable/disable console output
     * @param enable Enable console output
     */
    void setConsoleOutput(bool enable);
    
    /**
     * @brief Enable/disable file output
     * @param enable Enable file output
     * @param log_file_path Path to log file
     */
    void setFileOutput(bool enable, const std::string& log_file_path = "");
    
    /**
     * @brief Flush all log outputs
     */
    void flush();

private:
    Logger() = default;
    ~Logger();
    
    // Non-copyable, non-movable
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;
    
    void writeLog(const std::string& formatted_message);
    std::string formatMessage(LogLevel level, 
                             const std::string& message,
                             const std::string& component) const;
    std::string levelToString(LogLevel level) const;
    std::string getCurrentTimestamp() const;
    
    mutable std::mutex mutex_;
    LogLevel min_level_ = LogLevel::INFO;
    bool console_output_ = true;
    bool file_output_ = false;
    std::unique_ptr<std::ofstream> log_file_;
    std::string log_file_path_;
};

/**
 * @brief RAII log stream class for convenient logging
 */
class LogStream {
public:
    LogStream(LogLevel level, const std::string& component = "")
        : level_(level), component_(component) {}
    
    ~LogStream() {
        Logger::getInstance().log(level_, stream_.str(), component_);
    }
    
    template<typename T>
    LogStream& operator<<(const T& value) {
        stream_ << value;
        return *this;
    }

private:
    LogLevel level_;
    std::string component_;
    std::ostringstream stream_;
};

/**
 * @brief Convenience macros for logging
 */
#define UNLOOK_LOG_DEBUG(component) \
    unlook::core::LogStream(unlook::core::LogLevel::DEBUG, component)

#define UNLOOK_LOG_INFO(component) \
    unlook::core::LogStream(unlook::core::LogLevel::INFO, component)

#define UNLOOK_LOG_WARNING(component) \
    unlook::core::LogStream(unlook::core::LogLevel::WARNING, component)

#define UNLOOK_LOG_ERROR(component) \
    unlook::core::LogStream(unlook::core::LogLevel::ERROR, component)

#define UNLOOK_LOG_CRITICAL(component) \
    unlook::core::LogStream(unlook::core::LogLevel::CRITICAL, component)

// Simplified macros without component
#define LOG_DEBUG() UNLOOK_LOG_DEBUG("")
#define LOG_INFO() UNLOOK_LOG_INFO("")
#define LOG_WARNING() UNLOOK_LOG_WARNING("")
#define LOG_ERROR() UNLOOK_LOG_ERROR("")
#define LOG_CRITICAL() UNLOOK_LOG_CRITICAL("")

} // namespace core
} // namespace unlook