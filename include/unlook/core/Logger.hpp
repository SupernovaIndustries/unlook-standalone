#pragma once

#include <string>
#include <sstream>
#include <mutex>
#include <memory>
#include <chrono>
#include <fstream>

namespace unlook {
namespace core {

/**
 * Logger severity levels
 */
enum class LogLevel {
    TRACE = 0,
    DEBUG = 1,
    INFO = 2,
    WARNING = 3,
    ERROR = 4,
    CRITICAL = 5
};

/**
 * Simple thread-safe logger
 */
class Logger {
public:
    /**
     * Get singleton instance
     */
    static Logger& getInstance();
    
    /**
     * Set minimum log level
     */
    void setLevel(LogLevel level) { minLevel_ = level; }
    
    /**
     * Get current log level
     */
    LogLevel getLevel() const { return minLevel_; }
    
    /**
     * Enable/disable console output
     */
    void setConsoleOutput(bool enable) { consoleOutput_ = enable; }
    
    /**
     * Set log file
     */
    bool setLogFile(const std::string& filename);
    
    /**
     * Close log file
     */
    void closeLogFile();
    
    /**
     * Initialize logger with configuration
     */
    bool initialize(LogLevel level, bool consoleOutput, bool fileOutput, const std::string& filename = "");

    /**
     * Initialize logger with automatic timestamped log file
     * Creates log directory if needed, generates filename with timestamp
     * @param logDirectory Directory for log files (default: /home/unlook/log)
     * @param level Minimum log level to capture
     * @return true if initialization successful, false otherwise
     */
    bool initializeWithTimestamp(const std::string& logDirectory = "/home/unlook/log",
                                  LogLevel level = LogLevel::INFO);

    /**
     * Get current log file path
     * @return Path to current log file, empty string if no file logging
     */
    std::string getCurrentLogFile() const;

    /**
     * Flush all pending log messages
     */
    void flush();
    
    /**
     * Log message
     */
    void log(LogLevel level, const std::string& message, 
             const std::string& file = "", int line = 0);
    
    /**
     * Log message (overload for compatibility)
     */
    void log(LogLevel level, const std::string& message, const std::string& file);
    
    // Convenience methods
    void trace(const std::string& msg, const std::string& file = "", int line = 0) {
        log(LogLevel::TRACE, msg, file, line);
    }
    
    void debug(const std::string& msg, const std::string& file = "", int line = 0) {
        log(LogLevel::DEBUG, msg, file, line);
    }
    
    void info(const std::string& msg, const std::string& file = "", int line = 0) {
        log(LogLevel::INFO, msg, file, line);
    }
    
    void warning(const std::string& msg, const std::string& file = "", int line = 0) {
        log(LogLevel::WARNING, msg, file, line);
    }
    
    void error(const std::string& msg, const std::string& file = "", int line = 0) {
        log(LogLevel::ERROR, msg, file, line);
    }
    
    void critical(const std::string& msg, const std::string& file = "", int line = 0) {
        log(LogLevel::CRITICAL, msg, file, line);
    }
    
private:
    Logger();
    ~Logger();
    
    // Delete copy/move
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    
    std::string levelToString(LogLevel level) const;
    std::string getTimestamp() const;
    std::string formatMessage(LogLevel level, const std::string& message,
                               const std::string& file, int line) const;
    std::string generateTimestampedFilename(const std::string& directory) const;
    bool createDirectoryIfNeeded(const std::string& directory) const;

    LogLevel minLevel_ = LogLevel::INFO;
    bool consoleOutput_ = true;
    std::string currentLogFile_;

    std::mutex mutex_;
    std::ofstream logFile_;
};

// Convenience macros
#define LOG_TRACE(msg) unlook::core::Logger::getInstance().trace(msg, __FILE__, __LINE__)
#define LOG_DEBUG(msg) unlook::core::Logger::getInstance().debug(msg, __FILE__, __LINE__)
#define LOG_INFO(msg) unlook::core::Logger::getInstance().info(msg, __FILE__, __LINE__)
#define LOG_WARNING(msg) unlook::core::Logger::getInstance().warning(msg, __FILE__, __LINE__)
#define LOG_ERROR(msg) unlook::core::Logger::getInstance().error(msg, __FILE__, __LINE__)
#define LOG_CRITICAL(msg) unlook::core::Logger::getInstance().critical(msg, __FILE__, __LINE__)

// Stream-style logging support
class LogStream {
public:
    LogStream(LogLevel level, const std::string& component = "")
        : level_(level), component_(component) {}
    
    ~LogStream() {
        Logger::getInstance().log(level_, stream_.str(), component_, 0);
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

// UNLOOK_LOG_* macros for compatibility with existing code
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

} // namespace core
} // namespace unlook