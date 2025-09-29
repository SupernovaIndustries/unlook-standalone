#include "unlook/core/QtLogHandler.hpp"
#include "unlook/core/Logger.hpp"
#include <iostream>
#include <cstdio>

namespace unlook {
namespace core {

// Static member initialization
QtMessageHandler QtLogHandler::previousHandler_ = nullptr;

void QtLogHandler::install() {
    previousHandler_ = qInstallMessageHandler(messageHandler);
}

void QtLogHandler::uninstall() {
    if (previousHandler_) {
        qInstallMessageHandler(previousHandler_);
        previousHandler_ = nullptr;
    } else {
        qInstallMessageHandler(nullptr); // Restore default
    }
}

void QtLogHandler::messageHandler(QtMsgType type, const QMessageLogContext& context, const QString& msg) {
    auto& logger = Logger::getInstance();

    // Map Qt message type to Unlook LogLevel
    LogLevel level;
    const char* levelStr = nullptr;

    switch (type) {
        case QtDebugMsg:
            level = LogLevel::DEBUG;
            levelStr = "DEBUG";
            break;
        case QtInfoMsg:
            level = LogLevel::INFO;
            levelStr = "INFO";
            break;
        case QtWarningMsg:
            level = LogLevel::WARNING;
            levelStr = "WARNING";
            break;
        case QtCriticalMsg:
            level = LogLevel::ERROR;
            levelStr = "CRITICAL";
            break;
        case QtFatalMsg:
            level = LogLevel::CRITICAL;
            levelStr = "FATAL";
            break;
        default:
            level = LogLevel::INFO;
            levelStr = "UNKNOWN";
            break;
    }

    // Format message with context information
    std::string message = msg.toStdString();

    // Add source location if available
    if (context.file && context.line > 0) {
        // Extract filename from full path
        const char* filename = context.file;
        const char* lastSlash = nullptr;
        for (const char* p = filename; *p; ++p) {
            if (*p == '/' || *p == '\\') {
                lastSlash = p;
            }
        }
        if (lastSlash) {
            filename = lastSlash + 1;
        }

        // Add function name if available
        if (context.function) {
            message += " [" + std::string(context.function) + "]";
        }

        // Log with file and line information
        logger.log(level, message, context.file, context.line);
    } else {
        // Log without source location
        logger.log(level, message);
    }

    // CRITICAL: Maintain console output for development
    // Qt messages go to stderr by default
    if (level >= LogLevel::ERROR) {
        std::fprintf(stderr, "[Qt-%s] %s\n", levelStr, message.c_str());
    } else {
        std::fprintf(stdout, "[Qt-%s] %s\n", levelStr, message.c_str());
    }
    std::fflush(stdout);
    std::fflush(stderr);

    // Handle fatal messages (terminate application)
    if (type == QtFatalMsg) {
        logger.flush();
        std::abort();
    }
}

} // namespace core
} // namespace unlook