#pragma once

#include "types.hpp"
#include <stdexcept>
#include <string>

/**
 * @file exception.h
 * @brief Exception handling system for the Unlook 3D Scanner API
 */

namespace unlook {
namespace core {

/**
 * @brief Base exception class for all Unlook API exceptions
 * 
 * Provides comprehensive error reporting with result codes,
 * context information, and stack trace support.
 */
class Exception : public std::runtime_error {
public:
    /**
     * @brief Construct exception with result code and message
     * @param code Result code indicating error type
     * @param message Detailed error description
     * @param context Additional context information
     */
    Exception(ResultCode code, 
              const std::string& message, 
              const std::string& context = "")
        : std::runtime_error(formatMessage(code, message, context))
        , result_code_(code)
        , message_(message)
        , context_(context) {}

    /**
     * @brief Get the result code
     * @return ResultCode indicating error type
     */
    ResultCode getResultCode() const noexcept { return result_code_; }
    
    /**
     * @brief Get the original error message
     * @return Error message without formatting
     */
    const std::string& getMessage() const noexcept { return message_; }
    
    /**
     * @brief Get the error context
     * @return Context information
     */
    const std::string& getContext() const noexcept { return context_; }

private:
    ResultCode result_code_;
    std::string message_;
    std::string context_;
    
    static std::string formatMessage(ResultCode code, 
                                   const std::string& message,
                                   const std::string& context);
};

/**
 * @brief Hardware-related exceptions
 */
class HardwareException : public Exception {
public:
    HardwareException(const std::string& message, 
                     const std::string& context = "")
        : Exception(ResultCode::ERROR_HARDWARE_FAILURE, message, context) {}
};

/**
 * @brief Camera-related exceptions
 */
class CameraException : public Exception {
public:
    CameraException(ResultCode code,
                   const std::string& message, 
                   const std::string& context = "")
        : Exception(code, message, context) {}
};

/**
 * @brief Calibration-related exceptions
 */
class CalibrationException : public Exception {
public:
    CalibrationException(const std::string& message, 
                        const std::string& context = "")
        : Exception(ResultCode::ERROR_CALIBRATION_INVALID, message, context) {}
};

/**
 * @brief File I/O related exceptions
 */
class FileException : public Exception {
public:
    FileException(ResultCode code,
                  const std::string& message, 
                  const std::string& context = "")
        : Exception(code, message, context) {}
};

/**
 * @brief Synchronization-related exceptions
 */
class SyncException : public Exception {
public:
    SyncException(const std::string& message, 
                  const std::string& context = "")
        : Exception(ResultCode::ERROR_SYNC_FAILURE, message, context) {}
};

/**
 * @brief Convert result code to string representation
 * @param code Result code to convert
 * @return String representation of result code
 */
std::string resultCodeToString(ResultCode code);

/**
 * @brief Macro for throwing exceptions with automatic context
 */
#define UNLOOK_THROW(ExceptionType, message) \
    throw ExceptionType(message, std::string(__FILE__) + ":" + std::to_string(__LINE__))

#define UNLOOK_THROW_CODE(ExceptionType, code, message) \
    throw ExceptionType(code, message, std::string(__FILE__) + ":" + std::to_string(__LINE__))

} // namespace core
} // namespace unlook