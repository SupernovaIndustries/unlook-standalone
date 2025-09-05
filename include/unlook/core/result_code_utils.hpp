#pragma once

#include "unlook/core/types.hpp"
#include <string>

namespace unlook {
namespace core {

/**
 * @brief Convert ResultCode to human-readable string
 */
inline std::string resultCodeToString(ResultCode code) {
    switch (code) {
        case ResultCode::SUCCESS:
            return "Success";
        case ResultCode::ERROR_GENERIC:
            return "Generic error";
        case ResultCode::ERROR_INVALID_PARAMETER:
            return "Invalid parameter";
        case ResultCode::ERROR_NOT_INITIALIZED:
            return "Not initialized";
        case ResultCode::ERROR_ALREADY_INITIALIZED:
            return "Already initialized";
        case ResultCode::ERROR_HARDWARE_FAILURE:
            return "Hardware failure";
        case ResultCode::ERROR_CALIBRATION_INVALID:
            return "Invalid calibration";
        case ResultCode::ERROR_CAMERA_NOT_FOUND:
            return "Camera not found";
        case ResultCode::ERROR_CAMERA_ACCESS_DENIED:
            return "Camera access denied";
        case ResultCode::ERROR_MEMORY_ALLOCATION:
            return "Memory allocation failed";
        case ResultCode::ERROR_FILE_NOT_FOUND:
            return "File not found";
        case ResultCode::ERROR_FILE_IO:
            return "File I/O error";
        case ResultCode::ERROR_TIMEOUT:
            return "Operation timeout";
        case ResultCode::ERROR_THREAD_FAILURE:
            return "Thread failure";
        case ResultCode::ERROR_SYNC_FAILURE:
            return "Synchronization failure";
        default:
            return "Unknown error code: " + std::to_string(static_cast<int>(code));
    }
}

} // namespace core
} // namespace unlook