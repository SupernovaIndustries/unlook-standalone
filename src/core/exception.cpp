#include "unlook/core/exception.h"
#include <sstream>

namespace unlook {
namespace core {

std::string Exception::formatMessage(ResultCode code, 
                                   const std::string& message,
                                   const std::string& context) {
    std::ostringstream oss;
    oss << "[" << resultCodeToString(code) << "] " << message;
    if (!context.empty()) {
        oss << " (Context: " << context << ")";
    }
    return oss.str();
}

std::string resultCodeToString(ResultCode code) {
    switch (code) {
        case ResultCode::SUCCESS:
            return "SUCCESS";
        case ResultCode::ERROR_GENERIC:
            return "ERROR_GENERIC";
        case ResultCode::ERROR_INVALID_PARAMETER:
            return "ERROR_INVALID_PARAMETER";
        case ResultCode::ERROR_NOT_INITIALIZED:
            return "ERROR_NOT_INITIALIZED";
        case ResultCode::ERROR_ALREADY_INITIALIZED:
            return "ERROR_ALREADY_INITIALIZED";
        case ResultCode::ERROR_HARDWARE_FAILURE:
            return "ERROR_HARDWARE_FAILURE";
        case ResultCode::ERROR_CALIBRATION_INVALID:
            return "ERROR_CALIBRATION_INVALID";
        case ResultCode::ERROR_CAMERA_NOT_FOUND:
            return "ERROR_CAMERA_NOT_FOUND";
        case ResultCode::ERROR_CAMERA_ACCESS_DENIED:
            return "ERROR_CAMERA_ACCESS_DENIED";
        case ResultCode::ERROR_MEMORY_ALLOCATION:
            return "ERROR_MEMORY_ALLOCATION";
        case ResultCode::ERROR_FILE_NOT_FOUND:
            return "ERROR_FILE_NOT_FOUND";
        case ResultCode::ERROR_FILE_IO:
            return "ERROR_FILE_IO";
        case ResultCode::ERROR_TIMEOUT:
            return "ERROR_TIMEOUT";
        case ResultCode::ERROR_THREAD_FAILURE:
            return "ERROR_THREAD_FAILURE";
        case ResultCode::ERROR_SYNC_FAILURE:
            return "ERROR_SYNC_FAILURE";
        default:
            return "UNKNOWN_ERROR";
    }
}

} // namespace core
} // namespace unlook