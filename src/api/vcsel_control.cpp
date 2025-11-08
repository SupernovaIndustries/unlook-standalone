#include <unlook/api/vcsel_control.h>
#include <unlook/hardware/VCSELProjector.hpp>
#include <unlook/core/Logger.hpp>

#include <memory>
#include <string>
#include <cstring>
#include <map>
#include <mutex>
#include <chrono>

// Internal handle structure
struct unlook_vcsel_projector {
    std::shared_ptr<unlook::hardware::VCSELProjector> projector;
    std::mutex callback_mutex;

    // C callback storage with user data
    unlook_vcsel_projection_callback_t projection_callback = nullptr;
    void* projection_user_data = nullptr;

    unlook_vcsel_thermal_callback_t thermal_callback = nullptr;
    void* thermal_user_data = nullptr;

    unlook_vcsel_error_callback_t error_callback = nullptr;
    void* error_user_data = nullptr;

    unlook_vcsel_sync_callback_t sync_callback = nullptr;
    void* sync_user_data = nullptr;
};

// Helper functions for type conversion
namespace {

using namespace unlook::hardware;

VCSELProjector::ProjectionMode convert_mode(unlook_vcsel_mode_t mode) {
    switch (mode) {
        case UNLOOK_VCSEL_MODE_DISABLED: return VCSELProjector::ProjectionMode::DISABLED;
        case UNLOOK_VCSEL_MODE_DEPTH_CAPTURE: return VCSELProjector::ProjectionMode::DEPTH_CAPTURE;
        case UNLOOK_VCSEL_MODE_FACE_RECOGNITION: return VCSELProjector::ProjectionMode::FACE_RECOGNITION;
        case UNLOOK_VCSEL_MODE_STRUCTURED_LIGHT: return VCSELProjector::ProjectionMode::STRUCTURED_LIGHT;
        case UNLOOK_VCSEL_MODE_FLOOD_ILLUMINATION: return VCSELProjector::ProjectionMode::FLOOD_ILLUMINATION;
        case UNLOOK_VCSEL_MODE_DIAGNOSTIC: return VCSELProjector::ProjectionMode::DIAGNOSTIC;
        default: return VCSELProjector::ProjectionMode::DISABLED;
    }
}

unlook_vcsel_mode_t convert_mode(VCSELProjector::ProjectionMode mode) {
    switch (mode) {
        case VCSELProjector::ProjectionMode::DISABLED: return UNLOOK_VCSEL_MODE_DISABLED;
        case VCSELProjector::ProjectionMode::DEPTH_CAPTURE: return UNLOOK_VCSEL_MODE_DEPTH_CAPTURE;
        case VCSELProjector::ProjectionMode::FACE_RECOGNITION: return UNLOOK_VCSEL_MODE_FACE_RECOGNITION;
        case VCSELProjector::ProjectionMode::STRUCTURED_LIGHT: return UNLOOK_VCSEL_MODE_STRUCTURED_LIGHT;
        case VCSELProjector::ProjectionMode::FLOOD_ILLUMINATION: return UNLOOK_VCSEL_MODE_FLOOD_ILLUMINATION;
        case VCSELProjector::ProjectionMode::DIAGNOSTIC: return UNLOOK_VCSEL_MODE_DIAGNOSTIC;
        default: return UNLOOK_VCSEL_MODE_DISABLED;
    }
}

VCSELProjector::PatternType convert_pattern(unlook_vcsel_pattern_t pattern) {
    switch (pattern) {
        case UNLOOK_VCSEL_PATTERN_NONE: return VCSELProjector::PatternType::NONE;
        case UNLOOK_VCSEL_PATTERN_DOTS_15K: return VCSELProjector::PatternType::DOTS_15K;
        case UNLOOK_VCSEL_PATTERN_LINES_HORIZONTAL: return VCSELProjector::PatternType::LINES_HORIZONTAL;
        case UNLOOK_VCSEL_PATTERN_LINES_VERTICAL: return VCSELProjector::PatternType::LINES_VERTICAL;
        case UNLOOK_VCSEL_PATTERN_GRID: return VCSELProjector::PatternType::GRID;
        case UNLOOK_VCSEL_PATTERN_RANDOM_SPECKLE: return VCSELProjector::PatternType::RANDOM_SPECKLE;
        case UNLOOK_VCSEL_PATTERN_ADAPTIVE: return VCSELProjector::PatternType::ADAPTIVE;
        case UNLOOK_VCSEL_PATTERN_CUSTOM: return VCSELProjector::PatternType::CUSTOM;
        default: return VCSELProjector::PatternType::NONE;
    }
}

unlook_vcsel_pattern_t convert_pattern(VCSELProjector::PatternType pattern) {
    switch (pattern) {
        case VCSELProjector::PatternType::NONE: return UNLOOK_VCSEL_PATTERN_NONE;
        case VCSELProjector::PatternType::DOTS_15K: return UNLOOK_VCSEL_PATTERN_DOTS_15K;
        case VCSELProjector::PatternType::LINES_HORIZONTAL: return UNLOOK_VCSEL_PATTERN_LINES_HORIZONTAL;
        case VCSELProjector::PatternType::LINES_VERTICAL: return UNLOOK_VCSEL_PATTERN_LINES_VERTICAL;
        case VCSELProjector::PatternType::GRID: return UNLOOK_VCSEL_PATTERN_GRID;
        case VCSELProjector::PatternType::RANDOM_SPECKLE: return UNLOOK_VCSEL_PATTERN_RANDOM_SPECKLE;
        case VCSELProjector::PatternType::ADAPTIVE: return UNLOOK_VCSEL_PATTERN_ADAPTIVE;
        case VCSELProjector::PatternType::CUSTOM: return UNLOOK_VCSEL_PATTERN_CUSTOM;
        default: return UNLOOK_VCSEL_PATTERN_NONE;
    }
}

VCSELProjector::ProjectorConfig convert_config(const unlook_vcsel_config_t& c_config) {
    VCSELProjector::ProjectorConfig config;

    config.mode = convert_mode(c_config.mode);
    config.pattern = convert_pattern(c_config.pattern);
    config.vcsel_current_ma = c_config.vcsel_current_ma;
    config.flood_current_ma = c_config.flood_current_ma;
    config.projection_duration_ms = c_config.projection_duration_ms;
    config.cool_down_delay_ms = c_config.cool_down_delay_ms;
    config.sync_tolerance_us = c_config.sync_tolerance_us;
    config.max_temperature_c = c_config.max_temperature_c;
    config.max_duty_cycle = c_config.max_duty_cycle;
    config.max_continuous_time_ms = c_config.max_continuous_time_ms;
    config.mandatory_pause_ms = c_config.mandatory_pause_ms;
    config.enable_flood_assist = c_config.enable_flood_assist;
    config.enable_thermal_protection = c_config.enable_thermal_protection;
    config.enable_camera_sync = c_config.enable_camera_sync;
    config.adaptive_current = c_config.adaptive_current;
    config.face_distance_mm = c_config.face_distance_mm;

    return config;
}

unlook_vcsel_config_t convert_config(const VCSELProjector::ProjectorConfig& cpp_config) {
    unlook_vcsel_config_t config = {};

    config.mode = convert_mode(cpp_config.mode);
    config.pattern = convert_pattern(cpp_config.pattern);
    config.vcsel_current_ma = cpp_config.vcsel_current_ma;
    config.flood_current_ma = cpp_config.flood_current_ma;
    config.projection_duration_ms = cpp_config.projection_duration_ms;
    config.cool_down_delay_ms = cpp_config.cool_down_delay_ms;
    config.sync_tolerance_us = cpp_config.sync_tolerance_us;
    config.max_temperature_c = cpp_config.max_temperature_c;
    config.max_duty_cycle = cpp_config.max_duty_cycle;
    config.max_continuous_time_ms = cpp_config.max_continuous_time_ms;
    config.mandatory_pause_ms = cpp_config.mandatory_pause_ms;
    config.enable_flood_assist = cpp_config.enable_flood_assist;
    config.enable_thermal_protection = cpp_config.enable_thermal_protection;
    config.enable_camera_sync = cpp_config.enable_camera_sync;
    config.adaptive_current = cpp_config.adaptive_current;
    config.face_distance_mm = cpp_config.face_distance_mm;

    return config;
}

unlook_vcsel_status_t convert_status(const VCSELProjector::ProjectorStatus& cpp_status) {
    unlook_vcsel_status_t status = {};

    status.initialized = cpp_status.initialized;
    status.projection_active = cpp_status.projection_active;
    status.ready = true; // Would need isReady() call
    status.current_mode = convert_mode(cpp_status.current_mode);
    status.current_pattern = convert_pattern(cpp_status.current_pattern);
    status.vcsel_current_ma = cpp_status.vcsel_current_ma;
    status.flood_current_ma = cpp_status.flood_current_ma;
    status.temperature_c = cpp_status.temperature_c;
    status.thermal_protection_active = cpp_status.thermal_protection_active;
    status.projection_cycles = cpp_status.projection_cycles;
    status.sync_captures = cpp_status.sync_captures;
    status.thermal_events = cpp_status.thermal_events;
    status.avg_projection_time_ms = cpp_status.avg_projection_time_ms;
    status.current_duty_cycle = cpp_status.current_duty_cycle;
    status.hardware_ok = cpp_status.hardware_ok;
    status.sync_ok = cpp_status.sync_ok;
    status.thermal_ok = cpp_status.thermal_ok;
    status.last_update_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        cpp_status.last_update.time_since_epoch()).count();

    strncpy(status.last_error, cpp_status.last_error.c_str(), sizeof(status.last_error) - 1);
    status.last_error[sizeof(status.last_error) - 1] = '\0';

    return status;
}

unlook_vcsel_result_t convert_result(bool success) {
    return success ? UNLOOK_VCSEL_SUCCESS : UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
}

} // anonymous namespace

// C API Implementation

extern "C" {

void unlook_vcsel_get_version(int* major, int* minor, int* patch) {
    if (major) *major = UNLOOK_VCSEL_API_VERSION_MAJOR;
    if (minor) *minor = UNLOOK_VCSEL_API_VERSION_MINOR;
    if (patch) *patch = UNLOOK_VCSEL_API_VERSION_PATCH;
}

unlook_vcsel_result_t unlook_vcsel_get_default_config(unlook_vcsel_config_t* config) {
    if (!config) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    VCSELProjector::ProjectorConfig default_config;
    *config = convert_config(default_config);

    return UNLOOK_VCSEL_SUCCESS;
}

unlook_vcsel_result_t unlook_vcsel_create(const unlook_vcsel_config_t* config, unlook_vcsel_handle_t* handle) {
    if (!handle) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        auto projector_handle = std::make_unique<unlook_vcsel_projector>();
        projector_handle->projector = std::make_shared<VCSELProjector>();

        VCSELProjector::ProjectorConfig cpp_config;
        if (config) {
            cpp_config = convert_config(*config);
        }

        if (!projector_handle->projector->initialize(cpp_config)) {
            return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
        }

        *handle = projector_handle.release();
        return UNLOOK_VCSEL_SUCCESS;

    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: Failed to create projector: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_destroy(unlook_vcsel_handle_t handle) {
    if (!handle) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        handle->projector->shutdown();
        delete handle;
        return UNLOOK_VCSEL_SUCCESS;
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: Failed to destroy projector: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_enable_depth_capture(unlook_vcsel_handle_t handle,
                                                       unlook_vcsel_pattern_t pattern) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        bool success = handle->projector->enableDepthCapture(convert_pattern(pattern));
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: enableDepthCapture failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_enable_face_recognition(unlook_vcsel_handle_t handle,
                                                          float distance_mm) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        bool success = handle->projector->enableFaceRecognition(distance_mm);
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: enableFaceRecognition failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_enable_structured_light(unlook_vcsel_handle_t handle,
                                                          unlook_vcsel_pattern_t pattern) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        bool success = handle->projector->enableStructuredLight(convert_pattern(pattern));
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: enableStructuredLight failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_disable_projection(unlook_vcsel_handle_t handle) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        bool success = handle->projector->disableProjection();
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: disableProjection failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_trigger_capture(unlook_vcsel_handle_t handle,
                                                  uint32_t exposure_time_us,
                                                  unlook_vcsel_projection_callback_t callback,
                                                  void* user_data) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        VCSELProjector::ProjectionCallback cpp_callback = nullptr;

        if (callback) {
            // Store callback and user data
            {
                std::lock_guard<std::mutex> lock(handle->callback_mutex);
                handle->projection_callback = callback;
                handle->projection_user_data = user_data;
            }

            cpp_callback = [handle](VCSELProjector::PatternType pattern, uint64_t timestamp_ns) {
                std::lock_guard<std::mutex> lock(handle->callback_mutex);
                if (handle->projection_callback) {
                    handle->projection_callback(convert_pattern(pattern), timestamp_ns, handle->projection_user_data);
                }
            };
        }

        bool success = handle->projector->triggerStructuredLightCapture(exposure_time_us, cpp_callback);
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: triggerCapture failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_trigger_face_illumination(unlook_vcsel_handle_t handle,
                                                           uint32_t duration_ms,
                                                           unlook_vcsel_projection_callback_t callback,
                                                           void* user_data) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        VCSELProjector::ProjectionCallback cpp_callback = nullptr;

        if (callback) {
            {
                std::lock_guard<std::mutex> lock(handle->callback_mutex);
                handle->projection_callback = callback;
                handle->projection_user_data = user_data;
            }

            cpp_callback = [handle](VCSELProjector::PatternType pattern, uint64_t timestamp_ns) {
                std::lock_guard<std::mutex> lock(handle->callback_mutex);
                if (handle->projection_callback) {
                    handle->projection_callback(convert_pattern(pattern), timestamp_ns, handle->projection_user_data);
                }
            };
        }

        bool success = handle->projector->triggerFaceIllumination(duration_ms, cpp_callback);
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: triggerFaceIllumination failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_set_pattern(unlook_vcsel_handle_t handle,
                                              unlook_vcsel_pattern_t pattern,
                                              uint16_t vcsel_current_ma,
                                              uint16_t flood_current_ma) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        bool success = handle->projector->setProjectionPattern(convert_pattern(pattern),
                                                              vcsel_current_ma,
                                                              flood_current_ma);
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: setPattern failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_set_manual_projection(unlook_vcsel_handle_t handle,
                                                        bool enable,
                                                        uint32_t duration_ms) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        bool success = handle->projector->setManualProjection(enable, duration_ms);
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: setManualProjection failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_configure_sync(unlook_vcsel_handle_t handle,
                                                 bool enable,
                                                 uint32_t tolerance_us) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        bool success = handle->projector->configureCameraSync(enable, tolerance_us);
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: configureSync failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_SYNC_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_adapt_current(unlook_vcsel_handle_t handle,
                                                float ambient_light_lux,
                                                float target_distance_mm) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        bool success = handle->projector->adaptCurrentForConditions(ambient_light_lux, target_distance_mm);
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: adaptCurrent failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_emergency_shutdown(unlook_vcsel_handle_t handle) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        handle->projector->emergencyShutdown();
        return UNLOOK_VCSEL_SUCCESS;
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: emergencyShutdown failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_EMERGENCY_SHUTDOWN;
    }
}

unlook_vcsel_result_t unlook_vcsel_get_status(unlook_vcsel_handle_t handle,
                                             unlook_vcsel_status_t* status) {
    if (!handle || !handle->projector || !status) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        auto cpp_status = handle->projector->getStatus();
        *status = convert_status(cpp_status);
        status->ready = handle->projector->isReady();
        return UNLOOK_VCSEL_SUCCESS;
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: getStatus failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_get_config(unlook_vcsel_handle_t handle,
                                             unlook_vcsel_config_t* config) {
    if (!handle || !handle->projector || !config) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        auto cpp_config = handle->projector->getConfig();
        *config = convert_config(cpp_config);
        return UNLOOK_VCSEL_SUCCESS;
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: getConfig failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_get_metrics(unlook_vcsel_handle_t handle,
                                              unlook_vcsel_metrics_t* metrics) {
    if (!handle || !handle->projector || !metrics) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        auto cpp_metrics = handle->projector->getPerformanceMetrics();

        metrics->avg_sync_error_us = cpp_metrics.avg_sync_error_us;
        metrics->max_sync_error_us = cpp_metrics.max_sync_error_us;
        metrics->thermal_efficiency = cpp_metrics.thermal_efficiency;
        metrics->power_consumption_w = cpp_metrics.power_consumption_w;
        metrics->total_projection_time_ms = cpp_metrics.total_projection_time_ms;
        metrics->successful_syncs = cpp_metrics.successful_syncs;
        metrics->failed_syncs = cpp_metrics.failed_syncs;

        return UNLOOK_VCSEL_SUCCESS;
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: getMetrics failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_get_thermal_status(unlook_vcsel_handle_t handle,
                                                     unlook_vcsel_thermal_t* thermal) {
    if (!handle || !handle->projector || !thermal) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        auto cpp_thermal = handle->projector->getThermalStatus();

        thermal->current_temp_c = cpp_thermal.current_temp_c;
        thermal->max_safe_temp_c = cpp_thermal.max_safe_temp_c;
        thermal->thermal_protection_active = cpp_thermal.thermal_protection_active;
        thermal->throttled_current_ma = cpp_thermal.throttled_current_ma;
        thermal->original_current_ma = 250; // Default assumption
        thermal->throttle_percent = 0.0f;   // Would need calculation

        return UNLOOK_VCSEL_SUCCESS;
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: getThermalStatus failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_is_ready(unlook_vcsel_handle_t handle, bool* ready) {
    if (!handle || !handle->projector || !ready) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        *ready = handle->projector->isReady();
        return UNLOOK_VCSEL_SUCCESS;
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: isReady failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_is_projection_active(unlook_vcsel_handle_t handle, bool* active) {
    if (!handle || !handle->projector || !active) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        *active = handle->projector->isProjectionActive();
        return UNLOOK_VCSEL_SUCCESS;
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: isProjectionActive failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_run_diagnostics(unlook_vcsel_handle_t handle,
                                                   unlook_vcsel_diagnostic_t* result) {
    if (!handle || !handle->projector || !result) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        auto cpp_result = handle->projector->runDiagnostics();

        result->i2c_communication = cpp_result.i2c_communication;
        result->gpio_control = cpp_result.gpio_control;
        result->led1_functional = cpp_result.led1_functional;
        result->led2_functional = cpp_result.led2_functional;
        result->thermal_sensor = cpp_result.thermal_sensor;
        result->sync_timing = cpp_result.sync_timing;
        result->measured_current_led1_ma = cpp_result.measured_current_led1_ma;
        result->measured_current_led2_ma = cpp_result.measured_current_led2_ma;
        result->measured_temperature_c = cpp_result.measured_temperature_c;

        result->error_count = std::min(cpp_result.error_details.size(), static_cast<size_t>(10));
        for (size_t i = 0; i < result->error_count; i++) {
            strncpy(result->error_details[i], cpp_result.error_details[i].c_str(), 127);
            result->error_details[i][127] = '\0';
        }

        return UNLOOK_VCSEL_SUCCESS;
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: runDiagnostics failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

// Additional utility functions

const char* unlook_vcsel_get_error_string(unlook_vcsel_result_t result) {
    switch (result) {
        case UNLOOK_VCSEL_SUCCESS: return "Success";
        case UNLOOK_VCSEL_ERROR_NOT_INITIALIZED: return "Not initialized";
        case UNLOOK_VCSEL_ERROR_INVALID_PARAMETER: return "Invalid parameter";
        case UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE: return "Hardware failure";
        case UNLOOK_VCSEL_ERROR_THERMAL_PROTECTION: return "Thermal protection active";
        case UNLOOK_VCSEL_ERROR_SYNC_FAILURE: return "Synchronization failure";
        case UNLOOK_VCSEL_ERROR_SAFETY_VIOLATION: return "Safety violation";
        case UNLOOK_VCSEL_ERROR_TIMEOUT: return "Operation timeout";
        case UNLOOK_VCSEL_ERROR_NOT_READY: return "System not ready";
        case UNLOOK_VCSEL_ERROR_EMERGENCY_SHUTDOWN: return "Emergency shutdown";
        default: return "Unknown error";
    }
}

const char* unlook_vcsel_get_mode_string(unlook_vcsel_mode_t mode) {
    switch (mode) {
        case UNLOOK_VCSEL_MODE_DISABLED: return "Disabled";
        case UNLOOK_VCSEL_MODE_DEPTH_CAPTURE: return "Depth Capture";
        case UNLOOK_VCSEL_MODE_FACE_RECOGNITION: return "Face Recognition";
        case UNLOOK_VCSEL_MODE_STRUCTURED_LIGHT: return "Structured Light";
        case UNLOOK_VCSEL_MODE_FLOOD_ILLUMINATION: return "Flood Illumination";
        case UNLOOK_VCSEL_MODE_DIAGNOSTIC: return "Diagnostic";
        default: return "Unknown mode";
    }
}

const char* unlook_vcsel_get_pattern_string(unlook_vcsel_pattern_t pattern) {
    switch (pattern) {
        case UNLOOK_VCSEL_PATTERN_NONE: return "None (Flood Only)";
        case UNLOOK_VCSEL_PATTERN_DOTS_15K: return "15k Dots";
        case UNLOOK_VCSEL_PATTERN_LINES_HORIZONTAL: return "Horizontal Lines";
        case UNLOOK_VCSEL_PATTERN_LINES_VERTICAL: return "Vertical Lines";
        case UNLOOK_VCSEL_PATTERN_GRID: return "Grid Pattern";
        case UNLOOK_VCSEL_PATTERN_RANDOM_SPECKLE: return "Random Speckle";
        case UNLOOK_VCSEL_PATTERN_ADAPTIVE: return "Adaptive Pattern";
        case UNLOOK_VCSEL_PATTERN_CUSTOM: return "Custom Pattern";
        default: return "Unknown pattern";
    }
}

bool unlook_vcsel_validate_config(const unlook_vcsel_config_t* config,
                                 char* error_message,
                                 size_t buffer_size) {
    if (!config) {
        if (error_message && buffer_size > 0) {
            strncpy(error_message, "Configuration is null", buffer_size - 1);
            error_message[buffer_size - 1] = '\0';
        }
        return false;
    }

    if (config->vcsel_current_ma == 0 || config->vcsel_current_ma > 400) {
        if (error_message && buffer_size > 0) {
            snprintf(error_message, buffer_size, "Invalid VCSEL current: %dmA (valid range: 1-400mA)",
                    config->vcsel_current_ma);
        }
        return false;
    }

    if (config->flood_current_ma > 250) {
        if (error_message && buffer_size > 0) {
            snprintf(error_message, buffer_size, "Invalid flood current: %dmA (max: 250mA)",
                    config->flood_current_ma);
        }
        return false;
    }

    if (config->max_duty_cycle <= 0.0f || config->max_duty_cycle > 1.0f) {
        if (error_message && buffer_size > 0) {
            snprintf(error_message, buffer_size, "Invalid duty cycle: %.2f (valid range: 0.01-1.0)",
                    config->max_duty_cycle);
        }
        return false;
    }

    return true;
}

// Convenience functions

unlook_vcsel_result_t unlook_vcsel_quick_setup_depth(unlook_vcsel_handle_t* handle) {
    if (!handle) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    unlook_vcsel_config_t config;
    unlook_vcsel_result_t result = unlook_vcsel_get_default_config(&config);
    if (result != UNLOOK_VCSEL_SUCCESS) {
        return result;
    }

    config.mode = UNLOOK_VCSEL_MODE_DEPTH_CAPTURE;
    config.pattern = UNLOOK_VCSEL_PATTERN_DOTS_15K;
    config.enable_camera_sync = true;

    result = unlook_vcsel_create(&config, handle);
    if (result != UNLOOK_VCSEL_SUCCESS) {
        return result;
    }

    return unlook_vcsel_enable_depth_capture(*handle, UNLOOK_VCSEL_PATTERN_DOTS_15K);
}

unlook_vcsel_result_t unlook_vcsel_quick_setup_face(unlook_vcsel_handle_t* handle, float face_distance_mm) {
    if (!handle) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    unlook_vcsel_config_t config;
    unlook_vcsel_result_t result = unlook_vcsel_get_default_config(&config);
    if (result != UNLOOK_VCSEL_SUCCESS) {
        return result;
    }

    config.mode = UNLOOK_VCSEL_MODE_FACE_RECOGNITION;
    config.pattern = UNLOOK_VCSEL_PATTERN_ADAPTIVE;
    config.face_distance_mm = face_distance_mm;
    config.enable_camera_sync = false;

    result = unlook_vcsel_create(&config, handle);
    if (result != UNLOOK_VCSEL_SUCCESS) {
        return result;
    }

    return unlook_vcsel_enable_face_recognition(*handle, face_distance_mm);
}

unlook_vcsel_result_t unlook_vcsel_simple_capture(unlook_vcsel_handle_t handle) {
    if (!handle) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    return unlook_vcsel_trigger_capture(handle, 10000, nullptr, nullptr); // 10ms exposure
}

unlook_vcsel_result_t unlook_vcsel_wait_for_completion(unlook_vcsel_handle_t handle, uint32_t timeout_ms) {
    if (!handle) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - start_time).count() < timeout_ms) {

        bool active = false;
        unlook_vcsel_result_t result = unlook_vcsel_is_projection_active(handle, &active);
        if (result != UNLOOK_VCSEL_SUCCESS) {
            return result;
        }

        if (!active) {
            return UNLOOK_VCSEL_SUCCESS;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return UNLOOK_VCSEL_ERROR_TIMEOUT;
}

// Callback setters (implement remaining ones)
unlook_vcsel_result_t unlook_vcsel_set_projection_callback(unlook_vcsel_handle_t handle,
                                                          unlook_vcsel_projection_callback_t callback,
                                                          void* user_data) {
    if (!handle) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    std::lock_guard<std::mutex> lock(handle->callback_mutex);
    handle->projection_callback = callback;
    handle->projection_user_data = user_data;

    return UNLOOK_VCSEL_SUCCESS;
}

// Implement remaining callback setters similarly...
unlook_vcsel_result_t unlook_vcsel_set_thermal_callback(unlook_vcsel_handle_t handle,
                                                       unlook_vcsel_thermal_callback_t callback,
                                                       void* user_data) {
    if (!handle) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    std::lock_guard<std::mutex> lock(handle->callback_mutex);
    handle->thermal_callback = callback;
    handle->thermal_user_data = user_data;

    // Set up C++ callback wrapper
    if (callback) {
        auto cpp_callback = [handle](bool thermal_active, float temperature_c) {
            std::lock_guard<std::mutex> cb_lock(handle->callback_mutex);
            if (handle->thermal_callback) {
                handle->thermal_callback(thermal_active, temperature_c, handle->thermal_user_data);
            }
        };
        handle->projector->setThermalCallback(cpp_callback);
    } else {
        handle->projector->setThermalCallback(nullptr);
    }

    return UNLOOK_VCSEL_SUCCESS;
}

unlook_vcsel_result_t unlook_vcsel_set_error_callback(unlook_vcsel_handle_t handle,
                                                     unlook_vcsel_error_callback_t callback,
                                                     void* user_data) {
    if (!handle) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    std::lock_guard<std::mutex> lock(handle->callback_mutex);
    handle->error_callback = callback;
    handle->error_user_data = user_data;

    if (callback) {
        auto cpp_callback = [handle](const std::string& error) {
            std::lock_guard<std::mutex> cb_lock(handle->callback_mutex);
            if (handle->error_callback) {
                handle->error_callback(error.c_str(), handle->error_user_data);
            }
        };
        handle->projector->setErrorCallback(cpp_callback);
    } else {
        handle->projector->setErrorCallback(nullptr);
    }

    return UNLOOK_VCSEL_SUCCESS;
}

unlook_vcsel_result_t unlook_vcsel_set_sync_callback(unlook_vcsel_handle_t handle,
                                                    unlook_vcsel_sync_callback_t callback,
                                                    void* user_data) {
    if (!handle) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    std::lock_guard<std::mutex> lock(handle->callback_mutex);
    handle->sync_callback = callback;
    handle->sync_user_data = user_data;

    if (callback) {
        auto cpp_callback = [handle](uint64_t camera_timestamp_ns, uint64_t led_timestamp_ns) {
            std::lock_guard<std::mutex> cb_lock(handle->callback_mutex);
            if (handle->sync_callback) {
                handle->sync_callback(camera_timestamp_ns, led_timestamp_ns, handle->sync_user_data);
            }
        };
        handle->projector->setSyncCallback(cpp_callback);
    } else {
        handle->projector->setSyncCallback(nullptr);
    }

    return UNLOOK_VCSEL_SUCCESS;
}

// Implement remaining functions (calibration, optimization)
unlook_vcsel_result_t unlook_vcsel_calibrate_intensity(unlook_vcsel_handle_t handle,
                                                      float target_distance_mm) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        bool success = handle->projector->calibrateProjectionIntensity(target_distance_mm);
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: calibrateIntensity failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_optimize_for_depth(unlook_vcsel_handle_t handle) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        bool success = handle->projector->optimizeForDepthAccuracy();
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: optimizeForDepth failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

unlook_vcsel_result_t unlook_vcsel_optimize_for_faces(unlook_vcsel_handle_t handle) {
    if (!handle || !handle->projector) {
        return UNLOOK_VCSEL_ERROR_INVALID_PARAMETER;
    }

    try {
        bool success = handle->projector->optimizeForFaceRecognition();
        return convert_result(success);
    } catch (const std::exception& e) {
        unlook::core::Logger::error("VCSEL API: optimizeForFaces failed: {}", e.what());
        return UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE;
    }
}

} // extern "C"