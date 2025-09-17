/**
 * @file vcsel_control.h
 * @brief Public API for VCSEL Projector Control
 *
 * High-level C API for controlling the OSRAM BELAGO 15k VCSEL projector system.
 * Provides easy integration with external applications and language bindings.
 *
 * Features:
 * - Industrial-grade 15k points structured light projection
 * - Hardware-synchronized depth capture integration
 * - Face recognition optimized illumination
 * - Comprehensive thermal protection and safety
 * - Real-time performance monitoring
 * - Thread-safe operation
 *
 * Safety Features:
 * - Automatic thermal throttling and protection
 * - Current limiting to prevent hardware damage
 * - Emergency shutdown capabilities (<5ms response)
 * - Duty cycle monitoring and enforcement
 *
 * Integration Points:
 * - Camera system synchronization
 * - GUI controls and status display
 * - Depth processing pipeline
 * - Face recognition system
 *
 * @author Unlook Hardware Interface Agent
 * @version 1.0.0
 * @date 2024
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// API Version
#define UNLOOK_VCSEL_API_VERSION_MAJOR 1
#define UNLOOK_VCSEL_API_VERSION_MINOR 0
#define UNLOOK_VCSEL_API_VERSION_PATCH 0

// Error Codes
typedef enum {
    UNLOOK_VCSEL_SUCCESS = 0,
    UNLOOK_VCSEL_ERROR_NOT_INITIALIZED = -1,
    UNLOOK_VCSEL_ERROR_INVALID_PARAMETER = -2,
    UNLOOK_VCSEL_ERROR_HARDWARE_FAILURE = -3,
    UNLOOK_VCSEL_ERROR_THERMAL_PROTECTION = -4,
    UNLOOK_VCSEL_ERROR_SYNC_FAILURE = -5,
    UNLOOK_VCSEL_ERROR_SAFETY_VIOLATION = -6,
    UNLOOK_VCSEL_ERROR_TIMEOUT = -7,
    UNLOOK_VCSEL_ERROR_NOT_READY = -8,
    UNLOOK_VCSEL_ERROR_EMERGENCY_SHUTDOWN = -9
} unlook_vcsel_result_t;

// Projection Modes
typedef enum {
    UNLOOK_VCSEL_MODE_DISABLED = 0,
    UNLOOK_VCSEL_MODE_DEPTH_CAPTURE = 1,
    UNLOOK_VCSEL_MODE_FACE_RECOGNITION = 2,
    UNLOOK_VCSEL_MODE_STRUCTURED_LIGHT = 3,
    UNLOOK_VCSEL_MODE_FLOOD_ILLUMINATION = 4,
    UNLOOK_VCSEL_MODE_DIAGNOSTIC = 5
} unlook_vcsel_mode_t;

// Pattern Types
typedef enum {
    UNLOOK_VCSEL_PATTERN_NONE = 0,
    UNLOOK_VCSEL_PATTERN_DOTS_15K = 1,
    UNLOOK_VCSEL_PATTERN_LINES_HORIZONTAL = 2,
    UNLOOK_VCSEL_PATTERN_LINES_VERTICAL = 3,
    UNLOOK_VCSEL_PATTERN_GRID = 4,
    UNLOOK_VCSEL_PATTERN_RANDOM_SPECKLE = 5,
    UNLOOK_VCSEL_PATTERN_ADAPTIVE = 6,
    UNLOOK_VCSEL_PATTERN_CUSTOM = 7
} unlook_vcsel_pattern_t;

// Configuration Structure
typedef struct {
    // Basic configuration
    unlook_vcsel_mode_t mode;
    unlook_vcsel_pattern_t pattern;

    // Current settings (mA)
    uint16_t vcsel_current_ma;     // VCSEL LED current
    uint16_t flood_current_ma;     // Flood LED current

    // Timing configuration (ms/us)
    uint32_t projection_duration_ms;
    uint32_t cool_down_delay_ms;
    uint32_t sync_tolerance_us;

    // Safety parameters
    float max_temperature_c;
    float max_duty_cycle;
    uint32_t max_continuous_time_ms;
    uint32_t mandatory_pause_ms;

    // Feature flags
    bool enable_flood_assist;
    bool enable_thermal_protection;
    bool enable_camera_sync;
    bool adaptive_current;

    // Face recognition optimization
    float face_distance_mm;
} unlook_vcsel_config_t;

// Status Structure
typedef struct {
    // System state
    bool initialized;
    bool projection_active;
    bool ready;
    unlook_vcsel_mode_t current_mode;
    unlook_vcsel_pattern_t current_pattern;

    // Current measurements
    uint16_t vcsel_current_ma;
    uint16_t flood_current_ma;
    float temperature_c;
    bool thermal_protection_active;

    // Performance metrics
    uint64_t projection_cycles;
    uint64_t sync_captures;
    uint64_t thermal_events;
    double avg_projection_time_ms;
    double current_duty_cycle;

    // System health
    bool hardware_ok;
    bool sync_ok;
    bool thermal_ok;

    // Error information
    char last_error[256];
    uint64_t last_update_timestamp_ns;
} unlook_vcsel_status_t;

// Performance Metrics Structure
typedef struct {
    double avg_sync_error_us;
    double max_sync_error_us;
    double thermal_efficiency;
    double power_consumption_w;
    uint64_t total_projection_time_ms;
    uint64_t successful_syncs;
    uint64_t failed_syncs;
} unlook_vcsel_metrics_t;

// Thermal Status Structure
typedef struct {
    float current_temp_c;
    float max_safe_temp_c;
    bool thermal_protection_active;
    uint16_t throttled_current_ma;
    uint16_t original_current_ma;
    float throttle_percent;
} unlook_vcsel_thermal_t;

// Diagnostic Result Structure
typedef struct {
    bool i2c_communication;
    bool gpio_control;
    bool led1_functional;
    bool led2_functional;
    bool thermal_sensor;
    bool sync_timing;
    float measured_current_led1_ma;
    float measured_current_led2_ma;
    float measured_temperature_c;
    char error_details[10][128]; // Up to 10 error messages
    size_t error_count;
} unlook_vcsel_diagnostic_t;

// Callback Function Types
typedef void (*unlook_vcsel_projection_callback_t)(unlook_vcsel_pattern_t pattern, uint64_t timestamp_ns, void* user_data);
typedef void (*unlook_vcsel_thermal_callback_t)(bool thermal_active, float temperature_c, void* user_data);
typedef void (*unlook_vcsel_error_callback_t)(const char* error_message, void* user_data);
typedef void (*unlook_vcsel_sync_callback_t)(uint64_t camera_timestamp_ns, uint64_t led_timestamp_ns, void* user_data);

// Opaque handle for VCSEL projector instance
typedef struct unlook_vcsel_projector* unlook_vcsel_handle_t;

/**
 * @brief Get API version information
 * @param major Pointer to store major version
 * @param minor Pointer to store minor version
 * @param patch Pointer to store patch version
 */
void unlook_vcsel_get_version(int* major, int* minor, int* patch);

/**
 * @brief Get default configuration
 * @param config Pointer to configuration structure to fill
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_get_default_config(unlook_vcsel_config_t* config);

/**
 * @brief Create and initialize VCSEL projector
 * @param config Configuration parameters (NULL for defaults)
 * @param handle Pointer to store handle
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_create(const unlook_vcsel_config_t* config, unlook_vcsel_handle_t* handle);

/**
 * @brief Destroy VCSEL projector and cleanup resources
 * @param handle VCSEL projector handle
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_destroy(unlook_vcsel_handle_t handle);

/**
 * @brief Enable depth capture mode
 * @param handle VCSEL projector handle
 * @param pattern Pattern type to use (UNLOOK_VCSEL_PATTERN_DOTS_15K recommended)
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_enable_depth_capture(unlook_vcsel_handle_t handle,
                                                       unlook_vcsel_pattern_t pattern);

/**
 * @brief Enable face recognition mode
 * @param handle VCSEL projector handle
 * @param distance_mm Typical face distance in mm (300mm default)
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_enable_face_recognition(unlook_vcsel_handle_t handle,
                                                          float distance_mm);

/**
 * @brief Enable structured light mode
 * @param handle VCSEL projector handle
 * @param pattern Structured light pattern type
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_enable_structured_light(unlook_vcsel_handle_t handle,
                                                          unlook_vcsel_pattern_t pattern);

/**
 * @brief Disable projection (safe shutdown)
 * @param handle VCSEL projector handle
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_disable_projection(unlook_vcsel_handle_t handle);

/**
 * @brief Trigger synchronized structured light capture
 * Main function for depth capture integration
 * @param handle VCSEL projector handle
 * @param exposure_time_us Camera exposure time for optimization
 * @param callback Optional callback when projection complete
 * @param user_data User data passed to callback
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_trigger_capture(unlook_vcsel_handle_t handle,
                                                  uint32_t exposure_time_us,
                                                  unlook_vcsel_projection_callback_t callback,
                                                  void* user_data);

/**
 * @brief Trigger face recognition illumination
 * @param handle VCSEL projector handle
 * @param duration_ms Illumination duration
 * @param callback Optional completion callback
 * @param user_data User data passed to callback
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_trigger_face_illumination(unlook_vcsel_handle_t handle,
                                                           uint32_t duration_ms,
                                                           unlook_vcsel_projection_callback_t callback,
                                                           void* user_data);

/**
 * @brief Set projection pattern and parameters
 * @param handle VCSEL projector handle
 * @param pattern Pattern type
 * @param vcsel_current_ma VCSEL current (0 = use default)
 * @param flood_current_ma Flood current (0 = use default)
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_set_pattern(unlook_vcsel_handle_t handle,
                                              unlook_vcsel_pattern_t pattern,
                                              uint16_t vcsel_current_ma,
                                              uint16_t flood_current_ma);

/**
 * @brief Manual projection control (for testing)
 * @param handle VCSEL projector handle
 * @param enable Enable or disable projection
 * @param duration_ms Duration in milliseconds (0 = until disabled)
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_set_manual_projection(unlook_vcsel_handle_t handle,
                                                        bool enable,
                                                        uint32_t duration_ms);

/**
 * @brief Configure camera synchronization
 * @param handle VCSEL projector handle
 * @param enable Enable sync with camera system
 * @param tolerance_us Timing tolerance in microseconds
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_configure_sync(unlook_vcsel_handle_t handle,
                                                 bool enable,
                                                 uint32_t tolerance_us);

/**
 * @brief Adaptive current control for conditions
 * @param handle VCSEL projector handle
 * @param ambient_light_lux Ambient light level
 * @param target_distance_mm Distance to target
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_adapt_current(unlook_vcsel_handle_t handle,
                                                float ambient_light_lux,
                                                float target_distance_mm);

/**
 * @brief Emergency shutdown - immediate LED disable
 * Thread-safe, can be called from any context
 * @param handle VCSEL projector handle
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_emergency_shutdown(unlook_vcsel_handle_t handle);

/**
 * @brief Get current projector status
 * @param handle VCSEL projector handle
 * @param status Pointer to status structure to fill
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_get_status(unlook_vcsel_handle_t handle,
                                             unlook_vcsel_status_t* status);

/**
 * @brief Get current configuration
 * @param handle VCSEL projector handle
 * @param config Pointer to configuration structure to fill
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_get_config(unlook_vcsel_handle_t handle,
                                             unlook_vcsel_config_t* config);

/**
 * @brief Get performance metrics
 * @param handle VCSEL projector handle
 * @param metrics Pointer to metrics structure to fill
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_get_metrics(unlook_vcsel_handle_t handle,
                                              unlook_vcsel_metrics_t* metrics);

/**
 * @brief Get thermal status
 * @param handle VCSEL projector handle
 * @param thermal Pointer to thermal structure to fill
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_get_thermal_status(unlook_vcsel_handle_t handle,
                                                     unlook_vcsel_thermal_t* thermal);

/**
 * @brief Check if projector is ready for operation
 * @param handle VCSEL projector handle
 * @param ready Pointer to store ready status
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_is_ready(unlook_vcsel_handle_t handle, bool* ready);

/**
 * @brief Check if projection is currently active
 * @param handle VCSEL projector handle
 * @param active Pointer to store active status
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_is_projection_active(unlook_vcsel_handle_t handle, bool* active);

/**
 * @brief Perform comprehensive self-test
 * @param handle VCSEL projector handle
 * @param result Pointer to diagnostic result structure
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_run_diagnostics(unlook_vcsel_handle_t handle,
                                                   unlook_vcsel_diagnostic_t* result);

/**
 * @brief Calibrate projection intensity for distance
 * @param handle VCSEL projector handle
 * @param target_distance_mm Target distance for calibration
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_calibrate_intensity(unlook_vcsel_handle_t handle,
                                                      float target_distance_mm);

/**
 * @brief Optimize projector for depth accuracy
 * @param handle VCSEL projector handle
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_optimize_for_depth(unlook_vcsel_handle_t handle);

/**
 * @brief Optimize projector for face recognition
 * @param handle VCSEL projector handle
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_optimize_for_faces(unlook_vcsel_handle_t handle);

/**
 * @brief Set projection completion callback
 * @param handle VCSEL projector handle
 * @param callback Callback function
 * @param user_data User data passed to callback
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_set_projection_callback(unlook_vcsel_handle_t handle,
                                                          unlook_vcsel_projection_callback_t callback,
                                                          void* user_data);

/**
 * @brief Set thermal event callback
 * @param handle VCSEL projector handle
 * @param callback Callback function
 * @param user_data User data passed to callback
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_set_thermal_callback(unlook_vcsel_handle_t handle,
                                                       unlook_vcsel_thermal_callback_t callback,
                                                       void* user_data);

/**
 * @brief Set error callback
 * @param handle VCSEL projector handle
 * @param callback Callback function
 * @param user_data User data passed to callback
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_set_error_callback(unlook_vcsel_handle_t handle,
                                                     unlook_vcsel_error_callback_t callback,
                                                     void* user_data);

/**
 * @brief Set camera sync callback
 * @param handle VCSEL projector handle
 * @param callback Callback function
 * @param user_data User data passed to callback
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_set_sync_callback(unlook_vcsel_handle_t handle,
                                                    unlook_vcsel_sync_callback_t callback,
                                                    void* user_data);

/**
 * @brief Get human-readable error string
 * @param result Error code
 * @return Error description string
 */
const char* unlook_vcsel_get_error_string(unlook_vcsel_result_t result);

/**
 * @brief Get human-readable mode string
 * @param mode Projection mode
 * @return Mode description string
 */
const char* unlook_vcsel_get_mode_string(unlook_vcsel_mode_t mode);

/**
 * @brief Get human-readable pattern string
 * @param pattern Pattern type
 * @return Pattern description string
 */
const char* unlook_vcsel_get_pattern_string(unlook_vcsel_pattern_t pattern);

/**
 * @brief Validate configuration parameters
 * @param config Configuration to validate
 * @param error_message Buffer for error message (can be NULL)
 * @param buffer_size Size of error message buffer
 * @return true if configuration is valid
 */
bool unlook_vcsel_validate_config(const unlook_vcsel_config_t* config,
                                 char* error_message,
                                 size_t buffer_size);

// Convenience Functions for Common Use Cases

/**
 * @brief Quick setup for depth capture
 * Creates projector, enables depth capture mode, and performs basic setup
 * @param handle Pointer to store handle
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_quick_setup_depth(unlook_vcsel_handle_t* handle);

/**
 * @brief Quick setup for face recognition
 * Creates projector, enables face recognition mode, and performs basic setup
 * @param handle Pointer to store handle
 * @param face_distance_mm Typical face distance (300mm default)
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_quick_setup_face(unlook_vcsel_handle_t* handle, float face_distance_mm);

/**
 * @brief Simple synchronized capture
 * Performs a single synchronized capture with automatic cleanup
 * @param handle VCSEL projector handle
 * @return Result code
 */
unlook_vcsel_result_t unlook_vcsel_simple_capture(unlook_vcsel_handle_t handle);

/**
 * @brief Wait for projection to complete
 * Blocks until current projection finishes
 * @param handle VCSEL projector handle
 * @param timeout_ms Timeout in milliseconds
 * @return Result code (UNLOOK_VCSEL_ERROR_TIMEOUT if timeout exceeded)
 */
unlook_vcsel_result_t unlook_vcsel_wait_for_completion(unlook_vcsel_handle_t handle, uint32_t timeout_ms);

// Thread Safety: All functions are thread-safe unless otherwise noted
// Memory Management: Caller is responsible for freeing resources created with unlook_vcsel_create
// Error Handling: All functions return unlook_vcsel_result_t - check return value before proceeding

#ifdef __cplusplus
}
#endif