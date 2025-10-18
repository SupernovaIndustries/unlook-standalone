/**
 * @file GeometricSwipeDetector.cpp
 * @brief Implementation of geometric swipe detection
 */

#include "unlook/gesture/GeometricSwipeDetector.hpp"
#include "unlook/core/Logger.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

namespace unlook {
namespace gesture {

/**
 * @brief PIMPL implementation for GeometricSwipeDetector
 */
class GeometricSwipeDetector::Impl {
public:
    SwipeConfig config;
    float last_confidence = 0.0f;

    // Last detection motion statistics
    cv::Point2f last_displacement;
    cv::Point2f last_velocity;
    float last_scale_change = 0.0f;

    // Debouncing state
    GestureType last_detected_gesture = GestureType::UNKNOWN;
    std::chrono::steady_clock::time_point last_detection_time;
    int frames_since_detection = 0;

    explicit Impl(const SwipeConfig& cfg)
        : config(cfg)
        , last_detection_time(std::chrono::steady_clock::now() - std::chrono::hours(1)) {
    }

    /**
     * @brief Check if currently in debounce period
     */
    bool is_debouncing() const {
        if (last_detected_gesture == GestureType::UNKNOWN) {
            return false;
        }

        // Check frame-based debounce
        if (frames_since_detection < config.debounce_frames) {
            return true;
        }

        // Check time-based debounce
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration<double, std::milli>(
            now - last_detection_time
        ).count();

        return elapsed_ms < config.debounce_duration_ms;
    }

    /**
     * @brief Detect swipe gesture from temporal buffer
     */
    GestureType detect(const TemporalBuffer& buffer) {
        frames_since_detection++;

        // Reset state
        last_confidence = 0.0f;
        last_displacement = cv::Point2f(0, 0);
        last_velocity = cv::Point2f(0, 0);
        last_scale_change = 0.0f;

        LOG_INFO("GeometricSwipeDetector: Analyzing buffer with " + std::to_string(buffer.size()) + " frames");

        // Check debouncing
        if (is_debouncing()) {
            LOG_INFO("  SKIP: In debounce period (frames_since=" + std::to_string(frames_since_detection) +
                    ", debounce_frames=" + std::to_string(config.debounce_frames) + ")");
            return GestureType::UNKNOWN;
        }

        // Check minimum buffer requirements
        if (!buffer.has_minimum_frames(config.min_frames)) {
            LOG_INFO("  SKIP: Buffer has too few frames (" + std::to_string(buffer.size()) +
                    " < min=" + std::to_string(config.min_frames) + ")");
            return GestureType::UNKNOWN;
        }

        size_t frame_count = buffer.size();
        if (frame_count > static_cast<size_t>(config.max_frames)) {
            LOG_INFO("  SKIP: Gesture too slow (" + std::to_string(frame_count) +
                    " > max=" + std::to_string(config.max_frames) + " frames)");
            return GestureType::UNKNOWN;  // Gesture too slow
        }

        // Check duration constraints
        double duration_ms = buffer.compute_duration_ms();
        if (duration_ms < config.min_duration_ms || duration_ms > config.max_duration_ms) {
            LOG_INFO("  SKIP: Duration out of range (" + std::to_string(duration_ms) +
                    " ms, min=" + std::to_string(config.min_duration_ms) +
                    ", max=" + std::to_string(config.max_duration_ms) + ")");
            return GestureType::UNKNOWN;
        }

        // Compute motion statistics
        last_displacement = buffer.compute_displacement_vector();
        last_velocity = buffer.compute_average_velocity();
        last_scale_change = buffer.compute_scale_change();

        float displacement_magnitude = std::sqrt(
            last_displacement.x * last_displacement.x +
            last_displacement.y * last_displacement.y
        );

        float velocity_magnitude = std::sqrt(
            last_velocity.x * last_velocity.x +
            last_velocity.y * last_velocity.y
        );

        LOG_INFO("  Motion statistics:");
        LOG_INFO("    displacement_magnitude: " + std::to_string(displacement_magnitude) + " px");
        LOG_INFO("    velocity_magnitude: " + std::to_string(velocity_magnitude) + " px/frame");
        LOG_INFO("    duration: " + std::to_string(duration_ms) + " ms");
        LOG_INFO("    scale_change: " + std::to_string(last_scale_change));

        // Detect gesture type based on dominant motion axis
        GestureType detected = GestureType::UNKNOWN;
        float confidence = 0.0f;

        // ========== PRIORITY 1: Check depth swipes FIRST (FORWARD/BACKWARD) ==========
        // These don't require lateral movement, only scale change
        if (std::abs(last_scale_change) > config.scale_change_threshold) {
            // Check if scale change is consistent (not just noise)
            float scale_velocity = last_scale_change / static_cast<float>(frame_count);
            if (std::abs(scale_velocity) > config.min_scale_velocity) {

                if (last_scale_change > 0) {
                    detected = GestureType::SWIPE_FORWARD;
                } else {
                    detected = GestureType::SWIPE_BACKWARD;
                }

                // Calculate confidence
                confidence = 0.5f;
                confidence += std::min(0.3f, (std::abs(last_scale_change) - config.scale_change_threshold) / config.scale_change_threshold * 0.2f);
                confidence += std::min(0.2f, std::abs(scale_velocity) / config.min_scale_velocity * 0.1f);

                LOG_INFO("  >>> DEPTH SWIPE DETECTED (ABSOLUTE PRIORITY) <<<");
                LOG_INFO("      Type: " + std::string((last_scale_change > 0) ? "FORWARD" : "BACKWARD"));
                LOG_INFO("      scale_change: " + std::to_string(last_scale_change));
                LOG_INFO("      scale_velocity: " + std::to_string(scale_velocity));
                LOG_INFO("      confidence: " + std::to_string(confidence));

                // Clamp and return immediately - depth swipes have absolute priority
                confidence = std::max(0.0f, std::min(1.0f, confidence));
                last_confidence = confidence;
                last_detected_gesture = detected;
                last_detection_time = std::chrono::steady_clock::now();
                frames_since_detection = 0;
                return detected;
            }
        }

        // ========== Check minimum motion thresholds for lateral swipes ==========
        // These only apply to LEFT/RIGHT/UP/DOWN, not FORWARD/BACKWARD
        if (displacement_magnitude < config.min_displacement) {
            LOG_INFO("  FAIL: Displacement too small (" + std::to_string(displacement_magnitude) +
                    " < " + std::to_string(config.min_displacement) + ")");
            return GestureType::UNKNOWN;
        }

        if (velocity_magnitude < config.min_velocity) {
            LOG_INFO("  FAIL: Velocity too low (" + std::to_string(velocity_magnitude) +
                    " < " + std::to_string(config.min_velocity) + ")");
            return GestureType::UNKNOWN;
        }

        // Calculate axis dominance
        float abs_dx = std::abs(last_displacement.x);
        float abs_dy = std::abs(last_displacement.y);
        float abs_vx = std::abs(last_velocity.x);
        float abs_vy = std::abs(last_velocity.y);

        // Normalize direction vectors
        float dx_ratio = (displacement_magnitude > 0.0f) ? (abs_dx / displacement_magnitude) : 0.0f;
        float dy_ratio = (displacement_magnitude > 0.0f) ? (abs_dy / displacement_magnitude) : 0.0f;
        float vx_ratio = (velocity_magnitude > 0.0f) ? (abs_vx / velocity_magnitude) : 0.0f;
        float vy_ratio = (velocity_magnitude > 0.0f) ? (abs_vy / velocity_magnitude) : 0.0f;

        LOG_INFO("  Direction analysis:");
        LOG_INFO("    dx_ratio: " + std::to_string(dx_ratio) + " (threshold: " + std::to_string(config.direction_threshold) + ")");
        LOG_INFO("    dy_ratio: " + std::to_string(dy_ratio) + " (threshold: " + std::to_string(config.direction_threshold) + ")");
        LOG_INFO("    vx_ratio: " + std::to_string(vx_ratio) + " (purity: " + std::to_string(config.direction_purity) + ")");
        LOG_INFO("    vy_ratio: " + std::to_string(vy_ratio) + " (purity: " + std::to_string(config.direction_purity) + ")");
        LOG_INFO("    Dominant axis: " + std::string((abs_dx > abs_dy) ? "HORIZONTAL" : "VERTICAL"));

        // NOTE: Depth swipes (FORWARD/BACKWARD) are checked FIRST at the top of this function
        // and return immediately if detected. We only reach here for lateral swipes.

        // Check horizontal swipes (LEFT/RIGHT)
        if (
            dx_ratio > config.direction_threshold &&
            vx_ratio > config.direction_purity &&
            abs_dx > config.min_displacement_horizontal &&
            abs_vx > config.min_velocity) {

            if (last_displacement.x < 0) {
                detected = GestureType::SWIPE_LEFT;
            } else {
                detected = GestureType::SWIPE_RIGHT;
            }

            // Calculate confidence based on strength of motion
            confidence = 0.5f;  // Base confidence
            confidence += std::min(0.2f, (abs_vx - config.min_velocity) / config.min_velocity * 0.1f);
            confidence += std::min(0.2f, (abs_dx - config.min_displacement_horizontal) / config.min_displacement_horizontal * 0.1f);
            confidence += (dx_ratio - config.direction_threshold) * 0.5f;

            LOG_INFO("  HORIZONTAL SWIPE DETECTED: " + std::string((last_displacement.x < 0) ? "LEFT" : "RIGHT"));
        } else {
            // Log why horizontal swipe failed
            if (dx_ratio > 0.3f) {  // Only log if there was significant horizontal movement
                LOG_INFO("  FAIL: Horizontal swipe checks:");
                if (dx_ratio <= config.direction_threshold) {
                    LOG_INFO("    dx_ratio too low: " + std::to_string(dx_ratio) + " <= " + std::to_string(config.direction_threshold));
                }
                if (vx_ratio <= config.direction_purity) {
                    LOG_INFO("    vx_ratio too low: " + std::to_string(vx_ratio) + " <= " + std::to_string(config.direction_purity));
                }
                if (abs_dx <= config.min_displacement_horizontal) {
                    LOG_INFO("    abs_dx too small: " + std::to_string(abs_dx) + " <= " + std::to_string(config.min_displacement_horizontal));
                }
            }
        }

        // Priority 2: Check vertical swipes (UP/DOWN)
        if (detected == GestureType::UNKNOWN &&
            dy_ratio > config.direction_threshold &&
            vy_ratio > config.direction_purity &&
            abs_dy > config.min_displacement_vertical &&
            abs_vy > config.min_velocity) {

            if (last_displacement.y < 0) {
                detected = GestureType::SWIPE_UP;
            } else {
                detected = GestureType::SWIPE_DOWN;
            }

            // Calculate confidence
            confidence = 0.5f;
            confidence += std::min(0.2f, (abs_vy - config.min_velocity) / config.min_velocity * 0.1f);
            confidence += std::min(0.2f, (abs_dy - config.min_displacement_vertical) / config.min_displacement_vertical * 0.1f);
            confidence += (dy_ratio - config.direction_threshold) * 0.5f;

            LOG_INFO("  VERTICAL SWIPE DETECTED: " + std::string((last_displacement.y < 0) ? "UP" : "DOWN"));
        } else if (detected == GestureType::UNKNOWN && dy_ratio > 0.3f) {
            // Log why vertical swipe failed
            LOG_INFO("  FAIL: Vertical swipe checks:");
            if (dy_ratio <= config.direction_threshold) {
                LOG_INFO("    dy_ratio too low: " + std::to_string(dy_ratio) + " <= " + std::to_string(config.direction_threshold));
            }
            if (vy_ratio <= config.direction_purity) {
                LOG_INFO("    vy_ratio too low: " + std::to_string(vy_ratio) + " <= " + std::to_string(config.direction_purity));
            }
            if (abs_dy <= config.min_displacement_vertical) {
                LOG_INFO("    abs_dy too small: " + std::to_string(abs_dy) + " <= " + std::to_string(config.min_displacement_vertical));
            }
        }

        // NOTE: Depth swipes (FORWARD/BACKWARD) are now checked FIRST (Priority 1) above
        // This section has been moved to give depth swipes priority over lateral movement

        // Clamp confidence to [0, 1]
        confidence = std::max(0.0f, std::min(1.0f, confidence));

        // Store results
        last_confidence = confidence;

        // Final result logging
        if (detected != GestureType::UNKNOWN && confidence > 0.4f) {
            LOG_INFO("  >>> SWIPE CONFIRMED <<<");
            LOG_INFO("    Type: " + std::to_string(static_cast<int>(detected)));
            LOG_INFO("    Confidence: " + std::to_string(confidence));

            last_detected_gesture = detected;
            last_detection_time = std::chrono::steady_clock::now();
            frames_since_detection = 0;
        } else if (detected != GestureType::UNKNOWN) {
            LOG_INFO("  SWIPE REJECTED: Confidence too low (" + std::to_string(confidence) + " <= 0.4)");
        } else {
            LOG_INFO("  RESULT: No swipe detected");
        }

        return detected;
    }

    void reset() {
        last_detected_gesture = GestureType::UNKNOWN;
        last_detection_time = std::chrono::steady_clock::now() - std::chrono::hours(1);
        frames_since_detection = 999;
        last_confidence = 0.0f;
    }
};

// ===== Public API Implementation =====

GeometricSwipeDetector::GeometricSwipeDetector()
    : pImpl(std::make_unique<Impl>(SwipeConfig{})) {
}

GeometricSwipeDetector::GeometricSwipeDetector(const SwipeConfig& config)
    : pImpl(std::make_unique<Impl>(config)) {
}

GeometricSwipeDetector::~GeometricSwipeDetector() = default;

void GeometricSwipeDetector::configure(const SwipeConfig& config) {
    if (config.is_valid()) {
        pImpl->config = config;
    }
}

GestureType GeometricSwipeDetector::detect(const TemporalBuffer& buffer) {
    return pImpl->detect(buffer);
}

float GeometricSwipeDetector::get_confidence() const {
    return pImpl->last_confidence;
}

void GeometricSwipeDetector::get_motion_stats(cv::Point2f& displacement,
                                               cv::Point2f& velocity,
                                               float& scale_change) const {
    displacement = pImpl->last_displacement;
    velocity = pImpl->last_velocity;
    scale_change = pImpl->last_scale_change;
}

void GeometricSwipeDetector::reset() {
    pImpl->reset();
}

bool GeometricSwipeDetector::is_debouncing() const {
    return pImpl->is_debouncing();
}

SwipeConfig GeometricSwipeDetector::get_config() const {
    return pImpl->config;
}

} // namespace gesture
} // namespace unlook
