/**
 * @file GestureRecognitionSystem.cpp
 * @brief Implementation of main gesture recognition system
 */

#include <unlook/gesture/GestureRecognitionSystem.hpp>
#include <unlook/gesture/TemporalBuffer.hpp>
#include <unlook/gesture/GeometricSwipeDetector.hpp>
#include <unlook/gesture/MediaPipeWrapper.hpp>
#include <unlook/gesture/MotionBlurCompensation.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <sstream>

namespace unlook {
namespace gesture {

// PIMPL implementation
class GestureRecognitionSystem::Impl {
public:
    bool initialized = false;
    bool running = false;
    GestureConfig config;
    std::string last_error;

    // MediaPipe backend
    std::unique_ptr<MediaPipeWrapper> mediapipe_detector;

    // Motion blur compensation preprocessing
    std::unique_ptr<MotionBlurCompensation> motion_blur_compensator;

    // Temporal processing for swipe detection
    std::unique_ptr<TemporalBuffer> temporal_buffer;
    std::unique_ptr<GeometricSwipeDetector> swipe_detector;

    // Callback
    GestureCallback callback = nullptr;
    void* user_data = nullptr;

    // Performance stats
    double avg_detection_time = 0.0;
    double avg_classification_time = 0.0;
    double avg_total_time = 0.0;
    size_t frame_count = 0;

    // Debug visualization
    cv::Mat debug_frame;
};

GestureRecognitionSystem::GestureRecognitionSystem()
    : pImpl(std::make_unique<Impl>()) {
}

GestureRecognitionSystem::~GestureRecognitionSystem() {
    stop();
}

bool GestureRecognitionSystem::initialize(
    std::shared_ptr<camera::SynchronizedCameraSystem> camera_system,
    const GestureConfig& config) {

    if (!config.is_valid()) {
        pImpl->last_error = "Invalid configuration";
        return false;
    }

    // Camera system is optional - can be nullptr if using process_frame() directly
    pImpl->config = config;

    LOG_INFO("GestureRecognitionSystem: Initializing MediaPipe backend");

    try {
        std::string model_path = "/home/alessandro/unlook-gesture/models/gesture_recognizer.task";

        pImpl->mediapipe_detector = std::make_unique<MediaPipeWrapper>(
            model_path,
            1,    // num_hands = 1 (single hand)
            0.4f, // min_detection_confidence (initial detection)
            0.3f  // min_tracking_confidence (lower for fast motion tolerance)
        );

        LOG_INFO("MediaPipe backend initialized successfully");

        // Initialize motion blur compensation (lightweight preprocessing)
        MotionBlurConfig blur_config;
        blur_config.bilateral_diameter = 5;   // Fast mode (5x5 kernel)
        blur_config.sigma_color = 50.0;       // Moderate smoothing
        blur_config.sigma_space = 50.0;       // Moderate spatial
        blur_config.enabled = true;           // Enable by default
        pImpl->motion_blur_compensator = std::make_unique<MotionBlurCompensation>(blur_config);

        LOG_INFO("Motion blur compensation initialized (bilateral filter d=5)");

        // Initialize temporal buffer and swipe detector for swipe gestures
        // Buffer: 7 frames capacity, 2 gap frames tolerance for fast swipes
        pImpl->temporal_buffer = std::make_unique<TemporalBuffer>(7, 2);
        pImpl->swipe_detector = std::make_unique<GeometricSwipeDetector>();

        // Configure swipe detector - BALANCED thresholds
        SwipeConfig swipe_config;
        // Directional swipes (LEFT/RIGHT/UP/DOWN) - require clear movement
        swipe_config.min_velocity = 30.0f;                      // 30 px/frame → very fast movement
        swipe_config.min_displacement = 100.0f;                 // 100 px → very clear gesture
        swipe_config.min_displacement_horizontal = 100.0f;      // 100 px horizontal
        swipe_config.min_displacement_vertical = 100.0f;        // 100 px vertical

        // Depth swipes (FORWARD/BACKWARD) - based on scale change
        swipe_config.scale_change_threshold = 0.20f;            // 20% scale change (real bounding box now!)
        swipe_config.min_scale_velocity = 0.015f;               // 1.5% scale change per frame

        swipe_config.direction_threshold = 0.5f;                // 50% axis dominance
        swipe_config.direction_purity = 0.5f;                   // 50% direction purity
        swipe_config.min_frames = 5;  // Minimum 5 consecutive frames with hand
        pImpl->swipe_detector->configure(swipe_config);

        LOG_INFO("Temporal buffer and swipe detector initialized (BALANCED THRESHOLDS)");
        LOG_INFO("  buffer_capacity=7 frames");
        LOG_INFO("  max_gap_frames=2 (tolerance for brief tracking loss)");
        LOG_INFO("  Directional swipes (LEFT/RIGHT/UP/DOWN):");
        LOG_INFO("    min_velocity=" + std::to_string(swipe_config.min_velocity) + " px/frame");
        LOG_INFO("    min_displacement=" + std::to_string(swipe_config.min_displacement) + " px");
        LOG_INFO("  Depth swipes (FORWARD/BACKWARD):");
        LOG_INFO("    scale_change_threshold=" + std::to_string(swipe_config.scale_change_threshold) + " (20% - real bbox)");
        LOG_INFO("    min_scale_velocity=" + std::to_string(swipe_config.min_scale_velocity) + " (1.5% per frame)");
        LOG_INFO("  direction_threshold=" + std::to_string(swipe_config.direction_threshold));
        LOG_INFO("  min_frames=" + std::to_string(swipe_config.min_frames));

        pImpl->initialized = true;
        LOG_INFO("GestureRecognitionSystem initialized successfully (MediaPipe only)");
        return true;

    } catch (const std::exception& e) {
        pImpl->last_error = "MediaPipe initialization failed: " + std::string(e.what());
        LOG_ERROR(pImpl->last_error);
        return false;
    }
}

bool GestureRecognitionSystem::is_initialized() const {
    return pImpl->initialized;
}

bool GestureRecognitionSystem::start() {
    if (!pImpl->initialized) {
        pImpl->last_error = "System not initialized";
        return false;
    }

    if (pImpl->running) {
        return true; // Already running
    }

    // TODO: Start processing thread

    pImpl->running = true;
    return true;
}

void GestureRecognitionSystem::stop() {
    if (!pImpl->running) {
        return;
    }

    // TODO: Stop processing thread
    // TODO: Cleanup resources

    pImpl->running = false;
}

bool GestureRecognitionSystem::is_running() const {
    return pImpl->running;
}

bool GestureRecognitionSystem::process_frame(const cv::Mat& frame, GestureResult& result) {
    if (!pImpl->initialized) {
        pImpl->last_error = "System not initialized";
        return false;
    }

    if (frame.empty()) {
        pImpl->last_error = "Empty input frame";
        return false;
    }

    auto frame_start = std::chrono::high_resolution_clock::now();

    // Initialize result
    result = GestureResult();
    result.type = GestureType::UNKNOWN;
    result.timestamp = std::chrono::steady_clock::now();

    // Create debug frame if enabled
    if (pImpl->config.enable_debug_viz) {
        pImpl->debug_frame = frame.clone();
    }

    static int process_frame_count = 0;
    process_frame_count++;

    // ============================================================================
    // MOTION BLUR COMPENSATION (PREPROCESSING)
    // ============================================================================
    cv::Mat preprocessed_frame = frame;
    if (pImpl->motion_blur_compensator && pImpl->motion_blur_compensator->is_enabled()) {
        preprocessed_frame = pImpl->motion_blur_compensator->process(frame);
    }

    // ============================================================================
    // MEDIAPIPE PROCESSING (ONLY BACKEND)
    // ============================================================================
    auto det_start = std::chrono::high_resolution_clock::now();

    std::string gesture_name;
    std::vector<std::vector<float>> landmarks_mp;
    float confidence_mp;

    if (!pImpl->mediapipe_detector->detect(preprocessed_frame, gesture_name, landmarks_mp, confidence_mp)) {
        pImpl->last_error = "MediaPipe detection failed: " + pImpl->mediapipe_detector->getLastError();
        LOG_ERROR(pImpl->last_error);
        return false;
    }

    auto det_end = std::chrono::high_resolution_clock::now();
    double det_time = std::chrono::duration<double, std::milli>(det_end - det_start).count();

    // Enhanced logging with tracking quality info
    if (process_frame_count <= 5 || process_frame_count % 50 == 0) {
        LOG_INFO("GestureRecognitionSystem: Frame #" + std::to_string(process_frame_count));
        LOG_INFO("  MediaPipe detection time: " + std::to_string(det_time) + "ms");

        if (landmarks_mp.empty()) {
            LOG_INFO("  Hand: NOT DETECTED");
        } else {
            LOG_INFO("  Hand: DETECTED (landmarks=" + std::to_string(landmarks_mp.size()) +
                    ", confidence=" + std::to_string(confidence_mp) + ")");
            LOG_INFO("  Gesture: " + (gesture_name.empty() ? "none" : gesture_name));
        }

        // Buffer status
        LOG_INFO("  Temporal buffer: " + std::to_string(pImpl->temporal_buffer->size()) + "/" +
                std::to_string(pImpl->temporal_buffer->capacity()) + " frames" +
                " (gap_frames=" + std::to_string(pImpl->temporal_buffer->get_gap_frames()) + "/" +
                std::to_string(pImpl->temporal_buffer->get_max_gap_frames()) + ")");
    }

    // Process hand detection (active or inactive)
    TrackedHand tracked_hand;
    tracked_hand.track_id = 0;  // Single hand tracking
    tracked_hand.timestamp = std::chrono::steady_clock::now();

    if (!landmarks_mp.empty()) {
        // Hand detected - create active TrackedHand
        HandLandmarks hand_landmarks;
        hand_landmarks.confidence = confidence_mp;

        int h = frame.rows;
        int w = frame.cols;

        // Calculate centroid position and bounding box
        float sum_x = 0.0f, sum_y = 0.0f;
        float min_x = 1.0f, max_x = 0.0f;
        float min_y = 1.0f, max_y = 0.0f;
        int valid_points = 0;

        for (size_t i = 0; i < landmarks_mp.size() && i < 21; ++i) {
            hand_landmarks.points[i] = cv::Point3f(
                landmarks_mp[i][0],  // x [0,1]
                landmarks_mp[i][1],  // y [0,1]
                landmarks_mp[i][2]   // z (depth)
            );

            // Sum for centroid calculation (in pixel coordinates)
            sum_x += landmarks_mp[i][0] * w;
            sum_y += landmarks_mp[i][1] * h;

            // Track min/max for bounding box (in normalized coords)
            min_x = std::min(min_x, landmarks_mp[i][0]);
            max_x = std::max(max_x, landmarks_mp[i][0]);
            min_y = std::min(min_y, landmarks_mp[i][1]);
            max_y = std::max(max_y, landmarks_mp[i][1]);

            valid_points++;
        }

        tracked_hand.position = cv::Point2f(sum_x / valid_points, sum_y / valid_points);
        tracked_hand.velocity = cv::Point2f(0.0f, 0.0f);  // Will be calculated by temporal buffer

        // Calculate actual hand size from bounding box of landmarks (in pixels)
        float hand_width = (max_x - min_x) * w;
        float hand_height = (max_y - min_y) * h;
        tracked_hand.size = cv::Size2f(hand_width, hand_height);
        tracked_hand.landmarks = hand_landmarks;
        tracked_hand.confidence = confidence_mp;
        tracked_hand.frames_since_detection = 0;
        tracked_hand.is_active = true;

        // Add active hand to temporal buffer
        pImpl->temporal_buffer->push(tracked_hand);

        // Check if MediaPipe directly recognized a gesture
        if (!gesture_name.empty() && confidence_mp >= pImpl->config.min_gesture_confidence) {
            // Map MediaPipe gesture to our GestureType
            // For now, just report it as UNKNOWN (MediaPipe gestures are different from swipes)
            result.type = GestureType::UNKNOWN; // Will be replaced by swipe detection
            result.confidence = confidence_mp;
            result.landmarks = hand_landmarks;

            LOG_INFO("MediaPipe recognized gesture: " + gesture_name + " (not using for now)");
        }

        // Use swipe detector for swipe gestures (MediaPipe doesn't detect swipes)
        if (pImpl->temporal_buffer->is_full()) {
            auto gesture_start = std::chrono::high_resolution_clock::now();
            GestureType swipe_gesture = pImpl->swipe_detector->detect(*pImpl->temporal_buffer);
            auto gesture_end = std::chrono::high_resolution_clock::now();
            double gesture_time = std::chrono::duration<double, std::milli>(gesture_end - gesture_start).count();

            if (swipe_gesture != GestureType::UNKNOWN) {
                result.type = swipe_gesture;
                result.confidence = pImpl->swipe_detector->get_confidence();
                result.landmarks = hand_landmarks;

                // Detailed swipe detection logging
                LOG_INFO(">>> SWIPE GESTURE DETECTED <<<");
                LOG_INFO("  Type: " + result.get_gesture_name());
                LOG_INFO("  Confidence: " + std::to_string(result.confidence));
                LOG_INFO("  Buffer frames used: " + std::to_string(pImpl->temporal_buffer->size()));
                LOG_INFO("  Displacement: " + std::to_string(pImpl->temporal_buffer->compute_total_displacement()) + "px");
                LOG_INFO("  Duration: " + std::to_string(pImpl->temporal_buffer->compute_duration_ms()) + "ms");
                cv::Point2f avg_vel = pImpl->temporal_buffer->compute_average_velocity();
                LOG_INFO("  Avg velocity: (" + std::to_string(avg_vel.x) + ", " + std::to_string(avg_vel.y) + ") px/frame");

                // Trigger callback
                if (pImpl->callback) {
                    pImpl->callback(result, pImpl->user_data);
                }

                // Clear buffer for next gesture
                pImpl->temporal_buffer->clear();
                pImpl->swipe_detector->reset();
            }

            pImpl->avg_classification_time = (pImpl->avg_classification_time * pImpl->frame_count + gesture_time) /
                                            (pImpl->frame_count + 1);
        }

        // Debug visualization
        if (pImpl->config.enable_debug_viz && !pImpl->debug_frame.empty()) {
            int h = pImpl->debug_frame.rows;
            int w = pImpl->debug_frame.cols;

            // Draw landmarks
            for (size_t i = 0; i < 21; ++i) {
                int x = static_cast<int>(hand_landmarks.points[i].x * w);
                int y = static_cast<int>(hand_landmarks.points[i].y * h);
                cv::circle(pImpl->debug_frame, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
            }

            // Draw gesture info (inference time instead of FPS)
            std::string info = "Inference: " + std::to_string(static_cast<int>(det_time)) + "ms";
            cv::putText(pImpl->debug_frame, info, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            if (!gesture_name.empty()) {
                cv::putText(pImpl->debug_frame, "MP: " + gesture_name, cv::Point(10, 60),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            }
        }
    } else {
        // No hand detected - push inactive hand for gap tolerance
        tracked_hand.position = cv::Point2f(0.0f, 0.0f);
        tracked_hand.velocity = cv::Point2f(0.0f, 0.0f);
        tracked_hand.size = cv::Size2f(0.0f, 0.0f);
        tracked_hand.confidence = 0.0f;
        tracked_hand.frames_since_detection = 0;
        tracked_hand.is_active = false;  // Mark as inactive

        // Push inactive hand - TemporalBuffer handles gap tolerance logic
        pImpl->temporal_buffer->push(tracked_hand);
    }

    // Update performance stats
    auto frame_end = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double, std::milli>(frame_end - frame_start).count();
    result.processing_time_ms = total_time;

    pImpl->avg_detection_time = (pImpl->avg_detection_time * pImpl->frame_count + det_time) /
                               (pImpl->frame_count + 1);
    pImpl->avg_total_time = (pImpl->avg_total_time * pImpl->frame_count + total_time) /
                           (pImpl->frame_count + 1);
    pImpl->frame_count++;

    return true;
}

void GestureRecognitionSystem::set_gesture_callback(GestureCallback callback, void* user_data) {
    pImpl->callback = callback;
    pImpl->user_data = user_data;
}

GestureConfig GestureRecognitionSystem::get_config() const {
    return pImpl->config;
}

bool GestureRecognitionSystem::set_config(const GestureConfig& config) {
    if (pImpl->running) {
        pImpl->last_error = "Cannot update config while running";
        return false;
    }

    if (!config.is_valid()) {
        pImpl->last_error = "Invalid configuration";
        return false;
    }

    pImpl->config = config;
    return true;
}

void GestureRecognitionSystem::get_performance_stats(
    double& avg_detection_time_ms,
    double& avg_classification_time_ms,
    double& avg_total_time_ms,
    double& fps) const {

    avg_detection_time_ms = pImpl->avg_detection_time;
    avg_classification_time_ms = pImpl->avg_classification_time;
    avg_total_time_ms = pImpl->avg_total_time;

    if (pImpl->avg_total_time > 0.0) {
        fps = 1000.0 / pImpl->avg_total_time;
    } else {
        fps = 0.0;
    }
}

void GestureRecognitionSystem::reset_performance_stats() {
    pImpl->avg_detection_time = 0.0;
    pImpl->avg_classification_time = 0.0;
    pImpl->avg_total_time = 0.0;
    pImpl->frame_count = 0;
}

std::string GestureRecognitionSystem::get_last_error() const {
    return pImpl->last_error;
}

void GestureRecognitionSystem::set_debug_visualization(bool enable) {
    pImpl->config.enable_debug_viz = enable;
}

cv::Mat GestureRecognitionSystem::get_debug_frame() const {
    return pImpl->debug_frame;
}

} // namespace gesture
} // namespace unlook
