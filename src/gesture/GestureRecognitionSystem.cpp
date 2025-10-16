/**
 * @file GestureRecognitionSystem.cpp
 * @brief Implementation of main gesture recognition system
 */

#include <unlook/gesture/GestureRecognitionSystem.hpp>
#include <unlook/gesture/HandDetector.hpp>
#include <unlook/gesture/HandLandmarkExtractor.hpp>
#include <unlook/gesture/HandTracker.hpp>
#include <unlook/gesture/TemporalBuffer.hpp>
#include <unlook/gesture/GeometricSwipeDetector.hpp>
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

    // Core components
    std::unique_ptr<HandDetector> hand_detector;
    std::unique_ptr<HandLandmarkExtractor> landmark_extractor;
    std::unique_ptr<HandTracker> hand_tracker;
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
    // if (!camera_system) {
    //     pImpl->last_error = "Camera system is null";
    //     return false;
    // }

    pImpl->config = config;

    // Initialize all components
    pImpl->hand_detector = std::make_unique<HandDetector>();
    pImpl->landmark_extractor = std::make_unique<HandLandmarkExtractor>();
    pImpl->hand_tracker = std::make_unique<HandTracker>();
    pImpl->temporal_buffer = std::make_unique<TemporalBuffer>(30);  // 1 second at 30fps
    pImpl->swipe_detector = std::make_unique<GeometricSwipeDetector>();

    // Configure hand detector
    HandDetectorConfig det_config;
    // Use model path from config (set by GUI or application)
    if (!config.palm_detection_model_path.empty()) {
        det_config.model_path = config.palm_detection_model_path;
    } else {
        // Fallback to relative path (may fail if working directory is wrong)
        det_config.model_path = "third-party/hand-gesture-recognition-using-onnx/model/palm_detection/palm_detection_full_inf_post_192x192.onnx";
        LOG_WARNING("Using fallback relative path for palm detection model - may fail!");
    }
    det_config.score_threshold = config.min_detection_confidence;
    det_config.max_num_hands = config.max_num_hands;

    LOG_INFO("Initializing hand detector with model: " + det_config.model_path);

    if (!pImpl->hand_detector->initialize(det_config)) {
        pImpl->last_error = "Failed to initialize hand detector: " +
                           pImpl->hand_detector->get_last_error();
        LOG_ERROR(pImpl->last_error);
        return false;
    }

    LOG_INFO("Hand detector initialized successfully");

    // Configure landmark extractor
    HandLandmarkConfig landmark_config;
    // Use model path from config (set by GUI or application)
    if (!config.hand_landmark_model_path.empty()) {
        landmark_config.model_path = config.hand_landmark_model_path;
    } else {
        // Fallback to relative path (may fail if working directory is wrong)
        landmark_config.model_path = "third-party/hand-gesture-recognition-using-onnx/model/hand_landmark/hand_landmark_sparse_Nx3x224x224.onnx";
        LOG_WARNING("Using fallback relative path for hand landmark model - may fail!");
    }
    landmark_config.confidence_threshold = config.min_tracking_confidence;

    LOG_INFO("Initializing landmark extractor with model: " + landmark_config.model_path);

    if (!pImpl->landmark_extractor->initialize(landmark_config)) {
        pImpl->last_error = "Failed to initialize landmark extractor: " +
                           pImpl->landmark_extractor->get_last_error();
        LOG_ERROR(pImpl->last_error);
        return false;
    }

    LOG_INFO("Landmark extractor initialized successfully");

    // Configure hand tracker
    HandTrackerConfig tracker_config;
    tracker_config.max_missed_frames = 5;
    tracker_config.position_noise = 0.1f;
    tracker_config.velocity_noise = 0.5f;
    tracker_config.measurement_noise = 1.0f;
    pImpl->hand_tracker->configure(tracker_config);

    // Configure swipe detector
    SwipeConfig swipe_config;
    swipe_config.min_velocity = 50.0f;
    swipe_config.min_displacement = 100.0f;
    swipe_config.direction_threshold = 0.7f;
    swipe_config.scale_change_threshold = 0.25f;
    swipe_config.min_frames = 10;
    pImpl->swipe_detector->configure(swipe_config);

    LOG_INFO("GestureRecognitionSystem initialized successfully");
    pImpl->initialized = true;
    return true;
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

    // Step 1: Detect hands
    auto det_start = std::chrono::high_resolution_clock::now();
    std::vector<HandDetection> detections;

    static int process_frame_count = 0;
    process_frame_count++;

    if (!pImpl->hand_detector->detect(frame, detections)) {
        pImpl->last_error = "Hand detection failed: " +
                           pImpl->hand_detector->get_last_error();
        LOG_ERROR(pImpl->last_error);
        return false;
    }
    auto det_end = std::chrono::high_resolution_clock::now();
    double det_time = std::chrono::duration<double, std::milli>(det_end - det_start).count();

    // Log detection results periodically
    if (process_frame_count <= 5 || process_frame_count % 50 == 0) {
        LOG_INFO("GestureRecognitionSystem: Frame #" + std::to_string(process_frame_count));
        LOG_INFO("  Hand detection time: " + std::to_string(det_time) + "ms");
        LOG_INFO("  Detections found: " + std::to_string(detections.size()));

        if (detections.size() > 0) {
            LOG_INFO("  HAND DETECTED! Processing landmarks...");
        } else {
            LOG_DEBUG("  No hands detected in this frame");
        }
    }

    // Convert HandDetection to cv::Rect for tracker
    std::vector<cv::Rect> detection_boxes;
    for (const auto& det : detections) {
        detection_boxes.push_back(det.bounding_box);
    }

    // Step 2: Extract landmarks for each detection
    auto landmark_start = std::chrono::high_resolution_clock::now();
    std::vector<HandLandmarks> landmarks_list;
    for (const auto& bbox : detection_boxes) {
        HandLandmarks landmarks;
        if (pImpl->landmark_extractor->extract(frame, bbox, landmarks)) {
            landmarks_list.push_back(landmarks);
        } else {
            // Failed landmark extraction - use empty landmarks
            landmarks_list.push_back(HandLandmarks{});
        }
    }
    auto landmark_end = std::chrono::high_resolution_clock::now();
    double landmark_time = std::chrono::duration<double, std::milli>(landmark_end - landmark_start).count();

    // Step 3: Update tracker
    pImpl->hand_tracker->update(detection_boxes, landmarks_list);
    auto tracks = pImpl->hand_tracker->get_active_tracks();

    // Step 4: For primary hand, update temporal buffer and detect gestures
    if (!tracks.empty()) {
        const auto& primary_hand = tracks[0];  // Use first tracked hand

        // Push to temporal buffer
        pImpl->temporal_buffer->push(primary_hand);

        // Step 5: Detect gesture from temporal data
        if (pImpl->temporal_buffer->is_full()) {
            auto gesture_start = std::chrono::high_resolution_clock::now();
            GestureType gesture = pImpl->swipe_detector->detect(*pImpl->temporal_buffer);
            auto gesture_end = std::chrono::high_resolution_clock::now();
            double gesture_time = std::chrono::duration<double, std::milli>(gesture_end - gesture_start).count();

            if (gesture != GestureType::UNKNOWN) {
                // Populate result
                result.type = gesture;
                result.confidence = pImpl->swipe_detector->get_confidence();
                result.landmarks = primary_hand.landmarks;
                result.center_position = primary_hand.position;
                result.bounding_box = primary_hand.get_bounding_box();

                std::ostringstream msg;
                msg << "Gesture detected: " << result.get_gesture_name()
                    << " (confidence: " << result.confidence << ")";
                LOG_INFO(msg.str());

                // Trigger callback if set
                if (pImpl->callback) {
                    pImpl->callback(result, pImpl->user_data);
                }

                // Clear buffer for next gesture
                pImpl->temporal_buffer->clear();
                pImpl->swipe_detector->reset();
            }

            // Update classification time stats
            pImpl->avg_classification_time = (pImpl->avg_classification_time * pImpl->frame_count + gesture_time) /
                                            (pImpl->frame_count + 1);
        }

        // Debug visualization
        if (pImpl->config.enable_debug_viz && !pImpl->debug_frame.empty()) {
            // Draw hand bounding box
            cv::rectangle(pImpl->debug_frame, primary_hand.get_bounding_box(),
                         cv::Scalar(0, 255, 0), 2);

            // Draw velocity vector
            cv::Point2f vel_end = primary_hand.position + primary_hand.velocity * 5.0f;
            cv::arrowedLine(pImpl->debug_frame, primary_hand.position, vel_end,
                          cv::Scalar(255, 0, 0), 2);

            // Draw landmarks if available
            if (primary_hand.landmarks.is_valid()) {
                for (int i = 0; i < 21; ++i) {
                    cv::Point2f pt = primary_hand.landmarks.get_pixel_position(i);
                    if (pt.x >= 0 && pt.y >= 0) {
                        cv::circle(pImpl->debug_frame, pt, 3, cv::Scalar(0, 0, 255), -1);
                    }
                }
            }

            // Draw gesture info
            std::string info = "Buffer: " + std::to_string(pImpl->temporal_buffer->size()) + "/30";
            cv::putText(pImpl->debug_frame, info, cv::Point(10, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
        }
    }

    // Calculate total processing time
    auto frame_end = std::chrono::high_resolution_clock::now();
    double total_time = std::chrono::duration<double, std::milli>(frame_end - frame_start).count();

    result.processing_time_ms = total_time;

    // Update performance stats
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
