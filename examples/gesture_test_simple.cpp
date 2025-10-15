/**
 * @file gesture_test_simple.cpp
 * @brief Simplified headless test for gesture recognition (no camera integration)
 *
 * Tests gesture recognition system using OpenCV VideoCapture (webcam/video file).
 * This version doesn't require the Unlook camera system, useful for testing
 * gesture algorithms independently.
 *
 * @copyright 2025 Unlook Project
 * @license MIT License
 */

#include <unlook/gesture/GestureRecognitionSystem.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <csignal>
#include <chrono>
#include <thread>
#include <atomic>
#include <iomanip>
#include <map>

using namespace unlook;

// Global flag for CTRL+C handling
static volatile bool running = true;

// Statistics counters
static std::atomic<int> total_frames{0};
static std::atomic<int> gesture_count{0};
static std::map<gesture::GestureType, int> gesture_histogram;
static std::mutex histogram_mutex;

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n\nReceived SIGINT, shutting down..." << std::endl;
        running = false;
    }
}

/**
 * @brief Callback invoked when a gesture is detected
 */
void on_gesture_detected(const gesture::GestureResult& result, void* user_data) {
    if (!result.is_valid()) {
        return;
    }

    gesture_count++;

    // Update histogram
    {
        std::lock_guard<std::mutex> lock(histogram_mutex);
        gesture_histogram[result.type]++;
    }

    // Print detection info
    std::cout << "\n";
    std::cout << "========================================" << std::endl;
    std::cout << "🎯 GESTURE DETECTED: " << result.get_gesture_name() << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Confidence:      " << std::fixed << std::setprecision(2)
              << (result.confidence * 100.0f) << "%" << std::endl;
    std::cout << "  Processing time: " << std::setprecision(1)
              << result.processing_time_ms << " ms" << std::endl;
    std::cout << "  Center position: (" << static_cast<int>(result.center_position.x)
              << ", " << static_cast<int>(result.center_position.y) << ")" << std::endl;
    std::cout << "  Bounding box:    " << result.bounding_box.width << "x"
              << result.bounding_box.height << " px" << std::endl;
    std::cout << "  Hand:            " << (result.landmarks.is_right_hand ? "Right" : "Left") << std::endl;
    std::cout << "  Total gestures:  " << gesture_count.load() << std::endl;
    std::cout << "========================================\n" << std::endl;
}

/**
 * @brief Print current statistics to console
 */
void print_statistics(const gesture::GestureRecognitionSystem& gesture_system,
                     double elapsed_seconds) {
    double det_time, class_time, total_time, avg_fps;
    gesture_system.get_performance_stats(det_time, class_time, total_time, avg_fps);

    std::cout << "\n";
    std::cout << "=======================================" << std::endl;
    std::cout << "           RUNTIME STATISTICS          " << std::endl;
    std::cout << "=======================================" << std::endl;
    std::cout << "Session duration:     " << std::fixed << std::setprecision(1)
              << elapsed_seconds << " seconds" << std::endl;
    std::cout << "Frames processed:     " << total_frames.load() << std::endl;
    std::cout << "Gestures detected:    " << gesture_count.load() << std::endl;
    std::cout << "Average FPS:          " << std::setprecision(1) << avg_fps << std::endl;
    std::cout << "Avg processing time:  " << std::setprecision(2) << total_time << " ms" << std::endl;
    std::cout << "  - Detection:        " << det_time << " ms" << std::endl;
    std::cout << "  - Classification:   " << class_time << " ms" << std::endl;
    std::cout << "=======================================" << std::endl;

    // Print gesture histogram
    std::lock_guard<std::mutex> lock(histogram_mutex);
    if (!gesture_histogram.empty()) {
        std::cout << "\nGesture Histogram:" << std::endl;
        std::cout << "---------------------------------------" << std::endl;
        for (const auto& entry : gesture_histogram) {
            std::cout << "  " << std::setw(20) << std::left
                      << gesture::gesture_type_to_string(entry.first)
                      << ": " << entry.second << std::endl;
        }
        std::cout << "=======================================" << std::endl;
    }
}

int main(int argc, char** argv) {
    // Setup signal handler for clean shutdown
    std::signal(SIGINT, signal_handler);

    // Parse command line arguments
    bool enable_debug_viz = false;
    float min_confidence = 0.7f;
    int camera_id = 0;
    std::string video_file;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--debug" || arg == "-d") {
            enable_debug_viz = true;
        } else if (arg == "--confidence" || arg == "-c") {
            if (i + 1 < argc) {
                min_confidence = std::stof(argv[++i]);
            }
        } else if (arg == "--camera") {
            if (i + 1 < argc) {
                camera_id = std::stoi(argv[++i]);
            }
        } else if (arg == "--video" || arg == "-v") {
            if (i + 1 < argc) {
                video_file = argv[++i];
            }
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --debug, -d              Enable debug visualization" << std::endl;
            std::cout << "  --confidence, -c <val>   Set min gesture confidence (0.0-1.0)" << std::endl;
            std::cout << "  --camera <id>            Camera ID to use (default: 0)" << std::endl;
            std::cout << "  --video, -v <file>       Use video file instead of camera" << std::endl;
            std::cout << "  --help, -h               Show this help message" << std::endl;
            return 0;
        }
    }

    // Initialize logger
    core::Logger::getInstance().setLevel(core::LogLevel::INFO);

    std::cout << "\n";
    std::cout << "=========================================" << std::endl;
    std::cout << "   UNLOOK GESTURE RECOGNITION TEST" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Configuration:" << std::endl;
    if (!video_file.empty()) {
        std::cout << "  Input:            " << video_file << std::endl;
    } else {
        std::cout << "  Camera ID:        " << camera_id << std::endl;
    }
    std::cout << "  Min confidence:   " << (min_confidence * 100.0f) << "%" << std::endl;
    std::cout << "  Debug viz:        " << (enable_debug_viz ? "Enabled" : "Disabled") << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "\nPress CTRL+C to stop\n" << std::endl;

    try {
        // 1. Initialize video capture
        std::cout << "Opening video source..." << std::endl;
        cv::VideoCapture cap;

        if (!video_file.empty()) {
            cap.open(video_file);
        } else {
            cap.open(camera_id);
        }

        if (!cap.isOpened()) {
            std::cerr << "ERROR: Failed to open video source!" << std::endl;
            return 1;
        }

        // Get video properties
        double fps = cap.get(cv::CAP_PROP_FPS);
        int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));

        std::cout << "Video source opened successfully!" << std::endl;
        std::cout << "  Resolution: " << frame_width << "x" << frame_height << std::endl;
        std::cout << "  FPS: " << fps << std::endl;

        // 2. Initialize gesture recognition system
        std::cout << "\nInitializing gesture recognition system..." << std::endl;
        gesture::GestureRecognitionSystem gesture_system;

        gesture::GestureConfig gesture_config;
        gesture_config.min_detection_confidence = 0.5f;
        gesture_config.min_tracking_confidence = 0.5f;
        gesture_config.min_gesture_confidence = min_confidence;
        gesture_config.max_num_hands = 1;
        gesture_config.enable_debug_viz = enable_debug_viz;
        gesture_config.enable_temporal_smoothing = true;
        gesture_config.smoothing_window_size = 5;
        gesture_config.process_every_n_frames = 1;

        // Initialize without camera system (manual frame feeding)
        if (!gesture_system.initialize(nullptr, gesture_config)) {
            std::cerr << "ERROR: Failed to initialize gesture system: "
                      << gesture_system.get_last_error() << std::endl;
            return 1;
        }

        std::cout << "Gesture recognition system ready!" << std::endl;
        std::cout << "\n=========================================" << std::endl;
        std::cout << "Supported Gestures:" << std::endl;
        std::cout << "  • Swipe LEFT/RIGHT   - Horizontal movement" << std::endl;
        std::cout << "  • Swipe UP/DOWN      - Vertical movement" << std::endl;
        std::cout << "  • Swipe FORWARD      - Hand toward camera" << std::endl;
        std::cout << "  • Swipe BACKWARD     - Hand away from camera" << std::endl;
        std::cout << "  • Open Palm          - Hand open" << std::endl;
        std::cout << "  • Closed Fist        - Hand closed" << std::endl;
        std::cout << "  • Point Up/Down      - Index finger pointing" << std::endl;
        std::cout << "  • Thumbs Up/Down     - Thumb gesture" << std::endl;
        std::cout << "=========================================" << std::endl;
        std::cout << "\nDetecting gestures...\n" << std::endl;

        // 3. Main processing loop
        auto start_time = std::chrono::steady_clock::now();
        auto last_stats_time = start_time;
        cv::Mat frame;

        while (running) {
            // Capture frame from video source
            if (!cap.read(frame)) {
                if (video_file.empty()) {
                    std::cerr << "Warning: Failed to capture frame from camera" << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    continue;
                } else {
                    // End of video file
                    std::cout << "\nEnd of video file reached." << std::endl;
                    break;
                }
            }

            if (frame.empty()) {
                std::cerr << "Warning: Empty frame received" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // Ensure BGR format
            cv::Mat bgr_frame;
            if (frame.channels() == 1) {
                cv::cvtColor(frame, bgr_frame, cv::COLOR_GRAY2BGR);
            } else {
                bgr_frame = frame;
            }

            // Process frame for gesture detection
            gesture::GestureResult result;
            if (gesture_system.process_frame(bgr_frame, result)) {
                total_frames++;

                // Check if gesture detected
                if (result.is_valid()) {
                    on_gesture_detected(result, nullptr);
                }
            } else {
                std::cerr << "Warning: Frame processing failed: "
                          << gesture_system.get_last_error() << std::endl;
            }

            // Print statistics every 5 seconds
            auto now = std::chrono::steady_clock::now();
            double elapsed_since_stats = std::chrono::duration<double>(now - last_stats_time).count();
            if (elapsed_since_stats >= 5.0) {
                double total_elapsed = std::chrono::duration<double>(now - start_time).count();
                print_statistics(gesture_system, total_elapsed);
                last_stats_time = now;
            }

            // Small delay to prevent overwhelming CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Final statistics
        auto end_time = std::chrono::steady_clock::now();
        double total_elapsed = std::chrono::duration<double>(end_time - start_time).count();

        std::cout << "\n\nStopping..." << std::endl;
        gesture_system.stop();
        cap.release();

        // Print final session report
        std::cout << "\n";
        std::cout << "=========================================" << std::endl;
        std::cout << "         FINAL SESSION REPORT" << std::endl;
        std::cout << "=========================================" << std::endl;
        print_statistics(gesture_system, total_elapsed);
        std::cout << "\nSession completed successfully!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\nException occurred: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
