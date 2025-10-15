/**
 * @file gesture_test_camera.cpp
 * @brief Gesture recognition test with REAL Unlook hardware cameras
 *
 * Uses the actual IMX296 stereo cameras with hardware sync.
 * Single camera mode for gesture detection (no stereo processing needed).
 */

#include <unlook/gesture/GestureRecognitionSystem.hpp>
#include <unlook/camera/CameraSystem.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <atomic>
#include <chrono>
#include <map>

using namespace unlook;

// Global flag for clean shutdown
std::atomic<bool> should_exit(false);

void signal_handler(int signal) {
    std::cout << "\n\nReceived signal " << signal << ", shutting down..." << std::endl;
    should_exit = true;
}

// Gesture callback
std::map<gesture::GestureType, int> gesture_counts;

void on_gesture_detected(const gesture::GestureResult& result, void* user_data) {
    std::cout << "\n🎯 GESTURE: " << result.get_gesture_name()
              << " (confidence: " << std::fixed << std::setprecision(2) << result.confidence << ")" << std::endl;

    // Update gesture count
    gesture_counts[result.type]++;
}

void print_usage(const char* prog_name) {
    std::cout << "Usage: " << prog_name << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help           Show this help" << std::endl;
    std::cout << "  -d, --display        Show live camera feed" << std::endl;
    std::cout << "  -c, --confidence <v> Min gesture confidence (0.0-1.0, default: 0.7)" << std::endl;
    std::cout << "  -v, --verbose        Verbose logging" << std::endl;
}

int main(int argc, char** argv) {
    // Parse arguments
    bool show_display = false;
    float min_confidence = 0.7f;
    bool verbose = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "-d" || arg == "--display") {
            show_display = true;
        } else if (arg == "-c" || arg == "--confidence") {
            if (i + 1 < argc) {
                min_confidence = std::stof(argv[++i]);
            }
        } else if (arg == "-v" || arg == "--verbose") {
            verbose = true;
        }
    }

    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize logger
    core::Logger::getInstance().setLevel(
        verbose ? core::LogLevel::DEBUG : core::LogLevel::INFO
    );

    std::cout << "\n=== Unlook Gesture Recognition - Hardware Camera Test ===" << std::endl;
    std::cout << "Using: IMX296 Stereo Cameras with Hardware Sync" << std::endl;

    // 1. Initialize REAL camera system
    std::cout << "\n[1/3] Initializing camera system..." << std::endl;
    auto camera_system = camera::CameraSystem::getInstance();

    if (!camera_system->initialize()) {
        std::cerr << "ERROR: Failed to initialize camera system!" << std::endl;
        return 1;
    }

    if (!camera_system->startCapture()) {
        std::cerr << "ERROR: Failed to start camera capture!" << std::endl;
        return 1;
    }

    std::cout << "✓ Camera system ready (Hardware sync enabled)" << std::endl;

    // 2. Initialize gesture recognition
    std::cout << "\n[2/3] Initializing gesture recognition..." << std::endl;
    gesture::GestureRecognitionSystem gesture_system;

    gesture::GestureConfig config;
    config.min_detection_confidence = 0.5f;
    config.min_tracking_confidence = 0.5f;
    config.min_gesture_confidence = min_confidence;
    config.max_num_hands = 1;
    config.enable_debug_viz = show_display;

    // Camera system can be null for gesture system (not used internally)
    if (!gesture_system.initialize(nullptr, config)) {
        std::cerr << "ERROR: Failed to initialize gesture system: "
                  << gesture_system.get_last_error() << std::endl;
        camera_system->stopCapture();
        return 1;
    }

    gesture_system.set_gesture_callback(on_gesture_detected, nullptr);

    std::cout << "✓ Gesture recognition ready" << std::endl;
    std::cout << "   Models: " << std::endl;
    std::cout << "   - Palm detection: 192x192 ONNX" << std::endl;
    std::cout << "   - Hand landmarks: 21 keypoints" << std::endl;
    std::cout << "   - Swipe detector: 6 directions" << std::endl;

    // 3. Setup display if requested
    cv::Mat display_frame;
    if (show_display) {
        cv::namedWindow("Gesture Recognition", cv::WINDOW_NORMAL);
        cv::resizeWindow("Gesture Recognition", 800, 600);
        std::cout << "\n✓ Display window created (press 'q' to quit)" << std::endl;
    }

    // 4. Main processing loop
    std::cout << "\n[3/3] Starting gesture detection..." << std::endl;
    std::cout << "\nSupported gestures:" << std::endl;
    std::cout << "  • Swipe LEFT/RIGHT - horizontal hand movement" << std::endl;
    std::cout << "  • Swipe UP/DOWN - vertical hand movement" << std::endl;
    std::cout << "  • Swipe FORWARD - hand toward camera (size increase)" << std::endl;
    std::cout << "  • Swipe BACKWARD - hand away from camera (size decrease)" << std::endl;
    std::cout << "\nPress CTRL+C to stop\n" << std::endl;

    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (!should_exit) {
        // Capture stereo frame from REAL hardware cameras
        camera::StereoFrame stereo_frame;
        if (!camera_system->captureStereoFrame(stereo_frame, 100)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Use LEFT camera (Camera 1) for gesture detection
        cv::Mat& frame = stereo_frame.leftImage;

        if (frame.empty()) {
            continue;
        }

        // Convert from SBGGR10 to BGR if needed
        cv::Mat bgr_frame;
        if (frame.channels() == 1) {
            cv::cvtColor(frame, bgr_frame, cv::COLOR_BayerBG2BGR);
        } else {
            bgr_frame = frame;
        }

        // Process frame for gesture detection
        gesture::GestureResult result;
        if (gesture_system.process_frame(bgr_frame, result)) {
            frame_count++;

            // Print FPS every 30 frames
            if (frame_count % 30 == 0) {
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - start_time).count();
                double fps = frame_count / elapsed;

                std::cout << "FPS: " << std::fixed << std::setprecision(1) << fps
                          << " | Frames: " << frame_count
                          << " | Sync: " << std::setprecision(2) << stereo_frame.syncErrorMs << " ms"
                          << std::endl;

                // Performance stats
                double det_time, class_time, total_time, avg_fps;
                gesture_system.get_performance_stats(det_time, class_time, total_time, avg_fps);
                std::cout << "Processing: " << std::setprecision(1) << total_time << " ms/frame" << std::endl;
            }

            // Display if enabled
            if (show_display) {
                display_frame = gesture_system.get_debug_frame();
                if (display_frame.empty()) {
                    display_frame = bgr_frame.clone();
                }

                // Add FPS overlay
                auto now = std::chrono::steady_clock::now();
                double elapsed = std::chrono::duration<double>(now - start_time).count();
                double fps = frame_count / elapsed;

                std::string fps_text = "FPS: " + std::to_string((int)fps);
                cv::putText(display_frame, fps_text, cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

                cv::imshow("Gesture Recognition", display_frame);

                char key = cv::waitKey(1);
                if (key == 'q' || key == 'Q') {
                    should_exit = true;
                }
            }
        }
    }

    // Cleanup and final statistics
    std::cout << "\n\n=== Shutting down ===" << std::endl;
    gesture_system.stop();
    camera_system->stopCapture();

    if (show_display) {
        cv::destroyAllWindows();
    }

    // Print final statistics
    std::cout << "\n=== Session Statistics ===" << std::endl;
    std::cout << "Total frames processed: " << frame_count << std::endl;

    auto end_time = std::chrono::steady_clock::now();
    double total_elapsed = std::chrono::duration<double>(end_time - start_time).count();
    double final_fps = frame_count / total_elapsed;
    std::cout << "Average FPS: " << std::fixed << std::setprecision(1) << final_fps << std::endl;

    double det_time, class_time, total_time, avg_fps;
    gesture_system.get_performance_stats(det_time, class_time, total_time, avg_fps);
    std::cout << "Average processing time: " << std::setprecision(1) << total_time << " ms" << std::endl;

    // Gesture counts
    std::cout << "\n=== Detected Gestures ===" << std::endl;
    int total_gestures = 0;
    for (const auto& pair : gesture_counts) {
        std::cout << "  " << gesture::gesture_type_to_string(pair.first)
                  << ": " << pair.second << std::endl;
        total_gestures += pair.second;
    }
    std::cout << "Total gestures detected: " << total_gestures << std::endl;

    std::cout << "\nGoodbye!" << std::endl;
    return 0;
}
