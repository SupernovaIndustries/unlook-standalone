/*
 * Real-time Performance Test for Unlook 3D Scanner
 * Validates 30 FPS continuous capture with proper grayscale conversion
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <iomanip>
#include <signal.h>

#include <unlook/realtime/RealtimePipeline.hpp>
#include <unlook/camera/CameraSystem.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace unlook;
using namespace std::chrono;

// Global flag for clean shutdown
std::atomic<bool> g_running(true);

void signal_handler(int signal) {
    std::cout << "\n[Test] Shutting down gracefully..." << std::endl;
    g_running = false;
}

// Performance statistics
struct PerformanceStats {
    std::atomic<int> frames_received{0};
    std::atomic<int> frames_displayed{0};
    std::atomic<double> total_latency_ms{0.0};
    std::atomic<double> min_latency_ms{999999.0};
    std::atomic<double> max_latency_ms{0.0};
    steady_clock::time_point start_time;
    steady_clock::time_point last_frame_time;
    
    void recordFrame(double latency_ms) {
        frames_received++;
        total_latency_ms += latency_ms;
        
        double current_min = min_latency_ms.load();
        while (latency_ms < current_min && 
               !min_latency_ms.compare_exchange_weak(current_min, latency_ms));
        
        double current_max = max_latency_ms.load();
        while (latency_ms > current_max && 
               !max_latency_ms.compare_exchange_weak(current_max, latency_ms));
        
        last_frame_time = steady_clock::now();
    }
    
    void print() const {
        auto elapsed = duration<double>(steady_clock::now() - start_time).count();
        int total_frames = frames_received.load();
        
        std::cout << "\n===== PERFORMANCE REPORT =====\n";
        std::cout << "Test Duration: " << std::fixed << std::setprecision(1) 
                  << elapsed << " seconds\n";
        std::cout << "Frames Received: " << total_frames << "\n";
        std::cout << "Frames Displayed: " << frames_displayed.load() << "\n";
        
        if (total_frames > 0) {
            double avg_fps = total_frames / elapsed;
            double avg_latency = total_latency_ms.load() / total_frames;
            
            std::cout << "Average FPS: " << std::fixed << std::setprecision(1) 
                      << avg_fps << " FPS";
            
            // Color code the FPS result
            if (avg_fps >= 30.0) {
                std::cout << " ✓ PASS (≥30 FPS required)";
            } else if (avg_fps >= 25.0) {
                std::cout << " ⚠ MARGINAL (<30 FPS)";
            } else {
                std::cout << " ✗ FAIL (<30 FPS required)";
            }
            std::cout << "\n";
            
            std::cout << "Latency - Avg: " << std::fixed << std::setprecision(2) 
                      << avg_latency << " ms, "
                      << "Min: " << min_latency_ms.load() << " ms, "
                      << "Max: " << max_latency_ms.load() << " ms\n";
        }
        
        std::cout << "==============================\n";
    }
};

void testRealtimePipeline() {
    std::cout << "[Test] Starting Real-time Pipeline Performance Test\n";
    std::cout << "[Test] Target: 30 FPS at VGA resolution (640x480)\n";
    std::cout << "[Test] Press Ctrl+C to stop\n\n";
    
    // Create pipeline
    auto pipeline = std::make_unique<realtime::RealtimePipeline>();
    
    // Configure for optimal performance
    realtime::RealtimePipeline::Config config;
    config.preview_width = 640;
    config.preview_height = 480;
    config.full_width = 1456;
    config.full_height = 1088;
    config.target_fps = 30.0;
    config.exposure_time_us = 15000.0;  // 15ms proper exposure
    config.analog_gain = 3.0;           // 3x gain for indoor
    config.auto_exposure = true;
    config.enable_thread_affinity = true;
    config.enable_frame_dropping = true;
    config.frame_queue_size = 5;
    config.memory_pool_size = 10;
    
    // Initialize pipeline
    if (!pipeline->initialize(config)) {
        std::cerr << "[Test] Failed to initialize pipeline\n";
        return;
    }
    
    // Performance tracking
    PerformanceStats stats;
    stats.start_time = steady_clock::now();
    
    // Create OpenCV windows for display
    cv::namedWindow("LEFT Camera - Grayscale", cv::WINDOW_NORMAL);
    cv::namedWindow("RIGHT Camera - Grayscale", cv::WINDOW_NORMAL);
    cv::resizeWindow("LEFT Camera - Grayscale", 640, 480);
    cv::resizeWindow("RIGHT Camera - Grayscale", 640, 480);
    
    // Frame callback
    auto frame_callback = [&stats](const core::StereoFramePair& frame) {
        auto now = steady_clock::now();
        
        // Calculate frame-to-frame latency
        if (stats.frames_received > 0) {
            double latency_ms = duration<double, std::milli>(
                now - stats.last_frame_time).count();
            stats.recordFrame(latency_ms);
        } else {
            stats.recordFrame(0.0);
        }
        
        // Display frames (non-blocking)
        if (!frame.left_frame.image.empty() && !frame.right_frame.image.empty()) {
            try {
                cv::imshow("LEFT Camera - Grayscale", frame.left_frame.image);
                cv::imshow("RIGHT Camera - Grayscale", frame.right_frame.image);
                stats.frames_displayed++;
                
                // Non-blocking key check
                if (cv::waitKey(1) == 27) { // ESC key
                    g_running = false;
                }
            } catch (const cv::Exception& e) {
                std::cerr << "[Test] OpenCV display error: " << e.what() << std::endl;
            }
        }
        
        // Print periodic updates
        if (stats.frames_received % 30 == 0) {
            auto elapsed = duration<double>(now - stats.start_time).count();
            double current_fps = stats.frames_received.load() / elapsed;
            std::cout << "[Test] Frame " << stats.frames_received.load() 
                      << " - Current FPS: " << std::fixed << std::setprecision(1) 
                      << current_fps;
            
            // Check image properties
            if (!frame.left_frame.image.empty()) {
                std::cout << " - Image: " << frame.left_frame.image.cols 
                          << "x" << frame.left_frame.image.rows
                          << " type:" << frame.left_frame.image.type();
                
                // Calculate average brightness
                cv::Scalar mean = cv::mean(frame.left_frame.image);
                std::cout << " brightness:" << std::fixed << std::setprecision(0) 
                          << mean[0];
            }
            
            std::cout << " sync:" << (frame.synchronized ? "OK" : "FAIL") 
                      << std::endl;
        }
    };
    
    // Start pipeline
    std::cout << "[Test] Starting capture...\n";
    if (!pipeline->start(frame_callback)) {
        std::cerr << "[Test] Failed to start pipeline\n";
        return;
    }
    
    // Monitor performance
    auto monitor_start = steady_clock::now();
    while (g_running) {
        std::this_thread::sleep_for(milliseconds(100));
        
        // Get pipeline metrics
        auto metrics = pipeline->getMetrics();
        
        // Check for stalls
        auto elapsed = duration<double>(steady_clock::now() - monitor_start).count();
        if (elapsed > 5.0 && metrics.fps_acquisition < 5.0) {
            std::cerr << "\n[Test] WARNING: Pipeline stalled! "
                      << "Acquisition FPS: " << metrics.fps_acquisition << "\n";
        }
        
        // Print detailed metrics every 5 seconds
        if (static_cast<int>(elapsed) % 5 == 0) {
            std::cout << "\n[Pipeline Metrics] "
                      << "Acq:" << metrics.fps_acquisition << " FPS, "
                      << "Proc:" << metrics.fps_processing << " FPS, "
                      << "Latency:" << metrics.latency_ms << " ms, "
                      << "Drops:" << metrics.frame_drops << "/"
                      << metrics.total_frames << "\n";
        }
    }
    
    // Stop pipeline
    std::cout << "\n[Test] Stopping capture...\n";
    pipeline->stop();
    
    // Clean up windows
    cv::destroyAllWindows();
    
    // Print final report
    stats.print();
    
    // Final pipeline metrics
    auto final_metrics = pipeline->getMetrics();
    std::cout << "\n[Pipeline Final Stats]\n";
    std::cout << "Total Frames: " << final_metrics.total_frames << "\n";
    std::cout << "Frame Drops: " << final_metrics.frame_drops << " ("
              << std::fixed << std::setprecision(1)
              << (100.0 * final_metrics.frame_drops / 
                  std::max(1UL, final_metrics.total_frames))
              << "%)\n";
}

void testCameraSystem() {
    std::cout << "\n[Test] Testing CameraSystem Integration\n";
    
    auto camera_system = camera::CameraSystem::getInstance();
    
    // Initialize with proper settings
    camera::CameraConfig config;
    config.width = 1456;
    config.height = 1088;
    config.exposureTime = 15000.0;  // 15ms
    config.gain = 3.0;
    config.autoExposure = true;
    config.targetFps = 30.0;
    config.enableSync = true;
    
    if (!camera_system->initialize(config)) {
        std::cerr << "[Test] Failed to initialize CameraSystem\n";
        return;
    }
    
    PerformanceStats stats;
    stats.start_time = steady_clock::now();
    
    // Set frame callback
    camera_system->setFrameCallback(
        [&stats](const camera::StereoFrame& frame) {
            stats.recordFrame(0.0);
            
            if (stats.frames_received % 30 == 0) {
                auto elapsed = duration<double>(
                    steady_clock::now() - stats.start_time).count();
                double fps = stats.frames_received.load() / elapsed;
                std::cout << "[CameraSystem] FPS: " << std::fixed 
                          << std::setprecision(1) << fps << "\n";
            }
        }
    );
    
    // Start capture
    if (!camera_system->startCapture()) {
        std::cerr << "[Test] Failed to start capture\n";
        return;
    }
    
    // Run for 10 seconds
    std::cout << "[Test] Running for 10 seconds...\n";
    std::this_thread::sleep_for(seconds(10));
    
    // Stop and report
    camera_system->stopCapture();
    camera_system->shutdown();
    
    stats.print();
}

int main(int argc, char* argv[]) {
    // Set up signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    std::cout << "======================================\n";
    std::cout << "Unlook 3D Scanner Performance Test\n";
    std::cout << "Target: 30 FPS @ VGA (640x480)\n";
    std::cout << "Platform: ARM64 (Raspberry Pi CM4/CM5)\n";
    std::cout << "======================================\n\n";
    
    // Test real-time pipeline
    testRealtimePipeline();
    
    if (!g_running) {
        std::cout << "\n[Test] Test interrupted by user\n";
        return 0;
    }
    
    // Optional: Test CameraSystem integration
    if (argc > 1 && std::string(argv[1]) == "--full") {
        testCameraSystem();
    }
    
    std::cout << "\n[Test] Performance test completed\n";
    
    return 0;
}