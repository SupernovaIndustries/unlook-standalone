/**
 * Hardware Sync Test for IMX296 Cameras
 * 
 * This test verifies that the hardware synchronization is working correctly
 * by capturing frames from both cameras simultaneously and measuring sync error.
 */

#include <unlook/camera/CameraSystem.hpp>
#include <unlook/core/logger.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <atomic>

using namespace unlook::camera;
using namespace std::chrono;

// Stats tracking
struct SyncStats {
    std::atomic<size_t> frame_count{0};
    std::atomic<size_t> sync_errors{0};
    std::atomic<double> min_sync_error{1000.0};
    std::atomic<double> max_sync_error{0.0};
    std::atomic<double> avg_sync_error{0.0};
    std::atomic<double> fps{0.0};
};

void printStats(const SyncStats& stats) {
    std::cout << "\r[SYNC STATS] "
              << "Frames: " << stats.frame_count 
              << " | FPS: " << std::fixed << std::setprecision(1) << stats.fps.load()
              << " | Sync Error (ms): Min=" << std::fixed << std::setprecision(3) << stats.min_sync_error.load()
              << " Max=" << stats.max_sync_error.load()
              << " Avg=" << stats.avg_sync_error.load()
              << " | Errors: " << stats.sync_errors
              << "     " << std::flush;
}

int main() {
    std::cout << "=== Hardware Sync Test for IMX296 Cameras ===" << std::endl;
    std::cout << "Testing synchronized capture with XVS/XHS hardware sync" << std::endl;
    std::cout << "Camera 1 (LEFT/MASTER): /base/soc/i2c0mux/i2c@1/imx296@1a" << std::endl;
    std::cout << "Camera 0 (RIGHT/SLAVE): /base/soc/i2c0mux/i2c@0/imx296@1a" << std::endl;
    std::cout << "Target: <1ms synchronization error" << std::endl;
    std::cout << "---------------------------------------------" << std::endl;
    
    // Get camera system instance
    auto camera_system = CameraSystem::getInstance();
    
    // Configure for hardware sync
    CameraConfig config;
    config.width = 1456;
    config.height = 1088;
    config.targetFps = 30.0;
    config.enableSync = true;  // Enable hardware synchronization
    config.syncToleranceMs = 1.0;  // 1ms tolerance for sync
    config.exposureTime = 10000.0;  // 10ms exposure
    config.analogGain = 1.0;
    config.autoExposure = false;  // Manual control for testing
    
    std::cout << "\n[INIT] Initializing camera system with hardware sync..." << std::endl;
    
    if (!camera_system->initialize(config)) {
        std::cerr << "[ERROR] Failed to initialize camera system!" << std::endl;
        std::cerr << "Status: " << camera_system->getStatus().errorMessage << std::endl;
        return 1;
    }
    
    std::cout << "[INIT] Camera system initialized successfully" << std::endl;
    std::cout << "[INIT] Hardware sync enabled: " << 
                 (camera_system->isHardwareSyncEnabled() ? "YES" : "NO") << std::endl;
    
    // Test single capture first
    std::cout << "\n[TEST] Testing single synchronized capture..." << std::endl;
    
    StereoFrame test_frame;
    if (camera_system->captureStereoFrame(test_frame, 5000)) {
        std::cout << "[TEST] Single capture successful!" << std::endl;
        std::cout << "  Left timestamp:  " << test_frame.leftTimestampNs << " ns" << std::endl;
        std::cout << "  Right timestamp: " << test_frame.rightTimestampNs << " ns" << std::endl;
        std::cout << "  Sync error: " << test_frame.syncErrorMs << " ms" << std::endl;
        std::cout << "  Synchronized: " << (test_frame.isSynchronized ? "YES" : "NO") << std::endl;
        
        if (!test_frame.isSynchronized) {
            std::cerr << "[WARNING] Frames are not synchronized! Error: " 
                      << test_frame.syncErrorMs << " ms" << std::endl;
        }
    } else {
        std::cerr << "[ERROR] Single capture failed!" << std::endl;
        return 1;
    }
    
    // Now test continuous capture
    std::cout << "\n[TEST] Starting continuous synchronized capture..." << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    std::cout << "---------------------------------------------" << std::endl;
    
    SyncStats stats;
    auto start_time = steady_clock::now();
    auto last_fps_time = start_time;
    size_t fps_counter = 0;
    
    // Set up frame callback for continuous capture
    camera_system->setFrameCallback([&](const StereoFrame& frame) {
        stats.frame_count++;
        fps_counter++;
        
        // Update sync statistics
        double sync_error = frame.syncErrorMs;
        
        // Update min/max
        double current_min = stats.min_sync_error.load();
        while (sync_error < current_min && 
               !stats.min_sync_error.compare_exchange_weak(current_min, sync_error));
        
        double current_max = stats.max_sync_error.load();
        while (sync_error > current_max && 
               !stats.max_sync_error.compare_exchange_weak(current_max, sync_error));
        
        // Update average (exponential moving average)
        double current_avg = stats.avg_sync_error.load();
        stats.avg_sync_error = current_avg * 0.95 + sync_error * 0.05;
        
        // Count sync errors
        if (!frame.isSynchronized) {
            stats.sync_errors++;
        }
        
        // Calculate FPS every second
        auto now = steady_clock::now();
        auto fps_duration = duration_cast<milliseconds>(now - last_fps_time).count();
        if (fps_duration >= 1000) {
            stats.fps = (fps_counter * 1000.0) / fps_duration;
            fps_counter = 0;
            last_fps_time = now;
        }
        
        // Print stats every 10 frames
        if (stats.frame_count % 10 == 0) {
            printStats(stats);
        }
        
        // Optional: Save frames for visual inspection
        if (stats.frame_count == 10) {  // Save 10th frame pair
            cv::imwrite("test_left_sync.png", frame.leftImage);
            cv::imwrite("test_right_sync.png", frame.rightImage);
            std::cout << "\n[SAVE] Saved frame pair for inspection" << std::endl;
        }
    });
    
    // Start continuous capture
    if (!camera_system->startCapture()) {
        std::cerr << "[ERROR] Failed to start continuous capture!" << std::endl;
        return 1;
    }
    
    // Run for 30 seconds or until interrupted
    try {
        std::this_thread::sleep_for(seconds(30));
    } catch (const std::exception& e) {
        std::cout << "\n[STOP] Interrupted by user" << std::endl;
    }
    
    // Stop capture
    camera_system->stopCapture();
    
    // Print final statistics
    std::cout << "\n\n=== Final Statistics ===" << std::endl;
    std::cout << "Total frames captured: " << stats.frame_count << std::endl;
    std::cout << "Average FPS: " << stats.fps.load() << std::endl;
    std::cout << "Sync errors (>1ms): " << stats.sync_errors << std::endl;
    std::cout << "Sync error rate: " << std::fixed << std::setprecision(2) 
              << (stats.sync_errors * 100.0 / stats.frame_count) << "%" << std::endl;
    std::cout << "Min sync error: " << std::fixed << std::setprecision(3) 
              << stats.min_sync_error.load() << " ms" << std::endl;
    std::cout << "Max sync error: " << stats.max_sync_error.load() << " ms" << std::endl;
    std::cout << "Avg sync error: " << stats.avg_sync_error.load() << " ms" << std::endl;
    
    // Determine success
    bool success = (stats.max_sync_error < 1.0 && stats.sync_errors == 0);
    std::cout << "\n[RESULT] Hardware sync test: " 
              << (success ? "PASSED ✓" : "FAILED ✗") << std::endl;
    
    if (!success) {
        std::cout << "[INFO] Sync errors detected. This may indicate:" << std::endl;
        std::cout << "  - XVS/XHS wiring issues" << std::endl;
        std::cout << "  - MAS pin configuration problems" << std::endl;
        std::cout << "  - Timing configuration mismatch" << std::endl;
    }
    
    // Cleanup
    camera_system->shutdown();
    
    return success ? 0 : 1;
}