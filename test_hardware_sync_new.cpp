/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Test program for HardwareSyncCapture implementation
 * Tests proper libcamera-sync-fix request lifecycle patterns
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <iomanip>

#include <unlook/camera/HardwareSyncCapture.hpp>

using namespace unlook::camera;

std::atomic<bool> running{true};

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    running = false;
}

int main() {
    // Set up signal handling
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "=== Hardware Synchronized Capture Test ===" << std::endl;
    std::cout << "Testing libcamera-sync-fix request lifecycle patterns" << std::endl;
    std::cout << "Hardware: IMX296 stereo cameras with XVS/XHS sync" << std::endl;
    std::cout << "Pattern: Camera 1=MASTER, Camera 0=SLAVE" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;
    std::cout << std::endl;

    // Create synchronized capture system
    HardwareSyncCapture capture;

    // Configure for IMX296 cameras
    HardwareSyncCapture::CameraConfig config;
    config.width = 1456;
    config.height = 1088;
    config.format = libcamera::formats::SBGGR10;  
    config.buffer_count = 4;  // Standard libcamera pool size

    // Initialize
    std::cout << "Initializing synchronized camera system..." << std::endl;
    if (!capture.initialize(config)) {
        std::cerr << "Failed to initialize camera system" << std::endl;
        return 1;
    }

    // Set frame callback for continuous capture
    int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    
    capture.setFrameCallback([&](const HardwareSyncCapture::StereoFrame& frame) {
        frame_count++;
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        double fps = elapsed > 0 ? frame_count / static_cast<double>(elapsed) : 0.0;

        std::cout << "Frame " << frame_count 
                  << " - Sync error: " << std::fixed << std::setprecision(3) 
                  << frame.sync_error_ms << " ms"
                  << " - FPS: " << std::fixed << std::setprecision(1) << fps
                  << " - Left: " << frame.left_image.cols << "x" << frame.left_image.rows
                  << " - Right: " << frame.right_image.cols << "x" << frame.right_image.rows
                  << std::endl;
                  
        if (frame.sync_error_ms > 1.0) {
            std::cout << "  WARNING: Sync error exceeds 1ms threshold!" << std::endl;
        }
    });

    // Start synchronized capture
    std::cout << "Starting synchronized capture..." << std::endl;
    if (!capture.start()) {
        std::cerr << "Failed to start synchronized capture" << std::endl;
        return 1;
    }

    std::cout << "Capture started successfully!" << std::endl;
    std::cout << "Monitoring synchronized frames..." << std::endl;
    std::cout << std::endl;

    // Monitor capture while running
    auto last_stats_time = std::chrono::steady_clock::now();
    
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_time).count() >= 5) {
            // Print statistics every 5 seconds
            auto stats = capture.getStats();
            
            std::cout << std::endl;
            std::cout << "=== Synchronization Statistics ===" << std::endl;
            std::cout << "Frames captured: " << stats.frames_captured << std::endl;
            std::cout << "Sync errors (>1ms): " << stats.sync_errors << std::endl;
            std::cout << "Max sync error: " << std::fixed << std::setprecision(3) 
                      << stats.max_sync_error_ms << " ms" << std::endl;
            std::cout << "Avg sync error: " << std::fixed << std::setprecision(3)
                      << stats.avg_sync_error_ms << " ms" << std::endl;
            std::cout << "Measured FPS: " << std::fixed << std::setprecision(1)
                      << stats.measured_fps << std::endl;
            std::cout << "Success rate: " << std::fixed << std::setprecision(1)
                      << (stats.frames_captured > 0 ? 
                          (stats.frames_captured - stats.sync_errors) * 100.0 / stats.frames_captured : 0.0)
                      << "%" << std::endl;
            std::cout << "=================================" << std::endl;
            std::cout << std::endl;
            
            last_stats_time = now;
        }
    }

    // Stop capture
    std::cout << "Stopping synchronized capture..." << std::endl;
    capture.stop();

    // Final statistics
    auto stats = capture.getStats();
    auto total_time = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time).count();

    std::cout << std::endl;
    std::cout << "=== Final Test Results ===" << std::endl;
    std::cout << "Test duration: " << total_time << " seconds" << std::endl;
    std::cout << "Total frames: " << stats.frames_captured << std::endl;
    std::cout << "Sync failures: " << stats.sync_errors << std::endl;
    std::cout << "Max sync error: " << std::fixed << std::setprecision(3) 
              << stats.max_sync_error_ms << " ms" << std::endl;
    std::cout << "Avg sync error: " << std::fixed << std::setprecision(3)
              << stats.avg_sync_error_ms << " ms" << std::endl;
    std::cout << "Average FPS: " << std::fixed << std::setprecision(1)
              << stats.measured_fps << std::endl;
    
    if (stats.frames_captured > 0) {
        double success_rate = (stats.frames_captured - stats.sync_errors) * 100.0 / stats.frames_captured;
        std::cout << "Success rate: " << std::fixed << std::setprecision(1) 
                  << success_rate << "%" << std::endl;
                  
        if (stats.max_sync_error_ms <= 1.0) {
            std::cout << "✓ PASS: All frames within 1ms sync tolerance" << std::endl;
        } else if (stats.avg_sync_error_ms <= 1.0 && success_rate >= 95.0) {
            std::cout << "✓ PASS: Average sync error within tolerance" << std::endl;  
        } else {
            std::cout << "✗ FAIL: Synchronization precision not meeting requirements" << std::endl;
        }
    }
    
    std::cout << "==========================" << std::endl;
    std::cout << std::endl;
    std::cout << "Hardware sync test completed." << std::endl;
    
    return 0;
}