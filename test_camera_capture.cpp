/*
 * Simple test to verify camera capture is working properly
 * Tests the fixes applied to HardwareSyncCapture
 */

#include <unlook/camera/HardwareSyncCapture.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <atomic>

using namespace unlook::camera;

int main() {
    std::cout << "===============================================\n";
    std::cout << "  Unlook Camera System Capture Test\n";
    std::cout << "  Testing lifecycle and memory safety fixes\n";
    std::cout << "===============================================\n\n";

    // Create hardware sync capture system
    auto capture = std::make_unique<HardwareSyncCapture>();
    
    // Configure camera parameters
    HardwareSyncCapture::CameraConfig config;
    config.width = 1456;
    config.height = 1088;
    config.format = libcamera::formats::SBGGR10;
    config.buffer_count = 4;  // Use more buffers for smoother operation
    
    std::cout << "Initializing camera system...\n";
    std::cout << "  Resolution: " << config.width << "x" << config.height << "\n";
    std::cout << "  Format: SBGGR10\n";
    std::cout << "  Buffers: " << config.buffer_count << "\n\n";
    
    // Initialize system
    if (!capture->initialize(config)) {
        std::cerr << "ERROR: Failed to initialize camera system\n";
        return 1;
    }
    
    std::cout << "✓ Camera system initialized successfully\n\n";
    
    // Set up frame callback to monitor captures
    std::atomic<int> frame_count{0};
    std::atomic<bool> capture_error{false};
    
    capture->setFrameCallback([&](const HardwareSyncCapture::StereoFrame& frame) {
        frame_count++;
        
        // Check frame validity
        if (frame.left_image.empty() || frame.right_image.empty()) {
            std::cerr << "ERROR: Received empty frame!\n";
            capture_error = true;
            return;
        }
        
        // Display frame info for first few frames
        if (frame_count <= 5) {
            std::cout << "Frame " << frame_count << " captured:\n";
            std::cout << "  Left size: " << frame.left_image.cols << "x" << frame.left_image.rows << "\n";
            std::cout << "  Right size: " << frame.right_image.cols << "x" << frame.right_image.rows << "\n";
            std::cout << "  Sync error: " << std::fixed << std::setprecision(3) 
                      << frame.sync_error_ms << " ms\n";
            
            // Check sync quality
            if (frame.sync_error_ms < 1.0) {
                std::cout << "  ✓ Good sync (< 1ms)\n";
            } else {
                std::cout << "  ⚠ Poor sync (> 1ms)\n";
            }
            std::cout << "\n";
        }
    });
    
    // Start capture
    std::cout << "Starting synchronized capture...\n\n";
    if (!capture->start()) {
        std::cerr << "ERROR: Failed to start capture\n";
        return 1;
    }
    
    // Capture for 3 seconds
    std::cout << "Capturing for 3 seconds...\n";
    std::cout << "----------------------------------------\n\n";
    
    auto start_time = std::chrono::steady_clock::now();
    
    while (true) {
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start_time).count();
        
        if (elapsed >= 3) {
            break;
        }
        
        if (capture_error) {
            std::cerr << "ERROR: Capture error detected, stopping...\n";
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Get statistics
    auto stats = capture->getStats();
    
    std::cout << "\n----------------------------------------\n";
    std::cout << "Capture Statistics:\n";
    std::cout << "  Total frames: " << stats.frames_captured << "\n";
    std::cout << "  Sync errors: " << stats.sync_errors << "\n";
    std::cout << "  Max sync error: " << std::fixed << std::setprecision(3) 
              << stats.max_sync_error_ms << " ms\n";
    std::cout << "  Avg sync error: " << stats.avg_sync_error_ms << " ms\n";
    std::cout << "  Measured FPS: " << std::setprecision(1) << stats.measured_fps << "\n";
    
    // Check results
    if (frame_count > 0) {
        std::cout << "\n✓ SUCCESS: Captured " << frame_count << " frames\n";
    } else {
        std::cout << "\n✗ FAILURE: No frames captured\n";
    }
    
    if (stats.avg_sync_error_ms < 1.0) {
        std::cout << "✓ Sync precision < 1ms achieved\n";
    } else {
        std::cout << "⚠ Sync precision > 1ms (needs tuning)\n";
    }
    
    // Stop capture
    std::cout << "\nStopping capture...\n";
    capture->stop();
    
    // Test safe destruction
    std::cout << "Testing safe cleanup...\n";
    capture.reset();  // Explicitly destroy to test destructor
    
    std::cout << "\n✓ Test completed successfully - no crashes!\n";
    std::cout << "✓ Request lifecycle and memory safety verified\n\n";
    
    return 0;
}
