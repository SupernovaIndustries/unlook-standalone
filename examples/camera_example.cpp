/**
 * @file camera_example.cpp
 * @brief Camera system usage example for the Unlook 3D Scanner API
 * 
 * This example demonstrates:
 * 1. Camera initialization and configuration
 * 2. Hardware synchronization testing
 * 3. Single frame capture
 * 4. Continuous capture with callbacks
 * 5. Camera parameter adjustment
 */

#include <unlook/unlook.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>

// Global variables for callback demo
std::atomic<int> frames_received{0};
std::atomic<bool> save_next_frame{false};

/**
 * @brief Frame callback function for continuous capture
 */
void frameCallback(const cv::Mat& left_frame, 
                  const cv::Mat& right_frame,
                  uint64_t timestamp_us,
                  void* user_data) {
    
    frames_received++;
    
    // Print progress every 10 frames
    if (frames_received % 10 == 0) {
        std::cout << "Received " << frames_received << " frame pairs (timestamp: " 
                 << timestamp_us << " μs)" << std::endl;
    }
    
    // Save a frame if requested
    if (save_next_frame.load()) {
        save_next_frame = false;
        
        std::string left_filename = "captured_left_" + std::to_string(frames_received) + ".png";
        std::string right_filename = "captured_right_" + std::to_string(frames_received) + ".png";
        
        if (!left_frame.empty()) {
            cv::imwrite(left_filename, left_frame);
        }
        if (!right_frame.empty()) {
            cv::imwrite(right_filename, right_frame);
        }
        
        std::cout << "Saved frame pair: " << left_filename << ", " << right_filename << std::endl;
    }
}

int main() {
    std::cout << "=== Unlook 3D Scanner API - Camera Example ===" << std::endl;
    std::cout << "Version: " << unlook::getVersionString() << std::endl;
    std::cout << std::endl;
    
    // Initialize API
    unlook::initialize(unlook::core::LogLevel::INFO, true);
    
    try {
        // Create and initialize scanner
        unlook::api::UnlookScanner scanner;
        
        std::cout << "Initializing scanner..." << std::endl;
        auto result = scanner.initialize(unlook::core::ScannerMode::STANDALONE);
        
        if (result != unlook::core::ResultCode::SUCCESS) {
            std::cerr << "Scanner initialization failed: " << scanner.getLastError() << std::endl;
            return 1;
        }
        
        // Get camera system
        auto* camera = scanner.getCameraSystem();
        if (!camera) {
            std::cerr << "Camera system not available" << std::endl;
            return 1;
        }
        
        std::cout << "Camera system ready!" << std::endl;
        
        // Display initial camera configuration
        auto config = camera->getCurrentConfig();
        std::cout << "\nInitial Camera Configuration:" << std::endl;
        std::cout << "  Resolution: " << config.width << "x" << config.height << std::endl;
        std::cout << "  Exposure: " << config.exposure_time_us << " μs" << std::endl;
        std::cout << "  Gain: " << config.gain << std::endl;
        std::cout << "  Auto Exposure: " << (config.auto_exposure ? "On" : "Off") << std::endl;
        std::cout << "  Auto Gain: " << (config.auto_gain ? "On" : "Off") << std::endl;
        
        // Check synchronization status
        auto sync_status = camera->getSyncStatus();
        std::cout << "\nSynchronization Status: ";
        switch (sync_status) {
            case unlook::core::SyncStatus::NOT_INITIALIZED:
                std::cout << "Not Initialized" << std::endl;
                break;
            case unlook::core::SyncStatus::SYNCHRONIZING:
                std::cout << "Synchronizing..." << std::endl;
                break;
            case unlook::core::SyncStatus::SYNCHRONIZED:
                std::cout << "Synchronized ✓" << std::endl;
                break;
            case unlook::core::SyncStatus::SYNC_FAILED:
                std::cout << "Failed ✗" << std::endl;
                break;
        }
        
        // Test camera parameter adjustments
        std::cout << "\nTesting camera parameter adjustments..." << std::endl;
        
        // Adjust exposure
        uint32_t new_exposure = 12000; // 12ms
        result = camera->setExposure(new_exposure);
        if (result == unlook::core::ResultCode::SUCCESS) {
            std::cout << "Exposure set to " << new_exposure << " μs" << std::endl;
            std::cout << "Current exposure: " << camera->getExposure() << " μs" << std::endl;
        }
        
        // Adjust gain
        float new_gain = 2.0f;
        result = camera->setGain(new_gain);
        if (result == unlook::core::ResultCode::SUCCESS) {
            std::cout << "Gain set to " << new_gain << std::endl;
            std::cout << "Current gain: " << camera->getGain() << std::endl;
        }
        
        // Test exposure modes
        result = camera->setExposureMode(unlook::api::ExposureMode::FIXED);
        if (result == unlook::core::ResultCode::SUCCESS) {
            std::cout << "Exposure mode set to FIXED" << std::endl;
        }
        
        // Test single frame capture
        std::cout << "\nTesting single frame capture..." << std::endl;
        
        cv::Mat left_frame, right_frame;
        result = camera->captureSingleFrame(left_frame, right_frame, 5000); // 5 second timeout
        
        if (result == unlook::core::ResultCode::SUCCESS) {
            std::cout << "Single frame captured successfully!" << std::endl;
            std::cout << "Left frame size: " << left_frame.cols << "x" << left_frame.rows << std::endl;
            std::cout << "Right frame size: " << right_frame.cols << "x" << right_frame.rows << std::endl;
            
            // Save single frame
            if (!left_frame.empty()) {
                cv::imwrite("single_left.png", left_frame);
                std::cout << "Saved: single_left.png" << std::endl;
            }
            if (!right_frame.empty()) {
                cv::imwrite("single_right.png", right_frame);
                std::cout << "Saved: single_right.png" << std::endl;
            }
        } else {
            std::cout << "Single frame capture failed: " << camera->getLastError() << std::endl;
        }
        
        // Test L/R swap functionality
        std::cout << "\nTesting L/R swap functionality..." << std::endl;
        bool original_swap = camera->isLRSwapped();
        std::cout << "Original L/R swap state: " << (original_swap ? "Swapped" : "Normal") << std::endl;
        
        camera->setLRSwap(!original_swap);
        std::cout << "L/R swap toggled to: " << (camera->isLRSwapped() ? "Swapped" : "Normal") << std::endl;
        
        // Restore original state
        camera->setLRSwap(original_swap);
        
        // Test continuous capture with callback
        std::cout << "\nTesting continuous capture with callback..." << std::endl;
        
        // Set up frame callback
        camera->setFrameCallback(frameCallback, nullptr);
        
        // Start continuous capture
        result = camera->startCapture();
        if (result != unlook::core::ResultCode::SUCCESS) {
            std::cout << "Failed to start capture: " << camera->getLastError() << std::endl;
        } else {
            std::cout << "Continuous capture started..." << std::endl;
            
            // Run for 5 seconds
            auto start_time = std::chrono::steady_clock::now();
            
            while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(5)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                // Save a frame after 2 seconds
                if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(2) && 
                    !save_next_frame.load()) {
                    save_next_frame = true;
                }
            }
            
            // Stop capture
            result = camera->stopCapture();
            if (result == unlook::core::ResultCode::SUCCESS) {
                std::cout << "Continuous capture stopped" << std::endl;
            }
        }
        
        // Get final frame statistics
        uint64_t frames_captured, frames_dropped;
        double avg_frame_rate;
        camera->getFrameStats(frames_captured, frames_dropped, avg_frame_rate);
        
        std::cout << "\nFinal Frame Statistics:" << std::endl;
        std::cout << "  Frames Captured: " << frames_captured << std::endl;
        std::cout << "  Frames Dropped: " << frames_dropped << std::endl;
        std::cout << "  Average Frame Rate: " << avg_frame_rate << " fps" << std::endl;
        std::cout << "  Callback Received: " << frames_received.load() << std::endl;
        
        if (frames_captured > 0) {
            double efficiency = (double)(frames_captured - frames_dropped) / frames_captured * 100.0;
            std::cout << "  Capture Efficiency: " << efficiency << "%" << std::endl;
        }
        
        // Test camera temperature monitoring
        float left_temp, right_temp;
        result = camera->getCameraTemperature(left_temp, right_temp);
        if (result == unlook::core::ResultCode::SUCCESS) {
            std::cout << "\nCamera Temperatures:" << std::endl;
            std::cout << "  Left Camera: " << left_temp << "°C" << std::endl;
            std::cout << "  Right Camera: " << right_temp << "°C" << std::endl;
        }
        
        // Final configuration check
        config = camera->getCurrentConfig();
        std::cout << "\nFinal Camera Configuration:" << std::endl;
        std::cout << "  Resolution: " << config.width << "x" << config.height << std::endl;
        std::cout << "  Exposure: " << config.exposure_time_us << " μs" << std::endl;
        std::cout << "  Gain: " << config.gain << std::endl;
        std::cout << "  Exposure Mode: " << static_cast<int>(camera->getExposureMode()) << std::endl;
        std::cout << "  Gain Mode: " << static_cast<int>(camera->getGainMode()) << std::endl;
        std::cout << "  L/R Swapped: " << (camera->isLRSwapped() ? "Yes" : "No") << std::endl;
        
        // Shutdown scanner
        std::cout << "\nShutting down..." << std::endl;
        scanner.shutdown();
        
    } catch (const unlook::core::Exception& e) {
        std::cerr << "Unlook exception: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
        return 1;
    }
    
    // Cleanup
    unlook::shutdown();
    
    std::cout << "\nCamera example completed successfully!" << std::endl;
    std::cout << "Check current directory for saved image files." << std::endl;
    
    return 0;
}