/**
 * @file basic_api_example.cpp
 * @brief Basic usage example of the Unlook 3D Scanner API
 * 
 * This example demonstrates the fundamental API usage pattern:
 * 1. Initialize the scanner
 * 2. Access subsystem APIs
 * 3. Perform basic operations
 * 4. Proper cleanup
 */

#include <unlook/unlook.h>
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "=== Unlook 3D Scanner API - Basic Example ===" << std::endl;
    std::cout << "Version: " << unlook::getVersionString() << std::endl;
    std::cout << std::endl;
    
    // Initialize global API (optional but recommended)
    auto init_result = unlook::initialize(unlook::core::LogLevel::INFO, true);
    if (init_result != unlook::core::ResultCode::SUCCESS) {
        std::cerr << "Failed to initialize Unlook API" << std::endl;
        return 1;
    }
    
    try {
        // Create scanner instance
        std::cout << "Creating scanner instance..." << std::endl;
        unlook::api::UnlookScanner scanner;
        
        // Set up status callback for monitoring
        scanner.setStatusCallback([](const unlook::core::ScannerStatus& status, void* user_data) {
            std::cout << "Status update: Initialized=" << status.initialized 
                     << ", Sync=" << static_cast<int>(status.sync_status)
                     << ", Calibration=" << status.calibration_loaded << std::endl;
        });
        
        // Initialize scanner in standalone mode
        std::cout << "Initializing scanner (standalone mode)..." << std::endl;
        auto result = scanner.initialize(unlook::core::ScannerMode::STANDALONE);
        
        if (result != unlook::core::ResultCode::SUCCESS) {
            std::cerr << "Scanner initialization failed: " << scanner.getLastError() << std::endl;
            return 1;
        }
        
        std::cout << "Scanner initialized successfully!" << std::endl;
        
        // Get current status
        auto status = scanner.getStatus();
        std::cout << "\nCurrent Status:" << std::endl;
        std::cout << "  Initialized: " << (status.initialized ? "Yes" : "No") << std::endl;
        std::cout << "  Mode: " << (status.mode == unlook::core::ScannerMode::STANDALONE ? "Standalone" : "Companion") << std::endl;
        std::cout << "  Sync Status: " << static_cast<int>(status.sync_status) << std::endl;
        std::cout << "  Calibration Loaded: " << (status.calibration_loaded ? "Yes" : "No") << std::endl;
        
        // Access camera system
        std::cout << "\nAccessing camera system..." << std::endl;
        auto* camera = scanner.getCameraSystem();
        
        if (camera && camera->isInitialized()) {
            std::cout << "Camera system ready!" << std::endl;
            
            // Get camera configuration
            auto config = camera->getCurrentConfig();
            std::cout << "Camera Configuration:" << std::endl;
            std::cout << "  Resolution: " << config.width << "x" << config.height << std::endl;
            std::cout << "  Exposure: " << config.exposure_us << " μs" << std::endl;
            std::cout << "  Gain: " << config.gain << std::endl;
            
            // Test camera controls
            std::cout << "\nTesting camera controls..." << std::endl;
            
            // Set exposure
            result = camera->setExposure(15000);
            if (result == unlook::core::ResultCode::SUCCESS) {
                std::cout << "Exposure set to 15000 μs" << std::endl;
            }
            
            // Set gain
            result = camera->setGain(1.5f);
            if (result == unlook::core::ResultCode::SUCCESS) {
                std::cout << "Gain set to 1.5" << std::endl;
            }
            
            // Get frame statistics
            uint64_t frames_captured, frames_dropped;
            double frame_rate;
            camera->getFrameStats(frames_captured, frames_dropped, frame_rate);
            std::cout << "Frame Stats: Captured=" << frames_captured 
                     << ", Dropped=" << frames_dropped 
                     << ", Rate=" << frame_rate << " fps" << std::endl;
        } else {
            std::cout << "Camera system not available" << std::endl;
        }
        
        // Access depth processor
        std::cout << "\nAccessing depth processor..." << std::endl;
        auto* depth = scanner.getDepthProcessor();
        
        if (depth && depth->isInitialized()) {
            std::cout << "Depth processor ready!" << std::endl;
            
            if (depth->hasValidCalibration()) {
                auto quality = depth->getCalibrationQuality();
                std::cout << "Calibration Quality:" << std::endl;
                std::cout << "  RMS Error: " << quality.rms_error << " pixels" << std::endl;
                std::cout << "  Baseline: " << quality.baseline_mm << " mm" << std::endl;
                std::cout << "  Precision: " << quality.precision_mm << " mm @ 100mm" << std::endl;
                std::cout << "  Valid: " << (quality.is_valid ? "Yes" : "No") << std::endl;
            } else {
                std::cout << "No valid calibration loaded" << std::endl;
            }
            
            // Get stereo parameters
            auto stereo_params = depth->getStereoParams();
            std::cout << "Stereo Parameters:" << std::endl;
            std::cout << "  Algorithm: " << (stereo_params.use_boofcv ? "BoofCV" : "OpenCV SGBM") << std::endl;
            std::cout << "  Block Size: " << stereo_params.block_size << std::endl;
            std::cout << "  Num Disparities: " << stereo_params.num_disparities << std::endl;
        } else {
            std::cout << "Depth processor not available" << std::endl;
        }
        
        // Access calibration manager
        std::cout << "\nAccessing calibration manager..." << std::endl;
        auto* calibration = scanner.getCalibrationManager();
        
        if (calibration && calibration->isInitialized()) {
            std::cout << "Calibration manager ready!" << std::endl;
            
            auto pattern = calibration->getCurrentPattern();
            std::cout << "Calibration Pattern:" << std::endl;
            std::cout << "  Type: " << static_cast<int>(pattern.type) << std::endl;
            std::cout << "  Board Size: " << pattern.board_size.width << "x" << pattern.board_size.height << std::endl;
            std::cout << "  Square Size: " << pattern.square_size_mm << " mm" << std::endl;
        } else {
            std::cout << "Calibration manager not available" << std::endl;
        }
        
        // Perform system self-test
        std::cout << "\nPerforming system self-test..." << std::endl;
        result = scanner.performSelfTest();
        
        if (result == unlook::core::ResultCode::SUCCESS) {
            std::cout << "Self-test passed!" << std::endl;
        } else {
            std::cout << "Self-test failed: " << scanner.getLastError() << std::endl;
        }
        
        // Demonstrate configuration management
        std::cout << "\nTesting configuration management..." << std::endl;
        
        // Save current configuration
        result = scanner.saveConfiguration("example_config.conf");
        if (result == unlook::core::ResultCode::SUCCESS) {
            std::cout << "Configuration saved to example_config.conf" << std::endl;
        }
        
        // Wait a bit to see any background activity
        std::cout << "\nRunning for 2 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Shutdown scanner
        std::cout << "\nShutting down scanner..." << std::endl;
        result = scanner.shutdown();
        
        if (result == unlook::core::ResultCode::SUCCESS) {
            std::cout << "Scanner shutdown complete" << std::endl;
        } else {
            std::cout << "Shutdown error: " << scanner.getLastError() << std::endl;
        }
        
    } catch (const unlook::core::Exception& e) {
        std::cerr << "Unlook exception: " << e.what() << std::endl;
        std::cerr << "Result code: " << static_cast<int>(e.getResultCode()) << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
        return 1;
    }
    
    // Cleanup global API
    std::cout << "\nCleaning up..." << std::endl;
    unlook::shutdown();
    
    std::cout << "\nExample completed successfully!" << std::endl;
    return 0;
}