/**
 * Test program to diagnose and fix black camera images
 * Tests different exposure values and auto-exposure settings
 */

#include <unlook/camera/CameraSystem.hpp>
#include <unlook/camera/CameraUtils.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

using namespace unlook::core;
namespace camera = unlook::camera;

void testExposureSettings() {
    std::cout << "=== Camera Exposure Test ===" << std::endl;
    
    // Initialize logger
    Logger::getInstance().initialize(LogLevel::DEBUG, true, false);
    
    // Get camera system instance
    auto system = camera::CameraSystem::getInstance();
    
    // Configure with different exposure settings
    camera::CameraConfig config;
    config.width = 1456;
    config.height = 1088;
    config.targetFps = 30.0;
    config.enableSync = true;
    config.syncToleranceMs = 1.0;
    
    // Test different exposure values
    std::vector<double> exposureValues = {
        1000.0,   // 1ms
        5000.0,   // 5ms
        10000.0,  // 10ms
        20000.0,  // 20ms
        30000.0,  // 30ms
        50000.0   // 50ms
    };
    
    std::vector<double> gainValues = {
        1.0,  // 1x
        2.0,  // 2x
        4.0,  // 4x
        8.0   // 8x
    };
    
    // First try with auto-exposure
    std::cout << "\n1. Testing with AUTO-EXPOSURE enabled..." << std::endl;
    config.autoExposure = true;
    config.exposureTime = 10000.0;  // Start at 10ms
    config.analogGain = 2.0;        // Start at 2x gain
    
    if (!system->initialize(config)) {
        std::cerr << "Failed to initialize camera system" << std::endl;
        return;
    }
    
    // Capture a frame with auto-exposure
    std::cout << "Capturing frame with auto-exposure..." << std::endl;
    camera::StereoFrame frame;
    if (system->captureStereoFrame(frame, 2000)) {
        std::cout << "Frame captured!" << std::endl;
        
        // Check image brightness
        if (!frame.leftImage.empty()) {
            cv::Scalar meanBrightness = cv::mean(frame.leftImage);
            std::cout << "Left image mean brightness: " << meanBrightness[0] << std::endl;
            
            // Save image for inspection
            cv::imwrite("test_auto_exposure_left.png", frame.leftImage);
            
            // Display histogram
            cv::Mat hist;
            int histSize = 256;
            float range[] = {0, 256};
            const float* histRange = {range};
            cv::calcHist(&frame.leftImage, 1, nullptr, cv::Mat(), hist, 1, &histSize, &histRange);
            
            // Find min/max pixel values
            double minVal, maxVal;
            cv::minMaxLoc(frame.leftImage, &minVal, &maxVal);
            std::cout << "Pixel value range: [" << minVal << ", " << maxVal << "]" << std::endl;
            
            if (meanBrightness[0] < 10) {
                std::cout << "WARNING: Image is very dark!" << std::endl;
            }
        }
    } else {
        std::cerr << "Failed to capture frame with auto-exposure" << std::endl;
    }
    
    // Shutdown and reinitialize for manual exposure tests
    system->shutdown();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Test manual exposure settings
    std::cout << "\n2. Testing MANUAL exposure settings..." << std::endl;
    config.autoExposure = false;
    
    for (double exposure : exposureValues) {
        for (double gain : gainValues) {
            std::cout << "\nTesting exposure=" << exposure << "µs, gain=" << gain << "x" << std::endl;
            
            config.exposureTime = exposure;
            config.analogGain = gain;
            
            // Reinitialize with new settings
            if (!system->initialize(config)) {
                std::cerr << "Failed to initialize with exposure=" << exposure << std::endl;
                continue;
            }
            
            // Wait for settings to apply
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Capture frame
            if (system->captureStereoFrame(frame, 2000)) {
                if (!frame.leftImage.empty()) {
                    cv::Scalar meanBrightness = cv::mean(frame.leftImage);
                    std::cout << "  Mean brightness: " << meanBrightness[0] << std::endl;
                    
                    // Save best image
                    if (meanBrightness[0] > 50 && meanBrightness[0] < 200) {
                        std::string filename = "good_exposure_" + std::to_string(int(exposure)) + 
                                             "_" + std::to_string(int(gain)) + ".png";
                        cv::imwrite(filename, frame.leftImage);
                        std::cout << "  GOOD EXPOSURE! Saved to " << filename << std::endl;
                        
                        // Display the image
                        cv::namedWindow("Good Exposure", cv::WINDOW_NORMAL);
                        cv::imshow("Good Exposure", frame.leftImage);
                        cv::waitKey(2000);
                    }
                    
                    // Stop if we found a good setting
                    if (meanBrightness[0] > 30) {
                        std::cout << "  Found visible image!" << std::endl;
                        goto found_good_settings;
                    }
                }
            }
            
            system->shutdown();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
    
found_good_settings:
    
    // Test if the problem is with format conversion
    std::cout << "\n3. Testing RAW Bayer data..." << std::endl;
    if (system->captureStereoFrame(frame, 2000)) {
        if (!frame.leftImage.empty()) {
            // Check if the raw data has values
            std::cout << "Image type: " << frame.leftImage.type() << std::endl;
            std::cout << "Image depth: " << frame.leftImage.depth() << std::endl;
            std::cout << "Image channels: " << frame.leftImage.channels() << std::endl;
            
            // Sample some pixels directly
            std::cout << "Sample pixel values:" << std::endl;
            for (int y = 100; y < 110; y += 2) {
                for (int x = 100; x < 110; x += 2) {
                    if (frame.leftImage.type() == CV_8UC1) {
                        int val = frame.leftImage.at<uint8_t>(y, x);
                        std::cout << val << " ";
                    } else if (frame.leftImage.type() == CV_16UC1) {
                        int val = frame.leftImage.at<uint16_t>(y, x);
                        std::cout << val << " ";
                    }
                }
                std::cout << std::endl;
            }
        }
    }
    
    std::cout << "\n=== Exposure Test Complete ===" << std::endl;
    std::cout << "Check the generated images to see which settings work." << std::endl;
    
    // Clean shutdown
    cv::destroyAllWindows();
    system->shutdown();
}

int main() {
    try {
        testExposureSettings();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}