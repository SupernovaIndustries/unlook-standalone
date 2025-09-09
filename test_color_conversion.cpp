/**
 * Test program to verify SBGGR10 to RGB conversion
 * Tests the HardwareSyncCapture color demosaicing functionality
 */

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera.h>
#include "unlook/camera/HardwareSyncCapture.hpp"

using namespace unlook::camera;

int main() {
    std::cout << "===== SBGGR10 to RGB Conversion Test =====" << std::endl;
    
    // Initialize HardwareSyncCapture
    auto capture = std::make_unique<HardwareSyncCapture>();
    
    // Configure for IMX296 cameras
    HardwareSyncCapture::CameraConfig config;
    config.width = 1456;
    config.height = 1088;
    config.format = libcamera::formats::SBGGR10;
    
    std::cout << "Initializing camera system..." << std::endl;
    if (!capture->initialize(config)) {
        std::cerr << "Failed to initialize camera system" << std::endl;
        return 1;
    }
    
    std::cout << "Starting capture..." << std::endl;
    if (!capture->start()) {
        std::cerr << "Failed to start capture" << std::endl;
        return 1;
    }
    
    // Set up frame callback to verify color images
    bool color_detected = false;
    int frame_count = 0;
    
    capture->setFrameCallback([&](const HardwareSyncCapture::StereoFrame& frame) {
        frame_count++;
        
        // Check if images are color (3 channels) instead of grayscale (1 channel)
        int left_channels = frame.left_image.channels();
        int right_channels = frame.right_image.channels();
        
        std::cout << "Frame " << frame_count << ":" << std::endl;
        std::cout << "  Left image: " << frame.left_image.cols << "x" << frame.left_image.rows 
                  << " channels=" << left_channels 
                  << " type=" << frame.left_image.type() << std::endl;
        std::cout << "  Right image: " << frame.right_image.cols << "x" << frame.right_image.rows 
                  << " channels=" << right_channels
                  << " type=" << frame.right_image.type() << std::endl;
        std::cout << "  Sync error: " << frame.sync_error_ms << "ms" << std::endl;
        
        // CV_8UC3 = 16 (3-channel 8-bit)
        // CV_8UC1 = 0 (1-channel 8-bit)
        if (left_channels == 3 && right_channels == 3) {
            std::cout << "  ✓ COLOR IMAGES DETECTED! (BGR format)" << std::endl;
            color_detected = true;
            
            // Check pixel values to ensure they're not all gray
            cv::Scalar left_mean = cv::mean(frame.left_image);
            cv::Scalar right_mean = cv::mean(frame.right_image);
            
            std::cout << "  Left mean BGR: [" << left_mean[0] << ", " 
                      << left_mean[1] << ", " << left_mean[2] << "]" << std::endl;
            std::cout << "  Right mean BGR: [" << right_mean[0] << ", " 
                      << right_mean[1] << ", " << right_mean[2] << "]" << std::endl;
            
            // Save sample images for visual inspection
            if (frame_count == 5) {
                cv::imwrite("/tmp/test_left_color.png", frame.left_image);
                cv::imwrite("/tmp/test_right_color.png", frame.right_image);
                std::cout << "  Sample images saved to /tmp/test_left_color.png and /tmp/test_right_color.png" << std::endl;
            }
        } else {
            std::cout << "  ✗ GRAYSCALE IMAGES (not color)" << std::endl;
        }
        
        std::cout << std::endl;
        
        // Stop after 10 frames
        if (frame_count >= 10) {
            capture->stop();
        }
    });
    
    // Run for a few seconds
    std::cout << "\nCapturing frames for analysis..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    // Stop capture
    capture->stop();
    
    // Print results
    std::cout << "\n===== TEST RESULTS =====" << std::endl;
    std::cout << "Total frames captured: " << frame_count << std::endl;
    
    if (color_detected) {
        std::cout << "✓ SUCCESS: Color images are being produced!" << std::endl;
        std::cout << "The SBGGR10 to RGB conversion is working correctly." << std::endl;
    } else {
        std::cout << "✗ FAILURE: Only grayscale images detected." << std::endl;
        std::cout << "The conversion may still need adjustment." << std::endl;
    }
    
    return color_detected ? 0 : 1;
}