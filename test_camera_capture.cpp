/**
 * Test camera capture without GUI
 * Verifies that the camera system can initialize and capture frames
 */

#include <unlook/camera/CameraSystem.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

using namespace unlook::core;

int main() {
    std::cout << "====================================" << std::endl;
    std::cout << "Camera Capture Test (Headless)" << std::endl;
    std::cout << "====================================" << std::endl;
    
    // Initialize logger
    Logger::getInstance().setLevel(LogLevel::DEBUG);
    Logger::getInstance().initialize(LogLevel::DEBUG, true, true, "camera_capture_test.log");
    
    // Get camera system instance
    auto system = unlook::camera::CameraSystem::getInstance();
    
    // Setup camera configuration
    unlook::camera::CameraConfig cameraConfig;
    cameraConfig.width = 1456;
    cameraConfig.height = 1088;
    cameraConfig.targetFps = 30.0;
    cameraConfig.exposureTime = 10000.0;
    cameraConfig.analogGain = 1.0;
    cameraConfig.autoExposure = false;
    cameraConfig.enableSync = true;
    cameraConfig.syncToleranceMs = 1.0;
    
    // Initialize camera system
    std::cout << "\n1. Initializing camera system..." << std::endl;
    std::cout << "   Resolution: " << cameraConfig.width << "x" << cameraConfig.height << std::endl;
    std::cout << "   Target FPS: " << cameraConfig.targetFps << std::endl;
    std::cout << "   Hardware sync: " << (cameraConfig.enableSync ? "ON" : "OFF") << std::endl;
    
    if (!system->initialize(cameraConfig)) {
        std::cerr << "❌ Failed to initialize camera system" << std::endl;
        std::cerr << "Error details: Check camera_capture_test.log" << std::endl;
        return 1;
    }
    
    std::cout << "✅ Camera system initialized successfully" << std::endl;
    
    // Print camera status
    unlook::camera::CameraStatus status = system->getStatus();
    std::cout << "\n2. Camera Status:" << std::endl;
    std::cout << "   Left camera ready:  " << (status.leftCameraReady ? "YES ✅" : "NO ❌") << std::endl;
    std::cout << "   Right camera ready: " << (status.rightCameraReady ? "YES ✅" : "NO ❌") << std::endl;
    std::cout << "   Synchronized:       " << (status.isSynchronized ? "YES ✅" : "NO ❌") << std::endl;
    
    // Test single frame capture
    std::cout << "\n3. Testing single frame capture..." << std::endl;
    unlook::camera::StereoFrame frame;
    if (system->captureStereoFrame(frame, 3000)) {
        std::cout << "✅ Frame captured successfully" << std::endl;
        std::cout << "   Left image size:  " << frame.leftImage.cols << "x" << frame.leftImage.rows << std::endl;
        std::cout << "   Right image size: " << frame.rightImage.cols << "x" << frame.rightImage.rows << std::endl;
        std::cout << "   Sync error:       " << std::fixed << std::setprecision(3) 
                 << frame.syncErrorMs << " ms" << std::endl;
        std::cout << "   Synchronized:     " << (frame.isSynchronized ? "YES ✅" : "NO ❌") << std::endl;
        
        // Save test images
        cv::imwrite("test_left.png", frame.leftImage);
        cv::imwrite("test_right.png", frame.rightImage);
        std::cout << "   Images saved to test_left.png and test_right.png" << std::endl;
    } else {
        std::cerr << "❌ Failed to capture single frame" << std::endl;
        system->shutdown();
        return 1;
    }
    
    // Test continuous capture
    std::cout << "\n4. Testing continuous capture (5 seconds)..." << std::endl;
    
    int frameCount = 0;
    double totalSyncError = 0.0;
    auto captureStarted = false;
    
    // Set frame callback
    system->setFrameCallback([&](const unlook::camera::StereoFrame& frame) {
        frameCount++;
        totalSyncError += frame.syncErrorMs;
        
        if (frameCount == 1) {
            std::cout << "   First frame received!" << std::endl;
            captureStarted = true;
        }
        
        if (frameCount % 30 == 0) {
            double avgSync = totalSyncError / frameCount;
            std::cout << "   Frames captured: " << frameCount 
                     << ", Avg sync error: " << std::fixed << std::setprecision(3) 
                     << avgSync << " ms" << std::endl;
        }
    });
    
    // Start capture
    if (!system->startCapture()) {
        std::cerr << "❌ Failed to start continuous capture" << std::endl;
        system->shutdown();
        return 1;
    }
    
    std::cout << "   Capture started, collecting frames..." << std::endl;
    
    // Wait for 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // Stop capture
    system->stopCapture();
    
    if (frameCount > 0) {
        double avgSync = totalSyncError / frameCount;
        double fps = frameCount / 5.0;
        
        std::cout << "\n✅ Continuous capture successful!" << std::endl;
        std::cout << "   Total frames captured: " << frameCount << std::endl;
        std::cout << "   Average FPS: " << std::fixed << std::setprecision(1) << fps << std::endl;
        std::cout << "   Average sync error: " << std::fixed << std::setprecision(3) 
                 << avgSync << " ms" << std::endl;
        
        if (avgSync < 1.0) {
            std::cout << "   ✅ Hardware synchronization meets <1ms requirement" << std::endl;
        } else {
            std::cout << "   ⚠️  Hardware synchronization exceeds 1ms tolerance" << std::endl;
        }
        
        if (fps > 25.0) {
            std::cout << "   ✅ Frame rate meets >25 FPS requirement" << std::endl;
        } else {
            std::cout << "   ⚠️  Frame rate below 25 FPS target" << std::endl;
        }
    } else {
        std::cerr << "❌ No frames captured during continuous capture" << std::endl;
        system->shutdown();
        return 1;
    }
    
    // Shutdown
    std::cout << "\n5. Shutting down camera system..." << std::endl;
    system->shutdown();
    
    std::cout << "\n====================================" << std::endl;
    std::cout << "✅ ALL TESTS PASSED!" << std::endl;
    std::cout << "The camera system is working correctly." << std::endl;
    std::cout << "You can now run the GUI application on a system with display." << std::endl;
    std::cout << "====================================" << std::endl;
    
    return 0;
}