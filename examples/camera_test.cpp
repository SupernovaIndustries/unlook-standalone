/**
 * Camera System Test Application
 * 
 * Tests the hardware synchronized stereo camera system for the Unlook 3D Scanner.
 * Validates synchronization precision, frame capture, and auto-exposure functionality.
 */

#include <unlook/camera/CameraSystem.hpp>
#include <unlook/camera/CameraUtils.hpp>
#include <unlook/core/Logger.hpp>
#include <unlook/core/Configuration.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <atomic>
#include <numeric>
#include <chrono>
#include <thread>

using namespace unlook::core;
namespace camera = unlook::camera;

// Global flag for clean shutdown
std::atomic<bool> shouldExit(false);

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    shouldExit = true;
}

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help           Show this help message" << std::endl;
    std::cout << "  -c, --config FILE    Load configuration from file" << std::endl;
    std::cout << "  -f, --fps FPS        Set target FPS (default: 30)" << std::endl;
    std::cout << "  -e, --exposure US    Set exposure time in microseconds" << std::endl;
    std::cout << "  -g, --gain GAIN      Set analog gain" << std::endl;
    std::cout << "  -a, --auto           Enable auto-exposure" << std::endl;
    std::cout << "  -s, --sync           Test synchronization precision" << std::endl;
    std::cout << "  -d, --display        Display live camera feed" << std::endl;
    std::cout << "  -v, --verbose        Enable verbose logging" << std::endl;
}

void testSynchronization() {
    std::cout << "\n=== Testing Hardware Synchronization ===" << std::endl;
    
    auto system = camera::CameraSystem::getInstance();
    
    // Capture multiple frames and measure sync error
    const int numFrames = 100;
    std::vector<double> syncErrors;
    syncErrors.reserve(numFrames);
    
    std::cout << "Capturing " << numFrames << " frames to measure synchronization..." << std::endl;
    for (int i = 0; i < numFrames; ++i) {
        camera::StereoFrame frame;
        if (system->captureStereoFrame(frame, 1000)) {
            syncErrors.push_back(frame.syncErrorMs);
            
            if (i % 10 == 0) {
                std::cout << "Frame " << std::setw(3) << i << " - Sync error: " 
                         << std::fixed << std::setprecision(3) << frame.syncErrorMs << " ms"
                         << " [" << (frame.isSynchronized ? "OK" : "FAIL") << "]" << std::endl;
            }
        }
    }
    
    if (!syncErrors.empty()) {
        double avgError = std::accumulate(syncErrors.begin(), syncErrors.end(), 0.0) / syncErrors.size();
        double maxError = *std::max_element(syncErrors.begin(), syncErrors.end());
        double minError = *std::min_element(syncErrors.begin(), syncErrors.end());
        
        std::cout << "\n=== Synchronization Results ===" << std::endl;
        std::cout << "Average sync error: " << std::fixed << std::setprecision(3) << avgError << " ms" << std::endl;
        std::cout << "Max sync error:     " << std::fixed << std::setprecision(3) << maxError << " ms" << std::endl;
        std::cout << "Min sync error:     " << std::fixed << std::setprecision(3) << minError << " ms" << std::endl;
        
        if (avgError < 1.0) {
            std::cout << "✓ PASS: Hardware synchronization meets <1ms requirement" << std::endl;
        } else {
            std::cout << "✗ FAIL: Hardware synchronization exceeds 1ms tolerance" << std::endl;
        }
    }
}

void displayLiveFeed() {
    std::cout << "\n=== Live Camera Feed ===" << std::endl;
    std::cout << "Press 'q' to quit, 's' to save images, 'a' to toggle auto-exposure" << std::endl;
    
    auto system = camera::CameraSystem::getInstance();
    
    // Set up OpenCV windows
    cv::namedWindow("LEFT Camera", cv::WINDOW_NORMAL);
    cv::namedWindow("RIGHT Camera", cv::WINDOW_NORMAL);
    cv::resizeWindow("LEFT Camera", 728, 544);
    cv::resizeWindow("RIGHT Camera", 728, 544);
    
    system->setFrameCallback([](const camera::StereoFrame& frame) {
        // Display frames
        if (!frame.leftImage.empty()) {
            cv::imshow("LEFT Camera", frame.leftImage);
        }
        if (!frame.rightImage.empty()) {
            cv::imshow("RIGHT Camera", frame.rightImage);
        }
    });
    
    // FPS counter
    camera::CameraUtils::FPSCounter fpsCounter(30);
    int frameCount = 0;
    
    // Start capture
    std::cout << "Starting capture..." << std::endl;
    if (!system->startCapture()) {
        std::cerr << "Failed to start capture" << std::endl;
        return;
    }
    
    // Main display loop
    while (!shouldExit) {
        int key = cv::waitKey(1);
        
        if (key == 'q' || key == 27) {  // 'q' or ESC
            break;
        } else if (key == 's') {
            // Save current frames
            camera::StereoFrame frame;
            if (system->captureStereoFrame(frame)) {
                cv::imwrite("left_camera.png", frame.leftImage);
                cv::imwrite("right_camera.png", frame.rightImage);
                std::cout << "Images saved!" << std::endl;
            }
        } else if (key == 'a') {
            // Toggle auto-exposure
            camera::CameraConfig config = system->getConfig();
            config.autoExposure = !config.autoExposure;
            system->setAutoExposure(config.autoExposure);
            std::cout << "Auto-exposure: " << (config.autoExposure ? "ON" : "OFF") << std::endl;
        }
    }
    
    system->stopCapture();
    cv::destroyAllWindows();
}

int main(int argc, char* argv[]) {
    // Setup signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Parse command line arguments
    bool testSync = false;
    bool displayFeed = false;
    bool enableVerbose = false;
    bool autoExposure = false;
    double targetFps = 30.0;
    double exposureUs = 10000.0;
    double gain = 1.0;
    std::string configFile;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            configFile = argv[++i];
        } else if ((arg == "-f" || arg == "--fps") && i + 1 < argc) {
            targetFps = std::stod(argv[++i]);
        } else if ((arg == "-e" || arg == "--exposure") && i + 1 < argc) {
            exposureUs = std::stod(argv[++i]);
        } else if ((arg == "-g" || arg == "--gain") && i + 1 < argc) {
            gain = std::stod(argv[++i]);
        } else if (arg == "-a" || arg == "--auto") {
            autoExposure = true;
        } else if (arg == "-s" || arg == "--sync") {
            testSync = true;
        } else if (arg == "-d" || arg == "--display") {
            displayFeed = true;
        } else if (arg == "-v" || arg == "--verbose") {
            enableVerbose = true;
        }
    }
    
    // Setup logging
    if (enableVerbose) {
        Logger::getInstance().setLevel(LogLevel::DEBUG);
    } else {
        Logger::getInstance().setLevel(LogLevel::INFO);
    }
    
    // Initialize logger
    Logger::getInstance().initialize(LogLevel::INFO, true, true, "camera_test.log");
    
    std::cout << "=======================" << std::endl;
    std::cout << " Unlook Camera Test" << std::endl;
    std::cout << "=======================" << std::endl;
    
    // Load configuration
    // Configuration loading removed - using direct CameraConfig instead
    if (!configFile.empty()) {
        std::cout << "Note: Configuration file loading not yet implemented" << std::endl;
    }
    
    // Get camera system instance
    auto system = camera::CameraSystem::getInstance();
    
    // Setup camera configuration
    camera::CameraConfig cameraConfig;
    cameraConfig.width = 1456;
    cameraConfig.height = 1088;
    cameraConfig.targetFps = targetFps;
    cameraConfig.exposureTime = exposureUs;
    cameraConfig.analogGain = gain;
    cameraConfig.autoExposure = autoExposure;
    cameraConfig.enableSync = true;
    cameraConfig.syncToleranceMs = 1.0;
    
    // Initialize camera system
    std::cout << "\nInitializing camera system..." << std::endl;
    std::cout << "  Resolution: " << cameraConfig.width << "x" << cameraConfig.height << std::endl;
    std::cout << "  Target FPS: " << cameraConfig.targetFps << std::endl;
    std::cout << "  Exposure: " << cameraConfig.exposureTime << " µs" << std::endl;
    std::cout << "  Gain: " << cameraConfig.analogGain << std::endl;
    std::cout << "  Auto-exposure: " << (cameraConfig.autoExposure ? "ON" : "OFF") << std::endl;
    std::cout << "  Hardware sync: " << (cameraConfig.enableSync ? "ON" : "OFF") << std::endl;
    
    if (!system->initialize(cameraConfig)) {
        std::cerr << "Failed to initialize camera system" << std::endl;
        return 1;
    }
    
    // Print camera status
    std::cout << "\n=== Camera System Status ===" << std::endl;
    camera::CameraStatus status = system->getStatus();
    std::cout << "  Left camera ready:  " << (status.leftCameraReady ? "YES" : "NO") << std::endl;
    std::cout << "  Right camera ready: " << (status.rightCameraReady ? "YES" : "NO") << std::endl;
    std::cout << "  Synchronized:       " << (status.isSynchronized ? "YES" : "NO") << std::endl;
    
    // Run tests based on command line arguments
    if (testSync) {
        testSynchronization();
    }
    
    if (displayFeed) {
        displayLiveFeed();
    }
    
    // If no specific test requested, capture a single frame pair
    if (!testSync && !displayFeed) {
        std::cout << "\nCapturing single stereo frame..." << std::endl;
        camera::StereoFrame frame;
        if (system->captureStereoFrame(frame)) {
            std::cout << "✓ Frame captured successfully" << std::endl;
            std::cout << "  Left timestamp:  " << frame.leftTimestampNs << " ns" << std::endl;
            std::cout << "  Right timestamp: " << frame.rightTimestampNs << " ns" << std::endl;
            std::cout << "  Sync error:      " << std::fixed << std::setprecision(3) 
                     << frame.syncErrorMs << " ms" << std::endl;
            std::cout << "  Synchronized:    " << (frame.isSynchronized ? "YES" : "NO") << std::endl;
            
            // Save images
            cv::imwrite("left_test.png", frame.leftImage);
            cv::imwrite("right_test.png", frame.rightImage);
            std::cout << "Images saved to left_test.png and right_test.png" << std::endl;
        } else {
            std::cerr << "Failed to capture frame" << std::endl;
        }
    }
    
    // Print final statistics
    std::cout << "\n=== Final Statistics ===" << std::endl;
    double avgError, maxError;
    uint64_t errorCount;
    system->getSyncStats(avgError, maxError, errorCount);
    std::cout << "  Total frames:       " << status.totalFramesCaptured << std::endl;
    std::cout << "  Sync errors:        " << errorCount << std::endl;
    std::cout << "  Avg sync error:     " << std::fixed << std::setprecision(3) << avgError << " ms" << std::endl;
    std::cout << "  Max sync error:     " << std::fixed << std::setprecision(3) << maxError << " ms" << std::endl;
    
    // Shutdown
    std::cout << "\nShutting down camera system..." << std::endl;
    system->shutdown();
    
    std::cout << "Done." << std::endl;
    return 0;
}