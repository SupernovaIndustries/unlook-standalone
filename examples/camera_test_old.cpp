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

using namespace unlook::camera;
using namespace unlook::core;

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
    
    CameraSystem& system = CameraSystem::getInstance();
    
    // Capture multiple frames and measure sync error
    const int numFrames = 100;
    std::vector<double> syncErrors;
    
    std::cout << "Capturing " << numFrames << " frames for sync analysis..." << std::endl;
    
    for (int i = 0; i < numFrames; i++) {
        StereoFrame frame;
        if (system.captureStereoFrame(frame, 1000)) {
            syncErrors.push_back(frame.syncErrorMs);
            
            if (i % 10 == 0) {
                std::cout << "Frame " << i << ": sync error = " 
                          << std::fixed << std::setprecision(3) 
                          << frame.syncErrorMs << " ms" << std::endl;
            }
        }
    }
    
    // Calculate statistics
    if (!syncErrors.empty()) {
        double minError = *std::min_element(syncErrors.begin(), syncErrors.end());
        double maxError = *std::max_element(syncErrors.begin(), syncErrors.end());
        double avgError = std::accumulate(syncErrors.begin(), syncErrors.end(), 0.0) / syncErrors.size();
        
        std::cout << "\nSynchronization Statistics:" << std::endl;
        std::cout << "  Min error: " << std::fixed << std::setprecision(3) << minError << " ms" << std::endl;
        std::cout << "  Max error: " << std::fixed << std::setprecision(3) << maxError << " ms" << std::endl;
        std::cout << "  Avg error: " << std::fixed << std::setprecision(3) << avgError << " ms" << std::endl;
        
        // Check if within tolerance
        if (maxError <= 1.0) {
            std::cout << "  Result: PASS - Synchronization within 1ms tolerance" << std::endl;
        } else {
            std::cout << "  Result: FAIL - Synchronization exceeds 1ms tolerance" << std::endl;
        }
    }
}

void displayLiveFeed() {
    std::cout << "\n=== Live Camera Feed ===" << std::endl;
    std::cout << "Press 'q' to quit, 's' to save images, 'a' to toggle auto-exposure" << std::endl;
    
    CameraSystem& system = CameraSystem::getInstance();
    
    // Create display windows
    cv::namedWindow("Left Camera", cv::WINDOW_NORMAL);
    cv::namedWindow("Right Camera", cv::WINDOW_NORMAL);
    cv::namedWindow("Combined", cv::WINDOW_NORMAL);
    
    // Set frame callback for continuous display
    system.setFrameCallback([](const StereoFrame& frame) {
        if (shouldExit) return;
        
        // Add debug overlay
        cv::Mat leftDisplay = frame.leftImage.clone();
        cv::Mat rightDisplay = frame.rightImage.clone();
        
        CameraUtils::addDebugOverlay(leftDisplay, "LEFT (MASTER)", 
                                      30.0, frame.leftExposure, frame.leftGain, 
                                      frame.syncErrorMs);
        
        CameraUtils::addDebugOverlay(rightDisplay, "RIGHT (SLAVE)", 
                                      30.0, frame.rightExposure, frame.rightGain, 
                                      frame.syncErrorMs);
        
        // Create combined view
        cv::Mat combined;
        cv::hconcat(leftDisplay, rightDisplay, combined);
        
        // Display images
        cv::imshow("Left Camera", leftDisplay);
        cv::imshow("Right Camera", rightDisplay);
        cv::imshow("Combined", combined);
    });
    
    // Start capture
    if (!system.startCapture()) {
        std::cerr << "Failed to start camera capture" << std::endl;
        return;
    }
    
    // Main display loop
    while (!shouldExit) {
        int key = cv::waitKey(30);
        
        if (key == 'q' || key == 27) {  // 'q' or ESC
            break;
        } else if (key == 's') {
            // Save current frames
            StereoFrame frame;
            if (system.captureStereoFrame(frame)) {
                std::string timestamp = std::to_string(CameraUtils::getTimestampNs());
                cv::imwrite("left_" + timestamp + ".png", frame.leftImage);
                cv::imwrite("right_" + timestamp + ".png", frame.rightImage);
                std::cout << "Images saved with timestamp: " + timestamp << std::endl;
            }
        } else if (key == 'a') {
            // Toggle auto-exposure
            CameraConfig config = system.getConfig();
            config.autoExposure = !config.autoExposure;
            system.setAutoExposure(config.autoExposure);
            std::cout << "Auto-exposure: " << (config.autoExposure ? "ON" : "OFF") << std::endl;
        }
    }
    
    // Stop capture
    system.stopCapture();
    
    // Close windows
    cv::destroyAllWindows();
}

int main(int argc, char** argv) {
    // Setup signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // Parse command line arguments
    CameraConfig config;
    bool testSync = false;
    bool displayFeed = false;
    bool verbose = false;
    std::string configFile;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "-c" || arg == "--config") {
            if (i + 1 < argc) {
                configFile = argv[++i];
            }
        } else if (arg == "-f" || arg == "--fps") {
            if (i + 1 < argc) {
                config.targetFps = std::stod(argv[++i]);
            }
        } else if (arg == "-e" || arg == "--exposure") {
            if (i + 1 < argc) {
                config.exposureTime = std::stod(argv[++i]);
                config.autoExposure = false;
            }
        } else if (arg == "-g" || arg == "--gain") {
            if (i + 1 < argc) {
                config.analogGain = std::stod(argv[++i]);
            }
        } else if (arg == "-a" || arg == "--auto") {
            config.autoExposure = true;
        } else if (arg == "-s" || arg == "--sync") {
            testSync = true;
        } else if (arg == "-d" || arg == "--display") {
            displayFeed = true;
        } else if (arg == "-v" || arg == "--verbose") {
            verbose = true;
        }
    }
    
    // Setup logging
    Logger& logger = Logger::getInstance();
    logger.setLevel(verbose ? LogLevel::DEBUG : LogLevel::INFO);
    logger.setConsoleOutput(true);
    
    // Load configuration if specified
    if (!configFile.empty()) {
        Configuration& cfg = Configuration::getInstance();
        if (cfg.load(configFile)) {
            LOG_INFO("Loaded configuration from: " + configFile);
            config = cfg.getCameraConfig();
        } else {
            LOG_ERROR("Failed to load configuration from: " + configFile);
        }
    }
    
    // Print system information
    std::cout << "\n=== Unlook 3D Scanner - Camera System Test ===" << std::endl;
    std::cout << "Platform: " << (CameraUtils::isRaspberryPi() ? "Raspberry Pi" : "Other") << std::endl;
    
    if (CameraUtils::isRaspberryPi()) {
        std::cout << "CPU Temperature: " << std::fixed << std::setprecision(1) 
                  << CameraUtils::getCPUTemperature() << "°C" << std::endl;
    }
    
    // List available cameras
    std::cout << "\nAvailable cameras:" << std::endl;
    auto cameras = CameraUtils::listCameras();
    for (size_t i = 0; i < cameras.size(); i++) {
        std::cout << "  Camera " << i << ": " << cameras[i] << std::endl;
        
        // Validate IMX296
        if (CameraUtils::validateIMX296Camera(i)) {
            std::cout << "    -> IMX296 sensor detected" << std::endl;
        }
    }
    
    if (cameras.size() < 2) {
        std::cerr << "\nError: Need at least 2 cameras for stereo operation" << std::endl;
        return 1;
    }
    
    // Initialize camera system
    std::cout << "\nInitializing camera system..." << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Resolution: " << config.width << "x" << config.height << std::endl;
    std::cout << "  Target FPS: " << config.targetFps << std::endl;
    std::cout << "  Exposure: " << CameraUtils::formatExposure(config.exposureTime) << std::endl;
    std::cout << "  Gain: " << CameraUtils::formatGain(config.analogGain) << std::endl;
    std::cout << "  Auto-exposure: " << (config.autoExposure ? "ON" : "OFF") << std::endl;
    std::cout << "  Hardware sync: " << (config.enableSync ? "ON" : "OFF") << std::endl;
    std::cout << "  Baseline: " << config.baseline << " mm" << std::endl;
    
    CameraSystem& system = CameraSystem::getInstance();
    
    if (!system.initialize(config)) {
        std::cerr << "Failed to initialize camera system" << std::endl;
        return 1;
    }
    
    std::cout << "Camera system initialized successfully" << std::endl;
    
    // Get initial status
    CameraStatus status = system.getStatus();
    std::cout << "\nSystem Status:" << std::endl;
    std::cout << "  Left camera: " << (status.leftCameraReady ? "READY" : "NOT READY") << std::endl;
    std::cout << "  Right camera: " << (status.rightCameraReady ? "READY" : "NOT READY") << std::endl;
    std::cout << "  Synchronized: " << (status.isSynchronized ? "YES" : "NO") << std::endl;
    
    // Run tests
    if (testSync) {
        testSynchronization();
    }
    
    if (displayFeed) {
        displayLiveFeed();
    } else if (!testSync) {
        // Default: capture a single stereo frame
        std::cout << "\nCapturing single stereo frame..." << std::endl;
        
        StereoFrame frame;
        if (system.captureStereoFrame(frame)) {
            std::cout << "Frame captured successfully:" << std::endl;
            std::cout << "  Left timestamp: " << frame.leftTimestampNs << " ns" << std::endl;
            std::cout << "  Right timestamp: " << frame.rightTimestampNs << " ns" << std::endl;
            std::cout << "  Sync error: " << std::fixed << std::setprecision(3) 
                      << frame.syncErrorMs << " ms" << std::endl;
            std::cout << "  Synchronized: " << (frame.isSynchronized ? "YES" : "NO") << std::endl;
            
            // Save images
            cv::imwrite("test_left.png", frame.leftImage);
            cv::imwrite("test_right.png", frame.rightImage);
            std::cout << "Images saved as test_left.png and test_right.png" << std::endl;
        } else {
            std::cerr << "Failed to capture stereo frame" << std::endl;
        }
    }
    
    // Get final statistics
    double avgError, maxError;
    uint64_t errorCount;
    system.getSyncStats(avgError, maxError, errorCount);
    
    std::cout << "\nFinal Statistics:" << std::endl;
    std::cout << "  Total frames: " << status.totalFramesCaptured << std::endl;
    std::cout << "  Sync errors: " << errorCount << std::endl;
    std::cout << "  Avg sync error: " << std::fixed << std::setprecision(3) << avgError << " ms" << std::endl;
    std::cout << "  Max sync error: " << std::fixed << std::setprecision(3) << maxError << " ms" << std::endl;
    
    // Shutdown
    std::cout << "\nShutting down camera system..." << std::endl;
    system.shutdown();
    
    std::cout << "Test completed successfully" << std::endl;
    
    return 0;
}