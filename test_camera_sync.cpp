#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <signal.h>
#include "unlook/camera/camera_system.hpp"
#include "unlook/core/logger.h"

using namespace unlook;
using namespace unlook::camera;
using namespace unlook::core;

std::atomic<bool> running(true);
std::atomic<int> frame_count(0);

void signal_handler(int signal) {
    std::cout << "\nShutting down..." << std::endl;
    running = false;
}

int main() {
    signal(SIGINT, signal_handler);
    
    // Initialize logger
    Logger::getInstance().initialize(LogLevel::INFO, true, false);
    
    std::cout << "=== Unlook Camera System Test ===" << std::endl;
    std::cout << "Testing hardware-synchronized camera capture" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl << std::endl;
    
    // Get camera system instance
    auto camera_system = CameraSystem::getInstance();
    
    // Initialize cameras
    std::cout << "Initializing camera system..." << std::endl;
    if (!camera_system->initialize()) {
        std::cerr << "Failed to initialize camera system!" << std::endl;
        return 1;
    }
    
    std::cout << "Camera system initialized successfully" << std::endl;
    std::cout << "  - LEFT camera: MASTER (Camera 1)" << std::endl;
    std::cout << "  - RIGHT camera: SLAVE (Camera 0)" << std::endl;
    std::cout << "  - Resolution: 1456x1088 SBGGR10" << std::endl;
    std::cout << "  - Hardware sync: XVS/XHS enabled" << std::endl << std::endl;
    
    // Start capture with callback
    auto frame_callback = [](const core::StereoFramePair& frame_pair) {
        frame_count++;
        
        if (frame_count % 30 == 0) {  // Print every 30 frames
            std::cout << "Frame " << frame_count << ": ";
            
            if (frame_pair.synchronized) {
                std::cout << "SYNCHRONIZED";
            } else {
                std::cout << "NOT SYNCHRONIZED";
            }
            
            std::cout << " | Sync error: " << frame_pair.sync_error_ms << " ms";
            std::cout << " | Left: " << (frame_pair.left_frame.valid ? "OK" : "ERROR");
            std::cout << " | Right: " << (frame_pair.right_frame.valid ? "OK" : "ERROR");
            std::cout << std::endl;
        }
    };
    
    std::cout << "Starting synchronized capture..." << std::endl;
    if (!camera_system->startCapture(frame_callback)) {
        std::cerr << "Failed to start capture!" << std::endl;
        camera_system->shutdown();
        return 1;
    }
    
    std::cout << "Capture started successfully" << std::endl << std::endl;
    
    // Monitor capture
    auto start_time = std::chrono::steady_clock::now();
    int last_frame_count = 0;
    
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        
        int current_frames = frame_count.load();
        int fps = current_frames - last_frame_count;
        last_frame_count = current_frames;
        
        std::cout << "Status [" << elapsed << "s]: "
                  << "FPS: " << fps 
                  << " | Total frames: " << current_frames
                  << " | Avg sync error: " << camera_system->getAverageSyncError() << " ms"
                  << std::endl;
    }
    
    // Stop capture
    std::cout << std::endl << "Stopping capture..." << std::endl;
    camera_system->stopCapture();
    
    // Get final statistics
    std::cout << std::endl << "=== Final Statistics ===" << std::endl;
    std::cout << "Total frames captured: " << frame_count << std::endl;
    std::cout << "Average sync error: " << camera_system->getAverageSyncError() << " ms" << std::endl;
    
    // Shutdown
    camera_system->shutdown();
    
    std::cout << "Test completed successfully" << std::endl;
    return 0;
}