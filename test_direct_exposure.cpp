/**
 * Direct test of camera exposure settings using libcamera
 * This bypasses the Unlook system to test raw camera functionality
 */

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

using namespace libcamera;

// Convert SBGGR10 packed format to OpenCV Mat
cv::Mat convertSBGGR10ToMat(const uint8_t* data, size_t size, int width, int height) {
    cv::Mat bayer16(height, width, CV_16UC1);
    uint16_t* dst = reinterpret_cast<uint16_t*>(bayer16.data);
    
    // Unpack 10-bit packed data
    size_t pixel_count = width * height;
    for (size_t i = 0; i < pixel_count; i++) {
        size_t byte_index = (i * 5) / 4;
        int shift = ((i % 4) * 2);
        
        if (byte_index + 1 < size) {
            uint16_t value = (data[byte_index] << 2) | ((data[byte_index + 1] >> (6 - shift)) & 0x03);
            dst[i] = value << 6;  // Scale to 16-bit
        }
    }
    
    // Convert to BGR
    cv::Mat bgr;
    cv::cvtColor(bayer16, bgr, cv::COLOR_BayerBG2BGR);
    
    // Convert to 8-bit
    cv::Mat bgr8;
    bgr.convertTo(bgr8, CV_8UC3, 1.0/256.0);
    
    return bgr8;
}

int main() {
    std::cout << "===== Direct Camera Exposure Test =====" << std::endl;
    
    // Initialize camera manager
    auto cm = std::make_unique<CameraManager>();
    cm->start();
    
    // Get available cameras
    auto cameras = cm->cameras();
    if (cameras.empty()) {
        std::cerr << "No cameras found!" << std::endl;
        return 1;
    }
    
    std::cout << "Found " << cameras.size() << " camera(s)" << std::endl;
    
    // Use first camera
    auto camera = cameras[0];
    if (camera->acquire()) {
        std::cerr << "Failed to acquire camera" << std::endl;
        return 1;
    }
    
    std::cout << "Acquired camera: " << camera->id() << std::endl;
    
    // Generate configuration
    auto config = camera->generateConfiguration({StreamRole::Viewfinder});
    if (!config) {
        std::cerr << "Failed to generate configuration" << std::endl;
        return 1;
    }
    
    // Configure stream
    StreamConfiguration& cfg = config->at(0);
    cfg.size.width = 1456;
    cfg.size.height = 1088;
    cfg.pixelFormat = formats::SBGGR10;
    
    if (config->validate() == CameraConfiguration::Invalid) {
        std::cerr << "Invalid configuration" << std::endl;
        return 1;
    }
    
    if (camera->configure(config.get())) {
        std::cerr << "Failed to configure camera" << std::endl;
        return 1;
    }
    
    std::cout << "Camera configured: " << cfg.size.width << "x" << cfg.size.height << std::endl;
    
    // Allocate buffers
    FrameBufferAllocator allocator(camera);
    if (allocator.allocate(cfg.stream()) < 0) {
        std::cerr << "Failed to allocate buffers" << std::endl;
        return 1;
    }
    
    // Create requests
    std::vector<std::unique_ptr<Request>> requests;
    for (const auto& buffer : allocator.buffers(cfg.stream())) {
        auto request = camera->createRequest();
        if (!request) {
            std::cerr << "Failed to create request" << std::endl;
            return 1;
        }
        
        if (request->addBuffer(cfg.stream(), buffer.get())) {
            std::cerr << "Failed to add buffer" << std::endl;
            return 1;
        }
        
        requests.push_back(std::move(request));
    }
    
    // Test different exposure values
    std::vector<int32_t> exposure_values = {1000, 5000, 10000, 20000, 50000, 100000};
    std::vector<float> gain_values = {1.0f, 2.0f, 4.0f, 8.0f};
    
    for (int32_t exposure : exposure_values) {
        for (float gain : gain_values) {
            std::cout << "\nTesting exposure=" << exposure << "µs, gain=" << gain << "x" << std::endl;
            
            // Set controls
            ControlList controls;
            controls.set(controls::ExposureTime, exposure);
            controls.set(controls::AnalogueGain, gain);
            
            // Apply to all requests
            for (auto& request : requests) {
                request->controls() = controls;
            }
            
            // Start camera
            if (camera->start(&controls)) {
                std::cerr << "Failed to start camera" << std::endl;
                continue;
            }
            
            // Queue requests
            for (auto& request : requests) {
                if (camera->queueRequest(request.get())) {
                    std::cerr << "Failed to queue request" << std::endl;
                }
            }
            
            // Capture a few frames
            bool found_good = false;
            for (int i = 0; i < 10 && !found_good; i++) {
                // Wait for completion
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                // Process completed requests (simplified - normally would use event loop)
                // This is a simplified version for testing
            }
            
            // Check one frame manually
            // In real implementation, would process completed requests
            std::cout << "  (Processing would happen here in full implementation)" << std::endl;
            
            // Stop camera
            camera->stop();
        }
    }
    
    // Cleanup
    allocator.free(cfg.stream());
    camera->release();
    cm->stop();
    
    std::cout << "\nTest complete" << std::endl;
    return 0;
}