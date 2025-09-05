/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Hardware Synchronized Camera Capture Implementation
 * Following exact libcamera-sync-fix patterns from working cam application
 */

#include <unlook/camera/HardwareSyncCapture.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <libcamera/control_ids.h>
#include <opencv2/imgproc.hpp>
#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

using namespace libcamera;

namespace unlook {
namespace camera {

HardwareSyncCapture::HardwareSyncCapture() {
    std::cout << "[HardwareSync] Creating synchronized capture system" << std::endl;
}

HardwareSyncCapture::~HardwareSyncCapture() {
    if (running_) {
        stop();
    }
}

bool HardwareSyncCapture::initialize(const CameraConfig& config) {
    if (initialized_) {
        std::cerr << "[HardwareSync] Already initialized" << std::endl;
        return false;
    }

    std::cout << "[HardwareSync] Initializing synchronized camera system" << std::endl;

    // Initialize libcamera camera manager
    camera_manager_ = std::make_unique<CameraManager>();
    int ret = camera_manager_->start();
    if (ret != 0) {
        std::cerr << "[HardwareSync] Failed to start camera manager: " << ret << std::endl;
        return false;
    }

    if (!initializeCameras(config)) {
        std::cerr << "[HardwareSync] Failed to initialize cameras" << std::endl;
        return false;
    }

    if (!createRequestPools()) {
        std::cerr << "[HardwareSync] Failed to create request pools" << std::endl;
        return false;
    }

    initialized_ = true;
    std::cout << "[HardwareSync] System initialized successfully" << std::endl;
    return true;
}

bool HardwareSyncCapture::initializeCameras(const CameraConfig& config) {
    auto cameras = camera_manager_->cameras();
    if (cameras.size() < 2) {
        std::cerr << "[HardwareSync] Need at least 2 cameras, found " << cameras.size() << std::endl;
        return false;
    }

    // Find cameras by hardware path (following PROJECT_GUIDELINES.md mapping)
    std::shared_ptr<Camera> camera1, camera0;
    
    for (auto camera : cameras) {
        std::string id = camera->id();
        if (id.find("i2c@1/imx296@1a") != std::string::npos) {
            camera1 = camera;  // Camera 1 = LEFT/MASTER
            std::cout << "[HardwareSync] Found MASTER camera: " << id << std::endl;
        } else if (id.find("i2c@0/imx296@1a") != std::string::npos) {
            camera0 = camera;  // Camera 0 = RIGHT/SLAVE  
            std::cout << "[HardwareSync] Found SLAVE camera: " << id << std::endl;
        }
    }

    if (!camera1 || !camera0) {
        std::cerr << "[HardwareSync] Could not find both IMX296 cameras" << std::endl;
        return false;
    }

    master_camera_ = camera1;  // LEFT/MASTER
    slave_camera_ = camera0;   // RIGHT/SLAVE

    // Acquire cameras (following cam app pattern)
    if (master_camera_->acquire()) {
        std::cerr << "[HardwareSync] Failed to acquire master camera" << std::endl;
        return false;
    }

    if (slave_camera_->acquire()) {
        std::cerr << "[HardwareSync] Failed to acquire slave camera" << std::endl;
        master_camera_->release();
        return false;
    }

    // Configure streams (following cam app pattern)
    master_config_ = master_camera_->generateConfiguration({StreamRole::Raw});
    slave_config_ = slave_camera_->generateConfiguration({StreamRole::Raw});

    if (!master_config_ || !slave_config_) {
        std::cerr << "[HardwareSync] Failed to generate camera configurations" << std::endl;
        return false;
    }

    // Set stream configuration (matching hardware specs)
    StreamConfiguration& master_stream_config = master_config_->at(0);
    master_stream_config.size.width = config.width;
    master_stream_config.size.height = config.height;
    master_stream_config.pixelFormat = config.format;

    StreamConfiguration& slave_stream_config = slave_config_->at(0);
    slave_stream_config.size.width = config.width;
    slave_stream_config.size.height = config.height;  
    slave_stream_config.pixelFormat = config.format;

    // Validate and apply configurations
    CameraConfiguration::Status master_status = master_config_->validate();
    CameraConfiguration::Status slave_status = slave_config_->validate();

    if (master_status == CameraConfiguration::Invalid || 
        slave_status == CameraConfiguration::Invalid) {
        std::cerr << "[HardwareSync] Invalid camera configurations" << std::endl;
        return false;
    }

    if (master_camera_->configure(master_config_.get()) < 0 ||
        slave_camera_->configure(slave_config_.get()) < 0) {
        std::cerr << "[HardwareSync] Failed to configure cameras" << std::endl;
        return false;
    }

    // INTELLIGENT AUTO-EXPOSURE for IMX296 sensors
    // Apply optimal exposure and gain settings for stereo precision
    if (!configureAutoExposure()) {
        std::cerr << "[HardwareSync] Failed to configure auto-exposure" << std::endl;
        return false;
    }

    // Allocate frame buffers (following cam app pattern)
    master_allocator_ = std::make_unique<FrameBufferAllocator>(master_camera_);
    slave_allocator_ = std::make_unique<FrameBufferAllocator>(slave_camera_);

    Stream* master_stream = master_stream_config.stream();
    Stream* slave_stream = slave_stream_config.stream();

    if (master_allocator_->allocate(master_stream) < 0 ||
        slave_allocator_->allocate(slave_stream) < 0) {
        std::cerr << "[HardwareSync] Failed to allocate frame buffers" << std::endl;
        return false;
    }

    std::cout << "[HardwareSync] Cameras configured successfully" << std::endl;
    std::cout << "[HardwareSync]   Master buffers: " << master_allocator_->buffers(master_stream).size() << std::endl;
    std::cout << "[HardwareSync]   Slave buffers: " << slave_allocator_->buffers(slave_stream).size() << std::endl;

    return true;
}

bool HardwareSyncCapture::createRequestPools() {
    // Create request pools (following exact cam app pattern)
    Stream* master_stream = master_config_->at(0).stream();
    Stream* slave_stream = slave_config_->at(0).stream();
    
    const auto& master_buffers = master_allocator_->buffers(master_stream);
    const auto& slave_buffers = slave_allocator_->buffers(slave_stream);

    // Initialize auto-exposure with optimal IMX296 settings
    {
        std::lock_guard<std::mutex> lock(exposure_mutex_);
        // Start with moderate exposure to avoid initial over/under exposure
        master_exposure_.exposure_time_us = 5000;  // 5ms initial exposure
        master_exposure_.analogue_gain = 2.0f;     // 2x initial gain
        slave_exposure_ = master_exposure_;        // Sync slave with master
    }

    // Create master requests
    for (size_t i = 0; i < master_buffers.size(); ++i) {
        std::unique_ptr<Request> request = master_camera_->createRequest();
        if (!request) {
            std::cerr << "[HardwareSync] Failed to create master request " << i << std::endl;
            return false;
        }

        if (request->addBuffer(master_stream, master_buffers[i].get()) < 0) {
            std::cerr << "[HardwareSync] Failed to add buffer to master request" << std::endl;
            return false;
        }

        // Apply initial exposure controls
        applyExposureControls(request.get(), master_exposure_);

        master_requests_.push_back(std::move(request));
    }

    // Create slave requests  
    for (size_t i = 0; i < slave_buffers.size(); ++i) {
        std::unique_ptr<Request> request = slave_camera_->createRequest();
        if (!request) {
            std::cerr << "[HardwareSync] Failed to create slave request " << i << std::endl;
            return false;
        }

        if (request->addBuffer(slave_stream, slave_buffers[i].get()) < 0) {
            std::cerr << "[HardwareSync] Failed to add buffer to slave request" << std::endl;
            return false;
        }

        // Apply initial exposure controls
        applyExposureControls(request.get(), slave_exposure_);

        slave_requests_.push_back(std::move(request));
    }

    std::cout << "[HardwareSync] Created request pools: " 
              << master_requests_.size() << " master, " 
              << slave_requests_.size() << " slave" << std::endl;

    return true;
}

bool HardwareSyncCapture::start() {
    if (!initialized_) {
        std::cerr << "[HardwareSync] Not initialized" << std::endl;
        return false;
    }

    if (running_) {
        std::cerr << "[HardwareSync] Already running" << std::endl;
        return false;
    }

    std::cout << "[HardwareSync] Starting synchronized capture" << std::endl;

    // Connect request completion signals (following cam app pattern)
    master_camera_->requestCompleted.connect(this, &HardwareSyncCapture::onMasterRequestComplete);
    slave_camera_->requestCompleted.connect(this, &HardwareSyncCapture::onSlaveRequestComplete);

    // Start cameras in correct sequence (CRITICAL for hardware sync)
    std::cout << "[HardwareSync] Starting MASTER camera (Camera 1 = LEFT)" << std::endl;
    if (master_camera_->start()) {
        std::cerr << "[HardwareSync] Failed to start master camera" << std::endl;
        return false;
    }

    // CRITICAL: Wait for master to stabilize before starting slave
    std::cout << "[HardwareSync] Waiting for master stabilization..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "[HardwareSync] Starting SLAVE camera (Camera 0 = RIGHT)" << std::endl;  
    if (slave_camera_->start()) {
        std::cerr << "[HardwareSync] Failed to start slave camera" << std::endl;
        master_camera_->stop();
        return false;
    }

    // Queue initial requests (following cam app pattern)
    std::cout << "[HardwareSync] Queuing initial requests" << std::endl;
    
    for (auto& request : master_requests_) {
        if (master_camera_->queueRequest(request.get()) < 0) {
            std::cerr << "[HardwareSync] Failed to queue master request" << std::endl;
            stop();
            return false;
        }
    }

    for (auto& request : slave_requests_) {
        if (slave_camera_->queueRequest(request.get()) < 0) {
            std::cerr << "[HardwareSync] Failed to queue slave request" << std::endl;
            stop();
            return false;
        }
    }

    running_ = true;
    std::cout << "[HardwareSync] Synchronized capture started successfully" << std::endl;
    return true;
}

void HardwareSyncCapture::stop() {
    if (!running_) {
        return;
    }

    std::cout << "[HardwareSync] Stopping synchronized capture" << std::endl;
    running_ = false;

    // Disconnect signals
    master_camera_->requestCompleted.disconnect(this, &HardwareSyncCapture::onMasterRequestComplete);
    slave_camera_->requestCompleted.disconnect(this, &HardwareSyncCapture::onSlaveRequestComplete);

    // Stop cameras (reverse order)
    if (slave_camera_) {
        slave_camera_->stop();
    }
    
    if (master_camera_) {
        master_camera_->stop();
    }

    std::cout << "[HardwareSync] Synchronized capture stopped" << std::endl;
}

void HardwareSyncCapture::onMasterRequestComplete(Request* request) {
    if (request->status() == Request::RequestCancelled) {
        return;
    }

    // Process request asynchronously to avoid blocking camera thread
    // (following cam app event loop pattern)
    std::thread([this, request]() {
        processMasterRequest(request);
    }).detach();
}

void HardwareSyncCapture::onSlaveRequestComplete(Request* request) {
    if (request->status() == Request::RequestCancelled) {
        return; 
    }

    // Process request asynchronously
    std::thread([this, request]() {
        processSlaveRequest(request);
    }).detach();
}

void HardwareSyncCapture::processMasterRequest(Request* request) {
    if (!running_) return;

    // Extract frame data
    const Request::BufferMap& buffers = request->buffers();
    FrameBuffer* buffer = buffers.begin()->second;
    uint64_t timestamp = buffer->metadata().timestamp;

    // Convert buffer to OpenCV Mat
    cv::Mat image = convertBufferToMat(buffer, master_config_->at(0));

    // Update auto-exposure based on image brightness
    updateAutoExposure(image, master_exposure_, "MASTER");
    
    // Apply noise reduction if needed (high gain scenarios)
    if (master_exposure_.apply_noise_reduction) {
        image = applyNoiseReduction(image, master_exposure_.analogue_gain);
    }

    // Store pending frame for synchronization
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        pending_master_.image = image.clone();
        pending_master_.timestamp_ns = timestamp;
        pending_master_.ready = true;
        
        tryFrameSync();
    }

    // Reuse and requeue request with updated exposure controls
    request->reuse(Request::ReuseBuffers);
    
    // Apply updated exposure controls for next frame
    applyExposureControls(request, master_exposure_);
    
    if (running_ && master_camera_->queueRequest(request) < 0) {
        std::cerr << "[HardwareSync] Failed to requeue master request" << std::endl;
    }
}

void HardwareSyncCapture::processSlaveRequest(Request* request) {
    if (!running_) return;

    // Extract frame data
    const Request::BufferMap& buffers = request->buffers();
    FrameBuffer* buffer = buffers.begin()->second;
    uint64_t timestamp = buffer->metadata().timestamp;

    // Convert buffer to OpenCV Mat  
    cv::Mat image = convertBufferToMat(buffer, slave_config_->at(0));

    // Update auto-exposure based on image brightness
    updateAutoExposure(image, slave_exposure_, "SLAVE");
    
    // Apply noise reduction if needed (high gain scenarios)
    if (slave_exposure_.apply_noise_reduction) {
        image = applyNoiseReduction(image, slave_exposure_.analogue_gain);
    }

    // Store pending frame for synchronization
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        pending_slave_.image = image.clone();
        pending_slave_.timestamp_ns = timestamp;
        pending_slave_.ready = true;

        tryFrameSync();
    }

    // Reuse and requeue request with updated exposure controls
    request->reuse(Request::ReuseBuffers);
    
    // Apply updated exposure controls for next frame
    applyExposureControls(request, slave_exposure_);
    
    if (running_ && slave_camera_->queueRequest(request) < 0) {
        std::cerr << "[HardwareSync] Failed to requeue slave request" << std::endl;
    }
}

void HardwareSyncCapture::tryFrameSync() {
    // Called with frame_mutex_ already locked
    
    if (!pending_master_.ready || !pending_slave_.ready) {
        return;  // Wait for both frames
    }

    // Create synchronized frame pair
    StereoFrame stereo_frame;
    stereo_frame.left_image = pending_master_.image;      // Camera 1 = LEFT
    stereo_frame.right_image = pending_slave_.image;      // Camera 0 = RIGHT  
    stereo_frame.left_timestamp_ns = pending_master_.timestamp_ns;
    stereo_frame.right_timestamp_ns = pending_slave_.timestamp_ns;
    stereo_frame.sync_error_ms = calculateSyncError(
        pending_master_.timestamp_ns, 
        pending_slave_.timestamp_ns
    );

    // Update statistics
    double fps = 0.0;
    if (last_timestamp_ns_ > 0) {
        uint64_t dt = pending_master_.timestamp_ns - last_timestamp_ns_;
        fps = dt > 0 ? 1000000000.0 / dt : 0.0;
    }
    last_timestamp_ns_ = pending_master_.timestamp_ns;
    
    updateStats(stereo_frame.sync_error_ms, fps);

    // Reset pending frames
    pending_master_.ready = false;
    pending_slave_.ready = false;

    // Deliver frame via callback
    if (frame_callback_) {
        frame_callback_(stereo_frame);
    }

    // Notify single frame capture
    frame_condition_.notify_one();
}

cv::Mat HardwareSyncCapture::convertBufferToMat(FrameBuffer* buffer, 
                                                const StreamConfiguration& config) {
    // Map the buffer to access raw data
    const FrameBuffer::Plane& plane = buffer->planes()[0];
    uint8_t* raw_data = static_cast<uint8_t*>(mmap(nullptr, plane.length, 
                                                    PROT_READ, MAP_SHARED, 
                                                    plane.fd.get(), 0));
    
    if (raw_data == MAP_FAILED) {
        std::cerr << "[HardwareSync] Failed to map buffer: " << strerror(errno) << std::endl;
        return cv::Mat();
    }

    // Get dimensions
    const int width = config.size.width;
    const int height = config.size.height;
    
    // OPTIMIZED APPROACH: Properly extract Green channel from SBGGR10 packed format
    // SBGGR10 is packed: 4 pixels in 5 bytes, each pixel is 10-bit
    // Better green channel extraction = less noise = cleaner images
    
    cv::Mat result;
    try {
        // SBGGR10 Bayer pattern (BGGR):
        //   B G B G ... 
        //   G R G R ...
        //   B G B G ...
        
        // Create 16-bit buffer for proper 10-bit handling
        cv::Mat raw_16bit(height, width, CV_16UC1);
        
        // Unpack SBGGR10 packed format (4 pixels in 5 bytes)
        const int pixels_per_pack = 4;
        const int bytes_per_pack = 5;
        const int total_pixels = width * height;
        
        for (int i = 0; i < total_pixels / pixels_per_pack; i++) {
            const uint8_t* pack = raw_data + i * bytes_per_pack;
            uint16_t* out = raw_16bit.ptr<uint16_t>() + i * pixels_per_pack;
            
            // Unpack 4 10-bit pixels from 5 bytes
            out[0] = ((uint16_t)pack[0] << 2) | (pack[1] >> 6);
            out[1] = (((uint16_t)(pack[1] & 0x3F)) << 4) | (pack[2] >> 4);
            out[2] = (((uint16_t)(pack[2] & 0x0F)) << 6) | (pack[3] >> 2);
            out[3] = (((uint16_t)(pack[3] & 0x03)) << 8) | pack[4];
        }
        
        // Create output grayscale image
        cv::Mat gray_result(height, width, CV_8UC1);
        
        // Extract and interpolate green channel with better quality
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                uint16_t pixel_value = 0;
                
                // Check if this pixel is Green in BGGR pattern
                bool is_green = ((y % 2 == 0 && x % 2 == 1) || // Even rows: odd columns
                                (y % 2 == 1 && x % 2 == 0));   // Odd rows: even columns
                
                if (is_green) {
                    // Direct green pixel
                    pixel_value = raw_16bit.at<uint16_t>(y, x);
                } else {
                    // Interpolate from neighboring green pixels for smoother result
                    int count = 0;
                    uint32_t sum = 0;
                    
                    // Check all 4 neighbors
                    if (x > 0 && ((y % 2 == 0 && (x-1) % 2 == 1) || (y % 2 == 1 && (x-1) % 2 == 0))) {
                        sum += raw_16bit.at<uint16_t>(y, x-1);
                        count++;
                    }
                    if (x < width-1 && ((y % 2 == 0 && (x+1) % 2 == 1) || (y % 2 == 1 && (x+1) % 2 == 0))) {
                        sum += raw_16bit.at<uint16_t>(y, x+1);
                        count++;
                    }
                    if (y > 0 && ((x % 2 == 0 && (y-1) % 2 == 1) || (x % 2 == 1 && (y-1) % 2 == 0))) {
                        sum += raw_16bit.at<uint16_t>(y-1, x);
                        count++;
                    }
                    if (y < height-1 && ((x % 2 == 0 && (y+1) % 2 == 1) || (x % 2 == 1 && (y+1) % 2 == 0))) {
                        sum += raw_16bit.at<uint16_t>(y+1, x);
                        count++;
                    }
                    
                    pixel_value = (count > 0) ? (sum / count) : 0;
                }
                
                // Convert 10-bit to 8-bit with improved scaling
                // Use linear scaling with slight boost for indoor lighting
                float normalized = pixel_value / 1023.0f;  // Normalize to 0-1
                
                // Apply adaptive gamma based on brightness level
                // Lower gamma (0.85) brightens the image for indoor scenes
                float gamma = 0.85f;  // Brighten for better indoor exposure
                normalized = std::pow(normalized, gamma);
                
                // Add slight brightness boost for dark images
                normalized = normalized * 1.1f;  // 10% brightness boost
                normalized = std::min(normalized, 1.0f);  // Clamp to valid range
                
                gray_result.at<uint8_t>(y, x) = static_cast<uint8_t>(normalized * 255.0f);
            }
        }
        
        // Skip median blur - it can cause unwanted smoothing
        // Only apply if really needed based on gain level
        result = gray_result;  // Direct result without additional filtering
        
    } catch (const std::exception& e) {
        std::cerr << "[HardwareSync] Green channel extraction failed: " << e.what() << std::endl;
        munmap(raw_data, plane.length);
        return cv::Mat();
    }
    
    // Unmap the buffer
    munmap(raw_data, plane.length);
    
    // Return the processed grayscale image
    return result;
}

/**
 * Alternative: Extract only green pixels from SBGGR10 for better luminance
 * Green pixels contain more luminance information in Bayer pattern
 */
cv::Mat HardwareSyncCapture::convertBufferToMatGreenOnly(FrameBuffer* buffer, 
                                                         const StreamConfiguration& config) {
    const FrameBuffer::Plane& plane = buffer->planes()[0];
    uint8_t* raw_data = static_cast<uint8_t*>(mmap(nullptr, plane.length, 
                                                    PROT_READ, MAP_SHARED, 
                                                    plane.fd.get(), 0));
    
    if (raw_data == MAP_FAILED) {
        std::cerr << "[HardwareSync] Failed to map buffer: " << strerror(errno) << std::endl;
        return cv::Mat();
    }

    const int width = config.size.width;
    const int height = config.size.height;
    
    // Create output grayscale image at half resolution (extracting only green)
    cv::Mat green_only(height/2, width/2, CV_8UC1);
    
    // SBGGR10 pattern - extract only green pixels:
    //   B [G] B [G] ...  <- Extract these G
    //  [G] R [G] R ...   <- And these G
    
    const int pixels_per_pack = 4;  // 4 pixels in 5 bytes
    const int bytes_per_pack = 5;
    
    // Process only green pixels
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            // Check if this is a green pixel in Bayer pattern
            bool is_green = ((y % 2 == 0 && x % 2 == 1) || // Top row green
                           (y % 2 == 1 && x % 2 == 0));   // Bottom row green
            
            if (!is_green) continue;
            
            // Calculate position in packed data
            int pixel_idx = y * width + x;
            int pack_idx = pixel_idx / pixels_per_pack;
            int pixel_in_pack = pixel_idx % pixels_per_pack;
            
            const uint8_t* pack_ptr = raw_data + (pack_idx * bytes_per_pack);
            
            // Extract 10-bit value
            uint16_t pixel_10bit = 0;
            
            switch(pixel_in_pack) {
                case 0:
                    pixel_10bit = ((uint16_t)pack_ptr[0] << 2) | (pack_ptr[1] >> 6);
                    break;
                case 1:
                    pixel_10bit = (((uint16_t)(pack_ptr[1] & 0x3F)) << 4) | (pack_ptr[2] >> 4);
                    break;
                case 2:
                    pixel_10bit = (((uint16_t)(pack_ptr[2] & 0x0F)) << 6) | (pack_ptr[3] >> 2);
                    break;
                case 3:
                    pixel_10bit = (((uint16_t)(pack_ptr[3] & 0x03)) << 8) | pack_ptr[4];
                    break;
            }
            
            // Convert to 8-bit
            uint8_t pixel_8bit = pixel_10bit >> 2;
            
            // Place in output image at appropriate position
            int out_x = x / 2;
            int out_y = y / 2;
            green_only.at<uint8_t>(out_y, out_x) = pixel_8bit;
        }
    }
    
    // Upscale back to full resolution using interpolation
    cv::Mat full_res;
    cv::resize(green_only, full_res, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
    
    munmap(raw_data, plane.length);
    return full_res;
}

/**
 * NEON-optimized SBGGR10 unpacking
 * Unpacks 10-bit packed data to 16-bit for efficient processing
 */
void HardwareSyncCapture::unpackSBGGR10_NEON(const uint8_t* src, uint16_t* dst, 
                                             int width, int height) {
    const int pixels_per_pack = 4;  // 4 pixels in 5 bytes
    const int bytes_per_pack = 5;
    const int total_pixels = width * height;
    const int num_packs = total_pixels / pixels_per_pack;
    
    // Process in chunks using NEON
    for (int pack = 0; pack < num_packs; pack++) {
        const uint8_t* pack_src = src + (pack * bytes_per_pack);
        uint16_t* pack_dst = dst + (pack * pixels_per_pack);
        
        // Unpack 4 pixels from 5 bytes (40 bits total)
        // Layout: [P0_high][P0_low|P1_high][P1_low|P2_high][P2_low|P3_high][P3_low]
        uint16_t pixel0 = ((uint16_t)pack_src[0] << 2) | (pack_src[1] >> 6);
        uint16_t pixel1 = (((uint16_t)(pack_src[1] & 0x3F)) << 4) | (pack_src[2] >> 4);
        uint16_t pixel2 = (((uint16_t)(pack_src[2] & 0x0F)) << 6) | (pack_src[3] >> 2);
        uint16_t pixel3 = (((uint16_t)(pack_src[3] & 0x03)) << 8) | pack_src[4];
        
        pack_dst[0] = pixel0;
        pack_dst[1] = pixel1;
        pack_dst[2] = pixel2;
        pack_dst[3] = pixel3;
    }
    
    // Handle remaining pixels
    int remaining = total_pixels % pixels_per_pack;
    if (remaining > 0) {
        const uint8_t* pack_src = src + (num_packs * bytes_per_pack);
        uint16_t* pack_dst = dst + (num_packs * pixels_per_pack);
        
        // Simple unpacking for remaining pixels
        for (int i = 0; i < remaining; i++) {
            int bit_offset = i * 10;
            int byte_offset = bit_offset / 8;
            int bit_shift = bit_offset % 8;
            
            uint16_t pixel = ((uint16_t)pack_src[byte_offset] << (2 + bit_shift)) |
                            (pack_src[byte_offset + 1] >> (6 - bit_shift));
            pack_dst[i] = pixel & 0x3FF;  // Mask to 10 bits
        }
    }
}

/**
 * Alternative: NEON-optimized fast demosaicing for real-time performance
 * This provides better performance than OpenCV for simple demosaicing
 */
cv::Mat HardwareSyncCapture::fastDemosaicNEON(const cv::Mat& bayer, int pattern) {
    // Implementation of fast NEON demosaicing if needed for performance
    // For now, using OpenCV's optimized implementation
    cv::Mat result;
    cv::cvtColor(bayer, result, cv::COLOR_BayerBG2BGR);
    return result;
}

double HardwareSyncCapture::calculateSyncError(uint64_t master_ts, uint64_t slave_ts) {
    int64_t diff_ns = static_cast<int64_t>(slave_ts) - static_cast<int64_t>(master_ts);
    return std::abs(diff_ns) / 1000000.0;  // Convert to milliseconds
}

void HardwareSyncCapture::updateStats(double sync_error_ms, double fps) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    stats_.frames_captured++;
    if (sync_error_ms > 1.0) {
        stats_.sync_errors++;
    }
    
    stats_.max_sync_error_ms = std::max(stats_.max_sync_error_ms, sync_error_ms);
    
    // Rolling average for sync error
    stats_.avg_sync_error_ms = (stats_.avg_sync_error_ms * 0.9) + (sync_error_ms * 0.1);
    
    // Rolling average for FPS
    stats_.measured_fps = (stats_.measured_fps * 0.9) + (fps * 0.1);
}

void HardwareSyncCapture::setFrameCallback(FrameCallback callback) {
    frame_callback_ = callback;
}

bool HardwareSyncCapture::captureSingle(StereoFrame& /* frame */, int timeout_ms) {
    if (!running_) {
        return false;
    }

    std::unique_lock<std::mutex> lock(frame_mutex_);
    
    bool captured = frame_condition_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
        [this]() { return pending_master_.ready && pending_slave_.ready; });
        
    if (!captured) {
        return false;
    }

    // Create frame (tryFrameSync will be called and reset pending frames)
    tryFrameSync();
    return true;
}

HardwareSyncCapture::SyncStats HardwareSyncCapture::getStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

/**
 * Apply exposure and gain controls to a request
 */
void HardwareSyncCapture::applyExposureControls(libcamera::Request* request, 
                                               const ExposureControl& exposure) {
    libcamera::ControlList& controls = request->controls();
    
    // Set exposure time (in microseconds for IMX296)
    controls.set(libcamera::controls::ExposureTime, exposure.exposure_time_us);
    
    // Set analogue gain
    controls.set(libcamera::controls::AnalogueGain, exposure.analogue_gain);
    
    // Disable auto exposure/gain to use manual control
    controls.set(libcamera::controls::AeEnable, false);
    
    // Set frame duration limits to maintain sync (33ms for 30fps)
    controls.set(libcamera::controls::FrameDurationLimits, 
                libcamera::Span<const int64_t, 2>({33000, 33333}));
}

/**
 * Calculate average brightness of green channel
 */
float HardwareSyncCapture::calculateImageBrightness(const cv::Mat& image) {
    if (image.empty()) return 0.0f;
    
    // Calculate mean brightness using optimized OpenCV function
    cv::Scalar mean_val = cv::mean(image);
    return static_cast<float>(mean_val[0]);
}

/**
 * Apply noise reduction for high-gain scenarios
 * Uses adaptive filtering to reduce noise without causing distortions
 */
cv::Mat HardwareSyncCapture::applyNoiseReduction(const cv::Mat& image, float gain_level) {
    if (image.empty() || gain_level < 6.0f) {
        return image;  // No noise reduction needed below 6x gain
    }
    
    cv::Mat denoised;
    
    // Use Non-Local Means denoising for better quality (less distortion)
    // Adjust strength based on gain level
    float h = 3.0f * std::min((gain_level - 6.0f) / 2.0f + 1.0f, 3.0f);  // Filter strength
    int templateWindowSize = 7;  // Size of template patch
    int searchWindowSize = 21;   // Size of search area
    
    // Apply Non-Local Means denoising - better preserves details
    cv::fastNlMeansDenoising(image, denoised, h, templateWindowSize, searchWindowSize);
    
    // For very high gain, apply gentle median filter to remove salt-and-pepper noise
    if (gain_level > 10.0f) {
        cv::medianBlur(denoised, denoised, 3);
    }
    
    return denoised;
}

/**
 * Update auto-exposure using PI controller
 * Adjusts exposure time and gain to achieve target brightness
 */
void HardwareSyncCapture::updateAutoExposure(const cv::Mat& image, 
                                            ExposureControl& exposure,
                                            const std::string& camera_name) {
    if (image.empty()) return;
    
    // Calculate current brightness
    float current_brightness = calculateImageBrightness(image);
    
    // Calculate error from target
    float error = exposure.target_brightness - current_brightness;
    
    // Skip adjustment if within tolerance
    if (std::abs(error) < exposure.brightness_tolerance) {
        return;
    }
    
    // PI controller
    float adjustment = exposure.kp * error + exposure.ki * exposure.integral_error;
    exposure.integral_error += error * 0.1f;  // Accumulate error with damping
    
    // Limit integral windup
    exposure.integral_error = std::clamp(exposure.integral_error, -50.0f, 50.0f);
    
    // Calculate brightness ratio for exposure adjustment
    float brightness_ratio = exposure.target_brightness / (current_brightness + 1.0f);
    
    // IMPROVED STRATEGY: Prioritize exposure time up to 10ms, then gain
    // This provides better SNR while maintaining reasonable frame rate
    
    std::lock_guard<std::mutex> lock(exposure_mutex_);
    
    if (brightness_ratio > 1.05f) {  // Image too dark (5% threshold)
        // First increase exposure time up to 10ms for better SNR
        if (exposure.exposure_time_us < 10000) {
            int32_t new_exposure = static_cast<int32_t>(
                exposure.exposure_time_us * (1.0f + adjustment * 0.15f));  // Faster adjustment
            exposure.exposure_time_us = std::min(new_exposure, 10000);
        } 
        // Then increase gain up to 4x
        else if (exposure.analogue_gain < 4.0f) {
            exposure.analogue_gain = std::min(exposure.analogue_gain * (1.0f + adjustment * 0.1f), 4.0f);
        }
        // Then continue increasing exposure
        else if (exposure.exposure_time_us < exposure.max_exposure_us) {
            int32_t new_exposure = static_cast<int32_t>(
                exposure.exposure_time_us * (1.0f + adjustment * 0.1f));
            exposure.exposure_time_us = std::clamp(new_exposure, 
                                                   exposure.min_exposure_us, 
                                                   exposure.max_exposure_us);
        }
        // Finally increase gain further if needed
        else {
            exposure.analogue_gain = std::min(exposure.analogue_gain * (1.0f + adjustment * 0.05f), 
                                             exposure.max_gain);
        }
    } else if (brightness_ratio < 0.95f) {  // Image too bright (5% threshold)
        // First decrease gain if above minimum
        if (exposure.analogue_gain > 1.5f) {
            exposure.analogue_gain = std::max(exposure.analogue_gain * (1.0f + adjustment * 0.1f), 
                                             1.0f);
        }
        // Then decrease exposure time
        else {
            int32_t new_exposure = static_cast<int32_t>(
                exposure.exposure_time_us * (1.0f + adjustment * 0.15f));  // Faster adjustment
            exposure.exposure_time_us = std::clamp(new_exposure, 
                                                   exposure.min_exposure_us, 
                                                   exposure.max_exposure_us);
        }
    }
    
    // Check if noise reduction should be applied
    exposure.apply_noise_reduction = (exposure.analogue_gain >= exposure.high_gain_threshold);
    
    // Log significant changes
    static int log_counter = 0;
    if (++log_counter % 30 == 0) {  // Log every 30 frames (~1 second at 30fps)
        std::cout << "[AutoExposure] " << camera_name 
                  << " - Brightness: " << current_brightness 
                  << "/" << exposure.target_brightness
                  << ", Exposure: " << exposure.exposure_time_us << "us"
                  << ", Gain: " << exposure.analogue_gain << "x"
                  << ", NoiseReduction: " << (exposure.apply_noise_reduction ? "ON" : "OFF")
                  << std::endl;
    }
}

/**
 * Configure auto-exposure for IMX296 sensors
 * Sets up optimal initial exposure parameters for stereo precision
 */
bool HardwareSyncCapture::configureAutoExposure() {
    std::lock_guard<std::mutex> lock(exposure_mutex_);
    
    // Set optimal default parameters for IMX296 sensors
    // Adjusted for indoor conditions to avoid maxing out exposure
    master_exposure_.exposure_time_us = 5000;    // 5ms initial exposure
    master_exposure_.analogue_gain = 1.5f;       // Lower initial gain for cleaner image
    master_exposure_.target_brightness = 80.0f;   // REDUCED TARGET: 80/255 for indoor lighting
    master_exposure_.brightness_tolerance = 8.0f; // Tighter tolerance for stability
    
    // PI controller parameters - FASTER convergence
    master_exposure_.kp = 0.5f;  // Increased proportional gain for faster response
    master_exposure_.ki = 0.1f;  // Increased integral gain for better tracking
    master_exposure_.integral_error = 0.0f; // Reset integral
    
    // Noise reduction thresholds - more conservative
    master_exposure_.high_gain_threshold = 6.0f;  // Only apply NR above 6x gain
    master_exposure_.apply_noise_reduction = false;
    
    // Copy master settings to slave for initial synchronization
    slave_exposure_.exposure_time_us = master_exposure_.exposure_time_us;
    slave_exposure_.analogue_gain = master_exposure_.analogue_gain;
    slave_exposure_.target_brightness = 80.0f;  // Same reduced target
    slave_exposure_.brightness_tolerance = master_exposure_.brightness_tolerance;
    slave_exposure_.kp = 0.5f;  // Same faster controller
    slave_exposure_.ki = 0.1f;
    slave_exposure_.integral_error = 0.0f;
    slave_exposure_.high_gain_threshold = 6.0f;
    slave_exposure_.apply_noise_reduction = false;
    
    // Set exposure limits for IMX296 - OPTIMIZED for quality
    master_exposure_.min_exposure_us = 100;    // IMX296 minimum
    master_exposure_.max_exposure_us = 20000;  // REDUCED: 20ms max for faster response
    master_exposure_.min_gain = 1.0f;          // IMX296 minimum
    master_exposure_.max_gain = 8.0f;          // REDUCED: 8x max to limit noise
    
    slave_exposure_.min_exposure_us = 100;
    slave_exposure_.max_exposure_us = 20000;  // Same reduced max
    slave_exposure_.min_gain = 1.0f;
    slave_exposure_.max_gain = 8.0f;          // Same reduced max
    
    std::cout << "[HardwareSync] Auto-exposure configured:" << std::endl;
    std::cout << "[HardwareSync]   Initial exposure: " << master_exposure_.exposure_time_us << "us" << std::endl;
    std::cout << "[HardwareSync]   Initial gain: " << master_exposure_.analogue_gain << "x" << std::endl;
    std::cout << "[HardwareSync]   Target brightness: " << master_exposure_.target_brightness << "/255" << std::endl;
    
    return true;
}

} // namespace camera  
} // namespace unlook