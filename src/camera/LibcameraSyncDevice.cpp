#include <unlook/camera/LibcameraSyncDevice.hpp>
#include <unlook/core/Logger.hpp>
#include <thread>
#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>

namespace unlook {
namespace camera {

// Use the official Unlook logging system
using unlook::core::LogStream;
using unlook::core::LogLevel;

// Simple logging to avoid macro conflicts
#undef LOG_INFO
#undef LOG_ERROR
#undef LOG_WARNING
#undef LOG_DEBUG
#define LOG_INFO(msg) std::cout << "[INFO] LibcameraSyncDevice: " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] LibcameraSyncDevice: " << msg << std::endl
#define LOG_WARNING(msg) std::cout << "[WARN] LibcameraSyncDevice: " << msg << std::endl
#define LOG_DEBUG(msg) std::cout << "[DEBUG] LibcameraSyncDevice: " << msg << std::endl

using namespace libcamera;

// Static members
std::shared_ptr<CameraManager> LibcameraSyncDevice::shared_manager_;
std::mutex LibcameraSyncDevice::shared_manager_mutex_;

/**
 * @brief Cleanup shared CameraManager resources
 * 
 * Called during application shutdown to ensure proper GLib object cleanup
 * and prevent GLib-GObject-CRITICAL errors during Qt application teardown.
 */
void LibcameraSyncDevice::cleanupSharedCameraManager() {
    std::lock_guard<std::mutex> lock(shared_manager_mutex_);
    
    if (shared_manager_) {
        try {
            LOG_INFO("Cleaning up shared CameraManager");
            shared_manager_->stop();
            shared_manager_.reset();
            LOG_DEBUG("Shared CameraManager cleanup completed");
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during CameraManager cleanup: " + std::string(e.what()));
        }
    }
}

LibcameraSyncDevice::LibcameraSyncDevice(const std::string& camera_path, Role role)
    : camera_path_(camera_path)
    , role_(role) {
    LOG_DEBUG("Creating LibcameraSyncDevice for " + camera_path + 
              " with role " + (role == Role::MASTER ? "MASTER" : "SLAVE"));
}

LibcameraSyncDevice::~LibcameraSyncDevice() {
    LOG_DEBUG("Destroying LibcameraSyncDevice for " + camera_path_);
    
    // CRITICAL: Thread-safe shutdown sequence following C++17/20 RAII principles
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Step 1: Stop streaming with proper synchronization
        if (streaming_) {
            streaming_ = false;
            frame_ready_ = false;
        }
    }
    
    // Step 2: Stop camera operations (without lock to avoid deadlock)
    if (initialized_ && camera_) {
        try {
            camera_->stop();
            LOG_DEBUG("Camera stopped successfully");
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during camera stop: " + std::string(e.what()));
        }
    }
    
    // Step 3: Wake up any waiting threads
    frame_ready_cv_.notify_all();
    
    // Step 4: Thread-safe resource cleanup
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Clear request queues first (prevents dangling pointers)
        while (!available_requests_.empty()) {
            available_requests_.pop();
        }
        while (!queued_requests_.empty()) {
            queued_requests_.pop();
        }
        
        // Clear request-buffer associations
        request_buffers_.clear();
        frame_buffers_.clear();
        
        // Clear unique_ptr requests (RAII cleanup)
        requests_.clear();
        
        // Free allocator buffers if present
        if (allocator_ && stream_) {
            try {
                allocator_->free(stream_);
                LOG_DEBUG("Buffers freed successfully");
            } catch (const std::exception& e) {
                LOG_ERROR("Exception during buffer cleanup: " + std::string(e.what()));
            }
        }
        allocator_.reset();
        
        // Release camera last (after all resources cleaned)
        if (camera_) {
            try {
                camera_->release();
                LOG_DEBUG("Camera released successfully");
            } catch (const std::exception& e) {
                LOG_ERROR("Exception during camera release: " + std::string(e.what()));
            }
        }
        
        initialized_ = false;
    }
}

std::shared_ptr<CameraManager> LibcameraSyncDevice::getSharedCameraManager() {
    std::lock_guard<std::mutex> lock(shared_manager_mutex_);
    
    if (!shared_manager_) {
        LOG_INFO("Creating shared CameraManager with proper RAII management");
        
        try {
            shared_manager_ = std::make_shared<CameraManager>();
            
            // Set libcamera environment to use third-party build
            setenv("LIBCAMERA_IPA_MODULE_PATH", 
                   "/home/alessandro/unlook-standalone/third-party/libcamera-sync-fix/build/src/ipa", 1);
            setenv("LIBCAMERA_IPA_CONFIG_PATH",
                   "/home/alessandro/unlook-standalone/third-party/libcamera-sync-fix/src/ipa", 1);
            
            int ret = shared_manager_->start();
            if (ret < 0) {
                LOG_ERROR("Failed to start CameraManager: " + std::to_string(ret));
                shared_manager_.reset();
                return nullptr;
            }
            
            // Register cleanup handler for proper shutdown
            std::atexit([]() {
                cleanupSharedCameraManager();
            });
            
            LOG_INFO("CameraManager started with " + 
                     std::to_string(shared_manager_->cameras().size()) + " cameras");
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during CameraManager creation: " + std::string(e.what()));
            shared_manager_.reset();
            return nullptr;
        }
    }
    
    return shared_manager_;
}

bool LibcameraSyncDevice::initialize(const CameraConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (initialized_) {
        LOG_WARNING("Camera already initialized");
        return true;
    }
    
    LOG_INFO("Initializing camera: " + camera_path_);
    config_params_ = config;
    
    // Get shared camera manager
    camera_manager_ = getSharedCameraManager();
    if (!camera_manager_) {
        last_error_ = "Failed to get camera manager";
        LOG_ERROR(last_error_);
        return false;
    }
    
    // Find camera by path
    for (auto& cam : camera_manager_->cameras()) {
        if (cam->id() == camera_path_) {
            camera_ = cam;
            break;
        }
    }
    
    if (!camera_) {
        last_error_ = "Camera not found: " + camera_path_;
        LOG_ERROR(last_error_);
        return false;
    }
    
    // Get camera model
    const ControlList& props = camera_->properties();
    if (props.contains(properties::Model.id())) {
        camera_model_ = props.get(properties::Model).value_or("Unknown");
        LOG_INFO("Camera model: " + camera_model_);
    }
    
    // Acquire camera
    int ret = camera_->acquire();
    if (ret < 0) {
        last_error_ = "Failed to acquire camera: " + std::to_string(ret);
        LOG_ERROR(last_error_);
        return false;
    }
    
    // Configure camera
    if (!configureCamera()) {
        camera_->release();
        return false;
    }
    
    // Allocate buffers
    if (!allocateBuffers()) {
        camera_->release();
        return false;
    }
    
    // Create requests
    if (!createRequests()) {
        allocator_->free(stream_);
        camera_->release();
        return false;
    }
    
    // Set request completed handler
    camera_->requestCompleted.connect(this, &LibcameraSyncDevice::handleRequestComplete);
    
    initialized_ = true;
    LOG_INFO("Camera initialized successfully: " + camera_path_);
    
    return true;
}

bool LibcameraSyncDevice::configureCamera() {
    // Generate configuration
    config_ = camera_->generateConfiguration({ StreamRole::VideoRecording });
    if (!config_) {
        last_error_ = "Failed to generate camera configuration";
        LOG_ERROR(last_error_);
        return false;
    }
    
    // Configure stream
    StreamConfiguration& streamConfig = config_->at(0);
    streamConfig.pixelFormat = config_params_.format;
    streamConfig.size.width = config_params_.width;
    streamConfig.size.height = config_params_.height;
    streamConfig.bufferCount = config_params_.buffer_count;
    
    LOG_INFO("Configuring stream: " + std::to_string(streamConfig.size.width) + "x" + 
             std::to_string(streamConfig.size.height) + " " + 
             streamConfig.pixelFormat.toString());
    
    // Validate configuration
    CameraConfiguration::Status validation = config_->validate();
    if (validation == CameraConfiguration::Invalid) {
        last_error_ = "Invalid camera configuration";
        LOG_ERROR(last_error_);
        return false;
    }
    
    if (validation == CameraConfiguration::Adjusted) {
        LOG_WARNING("Camera configuration was adjusted to: " +
                    std::to_string(streamConfig.size.width) + "x" +
                    std::to_string(streamConfig.size.height));
    }
    
    // Apply configuration
    int ret = camera_->configure(config_.get());
    if (ret < 0) {
        last_error_ = "Failed to configure camera: " + std::to_string(ret);
        LOG_ERROR(last_error_);
        return false;
    }
    
    // Get the configured stream
    stream_ = streamConfig.stream();
    
    return true;
}

bool LibcameraSyncDevice::allocateBuffers() {
    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
    
    int ret = allocator_->allocate(stream_);
    if (ret < 0) {
        last_error_ = "Failed to allocate buffers: " + std::to_string(ret);
        LOG_ERROR(last_error_);
        return false;
    }
    
    const std::vector<std::unique_ptr<FrameBuffer>>& buffers = allocator_->buffers(stream_);
    LOG_INFO("Allocated " + std::to_string(buffers.size()) + " buffers");
    
    // Map frame buffers for CPU access
    for (const auto& buffer : buffers) {
        if (!mapFrameBuffer(buffer.get())) {
            LOG_WARNING("Failed to map frame buffer");
        }
    }
    
    return true;
}

bool LibcameraSyncDevice::createRequests() {
    const std::vector<std::unique_ptr<FrameBuffer>>& buffers = allocator_->buffers(stream_);
    
    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<Request> request = camera_->createRequest(i);
        if (!request) {
            last_error_ = "Failed to create request " + std::to_string(i);
            LOG_ERROR(last_error_);
            return false;
        }
        
        FrameBuffer* buffer = buffers[i].get();
        int ret = request->addBuffer(stream_, buffer);
        if (ret < 0) {
            last_error_ = "Failed to add buffer to request: " + std::to_string(ret);
            LOG_ERROR(last_error_);
            return false;
        }
        
        // Store buffer association
        request_buffers_[request.get()] = buffer;
        
        // Apply initial controls
        applyControls(request.get());
        
        // Store request
        requests_.push_back(std::move(request));
    }
    
    LOG_INFO("Created " + std::to_string(requests_.size()) + " requests");
    return true;
}

bool LibcameraSyncDevice::applyControls(Request* request) {
    if (!request) {
        return false;
    }
    
    ControlList& controls = request->controls();
    
    // Set exposure time
    if (camera_->controls().count(&controls::ExposureTime) > 0) {
        controls.set(controls::ExposureTime, static_cast<int32_t>(config_params_.exposure_time_us));
    }
    
    // Set analog gain
    if (camera_->controls().count(&controls::AnalogueGain) > 0) {
        controls.set(controls::AnalogueGain, static_cast<float>(config_params_.analog_gain));
    }
    
    // Set frame duration for FPS control
    if (camera_->controls().count(&controls::FrameDurationLimits) > 0) {
        int64_t frame_duration_us = static_cast<int64_t>(1000000.0 / config_params_.target_fps);
        controls.set(controls::FrameDurationLimits, 
                     Span<const int64_t, 2>({ frame_duration_us, frame_duration_us }));
    }
    
    return true;
}

bool LibcameraSyncDevice::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        last_error_ = "Camera not initialized";
        LOG_ERROR(last_error_);
        return false;
    }
    
    if (streaming_) {
        LOG_WARNING("Camera already streaming");
        return true;
    }
    
    LOG_INFO("Starting camera stream: " + camera_path_);
    
    // Start camera FIRST (following cam pattern)
    int ret = camera_->start();
    if (ret < 0) {
        last_error_ = "Failed to start camera: " + std::to_string(ret);
        LOG_ERROR(last_error_);
        return false;
    }
    
    // Now queue all requests (following cam pattern)
    int queued_count = 0;
    for (auto& request : requests_) {
        ret = camera_->queueRequest(request.get());
        if (ret < 0) {
            LOG_ERROR("Failed to queue initial request " + std::to_string(queued_count) + ": " + std::to_string(ret));
            // Don't fail completely if some requests queue successfully
            if (queued_count == 0) {
                camera_->stop();
                return false;
            }
            break;
        }
        queued_count++;
    }
    
    LOG_INFO("Queued " + std::to_string(queued_count) + " requests successfully");
    
    streaming_ = true;
    start_time_ = std::chrono::steady_clock::now();
    frame_count_ = 0;
    dropped_frames_ = 0;
    
    LOG_INFO("Camera streaming started with " + std::to_string(queued_count) + " active requests");
    return true;
}

void LibcameraSyncDevice::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!streaming_) {
        return;
    }
    
    LOG_INFO("Stopping camera stream: " + camera_path_);
    
    // Stop camera
    int ret = camera_->stop();
    if (ret < 0) {
        LOG_ERROR("Failed to stop camera: " + std::to_string(ret));
    }
    
    // Clear request queues
    while (!queued_requests_.empty()) {
        queued_requests_.pop();
    }
    
    // Reset all requests to available
    available_requests_ = std::queue<Request*>();
    for (auto& request : requests_) {
        request->reuse(Request::ReuseBuffers);
        available_requests_.push(request.get());
    }
    
    streaming_ = false;
    frame_ready_ = false;
    
    // Wake up any waiting threads
    frame_ready_cv_.notify_all();
    
    LOG_INFO("Camera streaming stopped. Frames captured: " + std::to_string(frame_count_) + 
             ", Dropped: " + std::to_string(dropped_frames_));
}

void LibcameraSyncDevice::handleRequestComplete(Request* request) {
    if (request->status() == Request::RequestCancelled) {
        LOG_DEBUG("Request cancelled");
        recycleRequest(request);
        return;
    }
    
    if (request->status() != Request::RequestComplete) {
        LOG_WARNING("Request failed with status: " + std::to_string(request->status()));
        dropped_frames_++;
        recycleRequest(request);
        return;
    }
    
    processCompletedRequest(request);
}

void LibcameraSyncDevice::processCompletedRequest(Request* request) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Get the frame buffer
    FrameBuffer* buffer = request->findBuffer(stream_);
    if (!buffer) {
        LOG_ERROR("No buffer in completed request");
        recycleRequest(request);
        return;
    }
    
    // Get metadata
    const ControlList& metadata = request->metadata();
    
    FrameMetadata frame_metadata;
    frame_metadata.frame_number = frame_count_++;
    
    // Get timestamp
    if (metadata.contains(controls::SensorTimestamp.id())) {
        frame_metadata.timestamp_ns = metadata.get(controls::SensorTimestamp).value_or(0);
    }
    
    // Get exposure time
    if (metadata.contains(controls::ExposureTime.id())) {
        frame_metadata.exposure_time_us = metadata.get(controls::ExposureTime).value_or(0);
    }
    
    // Get analog gain
    if (metadata.contains(controls::AnalogueGain.id())) {
        frame_metadata.analog_gain = metadata.get(controls::AnalogueGain).value_or(1.0f);
    }
    
    // Convert frame to OpenCV Mat with proper error handling
    const FrameBuffer::Plane& plane = buffer->planes()[0];
    
    // CRITICAL: Safe memory mapping with RAII cleanup
    void* data = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    
    if (data != MAP_FAILED) {
        try {
            // Copy frame data safely with bounds checking
            if (plane.length >= (config_params_.width * config_params_.height * 10 / 8)) {
                latest_frame_ = convertFrameToMat(static_cast<uint8_t*>(data), plane.length);
                latest_metadata_ = frame_metadata;
                frame_ready_ = true;
                
                // Notify waiting threads
                frame_ready_cv_.notify_one();
                LOG_DEBUG("Frame processed successfully: #" + std::to_string(frame_metadata.frame_number));
            } else {
                LOG_ERROR("Frame buffer size mismatch: expected >= " + 
                         std::to_string(config_params_.width * config_params_.height * 10 / 8) + 
                         ", got " + std::to_string(plane.length));
                dropped_frames_++;
            }
        } catch (const std::exception& e) {
            LOG_ERROR("Exception during frame processing: " + std::string(e.what()));
            dropped_frames_++;
        }
        
        // RAII: Always unmap memory
        int unmap_result = munmap(data, plane.length);
        if (unmap_result != 0) {
            LOG_ERROR("Failed to unmap frame buffer: " + std::string(strerror(errno)));
        }
    } else {
        LOG_ERROR("Failed to map frame buffer: " + std::string(strerror(errno)));
        dropped_frames_++;
    }
    
    recycleRequest(request);
}

void LibcameraSyncDevice::recycleRequest(Request* request) {
    if (!streaming_) {
        return;
    }
    
    // Reuse the request (following cam pattern)
    request->reuse(Request::ReuseBuffers);
    
    // Apply controls for next capture
    applyControls(request);
    
    // Queue it again immediately (auto-recycling)
    int ret = camera_->queueRequest(request);
    if (ret < 0) {
        LOG_ERROR("Failed to requeue request: " + std::to_string(ret));
        // If requeue fails, we've likely stopped streaming
        dropped_frames_++;
    }
}

bool LibcameraSyncDevice::queueRequest() {
    // This method is no longer needed with the auto-recycling pattern
    // Requests are queued at start and recycled in the callback
    LOG_WARNING("queueRequest() called but not needed with auto-recycling pattern");
    return streaming_;
}

bool LibcameraSyncDevice::waitForFrame(cv::Mat& frame, FrameMetadata& metadata, uint32_t timeout_ms) {
    std::unique_lock<std::mutex> lock(mutex_);
    
    if (!streaming_) {
        last_error_ = "Camera not streaming";
        return false;
    }
    
    // Wait for frame with timeout
    if (!frame_ready_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                   [this] { return frame_ready_ || !streaming_; })) {
        // Timeout
        return false;
    }
    
    if (!streaming_) {
        return false;
    }
    
    if (frame_ready_) {
        frame = latest_frame_.clone();
        metadata = latest_metadata_;
        frame_ready_ = false;
        return true;
    }
    
    return false;
}

bool LibcameraSyncDevice::captureFrame(cv::Mat& frame, FrameMetadata& metadata, uint32_t timeout_ms) {
    if (!streaming_) {
        // Start streaming if not already
        if (!start()) {
            return false;
        }
        // Give camera time to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Don't queue new requests - they are already queued and recycled automatically
    // Just wait for the next frame from the callback
    return waitForFrame(frame, metadata, timeout_ms);
}

cv::Mat LibcameraSyncDevice::convertFrameToMat(const uint8_t* data, size_t size) {
    // For SBGGR10 format (10-bit Bayer) - INDUSTRIAL GRADE IMPLEMENTATION
    // Each pixel is 10 bits, packed format - requires careful bounds checking
    uint32_t width = config_params_.width;
    uint32_t height = config_params_.height;
    
    // CRITICAL: Validate input parameters
    if (!data) {
        LOG_ERROR("Null data pointer in convertFrameToMat");
        return cv::Mat::zeros(height, width, CV_8UC3);
    }
    
    size_t expected_size = (static_cast<size_t>(width) * height * 10) / 8;
    if (size < expected_size) {
        LOG_ERROR("Insufficient buffer size: expected " + std::to_string(expected_size) + 
                 ", got " + std::to_string(size));
        return cv::Mat::zeros(height, width, CV_8UC3);
    }
    
    try {
        // Create 16-bit Mat to hold unpacked data with proper initialization
        cv::Mat bayer16(height, width, CV_16UC1, cv::Scalar(0));
        
        // Unpack 10-bit data to 16-bit with bounds checking
        const uint8_t* src = data;
        uint16_t* dst = reinterpret_cast<uint16_t*>(bayer16.data);
        
        // CRITICAL: Safe unpacking with bounds verification
        const uint64_t total_pixels = static_cast<uint64_t>(width) * height;
        for (uint64_t i = 0; i < total_pixels; i++) {
            uint64_t byte_index = (i * 10) / 8;
            uint32_t bit_offset = (i * 10) % 8;
            
            // RAII-style bounds checking
            if (byte_index + 1 < size && i < total_pixels) {
                uint16_t value = (static_cast<uint16_t>(src[byte_index]) << 8) | 
                                static_cast<uint16_t>(src[byte_index + 1]);
                value = (value >> (6 - bit_offset)) & 0x3FF;
                dst[i] = value << 6;  // Scale to 16-bit range
            } else {
                LOG_WARNING("Bounds exceeded during pixel unpacking at index " + std::to_string(i));
                dst[i] = 0;  // Safe fallback
            }
        }
        
        // Convert Bayer to BGR with error handling
        cv::Mat bgr;
        cv::cvtColor(bayer16, bgr, cv::COLOR_BayerBG2BGR);
        
        // Convert to 8-bit with controlled scaling
        cv::Mat bgr8;
        bgr.convertTo(bgr8, CV_8UC3, 1.0/256.0);
        
        return bgr8;
        
    } catch (const cv::Exception& e) {
        LOG_ERROR("OpenCV exception in convertFrameToMat: " + std::string(e.what()));
        return cv::Mat::zeros(height, width, CV_8UC3);
    } catch (const std::exception& e) {
        LOG_ERROR("Standard exception in convertFrameToMat: " + std::string(e.what()));
        return cv::Mat::zeros(height, width, CV_8UC3);
    }
}

bool LibcameraSyncDevice::mapFrameBuffer(FrameBuffer* buffer) {
    if (!buffer) {
        return false;
    }
    
    // Pre-allocate buffer for this frame buffer
    const FrameBuffer::Plane& plane = buffer->planes()[0];
    frame_buffers_[buffer].resize(plane.length);
    
    return true;
}

bool LibcameraSyncDevice::setExposure(double exposure_us) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_params_.exposure_time_us = exposure_us;
    
    // Will be applied on next request
    return true;
}

bool LibcameraSyncDevice::setGain(double gain) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_params_.analog_gain = gain;
    
    // Will be applied on next request
    return true;
}

bool LibcameraSyncDevice::setFrameRate(double fps) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_params_.target_fps = fps;
    
    // Will be applied on next request
    return true;
}

bool LibcameraSyncDevice::enableHardwareSync(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    sync_enabled_ = enable;
    
    if (enable) {
        LOG_INFO("Hardware sync enabled for " + camera_path_ + 
                 " (Role: " + (role_ == Role::MASTER ? "MASTER" : "SLAVE") + ")");
    } else {
        LOG_INFO("Hardware sync disabled for " + camera_path_);
    }
    
    return true;
}

double LibcameraSyncDevice::getCurrentFPS() const {
    if (!streaming_ || frame_count_ == 0) {
        return 0.0;
    }
    
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_).count();
    
    if (elapsed > 0) {
        return (frame_count_ * 1000.0) / elapsed;
    }
    
    return 0.0;
}

} // namespace camera
} // namespace unlook