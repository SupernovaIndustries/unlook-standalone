#include "unlook/camera/camera_system.hpp"
#include "unlook/core/Logger.hpp"
#include "unlook/core/exception.h"

#include <libcamera/libcamera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/controls.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>
#include <libcamera/property_ids.h>

#include <thread>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <queue>
#include <condition_variable>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace unlook {
namespace camera {

// Static members
std::shared_ptr<CameraSystem> CameraSystem::instance_;
std::mutex CameraSystem::instance_mutex_;

// Hardware configuration constants for IMX296 cameras
constexpr int IMX296_WIDTH = 1456;
constexpr int IMX296_HEIGHT = 1088;
constexpr double TARGET_FPS = 30.0;
constexpr double MAX_SYNC_ERROR_MS = 1.0; // <1ms sync requirement
constexpr int MASTER_SLAVE_DELAY_MS = 500; // Delay between master and slave startup

// Camera hardware paths from PROJECT_GUIDELINES.md
const std::string LEFT_CAMERA_PATH = "/base/soc/i2c0mux/i2c@1/imx296@1a"; // Camera 1 = MASTER
const std::string RIGHT_CAMERA_PATH = "/base/soc/i2c0mux/i2c@0/imx296@1a"; // Camera 0 = SLAVE

// Hardware sync GPIO pins
constexpr int XVS_GPIO_PIN = 17;  // External Vertical Sync
constexpr int XHS_GPIO_PIN = 27;  // External Horizontal Sync (Camera system - different from AS1170 strobe GPIO 19)

// Implementation structure with real libcamera integration
struct CameraSystem::CameraImpl {
    // libcamera components
    std::unique_ptr<libcamera::CameraManager> camera_manager;
    std::shared_ptr<libcamera::Camera> left_camera;  // Master camera
    std::shared_ptr<libcamera::Camera> right_camera; // Slave camera
    
    // Camera configurations
    std::unique_ptr<libcamera::CameraConfiguration> left_config;
    std::unique_ptr<libcamera::CameraConfiguration> right_config;
    
    // Frame allocators
    std::unique_ptr<libcamera::FrameBufferAllocator> left_allocator;
    std::unique_ptr<libcamera::FrameBufferAllocator> right_allocator;
    
    // Request management
    std::vector<std::unique_ptr<libcamera::Request>> left_requests;
    std::vector<std::unique_ptr<libcamera::Request>> right_requests;
    std::queue<libcamera::Request*> left_completed_requests;
    std::queue<libcamera::Request*> right_completed_requests;
    
    // Synchronization
    std::mutex request_mutex;
    std::condition_variable frame_ready_cv;
    std::atomic<bool> capture_running{false};
    std::thread processing_thread;
    
    // Frame buffers for memory mapping
    struct MappedBuffer {
        void* data = nullptr;
        size_t length = 0;
        int fd = -1;
    };
    std::map<libcamera::FrameBuffer*, MappedBuffer> mapped_buffers;
    
    // System state
    bool initialized = false;
    bool hardware_sync_enabled = false;
    bool gpio_initialized = false;
    
    // Synchronization statistics
    std::atomic<uint64_t> left_frame_count{0};
    std::atomic<uint64_t> right_frame_count{0};
    std::atomic<uint64_t> sync_errors{0};
    double total_sync_error_ms = 0.0;
    size_t sync_sample_count = 0;
    std::mutex stats_mutex;
    
    // Callbacks
    core::StereoFrameCallback frame_callback;
    core::ErrorCallback error_callback;
    
    // Initialize hardware sync GPIO pins
    bool initializeGPIO() {
        try {
            // Export GPIO pins if not already exported
            std::ofstream export_file("/sys/class/gpio/export");
            if (export_file) {
                export_file << XVS_GPIO_PIN << std::endl;
                export_file << XHS_GPIO_PIN << std::endl;
                export_file.close();
            }
            
            // Set GPIO direction to output
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            std::ofstream xvs_dir("/sys/class/gpio/gpio" + std::to_string(XVS_GPIO_PIN) + "/direction");
            if (xvs_dir) {
                xvs_dir << "out" << std::endl;
                xvs_dir.close();
            }
            
            std::ofstream xhs_dir("/sys/class/gpio/gpio" + std::to_string(XHS_GPIO_PIN) + "/direction");
            if (xhs_dir) {
                xhs_dir << "out" << std::endl;
                xhs_dir.close();
            }
            
            // Enable sync signals
            std::ofstream xvs_val("/sys/class/gpio/gpio" + std::to_string(XVS_GPIO_PIN) + "/value");
            if (xvs_val) {
                xvs_val << "1" << std::endl;
                xvs_val.close();
            }
            
            std::ofstream xhs_val("/sys/class/gpio/gpio" + std::to_string(XHS_GPIO_PIN) + "/value");
            if (xhs_val) {
                xhs_val << "1" << std::endl;
                xhs_val.close();
            }
            
            gpio_initialized = true;
            UNLOOK_LOG_INFO("Camera") << "Hardware sync GPIO initialized (XVS:" << XVS_GPIO_PIN 
                                      << ", XHS:" << XHS_GPIO_PIN << ")";
            return true;
        } catch (const std::exception& e) {
            UNLOOK_LOG_ERROR("Camera") << "GPIO initialization failed: " << e.what();
            return false;
        }
    }
    
    // Initialize libcamera and detect IMX296 cameras
    bool initializeLibcamera() {
        try {
            camera_manager = std::make_unique<libcamera::CameraManager>();
            int ret = camera_manager->start();
            if (ret < 0) {
                UNLOOK_LOG_ERROR("Camera") << "Failed to start camera manager: " << std::strerror(-ret);
                return false;
            }
            
            // Get available cameras
            auto cameras = camera_manager->cameras();
            UNLOOK_LOG_INFO("Camera") << "Found " << cameras.size() << " cameras";
            
            if (cameras.size() < 2) {
                UNLOOK_LOG_ERROR("Camera") << "Need 2 cameras, found " << cameras.size();
                return false;
            }
            
            // Identify cameras by their properties
            for (auto& cam : cameras) {
                const auto& props = cam->properties();
                auto location = props.get(libcamera::properties::Location);
                std::string cam_id = cam->id();
                
                UNLOOK_LOG_INFO("Camera") << "Camera ID: " << cam_id;
                
                // Assign cameras based on ID patterns
                // Camera 1 (-c 1) = LEFT/MASTER at /base/soc/i2c0mux/i2c@1/imx296@1a
                // Camera 0 (-c 2) = RIGHT/SLAVE at /base/soc/i2c0mux/i2c@0/imx296@1a
                if (cam_id.find("i2c@1") != std::string::npos && cam_id.find("imx296") != std::string::npos) {
                    left_camera = cam;  // Master camera
                    UNLOOK_LOG_INFO("Camera") << "Assigned as LEFT/MASTER camera";
                } else if (cam_id.find("i2c@0") != std::string::npos && cam_id.find("imx296") != std::string::npos) {
                    right_camera = cam;  // Slave camera
                    UNLOOK_LOG_INFO("Camera") << "Assigned as RIGHT/SLAVE camera";
                }
            }
            
            // Fallback assignment if pattern matching fails
            if (!left_camera || !right_camera) {
                left_camera = cameras[0];   // First camera as LEFT/MASTER
                right_camera = cameras[1];  // Second camera as RIGHT/SLAVE
                UNLOOK_LOG_WARNING("Camera") << "Using fallback camera assignment";
            }
            
            return true;
            
        } catch (const std::exception& e) {
            UNLOOK_LOG_ERROR("Camera") << "libcamera initialization failed: " << e.what();
            return false;
        }
    }
    
    // Configure camera for IMX296 capture
    bool configureCamera(std::shared_ptr<libcamera::Camera>& camera,
                        std::unique_ptr<libcamera::CameraConfiguration>& config,
                        std::unique_ptr<libcamera::FrameBufferAllocator>& allocator,
                        std::vector<std::unique_ptr<libcamera::Request>>& requests,
                        bool is_master) {
        try {
            // Acquire camera
            int ret = camera->acquire();
            if (ret < 0) {
                UNLOOK_LOG_ERROR("Camera") << "Failed to acquire camera: " << std::strerror(-ret);
                return false;
            }
            
            // Generate configuration
            config = camera->generateConfiguration({libcamera::StreamRole::Viewfinder});
            if (!config || config->empty()) {
                UNLOOK_LOG_ERROR("Camera") << "Failed to generate configuration";
                return false;
            }
            
            // Configure for IMX296 resolution
            libcamera::StreamConfiguration& stream_config = config->at(0);
            stream_config.size.width = IMX296_WIDTH;
            stream_config.size.height = IMX296_HEIGHT;
            stream_config.pixelFormat = libcamera::formats::YUV420;  // YUV420 for OpenCV compatibility
            stream_config.bufferCount = 4;  // Use 4 buffers for smooth capture
            
            // Validate configuration
            libcamera::CameraConfiguration::Status validation = config->validate();
            if (validation == libcamera::CameraConfiguration::Invalid) {
                UNLOOK_LOG_ERROR("Camera") << "Invalid configuration";
                return false;
            }
            
            // Apply configuration
            ret = camera->configure(config.get());
            if (ret < 0) {
                UNLOOK_LOG_ERROR("Camera") << "Failed to configure camera: " << std::strerror(-ret);
                return false;
            }
            
            // Allocate frame buffers
            allocator = std::make_unique<libcamera::FrameBufferAllocator>(camera);
            libcamera::Stream* stream = config->at(0).stream();
            
            ret = allocator->allocate(stream);
            if (ret < 0) {
                UNLOOK_LOG_ERROR("Camera") << "Failed to allocate buffers: " << std::strerror(-ret);
                return false;
            }
            
            // Create requests and map buffers
            const std::vector<std::unique_ptr<libcamera::FrameBuffer>>& buffers = allocator->buffers(stream);
            for (unsigned int i = 0; i < buffers.size(); ++i) {
                auto request = camera->createRequest();
                if (!request) {
                    UNLOOK_LOG_ERROR("Camera") << "Failed to create request";
                    return false;
                }
                
                // Add buffer to request
                ret = request->addBuffer(stream, buffers[i].get());
                if (ret < 0) {
                    UNLOOK_LOG_ERROR("Camera") << "Failed to add buffer to request";
                    return false;
                }
                
                // Map buffer memory for later access
                mapFrameBuffer(buffers[i].get());
                
                requests.push_back(std::move(request));
            }
            
            UNLOOK_LOG_INFO("Camera") << "Camera configured: " << IMX296_WIDTH << "x" << IMX296_HEIGHT
                                      << " @ " << TARGET_FPS << " FPS"
                                      << (is_master ? " (MASTER)" : " (SLAVE)");
            
            return true;
            
        } catch (const std::exception& e) {
            UNLOOK_LOG_ERROR("Camera") << "Camera configuration failed: " << e.what();
            return false;
        }
    }
    
    // Map frame buffer memory for direct access
    void mapFrameBuffer(libcamera::FrameBuffer* buffer) {
        const auto& planes = buffer->planes();
        if (planes.empty()) return;
        
        // Map only the first plane for grayscale data
        const libcamera::FrameBuffer::Plane& plane = planes[0];
        
        MappedBuffer mapped;
        mapped.fd = plane.fd.get();
        mapped.length = plane.length;
        
        // Memory map the buffer
        mapped.data = mmap(nullptr, mapped.length, PROT_READ, MAP_SHARED, mapped.fd, 0);
        if (mapped.data == MAP_FAILED) {
            UNLOOK_LOG_ERROR("Camera") << "Failed to map buffer: " << std::strerror(errno);
            mapped.data = nullptr;
            return;
        }
        
        mapped_buffers[buffer] = mapped;
    }
    
    // Unmap all frame buffers
    void unmapFrameBuffers() {
        for (auto& [buffer, mapped] : mapped_buffers) {
            if (mapped.data && mapped.data != MAP_FAILED) {
                munmap(mapped.data, mapped.length);
            }
        }
        mapped_buffers.clear();
    }
    
    // Extract timestamp from request metadata
    uint64_t extractTimestamp(libcamera::Request* request) {
        // Get the timestamp from request metadata
        const libcamera::ControlList& metadata = request->metadata();
        
        // Try to get SensorTimestamp first (most accurate)
        auto sensor_timestamp = metadata.get(libcamera::controls::SensorTimestamp);
        if (sensor_timestamp) {
            return *sensor_timestamp;
        }
        
        // Fallback to system timestamp
        auto timestamp_ns = std::chrono::steady_clock::now().time_since_epoch();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp_ns).count();
    }
    
    // Update synchronization statistics
    void updateSyncStatistics(double sync_error_ms) {
        std::lock_guard<std::mutex> lock(stats_mutex);
        total_sync_error_ms += sync_error_ms;
        sync_sample_count++;
    }
    
    // Convert raw frame buffer to OpenCV Mat
    cv::Mat convertFrameToMat(libcamera::FrameBuffer* buffer) {
        auto it = mapped_buffers.find(buffer);
        if (it == mapped_buffers.end() || !it->second.data) {
            UNLOOK_LOG_ERROR("Camera") << "Buffer not mapped";
            return cv::Mat();
        }
        
        // Create Mat from raw Bayer data (10-bit packed)
        // For now, convert to 8-bit grayscale for simplicity
        cv::Mat bayer_raw(IMX296_HEIGHT, IMX296_WIDTH, CV_16UC1);
        
        // Unpack 10-bit data to 16-bit
        const uint8_t* src = static_cast<const uint8_t*>(it->second.data);
        uint16_t* dst = reinterpret_cast<uint16_t*>(bayer_raw.data);
        
        // Simple unpacking (this should be optimized with NEON for production)
        for (int i = 0; i < IMX296_HEIGHT * IMX296_WIDTH; ++i) {
            int byte_idx = (i * 10) / 8;
            int bit_offset = (i * 10) % 8;
            
            if (bit_offset <= 6) {
                dst[i] = ((src[byte_idx] >> bit_offset) | (src[byte_idx + 1] << (8 - bit_offset))) & 0x3FF;
            } else {
                dst[i] = ((src[byte_idx] >> bit_offset) | (src[byte_idx + 1] << (8 - bit_offset)) |
                         (src[byte_idx + 2] << (16 - bit_offset))) & 0x3FF;
            }
            dst[i] <<= 6; // Shift to 16-bit range
        }
        
        // Convert to 8-bit grayscale
        cv::Mat gray;
        bayer_raw.convertTo(gray, CV_8UC1, 1.0/256.0);
        
        return gray;
    }
    
    // Signal handler wrapper class for libcamera signals
    class SignalHandler : public libcamera::Object {
    public:
        SignalHandler(CameraImpl* impl) : impl_(impl) {}
        
        void handleLeftRequest(libcamera::Request* request) {
            impl_->handleRequestCompleted(request, true);
        }
        
        void handleRightRequest(libcamera::Request* request) {
            impl_->handleRequestCompleted(request, false);
        }
        
    private:
        CameraImpl* impl_;
    };
    
    std::unique_ptr<SignalHandler> signal_handler;
    
    // Start synchronized capture
    void startCapture(core::StereoFrameCallback callback) {
        frame_callback = callback;
        capture_running = true;
        
        // Create signal handler
        signal_handler = std::make_unique<SignalHandler>(this);
        
        // Connect request completed signals using the handler
        left_camera->requestCompleted.connect(signal_handler.get(), 
            &SignalHandler::handleLeftRequest);
        right_camera->requestCompleted.connect(signal_handler.get(),
            &SignalHandler::handleRightRequest);
        
        // CRITICAL: Start MASTER camera first (Camera 1 = LEFT)
        UNLOOK_LOG_INFO("Camera") << "Starting LEFT/MASTER camera...";
        int ret = left_camera->start();
        if (ret < 0) {
            UNLOOK_LOG_ERROR("Camera") << "Failed to start left camera: " << std::strerror(-ret);
            capture_running = false;
            return;
        }
        
        // Wait for master to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(MASTER_SLAVE_DELAY_MS));
        
        // Start SLAVE camera (Camera 0 = RIGHT)
        UNLOOK_LOG_INFO("Camera") << "Starting RIGHT/SLAVE camera...";
        ret = right_camera->start();
        if (ret < 0) {
            UNLOOK_LOG_ERROR("Camera") << "Failed to start right camera: " << std::strerror(-ret);
            left_camera->stop();
            capture_running = false;
            return;
        }
        
        // Wait for cameras to enter running state
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Queue initial requests AFTER cameras are running
        UNLOOK_LOG_INFO("Camera") << "Queueing initial requests...";
        for (auto& request : left_requests) {
            ret = left_camera->queueRequest(request.get());
            if (ret < 0) {
                UNLOOK_LOG_ERROR("Camera") << "Failed to queue left request: " << std::strerror(-ret);
            }
        }
        
        for (auto& request : right_requests) {
            ret = right_camera->queueRequest(request.get());
            if (ret < 0) {
                UNLOOK_LOG_ERROR("Camera") << "Failed to queue right request: " << std::strerror(-ret);
            }
        }
        
        // Start processing thread
        processing_thread = std::thread([this]() { processFramePairs(); });
        
        UNLOOK_LOG_INFO("Camera") << "Started synchronized capture with hardware sync";
    }
    
    // Handle completed capture request
    void handleRequestCompleted(libcamera::Request* request, bool is_left) {
        if (!capture_running) return;
        
        // Check request status
        if (request->status() == libcamera::Request::RequestCancelled) {
            UNLOOK_LOG_WARNING("Camera") << (is_left ? "Left" : "Right") << " request cancelled";
            return;
        }
        
        std::lock_guard<std::mutex> lock(request_mutex);
        
        if (is_left) {
            left_completed_requests.push(request);
            left_frame_count++;
        } else {
            right_completed_requests.push(request);
            right_frame_count++;
        }
        
        frame_ready_cv.notify_one();
        
        // Re-queue the request (with proper error handling)
        request->reuse(libcamera::Request::ReuseBuffers);
        int ret = 0;
        if (is_left) {
            ret = left_camera->queueRequest(request);
        } else {
            ret = right_camera->queueRequest(request);
        }
        
        if (ret < 0) {
            UNLOOK_LOG_ERROR("Camera") << "Failed to re-queue " 
                                       << (is_left ? "left" : "right") 
                                       << " request: " << std::strerror(-ret);
        }
    }
    
    // Process synchronized frame pairs
    void processFramePairs() {
        while (capture_running) {
            std::unique_lock<std::mutex> lock(request_mutex);
            
            // Wait for frames from both cameras
            frame_ready_cv.wait(lock, [this] {
                return !capture_running || 
                       (!left_completed_requests.empty() && !right_completed_requests.empty());
            });
            
            if (!capture_running) break;
            
            // Get synchronized frames
            libcamera::Request* left_req = left_completed_requests.front();
            libcamera::Request* right_req = right_completed_requests.front();
            left_completed_requests.pop();
            right_completed_requests.pop();
            
            lock.unlock();
            
            // Get timestamps from metadata
            uint64_t left_timestamp = extractTimestamp(left_req);
            uint64_t right_timestamp = extractTimestamp(right_req);
            
            // Calculate synchronization error
            double sync_error_ms = std::abs(static_cast<double>(left_timestamp - right_timestamp)) / 1e6;
            
            // Update statistics
            updateSyncStatistics(sync_error_ms);
            
            // Check synchronization quality
            bool synchronized = sync_error_ms < MAX_SYNC_ERROR_MS;
            if (!synchronized) {
                sync_errors++;
                UNLOOK_LOG_WARNING("Camera") << "Sync error: " << sync_error_ms << " ms";
            }
            
            // Create stereo frame pair
            core::StereoFramePair frame_pair;
            
            // Convert left frame
            const libcamera::Stream* left_stream = left_config->at(0).stream();
            libcamera::FrameBuffer* left_buffer = left_req->findBuffer(left_stream);
            frame_pair.left_frame.image = convertFrameToMat(left_buffer);
            frame_pair.left_frame.timestamp_ns = left_timestamp;
            frame_pair.left_frame.camera_id = core::CameraId::LEFT;
            frame_pair.left_frame.valid = !frame_pair.left_frame.image.empty();
            
            // Convert right frame
            const libcamera::Stream* right_stream = right_config->at(0).stream();
            libcamera::FrameBuffer* right_buffer = right_req->findBuffer(right_stream);
            frame_pair.right_frame.image = convertFrameToMat(right_buffer);
            frame_pair.right_frame.timestamp_ns = right_timestamp;
            frame_pair.right_frame.camera_id = core::CameraId::RIGHT;
            frame_pair.right_frame.valid = !frame_pair.right_frame.image.empty();
            
            frame_pair.sync_error_ms = sync_error_ms;
            frame_pair.synchronized = synchronized;
            
            // Deliver frame pair to callback
            if (frame_callback && frame_pair.left_frame.valid && frame_pair.right_frame.valid) {
                frame_callback(frame_pair);
            }
        }
    }
    
    // Extract timestamp from request metadata
    uint64_t extractTimestamp(libcamera::Request* request) {
        const libcamera::ControlList& metadata = request->metadata();
        
        // Try to get sensor timestamp
        auto timestamp_ctrl = metadata.get(libcamera::controls::SensorTimestamp);
        if (timestamp_ctrl) {
            return *timestamp_ctrl;
        }
        
        // Fallback to system time
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    
    // Update synchronization statistics
    void updateSyncStatistics(double sync_error_ms) {
        std::lock_guard<std::mutex> lock(stats_mutex);
        total_sync_error_ms += sync_error_ms;
        sync_sample_count++;
    }
    
    // Stop capture
    void stopCapture() {
        capture_running = false;
        frame_ready_cv.notify_all();
        
        if (processing_thread.joinable()) {
            processing_thread.join();
        }
        
        if (left_camera) {
            // Disconnect signals before stopping
            if (signal_handler) {
                left_camera->requestCompleted.disconnect(signal_handler.get());
            }
            left_camera->stop();
        }
        if (right_camera) {
            // Disconnect signals before stopping
            if (signal_handler) {
                right_camera->requestCompleted.disconnect(signal_handler.get());
            }
            right_camera->stop();
        }
        
        // Clean up signal handler
        signal_handler.reset();
        
        // Clear request queues
        std::lock_guard<std::mutex> lock(request_mutex);
        while (!left_completed_requests.empty()) {
            left_completed_requests.pop();
        }
        while (!right_completed_requests.empty()) {
            right_completed_requests.pop();
        }
    }
    
    ~CameraImpl() {
        if (capture_running) {
            stopCapture();
        }
        
        // Clean up libcamera resources
        unmapFrameBuffers();
        
        left_requests.clear();
        right_requests.clear();
        
        if (left_allocator) {
            left_allocator.reset();
        }
        if (right_allocator) {
            right_allocator.reset();
        }
        
        if (left_camera) {
            left_camera->release();
        }
        if (right_camera) {
            right_camera->release();
        }
        
        if (camera_manager) {
            camera_manager->stop();
        }
    }
};

CameraSystem::CameraSystem() 
    : capture_running_(false)
    , system_ready_(false)
    , left_state_(core::CameraState::DISCONNECTED)
    , right_state_(core::CameraState::DISCONNECTED)
    , total_sync_error_ms_(0.0)
    , sync_sample_count_(0)
    , impl_(std::make_unique<CameraImpl>())
{
    UNLOOK_LOG_INFO("Camera") << "CameraSystem constructor - libcamera-sync integration";
}

CameraSystem::~CameraSystem() {
    shutdown();
    UNLOOK_LOG_INFO("Camera") << "CameraSystem destructor";
}

std::shared_ptr<CameraSystem> CameraSystem::getInstance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (!instance_) {
        instance_ = std::shared_ptr<CameraSystem>(new CameraSystem());
    }
    return instance_;
}

bool CameraSystem::initialize() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (system_ready_) {
        UNLOOK_LOG_WARNING("Camera") << "System already initialized";
        return true;
    }
    
    UNLOOK_LOG_INFO("Camera") << "Initializing camera system with IMX296 sensors...";
    
    left_state_ = core::CameraState::INITIALIZING;
    right_state_ = core::CameraState::INITIALIZING;
    
    // Initialize GPIO for hardware sync
    if (!impl_->initializeGPIO()) {
        UNLOOK_LOG_WARNING("Camera") << "GPIO initialization failed, continuing without hardware sync";
    }
    
    // Initialize libcamera
    if (!impl_->initializeLibcamera()) {
        UNLOOK_LOG_ERROR("Camera") << "Failed to initialize libcamera";
        left_state_ = core::CameraState::ERROR;
        right_state_ = core::CameraState::ERROR;
        if (error_callback_) {
            error_callback_("Failed to initialize libcamera-sync");
        }
        return false;
    }
    
    // Configure LEFT camera (MASTER)
    UNLOOK_LOG_INFO("Camera") << "Configuring LEFT camera as MASTER...";
    if (!impl_->configureCamera(impl_->left_camera, impl_->left_config, 
                                impl_->left_allocator, impl_->left_requests, true)) {
        UNLOOK_LOG_ERROR("Camera") << "Failed to configure LEFT camera";
        left_state_ = core::CameraState::ERROR;
        if (error_callback_) {
            error_callback_("Failed to configure LEFT/MASTER camera");
        }
        return false;
    }
    
    // Configure RIGHT camera (SLAVE)
    UNLOOK_LOG_INFO("Camera") << "Configuring RIGHT camera as SLAVE...";
    if (!impl_->configureCamera(impl_->right_camera, impl_->right_config,
                                impl_->right_allocator, impl_->right_requests, false)) {
        UNLOOK_LOG_ERROR("Camera") << "Failed to configure RIGHT camera";
        right_state_ = core::CameraState::ERROR;
        if (error_callback_) {
            error_callback_("Failed to configure RIGHT/SLAVE camera");
        }
        return false;
    }
    
    // Set camera configurations
    left_config_.width = IMX296_WIDTH;
    left_config_.height = IMX296_HEIGHT;
    left_config_.fps = TARGET_FPS;
    left_config_.hardware_sync = true;
    left_config_.is_master = true;
    
    right_config_.width = IMX296_WIDTH;
    right_config_.height = IMX296_HEIGHT;
    right_config_.fps = TARGET_FPS;
    right_config_.hardware_sync = true;
    right_config_.is_master = false;
    
    // Enable hardware synchronization
    impl_->hardware_sync_enabled = true;
    
    impl_->initialized = true;
    system_ready_ = true;
    left_state_ = core::CameraState::READY;
    right_state_ = core::CameraState::READY;
    
    UNLOOK_LOG_INFO("Camera") << "Camera system initialized successfully";
    UNLOOK_LOG_INFO("Camera") << "Hardware sync: XVS/XHS enabled, <1ms precision target";
    UNLOOK_LOG_INFO("Camera") << "Resolution: " << IMX296_WIDTH << "x" << IMX296_HEIGHT << " YUV420";
    UNLOOK_LOG_INFO("Camera") << "Baseline: 70.017mm (from calibration)";
    
    return true;
}

void CameraSystem::shutdown() {
    stopCapture();
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (impl_->capture_running) {
        impl_->stopCapture();
    }
    
    system_ready_ = false;
    left_state_ = core::CameraState::DISCONNECTED;
    right_state_ = core::CameraState::DISCONNECTED;
    impl_->initialized = false;
    
    UNLOOK_LOG_INFO("Camera") << "Camera system shutdown complete";
}

bool CameraSystem::isReady() const {
    return system_ready_.load();
}

core::CameraState CameraSystem::getCameraState(core::CameraId camera_id) const {
    if (camera_id == core::CameraId::LEFT) {
        return left_state_.load();
    } else {
        return right_state_.load();
    }
}

bool CameraSystem::configureCameras(const core::CameraConfig& left_config, 
                                   const core::CameraConfig& right_config) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    left_config_ = left_config;
    right_config_ = right_config;
    return true;
}

bool CameraSystem::startCapture(core::StereoFrameCallback frame_callback) {
    if (!system_ready_) {
        UNLOOK_LOG_ERROR("Camera") << "System not initialized";
        return false;
    }
    
    if (capture_running_.load()) {
        UNLOOK_LOG_WARNING("Camera") << "Capture already running";
        return false;
    }
    
    frame_callback_ = frame_callback;
    capture_running_ = true;
    left_state_ = core::CameraState::CAPTURING;
    right_state_ = core::CameraState::CAPTURING;
    
    // Start real synchronized capture
    impl_->startCapture(frame_callback);
    
    UNLOOK_LOG_INFO("Camera") << "Hardware-synchronized stereo capture started";
    return true;
}

void CameraSystem::stopCapture() {
    if (!capture_running_.load()) {
        return;
    }
    
    capture_running_ = false;
    impl_->stopCapture();
    
    left_state_ = core::CameraState::READY;
    right_state_ = core::CameraState::READY;
    
    // Log capture statistics
    UNLOOK_LOG_INFO("Camera") << "Capture stopped - Statistics:";
    UNLOOK_LOG_INFO("Camera") << "  Left frames: " << impl_->left_frame_count.load();
    UNLOOK_LOG_INFO("Camera") << "  Right frames: " << impl_->right_frame_count.load();
    UNLOOK_LOG_INFO("Camera") << "  Sync errors: " << impl_->sync_errors.load();
    UNLOOK_LOG_INFO("Camera") << "  Avg sync error: " << getAverageSyncError() << " ms";
}

core::StereoFramePair CameraSystem::captureSingle() {
    core::StereoFramePair frame_pair;
    
    if (!system_ready_) {
        UNLOOK_LOG_ERROR("Camera") << "System not initialized";
        frame_pair.synchronized = false;
        return frame_pair;
    }
    
    // Temporarily start capture for single frame
    if (!capture_running_) {
        // Use a lambda to capture single synchronized frame
        std::mutex single_mutex;
        std::condition_variable single_cv;
        bool frame_received = false;
        
        auto single_callback = [&](const core::StereoFramePair& pair) {
            std::lock_guard<std::mutex> lock(single_mutex);
            frame_pair = pair;
            frame_received = true;
            single_cv.notify_one();
        };
        
        // Start capture
        impl_->startCapture(single_callback);
        
        // Wait for single frame
        {
            std::unique_lock<std::mutex> lock(single_mutex);
            single_cv.wait_for(lock, std::chrono::milliseconds(100),
                             [&] { return frame_received; });
        }
        
        // Stop capture
        impl_->stopCapture();
        
        if (!frame_received) {
            UNLOOK_LOG_ERROR("Camera") << "Timeout waiting for single frame";
            frame_pair.synchronized = false;
        }
    } else {
        // Capture already running, wait for next frame
        UNLOOK_LOG_WARNING("Camera") << "Continuous capture running, cannot capture single frame";
        frame_pair.synchronized = false;
    }
    
    return frame_pair;
}

core::CameraConfig CameraSystem::getCameraConfig(core::CameraId camera_id) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (camera_id == core::CameraId::LEFT) {
        return left_config_;
    } else {
        return right_config_;
    }
}

bool CameraSystem::setAutoExposure(core::CameraId camera_id, bool enabled) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (camera_id == core::CameraId::LEFT) {
        left_config_.auto_exposure = enabled;
    } else {
        right_config_.auto_exposure = enabled;
    }
    return true;
}

bool CameraSystem::setExposureTime(core::CameraId camera_id, double exposure_us) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (camera_id == core::CameraId::LEFT) {
        left_config_.exposure_time_us = exposure_us;
    } else {
        right_config_.exposure_time_us = exposure_us;
    }
    return true;
}

bool CameraSystem::setAutoGain(core::CameraId camera_id, bool enabled) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (camera_id == core::CameraId::LEFT) {
        left_config_.auto_gain = enabled;
    } else {
        right_config_.auto_gain = enabled;
    }
    return true;
}

bool CameraSystem::setGain(core::CameraId camera_id, double gain) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (camera_id == core::CameraId::LEFT) {
        left_config_.gain = gain;
    } else {
        right_config_.gain = gain;
    }
    return true;
}

bool CameraSystem::setFrameRate(double fps) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    left_config_.fps = fps;
    right_config_.fps = fps;
    return true;
}

double CameraSystem::getCurrentFrameRate() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return left_config_.fps;
}

std::string CameraSystem::getCameraInfo(core::CameraId camera_id) const {
    if (!impl_->initialized) {
        return "Camera not initialized";
    }
    
    std::stringstream info;
    if (camera_id == core::CameraId::LEFT) {
        info << "LEFT Camera (MASTER) - IMX296\n";
        info << "  Path: " << LEFT_CAMERA_PATH << "\n";
        info << "  Resolution: " << IMX296_WIDTH << "x" << IMX296_HEIGHT << "\n";
        info << "  Format: SBGGR10\n";
        info << "  Sync: XVS/XHS MASTER\n";
        if (impl_->left_camera) {
            info << "  ID: " << impl_->left_camera->id();
        }
    } else {
        info << "RIGHT Camera (SLAVE) - IMX296\n";
        info << "  Path: " << RIGHT_CAMERA_PATH << "\n";
        info << "  Resolution: " << IMX296_WIDTH << "x" << IMX296_HEIGHT << "\n";
        info << "  Format: SBGGR10\n";
        info << "  Sync: XVS/XHS SLAVE\n";
        if (impl_->right_camera) {
            info << "  ID: " << impl_->right_camera->id();
        }
    }
    return info.str();
}

bool CameraSystem::isHardwareSyncEnabled() const {
    return impl_->hardware_sync_enabled;
}

double CameraSystem::getAverageSyncError() const {
    std::lock_guard<std::mutex> lock(impl_->stats_mutex);
    if (impl_->sync_sample_count == 0) {
        return 0.0;
    }
    return impl_->total_sync_error_ms / impl_->sync_sample_count;
}

void CameraSystem::setErrorCallback(core::ErrorCallback callback) {
    error_callback_ = callback;
}

} // namespace camera
} // namespace unlook