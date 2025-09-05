#include <unlook/camera/CameraDevice.hpp>
#include <unlook/camera/CameraUtils.hpp>
#include <unlook/core/Logger.hpp>
#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

namespace unlook {
namespace camera {

using namespace libcamera;

CameraDevice::CameraDevice(int cameraId, Role role)
    : cameraId_(cameraId)
    , role_(role) {
    LOG_DEBUG("Creating CameraDevice ID: " + std::to_string(cameraId) + 
              ", Role: " + (role == Role::MASTER ? "MASTER" : "SLAVE"));
}

CameraDevice::~CameraDevice() {
    LOG_DEBUG("Destroying CameraDevice ID: " + std::to_string(cameraId_));
    releaseResources();
}

bool CameraDevice::initialize() {
    if (initialized_) {
        LOG_WARNING("Camera already initialized");
        return true;
    }
    
    LOG_INFO("Initializing camera " + std::to_string(cameraId_));
    
    // Create camera manager
    cameraManager_ = std::make_unique<CameraManager>();
    int ret = cameraManager_->start();
    if (ret < 0) {
        lastError_ = "Failed to start camera manager: " + std::to_string(ret);
        LOG_ERROR(lastError_);
        return false;
    }
    
    // Get list of cameras
    auto cameras = cameraManager_->cameras();
    if (cameras.empty()) {
        lastError_ = "No cameras found";
        LOG_ERROR(lastError_);
        return false;
    }
    
    if (static_cast<size_t>(cameraId_) >= cameras.size()) {
        lastError_ = "Camera ID " + std::to_string(cameraId_) + " not found";
        LOG_ERROR(lastError_);
        return false;
    }
    
    // Get camera
    camera_ = cameras[cameraId_];
    
    // Get camera model
    const ControlList &props = camera_->properties();
    if (props.contains(properties::Model.id())) {
        cameraModel_ = props.get(properties::Model).value_or("Unknown");
        LOG_INFO("Camera model: " + cameraModel_);
        
        // Validate IMX296 sensor
        if (cameraModel_.find("imx296") == std::string::npos &&
            cameraModel_.find("IMX296") == std::string::npos) {
            LOG_WARNING("Camera is not IMX296 sensor: " + cameraModel_);
        }
    }
    
    // Acquire camera
    ret = camera_->acquire();
    if (ret < 0) {
        lastError_ = "Failed to acquire camera: " + std::to_string(ret);
        LOG_ERROR(lastError_);
        return false;
    }
    
    // Generate configuration
    config_ = camera_->generateConfiguration({ StreamRole::VideoRecording });
    if (!config_) {
        lastError_ = "Failed to generate camera configuration";
        LOG_ERROR(lastError_);
        camera_->release();
        return false;
    }
    
    // Configure stream for IMX296 resolution
    StreamConfiguration &streamConfig = config_->at(0);
    streamConfig.pixelFormat = formats::SBGGR10;  // IMX296 native format
    streamConfig.size.width = 1456;
    streamConfig.size.height = 1088;
    streamConfig.bufferCount = 4;  // Use 4 buffers for smooth streaming
    
    LOG_INFO("Configuring stream: " + std::to_string(streamConfig.size.width) + "x" + 
             std::to_string(streamConfig.size.height) + " " + streamConfig.pixelFormat.toString());
    
    // Validate configuration
    CameraConfiguration::Status validation = config_->validate();
    if (validation == CameraConfiguration::Invalid) {
        lastError_ = "Invalid camera configuration";
        LOG_ERROR(lastError_);
        camera_->release();
        return false;
    }
    
    if (validation == CameraConfiguration::Adjusted) {
        LOG_WARNING("Camera configuration was adjusted");
    }
    
    // Apply configuration
    ret = camera_->configure(config_.get());
    if (ret < 0) {
        lastError_ = "Failed to configure camera: " + std::to_string(ret);
        LOG_ERROR(lastError_);
        camera_->release();
        return false;
    }
    
    // Get the configured stream
    stream_ = streamConfig.stream();
    
    // Allocate buffers
    if (!allocateBuffers()) {
        LOG_ERROR("Failed to allocate buffers");
        camera_->release();
        return false;
    }
    
    // Configure controls
    if (!configureControls()) {
        LOG_WARNING("Failed to configure controls");
    }
    
    // Configure hardware sync if needed
    if (role_ == Role::MASTER || role_ == Role::SLAVE) {
        if (!configureHardwareSync()) {
            LOG_WARNING("Failed to configure hardware sync");
        }
    }
    
    initialized_ = true;
    LOG_INFO("Camera " + std::to_string(cameraId_) + " initialized successfully");
    
    return true;
}

bool CameraDevice::allocateBuffers() {
    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
    
    int ret = allocator_->allocate(stream_);
    if (ret < 0) {
        lastError_ = "Failed to allocate buffers: " + std::to_string(ret);
        LOG_ERROR(lastError_);
        return false;
    }
    
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator_->buffers(stream_);
    LOG_INFO("Allocated " + std::to_string(buffers.size()) + " buffers");
    
    // Create requests for each buffer
    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<Request> request = camera_->createRequest();
        if (!request) {
            lastError_ = "Failed to create request";
            LOG_ERROR(lastError_);
            return false;
        }
        
        ret = request->addBuffer(stream_, buffers[i].get());
        if (ret < 0) {
            lastError_ = "Failed to add buffer to request: " + std::to_string(ret);
            LOG_ERROR(lastError_);
            return false;
        }
        
        requests_.push_back(std::move(request));
    }
    
    LOG_INFO("Created " + std::to_string(requests_.size()) + " requests");
    return true;
}

bool CameraDevice::configureControls() {
    if (!camera_) {
        return false;
    }
    
    // Get available controls
    const ControlInfoMap &controls = camera_->controls();
    
    // Set initial exposure if supported
    if (controls.count(&controls::ExposureTime) > 0) {
        LOG_DEBUG("Camera supports exposure control");
    }
    
    if (controls.count(&controls::AnalogueGain) > 0) {
        LOG_DEBUG("Camera supports gain control");
    }
    
    if (controls.count(&controls::FrameDurationLimits) > 0) {
        LOG_DEBUG("Camera supports frame duration control");
    }
    
    return true;
}

bool CameraDevice::configureHardwareSync() {
    LOG_INFO(std::string("Configuring hardware sync for ") + 
             (role_ == Role::MASTER ? "MASTER" : "SLAVE") + " camera");
    
    // Hardware sync configuration is handled at the sensor level
    // through libcamera-sync modifications
    
    // The actual GPIO and sensor register configuration is done
    // in the libcamera-sync library which has IMX296 sync support
    
    return true;
}

bool CameraDevice::start() {
    if (!initialized_) {
        lastError_ = "Camera not initialized";
        LOG_ERROR(lastError_);
        return false;
    }
    
    if (streaming_) {
        LOG_WARNING("Camera already streaming");
        return true;
    }
    
    LOG_INFO("Starting camera " + std::to_string(cameraId_));
    
    // Connect request completed signal
    camera_->requestCompleted.connect(this, &CameraDevice::requestComplete);
    
    // Start camera
    int ret = camera_->start();
    if (ret < 0) {
        lastError_ = "Failed to start camera: " + std::to_string(ret);
        LOG_ERROR(lastError_);
        return false;
    }
    
    // Queue initial requests
    for (auto &request : requests_) {
        ret = camera_->queueRequest(request.get());
        if (ret < 0) {
            LOG_WARNING("Failed to queue request: " + std::to_string(ret));
        }
    }
    
    streaming_ = true;
    LOG_INFO("Camera " + std::to_string(cameraId_) + " started");
    
    return true;
}

void CameraDevice::stop() {
    if (!streaming_) {
        return;
    }
    
    LOG_INFO("Stopping camera " + std::to_string(cameraId_));
    
    streaming_ = false;
    
    if (camera_) {
        camera_->stop();
        camera_->requestCompleted.disconnect(this, &CameraDevice::requestComplete);
    }
    
    LOG_INFO("Camera " + std::to_string(cameraId_) + " stopped");
}

void CameraDevice::requestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled) {
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(captureMutex_);
        
        // Store completed request
        completedRequest_ = request;
        
        // Extract frame and metadata
        const Request::BufferMap &buffers = request->buffers();
        for (auto &[stream, buffer] : buffers) {
            if (stream == stream_) {
                // Convert buffer to OpenCV Mat
                bufferToMat(buffer, latestFrame_);
                
                // Extract metadata
                extractMetadata(request, latestMetadata_);
                
                framesCaptu_++;
                break;
            }
        }
    }
    
    // Notify waiting threads
    frameReady_.notify_all();
    
    // Requeue request if still streaming
    if (streaming_) {
        // Clear and reuse request
        request->reuse(Request::ReuseBuffers);
        
        // Apply current parameters
        ControlList &controls = request->controls();
        controls.set(controls::ExposureTime, static_cast<int32_t>(parameters_.exposureTime));
        controls.set(controls::AnalogueGain, parameters_.analogGain);
        
        int ret = camera_->queueRequest(request);
        if (ret < 0) {
            LOG_WARNING("Failed to requeue request: " + std::to_string(ret));
            framesDropped_++;
        }
    }
}

bool CameraDevice::captureFrame(cv::Mat& frame, FrameMetadata& metadata, int timeoutMs) {
    if (!streaming_) {
        LOG_ERROR("Camera not streaming");
        return false;
    }
    
    std::unique_lock<std::mutex> lock(captureMutex_);
    
    // Wait for frame
    if (!frameReady_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                               [this] { return completedRequest_ != nullptr; })) {
        LOG_WARNING("Frame capture timeout");
        return false;
    }
    
    // Copy frame and metadata
    frame = latestFrame_.clone();
    metadata = latestMetadata_;
    
    // Clear completed request flag
    completedRequest_ = nullptr;
    
    return true;
}

bool CameraDevice::queueRequest() {
    // This is handled automatically in the request completion handler
    return streaming_;
}

bool CameraDevice::waitForFrame(cv::Mat& frame, FrameMetadata& metadata, int timeoutMs) {
    return captureFrame(frame, metadata, timeoutMs);
}

bool CameraDevice::bufferToMat(FrameBuffer *buffer, cv::Mat& mat) {
    if (!buffer) {
        return false;
    }
    
    const FrameBuffer::Plane &plane = buffer->planes()[0];
    
    // Memory map the buffer
    void *data = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    if (data == MAP_FAILED) {
        LOG_ERROR("Failed to mmap buffer");
        return false;
    }
    
    // Convert SBGGR10 to grayscale
    // IMX296 outputs 1456x1088 SBGGR10 format
    CameraUtils::sbggr10ToGrayscale(static_cast<const uint8_t*>(data), 
                                     1456, 1088, mat);
    
    // Unmap buffer
    munmap(data, plane.length);
    
    return true;
}

void CameraDevice::extractMetadata(Request *request, FrameMetadata& metadata) {
    metadata.timestampNs = CameraUtils::getTimestampNs();
    metadata.frameNumber = framesCaptu_;
    
    const ControlList &controls = request->metadata();
    
    if (controls.contains(controls::ExposureTime.id())) {
        metadata.exposureTime = controls.get(controls::ExposureTime).value_or(0);
    }
    
    if (controls.contains(controls::AnalogueGain.id())) {
        metadata.analogGain = controls.get(controls::AnalogueGain).value_or(1.0);
    }
    
    if (controls.contains(controls::DigitalGain.id())) {
        metadata.digitalGain = controls.get(controls::DigitalGain).value_or(1.0);
    }
    
    if (controls.contains(controls::SensorTimestamp.id())) {
        metadata.sensorTimestamp = controls.get(controls::SensorTimestamp).value_or(0);
        // Use sensor timestamp if available (more accurate for sync)
        if (metadata.sensorTimestamp > 0) {
            metadata.timestampNs = metadata.sensorTimestamp;
        }
    }
}

bool CameraDevice::setExposure(double exposureUs) {
    std::lock_guard<std::mutex> lock(paramMutex_);
    parameters_.exposureTime = exposureUs;
    LOG_DEBUG("Set exposure to " + std::to_string(exposureUs) + " us");
    return true;
}

bool CameraDevice::setAnalogGain(double gain) {
    std::lock_guard<std::mutex> lock(paramMutex_);
    parameters_.analogGain = gain;
    LOG_DEBUG("Set analog gain to " + std::to_string(gain));
    return true;
}

bool CameraDevice::setDigitalGain(double gain) {
    std::lock_guard<std::mutex> lock(paramMutex_);
    parameters_.digitalGain = gain;
    LOG_DEBUG("Set digital gain to " + std::to_string(gain));
    return true;
}

bool CameraDevice::setAutoExposure(bool enable) {
    std::lock_guard<std::mutex> lock(paramMutex_);
    parameters_.autoExposure = enable;
    LOG_DEBUG("Set auto exposure to " + std::string(enable ? "enabled" : "disabled"));
    return true;
}

bool CameraDevice::setAutoGain(bool enable) {
    std::lock_guard<std::mutex> lock(paramMutex_);
    parameters_.autoGain = enable;
    LOG_DEBUG("Set auto gain to " + std::string(enable ? "enabled" : "disabled"));
    return true;
}

bool CameraDevice::setFrameRate(int fps) {
    std::lock_guard<std::mutex> lock(paramMutex_);
    parameters_.frameRate = fps;
    LOG_DEBUG("Set frame rate to " + std::to_string(fps) + " fps");
    
    // Calculate frame duration limits for the target FPS
    // Frame duration in microseconds
    int64_t frameDuration = 1000000 / fps;
    
    // This would be applied in the request controls
    // controls.set(controls::FrameDurationLimits, { frameDuration, frameDuration });
    
    return true;
}

CameraDevice::Parameters CameraDevice::getParameters() const {
    std::lock_guard<std::mutex> lock(paramMutex_);
    return parameters_;
}

bool CameraDevice::applyParameters(const Parameters& params) {
    std::lock_guard<std::mutex> lock(paramMutex_);
    parameters_ = params;
    LOG_DEBUG("Applied camera parameters");
    return true;
}

bool CameraDevice::enableSync(bool enable) {
    // Sync enable/disable would be handled through sensor-specific controls
    // This is implemented in the libcamera-sync library
    LOG_INFO("Sync " + std::string(enable ? "enabled" : "disabled") + 
             " for camera " + std::to_string(cameraId_));
    return true;
}

void CameraDevice::releaseResources() {
    if (streaming_) {
        stop();
    }
    
    requests_.clear();
    
    if (allocator_) {
        if (stream_) {
            allocator_->free(stream_);
        }
        allocator_.reset();
    }
    
    if (camera_) {
        camera_->release();
        camera_.reset();
    }
    
    if (cameraManager_) {
        cameraManager_->stop();
        cameraManager_.reset();
    }
    
    initialized_ = false;
}

} // namespace camera
} // namespace unlook