#include <unlook/camera/RawCameraInterface.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/imgproc.hpp>
#include <sys/mman.h>
#include <fstream>
#include <chrono>
#include <cstring>
#include <algorithm>

namespace unlook {
namespace camera {

using namespace libcamera;

RawCameraInterface::RawCameraInterface(int cameraId)
    : cameraId_(cameraId) {
}

RawCameraInterface::~RawCameraInterface() {
    stop();
    if (allocator_) {
        allocator_->free(stream_);
    }
    if (camera_) {
        camera_->release();
    }
}

bool RawCameraInterface::initialize() {
    if (initialized_) {
        return true;
    }

    LOG_INFO("Initializing RAW camera interface for camera " + std::to_string(cameraId_));

    // Initialize camera manager
    cameraManager_ = std::make_unique<CameraManager>();
    int ret = cameraManager_->start();
    if (ret < 0) {
        lastError_ = "Failed to start camera manager";
        LOG_ERROR(lastError_);
        return false;
    }

    // Get camera list
    auto cameras = cameraManager_->cameras();
    if (cameras.empty()) {
        lastError_ = "No cameras found";
        LOG_ERROR(lastError_);
        return false;
    }

    if (static_cast<size_t>(cameraId_) >= cameras.size()) {
        lastError_ = "Camera ID out of range";
        LOG_ERROR(lastError_);
        return false;
    }

    // Acquire camera
    camera_ = cameras[cameraId_];
    ret = camera_->acquire();
    if (ret < 0) {
        lastError_ = "Failed to acquire camera";
        LOG_ERROR(lastError_);
        return false;
    }

    // Configure for RAW capture
    config_ = camera_->generateConfiguration({StreamRole::Raw});
    if (!config_) {
        lastError_ = "Failed to generate RAW configuration";
        LOG_ERROR(lastError_);
        camera_->release();
        return false;
    }

    // Set SBGGR10 format for IMX296
    StreamConfiguration &cfg = config_->at(0);
    cfg.pixelFormat = PixelFormat::fromString("SBGGR10");
    cfg.size.width = 1456;
    cfg.size.height = 1088;
    cfg.bufferCount = 4;

    // Validate configuration
    CameraConfiguration::Status status = config_->validate();
    if (status == CameraConfiguration::Invalid) {
        lastError_ = "Invalid camera configuration";
        LOG_ERROR(lastError_);
        camera_->release();
        return false;
    }

    // Apply configuration
    ret = camera_->configure(config_.get());
    if (ret < 0) {
        lastError_ = "Failed to configure camera";
        LOG_ERROR(lastError_);
        camera_->release();
        return false;
    }

    // Get stream
    stream_ = config_->at(0).stream();

    // Allocate buffers
    allocator_ = std::make_unique<FrameBufferAllocator>(camera_);
    ret = allocator_->allocate(stream_);
    if (ret < 0) {
        lastError_ = "Failed to allocate buffers";
        LOG_ERROR(lastError_);
        camera_->release();
        return false;
    }

    // Create requests
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator_->buffers(stream_);
    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<Request> request = camera_->createRequest();
        if (!request) {
            lastError_ = "Failed to create request";
            LOG_ERROR(lastError_);
            return false;
        }

        ret = request->addBuffer(stream_, buffers[i].get());
        if (ret < 0) {
            lastError_ = "Failed to add buffer to request";
            LOG_ERROR(lastError_);
            return false;
        }

        requests_.push_back(std::move(request));
    }

    initialized_ = true;
    LOG_INFO("RAW camera interface initialized successfully");
    return true;
}

bool RawCameraInterface::start() {
    if (!initialized_) {
        lastError_ = "Camera not initialized";
        LOG_ERROR(lastError_);
        return false;
    }

    if (streaming_) {
        return true;
    }

    LOG_INFO("Starting RAW camera stream");

    // Connect request completed signal - use member function
    camera_->requestCompleted.connect(this, &RawCameraInterface::requestComplete);

    // Start camera
    int ret = camera_->start();
    if (ret < 0) {
        lastError_ = "Failed to start camera";
        LOG_ERROR(lastError_);
        return false;
    }

    // Queue initial requests
    for (auto &request : requests_) {
        ret = camera_->queueRequest(request.get());
        if (ret < 0) {
            LOG_WARNING("Failed to queue request");
        }
    }

    streaming_ = true;
    LOG_INFO("RAW camera stream started");
    return true;
}

void RawCameraInterface::stop() {
    if (!streaming_) {
        return;
    }

    LOG_INFO("Stopping RAW camera stream");
    streaming_ = false;

    if (camera_) {
        camera_->stop();
    }

    LOG_INFO("RAW camera stream stopped");
}

bool RawCameraInterface::captureRawFrame(RawFrame& frame,
                                         const ProcessingOptions& options,
                                         int timeoutMs) {
    if (!streaming_) {
        lastError_ = "Camera not streaming";
        return false;
    }

    auto start = std::chrono::steady_clock::now();

    // Wait for frame with timeout
    std::unique_lock<std::mutex> lock(captureMutex_);
    if (!frameReady_.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                              [this] { return !streaming_ || completedRequest_ != nullptr; })) {
        lastError_ = "Capture timeout";
        return false;
    }

    // Get completed request
    Request *request = completedRequest_;
    completedRequest_ = nullptr;  // Reset for next capture

    if (!request) {
        lastError_ = "No completed request found";
        return false;
    }

    // Extract metadata
    const ControlList &metadata = request->metadata();
    frame.timestampNs = std::chrono::steady_clock::now().time_since_epoch().count();
    frame.frameNumber = frameCounter_++;

    if (metadata.contains(controls::ExposureTime.id())) {
        frame.exposureTime = metadata.get(controls::ExposureTime).value_or(0);
    }
    if (metadata.contains(controls::AnalogueGain.id())) {
        frame.analogGain = metadata.get(controls::AnalogueGain).value_or(1.0);
    }

    // Access raw buffer
    const Request::BufferMap &buffers = request->buffers();
    for (auto &[stream, buffer] : buffers) {
        if (stream == stream_) {
            if (!accessRawBuffer(buffer, frame)) {
                lastError_ = "Failed to access raw buffer";
                return false;
            }

            // Process raw frame
            const FrameBuffer::Plane &plane = buffer->planes()[0];
            void *data = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
            if (data == MAP_FAILED) {
                lastError_ = "Failed to mmap buffer";
                return false;
            }

            processRawFrame(static_cast<const uint8_t*>(data), plane.length, frame, options);

            munmap(data, plane.length);
            break;
        }
    }

    // Requeue request
    request->reuse(Request::ReuseBuffers);
    camera_->queueRequest(request);

    // Update statistics
    auto end = std::chrono::steady_clock::now();
    stats_.captureTimeMs = std::chrono::duration<double, std::milli>(end - start).count();

    return true;
}

void RawCameraInterface::requestComplete(Request* request) {
    std::lock_guard<std::mutex> lock(captureMutex_);
    completedRequest_ = request;
    // Signal frame ready
    frameReady_.notify_all();
}

bool RawCameraInterface::accessRawBuffer(FrameBuffer* buffer, RawFrame& frame) {
    if (!buffer) {
        return false;
    }

    const FrameBuffer::Plane &plane = buffer->planes()[0];

    // Memory map the buffer
    void *data = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    if (data == MAP_FAILED) {
        LOG_ERROR("Failed to mmap raw buffer");
        return false;
    }

    // Store raw data directly without conversion
    // SBGGR10 packed: 4 pixels in 5 bytes
    int width = 1456;
    int height = 1088;

    // Create 16-bit Mat for unpacked data
    frame.rawBayer = cv::Mat(height, width, CV_16UC1);

    // Unpack SBGGR10 to 16-bit
    std::vector<uint16_t> unpacked(width * height);
    unpackSBGGR10(static_cast<const uint8_t*>(data), width, height, unpacked);

    // Copy to Mat
    memcpy(frame.rawBayer.data, unpacked.data(), unpacked.size() * sizeof(uint16_t));

    munmap(data, plane.length);
    return true;
}

void RawCameraInterface::processRawFrame(const uint8_t* rawData, size_t dataSize,
                                         RawFrame& frame, const ProcessingOptions& options) {
    auto start = std::chrono::steady_clock::now();

    int width = 1456;
    int height = 1088;

    if (options.extractBlueOnly) {
        // Extract blue channel for VCSEL IR sensitivity
        extractBlueChannel(rawData, width, height, frame.blueChannel, &frame.blueChannel8);

        if (options.computeVariance) {
            frame.blueVariance = computeVariance(frame.blueChannel);
            if (debugMode_) {
                LOG_INFO("Blue channel variance: " + std::to_string(frame.blueVariance));
            }
        }
    }

    if (options.computeVariance && !frame.rawBayer.empty()) {
        frame.variance = computeVariance(frame.rawBayer);
        if (debugMode_) {
            LOG_INFO("Raw Bayer variance: " + std::to_string(frame.variance));
        }
    }

    if (options.saveBinaryDump) {
        std::string filename = options.dumpPath + "raw_" +
                              std::to_string(frame.frameNumber) + ".bin";
        saveRawBinaryDump(rawData, dataSize, filename);
    }

    // Update statistics
    auto end = std::chrono::steady_clock::now();
    stats_.processingTimeMs = std::chrono::duration<double, std::milli>(end - start).count();

    if (options.computeVariance) {
        std::lock_guard<std::mutex> lock(statsMutex_);
        stats_.avgVariance = frame.variance;
        stats_.avgBlueIntensity = cv::mean(frame.blueChannel)[0];
    }
}

bool RawCameraInterface::extractBlueChannel(const uint8_t* rawData,
                                           int width,
                                           int height,
                                           cv::Mat& output16,
                                           cv::Mat* output8) {
    // First unpack SBGGR10 to 16-bit
    std::vector<uint16_t> unpacked(width * height);
    unpackSBGGR10(rawData, width, height, unpacked);

    // Create output matrices for blue channel only
    // Blue channel will be half resolution (every other pixel in both dimensions)
    int blueWidth = width / 2;
    int blueHeight = height / 2;

    output16 = cv::Mat(blueHeight, blueWidth, CV_16UC1);

    // Extract blue pixels from SBGGR10 pattern
    // Blue pixels are at even row, even column positions
    for (int y = 0; y < blueHeight; y++) {
        for (int x = 0; x < blueWidth; x++) {
            int srcY = y * 2;      // Even rows
            int srcX = x * 2;      // Even columns
            int srcIdx = srcY * width + srcX;

            // Direct copy of blue pixel value (no interpolation)
            output16.at<uint16_t>(y, x) = unpacked[srcIdx];
        }
    }

    // Convert to 8-bit if requested
    if (output8) {
        convert16to8bit(output16, *output8, true);
    }

    return true;
}

void RawCameraInterface::unpackSBGGR10(const uint8_t* packedData,
                                       int width,
                                       int height,
                                       std::vector<uint16_t>& unpacked) {
    const int pixels_per_pack = 4;
    const int bytes_per_pack = 5;
    const int total_pixels = width * height;

    unpacked.resize(total_pixels);

    // Unpack SBGGR10: 4 pixels (40 bits) from 5 bytes
    for (int i = 0; i < total_pixels / pixels_per_pack; i++) {
        const uint8_t* pack = packedData + i * bytes_per_pack;
        uint16_t* out = unpacked.data() + i * pixels_per_pack;

        // Extract 4 10-bit pixels
        // This preserves the full 10-bit dynamic range
        out[0] = ((uint16_t)pack[0] << 2) | (pack[1] >> 6);
        out[1] = (((uint16_t)(pack[1] & 0x3F)) << 4) | (pack[2] >> 4);
        out[2] = (((uint16_t)(pack[2] & 0x0F)) << 6) | (pack[3] >> 2);
        out[3] = (((uint16_t)(pack[3] & 0x03)) << 8) | pack[4];
    }
}

double RawCameraInterface::computeVariance(const cv::Mat& image) {
    if (image.empty()) {
        return 0.0;
    }

    cv::Scalar mean, stddev;
    cv::meanStdDev(image, mean, stddev);

    // Return variance (stddev squared)
    return stddev[0] * stddev[0];
}

cv::Mat RawCameraInterface::computeLocalVariance(const cv::Mat& image, int windowSize) {
    cv::Mat variance(image.size(), CV_32FC1);

    // Ensure window size is odd
    if (windowSize % 2 == 0) {
        windowSize++;
    }

    int halfWindow = windowSize / 2;

    // Compute local variance for each pixel
    for (int y = halfWindow; y < image.rows - halfWindow; y++) {
        for (int x = halfWindow; x < image.cols - halfWindow; x++) {
            // Extract local window
            cv::Rect roi(x - halfWindow, y - halfWindow, windowSize, windowSize);
            cv::Mat window = image(roi);

            // Compute variance in window
            cv::Scalar mean, stddev;
            cv::meanStdDev(window, mean, stddev);
            variance.at<float>(y, x) = stddev[0] * stddev[0];
        }
    }

    return variance;
}

bool RawCameraInterface::saveRawBinaryDump(const void* data, size_t size,
                                           const std::string& filename) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        LOG_ERROR("Failed to open file for binary dump: " + filename);
        return false;
    }

    file.write(static_cast<const char*>(data), size);
    file.close();

    LOG_INFO("Saved raw binary dump: " + filename + " (" + std::to_string(size) + " bytes)");
    return true;
}

void RawCameraInterface::convert16to8bit(const cv::Mat& src16, cv::Mat& dst8, bool autoScale) {
    if (src16.type() != CV_16UC1) {
        LOG_ERROR("Input must be CV_16UC1");
        return;
    }

    if (autoScale) {
        // Find min/max for auto-scaling
        double minVal, maxVal;
        cv::minMaxLoc(src16, &minVal, &maxVal);

        // Scale to 8-bit range preserving dynamic range
        double scale = 255.0 / (maxVal - minVal);
        src16.convertTo(dst8, CV_8UC1, scale, -minVal * scale);

        if (minVal > 0 || maxVal < 1023) {  // 10-bit max
            LOG_DEBUG("Auto-scaled from [" + std::to_string(minVal) + ", " +
                     std::to_string(maxVal) + "] to [0, 255]");
        }
    } else {
        // Simple bit shift (10-bit to 8-bit)
        src16.convertTo(dst8, CV_8UC1, 0.25);  // Divide by 4 (shift right by 2)
    }
}

// StereoRawCameraSystem implementation

StereoRawCameraSystem::StereoRawCameraSystem() {
    leftCamera_ = std::make_unique<RawCameraInterface>(1);  // Camera 1 = LEFT
    rightCamera_ = std::make_unique<RawCameraInterface>(0); // Camera 0 = RIGHT
}

bool StereoRawCameraSystem::initialize() {
    LOG_INFO("Initializing stereo RAW camera system");

    if (!leftCamera_->initialize()) {
        LOG_ERROR("Failed to initialize left camera");
        return false;
    }

    if (!rightCamera_->initialize()) {
        LOG_ERROR("Failed to initialize right camera");
        return false;
    }

    LOG_INFO("Stereo RAW camera system initialized");
    return true;
}

bool StereoRawCameraSystem::start() {
    LOG_INFO("Starting stereo RAW camera system");

    if (!leftCamera_->start()) {
        LOG_ERROR("Failed to start left camera");
        return false;
    }

    if (!rightCamera_->start()) {
        LOG_ERROR("Failed to start right camera");
        return false;
    }

    LOG_INFO("Stereo RAW camera system started");
    return true;
}

void StereoRawCameraSystem::stop() {
    LOG_INFO("Stopping stereo RAW camera system");
    leftCamera_->stop();
    rightCamera_->stop();
}

bool StereoRawCameraSystem::captureStereoRaw(StereoRawFrame& stereoFrame,
                                             const RawCameraInterface::ProcessingOptions& options,
                                             int timeoutMs) {
    // Capture from both cameras
    bool leftOk = leftCamera_->captureRawFrame(stereoFrame.left, options, timeoutMs);
    bool rightOk = rightCamera_->captureRawFrame(stereoFrame.right, options, timeoutMs);

    if (!leftOk || !rightOk) {
        LOG_ERROR("Failed to capture from one or both cameras");
        return false;
    }

    // Calculate sync error
    int64_t timeDiff = std::abs(static_cast<int64_t>(stereoFrame.left.timestampNs) -
                                static_cast<int64_t>(stereoFrame.right.timestampNs));
    stereoFrame.syncErrorMs = timeDiff / 1000000.0;  // Convert ns to ms
    stereoFrame.synchronized = (stereoFrame.syncErrorMs < 1.0);  // <1ms sync requirement

    // Update variance comparison
    if (options.computeVariance) {
        std::lock_guard<std::mutex> lock(compMutex_);
        varianceComp_.leftRawVariance = stereoFrame.left.variance;
        varianceComp_.rightRawVariance = stereoFrame.right.variance;
        // Note: Processed variance would come from standard pipeline for comparison
    }

    if (stereoFrame.syncErrorMs > 1.0) {
        LOG_WARNING("Sync error exceeds 1ms: " + std::to_string(stereoFrame.syncErrorMs) + " ms");
    }

    return true;
}

} // namespace camera
} // namespace unlook