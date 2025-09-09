#include <unlook/camera/CameraUtils.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/imgproc.hpp>
#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/property_ids.h>
#include <sys/mman.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <ctime>
#include <cstring>

namespace unlook {
namespace camera {

bool CameraUtils::bayerToGrayscale(const uint8_t* bayerData, 
                                    int width, 
                                    int height,
                                    const std::string& bayerFormat,
                                    cv::Mat& output) {
    if (!bayerData) {
        LOG_ERROR("Null Bayer data pointer");
        return false;
    }
    
    // Create output Mat
    output.create(height, width, CV_8UC1);
    
    if (bayerFormat == "YUV420") {
        // YUV420: Y plane is already grayscale, just copy it
        cv::Mat y_plane(height, width, CV_8UC1, const_cast<uint8_t*>(bayerData));
        output = y_plane.clone();
        return true;
    } else {
        // Generic Bayer conversion (fallback)
        convertBayerToGray_generic(bayerData, output.data, width, height, 10);
        return true;
    }
}

bool CameraUtils::sbggr10ToGrayscale(const uint8_t* data,
                                      int width,
                                      int height,
                                      cv::Mat& output) {
    if (!data) {
        LOG_ERROR("Null data pointer");
        return false;
    }
    
    output.create(height, width, CV_8UC1);
    
    // SBGGR10 packed format: 4 pixels in 5 bytes
    // Each pixel is 10 bits, packed into bytes
    convertBayerToGray_SBGGR10_optimized(data, output.data, width, height);
    
    return true;
}

void CameraUtils::convertBayerToGray_SBGGR10_optimized(const uint8_t* src,
                                                        uint8_t* dst,
                                                        int width,
                                                        int height) {
    // SBGGR10 packed format: 4 pixels (40 bits) packed in 5 bytes
    // Proper unpacking is REQUIRED - cannot treat as 8-bit data!
    
    const int pixels_per_pack = 4;
    const int bytes_per_pack = 5;
    const int total_pixels = width * height;
    
    // Temporary buffer for 10-bit unpacked values
    std::vector<uint16_t> unpacked(total_pixels);
    
    // First: Unpack SBGGR10 to 16-bit values
    for (int i = 0; i < total_pixels / pixels_per_pack; i++) {
        const uint8_t* pack = src + i * bytes_per_pack;
        uint16_t* out = unpacked.data() + i * pixels_per_pack;
        
        // Unpack 4 10-bit pixels from 5 bytes (working implementation)
        out[0] = ((uint16_t)pack[0] << 2) | (pack[1] >> 6);
        out[1] = (((uint16_t)(pack[1] & 0x3F)) << 4) | (pack[2] >> 4);
        out[2] = (((uint16_t)(pack[2] & 0x0F)) << 6) | (pack[3] >> 2);
        out[3] = (((uint16_t)(pack[3] & 0x03)) << 8) | pack[4];
    }
    
    // Second: Convert Bayer pattern to grayscale
    // SBGGR10 Bayer pattern (BGGR):
    // B G B G ...
    // G R G R ... 
    // B G B G ...
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = y * width + x;
            uint16_t pixel_value;
            
            // Extract green pixels directly, interpolate others
            if ((y % 2 == 0 && x % 2 == 1) || (y % 2 == 1 && x % 2 == 0)) {
                // This is a Green pixel - use directly
                pixel_value = unpacked[idx];
            } else {
                // Non-green pixel - interpolate from surrounding greens
                uint32_t sum = 0;
                int count = 0;
                
                // Check 4-connected green neighbors
                if (x > 0 && (y % 2 == 0 && (x-1) % 2 == 1) || (y % 2 == 1 && (x-1) % 2 == 0)) {
                    sum += unpacked[idx - 1]; count++;
                }
                if (x < width-1 && (y % 2 == 0 && (x+1) % 2 == 1) || (y % 2 == 1 && (x+1) % 2 == 0)) {
                    sum += unpacked[idx + 1]; count++;
                }
                if (y > 0 && ((y-1) % 2 == 0 && x % 2 == 1) || ((y-1) % 2 == 1 && x % 2 == 0)) {
                    sum += unpacked[idx - width]; count++;
                }
                if (y < height-1 && ((y+1) % 2 == 0 && x % 2 == 1) || ((y+1) % 2 == 1 && x % 2 == 0)) {
                    sum += unpacked[idx + width]; count++;
                }
                
                pixel_value = count > 0 ? sum / count : unpacked[idx];
            }
            
            // Convert 10-bit to 8-bit (shift right by 2)
            dst[idx] = (uint8_t)(pixel_value >> 2);
        }
    }
}

void CameraUtils::convertBayerToGray_generic(const uint8_t* src,
                                              uint8_t* dst,
                                              int width,
                                              int height,
                                              int bpp) {
    // Generic Bayer to grayscale conversion
    // Simple 2x2 averaging
    
    int bytesPerPixel = (bpp + 7) / 8;
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int srcIdx = (y * width + x) * bytesPerPixel;
            int dstIdx = y * width + x;
            
            // For now, just take the first byte (8 MSB of each pixel)
            dst[dstIdx] = src[srcIdx];
        }
    }
}

bool CameraUtils::frameBufferToMat(libcamera::FrameBuffer* buffer,
                                    int width,
                                    int height,
                                    cv::Mat& output) {
    if (!buffer) {
        LOG_ERROR("Null frame buffer");
        return false;
    }
    
    const libcamera::FrameBuffer::Plane &plane = buffer->planes()[0];
    
    // Memory map the buffer
    void *data = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    if (data == MAP_FAILED) {
        LOG_ERROR("Failed to mmap frame buffer");
        return false;
    }
    
    // Convert to grayscale
    bool result = sbggr10ToGrayscale(static_cast<const uint8_t*>(data), 
                                      width, height, output);
    
    // Unmap buffer
    munmap(data, plane.length);
    
    return result;
}

uint64_t CameraUtils::getTimestampNs() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

std::vector<std::string> CameraUtils::listCameras() {
    std::vector<std::string> cameraList;
    
    libcamera::CameraManager manager;
    int ret = manager.start();
    if (ret < 0) {
        LOG_ERROR("Failed to start camera manager");
        return cameraList;
    }
    
    auto cameras = manager.cameras();
    for (const auto& camera : cameras) {
        cameraList.push_back(camera->id());
    }
    
    manager.stop();
    
    return cameraList;
}

std::string CameraUtils::getCameraInfo(libcamera::Camera* camera) {
    if (!camera) {
        return "Invalid camera";
    }
    
    std::stringstream info;
    info << "Camera ID: " << camera->id() << "\n";
    
    const libcamera::ControlList &props = camera->properties();
    
    if (props.contains(libcamera::properties::Model.id())) {
        info << "Model: " << props.get(libcamera::properties::Model).value_or("Unknown") << "\n";
    }
    
    if (props.contains(libcamera::properties::Location.id())) {
        int location = props.get(libcamera::properties::Location).value_or(0);
        info << "Location: " << location << "\n";
    }
    
    if (props.contains(libcamera::properties::Rotation.id())) {
        int rotation = props.get(libcamera::properties::Rotation).value_or(0);
        info << "Rotation: " << rotation << "\n";
    }
    
    return info.str();
}

bool CameraUtils::validateIMX296Camera(int cameraId) {
    libcamera::CameraManager manager;
    int ret = manager.start();
    if (ret < 0) {
        LOG_ERROR("Failed to start camera manager");
        return false;
    }
    
    auto cameras = manager.cameras();
    if (static_cast<size_t>(cameraId) >= cameras.size()) {
        LOG_ERROR("Camera ID out of range");
        manager.stop();
        return false;
    }
    
    auto camera = cameras[cameraId];
    const libcamera::ControlList &props = camera->properties();
    
    bool isIMX296 = false;
    if (props.contains(libcamera::properties::Model.id())) {
        std::string model = props.get(libcamera::properties::Model).value_or("");
        isIMX296 = (model.find("imx296") != std::string::npos ||
                    model.find("IMX296") != std::string::npos);
    }
    
    manager.stop();
    
    return isIMX296;
}

std::string CameraUtils::formatExposure(double exposureUs) {
    std::stringstream ss;
    
    if (exposureUs < 1000) {
        ss << std::fixed << std::setprecision(1) << exposureUs << " μs";
    } else if (exposureUs < 1000000) {
        ss << std::fixed << std::setprecision(2) << (exposureUs / 1000.0) << " ms";
    } else {
        ss << std::fixed << std::setprecision(3) << (exposureUs / 1000000.0) << " s";
    }
    
    return ss.str();
}

std::string CameraUtils::formatGain(double gain) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << gain << "x";
    return ss.str();
}

cv::Mat CameraUtils::calculateHistogram(const cv::Mat& image) {
    cv::Mat hist;
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    
    cv::calcHist(&image, 1, nullptr, cv::Mat(), hist, 1, &histSize, &histRange);
    
    return hist;
}

double CameraUtils::calculateBrightness(const cv::Mat& image) {
    if (image.empty() || image.type() != CV_8UC1) {
        return 0.0;
    }
    
    cv::Scalar meanValue = cv::mean(image);
    return meanValue[0];
}

void CameraUtils::applyGamma(cv::Mat& image, double gamma) {
    cv::Mat lookupTable(1, 256, CV_8U);
    uchar* p = lookupTable.ptr();
    
    for (int i = 0; i < 256; ++i) {
        p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }
    
    cv::LUT(image, lookupTable, image);
}

bool CameraUtils::isRaspberryPi() {
    std::ifstream file("/proc/cpuinfo");
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        if (line.find("Raspberry Pi") != std::string::npos) {
            return true;
        }
    }
    
    return false;
}

double CameraUtils::getCPUTemperature() {
    std::ifstream file("/sys/class/thermal/thermal_zone0/temp");
    if (!file.is_open()) {
        return 0.0;
    }
    
    int temp;
    file >> temp;
    file.close();
    
    return temp / 1000.0;  // Convert from millidegrees to degrees
}

void* CameraUtils::mmapFile(int fd, size_t size) {
    void* addr = mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        LOG_ERROR("mmap failed: " + std::string(strerror(errno)));
        return nullptr;
    }
    return addr;
}

void CameraUtils::munmapFile(void* addr, size_t size) {
    if (addr && addr != MAP_FAILED) {
        munmap(addr, size);
    }
}

// Timer implementation
CameraUtils::Timer::Timer() {
    reset();
}

void CameraUtils::Timer::reset() {
    start_ = std::chrono::steady_clock::now();
}

double CameraUtils::Timer::elapsedMs() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_);
    return duration.count() / 1000.0;
}

double CameraUtils::Timer::elapsedUs() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_);
    return static_cast<double>(duration.count());
}

// FPSCounter implementation
CameraUtils::FPSCounter::FPSCounter(int windowSize) 
    : windowSize_(windowSize) {
    timestamps_.reserve(windowSize);
}

void CameraUtils::FPSCounter::update() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto now = std::chrono::steady_clock::now();
    timestamps_.push_back(now);
    
    if (timestamps_.size() > windowSize_) {
        timestamps_.erase(timestamps_.begin());
    }
}

double CameraUtils::FPSCounter::getFPS() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (timestamps_.size() < 2) {
        return 0.0;
    }
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        timestamps_.back() - timestamps_.front());
    
    if (duration.count() == 0) {
        return 0.0;
    }
    
    return (timestamps_.size() - 1) * 1000.0 / duration.count();
}

void CameraUtils::FPSCounter::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    timestamps_.clear();
}

bool CameraUtils::saveDebugImage(const cv::Mat& image, const std::string& prefix) {
    if (image.empty()) {
        return false;
    }
    
    // Generate timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream filename;
    filename << prefix << "_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".png";
    
    // NOTE: cv::imwrite requires opencv_imgcodecs module
    // For now, just return true as placeholder
    return true;
}

void CameraUtils::addDebugOverlay(cv::Mat& image, 
                                   const std::string& cameraName,
                                   double fps,
                                   double exposure,
                                   double gain,
                                   double syncError) {
    if (image.empty()) {
        return;
    }
    
    // Convert to color if grayscale
    cv::Mat displayImage;
    if (image.channels() == 1) {
        cv::cvtColor(image, displayImage, cv::COLOR_GRAY2BGR);
    } else {
        displayImage = image;
    }
    
    // Add text overlay
    int y = 30;
    int lineHeight = 25;
    cv::Scalar textColor(0, 255, 0);  // Green
    double fontScale = 0.6;
    int thickness = 1;
    
    cv::putText(displayImage, cameraName, cv::Point(10, y), 
                cv::FONT_HERSHEY_SIMPLEX, fontScale, textColor, thickness);
    y += lineHeight;
    
    std::stringstream ss;
    ss << "FPS: " << std::fixed << std::setprecision(1) << fps;
    cv::putText(displayImage, ss.str(), cv::Point(10, y), 
                cv::FONT_HERSHEY_SIMPLEX, fontScale, textColor, thickness);
    y += lineHeight;
    
    ss.str("");
    ss << "Exp: " << formatExposure(exposure);
    cv::putText(displayImage, ss.str(), cv::Point(10, y), 
                cv::FONT_HERSHEY_SIMPLEX, fontScale, textColor, thickness);
    y += lineHeight;
    
    ss.str("");
    ss << "Gain: " << formatGain(gain);
    cv::putText(displayImage, ss.str(), cv::Point(10, y), 
                cv::FONT_HERSHEY_SIMPLEX, fontScale, textColor, thickness);
    
    if (syncError > 0) {
        y += lineHeight;
        ss.str("");
        ss << "Sync: " << std::fixed << std::setprecision(2) << syncError << " ms";
        cv::Scalar syncColor = (syncError <= 1.0) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
        cv::putText(displayImage, ss.str(), cv::Point(10, y), 
                    cv::FONT_HERSHEY_SIMPLEX, fontScale, syncColor, thickness);
    }
    
    // Copy back if needed
    if (image.channels() == 1) {
        cv::cvtColor(displayImage, image, cv::COLOR_BGR2GRAY);
    } else {
        image = displayImage;
    }
}

void CameraUtils::logCameraConfig(libcamera::CameraConfiguration* config) {
    if (!config) {
        return;
    }
    
    LOG_INFO("Camera configuration:");
    for (unsigned int i = 0; i < config->size(); i++) {
        const libcamera::StreamConfiguration &cfg = config->at(i);
        LOG_INFO("  Stream " + std::to_string(i) + ":");
        LOG_INFO("    Size: " + std::to_string(cfg.size.width) + "x" + 
                 std::to_string(cfg.size.height));
        LOG_INFO("    Format: " + cfg.pixelFormat.toString());
        LOG_INFO("    Buffers: " + std::to_string(cfg.bufferCount));
    }
}

bool CameraUtils::validateStereoSync(uint64_t leftTimestamp,
                                      uint64_t rightTimestamp,
                                      double maxErrorMs) {
    double syncError = calculateSyncError(leftTimestamp, rightTimestamp);
    
    if (syncError > maxErrorMs) {
        LOG_WARNING("Stereo sync error: " + std::to_string(syncError) + " ms");
        return false;
    }
    
    return true;
}

} // namespace camera
} // namespace unlook