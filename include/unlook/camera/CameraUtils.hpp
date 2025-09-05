#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <libcamera/libcamera.h>

namespace unlook {
namespace camera {

/**
 * Utility functions for camera operations
 */
class CameraUtils {
public:
    /**
     * Convert Bayer pattern buffer to grayscale
     * @param bayerData Raw Bayer data pointer
     * @param width Image width
     * @param height Image height
     * @param bayerFormat Bayer pattern format (e.g., SBGGR10)
     * @param output Output grayscale Mat
     */
    static bool bayerToGrayscale(const uint8_t* bayerData, 
                                  int width, 
                                  int height,
                                  const std::string& bayerFormat,
                                  cv::Mat& output);
    
    /**
     * Convert SBGGR10 packed format to grayscale
     * Specific optimization for IMX296 sensor output
     */
    static bool sbggr10ToGrayscale(const uint8_t* data,
                                    int width,
                                    int height,
                                    cv::Mat& output);
    
    /**
     * Convert libcamera FrameBuffer to OpenCV Mat
     */
    static bool frameBufferToMat(libcamera::FrameBuffer* buffer,
                                  int width,
                                  int height,
                                  cv::Mat& output);
    
    /**
     * Calculate frame timestamp in nanoseconds
     */
    static uint64_t getTimestampNs();
    
    /**
     * Convert nanoseconds to milliseconds
     */
    static double nsToMs(uint64_t ns) {
        return static_cast<double>(ns) / 1000000.0;
    }
    
    /**
     * Calculate sync error between two timestamps (ms)
     */
    static double calculateSyncError(uint64_t timestamp1Ns, uint64_t timestamp2Ns) {
        int64_t diff = static_cast<int64_t>(timestamp1Ns) - static_cast<int64_t>(timestamp2Ns);
        return std::abs(diff) / 1000000.0;  // Convert to ms
    }
    
    /**
     * List available cameras using libcamera
     */
    static std::vector<std::string> listCameras();
    
    /**
     * Get camera info string
     */
    static std::string getCameraInfo(libcamera::Camera* camera);
    
    /**
     * Validate IMX296 sensor presence
     */
    static bool validateIMX296Camera(int cameraId);
    
    /**
     * Format exposure time for display
     */
    static std::string formatExposure(double exposureUs);
    
    /**
     * Format gain value for display
     */
    static std::string formatGain(double gain);
    
    /**
     * Calculate histogram of grayscale image
     */
    static cv::Mat calculateHistogram(const cv::Mat& image);
    
    /**
     * Calculate image brightness (0-255)
     */
    static double calculateBrightness(const cv::Mat& image);
    
    /**
     * Apply gamma correction
     */
    static void applyGamma(cv::Mat& image, double gamma = 2.2);
    
    /**
     * Check if running on Raspberry Pi
     */
    static bool isRaspberryPi();
    
    /**
     * Get CPU temperature (Raspberry Pi)
     */
    static double getCPUTemperature();
    
    /**
     * Memory-map file for zero-copy access
     */
    static void* mmapFile(int fd, size_t size);
    
    /**
     * Unmap memory-mapped file
     */
    static void munmapFile(void* addr, size_t size);
    
    // Performance utilities
    
    /**
     * High-resolution timer for performance measurement
     */
    class Timer {
    public:
        Timer();
        void reset();
        double elapsedMs() const;
        double elapsedUs() const;
        
    private:
        std::chrono::steady_clock::time_point start_;
    };
    
    /**
     * FPS calculator
     */
    class FPSCounter {
    public:
        FPSCounter(int windowSize = 30);
        void update();
        double getFPS() const;
        void reset();
        
    private:
        std::vector<std::chrono::steady_clock::time_point> timestamps_;
        size_t windowSize_;
        mutable std::mutex mutex_;
    };
    
    // Debug utilities
    
    /**
     * Save image with timestamp
     */
    static bool saveDebugImage(const cv::Mat& image, const std::string& prefix);
    
    /**
     * Create debug overlay with camera info
     */
    static void addDebugOverlay(cv::Mat& image, 
                                 const std::string& cameraName,
                                 double fps,
                                 double exposure,
                                 double gain,
                                 double syncError = 0.0);
    
    /**
     * Log camera configuration
     */
    static void logCameraConfig(libcamera::CameraConfiguration* config);
    
    /**
     * Validate stereo pair synchronization
     */
    static bool validateStereoSync(uint64_t leftTimestamp,
                                    uint64_t rightTimestamp,
                                    double maxErrorMs = 1.0);
    
private:
    // Internal Bayer conversion functions
    static void convertBayerToGray_SBGGR10_optimized(const uint8_t* src,
                                                      uint8_t* dst,
                                                      int width,
                                                      int height);
    
    static void convertBayerToGray_generic(const uint8_t* src,
                                            uint8_t* dst,
                                            int width,
                                            int height,
                                            int bpp);
};

} // namespace camera
} // namespace unlook