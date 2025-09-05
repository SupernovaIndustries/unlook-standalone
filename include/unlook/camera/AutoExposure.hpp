#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <opencv2/core.hpp>
#include <functional>

namespace unlook {
namespace camera {

/**
 * Automatic exposure control system
 * 
 * Implements histogram-based auto-exposure with smooth adjustments
 * for optimal image quality in varying lighting conditions.
 */
class AutoExposure {
public:
    /**
     * Auto-exposure algorithm type
     */
    enum class Algorithm {
        HISTOGRAM_MEAN,      // Target mean brightness
        HISTOGRAM_PERCENTILE, // Target percentile brightness
        SPOT_METERING,       // Center-weighted metering
        MATRIX_METERING      // Multi-zone metering
    };
    
    /**
     * Auto-exposure configuration
     */
    struct Config {
        Algorithm algorithm = Algorithm::HISTOGRAM_PERCENTILE;
        
        // Target brightness (0-255)
        int targetBrightness = 80;  // REDUCED for indoor lighting
        int targetPercentile = 50;  // For percentile algorithm
        
        // Adjustment parameters - FASTER convergence
        double adjustmentSpeed = 0.5;  // INCREASED for faster response
        double exposureStep = 1.3;     // Larger steps for faster convergence
        double gainStep = 1.15;        // Slightly larger gain steps
        
        // Limits - OPTIMIZED for quality and speed
        double minExposureUs = 100.0;
        double maxExposureUs = 20000.0;  // REDUCED: 20ms for faster response
        double minGain = 1.0;
        double maxGain = 8.0;  // REDUCED: 8x max to limit noise
        
        // Metering region (normalized 0-1)
        cv::Rect2d meteringRegion = cv::Rect2d(0.25, 0.25, 0.5, 0.5);
        
        // Convergence
        int convergenceFrames = 10;  // Frames to consider converged
        double convergenceThreshold = 5.0;  // Brightness difference threshold
    };
    
    /**
     * Exposure parameters result
     */
    struct ExposureParams {
        double exposureUs;
        double analogGain;
        double digitalGain;
        bool isConverged;
        double currentBrightness;
        double targetBrightness;
    };
    
    /**
     * Callback for exposure updates
     */
    using UpdateCallback = std::function<void(const ExposureParams&)>;
    
    /**
     * Constructor
     */
    AutoExposure();
    
    /**
     * Destructor
     */
    ~AutoExposure();
    
    /**
     * Initialize with configuration
     */
    bool initialize(const Config& config);
    
    /**
     * Initialize with default configuration
     */
    bool initialize();
    
    /**
     * Shutdown auto-exposure
     */
    void shutdown();
    
    /**
     * Start auto-exposure thread
     */
    bool start();
    
    /**
     * Stop auto-exposure thread
     */
    void stop();
    
    /**
     * Process frame and calculate new exposure
     * @param frame Input grayscale frame
     * @return Calculated exposure parameters
     */
    ExposureParams processFrame(const cv::Mat& frame);
    
    /**
     * Process stereo pair (uses average brightness)
     */
    ExposureParams processStereo(const cv::Mat& leftFrame, const cv::Mat& rightFrame);
    
    /**
     * Set update callback
     */
    void setUpdateCallback(UpdateCallback callback);
    
    /**
     * Get current configuration
     */
    Config getConfig() const;
    
    /**
     * Update configuration
     */
    void setConfig(const Config& config);
    
    /**
     * Get current exposure parameters
     */
    ExposureParams getCurrentParams() const;
    
    /**
     * Check if converged
     */
    bool isConverged() const { return converged_; }
    
    /**
     * Reset convergence state
     */
    void resetConvergence();
    
    /**
     * Enable/disable auto-exposure
     */
    void setEnabled(bool enable) { enabled_ = enable; }
    
    /**
     * Check if enabled
     */
    bool isEnabled() const { return enabled_; }
    
    // Algorithm-specific methods
    
    /**
     * Set target brightness (0-255)
     */
    void setTargetBrightness(int brightness);
    
    /**
     * Set metering region (normalized coordinates)
     */
    void setMeteringRegion(const cv::Rect2d& region);
    
    /**
     * Set exposure limits
     */
    void setExposureLimits(double minUs, double maxUs);
    
    /**
     * Set gain limits
     */
    void setGainLimits(double minGain, double maxGain);
    
    /**
     * Set adjustment speed (0-1, higher = faster)
     */
    void setAdjustmentSpeed(double speed);
    
private:
    // Processing thread
    void processingThread();
    
    // Brightness calculation methods
    double calculateBrightness(const cv::Mat& frame);
    double calculateHistogramMean(const cv::Mat& frame);
    double calculateHistogramPercentile(const cv::Mat& frame, int percentile);
    double calculateSpotMetering(const cv::Mat& frame);
    double calculateMatrixMetering(const cv::Mat& frame);
    
    // Exposure adjustment
    ExposureParams adjustExposure(double currentBrightness);
    double smoothAdjustment(double current, double target, double speed);
    
    // Convergence tracking
    void updateConvergence(double brightness);
    
    // Member variables
    Config config_;
    ExposureParams currentParams_;
    
    // State
    std::atomic<bool> initialized_{false};
    std::atomic<bool> enabled_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> converged_{false};
    
    // Threading
    std::thread processingThread_;
    std::atomic<bool> shouldStop_{false};
    
    // Frame queue
    std::mutex frameMutex_;
    cv::Mat pendingFrame_;
    bool frameAvailable_ = false;
    std::condition_variable frameCondition_;
    
    // Callback
    std::mutex callbackMutex_;
    UpdateCallback updateCallback_;
    
    // Convergence tracking
    std::vector<double> brightnessHistory_;
    int convergenceCounter_ = 0;
    
    // Statistics
    std::atomic<uint64_t> framesProcessed_{0};
    std::atomic<double> avgProcessingTimeMs_{0.0};
};

} // namespace camera
} // namespace unlook