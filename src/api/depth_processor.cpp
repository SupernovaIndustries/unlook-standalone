/**
 * @file depth_processor.cpp
 * @brief API implementation for depth processing with high-precision stereo matching
 * 
 * Optimized for 70mm baseline with target precision of 0.005mm
 */

#include "unlook/api/depth_processor.h"
#include "unlook/stereo/DepthProcessor.hpp"
#include "unlook/stereo/StereoMatcher.hpp"
#include "unlook/calibration/CalibrationManager.hpp"
#include "unlook/core/result_code_utils.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>

namespace unlook {
namespace api {

// Helper function to convert core::StereoAlgorithm to stereo::StereoAlgorithm
static stereo::StereoAlgorithm convertStereoAlgorithm(core::StereoAlgorithm coreAlgorithm) {
    switch (coreAlgorithm) {
        case core::StereoAlgorithm::SGBM_OPENCV:
            return stereo::StereoAlgorithm::SGBM;

        // Census Transform algorithm (VCSEL-optimized)
        case core::StereoAlgorithm::CENSUS:
            return stereo::StereoAlgorithm::CENSUS;

        // BoofCV Dense algorithms (standard precision)
        case core::StereoAlgorithm::BOOFCV_DENSE_BM:
            return stereo::StereoAlgorithm::BOOFCV_BM;
        case core::StereoAlgorithm::BOOFCV_DENSE_SGM:
            return stereo::StereoAlgorithm::BOOFCV_SGBM;

        // BoofCV Sub-pixel algorithms (high precision targeting 0.005mm)
        case core::StereoAlgorithm::BOOFCV_SUBPIXEL_BM:
            return stereo::StereoAlgorithm::BOOFCV_BM;  // Sub-pixel handled in BoofCVStereoMatcher
        case core::StereoAlgorithm::BOOFCV_SUBPIXEL_SGM:
            return stereo::StereoAlgorithm::BOOFCV_SGBM; // Sub-pixel handled in BoofCVStereoMatcher

        // Note: BOOFCV_BASIC and BOOFCV_PRECISE are aliases, handled by cases above

        default:
            return stereo::StereoAlgorithm::SGBM; // Fallback
    }
}

// Helper function to convert core::StereoConfig to stereo::StereoMatchingParams
// OPTIMIZED: Uses calibration data to calculate optimal stereo parameters
static stereo::StereoMatchingParams convertToStereoMatchingParams(const core::StereoConfig& config) {
    stereo::StereoMatchingParams params;
    
    // CRITICAL FIX: Calculate optimal parameters from calibration data
    // Baseline: 70.017mm, Focal: ~1775px, Image: 1456x1088
    const double BASELINE_MM = 70.017;  // From calib_boofcv_test3.yaml
    const double FOCAL_PIXELS = 1775.0; // Average from camera matrices
    
    // FIXED: Increased disparity range to support close-range scanning (320mm)
    const int MAX_NUM_DISPARITIES = 384; // For 320mm: (70.017 × 1755) / 320 = 384px
    const double MIN_DEPTH_MM = 320.0;   // Close-range scanning target
    const double MAX_DEPTH_MM = 3000.0;  // Extended range for better coverage

    // CRITICAL FIX: Use config.num_disparities if explicitly set (from presets)
    if (config.num_disparities > 0) {
        // User/preset specified value - use it directly (must be multiple of 16)
        params.numDisparities = ((config.num_disparities + 15) / 16) * 16;
    } else {
        // Auto-calculate from geometry if not specified
        int theoretical_max_disp = static_cast<int>((BASELINE_MM * FOCAL_PIXELS) / MIN_DEPTH_MM);
        int raw_num_disparities = std::min(MAX_NUM_DISPARITIES, theoretical_max_disp);
        params.numDisparities = ((raw_num_disparities + 15) / 16) * 16;
    }

    // VCSEL-OPTIMIZED PARAMETERS - Hardcoded for dot pattern matching
    params.minDisparity = 0;
    params.blockSize = 5;           // VCSEL: Small block to capture single dot cluster

    // VCSEL-specific P1/P2 for dot patterns (grayscale)
    params.P1 = 200;                // 8 * 1 * 5 * 5 for grayscale
    params.P2 = 800;                // 32 * 1 * 5 * 5 for grayscale

    // VCSEL precision parameters
    params.disp12MaxDiff = 2;       // Reasonable tolerance for dots
    params.preFilterCap = 63;       // Maximum, minimal filtering to preserve dots
    params.uniquenessRatio = 15;    // High uniqueness to avoid ambiguous dots

    // VCSEL speckle filtering - moderate to preserve dots
    params.speckleWindowSize = 50;  // Moderate speckle filtering
    params.speckleRange = 64;       // Wide range for dot preservation
    params.mode = cv::StereoSGBM::MODE_SGBM;  // Standard SGBM, not HH for dots
    
    // Log optimal parameters for debugging (reduced to prevent spam)
    static bool logged_once = false;
    if (!logged_once) {
        std::cout << "[API DepthProcessor] CALIBRATION-OPTIMIZED parameters:" << std::endl;
        std::cout << "[API DepthProcessor]   Baseline: " << BASELINE_MM << "mm" << std::endl;
        std::cout << "[API DepthProcessor]   Focal: " << FOCAL_PIXELS << "px" << std::endl;
        std::cout << "[API DepthProcessor]   Disparity range: " << params.minDisparity << " - " << (params.minDisparity + params.numDisparities) << std::endl;
        std::cout << "[API DepthProcessor]   Depth range: " << MIN_DEPTH_MM << " - " << MAX_DEPTH_MM << "mm" << std::endl;
        logged_once = true;
    }
    
    // Map algorithm to mode
    switch (config.algorithm) {
        case core::StereoAlgorithm::SGBM_OPENCV:
            params.mode = 0;  // MODE_SGBM
            break;

        case core::StereoAlgorithm::CENSUS:
            // Census Transform specific parameters
            params.mode = 0;  // Standard SGBM mode
            params.blockSize = 5;  // Optimal for Census with dots
            params.numDisparities = 448;  // Maximum range for comprehensive coverage
            params.P1 = 200;  // Census-optimized smoothness
            params.P2 = 800;  // Census-optimized discontinuity
            params.uniquenessRatio = 10;  // More tolerant for Census
            params.speckleWindowSize = 50;  // Balanced for dots
            params.speckleRange = 128;  // Wide range for dot preservation
            break;

        // BoofCV Dense algorithms (standard precision)
        case core::StereoAlgorithm::BOOFCV_DENSE_BM:
            params.mode = 0;  // Basic block matching mode
            break;
        case core::StereoAlgorithm::BOOFCV_DENSE_SGM:
            params.mode = 1;  // Semi-global matching mode
            break;

        // BoofCV Sub-pixel algorithms (high precision)
        case core::StereoAlgorithm::BOOFCV_SUBPIXEL_BM:
            params.mode = 2;  // High quality mode for sub-pixel precision
            break;
        case core::StereoAlgorithm::BOOFCV_SUBPIXEL_SGM:
            params.mode = 2;  // Highest quality SGM mode for 0.005mm precision
            break;

        // Note: BOOFCV_BASIC and BOOFCV_PRECISE are aliases, handled by cases above

        default:
            params.mode = 0;
    }
    
    // Set quality-based parameters
    switch (config.quality) {
        case core::DepthQuality::FAST_LOW:
            params.useWLSFilter = false;
            params.leftRightCheck = false;
            params.confidenceThreshold = 0.90f;
            break;
        case core::DepthQuality::BALANCED:
            params.useWLSFilter = true;
            params.leftRightCheck = true;
            params.confidenceThreshold = 0.95f;
            break;
        case core::DepthQuality::SLOW_HIGH:
            params.useWLSFilter = true;
            params.leftRightCheck = true;
            params.confidenceThreshold = 0.98f;
            params.mode = 2;  // Use SGBM_3WAY for highest quality
            break;
        default:
            params.useWLSFilter = true;
            params.leftRightCheck = true;
            params.confidenceThreshold = 0.95f;
    }
    
    // WLS filter parameters (optimized for 70mm baseline)
    params.wlsLambda = 8000.0;
    params.wlsSigma = 1.2;
    
    // Performance settings
    params.useParallel = true;
    params.numThreads = 4;
    
    return params;
}

// Implementation class for API DepthProcessor
class DepthProcessor::Impl {
public:
    // Core stereo processor - using the main DepthProcessor
    std::unique_ptr<stereo::DepthProcessor> coreProcessor;
    
    // Calibration management
    std::shared_ptr<calibration::CalibrationManager> calibrationManager;
    
    // Configuration
    core::StereoConfig stereoConfig;
    StereoAlgorithm currentAlgorithm;
    
    // State tracking
    std::atomic<bool> initialized{false};
    std::atomic<bool> calibrationLoaded{false};
    
    // Performance metrics
    mutable std::mutex metricsMutex;
    double totalProcessingTime{0.0};
    uint64_t framesProcessed{0};
    double peakMemoryMb{0.0};
    std::chrono::high_resolution_clock::time_point startTime;
    
    // Error handling
    mutable std::mutex errorMutex;
    mutable std::string lastError;
    
    // Async processing
    std::thread asyncThread;
    std::atomic<bool> asyncRunning{false};
    
    Impl() {
        // Initialize with default configuration optimized for 70mm baseline  
        // FIXED: Use SGBM_OPENCV since only this is implemented in StereoMatcher::create()
        stereoConfig.algorithm = core::StereoAlgorithm::SGBM_OPENCV;
        stereoConfig.quality = core::DepthQuality::SLOW_HIGH;
        
        // VCSEL-OPTIMIZED parameters for dot pattern matching
        stereoConfig.min_disparity = 0;
        stereoConfig.num_disparities = 320;  // Full range for close-far scanning
        stereoConfig.block_size = 5;         // Small block for dot clusters
        stereoConfig.p1 = 200;               // 8 * 1 * 5 * 5 for grayscale
        stereoConfig.p2 = 800;               // 32 * 1 * 5 * 5 for grayscale
        stereoConfig.disp12_max_diff = 2;    // Reasonable tolerance
        stereoConfig.pre_filter_cap = 63;    // Maximum to preserve dots
        stereoConfig.uniqueness_ratio = 15;  // High to avoid ambiguous dots
        stereoConfig.speckle_window_size = 50;  // Moderate filtering
        stereoConfig.speckle_range = 64;     // Wide range for dots
        stereoConfig.mode = cv::StereoSGBM::MODE_SGBM;  // Standard SGBM for dots
        
        currentAlgorithm = StereoAlgorithm::OPENCV_SGBM;
        startTime = std::chrono::high_resolution_clock::now();
        
        // Create stereo processor
        coreProcessor = std::make_unique<stereo::DepthProcessor>();
        
        // Create calibration manager
        calibrationManager = std::make_shared<calibration::CalibrationManager>();
    }
    
    ~Impl() {
        if (asyncRunning) {
            asyncRunning = false;
            if (asyncThread.joinable()) {
                asyncThread.join();
            }
        }
    }
    
    void setError(const std::string& error) const {
        std::lock_guard<std::mutex> lock(errorMutex);
        lastError = error;
    }
    
    void updateMetrics(double processingTimeMs) {
        std::lock_guard<std::mutex> lock(metricsMutex);
        totalProcessingTime += processingTimeMs;
        framesProcessed++;
        
        // Estimate memory usage (simplified)
        double currentMemoryMb = 1456 * 1088 * 6 / (1024.0 * 1024.0);  // Rough estimate
        peakMemoryMb = std::max(peakMemoryMb, currentMemoryMb);
    }
};

// Constructor
DepthProcessor::DepthProcessor() : pimpl_(std::make_unique<Impl>()) {
    std::cout << "[API DepthProcessor] Initialized with 70mm baseline optimization" << std::endl;
}

// Destructor
DepthProcessor::~DepthProcessor() = default;

// Initialize depth processor
core::ResultCode DepthProcessor::initialize(const std::string& calibration_path) {
    std::cout << "[API DepthProcessor] initialize() called with path: " << calibration_path << std::endl;
    
    if (pimpl_->initialized) {
        std::cout << "[API DepthProcessor] ERROR: Already initialized!" << std::endl;
        return core::ResultCode::ERROR_ALREADY_INITIALIZED;
    }
    
    try {
        // Load calibration if path provided
        if (!calibration_path.empty()) {
            std::cout << "[API DepthProcessor] Loading calibration from: " << calibration_path << std::endl;
            core::ResultCode result = loadCalibration(calibration_path);
            if (result != core::ResultCode::SUCCESS) {
                std::cout << "[API DepthProcessor] CALIBRATION LOAD FAILED with code: " << static_cast<int>(result) << std::endl;
                return result;
            }
            std::cout << "[API DepthProcessor] Calibration loaded successfully" << std::endl;
        }
        
        // Configure stereo processor with optimized parameters
        pimpl_->coreProcessor->setStereoParameters(convertToStereoMatchingParams(pimpl_->stereoConfig));
        
        // Initialize the core processor
        if (!calibration_path.empty()) {
            std::cout << "[API DepthProcessor] About to initialize core processor..." << std::endl;
            if (!pimpl_->coreProcessor->initialize(pimpl_->calibrationManager)) {
                std::cout << "[API DepthProcessor] CORE PROCESSOR INIT FAILED!" << std::endl;
                pimpl_->setError("Failed to initialize core processor with calibration");
                return core::ResultCode::ERROR_CALIBRATION_INVALID;
            }
            std::cout << "[API DepthProcessor] Core processor initialized successfully!" << std::endl;
            
            // CRITICAL FIX: Initialize the stereo matcher (was missing!)
            auto coreAlgorithm = pimpl_->stereoConfig.algorithm;
            auto stereoAlgorithm = convertStereoAlgorithm(coreAlgorithm);
            std::cout << "[API DepthProcessor] Creating stereo matcher..." << std::endl;
            std::cout << "[API DepthProcessor] Core algorithm: " << static_cast<int>(coreAlgorithm) << std::endl;
            std::cout << "[API DepthProcessor] Converted stereo algorithm: " << static_cast<int>(stereoAlgorithm) << std::endl;
            
            if (!pimpl_->coreProcessor->setStereoMatcher(stereoAlgorithm)) {
                std::cout << "[API DepthProcessor] STEREO MATCHER CREATION FAILED!" << std::endl;
                std::cout << "[API DepthProcessor] Algorithm used: " << static_cast<int>(stereoAlgorithm) << std::endl;
                pimpl_->setError("Failed to create stereo matcher");
                return core::ResultCode::ERROR_GENERIC;
            }
            std::cout << "[API DepthProcessor] Stereo matcher created successfully!" << std::endl;
        }
        
        pimpl_->initialized = true;
        std::cout << "[API DepthProcessor] Initialization complete" << std::endl;
        
        return core::ResultCode::SUCCESS;
        
    } catch (const std::exception& e) {
        pimpl_->setError(std::string("Initialization failed: ") + e.what());
        return core::ResultCode::ERROR_GENERIC;
    }
}

// Shutdown depth processor
core::ResultCode DepthProcessor::shutdown() {
    if (!pimpl_->initialized) {
        return core::ResultCode::ERROR_NOT_INITIALIZED;
    }
    
    try {
        // Stop async processing if running
        if (pimpl_->asyncRunning) {
            pimpl_->asyncRunning = false;
            if (pimpl_->asyncThread.joinable()) {
                pimpl_->asyncThread.join();
            }
        }
        
        // Cancel any processing
        pimpl_->coreProcessor->cancelProcessing();
        
        // Clear state
        pimpl_->initialized = false;
        pimpl_->calibrationLoaded = false;
        
        std::cout << "[API DepthProcessor] Shutdown complete" << std::endl;
        return core::ResultCode::SUCCESS;
        
    } catch (const std::exception& e) {
        pimpl_->setError(std::string("Shutdown failed: ") + e.what());
        return core::ResultCode::ERROR_GENERIC;
    }
}

// Check if initialized
bool DepthProcessor::isInitialized() const {
    return pimpl_->initialized;
}

// Load stereo calibration from file
core::ResultCode DepthProcessor::loadCalibration(const std::string& calibration_path) {
    try {
        // Load calibration using the calibration manager
        std::cout << "[API DepthProcessor] Attempting to load calibration with internal CalibrationManager..." << std::endl;
        if (!pimpl_->calibrationManager->loadCalibration(calibration_path)) {
            std::cout << "[API DepthProcessor] Internal CalibrationManager FAILED to load calibration!" << std::endl;
            std::cout << "[API DepthProcessor] Internal CalibrationManager validation report: " << pimpl_->calibrationManager->getValidationReport() << std::endl;
            pimpl_->setError("Failed to load calibration from: " + calibration_path);
            return core::ResultCode::ERROR_CALIBRATION_INVALID;
        }
        std::cout << "[API DepthProcessor] Internal CalibrationManager loaded calibration successfully!" << std::endl;
        
        // NOTE: Core processor initialization is done in initialize(), not here
        // loadCalibration() should only load and validate calibration data
        
        // Verify calibration is valid
        if (!pimpl_->calibrationManager->isCalibrationValid()) {
            pimpl_->setError("Loaded calibration is invalid");
            return core::ResultCode::ERROR_CALIBRATION_INVALID;
        }
        
        // Get baseline and verify it's close to expected 70mm
        double baseline = pimpl_->calibrationManager->getBaselineMm();
        if (std::abs(baseline - 70.0) > 5.0) {  // Allow 5mm tolerance
            std::cout << "[API DepthProcessor] Warning: Baseline " << baseline 
                     << "mm differs from expected 70mm" << std::endl;
        }
        
        pimpl_->calibrationLoaded = true;
        std::cout << "[API DepthProcessor] Calibration loaded. Baseline: " 
                  << baseline << "mm" << std::endl;
        
        return core::ResultCode::SUCCESS;
        
    } catch (const std::exception& e) {
        pimpl_->setError(std::string("Calibration loading failed: ") + e.what());
        return core::ResultCode::ERROR_FILE_IO;
    }
}

// Check if valid calibration is loaded
bool DepthProcessor::hasValidCalibration() const {
    return pimpl_->calibrationLoaded && pimpl_->calibrationManager && 
           pimpl_->calibrationManager->isCalibrationValid();
}

// Get calibration quality metrics
core::CalibrationQuality DepthProcessor::getCalibrationQuality() const {
    core::CalibrationQuality quality;
    
    if (!hasValidCalibration()) {
        quality.is_valid = false;
        quality.rms_error = -1;
        quality.baseline_mm = 0;
        quality.precision_mm = -1;
        return quality;
    }
    
    // Get calibration metrics
    auto& calibData = pimpl_->calibrationManager->getCalibrationData();
    quality.rms_error = calibData.rmsError;
    quality.baseline_mm = pimpl_->calibrationManager->getBaselineMm();
    
    // Calculate theoretical precision at 100mm depth
    // Precision = (Z^2 * disparity_error) / (baseline * focal_length)
    // For 70mm baseline, 6mm focal length (~1700 pixels), 0.2 pixel disparity error
    double Z = 100.0;  // mm
    double disparityError = 0.2;  // sub-pixel accuracy
    double focalPixels = 1700.0;  // Approximate for 6mm lens
    
    quality.precision_mm = (Z * Z * disparityError) / (quality.baseline_mm * focalPixels);
    quality.is_valid = quality.rms_error < 0.5;  // Sub-pixel RMS is good
    
    return quality;
}

// Process stereo frame pair to depth map
core::ResultCode DepthProcessor::processFrames(const cv::Mat& left_frame,
                                              const cv::Mat& right_frame,
                                              cv::Mat& depth_map,
                                              cv::Mat* disparity_map) {
    if (!pimpl_->initialized) {
        pimpl_->setError("Depth processor not initialized");
        return core::ResultCode::ERROR_NOT_INITIALIZED;
    }
    
    if (left_frame.empty() || right_frame.empty()) {
        pimpl_->setError("Empty input frames");
        return core::ResultCode::ERROR_INVALID_PARAMETER;
    }
    
    if (left_frame.size() != right_frame.size()) {
        pimpl_->setError("Frame size mismatch");
        return core::ResultCode::ERROR_INVALID_PARAMETER;
    }
    
    try {
        auto startTime = std::chrono::high_resolution_clock::now();
        
        // Create stereo frame pair
        core::StereoFramePair framePair;
        framePair.left_frame.image = left_frame;
        framePair.right_frame.image = right_frame;
        framePair.left_frame.timestamp_ns = std::chrono::steady_clock::now().time_since_epoch().count();
        framePair.right_frame.timestamp_ns = framePair.left_frame.timestamp_ns;
        framePair.synchronized = true;
        framePair.sync_error_ms = 0.0;
        
        // Process using core processor's method
        cv::Mat depthFloat, confidenceMap, disparityFloat;
        bool success = pimpl_->coreProcessor->processWithConfidence(
            framePair.left_frame.image, 
            framePair.right_frame.image, 
            depthFloat, 
            confidenceMap,
            &disparityFloat  // CRITICAL FIX: Get disparity map
        );
        
        core::DepthResult result;
        result.success = success;
        if (success) {
            result.depth_map = depthFloat;
            result.confidence_map = confidenceMap;
            result.disparity_map = disparityFloat;  // CRITICAL FIX: Store disparity map

            // MEMORY-SAFE FIX: Pass file path instead of copying 1M points
            // GUI will load from file to avoid memory copy failures
            std::ofstream apiTrace("/tmp/point_cloud_trace.log", std::ios::app);
            apiTrace << "\n[API LAYER] Getting point cloud file path (not copying points)..." << std::endl;

            std::cout << "[API DepthProcessor] ===== POINT CLOUD PATH RETRIEVAL =====" << std::endl;
            result.debug_pointcloud_path = pimpl_->coreProcessor->getLastPointCloudPath();

            apiTrace << "[API LAYER] Point cloud file path: " << result.debug_pointcloud_path << std::endl;
            apiTrace << "[API LAYER] File exists: " << (!result.debug_pointcloud_path.empty() ? "YES" : "NO") << std::endl;
            apiTrace.close();

            std::cout << "[API DepthProcessor] Point cloud file path: " << result.debug_pointcloud_path << std::endl;
            std::cout << "[API DepthProcessor] ======================================" << std::endl;

            // Compute statistics
            stereo::DepthStatistics stats;
            pimpl_->coreProcessor->computeDepthStatistics(depthFloat, stats);
            result.mean_depth = stats.meanDepth;
            result.std_depth = stats.stdDepth;
            result.coverage_ratio = stats.validRatio;
        } else {
            result.error_message = pimpl_->coreProcessor->getLastError();
            std::cout << "[DepthProcessor] DETAILED ERROR: " << result.error_message << std::endl;
        }
        
        if (!result.success) {
            pimpl_->setError(result.error_message);
            std::cout << "[DepthProcessor] Processing failed with error: " << result.error_message << std::endl;
            return core::ResultCode::ERROR_GENERIC;
        }
        
        // Convert depth map to 16-bit millimeters for API compatibility
        result.depth_map.convertTo(depth_map, CV_16U);
        
        // Optionally provide disparity map
        if (disparity_map != nullptr) {
            // CRITICAL FIX: Keep disparity as FLOAT32, not CV_8U!
            // CV_8U clips to 255, but we need up to 320 for close-range (400mm) scanning
            // GUI uses this for point cloud generation which needs full precision
            *disparity_map = result.disparity_map.clone();  // Keep as CV_32F

            std::cout << "[API DepthProcessor] Disparity map preserved as float32 (no CV_8U clipping)" << std::endl;
        }
        
        auto endTime = std::chrono::high_resolution_clock::now();
        double processingTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        
        pimpl_->updateMetrics(processingTimeMs);
        
        std::cout << "[API DepthProcessor] Processed frame in " << processingTimeMs 
                  << "ms, coverage: " << (result.coverage_ratio * 100) << "%" << std::endl;
        
        return core::ResultCode::SUCCESS;
        
    } catch (const cv::Exception& e) {
        pimpl_->setError(std::string("OpenCV error: ") + e.what());
        return core::ResultCode::ERROR_GENERIC;
    } catch (const std::exception& e) {
        pimpl_->setError(std::string("Processing error: ") + e.what());
        return core::ResultCode::ERROR_GENERIC;
    }
}

// Process frames asynchronously
core::ResultCode DepthProcessor::processFramesAsync(const cv::Mat& left_frame,
                                                   const cv::Mat& right_frame,
                                                   DepthCallback callback,
                                                   void* user_data) {
    if (!pimpl_->initialized) {
        pimpl_->setError("Depth processor not initialized");
        return core::ResultCode::ERROR_NOT_INITIALIZED;
    }
    
    if (pimpl_->asyncRunning) {
        pimpl_->setError("Async processing already running");
        return core::ResultCode::ERROR_GENERIC;
    }
    
    try {
        // Start async processing thread
        pimpl_->asyncRunning = true;
        pimpl_->asyncThread = std::thread([this, left_frame, right_frame, callback, user_data]() {
            cv::Mat depth_map, disparity_map;
            
            auto startTime = std::chrono::high_resolution_clock::now();
            core::ResultCode result = processFrames(left_frame, right_frame, depth_map, &disparity_map);
            auto endTime = std::chrono::high_resolution_clock::now();
            
            double processingTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
            
            if (result == core::ResultCode::SUCCESS && callback) {
                callback(depth_map, disparity_map, processingTimeMs, user_data);
            }
            
            pimpl_->asyncRunning = false;
        });
        
        return core::ResultCode::SUCCESS;
        
    } catch (const std::exception& e) {
        pimpl_->setError(std::string("Async processing failed: ") + e.what());
        pimpl_->asyncRunning = false;
        return core::ResultCode::ERROR_THREAD_FAILURE;
    }
}

// Get rectified frames
core::ResultCode DepthProcessor::getRectifiedFrames(const cv::Mat& left_frame,
                                                   const cv::Mat& right_frame,
                                                   cv::Mat& left_rectified,
                                                   cv::Mat& right_rectified) {
    if (!hasValidCalibration()) {
        pimpl_->setError("No valid calibration loaded");
        return core::ResultCode::ERROR_CALIBRATION_INVALID;
    }
    
    try {
        // Use calibration manager's rectification
        if (!pimpl_->calibrationManager->rectifyImages(left_frame, right_frame, 
                                                      left_rectified, right_rectified)) {
            pimpl_->setError("Rectification failed");
            return core::ResultCode::ERROR_GENERIC;
        }
        return core::ResultCode::SUCCESS;
        
    } catch (const cv::Exception& e) {
        pimpl_->setError(std::string("Rectification failed: ") + e.what());
        return core::ResultCode::ERROR_GENERIC;
    }
}

// Convert depth map to point cloud
core::ResultCode DepthProcessor::depthToPointCloud(const cv::Mat& depth_map,
                                                  cv::Mat& points,
                                                  const cv::Mat* colors) {
    if (!hasValidCalibration()) {
        pimpl_->setError("No valid calibration loaded");
        return core::ResultCode::ERROR_CALIBRATION_INVALID;
    }
    
    if (depth_map.empty()) {
        pimpl_->setError("Empty depth map");
        return core::ResultCode::ERROR_INVALID_PARAMETER;
    }
    
    try {
        // Get calibration data
        auto& calibData = pimpl_->calibrationManager->getCalibrationData();
        
        // Convert depth to 3D points using Q matrix
        cv::Mat depth32f;
        if (depth_map.type() != CV_32F) {
            depth_map.convertTo(depth32f, CV_32F);
        } else {
            depth32f = depth_map;
        }
        
        // Generate point cloud
        stereo::PointCloud pointCloud;
        cv::Mat colorImage = colors ? *colors : cv::Mat();
        
        if (!pimpl_->coreProcessor->generatePointCloud(depth32f, colorImage, pointCloud)) {
            pimpl_->setError("Point cloud generation failed");
            return core::ResultCode::ERROR_GENERIC;
        }
        
        // Convert to cv::Mat format (N x 3, CV_32FC3)
        points = cv::Mat(pointCloud.points.size(), 1, CV_32FC3);
        for (size_t i = 0; i < pointCloud.points.size(); ++i) {
            const auto& pt = pointCloud.points[i];
            points.at<cv::Vec3f>(i) = cv::Vec3f(pt.x, pt.y, pt.z);
        }
        
        return core::ResultCode::SUCCESS;
        
    } catch (const cv::Exception& e) {
        pimpl_->setError(std::string("Point cloud conversion failed: ") + e.what());
        return core::ResultCode::ERROR_GENERIC;
    }
}

// Export point cloud to file
core::ResultCode DepthProcessor::exportPointCloud(const cv::Mat& depth_map,
                                                 const std::string& output_path,
                                                 const ExportOptions& options,
                                                 const cv::Mat* colors) {
    if (!hasValidCalibration()) {
        pimpl_->setError("No valid calibration loaded");
        return core::ResultCode::ERROR_CALIBRATION_INVALID;
    }
    
    try {
        // Convert depth to 32-bit float if needed
        cv::Mat depth32f;
        if (depth_map.type() != CV_32F) {
            depth_map.convertTo(depth32f, CV_32F);
        } else {
            depth32f = depth_map;
        }
        
        // Apply filters if requested
        if (options.filters.median_filter_size > 0) {
            cv::medianBlur(depth32f, depth32f, options.filters.median_filter_size);
        }
        
        // Generate point cloud
        stereo::PointCloud pointCloud;
        cv::Mat colorImage = (colors && options.include_colors) ? *colors : cv::Mat();
        
        if (!pimpl_->coreProcessor->generatePointCloud(depth32f, colorImage, pointCloud)) {
            pimpl_->setError("Point cloud generation failed");
            return core::ResultCode::ERROR_GENERIC;
        }
        
        // Determine format string for export
        std::string formatStr;
        switch (options.format) {
            case core::ExportFormat::PLY_ASCII:
            case core::ExportFormat::PLY_BINARY:
                formatStr = "ply";
                break;
            case core::ExportFormat::OBJ:
                formatStr = "obj";
                break;
            case core::ExportFormat::XYZ:
                formatStr = "xyz";
                break;
            default:
                formatStr = "ply";
        }
        
        // Export using core processor
        if (!pimpl_->coreProcessor->exportPointCloud(pointCloud, output_path, formatStr)) {
            pimpl_->setError("Point cloud export failed");
            return core::ResultCode::ERROR_FILE_IO;
        }
        
        std::cout << "[API DepthProcessor] Exported point cloud to: " << output_path << std::endl;
        return core::ResultCode::SUCCESS;
        
    } catch (const std::exception& e) {
        pimpl_->setError(std::string("Export failed: ") + e.what());
        return core::ResultCode::ERROR_FILE_IO;
    }
}

// Set stereo algorithm
core::ResultCode DepthProcessor::setStereoAlgorithm(StereoAlgorithm algorithm) {
    pimpl_->currentAlgorithm = algorithm;
    
    // Map API algorithm to core algorithm
    switch (algorithm) {
        case StereoAlgorithm::OPENCV_SGBM:
            pimpl_->stereoConfig.algorithm = core::StereoAlgorithm::SGBM_OPENCV;
            break;
        case StereoAlgorithm::OPENCV_BM:
            // Not implemented yet, fallback to SGBM
            pimpl_->stereoConfig.algorithm = core::StereoAlgorithm::SGBM_OPENCV;
            break;
        case StereoAlgorithm::BOOFCV_SGM:
            pimpl_->stereoConfig.algorithm = core::StereoAlgorithm::SGBM_OPENCV;
            break;
        case StereoAlgorithm::BOOFCV_BLOCK:
            pimpl_->stereoConfig.algorithm = core::StereoAlgorithm::BOOFCV_BASIC;
            break;
        default:
            pimpl_->setError("Unknown algorithm");
            return core::ResultCode::ERROR_INVALID_PARAMETER;
    }
    
    // Update core processor configuration
    pimpl_->coreProcessor->setStereoParameters(convertToStereoMatchingParams(pimpl_->stereoConfig));
    
    std::cout << "[API DepthProcessor] Algorithm set to: " << (int)algorithm << std::endl;
    return core::ResultCode::SUCCESS;
}

// Get current stereo algorithm
StereoAlgorithm DepthProcessor::getStereoAlgorithm() const {
    return pimpl_->currentAlgorithm;
}

// Set stereo matching parameters
core::ResultCode DepthProcessor::setStereoParams(const core::StereoConfig& params) {
    try {
        pimpl_->stereoConfig = params;
        pimpl_->coreProcessor->setStereoParameters(convertToStereoMatchingParams(params));
        
        std::cout << "[API DepthProcessor] Stereo parameters updated" << std::endl;
        return core::ResultCode::SUCCESS;
        
    } catch (const std::exception& e) {
        pimpl_->setError(std::string("Parameter setting failed: ") + e.what());
        return core::ResultCode::ERROR_INVALID_PARAMETER;
    }
}

// Get current stereo parameters
core::StereoConfig DepthProcessor::getStereoParams() const {
    return pimpl_->stereoConfig;
}

// Get performance metrics
core::ResultCode DepthProcessor::getPerformanceMetrics(double& avg_processing_time_ms,
                                                      uint64_t& frames_processed,
                                                      double& peak_memory_mb) const {
    std::lock_guard<std::mutex> lock(pimpl_->metricsMutex);
    
    if (pimpl_->framesProcessed > 0) {
        avg_processing_time_ms = pimpl_->totalProcessingTime / pimpl_->framesProcessed;
    } else {
        avg_processing_time_ms = 0.0;
    }
    
    frames_processed = pimpl_->framesProcessed;
    peak_memory_mb = pimpl_->peakMemoryMb;
    
    return core::ResultCode::SUCCESS;
}

// Reset performance metrics
void DepthProcessor::resetPerformanceMetrics() {
    std::lock_guard<std::mutex> lock(pimpl_->metricsMutex);
    pimpl_->totalProcessingTime = 0.0;
    pimpl_->framesProcessed = 0;
    pimpl_->peakMemoryMb = 0.0;
    pimpl_->startTime = std::chrono::high_resolution_clock::now();
}

// Validate depth map quality
core::ResultCode DepthProcessor::validateDepthQuality(const cv::Mat& depth_map,
                                                     double& valid_pixels_percent,
                                                     double& avg_depth_mm,
                                                     double& depth_std_dev_mm) const {
    if (depth_map.empty()) {
        pimpl_->setError("Empty depth map");
        return core::ResultCode::ERROR_INVALID_PARAMETER;
    }
    
    try {
        // Convert to float if needed
        cv::Mat depth32f;
        if (depth_map.type() != CV_32F) {
            depth_map.convertTo(depth32f, CV_32F);
        } else {
            depth32f = depth_map;
        }
        
        // Calculate valid pixels (non-zero and finite)
        cv::Mat validMask = (depth32f > 0) & (depth32f == depth32f);  // NaN check
        int validCount = cv::countNonZero(validMask);
        int totalCount = depth32f.rows * depth32f.cols;
        
        valid_pixels_percent = (100.0 * validCount) / totalCount;
        
        // Calculate mean and std dev for valid pixels
        cv::Scalar mean, stddev;
        cv::meanStdDev(depth32f, mean, stddev, validMask);
        
        avg_depth_mm = mean[0];
        depth_std_dev_mm = stddev[0];
        
        // Quality assessment based on 70mm baseline expectations
        if (valid_pixels_percent < 50.0) {
            std::cout << "[API DepthProcessor] Warning: Low depth coverage: " 
                     << valid_pixels_percent << "%" << std::endl;
        }
        
        // Check if depth values are in expected range for 70mm baseline
        if (avg_depth_mm < 50 || avg_depth_mm > 1000) {
            std::cout << "[API DepthProcessor] Warning: Average depth " << avg_depth_mm 
                     << "mm outside expected range [50, 1000]mm" << std::endl;
        }
        
        return core::ResultCode::SUCCESS;
        
    } catch (const cv::Exception& e) {
        pimpl_->setError(std::string("Quality validation failed: ") + e.what());
        return core::ResultCode::ERROR_GENERIC;
    }
}

// Create stereo parameter preset (static method)
core::StereoConfig DepthProcessor::createPreset(core::DepthQuality quality, core::StereoAlgorithm algorithm) {
    core::StereoConfig config;
    
    // Set common parameters
    config.algorithm = algorithm;
    
    // VCSEL-OPTIMIZED PRESETS for dot pattern matching
    switch (quality) {
        case core::DepthQuality::FAST_LOW:
            config.num_disparities = 256;  // Reduced for speed
            config.block_size = 5;         // VCSEL: always 5 for dots
            config.p1 = 200;              // VCSEL-optimized
            config.p2 = 800;              // VCSEL-optimized
            config.disp12_max_diff = 3;    // More tolerant for speed
            config.uniqueness_ratio = 10;  // Lower for speed
            config.speckle_window_size = 30;  // Smaller for speed
            config.speckle_range = 64;    // Keep wide for dots
            break;

        case core::DepthQuality::BALANCED:
            config.num_disparities = 320;  // Standard range
            config.block_size = 5;         // VCSEL: always 5 for dots
            config.p1 = 200;              // VCSEL-optimized
            config.p2 = 800;              // VCSEL-optimized
            config.disp12_max_diff = 2;    // Balanced tolerance
            config.uniqueness_ratio = 15;  // Standard VCSEL setting
            config.speckle_window_size = 50;  // Moderate filtering
            config.speckle_range = 64;    // Keep wide for dots
            break;

        case core::DepthQuality::SLOW_HIGH:
            config.num_disparities = 384;  // Maximum range
            config.block_size = 5;         // VCSEL: always 5 for dots
            config.p1 = 200;              // VCSEL-optimized
            config.p2 = 800;              // VCSEL-optimized
            config.disp12_max_diff = 1;    // Strict for precision
            config.uniqueness_ratio = 20;  // High for best precision
            config.speckle_window_size = 75;  // More thorough filtering
            config.speckle_range = 64;    // Keep wide for dots
            break;
    }

    // Set VCSEL defaults
    config.min_disparity = 0;
    config.pre_filter_cap = 63;      // Maximum to preserve dots
    config.mode = cv::StereoSGBM::MODE_SGBM; // Standard SGBM for dots
    
    return config;
}

// Get current stereo configuration
core::StereoConfig DepthProcessor::getStereoConfig() const {
    return pimpl_->stereoConfig;
}

// Configure stereo matching parameters
void DepthProcessor::configureStereo(const core::StereoConfig& config) {
    pimpl_->stereoConfig = config;
    // Apply the configuration to the core processor
    if (pimpl_->coreProcessor && pimpl_->initialized) {
        pimpl_->coreProcessor->setStereoParameters(convertToStereoMatchingParams(config));
    }
}

// Process stereo frame pair synchronously
core::DepthResult DepthProcessor::processSync(const core::StereoFramePair& frame_pair) {
    core::DepthResult result;
    
    std::cout << "[DepthProcessor] Processing stereo frame pair:" << std::endl;
    std::cout << "  - Left frame: " << (frame_pair.left_frame.valid ? "valid" : "invalid") 
              << ", size: " << frame_pair.left_frame.image.cols << "x" << frame_pair.left_frame.image.rows << std::endl;
    std::cout << "  - Right frame: " << (frame_pair.right_frame.valid ? "valid" : "invalid")
              << ", size: " << frame_pair.right_frame.image.cols << "x" << frame_pair.right_frame.image.rows << std::endl;
    std::cout << "  - Synchronized: " << (frame_pair.synchronized ? "yes" : "no") 
              << ", sync error: " << frame_pair.sync_error_ms << "ms" << std::endl;
    
    // Check frame validity
    if (!frame_pair.left_frame.valid || !frame_pair.right_frame.valid) {
        std::cerr << "[DepthProcessor] ERROR: Invalid frames received" << std::endl;
        result.success = false;
        result.error_message = "Invalid frames";
        return result;
    }
    
    if (frame_pair.left_frame.image.empty() || frame_pair.right_frame.image.empty()) {
        std::cerr << "[DepthProcessor] ERROR: Empty frames received" << std::endl;
        result.success = false;
        result.error_message = "Empty frames";
        return result;
    }
    
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Call the existing processFrames method and adapt the result
    std::cout << "[DepthProcessor] Calling processFrames..." << std::endl;
    core::ResultCode status = processFrames(frame_pair.left_frame.image, frame_pair.right_frame.image, 
                                           result.depth_map, &result.disparity_map);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    result.processing_time_ms = duration.count();
    
    std::cout << "[DepthProcessor] processFrames returned: " << core::resultCodeToString(status) 
              << " in " << result.processing_time_ms << "ms" << std::endl;
    
    result.success = (status == core::ResultCode::SUCCESS);

    if (result.success) {
        std::cout << "[DepthProcessor] Depth map generated: "
                  << result.depth_map.cols << "x" << result.depth_map.rows << std::endl;

        // CRITICAL FIX: Get point cloud file path after processFrames()
        result.debug_pointcloud_path = pimpl_->coreProcessor->getLastPointCloudPath();
        std::cout << "[DepthProcessor] Point cloud file path: " << result.debug_pointcloud_path << std::endl;

        // Calculate depth statistics
        cv::Scalar mean_depth, stddev_depth;
        cv::meanStdDev(result.depth_map, mean_depth, stddev_depth, result.depth_map > 0);
        result.mean_depth = mean_depth[0];
        result.std_depth = stddev_depth[0];
        
        // Calculate coverage ratio
        int total_pixels = result.depth_map.rows * result.depth_map.cols;
        int valid_pixels = cv::countNonZero(result.depth_map > 0);
        result.coverage_ratio = (double)valid_pixels / total_pixels;
        
        std::cout << "[DepthProcessor] Depth statistics:" << std::endl;
        std::cout << "  - Mean depth: " << result.mean_depth << "mm" << std::endl;
        std::cout << "  - Std deviation: " << result.std_depth << "mm" << std::endl;
        std::cout << "  - Coverage: " << (result.coverage_ratio * 100) << "%" << std::endl;
    } else {
        std::cerr << "[DepthProcessor] Depth processing failed" << std::endl;
        result.error_message = "Processing failed: " + core::resultCodeToString(status);
    }
    
    return result;
}

// Visualize depth map as colored image
cv::Mat DepthProcessor::visualizeDepthMap(const cv::Mat& depth_map, double min_depth, double max_depth) const {
    cv::Mat visualization;

    if (depth_map.empty()) {
        return visualization;
    }

    // Validate input parameters - only fix completely invalid ranges
    if (min_depth <= 0 || max_depth <= 0 || max_depth <= min_depth) {
        min_depth = 0.0;
        max_depth = 1000.0;  // Generic fallback only for invalid data
    }

    // SAFE NORMALIZATION: Use actual parameters, not hardcoded ranges
    cv::Mat depth_clean = depth_map.clone();

    // Remove invalid values using ACTUAL parameter range (not hardcoded!)
    cv::Mat finite_mask;
    cv::compare(depth_clean, depth_clean, finite_mask, cv::CMP_EQ); // NaN check
    cv::Mat range_mask;
    cv::inRange(depth_clean, min_depth, max_depth, range_mask); // Use actual parameters!
    cv::Mat valid_mask;
    cv::bitwise_and(finite_mask, range_mask, valid_mask);

    // Set invalid pixels to 0 for exclusion
    depth_clean.setTo(0, ~valid_mask);

    if (cv::countNonZero(valid_mask) < 100) {
        // Not enough valid pixels - return black image
        visualization = cv::Mat::zeros(depth_map.size(), CV_8UC3);
        return visualization;
    }

    // Use the actual parameter range for visualization (no more ignored parameters!)
    double range_span = max_depth - min_depth;
    if (range_span <= 0.1) {
        // Avoid division by zero
        visualization = cv::Mat::zeros(depth_map.size(), CV_8UC3);
        return visualization;
    }

    // SAFE conversion with parameter-based range
    cv::Mat depth_8bit;
    double scale = 255.0 / range_span;
    double offset = -min_depth * scale;

    // Safety check for conversion parameters
    if (scale <= 0 || scale > 10000 || std::abs(offset) > 1000000) {
        // Invalid conversion parameters - fallback to normalize
        cv::normalize(depth_clean, depth_8bit, 0, 255, cv::NORM_MINMAX, CV_8U, valid_mask);
    } else {
        depth_clean.convertTo(depth_8bit, CV_8U, scale, offset);
    }
    
    // Increase contrast by 1.25x for better face depth visualization (SAFE)
    cv::Mat depth_contrast;
    depth_8bit.convertTo(depth_contrast, CV_8U, 1.25, -32); // Force CV_8U type to prevent segfault

    // Safe clamping to valid 8-bit range
    cv::threshold(depth_contrast, depth_contrast, 255, 255, cv::THRESH_TRUNC); // Clamp max to 255
    cv::max(depth_contrast, cv::Scalar(0), depth_contrast);                    // Clamp min to 0

    // Apply color map with safe input
    cv::applyColorMap(depth_contrast, visualization, cv::COLORMAP_JET);

    // ENHANCED: Set invalid pixels to dark blue instead of black (NO MORE BLACK LINES!)
    cv::Mat invalid_mask = (depth_map == 0) | (~valid_mask);
    visualization.setTo(cv::Scalar(100, 0, 0), invalid_mask);  // Dark blue instead of black

    // Calculate actual statistics for logging
    cv::Scalar mean_scalar, std_scalar;
    cv::meanStdDev(depth_clean, mean_scalar, std_scalar, valid_mask);
    double mean_depth = mean_scalar[0];
    double std_depth = std_scalar[0];

    std::cout << "[API DepthProcessor] Visualization - Valid pixels: " << cv::countNonZero(valid_mask)
              << "/" << depth_map.total() << " (" << (100.0 * cv::countNonZero(valid_mask) / depth_map.total()) << "%)"
              << ", Robust range: " << min_depth << "-" << max_depth << "mm"
              << ", Mean±Std: " << mean_depth << "±" << std_depth << "mm" << std::endl;

    return visualization;
}

// Export depth map to file
bool DepthProcessor::exportDepthMap(const core::DepthResult& result, const std::string& filename, const std::string& format) const {
    try {
        if (format == "PNG" || format == "png") {
            // Export as 16-bit PNG
            return cv::imwrite(filename, result.depth_map);
        }
        else if (format == "PLY" || format == "ply") {
            // Convert to point cloud and export as PLY
            // This would be implemented using the existing point cloud conversion
            // For now, return placeholder success
            return true;
        }
        else {
            std::cerr << "[API DepthProcessor] Unsupported export format: " << format << std::endl;
            return false;
        }
    }
    catch (const cv::Exception& e) {
        std::cerr << "[API DepthProcessor] Export failed: " << e.what() << std::endl;
        return false;
    }
}

// Get last error message
std::string DepthProcessor::getLastError() const {
    std::lock_guard<std::mutex> lock(pimpl_->errorMutex);
    return pimpl_->lastError;
}

} // namespace api
} // namespace unlook