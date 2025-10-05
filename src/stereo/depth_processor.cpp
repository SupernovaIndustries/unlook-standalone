#include "unlook/stereo/depth_processor.hpp"
#include <iostream>
#include <fstream>
#include <chrono>

namespace unlook {
namespace stereo {

DepthProcessor::DepthProcessor()
    : calibration_loaded_(false)
    , processing_running_(false)
    , stats_{}
{
    // Initialize SGBM matcher with default parameters optimized for 70mm baseline
    sgbm_matcher_ = cv::StereoSGBM::create(
        0,     // minDisparity
        160,   // numDisparities (covers 70mm baseline range)
        7,     // blockSize (optimal for precision)
        8 * 3 * 7 * 7,   // P1
        32 * 3 * 7 * 7,  // P2
        1,     // disp12MaxDiff
        63,    // preFilterCap
        5,     // uniquenessRatio (strict for precision)
        100,   // speckleWindowSize
        32,    // speckleRange
        cv::StereoSGBM::MODE_SGBM_3WAY  // Best quality mode
    );
    
    // Initialize with default stereo config
    stereo_config_.min_disparity = 0;
    stereo_config_.num_disparities = 160;
    stereo_config_.block_size = 7;
    stereo_config_.p1 = 8 * 3 * 7 * 7;
    stereo_config_.p2 = 32 * 3 * 7 * 7;
    stereo_config_.disp12_max_diff = 1;
    stereo_config_.pre_filter_cap = 63;
    stereo_config_.uniqueness_ratio = 5;
    stereo_config_.speckle_window_size = 100;
    stereo_config_.speckle_range = 32;
    stereo_config_.mode = cv::StereoSGBM::MODE_SGBM_3WAY;
    stereo_config_.algorithm = core::StereoAlgorithm::SGBM_OPENCV;
    stereo_config_.quality = core::DepthQuality::BALANCED;
    
    stats_start_time_ = std::chrono::high_resolution_clock::now();
    
    std::cout << "[stereo::DepthProcessor] Initialized with 70mm baseline optimization" << std::endl;
}

DepthProcessor::~DepthProcessor() {
    stopAsync();
    std::cout << "[stereo::DepthProcessor] Destructor called" << std::endl;
}

bool DepthProcessor::initialize(const std::string& calibration_file) {
    std::cout << "[stereo::DepthProcessor] Initializing with calibration file: " << calibration_file << std::endl;
    return loadCalibration(calibration_file);
}

bool DepthProcessor::loadCalibration(const std::string& calibration_file) {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    
    try {
        cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            std::cerr << "Failed to open calibration file: " << calibration_file << std::endl;
            return false;
        }
        
        // Load camera matrices and distortion coefficients
        fs["camera_matrix_left"] >> camera_matrix_left_;
        fs["camera_matrix_right"] >> camera_matrix_right_;
        fs["dist_coeffs_left"] >> dist_coeffs_left_;
        fs["dist_coeffs_right"] >> dist_coeffs_right_;
        fs["R"] >> rotation_matrix_;
        fs["T"] >> translation_vector_;

        // CRITICAL: Load pre-computed rectification matrices from BoofCV calibration
        // These have industrial precision (RMS 0.24px) - DO NOT recalculate with stereoRectify!
        fs["R1"] >> rectification_left_;
        fs["R2"] >> rectification_right_;
        fs["P1"] >> projection_left_;
        fs["P2"] >> projection_right_;
        fs["Q"] >> disparity_to_depth_map_;

        std::cout << "✓ Loaded BoofCV pre-computed rectification matrices (industrial precision)" << std::endl;

        // CRITICAL VERIFICATION: Log Q matrix parameters
        double q_baseline = std::abs(1.0 / disparity_to_depth_map_.at<double>(3, 2));
        double q_focal = disparity_to_depth_map_.at<double>(2, 3);
        double q_cx = -disparity_to_depth_map_.at<double>(0, 3);
        double q_cy = -disparity_to_depth_map_.at<double>(1, 3);

        std::cout << "  Q matrix baseline: " << q_baseline << "mm (should be 70.017)" << std::endl;
        std::cout << "  Q matrix focal length: " << q_focal << "px (should be 1755.436)" << std::endl;
        std::cout << "  Q matrix cx: " << q_cx << "px (should be 713.058)" << std::endl;
        std::cout << "  Q matrix cy: " << q_cy << "px (should be 515.472)" << std::endl;

        // Verify Q matrix is correct
        if (std::abs(q_baseline - 70.017) > 0.01) {
            std::cerr << "❌ WARNING: Q baseline (" << q_baseline << ") != expected 70.017mm!" << std::endl;
        }
        if (std::abs(q_focal - 1755.436) > 1.0) {
            std::cerr << "❌ WARNING: Q focal (" << q_focal << ") != expected 1755.436px!" << std::endl;
        }
        
        // Generate rectification maps
        cv::initUndistortRectifyMap(camera_matrix_left_, dist_coeffs_left_,
                                   rectification_left_, projection_left_,
                                   image_size, CV_32FC1,
                                   rectification_map_left_1_, rectification_map_left_2_);
        
        cv::initUndistortRectifyMap(camera_matrix_right_, dist_coeffs_right_,
                                   rectification_right_, projection_right_,
                                   image_size, CV_32FC1,
                                   rectification_map_right_1_, rectification_map_right_2_);
        
        calibration_loaded_ = true;
        fs.release();
        
        std::cout << "Calibration loaded successfully from: " << calibration_file << std::endl;
        return true;
        
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV error loading calibration: " << e.what() << std::endl;
        calibration_loaded_ = false;
        return false;
    }
}

void DepthProcessor::configureStereo(const core::StereoConfig& config) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    stereo_config_ = config;
    
    // Update SGBM parameters
    sgbm_matcher_->setMinDisparity(config.min_disparity);
    sgbm_matcher_->setNumDisparities(config.num_disparities);
    sgbm_matcher_->setBlockSize(config.block_size);
    sgbm_matcher_->setP1(config.p1);
    sgbm_matcher_->setP2(config.p2);
    sgbm_matcher_->setDisp12MaxDiff(config.disp12_max_diff);
    sgbm_matcher_->setPreFilterCap(config.pre_filter_cap);
    sgbm_matcher_->setUniquenessRatio(config.uniqueness_ratio);
    sgbm_matcher_->setSpeckleWindowSize(config.speckle_window_size);
    sgbm_matcher_->setSpeckleRange(config.speckle_range);
    sgbm_matcher_->setMode(config.mode);
}

core::StereoConfig DepthProcessor::getStereoConfig() const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    return stereo_config_;
}

core::DepthResult DepthProcessor::processSync(const core::StereoFramePair& stereo_pair) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    core::DepthResult result = processInternal(stereo_pair);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    result.processing_time_ms = duration.count();
    
    updateStats(result.processing_time_ms);
    
    return result;
}

bool DepthProcessor::startAsync(core::DepthResultCallback result_callback) {
    if (processing_running_.load()) {
        return false;
    }
    
    result_callback_ = result_callback;
    processing_running_ = true;
    
    processing_thread_ = std::make_unique<std::thread>(&DepthProcessor::processingThread, this);
    
    std::cout << "Async depth processing started" << std::endl;
    return true;
}

void DepthProcessor::stopAsync() {
    if (!processing_running_.load()) {
        return;
    }
    
    processing_running_ = false;
    queue_condition_.notify_all();
    
    if (processing_thread_ && processing_thread_->joinable()) {
        processing_thread_->join();
    }
    processing_thread_.reset();
    
    std::cout << "Async depth processing stopped" << std::endl;
}

bool DepthProcessor::queueFramePair(const core::StereoFramePair& stereo_pair) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    if (processing_queue_.size() >= MAX_QUEUE_SIZE) {
        return false; // Queue is full
    }
    
    processing_queue_.push(stereo_pair);
    queue_condition_.notify_one();
    return true;
}

size_t DepthProcessor::getQueueSize() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return processing_queue_.size();
}

DepthProcessor::ProcessingStats DepthProcessor::getProcessingStats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void DepthProcessor::resetStats() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_ = ProcessingStats{};
    stats_start_time_ = std::chrono::high_resolution_clock::now();
}

bool DepthProcessor::isCalibrationLoaded() const {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    return calibration_loaded_;
}

bool DepthProcessor::getRectificationMaps(cv::Mat& left_map1, cv::Mat& left_map2,
                                         cv::Mat& right_map1, cv::Mat& right_map2) const {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    
    if (!calibration_loaded_) {
        return false;
    }
    
    left_map1 = rectification_map_left_1_.clone();
    left_map2 = rectification_map_left_2_.clone();
    right_map1 = rectification_map_right_1_.clone();
    right_map2 = rectification_map_right_2_.clone();
    
    return true;
}

void DepthProcessor::rectifyImages(const cv::Mat& left_raw, const cv::Mat& right_raw,
                                  cv::Mat& left_rect, cv::Mat& right_rect) const {
    std::lock_guard<std::mutex> lock(calibration_mutex_);
    
    if (!calibration_loaded_) {
        left_rect = left_raw.clone();
        right_rect = right_raw.clone();
        return;
    }
    
    cv::remap(left_raw, left_rect, rectification_map_left_1_, rectification_map_left_2_, cv::INTER_LINEAR);
    cv::remap(right_raw, right_rect, rectification_map_right_1_, rectification_map_right_2_, cv::INTER_LINEAR);
}

cv::Mat DepthProcessor::visualizeDepthMap(const cv::Mat& depth_map, 
                                         double min_depth, double max_depth) const {
    if (depth_map.empty()) {
        return cv::Mat();
    }
    
    cv::Mat visualization;
    cv::Mat normalized;
    
    // Normalize depth map to 0-255 range
    if (min_depth < max_depth) {
        depth_map.convertTo(normalized, CV_8U, 255.0 / (max_depth - min_depth), -min_depth * 255.0 / (max_depth - min_depth));
    } else {
        cv::normalize(depth_map, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    }
    
    // Apply color map
    cv::applyColorMap(normalized, visualization, cv::COLORMAP_JET);
    
    return visualization;
}

bool DepthProcessor::exportDepthMap(const core::DepthResult& result, 
                                   const std::string& filename, 
                                   const std::string& format) const {
    if (result.depth_map.empty()) {
        std::cerr << "[stereo::DepthProcessor] Empty depth map for export" << std::endl;
        return false;
    }
    
    try {
        if (format == "PNG" || format == "png") {
            // Convert to visualization for PNG
            cv::Mat visualization = visualizeDepthMap(result.depth_map, 100.0, 800.0);
            return cv::imwrite(filename, visualization);
        } else if (format == "PNG16" || format == "png16") {
            // Save as 16-bit PNG preserving depth values
            cv::Mat depth16;
            result.depth_map.convertTo(depth16, CV_16U);
            return cv::imwrite(filename, depth16);
        } else if (format == "PFM" || format == "pfm") {
            // Save in PFM format
            std::ofstream file(filename, std::ios::binary);
            if (!file.is_open()) return false;
            
            file << "Pf\n";
            file << result.depth_map.cols << " " << result.depth_map.rows << "\n";
            file << "-1.0\n";
            
            cv::Mat depth32f;
            if (result.depth_map.type() != CV_32F) {
                result.depth_map.convertTo(depth32f, CV_32F);
            } else {
                depth32f = result.depth_map;
            }
            
            for (int y = depth32f.rows - 1; y >= 0; --y) {
                const float* row = depth32f.ptr<float>(y);
                file.write(reinterpret_cast<const char*>(row), depth32f.cols * sizeof(float));
            }
            
            file.close();
            return true;
        } else if (format == "PLY" || format == "ply") {
            // Basic PLY export of depth as point cloud
            std::ofstream file(filename);
            if (!file.is_open()) return false;
            
            // Count valid points
            int validPoints = 0;
            for (int y = 0; y < result.depth_map.rows; ++y) {
                for (int x = 0; x < result.depth_map.cols; ++x) {
                    float depth = result.depth_map.at<float>(y, x);
                    if (depth > 0 && depth < 1000) validPoints++;
                }
            }
            
            // Write PLY header
            file << "ply\n";
            file << "format ascii 1.0\n";
            file << "element vertex " << validPoints << "\n";
            file << "property float x\n";
            file << "property float y\n";
            file << "property float z\n";
            file << "end_header\n";
            
            // Write points (simplified - using pixel coordinates as X,Y)
            for (int y = 0; y < result.depth_map.rows; ++y) {
                for (int x = 0; x < result.depth_map.cols; ++x) {
                    float depth = result.depth_map.at<float>(y, x);
                    if (depth > 0 && depth < 1000) {
                        file << x << " " << y << " " << depth << "\n";
                    }
                }
            }
            
            file.close();
            return true;
        } else {
            std::cerr << "[stereo::DepthProcessor] Unsupported export format: " << format << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "[stereo::DepthProcessor] Export failed: " << e.what() << std::endl;
        return false;
    }
}

core::StereoConfig DepthProcessor::createPreset(core::DepthQuality quality, 
                                               core::StereoAlgorithm algorithm) {
    core::StereoConfig config;
    config.algorithm = algorithm;
    config.quality = quality;
    
    // Base parameters optimized for 70mm baseline
    config.min_disparity = 0;
    
    switch (quality) {
        case core::DepthQuality::FAST_LOW:
            config.num_disparities = 96;   // Reduced for speed
            config.block_size = 9;          // Larger block for speed
            config.uniqueness_ratio = 5;   // Less strict
            config.p1 = 8 * 3 * 9 * 9;
            config.p2 = 32 * 3 * 9 * 9;
            config.disp12_max_diff = 2;
            config.pre_filter_cap = 31;
            config.speckle_window_size = 50;
            config.speckle_range = 16;
            config.mode = cv::StereoSGBM::MODE_SGBM;  // Fastest mode
            break;
            
        case core::DepthQuality::BALANCED:
            config.num_disparities = 160;  // Good coverage for 70mm baseline
            config.block_size = 7;         // Balanced
            config.uniqueness_ratio = 5;   // Moderate strictness
            config.p1 = 8 * 3 * 7 * 7;
            config.p2 = 32 * 3 * 7 * 7;
            config.disp12_max_diff = 1;
            config.pre_filter_cap = 63;
            config.speckle_window_size = 100;
            config.speckle_range = 32;
            config.mode = cv::StereoSGBM::MODE_SGBM;  // Standard mode
            break;
            
        case core::DepthQuality::SLOW_HIGH:
            config.num_disparities = 256;  // Maximum coverage
            config.block_size = 5;         // Smaller for detail
            config.uniqueness_ratio = 3;   // Very strict for precision
            config.p1 = 8 * 3 * 5 * 5;
            config.p2 = 32 * 3 * 5 * 5;
            config.disp12_max_diff = 1;
            config.pre_filter_cap = 63;
            config.speckle_window_size = 200;
            config.speckle_range = 64;
            config.mode = cv::StereoSGBM::MODE_SGBM_3WAY;  // Best quality
            break;
    }
    
    std::cout << "[stereo::DepthProcessor] Created preset for quality: " << (int)quality 
              << ", algorithm: " << (int)algorithm << std::endl;
    
    return config;
}

void DepthProcessor::processingThread() {
    std::cout << "Depth processing thread started" << std::endl;
    
    while (processing_running_.load()) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        // Wait for frames to process
        queue_condition_.wait(lock, [this]() {
            return !processing_queue_.empty() || !processing_running_.load();
        });
        
        if (!processing_running_.load()) {
            break;
        }
        
        if (processing_queue_.empty()) {
            continue;
        }
        
        // Get next frame pair
        core::StereoFramePair frame_pair = processing_queue_.front();
        processing_queue_.pop();
        lock.unlock();
        
        // Process frame pair
        core::DepthResult result = processSync(frame_pair);
        
        // Call result callback
        if (result_callback_) {
            result_callback_(result);
        }
    }
    
    std::cout << "Depth processing thread stopped" << std::endl;
}

core::DepthResult DepthProcessor::processInternal(const core::StereoFramePair& stereo_pair) {
    core::DepthResult result;
    
    if (!stereo_pair.synchronized || 
        stereo_pair.left_frame.image.empty() || 
        stereo_pair.right_frame.image.empty()) {
        result.success = false;
        result.error_message = "Invalid or unsynchronized stereo pair";
        return result;
    }
    
    try {
        switch (stereo_config_.algorithm) {
            case core::StereoAlgorithm::SGBM_OPENCV:
                result = processOpenCVSGBM(stereo_pair.left_frame.image, stereo_pair.right_frame.image);
                break;
                
            case core::StereoAlgorithm::BOOFCV_BASIC:
            case core::StereoAlgorithm::BOOFCV_PRECISE:
                result = processBoofCV(stereo_pair.left_frame.image, stereo_pair.right_frame.image);
                break;
                
            default:
                result.success = false;
                result.error_message = "Unknown stereo algorithm";
                break;
        }
    } catch (const cv::Exception& e) {
        result.success = false;
        result.error_message = "OpenCV error: " + std::string(e.what());
    }
    
    return result;
}

core::DepthResult DepthProcessor::processOpenCVSGBM(const cv::Mat& left, const cv::Mat& right) {
    core::DepthResult result;
    
    try {
        cv::Mat left_rect, right_rect;
        
        // Rectify images if calibration is available
        if (calibration_loaded_) {
            rectifyImages(left, right, left_rect, right_rect);
        } else {
            left_rect = left.clone();
            right_rect = right.clone();
        }
        
        // Convert to grayscale if needed
        if (left_rect.channels() > 1) {
            cv::cvtColor(left_rect, left_rect, cv::COLOR_BGR2GRAY);
        }
        if (right_rect.channels() > 1) {
            cv::cvtColor(right_rect, right_rect, cv::COLOR_BGR2GRAY);
        }
        
        // Compute disparity map
        cv::Mat disparity_16;
        sgbm_matcher_->compute(left_rect, right_rect, disparity_16);
        
        // Convert to floating point
        disparity_16.convertTo(result.disparity_map, CV_32F, 1.0/16.0);
        
        // Convert disparity to depth if calibration is available
        if (calibration_loaded_ && !disparity_to_depth_map_.empty()) {
            cv::reprojectImageTo3D(result.disparity_map, result.depth_map, disparity_to_depth_map_);
            
            // Extract depth channel (Z component)
            std::vector<cv::Mat> channels(3);
            cv::split(result.depth_map, channels);
            result.depth_map = channels[2]; // Z channel is the depth
        } else {
            // Without calibration, depth map is same as disparity
            result.depth_map = result.disparity_map.clone();
        }
        
        // Calculate quality metrics
        cv::Mat valid_pixels = result.depth_map > 0;
        result.coverage_ratio = cv::sum(valid_pixels)[0] / (result.depth_map.rows * result.depth_map.cols * 255.0);
        
        cv::Scalar mean_val, std_val;
        cv::meanStdDev(result.depth_map, mean_val, std_val, valid_pixels);
        result.mean_depth = mean_val[0];
        result.std_depth = std_val[0];
        
        // Create confidence map (simplified)
        result.confidence_map = cv::Mat::ones(result.depth_map.size(), CV_32F);
        
        result.success = true;
        
    } catch (const cv::Exception& e) {
        result.success = false;
        result.error_message = "SGBM processing error: " + std::string(e.what());
    }
    
    return result;
}

core::DepthResult DepthProcessor::processBoofCV(const cv::Mat& left, const cv::Mat& right) {
    core::DepthResult result;
    
    // TODO: Implement BoofCV integration
    result.success = false;
    result.error_message = "BoofCV processing not implemented yet";
    
    return result;
}

void DepthProcessor::updateStats(double processing_time_ms) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    stats_.frames_processed++;
    
    if (stats_.frames_processed == 1) {
        stats_.min_processing_time_ms = processing_time_ms;
        stats_.max_processing_time_ms = processing_time_ms;
        stats_.average_processing_time_ms = processing_time_ms;
    } else {
        stats_.min_processing_time_ms = std::min(stats_.min_processing_time_ms, processing_time_ms);
        stats_.max_processing_time_ms = std::max(stats_.max_processing_time_ms, processing_time_ms);
        
        // Running average
        stats_.average_processing_time_ms = 
            (stats_.average_processing_time_ms * (stats_.frames_processed - 1) + processing_time_ms) / stats_.frames_processed;
    }
    
    // Calculate FPS
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - stats_start_time_);
    if (elapsed.count() > 0) {
        stats_.fps = (stats_.frames_processed * 1000.0) / elapsed.count();
    }
}

} // namespace stereo
} // namespace unlook